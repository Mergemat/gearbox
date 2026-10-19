use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use gearbox_engine::Savestate;
use glam::Vec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaceWhat {
    Point,
    Box,
    Pose,
    Line,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Purpose {
    Hover,
    Place { what: PlaceWhat, eye: Vec3 },
    Car { eye: Vec3, commit: bool },
    Pivot,
    Wheel { tick: usize, index: u8 },
}

impl Purpose {
    fn coalesced(&self) -> Option<u8> {
        match self {
            Self::Hover => Some(0),
            Self::Car { commit: false, .. } => Some(1),
            _ => None,
        }
    }
}

const REACH: f32 = 6000.0;

struct Request {
    origin: Vec3,
    dir: Vec3,
    purpose: Purpose,
}

pub struct Picked {
    pub purpose: Purpose,
    pub hit: Option<gearbox_search::picking::Hit>,
    pub micros: u64,
    pub error: Option<String>,
}

pub struct Picker {
    tx: Sender<Request>,
    pub results: Arc<Mutex<Vec<Picked>>>,
}

impl Picker {
    pub fn spawn(origin: Savestate, wake: Arc<dyn Fn() + Send + Sync>) -> Self {
        let (tx, rx) = channel();
        let results = Arc::new(Mutex::new(Vec::new()));
        let out = results.clone();
        std::thread::Builder::new().name("pick".into()).spawn(move || run(origin, rx, out, wake)).expect("spawning the pick thread");
        Self { tx, results }
    }

    pub fn request(&self, origin: Vec3, dir: Vec3, purpose: Purpose) {
        let _ = self.tx.send(Request { origin, dir, purpose });
    }

    pub fn take(&self) -> Vec<Picked> {
        std::mem::take(&mut *self.results.lock().unwrap())
    }
}

fn run(origin: Savestate, rx: Receiver<Request>, out: Arc<Mutex<Vec<Picked>>>, wake: Arc<dyn Fn() + Send + Sync>) {
    let mut runner = origin.fork();
    while let Ok(first) = rx.recv() {
        let mut batch = vec![first];
        batch.extend(rx.try_iter());
        let latest = |k: u8| batch.iter().rposition(|r| r.purpose.coalesced() == Some(k));
        let keep = [latest(0), latest(1)];
        for (i, r) in batch.iter().enumerate() {
            if let Some(k) = r.purpose.coalesced()
                && keep[k as usize] != Some(i)
            {
                continue;
            }
            let t = Instant::now();
            let seg = (r.dir * REACH).to_array();
            let res = catch_unwind(AssertUnwindSafe(|| gearbox_search::picking::pick(&mut runner, r.origin.to_array(), seg)));
            let micros = t.elapsed().as_micros() as u64;
            let (hit, error) = match res {
                Ok(h) => (h, None),
                Err(p) => {
                    runner = origin.fork();
                    let msg = p.downcast_ref::<String>().cloned().or_else(|| p.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_else(|| "panic".into());
                    (None, Some(msg))
                }
            };
            out.lock().unwrap().push(Picked { purpose: r.purpose, hit, micros, error });
        }
        wake();
    }
}
