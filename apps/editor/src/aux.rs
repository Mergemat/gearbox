use std::collections::VecDeque;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use anyhow::Result;
use crate::engine::frame::Frame;
use crate::engine::input::InputFile;
use gearbox_engine::{Controls, Savestate, Session};
use gearbox_search::pose::Placement;

pub type Wake = Arc<dyn Fn() + Send + Sync>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Key {
    Run(u32),
    Candidate(u64),
}

pub enum Job {
    Simulate { key: Key, inputs: Arc<InputFile>, len: usize, start: Option<(usize, Placement)> },
    Gates,
}

#[derive(Clone, Debug)]
pub struct GateInfo {
    pub gate: gearbox_search::gate::Gate,
    pub order: Option<u32>,
}

pub enum Done {
    Frames { key: Key, frames: Arc<Vec<Frame>>, micros: u64, error: Option<String> },
    Gates(Result<Vec<GateInfo>, String>),
}

pub struct Aux {
    tx: Sender<Job>,
    pub done: Arc<Mutex<Vec<Done>>>,
}

pub fn panic_message(p: &(dyn std::any::Any + Send)) -> String {
    p.downcast_ref::<&str>().map(|s| s.to_string())
        .or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_else(|| "panic".into())
}

impl Aux {
    pub fn spawn(origin: Savestate, wake: Wake) -> Self {
        let (tx, rx) = channel();
        let done = Arc::new(Mutex::new(Vec::new()));
        let out = done.clone();
        std::thread::Builder::new().name("aux".into()).spawn(move || run(origin, rx, out, wake))
            .expect("spawning the aux thread");
        Self { tx, done }
    }

    pub fn send(&self, j: Job) {
        let _ = self.tx.send(j);
    }

    pub fn take(&self) -> Vec<Done> {
        std::mem::take(&mut *self.done.lock().unwrap())
    }
}

fn run(origin: Savestate, rx: Receiver<Job>, out: Arc<Mutex<Vec<Done>>>, wake: Wake) {
    let mut runner = origin.fork();
    let base = runner.save();
    let mut queue: VecDeque<Job> = VecDeque::new();
    loop {
        if queue.is_empty() {
            match rx.recv() {
                Ok(j) => queue.push_back(j),
                Err(_) => return,
            }
        }
        queue.extend(rx.try_iter());
        let mut seen = std::collections::HashSet::new();
        let mut jobs: Vec<Job> = Vec::new();
        for j in queue.drain(..).rev() {
            match &j {
                Job::Simulate { key, .. } if !seen.insert(*key) => {}
                _ => jobs.push(j),
            }
        }
        jobs.reverse();
        jobs.sort_by_key(|j| !matches!(j, Job::Simulate { key: Key::Candidate(_), .. }));
        let mut it = jobs.into_iter();
        let Some(job) = it.next() else { continue };
        queue.extend(it);
        let done = match job {
            Job::Gates => Done::Gates(catch_unwind(AssertUnwindSafe(|| gates(&mut runner)))
                .unwrap_or_else(|p| Err(panic_message(&*p)))),
            Job::Simulate { key, inputs, len, start } => {
                let t = Instant::now();
                runner.restore(&base);
                let (frames, error) = simulate(&mut runner, &inputs, len, start.as_ref());
                if error.is_some() {
                    runner = origin.fork();
                }
                Done::Frames { key, frames: Arc::new(frames), micros: t.elapsed().as_micros() as u64, error }
            }
        };
        out.lock().unwrap().push(done);
        wake();
    }
}

pub fn simulate(runner: &mut Session, inputs: &InputFile, len: usize,
    start: Option<&(usize, Placement)>) -> (Vec<Frame>, Option<String>)
{
    runner.seed = inputs.seed;
    let race_ms0 = runner.race_ms();
    let mut frames = Vec::with_capacity(len + 1);
    let r = catch_unwind(AssertUnwindSafe(|| -> Result<(), String> {
        let place = |runner: &mut Session, k: usize| -> Result<(), String> {
            match start {
                Some((tick, p)) if *tick == k => gearbox_search::pose::place(runner, p)
                    .map_err(|e| format!("placing the start state: {e:#}")),
                _ => Ok(()),
            }
        };
        place(runner, 0)?;
        frames.push(Frame::read(runner, Controls::default()));
        for k in 0..len {
            let c = inputs.at(race_ms0 + 10 * k as i32);
            runner.step(c);
            place(runner, k + 1)?;
            frames.push(Frame::read(runner, c));
        }
        Ok(())
    }));
    let error = match r {
        Ok(Ok(())) => None,
        Ok(Err(e)) => Some(e),
        Err(p) => Some(format!("stopped at tick {}: {}", frames.len().saturating_sub(1), panic_message(&*p))),
    };
    (frames, error)
}

fn gates(runner: &mut Session) -> Result<Vec<GateInfo>, String> {
    let gates = gearbox_search::gate::gates(runner).map_err(|e| format!("{e:#}"))?;
    Ok(gates
        .into_iter()
        .map(|g| {
            let order = (g.kind != 1).then(|| runner.waypoint_order(g.waypoint)).flatten()
                .filter(|o| *o != u32::MAX);
            GateInfo { gate: g, order }
        })
        .collect())
}

pub fn reach(origin: &Savestate, inputs: &InputFile, tick: usize,
    start: Option<&(usize, Placement)>) -> Result<Savestate>
{
    let mut n = origin.fork();
    n.seed = inputs.seed;
    let race_ms0 = n.race_ms();
    let place = |n: &mut Session, k: usize| -> Result<()> {
        match start {
            Some((t, p)) if *t == k => gearbox_search::pose::place(n, p),
            _ => Ok(()),
        }
    };
    place(&mut n, 0)?;
    for k in 0..tick {
        n.step(inputs.at(race_ms0 + 10 * k as i32));
        place(&mut n, k + 1)?;
    }
    Ok(n.save())
}

