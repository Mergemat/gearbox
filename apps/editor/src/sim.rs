use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::engine::display::World;
use crate::engine::frame::Frame;
use crate::engine::input::InputFile;
use crate::engine::session::{Command, Engine, Event, InputSel, Probe, SessionInfo, Source};
use crate::memo::Memo;
use gearbox_engine::Savestate;

pub struct Sim {
    engine: Engine,
    pub frames: Arc<Vec<Frame>>,
    generation: u64,
    rev: u64,
    crossings: Memo<u64, HashMap<u32, usize>>,
    pub len: usize,
    pub busy: Option<String>,
    pub stop: Option<(usize, String)>,
    pub rate: f64,
    pub savestates: usize,
    pub probe: Option<Probe>,
    pub info: Option<SessionInfo>,
    pub error: Option<String>,
}

#[derive(Default)]
pub struct Changes {
    pub opened: Option<Opened>,
    pub failed: Option<String>,
    pub frames: bool,
    pub probe: bool,
    pub errors: Vec<String>,
}

pub struct Opened {
    pub world: Arc<World>,
    pub origin: Savestate,
    pub inputs: InputFile,
    pub len: usize,
    pub world_error: Option<String>,
}

impl Sim {
    pub fn spawn(packs: std::path::PathBuf, wake: Arc<dyn Fn() + Send + Sync>) -> Self {
        let engine = Engine::spawn(packs, wake);
        Self {
            engine,
            frames: Arc::default(),
            generation: 0,
            rev: 0,
            crossings: Memo::default(),
            len: 0,
            busy: None,
            stop: None,
            rate: 0.0,
            savestates: 0,
            probe: None,
            info: None,
            error: None,
        }
    }

    pub fn open(&mut self, source: Source, inputs: InputSel, len: Option<usize>) {
        self.frames = Arc::default();
        self.rev += 1;
        self.info = None;
        self.error = None;
        self.engine.send(Command::Open { source, inputs, len });
    }

    pub fn simulate(&self, inputs: Arc<InputFile>, from: usize, len: usize, hold: Option<usize>) {
        self.engine.send(Command::Simulate { inputs, from_tick: from, len, hold });
    }

    pub fn set_start(&self, start: Option<(usize, gearbox_search::pose::Placement)>) {
        self.engine.send(Command::SetStart { start });
    }

    pub fn probe(&self, tick: usize, branches: bool) {
        self.engine.send(Command::Probe { tick, branches });
    }

    pub fn poll(&mut self) -> Changes {
        let mut ch = Changes::default();
        while let Ok(e) = self.engine.events.try_recv() {
            match e {
                Event::Opened { info, world, inputs, len, world_error, origin } => {
                    self.info = Some(info);
                    self.len = len;
                    self.error = None;
                    ch.opened = Some(Opened { world, origin, inputs, len, world_error });
                }
                Event::OpenFailed { error, .. } => {
                    self.error = Some(error.clone());
                    ch.failed = Some(error);
                }
            }
        }
        let s = self.engine.shared.lock().unwrap();
        if s.generation != self.generation {
            self.generation = s.generation;
            self.frames = Arc::new(s.frames.clone());
            self.rev += 1;
            ch.frames = true;
        }
        self.len = s.len;
        self.busy = s.busy.clone();
        if self.stop != s.stop {
            if let Some((t, m)) = &s.stop {
                ch.errors.push(format!("simulation stopped at tick {t}: {m}"));
            }
            self.stop = s.stop.clone();
        }
        self.rate = s.rate;
        self.savestates = s.savestates;
        if let Some(p) = &s.probe
            && self.probe.as_ref().is_none_or(|q| q.tick != p.tick || q.micros != p.micros)
        {
            self.probe = Some(p.clone());
            ch.probe = true;
        }
        ch
    }

    pub fn rev(&self) -> u64 {
        self.rev
    }

    pub fn first_crossings(&self) -> Rc<HashMap<u32, usize>> {
        self.crossings.get(self.rev, || {
            let mut first = HashMap::new();
            for (k, f) in self.frames.iter().enumerate() {
                if let Some((w, _)) = f.last_crossing {
                    first.entry(w).or_insert(k);
                }
            }
            first
        })
    }

    pub fn done(&self) -> bool {
        self.busy.is_none()
            && !self.frames.is_empty()
            && (self.frames.len() > self.len || self.stop.is_some())
    }
}

pub struct Waker {
    pending: AtomicBool,
    tx: futures::channel::mpsc::UnboundedSender<()>,
}

impl Waker {
    pub fn new() -> (Arc<Self>, futures::channel::mpsc::UnboundedReceiver<()>) {
        let (tx, rx) = futures::channel::mpsc::unbounded();
        (Arc::new(Self { pending: AtomicBool::new(false), tx }), rx)
    }
    pub fn wake(&self) {
        if !self.pending.swap(true, Ordering::AcqRel) {
            let _ = self.tx.unbounded_send(());
        }
    }
    pub fn taken(&self) {
        self.pending.store(false, Ordering::Release);
    }
}
