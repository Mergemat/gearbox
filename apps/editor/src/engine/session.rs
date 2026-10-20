use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use anyhow::Result;
use gearbox_engine::{Controls, Savestate, Session};

use super::frame::Frame;
use crate::engine::display::{self, World};
use crate::engine::input::InputFile;

pub const SAVESTATE_EVERY: usize = 100;
const CHUNK: usize = 250;
pub const BRANCH_TICKS: usize = 150;
pub const MAX_TICKS: usize = 100 * 60 * 10;
pub const DEFAULT_TICKS: usize = 6000;

#[derive(Clone, Debug, PartialEq)]
pub enum InputSel {
    New,
    File(PathBuf),
    Memory(InputFile, String),
}

impl InputSel {
    pub fn label(&self) -> String {
        match self {
            Self::New => "new (empty)".into(),
            Self::File(p) => p.file_name().unwrap_or_default().to_string_lossy().into_owned(),
            Self::Memory(_, l) => l.clone(),
        }
    }
}

pub type Notify = Arc<dyn Fn() + Send + Sync>;

#[derive(Clone, Debug, PartialEq)]
pub struct Source(pub PathBuf);

impl Source {
    pub fn stem(&self) -> String {
        crate::catalog::map_stem(&self.0)
            .unwrap_or_else(|| self.0.file_name().unwrap_or_default().to_string_lossy().into_owned())
    }
}

#[derive(Clone, Debug)]
pub struct SessionInfo {
    pub stem: String,
    pub inputs_label: String,
    pub race_ms0: i32,
}

#[derive(Clone, Debug)]
pub struct Probe {
    pub tick: usize,
    pub stepped: usize,
    pub micros: u64,
    pub branches: Vec<Branch>,
}

#[derive(Clone, Debug)]
pub struct Branch {
    pub points: Vec<[f32; 3]>,
}

#[derive(Default)]
pub struct Shared {
    pub generation: u64,
    pub frames: Vec<Frame>,
    pub len: usize,
    pub stop: Option<(usize, String)>,
    pub busy: Option<String>,
    pub rate: f64,
    pub savestates: usize,
    pub probe: Option<Probe>,
}

pub enum Command {
    Open { source: Source, inputs: InputSel, len: Option<usize> },
    Simulate { inputs: Arc<InputFile>, from_tick: usize, len: usize, hold: Option<usize> },
    Probe { tick: usize, branches: bool },
    SetStart { start: Option<(usize, gearbox_search::pose::Placement)> },
}

pub enum Event {
    Opened {
        info: SessionInfo,
        world: Arc<World>,
        inputs: InputFile,
        len: usize,
        world_error: Option<String>,
        origin: Savestate,
    },
    OpenFailed {
        error: String,
    },
}

pub struct Engine {
    tx: Sender<Command>,
    pub events: Receiver<Event>,
    pub shared: Arc<Mutex<Shared>>,
}

impl Engine {
    pub fn spawn(packs: PathBuf, notify: Notify) -> Self {
        let (tx, rx) = mpsc::channel();
        let (etx, events) = mpsc::channel();
        let shared = Arc::new(Mutex::new(Shared::default()));
        let w = Worker { packs, rx, events: etx, shared: shared.clone(), notify };
        std::thread::Builder::new()
            .name("sim".into())
            .spawn(move || w.run())
            .expect("spawning the simulation thread");
        Self { tx, events, shared }
    }

    pub fn send(&self, c: Command) {
        let _ = self.tx.send(c);
    }
}

struct Worker {
    packs: PathBuf,
    rx: Receiver<Command>,
    events: Sender<Event>,
    shared: Arc<Mutex<Shared>>,
    notify: Notify,
}

enum Exit {
    Open(Source, InputSel, Option<usize>),
    Closed,
}

fn default_len(inputs: &InputFile, race_ms0: i32) -> usize {
    (((inputs.end_ms() - race_ms0).max(0) / 10) as usize).max(DEFAULT_TICKS)
}

struct Opened {
    runner: Session,
    inputs: InputFile,
    world: World,
    world_error: Option<String>,
    origin: Savestate,
}

fn panic_message(p: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = p.downcast_ref::<&str>() {
        s.to_string()
    } else if let Some(s) = p.downcast_ref::<String>() {
        s.clone()
    } else {
        "panic".into()
    }
}

impl Worker {
    fn run(mut self) {
        let mut next = None;
        loop {
            let (source, inputs, len) = match next.take() {
                Some(o) => o,
                None => match self.rx.recv() {
                    Ok(Command::Open { source, inputs, len }) => (source, inputs, len),
                    Ok(_) => continue,
                    Err(_) => return,
                },
            };
            let r = catch_unwind(AssertUnwindSafe(|| self.open(&source, &inputs, len)));
            match r.unwrap_or_else(|p| Err(anyhow::anyhow!("{}", panic_message(&*p)))) {
                Ok(Exit::Open(s, i, l)) => next = Some((s, i, l)),
                Ok(Exit::Closed) => return,
                Err(e) => {
                    self.set_busy(None);
                    let _ = self.events.send(Event::OpenFailed { error: format!("{e:#}") });
                    (self.notify)();
                }
            }
        }
    }

    fn set_busy(&self, b: Option<String>) {
        self.shared.lock().unwrap().busy = b;
        (self.notify)();
    }

    fn open(&mut self, source: &Source, sel: &InputSel, len: Option<usize>) -> Result<Exit> {
        let stem = source.stem();
        self.set_busy(Some(format!("Loading {stem} …")));
        {
            let mut s = self.shared.lock().unwrap();
            *s = Shared { busy: s.busy.take(), ..Default::default() };
            s.generation += 1;
        }
        let given = match sel {
            InputSel::New => None,
            InputSel::File(p) => Some(InputFile::read(p)?),
            InputSel::Memory(f, _) => Some(f.clone()),
        };
        self.set_busy(Some(format!("Building {stem} from the map and packs …")));
        let session = gearbox_search::session::open(&self.packs, &source.0, given.as_ref())?;
        let inputs = given.unwrap_or_else(|| InputFile::blank(session.race_ms()));
        self.set_busy(Some(format!("Reading the collision world of {stem} …")));
        let (world, world_error) = match catch_unwind(AssertUnwindSafe(|| display::extract(&session))) {
            Ok(w) => (w, None),
            Err(p) => (World::default(), Some(panic_message(&*p))),
        };
        let origin = session.save();
        self.start(Opened { runner: session, inputs, world, world_error, origin }, &stem, sel, len)
    }

    fn start(&mut self, o: Opened, stem: &str, sel: &InputSel, len: Option<usize>) -> Result<Exit> {
        let Opened { runner, inputs, world, world_error, origin } = o;
        let race_ms0 = runner.race_ms();
        let stem = stem.to_string();
        let len = len.unwrap_or_else(|| default_len(&inputs, race_ms0)).clamp(1, MAX_TICKS);
        let info = SessionInfo { stem, inputs_label: sel.label(), race_ms0 };
        let mut s = Simulation {
            race_ms0,
            inputs: Arc::new(inputs.clone()),
            states: Vec::new(),
            cur: 0,
            len,
            hold: None,
            stop_at: None,
            probe: None,
            start: None,
            runner,
        };
        let f0 = Frame::read(&s.runner, Controls::default());
        s.save_state();
        {
            let mut sh = self.shared.lock().unwrap();
            sh.frames = vec![f0];
            sh.len = len;
            sh.generation += 1;
            sh.busy = None;
            sh.savestates = 1;
        }
        let _ = self.events.send(Event::Opened {
            info,
            world: Arc::new(world),
            inputs,
            len,
            world_error,
            origin: origin.clone(),
        });
        (self.notify)();
        self.session(s)
    }

    fn session(&mut self, mut s: Simulation) -> Result<Exit> {
        loop {
            let busy = s.has_work(&self.shared);
            let mut cmds = Vec::new();
            if !busy {
                match self.rx.recv() {
                    Ok(c) => cmds.push(c),
                    Err(_) => return Ok(Exit::Closed),
                }
            }
            loop {
                match self.rx.try_recv() {
                    Ok(c) => cmds.push(c),
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return Ok(Exit::Closed),
                }
            }
            for c in cmds {
                match c {
                    Command::Open { source, inputs, len } => return Ok(Exit::Open(source, inputs, len)),
                    Command::Simulate { inputs, from_tick, len, hold } => {
                        s.edit(&self.shared, inputs, from_tick, len, hold)
                    }
                    Command::Probe { tick, branches } => s.probe = Some((tick, branches)),
                    Command::SetStart { start } => s.set_start(&self.shared, start),
                }
            }
            if s.simulate_chunk(&self.shared) {
                (self.notify)();
                continue;
            }
            if let Some((tick, branches)) = s.probe.take() {
                let p = s.run_probe(tick, branches, &self.shared);
                if let Some(p) = p {
                    self.shared.lock().unwrap().probe = Some(p);
                    (self.notify)();
                }
            }
        }
    }
}

struct Simulation {
    runner: Session,
    race_ms0: i32,
    inputs: Arc<InputFile>,
    states: Vec<Savestate>,
    cur: usize,
    len: usize,
    hold: Option<usize>,
    stop_at: Option<usize>,
    probe: Option<(usize, bool)>,
    start: Option<(usize, gearbox_search::pose::Placement)>,
}

impl Simulation {
    fn control(&self, tick: usize) -> Controls {
        self.inputs.at(self.race_ms0 + 10 * tick as i32)
    }

    fn save_state(&mut self) {
        let st = self.runner.save();
        self.states.push(st);
    }

    fn goto(&mut self, tick: usize) -> Result<usize, String> {
        if self.cur == tick {
            return Ok(0);
        }
        let j = (tick / SAVESTATE_EVERY).min(self.states.len() - 1);
        self.runner.restore(&self.states[j]);
        self.cur = j * SAVESTATE_EVERY;
        self.arrive()?;
        let n = tick - self.cur;
        for _ in 0..n {
            let c = self.control(self.cur);
            self.step(c)?;
            self.arrive()?;
        }
        Ok(n)
    }

    fn arrive(&mut self) -> Result<(), String> {
        let Some((tick, placement)) = &self.start else { return Ok(()) };
        if *tick != self.cur {
            return Ok(());
        }
        let placement = placement.clone();
        match catch_unwind(AssertUnwindSafe(|| gearbox_search::pose::place(&mut self.runner, &placement))) {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => {
                self.cur = usize::MAX;
                Err(format!("placing the start state: {e:#}"))
            }
            Err(p) => {
                self.cur = usize::MAX;
                Err(format!("placing the start state: {}", panic_message(&*p)))
            }
        }
    }

    fn set_start(&mut self, shared: &Mutex<Shared>, start: Option<(usize, gearbox_search::pose::Placement)>) {
        let from = match (self.start.as_ref().map(|s| s.0), start.as_ref().map(|s| s.0)) {
            (Some(a), Some(b)) => a.min(b),
            (Some(a), None) | (None, Some(a)) => a,
            (None, None) => return,
        };
        self.start = start;
        if self.stop_at.is_some_and(|s| s >= from) {
            self.stop_at = None;
        }
        self.states.truncate(from / SAVESTATE_EVERY + 1);
        if self.cur != usize::MAX && self.cur >= from {
            self.cur = usize::MAX;
        }
        let mut s = shared.lock().unwrap();
        if s.frames.len() > from {
            s.frames.truncate(from);
            s.generation += 1;
        }
        if self.stop_at.is_none() {
            s.stop = None;
        }
        s.savestates = self.states.len();
    }

    fn step(&mut self, c: Controls) -> Result<(), String> {
        match catch_unwind(AssertUnwindSafe(|| self.runner.step(c))) {
            Ok(()) => {
                self.cur += 1;
                Ok(())
            }
            Err(p) => {
                self.cur = usize::MAX;
                Err(panic_message(&*p))
            }
        }
    }

    fn end(&self) -> usize {
        self.len.min(self.stop_at.unwrap_or(usize::MAX)).min(self.hold.unwrap_or(usize::MAX))
    }

    fn has_work(&self, shared: &Mutex<Shared>) -> bool {
        let have = shared.lock().unwrap().frames.len();
        have < self.end() + 1 || self.probe.is_some()
    }

    fn edit(
        &mut self,
        shared: &Mutex<Shared>,
        inputs: Arc<InputFile>,
        from_tick: usize,
        len: usize,
        hold: Option<usize>,
    ) {
        let from_tick = if inputs.seed != self.runner.seed {
            self.runner.restore(&self.states[0]);
            self.runner.seed = inputs.seed;
            self.states[0] = self.runner.save();
            self.cur = usize::MAX;
            0
        } else {
            from_tick
        };
        self.inputs = inputs;
        self.len = len.clamp(1, MAX_TICKS);
        self.hold = hold;
        let keep = from_tick + 1;
        if self.stop_at.is_some_and(|s| s >= from_tick) {
            self.stop_at = None;
        }
        self.states.truncate(from_tick / SAVESTATE_EVERY + 1);
        if self.cur != usize::MAX && self.cur > from_tick {
            self.cur = usize::MAX;
        }
        let mut s = shared.lock().unwrap();
        if s.frames.len() > keep {
            s.frames.truncate(keep);
            s.generation += 1;
        }
        if s.frames.len() > self.len + 1 {
            s.frames.truncate(self.len + 1);
            s.generation += 1;
        }
        if self.stop_at.is_none() {
            s.stop = None;
        }
        s.len = self.len;
        s.savestates = self.states.len();
    }

    fn simulate_chunk(&mut self, shared: &Mutex<Shared>) -> bool {
        let have = shared.lock().unwrap().frames.len();
        let end = self.end();
        if have > end {
            return false;
        }
        let t = Instant::now();
        let mut out = Vec::with_capacity(CHUNK);
        let mut stop = None;
        if have == 0 {
            if let Err(e) = self.goto(0) {
                shared.lock().unwrap().stop = Some((0, e.clone()));
                self.stop_at = Some(0);
                return true;
            }
            let f = Frame::read(&self.runner, Controls::default());
            let mut s = shared.lock().unwrap();
            s.frames.push(f);
            s.generation += 1;
            return true;
        }
        let mut k = have - 1;
        if let Err(e) = self.goto(k) {
            stop = Some((k, e));
        } else {
            while k < end && out.len() < CHUNK {
                let c = self.control(k);
                if let Err(e) = self.step(c) {
                    stop = Some((k, e));
                    break;
                }
                k += 1;
                if k.is_multiple_of(SAVESTATE_EVERY) && self.states.len() == k / SAVESTATE_EVERY {
                    self.save_state();
                }
                if let Err(e) = self.arrive() {
                    stop = Some((k - 1, e));
                    break;
                }
                let f = catch_unwind(AssertUnwindSafe(|| Frame::read(&self.runner, c)));
                match f {
                    Ok(f) => out.push(f),
                    Err(p) => {
                        stop = Some((k - 1, panic_message(&*p)));
                        break;
                    }
                }
            }
        }
        let el = t.elapsed().as_secs_f64();
        if let Some((tick, _)) = &stop {
            self.stop_at = Some(*tick);
        }
        let mut s = shared.lock().unwrap();
        if !out.is_empty() {
            s.rate = out.len() as f64 / el.max(1e-9);
        }
        s.frames.extend(out);
        s.generation += 1;
        if stop.is_some() {
            s.stop = stop;
        }
        s.savestates = self.states.len();
        true
    }

    fn run_probe(&mut self, tick: usize, branches: bool, shared: &Mutex<Shared>) -> Option<Probe> {
        if tick >= shared.lock().unwrap().frames.len() {
            return None;
        }
        let t = Instant::now();
        let stepped = self.goto(tick).ok()?;
        let micros = t.elapsed().as_micros() as u64;
        let mut out = Vec::new();
        if branches {
            let base = self.runner.save();
            let c = self.control(tick);
            for c in [-1.0, 0.0, 1.0].map(|steer| Controls { steer, ..c }) {
                self.runner.restore(&base);
                let mut points = vec![self.runner.body().pos.to_array()];
                for _ in 0..BRANCH_TICKS {
                    if catch_unwind(AssertUnwindSafe(|| self.runner.step(c))).is_err() {
                        break;
                    }
                    points.push(self.runner.body().pos.to_array());
                }
                out.push(Branch { points });
            }
            self.runner.restore(&base);
            self.cur = tick;
        }
        Some(Probe { tick, stepped, micros, branches: out })
    }
}
