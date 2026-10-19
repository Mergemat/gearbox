use std::path::{Path, PathBuf};

use crate::file::Trace;
use crate::state::{Controls, FieldDiff, TickState, first_diff};

pub type Error = Box<dyn std::error::Error + Send + Sync>;

pub trait Engine {
    fn step(&mut self, c: Controls);
    fn state(&mut self) -> TickState;
}

#[derive(Clone, Debug)]
pub struct Setup {
    pub map: PathBuf,
    pub t0: i32,
    pub seed: Option<u32>,
    pub packs: Option<PathBuf>,
}

pub trait Factory {
    fn name(&self) -> String;
    fn open(&self, setup: &Setup) -> Result<Box<dyn Engine>, Error>;
}

pub fn record(engine: &mut dyn Engine, steps: &[Controls]) -> Vec<TickState> {
    let mut out = Vec::with_capacity(steps.len() + 1);
    out.push(engine.state());
    for c in steps {
        engine.step(*c);
        out.push(engine.state());
    }
    out
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Exact { states: usize },
    Diff { tick: usize, race_ms: i32, diff: FieldDiff },
}

pub fn check(engine: &mut dyn Engine, trace: &Trace) -> Outcome {
    let mut tick = 0;
    loop {
        let want = &trace.states[tick];
        if let Some(diff) = first_diff(want, &engine.state()) {
            return Outcome::Diff { tick, race_ms: want.race_ms, diff };
        }
        let Some(c) = trace.case.steps.get(tick) else { break };
        engine.step(*c);
        tick += 1;
    }
    Outcome::Exact { states: trace.states.len() }
}

pub struct Perturbed {
    pub inner: Box<dyn Engine>,
    pub tick: usize,
    pub at: usize,
    pub word: usize,
}

impl Engine for Perturbed {
    fn step(&mut self, c: Controls) {
        self.inner.step(c);
        self.tick += 1;
    }
    fn state(&mut self) -> TickState {
        let s = self.inner.state();
        if self.tick != self.at {
            return s;
        }
        let mut w = s.words();
        w[self.word] ^= 1;
        TickState::from_words(&w, s.race)
    }
}

pub fn find_map(trace: &Trace, dir: &Path, search: &[PathBuf]) -> Option<PathBuf> {
    let m = &trace.case.map;
    let mut candidates = vec![PathBuf::from(&m.path)];
    for d in std::iter::once(&dir.join("maps")).chain(search) {
        candidates.push(d.join(&m.file));
        if let (Some(id), Ok(rd)) = (m.tmx, std::fs::read_dir(d)) {
            let prefix = format!("{id}_");
            let mut found: Vec<PathBuf> = rd.flatten().map(|e| e.path()).filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with(&prefix))).collect();
            found.sort();
            candidates.extend(found);
        }
    }
    candidates.into_iter().find(|p| {
        std::fs::metadata(p).is_ok_and(|md| md.len() == m.len) && std::fs::read(p).is_ok_and(|b| crate::file::fnv64(&b) == m.hash)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::file::{Case, MapRef};
    use crate::state::F32;

    struct Toy(TickState);
    impl Engine for Toy {
        fn step(&mut self, c: Controls) {
            self.0.race_ms += 10;
            self.0.body.position[0] = F32::new(self.0.body.position[0].get() + c.steer);
        }
        fn state(&mut self) -> TickState {
            self.0.clone()
        }
    }

    fn trace() -> Trace {
        let steps: Vec<Controls> = (0..6).map(|k| Controls { gas: 1.0, brake: 0.0, steer: k as f32 * 0.25 }).collect();
        let mut e = Toy(TickState { race_ms: -20, ..TickState::default() });
        let states = record(&mut e, &steps);
        let map = MapRef { file: String::new(), path: String::new(), hash: 0, len: 0, tmx: None };
        Trace { case: Case { id: "toy".into(), map, source: String::new(), t0: -20, seed: None, start_ms: -20, steps }, engine: "toy".into(), states }
    }

    #[test]
    fn check_passes_the_same_engine_and_names_the_first_perturbed_word() {
        let t = trace();
        assert_eq!(t.states.len(), 7);
        let fresh = || Box::new(Toy(TickState { race_ms: -20, ..TickState::default() }));
        assert_eq!(check(&mut *fresh(), &t), Outcome::Exact { states: 7 });
        let names = TickState::names();
        let word = names.iter().position(|(n, _)| n.to_string() == "wheels[1].steer").unwrap();
        let mut p = Perturbed { inner: fresh(), tick: 0, at: 4, word };
        match check(&mut p, &t) {
            Outcome::Diff { tick: 4, race_ms: 20, diff } => assert_eq!(diff.field, "wheels[1].steer"),
            other => panic!("{other:?}"),
        }
        let mut other = t.clone();
        other.case.steps[2].steer = 0.0;
        match check(&mut *fresh(), &other) {
            Outcome::Diff { tick, diff, .. } => assert_eq!((tick, diff.field.as_str()), (3, "body.position.x")),
            o => panic!("{o:?}"),
        }
    }
}
