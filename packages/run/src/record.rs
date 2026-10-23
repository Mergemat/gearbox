use std::path::Path;

use anyhow::{Context, Result, ensure};
use gearbox_engine::{Controls, Session, race::replay::InputState};
use gearbox_gbx::inputs::{DecodedInputs, EventKind};
use gearbox_gbx::{Ghost, Map, Replay};
use gearbox_packs::packs::PackSet;
use gearbox_world::OpenOptions;

use crate::Run;

const COUNTDOWN_MS: i32 = -1500;
const OVERRUN_TICKS: usize = 100;

#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    pub run: Run,
    pub respawns: Vec<Respawn>,
    pub time_ms: i32,
    pub checkpoints_ms: Vec<i32>,
    pub player: Option<String>,
    pub build: Option<String>,
    input_states: Vec<InputState>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Respawn {
    pub tick: usize,
    pub standing: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Driven {
    pub finish_ms: Option<i32>,
    pub waypoints: usize,
    pub position: [f32; 3],
}

impl Record {
    pub fn read(path: &Path) -> Result<Self> {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        Self::parse(&bytes).with_context(|| format!("reading the record in {}", path.display()))
    }

    pub fn parse(bytes: &[u8]) -> Result<Self> {
        Self::from_replay(&Replay::parse(bytes)?)
    }

    pub fn from_replay(replay: &Replay) -> Result<Self> {
        let ghost = replay
            .ghosts
            .iter()
            .find(|g| g.race_time_ms.is_some() && !g.inputs.is_empty())
            .context("no finished ghost with inputs")?;
        Self::from_ghost(ghost)
    }

    pub fn from_ghost(ghost: &Ghost) -> Result<Self> {
        let time_ms = ghost.race_time_ms.context("the ghost didn't finish")?;
        let inputs = ghost.inputs.first().context("the ghost has no inputs")?.decode()?;
        let respawns: Vec<Respawn> = inputs
            .events
            .iter()
            .filter(|e| e.tick != 0)
            .filter_map(|e| match e.kind {
                EventKind::Respawn => Some(Respawn { tick: e.tick, standing: false }),
                EventKind::SecondaryRespawn => Some(Respawn { tick: e.tick, standing: true }),
                EventKind::Horn(_) => None,
            })
            .collect();
        ensure!(!inputs.ticks.is_empty(), "the ghost has no input ticks");

        let t0 = if inputs.start_offset_ms >= 0 { COUNTDOWN_MS } else { inputs.start_offset_ms };
        let countdown = usize::try_from((inputs.start_offset_ms - t0) / 10)?;
        let respawns = respawns.into_iter().map(|r| Respawn { tick: r.tick + countdown, ..r }).collect();
        let input_states = input_states(&inputs, countdown);
        let driven = inputs.ticks.iter().map(|t| Controls {
            gas: f32::from(u8::from(t.gas)),
            brake: f32::from(u8::from(t.brake)),
            steer: f32::from(t.steer) / 127.0,
        });
        Ok(Self {
            run: Run {
                t0,
                seed: ghost.validation_seed.map(|s| s as u32).filter(|&s| s != 0),
                controls: std::iter::repeat_n(Controls::default(), countdown).chain(driven).collect(),
            },
            respawns,
            time_ms,
            checkpoints_ms: ghost.checkpoints_ms.iter().flatten().copied().collect(),
            player: ghost.nickname.clone().or_else(|| ghost.login.clone()),
            build: ghost.exe_version.clone(),
            input_states,
        })
    }

    pub fn build_date(&self) -> Option<&str> {
        let build = self.build.as_deref()?;
        let at = build.find("date=")? + "date=".len();
        build.get(at..at + 10)
    }

    pub fn open(&self, map: &Map, packs: &PackSet) -> Result<Session> {
        gearbox_world::open(map, packs, OpenOptions { t0: self.run.t0, seed: self.run.seed, car: None })
    }

    pub fn drive(&self, map: &Map, packs: &PackSet) -> Result<Driven> {
        Ok(self.drive_from(self.open(map, packs)?))
    }

    pub fn applied(&self, map: &Map, packs: &PackSet) -> Result<Run> {
        let mut session = self.open(map, packs)?;
        self.prepare(&mut session);
        let mut controls = Vec::with_capacity(self.run.controls.len());
        for tick in 0..self.run.controls.len() {
            ensure!(self.step(&mut session, tick), "respawn at tick {tick} without a spawn");
            controls.push(session.applied);
        }
        Ok(Run { t0: self.run.t0, seed: self.run.seed, controls })
    }

    fn prepare(&self, session: &mut Session) {
        session.seed = self.run.seed;
        if !self.respawns.is_empty() {
            session.track_respawns();
        }
    }

    pub fn drive_from(&self, mut session: Session) -> Driven {
        self.prepare(&mut session);
        let finish = |s: &Session| s.race().and_then(|r| r.finish).map(|t| t as i32);
        let ticks = ((self.time_ms - self.run.t0).max(0) as usize).div_ceil(10);
        for tick in 0..ticks.max(self.run.controls.len()) + OVERRUN_TICKS {
            if !self.step(&mut session, tick) || finish(&session).is_some() {
                break;
            }
        }
        Driven {
            finish_ms: finish(&session),
            waypoints: session.crossing_waypoints().len(),
            position: session.vehicle().pos.to_array(),
        }
    }

    pub fn step(&self, session: &mut Session, tick: usize) -> bool {
        session.seed = self.run.seed;
        for respawn in self.respawns.iter().filter(|r| r.tick == tick) {
            if !session.respawn(respawn.standing) {
                return false;
            }
        }
        let controls = self.run.controls.get(tick).or(self.run.controls.last()).copied().unwrap_or_default();
        let input = self.input_states.get(tick).or(self.input_states.last()).copied().unwrap_or_default();
        session.step_with_input(controls, input);
        true
    }
}

fn input_states(inputs: &DecodedInputs, countdown: usize) -> Vec<InputState> {
    let mut words = inputs.state_words.iter().peekable();
    let mut extras = inputs.extra_words.iter().peekable();
    let mut state = InputState::default();
    let mut states = vec![state; countdown];
    for (tick, controls) in inputs.ticks.iter().enumerate() {
        while let Some(&(_, word)) = words.next_if(|(t, _)| *t <= tick) {
            let packed = InputState::from_packed_bits(word);
            state.bits = packed.bits;
            state.buttons = packed.buttons;
        }
        while let Some(&(_, word)) = extras.next_if(|(t, _)| *t <= tick) {
            state.shorts = [word as u16, (word >> 16) as u16];
        }
        state.axes = [
            f32::from(controls.steer) / 127.0,
            f32::from(u8::from(controls.gas)),
            f32::from(u8::from(controls.brake)),
            0.0,
        ];
        states.push(state);
    }
    states
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(build: &str) -> Record {
        Record {
            run: Run { t0: COUNTDOWN_MS, seed: None, controls: vec![] },
            respawns: vec![],
            time_ms: 0,
            checkpoints_ms: vec![],
            player: None,
            build: Some(build.into()),
            input_states: Vec::new(),
        }
    }

    #[test]
    fn the_build_date_comes_from_the_exe_version() {
        let r = record("Trackmania date=2022-11-16_12_57 git=115165-0f026e419dc GameVersion=3.3.0");
        assert_eq!(r.build_date(), Some("2022-11-16"));
        assert_eq!(record("Trackmania GameVersion=3.3.0").build_date(), None);
    }
}
