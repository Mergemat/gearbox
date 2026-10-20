use std::path::Path;

use anyhow::{Context, Result, ensure};
use gearbox_engine::{Controls, Session};
use gearbox_gbx::inputs::EventKind;
use gearbox_gbx::{Ghost, Map, Replay};
use gearbox_packs::packs::PackSet;
use gearbox_world::OpenOptions;

use crate::Run;

const COUNTDOWN_MS: i32 = -1500;
const OVERRUN_TICKS: usize = 100;

#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    pub run: Run,
    pub time_ms: i32,
    pub checkpoints_ms: Vec<i32>,
    pub player: Option<String>,
    pub build: Option<String>,
}

#[derive(Debug)]
pub struct Unsupported(pub String);

impl std::fmt::Display for Unsupported {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Unsupported {}

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
        let events: Vec<_> = inputs
            .events
            .iter()
            .filter(|e| {
                !matches!(e.kind, EventKind::Horn(_)) && !(e.tick == 0 && e.kind == EventKind::Respawn)
            })
            .collect();
        if let Some(e) = events.first() {
            let first = format!("{:?} at tick {}", e.kind, e.tick);
            return Err(Unsupported(format!("{} input events, first {first}", events.len())).into());
        }
        if let Some(tick) = inputs.truncated_at {
            return Err(Unsupported(format!("inputs end early, at tick {tick}")).into());
        }
        ensure!(inputs.ticks.windows(2).any(|w| w[0] != w[1]), "ghost inputs never change");

        let t0 = if inputs.start_offset_ms >= 0 { COUNTDOWN_MS } else { inputs.start_offset_ms };
        let countdown = usize::try_from((inputs.start_offset_ms - t0) / 10)?;
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
            time_ms,
            checkpoints_ms: ghost.checkpoints_ms.iter().flatten().copied().collect(),
            player: ghost.nickname.clone().or_else(|| ghost.login.clone()),
            build: ghost.exe_version.clone(),
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
        let mut session = self.open(map, packs)?;
        let last = self.run.controls.last().copied().unwrap_or_default();
        let finish = |s: &Session| s.race().and_then(|r| r.finish).map(|t| t as i32);
        for &c in self.run.controls.iter().chain(std::iter::repeat_n(&last, OVERRUN_TICKS)) {
            session.step(c);
            if finish(&session).is_some() {
                break;
            }
        }
        Ok(Driven {
            finish_ms: finish(&session),
            waypoints: session.crossing_waypoints().len(),
            position: session.body().pos.to_array(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(build: &str) -> Record {
        Record {
            run: Run { t0: COUNTDOWN_MS, seed: None, controls: vec![] },
            time_ms: 0,
            checkpoints_ms: vec![],
            player: None,
            build: Some(build.into()),
        }
    }

    #[test]
    fn the_build_date_comes_from_the_exe_version() {
        let r = record("Trackmania date=2022-11-16_12_57 git=115165-0f026e419dc GameVersion=3.3.0");
        assert_eq!(r.build_date(), Some("2022-11-16"));
        assert_eq!(record("Trackmania GameVersion=3.3.0").build_date(), None);
    }
}
