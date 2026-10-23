use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use gearbox_gbx::record::CLASS_VEHICLE_VIS;
use gearbox_gbx::{Ghost, Replay};

use crate::engine::input::InputFile;

pub fn load(path: &Path) -> Result<Replay> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    Replay::parse(&bytes).with_context(|| format!("parsing {}", path.display()))
}

pub fn driven_ghost(replay: &Replay) -> Result<&Ghost> {
    replay
        .ghosts
        .iter()
        .find(|g| g.race_time_ms.is_some() && !g.inputs.is_empty())
        .context("no finished ghost with inputs")
}

pub fn inputs(replay: &Replay, map: &Path, packs: &Path) -> Result<InputFile> {
    let record = gearbox_run::Record::from_replay(replay)?;
    let map = gearbox_gbx::Map::parse(&std::fs::read(map).with_context(|| format!("reading {}", map.display()))?)?;
    let run = record.applied(&map, &gearbox_packs::packs::PackSet::open(packs)?)?;
    Ok(InputFile { t0: run.t0, seed: run.seed, controls: run.controls })
}

pub fn replays_for_map(map: &Path) -> Vec<PathBuf> {
    let is_run = |p: &Path| {
        let n = p.file_name().unwrap_or_default().to_string_lossy().to_lowercase();
        n.ends_with(".replay.gbx") || n.ends_with(".ghost.gbx")
    };
    let files = |d: &Path| std::fs::read_dir(d).into_iter().flat_map(|rd| rd.flatten().map(|e| e.path()));
    let mut out = Vec::new();
    let stem = crate::catalog::map_stem(map).unwrap_or_default();
    let id: String = stem.chars().take_while(char::is_ascii_digit).collect();
    if let (false, Some(dir)) = (id.is_empty(), map.parent()) {
        let prefix = format!("{id}_");
        out.extend(files(dir).filter(|p| {
            is_run(p) && p.file_name().unwrap_or_default().to_string_lossy().starts_with(&prefix)
        }));
    }
    let uid = std::fs::read(map).ok().and_then(|b| gearbox_gbx::Map::parse(&b).ok()).and_then(|m| m.uid);
    if let (Some(uid), Some(dir)) = (uid, map.parent()) {
        let small = |p: &Path| std::fs::metadata(p).is_ok_and(|m| m.len() < 64 << 20);
        out.extend(files(dir).filter(|p| {
            is_run(p) && small(p) && load(p).is_ok_and(|r| r.map_uid.as_deref() == Some(uid.as_str()))
        }));
    }
    let mut out: Vec<PathBuf> = out.into_iter().filter_map(|p| p.canonicalize().ok()).collect();
    out.sort();
    out.dedup();
    out
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GhostSample {
    pub time_ms: i32,
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub speed: f32,
}

#[derive(Clone, Debug, Default)]
pub struct GhostTrack {
    pub path: PathBuf,
    pub name: String,
    pub race_time_ms: Option<i32>,
    pub checkpoints_ms: Vec<i32>,
    pub exe: Option<String>,
    pub samples: Vec<GhostSample>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GhostDelta {
    pub distance: Option<f32>,
    pub behind_s: Option<f32>,
}

const SAMPLE_LAG_MS: i32 = 0;

impl GhostTrack {
    pub fn read(path: &Path) -> Result<Self> {
        let replay = load(path)?;
        Self::from_replay(&replay, path)
    }

    pub fn from_replay(replay: &Replay, path: &Path) -> Result<Self> {
        let ghost = driven_ghost(replay)
            .ok()
            .filter(|g| g.record.is_some())
            .or_else(|| replay.ghosts.iter().find(|g| g.record.is_some()))
            .context("no ghost with a record")?;
        let samples: Vec<GhostSample> = ghost
            .record
            .iter()
            .flat_map(|r| &r.entities)
            .find(|e| e.class_id == CLASS_VEHICLE_VIS)
            .into_iter()
            .flat_map(|e| &e.samples)
            .filter_map(|s| {
                s.vehicle.as_ref().map(|v| GhostSample {
                    time_ms: s.time_ms,
                    position: v.position,
                    rotation: v.rotation,
                    speed: v.speed,
                })
            })
            .collect();
        ensure!(samples.len() > 1, "the ghost's record has no vehicle samples");
        let file = path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .replace(".Replay.Gbx", "")
            .replace(".Ghost.Gbx", "");
        Ok(Self {
            path: path.to_path_buf(),
            name: ghost
                .nickname
                .clone()
                .filter(|n| !n.is_empty())
                .map_or(file.clone(), |n| format!("{n} ({file})")),
            race_time_ms: ghost.race_time_ms,
            checkpoints_ms: ghost.checkpoints_ms.iter().flatten().copied().collect(),
            exe: ghost.exe_version.clone(),
            samples,
        })
    }

    pub fn at(&self, race_ms: i32) -> Option<([f32; 3], [f32; 4], f32)> {
        let t = (race_ms - SAMPLE_LAG_MS) as f32;
        let s = &self.samples;
        if t < s.first()?.time_ms as f32 || t > s.last()?.time_ms as f32 {
            return None;
        }
        let i = s.partition_point(|x| (x.time_ms as f32) <= t).clamp(1, s.len() - 1);
        let (a, b) = (&s[i - 1], &s[i]);
        let u = ((t - a.time_ms as f32) / (b.time_ms - a.time_ms).max(1) as f32).clamp(0.0, 1.0);
        let lerp = |x: f32, y: f32| x + (y - x) * u;
        let p = std::array::from_fn(|k| lerp(a.position[k], b.position[k]));
        let dot: f32 = (0..4).map(|k| a.rotation[k] * b.rotation[k]).sum();
        let sign = if dot < 0.0 { -1.0 } else { 1.0 };
        let mut q: [f32; 4] = std::array::from_fn(|k| lerp(a.rotation[k], sign * b.rotation[k]));
        let n = q.iter().map(|x| x * x).sum::<f32>().sqrt().max(1e-9);
        q.iter_mut().for_each(|x| *x /= n);
        Some((p, q, lerp(a.speed, b.speed)))
    }

    pub fn delta(&self, race_ms: i32, pos: [f32; 3]) -> GhostDelta {
        let distance = self.at(race_ms).map(|(g, _, _)| dist(g, pos));
        let mut best: Option<(f32, f32)> = None;
        for w in self.samples.windows(2) {
            let (a, b) = (&w[0], &w[1]);
            if (a.time_ms + SAMPLE_LAG_MS - race_ms).abs() > 20_000 {
                continue;
            }
            let ab: [f32; 3] = std::array::from_fn(|k| b.position[k] - a.position[k]);
            let ap: [f32; 3] = std::array::from_fn(|k| pos[k] - a.position[k]);
            let len2: f32 = ab.iter().map(|x| x * x).sum();
            let u = if len2 > 0.0 {
                ((0..3).map(|k| ab[k] * ap[k]).sum::<f32>() / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let q = std::array::from_fn(|k| a.position[k] + ab[k] * u);
            let d = dist(q, pos);
            if best.is_none_or(|(bd, _)| d < bd) {
                let time = a.time_ms as f32 + (b.time_ms - a.time_ms) as f32 * u + SAMPLE_LAG_MS as f32;
                best = Some((d, time));
            }
        }
        let behind_s = best.filter(|(d, _)| *d <= 25.0).map(|(_, t)| (race_ms as f32 - t) / 1000.0);
        GhostDelta { distance, behind_s }
    }
}

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    (0..3).map(|k| (a[k] - b[k]) * (a[k] - b[k])).sum::<f32>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight() -> GhostTrack {
        let samples = (0..200)
            .map(|i| GhostSample {
                time_ms: 50 * i,
                position: [0.5 * i as f32, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
                speed: 10.0,
            })
            .collect();
        GhostTrack { samples, ..Default::default() }
    }

    #[test]
    fn ghost_position_interpolates_between_samples() {
        let g = straight();
        assert_eq!(g.at(1000).unwrap().0, [10.0, 0.0, 0.0]);
        assert!((g.at(1025).unwrap().0[0] - 10.25).abs() < 1e-5);
        assert!(g.at(-5).is_none() && g.at(9955).is_none());
    }

    #[test]
    fn delta_reports_time_behind_and_distance() {
        let g = straight();
        let d = g.delta(600, [5.0, 0.0, 0.0]);
        assert!((d.behind_s.unwrap() - 0.1).abs() < 1e-4, "{d:?}");
        assert!((d.distance.unwrap() - 1.0).abs() < 1e-4, "{d:?}");
        assert!(g.delta(400, [5.0, 1.0, 0.0]).behind_s.unwrap() < 0.0);
        assert_eq!(g.delta(600, [5.0, 40.0, 0.0]).behind_s, None);
    }

    #[test]
    fn import_matches_rig_inputs_export() {
        let Some(data) = std::env::var_os("GBX_TEST_DATA").map(PathBuf::from) else {
            eprintln!("skipped: GBX_TEST_DATA unset");
            return;
        };
        let replay = data.join("tmx/effects/1993_228726.Replay.Gbx");
        let map = data.join("tmx/effects/1993_Training_-_09.Map.Gbx");
        let expected = data.join("inputs/xeff1993s.txt.in");
        let Ok(game) = gearbox_packs::install::Game::locate(None) else { return };
        if !replay.exists() || !expected.exists() || !map.exists() {
            eprintln!("skipped: no {}", replay.display());
            return;
        }
        let got = inputs(&load(&replay).unwrap(), &map, &game.packs).unwrap();
        let rig = InputFile::read(&expected).unwrap();
        assert_eq!((got.t0, got.seed, got.controls.len()), (rig.t0, rig.seed, rig.controls.len()));
        for (i, (g, r)) in got.controls.iter().zip(&rig.controls).enumerate() {
            let limited = [0.2, 0.4, 0.6, 0.8, 1.0].iter().any(|k| g.steer == r.steer * k);
            assert!((g.gas, g.brake) == (r.gas, r.brake) && limited, "tick {i}: {g:?} vs {r:?}");
        }
        assert!(got.controls.iter().zip(&rig.controls).any(|(g, r)| g.steer != r.steer));
        let ghost = GhostTrack::read(&replay).unwrap();
        assert!(ghost.samples.len() > 100 && ghost.race_time_ms.is_some());
    }
}
