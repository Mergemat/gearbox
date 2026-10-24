use crate::Body;
use crate::search::MAX_THREADS;
use crate::{
    input::Timeline,
    mutation::Mutation,
    objective::Sample,
    pose::{Pose, PoseGoal},
    search::Origin,
    tricks::Observation,
};
use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::Controls;
use rand::{RngExt, SeedableRng, rngs::StdRng};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct Config {
    pub rollouts: usize,
    pub horizon_ticks: usize,
    pub sample_every: usize,
    pub threads: usize,
    pub seed: u64,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct PoseSample {
    pub tick: usize,
    pub pose: Pose,
}
#[derive(Clone, Debug)]
pub struct Rollout {
    pub index: usize,
    pub family: &'static str,
    pub inputs: Timeline,
    pub trajectory: Vec<PoseSample>,
    pub end: Pose,
}
pub struct Fan {
    pub rollouts: Vec<Rollout>,
    pub elapsed: Duration,
    pub simulated_ticks: u64,
}

pub const FAMILIES: [&str; 8] = [
    "left-gas",
    "right-gas",
    "straight-gas",
    "brake-tap",
    "gas-release",
    "steer-reversal",
    "trick-mutation",
    "trick-mutation",
];

fn inputs(
    t0: i32,
    seed: Option<u32>,
    config: &Config,
    index: usize,
    family: usize,
    trace: &[Sample],
) -> (&'static str, Timeline) {
    let mut rng =
        StdRng::seed_from_u64(config.seed.wrapping_add((index as u64).wrapping_mul(0x9e3779b97f4a7c15)));
    let mut t = Timeline {
        actions: Vec::new(),
        t0,
        seed,
        controls: vec![Controls { gas: 1.0, brake: 0.0, steer: 0.0 }; config.horizon_ticks],
    };
    let sign = if rng.random::<bool>() { 1.0 } else { -1.0 };
    let amplitude = if index < 8 { 1.0 } else { rng.random_range(0.05..=1.0) };
    let at = rng.random_range(0..config.horizon_ticks);
    let duration = rng.random_range(1..=20).min(config.horizon_ticks - at);
    match family {
        0 => {
            for c in &mut t.controls {
                c.steer = -amplitude;
            }
        }
        1 => {
            for c in &mut t.controls {
                c.steer = amplitude;
            }
        }
        2 => {}
        3 => {
            for c in &mut t.controls[at..at + duration] {
                c.brake = 1.0;
            }
        }
        4 => {
            for c in &mut t.controls[at..at + duration] {
                c.gas = 0.0;
            }
        }
        5 => {
            for (i, c) in t.controls.iter_mut().enumerate() {
                c.steer = if i < at { sign * amplitude } else { -sign * amplitude };
            }
        }
        _ => Mutation::default().explore(&mut t, 0..config.horizon_ticks, trace, 10, index, &mut rng),
    }
    (FAMILIES[family], t)
}

pub fn generate(origin: &dyn Origin, config: &Config) -> Result<Fan> {
    generate_families(origin, config, &FAMILIES)
}

pub fn generate_families(origin: &dyn Origin, config: &Config, families: &[&str]) -> Result<Fan> {
    let slots: Vec<usize> = (0..FAMILIES.len()).filter(|&s| families.contains(&FAMILIES[s])).collect();
    ensure!(!slots.is_empty(), "no rollout family selected");
    ensure!(
        config.rollouts > 0 && config.horizon_ticks > 0 && config.sample_every > 0,
        "fan sizes must be positive"
    );
    ensure!((1..=MAX_THREADS).contains(&config.threads), "threads must be in 1..={MAX_THREADS}");
    let began = Instant::now();
    let mut probe = origin.native()?;
    let (t0, seed) = (probe.race_ms(), probe.seed);
    let duration_ms = u32::try_from(config.horizon_ticks)?.checked_mul(10).context("fan clock overflow")?;
    let until = i64::from(t0) + i64::from(duration_ms);
    ensure!(
        until <= i64::from(i32::MAX)
            && u64::from(probe.clock.now) + u64::from(duration_ms) <= u64::from(u32::MAX),
        "fan clock overflow"
    );
    let mut trace = Vec::with_capacity(config.horizon_ticks);
    for _ in 0..config.horizon_ticks {
        probe.step(Controls { gas: 1.0, ..Default::default() });
        let body = Body::from(probe.body());
        trace.push(Sample { ms: probe.race_ms(), observation: Observation::read(&mut probe, body) });
    }
    drop(probe);
    let next = AtomicUsize::new(0);
    let rollouts = std::thread::scope(|scope| -> Result<Vec<Rollout>> {
        let handles: Vec<_> = (0..config.threads.min(config.rollouts))
            .map(|_| {
                let (trace, next, slots) = (&trace, &next, &slots);
                scope.spawn(move || -> Result<Vec<Rollout>> {
                    let mut n = origin.native()?;
                    let start = n.save();
                    let mut result = Vec::new();
                    loop {
                        let index = next.fetch_add(1, Ordering::Relaxed);
                        if index >= config.rollouts {
                            break;
                        }
                        n.restore(&start);
                        let (family, inputs) =
                            inputs(t0, seed, config, index, slots[index % slots.len()], trace);
                        let mut trajectory =
                            vec![PoseSample { tick: 0, pose: Pose::from_body(Body::from(n.body())) }];
                        for (i, c) in inputs.controls.iter().enumerate() {
                            n.step(*c);
                            if (i + 1) % config.sample_every == 0 || i + 1 == config.horizon_ticks {
                                let pose = Pose::from_body(Body::from(n.body()));
                                pose.validate()?;
                                trajectory.push(PoseSample { tick: i + 1, pose });
                            }
                        }
                        let end = trajectory.last().unwrap().pose;
                        result.push(Rollout { index, family, inputs, trajectory, end });
                    }
                    Ok(result)
                })
            })
            .collect();
        let mut out = Vec::new();
        let mut failure = None;
        for handle in handles {
            match handle.join() {
                Ok(Ok(mut r)) => out.append(&mut r),
                Ok(Err(e)) => {
                    failure.get_or_insert(e);
                }
                Err(_) => {
                    failure.get_or_insert(anyhow::anyhow!("fan worker panicked"));
                }
            }
        }
        if let Some(e) = failure {
            bail!(e);
        }
        out.sort_by_key(|r| r.index);
        Ok(out)
    })?;
    Ok(Fan {
        rollouts,
        elapsed: began.elapsed(),
        simulated_ticks: (config.rollouts as u64 + 1) * config.horizon_ticks as u64,
    })
}

pub struct Snap<'a> {
    pub rollout: &'a Rollout,
    pub pose: Pose,
    pub tick: usize,
    pub inputs: Timeline,
    pub error: f64,
}
pub fn snap<'a>(goal: &PoseGoal, fan: &'a Fan) -> Result<Option<Snap<'a>>> {
    goal.validate()?;
    let best = fan
        .rollouts
        .iter()
        .flat_map(|r| r.trajectory.iter().map(move |s| (r, s, goal.error(s.pose))))
        .min_by(|a, b| {
            a.2.total_cmp(&b.2).then_with(|| a.1.tick.cmp(&b.1.tick)).then_with(|| a.0.index.cmp(&b.0.index))
        });
    Ok(best.map(|(r, s, error)| Snap {
        rollout: r,
        pose: s.pose,
        tick: s.tick,
        inputs: Timeline {
            actions: r.inputs.actions[..s.tick.min(r.inputs.actions.len())].to_vec(),
            t0: r.inputs.t0,
            seed: r.inputs.seed,
            controls: r.inputs.controls[..s.tick].to_vec(),
        },
        error,
    }))
}
