use crate::Body;
use std::ops::Range;

use anyhow::Result;
use gearbox_engine::Controls;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::follow::steer_toward;
use crate::input::Timeline;
use crate::mutation::{Constraints, Mutation};
use crate::objective::{EvalOptions, Objective, Score, evaluate_with};
use crate::pose::{Placement, place};
use crate::search::Origin;

const POOL: usize = 6;
const DIFFERENT_M: f32 = 4.0;
const SHAPE_POINTS: usize = 16;
const MIN_SHARE: f64 = 0.08;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    Ticks,
    Timing,
    Edit,
    Turn,
    Cut,
    Car,
}

pub const CHANGES: [Change; 6] =
    [Change::Ticks, Change::Timing, Change::Edit, Change::Turn, Change::Cut, Change::Car];

impl Change {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ticks => "ticks",
            Self::Timing => "timing",
            Self::Edit => "edit",
            Self::Turn => "turn-in",
            Self::Cut => "cut",
            Self::Car => "car",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Settings {
    pub threads: usize,
    pub seed: u64,
    pub constraints: Constraints,
    pub eval: EvalOptions,
    pub placement: Option<Placement>,
}

#[derive(Clone, Debug)]
pub struct Line {
    pub inputs: Timeline,
    pub score: Score,
    pub trail: Vec<[f32; 3]>,
    pub change: Option<Change>,
    pub id: usize,
    pub placement: Option<Placement>,
}

pub struct Progress<'a> {
    pub tries: usize,
    pub lines: &'a [Line],
    pub latest: &'a [(Vec<[f32; 3]>, bool)],
    pub changes: [(Change, u32, u32); 6],
}

pub fn improve(
    origin: &dyn Origin,
    objective: &Objective,
    start: Timeline,
    range: Range<usize>,
    until: usize,
    s: &Settings,
    mut round: impl FnMut(&Progress) -> bool,
) -> Result<Vec<Line>> {
    anyhow::ensure!(range.start < range.end && range.end <= until, "the range must lie before Drive until");
    let from_ms = start.t0 + 10 * range.start as i32;
    let until_ms = start.t0 + 10 * until as i32;
    let judge = |inputs: &Timeline, car: Option<&Placement>| -> Result<(Score, Vec<[f32; 3]>)> {
        let mut n = native(origin, car)?;
        n.seed = inputs.seed;
        let ev = evaluate_with(
            &mut n,
            inputs,
            objective,
            until_ms,
            EvalOptions { from_ms: Some(from_ms), path_every: 1, trace: false, ..s.eval },
        )?;
        Ok((ev.score, ev.path))
    };
    let (score, trail) = judge(&start, s.placement.as_ref())?;
    let mut pool =
        vec![Line { inputs: start, score, trail, change: None, id: 0, placement: s.placement.clone() }];
    let mut rng = StdRng::seed_from_u64(s.seed);
    let mut weight = [1.0f64; 6];
    let allowed = CHANGES.map(|c| c != Change::Car || s.placement.is_some());
    let mut stats = CHANGES.map(|c| (c, 0u32, 0u32));
    let mut tries = 0;
    loop {
        let jobs: Vec<(usize, Change, u64)> = (0..s.threads.max(1))
            .map(|_| {
                let (a, b) = (rng.random_range(0..pool.len()), rng.random_range(0..pool.len()));
                let parent = if pool[a].score.quality_cmp(&pool[b].score).is_ge() { a } else { b };
                (parent, pick(&weight, &allowed, &mut rng), rng.random())
            })
            .collect();
        let pool_ref = &pool;
        let children: Vec<Result<Line>> = std::thread::scope(|scope| {
            let handles: Vec<_> = jobs
                .iter()
                .enumerate()
                .map(|(j, &(p, change, seed))| {
                    let id = tries + j + 1;
                    let range = range.clone();
                    scope.spawn(move || -> Result<Line> {
                        let parent = &pool_ref[p];
                        let mut rng = StdRng::seed_from_u64(seed);
                        let (inputs, placement) =
                            make(origin, parent, change, range, until, &s.constraints, &mut rng)?;
                        let (score, trail) = judge(&inputs, placement.as_ref())?;
                        Ok(Line { inputs, score, trail, change: Some(change), id, placement })
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|h| h.join().unwrap_or_else(|_| Err(anyhow::anyhow!("a try panicked"))))
                .collect()
        });
        let mut latest = Vec::new();
        for (child, &(_, change, _)) in children.into_iter().zip(&jobs) {
            let child = child?;
            tries += 1;
            let i = CHANGES.iter().position(|c| *c == change).unwrap_or(0);
            stats[i].1 += 1;
            let trail = child.trail.clone();
            let kept = keep(&mut pool, child);
            if kept {
                stats[i].2 += 1;
                weight[i] += 1.0;
            }
            latest.push((trail, kept));
        }
        for w in &mut weight {
            *w = 1.0 + (*w - 1.0) * 0.97;
        }
        pool.sort_by(|a, b| b.score.quality_cmp(&a.score));
        if !round(&Progress { tries, lines: &pool, latest: &latest, changes: stats }) {
            return Ok(pool);
        }
    }
}

fn pick(weight: &[f64; 6], allowed: &[bool; 6], rng: &mut StdRng) -> Change {
    let n = allowed.iter().filter(|a| **a).count() as f64;
    let total: f64 = weight.iter().zip(allowed).filter(|(_, a)| **a).map(|(w, _)| w).sum();
    let mut x = rng.random::<f64>();
    let mut last = CHANGES[0];
    for (i, w) in weight.iter().enumerate().filter(|(i, _)| allowed[*i]) {
        let p = MIN_SHARE + (1.0 - MIN_SHARE * n) * w / total;
        if x < p {
            return CHANGES[i];
        }
        x -= p;
        last = CHANGES[i];
    }
    last
}

fn native<'o>(origin: &'o dyn Origin, car: Option<&Placement>) -> Result<gearbox_engine::Session> {
    let mut n = origin.native()?;
    if let Some(p) = car {
        place(&mut n, p)?;
    }
    Ok(n)
}

fn turn_car(p: &Placement, rng: &mut StdRng) -> Placement {
    let mut angle =
        |max: f32| if rng.random_bool(0.5) { rng.random_range(-max..=max).to_radians() } else { 0.0 };
    let (yaw, pitch, roll) = (angle(25.0), angle(15.0), angle(15.0));
    let speed = rng.random_range(0.8..=1.2f32);
    let mul = |a: [f32; 4], b: [f32; 4]| {
        [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ]
    };
    let about = |axis: usize, a: f32| {
        let mut q = [(a / 2.0).cos(), 0.0, 0.0, 0.0];
        q[axis] = (a / 2.0).sin();
        q
    };
    let q = mul(mul(mul(p.pose.rotation, about(2, yaw)), about(1, pitch)), about(3, roll));
    let len = q.iter().map(|v| v * v).sum::<f32>().sqrt();
    let v = p.pose.velocity;
    let (c, sn) = (yaw.cos(), yaw.sin());
    let mut out = p.clone();
    out.pose.rotation = q.map(|x| x / len);
    out.pose.velocity = [(v[0] * c + v[2] * sn) * speed, v[1] * speed, (-v[0] * sn + v[2] * c) * speed];
    out
}

fn keep(pool: &mut Vec<Line>, child: Line) -> bool {
    let (near, d) = pool
        .iter()
        .enumerate()
        .map(|(i, l)| (i, difference(&child.trail, &l.trail)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap_or((0, f32::INFINITY));
    if d < DIFFERENT_M {
        if child.score.quality_cmp(&pool[near].score).is_gt() {
            pool[near] = child;
            return true;
        }
        return false;
    }
    if pool.len() < POOL {
        pool.push(child);
        return true;
    }
    let worst = (0..pool.len()).min_by(|&a, &b| pool[a].score.quality_cmp(&pool[b].score)).unwrap_or(0);
    if child.score.quality_cmp(&pool[worst].score).is_gt() {
        pool[worst] = child;
        return true;
    }
    false
}

pub fn difference(a: &[[f32; 3]], b: &[[f32; 3]]) -> f32 {
    let one_way = |a: &[[f32; 3]], b: &[[f32; 3]]| -> f32 {
        if a.is_empty() || b.is_empty() {
            return f32::INFINITY;
        }
        (0..SHAPE_POINTS)
            .map(|i| {
                let p = a[i * (a.len() - 1) / (SHAPE_POINTS - 1)];
                b.iter()
                    .map(|q| (0..3).map(|k| (p[k] - q[k]).powi(2)).sum::<f32>())
                    .fold(f32::INFINITY, f32::min)
                    .sqrt()
            })
            .fold(0.0, f32::max)
    };
    one_way(a, b).max(one_way(b, a))
}

fn make(
    origin: &dyn Origin,
    parent: &Line,
    change: Change,
    range: Range<usize>,
    until: usize,
    constraints: &Constraints,
    rng: &mut StdRng,
) -> Result<(Timeline, Option<Placement>)> {
    let mut car = parent.placement.clone();
    let mut line = parent.inputs.clone();
    if line.controls.len() < until {
        let last = line.controls.last().copied().unwrap_or_default();
        line.controls.resize(until, last);
    }
    let k = rng.random_range(range.clone());
    match change {
        Change::Ticks => {
            let end = (k + rng.random_range(1..=10)).min(range.end);
            let c = line.controls[k];
            let held = match rng.random_range(0..4) {
                0 => Controls { gas: 1.0 - c.gas.round(), ..c },
                1 => Controls { brake: 1.0 - c.brake.round(), ..c },
                _ => Controls { steer: [-1.0, -0.5, 0.0, 0.5, 1.0][rng.random_range(0..5)], ..c },
            };
            line.controls[k..end].fill(held);
        }
        Change::Timing => Mutation { max_changes: 2, max_shift_ticks: 10, ..Default::default() }.shift(
            &mut line.controls,
            range.clone(),
            rng,
        ),
        Change::Edit => Mutation {
            max_changes: 3,
            max_span_ticks: 40,
            max_shift_ticks: 10,
            max_steer_delta: 0.6,
            ..Default::default()
        }
        .apply(&mut line.controls, range.clone(), rng),
        Change::Turn => {
            let end = (k + rng.random_range(10..=100)).min(range.end);
            let steer = rng.random_range(-1.0..=1.0);
            let brake = if rng.random_bool(0.1) { rng.random_range(3..=10) } else { 0 };
            for (j, c) in line.controls[k..end].iter_mut().enumerate() {
                *c = Controls {
                    gas: if j < brake { 0.0 } else { 1.0 },
                    brake: if j < brake { 1.0 } else { 0.0 },
                    steer,
                };
            }
            drive_on(
                origin,
                car.as_ref(),
                &mut line,
                &parent.trail,
                range.start,
                end,
                range.end.min(until),
                lookahead(rng),
            )?;
        }
        Change::Cut => drive_on(
            origin,
            car.as_ref(),
            &mut line,
            &parent.trail,
            range.start,
            k,
            range.end.min(until),
            lookahead(rng),
        )?,
        Change::Car => car = car.map(|p| turn_car(&p, rng)),
    }
    constraints.apply(&parent.inputs.controls, &mut line.controls, range);
    Ok((line, car))
}

fn lookahead(rng: &mut StdRng) -> f32 {
    [5.0, 9.0, 15.0, 25.0, 40.0][rng.random_range(0..5)]
}

#[allow(clippy::too_many_arguments)]
fn drive_on(
    origin: &dyn Origin,
    car: Option<&Placement>,
    line: &mut Timeline,
    trail: &[[f32; 3]],
    trail_from: usize,
    from: usize,
    to: usize,
    ahead: f32,
) -> Result<()> {
    if trail.len() < 2 || from >= to {
        return Ok(());
    }
    let mut n = native(origin, car)?;
    n.seed = line.seed;
    let mut k = line.index(n.race_ms())?;
    while k < from {
        n.step(line.controls[k]);
        k += 1;
    }
    let dist = |a: [f32; 3], b: [f32; 3]| ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
    let mut near = (from.saturating_sub(trail_from)).min(trail.len() - 1);
    while k < to {
        let body = Body::from(n.body());
        let p = body.position();
        let mut best = near;
        for j in near..(near + 300).min(trail.len()) {
            if dist(p, trail[j]) < dist(p, trail[best]) {
                best = j;
            }
        }
        near = best;
        let (mut aim, mut run) = (near, 0.0);
        while aim + 1 < trail.len() && run < ahead {
            run += dist(trail[aim], trail[aim + 1]);
            aim += 1;
        }
        line.controls[k] = steer_toward(&body, trail[aim]);
        n.step(line.controls[k]);
        k += 1;
        if n.race().is_some_and(|r| r.finish.is_some()) {
            break;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_differ_by_their_farthest_point_not_by_speed() {
        let straight: Vec<[f32; 3]> = (0..100).map(|i| [0.0, 0.0, i as f32]).collect();
        let faster: Vec<[f32; 3]> = (0..50).map(|i| [0.0, 0.0, 2.0 * i as f32]).collect();
        assert!(difference(&straight, &faster) <= 1.0);
        let bulge: Vec<[f32; 3]> =
            (0..100).map(|i| [if (40..60).contains(&i) { 6.0 } else { 0.0 }, 0.0, i as f32]).collect();
        assert!(difference(&straight, &bulge) >= DIFFERENT_M);
    }

    #[test]
    fn every_change_keeps_a_share() {
        let mut rng = StdRng::seed_from_u64(1);
        let weight = [1000.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let mut n = [0usize; 6];
        let allowed = [true, true, true, true, true, false];
        for _ in 0..10_000 {
            let c = pick(&weight, &allowed, &mut rng);
            n[CHANGES.iter().position(|x| *x == c).unwrap()] += 1;
        }
        assert!(n[..5].iter().all(|&x| x as f64 >= 10_000.0 * MIN_SHARE * 0.8), "{n:?}");
        assert_eq!(n[5], 0);
    }

    #[test]
    fn a_turned_car_keeps_a_unit_rotation_and_its_speed_within_a_fifth() {
        let p = Placement {
            pose: crate::pose::Pose {
                position: [0.0; 3],
                rotation: [1.0, 0.0, 0.0, 0.0],
                velocity: [0.0, 0.0, 30.0],
            },
            angular_velocity: [0.0; 3],
            reset_contacts: false,
        };
        let mut rng = StdRng::seed_from_u64(3);
        let mut turned = 0;
        for _ in 0..200 {
            let q = turn_car(&p, &mut rng);
            let len = q.pose.rotation.iter().map(|v| v * v).sum::<f32>().sqrt();
            assert!((len - 1.0).abs() < 1e-5);
            let speed = q.pose.velocity.iter().map(|v| v * v).sum::<f32>().sqrt();
            assert!((24.0..=36.0).contains(&speed), "{speed}");
            turned += usize::from(q.pose.rotation != p.pose.rotation);
        }
        assert!(turned > 100);
    }
}
