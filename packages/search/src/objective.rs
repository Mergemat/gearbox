use crate::{
    gate::{Gate, crossing_waypoint},
    input::Timeline,
    tricks::{Detector, Events, Observation, TrickKind},
};
use anyhow::{Result, ensure};
use crate::Body;
use gearbox_engine::Session;
use std::hash::{Hash, Hasher};
use std::{cmp::Ordering, fmt};
use gearbox_engine::race::RaceState;

#[derive(Clone, Debug)]
pub enum Objective {
    Goals(crate::goals::Goals),
    Waypoint(usize),
    Finish,
    Position {
        at_ms: i32,
        point: [f32; 3],
    },
    Speed {
        at_ms: i32,
    },
    Gate {
        ordinal: usize,
        gate: Gate,
        time_penalty: f64,
    },
    GateSpeed {
        ordinal: usize,
        gate: Gate,
        time_penalty: f64,
    },
    Path {
        at_ms: i32,
        path: ReferencePath,
    },
    Trick {
        at_ms: i32,
        kind: TrickKind,
    },
    Aim {
        at_ms: i32,
        aim: crate::aim::Aim,
    },
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ReferencePath {
    pub points: Vec<[f32; 3]>,
    pub deviation_penalty: f64,
    #[serde(default)]
    pub stay_within: Option<f32>,
}
impl ReferencePath {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.points.len() >= 2 && self.points.iter().flatten().all(|v| v.is_finite()),
            "path requires at least two finite points"
        );
        ensure!(
            self.deviation_penalty.is_finite() && self.deviation_penalty >= 0.0,
            "invalid path deviation penalty"
        );
        ensure!(
            self.points.windows(2).any(|p| distance(p[0], p[1]) > 0.0),
            "path has zero length"
        );
        Ok(())
    }
    pub fn project(&self, point: [f32; 3], max_progress: f64) -> (f64, f64) {
        let (mut along, mut best_d, mut progress) = (0.0, f64::INFINITY, 0.0);
        for pair in self.points.windows(2) {
            let a = pair[0].map(f64::from);
            let d: [f64; 3] = std::array::from_fn(|k| f64::from(pair[1][k]) - a[k]);
            let len = d.iter().map(|v| v * v).sum::<f64>().sqrt();
            if len == 0.0 {
                continue;
            }
            if along > max_progress {
                break;
            }
            let t = (0..3)
                .map(|k| (f64::from(point[k]) - a[k]) * d[k])
                .sum::<f64>()
                / (len * len);
            let t = t.clamp(0.0, ((max_progress - along) / len).clamp(0.0, 1.0));
            let dist = (0..3)
                .map(|k| (f64::from(point[k]) - a[k] - t * d[k]).powi(2))
                .sum::<f64>();
            if dist < best_d {
                best_d = dist;
                progress = along + t * len;
            }
            along += len;
        }
        (progress, best_d.sqrt())
    }
}

pub const CUT_M: f64 = 0.05;

#[derive(Clone, Debug, PartialEq)]
pub enum Score {
    Goals {
        times: Vec<i32>,
        total: usize,
        error: f64,
        strayed: bool,
        cuts: u32,
    },
    Unreached,
    Race {
        time_ms: u32,
        speed_squared: f64,
    },
    Position {
        distance_squared: f64,
        speed_squared: f64,
    },
    Speed {
        speed_squared: f64,
    },
    Trick {
        ticks: u32,
        speed_squared: f64,
    },
    Aim {
        held: bool,
        value: f64,
        tie: f64,
    },
    Guided {
        time_ms: Option<u32>,
        cost: f64,
        speed_squared: f64,
        bonus: f64,
    },
}
impl Score {
    pub fn reached(&self) -> bool {
        if let Self::Goals { times, total, .. } = self {
            return times.len() == *total;
        }
        matches!(
            self,
            Self::Race { .. }
                | Self::Guided {
                    time_ms: Some(_),
                    ..
                }
        )
    }
    pub fn quality_cmp(&self, other: &Self) -> Ordering {
        match (self, other) {
            (
                Self::Goals {
                    times: a,
                    error: ea,
                    total,
                    strayed: sa,
                    cuts: ca,
                },
                Self::Goals {
                    times: b,
                    error: eb,
                    strayed: sb,
                    cuts: cb,
                    ..
                },
            ) => sb
                .cmp(sa)
                .then_with(|| a.len().cmp(&b.len()))
                .then_with(|| if a.len() < *total { (eb + CUT_M * f64::from(*cb)).total_cmp(&(ea + CUT_M * f64::from(*ca))) } else { Ordering::Equal })
                .then_with(|| b.last().cmp(&a.last()))
                .then_with(|| b.cmp(a))
                .then_with(|| cb.cmp(ca))
                .then_with(|| eb.total_cmp(ea)),
            (Self::Unreached, Self::Unreached) => Ordering::Equal,
            (Self::Unreached, _) => Ordering::Less,
            (_, Self::Unreached) => Ordering::Greater,
            (
                Self::Race {
                    time_ms: a,
                    speed_squared: sa,
                },
                Self::Race {
                    time_ms: b,
                    speed_squared: sb,
                },
            ) => b.cmp(a).then_with(|| sa.total_cmp(sb)),
            (
                Self::Position {
                    distance_squared: a,
                    speed_squared: sa,
                },
                Self::Position {
                    distance_squared: b,
                    speed_squared: sb,
                },
            ) => b.total_cmp(a).then_with(|| sa.total_cmp(sb)),
            (Self::Speed { speed_squared: a }, Self::Speed { speed_squared: b }) => a.total_cmp(b),
            (Self::Trick { ticks: a, speed_squared: sa }, Self::Trick { ticks: b, speed_squared: sb }) => a.cmp(b).then_with(|| sa.total_cmp(sb)),
            (Self::Aim { held: ha, value: va, tie: ta }, Self::Aim { held: hb, value: vb, tie: tb }) => ha.cmp(hb).then(va.total_cmp(vb)).then(ta.total_cmp(tb)),
            (
                Self::Guided {
                    time_ms: a,
                    cost: ca,
                    speed_squared: sa,
                    bonus: ba,
                },
                Self::Guided {
                    time_ms: b,
                    cost: cb,
                    speed_squared: sb,
                    bonus: bb,
                },
            ) => a
                .is_some()
                .cmp(&b.is_some())
                .then_with(|| cb.total_cmp(ca))
                .then_with(|| b.cmp(a))
                .then_with(|| sa.total_cmp(sb))
                .then_with(|| ba.total_cmp(bb)),
            _ => panic!("scores from different objectives"),
        }
    }
}
impl fmt::Display for Score {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Goals {
                times,
                total,
                error,
                strayed,
                ..
            } => write!(
                f,
                "{}/{} goals, times {:?}, error {:.6}{}",
                times.len(),
                total,
                times,
                error,
                if *strayed { ", strayed" } else { "" }
            ),
            Self::Unreached => write!(f, "unreached"),
            Self::Race {
                time_ms,
                speed_squared,
            } => write!(f, "{time_ms} ms, speed {:.9} m/s", speed_squared.sqrt()),
            Self::Position {
                distance_squared,
                speed_squared,
            } => write!(
                f,
                "distance {:.9} m, speed {:.9} m/s",
                distance_squared.sqrt(),
                speed_squared.sqrt()
            ),
            Self::Speed { speed_squared } => write!(f, "speed {:.9} m/s", speed_squared.sqrt()),
            Self::Trick { ticks, speed_squared } => write!(f, "{ticks} trick ticks, speed {:.9} m/s", speed_squared.sqrt()),
            Self::Aim { held, value, tie } => write!(f, "aim {value:.9} (tie {tie:.9}){}", if *held { "" } else { ", conditions not held" }),
            Self::Guided {
                time_ms,
                cost,
                speed_squared,
                bonus,
            } => write!(
                f,
                "{}, cost {cost:.6}, speed {:.6} m/s, trick bonus {bonus:.3}",
                time_ms.map_or("unreached".into(), |t| format!("{t} ms")),
                speed_squared.sqrt()
            ),
        }
    }
}
pub fn distance(a: [f32; 3], b: [f32; 3]) -> f64 {
    (0..3)
        .map(|k| (f64::from(a[k]) - f64::from(b[k])).powi(2))
        .sum::<f64>()
        .sqrt()
}
fn speed_squared(body: Body) -> f64 {
    body.local_velocity()
        .iter()
        .map(|v| f64::from(*v).powi(2))
        .sum()
}
impl Objective {
    pub fn validate(&self, start_ms: i32, until_ms: i32) -> Result<()> {
        match self {
            Self::Goals(goals) => goals.validate()?,
            Self::Waypoint(k) => ensure!(*k > 0, "waypoint indices start at 1"),
            Self::Gate {
                ordinal,
                gate,
                time_penalty,
            }
            | Self::GateSpeed {
                ordinal,
                gate,
                time_penalty,
            } => {
                ensure!(*ordinal > 0, "waypoint indices start at 1");
                gate.validate()?;
                ensure!(
                    time_penalty.is_finite() && *time_penalty >= 0.0,
                    "invalid time penalty"
                );
            }
            Self::Position { at_ms, point } => {
                ensure!(
                    point.iter().all(|v| v.is_finite()),
                    "target position must be finite"
                );
                ensure!(
                    *at_ms == until_ms && *at_ms > start_ms,
                    "invalid fixed target time"
                );
            }
            Self::Speed { at_ms } | Self::Trick { at_ms, .. } | Self::Aim { at_ms, .. } => ensure!(
                *at_ms == until_ms && *at_ms > start_ms,
                "invalid fixed target time"
            ),
            Self::Path { at_ms, path } => {
                path.validate()?;
                ensure!(
                    *at_ms == until_ms && *at_ms > start_ms,
                    "invalid path target time"
                );
            }
            Self::Finish => {}
        };
        Ok(())
    }
    pub fn race_time(&self, race: &RaceState) -> Option<u32> {
        match self {
            Self::Waypoint(k)
            | Self::Gate { ordinal: k, .. }
            | Self::GateSpeed { ordinal: k, .. } => k
                .checked_sub(1)
                .and_then(|i| race.checkpoints.get(i).copied()),
            Self::Finish => race.finish,
            _ => None,
        }
    }
    pub fn is_race(&self) -> bool {
        matches!(
            self,
            Self::Waypoint(_) | Self::Finish | Self::Gate { .. } | Self::GateSpeed { .. }
        )
    }
    fn target_time(&self, native: &mut Session) -> Option<u32> {
        let time = native.race().and_then(|r| self.race_time(&r));
        if let Self::Gate { gate, ordinal, .. } | Self::GateSpeed { gate, ordinal, .. } = self {
            time.filter(|_| crossing_waypoint(native, *ordinal) == Some(gate.waypoint))
        } else {
            time
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Sample {
    pub ms: i32,
    pub observation: Observation,
}
#[derive(Clone, Debug, PartialEq)]
pub struct Evaluation {
    pub score: Score,
    pub ticks: u64,
    pub body: Body,
    pub digest: u64,
    pub events: Events,
    pub trace: Vec<Sample>,
    pub path: Vec<[f32; 3]>,
    pub frontier_ms: Option<i32>,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct EvalOptions {
    pub from_ms: Option<i32>,
    pub full_rollout: bool,
    pub trick_bonus: f64,
    pub trace: bool,
    pub path_every: u32,
    pub checkpoint_every: u32,
    pub prune: bool,
}
pub fn evaluate(
    native: &mut Session,
    inputs: &Timeline,
    objective: &Objective,
    until_ms: i32,
) -> Result<Evaluation> {
    evaluate_with(native, inputs, objective, until_ms, EvalOptions::default())
}
pub fn evaluate_with(
    native: &mut Session,
    inputs: &Timeline,
    objective: &Objective,
    until_ms: i32,
    options: EvalOptions,
) -> Result<Evaluation> {
    evaluate_bounded(native, inputs, objective, until_ms, options, None)
}

fn hopeless(objective: &Objective, best: &Score, race_ms: i32, times: &[i32]) -> bool {
    let past = |t: i32| race_ms - 10 > t;
    match (objective, best) {
        (Objective::Goals(g), Score::Goals { times: inc, total, strayed: false, .. }) if !g.fastest && g.speed_worth_ms <= 0.0 && inc.len() == *total => {
            times.len() < *total && inc.last().is_some_and(|&t| past(t))
        }
        (Objective::Waypoint(_) | Objective::Finish, Score::Race { time_ms, .. }) => past(*time_ms as i32),
        (Objective::Gate { .. }, Score::Guided { time_ms: Some(t), .. }) => past(*t as i32),
        _ => false,
    }
}

pub fn evaluate_bounded(
    native: &mut Session,
    inputs: &Timeline,
    objective: &Objective,
    until_ms: i32,
    options: EvalOptions,
    best: Option<&Score>,
) -> Result<Evaluation> {
    evaluate_resumable(native, inputs, objective, until_ms, options, best, None, None)
}

#[derive(Clone)]
struct Acc {
    goals: crate::goals::Progress,
    detector: Detector,
    trace_len: usize,
    path_len: usize,
    closest: f64,
    path_progress: f64,
    path_cost: f64,
    previous: [f32; 3],
    path_started: bool,
    ticks: u64,
    digest: std::collections::hash_map::DefaultHasher,
    gate_frontier: Option<i32>,
    path_strayed: bool,
    aim_best: Option<crate::aim::Verdict>,
}

#[derive(Clone)]
pub struct Checkpoint {
    pub k: usize,
    state: gearbox_engine::Savestate,
    acc: Acc,
}

pub struct Resume<'a> {
    pub cp: &'a Checkpoint,
    pub parent: &'a Evaluation,
}

pub struct Recorder {
    pub from: usize,
    pub to: usize,
    pub every: usize,
    pub out: Vec<Checkpoint>,
}

#[allow(clippy::too_many_arguments)]
pub fn evaluate_resumable(
    native: &mut Session,
    inputs: &Timeline,
    objective: &Objective,
    until_ms: i32,
    options: EvalOptions,
    best: Option<&Score>,
    resume: Option<Resume<'_>>,
    mut record: Option<&mut Recorder>,
) -> Result<Evaluation> {
    let best = best.filter(|_| options.prune && !options.full_rollout);
    if let Objective::Goals(goals) = objective {
        goals.validate()?;
    }
    if let Some(r) = &resume {
        native.restore(&r.cp.state);
    }
    let start = inputs.index(native.race_ms())?;
    let end = inputs.index(until_ms)?;
    ensure!(start <= end, "evaluation ends before current state");
    let (mut goals_progress, mut detector, mut trace, mut path, mut closest, mut path_progress, mut path_cost, mut previous, mut path_started, mut ticks, mut digest, mut gate_frontier, mut path_strayed, mut aim_best);
    match &resume {
        Some(r) => {
            let a = r.cp.acc.clone();
            goals_progress = a.goals;
            detector = a.detector;
            trace = r.parent.trace[..a.trace_len].to_vec();
            path = r.parent.path[..a.path_len].to_vec();
            (closest, path_progress, path_cost, previous, path_started, ticks, digest, gate_frontier, path_strayed, aim_best) = (a.closest, a.path_progress, a.path_cost, a.previous, a.path_started, a.ticks, a.digest, a.gate_frontier, a.path_strayed, a.aim_best);
        }
        None => {
            goals_progress = crate::goals::Progress::new(native.race_ms());
            if let Objective::Goals(goals) = objective {
                let body = Body::from(native.body());
                goals_progress.observe(goals, native, body);
            }
            detector = Detector::default();
            trace = Vec::new();
            path = Vec::new();
            closest = f64::INFINITY;
            path_progress = 0.0;
            path_cost = f64::INFINITY;
            previous = Body::from(native.body()).position();
            path_started = false;
            ticks = 0;
            digest = std::collections::hash_map::DefaultHasher::new();
            gate_frontier = None;
            path_strayed = false;
            aim_best = None;
        }
    }
    let mut observed: Option<Observation> = None;
    let mut hit = if resume.is_none() && matches!(objective, Objective::Goals(g) if goals_progress.times.len() == g.ordered.len())
    {
        Some((native.race_ms().max(0) as u32, Body::from(native.body())))
    } else {
        None
    };
    for k in start..end {
        let c = &inputs.controls[k];
        if hit.is_some() && !options.full_rollout {
            break;
        }
        if let Some(rec) = record.as_deref_mut()
            && k > start
            && (rec.from..rec.to).contains(&k)
            && (k - rec.from) % rec.every.max(1) == 0
        {
            rec.out.push(Checkpoint {
                k,
                state: native.save(),
                acc: Acc {
                    goals: goals_progress.clone(),
                    detector: detector.clone(),
                    trace_len: trace.len(),
                    path_len: path.len(),
                    closest,
                    path_progress,
                    path_cost,
                    previous,
                    path_started,
                    ticks,
                    digest: digest.clone(),
                    gate_frontier,
                    path_strayed,
                    aim_best,
                },
            });
        }
        if let Objective::Goals(goals) = objective
            && goals_progress.times.len() < goals.ordered.len()
        {
            goals_progress.cuts += u32::from(c.gas < 0.5) + u32::from(c.brake >= 0.5);
        }
        native.step(*c);
        ticks += 1;
        if let Objective::Goals(goals) = objective {
            let body = Body::from(native.body());
            if !body.0.iter().all(|v| v.is_finite()) {
                return Ok(Evaluation {
                    score: Score::Unreached,
                    ticks,
                    body,
                    digest: digest.finish(),
                    events: detector.events,
                    trace,
                    path,
                    frontier_ms: None,
                });
            }
            goals_progress.observe(goals, native, body);
            if goals_progress.times.len() == goals.ordered.len() && hit.is_none() {
                hit = Some((native.race_ms().max(0) as u32, body));
            }
        }
        if options.from_ms.is_some_and(|ms| native.race_ms() <= ms) {
            previous = Body::from(native.body()).position();
            if hit.is_some() && !options.full_rollout {
                break;
            }
            continue;
        }
        let body = Body::from(native.body());
        body.0.map(f32::to_bits).hash(&mut digest);
        let position = body.position();
        if !position.iter().all(|v| v.is_finite()) || !speed_squared(body).is_finite() {
            return Ok(Evaluation {
                score: Score::Unreached,
                ticks,
                body,
                digest: digest.finish(),
                events: detector.events,
                trace,
                path,
                frontier_ms: None,
            });
        }
        if options.path_every > 0 && ticks % u64::from(options.path_every) == 0 {
            path.push(position);
        }
        if options.trace || options.trick_bonus > 0.0 || matches!(objective, Objective::Trick { .. } | Objective::Aim { .. }) {
            let observation = Observation::read(native, body);
            observed = Some(observation);
            if hit.is_none() {
                detector.observe(observation);
            }
            if options.trace {
                trace.push(Sample {
                    ms: native.race_ms(),
                    observation,
                });
            }
        }
        if let Objective::Aim { aim, .. } = objective
            && aim.judged == crate::aim::Judged::Best
            && let Some(observation) = observed
        {
            let v = aim.judge(&crate::aim::State { body, observation, events: &detector.events });
            if aim_best.is_none_or(|b| v.cmp(&b).is_gt()) {
                aim_best = Some(v);
            }
        }
        match objective {
            Objective::Gate {
                gate, time_penalty, ..
            }
            | Objective::GateSpeed {
                gate, time_penalty, ..
            } => {
                let c = gate.distance(position) + *time_penalty * f64::from(native.race_ms().max(0)) / 1000.0;
                if c < closest {
                    gate_frontier = Some(native.race_ms());
                }
                closest = closest.min(c);
            }
            Objective::Path { path, .. } => {
                if !path_started {
                    path_progress = path.project(previous, f64::INFINITY).0;
                    path_started = true;
                }
                let (p, d) =
                    path.project(position, path_progress + distance(previous, position) + 5.0);
                path_progress = p;
                path_cost = -p + path.deviation_penalty * d;
                if path.stay_within.is_some_and(|w| d > f64::from(w)) {
                    path_strayed = true;
                }
            }
            _ => {}
        }
        previous = position;
        if matches!(objective, Objective::Goals(_)) && hit.is_some() && !options.full_rollout {
            break;
        }
        if objective.is_race()
            && hit.is_none()
            && let Some(time) = objective.target_time(native)
        {
            hit = Some((time, body));
            if !options.full_rollout {
                break;
            }
        }
        if hit.is_none()
            && let Some(b) = best
            && hopeless(objective, b, native.race_ms(), &goals_progress.times)
        {
            break;
        }
    }
    let body = hit.map_or_else(|| Body::from(native.body()), |(_, b)| b);
    let speed_squared = speed_squared(body);
    let bonus = options.trick_bonus * f64::from(detector.events.total());
    let score = match objective {
        Objective::Goals(goals) if goals.fastest || goals.speed_worth_ms > 0.0 => {
            let left = goals.ordered.len() - goals_progress.times.len();
            let error = if goals_progress.error.is_finite() { goals_progress.error } else { 1e9 };
            let time_ms = hit.and(goals_progress.times.last().map(|t| (*t).max(0) as u32));
            let reached = if goals.fastest { -speed_squared.sqrt() } else { 1e7 + time_ms.map_or(0.0, f64::from) - goals.speed_worth_ms * speed_squared.sqrt() * 3.6 };
            Score::Guided {
                time_ms,
                cost: if hit.is_some() { reached } else { left as f64 * 1e6 + error },
                speed_squared,
                bonus: 0.0,
            }
        }
        Objective::Goals(goals) => goals_progress.score(goals.ordered.len()),
        Objective::Waypoint(_) | Objective::Finish => {
            hit.map_or(Score::Unreached, |(time_ms, _)| Score::Race {
                time_ms,
                speed_squared,
            })
        }
        Objective::Gate { .. } | Objective::GateSpeed { .. } => {
            let time_ms = hit.map(|(t, _)| t);
            let cost = time_ms.map_or(closest - bonus, |t| {
                if matches!(objective, Objective::GateSpeed { .. }) {
                    -speed_squared.sqrt()
                } else {
                    f64::from(t)
                }
            });
            Score::Guided {
                time_ms,
                cost,
                speed_squared,
                bonus,
            }
        }
        Objective::Position { point, .. } => Score::Position {
            distance_squared: (0..3)
                .map(|k| (f64::from(body.position()[k]) - f64::from(point[k])).powi(2))
                .sum(),
            speed_squared,
        },
        Objective::Speed { .. } => Score::Speed { speed_squared },
        Objective::Trick { kind, .. } => Score::Trick { ticks: detector.events.ticks(*kind), speed_squared },
        Objective::Aim { aim, .. } => {
            let v = match aim.judged {
                crate::aim::Judged::Best => aim_best,
                crate::aim::Judged::End => {
                    let observation = observed.unwrap_or_else(|| Observation::read(native, body));
                    Some(aim.judge(&crate::aim::State { body, observation, events: &detector.events }))
                }
            };
            v.map_or(Score::Aim { held: false, value: f64::NEG_INFINITY, tie: f64::NEG_INFINITY }, |v| Score::Aim { held: v.held, value: v.value, tie: v.tie })
        }
        Objective::Path { .. } => Score::Guided {
            time_ms: None,
            cost: path_cost - bonus + if path_strayed { 1e6 } else { 0.0 },
            speed_squared,
            bonus,
        },
    };
    Ok(Evaluation {
        score,
        ticks,
        body,
        digest: digest.finish(),
        events: detector.events,
        trace,
        path,
        frontier_ms: match objective {
            Objective::Goals(g) if goals_progress.times.len() < g.ordered.len() => goals_progress.frontier_ms,
            Objective::Gate { .. } | Objective::GateSpeed { .. } if hit.is_none() => gate_frontier,
            _ => None,
        },
    })
}
