use crate::{
    gate::{Gate, crossing_waypoint},
    pose::{Pose, PoseGoal},
};
use anyhow::{Result, ensure};
use crate::Body;
use gearbox_engine::Session;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Goal {
    Point { point: [f32; 3], radius: f32 },
    Box { trigger: TriggerBox },
    Pose { target: PoseGoal },
    Gate {
        ordinal: usize,
        gate: Gate,
        #[serde(default)]
        also: Vec<u32>,
    },
    Waypoint { ordinal: usize },
    Finish,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Goals {
    pub ordered: Vec<Goal>,
    #[serde(default)]
    pub along: Option<crate::objective::ReferencePath>,
    #[serde(default)]
    pub stay_within: Option<f32>,
    #[serde(skip)]
    pub track: Option<std::sync::Arc<crate::track::TrackFields>>,
    #[serde(default)]
    pub fastest: bool,
    #[serde(default)]
    pub speed_worth_ms: f64,
    #[serde(default)]
    pub conditions: Vec<Conditions>,
}

pub const CAR_HALF: [f32; 3] = [1.0, 0.6, 2.15];

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Conditions {
    #[serde(default)]
    pub min_speed: Option<f32>,
    #[serde(default)]
    pub pitch: Option<[f32; 2]>,
    #[serde(default)]
    pub slide: Option<[f32; 2]>,
    #[serde(default)]
    pub min_crossings: Option<u32>,
}

impl Conditions {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    pub fn shortfall(&self, body: Body, crossings: u32) -> f64 {
        let v = [body.0[16], body.0[17], body.0[18]].map(f64::from);
        let speed = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        let r = body.0.map(f64::from);
        let fwd = [r[2], r[5], r[8]];
        let outside = |x: f64, [lo, hi]: [f32; 2]| (f64::from(lo) - x).max(x - f64::from(hi)).max(0.0) / 5.0;
        let mut miss = 0.0;
        if let Some(m) = self.min_speed {
            miss += (f64::from(m) - speed).max(0.0);
        }
        if let Some(range) = self.pitch {
            miss += outside(fwd[1].clamp(-1.0, 1.0).asin().to_degrees(), range);
        }
        if let Some(range) = self.slide {
            let angle = if speed < 1e-3 { 0.0 } else { ((fwd[0] * v[0] + fwd[1] * v[1] + fwd[2] * v[2]) / speed).clamp(-1.0, 1.0).acos().to_degrees() };
            miss += outside(angle, range);
        }
        if let Some(c) = self.min_crossings {
            miss += 100.0 * f64::from(c.saturating_sub(crossings));
        }
        miss
    }
}
impl Goals {
    pub fn validate(&self) -> Result<()> {
        ensure!(!self.ordered.is_empty(), "goal list is empty");
        for goal in &self.ordered {
            match goal {
                Goal::Point { point, radius } => ensure!(
                    point.iter().all(|v| v.is_finite()) && radius.is_finite() && *radius >= 0.0,
                    "invalid point goal"
                ),
                Goal::Pose { target } => target.validate()?,
                Goal::Box { trigger } => trigger.validate()?,
                Goal::Gate { ordinal, gate, .. } => {
                    ensure!(*ordinal > 0, "waypoint ordinals start at 1");
                    gate.validate()?;
                }
                Goal::Waypoint { ordinal } => ensure!(*ordinal > 0, "waypoint ordinals start at 1"),
                Goal::Finish => {}
            }
        }
        Ok(())
    }
}
const NEAR_GOAL_M: f64 = 20.0;

fn goal_bottom(g: &Goal) -> Option<f64> {
    match g {
        Goal::Point { point, radius } => Some(f64::from(point[1] - radius)),
        Goal::Pose { target } => Some(f64::from(target.pose.position[1] - target.position_tolerance[1])),
        Goal::Box { trigger } => Some(f64::from(trigger.center[1]) - f64::from(trigger.half.iter().fold(0.0f32, |a, h| a.max(*h)))),
        Goal::Gate { gate, .. } => gate.boxes.iter().map(|b| f64::from(b.min[1])).reduce(f64::min),
        Goal::Waypoint { .. } | Goal::Finish => None,
    }
}

pub fn goal_point(g: &Goal) -> Option<[f32; 3]> {
    match g {
        Goal::Point { point, .. } => Some(*point),
        Goal::Box { trigger } => Some(trigger.center),
        Goal::Pose { target } => Some(target.pose.position),
        Goal::Gate { gate, .. } => gate.boxes.first().map(|b| std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5)),
        Goal::Waypoint { .. } | Goal::Finish => None,
    }
}
#[derive(Clone, Debug)]
pub(crate) struct Progress {
    pub times: Vec<i32>,
    pub error: f64,
    pub now: f64,
    activated: i32,
    start_ms: i32,
    last: Option<[f32; 3]>,
    line: Option<f64>,
    line_cap: f64,
    anchors: Vec<Option<f64>>,
    strayed: bool,
    pub frontier_ms: Option<i32>,
    pub cuts: u32,
}
impl Progress {
    pub fn new(start_ms: i32) -> Self {
        Self {
            times: Vec::new(),
            error: f64::INFINITY,
            now: f64::INFINITY,
            activated: start_ms,
            start_ms,
            last: None,
            line: None,
            line_cap: f64::INFINITY,
            anchors: Vec::new(),
            strayed: false,
            frontier_ms: None,
            cuts: 0,
        }
    }
    pub fn observe(&mut self, goals: &Goals, n: &mut Session, body: Body) {
        let last = self.last.replace(body.position());
        let mut along: Option<(f64, f64)> = None;
        if let Some(path) = &goals.along {
            let p = body.position();
            if let (Some(_), Some(a)) = (self.line, last) {
                self.line_cap += crate::objective::distance(a, p);
            }
            let (progress, off) = path.project(p, self.line_cap);
            if self.line.is_none() {
                self.line_cap = progress + 5.0;
            }
            self.line = Some(progress);
            if goals.stay_within.is_some_and(|w| off > f64::from(w)) {
                self.strayed = true;
            }
            if self.anchors.len() != goals.ordered.len() {
                let end = path.points.windows(2).map(|w| crate::objective::distance(w[0], w[1])).sum::<f64>();
                self.anchors = goals.ordered.iter().map(|g| goal_point(g).map_or(Some(end), |q| Some(path.project(q, f64::INFINITY).0))).collect();
            }
            along = Some((progress, off * path.deviation_penalty));
        }
        while let Some(goal) = goals.ordered.get(self.times.len()) {
            let sampled = matches!(goal, Goal::Point { .. } | Goal::Pose { .. } | Goal::Box { .. });
            let (error, time) = match goal {
                Goal::Point { point, radius } => {
                    let d = crate::objective::distance(body.position(), *point);
                    (d, (d <= f64::from(*radius)).then_some(n.race_ms()))
                }
                Goal::Pose { target } => {
                    let p = Pose::from_body(body);
                    (target.error(p), target.contains(p).then_some(n.race_ms()))
                }
                Goal::Box { trigger } => {
                    let p = body.position();
                    let hit = trigger.touches_car(body) || last.is_some_and(|a| trigger.segment_hits(a, p));
                    (trigger.car_distance(body), hit.then_some(n.race_ms()))
                }
                Goal::Gate { ordinal, gate, also } => {
                    let t = n
                        .race()
                        .and_then(|r| r.checkpoints.get(ordinal - 1).copied());
                    (
                        gate.distance(body.position()),
                        t.filter(|_| crossing_waypoint(n, *ordinal).is_some_and(|w| w == gate.waypoint || also.contains(&w)))
                            .map(|t| t as i32),
                    )
                }
                Goal::Waypoint { ordinal } => (
                    f64::INFINITY,
                    n.race()
                        .and_then(|r| r.checkpoints.get(ordinal - 1).copied())
                        .map(|t| t as i32),
                ),
                Goal::Finish => (
                    f64::INFINITY,
                    n.race().and_then(|r| r.finish).map(|t| t as i32),
                ),
            };
            let error = match (along, self.anchors.get(self.times.len()).copied().flatten()) {
                (Some((progress, off)), Some(anchor)) => (anchor - progress).max(0.0) + off,
                _ => error,
            };
            let below = goal_bottom(goal).map_or(0.0, |y| (y - f64::from(body.position()[1])).max(0.0));
            let near = error + below * f64::from(crate::track::CLIMB_COST - 1.0);
            let error = match goals.track.as_ref().and_then(|t| t.goals.get(self.times.len()).and_then(Option::as_ref).map(|f| (f, t.reach))) {
                Some((f, reach)) => {
                    let along = f.distance_within(body.position(), reach).unwrap_or(1e6 + error);
                    if error < NEAR_GOAL_M { along.min(near) } else { along }
                }
                None if matches!(goal, Goal::Gate { .. } | Goal::Point { .. } | Goal::Box { .. }) => near,
                None => error,
            };
            let i = self.times.len();
            let fresh = sampled || time.is_some_and(|t| t > n.race_ms() - 10);
            let short = match (time.filter(|t| *t >= self.activated), goals.conditions.get(i)) {
                (Some(_), Some(c)) if !c.is_empty() => Some(if fresh { c.shortfall(body, n.race().map_or(0, |r| r.checkpoints.len() as u32)) } else { f64::INFINITY }),
                _ => None,
            };
            let error = match short {
                Some(s) if s > 0.0 => error.max(0.0) + s,
                _ => error,
            };
            if error < self.error {
                self.frontier_ms = Some(n.race_ms());
            }
            self.error = self.error.min(error);
            self.now = error;
            if short.is_some_and(|s| s > 0.0) {
                break;
            }
            let Some(t) = time.filter(|t| *t >= self.activated) else {
                break;
            };
            self.times.push(t);
            self.activated = if sampled {
                (t - 9).max(self.start_ms)
            } else {
                t
            };
            self.error = 0.0;
            if self.times.len() < goals.ordered.len() {
                self.error = f64::INFINITY;
            }
        }
    }
    pub fn score(&self, total: usize) -> crate::objective::Score {
        crate::objective::Score::Goals {
            times: self.times.clone(),
            total,
            error: self.error,
            strayed: self.strayed,
            cuts: self.cuts,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TriggerBox {
    pub center: [f32; 3],
    pub half: [f32; 3],
    pub rotation: [f32; 4],
}

impl TriggerBox {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.center.iter().chain(&self.rotation).all(|v| v.is_finite()) && self.half.iter().all(|h| h.is_finite() && *h > 0.0),
            "invalid box goal"
        );
        let n: f32 = self.rotation.iter().map(|v| v * v).sum();
        ensure!((n - 1.0).abs() < 1e-3, "box goal rotation must be a unit quaternion");
        Ok(())
    }

    fn local(&self, p: [f32; 3]) -> [f64; 3] {
        self.local_dir([0, 1, 2].map(|k| f64::from(p[k]) - f64::from(self.center[k])))
    }

    fn local_dir(&self, v: [f64; 3]) -> [f64; 3] {
        let [x, y, z, w] = self.rotation.map(f64::from);
        let u = [-x, -y, -z];
        let cross = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let t = cross(u, v).map(|c| 2.0 * c);
        let ut = cross(u, t);
        [0, 1, 2].map(|k| v[k] + w * t[k] + ut[k])
    }

    fn car_box(&self, body: Body) -> ([f64; 3], [[f64; 3]; 3]) {
        let r = body.rotation().map(f64::from);
        let c = self.local(body.position());
        let axes = [0, 1, 2].map(|k| self.local_dir([r[k], r[3 + k], r[6 + k]]).map(|v| v * f64::from(CAR_HALF[k])));
        (c, axes)
    }

    pub fn touches_car(&self, body: Body) -> bool {
        let (c, a) = self.car_box(body);
        let h = self.half.map(f64::from);
        let dot = |x: [f64; 3], y: [f64; 3]| x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        let cross = |x: [f64; 3], y: [f64; 3]| [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
        let unit = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let separates = |n: [f64; 3]| {
            if dot(n, n) < 1e-12 {
                return false;
            }
            let rb: f64 = (0..3).map(|k| h[k] * n[k].abs()).sum();
            let rc: f64 = a.iter().map(|ax| dot(*ax, n).abs()).sum();
            dot(c, n).abs() > rb + rc
        };
        let mut axes: Vec<[f64; 3]> = unit.to_vec();
        axes.extend(a);
        for u in unit {
            for v in a {
                axes.push(cross(u, v));
            }
        }
        !axes.into_iter().any(separates)
    }

    pub fn car_distance(&self, body: Body) -> f64 {
        if self.touches_car(body) {
            return 0.0;
        }
        let (c, a) = self.car_box(body);
        let h = self.half.map(f64::from);
        let d = |p: [f64; 3]| (0..3).map(|k| (p[k].abs() - h[k]).max(0.0).powi(2)).sum::<f64>().sqrt();
        let mut best = d(c);
        for s in 0..8 {
            let sign = |k: usize| if s >> k & 1 == 0 { -1.0 } else { 1.0 };
            let p = [0, 1, 2].map(|i| c[i] + sign(0) * a[0][i] + sign(1) * a[1][i] + sign(2) * a[2][i]);
            best = best.min(d(p));
        }
        best
    }

    pub fn contains(&self, p: [f32; 3]) -> bool {
        let l = self.local(p);
        (0..3).all(|k| l[k].abs() <= f64::from(self.half[k]))
    }

    pub fn distance(&self, p: [f32; 3]) -> f64 {
        let l = self.local(p);
        (0..3).map(|k| (l[k].abs() - f64::from(self.half[k])).max(0.0).powi(2)).sum::<f64>().sqrt()
    }

    pub fn segment_hits(&self, a: [f32; 3], b: [f32; 3]) -> bool {
        let (la, lb) = (self.local(a), self.local(b));
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        for k in 0..3 {
            let h = f64::from(self.half[k]);
            let d = lb[k] - la[k];
            if d.abs() < 1e-12 {
                if la[k].abs() > h {
                    return false;
                }
                continue;
            }
            let (mut e0, mut e1) = ((-h - la[k]) / d, (h - la[k]) / d);
            if e0 > e1 {
                std::mem::swap(&mut e0, &mut e1);
            }
            t0 = t0.max(e0);
            t1 = t1.min(e1);
            if t0 > t1 {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::{Conditions, TriggerBox};
    use crate::Body;

    fn car(fwd: [f32; 3], v: [f32; 3]) -> Body {
        let mut b = [0.0f32; 22];
        b[..9].copy_from_slice(&[1.0, 0.0, 0.0, 0.0, fwd[2], fwd[1], 0.0, -fwd[1], fwd[2]]);
        b[16..19].copy_from_slice(&v);
        Body(b)
    }

    #[test]
    fn a_box_counts_any_part_of_the_car_touching_it() {
        let slab = TriggerBox { center: [0.0, 0.0, 0.0], half: [15.0, 0.3, 15.0], rotation: [0.0, 0.0, 0.0, 1.0] };
        let mut b = car([0.0, -1.0, 0.0], [0.0; 3]);
        b.0[10] = 2.0;
        assert!(!slab.contains(b.position()));
        assert!(slab.touches_car(b));
        assert_eq!(slab.car_distance(b), 0.0);
        b.0[10] = 3.45;
        assert!(!slab.touches_car(b));
        assert!((slab.car_distance(b) - 1.0).abs() < 0.01);
    }

    #[test]
    fn conditions_measure_how_far_off_the_car_is() {
        let h = std::f32::consts::FRAC_1_SQRT_2;
        let b = car([0.0, -h, h], [0.0, 0.0, 80.0]);
        let fast = Conditions { min_speed: Some(70.0), ..Default::default() };
        assert_eq!(fast.shortfall(b, 0), 0.0);
        let faster = Conditions { min_speed: Some(90.0), ..Default::default() };
        assert!((faster.shortfall(b, 0) - 10.0).abs() < 1e-3);
        assert_eq!(Conditions { pitch: Some([-90.0, -30.0]), ..Default::default() }.shortfall(b, 0), 0.0);
        assert!((Conditions { pitch: Some([-90.0, -60.0]), ..Default::default() }.shortfall(b, 0) - 3.0).abs() < 1e-3);
        assert_eq!(Conditions { slide: Some([40.0, 50.0]), ..Default::default() }.shortfall(b, 0), 0.0);
        assert_eq!(Conditions { min_crossings: Some(3), ..Default::default() }.shortfall(b, 1), 200.0);
    }

    fn turned() -> TriggerBox {
        let h = std::f32::consts::FRAC_1_SQRT_2;
        TriggerBox { center: [10.0, 0.0, 0.0], half: [2.0, 1.0, 0.5], rotation: [0.0, h, 0.0, h] }
    }

    #[test]
    fn rotation_moves_the_long_side() {
        let b = turned();
        assert!(b.contains([10.0, 0.0, 1.9]));
        assert!(!b.contains([11.9, 0.0, 0.0]));
        assert!((b.distance([10.0, 0.0, 5.0]) - 3.0).abs() < 1e-5);
    }

    #[test]
    fn a_fast_car_cannot_skip_a_thin_box() {
        let b = turned();
        assert!(!b.contains([8.0, 0.0, 0.0]) && !b.contains([12.0, 0.0, 0.0]));
        assert!(b.segment_hits([8.0, 0.0, 0.0], [12.0, 0.0, 0.0]));
        assert!(!b.segment_hits([8.0, 3.0, 0.0], [12.0, 3.0, 0.0]));
    }
}
