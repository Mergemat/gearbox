use std::ops::Range;
use std::sync::Arc;

use crate::engine::frame::Frame;
use crate::engine::input::InputFile;
use gearbox_engine::Controls;
use glam::{Mat3, Quat, Vec3};
use serde::{Deserialize, Serialize};

pub type Id = u32;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GateGoal {
    pub waypoint: u32,
    pub kind: u32,
    pub boxes: Vec<gearbox_search::gate::Bounds>,
    pub ordinal: Option<usize>,
    #[serde(default)]
    pub also: Vec<u32>,
    #[serde(default)]
    pub parts: Vec<GatePart>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GatePart {
    pub name: String,
    pub gate: GateGoal,
}

impl GateGoal {
    pub fn has(&self, w: u32) -> bool {
        self.waypoint == w || self.also.contains(&w)
    }

    pub fn of_parts(parts: Vec<GatePart>) -> Self {
        let first = &parts[0].gate;
        let mut g = GateGoal { waypoint: first.waypoint, kind: first.kind, boxes: Vec::new(), ordinal: first.ordinal, also: Vec::new(), parts: Vec::new() };
        for p in &parts {
            for w in std::iter::once(p.gate.waypoint).chain(p.gate.also.iter().copied()) {
                if !g.has(w) {
                    g.also.push(w);
                }
            }
            g.boxes.extend(p.gate.boxes.iter().cloned());
        }
        g.parts = parts;
        g
    }

    pub fn split(&self, name: &str) -> Vec<GatePart> {
        if self.parts.is_empty() { vec![GatePart { name: name.into(), gate: self.clone() }] } else { self.parts.clone() }
    }
}

pub fn parts_name(parts: &[GatePart]) -> String {
    parts.iter().map(|p| p.name.as_str()).collect::<Vec<_>>().join(" or ")
}

pub fn touching(gates: &[&gearbox_search::gate::Gate]) -> Vec<Vec<u32>> {
    const TOUCH: f32 = 1.0;
    let near = |a: &gearbox_search::gate::Gate, b: &gearbox_search::gate::Gate| a.boxes.iter().any(|p| b.boxes.iter().any(|q| (0..3).all(|k| p.min[k] - TOUCH <= q.max[k] && q.min[k] - TOUCH <= p.max[k])));
    let mut group: Vec<usize> = (0..gates.len()).collect();
    let root = |g: &[usize], mut i: usize| {
        while g[i] != i {
            i = g[i];
        }
        i
    };
    for i in 0..gates.len() {
        for j in i + 1..gates.len() {
            if near(gates[i], gates[j]) {
                let (a, b) = (root(&group, i), root(&group, j));
                group[a.max(b)] = a.min(b);
            }
        }
    }
    let mut out: Vec<(usize, Vec<u32>)> = Vec::new();
    for (i, g) in gates.iter().enumerate() {
        let r = root(&group, i);
        match out.iter_mut().find(|o| o.0 == r) {
            Some(o) => o.1.push(g.waypoint),
            None => out.push((r, vec![g.waypoint])),
        }
    }
    out.into_iter().map(|o| o.1).collect()
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseSpec {
    pub pos: [f32; 3],
    pub ypr: [f32; 3],
    pub velocity: [f32; 3],
    pub use_velocity: bool,
    pub pos_tol: [f32; 3],
    pub angle_tol: f32,
    pub vel_tol: [f32; 3],
    pub pos_weight: [f32; 3],
    pub angle_weight: f32,
    pub vel_weight: [f32; 3],
}

impl PoseSpec {
    pub fn at(pos: Vec3, yaw_deg: f32) -> Self {
        Self {
            pos: pos.into(),
            ypr: [yaw_deg, 0.0, 0.0],
            velocity: [0.0; 3],
            use_velocity: false,
            pos_tol: [1.0, 1.0, 1.0],
            angle_tol: 10.0,
            vel_tol: [2.0; 3],
            pos_weight: [1.0; 3],
            angle_weight: 10.0,
            vel_weight: [0.1; 3],
        }
    }

    pub fn goal(&self) -> gearbox_search::pose::PoseGoal {
        gearbox_search::pose::PoseGoal {
            pose: gearbox_search::pose::Pose { position: self.pos, rotation: ypr_quat(self.ypr), velocity: self.velocity },
            position_tolerance: self.pos_tol,
            angle_tolerance: self.angle_tol.to_radians(),
            velocity_tolerance: self.use_velocity.then_some(self.vel_tol),
            position_weights: self.pos_weight,
            angle_weight: self.angle_weight,
            velocity_weights: self.vel_weight,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum GoalKind {
    Gate(GateGoal),
    Point { pos: [f32; 3], radius: f32 },
    Pose(PoseSpec),
    Box { pos: [f32; 3], size: [f32; 3], ypr: [f32; 3] },
}

pub fn trigger_box(pos: [f32; 3], size: [f32; 3], ypr: [f32; 3]) -> gearbox_search::goals::TriggerBox {
    let q = ypr_rot(ypr);
    gearbox_search::goals::TriggerBox { center: pos, half: size.map(|v| (v * 0.5).max(0.01)), rotation: [q.x, q.y, q.z, q.w] }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Goal {
    pub id: Id,
    pub name: String,
    pub kind: GoalKind,
    #[serde(default = "yes")]
    pub visible: bool,
    #[serde(default)]
    pub when: When,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct When {
    pub min_kmh: Option<f32>,
    pub pitch: Option<[f32; 2]>,
    pub slide: Option<[f32; 2]>,
}

impl When {
    pub fn conditions(&self) -> gearbox_search::goals::Conditions {
        gearbox_search::goals::Conditions { min_speed: self.min_kmh.map(|v| v / 3.6), pitch: self.pitch, slide: self.slide, min_crossings: None }
    }

    pub fn summary(&self) -> Vec<String> {
        let mut v = Vec::new();
        if let Some(s) = self.min_kmh {
            v.push(format!("\u{2265} {s:.0} km/h"));
        }
        if let Some([a, b]) = self.pitch {
            v.push(match (a, b) {
                _ if b <= -30.0 => "nose down".into(),
                _ if a >= 30.0 => "nose up".into(),
                _ if a >= -20.0 && b <= 20.0 => "level".into(),
                _ => format!("nose {a:.0}\u{2026}{b:.0}\u{00b0}"),
            });
        }
        if let Some([a, b]) = self.slide {
            v.push(if a >= 45.0 && b <= 135.0 { "sideways".into() } else { format!("slide {a:.0}\u{2026}{b:.0}\u{00b0}") });
        }
        v
    }

    pub fn details(&self) -> String {
        let mut v = Vec::new();
        if let Some(s) = self.min_kmh {
            v.push(format!("at least {s:.0} km/h"));
        }
        if let Some([a, b]) = self.pitch {
            v.push(format!("nose between {a:.0}\u{00b0} and {b:.0}\u{00b0}"));
        }
        if let Some([a, b]) = self.slide {
            v.push(format!("slide angle between {a:.0}\u{00b0} and {b:.0}\u{00b0}"));
        }
        v.join(", ")
    }
}

fn yes() -> bool {
    true
}

impl Goal {
    pub fn pos(&self) -> Vec3 {
        match &self.kind {
            GoalKind::Gate(g) => g.boxes.first().map_or(Vec3::ZERO, |b| (Vec3::from(b.min) + Vec3::from(b.max)) * 0.5),
            GoalKind::Point { pos, .. } => Vec3::from(*pos),
            GoalKind::Pose(p) => Vec3::from(p.pos),
            GoalKind::Box { pos, .. } => Vec3::from(*pos),
        }
    }

    pub fn kind_label(&self) -> &'static str {
        match &self.kind {
            GoalKind::Gate(g) if g.kind == 1 => "finish gate",
            GoalKind::Gate(_) => "checkpoint gate",
            GoalKind::Point { .. } => "point",
            GoalKind::Pose(_) => "car pose",
            GoalKind::Box { .. } => "box trigger",
        }
    }

    pub fn is_gate(&self) -> bool {
        matches!(self.kind, GoalKind::Gate(_))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StartState {
    pub tick: usize,
    pub pos: [f32; 3],
    pub ypr: [f32; 3],
    pub velocity: [f32; 3],
    pub angular: [f32; 3],
    pub reset_contacts: bool,
}

impl StartState {
    pub fn from_placement(tick: usize, p: &gearbox_search::pose::Placement) -> Self {
        let r = p.pose.rotation;
        Self {
            tick,
            pos: p.pose.position,
            ypr: rot_ypr(Quat::from_xyzw(r[1], r[2], r[3], r[0])),
            velocity: p.pose.velocity,
            angular: p.angular_velocity.map(f32::to_degrees),
            reset_contacts: p.reset_contacts,
        }
    }

    pub fn placement(&self) -> gearbox_search::pose::Placement {
        gearbox_search::pose::Placement {
            pose: gearbox_search::pose::Pose { position: self.pos, rotation: ypr_quat(self.ypr), velocity: self.velocity },
            angular_velocity: self.angular.map(f32::to_radians),
            reset_contacts: self.reset_contacts,
        }
    }

    pub fn forward(&self) -> Vec3 {
        ypr_rot(self.ypr) * Vec3::Z
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Origin {
    Blank,
    Inputs(String),
    Replay(String),
    Search(String),
    Rollout(String),
    Snap(String),
    Record,
    Edit,
}

impl Origin {
    pub fn label(&self) -> String {
        match self {
            Self::Blank => "blank".into(),
            Self::Inputs(s) => format!("inputs {s}"),
            Self::Replay(s) => format!("replay {s}"),
            Self::Search(s) => format!("search: {s}"),
            Self::Rollout(s) => format!("rollout {s}"),
            Self::Snap(s) => format!("snap {s}"),
            Self::Record => "recorded".into(),
            Self::Edit => "edited".into(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    pub id: Id,
    pub name: String,
    pub inputs: Arc<InputFile>,
    pub len: usize,
    pub start: Option<StartState>,
    pub origin: Origin,
    pub at: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Bookmark {
    pub name: String,
    pub pivot: [f32; 3],
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Doc {
    pub inputs: Arc<InputFile>,
    pub len: usize,
    pub run_name: String,
    pub run_origin: Origin,
    pub goals: Vec<Goal>,
    pub start: Option<StartState>,
    pub runs: Vec<Run>,
    pub history: Vec<Run>,
    pub locks: Vec<Range<usize>>,
    pub bookmarks: Vec<Bookmark>,
    pub guides: Vec<Guide>,
    pub setups: Vec<Setup>,
    pub next_id: Id,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    pub id: Id,
    pub name: String,
    pub start: StartState,
    pub controls: Vec<[f32; 3]>,
    pub end_pos: [f32; 3],
    pub end_ypr: [f32; 3],
    pub end_velocity: [f32; 3],
    #[serde(default)]
    pub tricks: Option<[u32; 3]>,
}

impl Setup {
    pub fn controls(&self) -> Vec<Controls> {
        self.controls.iter().map(|&[gas, brake, steer]| Controls { gas, brake, steer }).collect()
    }

    pub fn start_pose(&self) -> PoseSpec {
        let mut p = PoseSpec::at(Vec3::from(self.start.pos), 0.0);
        p.ypr = self.start.ypr;
        p.velocity = self.start.velocity;
        p.use_velocity = true;
        p
    }

    pub fn end_pose(&self) -> PoseSpec {
        let mut p = PoseSpec::at(Vec3::from(self.end_pos), 0.0);
        p.ypr = self.end_ypr;
        p.velocity = self.end_velocity;
        p.use_velocity = true;
        p.pos_tol = [2.0; 3];
        p.angle_tol = 15.0;
        p.vel_tol = [4.0; 3];
        p
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Guide {
    pub id: Id,
    pub name: String,
    pub points: Vec<[f32; 3]>,
}

impl Guide {
    pub fn length(&self) -> f32 {
        self.points.windows(2).map(|w| Vec3::from(w[0]).distance(Vec3::from(w[1]))).sum()
    }

    pub fn project(&self, p: Vec3) -> Option<(f32, f32)> {
        let mut along = 0.0;
        let mut best: Option<(f32, f32)> = None;
        for w in self.points.windows(2) {
            let (a, b) = (Vec3::from(w[0]), Vec3::from(w[1]));
            let len = a.distance(b);
            let t = if len > 0.0 { ((p - a).dot(b - a) / (len * len)).clamp(0.0, 1.0) } else { 0.0 };
            let d = p.distance(a + (b - a) * t);
            if best.is_none_or(|(_, bd)| d < bd) {
                best = Some((along + t * len, d));
            }
            along += len;
        }
        best
    }
}

impl Doc {
    pub fn setup(&self, id: Id) -> Option<&Setup> {
        self.setups.iter().find(|s| s.id == id)
    }

    pub fn guide(&self, id: Id) -> Option<&Guide> {
        self.guides.iter().find(|g| g.id == id)
    }

    pub fn guide_mut(&mut self, id: Id) -> Option<&mut Guide> {
        self.guides.iter_mut().find(|g| g.id == id)
    }

    pub fn new(inputs: InputFile, len: usize, origin: Origin) -> Self {
        Self {
            inputs: Arc::new(inputs),
            len,
            run_name: "Run".into(),
            run_origin: origin,
            goals: Vec::new(),
            start: None,
            runs: Vec::new(),
            history: Vec::new(),
            locks: Vec::new(),
            bookmarks: Vec::new(),
            guides: Vec::new(),
            setups: Vec::new(),
            next_id: 1,
        }
    }

    pub fn id(&mut self) -> Id {
        self.next_id += 1;
        self.next_id - 1
    }

    pub fn goal(&self, id: Id) -> Option<&Goal> {
        self.goals.iter().find(|g| g.id == id)
    }

    pub fn goal_mut(&mut self, id: Id) -> Option<&mut Goal> {
        self.goals.iter_mut().find(|g| g.id == id)
    }

    pub fn run(&self, id: Id) -> Option<&Run> {
        self.runs.iter().chain(&self.history).find(|r| r.id == id)
    }

    pub fn goal_name(&self, prefix: &str) -> String {
        (1..).map(|n| format!("{prefix}{n}")).find(|n| !self.goals.iter().any(|g| &g.name == n)).unwrap()
    }

    pub fn current(&mut self, origin: Origin) -> Run {
        Run { id: self.id(), name: self.run_name.clone(), inputs: self.inputs.clone(), len: self.len, start: self.start.clone(), origin, at: clock() }
    }

    pub fn remember(&mut self, why: &str) {
        let mut r = self.current(self.run_origin.clone());
        r.name = format!("{} · before {why}", self.run_name);
        self.history.push(r);
        if self.history.len() > 200 {
            self.history.remove(0);
        }
    }

    pub fn locked(&self, tick: usize) -> bool {
        self.locks.iter().any(|r| r.contains(&tick))
    }

    pub fn lock(&mut self, r: Range<usize>) {
        let mut all = self.locks.clone();
        all.push(r);
        self.locks = merge_ranges(all);
    }

    pub fn unlock(&mut self, r: Range<usize>) {
        let mut out = Vec::new();
        for l in &self.locks {
            if l.end <= r.start || l.start >= r.end {
                out.push(l.clone());
                continue;
            }
            if l.start < r.start {
                out.push(l.start..r.start);
            }
            if l.end > r.end {
                out.push(r.end..l.end);
            }
        }
        self.locks = out;
    }
}

pub fn merge_ranges(mut v: Vec<Range<usize>>) -> Vec<Range<usize>> {
    v.retain(|r| r.start < r.end);
    v.sort_by_key(|r| r.start);
    let mut out: Vec<Range<usize>> = Vec::new();
    for r in v {
        match out.last_mut() {
            Some(l) if r.start <= l.end => l.end = l.end.max(r.end),
            _ => out.push(r),
        }
    }
    out
}

pub fn clock() -> String {
    let s = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    format!("{:02}:{:02}:{:02} UTC", (s / 3600) % 24, (s / 60) % 60, s % 60)
}

#[derive(Default)]
pub struct Undo {
    pub undo: Vec<(String, Doc)>,
    pub redo: Vec<(String, Doc)>,
}

impl Undo {
    pub fn push(&mut self, label: impl Into<String>, before: Doc) {
        self.undo.push((label.into(), before));
        if self.undo.len() > 400 {
            self.undo.remove(0);
        }
        self.redo.clear();
    }
}

pub fn ypr_rot(ypr: [f32; 3]) -> Quat {
    let [y, p, r] = ypr.map(f32::to_radians);
    Quat::from_rotation_y(y) * Quat::from_rotation_x(-p) * Quat::from_rotation_z(-r)
}

pub fn ypr_quat(ypr: [f32; 3]) -> [f32; 4] {
    let q = ypr_rot(ypr);
    [q.w, q.x, q.y, q.z]
}

pub fn rot_ypr(q: Quat) -> [f32; 3] {
    let m = Mat3::from_quat(q);
    let (s, u, f) = (m.x_axis, m.y_axis, m.z_axis);
    [f.x.atan2(f.z).to_degrees(), f.y.clamp(-1.0, 1.0).asin().to_degrees(), (-s.y).atan2(u.y).to_degrees()]
}

pub fn frame_rot(f: &Frame) -> Quat {
    Quat::from_mat3(&Mat3::from_cols(Vec3::from(f.axis(0)), Vec3::from(f.axis(1)), Vec3::from(f.axis(2)))).normalize()
}

pub fn frame_pose(f: &Frame) -> gearbox_search::pose::Pose {
    gearbox_search::pose::Pose::from_body(gearbox_search::Body(f.body))
}

pub fn control_at(inputs: &InputFile, race_ms0: i32, k: usize) -> Controls {
    inputs.at(race_ms0 + 10 * k as i32)
}

pub fn splice(new: &InputFile, old: &InputFile, race_ms0: i32, new_tick: usize, old_tick: usize) -> Option<InputFile> {
    let mut out = new.clone();
    let cut = out.index(race_ms0 + 10 * new_tick as i32)?;
    out.controls.truncate(cut);
    out.controls.resize(cut, Controls::default());
    let mut t = race_ms0 + 10 * old_tick as i32;
    while t < old.end_ms() {
        out.controls.push(old.at(t));
        t += 10;
    }
    Some(out)
}

pub fn first_difference(a: &InputFile, b: &InputFile, race_ms0: i32) -> Option<usize> {
    if a.seed != b.seed {
        return Some(0);
    }
    let end = a.end_ms().max(b.end_ms());
    let start = a.t0.min(b.t0).min(race_ms0);
    let mut ms = start;
    while ms < end {
        if a.at(ms) != b.at(ms) {
            return Some(((ms - race_ms0).max(0) / 10) as usize);
        }
        ms += 10;
    }
    None
}

pub fn diff_ranges(a: &InputFile, b: &InputFile, race_ms0: i32, len: usize) -> Vec<Range<usize>> {
    let mut out: Vec<Range<usize>> = Vec::new();
    for k in 0..len {
        if control_at(a, race_ms0, k) != control_at(b, race_ms0, k) {
            match out.last_mut() {
                Some(r) if r.end == k => r.end = k + 1,
                _ => out.push(k..k + 1),
            }
        }
    }
    out
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reach {
    pub tick: Option<usize>,
    pub ms: Option<i32>,
    pub out_of_order: bool,
    pub closest: Option<(usize, f32)>,
    pub speed_kmh: Option<f32>,
}

fn reach_from(g: &Goal, frames: &[Frame], from: usize) -> Option<(usize, i32)> {
    let fr = frames.get(from..)?;
    let cond = g.when.conditions();
    let ok = |f: &Frame| cond.is_empty() || cond.shortfall(gearbox_search::Body(f.body), u32::from(f.waypoints)) == 0.0;
    let reached = match &g.kind {
        GoalKind::Point { pos, radius } => {
            let p = Vec3::from(*pos);
            fr.iter().position(|f| Vec3::from(f.position()).distance(p) <= *radius && ok(f)).map(|i| (from + i, fr[i].race_ms))
        }
        GoalKind::Pose(spec) => {
            let goal = spec.goal();
            fr.iter().position(|f| goal.contains(frame_pose(f)) && ok(f)).map(|i| (from + i, fr[i].race_ms))
        }
        GoalKind::Box { pos, size, ypr } => {
            let b = trigger_box(*pos, *size, *ypr);
            (from..frames.len())
                .find(|&k| (b.touches_car(gearbox_search::Body(frames[k].body)) || (k > from && b.segment_hits(frames[k - 1].position(), frames[k].position()))) && ok(&frames[k]))
                .map(|k| (k, frames[k].race_ms))
        }
        GoalKind::Gate(gate) => {
            (from.max(1)..frames.len()).find_map(|k| {
                let (a, b) = (&frames[k - 1], &frames[k]);
                let new = b.waypoints > a.waypoints;
                let ord_ok = gate.ordinal.is_none_or(|o| b.waypoints as usize >= o && (a.waypoints as usize) < o);
                match b.last_crossing {
                    Some((w, t)) if new && gate.has(w) && ord_ok => Some((k, t as i32)),
                    _ => None,
                }
            })
        }
    };
    reached.filter(|(k, _)| matches!(g.kind, GoalKind::Gate(_)).then(|| frames.get(*k).is_some_and(ok)).unwrap_or(true))
}

fn closest(g: &Goal, frames: &[Frame]) -> Option<(usize, f32)> {
    let d = |f: &Frame| -> f32 {
        match &g.kind {
            GoalKind::Point { pos, .. } => Vec3::from(f.position()).distance(Vec3::from(*pos)),
            GoalKind::Pose(spec) => spec.goal().error(frame_pose(f)) as f32,
            GoalKind::Box { pos, size, ypr } => trigger_box(*pos, *size, *ypr).car_distance(gearbox_search::Body(f.body)) as f32,
            GoalKind::Gate(gate) => gearbox_search::gate::Gate { waypoint: gate.waypoint, kind: gate.kind, boxes: gate.boxes.clone() }.distance(f.position()) as f32,
        }
    };
    frames.iter().enumerate().map(|(k, f)| (k, d(f))).min_by(|a, b| a.1.total_cmp(&b.1))
}

pub fn evaluate_each(goals: &[Goal], frames: &[Frame]) -> Vec<Reach> {
    goals
        .iter()
        .map(|g| {
            let mut r = Reach { closest: closest(g, frames), ..Default::default() };
            if let Some((k, ms)) = reach_from(g, frames, 0) {
                r.tick = Some(k);
                r.ms = Some(ms);
                r.speed_kmh = frames.get(k).map(Frame::speed_kmh);
            }
            r
        })
        .collect()
}

pub fn evaluate(goals: &[Goal], frames: &[Frame]) -> Vec<Reach> {
    let mut from = 0;
    let mut chain = true;
    goals
        .iter()
        .map(|g| {
            let mut r = Reach { closest: closest(g, frames), ..Default::default() };
            let ordered = if chain { reach_from(g, frames, from) } else { None };
            match ordered {
                Some((k, ms)) => {
                    r.tick = Some(k);
                    r.ms = Some(ms);
                    r.speed_kmh = frames.get(k).map(Frame::speed_kmh);
                    from = k;
                }
                None => {
                    chain = false;
                    r.out_of_order = reach_from(g, frames, 0).is_some();
                }
            }
            r
        })
        .collect()
}

const NEAR_MISS_M: f32 = 30.0;

const ON_LINE_M: f32 = 60.0;

pub fn run_order(goals: &[Goal], reach: &[Reach], guides: &[Guide]) -> Vec<usize> {
    let placed_at = |i: usize| {
        let r = reach.get(i)?;
        r.tick.or(r.closest.filter(|c| c.1 <= NEAR_MISS_M).map(|c| c.0))
    };
    let mut order: Vec<usize> = (0..goals.len()).filter(|&i| placed_at(i).is_some()).collect();
    order.sort_by_key(|&i| (placed_at(i), i));
    let finish = |i: usize| matches!(&goals[i].kind, GoalKind::Gate(g) if g.kind == 1);
    let mut far: Vec<usize> = (0..goals.len()).filter(|&i| placed_at(i).is_none()).collect();
    let on = |g: &Guide, i: usize| g.project(goals[i].pos()).filter(|p| p.1 <= ON_LINE_M).map(|p| p.0);
    if let Some(g) = guides.iter().max_by_key(|g| far.iter().filter(|&&i| !finish(i) && on(g, i).is_some()).count()) {
        let mut along: Vec<(f32, usize)> = far.iter().filter(|&&i| !finish(i)).filter_map(|&i| Some((on(g, i)?, i))).collect();
        along.sort_by(|a, b| a.0.total_cmp(&b.0));
        far.retain(|i| !along.iter().any(|a| a.1 == *i));
        order.extend(along.into_iter().map(|a| a.1));
    }
    let mut from = order.last().map(|&i| goals[i].pos());
    while !far.is_empty() {
        let key = |&&i: &&usize| (finish(i), from.map_or(0.0, |p| p.distance(goals[i].pos())));
        let (k, _) = far.iter().enumerate().min_by(|a, b| key(&a.1).partial_cmp(&key(&b.1)).unwrap_or(std::cmp::Ordering::Equal)).expect("not empty");
        let i = far.remove(k);
        from = Some(goals[i].pos());
        order.push(i);
    }
    order
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(id: Id, x: f32) -> Goal {
        Goal { id, name: format!("P{id}"), kind: GoalKind::Point { pos: [x, 0.0, 0.0], radius: 3.0 }, visible: true, when: When::default() }
    }

    #[test]
    fn far_misses_go_after_the_run_nearest_first_and_the_finish_last() {
        let reached = |k: usize| Reach { tick: Some(k), closest: Some((k, 0.0)), ..Default::default() };
        let missed = |k: usize, d: f32| Reach { closest: Some((k, d)), ..Default::default() };
        let mut finish = point(5, 150.0);
        finish.kind = GoalKind::Gate(GateGoal { waypoint: 7, kind: 1, boxes: vec![gearbox_search::gate::Bounds { min: [149.0, 0.0, 0.0], max: [151.0, 1.0, 1.0] }], ordinal: None, also: Vec::new(), parts: Vec::new() });
        let goals = vec![point(1, 0.0), point(2, 100.0), point(3, 50.0), point(4, 200.0), finish, point(6, 300.0)];
        let reach = vec![reached(10), reached(90), missed(50, 12.0), missed(5, 80.0), missed(20, 85.0), missed(95, 190.0)];
        let ids: Vec<Id> = run_order(&goals, &reach, &[]).iter().map(|&i| goals[i].id).collect();
        assert_eq!(ids, [1, 3, 2, 4, 6, 5]);
    }

    #[test]
    fn far_misses_follow_a_line_drawn_past_them() {
        let reached = |k: usize| Reach { tick: Some(k), closest: Some((k, 0.0)), ..Default::default() };
        let missed = |k: usize| Reach { closest: Some((k, 70.0)), ..Default::default() };
        let mut p4 = point(4, 120.0);
        p4.kind = GoalKind::Point { pos: [120.0, 0.0, 60.0], radius: 3.0 };
        let goals = vec![point(1, 0.0), point(2, 100.0), point(3, 200.0), p4, point(5, 600.0)];
        let reach = vec![reached(10), reached(90), missed(95), missed(95), missed(95)];
        let line = Guide { id: 9, name: "Line".into(), points: vec![[100.0, 0.0, 0.0], [200.0, 0.0, 0.0], [200.0, 0.0, 60.0], [120.0, 0.0, 60.0]] };
        let ids: Vec<Id> = run_order(&goals, &reach, &[line]).iter().map(|&i| goals[i].id).collect();
        assert_eq!(ids, [1, 2, 3, 4, 5]);
    }

    #[test]
    fn a_spliced_section_keeps_the_old_inputs_after_it_moved_to_the_new_time() {
        let ms0 = -1550;
        let old = InputFile { t0: ms0, seed: None, controls: (0..100).map(|k| Controls { gas: 1.0, brake: 0.0, steer: k as f32 }).collect() };
        let mut new = old.clone();
        for c in &mut new.controls[10..45] {
            c.steer = -1.0;
        }
        let s = splice(&new, &old, ms0, 40, 50).unwrap();
        let steer: Vec<f32> = s.controls.iter().map(|c| c.steer).collect();
        assert_eq!(&steer[..10], &(0..10).map(|k| k as f32).collect::<Vec<_>>()[..]);
        assert!(steer[10..40].iter().all(|&x| x == -1.0));
        assert_eq!(steer[40], 50.0);
        assert_eq!(steer[89], 99.0);
        assert_eq!(steer.len(), 90);
    }

    #[test]
    fn finish_blocks_in_a_row_are_one_finish_and_far_ones_another() {
        let at = |w: u32, x: f32| gearbox_search::gate::Gate { waypoint: w, kind: 1, boxes: vec![gearbox_search::gate::Bounds { min: [x, 0.0, 0.0], max: [x + 32.0, 8.0, 2.0] }] };
        let gs = [at(7, 0.0), at(3, 500.0), at(8, 32.0), at(9, 64.5)];
        let refs: Vec<&gearbox_search::gate::Gate> = gs.iter().collect();
        assert_eq!(touching(&refs), vec![vec![7, 8, 9], vec![3]]);
    }

    #[test]
    fn yaw_pitch_roll_round_trip_through_quaternions() {
        for ypr in [[0.0, 0.0, 0.0], [90.0, 0.0, 0.0], [-135.0, 20.0, 0.0], [30.0, -10.0, 15.0], [170.0, 45.0, -60.0]] {
            let back = rot_ypr(ypr_rot(ypr));
            for k in 0..3 {
                assert!((back[k] - ypr[k]).abs() < 1e-3, "{ypr:?} -> {back:?}");
            }
        }
    }

    #[test]
    fn yaw_faces_the_engine_forward_axis() {
        let f = ypr_rot([90.0, 0.0, 0.0]) * Vec3::Z;
        assert!((f - Vec3::X).length() < 1e-6);
        let f = ypr_rot([0.0, 30.0, 0.0]) * Vec3::Z;
        assert!((f.y - 0.5).abs() < 1e-6 && f.z > 0.0);
    }

    #[test]
    fn engine_quaternion_matches_its_matrix() {
        let ypr = [33.0, -12.0, 7.0];
        let [w, x, y, z] = ypr_quat(ypr);
        let m = gearbox_engine::Quat::new(w, x, y, z).to_mat3().to_array();
        let g = Mat3::from_quat(ypr_rot(ypr));
        for (r, c) in (0..3).flat_map(|r| (0..3).map(move |c| (r, c))) {
            assert!((m[3 * r + c] - g.col(c)[r]).abs() < 1e-6, "{r},{c}");
        }
    }

    #[test]
    fn locks_merge_and_split() {
        let mut d = Doc::new(InputFile::blank(0), 100, Origin::Blank);
        d.lock(10..20);
        d.lock(15..30);
        d.lock(40..50);
        assert_eq!(d.locks, vec![10..30, 40..50]);
        d.unlock(20..45);
        assert_eq!(d.locks, vec![10..20, 45..50]);
        assert!(d.locked(10) && !d.locked(20) && d.locked(49));
    }

    #[test]
    fn diff_ranges_cover_changed_ticks() {
        let a = InputFile::parse("T0 0\n1 0 0\n1 0 0\n1 0 0\n1 0 0\n").unwrap();
        let mut b = a.clone();
        b.controls[1].steer = 1.0;
        b.controls[2].steer = 1.0;
        assert_eq!(diff_ranges(&a, &b, 0, 6), vec![1..3]);
        assert_eq!(first_difference(&a, &b, 0), Some(1));
        assert_eq!(first_difference(&a, &b, -20), Some(3));
    }
}
