use std::ops::Range;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use anyhow::{Result, bail, ensure};
use crate::engine::input::InputFile;
use gearbox_engine::Savestate;
use serde::{Deserialize, Serialize};
use gearbox_search::objective::{Objective, Score};
use gearbox_search::pose::Placement;

use crate::background::{reach, Wake, panic_message};
use crate::doc::{Goal, GoalKind, Id};
use crate::fmt::{fmt_ms, fmt_s};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObjKind {
    ReachGoals,
    Moment,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Aim {
    Goals,
    Moment,
}

impl ObjKind {
    pub fn aim(self) -> Aim {
        match self {
            Self::ReachGoals => Aim::Goals,
            Self::Moment => Aim::Moment,
        }
    }

    pub fn goals(self) -> bool {
        self.aim() == Aim::Goals
    }

    pub fn about(self) -> &'static str {
        match self {
            Self::ReachGoals => "Lines must reach the goals in order. Getting further wins (more goals, then closer to the next one, measured along the track); among those that reach them all, the last goal's Time, Both or Speed decides.",
            Self::Moment => "One moment counts: the measure there (or at its best tick of the range), as long as every condition holds; then the tie-break.",
        }
    }
    pub fn fixed_time(self) -> bool {
        matches!(self, Self::Moment)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum StartFrom {
    #[default]
    Run,
    Line(Id),
    Ghost,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Spec {
    pub kind: ObjKind,
    pub goals: Vec<Id>,
    #[serde(default)]
    pub by_speed: Vec<Id>,
    #[serde(default)]
    pub both: Vec<Id>,
    #[serde(default = "yes")]
    pub keep_cps: bool,
    #[serde(default = "gearbox_search::aim::Aim::fastest")]
    pub aim: gearbox_search::aim::Aim,
    pub at_tick: Option<usize>,
    #[serde(default)]
    pub start_from: StartFrom,
    #[serde(default = "yes")]
    pub vary_car: bool,
    pub window: Range<usize>,
    pub until: Option<usize>,
    pub channels: [bool; 3],
    pub digital_steer: bool,
    pub steer_limit: f32,
    pub honour_locks: bool,
    pub by_time: bool,
    pub attempts: usize,
    pub seconds: f32,
    #[serde(default)]
    pub go_on: bool,
    pub threads: usize,
    pub seed: u64,
    #[serde(default = "yes")]
    pub auto: bool,
}

fn yes() -> bool {
    true
}

pub fn trick_name(k: gearbox_search::tricks::TrickKind) -> &'static str {
    use gearbox_search::tricks::TrickKind::*;
    match k {
        Nosebug => "nosebug",
        Bugslide => "bugslide",
        Uberbug => "uberbug",
        Any => "trick (any)",
    }
}

pub fn quantity_label(q: &gearbox_search::aim::Quantity) -> (&'static str, &'static str, &'static str) {
    use gearbox_search::aim::Quantity::*;
    match q {
        Speed => ("Speed", "km/h", "icons/gauge.svg"),
        Height => ("Height", "m", "icons/height.svg"),
        Distance { .. } => ("Distance to a point", "m", "icons/ruler.svg"),
        Angle { .. } => ("Angle", "\u{00b0} off", "icons/protractor.svg"),
        Yaw => ("Yaw", "\u{00b0}", "icons/protractor.svg"),
        Pitch => ("Pitch", "\u{00b0}", "icons/protractor.svg"),
        Roll => ("Roll", "\u{00b0}", "icons/protractor.svg"),
        Trick(_) => ("Trick ticks", "ticks", "icons/sparkle.svg"),
    }
}

pub fn measure_text(m: &gearbox_search::aim::Measure, oriented: f64) -> String {
    let v = if m.higher { oriented } else { -oriented };
    let (_, unit, _) = quantity_label(&m.quantity);
    match m.quantity {
        gearbox_search::aim::Quantity::Trick(_) => format!("{} s held", fmt_s((v * 10.0) as i32)),
        _ => format!("{v:.1} {unit}"),
    }
}

pub fn aim_text(aim: &gearbox_search::aim::Aim, held: bool, value: f64, tie: f64) -> String {
    let mut t = measure_text(&aim.measure, value);
    if let Some(m) = &aim.then {
        t += &format!(" \u{00b7} {}", measure_text(m, tie));
    }
    if !held {
        t += " \u{00b7} conditions not met";
    }
    t
}

pub fn aim_sentence(aim: &gearbox_search::aim::Aim, at: &str) -> String {
    use gearbox_search::aim::{Condition, Quantity, Wheels};
    let q = |q: &Quantity| match q {
        Quantity::Angle { yaw, pitch, roll } => {
            let axes: Vec<String> = [("yaw", yaw), ("pitch", pitch), ("roll", roll)].iter().filter_map(|(n, v)| v.map(|v| format!("{n} {v:.0}\u{00b0}"))).collect();
            format!("the car's {}", if axes.is_empty() { "angle".into() } else { axes.join(", ") })
        }
        Quantity::Trick(k) => format!("a {}", trick_name(*k)),
        Quantity::Distance { .. } => "the distance to the point".into(),
        other => quantity_label(other).0.to_lowercase(),
    };
    let measure = match aim.measure.quantity {
        Quantity::Angle { .. } => format!("get {} as close as possible", q(&aim.measure.quantity)),
        Quantity::Trick(_) => format!("hold {} as long as possible", q(&aim.measure.quantity)),
        _ => format!("{} {}", if aim.measure.higher { "maximise" } else { "minimise" }, q(&aim.measure.quantity)),
    };
    let when = match aim.judged {
        gearbox_search::aim::Judged::End => format!("At {at}"),
        gearbox_search::aim::Judged::Best => format!("At its best tick up to {at}"),
    };
    let conds: Vec<String> = aim
        .conditions
        .iter()
        .map(|c| match c {
            Condition::Above(x, v) => format!("{} over {v:.0} {}", q(x), quantity_label(x).1),
            Condition::Below(x, v) => format!("{} under {v:.0} {}", q(x), quantity_label(x).1),
            Condition::Wheels { which, touching } => {
                let w = match which {
                    Wheels::Front => "the front wheels",
                    Wheels::Rear => "the rear wheels",
                    Wheels::All => "all four wheels",
                    Wheels::Any => "a wheel",
                };
                format!("{w} {}", if *touching { "touching" } else { "off the ground" })
            }
            Condition::Inside { .. } => "the car inside the box".into(),
        })
        .collect();
    let mut out = format!("{when}, {measure}");
    if !conds.is_empty() {
        out += &format!(", with {}", conds.join(" and "));
    }
    if let Some(m) = &aim.then {
        out += &format!("; then the {} {}", if m.higher { "higher" } else { "lower" }, q(&m.quantity));
    }
    out
}

impl Default for Spec {
    fn default() -> Self {
        Self {
            kind: ObjKind::ReachGoals,
            goals: Vec::new(),
            by_speed: Vec::new(),
            both: Vec::new(),
            keep_cps: true,
            aim: gearbox_search::aim::Aim::fastest(),
            at_tick: None,
            start_from: StartFrom::Run,
            vary_car: true,
            window: 0..0,
            until: None,
            channels: [true; 3],
            digital_steer: false,
            steer_limit: 1.0,
            honour_locks: true,
            by_time: true,
            attempts: 1024,
            seconds: 0.0,
            go_on: false,
            threads: 0,
            seed: 1,
            auto: true,
        }
    }
}

pub struct Ctx<'a> {
    pub goals: &'a [Goal],
    pub race_ms0: i32,
    pub reach: &'a [Option<usize>],
    pub waypoints_at: &'a dyn Fn(usize) -> usize,
}

impl Spec {
    pub fn ordered_goals<'a>(&self, goals: &'a [Goal]) -> Vec<&'a Goal> {
        if self.goals.is_empty() {
            goals.iter().collect()
        } else {
            self.goals.iter().filter_map(|id| goals.iter().find(|g| g.id == *id)).collect()
        }
    }

    pub fn from_saved(mut v: serde_json::Value, goals: &[Goal]) -> Result<Self> {
        let old = v.get("kind").and_then(|k| k.as_str()).map(str::to_owned);
        if let Some(k @ ("GateTime" | "GateSpeed" | "MatchPose" | "Finish")) = old.as_deref() {
            let target = v.get("target").and_then(|t| t.as_u64()).map(|t| t as Id);
            let one = if k == "Finish" { goals.iter().find(|g| matches!(&g.kind, GoalKind::Gate(x) if x.kind == 1)).map(|g| g.id) } else { target };
            v["kind"] = "ReachGoals".into();
            if let Some(id) = one {
                v["goals"] = serde_json::json!([id]);
                if k == "GateSpeed" {
                    v["by_speed"] = serde_json::json!([id]);
                }
            }
        }
        if let Some("GoalByGoal" | "DistanceAt" | "Path") = old.as_deref() {
            v["kind"] = "ReachGoals".into();
        }
        if let Some(k @ ("SpeedAt" | "Trick")) = old.as_deref() {
            let aim = if k == "SpeedAt" {
                gearbox_search::aim::Aim::fastest()
            } else {
                let trick = v.get("trick").cloned().and_then(|t| serde_json::from_value(t).ok()).unwrap_or(gearbox_search::tricks::TrickKind::Nosebug);
                gearbox_search::aim::Aim::longest_trick(trick)
            };
            (v["kind"], v["aim"]) = ("Moment".into(), serde_json::to_value(aim)?);
        }
        Ok(serde_json::from_value(v)?)
    }

    pub fn until_tick(&self, c: &Ctx<'_>) -> usize {
        if self.kind.fixed_time() {
            return self.at_tick.unwrap_or(self.window.end).max(self.window.end);
        }
        let n = self.ordered_goals(c.goals).len();
        let idx: Vec<usize> = self.ordered_goals(c.goals).iter().filter_map(|g| c.goals.iter().position(|x| x.id == g.id)).collect();
        let last = if n > 0 && idx.len() == n { idx.last().and_then(|&i| c.reach.get(i).copied().flatten()) } else { None };
        if let Some(u) = self.until {
            return u.max(self.window.end).max(last.map_or(0, |k| k + 150));
        }
        last.map_or(self.window.end + 300, |k| k + 150).max(self.window.end + 100)
    }

    pub fn sentence(&self, c: &Ctx<'_>) -> String {
        let t = |k: usize| format!("{} s", fmt_s(c.race_ms0 + 10 * k as i32));
        let judged = |g: &Goal| {
            if self.by_speed.contains(&g.id) {
                "speed"
            } else if self.both.contains(&g.id) {
                "time and speed"
            } else {
                "time"
            }
        };
        match self.kind {
            ObjKind::ReachGoals => {
                let gs = self.ordered_goals(c.goals);
                if gs.is_empty() {
                    return "Place a goal first (1\u{2013}4: gate, point, box, pose); the search aims for it".into();
                }
                let last = gs[gs.len() - 1];
                let tie = if gs.iter().all(|g| g.is_gate()) { "" } else { " \u{00b7} then closest to the next goal" };
                if gs.len() == 1 {
                    match judged(last) {
                        "speed" => format!("Maximise speed at {} \u{00b7} then minimise time to reach it{tie}", last.name),
                        "time" => format!("Minimise time to reach {}{tie}", last.name),
                        _ => format!("Minimise time to reach {}, less what its speed saves later{tie}", last.name),
                    }
                } else {
                    let names: Vec<String> = gs.iter().map(|g| g.name.clone()).collect();
                    format!("Reach {} in order \u{00b7} as far as possible, then by {} at the last{tie}", names.join(" \u{2192} "), judged(last))
                }
            }
            ObjKind::Moment => aim_sentence(&self.aim, &t(self.until_tick(c))),
        }
    }

    pub fn constraint_sentence(&self, c: &Ctx<'_>) -> String {
        let ch: Vec<&str> = ["gas", "brake", "steer"].iter().zip(self.channels).filter(|x| x.1).map(|x| *x.0).collect();
        let s = |k: usize| fmt_s(c.race_ms0 + 10 * k as i32);
        let mut out = format!(
            "Changing {} from {} to {} s, each try driven to {} s",
            if ch.is_empty() { "nothing".to_string() } else { ch.join(", ") },
            s(self.window.start),
            s(self.window.end),
            s(self.until_tick(c))
        );
        if self.channels[2] && self.digital_steer {
            out += " \u{00b7} keyboard steer";
        }
        if self.channels[2] && self.steer_limit < 1.0 {
            out += &format!(" \u{00b7} |steer| \u{2264} {:.2}", self.steer_limit);
        }
        out
    }

    pub fn semantics(&self) -> Vec<String> {
        let mut v: Vec<String> = match self.kind {
            ObjKind::ReachGoals => vec![
                "1. More goals reached, in list order, is better. Reaching always beats missing.".into(),
                "2. Still short of the last goal: the smaller error to the next unreached goal wins, observed after its predecessor was reached: the way left through open space to a point or a gate (below a goal counts as climbing); for a pose, \u{03a3} wᵢ\u{00b7}Δposᵢ\u{00b2} + w\u{03b8}\u{00b7}angle\u{00b2} (+ velocity terms).".into(),
                "3. Then the earlier time at the last goal reached, so finishing sooner beats hitting an early goal sooner; the earlier goals' times break ties. With the last goal on Speed: the higher speed there, then the earlier time; on Both: the time less each km/h's worth.".into(),
                "A goal counts only after the previous one; a gate counts only as crossing #N of its waypoint ID. A pose is reached when every axis is within its tolerance and the shortest rotation angle within its tolerance.".into(),
                format!("Misses are measured through open space: the map is cut into {TRACK_CELL} m cells, open where a surface passes or up to {} m above one; the distance is the shortest way through open cells to the goal.", TRACK_CELL * TRACK_CLEARANCE as f32),
            ],
            ObjKind::Moment => vec![
                "1. Every condition holding beats any try where one fails.".into(),
                "2. Then the measure: the higher or lower value, at the evaluation time or at the range's best tick. An angle counts its degrees off the target on each axis given, the shortest way round, combined (√ of the squares).".into(),
                "3. Then the tie-break measure.".into(),
                "Speed is |v| in km/h; height the car centre's y; yaw the heading (0 along +z), pitch the nose's elevation (−90: straight down), roll the bank. Trick ticks count the trick's signature so far (nosebug: nose down over 35\u{00b0}, front wheels on, rear off; bugslide: sliding over 40\u{00b0} for 10 ticks without losing speed; uberbug: a contact adding over 5 m/s and 10 % in a tick). Signatures, not proof.".into(),
            ],
        };
        v.push("Only ticks in the range change; inputs before it are replayed as they are.".into());
        v.push("Every change is tried, from a few ticks to a new turn-in with the rest driven on along the line; the best few lines that differ (by 4 m somewhere) are kept, so a different line that is slower for now survives while it gets better.".into());
        v
    }

    pub fn objective(&self, c: &Ctx<'_>) -> Result<(Objective, i32)> {
        let until = self.until_tick(c);
        let until_ms = c.race_ms0 + 10 * until as i32;
        let gate_of = |g: &Goal, before: usize| -> Result<(usize, gearbox_search::gate::Gate)> {
            let GoalKind::Gate(gate) = &g.kind else { bail!("{} is not a gate", g.name) };
            let i = c.goals.iter().position(|x| x.id == g.id).unwrap_or(0);
            let ordinal = match gate.ordinal {
                Some(o) => o,
                None => match c.reach.get(i).copied().flatten() {
                    Some(k) => (c.waypoints_at)(k),
                    None => (c.waypoints_at)(self.window.start) + 1 + before,
                },
            };
            Ok((ordinal, gearbox_search::gate::Gate { waypoint: gate.waypoint, kind: gate.kind, boxes: gate.boxes.clone() }))
        };
        let goal = |g: &Goal, before: usize| -> Result<gearbox_search::goals::Goal> {
            Ok(match &g.kind {
                GoalKind::Point { pos, radius } => gearbox_search::goals::Goal::Point { point: *pos, radius: *radius },
                GoalKind::Pose(p) => gearbox_search::goals::Goal::Pose { target: p.goal() },
                GoalKind::Box { pos, size, ypr } => gearbox_search::goals::Goal::Box { trigger: crate::doc::trigger_box(*pos, *size, *ypr) },
                GoalKind::Gate(_) => {
                    let (ordinal, gate) = gate_of(g, before)?;
                    let GoalKind::Gate(x) = &g.kind else { unreachable!() };
                    gearbox_search::goals::Goal::Gate { ordinal, gate, also: x.also.clone() }
                }
            })
        };
        let when = |g: &Goal| -> gearbox_search::goals::Conditions {
            let mut k = g.when.conditions();
            if self.keep_cps
                && let Some(i) = c.goals.iter().position(|x| x.id == g.id)
                && let Some(t) = c.reach.get(i).copied().flatten()
            {
                k.min_crossings = Some((c.waypoints_at)(t) as u32).filter(|n| *n > 0);
            }
            k
        };
        let obj = match self.kind {
            ObjKind::ReachGoals => {
                let gs = self.ordered_goals(c.goals);
                ensure!(!gs.is_empty(), "add at least one goal");
                let conditions = gs.iter().map(|g| when(g)).collect();
                let gs_last = gs.last().map(|g| g.id);
                let mut gates_before = 0;
                let mut ordered = Vec::new();
                for g in gs {
                    ordered.push(goal(g, gates_before)?);
                    if g.is_gate() {
                        gates_before += 1;
                    }
                }
                let fastest = gs_last.is_some_and(|id| self.by_speed.contains(&id));
                Objective::Goals(gearbox_search::goals::Goals { ordered, fastest, speed_worth_ms: 0.0, conditions, along: None, stay_within: None, track: None })
            }
            ObjKind::Moment => Objective::Aim { at_ms: until_ms, aim: self.aim.clone() },
        };
        Ok((obj, until_ms))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Metric {
    pub value: f64,
    pub unit: &'static str,
    pub reached: bool,
    pub lower: bool,
    pub text: String,
}

pub fn metric(s: &Score) -> Metric {
    match s {
        Score::Goals { times, total, error, .. } => {
            if times.len() == *total {
                let t = *times.last().unwrap_or(&0);
                let text = if *total == 1 { format!("reached at {} s", fmt_ms(t)) } else { format!("all {total} reached, last at {} s", fmt_ms(t)) };
                Metric { value: t as f64 / 1000.0, unit: "s", reached: true, lower: true, text }
            } else {
                let text = if times.is_empty() { "not reached yet".to_string() } else { format!("{} of {total} reached", times.len()) };
                Metric { value: *error, unit: "off", reached: false, lower: true, text }
            }
        }
        Score::Unreached => Metric { value: f64::NAN, unit: "", reached: false, lower: true, text: "not reached".into() },
        Score::Race { time_ms, speed_squared } => Metric { value: *time_ms as f64 / 1000.0, unit: "s", reached: true, lower: true, text: format!("{} \u{00b7} {:.1} km/h", fmt_ms(*time_ms as i32), speed_squared.sqrt() * 3.6) },
        Score::Position { distance_squared, speed_squared } => Metric { value: distance_squared.sqrt(), unit: "m", reached: true, lower: true, text: format!("{:.3} m \u{00b7} {:.1} km/h", distance_squared.sqrt(), speed_squared.sqrt() * 3.6) },
        Score::Trick { ticks, speed_squared } => Metric { value: f64::from(*ticks) / 100.0, unit: "s", reached: *ticks > 0, lower: false, text: format!("held {:.2} s \u{00b7} {:.1} km/h", f64::from(*ticks) / 100.0, speed_squared.sqrt() * 3.6) },
        Score::Aim { held, value, tie } => Metric { value: *value, unit: "", reached: *held, lower: false, text: format!("{value:.2}{}{}", if *tie != 0.0 { format!(" \u{00b7} {tie:.1}") } else { String::new() }, if *held { "" } else { " \u{00b7} conditions not met" }) },
        Score::Speed { speed_squared } => Metric { value: speed_squared.sqrt() * 3.6, unit: "km/h", reached: true, lower: false, text: format!("{:.2} km/h", speed_squared.sqrt() * 3.6) },
        Score::Guided { time_ms: Some(t), cost, speed_squared, .. } => {
            let speed = speed_squared.sqrt() * 3.6;
            if *cost < 0.0 {
                Metric { value: speed, unit: "km/h", reached: true, lower: false, text: format!("through at {speed:.1} km/h ({} s)", fmt_ms(*t as i32)) }
            } else {
                Metric { value: *t as f64 / 1000.0, unit: "s", reached: true, lower: true, text: format!("reached at {} s, {speed:.0} km/h", fmt_ms(*t as i32)) }
            }
        }
        Score::Guided { time_ms: None, cost, .. } => Metric { value: *cost, unit: "off", reached: false, lower: true, text: "not reached yet".into() },
    }
}

#[derive(Clone, Debug)]
pub struct Candidate {
    pub id: u64,
    pub start: Option<crate::doc::StartState>,
    pub attempts: usize,
    pub score: Score,
    pub inputs: Arc<InputFile>,
    pub elapsed: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mark {
    Missed,
    Reached,
}

impl Mark {
    pub fn reached(r: bool) -> Self {
        if r { Self::Reached } else { Self::Missed }
    }
}

#[derive(Default)]
pub struct Live {
    pub running: bool,
    pub attempts: usize,
    pub elapsed: f32,
    pub base: Option<Score>,
    pub best: Option<Score>,
    pub series: Vec<(usize, Metric)>,
    pub candidates: Vec<Candidate>,
    pub message: String,
    pub error: Option<String>,
    pub until_tick: usize,
    pub tries: std::collections::VecDeque<(Vec<[f32; 3]>, Mark)>,
    pub tries_gen: u64,
    pub aim: Option<gearbox_search::aim::Aim>,
}

impl Live {
    pub fn metric(&self, s: &Score) -> Metric {
        let mut m = metric(s);
        if let (Score::Aim { held, value, tie }, Some(aim)) = (s, &self.aim) {
            m.text = aim_text(aim, *held, *value, *tie);
        }
        m
    }
}

const MAX_TRIES: usize = 192;

pub struct Request {
    pub origin: Savestate,
    pub inputs: InputFile,
    pub race_ms0: i32,
    pub spec: Spec,
    pub objective: Objective,
    pub until_ms: i32,
    pub locks: Vec<Range<usize>>,
    pub start: Option<(usize, Placement)>,
    pub origin_tick: usize,
    pub start_line: Option<Vec<[f32; 3]>>,
    pub world: Option<(Arc<crate::engine::display::World>, [f32; 3], [f32; 3])>,
}

const TRACK_CELL: f32 = 3.0;
const TRACK_CLEARANCE: i32 = 4;
const TRACK_REACH: i32 = 8;

static TRACK_CACHE: Mutex<Option<(String, Arc<gearbox_search::track::TrackFields>)>> = Mutex::new(None);

fn track_fields(world: &Arc<crate::engine::display::World>, min: [f32; 3], max: [f32; 3], goals: &[gearbox_search::goals::Goal]) -> Arc<gearbox_search::track::TrackFields> {
    use gearbox_search::goals::Goal;
    let key = format!("{:p} {min:?} {max:?} {}", Arc::as_ptr(world), serde_json::to_string(goals).unwrap_or_default());
    if let Some((k, f)) = TRACK_CACHE.lock().unwrap().as_ref()
        && *k == key
    {
        return f.clone();
    }
    let inside = |t: &[[f32; 3]; 3]| (0..3).all(|a| t.iter().any(|v| v[a] >= min[a]) && t.iter().any(|v| v[a] <= max[a]));
    let dry = |t: &&crate::engine::display::Tri| crate::materials::family(t.surface) != crate::materials::Family::Water;
    let cells = Arc::new(gearbox_search::track::Cells::build(world.tris.iter().filter(dry).map(|t| t.v).filter(inside), TRACK_CELL, TRACK_CLEARANCE));
    let near = TRACK_CELL * 0.5;
    let fields = goals
        .iter()
        .map(|g| {
            let (center, test): ([f32; 3], Box<dyn Fn([f32; 3]) -> bool>) = match g.clone() {
                Goal::Point { point, radius } => (point, Box::new(move |p| gearbox_search::objective::distance(p, point) <= f64::from(radius + near))),
                Goal::Box { trigger } => (trigger.center, Box::new(move |p| trigger.distance(p) <= f64::from(near))),
                Goal::Pose { target } => {
                    let (c, tol) = (target.pose.position, target.position_tolerance);
                    (c, Box::new(move |p| (0..3).all(|a| (p[a] - c[a]).abs() <= tol[a] + near)))
                }
                Goal::Gate { gate, .. } => {
                    let c = gate.boxes.first().map_or([0.0; 3], |b| std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5));
                    (c, Box::new(move |p| gate.distance(p) <= f64::from(near)))
                }
                Goal::Waypoint { .. } | Goal::Finish => return None,
            };
            Some(gearbox_search::track::GoalField::build(cells.clone(), center, test))
        })
        .collect();
    let f = Arc::new(gearbox_search::track::TrackFields { goals: fields, reach: TRACK_REACH });
    *TRACK_CACHE.lock().unwrap() = Some((key, f.clone()));
    f
}

struct Cancelled;

pub struct Search {
    pub live: Arc<Mutex<Live>>,
    pub cancel: Arc<AtomicBool>,
    next_id: Arc<std::sync::atomic::AtomicU64>,
}

impl Default for Search {
    fn default() -> Self {
        Self { live: Arc::default(), cancel: Arc::default(), next_id: Arc::new(1.into()) }
    }
}

impl Search {
    pub fn running(&self) -> bool {
        self.live.lock().unwrap().running
    }

    pub fn stop(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    pub fn start(&mut self, req: Request, wake: Wake) {
        self.cancel.store(true, Ordering::Relaxed);
        self.cancel = Arc::new(AtomicBool::new(false));
        self.live = Arc::new(Mutex::new(Live { running: true, message: "starting \u{2026}".into(), until_tick: ((req.until_ms - req.race_ms0) / 10).max(0) as usize, aim: match &req.objective {
            Objective::Aim { aim, .. } => Some(aim.clone()),
            _ => None,
        }, ..Default::default() }));
        let (live, cancel, ids) = (self.live.clone(), self.cancel.clone(), self.next_id.clone());
        std::thread::Builder::new()
            .name("search".into())
            .spawn(move || {
                let r = catch_unwind(AssertUnwindSafe(|| run(&req, &live, &cancel, &ids, &wake)));
                let mut l = live.lock().unwrap();
                l.running = false;
                match r {
                    Ok(Ok(())) => {}
                    Ok(Err(e)) => l.error = Some(format!("{e:#}")),
                    Err(p) if p.is::<Cancelled>() => l.message = format!("stopped after {} attempts", l.attempts),
                    Err(p) => l.error = Some(format!("search panicked: {}", panic_message(&*p))),
                }
                drop(l);
                wake();
            })
            .expect("spawning the search thread");
    }
}

fn run(req: &Request, live: &Mutex<Live>, cancel: &AtomicBool, ids: &std::sync::atomic::AtomicU64, wake: &Wake) -> Result<()> {
    let s = &req.spec;
    let ms = |k: usize| req.race_ms0 + 10 * k as i32;
    ensure!(s.window.start < s.window.end, "the search range is empty: set it on the timeline or with From \u{2192} to");
    if let Some((p, _)) = &req.start {
        ensure!(s.window.start >= *p, "the window starts before the start state (tick {p}); inputs before a placed start can't be searched");
    }
    ensure!(s.channels.iter().any(|c| *c), "no channel may change");
    let car = match &req.start {
        Some((t, p)) if s.vary_car && *t == req.origin_tick => Some((*t, p.clone())),
        _ => None,
    };
    let origin = reach(&req.origin, &req.inputs, req.origin_tick, if car.is_some() { None } else { req.start.as_ref() })?;
    let mut objective = req.objective.clone();
    if let Objective::Goals(g) = &mut objective
        && let Some((world, min, max)) = &req.world
    {
        live.lock().unwrap().message = "measuring the track to the goals \u{2026}".into();
        wake();
        let t = Instant::now();
        g.track = Some(track_fields(world, *min, *max, &g.ordered));
        live.lock().unwrap().message = format!("track measured in {:.1} s", t.elapsed().as_secs_f32());
        wake();
    }
    let t0 = req.inputs.t0;
    let at = |race_ms: i32| ((race_ms - t0).max(0) / 10) as usize;
    let constraints = gearbox_search::mutation::Constraints {
        fixed: s.channels.map(|c| !c),
        digital_steer: s.digital_steer,
        steer_limit: (s.steer_limit < 1.0).then_some(s.steer_limit.clamp(0.0, 1.0)),
        locked: if s.honour_locks { req.locks.iter().map(|r| at(ms(r.start))..at(ms(r.end))).collect() } else { Vec::new() },
    };
    let threads = if s.threads == 0 { gearbox_search::search::default_threads() } else { s.threads.clamp(1, gearbox_search::search::MAX_THREADS) };
    let range = at(ms(s.window.start))..at(ms(s.window.end));
    let until = at(req.until_ms).max(range.end);
    let start = gearbox_search::input::Timeline { t0, seed: req.inputs.seed, controls: req.inputs.controls.clone() };
    let base = {
        let mut n = origin.fork();
        if let Some((_, p)) = &car {
            gearbox_search::pose::place(&mut n, p)?;
        }
        n.seed = start.seed;
        let opts = gearbox_search::objective::EvalOptions { from_ms: Some(t0 + 10 * range.start as i32), ..Default::default() };
        gearbox_search::objective::evaluate_with(&mut n, &start, &objective, req.until_ms, opts)?.score
    };
    live.lock().unwrap().base = Some(base);
    let start = match &req.start_line {
        Some(_) if car.is_some() => start,
        Some(points) => {
            ensure!(points.len() >= 2, "the line to start from needs at least two points");
            let mut held = start.clone();
            held.controls[range.clone()].fill(gearbox_engine::Controls { gas: 1.0, brake: 0.0, steer: 0.0 });
            gearbox_search::tries::drive_on(&origin, &held, range.start, range.start, range.end, points)?.line
        }
        None => start,
    };
    let settings = gearbox_search::improve::Settings { threads, seed: s.seed, constraints, eval: Default::default(), placement: car.as_ref().map(|c| c.1.clone()) };
    let car_tick = car.as_ref().map(|c| c.0);
    let began = Instant::now();
    let mut listed: std::collections::HashMap<usize, u64> = Default::default();
    gearbox_search::improve::improve(&origin, &objective, start, range, until, &settings, |p| {
        let elapsed = began.elapsed().as_secs_f32();
        let mut l = live.lock().unwrap();
        let best = &p.lines[0];
        if l.best.as_ref().is_none_or(|b| best.score.quality_cmp(b).is_gt()) {
            l.best = Some(best.score.clone());
            l.series.push((p.tries, metric(&best.score)));
        }
        l.candidates = p
            .lines
            .iter()
            .rev()
            .filter(|x| x.change.is_some())
            .map(|x| Candidate {
                id: *listed.entry(x.id).or_insert_with(|| ids.fetch_add(1, Ordering::Relaxed)),
                start: car_tick.zip(x.placement.as_ref()).map(|(tick, p)| crate::doc::StartState::from_placement(tick, p)),
                attempts: p.tries,
                score: x.score.clone(),
                inputs: Arc::new(InputFile { t0: x.inputs.t0, seed: x.inputs.seed, controls: x.inputs.controls.clone() }),
                elapsed,
            })
            .collect();
        for (trail, kept) in p.latest {
            if l.tries.len() >= MAX_TRIES {
                l.tries.pop_front();
            }
            l.tries.push_back((trail.iter().step_by(5).copied().collect(), Mark::reached(*kept)));
        }
        l.attempts = p.tries;
        l.elapsed = elapsed;
        l.tries_gen += 1;
        l.message = format!("{} tries \u{00b7} {} lines", p.tries, l.candidates.len());
        drop(l);
        wake();
        let spent = if s.by_time { s.seconds > 0.0 && elapsed >= s.seconds } else { p.tries >= s.attempts.max(1) };
        !cancel.load(Ordering::Relaxed) && !spent
    })?;
    let mut l = live.lock().unwrap();
    l.message = if cancel.load(Ordering::Relaxed) { format!("stopped after {} tries", l.attempts) } else { format!("done \u{00b7} {} tries in {:.1} s \u{00b7} {} lines", l.attempts, began.elapsed().as_secs_f32(), l.candidates.len()) };
    Ok(())
}

pub fn goal_times(score: &Score) -> Option<&[i32]> {
    match score {
        Score::Goals { times, .. } => Some(times),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx<'a>(goals: &'a [Goal], reach: &'a [Option<usize>], w: &'a dyn Fn(usize) -> usize) -> Ctx<'a> {
        Ctx { goals, race_ms0: -1500, reach, waypoints_at: w }
    }

    fn gate(id: Id, name: &str, wp: u32) -> Goal {
        Goal { id, name: name.into(), kind: GoalKind::Gate(crate::doc::GateGoal { waypoint: wp, kind: 2, boxes: vec![gearbox_search::gate::Bounds { min: [0.0; 3], max: [1.0; 3] }], ordinal: None, also: Vec::new(), parts: Vec::new() }), visible: true, when: Default::default() }
    }

    #[test]
    fn sentences_name_the_goals_and_the_tie_break() {
        let goals = vec![gate(1, "CP2", 7), Goal { id: 2, name: "P1".into(), kind: GoalKind::Point { pos: [1.0, 2.0, 3.0], radius: 3.0 }, visible: true, when: Default::default() }];
        let reach = [Some(800), None];
        let w = |_| 1usize;
        let c = ctx(&goals, &reach, &w);
        let s = Spec { kind: ObjKind::ReachGoals, goals: vec![1], window: 300..700, ..Default::default() };
        assert_eq!(s.sentence(&c), "Minimise time to reach CP2");
        let s = Spec { kind: ObjKind::ReachGoals, goals: vec![1], by_speed: vec![1], window: 300..700, ..Default::default() };
        assert_eq!(s.sentence(&c), "Maximise speed at CP2 \u{00b7} then minimise time to reach it");
        let s = Spec { kind: ObjKind::ReachGoals, window: 300..700, ..Default::default() };
        assert_eq!(s.sentence(&c), "Reach CP2 \u{2192} P1 in order \u{00b7} as far as possible, then by time at the last \u{00b7} then closest to the next goal");
        let s = Spec { kind: ObjKind::ReachGoals, goals: vec![1], window: 300..700, ..Default::default() };
        assert_eq!(s.until_tick(&c), 950);
        assert_eq!(s.constraint_sentence(&c), "Changing gas, brake, steer from 1.500 to 5.500 s, each try driven to 8.000 s");
    }

    #[test]
    fn gate_ordinals_come_from_the_run_or_count_from_the_window() {
        let goals = vec![gate(1, "CP1", 3), gate(2, "CP2", 5)];
        let w = |k: usize| if k >= 800 { 2 } else { 1 };
        let reach = [Some(800), None];
        let c = ctx(&goals, &reach, &w);
        let s = Spec { kind: ObjKind::ReachGoals, window: 300..700, ..Default::default() };
        let (o, _) = s.objective(&c).unwrap();
        let Objective::Goals(g) = o else { panic!() };
        let ords: Vec<usize> = g.ordered.iter().map(|g| if let gearbox_search::goals::Goal::Gate { ordinal, .. } = g { *ordinal } else { 0 }).collect();
        assert_eq!(ords, [2, 3]);
    }

    #[test]
    fn older_single_goal_and_finish_objectives_load_as_goals() {
        let fin = Goal { id: 9, name: "Finish".into(), kind: GoalKind::Gate(crate::doc::GateGoal { waypoint: 4, kind: 1, boxes: vec![gearbox_search::gate::Bounds { min: [0.0; 3], max: [1.0; 3] }], ordinal: None, also: vec![5], parts: Vec::new() }), visible: true, when: Default::default() };
        let goals = vec![gate(1, "CP1", 3), fin];
        let saved = |kind: &str| {
            let mut v = serde_json::to_value(Spec::default()).unwrap();
            v["kind"] = kind.into();
            v["target"] = 1.into();
            Spec::from_saved(v, &goals).unwrap()
        };
        let s = saved("GateSpeed");
        assert_eq!((s.kind, s.goals, s.by_speed), (ObjKind::ReachGoals, vec![1], vec![1]));
        let s = saved("GateTime");
        assert_eq!((s.goals, s.by_speed), (vec![1], vec![]));
        assert_eq!(saved("Finish").goals, vec![9]);
        assert_eq!((saved("SpeedAt").kind, saved("SpeedAt").aim), (ObjKind::Moment, gearbox_search::aim::Aim::fastest()));
        assert_eq!(saved("GoalByGoal").kind, ObjKind::ReachGoals);
    }

    #[test]
    fn metrics_say_which_way_is_better() {
        let m = metric(&Score::Goals { times: vec![1200, 3400], total: 2, error: 0.0, strayed: false, cuts: 0 });
        assert!(m.reached && m.lower && (m.value - 3.4).abs() < 1e-9);
        let m = metric(&Score::Speed { speed_squared: 100.0 });
        assert!(!m.lower && (m.value - 36.0).abs() < 1e-9);
        let m = metric(&Score::Guided { time_ms: None, cost: 4.5, speed_squared: 1.0, bonus: 0.0 });
        assert!(!m.reached && m.value == 4.5 && m.text == "not reached yet");
    }
}
