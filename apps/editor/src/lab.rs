use std::collections::HashMap;

use crate::app::{Editor, Sel};
use crate::doc::{self, GoalKind, Id, Setup};
use crate::optimize::ObjKind;

const REACH_TICKS: usize = 800;

const TRY_TICKS: usize = 100;
const TRY_SECONDS: f32 = 20.0;

impl Editor {
    pub fn save_setup(&mut self) {
        let Some(start) = self.doc.start.clone() else {
            return self.error(
                "lab",
                "place the car first, or save a setup from the run (Properties of the car at the playhead)",
            );
        };
        if self.cursor <= start.tick {
            return self.error("lab", "move the playhead past the start state: the setup is the inputs from the placement to the playhead");
        }
        let to = self.cursor;
        self.store_setup(start.clone(), start.tick, to, true);
    }

    pub fn save_setup_here(&mut self) {
        let Some(start) = self.doc.start.clone() else { return self.save_setup() };
        if self.cursor > start.tick {
            return self.save_setup();
        }
        let to = start.tick + self.experiment.ticks;
        if to >= self.sim.frames.len() {
            return self.error("lab", "the run isn't simulated that far yet");
        }
        self.store_setup(start.clone(), start.tick, to, true);
    }

    pub fn setup_clips(&self) -> Vec<(Id, usize, usize, String)> {
        self.doc
            .setups
            .iter()
            .filter_map(|s| {
                let i = self.doc.goals.iter().position(|g| g.name == format!("{} start", s.name))?;
                let k = self.reach.get(i)?.tick?;
                Some((s.id, k, k + s.controls.len(), s.name.clone()))
            })
            .collect()
    }

    pub fn route_ticks(&self) -> Option<(usize, usize)> {
        let stops = self.route_stops();
        let from = match self.route.from {
            RouteFrom::RaceStart => self.race_start_tick(),
            RouteFrom::Placed => self.doc.start.as_ref()?.tick,
            RouteFrom::Stop(i) => stops.get(i)?.tick?,
        };
        let to = stops.iter().find(|s| s.at == self.route.to)?.tick?;
        (to > from).then_some((from, to))
    }

    pub fn save_setup_from_run(&mut self, ticks: usize) {
        let Some(start) = self.start_from_cursor() else { return };
        let (from, to) = (self.cursor, self.cursor + ticks);
        if to >= self.sim.frames.len() {
            return self
                .error("lab", "the run isn't simulated that far: extend it or move the playhead back");
        }
        self.store_setup(start, from, to, false);
    }

    fn store_setup(&mut self, start: doc::StartState, from: usize, to: usize, placed: bool) {
        let Some(f) = self.sim.frames.get(to) else { return };
        let controls = (from..to).map(|k| self.control(k)).map(|c| [c.gas, c.brake, c.steer]).collect();
        let (end_pos, end_ypr, end_velocity) = (f.position(), doc::rot_ypr(doc::frame_rot(f)), f.velocity());
        let tricks = self
            .trick_ticks(from, to - from, placed)
            .map(|e| [e.nosebug_ticks, e.bugslide_ticks, e.uberbug_ticks]);
        let n = self.doc.setups.len() + 1;
        let mut id = 0;
        self.edit("save setup", None, |d| {
            id = d.id();
            d.setups.push(Setup {
                id,
                name: format!("Setup {n}"),
                start,
                controls,
                end_pos,
                end_ypr,
                end_velocity,
                tricks,
            });
        });
        self.sel = Sel::Setup(id);
        self.info(
            "lab",
            format!(
                "saved Setup {n}: {} ticks{}",
                to - from,
                if placed { " from the placed car" } else { " from the run" }
            ),
        );
    }

    pub fn trick_ticks(
        &self,
        from: usize,
        ticks: usize,
        placed: bool,
    ) -> Option<gearbox_search::tricks::Events> {
        let origin = self.origin.as_ref()?;
        let start = if placed { self.doc.start.as_ref().map(|s| (s.tick, s.placement())) } else { None };
        let at = crate::background::reach(origin, &self.doc.inputs, from, start.as_ref()).ok()?;
        let mut n = at.fork();
        let mut line = gearbox_search::input::Timeline {
            t0: self.doc.inputs.t0,
            seed: self.doc.inputs.seed,
            controls: self.doc.inputs.controls.clone(),
        };
        let until = self.race_ms0 + 10 * (from + ticks) as i32;
        let need = ((until - line.t0).max(0) / 10) as usize;
        if line.controls.len() < need {
            let last = line.controls.last().copied().unwrap_or_default();
            line.controls.resize(need, last);
        }
        let objective = gearbox_search::objective::Objective::Trick {
            at_ms: until,
            kind: gearbox_search::tricks::TrickKind::Any,
        };
        gearbox_search::objective::evaluate_with(
            &mut n,
            &line,
            &objective,
            until,
            gearbox_search::objective::EvalOptions::default(),
        )
        .ok()
        .map(|e| e.events)
    }

    pub fn check_setup_tricks(&mut self, id: Id) {
        let Some(s) = self.doc.setup(id).cloned() else { return };
        let name = format!("{} start", s.name);
        let tick = self
            .doc
            .goals
            .iter()
            .position(|g| g.name == name)
            .and_then(|i| self.reach.get(i))
            .and_then(|r| r.tick);
        let Some(tick) = tick else {
            return self.error("lab", format!("the run doesn't reach {name}: Reach and Join first"));
        };
        match self.trick_ticks(tick, s.controls.len(), false) {
            Some(e) => {
                self.trick_checks.insert(id, [e.nosebug_ticks, e.bugslide_ticks, e.uberbug_ticks]);
            }
            None => self.error("lab", "couldn't run the session for the check"),
        }
    }

    pub fn try_from(&mut self, k: usize) {
        self.try_range(k, TRY_TICKS);
    }

    pub fn try_setup_entry(&mut self, id: Id) {
        match self.setup_clips().into_iter().find(|c| c.0 == id) {
            Some((_, from, to, _)) => self.try_range(from, to - from),
            None => self.error("try", "the run doesn't reach this setup: join it first"),
        }
    }

    fn try_range(&mut self, k: usize, ticks: usize) {
        let k = k.max(self.race_start_tick());
        let end = (k + ticks).min(self.doc.len.saturating_sub(1));
        if end <= k {
            return self.error("try", "no run left after this moment");
        }
        let s = &mut self.spec;
        s.window = k..end;
        if !s.kind.goals() || s.goals.is_empty() {
            (s.kind, s.at_tick) = (ObjKind::Moment, Some(end));
            if s.aim.measure.quantity != gearbox_search::aim::Quantity::Speed || !s.aim.conditions.is_empty()
            {
                s.aim = gearbox_search::aim::Aim::fastest();
            }
        }
        (s.by_time, s.seconds) = (true, TRY_SECONDS);
        self.layout.right_tab = 1;
        self.job = "Try from here".into();
        self.info("try", format!("trying from {} s", crate::fmt::fmt_ms(self.race_ms0 + 10 * k as i32)));
        self.start_search();
    }

    pub fn try_setup(&mut self, id: Id) {
        let Some(s) = self.doc.setup(id).cloned() else { return };
        let tick = self.cursor.max(self.race_start_tick());
        let need = tick + s.controls.len() + 100;
        self.edit("try setup", None, |d| {
            d.remember("trying a setup");
            d.len = d.len.max(need);
            d.start = Some(doc::StartState { tick, ..s.start.clone() });
        });
        self.set_ticks(
            "try setup",
            Some("try setup"),
            s.controls().into_iter().enumerate().map(|(i, c)| (tick + i, c)),
        );
        self.info(
            "lab",
            format!("{}: placed at tick {tick}, {} ticks of its inputs", s.name, s.controls.len()),
        );
    }

    pub fn reach_setup(&mut self, id: Id) {
        let Some(s) = self.doc.setup(id).cloned() else { return };
        let name = format!("{} start", s.name);
        let goal = match self.doc.goals.iter().find(|g| g.name == name) {
            Some(g) => g.id,
            None => {
                let goal = self.add_goal(GoalKind::Pose(s.start_pose()));
                self.edit("name goal", None, |d| {
                    if let Some(g) = d.goals.iter_mut().find(|g| g.id == goal) {
                        g.name = name.clone();
                    }
                });
                goal
            }
        };
        self.reach_pose(goal);
    }

    pub fn reach_pose(&mut self, goal: Id) {
        let Some(name) =
            self.doc.goal(goal).filter(|g| matches!(g.kind, GoalKind::Pose(_))).map(|g| g.name.clone())
        else {
            return self.error("lab", "Reach needs a car pose goal");
        };
        if self.doc.start.is_some() {
            self.edit("reach: remove placement", None, |d| d.start = None);
        }
        self.spec.kind = ObjKind::ReachGoals;
        self.spec.goals = vec![goal];
        let end = (self.cursor + REACH_TICKS).min(self.doc.len.max(self.cursor + 1));
        self.spec.window = self.cursor..end;
        self.spec.until = Some(end);
        self.info("lab", format!("reaching {name} from tick {}", self.cursor));
        self.job = format!("Reach {name}");
        self.start_search();
    }

    pub fn join_setup(&mut self, id: Id) {
        let Some(s) = self.doc.setup(id).cloned() else { return };
        let name = format!("{} start", s.name);
        let reached = self
            .doc
            .goals
            .iter()
            .position(|g| g.name == name)
            .and_then(|i| self.reach.get(i))
            .and_then(|r| r.tick);
        let Some(tick) = reached else {
            return self.error("lab", format!("the run doesn't reach {name} yet: Reach it first"));
        };
        let need = tick + s.controls.len() + 100;
        self.edit("join setup", None, |d| {
            d.remember("joining a setup");
            d.len = d.len.max(need);
        });
        self.set_ticks(
            "join setup",
            Some("join setup"),
            s.controls().into_iter().enumerate().map(|(i, c)| (tick + i, c)),
        );
        let goal = self.add_goal(GoalKind::Pose(s.end_pose()));
        let end = format!("{} end", s.name);
        self.edit("name goal", None, |d| {
            if let Some(g) = d.goals.iter_mut().find(|g| g.id == goal) {
                g.name = end.clone();
            }
        });
        self.spec.kind = ObjKind::ReachGoals;
        self.spec.goals = vec![goal];
        self.spec.window = tick..tick + s.controls.len();
        self.spec.until = None;
        self.info("lab", format!("joined {} at tick {tick}; re-tuning its inputs towards {end}", s.name));
        self.job = format!("Join {}", s.name);
        self.start_search();
    }

    pub fn route_stops(&self) -> Vec<RouteStop> {
        let mut first: HashMap<u32, usize> = HashMap::new();
        for (k, f) in self.sim.frames.iter().enumerate() {
            if let Some((w, _)) = f.last_crossing {
                first.entry(w).or_insert(k);
            }
        }
        let mut cps: Vec<(bool, usize, u32, u32)> = self
            .gates
            .iter()
            .filter(|g| g.gate.kind != 1)
            .map(|g| {
                (
                    !first.contains_key(&g.gate.waypoint),
                    first.get(&g.gate.waypoint).copied().unwrap_or(0),
                    g.order.unwrap_or(u32::MAX),
                    g.gate.waypoint,
                )
            })
            .collect();
        cps.sort();
        let mut stops: Vec<RouteStop> = cps
            .into_iter()
            .map(|c| RouteStop {
                name: self.gate_name(c.3),
                at: RouteAt::Gate(c.3),
                tick: (!c.0).then_some(c.1),
            })
            .collect();
        for (i, g) in self.doc.goals.iter().enumerate() {
            if g.is_gate() {
                continue;
            }
            let Some(t) = self.reach.get(i).and_then(|r| r.tick) else { continue };
            let at = stops.iter().position(|s| s.tick.is_none_or(|k| k > t)).unwrap_or(stops.len());
            stops.insert(at, RouteStop { name: g.name.clone(), at: RouteAt::Goal(g.id), tick: Some(t) });
        }
        if let Some(fin) = self
            .gates
            .iter()
            .filter(|g| g.gate.kind == 1)
            .min_by_key(|g| (!first.contains_key(&g.gate.waypoint), g.gate.waypoint))
        {
            let w = fin.gate.waypoint;
            stops.push(RouteStop {
                name: "Finish".into(),
                at: RouteAt::Gate(w),
                tick: first.get(&w).copied(),
            });
        }
        stops
    }

    pub fn find_route(&mut self) {
        let r = self.route.clone();
        let stops = self.route_stops();
        let (from_tick, first) = match r.from {
            RouteFrom::RaceStart => (self.race_start_tick(), 0),
            RouteFrom::Placed => match &self.doc.start {
                Some(s) => (s.tick, 0),
                None => return self.error("route", "no placed car: place one, or route from the race start"),
            },
            RouteFrom::Stop(i) => match stops.get(i).and_then(|s| s.tick) {
                Some(t) => (t, i + 1),
                None => {
                    return self
                        .error("route", "the run doesn't get to that stop: route from an earlier one");
                }
            },
        };
        let Some(last) = stops.iter().position(|s| s.at == r.to) else {
            return self.error("route", "pick where the route goes");
        };
        if last < first {
            return self.error("route", "the route goes to a stop before where it starts");
        }
        if r.from != RouteFrom::Placed && self.doc.start.is_some() {
            self.edit("route: remove placement", None, |d| d.start = None);
        }
        let mut ids = Vec::new();
        for s in &stops[first..=last] {
            let id = match s.at {
                RouteAt::Goal(id) => Some(id),
                RouteAt::Gate(w) => {
                    match self.doc.goals.iter().find(|g| matches!(&g.kind, GoalKind::Gate(x) if x.has(w))) {
                        Some(g) => Some(g.id),
                        None => self.gate_goal(w).map(|gg| self.add_goal(GoalKind::Gate(gg))),
                    }
                }
            };
            if let Some(id) = id.filter(|id| !ids.contains(id)) {
                ids.push(id);
            }
        }
        self.spec.goals = ids;
        self.sel = Sel::Route;
        self.spec.kind = ObjKind::ReachGoals;
        self.spec.window = from_tick..self.doc.len.max(from_tick + 1);
        self.spec.until = None;
        (self.spec.by_time, self.spec.seconds) = (true, r.seconds);
        let to = stops[last].name.clone();
        self.info("route", format!("route to {to}: {} stops", self.spec.goals.len()));
        self.job = format!("Route {}", crate::ui::route::route_title(self));
        self.start_search();
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ExperimentKind {
    Nosebug,
    Bugslide,
    Uberbug,
    Fastest,
}

impl ExperimentKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Nosebug => "hold a nosebug",
            Self::Bugslide => "hold a bugslide",
            Self::Uberbug => "uberbugs",
            Self::Fastest => "fastest at the end",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Experiment {
    pub kind: ExperimentKind,
    pub ticks: usize,
    pub seconds: f32,
}

impl Default for Experiment {
    fn default() -> Self {
        Self { kind: ExperimentKind::Nosebug, ticks: 150, seconds: 30.0 }
    }
}

impl Editor {
    pub fn search_from_car(&mut self) {
        let Some(start) = self.doc.start.clone() else { return };
        let len = self.spec.window.len().max(10);
        let end = start.tick + len;
        if self.doc.len < end + 100 {
            self.edit("search from the car: extend run", None, |d| d.len = end + 100);
        }
        self.spec.window = start.tick..end;
        if self.spec.kind.fixed_time() {
            self.spec.at_tick = Some(end);
        }
        self.layout.right_tab = 1;
        self.info(
            "search",
            format!("range from the car at {} s", crate::fmt::fmt_ms(self.race_ms0 + 10 * start.tick as i32)),
        );
    }

    pub fn run_experiment(&mut self) {
        let Some(start) = self.doc.start.clone() else {
            return self.error("lab", "place the car first: experiments start from a placed car");
        };
        let x = self.experiment.clone();
        let end = start.tick + x.ticks.max(10);
        let need = end + 100;
        if self.doc.len < need {
            self.edit("experiment: extend run", None, |d| d.len = need);
        }
        self.spec.kind = ObjKind::Moment;
        self.spec.aim = match x.kind {
            ExperimentKind::Fastest => gearbox_search::aim::Aim::fastest(),
            ExperimentKind::Bugslide => {
                gearbox_search::aim::Aim::longest_trick(gearbox_search::tricks::TrickKind::Bugslide)
            }
            ExperimentKind::Uberbug => {
                gearbox_search::aim::Aim::longest_trick(gearbox_search::tricks::TrickKind::Uberbug)
            }
            ExperimentKind::Nosebug => {
                gearbox_search::aim::Aim::longest_trick(gearbox_search::tricks::TrickKind::Nosebug)
            }
        };
        self.sel = Sel::Start;
        self.spec.window = start.tick..end;
        self.spec.until = Some(end);
        self.spec.at_tick = Some(end);
        (self.spec.by_time, self.spec.seconds) = (true, x.seconds);
        self.info(
            "lab",
            format!("solve: {} for {:.2} s after the car", x.kind.label(), x.ticks as f32 / 100.0),
        );
        self.job = format!(
            "Solve {}",
            match x.kind {
                ExperimentKind::Nosebug => "nosebug",
                ExperimentKind::Bugslide => "bugslide",
                ExperimentKind::Uberbug => "uberbugs",
                ExperimentKind::Fastest => "top speed",
            }
        );
        self.start_search();
    }
}

#[derive(Clone, Debug)]
pub struct RouteStop {
    pub name: String,
    pub at: RouteAt,
    pub tick: Option<usize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum RouteAt {
    Gate(u32),
    Goal(Id),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum RouteFrom {
    RaceStart,
    Placed,
    Stop(usize),
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RouteSpec {
    pub from: RouteFrom,
    pub to: RouteAt,
    pub seconds: f32,
}

impl Default for RouteSpec {
    fn default() -> Self {
        Self { from: RouteFrom::RaceStart, to: RouteAt::Gate(u32::MAX), seconds: 120.0 }
    }
}

pub fn pose_miss(spec: &doc::PoseSpec, f: &crate::engine::frame::Frame) -> (f32, f32, f32) {
    let d = glam::Vec3::from(f.position()).distance(glam::Vec3::from(spec.pos));
    let angle = doc::frame_rot(f).angle_between(doc::ypr_rot(spec.ypr)).to_degrees();
    let speed = (glam::Vec3::from(f.velocity()).length() - glam::Vec3::from(spec.velocity).length()) * 3.6;
    (d, angle, speed)
}

impl Editor {
    pub fn setup_readout(&self, name: &str) -> Option<String> {
        let i = self.doc.goals.iter().position(|g| g.name == name)?;
        let GoalKind::Pose(spec) = &self.doc.goals[i].kind else { return None };
        let r = self.reach.get(i)?;
        let (tick, word) = match (r.tick, r.closest) {
            (Some(t), _) => (t, "reached at"),
            (None, Some((t, _))) => (t, "closest at"),
            _ => return Some("not simulated yet".into()),
        };
        let f = self.sim.frames.get(tick)?;
        let (d, a, v) = pose_miss(spec, f);
        Some(format!(
            "{word} {} s \u{00b7} {d:.2} m, {a:.1}\u{00b0}, {v:+.1} km/h",
            crate::fmt::fmt_ms(self.race_ms0 + 10 * tick as i32)
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrickSpan {
    pub kind: gearbox_search::tricks::TrickKind,
    pub from: usize,
    pub to: usize,
}

pub fn trick_spans(frames: &[crate::engine::frame::Frame]) -> Vec<TrickSpan> {
    use gearbox_search::tricks::{Detector, Observation, TrickKind};
    let mut d = Detector::default();
    let mut out: Vec<TrickSpan> = Vec::new();
    for (k, f) in frames.iter().enumerate() {
        let v = glam::Vec3::from(f.velocity());
        let fwd = f.axis(2);
        let heading = f64::from(fwd[0]).atan2(f64::from(fwd[2]));
        let travel = f64::from(v.x).atan2(f64::from(v.z));
        let slip = (travel - heading).sin().atan2((travel - heading).cos()).abs();
        let before = d.events;
        d.observe(Observation {
            speed: f64::from(v.length()),
            slip,
            pitch: fwd[1].clamp(-1.0, 1.0).asin(),
            wheels: f.wheels.map(|w| w.contact),
            body_contact: false,
            wall_contact: false,
        });
        let e = d.events;
        let kind = if e.nosebug_ticks > before.nosebug_ticks {
            TrickKind::Nosebug
        } else if e.bugslide_ticks > before.bugslide_ticks {
            TrickKind::Bugslide
        } else if e.uberbug_ticks > before.uberbug_ticks {
            TrickKind::Uberbug
        } else {
            continue;
        };
        match out.last_mut() {
            Some(s) if s.kind == kind && k <= s.to + 3 => s.to = k + 1,
            _ => out.push(TrickSpan { kind, from: k, to: k + 1 }),
        }
    }
    out
}

pub fn trick_label(k: gearbox_search::tricks::TrickKind) -> &'static str {
    match k {
        gearbox_search::tricks::TrickKind::Nosebug => "nosebug",
        gearbox_search::tricks::TrickKind::Bugslide => "bugslide",
        gearbox_search::tricks::TrickKind::Uberbug => "uberbug",
        gearbox_search::tricks::TrickKind::Any => "trick",
    }
}

pub fn trick_color(k: gearbox_search::tricks::TrickKind) -> u32 {
    match k {
        gearbox_search::tricks::TrickKind::Nosebug => crate::theme::WARN,
        gearbox_search::tricks::TrickKind::Bugslide => crate::theme::GHOST,
        _ => crate::theme::OK,
    }
}
