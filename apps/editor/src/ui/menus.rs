use std::rc::Rc;

use glam::{Vec2, Vec3};
use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, Context, FontWeight, InteractiveElement, IntoElement, ParentElement,
    StatefulInteractiveElement, Styled, deferred, div, px,
};

use crate::app::*;
use crate::camera;
use crate::doc::GoalKind;
use crate::fmt::fmt_ms;
use crate::lab::{ExperimentKind, RouteAt, RouteFrom};
use crate::picker::Purpose;
use crate::theme::{self, c};
use crate::ui::widgets::*;

pub type Act = Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>;

#[derive(Clone)]
pub struct Item {
    pub label: String,
    pub note: String,
    pub act: Act,
}

fn item(
    label: impl Into<String>,
    note: impl Into<String>,
    act: impl Fn(&mut Editor, &mut Context<Editor>) + 'static,
) -> Item {
    Item { label: label.into(), note: note.into(), act: Rc::new(act) }
}

#[derive(Clone)]
pub enum Row {
    Title(String),
    Item(Item),
    Sep,
}

#[derive(Clone, Copy, Default)]
pub struct Palette {
    pub sel: usize,
    pub step: Option<Step>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    Solve,
    RouteTo,
    RouteFrom,
    Reach,
    SearchTo,
    Select,
    Add,
    View,
    File,
}

impl Step {
    const ALL: [Step; 9] = [
        Step::Solve,
        Step::RouteTo,
        Step::RouteFrom,
        Step::Reach,
        Step::SearchTo,
        Step::Select,
        Step::Add,
        Step::View,
        Step::File,
    ];
    fn label(self) -> &'static str {
        match self {
            Self::Solve => "Solve",
            Self::RouteTo => "Route to",
            Self::RouteFrom => "Route from",
            Self::Reach => "Reach",
            Self::SearchTo => "Search to",
            Self::Select => "Select",
            Self::Add => "Add",
            Self::View => "View",
            Self::File => "File",
        }
    }
    fn placeholder(self) -> &'static str {
        match self {
            Self::Solve => "What to find from the car\u{2026}",
            Self::RouteTo | Self::RouteFrom => "Which stop\u{2026}",
            Self::Reach => "Which setup\u{2026}",
            Self::SearchTo => "Which goal\u{2026}",
            Self::Select => "Which object\u{2026}",
            Self::Add => "Which goal\u{2026}",
            Self::View => "Which view\u{2026}",
            Self::File => "Which file action\u{2026}",
        }
    }
}

#[derive(Clone)]
pub struct Entry {
    icon: &'static str,
    label: String,
    hint: String,
    key: Option<String>,
    crumb: Option<&'static str>,
    step: Option<Step>,
    act: Option<Act>,
}

impl Entry {
    fn run(
        icon: &'static str,
        label: impl Into<String>,
        hint: impl Into<String>,
        act: impl Fn(&mut Editor, &mut Context<Editor>) + 'static,
    ) -> Self {
        Self {
            icon,
            label: label.into(),
            hint: hint.into(),
            key: None,
            crumb: None,
            step: None,
            act: Some(Rc::new(act)),
        }
    }
    fn open(icon: &'static str, label: &str, hint: &str, step: Step) -> Self {
        Self {
            icon,
            label: label.into(),
            hint: hint.into(),
            key: None,
            crumb: None,
            step: Some(step),
            act: None,
        }
    }
    fn key(mut self, k: &str) -> Self {
        self.key = Some(k.into());
        self
    }
}

fn capital(s: &str) -> String {
    crate::ui::route::capital(s)
}

const PALETTE: &str = "palette";

const SOLVES: [(ExperimentKind, &str, &str); 4] = [
    (ExperimentKind::Nosebug, "nosebug", "longest hold"),
    (ExperimentKind::Bugslide, "bugslide", "longest hold"),
    (ExperimentKind::Uberbug, "uberbug", "most"),
    (ExperimentKind::Fastest, "top speed", "at the end"),
];

impl Editor {
    fn solve_from_car(&mut self, k: ExperimentKind) {
        self.experiment.kind = k;
        self.run_experiment();
    }

    pub fn part_menu(
        &mut self,
        id: crate::doc::Id,
        j: usize,
        at: gpui::Point<gpui::Pixels>,
        cx: &mut Context<Self>,
    ) {
        let Some(GoalKind::Gate(g)) = self.doc.goal(id).map(|g| g.kind.clone()) else { return };
        let Some(part) = g.parts.get(j) else { return };
        self.select(Sel::Goal(id));
        let rows = vec![
            Row::Title(part.name.clone()),
            Row::Item(item("Take out", "", move |e, _| e.take_out(id, j))),
            Row::Item(item("Unmerge all", "", move |e, _| e.unmerge(id))),
        ];
        self.show_rows(at, rows);
        cx.stop_propagation();
        cx.notify();
    }

    pub fn row_menu(&mut self, sel: Sel, at: gpui::Point<gpui::Pixels>, cx: &mut Context<Self>) {
        if !self.multi.contains(&sel) {
            self.select(sel);
        }
        let rows = self.context_rows(sel, None);
        if rows.len() > 1 {
            self.show_rows(at, rows);
        }
        cx.stop_propagation();
        cx.notify();
    }

    pub fn context_rows(&self, sel: Sel, at: Option<Vec2>) -> Vec<Row> {
        let mut v = Vec::new();
        let ms = |k: usize| fmt_ms(self.race_ms0 + 10 * k as i32);
        match sel {
            Sel::Goal(_) | Sel::Gate(_) if self.multi.len() > 1 => {
                v.push(Row::Title(format!("{} selected", self.multi.len())));
                v.push(Row::Item(item("Merge", "\u{2318}J", |e, _| e.merge_selected())));
                if self.multi.iter().any(|s| matches!(s, Sel::Goal(_))) {
                    v.push(Row::Sep);
                    v.push(Row::Item(item("Delete goals", "\u{232b}", |e, _| e.delete_selected())));
                }
            }
            Sel::Start => {
                let Some(s) = &self.doc.start else { return v };
                v.push(Row::Title(format!(
                    "Car \u{00b7} {} s \u{00b7} {:.0} km/h",
                    ms(s.tick),
                    Vec3::from(s.velocity).length() * 3.6
                )));
                for (k, name, note) in SOLVES {
                    v.push(Row::Item(item(format!("Solve {name} from here"), note, move |e, _| {
                        e.solve_from_car(k)
                    })));
                }
                v.push(Row::Sep);
                v.push(Row::Item(item("Save as setup", "", |e, _| e.save_setup_here())));
                v.push(Row::Item(item("Route from here", "", |e, _| {
                    e.route.from = RouteFrom::Placed;
                    e.select(Sel::Route);
                })));
                v.push(Row::Sep);
                v.push(Row::Item(item("Play from here", "\u{2318}P", |e, _| e.simulate_from_start())));
                v.push(Row::Item(item("Focus", "F", |e, _| e.focus_selection())));
                v.push(Row::Sep);
                v.push(Row::Item(item("Delete", "\u{232b}", |e, _| e.delete_selected())));
            }
            Sel::Setup(id) => {
                let Some(st) = self.doc.setup(id) else { return v };
                v.push(Row::Title(format!("{} \u{00b7} {:.2} s", st.name, st.controls.len() as f32 / 100.0)));
                v.push(Row::Item(item(
                    format!("Reach from the playhead ({} s)", ms(self.cursor)),
                    "",
                    move |e, _| e.reach_setup(id),
                )));
                v.push(Row::Item(item("Join where the run reaches it", "", move |e, _| e.join_setup(id))));
                if self.setup_clips().iter().any(|c| c.0 == id) {
                    v.push(Row::Item(item("Try its entry", "", move |e, _| e.try_setup_entry(id))));
                }
                v.push(Row::Item(item("Place at the playhead", "", move |e, _| e.try_setup(id))));
                v.push(Row::Item(item("Count its tricks in the run", "", move |e, _| {
                    e.check_setup_tricks(id)
                })));
                v.push(Row::Sep);
                v.push(Row::Item(item("Focus", "F", |e, _| e.focus_selection())));
                v.push(Row::Item(item("Delete", "\u{232b}", |e, _| e.delete_selected())));
            }
            Sel::Gate(w) => v.extend(self.stop_rows(RouteAt::Gate(w), None)),
            Sel::Route => {
                v.push(Row::Title(format!("Route {}", crate::ui::route::route_title(self))));
                v.push(Row::Item(item("Find the route", "", |e, _| e.find_route())));
                v.push(Row::Item(item("Frame it", "", |e, _| e.frame_route())));
                v.push(Row::Item(item("From the race start", "", |e, _| {
                    e.route.from = RouteFrom::RaceStart
                })));
                if self.doc.start.is_some() {
                    v.push(Row::Item(item("From the car", "", |e, _| e.route.from = RouteFrom::Placed)));
                }
            }
            Sel::Goal(id) => match self.doc.goal(id).map(|g| g.kind.clone()) {
                Some(GoalKind::Gate(g)) => {
                    v.extend(self.stop_rows(RouteAt::Gate(g.waypoint), Some(id)));
                    if g.parts.len() > 1 {
                        v.push(Row::Sep);
                        v.push(Row::Item(item("Unmerge", "", move |e, _| e.unmerge(id))));
                    }
                }
                Some(GoalKind::Pose(_)) => {
                    v.push(Row::Title(self.doc.goal(id).map(|g| g.name.clone()).unwrap_or_default()));
                    v.push(Row::Item(item(
                        format!("Reach from the playhead ({} s)", ms(self.cursor)),
                        "",
                        move |e, _| e.reach_pose(id),
                    )));
                    v.push(Row::Item(item("Route to here", "", move |e, _| {
                        e.route.to = RouteAt::Goal(id);
                        e.select(Sel::Route);
                    })));
                    v.push(Row::Sep);
                    v.push(Row::Item(item("Focus", "F", |e, _| e.focus_selection())));
                    v.push(Row::Item(item("Delete", "\u{232b}", |e, _| e.delete_selected())));
                }
                Some(_) => {
                    v.push(Row::Title(self.doc.goal(id).map(|g| g.name.clone()).unwrap_or_default()));
                    v.push(Row::Item(item("Search to here from the playhead", "", move |e, _| {
                        e.search_toward(id)
                    })));
                    v.push(Row::Item(item("Route to here", "", move |e, _| {
                        e.route.to = RouteAt::Goal(id);
                        e.select(Sel::Route);
                    })));
                    v.push(Row::Sep);
                    v.push(Row::Item(item("Focus", "F", |e, _| e.focus_selection())));
                    v.push(Row::Item(item("Duplicate", "\u{2318}D", |e, _| e.duplicate_selected())));
                    v.push(Row::Item(item("Delete", "\u{232b}", |e, _| e.delete_selected())));
                }
                None => {}
            },
            _ => {
                if let Some(k) = at.and_then(|m| self.trail_under(m)) {
                    v.push(Row::Title(format!(
                        "Run at {} s \u{00b7} {:.0} km/h",
                        ms(k),
                        self.sim.frames[k].speed_kmh()
                    )));
                    v.push(Row::Item(item("Put the playhead here", "", move |e, _| {
                        e.playing = false;
                        e.set_cursor(k);
                    })));
                    v.push(Row::Item(item("Lab car from here", "", move |e, _| {
                        e.set_cursor(k);
                        if let Some(s) = e.start_from_cursor() {
                            e.place_start(s);
                        }
                    })));
                    v.push(Row::Item(item("Setup from here (1.5 s)", "", move |e, _| {
                        e.set_cursor(k);
                        e.save_setup_from_run(150);
                    })));
                    v.push(Row::Item(item("Car pose goal here", "", move |e, _| {
                        e.set_cursor(k);
                        e.pose_goal_from_cursor();
                    })));
                    v.push(Row::Sep);
                }
                if let Some(m) = at {
                    let eye = self.camera.eye();
                    v.push(Row::Item(item("Place a car here", "C", move |e, _| {
                        e.pick_at(m, Purpose::Car { eye, commit: true })
                    })));
                    v.push(Row::Item(item("Point goal here", "2", move |e, _| {
                        e.pick_at(m, Purpose::Place { what: crate::picker::PlaceWhat::Point, eye })
                    })));
                    v.push(Row::Item(item("Car pose goal here", "4", move |e, _| {
                        e.pick_at(m, Purpose::Place { what: crate::picker::PlaceWhat::Pose, eye })
                    })));
                    v.push(Row::Sep);
                }
                v.push(Row::Item(item("Frame the track", "Home", |e, _| e.frame_all())));
                v.push(Row::Item(item("Search commands\u{2026}", "\u{2318}K", |e, _| e.open_palette())));
            }
        }
        v
    }

    pub fn tick_rows(&self, k: usize) -> Vec<Row> {
        let mut v = vec![Row::Title(format!("Run at {} s", fmt_ms(self.race_ms0 + 10 * k as i32)))];
        v.push(Row::Item(item("Try from here", "", move |e, _| e.try_from(k))));
        v.push(Row::Sep);
        v.push(Row::Item(item("Lab car from here", "", move |e, _| {
            e.set_cursor(k);
            if let Some(s) = e.start_from_cursor() {
                e.place_start(s);
            }
        })));
        v.push(Row::Item(item("Setup from here (1.5 s)", "", move |e, _| {
            e.set_cursor(k);
            e.save_setup_from_run(150);
        })));
        v.push(Row::Item(item("Car pose goal here", "", move |e, _| {
            e.set_cursor(k);
            e.pose_goal_from_cursor();
        })));
        v.push(Row::Sep);
        v.push(Row::Item(item("Search window starts here", "[", move |e, _| {
            let end = e.spec.window.end.max(k + 1);
            e.spec.window = k..end;
        })));
        v.push(Row::Item(item("Search window ends here", "]", move |e, _| {
            let start = e.spec.window.start.min(k.saturating_sub(1));
            e.spec.window = start..k;
        })));
        v
    }

    fn stop_rows(&self, at: RouteAt, goal: Option<crate::doc::Id>) -> Vec<Row> {
        let RouteAt::Gate(w) = at else { return Vec::new() };
        let stops = self.route_stops();
        let i = stops.iter().position(|s| s.at == at);
        let when = i
            .and_then(|i| stops[i].tick)
            .map_or(String::new(), |k| format!(" \u{00b7} {} s", fmt_ms(self.race_ms0 + 10 * k as i32)));
        let mut v = vec![Row::Title(format!("{}{when}", self.gate_name(w)))];
        if let Some(i) = i.filter(|i| stops[*i].tick.is_some()) {
            v.push(Row::Item(item("Route from here", "", move |e, _| {
                e.route.from = RouteFrom::Stop(i);
                e.select(Sel::Route);
            })));
        }
        v.push(Row::Item(item("Route to here", "", move |e, _| {
            e.route.to = at;
            e.select(Sel::Route);
        })));
        match goal {
            Some(id) => v.push(Row::Item(item("Search to here from the playhead", "", move |e, _| {
                e.search_toward(id)
            }))),
            None => v.push(Row::Item(item("Add as goal", "", move |e, _| {
                if let Some(g) = e.gate_goal(w) {
                    e.add_goal(GoalKind::Gate(g));
                }
            }))),
        }
        v.push(Row::Sep);
        v.push(Row::Item(item("Focus", "F", |e, _| e.focus_selection())));
        v
    }

    pub fn trail_under(&self, m: Vec2) -> Option<usize> {
        let size = self.vsize();
        let from = self.start_tick().unwrap_or(0);
        self.sim
            .frames
            .iter()
            .enumerate()
            .skip(from)
            .filter_map(|(k, f)| {
                camera::project(&self.vp, size, Vec3::from(f.position())).map(|q| (k, q.distance(m)))
            })
            .filter(|x| x.1 < 9.0)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
    }

    pub fn pick_at(&mut self, m: Vec2, p: Purpose) {
        let (o, d) = camera::ray(&self.vp, self.vsize(), m);
        if let Some(pk) = &self.picker {
            pk.request(o, d, p);
        }
    }

    pub fn open_context_menu(&mut self, m: Vec2) {
        let under = self.object_under_pub(m);
        if let Some(s) = under {
            self.select(s);
        }
        let rows = self.context_rows(under.unwrap_or(Sel::None), Some(m));
        self.show_rows(
            gpui::point(
                px(m.x + f32::from(self.viewport.origin.x)),
                px(m.y + f32::from(self.viewport.origin.y)),
            ),
            rows,
        );
    }

    pub fn show_rows(&mut self, at: gpui::Point<gpui::Pixels>, rows: Vec<Row>) {
        let items = rows
            .into_iter()
            .map(|r| match r {
                Row::Title(t) => {
                    (format!("#{t}"), None, Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}) as Act)
                }
                Row::Sep => {
                    ("-".to_string(), None, Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}) as Act)
                }
                Row::Item(i) => (
                    if i.note.is_empty() { i.label } else { format!("{}\t{}", i.label, i.note) },
                    None,
                    i.act,
                ),
            })
            .collect();
        self.popup = Some(Popup { at, items });
    }

    pub fn open_palette(&mut self) {
        self.popup = None;
        self.palette = Some(Palette::default());
        self.editing = Some(FieldEdit { id: PALETTE.into(), text: String::new(), fresh: false });
    }

    pub fn close_palette(&mut self) {
        self.palette = None;
        if self.editing.as_ref().is_some_and(|e| e.id == PALETTE) {
            self.editing = None;
        }
    }

    fn palette_query(&self) -> String {
        self.editing.as_ref().filter(|e| e.id == PALETTE).map(|e| e.text.clone()).unwrap_or_default()
    }

    fn palette_root(&self) -> Vec<Vec<Entry>> {
        let mut groups: Vec<Vec<Entry>> = Vec::new();
        let ms = |k: usize| fmt_ms(self.race_ms0 + 10 * k as i32);
        let mut sel: Vec<Entry> = Vec::new();
        match self.sel {
            Sel::Start if self.doc.start.is_some() => {
                sel.push(Entry::open(
                    "\u{25c6}",
                    "Solve from the car",
                    "nosebug, bugslide, speed",
                    Step::Solve,
                ));
                sel.push(Entry::run("\u{002b}", "Save the car as a setup", "", |e, _| e.save_setup_here()));
                sel.push(Entry::run("\u{21a6}", "Route from the car", "", |e, _| {
                    e.route.from = RouteFrom::Placed;
                    e.select(Sel::Route);
                }));
            }
            Sel::Setup(id) => {
                let n = self.doc.setup(id).map_or(String::new(), |s| s.name.clone());
                sel.push(Entry::run(
                    "\u{25c6}",
                    format!("Reach {n} from the playhead"),
                    format!("{} s", ms(self.cursor)),
                    move |e, _| e.reach_setup(id),
                ));
                sel.push(Entry::run("\u{25c6}", format!("Join {n}"), "", move |e, _| e.join_setup(id)));
            }
            Sel::Route => sel.push(Entry::run(
                "\u{21a6}",
                "Find the route",
                crate::ui::route::route_title(self),
                |e, _| e.find_route(),
            )),
            Sel::Goal(id) => {
                let n = self.doc.goal(id).map_or(String::new(), |g| g.name.clone());
                sel.push(Entry::run(
                    "\u{25ce}",
                    format!("Search to {n}"),
                    "from the playhead",
                    move |e, _| e.search_toward(id),
                ));
            }
            _ => {}
        }
        if !sel.is_empty() {
            groups.push(sel);
        }
        let mut doing = vec![
            Entry::open("\u{21a6}", "Route to", &crate::ui::route::route_title(self), Step::RouteTo),
            Entry::open("\u{21a6}", "Route from", "", Step::RouteFrom),
        ];
        if self.doc.start.is_some() && self.sel != Sel::Start {
            doing.push(Entry::open(
                "\u{25c6}",
                "Solve from the car",
                "nosebug, bugslide, speed",
                Step::Solve,
            ));
        }
        if !self.doc.setups.is_empty() {
            doing.push(Entry::open(
                "\u{25c6}",
                "Reach a setup",
                &format!("{} setups", self.doc.setups.len()),
                Step::Reach,
            ));
        }
        if !self.doc.goals.is_empty() {
            doing.push(Entry::open(
                "\u{25ce}",
                "Search to a goal",
                &format!("{} goals", self.doc.goals.len()),
                Step::SearchTo,
            ));
        }
        doing.push(Entry::run("\u{25b6}", "Place a car", "", |e, _| e.set_tool(Tool::PlaceCar)).key("C"));
        doing.push(Entry::run("\u{25b6}", "Lab car from the playhead", "", |e, _| {
            if let Some(s) = e.start_from_cursor() {
                e.place_start(s);
            }
        }));
        groups.push(doing);
        groups.push(vec![
            Entry::open("\u{2316}", "Select", "goals, checkpoints, setups", Step::Select),
            Entry::open("\u{25a3}", "Add a goal", "gate, point, box, pose, line", Step::Add),
            Entry::open("\u{25a2}", "View", "frame, physics, follow", Step::View),
            Entry::open("\u{2630}", "File", "projects, replays, ghosts", Step::File),
        ]);
        groups
    }

    fn palette_step(&self, step: Step) -> Vec<Entry> {
        let ms = |k: usize| fmt_ms(self.race_ms0 + 10 * k as i32);
        let mut v = Vec::new();
        match step {
            Step::Solve => {
                for (k, name, note) in SOLVES {
                    v.push(Entry::run("\u{25c6}", capital(name), note, move |e, _| e.solve_from_car(k)));
                }
            }
            Step::RouteTo | Step::RouteFrom => {
                if step == Step::RouteFrom {
                    v.push(Entry::run("\u{21a6}", "Race start", "", |e, _| {
                        e.route.from = RouteFrom::RaceStart;
                        e.select(Sel::Route);
                        e.frame_route();
                    }));
                    if self.doc.start.is_some() {
                        v.push(Entry::run("\u{21a6}", "The car", "", |e, _| {
                            e.route.from = RouteFrom::Placed;
                            e.select(Sel::Route);
                            e.frame_route();
                        }));
                    }
                }
                for (i, s) in self.route_stops().into_iter().enumerate() {
                    let t = s.tick.map_or("not reached".to_string(), |k| format!("{} s", ms(k)));
                    let at = s.at;
                    if step == Step::RouteTo {
                        v.push(Entry::run("\u{21a6}", s.name, t, move |e, _| {
                            e.route.to = at;
                            e.select(Sel::Route);
                            e.frame_route();
                        }));
                    } else if s.tick.is_some() {
                        v.push(Entry::run("\u{21a6}", s.name, t, move |e, _| {
                            e.route.from = RouteFrom::Stop(i);
                            e.select(Sel::Route);
                            e.frame_route();
                        }));
                    }
                }
            }
            Step::Reach => {
                for st in &self.doc.setups {
                    let id = st.id;
                    v.push(Entry::run(
                        "\u{25c6}",
                        st.name.clone(),
                        format!("{:.2} s", st.controls.len() as f32 / 100.0),
                        move |e, _| e.reach_setup(id),
                    ));
                }
            }
            Step::SearchTo => {
                for g in &self.doc.goals {
                    let id = g.id;
                    v.push(Entry::run("\u{25ce}", g.name.clone(), g.kind_label(), move |e, _| {
                        e.search_toward(id)
                    }));
                }
            }
            Step::Select => {
                let pick = |s: Sel| {
                    move |e: &mut Editor, _: &mut Context<Editor>| {
                        e.select(s);
                        e.focus_selection();
                    }
                };
                if self.doc.start.is_some() {
                    v.push(Entry::run("\u{25b6}", "The car", "lab", pick(Sel::Start)));
                }
                for st in &self.doc.setups {
                    v.push(Entry::run("\u{25c6}", st.name.clone(), "setup", pick(Sel::Setup(st.id))));
                }
                v.push(Entry::run(
                    "\u{21a6}",
                    "The route",
                    crate::ui::route::route_title(self),
                    pick(Sel::Route),
                ));
                for g in &self.doc.goals {
                    v.push(Entry::run("\u{25ce}", g.name.clone(), g.kind_label(), pick(Sel::Goal(g.id))));
                }
                for g in &self.gates {
                    let w = g.gate.waypoint;
                    if !self.doc.goals.iter().any(|x| matches!(&x.kind, GoalKind::Gate(gg) if gg.has(w))) {
                        v.push(Entry::run("\u{25ad}", self.gate_name(w), "checkpoint", pick(Sel::Gate(w))));
                    }
                }
            }
            Step::Add => {
                v.push(
                    Entry::run("\u{25ae}", "Gate goal", "click a checkpoint", |e, _| {
                        e.set_tool(Tool::GateGoal)
                    })
                    .key("1"),
                );
                v.push(
                    Entry::run("\u{25cf}", "Point goal", "click a surface", |e, _| {
                        e.set_tool(Tool::PointGoal)
                    })
                    .key("2"),
                );
                v.push(
                    Entry::run("\u{25a3}", "Box trigger", "click a surface", |e, _| {
                        e.set_tool(Tool::BoxGoal)
                    })
                    .key("3"),
                );
                v.push(
                    Entry::run("\u{25b2}", "Car pose goal", "click a surface", |e, _| {
                        e.set_tool(Tool::PoseGoal)
                    })
                    .key("4"),
                );
                v.push(Entry::run("\u{25b2}", "Car pose goal from the playhead", "", |e, _| {
                    e.pose_goal_from_cursor()
                }));
                v.push(
                    Entry::run("\u{223f}", "Guide line", "click along the way", |e, _| {
                        e.set_tool(Tool::Line)
                    })
                    .key("5"),
                );
            }
            Step::View => {
                v.push(Entry::run("\u{25a2}", "Frame the track", "", |e, _| e.frame_all()).key("Home"));
                v.push(
                    Entry::run("\u{25a2}", "Focus the selection", "", |e, _| e.focus_selection()).key("F"),
                );
                v.push(
                    Entry::run("\u{25a2}", "Follow the car", "", |e, _| {
                        let on = e.camera.mode != crate::camera::Mode::Follow;
                        e.camera.set_follow(on);
                    })
                    .key("\u{21e7}F"),
                );
                v.push(
                    Entry::run("\u{25a2}", "Physics view", "", |e, _| e.show.physics = !e.show.physics)
                        .key("P"),
                );
                v.push(
                    Entry::run("\u{25a2}", "Stadium surroundings", "", |e, _| {
                        e.show.stadium = !e.show.stadium
                    })
                    .key("B"),
                );
                v.push(Entry::run("\u{25a2}", "Tricks along the run", "", |e, _| {
                    e.show.tricks = !e.show.tricks
                }));
                v.push(Entry::run("\u{25a2}", "Setups", "", |e, _| e.show.setups = !e.show.setups));
            }
            Step::File => {
                v.push(
                    Entry::run("\u{2630}", "Open a project", "", |e, _| e.show_projects()).key("\u{2318}O"),
                );
                v.push(
                    Entry::run("\u{2630}", "Import replay inputs", "", |e, cx| e.prompt_replay(false, cx))
                        .key("\u{2318}I"),
                );
                v.push(
                    Entry::run("\u{2630}", "Add a replay as a ghost", "", |e, cx| e.prompt_replay(true, cx))
                        .key("\u{2318}\u{21e7}I"),
                );
                v.push(Entry::run("\u{2630}", "Shortcuts", "", |e, _| e.cheatsheet = true).key("?"));
            }
        }
        v
    }

    fn palette_groups(&self) -> Vec<Vec<Entry>> {
        let q = self.palette_query().to_lowercase();
        let words: Vec<&str> = q.split_whitespace().collect();
        let step = self.palette.and_then(|p| p.step);
        let page: Vec<Vec<Entry>> = match step {
            Some(s) => vec![self.palette_step(s)],
            None => self.palette_root(),
        };
        if words.is_empty() {
            return page;
        }
        let mut all: Vec<Entry> = page.into_iter().flatten().filter(|e| e.step.is_none()).collect();
        if step.is_none() {
            for s in Step::ALL {
                let label = s.label();
                all.extend(self.palette_step(s).into_iter().map(|mut e| {
                    e.crumb = Some(label);
                    e
                }));
            }
        }
        let text = |e: &Entry| format!("{} {} {}", e.crumb.unwrap_or(""), e.label, e.hint).to_lowercase();
        let mut hits: Vec<(usize, Entry)> = all
            .into_iter()
            .filter(|e| words.iter().all(|w| text(e).contains(w)))
            .map(|e| {
                let l = e.label.to_lowercase();
                let rank =
                    if words.iter().all(|w| l.split_whitespace().any(|x| x.starts_with(w))) { 0 } else { 1 };
                (rank, e)
            })
            .collect();
        hits.sort_by_key(|h| h.0);
        vec![hits.into_iter().map(|h| h.1).take(30).collect()]
    }

    pub fn palette_key(&mut self, key: &str, cx: &mut Context<Editor>) -> bool {
        let Some(p) = self.palette else { return false };
        let flat: Vec<Entry> = self.palette_groups().into_iter().flatten().collect();
        let n = flat.len();
        let empty = self.palette_query().is_empty();
        match key {
            "down" => self.palette = Some(Palette { sel: (p.sel + 1).min(n.saturating_sub(1)), ..p }),
            "up" => self.palette = Some(Palette { sel: p.sel.saturating_sub(1), ..p }),
            "enter" | "right" => {
                if key == "right" && flat.get(p.sel).is_none_or(|e| e.step.is_none()) {
                    return false;
                }
                if let Some(e) = flat.get(p.sel).cloned() {
                    self.palette_pick(e, cx);
                }
            }
            "escape" if p.step.is_some() => self.palette_back(),
            "left" | "backspace" if p.step.is_some() && empty => self.palette_back(),
            "escape" => self.close_palette(),
            _ => {
                self.palette = Some(Palette { sel: 0, ..p });
                return false;
            }
        }
        cx.notify();
        true
    }

    fn palette_back(&mut self) {
        self.palette = Some(Palette::default());
        if let Some(e) = &mut self.editing {
            e.text.clear();
        }
    }

    fn palette_pick(&mut self, e: Entry, cx: &mut Context<Editor>) {
        match (e.step, e.act) {
            (Some(s), _) => {
                self.palette = Some(Palette { sel: 0, step: Some(s) });
                if let Some(f) = &mut self.editing {
                    f.text.clear();
                }
            }
            (None, Some(act)) => {
                self.close_palette();
                act(self, cx);
                self.dirty = true;
            }
            _ => {}
        }
    }

    pub fn palette_overlay(&mut self, cx: &mut Context<Self>) -> Option<AnyElement> {
        let p = self.palette?;
        let text = self.palette_query();
        let groups = self.palette_groups();
        let mut rows: Vec<AnyElement> = Vec::new();
        let mut i = 0usize;
        let count: usize = groups.iter().map(|g| g.len()).sum();
        for (gi, g) in groups.into_iter().enumerate() {
            if gi > 0 && !g.is_empty() {
                rows.push(
                    div()
                        .h(px(1.0))
                        .mx(px(theme::SP2))
                        .my(px(theme::SP2))
                        .bg(c(theme::HAIRLINE))
                        .into_any_element(),
                );
            }
            for e in g {
                let on = i == p.sel;
                let pick = e.clone();
                rows.push(
                    div()
                        .id(("pal", i))
                        .relative()
                        .children(track(format!("pal-{i}")))
                        .h(px(34.0))
                        .px(px(theme::SP3))
                        .flex()
                        .items_center()
                        .gap(px(theme::SP3))
                        .rounded(px(theme::R_CTL + 2.0))
                        .cursor_pointer()
                        .when(on, |d| d.bg(c(theme::SELECT)))
                        .hover(|s| s.bg(c(theme::SELECT)))
                        .child(
                            div()
                                .size(px(22.0))
                                .flex_none()
                                .rounded(px(theme::R_CTL + 1.0))
                                .bg(c(theme::WELL))
                                .border_1()
                                .border_color(c(theme::HAIRLINE))
                                .flex()
                                .items_center()
                                .justify_center()
                                .text_size(px(theme::FS_SMALL))
                                .text_color(c(theme::MUTED))
                                .child(e.icon),
                        )
                        .child(
                            div()
                                .flex_1()
                                .min_w_0()
                                .truncate()
                                .flex()
                                .gap(px(theme::SP1))
                                .text_size(px(theme::FS_EMPH))
                                .children(e.crumb.map(|c0| {
                                    div().text_color(c(theme::MUTED)).child(format!("{c0} \u{203a}"))
                                }))
                                .child(e.label.clone()),
                        )
                        .when(!e.hint.is_empty(), |d| {
                            d.child(
                                div()
                                    .flex_none()
                                    .max_w(px(200.0))
                                    .truncate()
                                    .text_size(px(theme::FS_SMALL))
                                    .text_color(c(theme::FAINT))
                                    .child(e.hint.clone()),
                            )
                        })
                        .children(e.key.clone().map(kbd))
                        .when(e.step.is_some(), |d| {
                            d.child(div().text_color(c(theme::FAINT)).child("\u{203a}"))
                        })
                        .on_mouse_down(
                            gpui::MouseButton::Left,
                            cx.listener(move |this, _, _, cx| {
                                this.palette_pick(pick.clone(), cx);
                                cx.stop_propagation();
                                cx.notify();
                            }),
                        )
                        .into_any_element(),
                );
                i += 1;
            }
        }
        let placeholder = match p.step {
            Some(s) => s.placeholder(),
            None => "Search or run a command\u{2026}",
        };
        let foot = if text.is_empty() && p.step.is_none() {
            "\u{2191}\u{2193} move   \u{21b5} run   \u{2192} open   esc close".to_string()
        } else if p.step.is_some() && text.is_empty() {
            "\u{2191}\u{2193} move   \u{21b5} run   \u{2190} back   esc back".to_string()
        } else {
            format!("{count} result{}   \u{21b5} run", if count == 1 { "" } else { "s" })
        };
        Some(
            deferred(
                div()
                    .id("palette")
                    .absolute()
                    .size_full()
                    .flex()
                    .justify_center()
                    .pt(px(90.0))
                    .bg(theme::alpha(theme::BG, 0.45))
                    .occlude()
                    .on_mouse_down(
                        gpui::MouseButton::Left,
                        cx.listener(|this, _, _, cx| {
                            this.close_palette();
                            cx.notify();
                        }),
                    )
                    .child(
                        div()
                            .id("palette-box")
                            .w(px(600.0))
                            .max_h(px(520.0))
                            .flex()
                            .flex_col()
                            .rounded(px(theme::RADIUS + 2.0))
                            .bg(c(theme::RAISED))
                            .border_1()
                            .border_color(c(theme::HAIRLINE))
                            .shadow_lg()
                            .on_mouse_down(gpui::MouseButton::Left, |_, _, cx| cx.stop_propagation())
                            .child(
                                div()
                                    .h(px(52.0))
                                    .flex_none()
                                    .px(px(theme::SP4 + 2.0))
                                    .flex()
                                    .items_center()
                                    .gap(px(theme::SP2))
                                    .border_b_1()
                                    .border_color(c(theme::HAIRLINE))
                                    .text_size(px(theme::FS_TITLE))
                                    .children(p.step.map(|s| {
                                        div()
                                            .px(px(theme::SP2))
                                            .py(px(3.0))
                                            .rounded(px(theme::R_CTL + 1.0))
                                            .bg(c(theme::SELECT))
                                            .text_size(px(theme::FS_BODY))
                                            .child(s.label())
                                    }))
                                    .when(!text.is_empty(), |d| d.child(text.clone()))
                                    .child(div().w(px(1.5)).h(px(18.0)).bg(c(theme::PIN)))
                                    .when(text.is_empty(), |d| {
                                        d.child(div().text_color(c(theme::FAINT)).child(placeholder))
                                    }),
                            )
                            .child(
                                div()
                                    .id("palette-rows")
                                    .flex_1()
                                    .min_h_0()
                                    .overflow_y_scroll()
                                    .p(px(theme::SP2))
                                    .flex()
                                    .flex_col()
                                    .gap(px(2.0))
                                    .children(rows)
                                    .when(count == 0, |d| {
                                        d.child(div().p(px(theme::SP3)).child(muted("Nothing by that name.")))
                                    }),
                            )
                            .child(
                                div()
                                    .h(px(34.0))
                                    .flex_none()
                                    .px(px(theme::SP4 + 2.0))
                                    .flex()
                                    .items_center()
                                    .border_t_1()
                                    .border_color(c(theme::HAIRLINE))
                                    .text_size(px(theme::FS_SMALL))
                                    .text_color(c(theme::FAINT))
                                    .child(foot),
                            ),
                    ),
            )
            .with_priority(3)
            .into_any_element(),
        )
    }

    pub fn job_pill(&mut self, cx: &mut Context<Self>) -> Option<AnyElement> {
        let (running, attempts, best) = {
            let l = self.search.live.lock().unwrap();
            (l.running, l.attempts, l.best.as_ref().map(|b| l.metric(b).text))
        };
        if !running {
            return None;
        }
        Some(
            div()
                .id("job")
                .relative()
                .children(track("job"))
                .h(px(26.0))
                .px(px(theme::SP3))
                .flex()
                .items_center()
                .gap(px(theme::SP2))
                .rounded(px(theme::R_CTL + 1.0))
                .bg(c(theme::RAISED))
                .border_1()
                .border_color(c(theme::HAIRLINE))
                .cursor_pointer()
                .text_size(px(theme::FS_BODY))
                .child(div().size(px(6.0)).rounded_full().bg(c(theme::PIN)))
                .child(div().font_weight(FontWeight::MEDIUM).child(self.job.clone()))
                .child(div().text_color(c(theme::MUTED)).child(format!("{attempts} tries")))
                .children(best.map(|b| div().text_color(c(theme::OK)).child(b)))
                .child(
                    div()
                        .id("job-stop")
                        .ml(px(theme::SP1))
                        .px(px(theme::SP1))
                        .rounded(px(3.0))
                        .text_color(c(theme::MUTED))
                        .hover(|s| s.text_color(c(theme::ERROR)).bg(c(theme::SELECT)))
                        .child("\u{25a0}")
                        .tooltip(tip("Stop  \u{2318}.", None))
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.search.stop();
                            cx.stop_propagation();
                            cx.notify();
                        })),
                )
                .tooltip(tip(
                    "Running",
                    Some("Its tries draw in the view. Click for its settings and results."),
                ))
                .on_click(cx.listener(|this, _, _, cx| {
                    this.layout.right_tab = 1;
                    cx.notify();
                }))
                .into_any_element(),
        )
    }

    pub fn route_from_name(&self) -> String {
        match self.route.from {
            RouteFrom::RaceStart => "Start".into(),
            RouteFrom::Placed => "Car".into(),
            RouteFrom::Stop(i) => self.route_stops().get(i).map_or("?".into(), |s| s.name.clone()),
        }
    }

    pub fn route_to_name(&self) -> String {
        self.route_stops().iter().find(|s| s.at == self.route.to).map_or("Finish".into(), |s| s.name.clone())
    }
}
