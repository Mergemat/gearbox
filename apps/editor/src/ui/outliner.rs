use std::rc::Rc;

use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, Context, FontWeight, InteractiveElement, IntoElement, ParentElement, Render, SharedString,
    StatefulInteractiveElement, Styled, Window, div, px,
};

use crate::app::*;
use crate::doc::GoalKind;
use crate::fmt::{fmt_delta, fmt_ms};
use crate::theme::{self, c};
use crate::ui::widgets::{muted, tip};

#[derive(Clone)]
pub(crate) struct GoalDrag(pub u32, pub SharedString);

pub(crate) struct DragGhost(pub SharedString);

impl Render for DragGhost {
    fn render(&mut self, _: &mut Window, _: &mut Context<Self>) -> impl IntoElement {
        div()
            .px(px(theme::SP2))
            .py(px(3.0))
            .rounded(px(4.0))
            .bg(c(theme::SELECT))
            .border_1()
            .border_color(c(theme::PIN))
            .text_size(px(theme::FS_BODY))
            .text_color(c(theme::TEXT))
            .child(self.0.clone())
    }
}

fn swatch(color: u32, shape: &'static str) -> gpui::Div {
    div()
        .w(px(14.0))
        .flex_none()
        .flex()
        .justify_center()
        .text_size(px(theme::FS_SMALL))
        .text_color(c(color))
        .child(shape)
}

fn type_icon(path: &'static str, color: u32) -> gpui::Div {
    div().w(px(14.0)).flex_none().flex().justify_center().child(crate::ui::widgets::icon(path, color, 14.0))
}

fn gate_icon(kind: u32) -> &'static str {
    if kind == 1 { "icons/finish.svg" } else { "icons/gate.svg" }
}

type Act = Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>;

pub(crate) fn add_items() -> Vec<(String, Option<String>, Act)> {
    let tool = |t: Tool, label: &str| {
        (
            format!("@{}|{label}\t{}", t.icon(), t.key()),
            None,
            Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| e.set_tool(t)) as Act,
        )
    };
    vec![
        ("#Add".into(), None, Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}) as Act),
        tool(Tool::GateGoal, "Gate goal"),
        tool(Tool::PointGoal, "Point goal"),
        tool(Tool::BoxGoal, "Box goal"),
        tool(Tool::PoseGoal, "Pose goal"),
        (
            "@icons/pose.svg|Pose goal from the car at the playhead".into(),
            None,
            Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.pose_goal_from_cursor()) as Act,
        ),
        ("-".into(), None, Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}) as Act),
        tool(Tool::PlaceCar, "Car"),
        tool(Tool::Line, "Guide line"),
        (
            "@icons/ghost.svg|Ghost from a replay\u{2026}\t\u{2318}\u{21e7}I".into(),
            None,
            Rc::new(|e: &mut Editor, cx: &mut Context<Editor>| e.prompt_replay(true, cx)) as Act,
        ),
    ]
}

impl Editor {
    fn group(
        &mut self,
        key: &str,
        title: String,
        count: Option<usize>,
        actions: Vec<AnyElement>,
        cx: &mut Context<Self>,
    ) -> (bool, AnyElement) {
        let open = !self.collapsed.contains(key);
        let k = key.to_string();
        let el = div()
            .id(gpui::ElementId::Name(format!("grp-{key}").into()))
            .relative()
            .child(crate::ui::widgets::track(format!("grp-{key}")))
            .h(px(theme::H_ROW))
            .px(px(theme::SP2))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .cursor_pointer()
            .border_t_1()
            .border_color(c(theme::HAIRLINE))
            .child(
                div().w(px(10.0)).text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(if open {
                    "\u{25be}"
                } else {
                    "\u{25b8}"
                }),
            )
            .child(
                div()
                    .text_size(px(theme::FS_BODY))
                    .font_weight(FontWeight::SEMIBOLD)
                    .text_color(c(theme::MUTED))
                    .child(title),
            )
            .children(count.map(|n| {
                div()
                    .font_family(theme::MONO)
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(theme::FAINT))
                    .child(n.to_string())
            }))
            .child(div().flex_1())
            .children(actions)
            .on_click(cx.listener(move |this, _, _, cx| {
                if !this.collapsed.remove(&k) {
                    this.collapsed.insert(k.clone());
                }
                cx.notify();
            }))
            .into_any_element();
        (open, el)
    }

    #[allow(clippy::too_many_arguments)]
    fn orow(
        &self,
        id: String,
        selected: bool,
        sw: gpui::Div,
        name: String,
        sub: String,
        value: Option<(String, u32)>,
        delta: Option<(String, u32)>,
    ) -> gpui::Stateful<gpui::Div> {
        div()
            .id(gpui::ElementId::Name(id.clone().into()))
            .relative()
            .child(crate::ui::widgets::track(id))
            .h(px(theme::H_ROW))
            .pl(px(theme::SP4))
            .pr(px(theme::SP2))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .cursor_pointer()
            .when(selected, |d| d.bg(c(theme::SELECT)))
            .when(!selected, |d| d.hover(|s| s.bg(c(theme::RAISED))))
            .child(sw)
            .child(
                div()
                    .flex_1()
                    .min_w_0()
                    .flex()
                    .items_baseline()
                    .gap(px(theme::SP2))
                    .child(div().text_size(px(theme::FS_BODY)).truncate().child(name))
                    .child(
                        div()
                            .text_size(px(theme::FS_SMALL))
                            .text_color(c(theme::FAINT))
                            .truncate()
                            .child(sub),
                    ),
            )
            .children(value.map(|(v, col)| {
                div().font_family(theme::MONO).text_size(px(theme::FS_SMALL)).text_color(c(col)).child(v)
            }))
            .children(delta.map(|(v, col)| {
                div()
                    .w(px(50.0))
                    .flex()
                    .justify_end()
                    .font_family(theme::MONO)
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(col))
                    .child(v)
            }))
    }

    pub fn eye(&self, id: String, on: bool) -> gpui::Stateful<gpui::Div> {
        div()
            .id(gpui::ElementId::Name(id.into()))
            .w(px(18.0))
            .flex()
            .justify_center()
            .child(crate::ui::widgets::icon(
                if on { "icons/eye.svg" } else { "icons/eyeoff.svg" },
                if on { theme::MUTED } else { theme::FAINT },
                13.0,
            ))
            .tooltip(tip(if on { "Hide" } else { "Show" }, None))
    }

    pub fn outliner(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let mut rows: Vec<AnyElement> = Vec::new();
        let comparable = !self.doc.goals.is_empty()
            && (!self.doc.history.is_empty() || !self.doc.runs.is_empty() || !self.ghosts.is_empty());
        let mut goal_actions = Vec::new();
        if comparable {
            let name = self
                .reference_choices()
                .into_iter()
                .find(|c| c.1 == self.reference)
                .map_or_else(|| self.ref_label(self.reference), |c| c.0);
            let label = format!("vs {name} \u{25be}");
            goal_actions.push(
                div()
                    .id("ref-pick")
                    .max_w(px(150.0))
                    .truncate()
                    .px(px(theme::SP1))
                    .rounded(px(3.0))
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(theme::FAINT))
                    .hover(|s| s.text_color(c(theme::TEXT)).bg(c(theme::RAISED)))
                    .child(label)
                    .tooltip(tip(
                        "Compare with",
                        Some(
                            "Goal deltas compare the current run with this run: negative (green) is earlier.",
                        ),
                    ))
                    .on_click(cx.listener(|this, e: &gpui::ClickEvent, _, cx| {
                        let items = this
                            .reference_choices()
                            .into_iter()
                            .map(|(name, r)| {
                                let on = this.reference == r;
                                (
                                    format!("{}{name}", if on { "@icons/check.svg|" } else { "" }),
                                    None,
                                    Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| {
                                        e.reference = r;
                                        e.reevaluate();
                                    })
                                        as Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>,
                                )
                            })
                            .collect();
                        this.popup = Some(Popup { at: e.position(), items });
                        cx.stop_propagation();
                        cx.notify();
                    }))
                    .into_any_element(),
            );
        }
        let (open, g) = self.group("goals", "Goals".into(), None, goal_actions, cx);
        rows.push(g);
        if open {
            if self.doc.goals.is_empty() {
                rows.push(
                    div()
                        .px(px(theme::SP4))
                        .py(px(theme::SP1))
                        .child(muted("None yet: + above, or 1\u{2013}4 in the view."))
                        .into_any_element(),
                );
            }
            let ref_times = self.ref_times.clone();
            let order: Rc<Vec<Sel>> = Rc::new(self.doc.goals.iter().map(|g| Sel::Goal(g.id)).collect());
            for (i, goal) in self.doc.goals.clone().iter().enumerate() {
                let r = self.reach.get(i).copied().unwrap_or_default();
                let (path, col) = match &goal.kind {
                    GoalKind::Gate(g) => (gate_icon(g.kind), theme::PIN),
                    GoalKind::Point { .. } => ("icons/point.svg", theme::PIN),
                    GoalKind::Box { .. } => ("icons/box.svg", theme::PIN),
                    GoalKind::Pose(_) => ("icons/pose.svg", theme::PIN),
                };
                let value = match (r.ms, r.closest) {
                    (Some(ms), _) => Some((fmt_ms(ms), theme::TEXT)),
                    (None, _) if r.out_of_order => Some(("out of order".into(), theme::WARN)),
                    (None, Some((_, d))) if matches!(goal.kind, GoalKind::Pose(_)) => {
                        Some((format!("off \u{00b7} {d:.1}"), theme::FAINT))
                    }
                    (None, Some((_, d))) => Some((format!("miss \u{00b7} {d:.1} m"), theme::FAINT)),
                    _ => Some(("\u{2014}".into(), theme::FAINT)),
                };
                let delta = match (r.ms, ref_times.get(i).copied().flatten()) {
                    (Some(a), Some(b)) => Some((
                        fmt_delta(a - b),
                        if a < b {
                            theme::OK
                        } else if a > b {
                            theme::ERROR
                        } else {
                            theme::MUTED
                        },
                    )),
                    (Some(_), None) if !ref_times.is_empty() => Some(("new".into(), theme::OK)),
                    _ => None,
                };
                let id = goal.id;
                let sel = self.sel == Sel::Goal(id) || self.multi.contains(&Sel::Goal(id));
                let eye = self.eye(format!("goal-eye-{id}"), goal.visible).on_click(cx.listener(
                    move |this, _, _, cx| {
                        this.edit("toggle goal visibility", None, |d| {
                            if let Some(g) = d.goal_mut(id) {
                                g.visible = !g.visible;
                            }
                        });
                        cx.stop_propagation();
                        cx.notify();
                    },
                ));
                let row = self
                    .orow(
                        format!("goal-{id}"),
                        sel,
                        type_icon(path, col),
                        goal.name.clone(),
                        String::new(),
                        value,
                        delta,
                    )
                    .on_mouse_down(
                        gpui::MouseButton::Right,
                        cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                            this.row_menu(Sel::Goal(id), e.position, cx)
                        }),
                    )
                    .child(eye)
                    .on_click(cx.listener({
                        let order = order.clone();
                        move |this, e: &gpui::ClickEvent, _, cx| {
                            let m = e.modifiers();
                            if m.shift {
                                this.select_range(Sel::Goal(id), &order);
                            } else if m.platform {
                                this.toggle(Sel::Goal(id));
                            } else {
                                this.select(Sel::Goal(id));
                                if let Some(k) = this.reach.get(i).and_then(|r| r.tick) {
                                    this.set_cursor(k);
                                }
                            }
                            cx.notify();
                        }
                    }));
                rows.push(row.into_any_element());
                if let GoalKind::Gate(g) = &goal.kind {
                    for (j, part) in g.parts.iter().enumerate() {
                        let row = self
                            .orow(
                                format!("goal-{id}-part-{j}"),
                                false,
                                type_icon(gate_icon(part.gate.kind), theme::FAINT),
                                part.name.clone(),
                                format!("waypoint {}", part.gate.waypoint),
                                None,
                                None,
                            )
                            .pl(px(theme::SP4 + theme::SP5))
                            .on_mouse_down(
                                gpui::MouseButton::Right,
                                cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                                    this.part_menu(id, j, e.position, cx)
                                }),
                            )
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.select(Sel::Goal(id));
                                cx.notify();
                            }));
                        rows.push(row.into_any_element());
                    }
                }
            }
        }
        let cars = self.doc.start.is_some() as usize + self.doc.setups.len();
        let (open, g) = self.group("lab", "Cars".into(), None, vec![], cx);
        if cars > 0 {
            rows.push(g);
        }
        if open && cars > 0 {
            if let Some(s) = self.doc.start.clone() {
                let ms = self.race_ms0 + 10 * s.tick as i32;
                let row = self
                    .orow(
                        "start-row".into(),
                        self.sel == Sel::Start,
                        type_icon("icons/car.svg", theme::PIN),
                        format!("Car at {} s", fmt_ms(ms)),
                        String::new(),
                        Some((
                            format!("{:.0} km/h", glam::Vec3::from(s.velocity).length() * 3.6),
                            theme::MUTED,
                        )),
                        None,
                    )
                    .on_mouse_down(
                        gpui::MouseButton::Right,
                        cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                            this.row_menu(Sel::Start, e.position, cx)
                        }),
                    )
                    .on_click(cx.listener(|this, _, _, cx| {
                        this.select(Sel::Start);
                        cx.notify();
                    }));
                rows.push(row.into_any_element());
            }
            for st in self.doc.setups.clone() {
                let id = st.id;
                let best = st.tricks.and_then(|t| {
                    use gearbox_search::tricks::TrickKind as K;
                    [(K::Nosebug, t[0]), (K::Bugslide, t[1]), (K::Uberbug, t[2])]
                        .into_iter()
                        .filter(|x| x.1 > 0)
                        .max_by_key(|x| x.1)
                });
                let value = best.map(|(k, n)| {
                    (
                        format!("{} {:.2}", crate::lab::trick_label(k), n as f32 / 100.0),
                        crate::lab::trick_color(k),
                    )
                });
                let row = self
                    .orow(
                        format!("setup-{id}"),
                        self.sel == Sel::Setup(id),
                        type_icon("icons/setup.svg", theme::SETUP),
                        st.name.clone(),
                        format!("{:.2} s", st.controls.len() as f32 / 100.0),
                        value,
                        None,
                    )
                    .on_mouse_down(
                        gpui::MouseButton::Right,
                        cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                            this.row_menu(Sel::Setup(id), e.position, cx)
                        }),
                    )
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.select(Sel::Setup(id));
                        cx.notify();
                    }));
                rows.push(row.into_any_element());
            }
        }
        let (open, g) = self.group("route", "Route".into(), None, vec![], cx);
        rows.push(g);
        if open {
            let t =
                self.route_ticks().map(|(a, b)| (format!("{} s", fmt_ms(10 * (b - a) as i32)), theme::MUTED));
            let row = self
                .orow(
                    "route-row".into(),
                    self.sel == Sel::Route,
                    type_icon("icons/route.svg", theme::PIN),
                    crate::ui::route::route_title(self),
                    String::new(),
                    t,
                    None,
                )
                .on_mouse_down(
                    gpui::MouseButton::Right,
                    cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                        this.row_menu(Sel::Route, e.position, cx)
                    }),
                )
                .on_click(cx.listener(|this, _, _, cx| {
                    this.select(Sel::Route);
                    this.frame_route();
                    cx.notify();
                }));
            rows.push(row.into_any_element());
        }
        let (open, g) = self.group("lines", "Lines".into(), Some(self.doc.guides.len()), vec![], cx);
        if !self.doc.guides.is_empty() {
            rows.push(g);
        }
        if open {
            for gl in self.doc.guides.clone() {
                let id = gl.id;
                let row = self
                    .orow(
                        format!("guide-{id}"),
                        self.sel == Sel::Guide(id),
                        type_icon("icons/line.svg", theme::WARN),
                        gl.name.clone(),
                        format!("{} points", gl.points.len()),
                        Some((format!("{:.0} m", gl.length()), theme::MUTED)),
                        None,
                    )
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.select(Sel::Guide(id));
                        cx.notify();
                    }));
                rows.push(row.into_any_element());
            }
        }
        let (open, g) = self.group("runs", "Runs".into(), None, vec![], cx);
        rows.push(g);
        if open {
            let fin = self.sim.frames.iter().find_map(|f| f.finish).map(|t| fmt_ms(t as i32));
            let cur = self
                .orow(
                    "run-current".into(),
                    self.sel == Sel::Current,
                    type_icon("icons/play.svg", theme::TEXT),
                    self.doc.run_name.clone(),
                    self.doc.run_origin.label(),
                    Some(fin.map_or(("\u{2014}".into(), theme::FAINT), |f| (f, theme::TEXT))),
                    None,
                )
                .child(self.ab_badges(RunRef::Current))
                .on_click(cx.listener(|this, _, _, cx| {
                    this.select(Sel::Current);
                    cx.notify();
                }));
            rows.push(cur.into_any_element());
            for r in self.doc.runs.clone() {
                let id = r.id;
                let fin = self.ref_frames(RunRef::Run(id)).map(|f| {
                    f.iter()
                        .find_map(|f| f.finish)
                        .map_or(("\u{2014}".to_string(), theme::FAINT), |t| (fmt_ms(t as i32), theme::MUTED))
                });
                let row = self
                    .orow(
                        format!("run-{id}"),
                        self.sel == Sel::Run(id),
                        type_icon("icons/keep.svg", theme::GHOST),
                        r.name.clone(),
                        r.origin.label(),
                        fin,
                        None,
                    )
                    .child(self.ab_badges(RunRef::Run(id)))
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.select(Sel::Run(id));
                        let _ = this.ref_frames(RunRef::Run(id));
                        cx.notify();
                    }));
                rows.push(row.into_any_element());
            }
            let n = self.doc.history.len();
            let hopen = !self.collapsed.contains("history");
            if n > 0 {
                let row = self
                    .orow(
                        "history-row".into(),
                        false,
                        type_icon("icons/history.svg", theme::MUTED),
                        "History".into(),
                        String::new(),
                        Some((n.to_string(), theme::FAINT)),
                        None,
                    )
                    .child(
                        div()
                            .w(px(18.0))
                            .flex()
                            .justify_center()
                            .text_size(px(theme::FS_SMALL))
                            .text_color(c(theme::FAINT))
                            .child(if hopen { "\u{25be}" } else { "\u{25b8}" }),
                    )
                    .on_click(cx.listener(|this, _, _, cx| {
                        if !this.collapsed.remove("history") {
                            this.collapsed.insert("history".into());
                        }
                        cx.notify();
                    }));
                rows.push(row.into_any_element());
            }
            if hopen && n > 0 {
                for r in self.doc.history.clone().iter().rev().take(60) {
                    let id = r.id;
                    let row = self
                        .orow(
                            format!("hist-{id}"),
                            self.sel == Sel::Run(id),
                            swatch(theme::FAINT, "\u{25e6}"),
                            r.name.clone(),
                            r.at.clone(),
                            None,
                            None,
                        )
                        .pl(px(theme::SP4 + theme::SP5))
                        .on_click(cx.listener(move |this, _, _, cx| {
                            this.select(Sel::Run(id));
                            cx.notify();
                        }));
                    rows.push(row.into_any_element());
                }
            }
        }
        let (open, g) = self.group("ghosts", "Ghosts".into(), Some(self.ghosts.len()), vec![], cx);
        if !self.ghosts.is_empty() {
            rows.push(g);
        }
        if open {
            let fin = self.sim.frames.iter().find_map(|f| f.finish).map(|t| t as i32);
            for i in 0..self.ghosts.len() {
                let g = &self.ghosts[i].track;
                let t = g.race_time_ms;
                let delta = match (fin, t) {
                    (Some(a), Some(b)) => {
                        Some((fmt_delta(a - b), if a < b { theme::OK } else { theme::ERROR }))
                    }
                    _ => None,
                };
                let vis = self.ghosts[i].visible;
                let eye =
                    self.eye(format!("ghost-eye-{i}"), vis).on_click(cx.listener(move |this, _, _, cx| {
                        if let Some(g) = this.ghosts.get_mut(i) {
                            g.visible = !g.visible;
                        }
                        this.dirty = true;
                        cx.stop_propagation();
                        cx.notify();
                    }));
                let row = self
                    .orow(
                        format!("ghost-{i}"),
                        self.sel == Sel::Ghost(i),
                        type_icon("icons/ghost.svg", theme::GHOST),
                        g.name.clone(),
                        format!("{} samples", g.samples.len()),
                        Some((t.map_or("unfinished".into(), fmt_ms), theme::GHOST)),
                        delta,
                    )
                    .child(self.ab_badges(RunRef::Ghost(i)))
                    .child(eye)
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.select(Sel::Ghost(i));
                        cx.notify();
                    }));
                rows.push(row.into_any_element());
            }
        }
        let (open, g) =
            self.group("world", "Checkpoints & finishes".into(), Some(self.gates.len()), vec![], cx);
        rows.push(g);
        if open {
            let crossing = |w: u32| {
                self.sim
                    .frames
                    .iter()
                    .find(|f| f.last_crossing.is_some_and(|c| c.0 == w))
                    .and_then(|f| f.last_crossing)
                    .map(|c| c.1)
            };
            let mut gates: Vec<(Option<u32>, u32, String, u32)> = self
                .gates
                .iter()
                .map(|g| {
                    (crossing(g.gate.waypoint), g.gate.waypoint, self.gate_name(g.gate.waypoint), g.gate.kind)
                })
                .collect();
            gates.sort_by_key(|g| (g.0.is_none(), g.0, g.1));
            let order: Rc<Vec<Sel>> = Rc::new(gates.iter().map(|g| Sel::Gate(g.1)).collect());
            for (crossed, w, name, kind) in gates {
                let is_goal = self.doc.goals.iter().any(|g| matches!(&g.kind, GoalKind::Gate(x) if x.has(w)));
                let crossed = crossed.map(|t| fmt_ms(t as i32));
                let row = self
                    .orow(
                        format!("gate-{w}"),
                        self.sel == Sel::Gate(w) || self.multi.contains(&Sel::Gate(w)),
                        type_icon(gate_icon(kind), if kind == 1 { theme::WARN } else { theme::MUTED }),
                        name,
                        format!("waypoint {w}{}", if is_goal { " \u{00b7} goal" } else { "" }),
                        crossed.map(|t| (t, theme::MUTED)),
                        None,
                    )
                    .on_mouse_down(
                        gpui::MouseButton::Right,
                        cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                            this.row_menu(Sel::Gate(w), e.position, cx)
                        }),
                    )
                    .on_click(cx.listener({
                        let order = order.clone();
                        move |this, e: &gpui::ClickEvent, _, cx| {
                            let m = e.modifiers();
                            if m.shift {
                                this.select_range(Sel::Gate(w), &order);
                            } else if m.platform {
                                this.toggle(Sel::Gate(w));
                            } else {
                                this.select(Sel::Gate(w));
                            }
                            cx.notify();
                        }
                    }));
                rows.push(row.into_any_element());
            }
            if self.show.physics {
                use crate::materials::Family;
                let fams = [
                    Family::Road,
                    Family::Ice,
                    Family::Dirt,
                    Family::Grass,
                    Family::Plastic,
                    Family::Wood,
                    Family::Metal,
                    Family::Water,
                    Family::Other,
                ];
                for f in fams {
                    let hidden = self.show.hidden_families.contains(&f);
                    let row = self
                        .orow(
                            format!("fam-{f:?}"),
                            false,
                            swatch(theme::MUTED, "\u{25a0}"),
                            format!("{f:?} surfaces"),
                            String::new(),
                            None,
                            None,
                        )
                        .child(self.eye(format!("fam-eye-{f:?}"), !hidden))
                        .on_click(cx.listener(move |this, _, _, cx| {
                            if !this.show.hidden_families.contains(&f) {
                                this.show.hidden_families.push(f);
                            } else {
                                this.show.hidden_families.retain(|x| *x != f);
                            }
                            this.rebuild_world_mesh(cx);
                            cx.notify();
                        }));
                    rows.push(row.into_any_element());
                }
            }
        }
        let add = div()
            .id("outliner-add")
            .relative()
            .child(crate::ui::widgets::track("outliner-add"))
            .size(px(24.0))
            .flex()
            .items_center()
            .justify_center()
            .rounded(px(theme::R_CTL))
            .cursor_pointer()
            .hover(|s| s.bg(c(theme::RAISED)))
            .child(crate::ui::widgets::icon("icons/plus.svg", theme::MUTED, 14.0))
            .tooltip(tip("Add", Some("Goals, a car, a guide line or a ghost.")))
            .on_click(cx.listener(|this, e: &gpui::ClickEvent, _, cx| {
                this.popup = Some(Popup { at: e.position(), items: add_items() });
                cx.stop_propagation();
                cx.notify();
            }));
        div()
            .id("outliner")
            .size_full()
            .flex()
            .flex_col()
            .child(
                div()
                    .h(px(theme::H_BAR))
                    .flex_none()
                    .pl(px(theme::PAD))
                    .pr(px(theme::SP2))
                    .flex()
                    .items_center()
                    .border_b_1()
                    .border_color(c(theme::HAIRLINE))
                    .child(
                        div()
                            .flex_1()
                            .text_size(px(theme::FS_BODY))
                            .font_weight(FontWeight::SEMIBOLD)
                            .child("Outliner"),
                    )
                    .child(add),
            )
            .child(
                div().id("outliner-rows").flex_1().min_h_0().overflow_y_scroll().pb(px(20.0)).children(rows),
            )
    }

    fn ab_badges(&self, r: RunRef) -> gpui::Div {
        let (a, b) = self.compare;
        div()
            .flex()
            .gap(px(2.0))
            .when(a == r, |d| {
                d.child(
                    div()
                        .px(px(3.0))
                        .rounded(px(2.0))
                        .bg(c(theme::TEXT))
                        .text_color(c(theme::BG))
                        .text_size(px(theme::FS_SMALL))
                        .font_weight(FontWeight::SEMIBOLD)
                        .child("A"),
                )
            })
            .when(b == r, |d| {
                d.child(
                    div()
                        .px(px(3.0))
                        .rounded(px(2.0))
                        .bg(c(theme::GHOST))
                        .text_color(c(theme::BG))
                        .text_size(px(theme::FS_SMALL))
                        .font_weight(FontWeight::SEMIBOLD)
                        .child("B"),
                )
            })
    }

    pub fn reference_choices(&self) -> Vec<(String, RunRef)> {
        let mut v = vec![("Previous version".to_string(), RunRef::Previous)];
        for r in &self.doc.runs {
            v.push((r.name.clone(), RunRef::Run(r.id)));
        }
        for (i, g) in self.ghosts.iter().enumerate() {
            v.push((format!("Ghost {}", g.track.name), RunRef::Ghost(i)));
        }
        v
    }

    pub fn pose_goal_from_cursor(&mut self) {
        if let Some(f) = self.sim.frames.get(self.cursor) {
            let mut p = crate::doc::PoseSpec::at(glam::Vec3::from(f.position()), 0.0);
            p.ypr = crate::doc::rot_ypr(crate::doc::frame_rot(f));
            p.velocity = f.velocity();
            self.add_goal(GoalKind::Pose(p));
        }
    }
}
