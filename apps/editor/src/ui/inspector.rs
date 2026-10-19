use std::rc::Rc;

use glam::Vec3;
use gpui::{
    AnyElement, Context, FontWeight, InteractiveElement, IntoElement, ParentElement,
    StatefulInteractiveElement, Styled, div, px,
};

use crate::app::*;
use crate::doc::{GoalKind, PoseSpec, StartState, When};
use crate::fmt::{fmt_delta, fmt_ms};
use crate::theme::{self, c};
use crate::ui::widgets::*;

type Set3 = Rc<dyn Fn(&mut Editor, usize, f32)>;

impl Editor {
    fn v3(&mut self, id: &str, v: [f32; 3], f: Num, set: Set3, cx: &mut Context<Self>) -> AnyElement {
        let mut row = div().flex().gap(px(theme::SP1)).w_full();
        for k in 0..3 {
            let s = set.clone();
            row = row.child(self.num(
                format!("{id}.{k}"),
                v[k] as f64,
                f,
                move |e, x, _| s(e, k, x as f32),
                cx,
            ));
        }
        row.into_any_element()
    }

    pub(crate) fn ro(&self, s: impl Into<gpui::SharedString>) -> gpui::Div {
        mono(s, theme::FS_SMALL).text_color(c(theme::MUTED))
    }

    pub fn inspector(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        self.field_value.retain(|_, _| true);
        let (title, body): (String, Vec<AnyElement>) = match self.sel {
            Sel::Goal(id) => match self.doc.goal(id).cloned() {
                Some(g) => {
                    let k = g.kind_label();
                    (format!("{}{} goal", k[..1].to_uppercase(), &k[1..]), self.goal_props(id, cx))
                }
                None => ("Playhead".into(), self.car_props(cx)),
            },
            Sel::Start => ("Car".into(), self.start_props(cx)),
            Sel::Run(id) => ("Run".into(), self.run_props(id, cx)),
            Sel::Ghost(i) => ("Ghost".into(), self.ghost_props(i, cx)),
            Sel::Gate(w) => (self.gate_name(w), self.gate_props(w, cx)),
            Sel::Guide(id) => ("Guide line".into(), self.guide_props(id, cx)),
            Sel::Setup(id) => {
                (self.doc.setup(id).map_or("Setup".into(), |s| s.name.clone()), self.setup_props(id, cx))
            }
            Sel::Route => (format!("Route  {}", crate::ui::route::route_title(self)), self.route_props(cx)),
            Sel::Candidate(id) => (format!("Candidate #{id}"), self.candidate_props(id, cx)),
            Sel::Current => (self.doc.run_name.clone(), self.current_run_props(cx)),
            Sel::None => ("Playhead".into(), self.car_props(cx)),
        };
        div()
            .id("inspector")
            .size_full()
            .flex()
            .flex_col()
            .child(
                div()
                    .px(px(theme::PAD))
                    .pt(px(theme::PAD))
                    .pb(px(theme::SP1))
                    .text_size(px(theme::FS_EMPH))
                    .font_weight(FontWeight::SEMIBOLD)
                    .truncate()
                    .child(title),
            )
            .child(
                div()
                    .id("inspector-body")
                    .flex_1()
                    .min_h_0()
                    .overflow_y_scroll()
                    .pb(px(theme::SP2))
                    .flex()
                    .flex_col()
                    .children(body.into_iter().map(|e| div().flex_none().child(e))),
            )
    }

    fn goal_props(&mut self, id: u32, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(g) = self.doc.goal(id).cloned() else { return v };
        let i = self.doc.goals.iter().position(|x| x.id == id).unwrap_or(0);
        let r = self.reach.get(i).copied().unwrap_or_default();
        let nf = self.text_field(
            format!("goal.{id}.name"),
            &g.name,
            move |e, s, _| {
                e.edit("rename goal", None, |d| {
                    if let Some(g) = d.goal_mut(id) {
                        g.name = s;
                    }
                })
            },
            cx,
        );
        v.push(prop("Name", nf).into_any_element());
        let at = |k: usize| {
            self.sim.frames.get(k).map_or(String::new(), |f| format!(" at {} s", fmt_ms(f.race_ms)))
        };
        let reached = match r.ms {
            Some(ms) => format!("{} s \u{00b7} {:.0} km/h", fmt_ms(ms), r.speed_kmh.unwrap_or(0.0)),
            _ if r.out_of_order => "only before the previous goal".into(),
            _ => match r.closest {
                Some((k, d)) if matches!(g.kind, GoalKind::Pose(_)) => {
                    format!("misses (pose error {d:.1}{})", at(k))
                }
                Some((k, d)) => format!("misses by {d:.1} m{}", at(k)),
                None => "\u{2014}".into(),
            },
        };
        v.push(
            prop(
                "Your run",
                div()
                    .text_size(px(theme::FS_BODY))
                    .text_color(c(if r.ms.is_some() { theme::TEXT } else { theme::MUTED }))
                    .child(reached),
            )
            .into_any_element(),
        );
        if let (Some(a), Some(Some(b))) = (r.ms, self.ref_times.get(i)) {
            v.push(
                prop(
                    format!("vs {}", self.ref_label(self.reference)),
                    self.ro(fmt_delta(a - b)).text_color(c(if a <= *b { theme::OK } else { theme::ERROR })),
                )
                .into_any_element(),
            );
        }
        let edit = move |e: &mut Editor, label: &str, f: Box<dyn FnOnce(&mut GoalKind)>| {
            let key = format!("goal.{id}.{label}");
            e.edit(label, Some(&key), |d| {
                if let Some(g) = d.goal_mut(id) {
                    f(&mut g.kind);
                }
            })
        };
        match g.kind.clone() {
            GoalKind::Gate(gate) => {
                v.push(heading("Gate").into_any_element());
                v.push(
                    prop(
                        "Waypoint",
                        self.ro(format!(
                            "{} \u{00b7} {} \u{00b7} {} trigger box{}",
                            gate.waypoint,
                            if gate.kind == 1 { "finish" } else { "checkpoint" },
                            gate.boxes.len(),
                            if gate.boxes.len() == 1 { "" } else { "es" }
                        )),
                    )
                    .into_any_element(),
                );
                let auto = gate.ordinal.is_none();
                let ord = gate
                    .ordinal
                    .unwrap_or_else(|| r.tick.map_or(0, |k| self.sim.frames[k].waypoints as usize))
                    as f64;
                let of = self.num(
                    format!("goal.{id}.ordinal"),
                    ord,
                    INT.range(1.0, 999.0),
                    move |e, x, _| {
                        edit(
                            e,
                            "gate ordinal",
                            Box::new(move |k| {
                                if let GoalKind::Gate(g) = k {
                                    g.ordinal = Some(x.round() as usize)
                                }
                            }),
                        )
                    },
                    cx,
                );
                v.push(
                    prop(
                        "Crossing #",
                        div().flex().items_center().gap(px(theme::SP2)).w_full().child(of).child(checkbox(format!("goal-{id}-auto"), auto, "from the run").on_click(cx.listener(move |this, _, _, cx| {
                            edit(this, "gate ordinal", Box::new(move |k| if let GoalKind::Gate(g) = k { g.ordinal = if g.ordinal.is_some() { None } else { Some(ord.max(1.0) as usize) } }));
                            cx.notify();
                        }))),
                    )
                    .id("gate-ord-row")
                    .tooltip(tip("Crossing ordinal", Some("Which crossing this gate must be, counting every checkpoint, lap and finish (1-based). With \u{201c}from the run\u{201d} it is the crossing number the current run has there, else one more than the crossings before the search window.")))

                    .into_any_element(),
                );
            }
            GoalKind::Point { pos, radius } => {
                v.push(heading("Point").into_any_element());
                let set: Set3 = Rc::new(move |e, k, x| {
                    edit(
                        e,
                        "move point",
                        Box::new(move |g| {
                            if let GoalKind::Point { pos, .. } = g {
                                pos[k] = x
                            }
                        }),
                    )
                });
                let row = self.v3(&format!("goal.{id}.pos"), pos, M, set, cx);
                v.push(prop("Position", row).into_any_element());
                let rf = self.num(
                    format!("goal.{id}.radius"),
                    radius as f64,
                    M.range(0.05, 500.0),
                    move |e, x, _| {
                        edit(
                            e,
                            "point radius",
                            Box::new(move |g| {
                                if let GoalKind::Point { radius, .. } = g {
                                    *radius = x as f32
                                }
                            }),
                        )
                    },
                    cx,
                );
                v.push(prop("Radius", rf).id("ip1").tooltip(tip("Reach radius", Some("Reached by the first state (end of a tick) whose car position is within this distance."))).into_any_element());
                let ground = self.ground_below(Vec3::from(pos));
                v.push(
                    prop(
                        "Height",
                        self.ro(ground.map_or("\u{2014}".into(), |gy| {
                            format!("{:.2} m above the surface below", pos[1] - gy)
                        })),
                    )
                    .into_any_element(),
                );
            }
            GoalKind::Pose(p) => v.extend(self.pose_props(id, &p, cx)),
            GoalKind::Box { pos, size, ypr } => {
                v.push(heading("Box trigger").into_any_element());
                let set: Set3 = Rc::new(move |e, k, x| {
                    edit(
                        e,
                        "move box",
                        Box::new(move |g| {
                            if let GoalKind::Box { pos, .. } = g {
                                pos[k] = x
                            }
                        }),
                    )
                });
                let row = self.v3(&format!("goal.{id}.pos"), pos, M, set, cx);
                v.push(prop("Center", row).into_any_element());
                let set: Set3 = Rc::new(move |e, k, x| {
                    edit(
                        e,
                        "box size",
                        Box::new(move |g| {
                            if let GoalKind::Box { size, .. } = g {
                                size[k] = x.max(0.05)
                            }
                        }),
                    )
                });
                let row = self.v3(&format!("goal.{id}.size"), size, M, set, cx);
                v.push(prop("Size", row).id("box-size").tooltip(tip("Box size (m)", Some("Width across, height, and length along its facing direction. Reached when the car's centre is inside at a tick, or passes through between two ticks."))).into_any_element());
                let set: Set3 = Rc::new(move |e, k, x| {
                    edit(
                        e,
                        "box rotation",
                        Box::new(move |g| {
                            if let GoalKind::Box { ypr, .. } = g {
                                ypr[k] = x
                            }
                        }),
                    )
                });
                let row = self.v3(&format!("goal.{id}.ypr"), ypr, DEG, set, cx);
                v.push(prop("Rotation", row).into_any_element());
            }
        }
        v.extend(self.when_props(id, &g.when, cx));
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_wrap()
                    .gap(px(theme::SP1))
                    .child(
                        btn("goal-obj", "Search to here", Btn::Primary)
                            .tooltip(tip("Search towards this goal from the playhead", None))
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.search_toward(id);
                                cx.notify();
                            })),
                    )
                    .child(btn("goal-focus", "Focus  F", Btn::Quiet).on_click(cx.listener(
                        |this, _, _, cx| {
                            this.focus_selection();
                            cx.notify();
                        },
                    )))
                    .child(btn("goal-del", "Delete", Btn::Danger).on_click(cx.listener(|this, _, _, cx| {
                        this.delete_selected();
                        cx.notify();
                    }))),
            )
            .into_any_element(),
        );
        v
    }

    fn when_props(&mut self, id: u32, w: &When, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let ed = move |e: &mut Editor, label: &str, f: Box<dyn FnOnce(&mut When)>| {
            let key = format!("goal.{id}.when.{label}");
            e.edit(label, Some(&key), |d| {
                if let Some(g) = d.goal_mut(id) {
                    f(&mut g.when);
                }
            })
        };
        let presets: Vec<(String, u8)> = vec![
            ("Nose down (nosebug)".into(), 1),
            ("Sideways (bugslide)".into(), 2),
            ("Level, wheels down".into(), 3),
            ("None".into(), 0),
        ];
        let menu = self.menu_button(
            &format!("when-presets-{id}"),
            "Presets",
            presets,
            move |e, p, _| {
                ed(
                    e,
                    "goal conditions",
                    Box::new(move |w| match p {
                        1 => w.pitch = Some([-90.0, -50.0]),
                        2 => w.slide = Some([60.0, 120.0]),
                        3 => w.pitch = Some([-15.0, 15.0]),
                        _ => *w = When::default(),
                    }),
                )
            },
            cx,
        );
        v.push(heading("Counts only when").child(div().flex_1()).child(menu).into_any_element());
        let on = w.min_kmh.is_some();
        let mut row = div().flex().items_center().gap(px(theme::SP2)).w_full().child(
            checkbox(format!("when-speed-{id}"), on, "at least").on_click(cx.listener(
                move |this, _, _, cx| {
                    ed(
                        this,
                        "goal speed condition",
                        Box::new(move |w| w.min_kmh = if on { None } else { Some(300.0) }),
                    );
                    cx.notify();
                },
            )),
        );
        if let Some(kmh) = w.min_kmh {
            row = row.child(self.num(
                format!("goal.{id}.when.kmh"),
                f64::from(kmh),
                KMH.range(0.0, 1000.0),
                move |e, x, _| ed(e, "goal speed condition", Box::new(move |w| w.min_kmh = Some(x as f32))),
                cx,
            ));
        }
        v.push(
            prop("Speed", row)
                .id("when-speed")
                .tooltip(tip(
                    "Fast enough",
                    Some("The goal only counts if the car is at least this fast when it gets there."),
                ))
                .into_any_element(),
        );
        let range = |this: &mut Editor,
                     key: &'static str,
                     label: &'static str,
                     cur: Option<[f32; 2]>,
                     default: [f32; 2],
                     set: fn(&mut When, Option<[f32; 2]>),
                     cx: &mut Context<Editor>|
         -> gpui::Div {
            let on = cur.is_some();
            let mut row = div().flex().items_center().gap(px(theme::SP1)).w_full().child(
                checkbox(format!("when-{key}-{id}"), on, "between").on_click(cx.listener(
                    move |this, _, _, cx| {
                        ed(this, label, Box::new(move |w| set(w, if on { None } else { Some(default) })));
                        cx.notify();
                    },
                )),
            );
            if let Some([lo, hi]) = cur {
                row = row
                    .child(this.num(
                        format!("goal.{id}.when.{key}0"),
                        f64::from(lo),
                        DEG.range(-180.0, 180.0),
                        move |e, x, _| ed(e, label, Box::new(move |w| set(w, Some([x as f32, hi])))),
                        cx,
                    ))
                    .child(muted("to"))
                    .child(this.num(
                        format!("goal.{id}.when.{key}1"),
                        f64::from(hi),
                        DEG.range(-180.0, 180.0),
                        move |e, x, _| ed(e, label, Box::new(move |w| set(w, Some([lo, x as f32])))),
                        cx,
                    ));
            }
            row
        };
        let row =
            range(self, "pitch", "goal nose condition", w.pitch, [-90.0, -50.0], |w, r| w.pitch = r, cx);
        v.push(prop("Nose", row).id("when-pitch").tooltip(tip("Nose angle", Some("The car's nose above the horizon: -90\u{00b0} points straight down, 0 level, 90 straight up. Landing nose down sets up a nosebug."))).into_any_element());
        let row =
            range(self, "slide", "goal slide condition", w.slide, [60.0, 120.0], |w, r| w.slide = r, cx);
        v.push(prop("Slide", row).id("when-slide").tooltip(tip("Slide angle", Some("The angle between where the car points and where it is going: 0 driving straight, about 90 sideways (a bugslide), 180 backwards."))).into_any_element());
        v
    }

    fn pose_props(&mut self, id: u32, p: &PoseSpec, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let ed = move |e: &mut Editor, label: &'static str, f: Box<dyn FnOnce(&mut PoseSpec)>| {
            let key = format!("goal.{id}.{label}");
            e.edit(label, Some(&key), |d| {
                if let Some(g) = d.goal_mut(id)
                    && let GoalKind::Pose(p) = &mut g.kind
                {
                    f(p);
                }
            })
        };
        let s: Set3 = Rc::new(move |e, k, x| ed(e, "pose position", Box::new(move |p| p.pos[k] = x)));
        let row = self.v3(&format!("goal.{id}.pos"), p.pos, M, s, cx);
        v.push(prop("Position", row).into_any_element());
        let s: Set3 = Rc::new(move |e, k, x| ed(e, "pose rotation", Box::new(move |p| p.ypr[k] = x)));
        let row = self.v3(&format!("goal.{id}.ypr"), p.ypr, DEG, s, cx);
        v.push(prop("Rotation", row).into_any_element());
        let pt = p.pos_tol.iter().copied().fold(0.0f32, f32::max);
        let pf = self.num(
            format!("goal.{id}.ptol1"),
            pt as f64,
            M.range(0.0, 100.0),
            move |e, x, _| ed(e, "pose tolerance", Box::new(move |p| p.pos_tol = [x.max(0.0) as f32; 3])),
            cx,
        );
        let af = self.num(
            format!("goal.{id}.atol"),
            p.angle_tol as f64,
            DEG.range(0.0, 180.0),
            move |e, x, _| ed(e, "angle tolerance", Box::new(move |p| p.angle_tol = x as f32)),
            cx,
        );
        v.push(prop("Within \u{00b1}", div().flex().gap(px(theme::SP1)).w_full().child(pf).child(af)).id("pose-tol").tooltip(tip("Reached when", Some("the car is within this distance on every axis and its rotation within this angle (shortest angle, all axes together)."))).into_any_element());
        let use_v = p.use_velocity;
        v.push(
            prop(
                "",
                checkbox(format!("goal-{id}-usev"), use_v, "match the speed too").on_click(cx.listener(
                    move |this, _, _, cx| {
                        ed(this, "pose velocity check", Box::new(|p| p.use_velocity = !p.use_velocity));
                        cx.notify();
                    },
                )),
            )
            .into_any_element(),
        );
        if use_v {
            let speed = Vec3::from(p.velocity).length();
            let fwd = crate::doc::ypr_rot(p.ypr) * Vec3::Z;
            let sf = self.num(
                format!("goal.{id}.speed"),
                (speed * 3.6) as f64,
                KMH.range(0.0, 1500.0),
                move |e, x, _| {
                    ed(e, "pose speed", Box::new(move |p| p.velocity = (fwd * (x as f32 / 3.6)).into()))
                },
                cx,
            );
            v.push(prop("Speed", sf).into_any_element());
        }
        let more = self.collapsed.contains(&format!("pose-more-{id}"));
        v.push(
            div()
                .id("pose-more")
                .px(px(theme::PAD))
                .pl(px(104.0))
                .pt(px(theme::SP1))
                .text_size(px(theme::FS_BODY))
                .text_color(c(theme::MUTED))
                .cursor_pointer()
                .hover(|s| s.text_color(c(theme::TEXT)))
                .child(if more { "More \u{25be}" } else { "More \u{25b8}" })
                .on_click(cx.listener(move |this, _, _, cx| {
                    let k = format!("pose-more-{id}");
                    if !this.collapsed.remove(&k) {
                        this.collapsed.insert(k);
                    }
                    cx.notify();
                }))
                .into_any_element(),
        );
        if more {
            let s: Set3 =
                Rc::new(move |e, k, x| ed(e, "pose tolerance", Box::new(move |p| p.pos_tol[k] = x.max(0.0))));
            let row = self.v3(&format!("goal.{id}.ptol"), p.pos_tol, M, s, cx);
            v.push(prop("\u{00b1} per axis", row).into_any_element());
            if use_v {
                let s: Set3 =
                    Rc::new(move |e, k, x| ed(e, "pose velocity", Box::new(move |p| p.velocity[k] = x)));
                let row = self.v3(&format!("goal.{id}.vel"), p.velocity, MS, s, cx);
                v.push(prop("Velocity", row).into_any_element());
                let s: Set3 = Rc::new(move |e, k, x| {
                    ed(e, "velocity tolerance", Box::new(move |p| p.vel_tol[k] = x.max(0.0)))
                });
                let row = self.v3(&format!("goal.{id}.vtol"), p.vel_tol, MS, s, cx);
                v.push(prop("\u{00b1} velocity", row).into_any_element());
            }
            let s: Set3 = Rc::new(move |e, k, x| {
                ed(e, "position weight", Box::new(move |p| p.pos_weight[k] = x.max(0.0)))
            });
            let row = self.v3(&format!("goal.{id}.pw"), p.pos_weight, PLAIN, s, cx);
            v.push(prop("Miss weight", row).id("pose-pw").tooltip(tip("How misses are ranked", Some("A try that misses the pose scores \u{03a3} weight \u{00d7} (axis error in m)\u{00b2} + angle weight \u{00d7} (angle in rad)\u{00b2}; smaller is closer."))).into_any_element());
            let aw = self.num(
                format!("goal.{id}.aw"),
                p.angle_weight as f64,
                PLAIN.range(0.0, 1e6),
                move |e, x, _| ed(e, "angle weight", Box::new(move |p| p.angle_weight = x as f32)),
                cx,
            );
            v.push(prop("Angle weight", aw).into_any_element());
        }
        v.push(
            prop(
                "",
                btn("pose-reach", "Reach from playhead", Btn::Normal).tooltip(tip("Reach this pose", Some("Drives the run from the playhead into this car pose (place, rotation and, if on, velocity) with Build: many cars at once, the ones likely to match it soonest go on. Removes a placed car first: the run gets there itself."))).on_click(cx.listener(move |this, _, _, cx| {
                    this.reach_pose(id);
                    cx.notify();
                })),
            )
            .into_any_element(),
        );
        v
    }

    fn start_props(&mut self, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(s) = self.doc.start.clone() else { return v };
        let ed = |e: &mut Editor, label: &'static str, f: Box<dyn FnOnce(&mut StartState)>| {
            e.edit(label, Some(label), |d| {
                if let Some(s) = &mut d.start {
                    f(s);
                }
            })
        };
        v.push(heading("When").into_any_element());
        let tf = self.time_field(
            "start.tick",
            s.tick,
            move |e, k, _| ed(e, "start tick", Box::new(move |s| s.tick = k)),
            cx,
        );
        v.push(prop("At", tf).into_any_element());
        if s.tick < self.race_start_tick() {
            v.push(
                prop(
                    "",
                    div()
                        .text_size(px(theme::FS_BODY))
                        .text_color(c(theme::WARN))
                        .child("Before the race start: the start respawn will move the car back."),
                )
                .into_any_element(),
            );
        }
        v.push(heading("Pose").into_any_element());
        let set: Set3 = Rc::new(move |e, k, x| ed(e, "start position", Box::new(move |s| s.pos[k] = x)));
        let row = self.v3("start.pos", s.pos, M, set, cx);
        v.push(prop("Position", row).into_any_element());
        let set: Set3 = Rc::new(move |e, k, x| ed(e, "start rotation", Box::new(move |s| s.ypr[k] = x)));
        let row = self.v3("start.ypr", s.ypr, DEG, set, cx);
        v.push(prop("Yaw pitch roll", row).into_any_element());
        v.push(heading("Motion").into_any_element());
        let set: Set3 = Rc::new(move |e, k, x| ed(e, "start velocity", Box::new(move |s| s.velocity[k] = x)));
        let row = self.v3("start.vel", s.velocity, MS, set, cx);
        v.push(prop("Velocity (world)", row).into_any_element());
        let speed = Vec3::from(s.velocity).length();
        let fwd = s.forward();
        let sf = self.num(
            "start.speed",
            (Vec3::from(s.velocity).dot(fwd) * 3.6) as f64,
            KMH.range(-500.0, 1500.0),
            move |e, x, _| {
                ed(e, "start speed", Box::new(move |s| s.velocity = (s.forward() * (x as f32 / 3.6)).into()))
            },
            cx,
        );
        v.push(
            prop(
                "Along heading",
                div()
                    .flex()
                    .gap(px(theme::SP1))
                    .w_full()
                    .child(sf)
                    .child(self.ro(format!("|v| {speed:.3} m/s"))),
            )
            .id("ip3")
            .tooltip(tip(
                "Speed along the heading",
                Some("Sets the velocity to this speed along the car's forward axis (km/h)."),
            ))
            .into_any_element(),
        );
        let set: Set3 =
            Rc::new(move |e, k, x| ed(e, "start angular velocity", Box::new(move |s| s.angular[k] = x)));
        let deg_s = Num { unit: "\u{00b0}/s", ..DEG };
        let row = self.v3("start.ang", s.angular, deg_s, set, cx);
        v.push(prop("Angular (world)", row).into_any_element());
        v.push(
            div()
                .px(px(theme::PAD))
                .pt(px(theme::SP3))
                .flex()
                .gap(px(theme::SP2))
                .child(
                    btn_icon("car-search", "icons/sparkle.svg", "Search from this car", Btn::Normal)
                        .tooltip(tip(
                            "Search from this car",
                            Some("Opens Search with the range starting at the car."),
                        ))
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.search_from_car();
                            cx.notify();
                        })),
                )
                .into_any_element(),
        );
        v.push(heading("Contacts").into_any_element());
        let reset = s.reset_contacts;
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_col()
                    .gap(px(2.0))
                    .child(checkbox("start-reset", reset, "Reset contacts (game respawn)").on_click(cx.listener(move |this, _, _, cx| {
                        ed(this, "start contacts", Box::new(|s| s.reset_contacts = !s.reset_contacts));
                        cx.notify();
                    })))
                    .child(muted(if reset { "Wheels, contacts and effects reset as the game's respawn does; then the velocities above are applied." } else { "Keeps the wheels' contact and effect history from before the placement (approximate after a teleport)." })),
            )
            .into_any_element(),
        );
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_wrap()
                    .gap(px(theme::SP1))
                    .child(btn("start-sim", "Play from here", Btn::Normal).tooltip(tip("Simulate from here (\u{2318}P)", Some("Plays the run from the start state's tick: the session places the car on arriving there and simulates on."))).on_click(cx.listener(|this, _, _, cx| {
                        this.simulate_from_start();
                        cx.notify();
                    })))
                    .child(btn("start-save", "Save as setup", Btn::Normal).tooltip(tip("Save as setup", Some("Keeps the placed car and the run's inputs from it to the playhead as a setup (Setups in the outliner): reach it from any run later and join it on."))).on_click(cx.listener(|this, _, _, cx| {
                        this.save_setup();
                        cx.notify();
                    })))
                    .child(btn("start-cursor", "From car at cursor", Btn::Quiet).on_click(cx.listener(|this, _, _, cx| {
                        if let Some(s) = this.start_from_cursor() {
                            this.place_start(s);
                        }
                        cx.notify();
                    })))
                    .child(btn("start-del", "Remove", Btn::Danger).on_click(cx.listener(|this, _, _, cx| {
                        this.delete_selected();
                        cx.notify();
                    }))),
            )
            .into_any_element(),
        );
        v
    }

    fn run_props(&mut self, id: u32, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(r) = self.doc.run(id).cloned() else { return v };
        let is_alt = self.doc.runs.iter().any(|x| x.id == id);
        if is_alt {
            let nf = self.text_field(
                format!("run.{id}.name"),
                &r.name,
                move |e, s, _| {
                    e.edit("rename run", None, |d| {
                        if let Some(r) = d.runs.iter_mut().find(|r| r.id == id) {
                            r.name = s;
                        }
                    })
                },
                cx,
            );
            v.push(prop("Name", nf).into_any_element());
        } else {
            v.push(prop("Name", self.ro(r.name.clone())).into_any_element());
        }
        v.push(prop("Origin", self.ro(r.origin.label())).into_any_element());
        v.push(prop("Stored", self.ro(r.at.clone())).into_any_element());
        v.push(
            prop("Length", self.ro(format!("{} ticks \u{00b7} {:.2} s", r.len, r.len as f32 / 100.0)))
                .into_any_element(),
        );
        v.push(
            prop(
                "Start",
                self.ro(r
                    .start
                    .as_ref()
                    .map_or("map start".into(), |s| format!("placed at tick {}", s.tick))),
            )
            .into_any_element(),
        );
        let frames = self.ref_frames(RunRef::Run(id));
        match &frames {
            Some(f) => {
                let fin = f.iter().find_map(|f| f.finish).map_or("not finished".into(), |t| fmt_ms(t as i32));
                v.push(prop("Finish", self.ro(fin)).into_any_element());
                let goals = crate::doc::evaluate(&self.doc.goals, f);
                for (g, rr) in self.doc.goals.iter().zip(goals) {
                    v.push(
                        prop(g.name.clone(), self.ro(rr.ms.map_or("miss".into(), fmt_ms))).into_any_element(),
                    );
                }
            }
            None => v.push(prop("", muted("simulating\u{2026}")).into_any_element()),
        }
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_wrap()
                    .gap(px(theme::SP1))
                    .child(
                        btn("run-current", "Make current", Btn::Primary)
                            .tooltip(tip(
                                "Make this the current run (undoable; the current one goes to History)",
                                None,
                            ))
                            .on_click(cx.listener(move |this, _, _, cx| {
                                if let Some(r) = this.doc.run(id).cloned() {
                                    this.edit("make run current", None, |d| {
                                        d.remember("switching runs");
                                        d.inputs = r.inputs.clone();
                                        d.len = r.len;
                                        d.start = r.start.clone();
                                        d.run_origin = r.origin.clone();
                                        d.run_name = r.name.clone();
                                    });
                                    this.sel = Sel::Current;
                                }
                                cx.notify();
                            })),
                    )
                    .child(btn("run-a", "Set as A", Btn::Normal).on_click(cx.listener(
                        move |this, _, _, cx| {
                            this.compare.0 = RunRef::Run(id);
                            this.layout.bottom_tab = 1;
                            cx.notify();
                        },
                    )))
                    .child(btn("run-b", "Set as B", Btn::Normal).on_click(cx.listener(
                        move |this, _, _, cx| {
                            this.compare.1 = RunRef::Run(id);
                            this.layout.bottom_tab = 1;
                            cx.notify();
                        },
                    )))
                    .child(btn("run-ref", "Use as reference", Btn::Quiet).on_click(cx.listener(
                        move |this, _, _, cx| {
                            this.reference = RunRef::Run(id);
                            this.reevaluate();
                            cx.notify();
                        },
                    )))
                    .child(btn("run-del", "Delete", Btn::Danger).on_click(cx.listener(|this, _, _, cx| {
                        this.delete_selected();
                        cx.notify();
                    }))),
            )
            .into_any_element(),
        );
        v
    }

    fn ghost_props(&mut self, i: usize, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(g) = self.ghosts.get(i).map(|g| g.track.clone()) else { return v };
        v.push(
            prop("Replay", self.ro(g.path.file_name().unwrap_or_default().to_string_lossy().to_string()))
                .into_any_element(),
        );
        v.push(prop("Driver", self.ro(g.name.clone())).into_any_element());
        v.push(prop("Time", self.ro(g.race_time_ms.map_or("unfinished".into(), fmt_ms))).into_any_element());
        v.push(
            prop(
                "Checkpoints",
                self.ro(g.checkpoints_ms.iter().map(|t| fmt_ms(*t)).collect::<Vec<_>>().join("  ")),
            )
            .into_any_element(),
        );
        v.push(
            prop("Samples", self.ro(format!("{} (every 50 ms, mm-quantised)", g.samples.len())))
                .into_any_element(),
        );
        v.push(prop("Game build", self.ro(g.exe.clone().unwrap_or_default())).into_any_element());
        if let Some(f) = self.sim.frames.get(self.cursor) {
            let d = g.delta(f.race_ms, f.position());
            v.push(
                prop(
                    "At the cursor",
                    self.ro(format!(
                        "{} \u{00b7} {}",
                        d.behind_s.map_or("no time match".into(), |s| format!(
                            "{} s {}",
                            fmt_delta((s * 1000.0).round() as i32),
                            if s > 0.0 { "behind" } else { "ahead" }
                        )),
                        d.distance.map_or(String::new(), |m| format!("{m:.2} m apart"))
                    )),
                )
                .into_any_element(),
            );
        }
        let path = g.path.clone();
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_wrap()
                    .gap(px(theme::SP1))
                    .child(btn("ghost-b", "Compare (B)", Btn::Normal).on_click(cx.listener(
                        move |this, _, _, cx| {
                            this.compare = (RunRef::Current, RunRef::Ghost(i));
                            this.layout.bottom_tab = 1;
                            cx.notify();
                        },
                    )))
                    .child(btn("ghost-ref", "Use as reference", Btn::Quiet).on_click(cx.listener(
                        move |this, _, _, cx| {
                            this.reference = RunRef::Ghost(i);
                            this.reevaluate();
                            cx.notify();
                        },
                    )))
                    .child(
                        btn("ghost-import", "Import its inputs", Btn::Normal)
                            .tooltip(tip("Replace the run with the replay's driven inputs (undoable)", None))
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.import_replay(&path, cx);
                                cx.notify();
                            })),
                    )
                    .child(btn("ghost-del", "Remove", Btn::Danger).on_click(cx.listener(
                        |this, _, _, cx| {
                            this.delete_selected();
                            cx.notify();
                        },
                    ))),
            )
            .into_any_element(),
        );
        v
    }

    fn guide_props(&mut self, id: u32, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(g) = self.doc.guide(id).cloned() else { return v };
        let nf = self.text_field(
            format!("guide.{id}.name"),
            &g.name,
            move |e, s, _| {
                e.edit("rename line", None, |d| {
                    if let Some(g) = d.guide_mut(id) {
                        g.name = s;
                    }
                })
            },
            cx,
        );
        v.push(prop("Name", nf).into_any_element());
        v.push(
            prop("Length", self.ro(format!("{:.0} m \u{00b7} {} points", g.length(), g.points.len())))
                .into_any_element(),
        );
        let drawing = self.drawing == Some(id);
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_wrap()
                    .gap(px(theme::SP1))
                    .child(
                        btn(
                            "guide-more",
                            if drawing { "Drawing\u{2026} (Esc ends)" } else { "Add points" },
                            Btn::Normal,
                        )
                        .on_click(cx.listener(move |this, _, _, cx| {
                            this.set_tool(Tool::Line);
                            this.drawing = Some(id);
                            cx.notify();
                        })),
                    )
                    .child(btn("guide-undo", "Remove last point", Btn::Quiet).on_click(cx.listener(
                        move |this, _, _, cx| {
                            this.edit("remove line point", None, |d| {
                                if let Some(g) = d.guide_mut(id) {
                                    g.points.pop();
                                }
                            });
                            cx.notify();
                        },
                    )))
                    .child(btn("guide-del", "Delete", Btn::Danger).on_click(cx.listener(
                        |this, _, _, cx| {
                            this.delete_selected();
                            cx.notify();
                        },
                    ))),
            )
            .into_any_element(),
        );
        v.push(div().px(px(theme::PAD)).pt(px(theme::SP1)).text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child("The search can measure misses along this line: Search \u{2192} Misses \u{2192} Along a line.").into_any_element());
        v
    }

    fn setup_props(&mut self, id: u32, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(st) = self.doc.setup(id).cloned() else { return v };
        let nf = self.text_field(
            format!("setup.{id}.name"),
            &st.name,
            move |e, s, _| {
                e.edit("rename setup", None, |d| {
                    if let Some(x) = d.setups.iter_mut().find(|x| x.id == id) {
                        x.name = s;
                    }
                })
            },
            cx,
        );
        v.push(prop("Name", nf).into_any_element());
        let kmh = |v: [f32; 3]| glam::Vec3::from(v).length() * 3.6;
        v.push(
            prop(
                "Starts",
                self.ro(format!(
                    "{:.0} km/h \u{00b7} yaw {:.0}\u{00b0} pitch {:.0}\u{00b0} roll {:.0}\u{00b0}",
                    kmh(st.start.velocity),
                    st.start.ypr[0],
                    st.start.ypr[1],
                    st.start.ypr[2]
                )),
            )
            .into_any_element(),
        );
        v.push(
            prop(
                "Inputs",
                self.ro(format!("{} ticks ({:.2} s)", st.controls.len(), st.controls.len() as f32 / 100.0)),
            )
            .into_any_element(),
        );
        v.push(
            prop(
                "Ends",
                self.ro(format!(
                    "{:.0} km/h \u{00b7} {:.0} m from the start",
                    kmh(st.end_velocity),
                    glam::Vec3::from(st.end_pos).distance(glam::Vec3::from(st.start.pos))
                )),
            )
            .into_any_element(),
        );
        v.push(prop("Tricks", crate::ui::solve::trick_badges(st.tricks)).into_any_element());
        if let Some(t) = self.trick_checks.get(&id).copied() {
            v.push(prop("In your run", crate::ui::solve::trick_badges(Some(t))).into_any_element());
        }
        if let Some(t) = self.setup_readout(&format!("{} start", st.name)) {
            v.push(prop("Run at start", self.ro(t)).into_any_element());
        }
        if let Some(t) = self.setup_readout(&format!("{} end", st.name)) {
            v.push(prop("Run at end", self.ro(t)).into_any_element());
        }
        v.push(
            prop(
                "",
                div()
                    .flex()
                    .flex_wrap()
                    .gap(px(theme::SP1))
                    .child(btn("setup-reach", "Reach from playhead", Btn::Primary).tooltip(tip("Reach this setup", Some("Drives the run from the playhead into the setup's start: place, rotation and velocity, within a pose goal's tolerances. Removes the placed car (the run gets there itself) and runs Build towards a new pose goal."))).on_click(cx.listener(move |this, _, _, cx| {
                        this.reach_setup(id);
                        cx.notify();
                    })))
                    .child(btn("setup-join", "Join", Btn::Normal).tooltip(tip("Join the setup", Some("Where the run reaches the setup's start, writes the setup's inputs, then re-tunes them tick by tick towards the setup's end: the run arrives near the start, never exactly on it."))).on_click(cx.listener(move |this, _, _, cx| {
                        this.join_setup(id);
                        cx.notify();
                    })))
                    .child(btn("setup-check", "Check tricks", Btn::Quiet).tooltip(tip("Check tricks", Some("Counts the trick signatures where the run joined the setup, for the setup's length, to compare with the setup's own."))).on_click(cx.listener(move |this, _, _, cx| {
                        this.check_setup_tricks(id);
                        cx.notify();
                    })))
                    .child(btn("setup-try", "Try at playhead", Btn::Quiet).tooltip(tip("Try the setup", Some("Places the setup's car at the playhead and writes its inputs from there."))).on_click(cx.listener(move |this, _, _, cx| {
                        this.try_setup(id);
                        cx.notify();
                    })))
                    .child(btn("setup-del", "Delete", Btn::Danger).on_click(cx.listener(|this, _, _, cx| {
                        this.delete_selected();
                        cx.notify();
                    }))),
            )
            .into_any_element(),
        );
        v
    }

    fn gate_props(&mut self, w: u32, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(g) = self.gates.iter().find(|g| g.gate.waypoint == w).cloned() else { return v };
        v.push(
            prop(
                "Waypoint",
                self.ro(format!("{} \u{00b7} {}", w, if g.gate.kind == 1 { "finish" } else { "checkpoint" })),
            )
            .into_any_element(),
        );
        v.push(
            prop("Map order", self.ro(g.order.map_or("\u{2014}".into(), |o| o.to_string())))
                .into_any_element(),
        );
        v.push(prop("Trigger boxes", self.ro(g.gate.boxes.len().to_string())).into_any_element());
        let crossings: Vec<String> = self
            .sim
            .frames
            .windows(2)
            .filter(|p| p[1].waypoints > p[0].waypoints && p[1].last_crossing.is_some_and(|c| c.0 == w))
            .map(|p| format!("#{} at {}", p[1].waypoints, fmt_ms(p[1].last_crossing.unwrap().1 as i32)))
            .collect();
        v.push(
            prop(
                "Run crosses",
                self.ro(if crossings.is_empty() { "never".into() } else { crossings.join(", ") }),
            )
            .into_any_element(),
        );
        v.push(
            prop(
                "",
                btn("gate-goal", "Add as goal", Btn::Primary).on_click(cx.listener(move |this, _, _, cx| {
                    if let Some(gate) = this.gate_goal(w) {
                        this.add_goal(GoalKind::Gate(gate));
                    }
                    cx.notify();
                })),
            )
            .into_any_element(),
        );
        v
    }

    fn candidate_props(&mut self, id: u64, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let cand = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).cloned();
        let Some(cand) = cand else { return v };
        let m = self.search.live.lock().unwrap().metric(&cand.score);
        v.push(prop("Score", self.ro(m.text.clone()).text_color(c(theme::TEXT))).into_any_element());
        v.push(prop("Engine score", self.ro(cand.score.to_string())).into_any_element());
        v.push(
            prop(
                "Found",
                self.ro(format!("listed after {} tries \u{00b7} {:.1} s", cand.attempts, cand.elapsed)),
            )
            .into_any_element(),
        );
        let changed = crate::doc::diff_ranges(&self.doc.inputs, &cand.inputs, self.race_ms0, self.doc.len);
        let n: usize = changed.iter().map(|r| r.len()).sum();
        v.push(
            prop(
                "Changed ticks",
                self.ro(format!(
                    "{n} in {} range{}",
                    changed.len(),
                    if changed.len() == 1 { "" } else { "s" }
                )),
            )
            .into_any_element(),
        );
        if let Some(f) = self.ref_frames(RunRef::Candidate(id)) {
            let goals = crate::doc::evaluate(&self.doc.goals, &f);
            for (i, (g, r)) in self.doc.goals.iter().zip(goals).enumerate() {
                let cur = self.reach.get(i).and_then(|x| x.ms);
                let d = match (r.ms, cur) {
                    (Some(a), Some(b)) => format!("  {}", fmt_delta(a - b)),
                    (Some(_), None) => "  newly reached".into(),
                    (None, Some(_)) => "  LOST".into(),
                    _ => String::new(),
                };
                v.push(
                    prop(g.name.clone(), self.ro(format!("{}{d}", r.ms.map_or("miss".into(), fmt_ms))))
                        .into_any_element(),
                );
            }
        }
        v.push(prop("Replay check", div().text_size(px(theme::FS_BODY)).text_color(c(theme::OK)).child(format!("\u{2713} tas replayed it from a fresh runner: same score, body bits, trajectory digest{}", self.checks.get(&id).map_or(String::new(), |m| format!(" \u{00b7} {m}"))))).into_any_element());
        v.push(self.candidate_actions(id, cx));
        v
    }

    fn ground_below(&self, p: Vec3) -> Option<f32> {
        self.hover_hit.filter(|h| (h.1.x - p.x).abs() < 0.5 && (h.1.z - p.z).abs() < 0.5).map(|h| h.1.y)
    }
}
