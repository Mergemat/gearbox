use glam::Vec3;
use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, Context, FontWeight, InteractiveElement, IntoElement, ParentElement,
    StatefulInteractiveElement, Styled, div, px,
};

use crate::app::*;
use crate::fmt::fmt_ms;
use crate::theme::{self, c};
use crate::ui::widgets::*;

fn section(children: Vec<AnyElement>) -> AnyElement {
    div()
        .mt(px(theme::SP2))
        .pt(px(theme::SP2))
        .pb(px(theme::SP3))
        .px(px(theme::PAD))
        .border_t_1()
        .border_color(c(theme::HAIRLINE))
        .flex()
        .flex_col()
        .gap(px(theme::SP2))
        .children(children)
        .into_any_element()
}

fn title(s: &str) -> AnyElement {
    div()
        .h(px(24.0))
        .flex()
        .items_center()
        .text_size(px(theme::FS_EMPH))
        .font_weight(FontWeight::SEMIBOLD)
        .child(s.to_string())
        .into_any_element()
}

fn stat(value: String, unit: &str, what: &str) -> gpui::Div {
    div()
        .flex_1()
        .flex()
        .flex_col()
        .gap(px(2.0))
        .child(
            div().flex().items_baseline().gap(px(3.0)).child(mono(value, 18.0)).child(
                div().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child(unit.to_string()),
            ),
        )
        .child(div().text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(what.to_string()))
}

impl Editor {
    pub fn car_props(&mut self, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let Some(f) = self.sim.frames.get(self.cursor).copied() else {
            v.push(prop("", muted("No frames yet.")).into_any_element());
            return v;
        };
        let vel = Vec3::from(f.velocity());
        let fwd = Vec3::from(f.axis(2));
        let drift = if vel.length() > 1.0 { vel.angle_between(fwd).to_degrees() } else { 0.0 };
        let cps = self.route_stops().iter().filter(|s| matches!(s.at, crate::lab::RouteAt::Gate(_))).count();
        v.push(
            div()
                .px(px(theme::PAD))
                .pb(px(theme::SP1))
                .text_size(px(theme::FS_SMALL))
                .font_family(theme::MONO)
                .text_color(c(theme::MUTED))
                .child(format!("{} s \u{00b7} tick {}", fmt_ms(f.race_ms), self.cursor))
                .into_any_element(),
        );
        v.push(
            div()
                .px(px(theme::PAD))
                .pt(px(theme::SP2))
                .pb(px(theme::SP2))
                .flex()
                .gap(px(theme::SP3))
                .child(stat(format!("{:.0}", vel.length() * 3.6), "km/h", "Speed"))
                .child(stat(format!("{drift:.1}"), "\u{00b0}", "Drift"))
                .child(stat(
                    if cps > 0 { format!("{}/{cps}", f.waypoints) } else { f.waypoints.to_string() },
                    "",
                    "Checkpoints",
                ))
                .into_any_element(),
        );
        let w = f.wheels;
        let wheel = |k: usize, left: f32, top: f32| {
            let on = w[k].contact;
            div()
                .absolute()
                .left(px(left))
                .top(px(top))
                .w(px(8.0))
                .h(px(14.0))
                .rounded(px(2.0))
                .border_1()
                .border_color(c(if on { theme::OK } else { theme::FAINT }))
                .when(on, |d| d.bg(c(theme::OK)))
        };
        let diagram = div()
            .relative()
            .w(px(44.0))
            .h(px(60.0))
            .flex_none()
            .child(
                div()
                    .absolute()
                    .left(px(10.0))
                    .top(px(2.0))
                    .w(px(24.0))
                    .h(px(56.0))
                    .rounded(px(8.0))
                    .border_1()
                    .border_color(c(theme::FAINT)),
            )
            .child(wheel(0, 1.0, 7.0))
            .child(wheel(1, 35.0, 7.0))
            .child(wheel(3, 1.0, 40.0))
            .child(wheel(2, 35.0, 40.0));
        let surf = |k: usize| crate::materials::surface_name(w[k].material).to_string();
        let pair = |a: usize, b: usize, name: &str| -> Vec<String> {
            match (w[a].contact, w[b].contact) {
                (true, true) if surf(a) == surf(b) => vec![format!("{name} wheels on {}", surf(a))],
                (false, false) => vec![format!("{name} wheels in the air")],
                _ => [(a, "left"), (b, "right")]
                    .iter()
                    .map(|(k, side)| {
                        if w[*k].contact {
                            format!("{name} {side} on {}", surf(*k))
                        } else {
                            format!("{name} {side} in the air")
                        }
                    })
                    .collect(),
            }
        };
        let lines: Vec<String> = pair(0, 1, "Front").into_iter().chain(pair(3, 2, "Rear")).collect();
        v.push(section(vec![
            div()
                .flex()
                .items_center()
                .gap(px(theme::SP4))
                .child(diagram)
                .child(
                    div().flex().flex_col().gap(px(3.0)).children(
                        lines.into_iter().map(|l| {
                            div().text_size(px(theme::FS_BODY)).text_color(c(theme::MUTED)).child(l)
                        }),
                    ),
                )
                .into_any_element(),
        ]));
        let ctl = f.controls;
        let pedal = |name: &'static str, on: bool, col: u32| {
            div()
                .px(px(theme::SP2))
                .h(px(20.0))
                .flex()
                .items_center()
                .rounded(px(10.0))
                .text_size(px(theme::FS_SMALL))
                .when(on, |d| d.bg(theme::alpha(col, 0.18)).text_color(c(col)))
                .when(!on, |d| d.border_1().border_color(c(theme::HAIRLINE)).text_color(c(theme::FAINT)))
                .child(name)
        };
        let s = ctl.steer.clamp(-1.0, 1.0);
        let steer = div()
            .relative()
            .flex_1()
            .h(px(8.0))
            .rounded(px(4.0))
            .bg(c(theme::WELL))
            .border_1()
            .border_color(c(theme::HAIRLINE))
            .child(div().absolute().left(gpui::relative(0.5)).top_0().w(px(1.0)).h_full().bg(c(theme::FAINT)))
            .child(
                div()
                    .absolute()
                    .top(px(1.0))
                    .h(px(4.0))
                    .rounded(px(2.0))
                    .bg(c(theme::TEXT))
                    .left(gpui::relative(0.5 + s.min(0.0) * 0.5))
                    .w(gpui::relative(s.abs() * 0.5)),
            );
        v.push(section(vec![
            div()
                .flex()
                .items_center()
                .gap(px(theme::SP2))
                .child(pedal("Gas", ctl.gas > 0.0, theme::OK))
                .child(pedal("Brake", ctl.brake > 0.0, theme::ERROR))
                .child(div().w(px(theme::SP2)))
                .child(steer)
                .child(
                    mono(format!("{:+.2}", ctl.steer), theme::FS_SMALL)
                        .w(px(40.0))
                        .text_color(c(theme::MUTED)),
                )
                .into_any_element(),
        ]));
        v.push(section(vec![div()
            .flex()
            .flex_wrap()
            .gap(px(theme::SP2))
            .child(btn("car-start", "Lab car", Btn::Normal).tooltip(tip("A car here", Some("Places a car in the lab with this pose and velocity, to solve from or drag about."))).on_click(cx.listener(|this, _, _, cx| {
                if let Some(s) = this.start_from_cursor() {
                    this.place_start(s);
                }
                cx.notify();
            })))
            .child(btn("car-pose", "Pose goal", Btn::Normal).tooltip(tip("A car pose goal here", Some("A goal at this place, angle and speed, to reach from elsewhere."))).on_click(cx.listener(|this, _, _, cx| {
                this.pose_goal_from_cursor();
                cx.notify();
            })))
            .child(btn("car-setup", "Setup", Btn::Normal).tooltip(tip("A setup from here", Some("Keeps this car and the run's inputs for the length beside it as a setup."))).on_click(cx.listener(|this, _, _, cx| {
                let n = this.setup_ticks;
                this.save_setup_from_run(n);
                cx.notify();
            })))
            .child(self.duration_field("car-setup-len", self.setup_ticks, |e, v, _| e.setup_ticks = v, cx))
            .child(checkbox("what-if", self.show.branches, "What if").tooltip(tip("What if", Some("Three 1.5 s futures from here with full left, straight and full right steering."))).on_click(cx.listener(|this, _, _, cx| {
                this.show.branches = !this.show.branches;
                this.sim.probe(this.cursor, this.show.branches);
                this.dirty = true;
                cx.notify();
            })))
            .into_any_element()]));
        let open = !self.collapsed.contains("car-transform");
        let head = div()
            .id("car-transform-h")
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .cursor_pointer()
            .child(
                div().w(px(10.0)).text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(if open {
                    "\u{25be}"
                } else {
                    "\u{25b8}"
                }),
            )
            .child(title("Transform"))
            .child(div().flex_1())
            .child(div().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child("world"))
            .on_click(cx.listener(|this, _, _, cx| {
                if !this.collapsed.remove("car-transform") {
                    this.collapsed.insert("car-transform".into());
                }
                cx.notify();
            }));
        let mut tr: Vec<AnyElement> = vec![head.into_any_element()];
        if open {
            let cell = |s: String| {
                div()
                    .flex_1()
                    .h(px(22.0))
                    .flex()
                    .items_center()
                    .justify_center()
                    .font_family(theme::MONO)
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(theme::MUTED))
                    .child(s)
            };
            let row3 = |name: &str, x: [f32; 3], d: usize| {
                div()
                    .flex()
                    .items_center()
                    .gap(px(theme::SP1))
                    .child(
                        div()
                            .w(px(72.0))
                            .flex_none()
                            .text_size(px(theme::FS_BODY))
                            .text_color(c(theme::MUTED))
                            .child(name.to_string()),
                    )
                    .children(x.map(|a| cell(format!("{a:.d$}"))))
            };
            let p = f.position();
            let ypr = f.yaw_pitch_roll();
            let av = Vec3::from(f.angular_velocity());
            tr.push(
                div()
                    .flex()
                    .gap(px(theme::SP1))
                    .child(div().w(px(72.0)).flex_none())
                    .children(["X", "Y", "Z"].map(|a| {
                        div()
                            .flex_1()
                            .flex()
                            .justify_center()
                            .text_size(px(theme::FS_SMALL))
                            .text_color(c(theme::FAINT))
                            .child(a)
                    }))
                    .into_any_element(),
            );
            tr.push(row3("Position", p, 2).into_any_element());
            tr.push(row3("Rotation", ypr, 1).into_any_element());
            tr.push(row3("Velocity", vel.into(), 2).into_any_element());
            tr.push(
                row3("Spin", [av.x.to_degrees(), av.y.to_degrees(), av.z.to_degrees()], 1).into_any_element(),
            );
            if let Some(pr) = self.sim.probe.as_ref().filter(|p| p.tick == self.cursor) {
                tr.push(
                    muted(format!(
                        "Full state restored in {:.2} ms ({} ticks from a savestate)",
                        pr.micros as f32 / 1000.0,
                        pr.stepped
                    ))
                    .text_size(px(theme::FS_SMALL))
                    .into_any_element(),
                );
            }
        }
        v.push(section(tr));
        v
    }

    pub fn current_run_props(&mut self, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        let fin = self.sim.frames.iter().find_map(|f| f.finish).map(|t| fmt_ms(t as i32));
        v.push(
            div()
                .px(px(theme::PAD))
                .pt(px(theme::SP2))
                .pb(px(theme::SP2))
                .flex()
                .gap(px(theme::SP3))
                .child(stat(fin.unwrap_or("\u{2014}".into()), "s", "Finish"))
                .child(stat(format!("{:.2}", self.doc.len as f32 / 100.0), "s", "Length"))
                .child(stat(self.doc.inputs.trimmed().controls.len().to_string(), "ticks", "Inputs"))
                .into_any_element(),
        );
        let name = self.doc.run_name.clone();
        let nf =
            self.text_field("run.name", &name, |e, s, _| e.edit("rename run", None, |d| d.run_name = s), cx);
        v.push(prop("Name", nf).into_any_element());
        let len = self.doc.len;
        let lf = self.duration_field(
            "run.len",
            len,
            |e, n, _| e.edit("run length", Some("run.len"), |d| d.len = n),
            cx,
        );
        v.push(prop("Length", lf).into_any_element());
        v.push(prop("Origin", self.ro(self.doc.run_origin.label())).into_any_element());
        let i = &self.doc.inputs;
        v.push(
            prop(
                "Replay",
                self.ro(format!(
                    "T0 {} ms \u{00b7} {}",
                    i.t0,
                    i.seed.map_or("raw injection".into(), |s| format!("seed {s}"))
                )),
            )
            .into_any_element(),
        );
        v.push(prop("Savestates", self.ro(self.sim.savestates.to_string())).into_any_element());
        v.push(
            div()
                .px(px(theme::PAD))
                .pt(px(theme::SP3))
                .flex()
                .flex_wrap()
                .gap(px(theme::SP2))
                .child(
                    btn("run-keep", "Keep a copy", Btn::Normal)
                        .tooltip(tip("Keep as an alternative run", None))
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.edit("keep run", None, |d| {
                                let mut r = d.current(d.run_origin.clone());
                                r.name = format!("{} copy", d.run_name);
                                d.runs.push(r);
                            });
                            cx.notify();
                        })),
                )
                .child(
                    btn("run-save-in", "Export inputs", Btn::Normal)
                        .tooltip(tip(
                            "Write inputs/<name>.txt.in in the editor folder",
                            Some("Trailing neutral ticks trimmed."),
                        ))
                        .on_click(cx.listener(|this, _, _, cx| {
                            let p = this.home.join("inputs").join(format!("{}.txt.in", this.project_name));
                            match std::fs::create_dir_all(this.home.join("inputs"))
                                .map_err(anyhow::Error::from)
                                .and_then(|()| this.doc.inputs.trimmed().write(&p))
                            {
                                Ok(()) => this.info("io", format!("wrote {}", p.display())),
                                Err(e) => this.error("io", format!("{e:#}")),
                            }
                            cx.notify();
                        })),
                )
                .child(
                    btn("run-clear", "Clear", Btn::Danger)
                        .tooltip(tip("A new, empty run (undoable)", None))
                        .on_click(cx.listener(|this, _, _, cx| {
                            let ms0 = this.race_ms0;
                            this.replace_run(
                                "clear run",
                                crate::engine::input::InputFile::blank(ms0),
                                crate::doc::Origin::Blank,
                            );
                            cx.notify();
                        })),
                )
                .into_any_element(),
        );
        v
    }
}
