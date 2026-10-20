use gpui::StatefulInteractiveElement;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use gpui::{
    App, Bounds, Context, DispatchPhase, FontWeight, Hitbox, HitboxBehavior, IntoElement, MouseDownEvent,
    ParentElement, Path, PathBuilder, Pixels, Styled, Window, canvas, div, fill, point, px, size,
};

use crate::engine::frame::Frame;
use crate::replay::GhostTrack;
use glam::Vec3;

use crate::app::*;
use crate::doc::{Goal, GoalKind};
use crate::fmt::{fmt_delta, fmt_ms};
use crate::theme::{self, c};
use crate::ui::widgets::*;

pub type Track = Vec<(i32, Vec3, f32)>;

pub fn frames_track(f: &[Frame]) -> Track {
    f.iter().map(|f| (f.race_ms, Vec3::from(f.position()), f.speed_kmh())).collect()
}

pub fn ghost_track(g: &GhostTrack) -> Track {
    g.samples.iter().map(|s| (s.time_ms, Vec3::from(s.position), s.speed * 3.6)).collect()
}

pub fn time_delta(a: &Track, b: &Track) -> Vec<Option<f32>> {
    let mut j = 0usize;
    a.iter()
        .map(|&(ta, pa, _)| {
            if b.len() < 2 {
                return None;
            }
            let lo = j.saturating_sub(40);
            let hi = (j + 400).min(b.len() - 1);
            let mut best: Option<(f32, f32, usize)> = None;
            for i in lo..hi {
                let (t0, p0, _) = b[i];
                let (t1, p1, _) = b[i + 1];
                let ab = p1 - p0;
                let u = if ab.length_squared() > 0.0 {
                    ((pa - p0).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let d = pa.distance(p0 + ab * u);
                if best.is_none_or(|b| d < b.0) {
                    best = Some((d, t0 as f32 + (t1 - t0) as f32 * u, i));
                }
            }
            let (d, tb, i) = best?;
            j = i;
            (d <= 25.0).then_some(ta as f32 - tb)
        })
        .collect()
}

pub fn ghost_reach(goals: &[Goal], g: &GhostTrack, race_ms0: i32) -> Vec<Option<i32>> {
    let end = g.samples.last().map_or(race_ms0, |s| s.time_ms + 50);
    let mut from = race_ms0;
    let mut chain = true;
    goals
        .iter()
        .map(|goal| {
            if !chain {
                return None;
            }
            let hit = (from..end).step_by(10).find(|&t| {
                let Some((p, q, _)) = g.at(t) else { return false };
                let p = Vec3::from(p);
                match &goal.kind {
                    GoalKind::Point { pos, radius } => p.distance(Vec3::from(*pos)) <= *radius,
                    GoalKind::Box { pos, size, ypr } => {
                        crate::doc::trigger_box(*pos, *size, *ypr).contains(p.into())
                    }
                    GoalKind::Pose(s) => {
                        let pose = gearbox_search::pose::Pose {
                            position: p.into(),
                            rotation: [q[3], q[0], q[1], q[2]],
                            velocity: s.velocity,
                        };
                        let mut goal = s.goal();
                        goal.velocity_tolerance = None;
                        goal.contains(pose)
                    }
                    GoalKind::Gate(gate) => {
                        gate.boxes.iter().any(|b| (0..3).all(|k| b.min[k] <= p[k] && p[k] <= b.max[k]))
                    }
                }
            });
            match hit {
                Some(t) => {
                    from = t;
                    Some(t)
                }
                None => {
                    chain = false;
                    None
                }
            }
        })
        .collect()
}

#[derive(Clone)]
pub enum Side {
    Frames(Arc<Vec<Frame>>),
    Ghost(usize, usize, String),
}

impl PartialEq for Side {
    fn eq(&self, o: &Self) -> bool {
        match (self, o) {
            (Side::Frames(a), Side::Frames(b)) => Arc::ptr_eq(a, b),
            (Side::Ghost(a, n, s), Side::Ghost(b, m, t)) => a == b && n == m && s == t,
            _ => false,
        }
    }
}

pub struct Compared {
    a: Track,
    b: Track,
    delta: Vec<Option<f32>>,
    speed_b: Vec<Option<f32>>,
}

#[derive(Default)]
pub struct GraphPaths(RefCell<Option<(Rc<Compared>, Bounds<Pixels>, Vec<(Path<Pixels>, gpui::Hsla)>)>>);

struct Graph {
    data: Rc<Compared>,
    paths: Rc<GraphPaths>,
    cursor_ms: i32,
}

impl Editor {
    fn side(&mut self, r: RunRef) -> Option<Side> {
        match r {
            RunRef::Ghost(i) => {
                let g = &self.ghosts.get(i)?.track;
                Some(Side::Ghost(g.samples.as_ptr() as usize, g.samples.len(), g.name.clone()))
            }
            r => self.ref_frames(r).map(Side::Frames),
        }
    }

    fn side_track(&self, s: &Side) -> Option<Track> {
        match s {
            Side::Frames(f) => Some(frames_track(f)),
            Side::Ghost(p, n, _) => self
                .ghosts
                .iter()
                .find(|g| g.track.samples.as_ptr() as usize == *p && g.track.samples.len() == *n)
                .map(|g| ghost_track(&g.track)),
        }
    }

    fn compared(&mut self, a: RunRef, b: RunRef) -> Option<Rc<Compared>> {
        let (sa, sb) = (self.side(a)?, self.side(b)?);
        let (ta, tb) = (self.side_track(&sa)?, self.side_track(&sb)?);
        Some(self.memo.compare.get((sa, sb), || {
            let delta = time_delta(&ta, &tb);
            let speed_b = ta
                .iter()
                .map(|&(t, _, _)| {
                    let i = tb.partition_point(|x| x.0 <= t);
                    (i > 0 && i < tb.len()).then(|| tb[i - 1].2)
                })
                .collect();
            Compared { a: ta, b: tb, delta, speed_b }
        }))
    }

    fn compare_choices(&self) -> Vec<(String, RunRef)> {
        let mut v = vec![
            (format!("{} (current)", self.doc.run_name), RunRef::Current),
            ("Previous version".to_string(), RunRef::Previous),
        ];
        for r in &self.doc.runs {
            v.push((r.name.clone(), RunRef::Run(r.id)));
        }
        for (i, g) in self.ghosts.iter().enumerate() {
            v.push((format!("Ghost {}", g.track.name), RunRef::Ghost(i)));
        }
        if let Some(id) = self.preview {
            v.push((format!("Candidate #{id}"), RunRef::Candidate(id)));
        }
        for r in self.doc.history.iter().rev().take(10) {
            v.push((format!("History: {}", r.name), RunRef::Run(r.id)));
        }
        v
    }

    pub fn compare_panel(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let choices = self.compare_choices();
        let (a, b) = self.compare;
        let sa = self.dropdown("cmp.a", a, choices.clone(), |e, v, _| e.compare.0 = v, cx);
        let sb = self.dropdown("cmp.b", b, choices, |e, v, _| e.compare.1 = v, cx);
        let data = self.compared(a, b);
        let header = div()
            .h(px(30.0))
            .flex_none()
            .px(px(theme::PAD))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .child(
                div()
                    .px(px(theme::SP1))
                    .rounded(px(2.0))
                    .bg(c(theme::TEXT))
                    .text_color(c(theme::BG))
                    .text_size(px(theme::FS_SMALL))
                    .font_weight(FontWeight::SEMIBOLD)
                    .child("A"),
            )
            .child(div().w(px(220.0)).child(sa))
            .child(
                div()
                    .px(px(theme::SP1))
                    .rounded(px(2.0))
                    .bg(c(theme::GHOST))
                    .text_color(c(theme::BG))
                    .text_size(px(theme::FS_SMALL))
                    .font_weight(FontWeight::SEMIBOLD)
                    .child("B"),
            )
            .child(div().w(px(220.0)).child(sb))
            .child(btn("cmp-swap", "Swap", Btn::Quiet).on_click(cx.listener(|e, _, _, cx| {
                e.compare = (e.compare.1, e.compare.0);
                cx.notify();
            })));
        let Some(data) = data else {
            return div()
                .size_full()
                .flex()
                .flex_col()
                .child(header)
                .child(
                    div()
                        .p(px(theme::PAD))
                        .child(muted("Simulating the runs to compare \u{2026} (or pick two runs)")),
                )
                .into_any_element();
        };
        let (ta, tb, delta) = (&data.a, &data.b, &data.delta);
        let fin = |t: &Track| t.last().map(|x| x.0);
        let finish_a = match a {
            RunRef::Ghost(i) => self.ghosts.get(i).and_then(|g| g.track.race_time_ms),
            r => self.ref_frames(r).and_then(|f| f.iter().find_map(|f| f.finish)).map(|t| t as i32),
        };
        let finish_b = match b {
            RunRef::Ghost(i) => self.ghosts.get(i).and_then(|g| g.track.race_time_ms),
            r => self.ref_frames(r).and_then(|f| f.iter().find_map(|f| f.finish)).map(|t| t as i32),
        };
        let (mut gain, mut loss) = (0.0f32, 0.0f32);
        for d in delta.iter().flatten() {
            gain = gain.min(*d);
            loss = loss.max(*d);
        }
        let cur = self.sim.frames.get(self.cursor).map_or(self.race_ms0, |f| f.race_ms);
        let at_cursor = ta.iter().position(|x| x.0 >= cur).and_then(|i| delta.get(i).copied().flatten());
        let summary = div()
            .px(px(theme::PAD))
            .h(px(24.0))
            .flex()
            .items_center()
            .gap(px(theme::SP3))
            .child(mono(format!("A finish {}", finish_a.map_or("\u{2014}".into(), fmt_ms)), 10.0))
            .child(
                mono(format!("B finish {}", finish_b.map_or("\u{2014}".into(), fmt_ms)), 10.0)
                    .text_color(c(theme::GHOST)),
            )
            .children(match (finish_a, finish_b) {
                (Some(x), Some(y)) => Some(
                    mono(format!("A\u{2212}B {}", fmt_delta(x - y)), theme::FS_SMALL)
                        .text_color(c(if x <= y { theme::OK } else { theme::ERROR })),
                ),
                _ => None,
            })
            .child(mono(
                format!("at cursor {}", at_cursor.map_or("\u{2014}".into(), |d| fmt_delta(d.round() as i32))),
                10.0,
            ))
            .child(
                mono(
                    format!(
                        "most ahead {}  most behind {}",
                        fmt_delta(gain.round() as i32),
                        fmt_delta(loss.round() as i32)
                    ),
                    10.0,
                )
                .text_color(c(theme::MUTED)),
            )
            .child(div().flex_1())
            .child(muted(format!(
                "{} vs {} samples \u{00b7} ends {} / {}",
                ta.len(),
                tb.len(),
                fin(ta).map_or("".into(), fmt_ms),
                fin(tb).map_or("".into(), fmt_ms)
            )));
        let g = Graph { data: data.clone(), paths: self.memo.compare_paths.clone(), cursor_ms: cur };
        let this = cx.entity();
        let graph = canvas(
            |b, window, _| window.insert_hitbox(b, HitboxBehavior::Normal),
            move |b: Bounds<Pixels>, hb: Hitbox, window: &mut Window, _: &mut App| {
                paint_graph(window, b, &g);
                let t = this.clone();
                let (t0, t1) = (g.data.a.first().map_or(0, |x| x.0), g.data.a.last().map_or(1, |x| x.0));
                window.on_mouse_event(move |e: &MouseDownEvent, phase, window, cx| {
                    if phase == DispatchPhase::Bubble && hb.is_hovered(window) {
                        let f = ((f32::from(e.position.x) - f32::from(b.origin.x) - 40.0)
                            / (f32::from(b.size.width) - 50.0))
                            .clamp(0.0, 1.0);
                        let ms = t0 + ((t1 - t0) as f32 * f) as i32;
                        t.update(cx, |ed, cx| {
                            let k = ((ms - ed.race_ms0) / 10).max(0) as usize;
                            ed.playing = false;
                            ed.set_cursor(k);
                            cx.notify();
                        });
                    }
                });
            },
        )
        .size_full();
        div()
            .size_full()
            .flex()
            .flex_col()
            .child(header)
            .child(summary)
            .child(
                div()
                    .flex_1()
                    .min_h_0()
                    .mx(px(theme::PAD))
                    .mb(px(theme::SP2))
                    .rounded(px(theme::R_CTL))
                    .bg(c(theme::WELL))
                    .child(graph),
            )
            .into_any_element()
    }
}

fn paint_graph(window: &mut Window, b: Bounds<Pixels>, g: &Graph) {
    let (x0, y0) = (f32::from(b.origin.x) + 40.0, f32::from(b.origin.y) + 8.0);
    let (w, h) = (f32::from(b.size.width) - 50.0, f32::from(b.size.height) - 16.0);
    let a = &g.data.a;
    if a.len() < 2 || w <= 0.0 {
        return;
    }
    let rect = |x: f32, y: f32, ww: f32, hh: f32| Bounds {
        origin: point(px(x), px(y)),
        size: size(px(ww.max(0.0)), px(hh.max(0.0))),
    };
    let (t0, t1) = (a[0].0 as f32, a.last().unwrap().0 as f32);
    let xt = |t: f32| x0 + (t - t0) / (t1 - t0).max(1.0) * w;
    let top_mid = y0 + h * 0.3;
    let bot_mid = y0 + h * 0.78;
    window.paint_quad(fill(rect(x0, top_mid, w, 1.0), theme::alpha(theme::HAIRLINE, 1.0)));
    window.paint_quad(fill(rect(x0, bot_mid, w, 1.0), theme::alpha(theme::HAIRLINE, 1.0)));
    let mut cache = g.paths.0.borrow_mut();
    if !cache.as_ref().is_some_and(|(d, cb, _)| Rc::ptr_eq(d, &g.data) && *cb == b) {
        *cache = Some((g.data.clone(), b, graph_paths(&g.data, b)));
    }
    for (path, color) in &cache.as_ref().unwrap().2 {
        window.paint_path(path.clone(), *color);
    }
    let cx = xt(g.cursor_ms as f32);
    if cx >= x0 && cx <= x0 + w {
        window.paint_quad(fill(rect(cx - 0.5, y0, 1.0, h), theme::alpha(theme::TEXT, 0.8)));
    }
}

fn graph_paths(g: &Compared, b: Bounds<Pixels>) -> Vec<(Path<Pixels>, gpui::Hsla)> {
    let (x0, y0) = (f32::from(b.origin.x) + 40.0, f32::from(b.origin.y) + 8.0);
    let (w, h) = (f32::from(b.size.width) - 50.0, f32::from(b.size.height) - 16.0);
    let a = &g.a;
    let (t0, t1) = (a[0].0 as f32, a.last().unwrap().0 as f32);
    let xt = |t: f32| x0 + (t - t0) / (t1 - t0).max(1.0) * w;
    let dmax = g.delta.iter().flatten().fold(20.0f32, |m, d| m.max(d.abs())) * 1.1;
    let top_mid = y0 + h * 0.3;
    let dy = |d: f32| top_mid - d / dmax * (h * 0.28);
    let mut out = Vec::new();
    let mut segs: Vec<(f32, f32, f32)> = Vec::new();
    for (i, d) in g.delta.iter().enumerate() {
        if let Some(d) = d {
            segs.push((xt(a[i].0 as f32), dy(*d), *d));
        }
    }
    let mut run: Option<(bool, PathBuilder)> = None;
    for w2 in segs.windows(2) {
        let behind = w2[1].2 > 0.0;
        if run.as_ref().is_some_and(|(r, _)| *r != behind)
            && let Some((r, p)) = run.take()
            && let Ok(path) = p.build()
        {
            out.push((path, theme::alpha(if r { theme::ERROR } else { theme::OK }, 0.95)));
        }
        let (_, p) = run.get_or_insert_with(|| {
            let mut p = PathBuilder::stroke(px(1.5));
            p.move_to(point(px(w2[0].0), px(w2[0].1)));
            (behind, p)
        });
        p.line_to(point(px(w2[1].0), px(w2[1].1)));
    }
    if let Some((r, p)) = run
        && let Ok(path) = p.build()
    {
        out.push((path, theme::alpha(if r { theme::ERROR } else { theme::OK }, 0.95)));
    }
    let bot_mid = y0 + h * 0.78;
    let smax =
        a.iter().zip(&g.speed_b).filter_map(|(a, b)| b.map(|b| (a.2 - b).abs())).fold(10.0f32, f32::max)
            * 1.1;
    let mut p = PathBuilder::stroke(px(1.0));
    let mut started = false;
    for (a, b) in a.iter().zip(&g.speed_b) {
        let Some(bv) = b else { continue };
        let x = xt(a.0 as f32);
        let y = bot_mid - (a.2 - bv) / smax * (h * 0.2);
        if started {
            p.line_to(point(px(x), px(y)));
        } else {
            p.move_to(point(px(x), px(y)));
            started = true;
        }
    }
    if started && let Ok(path) = p.build() {
        out.push((path, theme::alpha(theme::GHOST, 0.9)));
    }
    out
}
