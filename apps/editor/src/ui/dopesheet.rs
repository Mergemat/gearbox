use std::ops::Range;
use std::sync::Arc;

use gearbox_engine::Controls;

use gpui::{
    App, Bounds, Context, CursorStyle, DispatchPhase, Font, FontWeight, Hitbox, HitboxBehavior,
    InteractiveElement, IntoElement, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent,
    ParentElement, PathBuilder, PinchEvent, Pixels, Point, ScrollWheelEvent, SharedString,
    StatefulInteractiveElement, Styled, TextAlign, TextRun, Window, canvas, div, fill, point, px, size,
};

use crate::app::*;
use crate::doc;
use crate::fmt::fmt_ms;
use crate::theme::{self, c};
use crate::ui::widgets::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lane {
    Ruler,
    Track,
    Plan,
    Tricks,
    Speed,
    Steer,
    Gas,
    Brake,
    Overview,
}

pub const GUTTER: f32 = 132.0;
const RULER_H: f32 = 28.0;
pub const STRIP_Y: f32 = 18.0;
const STRIP_H: f32 = 9.0;
const TRACK_H: f32 = 20.0;
const PEDAL_H: f32 = 14.0;
const PLAN_H: f32 = 20.0;
const TRICKS_H: f32 = 14.0;
const OVER_H: f32 = 16.0;
const GAP: f32 = 2.0;
const NEAR: f32 = 6.0;

#[derive(Clone, Debug)]
pub enum TlDrag {
    Scrub,
    Select { anchor: usize },
    Paint { lane: Lane, value: f32, last: Option<(usize, f32)>, stroke: u64 },
    Pan { last_x: f32, start0: f64 },
    Window { edge: u8, anchor: usize, w0: Range<usize> },
    Split,
    Overview { edge: u8, grab: f64, start0: f64, end0: f64 },
}

#[derive(Default)]
pub struct TlView {
    pub start: f64,
    pub ppt: f64,
    pub fit: bool,
    pub bounds: Bounds<Pixels>,
    pub channel: usize,
    pub strokes: u64,
    pub hover: Option<(usize, Lane)>,
    pub split: f32,
}

impl TlView {
    fn split(&self) -> f32 {
        if self.split > 0.0 { self.split } else { 0.55 }
    }

    pub fn width(&self) -> f32 {
        (f32::from(self.bounds.size.width) - GUTTER - 8.0).max(200.0)
    }
}

fn lanes(h: f32, extra: (bool, bool), split: f32) -> Vec<(Lane, f32, f32)> {
    let (plan, tricks) = (if extra.0 { PLAN_H } else { 0.0 }, if extra.1 { TRICKS_H } else { 0.0 });
    let fixed = RULER_H + TRACK_H + plan + tricks + 2.0 * PEDAL_H + OVER_H + 9.0 * GAP;
    let flex = (h - fixed).max(40.0);
    let order = [
        (Lane::Ruler, RULER_H),
        (Lane::Track, TRACK_H),
        (Lane::Plan, plan),
        (Lane::Tricks, tricks),
        (Lane::Speed, (flex * split).round()),
        (Lane::Steer, (flex * (1.0 - split)).round()),
        (Lane::Gas, PEDAL_H),
        (Lane::Brake, PEDAL_H),
        (Lane::Overview, OVER_H),
    ];
    let mut y = 0.0;
    order
        .iter()
        .map(|(l, hh)| {
            let r = (*l, y, *hh);
            y += hh
                + match l {
                    _ if *hh == 0.0 => 0.0,
                    Lane::Ruler => 0.0,
                    Lane::Brake => 3.0 * GAP,
                    _ => GAP,
                };
            r
        })
        .collect()
}

fn lane_name(l: Lane) -> (&'static str, Option<u32>) {
    match l {
        Lane::Ruler => ("", None),
        Lane::Track => ("Splits", None),
        Lane::Plan => ("Plan", Some(theme::SETUP)),
        Lane::Tricks => ("Tricks", Some(theme::WARN)),
        Lane::Speed => ("Speed", Some(theme::SPEED)),
        Lane::Steer => ("Steer", Some(theme::STEER)),
        Lane::Gas => ("Gas", Some(theme::OK)),
        Lane::Brake => ("Brake", Some(theme::ERROR)),
        Lane::Overview => ("Overview", None),
    }
}

#[derive(Clone)]
struct Mark {
    k: usize,
    name: String,
    color: u32,
    solid: bool,
    delta: Option<i32>,
}

pub struct Paint {
    len: usize,
    simulated: usize,
    race_ms0: i32,
    overview: (f64, f64),
    start: f64,
    ppt: f64,
    split: f32,
    speed: Arc<Vec<f32>>,
    b_speed: Option<Vec<(i32, f32)>>,
    b_label: Option<&'static str>,
    controls: Vec<Controls>,
    preview: Option<(Vec<Controls>, Vec<Range<usize>>)>,
    sel: Option<Range<usize>>,
    locks: Vec<Range<usize>>,
    window: Range<usize>,
    t0_tick: usize,
    start_tick: Option<usize>,
    marks: Vec<Mark>,
    ghost_ticks: Vec<usize>,
    paint_tool: bool,
    channel: usize,
    route: Option<(usize, usize, String)>,
    clips: Vec<(usize, usize, String, bool)>,
    tricks: Vec<crate::lab::TrickSpan>,
    extra: (bool, bool),
}

fn shape(window: &mut Window, s: &str, sz: f32, color: u32, alpha: f32, mono: bool) -> gpui::ShapedLine {
    let font = Font {
        family: SharedString::from(if mono { theme::MONO } else { theme::SANS }),
        weight: FontWeight::NORMAL,
        ..Default::default()
    };
    let run = TextRun {
        len: s.len(),
        font,
        color: theme::alpha(color, alpha),
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    window.text_system().shape_line(SharedString::from(s.to_string()), px(sz), &[run], None)
}

fn text(
    window: &mut Window,
    cx: &mut App,
    s: &str,
    x: f32,
    y: f32,
    sz: f32,
    color: u32,
    alpha: f32,
    mono: bool,
) -> f32 {
    let line = shape(window, s, sz, color, alpha, mono);
    let w = f32::from(line.width);
    paint_text(window, cx, line, point(px(x), px(y)), px(sz * 1.3));
    w
}

fn text_r(
    window: &mut Window,
    cx: &mut App,
    s: &str,
    x: f32,
    y: f32,
    sz: f32,
    color: u32,
    alpha: f32,
    mono: bool,
) {
    let line = shape(window, s, sz, color, alpha, mono);
    let w = f32::from(line.width);
    paint_text(window, cx, line, point(px(x - w), px(y)), px(sz * 1.3));
}

#[derive(Clone)]
enum Prim {
    Quad(gpui::PaintQuad),
    Path(gpui::Path<Pixels>, gpui::Hsla),
    Text(gpui::ShapedLine, Point<Pixels>, Pixels),
}

thread_local! {
    static RECORDING: std::cell::RefCell<Option<Vec<Prim>>> = const { std::cell::RefCell::new(None) };
}

fn record(p: impl FnOnce() -> Prim) {
    RECORDING.with(|r| {
        if let Some(v) = r.borrow_mut().as_mut() {
            v.push(p());
        }
    });
}

fn quad(window: &mut Window, q: gpui::PaintQuad) {
    record(|| Prim::Quad(q.clone()));
    window.paint_quad(q);
}

fn draw_path(window: &mut Window, p: gpui::Path<Pixels>, color: gpui::Hsla) {
    record(|| Prim::Path(p.clone(), color));
    window.paint_path(p, color);
}

fn paint_text(window: &mut Window, cx: &mut App, line: gpui::ShapedLine, at: Point<Pixels>, lh: Pixels) {
    let _ = line.paint(at, lh, TextAlign::Left, None, window, cx);
    record(|| Prim::Text(line, at, lh));
}

#[derive(Default)]
pub struct LaneCache(std::cell::RefCell<Option<(std::rc::Rc<Paint>, Bounds<Pixels>, Vec<Prim>)>>);

fn paint_static(
    window: &mut Window,
    cx: &mut App,
    b: Bounds<Pixels>,
    p: &std::rc::Rc<Paint>,
    cache: &LaneCache,
) {
    let mut c = cache.0.borrow_mut();
    if let Some((cp, cb, prims)) = c.as_ref()
        && std::rc::Rc::ptr_eq(cp, p)
        && *cb == b
    {
        window.paint_layer(b, |window| {
            for prim in prims {
                match prim {
                    Prim::Quad(q) => window.paint_quad(q.clone()),
                    Prim::Path(path, color) => window.paint_path(path.clone(), *color),
                    Prim::Text(line, at, lh) => {
                        let _ = line.paint(*at, *lh, TextAlign::Left, None, window, cx);
                    }
                }
            }
        });
        return;
    }
    RECORDING.with(|r| *r.borrow_mut() = Some(Vec::new()));
    paint_lanes(window, cx, b, p, None);
    let prims = RECORDING.with(|r| r.borrow_mut().take()).unwrap_or_default();
    *c = Some((p.clone(), b, prims));
}

fn ruler_step(ppt: f64, min_px: f64) -> usize {
    for s in [1usize, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 3000, 6000, 12000, 30000] {
        if s as f64 * ppt >= min_px {
            return s;
        }
    }
    60000
}

fn sample_at(bs: &[(i32, f32)], ms: f32) -> Option<f32> {
    let i = bs.partition_point(|s| (s.0 as f32) < ms);
    if i == 0 || i >= bs.len() {
        return None;
    }
    let ((ta, va), (tb, vb)) = (bs[i - 1], bs[i]);
    let f = if tb > ta { (ms - ta as f32) / (tb - ta) as f32 } else { 0.0 };
    Some(va + (vb - va) * f)
}

impl Editor {
    fn tl_tick_at(&self, x: f32) -> usize {
        let b = self.tl.bounds;
        let lx = x - f32::from(b.origin.x) - GUTTER;
        (self.tl.start + lx as f64 / self.tl.ppt.max(1e-6)).max(self.race_start_tick() as f64) as usize
    }

    pub fn tl_lanes(&self) -> Vec<(Lane, f32, f32)> {
        let b = self.tl.bounds;
        lanes(f32::from(b.size.height), self.tl_extra(), self.tl.split())
            .into_iter()
            .map(|(l, y, h)| (l, y + f32::from(b.origin.y), h))
            .collect()
    }

    fn tl_lane_at(&self, y: f32) -> Option<(Lane, f32, f32)> {
        self.tl_lanes().into_iter().find(|(_, y0, h)| *h > 0.0 && y >= *y0 && y < y0 + h + GAP)
    }

    fn tl_on_split(&self, y: f32) -> bool {
        self.tl_lanes()
            .into_iter()
            .find(|l| l.0 == Lane::Steer)
            .is_some_and(|(_, y0, _)| (y - (y0 - GAP * 0.5)).abs() <= 3.0)
    }

    fn tl_extra(&self) -> (bool, bool) {
        let plan = !self.setup_clips().is_empty()
            || ((self.sel == Sel::Route || self.job.starts_with("Route")) && self.route_ticks().is_some());
        (plan, !self.trick_spans.is_empty())
    }

    fn tl_fit_end(&self) -> usize {
        let len = self.doc.len.max(1);
        let finish = self.sim.frames.iter().position(|f| f.finish.is_some()).map(|k| k + 100);
        let last_input = self
            .doc
            .inputs
            .controls
            .iter()
            .rposition(|c| *c != gearbox_engine::Controls::default())
            .map(|i| ((self.doc.inputs.t0 - self.race_ms0) / 10).max(0) as usize + i + 200);
        finish.or(last_input).unwrap_or(len).max(self.spec.window.end + 50).clamp(100, len.max(100))
    }

    pub fn tl_overview_span(&self) -> (f64, f64) {
        let k0 = self.race_start_tick() as f64;
        let view_end = self.tl.start + self.tl.width() as f64 / self.tl.ppt.max(1e-6);
        (k0, (self.tl_fit_end() as f64).max(view_end).max(k0 + 1.0))
    }

    fn tl_overview_tick(&self, x: f32) -> f64 {
        let (k0, k1) = self.tl_overview_span();
        let lx = (x - f32::from(self.tl.bounds.origin.x) - GUTTER) / self.tl.width();
        k0 + lx.clamp(0.0, 1.0) as f64 * (k1 - k0)
    }

    fn tl_snap(&self, k: usize, free: bool) -> usize {
        if free {
            return k;
        }
        let reach = (NEAR as f64 / self.tl.ppt.max(1e-6)).ceil() as i64;
        self.markers()
            .into_iter()
            .filter(|m| (*m as i64 - k as i64).abs() <= reach)
            .min_by_key(|m| (*m as i64 - k as i64).unsigned_abs())
            .unwrap_or(k)
    }

    pub fn fit_timeline(&mut self) {
        match self.tick_sel.clone() {
            Some(r) if r.len() > 1 && self.tl.ppt > 0.0 => {
                let pad = (r.len() as f64 * 0.08).max(5.0);
                self.tl.ppt = (self.tl.width() as f64 / (r.len() as f64 + 2.0 * pad)).clamp(0.02, 40.0);
                self.tl.start = (r.start as f64 - pad).max(self.race_start_tick() as f64);
            }
            _ => self.tl.fit = true,
        }
    }

    pub fn zoom_timeline(&mut self, f: f32) {
        let x0 = f32::from(self.tl.bounds.origin.x) + GUTTER;
        let cx = x0 + ((self.cursor as f64 - self.tl.start) * self.tl.ppt) as f32;
        let x = if cx >= x0 && cx <= x0 + self.tl.width() { cx } else { x0 + self.tl.width() * 0.5 };
        self.tl_zoom_at(x, f);
    }

    pub fn follow_playhead(&mut self) -> bool {
        let dragging_view =
            matches!(self.drag, Some(Drag::Timeline(TlDrag::Pan { .. } | TlDrag::Overview { .. })));
        if !self.playing || dragging_view || self.tl.ppt <= 0.0 {
            return false;
        }
        let vis = self.tl.width() as f64 / self.tl.ppt;
        let k = self.cursor as f64;
        if k < self.tl.start || k > self.tl.start + vis * 0.95 {
            self.tl.start = (k - vis * 0.05).max(self.race_start_tick() as f64);
            return true;
        }
        false
    }

    pub fn dopesheet(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        if self.tl.ppt <= 0.0 || self.tl.fit {
            let k0 = self.race_start_tick();
            let span = self.tl_fit_end().saturating_sub(k0).max(100);
            self.tl.ppt = self.tl.width() as f64 / span as f64;
            self.tl.start = k0 as f64;
            self.tl.fit = false;
            self.rev += 1;
        }
        if self.tl_paint.as_ref().is_none_or(|(rev, _)| *rev != self.rev) {
            let p = self.lanes_paint();
            self.tl_paint = Some((self.rev, std::rc::Rc::new(p)));
        }
        self.dopesheet_body(cx)
    }

    fn lanes_paint(&mut self) -> Paint {
        let len = self.doc.len.max(1);
        let simulated = self.sim.frames.len();
        let speed: Arc<Vec<f32>> = Arc::new(self.sim.frames.iter().map(|f| f.speed_kmh()).collect());
        let controls: Vec<Controls> = (0..len).map(|k| self.control(k)).collect();
        let preview = self.preview.and_then(|id| {
            let c = self
                .search
                .live
                .lock()
                .unwrap()
                .candidates
                .iter()
                .find(|c| c.id == id)
                .map(|c| c.inputs.clone())?;
            let diff = doc::diff_ranges(&self.doc.inputs, &c, self.race_ms0, len);
            Some(((0..len).map(|k| doc::control_at(&c, self.race_ms0, k)).collect::<Vec<_>>(), diff))
        });
        let b_frames = match self.compare.1 {
            RunRef::Run(id) => self.runs_frames.get(&crate::background::Key::Run(id)),
            RunRef::Candidate(id) => self.runs_frames.get(&crate::background::Key::Candidate(id)),
            RunRef::Previous => {
                self.doc.history.last().and_then(|h| self.runs_frames.get(&crate::background::Key::Run(h.id)))
            }
            _ => None,
        };
        let (b_speed, b_splits): (Option<Vec<(i32, f32)>>, Option<Vec<i32>>) = match self.compare.1 {
            RunRef::Ghost(i) => match self.ghosts.get(i) {
                Some(g) => (
                    Some(g.track.samples.iter().map(|s| (s.time_ms, s.speed * 3.6)).collect()),
                    Some(g.track.checkpoints_ms.clone()),
                ),
                None => (None, None),
            },
            _ => match b_frames {
                Some(f) => (
                    Some(f.iter().map(|f| (f.race_ms, f.speed_kmh())).collect()),
                    Some(
                        f.windows(2)
                            .filter(|w| w[1].waypoints > w[0].waypoints)
                            .map(|w| w[1].last_crossing.map_or(w[1].race_ms, |c| c.1 as i32))
                            .collect(),
                    ),
                ),
                None => (None, None),
            },
        };
        let b_label = b_speed.as_ref().map(|_| match self.compare.1 {
            RunRef::Ghost(_) => "ghost",
            RunRef::Previous => "previous",
            RunRef::Candidate(_) => "result",
            _ => "run",
        });
        let mut marks: Vec<Mark> = Vec::new();
        for (g, r) in self.doc.goals.iter().zip(&self.reach) {
            if let Some(k) = r.tick {
                marks.push(Mark { k, name: g.name.clone(), color: theme::PIN, solid: true, delta: None });
            } else if let Some((k, _)) = r.closest {
                marks.push(Mark {
                    k,
                    name: format!("{} (closest)", g.name),
                    color: theme::PIN,
                    solid: false,
                    delta: None,
                });
            }
        }
        let mut crossing = 0;
        for k in 1..self.sim.frames.len() {
            let (a, b) = (&self.sim.frames[k - 1], &self.sim.frames[k]);
            if b.waypoints > a.waypoints {
                let name = b.last_crossing.map_or("CP".to_string(), |(w, _)| self.gate_name(w));
                let at = b.last_crossing.map_or(b.race_ms, |c| c.1 as i32);
                let delta = b_splits.as_ref().and_then(|s| s.get(crossing)).map(|t| at - t);
                crossing += 1;
                marks.push(Mark {
                    k,
                    name,
                    color: if b.finish.is_some() && a.finish.is_none() { theme::WARN } else { theme::TEXT },
                    solid: true,
                    delta,
                });
            }
        }
        let ghost_ticks = self
            .ghosts
            .iter()
            .filter(|g| g.visible)
            .flat_map(|g| g.track.checkpoints_ms.iter().map(|t| ((t - self.race_ms0) / 10).max(0) as usize))
            .collect();
        Paint {
            len,
            simulated,
            race_ms0: self.race_ms0,
            overview: self.tl_overview_span(),
            start: self.tl.start,
            ppt: self.tl.ppt,
            split: self.tl.split(),
            speed,
            b_speed,
            b_label,
            controls,
            preview,
            sel: self.tick_sel.clone(),
            locks: self.doc.locks.clone(),
            window: self.spec.window.clone(),
            t0_tick: ((self.doc.inputs.t0 - self.race_ms0).max(0) / 10) as usize,
            start_tick: self.start_tick(),
            marks,
            ghost_ticks,
            paint_tool: self.paint_tool,
            channel: self.tl.channel,
            route: (self.sel == Sel::Route || self.job.starts_with("Route"))
                .then(|| self.route_ticks())
                .flatten()
                .map(|(a, b)| (a, b, crate::ui::route::route_title(self))),
            clips: self
                .setup_clips()
                .into_iter()
                .map(|(id, a, b, name)| (a, b, name, self.sel == Sel::Setup(id)))
                .collect(),
            tricks: self.trick_spans.clone(),
            extra: self.tl_extra(),
        }
    }

    fn dopesheet_body(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let p = self.tl_paint.as_ref().unwrap().1.clone();
        let live = (self.cursor, self.tl.hover);
        let cache = self.tl_cache.clone();
        let this = cx.entity();
        let lanes_el = canvas(
            |b, window, _| window.insert_hitbox(b, HitboxBehavior::Normal),
            move |b: Bounds<Pixels>, hb: Hitbox, window: &mut Window, cx: &mut App| {
                paint_static(window, cx, b, &p, &cache);
                paint_lanes(window, cx, b, &p, Some(live));
                let style = cursor_at(&p, b, window.mouse_position(), window.modifiers().alt);
                window.set_cursor_style(style, &hb);
                this.update(cx, |e, _| {
                    e.tl.bounds = b;
                    e.lanes = b;
                });
                let t = this.clone();
                let h = hb.clone();
                window.on_mouse_event(move |ev: &MouseDownEvent, phase, window, cx| {
                    if phase == DispatchPhase::Bubble && h.is_hovered(window) {
                        t.update(cx, |e, cx| e.timeline_down(ev, cx));
                    }
                });
                let t = this.clone();
                let h = hb.clone();
                window.on_mouse_event(move |ev: &ScrollWheelEvent, phase, window, cx| {
                    if phase == DispatchPhase::Bubble && h.is_hovered(window) {
                        t.update(cx, |e, cx| e.timeline_scroll(ev, cx));
                    }
                });
                let t = this.clone();
                let h = hb.clone();
                window.on_mouse_event(move |ev: &PinchEvent, phase, window, cx| {
                    if phase == DispatchPhase::Bubble && h.is_hovered(window) {
                        t.update(cx, |e, cx| e.timeline_zoom(f32::from(ev.position.x), 1.0 + ev.delta, cx));
                    }
                });
                let t = this.clone();
                window.on_mouse_event(move |ev: &MouseMoveEvent, phase, window, cx| {
                    if phase != DispatchPhase::Bubble || t.read(cx).drag.is_some() {
                        return;
                    }
                    let inside = hb.is_hovered(window);
                    t.update(cx, |e, cx| {
                        let h = inside
                            .then(|| {
                                let lane = e.tl_lane_at(f32::from(ev.position.y)).map(|l| l.0)?;
                                (f32::from(ev.position.x) >= f32::from(e.tl.bounds.origin.x) + GUTTER)
                                    .then(|| (e.tl_tick_at(f32::from(ev.position.x)).min(e.doc.len), lane))
                            })
                            .flatten();
                        if h != e.tl.hover || inside {
                            e.tl.hover = h;
                            e.notify_pane(crate::ui::panes::PaneKind::Bottom, cx);
                        }
                    });
                });
            },
        )
        .size_full();
        let playing = self.playing;
        let sel = self.tick_sel.clone();
        let locked_sel =
            sel.as_ref().is_some_and(|r| self.doc.locks.iter().any(|l| l.start < r.end && r.start < l.end));
        let ms0 = self.race_ms0;
        let tick_f = self.num(
            "tl.tick",
            f64::from(ms0 + 10 * self.cursor as i32) / 1000.0,
            Num { decimals: 2, ..SEC }.w(84.0),
            move |e, x, _| {
                e.playing = false;
                e.set_cursor((((x * 1000.0).round() as i32 - ms0) / 10).max(0) as usize)
            },
            cx,
        );
        let len_f = self.num(
            "tl.len",
            self.doc.len as f64 / 100.0,
            Num::new(1, "s", 0.1).w(56.0).range(1.0, 600.0),
            |e, x, _| {
                let n = (x * 100.0).round() as usize;
                e.edit("run length", Some("run.len"), |d| d.len = n)
            },
            cx,
        );
        let sp = self.choice(
            "tl.speed",
            (self.play_speed * 100.0).round() as u32,
            vec![
                ("\u{00bc}\u{00d7}", 25u32),
                ("\u{00bd}\u{00d7}", 50),
                ("1\u{00d7}", 100),
                ("2\u{00d7}", 200),
            ],
            |e, v, _| e.play_speed = v as f32 / 100.0,
            cx,
        );
        let chan_names = ["@icons/gas.svg|Gas", "@icons/brake.svg|Brake", "@icons/steer.svg|Steer"];
        let ch = self.tl.channel;
        let chan = self.choice(
            "tl.channel",
            ch,
            (0..3).map(|i| (chan_names[i], i)).collect(),
            |e, v, _| e.tl.channel = v,
            cx,
        );
        let (vmin, vmax) = if ch == 2 { (-1.0, 1.0) } else { (0.0, 1.0) };
        let val = self.num(
            "tl.value",
            self.set_value as f64,
            PLAIN.w(54.0).range(vmin, vmax),
            |e, x, _| {
                e.set_value = x as f32;
                e.set_selection_value(x as f32);
            },
            cx,
        );
        let sep = || div().w(px(1.0)).h(px(16.0)).mx(px(theme::SP1)).bg(c(theme::HAIRLINE));
        let transport = div()
            .flex()
            .items_center()
            .gap(px(2.0))
            .child(
                icon_square("tl-start", "icons/tostart.svg", false)
                    .tooltip(tip("To the start", None))
                    .on_click(cx.listener(|e, _, _, cx| {
                        e.playing = false;
                        e.set_cursor(0);
                        cx.notify();
                    })),
            )
            .child(
                icon_square("tl-back", "icons/stepback.svg", false)
                    .tooltip(tip("Back one tick (\u{2190}, \u{21e7}: ten)", None))
                    .on_click(cx.listener(|e, _, _, cx| {
                        e.step(-1);
                        cx.notify();
                    })),
            )
            .child(
                icon_square("tl-play", if playing { "icons/pause.svg" } else { "icons/play.svg" }, true)
                    .tooltip(tip("Play / pause (Space)", None))
                    .on_click(cx.listener(|e, _, _, cx| {
                        e.toggle_play();
                        cx.notify();
                    })),
            )
            .child(
                icon_square("tl-fwd", "icons/stepfwd.svg", false)
                    .tooltip(tip("Forward one tick (\u{2192}, \u{21e7}: ten)", None))
                    .on_click(cx.listener(|e, _, _, cx| {
                        e.step(1);
                        cx.notify();
                    })),
            );
        let time = div()
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .child(
                div()
                    .id("tl-time")
                    .child(tick_f)
                    .tooltip(tip("Playhead", Some("Race time in seconds; type one to jump there."))),
            )
            .child(
                mono(format!("tick {}", self.cursor), theme::FS_SMALL)
                    .text_color(c(theme::FAINT))
                    .whitespace_nowrap(),
            );
        let selection = sel.map(|r| {
            div()
                .flex()
                .items_center()
                .gap(px(2.0))
                .pl(px(theme::SP2))
                .pr(px(2.0))
                .h(px(30.0))
                .rounded(px(theme::R_CTL + 1.0))
                .bg(c(theme::SELECT))
                .child(mono(format!("{:.2} s", r.len() as f32 / 100.0), theme::FS_SMALL).mr(px(theme::SP1)).whitespace_nowrap())
                .child(chan)
                .child(val)
                .child(icon_square("tl-set", "icons/check.svg", false).tooltip(tip("Set that input over the selection (Enter)", None)).on_click(cx.listener(|e, _, _, cx| {
                    let v = e.set_value;
                    e.set_selection_value(v);
                    cx.notify();
                })))
                .child(icon_square("tl-clear", "icons/clear.svg", false).tooltip(tip("Release every input in the selection (X)", Some("\u{2325}\u{2190} \u{2325}\u{2192} move the selected inputs a tick earlier or later; \u{2318}C \u{2318}V copy and paste at the playhead."))).on_click(cx.listener(|e, _, _, cx| {
                    e.clear_selection_inputs();
                    cx.notify();
                })))
                .child(icon_square("tl-lock", if locked_sel { "icons/unlock.svg" } else { "icons/lock.svg" }, false).tooltip(tip(if locked_sel { "Unlock (\u{2318}\u{21e7}L)" } else { "Lock (\u{2318}L)" }, Some("Locked ticks are hatched; painting and the search leave them as they are."))).on_click(cx.listener(move |e, _, _, cx| {
                    e.lock_selection(!locked_sel);
                    cx.notify();
                })))
                .child(btn_icon("tl-win", "icons/sparkle.svg", "Search here", Btn::Quiet).px(px(theme::SP2)).tooltip(tip("Let the search change only these ticks (\u{2318}W)", None)).on_click(cx.listener(|e, _, _, cx| {
                    if let Some(r) = e.tick_sel.clone() {
                        e.spec.window = r;
                    }
                    cx.notify();
                })))
        });
        let tb = div()
            .h(px(38.0))
            .flex_none()
            .px(px(theme::SP3))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .child(transport)
            .child(time)
            .child(sp)
            .children(selection)
            .child(div().flex_1())
            .child(chip_icon("tl-paint", self.paint_tool, "icons/brush.svg", "Paint").tooltip(tip("Paint inputs (V)", Some("Drag in Steer to draw it (\u{21e7}: quarter steps), in Gas or Brake to switch them. \u{2325}-drag paints without the tool. Locked ticks are skipped."))).on_click(cx.listener(|e, _, _, cx| {
                e.paint_tool = !e.paint_tool;
                cx.notify();
            })))
            .child(chip_icon("tl-rec", self.record_mode, "icons/record.svg", "Record").tooltip(tip("Record (R)", Some("Space plays and writes the held keys (arrows or WASD, \u{21e7} half steer) as playback passes each tick."))).on_click(cx.listener(|e, _, _, cx| {
                e.record_mode = !e.record_mode;
                if !e.record_mode {
                    e.stop_recording();
                }
                cx.notify();
            })))
            .child(btn_icon("tl-newline", "icons/branch.svg", "New line", Btn::Quiet).px(px(theme::SP2)).tooltip(tip("Start a different line at the playhead", Some("Keeps the current run under Runs, releases every input after the playhead and starts the search range there. On released inputs the search builds a line from scratch instead of editing the old one."))).on_click(cx.listener(|e, _, _, cx| {
                e.new_line_from_here();
                cx.notify();
            })))
            .child(sep())
            .child(div().id("tl-len").child(len_f).tooltip(tip("Run length", Some("Ticks after the last input are released; the search extends the run when it needs to."))))
            .child(icon_square("tl-fit", "icons/fit.svg", false).tooltip(tip("Fit the selection, or the run (\\)", Some("= and - zoom about the playhead; drag the overview's box or its ends to pan and zoom."))).on_click(cx.listener(|e, _, _, cx| {
                e.fit_timeline();
                cx.notify();
            })));
        div().size_full().flex().flex_col().child(tb).child(div().flex_1().min_h_0().child(lanes_el))
    }

    fn timeline_down(&mut self, e: &MouseDownEvent, cx: &mut Context<Self>) {
        let (x, y) = (f32::from(e.position.x), f32::from(e.position.y));
        let k = self.tl_tick_at(x).min(self.doc.len);
        let Some((lane, y0, h)) = self.tl_lane_at(y) else { return };
        if e.button == MouseButton::Left && self.tl_on_split(y) {
            self.drag = Some(Drag::Timeline(TlDrag::Split));
            return;
        }
        if x < f32::from(self.tl.bounds.origin.x) + GUTTER {
            match lane {
                Lane::Gas => self.tl.channel = 0,
                Lane::Brake => self.tl.channel = 1,
                Lane::Steer => self.tl.channel = 2,
                _ => {}
            }
            cx.notify();
            return;
        }
        if e.button == MouseButton::Middle {
            self.drag = Some(Drag::Timeline(TlDrag::Pan { last_x: x, start0: self.tl.start }));
            return;
        }
        if e.button == MouseButton::Right {
            let mut target = None;
            if lane == Lane::Plan {
                target =
                    self.setup_clips().into_iter().find(|c| (c.1..c.2).contains(&k)).map(|c| Sel::Setup(c.0));
                if target.is_none() && self.route_ticks().is_some_and(|(a, b)| (a..b).contains(&k)) {
                    target = Some(Sel::Route);
                }
            }
            let rows = match target {
                Some(s) => {
                    self.select(s);
                    self.context_rows(s, None)
                }
                None => self.tick_rows(k),
            };
            self.show_rows(e.position, rows);
            cx.stop_propagation();
            cx.notify();
            return;
        }
        if e.button != MouseButton::Left {
            return;
        }
        let paint = self.paint_tool || e.modifiers.alt;
        let ppt = self.tl.ppt as f32;
        let near = |t: usize| ((t as f32 - k as f32) * ppt).abs() < NEAR;
        let d = match lane {
            Lane::Ruler if y >= y0 + STRIP_Y - 2.0 => {
                let w = self.spec.window.clone();
                let edge = if near(w.start) {
                    0
                } else if near(w.end) {
                    1
                } else if w.contains(&k) {
                    2
                } else {
                    3
                };
                TlDrag::Window { edge, anchor: k, w0: w }
            }
            Lane::Ruler => {
                self.playing = false;
                self.set_cursor(self.tl_snap(k, e.modifiers.shift));
                TlDrag::Scrub
            }
            Lane::Track => {
                let m = self
                    .markers()
                    .into_iter()
                    .filter(|m| near(*m))
                    .min_by_key(|m| (*m as i64 - k as i64).unsigned_abs());
                self.playing = false;
                self.set_cursor(m.unwrap_or(k));
                if let Some(m) = m
                    && let Some(i) = self.reach.iter().position(|r| r.tick == Some(m))
                {
                    let id = self.doc.goals[i].id;
                    self.select(Sel::Goal(id));
                }
                TlDrag::Scrub
            }
            Lane::Overview => {
                let vis = self.tl.width() as f64 / self.tl.ppt.max(1e-6);
                let (start, end) = (self.tl.start, self.tl.start + vis);
                let (k0, k1) = self.tl_overview_span();
                let ox = |t: f64| {
                    f32::from(self.tl.bounds.origin.x)
                        + GUTTER
                        + ((t - k0) / (k1 - k0)) as f32 * self.tl.width()
                };
                let at = self.tl_overview_tick(x);
                let (edge, grab) = if (x - ox(start)).abs() < NEAR {
                    (0, 0.0)
                } else if (x - ox(end)).abs() < NEAR {
                    (1, 0.0)
                } else if at > start && at < end {
                    (2, at - start)
                } else {
                    self.tl.start = (at - vis * 0.5).max(k0);
                    (2, at - self.tl.start)
                };
                TlDrag::Overview { edge, grab, start0: start, end0: end }
            }
            Lane::Plan | Lane::Tricks => {
                self.playing = false;
                self.set_cursor(k);
                if lane == Lane::Plan {
                    if let Some((id, ..)) = self.setup_clips().into_iter().find(|c| (c.1..c.2).contains(&k)) {
                        self.select(Sel::Setup(id));
                    } else if self.route_ticks().is_some_and(|(a, b)| (a..b).contains(&k)) {
                        self.select(Sel::Route);
                    }
                } else if let Some(t) = self.trick_spans.iter().find(|t| k + 3 >= t.from && k <= t.to + 3) {
                    self.set_cursor(t.from);
                }
                TlDrag::Scrub
            }
            Lane::Steer | Lane::Gas | Lane::Brake if paint => {
                let ch = match lane {
                    Lane::Gas => 0,
                    Lane::Brake => 1,
                    _ => 2,
                };
                self.tl.channel = ch;
                let cur = self.control(k);
                let value = match lane {
                    Lane::Steer => steer_at(y, y0, h, e.modifiers.shift),
                    Lane::Gas => {
                        if cur.gas > 0.5 {
                            0.0
                        } else {
                            1.0
                        }
                    }
                    _ => {
                        if cur.brake > 0.5 {
                            0.0
                        } else {
                            1.0
                        }
                    }
                };
                self.tl.strokes += 1;
                let stroke = self.tl.strokes;
                let mut d = TlDrag::Paint { lane, value, last: None, stroke };
                self.paint_at(&mut d, k, y, e.modifiers.shift);
                d
            }
            _ => {
                match lane {
                    Lane::Gas => self.tl.channel = 0,
                    Lane::Brake => self.tl.channel = 1,
                    Lane::Steer => self.tl.channel = 2,
                    _ => {}
                }
                let anchor = match (&self.tick_sel, e.modifiers.shift) {
                    (Some(r), true) => {
                        if k >= r.start {
                            r.start
                        } else {
                            r.end
                        }
                    }
                    _ => k,
                };
                self.tick_sel = Some(anchor.min(k)..anchor.max(k) + 1);
                TlDrag::Select { anchor }
            }
        };
        self.drag = Some(Drag::Timeline(d));
        cx.notify();
    }

    fn paint_at(&mut self, d: &mut TlDrag, k: usize, y: f32, fine: bool) {
        let TlDrag::Paint { lane, value, last, stroke } = d else { return };
        let (lane, stroke) = (*lane, *stroke);
        let Some((_, y0, h)) = self.tl_lanes().into_iter().find(|l| l.0 == lane) else { return };
        let v = if lane == Lane::Steer {
            let mut v = steer_at(y, y0, h, fine);
            if self.spec.digital_steer {
                v = if v > 0.5 {
                    1.0
                } else if v < -0.5 {
                    -1.0
                } else {
                    0.0
                };
            }
            v
        } else {
            *value
        };
        let (a, va) = last.unwrap_or((k, v));
        let (lo, hi) = (a.min(k), a.max(k));
        let ticks: Vec<(usize, Controls)> = (lo..=hi)
            .map(|t| {
                let f = if hi == lo { 1.0 } else { (t - a) as f32 / (k as f32 - a as f32) };
                let vv = if lane == Lane::Steer { va + (v - va) * f.clamp(0.0, 1.0) } else { v };
                let mut c = self.control(t);
                match lane {
                    Lane::Gas => c.gas = vv,
                    Lane::Brake => c.brake = vv,
                    _ => c.steer = vv,
                }
                (t, c)
            })
            .collect();
        *last = Some((k, v));
        self.set_ticks("paint inputs", Some(&format!("paint-{stroke}")), ticks.into_iter());
    }

    pub fn timeline_move(&mut self, e: &MouseMoveEvent, cx: &mut Context<Self>) {
        let Some(Drag::Timeline(mut d)) = self.drag.clone() else { return };
        let (x, y) = (f32::from(e.position.x), f32::from(e.position.y));
        let k = self.tl_tick_at(x);
        match &mut d {
            TlDrag::Scrub => {
                self.set_cursor(self.tl_snap(k, e.modifiers.shift));
            }
            TlDrag::Select { anchor } => {
                let a = *anchor;
                self.tick_sel = Some(a.min(k)..(a.max(k) + 1).min(self.doc.len.max(1)));
            }
            TlDrag::Paint { .. } => self.paint_at(&mut d, k, y, e.modifiers.shift),
            TlDrag::Pan { last_x, start0 } => {
                let dx = x - *last_x;
                self.tl.start = (*start0 - dx as f64 / self.tl.ppt).max(self.race_start_tick() as f64);
            }
            TlDrag::Window { edge, anchor, w0 } => {
                let w = match edge {
                    0 => k.min(w0.end.saturating_sub(1))..w0.end,
                    1 => w0.start..k.max(w0.start + 1),
                    2 => {
                        let d = k as i64 - *anchor as i64;
                        let s = (w0.start as i64 + d).max(0) as usize;
                        s..s + w0.len()
                    }
                    _ => {
                        let a = *anchor;
                        a.min(k)..a.max(k).max(a.min(k) + 1)
                    }
                };
                self.spec.window = w;
            }
            TlDrag::Split => {
                let ls = self.tl_lanes();
                if let (Some(s), Some(t)) =
                    (ls.iter().find(|l| l.0 == Lane::Speed), ls.iter().find(|l| l.0 == Lane::Steer))
                {
                    let total = s.2 + t.2;
                    self.tl.split = ((y - s.1) / total.max(1.0)).clamp(0.2, 0.8);
                }
            }
            TlDrag::Overview { edge, grab, start0, end0 } => {
                let at = self.tl_overview_tick(x);
                let k0 = self.race_start_tick() as f64;
                let w = self.tl.width() as f64;
                match edge {
                    0 => {
                        let s = at.min(*end0 - 20.0).max(k0);
                        self.tl.ppt = (w / (*end0 - s)).clamp(0.02, 40.0);
                        self.tl.start = s;
                    }
                    1 => {
                        let end = at.max(*start0 + 20.0);
                        self.tl.ppt = (w / (end - *start0)).clamp(0.02, 40.0);
                        self.tl.start = *start0;
                    }
                    _ => self.tl.start = (at - *grab).max(k0),
                }
            }
        }
        self.drag = Some(Drag::Timeline(d));
        cx.notify();
    }

    pub fn timeline_up(&mut self, e: &MouseUpEvent, cx: &mut Context<Self>) {
        match self.drag.clone() {
            Some(Drag::Timeline(TlDrag::Select { anchor })) => {
                let k = self.tl_tick_at(f32::from(e.position.x));
                if k == anchor {
                    self.tick_sel = None;
                    self.playing = false;
                    self.set_cursor(k);
                }
            }
            Some(Drag::Timeline(TlDrag::Window { edge, anchor, w0 })) => {
                if edge == 3 && self.spec.window.len() <= 1 {
                    self.spec.window = w0;
                    self.playing = false;
                    self.set_cursor(anchor);
                } else {
                    let w = self.spec.window.clone();
                    self.info(
                        "search",
                        format!("window: ticks {}\u{2013}{} ({} ticks)", w.start, w.end, w.len()),
                    );
                }
            }
            _ => {}
        }
        self.drag = None;
        cx.notify();
    }

    fn timeline_scroll(&mut self, e: &ScrollWheelEvent, cx: &mut Context<Self>) {
        let d = e.delta.pixel_delta(px(16.0));
        let (dx, dy) = (f32::from(d.x), f32::from(d.y));
        if e.modifiers.platform || e.modifiers.control || (!e.delta.precise() && !e.modifiers.shift) {
            self.timeline_zoom(f32::from(e.position.x), 1.0 + dy * 0.004, cx);
        } else {
            let pan = if dx.abs() > dy.abs() { dx } else { dy };
            self.tl.start = (self.tl.start - pan as f64 / self.tl.ppt).max(self.race_start_tick() as f64);
            cx.notify();
        }
    }

    fn timeline_zoom(&mut self, x: f32, f: f32, cx: &mut Context<Self>) {
        self.tl_zoom_at(x, f);
        cx.notify();
    }

    fn tl_zoom_at(&mut self, x: f32, f: f32) {
        let at = self.tl.start + (x - f32::from(self.tl.bounds.origin.x) - GUTTER) as f64 / self.tl.ppt;
        self.tl.ppt = (self.tl.ppt * f as f64).clamp(0.02, 40.0);
        self.tl.start = (at - (x - f32::from(self.tl.bounds.origin.x) - GUTTER) as f64 / self.tl.ppt)
            .max(self.race_start_tick() as f64);
    }

    pub fn copy_inputs(&mut self) {
        if let Some(r) = self.tick_sel.clone() {
            self.clipboard = Some(r.clone().map(|k| self.control(k)).collect());
            self.info("edit", format!("copied {} ticks", r.len()));
        }
    }

    pub fn paste_inputs(&mut self) {
        let Some(c) = self.clipboard.clone() else { return };
        let at = self.cursor;
        let n = c.len();
        if at + n > self.doc.len {
            let need = at + n + 100;
            self.edit("extend run", None, |d| d.len = need);
        }
        self.set_ticks("paste inputs", None, c.into_iter().enumerate().map(|(i, x)| (at + i, x)));
        self.tick_sel = Some(at..at + n);
        self.info("edit", format!("pasted {n} ticks at tick {at}"));
    }

    pub fn lock_selection(&mut self, on: bool) {
        if let Some(r) = self.tick_sel.clone() {
            self.edit(if on { "lock ticks" } else { "unlock ticks" }, None, |d| {
                if on { d.lock(r) } else { d.unlock(r) }
            });
        }
    }

    pub fn clear_selection_inputs(&mut self) {
        if let Some(r) = self.tick_sel.clone() {
            self.set_ticks("clear inputs", None, r.map(|k| (k, Controls::default())));
        }
    }

    pub fn set_selection_value(&mut self, v: f32) {
        let Some(r) = self.tick_sel.clone() else { return };
        let ch = self.tl.channel;
        let ticks: Vec<(usize, Controls)> = r
            .map(|k| {
                let mut c = self.control(k);
                match ch {
                    0 => c.gas = v.clamp(0.0, 1.0).round(),
                    1 => c.brake = v.clamp(0.0, 1.0).round(),
                    _ => c.steer = v.clamp(-1.0, 1.0),
                }
                (k, c)
            })
            .collect();
        self.set_ticks("set inputs", Some("tl.value"), ticks.into_iter());
    }

    pub fn shift_selection(&mut self, n: isize) {
        let Some(r) = self.tick_sel.clone() else { return };
        if n < 0 && r.start < n.unsigned_abs() {
            return;
        }
        let src: Vec<Controls> = r.clone().map(|k| self.control(k)).collect();
        let dst = (r.start as isize + n) as usize;
        let mut ticks: Vec<(usize, Controls)> = Vec::new();
        if n > 0 {
            let hold = if r.start > 0 { self.control(r.start - 1) } else { Controls::default() };
            for k in r.start..dst.min(r.end) {
                ticks.push((k, hold));
            }
        } else {
            let hold = self.control(r.end);
            for k in (dst + r.len()).max(r.start)..r.end {
                ticks.push((k, hold));
            }
        }
        for (i, c) in src.into_iter().enumerate() {
            ticks.push((dst + i, c));
        }
        self.set_ticks("shift inputs", None, ticks.into_iter());
        self.tick_sel = Some(dst..dst + r.len());
    }
}

fn steer_at(y: f32, y0: f32, h: f32, fine: bool) -> f32 {
    let v = (1.0 - 2.0 * (y - y0 - 2.0) / (h - 4.0).max(1.0)).clamp(-1.0, 1.0);
    if fine { (v * 4.0).round() / 4.0 } else { (v * 127.0).round() / 127.0 }
}

fn cursor_at(p: &Paint, b: Bounds<Pixels>, m: Point<Pixels>, alt: bool) -> CursorStyle {
    let (lx, ly) = (f32::from(m.x) - f32::from(b.origin.x), f32::from(m.y) - f32::from(b.origin.y));
    let ls = lanes(f32::from(b.size.height), p.extra, p.split);
    if ls.iter().find(|l| l.0 == Lane::Steer).is_some_and(|(_, y0, _)| (ly - (y0 - GAP * 0.5)).abs() <= 3.0) {
        return CursorStyle::ResizeUpDown;
    }
    let Some(&(lane, y0, _)) = ls.iter().find(|(_, y0, h)| *h > 0.0 && ly >= *y0 && ly < y0 + h + GAP) else {
        return CursorStyle::Arrow;
    };
    if lx < GUTTER {
        return CursorStyle::Arrow;
    }
    let w = (f32::from(b.size.width) - GUTTER - 8.0).max(200.0);
    let x_of = |k: f64| GUTTER + ((k - p.start) * p.ppt) as f32;
    let k = p.start + (lx - GUTTER) as f64 / p.ppt.max(1e-6);
    match lane {
        Lane::Ruler if ly >= y0 + STRIP_Y - 2.0 => {
            if (lx - x_of(p.window.start as f64)).abs() < NEAR
                || (lx - x_of(p.window.end as f64)).abs() < NEAR
            {
                CursorStyle::ResizeLeftRight
            } else if (p.window.start as f64..p.window.end as f64).contains(&k) {
                CursorStyle::OpenHand
            } else {
                CursorStyle::Arrow
            }
        }
        Lane::Overview => {
            let (k0, k1) = p.overview;
            let ox = |t: f64| GUTTER + ((t - k0) / (k1 - k0)) as f32 * w;
            let (a, e) = (ox(p.start), ox(p.start + w as f64 / p.ppt.max(1e-6)));
            if (lx - a).abs() < NEAR || (lx - e).abs() < NEAR {
                CursorStyle::ResizeLeftRight
            } else if lx > a && lx < e {
                CursorStyle::OpenHand
            } else {
                CursorStyle::PointingHand
            }
        }
        Lane::Steer | Lane::Gas | Lane::Brake if p.paint_tool || alt => CursorStyle::Crosshair,
        _ => CursorStyle::Arrow,
    }
}

fn paint_lanes(
    window: &mut Window,
    cx: &mut App,
    b: Bounds<Pixels>,
    p: &Paint,
    live: Option<(usize, Option<(usize, Lane)>)>,
) {
    let dynamic = live.is_some();
    let (cursor, hover) = live.unwrap_or((0, None));
    let (bx, by) = (f32::from(b.origin.x), f32::from(b.origin.y));
    let (bw, bh) = (f32::from(b.size.width), f32::from(b.size.height));
    let x0 = bx + GUTTER;
    let w = (bw - GUTTER - 8.0).max(1.0);
    let ppt = p.ppt as f32;
    let x_of = |k: f32| x0 + (k - p.start as f32) * ppt;
    let rect = |x: f32, y: f32, ww: f32, hh: f32| Bounds {
        origin: point(px(x), px(y)),
        size: size(px(ww.max(0.0)), px(hh.max(0.0))),
    };
    let ls = lanes(bh, p.extra, p.split);
    let lane = |l: Lane| ls.iter().find(|x| x.0 == l).map(|x| (by + x.1, x.2)).unwrap();
    let first = p.start.floor().max(0.0) as usize;
    let last = ((p.start + w as f64 / p.ppt).ceil() as usize).min(p.len);
    let (ry, rh) = lane(Lane::Ruler);
    let (ty, th) = lane(Lane::Track);
    let (spy, sph) = lane(Lane::Speed);
    let (sty, sth) = lane(Lane::Steer);
    let (gy, gh) = lane(Lane::Gas);
    let (bry, brh) = lane(Lane::Brake);
    let (oy, oh) = lane(Lane::Overview);
    let (top, bottom) = (ty + th + GAP, bry + brh);
    let (sy0, sh) = (ry + STRIP_Y, STRIP_H);
    let span = |a: usize, e: usize| (x_of(a as f32).max(x0), x_of(e as f32).min(x0 + w));
    let cols = w as usize;
    let col_ticks = |col: usize| {
        let a = (p.start + col as f64 / p.ppt).floor().max(0.0) as usize;
        let e = ((p.start + (col + 1) as f64 / p.ppt).ceil() as usize).max(a + 1);
        (a, e)
    };
    let hovering = hover.is_some_and(|(_, l)| l != Lane::Overview);
    let at = hover.filter(|_| hovering).map_or(cursor, |h| h.0);
    let value_col = if hovering { theme::GHOST } else { theme::TEXT };
    window.paint_layer(b, |window| {
        if !dynamic {
            quad(window, fill(rect(x0, ry, w, rh + th), theme::alpha(theme::PANEL, 1.0)));
            quad(window, fill(rect(x0, sy0, w, sh), theme::alpha(theme::WELL, 0.6)));
            for &(l, y, h) in &ls {
                if !matches!(l, Lane::Ruler | Lane::Track | Lane::Overview) && h > 0.0 {
                    quad(window, fill(rect(x0, by + y, w, h), theme::alpha(theme::WELL, 1.0)));
                }
            }
            quad(window, fill(rect(x0 - 1.0, ty, 1.0, bottom - ty), theme::alpha(theme::HAIRLINE, 1.0)));
            let step = ruler_step(p.ppt, 64.0) as i32 * 10;
            let minor = (step / 5).max(10);
            let end_ms = p.race_ms0 + 10 * last as i32;
            let mut ms = (p.race_ms0 + 10 * first as i32).div_euclid(minor) * minor;
            while ms <= end_ms {
                let x = x_of(((ms - p.race_ms0) as f32) / 10.0);
                if x >= x0 && ms >= p.race_ms0 {
                    if ms.rem_euclid(step) == 0 {
                        quad(window, fill(rect(x, ry + 11.0, 1.0, 6.0), theme::alpha(theme::MUTED, 0.8)));
                        quad(
                            window,
                            fill(rect(x, top, 1.0, bottom - top), theme::alpha(theme::HAIRLINE, 0.55)),
                        );
                        let label = if step % 1000 == 0 {
                            format!("{}s", ms / 1000)
                        } else if step % 100 == 0 {
                            format!("{:.1}s", ms as f64 / 1000.0)
                        } else {
                            format!("{:.2}", ms as f64 / 1000.0)
                        };
                        text(
                            window,
                            cx,
                            &label,
                            x + 4.0,
                            ry + 2.0,
                            9.5,
                            if ms == 0 { theme::TEXT } else { theme::MUTED },
                            1.0,
                            true,
                        );
                    } else {
                        quad(window, fill(rect(x, ry + 14.0, 1.0, 3.0), theme::alpha(theme::FAINT, 0.7)));
                    }
                }
                ms += minor;
            }
            let zero = ((-p.race_ms0).max(0) / 10) as usize;
            let (ca, cb) = span(0, zero);
            if cb > ca {
                quad(window, fill(rect(ca, ty, cb - ca, bottom - ty), theme::alpha(theme::BG, 0.45)));
                if cb - ca > 70.0 {
                    text(window, cx, "countdown", ca + 6.0, ty + 4.0, 9.5, theme::FAINT, 1.0, false);
                }
            }
            let col_a: Vec<Option<f32>> = (0..cols)
                .map(|col| {
                    let (a, e) = col_ticks(col);
                    (a < p.speed.len())
                        .then(|| p.speed[a..e.min(p.speed.len())].iter().copied().fold(0.0f32, f32::max))
                })
                .collect();
            let col_b: Vec<Option<f32>> = match &p.b_speed {
                Some(bs) => (0..cols)
                    .map(|col| {
                        sample_at(
                            bs,
                            p.race_ms0 as f32 + 10.0 * (p.start + (col as f64 + 0.5) / p.ppt) as f32,
                        )
                    })
                    .collect(),
                None => vec![None; cols],
            };
            let (mut lo, mut hi) = col_a
                .iter()
                .chain(&col_b)
                .flatten()
                .fold((f32::MAX, f32::MIN), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
            if lo > hi {
                (lo, hi) = (0.0, 100.0);
            }
            let pad = ((hi - lo) * 0.12).max((10.0 - (hi - lo)).max(0.0) * 0.5) + 2.0;
            let (lo, hi) = ((lo - pad).max(0.0), hi + pad);
            let y_of_v = |v: f32| spy + sph - 4.0 - (v - lo) / (hi - lo) * (sph - 10.0);
            let gstep = [1.0f32, 2.0, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0, 200.0]
                .into_iter()
                .find(|s| s / (hi - lo) * (sph - 10.0) >= 18.0)
                .unwrap_or(500.0);
            let mut g = (lo / gstep).ceil() * gstep;
            while g < hi && sph > 24.0 {
                let y = y_of_v(g);
                if y > spy + 9.0 {
                    quad(window, fill(rect(x0, y, w, 1.0), theme::alpha(theme::HAIRLINE, 0.5)));
                    text_r(
                        window,
                        cx,
                        &format!("{g:.0}"),
                        x0 + w - 4.0,
                        y - 12.0,
                        9.0,
                        theme::FAINT,
                        1.0,
                        true,
                    );
                }
                g += gstep;
            }
            for col in 0..cols {
                if let (Some(a), Some(bv)) = (col_a[col], col_b[col]) {
                    let (ya, yb) = (y_of_v(a), y_of_v(bv));
                    quad(
                        window,
                        fill(
                            rect(x0 + col as f32, ya.min(yb), 1.0, (ya - yb).abs()),
                            theme::alpha(if a >= bv { theme::OK } else { theme::ERROR }, 0.22),
                        ),
                    );
                }
            }
            let curve = |window: &mut Window, vals: &[Option<f32>], width: f32, color: gpui::Hsla| {
                let mut line = PathBuilder::stroke(px(width));
                let mut started = false;
                for (col, v) in vals.iter().enumerate() {
                    let Some(v) = v else {
                        started = false;
                        continue;
                    };
                    let pt = point(px(x0 + col as f32 + 0.5), px(y_of_v(*v)));
                    if started {
                        line.line_to(pt);
                    } else {
                        line.move_to(pt);
                        started = true;
                    }
                }
                if let Ok(path) = line.build() {
                    draw_path(window, path, color);
                }
            };
            if p.b_speed.is_some() {
                curve(window, &col_b, 1.2, theme::alpha(theme::GHOST, 0.9));
            }
            curve(window, &col_a, 1.6, theme::alpha(theme::SPEED, 1.0));
            if p.simulated < p.len + 1 {
                let a = x_of(p.simulated as f32).max(x0);
                if a < x0 + w {
                    quad(window, fill(rect(a, spy, x0 + w - a, sph), theme::alpha(theme::PANEL, 0.7)));
                }
            }
            let mid = sty + sth * 0.5;
            let amp = sth * 0.5 - 5.0;
            quad(window, fill(rect(x0, mid - 0.5, w, 1.0), theme::alpha(theme::HAIRLINE, 1.0)));
            for g in [-0.5f32, 0.5] {
                quad(window, fill(rect(x0, mid - g * amp, w, 1.0), theme::alpha(theme::HAIRLINE, 0.35)));
            }
            let per_tick = ppt >= 2.0;
            let draw_inputs = |window: &mut Window,
                               ctrl: &[Controls],
                               colors: [u32; 3],
                               only: Option<&[Range<usize>]>| {
                let inside = |k: usize| only.is_none_or(|rs| rs.iter().any(|r| r.contains(&k)));
                let [sc, gc, bc] = colors;
                let mut edge = PathBuilder::stroke(px(1.2));
                let mut started = false;
                let mut edge_to = |x: f32, s: f32, on: bool| {
                    let pt = point(px(x), px(mid - s * amp));
                    if !on {
                        started = false;
                    } else if started {
                        edge.line_to(pt);
                    } else {
                        edge.move_to(pt);
                        started = true;
                    }
                };
                if per_tick {
                    let gap = if ppt >= 5.0 { 1.0 } else { 0.0 };
                    for k in first..last.min(ctrl.len()) {
                        if !inside(k) {
                            edge_to(0.0, 0.0, false);
                            continue;
                        }
                        let c = ctrl[k];
                        let x = x_of(k as f32);
                        let ww = ppt - gap;
                        let sv = c.steer * amp;
                        if c.steer != 0.0 {
                            quad(
                                window,
                                fill(
                                    rect(x, if sv > 0.0 { mid - sv } else { mid }, ww, sv.abs().max(1.0)),
                                    theme::alpha(sc, 0.38),
                                ),
                            );
                        }
                        edge_to(x, c.steer, true);
                        edge_to(x + ppt, c.steer, true);
                        if c.gas > 0.0 {
                            quad(
                                window,
                                fill(rect(x, gy + 4.0, ww, gh - 8.0), theme::alpha(gc, 0.5 * c.gas)),
                            );
                        }
                        if c.brake > 0.0 {
                            quad(
                                window,
                                fill(rect(x, bry + 3.0, ww, brh - 6.0), theme::alpha(bc, 0.95 * c.brake)),
                            );
                        }
                    }
                } else {
                    for col in 0..cols {
                        let (a, e) = col_ticks(col);
                        let e = e.min(ctrl.len());
                        if a >= ctrl.len() {
                            break;
                        }
                        if !(a..e).any(inside) {
                            edge_to(0.0, 0.0, false);
                            continue;
                        }
                        let x = x0 + col as f32;
                        let (mut lo, mut hi, mut gas, mut brk) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
                        for c in &ctrl[a..e] {
                            lo = lo.min(c.steer);
                            hi = hi.max(c.steer);
                            gas = gas.max(c.gas);
                            brk = brk.max(c.brake);
                        }
                        let (yt, yb) = (mid - hi * amp, mid - lo * amp);
                        if yb - yt > 0.5 {
                            quad(window, fill(rect(x, yt, 1.0, yb - yt), theme::alpha(sc, 0.38)));
                        }
                        edge_to(x + 0.5, ctrl[a].steer, true);
                        if gas > 0.0 {
                            quad(window, fill(rect(x, gy + 4.0, 1.0, gh - 8.0), theme::alpha(gc, 0.5 * gas)));
                        }
                        if brk > 0.0 {
                            quad(
                                window,
                                fill(rect(x, bry + 3.0, 1.0, brh - 6.0), theme::alpha(bc, 0.95 * brk)),
                            );
                        }
                    }
                }
                if let Ok(path) = edge.build() {
                    draw_path(window, path, theme::alpha(sc, 1.0));
                }
            };
            draw_inputs(window, &p.controls, [theme::STEER, theme::OK, theme::ERROR], None);
            let mut off: Option<usize> = None;
            for k in first..=last.min(p.controls.len()) {
                let on = k >= last.min(p.controls.len()) || p.controls[k].gas > 0.0;
                match (on, off) {
                    (false, None) => off = Some(k),
                    (true, Some(a)) => {
                        let (xa, xb) = span(a, k);
                        if xb - xa > 30.0 {
                            text(
                                window,
                                cx,
                                "off",
                                xa + (xb - xa) * 0.5 - 7.0,
                                gy + 2.0,
                                8.5,
                                theme::FAINT,
                                1.0,
                                false,
                            );
                        }
                        off = None;
                    }
                    _ => {}
                }
            }
            if let Some((ctrl, diff)) = &p.preview {
                for r in diff {
                    let (a, bb) = span(r.start, r.end);
                    if bb > a {
                        quad(
                            window,
                            fill(rect(a, sty, bb - a, bottom - sty), theme::alpha(theme::GHOST, 0.10)),
                        );
                        quad(window, fill(rect(a, sty, bb - a, 1.0), theme::alpha(theme::GHOST, 0.9)));
                    }
                }
                draw_inputs(window, ctrl, [theme::GHOST; 3], Some(diff));
            }
            for r in &p.locks {
                let (a, bb) = span(r.start, r.end);
                if bb <= a {
                    continue;
                }
                quad(window, fill(rect(a, ty + th - 3.0, bb - a, 3.0), theme::alpha(theme::WARN, 0.8)));
                let mut x = a;
                while x < bb {
                    quad(window, fill(rect(x, sty, 1.0, bottom - sty), theme::alpha(theme::WARN, 0.14)));
                    x += 6.0;
                }
            }
            let mut shade = |window: &mut Window, upto: usize, label: &str| {
                let (a, xe) = span(0, upto);
                if xe > a {
                    quad(window, fill(rect(a, sty, xe - a, bottom - sty), theme::alpha(theme::BG, 0.55)));
                    if xe - a > 150.0 {
                        text(window, cx, label, a + 6.0, sty + 4.0, 9.5, theme::MUTED, 1.0, false);
                    }
                }
            };
            if p.t0_tick > zero {
                shade(window, p.t0_tick, "before the first input");
            }
            if let Some(st) = p.start_tick {
                shade(window, st, "before the placed car");
            }
            let (wa, wb) = span(p.window.start, p.window.end);
            quad(window, fill(rect(x0, top, (wa - x0).max(0.0), bottom - top), theme::alpha(theme::BG, 0.4)));
            quad(
                window,
                fill(
                    rect(wb.max(x0), top, (x0 + w - wb.max(x0)).max(0.0), bottom - top),
                    theme::alpha(theme::BG, 0.4),
                ),
            );
            if wb > wa {
                quad(window, fill(rect(wa, sy0, wb - wa, sh), theme::alpha(theme::PIN, 0.3)));
                if wb - wa > 110.0 {
                    text(
                        window,
                        cx,
                        &format!("Search  {:.2} s", p.window.len() as f32 / 100.0),
                        wa + 8.0,
                        sy0 - 1.0,
                        9.0,
                        theme::TEXT,
                        0.95,
                        false,
                    );
                }
            }
            for (k, right) in [(p.window.start, false), (p.window.end, true)] {
                let x = x_of(k as f32);
                if x >= x0 - 1.0 && x <= x0 + w + 1.0 {
                    quad(
                        window,
                        fill(
                            rect(if right { x - 3.0 } else { x }, sy0 - 1.0, 3.0, sh + 2.0),
                            theme::alpha(theme::PIN, 1.0),
                        ),
                    );
                    quad(window, fill(rect(x - 0.5, top, 1.0, bottom - top), theme::alpha(theme::PIN, 0.4)));
                }
            }
            let (py, ph) = lane(Lane::Plan);
            if let Some((a, e, name)) = &p.route {
                let (xa, xb) = span(*a, *e);
                if xb > xa {
                    quad(window, fill(rect(xa, py + 2.0, xb - xa, ph - 4.0), theme::alpha(theme::PIN, 0.16)));
                    quad(window, fill(rect(xa, py + 2.0, 1.5, ph - 4.0), theme::alpha(theme::PIN, 0.9)));
                    quad(
                        window,
                        fill(rect(xb - 1.5, py + 2.0, 1.5, ph - 4.0), theme::alpha(theme::PIN, 0.9)),
                    );
                    if xb - xa > 60.0 {
                        text(window, cx, name, xa + 8.0, py + 4.0, 9.5, theme::PIN, 1.0, false);
                    }
                }
            }
            for (a, e, name, sel) in &p.clips {
                let (xa, xb) = span(*a, *e);
                if xb > xa {
                    quad(
                        window,
                        fill(
                            rect(xa, py + 2.0, xb - xa, ph - 4.0),
                            theme::alpha(theme::SETUP, if *sel { 0.55 } else { 0.32 }),
                        ),
                    );
                    if xb - xa > 40.0 {
                        text(window, cx, name, xa + 6.0, py + 4.0, 9.5, theme::TEXT, 1.0, false);
                    }
                }
            }
            let (tky, tkh) = lane(Lane::Tricks);
            for t in &p.tricks {
                let (xa, xb) = span(t.from, t.to);
                if xb > xa - 1.0 {
                    quad(
                        window,
                        fill(
                            rect(xa, tky + 3.0, (xb - xa).max(2.0), tkh - 6.0),
                            theme::alpha(crate::lab::trick_color(t.kind), 0.95),
                        ),
                    );
                }
            }
            if let Some(r) = &p.sel {
                let (a, bb) = span(r.start, r.end);
                if bb > a {
                    quad(window, fill(rect(a, top, bb - a, bottom - top), theme::alpha(theme::TEXT, 0.07)));
                    for x in [a, bb] {
                        quad(
                            window,
                            fill(rect(x - 0.5, top, 1.0, bottom - top), theme::alpha(theme::TEXT, 0.5)),
                        );
                    }
                }
            }
            let cy = ty + th * 0.5;
            for &k in &p.ghost_ticks {
                let x = x_of(k as f32);
                if x >= x0 && x <= x0 + w {
                    let mut t = PathBuilder::fill();
                    t.move_to(point(px(x - 3.0), px(ty + th)));
                    t.line_to(point(px(x + 3.0), px(ty + th)));
                    t.line_to(point(px(x), px(ty + th - 5.0)));
                    t.close();
                    if let Ok(path) = t.build() {
                        draw_path(window, path, theme::alpha(theme::GHOST, 1.0));
                    }
                }
            }
            let mut last_label_end = f32::MIN;
            let mut marks = p.marks.clone();
            marks.sort_by_key(|m| m.k);
            for m in &marks {
                let x = x_of(m.k as f32);
                if x < x0 || x > x0 + w {
                    continue;
                }
                let r = 4.5;
                let mut d = PathBuilder::fill();
                d.move_to(point(px(x), px(cy - r)));
                d.line_to(point(px(x + r), px(cy)));
                d.line_to(point(px(x), px(cy + r)));
                d.line_to(point(px(x - r), px(cy)));
                d.close();
                if let Ok(path) = d.build() {
                    draw_path(window, path, theme::alpha(m.color, if m.solid { 1.0 } else { 0.45 }));
                }
                quad(window, fill(rect(x - 0.5, top, 1.0, bottom - top), theme::alpha(m.color, 0.2)));
                let delta = m.delta.map(|d| {
                    let sign = if d.abs() < 5 {
                        "\u{00b1}"
                    } else if d < 0 {
                        "\u{2212}"
                    } else {
                        "+"
                    };
                    (
                        format!("{sign}{:.2}", d.abs() as f32 / 1000.0),
                        if d.abs() < 5 {
                            theme::MUTED
                        } else if d < 0 {
                            theme::OK
                        } else {
                            theme::ERROR
                        },
                    )
                });
                let lw =
                    m.name.len() as f32 * 5.8 + delta.as_ref().map_or(0.0, |d| d.0.len() as f32 * 6.0 + 6.0);
                if x + 9.0 > last_label_end + 8.0 && x + 9.0 + lw < x0 + w {
                    let nw = text(
                        window,
                        cx,
                        &m.name,
                        x + 9.0,
                        cy - 7.0,
                        10.5,
                        m.color,
                        if m.solid { 0.95 } else { 0.6 },
                        false,
                    );
                    if let Some((s, col)) = &delta {
                        text(window, cx, s, x + 9.0 + nw + 6.0, cy - 6.0, 9.5, *col, 1.0, true);
                    }
                    last_label_end = x + 9.0 + lw;
                }
            }
            if let Some(st) = p.start_tick {
                let x = x_of(st as f32);
                if x >= x0 && x <= x0 + w {
                    let mut d = PathBuilder::fill();
                    d.move_to(point(px(x), px(ty + 4.0)));
                    d.line_to(point(px(x + 7.0), px(cy)));
                    d.line_to(point(px(x), px(ty + th - 4.0)));
                    d.close();
                    if let Ok(path) = d.build() {
                        draw_path(window, path, theme::alpha(theme::PIN, 1.0));
                    }
                    quad(window, fill(rect(x - 0.5, top, 1.0, bottom - top), theme::alpha(theme::PIN, 0.6)));
                }
            }
        }
        if dynamic {
            let ctl = p.controls.get(at).copied().unwrap_or_default();
            for &(l, y, h) in &ls {
                let y = by + y;
                if h <= 0.0 || l == Lane::Ruler {
                    continue;
                }
                let (name, dot) = lane_name(l);
                let active = matches!((l, p.channel), (Lane::Gas, 0) | (Lane::Brake, 1) | (Lane::Steer, 2));
                let ny = if h > 30.0 { y + 6.0 } else { y + (h - 14.0) * 0.5 };
                let nx = if let Some(col) = dot {
                    quad(
                        window,
                        fill(rect(bx + 13.0, ny + 4.5, 6.0, 6.0), theme::alpha(col, 1.0))
                            .corner_radii(px(3.0)),
                    );
                    bx + 25.0
                } else {
                    bx + 13.0
                };
                let faint = matches!(l, Lane::Overview);
                text(
                    window,
                    cx,
                    name,
                    nx,
                    ny,
                    11.0,
                    if active {
                        theme::TEXT
                    } else if faint {
                        theme::FAINT
                    } else {
                        theme::MUTED
                    },
                    1.0,
                    false,
                );
                let vx = bx + GUTTER - 12.0;
                match l {
                    Lane::Speed => {
                        if let Some(v) = p.speed.get(at) {
                            text_r(
                                window,
                                cx,
                                &format!("{v:.0} km/h"),
                                vx,
                                ny + 1.0,
                                9.5,
                                value_col,
                                1.0,
                                true,
                            );
                            let bv = p
                                .b_speed
                                .as_ref()
                                .and_then(|bs| sample_at(bs, (p.race_ms0 + 10 * at as i32) as f32));
                            if let (Some(bv), true) = (bv, h > 48.0) {
                                text_r(
                                    window,
                                    cx,
                                    &format!("B {bv:.0}"),
                                    vx,
                                    ny + 16.0,
                                    9.0,
                                    theme::GHOST,
                                    0.85,
                                    true,
                                );
                                let d = v - bv;
                                text_r(
                                    window,
                                    cx,
                                    &format!("{d:+.0}"),
                                    vx,
                                    ny + 30.0,
                                    9.0,
                                    if d >= 0.0 { theme::OK } else { theme::ERROR },
                                    1.0,
                                    true,
                                );
                            }
                        }
                    }
                    Lane::Steer => {
                        text_r(
                            window,
                            cx,
                            &format!("{:+.2}", ctl.steer),
                            vx,
                            ny + 1.0,
                            9.5,
                            if ctl.steer == 0.0 { theme::FAINT } else { value_col },
                            1.0,
                            true,
                        );
                        if h > 46.0 {
                            text(window, cx, "\u{2191} right", nx, ny + 16.0, 9.0, theme::FAINT, 1.0, false);
                            text(
                                window,
                                cx,
                                "\u{2193} left",
                                nx,
                                y + h - 16.0,
                                9.0,
                                theme::FAINT,
                                1.0,
                                false,
                            );
                        }
                    }
                    Lane::Gas => text_r(
                        window,
                        cx,
                        if ctl.gas > 0.0 { "on" } else { "off" },
                        vx,
                        ny + 1.0,
                        9.5,
                        if ctl.gas > 0.0 { value_col } else { theme::FAINT },
                        1.0,
                        true,
                    ),
                    Lane::Brake => text_r(
                        window,
                        cx,
                        if ctl.brake > 0.0 { "on" } else { "off" },
                        vx,
                        ny + 1.0,
                        9.5,
                        if ctl.brake > 0.0 {
                            if hovering { theme::GHOST } else { theme::ERROR }
                        } else {
                            theme::FAINT
                        },
                        1.0,
                        true,
                    ),
                    Lane::Track => {
                        if let Some(bl) = p.b_label {
                            text_r(
                                window,
                                cx,
                                &format!("vs {bl}"),
                                vx,
                                ny + 1.0,
                                9.5,
                                theme::GHOST,
                                1.0,
                                false,
                            );
                        }
                    }
                    _ => {}
                }
            }
        }
        if !dynamic {
            text(window, cx, "Search", bx + 13.0, sy0 - 2.0, 9.5, theme::FAINT, 1.0, false);
            if p.paint_tool {
                text(window, cx, "painting", bx + 13.0, ry + 2.0, 9.5, theme::PIN, 1.0, false);
            }
        }
        let tag_w = |s: &str| s.len() as f32 * 6.0 + 12.0;
        if dynamic {
            if let Some((k, l)) = hover
                && l != Lane::Overview
            {
                let hx = x_of(k as f32);
                if hx >= x0 && hx <= x0 + w {
                    quad(
                        window,
                        fill(rect(hx - 0.5, ry, 1.0, bottom - ry), theme::alpha(theme::GHOST, 0.55)),
                    );
                    let s = fmt_ms(p.race_ms0 + 10 * k as i32);
                    let lw = tag_w(&s);
                    let lx = (hx - lw * 0.5).clamp(x0, x0 + w - lw);
                    quad(
                        window,
                        fill(rect(lx, ry + 1.5, lw, 15.0), theme::alpha(theme::PANEL, 1.0))
                            .border_widths(px(1.0))
                            .border_color(theme::alpha(theme::GHOST, 1.0))
                            .corner_radii(px(3.0)),
                    );
                    text(window, cx, &s, lx + 6.0, ry + 3.0, 9.5, theme::GHOST, 1.0, true);
                }
            }
            let cxp = x_of(cursor as f32);
            if cxp >= x0 && cxp <= x0 + w {
                quad(
                    window,
                    fill(
                        rect(cxp - 0.75, ry + 16.0, 1.5, bottom - ry - 16.0),
                        theme::alpha(theme::TEXT, 0.95),
                    ),
                );
                let label = fmt_ms(p.race_ms0 + 10 * cursor as i32);
                let lw = tag_w(&label);
                let lx = (cxp - lw * 0.5).clamp(x0, x0 + w - lw);
                quad(
                    window,
                    fill(rect(lx, ry + 1.5, lw, 15.0), theme::alpha(theme::TEXT, 1.0)).corner_radii(px(3.0)),
                );
                let mut n = PathBuilder::fill();
                n.move_to(point(px(cxp - 4.0), px(ry + 16.0)));
                n.line_to(point(px(cxp + 4.0), px(ry + 16.0)));
                n.line_to(point(px(cxp), px(ry + 21.0)));
                n.close();
                if let Ok(path) = n.build() {
                    draw_path(window, path, theme::alpha(theme::TEXT, 1.0));
                }
                text(window, cx, &label, lx + 6.0, ry + 3.0, 9.5, theme::BG, 1.0, true);
            }
        }
        let (ok0, ok1) = (p.overview.0 as f32, p.overview.1 as f32);
        let ox = |k: f32| x0 + (k - ok0) / (ok1 - ok0) * w;
        if !dynamic {
            quad(window, fill(rect(bx, oy - 2.0 * GAP - 1.0, bw, 1.0), theme::alpha(theme::HAIRLINE, 1.0)));
            quad(window, fill(rect(x0, oy, w, oh), theme::alpha(theme::WELL, 1.0)).corner_radii(px(3.0)));
            let vmax = p.speed.iter().copied().fold(50.0f32, f32::max) * 1.05;
            let mut line = PathBuilder::stroke(px(1.0));
            let mut started = false;
            let mut col = 0.0;
            while col < w {
                let k = (ok0 + col / w * (ok1 - ok0)) as usize;
                let Some(v) = p.speed.get(k) else { break };
                let pt = point(px(x0 + col), px(oy + oh - 3.0 - v / vmax * (oh - 6.0)));
                if started {
                    line.line_to(pt);
                } else {
                    line.move_to(pt);
                    started = true;
                }
                col += 2.0;
            }
            if started && let Ok(path) = line.build() {
                draw_path(window, path, theme::alpha(theme::MUTED, 0.6));
            }
            for m in p.marks.iter().filter(|m| (m.k as f32) <= ok1) {
                quad(window, fill(rect(ox(m.k as f32), oy + 2.0, 1.0, oh - 4.0), theme::alpha(m.color, 0.6)));
            }
            let n = p.controls.len().min(ok1 as usize);
            let mut k = 0;
            while k < n {
                if p.controls[k].brake > 0.0 {
                    let a = k;
                    while k < n && p.controls[k].brake > 0.0 {
                        k += 1;
                    }
                    let (xa, xb) = (ox(a as f32), ox(k as f32));
                    quad(
                        window,
                        fill(
                            rect(xa, oy + oh - 6.0, (xb - xa).max(2.0), 3.0),
                            theme::alpha(theme::ERROR, 0.95),
                        ),
                    );
                } else {
                    k += 1;
                }
            }
            let (owa, owb) = (ox(p.window.start as f32).max(x0), ox(p.window.end as f32).min(x0 + w));
            quad(window, fill(rect(owa, oy + oh - 3.0, owb - owa, 3.0), theme::alpha(theme::PIN, 0.8)));
            let (va, vb) = (ox(p.start as f32).max(x0), ox((p.start + w as f64 / p.ppt) as f32).min(x0 + w));
            quad(
                window,
                fill(rect(va, oy, vb - va, oh), theme::alpha(theme::TEXT, 0.08))
                    .border_widths(px(1.0))
                    .border_color(theme::alpha(theme::TEXT, 0.55))
                    .corner_radii(px(3.0)),
            );
            if vb - va > 14.0 {
                for x in [va + 3.0, vb - 5.0] {
                    quad(window, fill(rect(x, oy + 4.0, 2.0, oh - 8.0), theme::alpha(theme::TEXT, 0.6)));
                }
            }
        }
        if dynamic {
            quad(window, fill(rect(ox(cursor as f32) - 0.75, oy, 1.5, oh), theme::alpha(theme::TEXT, 0.95)));
        }
    });
}
