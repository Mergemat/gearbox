use std::rc::Rc;

use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, AnyView, App, AppContext, ClickEvent, Context, Div, FontWeight, InteractiveElement, IntoElement, MouseButton, MouseDownEvent, ParentElement, Render, SharedString, Stateful,
    StatefulInteractiveElement, Styled, Window, div, px,
};

use crate::app::{Drag, Editor, Popup, Setter};
use crate::theme::{self, c};

thread_local! {
    pub static HITS: std::cell::RefCell<std::collections::HashMap<String, gpui::Bounds<gpui::Pixels>>> = Default::default();
}

pub fn track(id: impl Into<String>) -> impl IntoElement {
    let id = id.into();
    gpui::canvas(
        move |b, window, _| {
            let seen = b.intersect(&window.content_mask().bounds);
            if seen.size.width > px(0.0) && seen.size.height > px(0.0) {
                HITS.with(|h| h.borrow_mut().insert(id, seen));
            }
        },
        |_, _, _, _| {},
    )
    .absolute()
    .top_0()
    .left_0()
    .size_full()
}

pub struct FieldEdit {
    pub id: String,
    pub text: String,
    pub fresh: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Num {
    pub decimals: usize,
    pub unit: &'static str,
    pub step: f64,
    pub min: f64,
    pub max: f64,
    pub width: f32,
}

impl Num {
    pub fn new(decimals: usize, unit: &'static str, step: f64) -> Self {
        Self { decimals, unit, step, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 }
    }
    pub fn range(mut self, min: f64, max: f64) -> Self {
        self.min = min;
        self.max = max;
        self
    }
    pub fn w(mut self, w: f32) -> Self {
        self.width = w;
        self
    }
    pub fn fmt(&self, v: f64) -> String {
        format!("{:.*}", self.decimals, v)
    }
}

pub const M: Num = Num { decimals: 3, unit: "m", step: 0.02, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 };
pub const DEG: Num = Num { decimals: 2, unit: "\u{00b0}", step: 0.25, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 };
pub const MS: Num = Num { decimals: 3, unit: "m/s", step: 0.05, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 };
pub const KMH: Num = Num { decimals: 1, unit: "km/h", step: 0.2, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 };
pub const SEC: Num = Num { decimals: 3, unit: "s", step: 0.01, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 };
pub const PLAIN: Num = Num { decimals: 3, unit: "", step: 0.01, min: f64::NEG_INFINITY, max: f64::INFINITY, width: 0.0 };
pub const INT: Num = Num { decimals: 0, unit: "", step: 0.25, min: 0.0, max: 1e9, width: 0.0 };

pub fn eval(s: &str) -> Option<f64> {
    struct P<'a> {
        s: &'a [u8],
        i: usize,
    }
    impl P<'_> {
        fn ws(&mut self) {
            while self.i < self.s.len() && self.s[self.i] == b' ' {
                self.i += 1;
            }
        }
        fn expr(&mut self) -> Option<f64> {
            let mut v = self.term()?;
            loop {
                self.ws();
                match self.s.get(self.i) {
                    Some(b'+') => {
                        self.i += 1;
                        v += self.term()?
                    }
                    Some(b'-') => {
                        self.i += 1;
                        v -= self.term()?
                    }
                    _ => return Some(v),
                }
            }
        }
        fn term(&mut self) -> Option<f64> {
            let mut v = self.atom()?;
            loop {
                self.ws();
                match self.s.get(self.i) {
                    Some(b'*') => {
                        self.i += 1;
                        v *= self.atom()?
                    }
                    Some(b'/') => {
                        self.i += 1;
                        v /= self.atom()?
                    }
                    _ => return Some(v),
                }
            }
        }
        fn atom(&mut self) -> Option<f64> {
            self.ws();
            match self.s.get(self.i)? {
                b'-' => {
                    self.i += 1;
                    Some(-self.atom()?)
                }
                b'+' => {
                    self.i += 1;
                    self.atom()
                }
                b'(' => {
                    self.i += 1;
                    let v = self.expr()?;
                    self.ws();
                    (self.s.get(self.i) == Some(&b')')).then(|| {
                        self.i += 1;
                        v
                    })
                }
                _ => {
                    let st = self.i;
                    while self.i < self.s.len() && (self.s[self.i].is_ascii_digit() || self.s[self.i] == b'.' || ((self.s[self.i] == b'e' || self.s[self.i] == b'E') && self.i > st)) {
                        self.i += 1;
                    }
                    std::str::from_utf8(&self.s[st..self.i]).ok()?.parse().ok()
                }
            }
        }
    }
    let t = s.trim().replace(',', ".").replace('\u{2212}', "-");
    let mut p = P { s: t.as_bytes(), i: 0 };
    let v = p.expr()?;
    p.ws();
    (p.i == p.s.len() && v.is_finite()).then_some(v)
}

pub struct Tip {
    pub title: SharedString,
    pub body: Option<SharedString>,
}

impl Render for Tip {
    fn render(&mut self, _: &mut Window, _: &mut Context<Self>) -> impl IntoElement {
        div()
            .max_w(px(340.0))
            .px(px(theme::SP2))
            .py(px(theme::SP2))
            .rounded(px(theme::R_CTL))
            .bg(c(theme::RAISED))
            .border_1()
            .border_color(c(theme::HAIRLINE))
            .font_family(theme::SANS)
            .text_size(px(theme::FS_BODY))
            .text_color(c(theme::TEXT))
            .child(self.title.clone())
            .children(self.body.clone().map(|b| div().mt(px(3.0)).text_color(c(theme::MUTED)).child(b)))
    }
}

pub fn tip(title: impl Into<SharedString>, body: Option<&str>) -> impl Fn(&mut Window, &mut App) -> AnyView + 'static {
    let (title, body): (SharedString, Option<SharedString>) = (title.into(), body.map(|b| SharedString::from(b.to_string())));
    move |_, cx| cx.new(|_| Tip { title: title.clone(), body: body.clone() }).into()
}

pub fn text(s: impl Into<SharedString>) -> Div {
    div().text_size(px(theme::FS_BODY)).text_color(c(theme::TEXT)).child(s.into())
}

pub fn muted(s: impl Into<SharedString>) -> Div {
    div().text_size(px(theme::FS_BODY)).text_color(c(theme::MUTED)).child(s.into())
}

pub fn mono(s: impl Into<SharedString>, size: f32) -> Div {
    div().font_family(theme::MONO).text_size(px(size)).child(s.into())
}

pub fn kbd(s: impl Into<SharedString>) -> Div {
    div()
        .px(px(theme::SP1))
        .h(px(16.0))
        .flex()
        .items_center()
        .rounded(px(3.0))
        .border_1()
        .border_color(c(theme::HAIRLINE))
        .bg(c(theme::WELL))
        .font_family(theme::MONO)
        .text_size(px(theme::FS_SMALL))
        .text_color(c(theme::MUTED))
        .child(s.into())
}

pub fn heading(s: impl Into<SharedString>) -> Div {
    div()
        .mt(px(theme::SP4 + 2.0))
        .mb(px(theme::SP1))
        .h(px(24.0))
        .px(px(theme::PAD))
        .flex()
        .items_center()
        .gap(px(theme::SP2))
        .text_size(px(theme::FS_EMPH))
        .font_weight(FontWeight::SEMIBOLD)
        .text_color(c(theme::TEXT))
        .child(s.into())
}

pub fn prop(label: impl Into<SharedString>, content: impl IntoElement) -> Div {
    div()
        .min_h(px(34.0))
        .px(px(theme::PAD))
        .flex()
        .items_center()
        .gap(px(theme::SP3))
        .child(div().w(px(88.0)).flex_none().text_size(px(theme::FS_BODY)).text_color(c(theme::MUTED)).truncate().child(label.into()))
        .child(div().flex_1().min_w_0().child(content))
}

pub fn prop_top(label: impl Into<SharedString>, content: impl IntoElement) -> Div {
    div()
        .py(px(5.0))
        .px(px(theme::PAD))
        .flex()
        .items_start()
        .gap(px(theme::SP3))
        .child(div().w(px(88.0)).h(px(24.0)).flex_none().flex().items_center().text_size(px(theme::FS_BODY)).text_color(c(theme::MUTED)).truncate().child(label.into()))
        .child(div().flex_1().min_w_0().child(content))
}

pub fn segment(id: impl Into<SharedString>, on: bool, label: impl Into<SharedString>) -> Stateful<Div> {
    let id: SharedString = id.into();
    let label: SharedString = label.into();
    let (path, label) = match label.strip_prefix('@').and_then(|l| l.split_once('|')) {
        Some((p, l)) => (Some(p.to_string()), SharedString::from(l.to_string())),
        None => (None, label),
    };
    div()
        .id(gpui::ElementId::Name(id.clone()))
        .relative()
        .child(track(id.to_string()))
        .h(px(22.0))
        .px(px(theme::SP2 + 2.0))
        .flex()
        .items_center()
        .rounded(px(3.0))
        .text_size(px(theme::FS_BODY))
        .cursor_pointer()
        .when(on, |d| d.bg(c(theme::SELECT)).text_color(c(theme::TEXT)))
        .when(!on, |d| d.text_color(c(theme::FAINT)).hover(|s| s.text_color(c(theme::TEXT))))
        .when_some(path, |d, p| d.gap(px(theme::SP1 + 1.0)).child(icon(p, if on { theme::TEXT } else { theme::FAINT }, 13.0)))
        .child(label)
}

pub fn segments() -> Div {
    div().flex().flex_none().items_center().p(px(2.0)).gap(px(2.0)).rounded(px(theme::R_CTL)).bg(c(theme::WELL)).border_1().border_color(c(theme::HAIRLINE))
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Btn {
    Normal,
    Primary,
    Quiet,
    Danger,
}

pub fn icon(path: impl Into<SharedString>, color: u32, size: f32) -> gpui::Svg {
    gpui::svg().path(path).size(px(size)).flex_none().text_color(c(color))
}

pub fn labelled(label: &str, color: u32) -> Div {
    match label.strip_prefix('@').and_then(|l| l.split_once('|')) {
        Some((path, text)) => div().flex().items_center().gap(px(theme::SP2)).min_w_0().child(icon(path.to_string(), color, 13.0)).child(div().min_w_0().truncate().child(text.to_string())),
        None => div().min_w_0().truncate().child(label.to_string()),
    }
}

pub fn btn_icon(id: impl Into<SharedString>, path: &'static str, label: impl Into<SharedString>, kind: Btn) -> Stateful<Div> {
    let tint = match kind {
        Btn::Primary => 0x1B0A12,
        Btn::Quiet => theme::MUTED,
        _ => theme::TEXT,
    };
    let label: SharedString = label.into();
    btn(id, "", kind).gap(px(theme::SP2)).child(icon(path, tint, 13.0)).child(label)
}

pub fn btn(id: impl Into<SharedString>, label: impl Into<SharedString>, kind: Btn) -> Stateful<Div> {
    let id: SharedString = id.into();
    let d = div()
        .id(gpui::ElementId::Name(id.clone()))
        .h(px(theme::H_CTL))
        .px(px(theme::PAD))
        .flex()
        .flex_none()
        .items_center()
        .justify_center()
        .gap(px(theme::SP1))
        .rounded(px(theme::R_CTL))
        .text_size(px(theme::FS_BODY))
        .whitespace_nowrap()
        .cursor_pointer()
        .relative()
        .child(track(id.to_string()))
        .child(label.into());
    match kind {
        Btn::Normal => d.bg(c(theme::RAISED)).border_1().border_color(c(theme::HAIRLINE)).text_color(c(theme::TEXT)).hover(|s| s.bg(c(theme::SELECT))),
        Btn::Primary => d.bg(c(theme::PIN)).text_color(gpui::rgb(0x1B0A12)).font_weight(FontWeight::SEMIBOLD).hover(|s| s.opacity(0.9)),
        Btn::Quiet => d.text_color(c(theme::MUTED)).hover(|s| s.bg(c(theme::RAISED)).text_color(c(theme::TEXT))),
        Btn::Danger => d.border_1().border_color(c(theme::HAIRLINE)).text_color(c(theme::ERROR)).hover(|s| s.bg(c(theme::RAISED))),
    }
}

pub fn icon_btn(id: impl Into<SharedString>, glyph: &'static str, what: &'static str) -> Stateful<Div> {
    let id: SharedString = id.into();
    div()
        .id(gpui::ElementId::Name(id.clone()))
        .relative()
        .child(track(id.to_string()))
        .size(px(24.0))
        .flex_none()
        .flex()
        .items_center()
        .justify_center()
        .rounded(px(theme::R_CTL))
        .bg(c(theme::RAISED))
        .border_1()
        .border_color(c(theme::HAIRLINE))
        .text_size(px(theme::FS_BODY))
        .text_color(c(theme::MUTED))
        .cursor_pointer()
        .hover(|s| s.text_color(c(theme::TEXT)).border_color(c(theme::FAINT)))
        .child(glyph)
        .tooltip(tip(what, None))
}

pub fn icon_square(id: impl Into<SharedString>, path: &'static str, on: bool) -> Stateful<Div> {
    let id: SharedString = id.into();
    div()
        .id(gpui::ElementId::Name(id.clone()))
        .relative()
        .child(track(id.to_string()))
        .size(px(theme::H_CTL))
        .flex_none()
        .flex()
        .items_center()
        .justify_center()
        .rounded(px(theme::R_CTL))
        .cursor_pointer()
        .when(on, |d| d.bg(c(theme::SELECT)))
        .when(!on, |d| d.hover(|s| s.bg(c(theme::RAISED))))
        .child(icon(path, if on { theme::TEXT } else { theme::MUTED }, 14.0))
}

pub fn chip_icon(id: impl Into<SharedString>, on: bool, path: &'static str, label: impl Into<SharedString>) -> Stateful<Div> {
    let label: SharedString = label.into();
    chip(id, on, "").gap(px(theme::SP1 + 2.0)).child(icon(path, if on { theme::TEXT } else { theme::MUTED }, 13.0)).child(label)
}

pub fn chip(id: impl Into<SharedString>, on: bool, label: impl Into<SharedString>) -> Stateful<Div> {
    let id: SharedString = id.into();
    div()
        .id(gpui::ElementId::Name(id.clone()))
        .relative()
        .child(track(id.to_string()))
        .h(px(theme::H_CTL))
        .px(px(theme::SP2))
        .flex()
        .flex_none()
        .items_center()
        .gap(px(theme::SP1))
        .rounded(px(theme::R_CTL))
        .text_size(px(theme::FS_BODY))
        .cursor_pointer()
        .whitespace_nowrap()
        .when(on, |d| d.bg(c(theme::SELECT)).text_color(c(theme::TEXT)).border_1().border_color(c(theme::FAINT)))
        .when(!on, |d| d.text_color(c(theme::MUTED)).border_1().border_color(c(theme::HAIRLINE)).hover(|s| s.text_color(c(theme::TEXT))))
        .child(label.into())
}

pub fn checkbox(id: impl Into<SharedString>, on: bool, label: impl Into<SharedString>) -> Stateful<Div> {
    let id: SharedString = id.into();
    div()
        .id(gpui::ElementId::Name(id.clone()))
        .relative()
        .child(track(id.to_string()))
        .h(px(20.0))
        .flex()
        .items_center()
        .gap(px(theme::SP2))
        .cursor_pointer()
        .text_size(px(theme::FS_BODY))
        .text_color(c(if on { theme::TEXT } else { theme::MUTED }))
        .child(
            div()
                .size(px(12.0))
                .rounded(px(3.0))
                .border_1()
                .border_color(c(if on { theme::PIN } else { theme::FAINT }))
                .when(on, |d| d.bg(c(theme::PIN)))
                .flex()
                .items_center()
                .justify_center()
                .text_size(px(theme::FS_SMALL))
                .text_color(gpui::rgb(0x1B0A12))
                .child(if on { "\u{2713}" } else { "" }),
        )
        .child(label.into())
}

impl Editor {
    pub fn num(&mut self, id: impl Into<String>, v: f64, f: Num, set: impl Fn(&mut Editor, f64, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let id: String = id.into();
        let (min, max) = (f.min, f.max);
        let set = Rc::new(move |e: &mut Editor, x: f64, cx: &mut Context<Editor>| set(e, x.clamp(min, max), cx));
        self.setters.insert(id.clone(), Setter::Num(set));
        self.field_order.push(id.clone());
        self.field_value.insert(id.clone(), f.fmt(v));
        let editing = self.editing.as_ref().filter(|e| e.id == id);
        let err = self.field_errors.get(&id).cloned();
        let shown = match editing {
            Some(e) => e.text.clone(),
            None => f.fmt(v),
        };
        let caret = editing.is_some();
        let fresh = editing.is_some_and(|e| e.fresh);
        let el_id: SharedString = format!("num:{id}").into();
        let drag_id = id.clone();
        let mut d = div()
            .id(gpui::ElementId::Name(el_id.clone()))
            .relative()
            .child(track(el_id.to_string()))
            .h(px(24.0))
            .px(px(theme::SP1))
            .flex()
            .items_center()
            .gap(px(theme::SP1))
            .rounded(px(theme::R_CTL))
            .bg(c(theme::WELL))
            .border_1()
            .border_color(c(if err.is_some() { theme::ERROR } else if caret { theme::PIN } else { theme::HAIRLINE }))
            .font_family(theme::MONO)
            .text_size(px(theme::FS_SMALL))
            .cursor_ew_resize()
            .hover(|s| s.border_color(c(theme::FAINT)))
            .child(
                div()
                    .flex_1()
                    .min_w_0()
                    .flex()
                    .justify_end()
                    .overflow_hidden()
                    .text_color(c(theme::TEXT))
                    .when(fresh, |d| d.bg(theme::alpha(theme::PIN, 0.25)))
                    .child(shown)
                    .when(caret && !fresh, |d| d.child(div().w(px(1.0)).h(px(12.0)).bg(c(theme::PIN)))),
            )
            .when(!f.unit.is_empty(), |d| d.child(div().flex_none().text_color(c(theme::FAINT)).text_size(px(theme::FS_SMALL)).child(f.unit)))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |this, e: &MouseDownEvent, _, cx| {
                    if this.editing.as_ref().is_some_and(|x| x.id == drag_id) {
                        return;
                    }
                    this.commit_field(cx);
                    let mut step = f.step;
                    if e.modifiers.shift {
                        step *= 0.1;
                    }
                    if e.modifiers.platform {
                        step *= 10.0;
                    }
                    this.drag = Some(Drag::Field { id: drag_id.clone(), x0: f32::from(e.position.x), v0: v, step, moved: false });
                    cx.stop_propagation();
                }),
            );
        d = if f.width > 0.0 { d.w(px(f.width)) } else { d.flex_1().min_w(px(54.0)) };
        if let Some(e) = err {
            d = d.tooltip(tip("Not applied", Some(&e)));
        }
        d.into_any_element()
    }

    pub fn choice<T: Copy + PartialEq + 'static>(&mut self, id: &str, current: T, items: Vec<(&str, T)>, set: impl Fn(&mut Editor, T, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let set = Rc::new(set);
        div()
            .flex()
            .flex_none()
            .child(segments().children(items.into_iter().map(|(label, v)| {
                let set = set.clone();
                let key = label.rsplit('|').next().unwrap_or(label);
                segment(format!("{id}-{key}"), v == current, label.to_string()).on_click(cx.listener(move |this, _, _, cx| {
                    set(this, v, cx);
                    this.dirty = true;
                    cx.notify();
                }))
            })))
            .into_any_element()
    }

    pub fn mini_choice<T: Copy + PartialEq + 'static>(&mut self, id: &str, current: T, items: Vec<(&str, T)>, set: impl Fn(&mut Editor, T, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let set = Rc::new(set);
        div()
            .flex()
            .child(segments().children(items.into_iter().map(|(label, v)| {
                let set = set.clone();
                segment(format!("{id}-{label}"), v == current, label.to_string()).h(px(18.0)).px(px(theme::SP1 + 2.0)).text_size(px(theme::FS_SMALL)).on_click(cx.listener(move |this, _, _, cx| {
                    set(this, v, cx);
                    this.dirty = true;
                    cx.notify();
                }))
            })))
            .into_any_element()
    }

    pub fn time_field(&mut self, id: &str, tick: usize, set: impl Fn(&mut Editor, usize, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let ms0 = self.race_ms0;
        let set = Rc::new(set);
        let s2 = set.clone();
        let secs = f64::from(ms0 + 10 * tick as i32) / 1000.0;
        let f = self.num(id, secs, Num { decimals: 2, ..SEC }.w(92.0), move |e, x, cx| {
            let k = (((x * 1000.0).round() as i32 - ms0) / 10).max(0) as usize;
            set(e, k.max(e.race_start_tick()), cx)
        }, cx);
        div()
            .flex()
            .items_center()
            .gap(px(theme::SP1))
            .child(f)
            .child(icon_btn(format!("{id}-playhead"), "\u{2316}", "Take the playhead").on_click(cx.listener(move |this, _, _, cx| {
                let k = this.cursor;
                s2(this, k, cx);
                this.dirty = true;
                cx.notify();
            })))
            .into_any_element()
    }

    pub fn duration_field(&mut self, id: &str, ticks: usize, set: impl Fn(&mut Editor, usize, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let f = self.num(id, ticks as f64 / 100.0, Num { decimals: 2, min: 0.01, max: 600.0, ..SEC }.w(92.0), move |e, x, cx| set(e, (x * 100.0).round().max(1.0) as usize, cx), cx);
        div().flex().child(f).into_any_element()
    }

    pub fn budget_field(&mut self, id: &str, seconds: f32, set: impl Fn(&mut Editor, f32, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let set = Rc::new(set);
        let (s1, s2) = (set.clone(), set);
        let until = seconds <= 0.0;
        let f = self.num(id, if until { 120.0 } else { f64::from(seconds) }, Num { decimals: 0, step: 1.0, min: 1.0, max: 86400.0, ..SEC }.w(92.0), move |e, x, cx| s1(e, x as f32, cx), cx);
        div()
            .flex()
            .items_center()
            .gap(px(theme::SP3))
            .child(div().when(until, |d| d.opacity(0.4)).child(f))
            .child(checkbox(format!("{id}-until"), until, "until stopped").on_click(cx.listener(move |this, _, _, cx| {
                s2(this, if until { 120.0 } else { 0.0 }, cx);
                cx.notify();
            })))
            .into_any_element()
    }

    pub fn text_field(&mut self, id: impl Into<String>, value: &str, set: impl Fn(&mut Editor, String, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let id: String = id.into();
        self.setters.insert(id.clone(), Setter::Text(Rc::new(set)));
        self.field_order.push(id.clone());
        self.field_value.insert(id.clone(), value.to_string());
        let editing = self.editing.as_ref().filter(|e| e.id == id);
        let shown = editing.map_or(value.to_string(), |e| e.text.clone());
        let caret = editing.is_some();
        let fresh = editing.is_some_and(|e| e.fresh);
        let (vid, value) = (id.clone(), value.to_string());
        div()
            .id(gpui::ElementId::Name(format!("text:{id}").into()))
            .relative()
            .child(track(format!("text:{id}")))
            .h(px(24.0))
            .flex_1()
            .min_w_0()
            .px(px(theme::SP2))
            .flex()
            .items_center()
            .rounded(px(theme::R_CTL))
            .bg(c(theme::WELL))
            .border_1()
            .border_color(c(if caret { theme::PIN } else { theme::HAIRLINE }))
            .text_size(px(theme::FS_BODY))
            .cursor_text()
            .child(div().overflow_hidden().truncate().when(fresh, |d| d.bg(theme::alpha(theme::PIN, 0.25))).child(shown))
            .when(caret && !fresh, |d| d.child(div().w(px(1.0)).h(px(13.0)).bg(c(theme::PIN))))
            .on_click(cx.listener(move |this, _: &ClickEvent, _, cx| {
                if this.editing.as_ref().is_none_or(|e| e.id != vid) {
                    this.commit_field(cx);
                    this.editing = Some(FieldEdit { id: vid.clone(), text: value.clone(), fresh: true });
                }
                cx.notify();
            }))
            .into_any_element()
    }

    pub fn menu_button<T: Copy + 'static>(&mut self, id: &str, label: &str, items: Vec<(String, T)>, set: impl Fn(&mut Editor, T, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let set = Rc::new(set);
        let empty = items.is_empty();
        div()
            .id(gpui::ElementId::Name(format!("menu:{id}").into()))
            .relative()
            .child(track(format!("menu:{id}")))
            .h(px(theme::H_CTL))
            .px(px(theme::SP2))
            .flex()
            .flex_none()
            .items_center()
            .rounded(px(theme::R_CTL))
            .text_size(px(theme::FS_BODY))
            .text_color(c(if empty { theme::FAINT } else { theme::MUTED }))
            .when(!empty, |d| d.cursor_pointer().hover(|s| s.text_color(c(theme::TEXT)).bg(c(theme::RAISED))))
            .child(label.to_string())
            .when(!empty, |d| {
                d.on_mouse_down(
                    MouseButton::Left,
                    cx.listener(move |this, e: &MouseDownEvent, _, cx| {
                        let items = items
                            .iter()
                            .map(|(l, v)| {
                                let (set, v) = (set.clone(), *v);
                                let f: Rc<dyn Fn(&mut Editor, &mut Context<Editor>)> = Rc::new(move |ed, cx| set(ed, v, cx));
                                (l.clone(), None, f)
                            })
                            .collect();
                        this.popup = Some(Popup { at: e.position, items });
                        cx.stop_propagation();
                        cx.notify();
                    }),
                )
            })
            .into_any_element()
    }

    pub fn dropdown<T: Copy + PartialEq + 'static>(&mut self, id: &str, current: T, items: Vec<(String, T)>, set: impl Fn(&mut Editor, T, &mut Context<Editor>) + 'static, cx: &mut Context<Editor>) -> AnyElement {
        let label = items.iter().find(|i| i.1 == current).map_or("\u{2014}".to_string(), |i| i.0.clone());
        let set = Rc::new(set);
        div()
            .id(gpui::ElementId::Name(format!("sel:{id}").into()))
            .relative()
            .child(track(format!("sel:{id}")))
            .h(px(24.0))
            .flex_1()
            .min_w_0()
            .px(px(theme::SP2))
            .flex()
            .items_center()
            .gap(px(theme::SP1))
            .rounded(px(theme::R_CTL))
            .bg(c(theme::RAISED))
            .border_1()
            .border_color(c(theme::HAIRLINE))
            .text_size(px(theme::FS_BODY))
            .cursor_pointer()
            .hover(|s| s.border_color(c(theme::FAINT)))
            .child(labelled(&label, theme::MUTED).flex_1())
            .child(div().text_color(c(theme::MUTED)).text_size(px(theme::FS_SMALL)).child("\u{25be}"))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |this, e: &MouseDownEvent, _, cx| {
                    let items = items
                        .iter()
                        .map(|(l, v)| {
                            let (set, v) = (set.clone(), *v);
                            let f: Rc<dyn Fn(&mut Editor, &mut Context<Editor>)> = Rc::new(move |ed, cx| set(ed, v, cx));
                            (l.clone(), (v == current).then(|| "\u{2713}".to_string()), f)
                        })
                        .collect();
                    this.popup = Some(Popup { at: e.position, items });
                    cx.stop_propagation();
                    cx.notify();
                }),
            )
            .into_any_element()
    }

    pub fn commit_field(&mut self, cx: &mut Context<Editor>) {
        let Some(e) = self.editing.take() else { return };
        match self.setters.get(&e.id) {
            Some(Setter::Num(f)) => match eval(&e.text) {
                Some(v) => {
                    let f = f.clone();
                    self.field_errors.remove(&e.id);
                    f(self, v, cx);
                }
                None if e.fresh => {}
                None => {
                    self.field_errors.insert(e.id.clone(), format!("\u{201c}{}\u{201d} is not a number", e.text));
                    let id = e.id.clone();
                    self.editing = Some(e);
                    self.error("edit", format!("{id}: not a number"));
                }
            },
            Some(Setter::Text(f)) => {
                let f = f.clone();
                f(self, e.text.trim().to_string(), cx);
            }
            None => {}
        }
        self.dirty = true;
    }

    pub fn field_key(&mut self, key: &str, ch: Option<&str>, cx: &mut Context<Editor>) -> bool {
        let Some(e) = &mut self.editing else { return false };
        match key {
            "enter" => self.commit_field(cx),
            "escape" => {
                let id = e.id.clone();
                self.field_errors.remove(&id);
                self.editing = None;
            }
            "tab" => {
                let id = e.id.clone();
                self.commit_field(cx);
                if self.editing.is_none()
                    && let Some(i) = self.field_order.iter().position(|f| *f == id)
                    && let Some(next) = self.field_order.get(i + 1).cloned()
                {
                    self.begin_edit(&next);
                }
            }
            "backspace" => {
                if e.fresh {
                    e.text.clear();
                    e.fresh = false;
                } else {
                    e.text.pop();
                }
            }
            _ => match ch {
                Some(s) if !s.is_empty() && s.chars().all(|c| !c.is_control()) => {
                    if e.fresh {
                        e.text.clear();
                        e.fresh = false;
                    }
                    e.text.push_str(s);
                }
                _ => return true,
            },
        }
        cx.notify();
        true
    }

    pub fn begin_edit(&mut self, id: &str) {
        let text = self.field_value.get(id).cloned().unwrap_or_default();
        self.editing = Some(FieldEdit { id: id.to_string(), text, fresh: true });
    }
}

impl Editor {
}
