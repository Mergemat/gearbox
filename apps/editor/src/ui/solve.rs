use gpui::prelude::FluentBuilder;
use gpui::{AnyElement, Context, InteractiveElement, IntoElement, ParentElement, StatefulInteractiveElement, Styled, div, px};

use crate::app::*;
use crate::theme::{self, c};
use crate::ui::widgets::*;

const SHOWN_TAKES: usize = 6;

fn trick_badge(kind: gearbox_search::tricks::TrickKind, ticks: u32) -> gpui::Div {
    let col = crate::lab::trick_color(kind);
    div()
        .flex()
        .items_center()
        .gap(px(theme::SP1))
        .px(px(theme::SP2))
        .h(px(18.0))
        .rounded(px(9.0))
        .bg(theme::alpha(col, 0.16))
        .text_size(px(theme::FS_SMALL))
        .text_color(c(col))
        .child(format!("{} {:.2} s", crate::lab::trick_label(kind), ticks as f32 / 100.0))
}

pub fn trick_badges(t: Option<[u32; 3]>) -> gpui::Div {
    use gearbox_search::tricks::TrickKind as K;
    let mut row = div().flex().flex_wrap().gap(px(theme::SP1));
    let Some(t) = t else { return row.child(muted("not counted")) };
    let mut any = false;
    for (k, n) in [(K::Nosebug, t[0]), (K::Bugslide, t[1]), (K::Uberbug, t[2])] {
        if n > 0 {
            row = row.child(trick_badge(k, n));
            any = true;
        }
    }
    if !any {
        row = row.child(muted("none"));
    }
    row
}

impl Editor {
    pub fn run_button(&mut self, id: &str, label: &str, go: impl Fn(&mut Editor) + 'static, cx: &mut Context<Self>) -> AnyElement {
        let attempts = self.search.live.lock().unwrap().attempts;
        if self.search.running() {
            div()
                .flex()
                .items_center()
                .gap(px(theme::SP3))
                .child(btn(format!("{id}-stop"), "Stop", Btn::Danger).on_click(cx.listener(|this, _, _, cx| {
                    this.search.stop();
                    cx.notify();
                })))
                .child(muted(format!("{attempts} tries")))
                .into_any_element()
        } else {
            btn(id.to_string(), label.to_string(), Btn::Primary)
                .on_click(cx.listener(move |this, _, _, cx| {
                    go(this);
                    cx.notify();
                }))
                .into_any_element()
        }
    }

    pub fn takes(&mut self, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let (cands, err) = {
            let l = self.search.live.lock().unwrap();
            (l.candidates.clone(), l.error.clone())
        };
        let mut v: Vec<AnyElement> = Vec::new();
        if cands.is_empty() && err.is_none() {
            return v;
        }
        v.push(heading(format!("Takes \u{00b7} {}", cands.len())).into_any_element());
        if let Some(e) = err {
            v.push(div().px(px(theme::PAD)).pb(px(theme::SP2)).text_size(px(theme::FS_BODY)).text_color(c(theme::ERROR)).child(e).into_any_element());
        }
        let n = cands.len();
        for (i, cand) in cands.iter().rev().take(SHOWN_TAKES).enumerate() {
            let id = cand.id;
            let on = self.preview == Some(id);
            let m = self.search.live.lock().unwrap().metric(&cand.score);
            v.push(
                div()
                    .id(("take", id as usize))
                    .relative()
                    .child(track(format!("take-{id}")))
                    .mx(px(theme::SP2))
                    .px(px(theme::SP2))
                    .h(px(30.0))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP3))
                    .rounded(px(theme::R_CTL))
                    .cursor_pointer()
                    .when(on, |d| d.bg(c(theme::SELECT)))
                    .when(!on, |d| d.hover(|s| s.bg(c(theme::RAISED))))
                    .child(div().w(px(52.0)).flex_none().text_size(px(theme::FS_BODY)).text_color(c(if i == 0 { theme::TEXT } else { theme::MUTED })).child(format!("Take {}", n - i)))
                    .child(div().flex_1().min_w_0().truncate().font_family(theme::MONO).text_size(px(theme::FS_SMALL)).text_color(c(if i == 0 { theme::OK } else { theme::MUTED })).child(m.text))
                    .child(btn(format!("take-use-{id}"), "Apply", if i == 0 { Btn::Normal } else { Btn::Quiet }).tooltip(tip("Apply this take", Some("It becomes your run (undoable; the old one goes to History)."))).on_click(cx.listener(move |this, _, _, cx| {
                        this.accept(id);
                        cx.stop_propagation();
                        cx.notify();
                    })))
                    .on_click(cx.listener(move |this, _, _, cx| {
                        if this.preview == Some(id) {
                            this.preview = None;
                        } else {
                            this.preview = Some(id);
                            this.auto_preview = false;
                            let _ = this.ref_frames(RunRef::Candidate(id));
                        }
                        this.dirty = true;
                        cx.notify();
                    }))
                    .into_any_element(),
            );
        }
        if n > SHOWN_TAKES {
            v.push(div().px(px(theme::PAD)).py(px(theme::SP1)).child(btn("takes-all", format!("All {n} in Search"), Btn::Quiet).on_click(cx.listener(|this, _, _, cx| {
                this.layout.right_tab = 1;
                cx.notify();
            }))).into_any_element());
        }
        v
    }
}
