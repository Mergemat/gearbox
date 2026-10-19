use gpui::prelude::FluentBuilder;
use gpui::{
    Context, InteractiveElement, IntoElement, ParentElement, StatefulInteractiveElement, Styled, div, px,
};

use crate::app::*;
use crate::theme::{self, c};
use crate::ui::widgets::*;

impl Editor {
    pub fn console(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let rows = self.log.iter().rev().take(500).map(|l| {
            let col = match l.level {
                LogLevel::Error => theme::ERROR,
                LogLevel::Warn => theme::WARN,
                LogLevel::Info => theme::TEXT,
            };
            div()
                .px(px(theme::PAD))
                .py(px(1.0))
                .flex()
                .gap(px(theme::SP3))
                .font_family(theme::MONO)
                .text_size(px(theme::FS_SMALL))
                .when(l.level == LogLevel::Error, |d| d.bg(theme::alpha(theme::ERROR, 0.08)))
                .child(div().w(px(84.0)).flex_none().text_color(c(theme::FAINT)).child(l.at.clone()))
                .child(div().w(px(50.0)).flex_none().text_color(c(theme::MUTED)).child(l.source))
                .child(div().flex_1().text_color(c(col)).child(l.text.clone()))
        });
        div()
            .size_full()
            .flex()
            .flex_col()
            .child(
                div()
                    .h(px(theme::H_ROW))
                    .flex_none()
                    .px(px(theme::PAD))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP2))
                    .child(muted(format!("{} messages \u{00b7} newest first", self.log.len())))
                    .child(div().flex_1())
                    .child(btn("log-clear", "Clear", Btn::Quiet).on_click(cx.listener(|e, _, _, cx| {
                        e.log.clear();
                        cx.notify();
                    }))),
            )
            .child(div().id("console-rows").flex_1().min_h_0().overflow_y_scroll().children(rows))
    }
}
