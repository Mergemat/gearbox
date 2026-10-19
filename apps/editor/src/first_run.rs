use gpui::{
    App, AppContext, Bounds, Context, FontWeight, IntoElement, ParentElement, PathPromptOptions,
    Render, StatefulInteractiveElement, Styled, TitlebarOptions, Window, WindowBounds, WindowOptions, div, px, size,
};
use gearbox_packs::install::{Game, Origin};

use crate::theme::{self, c};
use crate::ui::widgets::{Btn, btn};

type Continue = Box<dyn FnOnce(Game, &mut App)>;

pub struct Locate {
    error: Option<String>,
    choosing: bool,
    next: Option<Continue>,
}

pub fn open(cx: &mut App, next: impl FnOnce(Game, &mut App) + 'static) {
    let bounds = Bounds::centered(None, size(px(460.0), px(220.0)), cx);
    let options = WindowOptions {
        window_bounds: Some(WindowBounds::Windowed(bounds)),
        titlebar: Some(TitlebarOptions { title: Some("Gearbox".into()), appears_transparent: true, ..Default::default() }),
        focus: true,
        show: true,
        ..Default::default()
    };
    let next: Continue = Box::new(next);
    cx.open_window(options, move |_, cx| cx.new(|_| Locate { error: None, choosing: false, next: Some(next) }))
        .expect("opening the window");
}

impl Locate {
    fn choose(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if self.choosing {
            return;
        }
        self.choosing = true;
        let picked = cx.prompt_for_paths(PathPromptOptions {
            files: false,
            directories: true,
            multiple: false,
            prompt: Some("Use This Folder".into()),
        });
        let window = window.window_handle();
        cx.spawn(async move |this, cx| {
            let folder = picked.await.ok().and_then(Result::ok).flatten().and_then(|paths| paths.into_iter().next());
            let _ = this.update(cx, |locate, cx| {
                locate.choosing = false;
                let Some(folder) = folder else { return cx.notify() };
                match Game::at(&folder, Origin::Given).and_then(|game| game.save().map(|_| game)) {
                    Ok(game) => {
                        if let Some(next) = locate.next.take() {
                            let _ = window.update(cx, |_, window, _| window.remove_window());
                            next(game, cx);
                        }
                    }
                    Err(e) => locate.error = Some(format!("{e:#}")),
                }
                cx.notify();
            });
        })
        .detach();
    }
}

impl Render for Locate {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .size_full()
            .flex()
            .flex_col()
            .justify_center()
            .gap(px(theme::SP3))
            .px(px(theme::SP5))
            .bg(c(theme::BG))
            .text_color(c(theme::TEXT))
            .text_size(px(theme::FS_BODY))
            .child(div().text_size(px(theme::FS_TITLE)).font_weight(FontWeight::SEMIBOLD).child("Trackmania not found"))
            .child(div().text_color(c(theme::MUTED)).child(
                "Gearbox reads maps, blocks and cars from your Trackmania install. Choose the Trackmania folder or its Packs folder.",
            ))
            .children(self.error.clone().map(|e| div().text_color(c(theme::ERROR)).child(e)))
            .child(
                div().flex().child(
                    btn("choose-game", if self.choosing { "Choosing\u{2026}" } else { "Choose Folder\u{2026}" }, Btn::Primary)
                        .on_click(cx.listener(|this, _, window, cx| this.choose(window, cx))),
                ),
            )
    }
}
