use std::cell::Cell;

use gpui::{
    AnyElement, AppContext, Context, Entity, IntoElement, ParentElement, Render, StyleRefinement, Styled,
    WeakEntity, Window, div,
};

use crate::app::Editor;
use crate::theme::{self, c};
use crate::ui::widgets::HITS;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PaneKind {
    Root,
    Title,
    Outliner,
    ViewHeader,
    Viewport,
    ViewRail,
    Right,
    Bottom,
}

thread_local! {
    static CURRENT: Cell<PaneKind> = const { Cell::new(PaneKind::Root) };
}

pub fn current() -> PaneKind {
    CURRENT.with(Cell::get)
}

pub struct Pane {
    editor: WeakEntity<Editor>,
    kind: PaneKind,
}

impl Render for Pane {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let kind = self.kind;
        self.editor
            .update(cx, |e, cx| e.render_pane(kind, window, cx))
            .unwrap_or_else(|_| div().into_any_element())
    }
}

pub struct Panes {
    pub title: Entity<Pane>,
    pub outliner: Entity<Pane>,
    pub view_header: Entity<Pane>,
    pub viewport: Entity<Pane>,
    pub view_rail: Entity<Pane>,
    pub right: Entity<Pane>,
    pub bottom: Entity<Pane>,
}

impl Panes {
    pub fn new(cx: &mut Context<Editor>) -> Self {
        let editor = cx.entity();
        cx.observe_self(|e, _| e.rev += 1).detach();
        let mut pane = |kind| {
            let weak = editor.downgrade();
            cx.new(|cx: &mut Context<Pane>| {
                cx.observe(&editor, |_, _, cx| cx.notify()).detach();
                Pane { editor: weak, kind }
            })
        };
        Self {
            title: pane(PaneKind::Title),
            outliner: pane(PaneKind::Outliner),
            view_header: pane(PaneKind::ViewHeader),
            viewport: pane(PaneKind::Viewport),
            view_rail: pane(PaneKind::ViewRail),
            right: pane(PaneKind::Right),
            bottom: pane(PaneKind::Bottom),
        }
    }

    fn get(&self, kind: PaneKind) -> Option<&Entity<Pane>> {
        match kind {
            PaneKind::Root => None,
            PaneKind::Title => Some(&self.title),
            PaneKind::Outliner => Some(&self.outliner),
            PaneKind::ViewHeader => Some(&self.view_header),
            PaneKind::Viewport => Some(&self.viewport),
            PaneKind::ViewRail => Some(&self.view_rail),
            PaneKind::Right => Some(&self.right),
            PaneKind::Bottom => Some(&self.bottom),
        }
    }
}

pub fn cached(pane: &Entity<Pane>, style: StyleRefinement) -> AnyElement {
    gpui::AnyView::from(pane.clone()).cached(style).into_any_element()
}

impl Editor {
    pub fn begin_pane(&mut self, kind: PaneKind) {
        CURRENT.with(|c| c.set(kind));
        let mut gone = Vec::new();
        self.field_order.retain(|(p, id)| {
            if *p == kind {
                gone.push(id.clone());
            }
            *p != kind
        });
        for id in gone {
            self.setters.remove(&id);
        }
        HITS.with(|h| h.borrow_mut().retain(|_, (p, _)| *p != kind));
    }

    pub fn fields_in_order(&self) -> Vec<String> {
        let mut v: Vec<&(PaneKind, String)> = self.field_order.iter().collect();
        v.sort_by_key(|(p, _)| *p);
        v.into_iter().map(|(_, id)| id.clone()).collect()
    }

    fn render_pane(&mut self, kind: PaneKind, _: &mut Window, cx: &mut Context<Self>) -> AnyElement {
        self.begin_pane(kind);
        let el = match kind {
            PaneKind::Root => div().into_any_element(),
            PaneKind::Title => self.title_bar(cx).into_any_element(),
            PaneKind::Outliner => {
                div().size_full().bg(c(theme::PANEL)).child(self.outliner(cx)).into_any_element()
            }
            PaneKind::ViewHeader => self.view_header(cx).into_any_element(),
            PaneKind::Viewport => self.viewport(cx).into_any_element(),
            PaneKind::ViewRail => return self.rail_overlay(cx),
            PaneKind::Right => self.right_column(cx),
            PaneKind::Bottom => self.bottom_panel(cx).into_any_element(),
        };
        let el = div().size_full().flex().flex_col().child(el).into_any_element();
        CURRENT.with(|c| c.set(PaneKind::Root));
        el
    }

    pub fn notify_pane(&self, kind: PaneKind, cx: &mut gpui::App) {
        if let Some(p) = self.panes.as_ref().and_then(|p| p.get(kind)) {
            p.update(cx, |_, cx| cx.notify());
        }
    }

    pub fn animate(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        if self.tick_pending {
            return;
        }
        self.tick_pending = true;
        let this = cx.entity().downgrade();
        window.on_next_frame(move |window, cx| {
            let _ = this.update(cx, |e, cx| e.tick(window, cx));
        });
    }

    fn tick(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.tick_pending = false;
        let cursor = self.cursor;
        self.playback();
        self.notify_pane(PaneKind::Viewport, cx);
        if self.cursor != cursor {
            self.notify_pane(PaneKind::Right, cx);
            self.notify_pane(PaneKind::Bottom, cx);
        }
        if self.follow_playhead() {
            self.rev += 1;
            self.notify_pane(PaneKind::Bottom, cx);
        }
        if self.playing || self.drag.is_some() || self.camera_moving() {
            self.animate(window, cx);
        }
    }

    pub fn redraw_view(&mut self, cx: &mut gpui::App) {
        self.dirty = true;
        self.notify_pane(PaneKind::Viewport, cx);
    }
}
