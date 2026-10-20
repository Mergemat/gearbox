pub mod aim;
pub mod car;
pub mod compare;
pub mod console;
pub mod dopesheet;
pub mod inspector;
pub mod menus;
pub mod outliner;
pub mod projects;
pub mod route;
pub mod search;
pub mod solve;
pub mod viewport;
pub mod widgets;

use std::time::Duration;

use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, Context, DispatchPhase, FontWeight, InteractiveElement, IntoElement, KeyDownEvent,
    KeyUpEvent, ModifiersChangedEvent, MouseButton, MouseMoveEvent, MouseUpEvent, ParentElement, Render,
    StatefulInteractiveElement, Styled, Window, anchored, canvas, deferred, div, px,
};

use crate::app::*;
use crate::theme::{self, c};
use widgets::{muted, tip};

impl Render for Editor {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        self.playback();
        self.script_in_frame = true;
        self.automate(window, cx);
        self.ui_frame_start = Some(crate::stats::thread_cpu_ms());
        widgets::HITS.with(|h| h.borrow_mut().clear());
        self.setters.clear();
        self.field_order.clear();
        if self.playing || self.drag.is_some() || self.camera_moving() {
            window.request_animation_frame();
        }
        if self.status.as_ref().is_some_and(|s| s.2.elapsed() > Duration::from_secs(6)) {
            self.status = None;
        }
        self.follow_search(cx);
        let mut ctx = String::from(if self.projects_open { "Projects" } else { "Editor" });
        if self.editing.is_some() {
            ctx += " TextField";
        }
        if matches!(self.drag, Some(Drag::Fly { .. })) {
            ctx += " Fly";
        }
        if self.record_mode && self.playing {
            ctx += " Drive";
        }
        let root = div()
            .id("root")
            .track_focus(&self.focus)
            .key_context(gpui::KeyContext::parse(&ctx).unwrap_or_default())
            .size_full()
            .flex()
            .flex_col()
            .bg(c(theme::BG))
            .font_family(theme::SANS)
            .text_color(c(theme::TEXT))
            .on_key_down(cx.listener(Self::key_down))
            .on_key_up(cx.listener(Self::key_up))
            .on_modifiers_changed(cx.listener(|this, e: &ModifiersChangedEvent, _, _| {
                this.shift_down = e.modifiers.shift;
                this.alt_down = e.modifiers.alt;
            }))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(|this, _, _, cx| {
                    if this.popup.take().is_some() {
                        cx.notify();
                    }
                    if this.editing.is_some() && this.palette.is_none() {
                        this.commit_field(cx);
                    }
                }),
            );
        let root = self.register_actions(root, cx);
        let root = root.child(self.global_listeners(cx)).child(self.title_bar(cx));
        let root = if self.projects_open {
            root.child(self.projects_screen(window, cx))
        } else {
            root.child(self.main_row(cx))
                .child(self.splitter_h(cx))
                .child(self.bottom_panel(cx))
                .child(self.status_bar(cx))
        };
        let this = cx.entity();
        root.children(self.popup_menu(cx))
            .children(self.palette_overlay(cx))
            .children(self.cheatsheet.then(|| self.cheatsheet_overlay(cx)))
            .child(
                canvas(|_, _, _| (), move |_, _, _, cx| this.update(cx, |e, _| e.ui_frame_done())).absolute(),
            )
    }
}

pub fn pretty_name(s: &str) -> String {
    let t = s
        .split_once('_')
        .filter(|(id, _)| !id.is_empty() && id.chars().all(|c| c.is_ascii_digit()))
        .map_or(s, |(_, rest)| rest);
    let t = t.replace('_', " ");
    let t = t.split_whitespace().collect::<Vec<_>>().join(" ");
    if t.is_empty() { s.to_string() } else { t }
}

impl Editor {
    pub fn camera_moving(&self) -> bool {
        self.camera.goal != self.camera.cur
            || self.camera.mode == crate::camera::Mode::Follow
            || self.fly_keys.iter().any(|k| *k)
    }

    fn register_actions(
        &mut self,
        d: gpui::Stateful<gpui::Div>,
        cx: &mut Context<Self>,
    ) -> gpui::Stateful<gpui::Div> {
        macro_rules! on {
            ($d:expr, $a:ty, $body:expr) => {
                $d.on_action(cx.listener(
                    move |this: &mut Editor, _: &$a, window: &mut Window, cx: &mut Context<Editor>| {
                        let f: &dyn Fn(&mut Editor, &mut Window, &mut Context<Editor>) = &$body;
                        f(this, window, cx);
                        this.dirty = true;
                        cx.notify();
                    },
                ))
            };
        }
        let d = on!(d, TogglePlay, |e, _, _| e.toggle_play());
        let d = on!(d, StepBack, |e, _, _| e.step(-1));
        let d = on!(d, StepForward, |e, _, _| e.step(1));
        let d = on!(d, StepBack10, |e, _, _| e.step(-10));
        let d = on!(d, StepForward10, |e, _, _| e.step(10));
        let d = on!(d, PrevMarker, |e, _, _| {
            let c = e.cursor;
            if let Some(k) = e.markers().into_iter().rev().find(|k| *k < c) {
                e.playing = false;
                e.set_cursor(k);
            }
        });
        let d = on!(d, NextMarker, |e, _, _| {
            let c = e.cursor;
            if let Some(k) = e.markers().into_iter().find(|k| *k > c) {
                e.playing = false;
                e.set_cursor(k);
            }
        });
        let d = on!(d, WindowStart, |e, _, _| e.set_cursor(e.spec.window.start));
        let d = on!(d, WindowEnd, |e, _, _| e.set_cursor(e.spec.window.end));
        let d = on!(d, UndoAction, |e, _, _| e.undo());
        let d = on!(d, RedoAction, |e, _, _| e.redo());
        let d = on!(d, DeleteSelection, |e, _, _| e.delete_selected());
        let d = on!(d, Duplicate, |e, _, _| e.duplicate_selected());
        let d = on!(d, MergeGoals, |e, _, _| e.merge_selected());
        let d = on!(d, CopyInputs, |e, _, _| e.copy_inputs());
        let d = on!(d, PasteInputs, |e, _, _| e.paste_inputs());
        let d = on!(d, LockSelection, |e, _, _| e.lock_selection(true));
        let d = on!(d, UnlockSelection, |e, _, _| e.lock_selection(false));
        let d = on!(d, ClearInputs, |e, _, _| e.clear_selection_inputs());
        let d = on!(d, ShiftLeft, |e, _, _| e.shift_selection(-1));
        let d = on!(d, ShiftRight, |e, _, _| e.shift_selection(1));
        let d = on!(d, ShiftLeft10, |e, _, _| e.shift_selection(-10));
        let d = on!(d, ShiftRight10, |e, _, _| e.shift_selection(10));
        let d = on!(d, SelectAllTicks, |e, _, _| e.tick_sel = Some(0..e.doc.len));
        let d = on!(d, Escape, |e, _, _| e.escape());
        let d = on!(d, ToolSelect, |e, _, _| e.set_tool(Tool::Select));
        let d = on!(d, ToolMove, |e, _, _| e.set_tool(Tool::Move));
        let d = on!(d, ToolRotate, |e, _, _| e.set_tool(Tool::Rotate));
        let d = on!(d, ToolScale, |e, _, _| e.set_tool(Tool::Scale));
        let d = on!(d, ToolCar, |e, _, _| e.set_tool(Tool::PlaceCar));
        let d = on!(d, ToolGate, |e, _, _| e.set_tool(Tool::GateGoal));
        let d = on!(d, ToolPoint, |e, _, _| e.set_tool(Tool::PointGoal));
        let d = on!(d, ToolBox, |e, _, _| e.set_tool(Tool::BoxGoal));
        let d = on!(d, ToolPose, |e, _, _| e.set_tool(Tool::PoseGoal));
        let d = on!(d, ToolLine, |e, _, _| e.set_tool(Tool::Line));
        let d = on!(d, Focus, |e, _, _| e.focus_selection());
        let d = on!(d, FrameAll, |e, _, _| e.frame_all());
        let d = on!(d, ToggleFollow, |e, _, _| {
            let on = e.camera.mode != crate::camera::Mode::Follow;
            e.camera.set_follow(on);
        });
        let d = on!(d, ViewTop, |e, _, _| e.camera.axis_view(e.camera.goal.yaw, crate::camera::PITCH_LIMIT));
        let d = on!(d, ViewFront, |e, _, _| e.camera.axis_view(std::f32::consts::PI, 0.0));
        let d = on!(d, ViewRight, |e, _, _| e.camera.axis_view(-std::f32::consts::FRAC_PI_2, 0.0));
        let d = on!(d, Recall1, |e, _, _| e.recall(0));
        let d = on!(d, Recall2, |e, _, _| e.recall(1));
        let d = on!(d, Recall3, |e, _, _| e.recall(2));
        let d = on!(d, Recall4, |e, _, _| e.recall(3));
        let d = on!(d, Recall5, |e, _, _| e.recall(4));
        let d = on!(d, SaveView1, |e, _, _| e.save_view(0));
        let d = on!(d, SaveView2, |e, _, _| e.save_view(1));
        let d = on!(d, SaveView3, |e, _, _| e.save_view(2));
        let d = on!(d, SaveView4, |e, _, _| e.save_view(3));
        let d = on!(d, SaveView5, |e, _, _| e.save_view(4));
        let d = on!(d, TogglePhysics, |e, _, _| e.show.physics = !e.show.physics);
        let d = on!(d, ToggleStadium, |e, _, _| e.show.stadium = !e.show.stadium);
        let d = on!(d, ToggleContacts, |e, _, _| e.show.contacts = !e.show.contacts);
        let d = on!(d, ToggleRecord, |e, _, _| {
            e.record_mode = !e.record_mode;
            if !e.record_mode {
                e.stop_recording();
            }
            let m = if e.record_mode {
                "record mode: Space records from the cursor; arrows/WASD drive, Shift half steer"
            } else {
                "record mode off"
            };
            e.info("edit", m);
        });
        let d = on!(d, RunSearch, |e, _, _| {
            e.job = "Search".into();
            e.start_search();
        });
        let d = on!(d, StopSearch, |e, _, _| e.search.stop());
        let d = on!(d, OpenMap, |e, _, _| e.show_maps(None));
        let d = on!(d, PmUp, |e, _, _| e.pm_step(-1, true));
        let d = on!(d, PmDown, |e, _, _| e.pm_step(1, true));
        let d = on!(d, PmLeft, |e, _, _| e.pm_step(-1, false));
        let d = on!(d, PmRight, |e, _, _| e.pm_step(1, false));
        let d = on!(d, PmOpen, |e, _, cx| e.pm_open_selected(cx));
        let d = on!(d, PmRename, |e, _, _| e.pm_rename_selected());
        let d = on!(d, PmDuplicate, |e, _, _| {
            if let Some(p) = e.pm.selected.clone().filter(|_| !e.pm.picking_map) {
                e.duplicate_project(&p);
            }
        });
        let d = on!(d, PmTrash, |e, _, _| {
            if let Some(p) = e.pm.selected.clone().filter(|_| !e.pm.picking_map) {
                e.trash_project(&p);
            }
        });
        let d = on!(d, PmUndo, |e, _, _| e.untrash_project());
        let d = on!(d, PmFind, |e, _, _| e.begin_edit("pm.filter"));
        let d = on!(d, PmBack, |e, _, _| e.pm_back());
        let d = on!(d, PmBrowse, |e, _, cx| e.prompt_map_file(cx));
        let d = on!(d, PmShow, |e, _, _| {
            if let Some(p) = e.pm_selected_path() {
                projects::reveal(&p);
            }
        });
        let d = on!(d, OpenProject, |e, _, _| e.show_projects());
        let d = on!(d, SaveProject, |e, _, _| e.save_project(None));
        let d = on!(d, SaveProjectAs, |e, _, _| {
            if let Some(p) = e.project_path.clone() {
                e.duplicate_project(&p);
                e.show_projects();
            }
        });
        let d = on!(d, ImportReplay, |e, _, cx| e.prompt_replay(false, cx));
        let d = on!(d, AddGhost, |e, _, cx| e.prompt_replay(true, cx));
        let d = on!(d, Cheatsheet, |e, _, _| e.cheatsheet = !e.cheatsheet);
        let d = on!(d, PaintTool, |e, _, _| e.paint_tool = !e.paint_tool);
        let d = on!(d, FitTimeline, |e, _, _| e.fit_timeline());
        let d = on!(d, ZoomIn, |e, _, _| e.zoom_timeline(1.25));
        let d = on!(d, ZoomOut, |e, _, _| e.zoom_timeline(0.8));
        let d = on!(d, SetValue, |e, _, _| {
            if e.tick_sel.is_some() {
                e.begin_edit("tl.value");
            }
        });
        let d = on!(d, CommandPalette, |e, _, _| e.open_palette());
        let d = on!(d, SimulateFromHere, |e, _, _| e.simulate_from_start());
        let d = on!(d, UseSelectionAsWindow, |e, _, _| {
            if let Some(r) = e.tick_sel.clone() {
                e.spec.window = r;
            }
        });
        d.on_action(|_: &Quit, _: &mut Window, cx: &mut gpui::App| quit(cx))
    }

    pub fn escape(&mut self) {
        if self.popup.take().is_some() || std::mem::take(&mut self.cheatsheet) {
            return;
        }
        if self.drag.take().is_some() {
            return;
        }
        if self.tool != Tool::Select {
            self.set_tool(Tool::Select);
            return;
        }
        if self.tick_sel.take().is_some() {
            return;
        }
        self.sel = Sel::None;
        self.preview = None;
    }

    pub fn set_tool(&mut self, t: Tool) {
        if t != Tool::Line {
            self.drawing = None;
        }
        self.tool = t;
        self.car_preview = None;
        self.info("tool", format!("{}: {}", t.label(), t.hint()));
    }

    pub fn tool_groups(&self) -> Vec<Vec<Tool>> {
        vec![
            vec![Tool::Select, Tool::Move, Tool::Rotate, Tool::Scale],
            vec![Tool::PlaceCar],
            vec![Tool::GateGoal, Tool::PointGoal, Tool::BoxGoal, Tool::PoseGoal],
            vec![Tool::Line],
        ]
    }

    pub fn simulate_from_start(&mut self) {
        match self.start_tick() {
            Some(t) => {
                self.set_cursor(t);
                self.playing = false;
                self.toggle_play();
                self.info("sim", format!("simulating from the start state (tick {t})"));
            }
            None => self.log(LogLevel::Warn, "sim", "no start state: place the car first (C)"),
        }
    }

    fn key_down(&mut self, e: &KeyDownEvent, _: &mut Window, cx: &mut Context<Self>) {
        let key = e.keystroke.key.as_str();
        if self.palette.is_some() && self.palette_key(key, cx) {
            cx.stop_propagation();
            return;
        }
        if self.editing.is_some() {
            let ch = e.keystroke.key_char.clone();
            if self.field_key(key, ch.as_deref(), cx) {
                cx.stop_propagation();
            }
            return;
        }
        if matches!(self.drag, Some(Drag::Fly { .. })) {
            let k = match key {
                "w" => Some(0),
                "s" => Some(1),
                "a" => Some(2),
                "d" => Some(3),
                "e" => Some(4),
                "q" => Some(5),
                _ => None,
            };
            if let Some(k) = k {
                self.fly_keys[k] = true;
                cx.stop_propagation();
                cx.notify();
            }
            return;
        }
        if self.record_mode && self.playing {
            if key == "space" {
                self.toggle_play();
            } else if let Some(k) = drive_key(key) {
                match k {
                    0 => self.drive.gas = true,
                    1 => self.drive.brake = true,
                    2 => self.drive.left = true,
                    _ => self.drive.right = true,
                }
            }
            self.drive.half = e.keystroke.modifiers.shift;
            cx.stop_propagation();
            cx.notify();
        }
    }

    fn key_up(&mut self, e: &KeyUpEvent, _: &mut Window, cx: &mut Context<Self>) {
        let key = e.keystroke.key.as_str();
        let k = match key {
            "w" => Some(0),
            "s" => Some(1),
            "a" => Some(2),
            "d" => Some(3),
            "e" => Some(4),
            "q" => Some(5),
            _ => None,
        };
        if let Some(k) = k {
            self.fly_keys[k] = false;
        }
        if let Some(k) = drive_key(key) {
            match k {
                0 => self.drive.gas = false,
                1 => self.drive.brake = false,
                2 => self.drive.left = false,
                _ => self.drive.right = false,
            }
        }
        cx.notify();
    }

    fn global_listeners(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let this = cx.entity();
        canvas(
            |_, _, _| (),
            move |_, _, window, _| {
                let t = this.clone();
                window.on_mouse_event(move |e: &MouseMoveEvent, phase, window, cx| {
                    if phase == DispatchPhase::Bubble {
                        t.update(cx, |app, cx| app.global_move(e, window, cx));
                    }
                });
                let t = this.clone();
                window.on_mouse_event(move |e: &MouseUpEvent, phase, window, cx| {
                    if phase == DispatchPhase::Bubble && t.read(cx).drag.is_some() {
                        t.update(cx, |app, cx| app.global_up(e, window, cx));
                    }
                });
            },
        )
        .absolute()
        .size(px(0.0))
    }

    fn global_move(&mut self, e: &MouseMoveEvent, window: &mut Window, cx: &mut Context<Self>) {
        match self.drag.clone() {
            Some(Drag::Field { id, x0, v0, step, moved }) => {
                let dx = f32::from(e.position.x) - x0;
                if moved || dx.abs() > 2.0 {
                    let v = v0 + (dx as f64) * step;
                    let set = match self.setters.get(&id) {
                        Some(Setter::Num(f)) => Some(f.clone()),
                        _ => None,
                    };
                    if let Some(f) = set {
                        f(self, v, cx);
                    }
                    self.drag = Some(Drag::Field { id, x0, v0, step, moved: true });
                    cx.notify();
                }
            }
            Some(Drag::Split { which, start, size0 }) => {
                let p = match which {
                    Split::Bottom => f32::from(e.position.y),
                    _ => f32::from(e.position.x),
                };
                let d = p - start;
                let wb = window.bounds().size;
                match which {
                    Split::Left => self.layout.left = (size0 + d).clamp(170.0, 520.0),
                    Split::Right => self.layout.right = (size0 - d).clamp(260.0, 640.0),
                    Split::Bottom => {
                        self.layout.bottom = (size0 - d).clamp(120.0, f32::from(wb.height) * 0.7)
                    }
                }
                self.dirty = true;
                cx.notify();
            }
            Some(Drag::Timeline(_)) => self.timeline_move(e, cx),
            Some(_) => self.viewport_move(e, cx),
            None => {}
        }
    }

    fn global_up(&mut self, e: &MouseUpEvent, window: &mut Window, cx: &mut Context<Self>) {
        match self.drag.clone() {
            Some(Drag::Field { id, moved, .. }) => {
                self.drag = None;
                if !moved {
                    self.begin_edit(&id);
                }
            }
            Some(Drag::Split { .. }) => {
                self.drag = None;
                self.save_layout();
            }
            Some(Drag::Timeline(_)) => self.timeline_up(e, cx),
            Some(_) => self.viewport_up(e, window, cx),
            None => {}
        }
        cx.notify();
    }

    fn splitter(&mut self, which: Split, vertical: bool, cx: &mut Context<Self>) -> impl IntoElement {
        let id = format!("split-{which:?}");
        let active = matches!(self.drag, Some(Drag::Split { which: w, .. }) if w == which);
        div()
            .id(gpui::ElementId::Name(id.into()))
            .flex_none()
            .when(vertical, |d| d.w(px(5.0)).h_full().cursor_col_resize())
            .when(!vertical, |d| d.h(px(5.0)).w_full().cursor_row_resize())
            .flex()
            .items_center()
            .justify_center()
            .bg(c(theme::BG))
            .child(
                div()
                    .when(vertical, |d| d.w(px(1.0)).h_full())
                    .when(!vertical, |d| d.h(px(1.0)).w_full())
                    .bg(c(if active { theme::PIN } else { theme::HAIRLINE })),
            )
            .hover(|s| s.bg(c(theme::RAISED)))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |this, e: &gpui::MouseDownEvent, _, cx| {
                    let (start, size0) = match which {
                        Split::Left => (f32::from(e.position.x), this.layout.left),
                        Split::Right => (f32::from(e.position.x), this.layout.right),
                        Split::Bottom => (f32::from(e.position.y), this.layout.bottom),
                    };
                    if e.click_count == 2 {
                        let d = crate::project::Layout::default();
                        match which {
                            Split::Left => this.layout.left = d.left,
                            Split::Right => this.layout.right = d.right,
                            Split::Bottom => this.layout.bottom = d.bottom,
                        }
                        this.save_layout();
                    } else {
                        this.drag = Some(Drag::Split { which, start, size0 });
                    }
                    cx.stop_propagation();
                    cx.notify();
                }),
            )
    }

    fn splitter_h(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        self.splitter(Split::Bottom, false, cx)
    }

    fn main_row(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let left = self.layout.left;
        let right = self.layout.right;
        div()
            .flex_1()
            .min_h_0()
            .flex()
            .child(div().w(px(left)).flex_none().h_full().bg(c(theme::PANEL)).child(self.outliner(cx)))
            .child(self.splitter(Split::Left, true, cx))
            .child(div().flex_1().min_w_0().h_full().child(self.viewport(cx)))
            .child(self.splitter(Split::Right, true, cx))
            .child(
                div()
                    .w(px(right))
                    .flex_none()
                    .h_full()
                    .flex()
                    .flex_col()
                    .bg(c(theme::PANEL))
                    .child(self.right_tabs(cx))
                    .child(div().flex_1().min_h_0().child(match self.layout.right_tab {
                        1 => self.search_panel(cx).into_any_element(),
                        _ => self.inspector(cx).into_any_element(),
                    })),
            )
    }

    pub fn right_tabs(&mut self, cx: &mut Context<Self>) -> AnyElement {
        let tab = self.layout.right_tab;
        let running = self.search.running();
        div()
            .h(px(theme::H_BAR))
            .flex_none()
            .flex()
            .items_end()
            .px(px(theme::SP2))
            .gap(px(2.0))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .children([("Properties", 0u8), ("Search", 1)].map(|(name, i)| {
                let on = tab == i;
                div()
                    .id(("rtab", i as usize))
                    .relative()
                    .child(widgets::track(format!("rtab-{i}")))
                    .h(px(theme::H_BAR))
                    .px(px(theme::PAD))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP2))
                    .text_size(px(theme::FS_BODY))
                    .cursor_pointer()
                    .when(on, |d| d.text_color(c(theme::TEXT)).border_b_2().border_color(c(theme::PIN)))
                    .when(!on, |d| d.text_color(c(theme::MUTED)).hover(|s| s.text_color(c(theme::TEXT))))
                    .child(name)
                    .when(i == 1 && running, |d| {
                        d.child(div().size(px(6.0)).rounded_full().bg(c(theme::PIN)))
                    })
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.layout.right_tab = i;
                        this.save_layout();
                        cx.notify();
                    }))
            }))
            .into_any_element()
    }

    fn bottom_panel(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let tab = self.layout.bottom_tab;
        let errors = self.log.iter().filter(|l| l.level == LogLevel::Error).count();
        let tabs = [("Timeline", 0u8), ("Compare", 1), ("Console", 2)];
        let header = div()
            .h(px(theme::H_BAR))
            .flex_none()
            .flex()
            .items_end()
            .px(px(theme::SP2))
            .gap(px(2.0))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .children(tabs.map(|(name, i)| {
                let on = tab == i;
                div()
                    .id(("btab", i as usize))
                    .relative()
                    .child(widgets::track(format!("btab-{i}")))
                    .h(px(theme::H_BAR))
                    .px(px(theme::PAD))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP2))
                    .text_size(px(theme::FS_BODY))
                    .cursor_pointer()
                    .when(on, |d| d.border_b_2().border_color(c(theme::PIN)).text_color(c(theme::TEXT)))
                    .when(!on, |d| d.text_color(c(theme::MUTED)).hover(|s| s.text_color(c(theme::TEXT))))
                    .child(name)
                    .when(i == 2 && errors > 0, |d| {
                        d.child(
                            div()
                                .px(px(theme::SP1))
                                .rounded(px(6.0))
                                .bg(c(theme::ERROR))
                                .text_color(c(theme::BG))
                                .text_size(px(theme::FS_SMALL))
                                .child(errors.to_string()),
                        )
                    })
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.layout.bottom_tab = i;
                        this.save_layout();
                        cx.notify();
                    }))
            }));
        let body: AnyElement = match tab {
            1 => self.compare_panel(cx).into_any_element(),
            2 => self.console(cx).into_any_element(),
            _ => self.dopesheet(cx).into_any_element(),
        };
        div()
            .h(px(self.layout.bottom))
            .flex_none()
            .flex()
            .flex_col()
            .bg(c(theme::PANEL))
            .child(header)
            .child(div().flex_1().min_h_0().child(body))
    }

    fn title_bar(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let name = match (&self.sim.info, &self.map) {
            (Some(i), _) => i.stem.clone(),
            (None, Some(m)) if self.open_error.is_some() => crate::catalog::map_stem(m).unwrap_or_default(),
            _ => "Opening\u{2026}".to_string(),
        };
        let drag_area = || {
            div()
                .flex_1()
                .h_full()
                .on_mouse_down(MouseButton::Left, |_, window, _| window.start_window_move())
        };
        let dirty = self.dirty_doc();
        let in_editor = !self.projects_open && self.map.is_some();
        let icon_btn = |id: String, icon: &'static str, on: bool| {
            div()
                .id(gpui::ElementId::Name(id.clone().into()))
                .relative()
                .child(widgets::track(id))
                .size(px(28.0))
                .flex()
                .items_center()
                .justify_center()
                .rounded(px(theme::R_CTL))
                .cursor_pointer()
                .when(on, |d| d.bg(c(theme::SELECT)))
                .when(!on, |d| d.hover(|s| s.bg(c(theme::RAISED))))
                .child(gpui::svg().path(icon).size(px(16.0)).text_color(c(if on {
                    theme::TEXT
                } else {
                    theme::MUTED
                })))
        };
        let palette = div()
            .id("palette-open")
            .relative()
            .child(widgets::track("palette-open"))
            .w(px(380.0))
            .h(px(26.0))
            .px(px(theme::SP3))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .rounded(px(theme::R_CTL + 1.0))
            .bg(c(theme::WELL))
            .border_1()
            .border_color(c(theme::HAIRLINE))
            .cursor_pointer()
            .hover(|s| s.border_color(c(theme::FAINT)))
            .child(
                div()
                    .flex_1()
                    .text_size(px(theme::FS_BODY))
                    .text_color(c(theme::FAINT))
                    .child("Search commands, objects, checkpoints\u{2026}"),
            )
            .child(widgets::kbd("\u{2318}K"))
            .on_click(cx.listener(|this, _, _, cx| {
                this.open_palette();
                cx.notify();
            }));
        let job = self.job_pill(cx);
        let title = if self.projects_open || self.map.is_none() {
            "Projects".to_string()
        } else {
            pretty_name(&self.project_name)
        };
        div()
            .h(px(40.0))
            .flex_none()
            .flex()
            .items_center()
            .pl(px(80.0))
            .pr(px(theme::SP2))
            .gap(px(theme::SP2))
            .bg(c(theme::PANEL))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .child(
                div()
                    .id("project-menu")
                    .relative()
                    .child(widgets::track("project-menu"))
                    .h(px(28.0))
                    .px(px(theme::SP2))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP2))
                    .rounded(px(theme::R_CTL))
                    .cursor_pointer()
                    .hover(|s| s.bg(c(theme::RAISED)))
                    .child(div().text_size(px(theme::FS_BODY)).font_weight(FontWeight::SEMIBOLD).child(title))
                    .when(in_editor && (dirty || self.project_path.is_none()), |d| {
                        d.child(div().size(px(6.0)).rounded_full().bg(c(theme::MUTED)))
                    })
                    .child(div().text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child("\u{25be}"))
                    .tooltip(tip(
                        if self.project_path.is_none() {
                            "Not saved yet"
                        } else if dirty {
                            "Saving\u{2026}"
                        } else {
                            "Saved"
                        },
                        Some(&format!("Map: {name}. Projects save themselves; this menu has the rest.")),
                    ))
                    .on_click(cx.listener(|this, e: &gpui::ClickEvent, _, cx| {
                        type Item =
                            (String, Option<String>, std::rc::Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>);
                        let mut items: Vec<Item> = vec![(
                            "All projects\t\u{2318}O".into(),
                            None,
                            std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show_projects()),
                        )];
                        items.push((
                            "New project\u{2026}\t\u{2318}\u{21e7}O".into(),
                            None,
                            std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show_maps(None)),
                        ));
                        if let Some(p) = this.project_path.clone().filter(|p| p.exists()) {
                            let (p1, p2, p3) = (p.clone(), p.clone(), p);
                            items.push((
                                "-".into(),
                                None,
                                std::rc::Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}),
                            ));
                            items.push((
                                "Rename\u{2026}".into(),
                                None,
                                std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| {
                                    e.show_projects();
                                    e.pm_begin_rename(&p1);
                                }),
                            ));
                            items.push((
                                "Duplicate".into(),
                                None,
                                std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| {
                                    e.duplicate_project(&p2);
                                    e.show_projects();
                                }),
                            ));
                            items.push((
                                "Show in Finder".into(),
                                None,
                                std::rc::Rc::new(move |_: &mut Editor, _: &mut Context<Editor>| {
                                    projects::reveal(&p3)
                                }),
                            ));
                        }
                        let recent: Vec<crate::project::Entry> = crate::project::list(&this.home)
                            .into_iter()
                            .filter(|x| this.project_path.as_ref() != Some(&x.path))
                            .take(5)
                            .collect();
                        if !recent.is_empty() {
                            items.push((
                                "-".into(),
                                None,
                                std::rc::Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}),
                            ));
                            items.push((
                                "#Open recent".into(),
                                None,
                                std::rc::Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {}),
                            ));
                            for x in recent {
                                let label = this.entry_name(&x);
                                items.push((
                                    label,
                                    None,
                                    std::rc::Rc::new(move |e: &mut Editor, cx: &mut Context<Editor>| {
                                        e.open_listed(&x.path, cx)
                                    }),
                                ));
                            }
                        }
                        this.popup = Some(Popup { at: e.position(), items });
                        cx.stop_propagation();
                        cx.notify();
                    })),
            )
            .child(drag_area())
            .when(in_editor, |d| d.child(palette))
            .child(drag_area())
            .child(
                icon_btn("help".into(), "icons/help.svg", self.cheatsheet)
                    .tooltip(tip("Shortcuts  ?", None))
                    .on_click(cx.listener(|this, _, _, cx| {
                        this.cheatsheet = !this.cheatsheet;
                        cx.notify();
                    })),
            )
            .when(in_editor, |d| d.children(job))
    }

    fn status_bar(&mut self, _cx: &mut Context<Self>) -> impl IntoElement {
        let sim = if self.sim.done() {
            String::new()
        } else {
            format!("simulating {}/{}", self.sim.frames.len().saturating_sub(1), self.doc.len)
        };
        let search = {
            let l = self.search.live.lock().unwrap();
            if l.running { format!("searching \u{00b7} {} tries", l.attempts) } else { String::new() }
        };
        let fps = {
            let i = self.stats.interval.pct(0.5);
            if i > 0.0 { format!("{:.0} fps", 1000.0 / i) } else { String::new() }
        };
        let status = self.status.as_ref().map(|(t, l, _)| (t.clone(), *l));
        let cell = |s: String| {
            div().font_family(theme::MONO).text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(s)
        };
        div()
            .h(px(22.0))
            .flex_none()
            .flex()
            .items_center()
            .gap(px(theme::SP3))
            .px(px(theme::PAD))
            .bg(c(theme::PANEL))
            .border_t_1()
            .border_color(c(theme::HAIRLINE))
            .when(!sim.is_empty(), |d| d.child(cell(sim)))
            .when(!search.is_empty(), |d| d.child(cell(search).text_color(c(theme::PIN))))
            .when(self.record_mode, |d| {
                d.child(
                    cell(if self.playing { "\u{25cf} REC".into() } else { "record mode".into() })
                        .text_color(c(theme::ERROR)),
                )
            })
            .child(div().flex_1().min_w_0().truncate().children(status.map(|(t, l)| {
                div()
                    .text_size(px(theme::FS_BODY))
                    .truncate()
                    .text_color(c(match l {
                        LogLevel::Error => theme::ERROR,
                        LogLevel::Warn => theme::WARN,
                        LogLevel::Info => theme::MUTED,
                    }))
                    .child(t)
            })))
            .child(cell(fps))
    }

    fn popup_menu(&mut self, cx: &mut Context<Self>) -> Option<AnyElement> {
        let p = self.popup.as_ref()?;
        let rows: Vec<AnyElement> = p
            .items
            .iter()
            .enumerate()
            .map(|(i, (label, mark, f))| {
                if label == "-" {
                    return div()
                        .h(px(1.0))
                        .mx(px(theme::SP1))
                        .my(px(theme::SP1))
                        .bg(c(theme::HAIRLINE))
                        .into_any_element();
                }
                if let Some(t) = label.strip_prefix('#') {
                    return div()
                        .h(px(24.0))
                        .px(px(theme::SP3))
                        .flex()
                        .items_center()
                        .text_size(px(theme::FS_SMALL))
                        .text_color(c(theme::FAINT))
                        .child(t.to_string())
                        .into_any_element();
                }
                let (text, key) = label
                    .split_once('\t')
                    .map_or((label.clone(), String::new()), |(a, b)| (a.to_string(), b.to_string()));
                let f = f.clone();
                div()
                    .id(("menu-item", i))
                    .relative()
                    .child(widgets::track(format!("menu-{i}")))
                    .h(px(26.0))
                    .px(px(theme::SP3))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP3))
                    .rounded(px(theme::R_CTL))
                    .text_size(px(theme::FS_BODY))
                    .cursor_pointer()
                    .hover(|s| s.bg(c(theme::SELECT)))
                    .when(mark.is_some(), |d| {
                        d.child(
                            div()
                                .w(px(10.0))
                                .text_color(c(theme::TEXT))
                                .child(mark.clone().unwrap_or_default()),
                        )
                    })
                    .child(widgets::labelled(&text, theme::MUTED).flex_1())
                    .when(!key.is_empty(), |d| {
                        d.child(div().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child(key))
                    })
                    .on_mouse_down(
                        MouseButton::Left,
                        cx.listener(move |this, _, _, cx| {
                            this.popup = None;
                            f(this, cx);
                            this.dirty = true;
                            cx.stop_propagation();
                            cx.notify();
                        }),
                    )
                    .into_any_element()
            })
            .collect();
        Some(
            deferred(
                anchored().position(p.at).snap_to_window_with_margin(px(8.0)).child(
                    div()
                        .id("menu")
                        .occlude()
                        .min_w(px(220.0))
                        .max_h(px(520.0))
                        .overflow_y_scroll()
                        .p(px(theme::SP1))
                        .rounded(px(theme::R_CTL + 2.0))
                        .bg(c(theme::RAISED))
                        .border_1()
                        .border_color(c(theme::HAIRLINE))
                        .shadow_lg()
                        .children(rows),
                ),
            )
            .with_priority(2)
            .into_any_element(),
        )
    }

    fn cheatsheet_overlay(&mut self, cx: &mut Context<Self>) -> AnyElement {
        let mut groups: Vec<(&str, Vec<(&str, &str)>)> = Vec::new();
        for (k, what, g) in SHORTCUTS {
            match groups.iter_mut().find(|x| x.0 == *g) {
                Some(x) => x.1.push((k, what)),
                None => groups.push((g, vec![(k, what)])),
            }
        }
        let cols: Vec<AnyElement> = groups
            .into_iter()
            .map(|(g, items)| {
                div()
                    .w(px(300.0))
                    .flex()
                    .flex_col()
                    .gap(px(theme::SP1))
                    .child(
                        div()
                            .text_size(px(theme::FS_EMPH))
                            .font_weight(FontWeight::SEMIBOLD)
                            .text_color(c(theme::TEXT))
                            .child(g.to_string()),
                    )
                    .children(items.into_iter().map(|(k, w)| {
                        div()
                            .flex()
                            .gap(px(theme::SP2))
                            .child(
                                div()
                                    .w(px(130.0))
                                    .flex_none()
                                    .font_family(theme::MONO)
                                    .text_size(px(theme::FS_SMALL))
                                    .text_color(c(theme::PIN))
                                    .child(k.to_string()),
                            )
                            .child(div().text_size(px(theme::FS_BODY)).child(w.to_string()))
                    }))
                    .into_any_element()
            })
            .collect();
        div()
            .id("cheatsheet")
            .absolute()
            .size_full()
            .flex()
            .items_center()
            .justify_center()
            .bg(theme::alpha(theme::BG, 0.72))
            .occlude()
            .on_click(cx.listener(|this, _, _, cx| {
                this.cheatsheet = false;
                cx.notify();
            }))
            .child(
                div()
                    .p(px(20.0))
                    .rounded(px(theme::RADIUS))
                    .bg(c(theme::PANEL))
                    .border_1()
                    .border_color(c(theme::HAIRLINE))
                    .flex()
                    .flex_col()
                    .gap(px(theme::SP3))
                    .child(
                        div()
                            .flex()
                            .items_center()
                            .child(
                                div()
                                    .flex_1()
                                    .text_size(px(theme::FS_TITLE))
                                    .font_weight(FontWeight::SEMIBOLD)
                                    .child("Shortcuts"),
                            )
                            .child(muted("Esc or click to close")),
                    )
                    .child(div().flex().flex_wrap().gap(px(18.0)).max_w(px(980.0)).children(cols)),
            )
            .into_any_element()
    }

    pub fn prompt_replay(&mut self, ghost: bool, cx: &mut Context<Self>) {
        let mine = self.map.as_ref().map(|m| crate::replay::replays_for_map(m)).unwrap_or_default();
        let mut items: Vec<(String, Option<String>, std::rc::Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>)> =
            mine.into_iter()
                .map(|p| {
                    let label = p.file_name().unwrap_or_default().to_string_lossy().to_string();
                    let f: std::rc::Rc<dyn Fn(&mut Editor, &mut Context<Editor>)> =
                        std::rc::Rc::new(move |e: &mut Editor, cx: &mut Context<Editor>| {
                            if ghost {
                                e.add_ghost(&p);
                            } else {
                                e.import_replay(&p, cx);
                            }
                        });
                    (label, None, f)
                })
                .collect();
        let browse: std::rc::Rc<dyn Fn(&mut Editor, &mut Context<Editor>)> =
            std::rc::Rc::new(move |_: &mut Editor, cx: &mut Context<Editor>| {
                let rx = cx.prompt_for_paths(gpui::PathPromptOptions {
                    files: true,
                    directories: false,
                    multiple: false,
                    prompt: Some("Choose a .Replay.Gbx or .Ghost.Gbx".into()),
                });
                cx.spawn(async move |this, cx| {
                    if let Ok(Ok(Some(paths))) = rx.await
                        && let Some(p) = paths.into_iter().next()
                    {
                        let _ = this.update(cx, |app, cx| {
                            if ghost { app.add_ghost(&p) } else { app.import_replay(&p, cx) }
                        });
                    }
                })
                .detach();
            });
        items.push(("Browse\u{2026}".into(), None, browse));
        let at = gpui::point(
            px(self.mouse.x + f32::from(self.viewport.origin.x)),
            px(self.mouse.y + f32::from(self.viewport.origin.y)),
        );
        let at = if self.mouse == glam::Vec2::ZERO { gpui::point(px(400.0), px(120.0)) } else { at };
        self.popup = Some(Popup { at, items });
        cx.notify();
    }

    pub fn ui_frame_done(&mut self) {
        if let Some(t) = self.ui_frame_start.take() {
            self.stats.ui_frame.push(crate::stats::thread_cpu_ms() - t);
        }
    }
}
