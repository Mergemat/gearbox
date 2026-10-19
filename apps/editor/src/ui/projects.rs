use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, Context, ExternalPaths, FontWeight, InteractiveElement, IntoElement, MouseButton,
    ParentElement, StatefulInteractiveElement, Styled, Window, div, px,
};

use crate::app::{Editor, MapSort, MapTab, Popup, ProjectSort};
use crate::fmt::fmt_ms;
use crate::mapcards::{MapCard, Source};
use crate::project::Entry;
use crate::theme::{self, c};
use crate::ui::widgets::*;

fn ago(t: SystemTime) -> String {
    let s = SystemTime::now().duration_since(t).map_or(0, |d| d.as_secs());
    match s {
        0..60 => "just now".into(),
        60..3600 => format!("{} min ago", s / 60),
        3600..86400 => format!("{} h ago", s / 3600),
        86400..172800 => "yesterday".into(),
        _ => format!("{} days ago", s / 86400),
    }
}

pub fn reveal(p: &Path) {
    let _ = std::process::Command::new("open").arg("-R").arg(p).spawn();
}

fn delta(ms: i32) -> (String, u32) {
    match ms {
        0 => ("\u{00b1}0.000".into(), theme::MUTED),
        d if d < 0 => (format!("\u{2212}{}", fmt_ms(-d)), theme::OK),
        d => (format!("+{}", fmt_ms(d)), theme::WARN),
    }
}

const SIDE_W: f32 = 210.0;
const PROPS_W: f32 = 320.0;
const CARD_MIN: f32 = 168.0;
const ROW_H: f32 = 56.0;
const UNDO_FOR: Duration = Duration::from_secs(10);

impl Editor {
    fn pm_filter(&self) -> String {
        match &self.editing {
            Some(e) if e.id == "pm.filter" && !e.fresh => e.text.to_lowercase(),
            _ => self.pm.filter.to_lowercase(),
        }
    }

    fn entry_card(&self, e: &Entry) -> Option<&MapCard> {
        self.pm.cards.get(&e.map)
    }

    pub fn entry_name(&self, e: &Entry) -> String {
        match self.entry_card(e) {
            Some(card) if !e.named => card.name.clone(),
            _ => e.name.clone(),
        }
    }

    pub fn pm_projects(&self) -> Vec<Entry> {
        let f = self.pm_filter();
        let mut v: Vec<Entry> = self
            .pm
            .entries
            .iter()
            .filter(|e| {
                let card = self.entry_card(e);
                f.is_empty()
                    || self.entry_name(e).to_lowercase().contains(&f)
                    || e.map.to_string_lossy().to_lowercase().contains(&f)
                    || card.is_some_and(|c| {
                        c.name.to_lowercase().contains(&f)
                            || c.author.as_deref().is_some_and(|a| a.to_lowercase().contains(&f))
                    })
            })
            .cloned()
            .collect();
        match self.pm.sort {
            ProjectSort::Edited => {}
            ProjectSort::Name => v.sort_by_key(|e| self.entry_name(e).to_lowercase()),
            ProjectSort::Map => v.sort_by_key(|e| {
                self.entry_card(e)
                    .map_or_else(|| e.map.to_string_lossy().to_lowercase(), |c| c.name.to_lowercase())
            }),
        }
        v
    }

    fn in_tab(card: &MapCard, t: MapTab) -> bool {
        match t {
            MapTab::Recent => card.source != Source::Test,
            MapTab::Mine => card.source == Source::Mine,
            MapTab::Downloads => card.source == Source::Downloads,
            MapTab::Test => card.source == Source::Test,
        }
    }

    fn map_matches(card: &MapCard, f: &str) -> bool {
        f.is_empty()
            || card.name.to_lowercase().contains(f)
            || card.author.as_deref().is_some_and(|a| a.to_lowercase().contains(f))
            || card.path.file_name().is_some_and(|n| n.to_string_lossy().to_lowercase().contains(f))
    }

    pub fn pm_maps(&self) -> Vec<MapCard> {
        let f = self.pm_filter();
        let mut v: Vec<MapCard> = self
            .pm
            .cards
            .values()
            .filter(|c| Self::in_tab(c, self.pm.tab) && Self::map_matches(c, &f))
            .cloned()
            .collect();
        match self.pm.map_sort {
            MapSort::Newest if self.pm.tab != MapTab::Test => v.sort_by(|a, b| b.modified.cmp(&a.modified)),
            MapSort::Newest | MapSort::Name => v.sort_by_key(|c| c.name.to_lowercase()),
            MapSort::AuthorTime => v.sort_by_key(|c| c.author_time_ms.unwrap_or(u32::MAX)),
        }
        v
    }

    fn pm_current(&self, list: &[Entry]) -> Option<Entry> {
        list.iter().find(|e| self.pm.selected.as_ref() == Some(&e.path)).or(list.first()).cloned()
    }

    fn pm_current_map(&self, list: &[MapCard]) -> Option<MapCard> {
        list.iter().find(|c| self.pm.selected_map.as_ref() == Some(&c.path)).or(list.first()).cloned()
    }

    pub fn pm_selected_path(&self) -> Option<PathBuf> {
        if self.pm.picking_map {
            self.pm_current_map(&self.pm_maps()).map(|c| c.path)
        } else {
            self.pm_current(&self.pm_projects()).map(|e| e.path)
        }
    }

    pub fn pm_step(&mut self, d: i32, rows: bool) {
        if self.pm.picking_map {
            let list = self.pm_maps();
            let at = self
                .pm_current_map(&list)
                .and_then(|c| list.iter().position(|x| x.path == c.path))
                .unwrap_or(0) as i32;
            let step = if rows { d * self.pm.cols.max(1) as i32 } else { d };
            if let Some(c) = list.get((at + step).clamp(0, list.len().saturating_sub(1) as i32) as usize) {
                self.pm.selected_map = Some(c.path.clone());
                self.pm.new_name = None;
            }
        } else {
            let list = self.pm_projects();
            let at =
                self.pm_current(&list).and_then(|e| list.iter().position(|x| x.path == e.path)).unwrap_or(0)
                    as i32;
            if let Some(e) = list.get((at + d).clamp(0, list.len().saturating_sub(1) as i32) as usize) {
                self.pm.selected = Some(e.path.clone());
            }
        }
    }

    pub fn pm_open_selected(&mut self, cx: &mut Context<Self>) {
        if self.pm.picking_map {
            if let Some(card) = self.pm_current_map(&self.pm_maps()) {
                let name = self.pm.new_name.take();
                self.new_project(&card.path, name, cx);
            }
        } else if let Some(e) = self.pm_current(&self.pm_projects()) {
            self.open_listed(&e.path, cx);
        }
    }

    pub fn pm_rename_selected(&mut self) {
        if self.pm.picking_map {
            self.begin_edit("pm.newname");
        } else if let Some(e) = self.pm_current(&self.pm_projects()) {
            self.pm_begin_rename(&e.path);
        }
    }

    pub fn pm_begin_rename(&mut self, path: &Path) {
        let Some(e) = self.pm.entries.iter().find(|e| e.path == path).cloned() else { return };
        self.pm.selected = Some(e.path.clone());
        self.pm.renaming = Some(e.path.clone());
        self.editing = Some(crate::ui::widgets::FieldEdit {
            id: "pm.rename".into(),
            text: self.entry_name(&e),
            fresh: true,
        });
    }

    pub fn pm_back(&mut self) {
        if self.popup.take().is_some() {
            return;
        }
        if self.pm.picking_map && !self.pm.entries.is_empty() {
            self.pm.picking_map = false;
        } else if self.map.is_some() {
            self.projects_open = false;
        }
    }

    pub fn prompt_map_file(&mut self, cx: &mut Context<Self>) {
        let rx = cx.prompt_for_paths(gpui::PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("Pick a .Map.Gbx".into()),
        });
        cx.spawn(async move |this, cx| {
            if let Ok(Ok(Some(paths))) = rx.await
                && let Some(p) = paths.into_iter().next()
            {
                let _ = this.update(cx, |ed, cx| ed.new_project(&p, None, cx));
            }
        })
        .detach();
    }

    fn project_menu(&mut self, path: PathBuf, map: PathBuf, at: gpui::Point<gpui::Pixels>) {
        type Item = (String, Option<String>, std::rc::Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>);
        let (p1, p2, p3, p4, p5) = (path.clone(), path.clone(), path.clone(), path.clone(), path);
        let items: Vec<Item> = vec![
            (
                "Open\t\u{21b5}".into(),
                None,
                std::rc::Rc::new(move |e: &mut Editor, cx: &mut Context<Editor>| e.open_listed(&p1, cx)),
            ),
            (
                "Rename\tF2".into(),
                None,
                std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| e.pm_begin_rename(&p2)),
            ),
            (
                "Duplicate\t\u{2318}D".into(),
                None,
                std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| e.duplicate_project(&p3)),
            ),
            (
                "Show in Finder\t\u{2318}\u{21e7}R".into(),
                None,
                std::rc::Rc::new(move |_: &mut Editor, _: &mut Context<Editor>| reveal(&p4)),
            ),
            (
                "New project on this map".into(),
                None,
                std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| {
                    e.show_maps(Some(map.clone()))
                }),
            ),
            ("-".into(), None, std::rc::Rc::new(|_: &mut Editor, _: &mut Context<Editor>| {})),
            (
                "Move to Trash\t\u{2318}\u{232b}".into(),
                None,
                std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| e.trash_project(&p5)),
            ),
        ];
        self.popup = Some(Popup { at, items });
    }

    pub fn projects_screen(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let picking = self.pm.picking_map;
        let grid_w = f32::from(window.viewport_size().width) - SIDE_W - PROPS_W - 2.0 * theme::PAD;
        self.pm.cols = (((grid_w + theme::SP3) / (CARD_MIN + theme::SP3)).floor() as usize).max(1);
        self.pm.card_w = (grid_w - theme::SP3 * (self.pm.cols - 1) as f32) / self.pm.cols as f32;
        let (middle, props): (AnyElement, AnyElement) = if picking {
            let maps = self.pm_maps();
            let cur = self.pm_current_map(&maps);
            (self.map_grid(&maps, cur.as_ref(), cx), self.map_props(cur.as_ref(), cx))
        } else {
            let list = self.pm_projects();
            let cur = self.pm_current(&list);
            (self.project_table(&list, cur.as_ref(), cx), self.project_props(cur.as_ref(), cx))
        };
        let toast = self.pm.trashed.as_ref().filter(|t| t.2.elapsed() < UNDO_FOR).map(|t| t.0.clone());
        let count = if picking {
            format!("{} maps", self.pm_maps().len())
        } else {
            format!("{} projects", self.pm_projects().len())
        };
        let hints = if picking {
            "\u{2190}\u{2191}\u{2192}\u{2193} select \u{00b7} \u{21b5} create \u{00b7} double-click creates \u{00b7} F2 name \u{00b7} \u{2318}F filter \u{00b7} esc projects \u{00b7} drop a .Map.Gbx to start from it"
        } else {
            "\u{2191}\u{2193} select \u{00b7} \u{21b5} open \u{00b7} F2 rename \u{00b7} \u{2318}D duplicate \u{00b7} \u{2318}\u{232b} Trash \u{00b7} \u{2318}F filter"
        };
        div()
            .flex_1()
            .min_h_0()
            .flex()
            .flex_col()
            .child(
                div()
                    .id("projects")
                    .relative()
                    .flex_1()
                    .min_h_0()
                    .flex()
                    .bg(c(theme::BG))
                    .on_drop(cx.listener(|e, paths: &ExternalPaths, _, cx| {
                        if let Some(p) = paths.paths().iter().find(|p| crate::catalog::map_stem(p).is_some())
                        {
                            e.new_project(p, None, cx);
                        }
                        cx.notify();
                    }))
                    .child(self.pm_sidebar(cx))
                    .child(
                        div().flex_1().min_w_0().flex().flex_col().child(self.pm_toolbar(cx)).child(middle),
                    )
                    .child(props)
                    .children(toast.map(|name| {
                        div()
                            .absolute()
                            .bottom(px(theme::SP4))
                            .left(px(0.0))
                            .right(px(0.0))
                            .flex()
                            .justify_center()
                            .child(
                                div()
                                    .flex()
                                    .items_center()
                                    .gap(px(theme::SP3))
                                    .pl(px(theme::SP3))
                                    .pr(px(theme::SP1))
                                    .py(px(theme::SP1))
                                    .rounded(px(theme::R_CTL + 2.0))
                                    .bg(c(theme::RAISED))
                                    .border_1()
                                    .border_color(c(theme::HAIRLINE))
                                    .shadow_lg()
                                    .text_size(px(theme::FS_BODY))
                                    .child(format!("Moved \u{201c}{name}\u{201d} to the Trash"))
                                    .child(btn("pm-undo", "Undo  \u{2318}Z", Btn::Normal).on_click(
                                        cx.listener(|e, _, _, cx| {
                                            e.untrash_project();
                                            cx.notify();
                                        }),
                                    )),
                            )
                    })),
            )
            .child(
                div()
                    .h(px(22.0))
                    .flex_none()
                    .flex()
                    .items_center()
                    .gap(px(theme::SP4))
                    .px(px(theme::PAD))
                    .bg(c(theme::PANEL))
                    .border_t_1()
                    .border_color(c(theme::HAIRLINE))
                    .text_size(px(theme::FS_SMALL))
                    .child(div().text_color(c(theme::MUTED)).child(count))
                    .child(div().flex_1().min_w_0().truncate().text_color(c(theme::FAINT)).child(hints)),
            )
    }

    fn pm_sidebar(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let picking = self.pm.picking_map;
        let tab = self.pm.tab;
        let f = self.pm_filter();
        let count = |t: MapTab| {
            self.pm.cards.values().filter(|c| Self::in_tab(c, t) && Self::map_matches(c, &f)).count()
        };
        let nav = |id: &str, icon_path: &'static str, label: &str, n: String, on: bool| {
            div()
                .id(gpui::ElementId::Name(id.to_string().into()))
                .relative()
                .child(track(id.to_string()))
                .h(px(28.0))
                .mx(px(theme::SP1 + 2.0))
                .px(px(theme::SP2))
                .flex()
                .items_center()
                .gap(px(theme::SP2))
                .rounded(px(theme::R_CTL))
                .text_size(px(theme::FS_BODY))
                .cursor_pointer()
                .when(on, |d| d.bg(c(theme::SELECT)).text_color(c(theme::TEXT)))
                .when(!on, |d| {
                    d.text_color(c(theme::MUTED)).hover(|s| s.bg(c(theme::RAISED)).text_color(c(theme::TEXT)))
                })
                .child(icon(icon_path, if on { theme::TEXT } else { theme::MUTED }, 14.0))
                .child(div().flex_1().truncate().child(label.to_string()))
                .child(div().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child(n))
        };
        let tabs = [
            (MapTab::Recent, "icons/history.svg", "Recent"),
            (MapTab::Mine, "icons/map.svg", "My maps"),
            (MapTab::Downloads, "icons/download.svg", "Downloads"),
            (MapTab::Test, "icons/flask.svg", "Test maps"),
        ];
        div()
            .w(px(SIDE_W))
            .flex_none()
            .flex()
            .flex_col()
            .py(px(theme::SP2))
            .gap(px(2.0))
            .bg(c(theme::PANEL))
            .border_r_1()
            .border_color(c(theme::HAIRLINE))
            .child(
                nav(
                    "pm-nav-projects",
                    "icons/projects.svg",
                    "Projects",
                    self.pm.entries.len().to_string(),
                    !picking,
                )
                .on_click(cx.listener(|e, _, _, cx| {
                    e.pm.picking_map = false;
                    cx.notify();
                })),
            )
            .child(
                div()
                    .pt(px(theme::SP4))
                    .pb(px(theme::SP1))
                    .px(px(theme::PAD))
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(theme::MUTED))
                    .child("New from a map"),
            )
            .children(tabs.map(|(t, ic, label)| {
                nav(&format!("pm-nav-{label}"), ic, label, count(t).to_string(), picking && tab == t)
                    .on_click(cx.listener(move |e, _, _, cx| {
                        if !e.pm.picking_map || e.pm.tab != t {
                            e.pm.selected_map = None;
                            e.pm.new_name = None;
                        }
                        e.pm.picking_map = true;
                        e.pm.tab = t;
                        cx.notify();
                    }))
            }))
            .child(
                nav(
                    "pm-nav-file",
                    "icons/folder.svg",
                    "Open a file\u{2026}",
                    "\u{2318}\u{21e7}O".into(),
                    false,
                )
                .on_click(cx.listener(|e, _, _, cx| e.prompt_map_file(cx))),
            )
            .child(div().flex_1())
            .when(self.pm.cards.len() < self.pm.cards_total, |d| {
                d.child(
                    div()
                        .px(px(theme::PAD))
                        .py(px(theme::SP1))
                        .text_size(px(theme::FS_SMALL))
                        .text_color(c(theme::FAINT))
                        .child(format!(
                            "Reading maps\u{2026} {} of {}",
                            self.pm.cards.len(),
                            self.pm.cards_total
                        )),
                )
            })
    }

    fn pm_toolbar(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let picking = self.pm.picking_map;
        let typing = self.editing.as_ref().is_some_and(|e| e.id == "pm.filter");
        let empty = self.pm.filter.is_empty() && !typing;
        let field = self.text_field("pm.filter", &self.pm.filter.clone(), |e, s, _| e.pm.filter = s, cx);
        let filter = div()
            .w(px(280.0))
            .relative()
            .flex()
            .child(field)
            .when(empty, |d| {
                d.child(
                    div()
                        .absolute()
                        .inset_0()
                        .px(px(theme::SP2))
                        .flex()
                        .items_center()
                        .text_size(px(theme::FS_BODY))
                        .text_color(c(theme::FAINT))
                        .child(div().flex_1().child(if picking {
                            "Filter by map or author"
                        } else {
                            "Filter by name, map, author"
                        }))
                        .child(kbd("\u{2318}F")),
                )
            })
            .when(!empty, |d| {
                d.child(
                    div()
                        .id("pm-clear")
                        .absolute()
                        .right(px(2.0))
                        .top(px(2.0))
                        .size(px(20.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .rounded(px(3.0))
                        .cursor_pointer()
                        .hover(|s| s.bg(c(theme::RAISED)))
                        .child(icon("icons/clear.svg", theme::MUTED, 12.0))
                        .on_click(cx.listener(|e, _, _, cx| {
                            e.pm.filter.clear();
                            e.editing = None;
                            cx.stop_propagation();
                            cx.notify();
                        })),
                )
            });
        let sort = if picking {
            self.choice(
                "pm-msort",
                self.pm.map_sort,
                vec![
                    ("Newest", MapSort::Newest),
                    ("Name", MapSort::Name),
                    ("Author time", MapSort::AuthorTime),
                ],
                |e, v, _| e.pm.map_sort = v,
                cx,
            )
        } else {
            self.choice(
                "pm-sort",
                self.pm.sort,
                vec![("Edited", ProjectSort::Edited), ("Name", ProjectSort::Name), ("Map", ProjectSort::Map)],
                |e, v, _| e.pm.sort = v,
                cx,
            )
        };
        div()
            .h(px(theme::H_BAR))
            .flex_none()
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .px(px(theme::PAD))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .child(filter)
            .child(div().flex_1())
            .child(muted("Sort"))
            .child(sort)
            .when(!picking, |d| {
                d.child(btn_icon("pm-new", "icons/plus.svg", "New project", Btn::Primary).on_click(
                    cx.listener(|e, _, _, cx| {
                        e.show_maps(None);
                        cx.notify();
                    }),
                ))
            })
            .when(self.map.is_some(), |d| {
                d.child(
                    btn("pm-back", "Back to editor", Btn::Quiet)
                        .tooltip(tip("Back to editor  esc", None))
                        .on_click(cx.listener(|e, _, _, cx| {
                            e.projects_open = false;
                            cx.notify();
                        })),
                )
            })
    }

    fn thumb(card: Option<&MapCard>, w: Option<f32>, h: f32) -> AnyElement {
        let sized = |d: gpui::Div| match w {
            Some(w) => d.w(px(w)).h(px(h)),
            None => d.w_full().h(px(h)),
        };
        match card.and_then(|c| c.thumb.clone()) {
            Some(t) => {
                let img = gpui::StyledImage::object_fit(
                    gpui::img(gpui::ImageSource::Render(t)),
                    gpui::ObjectFit::Cover,
                )
                .size_full();
                sized(div()).flex_none().overflow_hidden().child(img).into_any_element()
            }
            None => sized(div())
                .flex_none()
                .bg(c(theme::RAISED))
                .flex()
                .items_center()
                .justify_center()
                .text_size(px(theme::FS_SMALL))
                .text_color(c(theme::FAINT))
                .child(card.and_then(|c| c.environment.clone()).unwrap_or_default())
                .into_any_element(),
        }
    }

    fn project_table(&mut self, list: &[Entry], cur: Option<&Entry>, cx: &mut Context<Self>) -> AnyElement {
        if list.is_empty() {
            let text = if self.pm.entries.is_empty() {
                "No projects yet. Pick a map on the left to start one, or drop a .Map.Gbx here."
            } else {
                "No project matches the filter."
            };
            return div()
                .flex_1()
                .p(px(theme::SP5))
                .text_size(px(theme::FS_BODY))
                .text_color(c(theme::MUTED))
                .child(text)
                .into_any_element();
        }
        let col = |w: f32, s: &str| div().w(px(w)).flex_none().flex().justify_end().child(s.to_string());
        let header = div()
            .h(px(28.0))
            .flex_none()
            .flex()
            .items_center()
            .gap(px(theme::SP3))
            .px(px(theme::PAD))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .text_size(px(theme::FS_SMALL))
            .text_color(c(theme::FAINT))
            .child(div().w(px(72.0)).flex_none())
            .child(div().flex_1().child("Name"))
            .child(col(48.0, "Runs"))
            .child(col(80.0, "Best"))
            .child(col(80.0, "vs author"))
            .child(col(84.0, "Edited"));
        let rows: Vec<AnyElement> = list
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let card = self.entry_card(e).cloned();
                let path = e.path.clone();
                let selected = cur.is_some_and(|c| c.path == e.path);
                let open = self.project_path.as_deref() == Some(path.as_path()) && self.map.is_some();
                let renaming = self.pm.renaming.as_deref() == Some(path.as_path())
                    && self.editing.as_ref().is_some_and(|f| f.id == "pm.rename");
                let name = self.entry_name(e);
                let title: AnyElement = if renaming {
                    let p = path.clone();
                    div()
                        .w(px(320.0))
                        .flex()
                        .child(self.text_field(
                            "pm.rename",
                            &name,
                            move |ed, s, _| {
                                ed.rename_project(&p, &s);
                                ed.pm.renaming = None;
                            },
                            cx,
                        ))
                        .into_any_element()
                } else {
                    div()
                        .flex()
                        .items_center()
                        .gap(px(theme::SP2))
                        .min_w_0()
                        .child(
                            div()
                                .truncate()
                                .text_size(px(theme::FS_EMPH))
                                .font_weight(FontWeight::MEDIUM)
                                .child(name),
                        )
                        .when(open, |d| {
                            d.child(
                                div()
                                    .flex_none()
                                    .text_size(px(theme::FS_SMALL))
                                    .text_color(c(theme::PIN))
                                    .child("\u{25cf} open"),
                            )
                        })
                        .into_any_element()
                };
                let map_name = card.as_ref().map_or_else(
                    || crate::catalog::map_stem(&e.map).unwrap_or_else(|| e.map.display().to_string()),
                    |c| c.name.clone(),
                );
                let mut sub = match card.as_ref().and_then(|c| c.author.clone()) {
                    Some(a) => format!("{map_name} \u{00b7} {a}"),
                    None => map_name,
                };
                if e.legacy {
                    sub += " \u{00b7} older project file";
                }
                let best = e.best_ms();
                let vs =
                    best.zip(card.as_ref().and_then(|c| c.author_time_ms)).map(|(b, a)| delta(b - a as i32));
                let num = |w: f32, s: Option<(String, u32)>| {
                    div().w(px(w)).flex_none().flex().justify_end().child(match s {
                        Some((t, col)) => mono(t, theme::FS_SMALL).text_color(c(col)).into_any_element(),
                        None => div().text_color(c(theme::FAINT)).child("\u{2014}").into_any_element(),
                    })
                };
                let (pm, mp, pr) = (path.clone(), e.map.clone(), path.clone());
                div()
                    .id(("pm-row", i))
                    .relative()
                    .child(track(format!("proj-{i}")))
                    .h(px(ROW_H))
                    .flex_none()
                    .flex()
                    .items_center()
                    .gap(px(theme::SP3))
                    .px(px(theme::PAD))
                    .border_b_1()
                    .border_color(c(theme::HAIRLINE))
                    .cursor_pointer()
                    .when(selected, |d| d.bg(c(theme::SELECT)))
                    .when(!selected, |d| d.hover(|s| s.bg(c(theme::PANEL))))
                    .child(div().rounded(px(theme::R_CTL)).overflow_hidden().child(Self::thumb(
                        card.as_ref(),
                        Some(72.0),
                        40.0,
                    )))
                    .child(
                        div().flex_1().min_w_0().flex().flex_col().gap(px(2.0)).child(title).child(
                            div()
                                .truncate()
                                .text_size(px(theme::FS_SMALL))
                                .text_color(c(theme::MUTED))
                                .child(sub),
                        ),
                    )
                    .child(num(48.0, Some((e.runs.len().to_string(), theme::TEXT))))
                    .child(num(80.0, best.map(|b| (fmt_ms(b), theme::TEXT))))
                    .child(num(80.0, vs))
                    .child(
                        div()
                            .w(px(84.0))
                            .flex_none()
                            .flex()
                            .justify_end()
                            .text_size(px(theme::FS_SMALL))
                            .text_color(c(theme::MUTED))
                            .child(ago(e.modified)),
                    )
                    .on_mouse_down(
                        MouseButton::Right,
                        cx.listener(move |ed, ev: &gpui::MouseDownEvent, _, cx| {
                            ed.pm.selected = Some(pr.clone());
                            ed.project_menu(pr.clone(), mp.clone(), ev.position);
                            cx.stop_propagation();
                            cx.notify();
                        }),
                    )
                    .on_click(cx.listener(move |ed, ev: &gpui::ClickEvent, _, cx| {
                        if ed.pm.renaming.as_deref() == Some(pm.as_path())
                            && ed.editing.as_ref().is_some_and(|f| f.id == "pm.rename")
                        {
                            return;
                        }
                        ed.pm.selected = Some(pm.clone());
                        if ev.click_count() >= 2 {
                            ed.open_listed(&pm, cx);
                        }
                        cx.notify();
                    }))
                    .into_any_element()
            })
            .collect();
        div()
            .flex_1()
            .min_h_0()
            .flex()
            .flex_col()
            .child(header)
            .child(
                div().id("pm-rows").flex_1().min_h_0().overflow_y_scroll().flex().flex_col().children(rows),
            )
            .into_any_element()
    }

    fn props_panel() -> gpui::Div {
        div()
            .w(px(PROPS_W))
            .flex_none()
            .flex()
            .flex_col()
            .bg(c(theme::PANEL))
            .border_l_1()
            .border_color(c(theme::HAIRLINE))
    }

    fn props_title(s: String) -> gpui::Div {
        div()
            .pt(px(theme::SP3))
            .pb(px(theme::SP1))
            .px(px(theme::PAD))
            .text_size(px(theme::FS_TITLE))
            .font_weight(FontWeight::SEMIBOLD)
            .truncate()
            .child(s)
    }

    fn project_props(&mut self, cur: Option<&Entry>, cx: &mut Context<Self>) -> AnyElement {
        let Some(e) = cur.cloned() else { return Self::props_panel().into_any_element() };
        let card = self.entry_card(&e).cloned();
        let best = e.best_ms();
        let at = card.as_ref().and_then(|c| c.author_time_ms);
        let folder = e.path.parent().map(Path::to_path_buf).unwrap_or_default();
        let shown = if e.legacy { e.path.clone() } else { folder };
        let shown = shown
            .strip_prefix(&self.home)
            .map_or_else(|_| shown.display().to_string(), |r| r.display().to_string());
        let runs: Vec<AnyElement> = {
            let mut rs: Vec<(usize, &(String, Option<i32>))> = e.runs.iter().enumerate().collect();
            rs.sort_by_key(|(i, r)| (r.1.unwrap_or(i32::MAX), *i));
            let n = rs.len();
            let mut v: Vec<AnyElement> = rs
                .into_iter()
                .take(8)
                .map(|(i, (name, fin))| {
                    let frac =
                        fin.zip(best).map_or(0.0, |(f, b)| (b as f32 / f.max(1) as f32).clamp(0.0, 1.0));
                    let top = fin.is_some() && *fin == best;
                    div()
                        .h(px(24.0))
                        .px(px(theme::PAD))
                        .flex()
                        .items_center()
                        .gap(px(theme::SP2))
                        .text_size(px(theme::FS_BODY))
                        .child(
                            div()
                                .w(px(120.0))
                                .flex_none()
                                .truncate()
                                .text_color(c(if i == 0 { theme::TEXT } else { theme::MUTED }))
                                .child(name.clone()),
                        )
                        .child(div().flex_1().h(px(3.0)).rounded(px(2.0)).bg(c(theme::WELL)).child(
                            div().h_full().w(gpui::relative(frac)).rounded(px(2.0)).bg(c(if top {
                                theme::PIN
                            } else {
                                theme::FAINT
                            })),
                        ))
                        .child(div().w(px(56.0)).flex_none().flex().justify_end().child(match fin {
                            Some(t) => mono(fmt_ms(*t), theme::FS_SMALL).into_any_element(),
                            None => div().text_color(c(theme::FAINT)).child("\u{2014}").into_any_element(),
                        }))
                        .into_any_element()
                })
                .collect();
            if n > 8 {
                v.push(
                    div()
                        .h(px(24.0))
                        .px(px(theme::PAD))
                        .flex()
                        .items_center()
                        .text_size(px(theme::FS_SMALL))
                        .text_color(c(theme::FAINT))
                        .child(format!("{} more", n - 8))
                        .into_any_element(),
                );
            }
            v
        };
        let name = self.entry_name(&e);
        let (po, pd, pm, mp) = (e.path.clone(), e.path.clone(), e.path.clone(), e.map.clone());
        let (rp, rv) = (e.path.clone(), shown.clone());
        Self::props_panel()
            .child(Self::thumb(card.as_ref(), None, 180.0))
            .child(Self::props_title(name))
            .child(prop(
                "Map",
                text(card.as_ref().map_or_else(
                    || crate::catalog::map_stem(&e.map).unwrap_or_default(),
                    |c| c.name.clone(),
                ))
                .truncate(),
            ))
            .when_some(card.as_ref().and_then(|c| c.author.clone()), |d, a| {
                d.child(prop("Author", text(a).truncate()))
            })
            .when_some(at, |d, t| d.child(prop("Author time", mono(fmt_ms(t as i32), theme::FS_BODY))))
            .child(prop(
                "Best",
                match best {
                    Some(b) => div()
                        .flex()
                        .gap(px(theme::SP2))
                        .child(mono(fmt_ms(b), theme::FS_BODY))
                        .when_some(at.map(|a| delta(b - a as i32)), |d, (t, col)| {
                            d.child(mono(t, theme::FS_BODY).text_color(c(col)))
                        })
                        .into_any_element(),
                    None => muted("no finish yet").into_any_element(),
                },
            ))
            .child(prop("Goals \u{00b7} setups", text(format!("{} \u{00b7} {}", e.goals, e.setups))))
            .child(prop("Edited", text(ago(e.modified))))
            .child(prop(
                "Folder",
                div()
                    .id("pm-folder")
                    .truncate()
                    .font_family(theme::MONO)
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(theme::FAINT))
                    .cursor_pointer()
                    .hover(|s| s.text_color(c(theme::TEXT)))
                    .tooltip(tip("Show in Finder", Some(&rv)))
                    .child(shown)
                    .on_click(move |_, _, _| reveal(&rp)),
            ))
            .child(heading(format!("Runs  {}", e.runs.len())))
            .children(runs)
            .child(div().flex_1())
            .child(
                div()
                    .flex()
                    .gap(px(theme::SP2))
                    .p(px(theme::PAD))
                    .border_t_1()
                    .border_color(c(theme::HAIRLINE))
                    .child(btn("pm-open", "Open", Btn::Primary).flex_1().on_click(cx.listener(
                        move |ed, _, _, cx| {
                            ed.open_listed(&po, cx);
                            cx.notify();
                        },
                    )))
                    .child(btn("pm-dup", "Duplicate", Btn::Normal).on_click(cx.listener(
                        move |ed, _, _, cx| {
                            ed.duplicate_project(&pd);
                            cx.notify();
                        },
                    )))
                    .child(
                        btn("pm-more", "", Btn::Quiet)
                            .px(px(theme::SP2))
                            .child(icon("icons/more.svg", theme::MUTED, 14.0))
                            .on_click(cx.listener(move |ed, ev: &gpui::ClickEvent, _, cx| {
                                ed.project_menu(pm.clone(), mp.clone(), ev.position());
                                cx.stop_propagation();
                                cx.notify();
                            })),
                    ),
            )
            .into_any_element()
    }

    fn map_grid(&mut self, maps: &[MapCard], cur: Option<&MapCard>, cx: &mut Context<Self>) -> AnyElement {
        if maps.is_empty() {
            let text = if self.pm.cards.len() < self.pm.cards_total {
                "Reading maps\u{2026}"
            } else {
                "No map matches."
            };
            return div()
                .flex_1()
                .p(px(theme::SP5))
                .text_size(px(theme::FS_BODY))
                .text_color(c(theme::MUTED))
                .child(text)
                .into_any_element();
        }
        let mut per_map: std::collections::HashMap<&Path, usize> = std::collections::HashMap::new();
        for e in &self.pm.entries {
            *per_map.entry(e.map.as_path()).or_default() += 1;
        }
        let thumb_h = (self.pm.card_w * 0.6).max(90.0);
        let cards = maps.iter().take(300).enumerate().map(|(i, card)| {
            let path = card.path.clone();
            let selected = cur.is_some_and(|c| c.path == card.path);
            let projects = per_map.get(card.path.as_path()).copied().unwrap_or(0);
            let sub = match (&card.author, card.author_time_ms) {
                (Some(a), Some(t)) => format!("{a} \u{00b7} {}", fmt_ms(t as i32)),
                (Some(a), None) => a.clone(),
                (None, Some(t)) => fmt_ms(t as i32),
                (None, None) => String::new(),
            };
            div()
                .id(("pm-card", i))
                .relative()
                .child(track(format!("card-{i}")))
                .min_w_0()
                .rounded(px(theme::RADIUS - 2.0))
                .overflow_hidden()
                .bg(c(theme::PANEL))
                .border_1()
                .border_color(c(if selected { theme::TEXT } else { theme::HAIRLINE }))
                .cursor_pointer()
                .when(!selected, |d| d.hover(|s| s.border_color(c(theme::FAINT))))
                .child(Self::thumb(Some(card), None, thumb_h))
                .when(projects > 0, |d| {
                    d.child(
                        div()
                            .absolute()
                            .top(px(6.0))
                            .right(px(6.0))
                            .px(px(6.0))
                            .py(px(1.0))
                            .rounded(px(3.0))
                            .bg(theme::alpha(0x000000, 0.7))
                            .text_size(px(theme::FS_SMALL))
                            .child(if projects == 1 {
                                "1 project".to_string()
                            } else {
                                format!("{projects} projects")
                            }),
                    )
                })
                .child(
                    div()
                        .px(px(theme::SP2 + 1.0))
                        .py(px(theme::SP2 - 1.0))
                        .flex()
                        .flex_col()
                        .gap(px(2.0))
                        .child(
                            div()
                                .text_size(px(theme::FS_BODY))
                                .font_weight(FontWeight::MEDIUM)
                                .truncate()
                                .child(card.name.clone()),
                        )
                        .child(
                            div()
                                .text_size(px(theme::FS_SMALL))
                                .text_color(c(theme::MUTED))
                                .truncate()
                                .child(sub),
                        ),
                )
                .on_click(cx.listener(move |e, ev: &gpui::ClickEvent, _, cx| {
                    if e.pm.selected_map.as_ref() != Some(&path) {
                        e.pm.new_name = None;
                    }
                    e.pm.selected_map = Some(path.clone());
                    if ev.click_count() >= 2 {
                        let name = e.pm.new_name.take();
                        e.new_project(&path, name, cx);
                    }
                    cx.notify();
                }))
        });
        div()
            .id("pm-grid")
            .flex_1()
            .min_h_0()
            .overflow_y_scroll()
            .child(
                div()
                    .p(px(theme::PAD))
                    .grid()
                    .grid_cols(self.pm.cols as u16)
                    .gap(px(theme::SP3))
                    .children(cards),
            )
            .into_any_element()
    }

    fn map_props(&mut self, cur: Option<&MapCard>, cx: &mut Context<Self>) -> AnyElement {
        let Some(card) = cur.cloned() else { return Self::props_panel().into_any_element() };
        let from = match card.source {
            Source::Mine => "My maps",
            Source::Downloads => "Downloads",
            Source::Test => "Test maps",
        };
        let home = dirs::home_dir().map(|h| h.display().to_string()).unwrap_or_default();
        let file = card.path.display().to_string();
        let file = match file.strip_prefix(&home) {
            Some(r) if !home.is_empty() => format!("~{r}"),
            _ => file,
        };
        let existing: Vec<Entry> = self.pm.entries.iter().filter(|e| e.map == card.path).cloned().collect();
        let rows: Vec<AnyElement> = existing
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let p = e.path.clone();
                div()
                    .id(("pm-existing", i))
                    .h(px(26.0))
                    .mx(px(theme::SP1 + 2.0))
                    .px(px(theme::SP2))
                    .flex()
                    .items_center()
                    .gap(px(theme::SP2))
                    .rounded(px(theme::R_CTL))
                    .text_size(px(theme::FS_BODY))
                    .cursor_pointer()
                    .hover(|s| s.bg(c(theme::RAISED)))
                    .tooltip(tip("Open", None))
                    .child(div().flex_1().truncate().child(self.entry_name(e)))
                    .child(match e.best_ms() {
                        Some(b) => mono(fmt_ms(b), theme::FS_SMALL).into_any_element(),
                        None => div().text_color(c(theme::FAINT)).child("\u{2014}").into_any_element(),
                    })
                    .child(
                        div()
                            .w(px(64.0))
                            .flex()
                            .justify_end()
                            .text_size(px(theme::FS_SMALL))
                            .text_color(c(theme::FAINT))
                            .child(ago(e.modified)),
                    )
                    .on_click(cx.listener(move |ed, _, _, cx| {
                        ed.open_listed(&p, cx);
                        cx.notify();
                    }))
                    .into_any_element()
            })
            .collect();
        let taken = existing.iter().any(|e| self.entry_name(e) == card.name);
        let default_name =
            if taken { format!("{} ({})", card.name, existing.len() + 1) } else { card.name.clone() };
        let name = self.pm.new_name.clone().unwrap_or(default_name.clone());
        let name_field = self.text_field("pm.newname", &name, |e, s, _| e.pm.new_name = Some(s), cx);
        let (pc, pr) = (card.path.clone(), card.path.clone());
        let newest = existing.iter().max_by_key(|e| e.modified).map(|e| e.path.clone());
        Self::props_panel()
            .child(Self::thumb(Some(&card), None, 180.0))
            .child(Self::props_title(card.name.clone()))
            .when_some(card.author.clone(), |d, a| d.child(prop("Author", text(a).truncate())))
            .when_some(card.author_time_ms, |d, t| {
                d.child(prop("Author time", mono(fmt_ms(t as i32), theme::FS_BODY)))
            })
            .when_some(
                card.environment.clone().filter(|e| !e.chars().all(|c| c.is_ascii_digit())),
                |d, env| d.child(prop("Environment", text(env))),
            )
            .child(prop("From", text(format!("{from} \u{00b7} {}", ago(card.modified)))))
            .child(prop(
                "File",
                div()
                    .id("pm-file")
                    .truncate()
                    .font_family(theme::MONO)
                    .text_size(px(theme::FS_SMALL))
                    .text_color(c(theme::FAINT))
                    .cursor_pointer()
                    .hover(|s| s.text_color(c(theme::TEXT)))
                    .tooltip(tip("Show in Finder", Some(&file.clone())))
                    .child(file)
                    .on_click(move |_, _, _| reveal(&pr)),
            ))
            .when(!rows.is_empty(), |d| {
                d.child(heading(format!("Projects on this map  {}", rows.len()))).children(rows)
            })
            .child(div().flex_1())
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(theme::SP2))
                    .p(px(theme::PAD))
                    .border_t_1()
                    .border_color(c(theme::HAIRLINE))
                    .child(
                        div()
                            .flex()
                            .items_center()
                            .gap(px(theme::SP3))
                            .child(
                                div()
                                    .w(px(44.0))
                                    .text_size(px(theme::FS_BODY))
                                    .text_color(c(theme::MUTED))
                                    .child("Name"),
                            )
                            .child(div().flex_1().flex().child(name_field)),
                    )
                    .child(
                        div()
                            .flex()
                            .gap(px(theme::SP2))
                            .child(
                                btn("pm-create", "Create project  \u{21b5}", Btn::Primary).flex_1().on_click(
                                    cx.listener(move |e, _, _, cx| {
                                        let name = e.pm.new_name.take();
                                        e.new_project(&pc, name, cx);
                                        cx.notify();
                                    }),
                                ),
                            )
                            .when_some(newest, |d, p| {
                                d.child(btn("pm-open-existing", "Open existing", Btn::Normal).on_click(
                                    cx.listener(move |e, _, _, cx| {
                                        e.open_listed(&p, cx);
                                        cx.notify();
                                    }),
                                ))
                            }),
                    ),
            )
            .into_any_element()
    }
}
