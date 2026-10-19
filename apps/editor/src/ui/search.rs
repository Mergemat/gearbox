use gpui::prelude::FluentBuilder;
use gpui::{AnyElement, AppContext, Bounds, Context, InteractiveElement, IntoElement, ParentElement, PathBuilder, Pixels, StatefulInteractiveElement, Styled, Window, canvas, div, fill, point, px, size};

use crate::app::*;
use crate::doc::GoalKind;
use crate::fmt::{fmt_delta, fmt_ms};
use crate::optimize::{self, Aim, Metric, ObjKind};
use crate::theme::{self, c};
use crate::ui::widgets::*;

const PCT: Num = Num { decimals: 0, unit: "%", step: 0.5, min: 0.0, max: 200.0, width: 0.0 };

const SHOWN_CANDIDATES: usize = 50;

#[derive(Default)]
pub struct DiffCache {
    key: (u64, Vec<u32>, usize),
    diffs: std::collections::HashMap<u64, Vec<GoalDiff>>,
    first: std::collections::HashMap<u64, Option<usize>>,
}

#[derive(Clone)]
pub struct GoalDiff {
    name: String,
    yours: Option<(i32, f32)>,
    this: Option<(i32, f32)>,
    miss: Option<(f32, f32)>,
}

impl GoalDiff {
    fn changed(&self) -> bool {
        match (self.this, self.yours, self.miss) {
            (Some(a), Some(b), _) => a.0 != b.0 || (a.1 - b.1).abs() >= 0.05,
            (None, None, Some((a, b))) => (a - b).abs() > 0.05,
            (a, b, _) => a.is_some() != b.is_some(),
        }
    }

    fn table_row(&self) -> gpui::Div {
        let cell = |t: Option<(i32, f32)>| t.map_or("\u{2014}".to_string(), |(ms, v)| format!("{} \u{00b7} {v:.0}", fmt_ms(ms)));
        let (change, col) = match (self.this, self.yours, self.miss) {
            (Some((a, _)), Some((b, _)), _) => (fmt_delta(a - b), if a < b { theme::OK } else if a > b { theme::WARN } else { theme::MUTED }),
            (Some(_), None, _) => ("new".into(), theme::OK),
            (None, Some(_), _) => ("lost".into(), theme::ERROR),
            (None, None, Some((a, b))) if a.is_finite() && b.is_finite() => (format!("{:+.1} m", a - b), if a < b { theme::OK } else { theme::WARN }),
            _ => ("\u{2014}".into(), theme::MUTED),
        };
        div()
            .h(px(20.0))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .child(div().flex_1().min_w_0().truncate().text_size(px(theme::FS_BODY)).child(self.name.clone()))
            .child(mono(cell(self.yours), theme::FS_SMALL).w(px(104.0)).flex_none().whitespace_nowrap().text_color(c(theme::FAINT)))
            .child(mono(cell(self.this), theme::FS_SMALL).w(px(104.0)).flex_none().whitespace_nowrap())
            .child(mono(change, theme::FS_SMALL).w(px(64.0)).flex_none().whitespace_nowrap().text_color(c(col)))
    }
}

impl Editor {
    pub fn objective_for_goal(&mut self, id: u32) {
        let Some(g) = self.doc.goal(id).cloned() else { return };
        if !self.spec.kind.goals() {
            self.spec.kind = ObjKind::ReachGoals;
        }
        self.spec.goals = vec![id];
        let i = self.doc.goals.iter().position(|x| x.id == id).unwrap_or(0);
        let first = self.start_tick().unwrap_or(0).max(self.race_start_tick());
        let at = self.reach.get(i).and_then(|r| r.tick.or(r.closest.map(|c| c.0)));
        let from = match at {
            Some(k) if self.cursor > first && self.cursor + 10 < k => self.cursor,
            Some(k) => k.saturating_sub(300).max(first.min(k)),
            None => self.cursor.max(first),
        };
        let to = at.filter(|&k| k > from + 10).unwrap_or(from + 300);
        self.spec.window = from..to;
        self.info("search", format!("aim: {} from {} s", g.name, fmt_ms(self.race_ms0 + 10 * from as i32)));
    }

    pub fn search_toward(&mut self, id: u32) {
        self.objective_for_goal(id);
        self.job = format!("Search to {}", self.doc.goal(id).map_or("goal".into(), |g| g.name.clone()));
        self.start_search();
    }

    fn search_ctx(&self) -> (Vec<Option<usize>>, Vec<u16>) {
        self.search_ctx_parts()
    }

    fn secs(&self, tick: usize) -> f64 {
        f64::from(self.race_ms0 + 10 * tick as i32) / 1000.0
    }

    fn judging(&self) -> Vec<String> {
        let mut v = vec![self.spec.kind.about().to_string()];
        let goals = match self.spec.kind {
            ObjKind::ReachGoals => self.spec.ordered_goals(&self.doc.goals),
            _ => Vec::new(),
        };
        for (n, g) in goals.iter().enumerate() {
            let how = match &g.kind {
                GoalKind::Gate(x) if !x.also.is_empty() => format!("counts when the car crosses any of its {} blocks, at the game's own crossing time", x.also.len() + 1),
                GoalKind::Gate(_) => "counts when the car crosses its trigger, at the game's own crossing time".to_string(),
                GoalKind::Point { radius, .. } => format!("counts when the car's centre is within {radius:.1} m of it at a tick"),
                GoalKind::Box { size, .. } => format!("counts when the car's centre is inside the {:.0} \u{00d7} {:.0} \u{00d7} {:.0} m box, or passes through it between ticks", size[0], size[1], size[2]),
                GoalKind::Pose(p) => format!(
                    "counts when position is within \u{00b1}{:.1} m and rotation within \u{00b1}{:.0}\u{00b0}{}",
                    p.pos_tol.iter().copied().fold(0.0f32, f32::max),
                    p.angle_tol,
                    if p.use_velocity { ", velocity too" } else { "" }
                ),
            };
            let order = if goals.len() > 1 { format!("{}. ", n + 1) } else { String::new() };
            v.push(format!("{order}{}: {how}.", g.name));
        }
        v
    }

    pub fn search_panel(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let (reach, wps) = self.search_ctx();
        let w = |k: usize| wps.get(k.min(wps.len().saturating_sub(1))).copied().unwrap_or(0) as usize;
        let ctx = optimize::Ctx { goals: &self.doc.goals, race_ms0: self.race_ms0, reach: &reach, waypoints_at: &w };
        let sentence = self.spec.sentence(&ctx);
        let constraint = self.spec.constraint_sentence(&ctx);
        let semantics = self.spec.semantics();
        let until = self.spec.until_tick(&ctx);
        let judging = self.judging();
        let err = self.field_errors.get("search").cloned();
        let mut body: Vec<AnyElement> = Vec::new();
        let rules = std::iter::once(sentence.clone()).chain(std::iter::once(constraint)).chain(judging).chain(std::iter::once(String::new())).chain(semantics).collect::<Vec<_>>().join("\n");
        if let Some(e) = err {
            body.push(div().mx(px(theme::PAD)).mt(px(theme::SP2)).text_size(px(theme::FS_BODY)).text_color(c(theme::ERROR)).child(e).into_any_element());
        }
        let busy = self.search.running() || !self.search.live.lock().unwrap().candidates.is_empty();
        let aim = self.spec.kind.aim();
        let help = div().id("search-rules").flex().child(icon("icons/help.svg", theme::FAINT, 13.0)).tooltip(tip("How a try wins", Some(&rules))).into_any_element();
        let (open, h) = self.fold_heading("aim", "icons/target.svg", "Aim", busy, Some(help), cx);
        body.push(h);
        if open {
        let kind = self.choice("aim", aim, vec![("@icons/finish.svg|Reach goals", Aim::Goals), ("@icons/stopwatch.svg|Best at a time", Aim::Moment)], |e, a, _| {
            if e.spec.kind.aim() != a {
                e.spec.kind = match a {
                    Aim::Goals => ObjKind::ReachGoals,
                    Aim::Moment => ObjKind::Moment,
                };
            }
        }, cx);
        body.push(prop("Kind", kind).id("aim-row").tooltip(tip("Aim", Some("Reach goals: one or several in order. Best at a time: one moment, or the best tick up to it, judged by a measure, conditions and a tie-break."))).into_any_element());
        match aim {
            Aim::Goals => {
                if self.doc.goals.is_empty() {
                    body.push(prop("Goals", muted("None yet: right-click a checkpoint, or press 1\u{2013}4 and click in the view.")).into_any_element());
                } else {
                    let list = self.goal_list(cx);
                    body.push(prop("Goals", div()).into_any_element());
                    body.push(div().px(px(theme::PAD)).pb(px(theme::SP2)).child(list).into_any_element());
                }
                body.push(prop("Checkpoints", checkbox("spec-keep-cps", self.spec.keep_cps, "never miss one").tooltip(tip("Never miss a checkpoint", Some("A try that skips a checkpoint your run takes doesn't count."))).on_click(cx.listener(|this, _, _, cx| {
                    this.spec.keep_cps = !this.spec.keep_cps;
                    cx.notify();
                }))).into_any_element());
            }
            Aim::Moment => {
                let mut rows = self.aim_rows(cx);
                let at = self.spec.at_tick.unwrap_or(self.spec.window.end);
                let f = self.time_field("spec.at", at, |e, k, _| e.spec.at_tick = Some(k), cx);
                let label = if self.spec.aim.judged == gearbox_search::aim::Judged::Best { "Up to" } else { "At" };
                rows.insert(2, prop(label, f).into_any_element());
                body.extend(rows);
            }
        }
        }

        let (open, h) = self.fold_heading("change", "icons/steer.svg", "Change", busy, None, cx);
        body.push(h);
        if open {
        let from = self.time_field("spec.w0", self.spec.window.start, |e, k, _| e.spec.window = k..e.spec.window.end.max(k + 1), cx);
        let to = self.time_field("spec.w1", self.spec.window.end, |e, k, _| e.spec.window = e.spec.window.start.min(k.saturating_sub(1))..k, cx);
        let has_sel = self.tick_sel.is_some();
        let use_sel = div()
            .id("range-from-sel")
            .relative()
            .child(track("range-from-sel"))
            .size(px(24.0))
            .flex_none()
            .flex()
            .items_center()
            .justify_center()
            .rounded(px(theme::R_CTL))
            .when(has_sel, |d| d.cursor_pointer().hover(|s| s.bg(c(theme::RAISED))))
            .child(icon("icons/range.svg", if has_sel { theme::MUTED } else { theme::FAINT }, 14.0))
            .tooltip(tip("The timeline's selection (\u{2318}W)", Some("Select ticks in the timeline first.")))
            .on_click(cx.listener(|this, _, _, cx| {
                if let Some(r) = this.tick_sel.clone() {
                    this.spec.window = r;
                }
                cx.notify();
            }));
        let range = div().flex().items_center().gap(px(theme::SP2)).child(div().flex_1().child(from)).child(muted("to")).child(div().flex_1().child(to)).child(use_sel);
        body.push(prop("Range", range).id("from-row").tooltip(tip("Inputs it may change", Some("Inputs before and after stay exactly as they are. Drawn on the timeline; drag its edges there."))).into_any_element());
        let ch = self.spec.channels;
        let chans = segments()
            .children([("Steer", "icons/steer.svg", 2usize), ("Gas", "icons/gas.svg", 0), ("Brake", "icons/brake.svg", 1)].into_iter().map(|(n, path, k)| {
                segment(format!("ch-{n}"), ch[k], format!("@{path}|{n}"))
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.spec.channels[k] = !this.spec.channels[k];
                        cx.notify();
                    }))
            }));
        let chans = div()
            .flex()
            .flex_wrap()
            .items_center()
            .gap(px(theme::SP3))
            .child(chans)
            .when(ch[2], |d| {
                d.child(checkbox("steer-digital", self.spec.digital_steer, "keys only").tooltip(tip("Keyboard steer", Some("Steer is only full left, straight or full right."))).on_click(cx.listener(|this, _, _, cx| {
                    this.spec.digital_steer = !this.spec.digital_steer;
                    cx.notify();
                })))
            });
        body.push(prop("Inputs", chans).into_any_element());
        let mut lines: Vec<(String, optimize::StartFrom)> = vec![("@icons/play.svg|Your run".into(), optimize::StartFrom::Run)];
        lines.extend(self.doc.guides.iter().map(|g| (format!("@icons/line.svg|{}", g.name), optimize::StartFrom::Line(g.id))));
        if let Some(g) = self.ghosts.first() {
            lines.push((format!("@icons/ghost.svg|Ghost {}", g.track.name), optimize::StartFrom::Ghost));
        }
        let sel = self.dropdown("spec.start", self.spec.start_from, lines, |e, v, _| e.spec.start_from = v, cx);
        body.push(prop("Start from", sel).id("start-from-row").tooltip(tip("The line it challenges", Some("Your run, or a line you drew (or a ghost's), driven over the range with the gas held. It is only the first guess: the results go anywhere faster."))).into_any_element());
        if self.doc.start.is_some() {
            body.push(prop("Car", checkbox("spec-vary-car", self.spec.vary_car, "vary its angles and speed").tooltip(tip("Vary the placed car", Some("Each try may also turn the car (yaw, pitch, roll) and change its speed, so you don't have to guess a state that works. The result's car is shown with it; Use it places it."))).on_click(cx.listener(|this, _, _, cx| {
                this.spec.vary_car = !this.spec.vary_car;
                cx.notify();
            }))).into_any_element());
        }
        }

        let (open, h) = self.fold_heading("stop", "icons/stopwatch.svg", "Stop", busy, None, cx);
        body.push(h);
        if open {
        let secs = if self.spec.by_time { self.spec.seconds } else { 0.0 };
        let budget = self.budget_field("spec.budget", secs, |e, v, _| {
            e.spec.by_time = true;
            e.spec.seconds = v;
        }, cx);
        body.push(prop("After", budget).into_any_element());
        body.push(prop("Then", checkbox("spec-go-on", self.spec.go_on, "go on by itself").tooltip(tip("Go on by itself", Some("When the time is up and it found something better, it uses it, moves the range on by its length and searches again, until you stop it."))).on_click(cx.listener(|this, _, _, cx| {
            this.spec.go_on = !this.spec.go_on;
            cx.notify();
        }))).into_any_element());
        }
        let unreached: Vec<String> = self.aimed_goals().iter().filter(|g| self.doc.goals.iter().position(|x| x.id == g.id).and_then(|i| self.reach.get(i)).is_none_or(|r| r.tick.is_none())).map(|g| g.name.clone()).collect();
        if !unreached.is_empty() && self.spec.window.end + 50 < until {
            let end = self.secs(self.spec.window.end);
            body.push(
                div()
                    .mx(px(theme::PAD))
                    .mt(px(theme::SP3))
                    .p(px(theme::SP3))
                    .rounded(px(theme::R_CTL + 2.0))
                    .bg(theme::alpha(theme::WARN, 0.08))
                    .flex()
                    .flex_col()
                    .gap(px(theme::SP2))
                    .child(div().text_size(px(theme::FS_BODY)).child(format!("Your run never reaches {}. Tries can only change inputs up to {end:.2} s; after that the car keeps your run's inputs, so they may never get there.", unreached.join(", "))))
                    .child(div().flex().child(btn("win-extend", format!("Change inputs up to {:.2} s", self.secs(until)), Btn::Normal).on_click(cx.listener(move |this, _, _, cx| {
                        this.spec.window.end = until;
                        cx.notify();
                    }))))
                    .into_any_element(),
            );
        }
        let more = self.collapsed.contains("advanced-open");
        body.push(self.fold_row("adv-h", "icons/sliders.svg", "Advanced", more, None, "advanced-open", true, cx));
        if more {
            let locks = self.doc.locks.len();
            if locks > 0 {
                body.push(prop("Locks", checkbox("spec-locks", self.spec.honour_locks, format!("keep {locks} locked range{}", if locks == 1 { "" } else { "s" })).on_click(cx.listener(|this, _, _, cx| {
                    this.spec.honour_locks = !this.spec.honour_locks;
                    cx.notify();
                }))).into_any_element());
            }
            body.extend(self.advanced(until, cx));
        }
        body.push(self.live_section(cx));
        let running = self.search.running();
        let (attempts, elapsed, found, ended) = {
            let l = self.search.live.lock().unwrap();
            let ended = if let Some(why) = l.message.strip_prefix("stopped: ") {
                format!("Stopped: {why}")
            } else if l.message.starts_with("stopped") || l.error.is_some() {
                "Stopped".to_string()
            } else {
                "Done".to_string()
            };
            (l.attempts, l.elapsed, l.candidates.len(), ended)
        };
        let status = if running && self.search_stale.is_some() {
            "Aim changed \u{00b7} searching again\u{2026}".to_string()
        } else if running {
            format!("{attempts} tries \u{00b7} {:.0}/s \u{00b7} {found} found", attempts as f32 / elapsed.max(0.01))
        } else if attempts > 0 {
            format!("{ended} \u{00b7} {attempts} tries \u{00b7} {found} found")
        } else {
            "Ready".into()
        };
        let done = (running && self.spec.by_time && self.spec.seconds > 0.0).then(|| (elapsed / self.spec.seconds).clamp(0.0, 1.0));
        let footer = div()
            .h(px(theme::H_BAR + theme::SP2))
            .flex_none()
            .relative()
            .px(px(theme::PAD))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .border_t_1()
            .border_color(c(theme::HAIRLINE))
            .children(done.map(|f| div().absolute().top_0().left_0().h(px(2.0)).w(gpui::relative(f)).bg(c(theme::PIN))))
            .when(running && done.is_none(), |d| d.child(div().size(px(6.0)).flex_none().rounded_full().bg(c(theme::PIN))))
            .child(div().flex_1().min_w_0().truncate().text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(status))
            .child(checkbox("spec-auto", self.spec.auto, "Auto").tooltip(tip("Search when a goal is placed or moved", Some("A goal you place or move joins the aim and the search runs, with the rest of its setup kept. With nothing aimed at yet, it searches to the goal from the playhead.\n\nOn or off, a running search starts again by itself when a goal it aims at, the car or a setting changes."))).on_click(cx.listener(|this, _, _, cx| {
                this.spec.auto = !this.spec.auto;
                cx.notify();
            })))
            .child(if running {
                btn_icon("search-stop", "icons/stop.svg", "Stop  \u{2318}.", Btn::Danger).on_click(cx.listener(|this, _, _, cx| {
                    this.search.stop();
                    cx.notify();
                })).into_any_element()
            } else {
                btn_icon("search-run", "icons/sparkle.svg", "Search  \u{2318}R", Btn::Primary).on_click(cx.listener(|this, _, _, cx| {
                    this.start_search();
                    cx.notify();
                })).into_any_element()
            });
        div()
            .id("search-panel")
            .size_full()
            .flex()
            .flex_col()
            .child(div().id("search-body").flex_1().min_h_0().overflow_y_scroll().pb(px(16.0)).flex().flex_col().children(body.into_iter().map(|e| div().flex_none().child(e))))
            .child(footer)
    }

    fn fold_heading(&mut self, key: &str, path: &'static str, title: &str, busy: bool, extra: Option<AnyElement>, cx: &mut Context<Self>) -> (bool, AnyElement) {
        let k = if busy { format!("search-open-{key}") } else { format!("search-fold-{key}") };
        let open = self.collapsed.contains(&k) == busy;
        (open, self.fold_row(&format!("sec-{key}"), path, title, open, extra, &k, key != "aim", cx))
    }

    #[allow(clippy::too_many_arguments)]
    fn fold_row(&mut self, id: &str, path: &'static str, title: &str, open: bool, extra: Option<AnyElement>, key: &str, rule: bool, cx: &mut Context<Self>) -> AnyElement {
        let key = key.to_string();
        div()
            .id(gpui::ElementId::Name(id.to_string().into()))
            .relative()
            .child(track(id.to_string()))
            .mt(px(theme::SP3))
            .mx(px(theme::PAD))
            .pt(px(theme::SP2))
            .h(px(30.0))
            .when(rule, |d| d.border_t_1().border_color(c(theme::HAIRLINE)))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .cursor_pointer()
            .text_size(px(theme::FS_EMPH))
            .font_weight(gpui::FontWeight::SEMIBOLD)
            .text_color(c(if open { theme::TEXT } else { theme::MUTED }))
            .hover(|s| s.text_color(c(theme::TEXT)))
            .child(div().w(px(10.0)).text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child(if open { "\u{25be}" } else { "\u{25b8}" }))
            .child(icon(path, if open { theme::MUTED } else { theme::FAINT }, 14.0))
            .child(div().flex_1().child(title.to_string()))
            .children(extra)
            .on_click(cx.listener(move |this, _, _, cx| {
                if !this.collapsed.remove(&key) {
                    this.collapsed.insert(key.clone());
                }
                cx.notify();
            }))
            .into_any_element()
    }

    fn first_change(&mut self, id: u64) -> Option<usize> {
        if let Some(k) = self.diff_cache.first.get(&id) {
            return *k;
        }
        let inputs = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).map(|c| c.inputs.clone())?;
        let ms = |k: usize| self.race_ms0 + 10 * k as i32;
        let n = self.doc.len.max(((inputs.end_ms() - self.race_ms0).max(0) / 10) as usize);
        let k = (0..n).find(|&k| inputs.at(ms(k)) != self.doc.inputs.at(ms(k)));
        self.diff_cache.first.insert(id, k);
        k
    }

    fn advanced(&mut self, until: usize, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let mut v: Vec<AnyElement> = Vec::new();
        if !self.spec.kind.fixed_time() {
            let auto = self.spec.until.is_none();
            let f = self.time_field("spec.until", until, |e, k, _| e.spec.until = Some(k), cx);
            let field = div()
                .flex()
                .items_center()
                .gap(px(theme::SP2))
                .w_full()
                .child(f)
                .child(checkbox("spec-until-auto", auto, "auto").on_click(cx.listener(|this, _, _, cx| {
                    this.spec.until = if this.spec.until.is_some() { None } else { Some(this.spec.window.end + 300) };
                    cx.notify();
                })));
            v.push(prop("Drive until", field).id("adv-until").tooltip(tip("How far each try is driven", Some("Also a time limit: a try that gets to the goal later doesn't count. Set it just under a time to beat (a ghost's, say). Auto: 1.5 s after your run reaches the goal."))).into_any_element());
        }
        if self.spec.channels[2] {
            let f = self.num("spec.slimit", f64::from(self.spec.steer_limit) * 100.0, PCT.range(0.0, 100.0), |e, x, _| e.spec.steer_limit = (x / 100.0) as f32, cx);
            v.push(prop("Max steer", f).id("adv-steer").tooltip(tip("No try steers further than this", None)).into_any_element());
        }
        let f = self.num("spec.threads", self.spec.threads as f64, INT.range(0.0, gearbox_search::search::MAX_THREADS as f64), |e, x, _| e.spec.threads = x.round() as usize, cx);
        let help = format!("0: automatic ({}, every core but one).", gearbox_search::search::default_threads());
        v.push(prop("Threads", f).id("adv-threads").tooltip(tip("Parallel workers", Some(&help))).into_any_element());
        let f = self.num("spec.seed", self.spec.seed as f64, INT.range(0.0, 1e12), |e, x, _| e.spec.seed = x.round() as u64, cx);
        v.push(prop("Seed", f).id("adv-seed").tooltip(tip("Same seed and settings, same tries", None)).into_any_element());
        v
    }

    fn goal_list(&mut self, cx: &mut Context<Self>) -> AnyElement {
        let picked: Vec<(u32, String, Option<i32>, Option<(String, String)>)> = self
            .spec
            .ordered_goals(&self.doc.goals)
            .iter()
            .map(|g| {
                let when = g.when.summary();
                (g.id, g.name.clone(), self.doc.goals.iter().position(|x| x.id == g.id).and_then(|i| self.reach.get(i)).and_then(|r| r.ms), (!when.is_empty()).then(|| (when.join(" \u{00b7} "), g.when.details())))
            })
            .collect();
        let mut col = div().w_full().flex().flex_col().gap(px(2.0));
        if picked.is_empty() {
            col = col.child(div().h(px(24.0)).flex().items_center().child(muted("None yet")));
        }
        let mut latest: Option<i32> = None;
        for (n, (id, name, ms, when)) in picked.iter().cloned().enumerate() {
            let name_for_drag: gpui::SharedString = name.clone().into();
            let early = ms.zip(latest).is_some_and(|(a, b)| a < b);
            latest = latest.max(ms);
            let speed = self.spec.by_speed.contains(&id);
            let mut row = div()
                .h(px(26.0))
                .flex()
                .items_center()
                .gap(px(theme::SP2))
                .child(div().w(px(10.0)).flex_none().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child("\u{2807}"))
                .child(mono(format!("{}", n + 1), theme::FS_SMALL).w(px(14.0)).flex_none().text_color(c(theme::FAINT)))
                .child(div().flex_1().min_w_0().truncate().text_size(px(theme::FS_BODY)).child(name))
                .child(
                    div()
                        .id(("goal-at", id as usize))
                        .child(mono(ms.map_or("\u{2014}".to_string(), fmt_ms), theme::FS_SMALL).text_color(c(if early { theme::WARN } else { theme::FAINT })))
                        .when(early, |d| d.tooltip(tip("Out of order", Some("Your run gets here before the goal above it, so in this order it doesn't count. Drag it above that goal.")))),
                );
            if n + 1 == picked.len() {
                let both = self.spec.both.contains(&id);
                let mode = if speed { 2u8 } else if both { 1 } else { 0 };
                let judge = self.mini_choice(&format!("judge-{id}"), mode, vec![("Sooner", 0u8), ("Both", 1), ("Faster", 2)], move |e, v, _| {
                    e.spec.by_speed.retain(|x| *x != id);
                    e.spec.both.retain(|x| *x != id);
                    match v {
                        1 => e.spec.both.push(id),
                        2 => e.spec.by_speed.push(id),
                        _ => {}
                    }
                }, cx);
                row = row.child(div().id(("judge", id as usize)).child(judge).tooltip(tip("How this goal is judged", Some("Sooner: the earliest there wins. Faster: the fastest through it wins, even if later. Both: sooner, with each km/h more worth what it would save your run on the way to the next goal."))));
            }
            row = row.child(
                div()
                    .id(("goal-remove", id as usize))
                    .w(px(20.0))
                    .h(px(20.0))
                    .flex()
                    .items_center()
                    .justify_center()
                    .rounded(px(3.0))
                    .text_size(px(theme::FS_BODY))
                    .text_color(c(theme::FAINT))
                    .cursor_pointer()
                    .hover(|s| s.text_color(c(theme::TEXT)).bg(c(theme::RAISED)))
                    .child("\u{00d7}")
                    .tooltip(tip("Leave this goal out", None))
                    .on_click(cx.listener(move |this, _, _, cx| {
                        if this.spec.goals.is_empty() {
                            this.spec.goals = this.doc.goals.iter().map(|g| g.id).collect();
                        }
                        this.spec.goals.retain(|x| *x != id);
                        cx.notify();
                    })),
            );
            let line = when.map(|(w, all)| {
                div().id(("when", id as usize)).pl(px(20.0)).pb(px(4.0)).truncate().text_size(px(theme::FS_SMALL)).text_color(c(theme::WARN)).child(w).tooltip(tip("Counts only when", Some(&format!("{all}. Set in the goal's Properties."))))
            });
            col = col.child(
                div()
                    .id(("goal-row", id as usize))
                    .pl(px(theme::SP1))
                    .pr(px(2.0))
                    .rounded(px(theme::R_CTL))
                    .hover(|s| s.bg(c(theme::RAISED)))
                    .cursor_grab()
                    .child(row)
                    .children(line)
                    .on_drag(crate::ui::outliner::GoalDrag(id, name_for_drag.clone()), |d, _, _, cx| cx.new(|_| crate::ui::outliner::DragGhost(d.1.clone())))
                    .drag_over::<crate::ui::outliner::GoalDrag>(|s, _, _, _| s.border_t_2().border_color(c(theme::PIN)))
                    .on_drop(cx.listener(move |this, d: &crate::ui::outliner::GoalDrag, _, cx| {
                        let from = d.0;
                        if from != id {
                            if this.spec.goals.is_empty() {
                                this.spec.goals = this.spec.ordered_goals(&this.doc.goals).iter().map(|g| g.id).collect();
                            }
                            this.spec.goals.retain(|x| *x != from);
                            let at = this.spec.goals.iter().position(|x| *x == id).unwrap_or(this.spec.goals.len());
                            this.spec.goals.insert(at, from);
                        }
                        cx.notify();
                    })),
            );
        }
        let rest: Vec<(String, u32)> = self.doc.goals.iter().filter(|g| !picked.iter().any(|p| p.0 == g.id)).map(|g| (g.name.clone(), g.id)).collect();
        let add = self.menu_button("spec.add-goal", "+ Add goal", rest, |e, id, _| {
            if e.spec.goals.is_empty() {
                e.spec.goals = e.spec.ordered_goals(&e.doc.goals).iter().map(|g| g.id).collect();
            }
            let at = |id: u32| e.doc.goals.iter().position(|g| g.id == id).and_then(|i| e.reach.get(i)).and_then(|r| r.tick);
            let pos = at(id).and_then(|t| e.spec.goals.iter().position(|g| at(*g).is_some_and(|u| u > t))).unwrap_or(e.spec.goals.len());
            e.spec.goals.insert(pos, id);
        }, cx);
        col.child(div().flex().child(add)).into_any_element()
    }

    fn live_section(&mut self, cx: &mut Context<Self>) -> AnyElement {
        let (running, error, base, best, series, cands, until) = {
            let l = self.search.live.lock().unwrap();
            (l.running, l.error.clone(), l.base.clone(), l.best.clone(), l.series.clone(), l.candidates.clone(), l.until_tick)
        };
        let live = self.search.live.clone();
        let metric = move |s: &gearbox_search::objective::Score| live.lock().unwrap().metric(s);
        let mut v = div().flex().flex_col().child(div().mx(px(theme::PAD)).mt(px(theme::SP3)).pt(px(theme::SP2)).h(px(30.0)).border_t_1().border_color(c(theme::HAIRLINE)).flex().items_center().gap(px(theme::SP2)).text_size(px(theme::FS_EMPH)).font_weight(gpui::FontWeight::SEMIBOLD).child(div().w(px(10.0))).child(icon("icons/list.svg", theme::MUTED, 14.0)).child("Results"));
        if !running && cands.is_empty() && error.is_none() && base.is_none() {
            return v.child(div().px(px(theme::PAD)).child(muted("Nothing yet. Press Search (\u{2318}R)."))).into_any_element();
        }
        if let Some(e) = error {
            v = v.child(div().px(px(theme::PAD)).pb(px(theme::SP1)).text_size(px(theme::FS_BODY)).text_color(c(theme::ERROR)).child(e));
        }
        if let (Some(b), Some(bb)) = (&base, &best) {
            let (m0, m) = (metric(b), metric(bb));
            let better = bb.quality_cmp(b) == std::cmp::Ordering::Greater;
            v = v
                .child(prop("Best", div().text_size(px(theme::FS_BODY)).text_color(c(if better { theme::OK } else { theme::TEXT })).child(m.text)))
                .child(prop("Your run", div().text_size(px(theme::FS_BODY)).text_color(c(theme::MUTED)).child(if m0.reached { m0.text } else { "doesn't get there".into() })));
        }
        if series.len() > 1 {
            v = v.child(self.chart(series));
        }
        if cands.is_empty() {
            if running {
                v = v.child(div().px(px(theme::PAD)).pt(px(theme::SP1)).child(muted("No improvement yet.")));
            }
            return v.into_any_element();
        }
        let col = |s: &str, w: f32| div().w(px(w)).flex_none().flex().justify_end().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child(s.to_string());
        let head = div()
            .mx(px(theme::SP2))
            .px(px(theme::SP2))
            .h(px(22.0))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .child(div().w(px(34.0)).flex_none().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child("#"))
            .child(div().flex_1().min_w_0().text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child("at"))
            .child(col("time", 52.0))
            .child(col("vs yours", 58.0))
            .child(col("differs at", 64.0));
        let mut list = div().id("cands").max_h(px(360.0)).overflow_y_scroll().flex().flex_col().mt(px(theme::SP1)).child(head);
        let hidden = cands.len().saturating_sub(SHOWN_CANDIDATES);
        for cand in cands.iter().rev().take(SHOWN_CANDIDATES) {
            let id = cand.id;
            let sel = self.preview == Some(id);
            let diffs = self.candidate_diffs(id, until);
            let key = diffs.as_ref().and_then(|d| d.iter().rev().find(|g| g.this.is_some() && g.changed()).or_else(|| d.iter().find(|g| g.changed())).cloned());
            let lost = diffs.as_ref().map_or(0, |d| d.iter().filter(|g| g.yours.is_some() && g.this.is_none()).count());
            let (at, time, delta) = match &key {
                Some(g) => {
                    let time = g.this.map_or(("\u{2014}".to_string(), theme::FAINT), |(ms, _)| (fmt_ms(ms), theme::TEXT));
                    let delta = match (g.this, g.yours, g.miss) {
                        (Some((a, va)), Some((b, vb)), _) if a == b => (format!("{:+.1}", va - vb), if va > vb { theme::OK } else { theme::WARN }),
                        (Some((a, _)), Some((b, _)), _) => (fmt_delta(a - b), if a < b { theme::OK } else { theme::WARN }),
                        (Some(_), None, _) => ("new".into(), theme::OK),
                        (None, Some(_), _) => ("lost".into(), theme::ERROR),
                        (None, None, Some((a, b))) if a.is_finite() && b.is_finite() => (format!("{:+.1} m", a - b), if a < b { theme::OK } else { theme::WARN }),
                        _ => ("\u{2014}".into(), theme::FAINT),
                    };
                    (g.name.clone(), time, delta)
                }
                None if diffs.is_none() => ("\u{2026}".into(), ("".into(), theme::FAINT), ("".into(), theme::FAINT)),
                None => (metric(&cand.score).text, ("".into(), theme::FAINT), ("".into(), theme::FAINT)),
            };
            let first = self.first_change(id).map_or("\u{2014}".to_string(), |k| format!("{} s", fmt_ms(self.race_ms0 + 10 * k as i32)));
            list = list.child(
                div()
                    .id(("cand", id as usize))
                    .relative()
                    .child(track(format!("cand-{id}")))
                    .flex_none()
                    .mx(px(theme::SP2))
                    .px(px(theme::SP2))
                    .h(px(26.0))
                    .rounded(px(theme::R_CTL))
                    .cursor_pointer()
                    .flex()
                    .items_center()
                    .gap(px(theme::SP2))
                    .when(sel, |d| d.bg(c(theme::SELECT)))
                    .when(!sel, |d| d.hover(|s| s.bg(c(theme::RAISED))))
                    .child(mono(format!("#{id}"), theme::FS_SMALL).w(px(34.0)).flex_none().text_color(c(theme::GHOST)))
                    .child(div().flex_1().min_w_0().truncate().text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(at))
                    .when(lost > 0, |d| d.child(div().id(("lost", id as usize)).flex_none().child(icon("icons/clear.svg", theme::ERROR, 12.0)).tooltip(tip(format!("Loses {lost} of your goals"), None))))
                    .child(mono(time.0, theme::FS_SMALL).w(px(52.0)).flex_none().flex().justify_end().text_color(c(time.1)))
                    .child(mono(delta.0, theme::FS_SMALL).w(px(58.0)).flex_none().flex().justify_end().text_color(c(delta.1)))
                    .child(mono(first, theme::FS_SMALL).w(px(64.0)).flex_none().flex().justify_end().text_color(c(theme::MUTED)))
                    .on_click(cx.listener(move |this, _, _, cx| {
                        if this.sel == Sel::Candidate(id) {
                            this.preview = None;
                            this.sel = Sel::None;
                        } else {
                            this.select(Sel::Candidate(id));
                            this.layout.right_tab = 1;
                        }
                        this.dirty = true;
                        cx.notify();
                    })),
            );
            if sel {
                let table = diffs.unwrap_or_default();
                list = list.child(div().flex_none().mx(px(theme::SP2)).mb(px(theme::SP1)).px(px(theme::SP2)).py(px(theme::SP1)).rounded(px(theme::R_CTL)).bg(c(theme::WELL)).flex().flex_col().gap(px(2.0)).children(table.iter().map(|g| g.table_row())).child(self.candidate_actions(id, cx)));
            }
        }
        if hidden > 0 {
            list = list.child(div().flex_none().px(px(theme::PAD)).py(px(theme::SP1)).text_size(px(theme::FS_SMALL)).text_color(c(theme::FAINT)).child(format!("and {hidden} older")));
        }
        v = v.child(list);
        v.into_any_element()
    }

    fn candidate_diffs(&mut self, id: u64, until: usize) -> Option<Vec<GoalDiff>> {
        let key = (self.reach_gen, self.aimed_goals().iter().map(|g| g.id).collect::<Vec<_>>(), until);
        if self.diff_cache.key != key {
            self.diff_cache = DiffCache { key, ..Default::default() };
        }
        if let Some(d) = self.diff_cache.diffs.get(&id) {
            return Some(d.clone());
        }
        let d = self.candidate_diffs_now(id, until)?;
        self.diff_cache.diffs.insert(id, d.clone());
        Some(d)
    }

    fn candidate_diffs_now(&self, id: u64, until: usize) -> Option<Vec<GoalDiff>> {
        let judged = |f: &[crate::engine::frame::Frame]| f.len().min(until + 1);
        let f = self.runs_frames.get(&crate::aux::Key::Candidate(id))?;
        let f = &f[..judged(f)];
        let cur_frames = &self.sim.frames[..judged(&self.sim.frames)];
        let cur = self.aimed_reach(cur_frames);
        let aimed: Vec<u32> = self.aimed_goals().iter().map(|g| g.id).collect();
        let mut out: Vec<GoalDiff> = self
            .aimed_reach(f)
            .iter()
            .zip(&self.doc.goals)
            .enumerate()
            .filter(|(_, (_, g))| aimed.contains(&g.id))
            .map(|(i, (r, g))| {
                let y = cur.get(i).copied().unwrap_or_default();
                GoalDiff {
                    name: g.name.clone(),
                    yours: y.ms.map(|t| (t, y.speed_kmh.unwrap_or(0.0))),
                    this: r.ms.map(|t| (t, r.speed_kmh.unwrap_or(0.0))),
                    miss: (r.ms.is_none() && y.ms.is_none()).then(|| (r.closest.map_or(f32::NAN, |c| c.1), y.closest.map_or(f32::NAN, |c| c.1))),
                }
            })
            .collect();
        let fin = |f: &[crate::engine::frame::Frame]| f.iter().find(|x| x.finish.is_some()).map(|x| (x.finish.unwrap_or(0) as i32, x.speed_kmh()));
        let (a, b) = (fin(f), fin(cur_frames));
        let finish_aimed = self.aimed_goals().iter().any(|g| matches!(&g.kind, GoalKind::Gate(x) if x.kind == 1));
        if (a.is_some() || b.is_some()) && !finish_aimed {
            out.push(GoalDiff { name: "Finish".into(), yours: b, this: a, miss: None });
        }
        Some(out)
    }

    pub fn candidate_actions(&mut self, id: u64, cx: &mut Context<Self>) -> AnyElement {
        let check = self.checks.get(&id).cloned();
        let about = format!("It becomes the run (undoable; the old one goes to History). Verified: re-driven from scratch with identical score, body bits and trajectory{}.", check.map_or(String::new(), |m| format!(" \u{00b7} {m}")));
        div()
            .px(px(theme::SP2))
            .py(px(theme::SP1))
            .child(
                div()
                    .flex()
                    .gap(px(theme::SP1))
                    .child(btn_icon(format!("acc-{id}"), "icons/check.svg", "Use it", Btn::Primary).tooltip(tip("Use this improvement", Some(&about))).on_click(cx.listener(move |this, _, _, cx| {
                        this.accept(id);
                        cx.notify();
                    })))
                    .child(btn_icon(format!("goon-{id}"), "icons/goon.svg", "Use it and go on", Btn::Normal).tooltip(tip("Use it and go on", Some("It becomes the run, the range moves on by its own length, and the search goes on from there: one stretch (or one trick) at a time."))).on_click(cx.listener(move |this, _, _, cx| {
                        this.go_on(id);
                        cx.notify();
                    })))
                    .child(btn_icon(format!("keep-{id}"), "icons/keep.svg", "Keep as a run", Btn::Quiet).on_click(cx.listener(move |this, _, _, cx| {
                        this.keep_candidate(id);
                        cx.notify();
                    })))
                    .child(btn(format!("disc-{id}"), "Discard", Btn::Quiet).on_click(cx.listener(move |this, _, _, cx| {
                        this.discard_candidate(id);
                        cx.notify();
                    }))),
            )
            .into_any_element()
    }

    fn chart(&self, series: Vec<(usize, Metric)>) -> AnyElement {
        let unit = series.last().map(|s| s.1.unit).unwrap_or("");
        let lower = series.last().is_none_or(|s| s.1.lower);
        let pts: Vec<(f32, f32)> = series.iter().filter(|s| s.1.unit == unit && s.1.value.is_finite()).map(|s| (s.0 as f32, s.1.value as f32)).collect();
        let (lo, hi) = pts.iter().fold((f32::MAX, f32::MIN), |a, p| (a.0.min(p.1), a.1.max(p.1)));
        let xmax = series.last().map_or(1.0, |s| s.0 as f32).max(1.0);
        div()
            .mx(px(theme::PAD))
            .my(px(theme::SP1))
            .child(
                div().h(px(48.0)).rounded(px(theme::R_CTL)).bg(c(theme::WELL)).child(
                    canvas(
                        |_, _, _| (),
                        move |b: Bounds<Pixels>, _, window: &mut Window, _| {
                            let (x0, y0, w, h) = (f32::from(b.origin.x) + 6.0, f32::from(b.origin.y) + 6.0, f32::from(b.size.width) - 12.0, f32::from(b.size.height) - 12.0);
                            for i in 0..=3 {
                                let y = y0 + h * i as f32 / 3.0;
                                window.paint_quad(fill(Bounds { origin: point(px(x0), px(y)), size: size(px(w), px(1.0)) }, theme::alpha(theme::HAIRLINE, 0.6)));
                            }
                            if pts.len() < 1 {
                                return;
                            }
                            let span = (hi - lo).max(1e-6);
                            let yv = |v: f32| if lower { y0 + (v - lo) / span * h } else { y0 + h - (v - lo) / span * h };
                            let xv = |a: f32| x0 + a / xmax * w;
                            let mut path = PathBuilder::stroke(px(1.5));
                            path.move_to(point(px(xv(0.0)), px(yv(pts[0].1))));
                            let mut last = pts[0].1;
                            for &(a, v) in &pts {
                                path.line_to(point(px(xv(a)), px(yv(last))));
                                path.line_to(point(px(xv(a)), px(yv(v))));
                                last = v;
                            }
                            path.line_to(point(px(x0 + w), px(yv(last))));
                            if let Ok(p) = path.build() {
                                window.paint_path(p, theme::alpha(theme::PIN, 1.0));
                            }
                            for &(a, v) in &pts {
                                window.paint_quad(fill(Bounds { origin: point(px(xv(a) - 1.5), px(yv(v) - 1.5)), size: size(px(3.0), px(3.0)) }, theme::alpha(theme::TEXT, 0.8)));
                            }
                        },
                    )
                    .size_full(),
                ),
            )
            .into_any_element()
    }
}
