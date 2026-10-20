use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use glam::{Vec2, Vec3};
use gpui::{
    Context, KeyDownEvent, KeyUpEvent, Keystroke, Modifiers, MouseButton, MouseDownEvent, MouseMoveEvent,
    MouseUpEvent, PinchEvent, PlatformInput, ScrollDelta, ScrollWheelEvent, TouchPhase, Window, point, px,
};

use crate::app::{Editor, Sel, Tool};
use crate::doc::GoalKind;
use crate::ui::widgets::HITS;

pub enum Flow {
    Next,
    Wait,
    Fail(String),
}

type StepFn = Box<dyn FnMut(&mut Editor, &mut Ctl) -> Flow>;

pub struct Ctl {
    pub out: Vec<PlatformInput>,
    pub n: u32,
    pub log: Vec<String>,
    pub fails: Vec<String>,
    pub dir: PathBuf,
    pub f: HashMap<&'static str, f64>,
    pub v: HashMap<&'static str, Vec3>,
    pub shots: Vec<PathBuf>,
    pub perf: Vec<String>,
    pub quit_ok: bool,
}

fn road_ahead(e: &Editor) -> Option<Vec2> {
    let f = e.sim.frames.get(e.cursor)?;
    let (o, size) = (crate::app::v2(e.viewport.origin), e.vsize());
    let clear = |s: &Vec2| s.x > o.x + 80.0 && s.y < o.y + size.y - 60.0;
    [12.0, 9.0, 6.0, 4.0]
        .into_iter()
        .find_map(|d| e.wpt(Vec3::from(f.position()) + Vec3::from(f.axis(2)) * d).filter(clear))
}

fn newest(e: &Editor) -> Option<&crate::doc::Goal> {
    e.doc.goals.iter().max_by_key(|g| g.id)
}

fn gate_goal(e: &Editor) -> Option<(usize, &crate::doc::Goal)> {
    e.doc.goals.iter().enumerate().find(|(_, g)| matches!(g.kind, GoalKind::Gate(_)))
}

fn secs() -> f64 {
    static T0: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    T0.get_or_init(Instant::now).elapsed().as_secs_f64()
}

fn at(p: Vec2) -> gpui::Point<gpui::Pixels> {
    point(px(p.x), px(p.y))
}

fn ks(s: &str) -> Keystroke {
    let mut k = Keystroke::parse(s).unwrap_or_else(|_| panic!("keystroke {s}"));
    if k.modifiers == Modifiers::default() && k.key.chars().count() == 1 {
        k.key_char = Some(k.key.clone());
    }
    k
}

impl Ctl {
    pub fn say(&mut self, s: impl Into<String>) {
        let s = s.into();
        eprintln!("[script] {s}");
        self.log.push(s);
    }
    pub fn key(&mut self, s: &str) {
        self.key_down(s);
        self.key_up(s);
    }
    pub fn key_down(&mut self, s: &str) {
        self.out.push(PlatformInput::KeyDown(KeyDownEvent {
            keystroke: ks(s),
            is_held: false,
            prefer_character_input: false,
        }));
    }
    pub fn key_up(&mut self, s: &str) {
        self.out.push(PlatformInput::KeyUp(KeyUpEvent { keystroke: ks(s) }));
    }
    pub fn typ(&mut self, text: &str) {
        for c in text.chars() {
            let k = Keystroke {
                modifiers: Modifiers::default(),
                key: c.to_string(),
                key_char: Some(c.to_string()),
            };
            self.out.push(PlatformInput::KeyDown(KeyDownEvent {
                keystroke: k.clone(),
                is_held: false,
                prefer_character_input: false,
            }));
            self.out.push(PlatformInput::KeyUp(KeyUpEvent { keystroke: k }));
        }
    }
    pub fn move_to(&mut self, p: Vec2, pressed: Option<MouseButton>, modifiers: Modifiers) {
        self.out.push(PlatformInput::MouseMove(MouseMoveEvent {
            position: at(p),
            pressed_button: pressed,
            modifiers,
        }));
    }
    pub fn down(&mut self, p: Vec2, button: MouseButton, modifiers: Modifiers, count: usize) {
        self.out.push(PlatformInput::MouseDown(MouseDownEvent {
            button,
            position: at(p),
            modifiers,
            click_count: count,
            first_mouse: false,
        }));
    }
    pub fn up(&mut self, p: Vec2, button: MouseButton, modifiers: Modifiers, count: usize) {
        self.out.push(PlatformInput::MouseUp(MouseUpEvent {
            button,
            position: at(p),
            modifiers,
            click_count: count,
        }));
    }
    pub fn click(&mut self, p: Vec2) {
        self.move_to(p, None, Modifiers::default());
        self.down(p, MouseButton::Left, Modifiers::default(), 1);
        self.up(p, MouseButton::Left, Modifiers::default(), 1);
    }
    pub fn drag(&mut self, path: &[Vec2], button: MouseButton, modifiers: Modifiers) {
        self.move_to(path[0], None, modifiers);
        self.down(path[0], button, modifiers, 1);
        for p in &path[1..] {
            self.move_to(*p, Some(button), modifiers);
        }
        self.up(*path.last().unwrap(), button, modifiers, 1);
    }
    pub fn scroll_px(&mut self, p: Vec2, d: Vec2, modifiers: Modifiers, phase: TouchPhase) {
        self.out.push(PlatformInput::ScrollWheel(ScrollWheelEvent {
            position: at(p),
            delta: ScrollDelta::Pixels(point(px(d.x), px(d.y))),
            modifiers,
            touch_phase: phase,
        }));
    }
    pub fn scroll_lines(&mut self, p: Vec2, dy: f32, modifiers: Modifiers) {
        self.out.push(PlatformInput::ScrollWheel(ScrollWheelEvent {
            position: at(p),
            delta: ScrollDelta::Lines(point(0.0, dy)),
            modifiers,
            touch_phase: TouchPhase::Moved,
        }));
    }
    pub fn pinch(&mut self, p: Vec2, delta: f32) {
        self.out.push(PlatformInput::Pinch(PinchEvent {
            position: at(p),
            delta,
            modifiers: Modifiers::default(),
            phase: TouchPhase::Moved,
        }));
    }
    pub fn dbl_click_id(&mut self, id: &str) -> bool {
        let Some(p) = hit(id) else { return false };
        self.move_to(p, None, Modifiers::default());
        for n in 1..=2 {
            self.down(p, MouseButton::Left, Modifiers::default(), n);
            self.up(p, MouseButton::Left, Modifiers::default(), n);
        }
        true
    }
    pub fn click_id(&mut self, id: &str) -> bool {
        match hit(id) {
            Some(c) => {
                self.click(c);
                true
            }
            None => false,
        }
    }
    pub fn shot(&mut self, e: &Editor, name: &str) {
        let p = self.dir.join(format!("{name}.png"));
        if e.screenshot(&p) {
            self.shots.push(std::fs::canonicalize(&p).unwrap_or(p));
        } else {
            self.fails.push(format!("screenshot {name} failed"));
        }
    }
    pub fn rendered(&mut self, e: &Editor, name: &str) -> f32 {
        let p = self.dir.join(format!("{name}.jpg"));
        if !e.screenshot(&p) {
            self.fails.push(format!("screenshot {name} failed"));
            return 0.0;
        }
        self.shots.push(std::fs::canonicalize(&p).unwrap_or(p.clone()));
        let Ok(img) = image::open(&p).map(|i| i.to_luma8()) else { return 0.0 };
        let (o, size) = (crate::app::v2(e.viewport.origin), e.vsize());
        let k = e.scale;
        let (x0, y0) = (((o.x + size.x * 0.25) * k) as u32, ((o.y + size.y * 0.25) * k) as u32);
        let (x1, y1) = (
            (((o.x + size.x * 0.75) * k) as u32).min(img.width()),
            (((o.y + size.y * 0.75) * k) as u32).min(img.height()),
        );
        let px: Vec<f32> = (y0..y1)
            .flat_map(|y| (x0..x1).map(move |x| (x, y)))
            .map(|(x, y)| img.get_pixel(x, y).0[0] as f32)
            .collect();
        let n = px.len().max(1) as f32;
        let mean = px.iter().sum::<f32>() / n;
        (px.iter().map(|v| (v - mean) * (v - mean)).sum::<f32>() / n).sqrt()
    }
    pub fn check(&mut self, ok: bool, what: impl Into<String>) {
        let w = what.into();
        if ok {
            self.say(format!("  \u{2713} {w}"));
        } else {
            self.say(format!("  \u{2717} {w}"));
            self.fails.push(w);
        }
    }
}

pub fn hit(id: &str) -> Option<Vec2> {
    HITS.with(|h| h.borrow().get(id).copied()).map(|b| {
        Vec2::new(f32::from(b.origin.x + b.size.width * 0.5), f32::from(b.origin.y + b.size.height * 0.5))
    })
}

const SHIFT: Modifiers =
    Modifiers { shift: true, control: false, alt: false, platform: false, function: false };
const ALT: Modifiers =
    Modifiers { shift: false, control: false, alt: true, platform: false, function: false };
const NONE: Modifiers =
    Modifiers { shift: false, control: false, alt: false, platform: false, function: false };

pub struct Script {
    pub dir: PathBuf,
    pub frames: u32,
    steps: Vec<(&'static str, StepFn)>,
    i: usize,
    wait: u32,
    since: Instant,
    began: Instant,
    pub ctl: Ctl,
    record: Option<(PathBuf, Option<std::process::Child>)>,
    timer_frames: u32,
    done: bool,
}

impl Script {
    pub fn new(dir: PathBuf) -> Self {
        let _ = std::fs::create_dir_all(&dir);
        let ctl = Ctl {
            out: Vec::new(),
            n: 0,
            log: Vec::new(),
            fails: Vec::new(),
            dir: dir.clone(),
            f: HashMap::new(),
            v: HashMap::new(),
            shots: Vec::new(),
            perf: Vec::new(),
            quit_ok: false,
        };
        Self {
            dir,
            frames: 0,
            steps: Vec::new(),
            i: 0,
            wait: 0,
            since: Instant::now(),
            began: Instant::now(),
            ctl,
            record: None,
            timer_frames: 0,
            done: false,
        }
    }

    fn add(&mut self, name: &'static str, f: impl FnMut(&mut Editor, &mut Ctl) -> Flow + 'static) {
        self.steps.push((name, Box::new(f)));
    }
}

impl Editor {
    pub fn ready(&self) -> bool {
        self.world_ready
            && self.sim.done()
            && (self.show.physics || self.visual_state.is_none())
            && self.loading.is_none()
    }

    pub fn screenshot(&self, path: &Path) -> bool {
        let Some(id) = self.window_id else { return false };
        let _ = std::fs::create_dir_all(path.parent().unwrap_or(Path::new(".")));
        let kind = if path.extension().is_some_and(|x| x == "jpg") { "-tjpg" } else { "-tpng" };
        let ok = std::process::Command::new("screencapture")
            .arg("-x")
            .arg("-o")
            .arg(kind)
            .arg(format!("-l{id}"))
            .arg(path)
            .status()
            .is_ok_and(|s| s.success());
        eprintln!("screenshot {} {}", path.display(), if ok { "saved" } else { "FAILED" });
        ok
    }

    pub fn wpt(&self, p: Vec3) -> Option<Vec2> {
        let s = crate::camera::project(&self.vp, self.vsize(), p)?;
        let size = self.vsize();
        (s.x > 4.0 && s.y > 40.0 && s.x < size.x - 4.0 && s.y < size.y - 4.0)
            .then(|| s + crate::app::v2(self.viewport.origin))
    }

    pub fn vcenter(&self) -> Vec2 {
        crate::app::v2(self.viewport.origin) + self.vsize() * 0.5
    }

    pub fn tick_x(&self, k: usize) -> f32 {
        f32::from(self.tl.bounds.origin.x)
            + crate::ui::dopesheet::GUTTER
            + ((k as f64 - self.tl.start) * self.tl.ppt) as f32
            + 0.5
    }

    pub fn ruler_y(&self) -> f32 {
        f32::from(self.tl.bounds.origin.y) + 9.0
    }

    fn settled(&self) -> bool {
        self.camera.goal == self.camera.cur
    }

    pub fn automate(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        self.script_ran = Instant::now();
        if let Some(path) = self.opts.shot.clone()
            && self.script.as_ref().is_none_or(|s| s.steps.is_empty())
        {
            let cards_loading = self.projects_open && self.pm.cards.len() < self.pm.cards_total;
            if (!self.ready() && self.open_error.is_none() && !self.projects_open) || cards_loading {
                if self.script_in_frame {
                    window.request_animation_frame();
                }
                return;
            }
            let s = self.script.get_or_insert_with(|| Script::new(PathBuf::from("target/shots")));
            s.frames += 1;
            if self.script_in_frame {
                window.request_animation_frame();
            }
            if s.frames == 40 {
                self.screenshot(&path);
                self.opts.shot = None;
                if self.opts.exit {
                    cx.quit();
                }
            }
            return;
        }
        let Some(mut s) = self.script.take() else { return };
        if s.steps.is_empty() && s.i == 0 {
            if self.opts.record.is_some() {
                build_record(&mut s, self.opts.record.clone().unwrap());
            } else if self.opts.smoke {
                build_smoke(&mut s);
            } else {
                build_e2e(&mut s);
            }
            s.since = Instant::now();
            s.began = Instant::now();
        }
        if self.script_in_frame {
            window.request_animation_frame();
        }
        if s.wait > 0 {
            s.wait -= 1;
            self.script = Some(s);
            return;
        }
        if s.i >= s.steps.len() {
            self.finish_script(&mut s);
            self.script = Some(s);
            cx.quit();
            return;
        }
        let (name, mut f) = std::mem::replace(&mut s.steps[s.i], ("", Box::new(|_, _| Flow::Next)));
        s.ctl.n += 1;
        s.timer_frames += !self.script_in_frame as u32;
        let flow = f(self, &mut s.ctl);
        s.steps[s.i] = (name, f);
        cx.notify();
        let elapsed = s.since.elapsed();
        let advance = match flow {
            Flow::Next => {
                let timer = if s.timer_frames * 10 > s.ctl.n {
                    format!(" ({} of {} frames on the timer: the window drew none)", s.timer_frames, s.ctl.n)
                } else {
                    String::new()
                };
                s.ctl.say(format!(
                    "step {} \u{00b7} {name} \u{00b7} {:.2} s{timer}",
                    s.i + 1,
                    elapsed.as_secs_f32()
                ));
                true
            }
            Flow::Wait if elapsed > Duration::from_secs(90) => {
                s.ctl.fails.push(format!("{name}: timed out"));
                s.ctl.say(format!("step {} \u{00b7} {name} \u{00b7} TIMED OUT", s.i + 1));
                true
            }
            Flow::Wait => {
                if s.ctl.n % 600 == 0 {
                    eprintln!(
                        "[script] waiting in {name}: world {} sim done {} ({} frames / {} ticks, busy {:?}) visual {:?} loading {:?} gates {} origin {}",
                        self.world_ready,
                        self.sim.done(),
                        self.sim.frames.len(),
                        self.doc.len,
                        self.sim.busy,
                        self.visual_state,
                        self.loading,
                        self.gates.len(),
                        self.origin.is_some()
                    );
                }
                false
            }
            Flow::Fail(m) => {
                s.ctl.fails.push(format!("{name}: {m}"));
                s.ctl.say(format!("step {} \u{00b7} {name} \u{00b7} FAILED: {m}", s.i + 1));
                let shot = s.ctl.dir.join(format!("failed-{:02}.png", s.i + 1));
                if self.screenshot(&shot) {
                    s.ctl.shots.push(shot);
                }
                true
            }
        };
        if advance {
            s.i += 1;
            s.ctl.n = 0;
            s.timer_frames = 0;
            s.wait = 4;
            s.since = Instant::now();
        }
        let events = std::mem::take(&mut s.ctl.out);
        if let Some((_, child)) = &mut s.record
            && s.i >= s.steps.len()
            && let Some(c) = child
        {
            let _ = c.wait();
        }
        self.script = Some(s);
        if !events.is_empty() {
            cx.spawn_in(window, async move |_, cx| {
                let _ = cx.update(|window, cx| {
                    for e in events {
                        window.dispatch_event(e, cx);
                    }
                });
            })
            .detach();
        }
    }

    pub fn quit_script(&mut self) {
        let Some(mut s) = self.script.take() else { return };
        if !s.done && !s.steps.is_empty() {
            if let Some((name, _)) = s.steps.get(s.i) {
                let name = *name;
                if s.ctl.quit_ok {
                    s.ctl.say(format!(
                        "step {} \u{00b7} {name} \u{00b7} {:.2} s",
                        s.i + 1,
                        s.since.elapsed().as_secs_f32()
                    ));
                    s.i += 1;
                } else {
                    s.ctl.fails.push(format!("the app quit during step {}: {name}", s.i + 1));
                }
            }
            self.finish_script(&mut s);
        }
        self.script = Some(s);
    }

    fn finish_script(&mut self, s: &mut Script) {
        s.done = true;
        let mut r = String::new();
        r += &format!(
            "# editor scripted run \u{00b7} {:.1} s \u{00b7} {} of {} steps\n\n",
            s.began.elapsed().as_secs_f32(),
            s.i.min(s.steps.len()),
            s.steps.len()
        );
        r += &format!("{} checks failed\n\n", s.ctl.fails.len());
        for f in &s.ctl.fails {
            r += &format!("FAIL {f}\n");
        }
        r += "\n## Log\n";
        for l in &s.ctl.log {
            r += &format!("{l}\n");
        }
        r += "\n## Frame times\n";
        for p in &s.ctl.perf {
            r += &format!("{p}\n");
        }
        r += "\n## Screenshots\n";
        for p in &s.ctl.shots {
            r += &format!("{}\n", p.display());
        }
        r += "\n## Console (errors and warnings)\n";
        for l in self.log.iter().filter(|l| l.level != crate::app::LogLevel::Info) {
            r += &format!("{} [{}] {}\n", l.at, l.source, l.text);
        }
        let path = if s.record.is_some() {
            self.opts.record.clone().unwrap().with_extension("txt")
        } else {
            s.dir.join("report.txt")
        };
        let _ = std::fs::write(&path, &r);
        eprintln!("{r}");
        eprintln!("report: {}", path.display());
    }
}

fn perf_start(e: &mut Editor) {
    e.stats.clear();
}

fn perf_end(e: &Editor, c: &mut Ctl, label: &str) {
    let s = &e.stats;
    let line = format!(
        "{label}: frame interval p50 {:.2} ms, p95 {:.2} ms, p99 {:.2} ms, max {:.2} ms ({} frames, {:.0} fps median) \u{00b7} UI CPU p50 {:.2} ms p99 {:.2} ms \u{00b7} 3D GPU p50 {:.2} ms \u{00b7} input\u{2192}frame p50 {:.1} ms",
        s.interval.pct(0.5),
        s.interval.pct(0.95),
        s.interval.pct(0.99),
        s.interval.pct(1.0),
        s.interval.len(),
        1000.0 / s.interval.pct(0.5).max(0.01),
        s.ui_frame.pct(0.5),
        s.ui_frame.pct(0.99),
        s.gpu_scene.pct(0.5),
        s.input_latency.pct(0.5)
    );
    c.say(line.clone());
    c.perf.push(line);
}

fn build_e2e(s: &mut Script) {
    s.add("open a map blank", |e, c| {
        if !(e.ready() && !e.gates.is_empty() && e.origin.is_some()) {
            return Flow::Wait;
        }
        c.check(e.doc.inputs.controls.is_empty(), "blank run: no inputs");
        c.check(
            e.sim.frames.len() == e.doc.len + 1,
            format!("simulated every tick ({} frames for {} ticks)", e.sim.frames.len(), e.doc.len),
        );
        c.say(format!("  race time at tick 0: {} ms; {} gates", e.race_ms0, e.gates.len()));
        c.shot(e, "01-open-blank");
        Flow::Next
    });
    s.add("frame the track (Home)", |e, c| {
        if c.n == 1 {
            c.key("home");
            return Flow::Wait;
        }
        if !e.settled() || c.n < 5 {
            return Flow::Wait;
        }
        c.shot(e, "02-camera-frame");
        Flow::Next
    });
    s.add("orbit (Alt+LMB drag)", |e, c| {
        if c.n == 1 {
            c.f.insert("yaw", e.camera.goal.yaw as f64);
            c.f.insert("pitch", e.camera.goal.pitch as f64);
            let m = e.vcenter();
            let path: Vec<Vec2> = (0..=20).map(|i| m + Vec2::new(i as f32 * 12.0, i as f32 * -3.0)).collect();
            c.drag(&path, MouseButton::Left, ALT);
            return Flow::Wait;
        }
        if !e.settled() {
            return Flow::Wait;
        }
        let dy = e.camera.goal.yaw as f64 - c.f["yaw"];
        let dp = e.camera.goal.pitch as f64 - c.f["pitch"];
        c.check(
            (dy + 240.0 * 0.0055).abs() < 1e-3,
            format!("orbit: yaw {dy:+.4} rad for 240 pt right (expected \u{2212}1.32)"),
        );
        c.check((dp + 60.0 * 0.0055).abs() < 1e-3, format!("orbit: pitch {dp:+.4} rad for 60 pt up"));
        c.shot(e, "03-camera-orbit");
        Flow::Next
    });
    s.add("orbit (MMB drag)", |e, c| {
        if c.n == 1 {
            c.f.insert("yaw", e.camera.goal.yaw as f64);
            let m = e.vcenter();
            let path: Vec<Vec2> = (0..=10).map(|i| m - Vec2::new(i as f32 * 8.0, 0.0)).collect();
            c.drag(&path, MouseButton::Middle, NONE);
            return Flow::Wait;
        }
        if !e.settled() {
            return Flow::Wait;
        }
        let dy = e.camera.goal.yaw as f64 - c.f["yaw"];
        c.check((dy - 80.0 * 0.0055).abs() < 1e-3, format!("MMB orbit: yaw {dy:+.4} rad"));
        Flow::Next
    });
    s.add("pan (two-finger trackpad, Shift+MMB)", |e, c| {
        if c.n == 1 {
            c.v.insert("pivot", e.camera.goal.pivot);
            let m = e.vcenter();
            c.scroll_px(m, Vec2::ZERO, NONE, TouchPhase::Started);
            for _ in 0..12 {
                c.scroll_px(m, Vec2::new(-14.0, 6.0), NONE, TouchPhase::Moved);
            }
            c.scroll_px(m, Vec2::ZERO, NONE, TouchPhase::Ended);
            return Flow::Wait;
        }
        if c.n == 2 {
            let m = e.vcenter();
            c.v.insert("pivot2", e.camera.goal.pivot);
            let path: Vec<Vec2> = (0..=10).map(|i| m + Vec2::new(0.0, i as f32 * 6.0)).collect();
            c.drag(&path, MouseButton::Middle, SHIFT);
            return Flow::Wait;
        }
        if !e.settled() {
            return Flow::Wait;
        }
        let d1 = c.v["pivot2"].distance(c.v["pivot"]);
        let d2 = e.camera.goal.pivot.distance(c.v["pivot2"]);
        c.check(d1 > 1.0, format!("two-finger pan moved the pivot {d1:.1} m (no rotation)"));
        c.check(d2 > 1.0, format!("Shift+MMB pan moved the pivot {d2:.1} m"));
        c.shot(e, "04-camera-pan");
        Flow::Next
    });
    s.add("dolly to the cursor (wheel, pinch)", |e, c| {
        let Some(p0) = e.sim.frames.first().map(|f| Vec3::from(f.position())) else { return Flow::Wait };
        if c.n == 1 {
            let m = e.wpt(p0).unwrap_or(e.vcenter());
            c.v.insert("m", m.extend(0.0));
            c.move_to(m, None, NONE);
            return Flow::Wait;
        }
        let m = c.v["m"].truncate();
        if c.n == 2 {
            return Flow::Wait;
        }
        if c.n < 30 && e.hover_hit.is_none() {
            return Flow::Wait;
        }
        if !c.f.contains_key("d0") {
            let hp = e.hover_hit.map_or(p0, |h| h.1);
            c.v.insert("hit", hp);
            c.f.insert("d0", e.camera.goal.distance as f64);
            for _ in 0..4 {
                c.scroll_lines(m, 2.0, NONE);
            }
            return Flow::Wait;
        }
        if !c.f.contains_key("d1") {
            if !e.settled() {
                return Flow::Wait;
            }
            c.f.insert("d1", e.camera.goal.distance as f64);
            c.pinch(m, 0.25);
            return Flow::Wait;
        }
        if !e.settled() {
            return Flow::Wait;
        }
        let (d0, d1, d2) = (c.f["d0"], c.f["d1"], e.camera.goal.distance as f64);
        c.check(d1 < d0 * 0.6, format!("wheel dolly: {d0:.1} m \u{2192} {d1:.1} m"));
        c.check(d2 < d1, format!("pinch dolly: {d1:.1} m \u{2192} {d2:.1} m"));
        let stay = e.wpt(c.v["hit"]).map(|s| s.distance(m));
        c.check(
            stay.is_some_and(|d| d < 3.0),
            format!("the point under the cursor stays under it ({:.2} pt off)", stay.unwrap_or(f32::NAN)),
        );
        c.shot(e, "05-camera-dolly");
        Flow::Next
    });
    s.add("fly (hold RMB + W, E; wheel speed)", |e, c| {
        let m = e.vcenter();
        let t = secs() - c.f.get("t0").copied().unwrap_or(0.0);
        match c.n {
            1 => {
                c.f.insert("t0", secs());
                c.v.insert("eye", e.camera.eye());
                c.move_to(m, None, NONE);
                c.down(m, MouseButton::Right, NONE, 1);
                c.scroll_lines(m, -3.0, NONE);
                c.key_down("w");
                perf_start(e);
                Flow::Wait
            }
            _ if t < 1.5 => {
                c.move_to(m + Vec2::new((t * 90.0) as f32, 0.0), Some(MouseButton::Right), NONE);
                Flow::Wait
            }
            _ if !c.f.contains_key("up") => {
                c.f.insert("up", 1.0);
                c.key_up("w");
                c.key_down("e");
                Flow::Wait
            }
            _ if t < 1.8 => Flow::Wait,
            _ => {
                c.key_up("e");
                c.up(m + Vec2::new(90.0, 0.0), MouseButton::Right, NONE, 1);
                let moved = e.camera.eye().distance(c.v["eye"]);
                c.check(
                    moved > 5.0,
                    format!("fly moved the eye {moved:.1} m at {:.0} m/s", e.camera.fly_speed),
                );
                perf_end(e, c, "fly (RMB held, WASD, mouse look)");
                c.shot(e, "06-camera-fly");
                Flow::Next
            }
        }
    });
    s.add("views: top, bookmark, recall, focus", |e, c| match c.n {
        1 => {
            c.key("alt-1");
            Flow::Wait
        }
        2..=300 if !e.settled() || c.n < 10 => Flow::Wait,
        2..=300 => {
            c.check(e.camera.goal.pitch > 1.5, format!("top view: pitch {:.3} rad", e.camera.goal.pitch));
            c.shot(e, "07-camera-top");
            c.key("ctrl-shift-1");
            c.f.insert("bm", e.camera.goal.yaw as f64);
            c.n = 301;
            Flow::Wait
        }
        302 => {
            c.key("alt-3");
            Flow::Wait
        }
        303..=600 if !e.settled() || c.n < 312 => Flow::Wait,
        303..=600 => {
            c.key("ctrl-1");
            c.n = 601;
            Flow::Wait
        }
        _ if !e.settled() || c.n < 610 => Flow::Wait,
        _ => {
            c.check(
                e.doc.bookmarks.len() == 1 && e.camera.goal.pitch > 1.5,
                format!(
                    "camera bookmark saved and recalled ({} saved, pitch {:.2})",
                    e.doc.bookmarks.len(),
                    e.camera.goal.pitch
                ),
            );
            c.key("f");
            Flow::Next
        }
    });
    s.add("import a replay's inputs (\u{2318}I)", |e, c| {
        if c.n == 1 {
            c.key("cmd-i");
            return Flow::Wait;
        }
        if c.n == 4 {
            if !c.click_id("menu-0") {
                return Flow::Fail("no replay in the import menu".into());
            }
            return Flow::Wait;
        }
        if c.n < 8 || e.doc.inputs.controls.is_empty() || !e.ready() {
            return Flow::Wait;
        }
        let fin = e.sim.frames.iter().find_map(|f| f.finish);
        c.check(
            fin.is_some(),
            format!(
                "the imported run finishes: {}",
                fin.map_or("no".into(), |t| crate::fmt::fmt_ms(t as i32))
            ),
        );
        c.check(matches!(e.doc.run_origin, crate::doc::Origin::Replay(_)), "run origin: replay");
        c.check(e.doc.history.len() == 1, "the blank run is in the history");
        Flow::Next
    });
    s.add("add the replay's ghost (\u{2318}\u{21e7}I)", |e, c| {
        if c.n == 1 {
            c.key("cmd-shift-i");
            return Flow::Wait;
        }
        if c.n == 4 {
            c.click_id("menu-0");
            return Flow::Wait;
        }
        if e.ghosts.is_empty() {
            return Flow::Wait;
        }
        let g = &e.ghosts[0].track;
        c.check(g.samples.len() > 100, format!("ghost {}: {} samples", g.name, g.samples.len()));
        Flow::Next
    });
    s.add("follow the car while playing (\u{21e7}F, Space)", |e, c| match c.n {
        1 => {
            c.key("home");
            Flow::Wait
        }
        2..=30 => Flow::Wait,
        31 => {
            c.key("shift-f");
            c.key("space");
            perf_start(e);
            c.f.insert("t0", secs());
            c.f.insert("from", e.cursor as f64);
            Flow::Wait
        }
        _ if secs() - c.f["t0"] < 1.5 => {
            if secs() - c.f["t0"] > 1.0 && !c.f.contains_key("shot") {
                c.f.insert("shot", 1.0);
                c.shot(e, "08-follow-playback-ghost");
            }
            Flow::Wait
        }
        _ => {
            perf_end(e, c, "follow camera, playback, realistic view");
            let (moved, t) = (e.cursor as f64 - c.f["from"], secs() - c.f["t0"]);
            c.check(
                (moved - t * 100.0).abs() < t * 20.0 + 10.0,
                format!("playback ran at 1\u{00d7}: {moved} ticks in {t:.2} s"),
            );
            c.key("space");
            c.key("shift-f");
            Flow::Next
        }
    });
    s.add("gate goal: click the first checkpoint", |e, c| {
        let cp = e
            .sim
            .frames
            .windows(2)
            .find(|w| w[1].waypoints > w[0].waypoints)
            .and_then(|w| w[1].last_crossing)
            .map(|x| x.0);
        let Some(w) = cp else { return Flow::Fail("the run crosses no checkpoint".into()) };
        match c.n {
            1 => {
                e.collapsed.remove("world");
                Flow::Wait
            }
            2..=10 => {
                if c.click_id(&format!("gate-{w}")) {
                    c.key("f");
                    c.key("1");
                    c.n = 10;
                } else if c.n == 10 {
                    return Flow::Fail(format!("no gate-{w} row in the Outliner"));
                }
                Flow::Wait
            }
            11..=30 => Flow::Wait,
            31 if !e.settled() => {
                c.n = 30;
                Flow::Wait
            }
            31 => {
                let g = e.gates.iter().find(|g| g.gate.waypoint == w).unwrap();
                let b = &g.gate.boxes[0];
                let center = (Vec3::from(b.min) + Vec3::from(b.max)) * 0.5;
                let Some(p) = e.wpt(center) else { return Flow::Fail("gate off screen".into()) };
                c.move_to(p, None, NONE);
                c.click(p);
                Flow::Wait
            }
            _ if e.doc.goals.is_empty() => Flow::Wait,
            _ if !c.f.contains_key("placed") => {
                c.f.insert("placed", c.n as f64);
                let Some((i, g)) = gate_goal(e) else {
                    return Flow::Fail("the click added no gate goal".into());
                };
                c.check(true, format!("gate goal {} added", g.name));
                let ms = e.reach.get(i).and_then(|r| r.ms);
                c.check(
                    ms.is_some(),
                    format!("the run reaches it at {}", ms.map_or("-".into(), crate::fmt::fmt_ms)),
                );
                c.shot(e, "09-goal-gate");
                Flow::Wait
            }
            _ if !e.search.running() && c.n < c.f["placed"] as u32 + 60 => Flow::Wait,
            _ if !c.f.contains_key("auto") => {
                c.f.insert("auto", 1.0);
                c.check(
                    e.search.running() && e.job.starts_with("Search to"),
                    format!("placing the goal starts a search towards it ({})", e.job),
                );
                c.f.insert("live", std::sync::Arc::as_ptr(&e.search.live) as usize as f64);
                c.f.insert("edited", c.n as f64);
                e.edit("goal speed", None, |d| d.goals[0].when.min_kmh = Some(1.0));
                Flow::Wait
            }
            _ if !c.f.contains_key("followed") => {
                let again = std::sync::Arc::as_ptr(&e.search.live) as usize as f64 != c.f["live"];
                if !again && c.n < c.f["edited"] as u32 + 200 {
                    return Flow::Wait;
                }
                c.f.insert("followed", 1.0);
                c.check(again && e.search.running(), "changing the goal it aims at searches again");
                c.check(
                    e.aimed_goals().iter().map(|g| g.id).eq([e.doc.goals[0].id]),
                    "aimed at the same goal",
                );
                e.edit("goal speed", None, |d| d.goals[0].when.min_kmh = None);
                c.key("cmd-.");
                if !c.click_id("spec-auto") {
                    return Flow::Fail("no Auto checkbox".into());
                }
                Flow::Wait
            }
            _ if e.search.running() => Flow::Wait,
            _ => {
                c.check(!e.spec.auto, "Auto is off");
                c.key("escape");
                Flow::Next
            }
        }
    });
    s.add("point goal: click the road, lift it with its handle", |e, c| {
        let n = e.sim.frames.len();
        let k = n * 35 / 100;
        match c.n {
            1 => {
                e.select(Sel::None);
                c.click(Vec2::new(e.tick_x(k), e.ruler_y()));
                c.key("f");
                Flow::Wait
            }
            2..=40 => Flow::Wait,
            41 if !e.settled() => {
                c.n = 40;
                Flow::Wait
            }
            41 => {
                c.key("2");
                let Some(s) = road_ahead(e) else { return Flow::Fail("road ahead off screen".into()) };
                c.f.insert("goals", e.doc.goals.len() as f64);
                e.spec.auto = true;
                c.click(s);
                Flow::Wait
            }
            42..=200 if e.doc.goals.len() as f64 == c.f["goals"] => Flow::Wait,
            42..=200 => {
                let (gate, point) = (gate_goal(e).map(|(_, g)| g.id), newest(e).map(|g| g.id));
                let aimed = e.aimed_goals().iter().map(|g| g.id).collect::<Vec<_>>();
                c.check(
                    e.search.running()
                        && gate.is_some_and(|g| aimed.contains(&g))
                        && point.is_some_and(|p| aimed.contains(&p)),
                    format!("placing a second goal adds it to the aim ({aimed:?}), searching"),
                );
                e.search.stop();
                e.spec.auto = false;
                c.key("q");
                c.n = 201;
                Flow::Wait
            }
            202..=206 => Flow::Wait,
            207 => {
                let Some((gz, _)) = e.gizmo() else { return Flow::Fail("no lift handle".into()) };
                c.f.insert("y0", gz.center.y as f64);
                let a = e.wpt(gz.center + Vec3::Y * gz.scale * 0.8);
                let Some(a) = a else { return Flow::Fail("handle off screen".into()) };
                let path: Vec<Vec2> = (0..=12).map(|i| a - Vec2::new(0.0, i as f32 * 6.0)).collect();
                c.drag(&path, MouseButton::Left, NONE);
                Flow::Wait
            }
            208..=220 => Flow::Wait,
            _ => {
                let Some(g) = newest(e) else { return Flow::Fail("no goal".into()) };
                let y = g.pos().y as f64;
                c.check(
                    matches!(g.kind, GoalKind::Point { .. }),
                    format!("point goal {} placed by the engine raycast", g.name),
                );
                c.check(
                    y > c.f["y0"] + 0.5,
                    format!("lifted {:.2} m into the air with the handle", y - c.f["y0"]),
                );
                c.shot(e, "10-goal-point-lifted");
                Flow::Next
            }
        }
    });
    s.add("pose goal: place, rotate (ring), type a tolerance", |e, c| {
        let n = e.sim.frames.len();
        let k = n * 55 / 100;
        match c.n {
            1 => {
                e.select(Sel::None);
                c.click(Vec2::new(e.tick_x(k), e.ruler_y()));
                c.key("f");
                Flow::Wait
            }
            2..=40 => Flow::Wait,
            41 if !e.settled() => {
                c.n = 40;
                Flow::Wait
            }
            41 => {
                c.key("4");
                let Some(s) = road_ahead(e) else { return Flow::Fail("road ahead off screen".into()) };
                c.f.insert("goals", e.doc.goals.len() as f64);
                c.click(s);
                Flow::Wait
            }
            42..=200 if e.doc.goals.len() as f64 == c.f["goals"] => Flow::Wait,
            42..=200 => {
                c.key("e");
                c.n = 201;
                Flow::Wait
            }
            202..=206 => Flow::Wait,
            207 => {
                let Some((gz, _)) = e.gizmo() else { return Flow::Fail("no rotate gizmo".into()) };
                let Some(GoalKind::Pose(p)) = newest(e).map(|g| &g.kind) else {
                    return Flow::Fail("not a pose".into());
                };
                c.f.insert("yaw0", p.ypr[0] as f64);
                let n = gz.ring_axis(1);
                let u = n.any_orthonormal_vector();
                let v = n.cross(u);
                let r = gz.scale * 1.2;
                let path: Vec<Vec2> = (0..=10)
                    .filter_map(|i| {
                        let a = i as f32 / 10.0 * 0.6;
                        e.wpt(gz.center + (u * a.cos() + v * a.sin()) * r)
                    })
                    .collect();
                if path.len() < 5 {
                    return Flow::Fail("ring off screen".into());
                }
                c.drag(&path, MouseButton::Left, NONE);
                Flow::Wait
            }
            208..=215 => Flow::Wait,
            216 => {
                let id = newest(e).map_or(0, |g| g.id);
                if !c.click_id(&format!("num:goal.{id}.atol")) {
                    return Flow::Fail("tolerance field not shown".into());
                }
                Flow::Wait
            }
            217..=219 => Flow::Wait,
            220 => {
                c.typ("15");
                c.key("enter");
                Flow::Wait
            }
            221..=226 => Flow::Wait,
            _ => {
                let Some(GoalKind::Pose(p)) = newest(e).map(|g| &g.kind) else {
                    return Flow::Fail("not a pose".into());
                };
                let dyaw = (p.ypr[0] as f64 - c.f["yaw0"]).abs();
                c.check(dyaw > 5.0, format!("yaw ring rotated the pose by {dyaw:.1}\u{00b0}"));
                c.check(
                    (p.angle_tol - 15.0).abs() < 1e-4,
                    format!("typed rotation tolerance: {}\u{00b0}", p.angle_tol),
                );
                c.shot(e, "11-goal-pose");
                c.key("q");
                Flow::Next
            }
        }
    });
    s.add("undo / redo", |e, c| match c.n {
        1 => {
            let Some(g) = newest(e) else {
                return Flow::Fail("no goal to undo (pose goal step failed)".into());
            };
            c.f.insert("tol", if let GoalKind::Pose(p) = &g.kind { p.angle_tol as f64 } else { 0.0 });
            c.key("cmd-z");
            Flow::Wait
        }
        2..=4 => Flow::Wait,
        5 => {
            let t = if let Some(GoalKind::Pose(p)) = newest(e).map(|g| &g.kind) {
                p.angle_tol as f64
            } else {
                0.0
            };
            c.check((t - c.f["tol"]).abs() > 1e-3, format!("undo restored the tolerance to {t}\u{00b0}"));
            c.key("cmd-shift-z");
            Flow::Wait
        }
        6..=8 => Flow::Wait,
        _ => {
            let t = if let Some(GoalKind::Pose(p)) = newest(e).map(|g| &g.kind) {
                p.angle_tol as f64
            } else {
                0.0
            };
            c.check((t - c.f["tol"]).abs() < 1e-3, "redo reapplied it");
            Flow::Next
        }
    });
    s.add("objective: the gate, a 3 s budget", |e, c| {
        let Some(gid) = gate_goal(e).map(|(_, g)| g.id) else {
            return Flow::Fail("no gate goal (gate goal step failed)".into());
        };
        match c.n {
            1 => {
                c.click_id(&format!("goal-{gid}"));
                Flow::Wait
            }
            2..=4 => Flow::Wait,
            5 => {
                e.objective_for_goal(gid);
                if !c.click_id("rtab-1") {
                    return Flow::Fail("no Search tab".into());
                }
                Flow::Wait
            }
            6..=8 => Flow::Wait,
            9 => {
                if c.click_id("num:spec.budget") {
                    return Flow::Wait;
                }
                if c.f.contains_key("unfolded") || !c.click_id("sec-stop") {
                    return Flow::Fail("no budget field under Stop".into());
                }
                c.f.insert("unfolded", 1.0);
                c.n = 6;
                Flow::Wait
            }
            10..=11 => Flow::Wait,
            12 => {
                c.typ("3");
                c.key("enter");
                Flow::Wait
            }
            13..=16 => Flow::Wait,
            _ => {
                let (reach, wps) = e.search_ctx_parts();
                let w = |k: usize| wps.get(k.min(wps.len().saturating_sub(1))).copied().unwrap_or(0) as usize;
                let ctx = crate::optimize::Ctx {
                    goals: &e.doc.goals,
                    race_ms0: e.race_ms0,
                    reach: &reach,
                    waypoints_at: &w,
                };
                let s = e.spec.sentence(&ctx);
                c.say(format!("  objective: {s}"));
                c.say(format!("  constraints: {}", e.spec.constraint_sentence(&ctx)));
                let name = gate_goal(e).map_or(String::new(), |(_, g)| g.name.clone());
                c.check(
                    s.starts_with(&format!("Minimise time to reach {name}")),
                    "the objective is the gate, by time",
                );
                c.check(
                    e.spec.by_time && e.spec.seconds == 3.0,
                    format!("3 s typed into the budget ({} s)", e.spec.seconds),
                );
                c.shot(e, "12-search-objective");
                Flow::Next
            }
        }
    });
    s.add("run the search (\u{2318}R)", |e, c| {
        if c.n == 1 {
            c.key("cmd-r");
            perf_start(e);
            return Flow::Wait;
        }
        if e.search.running() || c.n < 10 {
            let m = e.vcenter();
            if c.n % 2 == 0 && c.n < 400 {
                c.move_to(m, None, ALT);
                c.down(m, MouseButton::Left, ALT, 1);
                c.move_to(m + Vec2::new(3.0, 0.0), Some(MouseButton::Left), ALT);
                c.up(m + Vec2::new(3.0, 0.0), MouseButton::Left, ALT, 1);
            }
            if c.n == 120 {
                c.shot(e, "13-search-running");
            }
            return Flow::Wait;
        }
        perf_end(e, c, "orbiting while the search runs (4 search threads)");
        let l = e.search.live.lock().unwrap();
        let (n, att, err, best) = (l.candidates.len(), l.attempts, l.error.clone(), l.best.clone());
        drop(l);
        c.check(err.is_none(), format!("search finished without error {}", err.unwrap_or_default()));
        c.say(format!(
            "  {att} attempts, {n} candidates, best {}",
            best.map_or("-".into(), |b| crate::optimize::metric(&b).text)
        ));
        c.check(n > 0, "at least one improving candidate (each verified by tas's fresh-runner replay)");
        c.shot(e, "14-search-done");
        Flow::Next
    });
    s.add("preview a candidate, accept it", |e, c| {
        let last = e.search.live.lock().unwrap().candidates.last().map(|x| x.id);
        let Some(id) = last else { return Flow::Fail("no candidate".into()) };
        match c.n {
            1 => {
                if e.sel != Sel::Candidate(id) && !c.click_id(&format!("cand-{id}")) {
                    return Flow::Fail(format!("no result row cand-{id}"));
                }
                Flow::Wait
            }
            _ if !e.runs_frames.contains_key(&crate::background::Key::Candidate(id)) || c.n < 4 => Flow::Wait,
            _ if !c.f.contains_key("previewed") => {
                c.f.insert("previewed", 1.0);
                c.check(e.preview == Some(id), "selected candidate previews (ghost path, timeline diff)");
                c.say(format!("  check: {}", e.checks.get(&id).cloned().unwrap_or_default()));
                c.shot(e, "15-candidate-preview");
                c.f.insert("before", gate_goal(e).and_then(|(i, _)| e.reach.get(i)?.ms).unwrap_or(0) as f64);
                if !c.click_id(&format!("acc-{id}")) {
                    return Flow::Fail("no Accept button".into());
                }
                Flow::Wait
            }
            _ if !matches!(e.doc.run_origin, crate::doc::Origin::Search(_)) && c.n < 300 => Flow::Wait,
            _ if e.accept_check.is_some() || !e.ready() => Flow::Wait,
            _ => {
                let msg = e.checks.get(&id).cloned().unwrap_or_default();
                c.check(msg.contains("identical"), format!("replays exactly: {msg}"));
                let after = gate_goal(e).and_then(|(i, _)| e.reach.get(i)?.ms).unwrap_or(0) as f64;
                c.say(format!(
                    "  gate time {} \u{2192} {}",
                    crate::fmt::fmt_ms(c.f["before"] as i32),
                    crate::fmt::fmt_ms(after as i32)
                ));
                c.check(
                    matches!(e.doc.run_origin, crate::doc::Origin::Search(_)),
                    "the candidate is the run (undoable)",
                );
                c.shot(e, "16-candidate-accepted");
                Flow::Next
            }
        }
    });
    s.add("A/B compare (previous version, then the ghost)", |e, c| match c.n {
        1 => {
            c.click_id("btab-1");
            Flow::Wait
        }
        2..=40 => Flow::Wait,
        41 => {
            c.shot(e, "17-compare-previous");
            c.click_id("ghost-0");
            Flow::Wait
        }
        42..=45 => Flow::Wait,
        46 => {
            if !c.click_id("ghost-b") {
                return Flow::Fail("no compare button on the ghost".into());
            }
            Flow::Wait
        }
        47..=70 => Flow::Wait,
        _ => {
            c.check(matches!(e.compare.1, crate::app::RunRef::Ghost(0)), "B is the replay ghost");
            c.shot(e, "18-compare-ghost");
            c.click_id("btab-0");
            Flow::Next
        }
    });
    s.add("place car on the surface and simulate", |e, c| {
        let n = e.sim.frames.len();
        let k = n * 25 / 100;
        match c.n {
            1 => {
                c.click(Vec2::new(e.tick_x(k), e.ruler_y()));
                c.key("f");
                Flow::Wait
            }
            2..=40 => Flow::Wait,
            41 if !e.settled() => {
                c.n = 40;
                Flow::Wait
            }
            41 => {
                c.key("c");
                let f = &e.sim.frames[e.cursor];
                let p = Vec3::from(f.position()) + Vec3::from(f.axis(2)) * 6.0 + Vec3::from(f.axis(0)) * 1.5;
                let Some(s) = e.wpt(p) else { return Flow::Fail("road off screen".into()) };
                c.v.insert("at", s.extend(0.0));
                c.move_to(s - Vec2::new(30.0, 0.0), None, NONE);
                c.move_to(s, None, NONE);
                Flow::Wait
            }
            42..=120 if e.car_preview.is_none() => Flow::Wait,
            42..=120 => {
                c.shot(e, "19-place-car-preview");
                c.click(c.v["at"].truncate());
                c.n = 121;
                Flow::Wait
            }
            122..=130 => Flow::Wait,
            131 => {
                if e.doc.start.is_none() {
                    return Flow::Fail("no start state".into());
                }
                if !c.click_id("num:start.speed") {
                    return Flow::Fail("no speed field".into());
                }
                Flow::Wait
            }
            132..=134 => Flow::Wait,
            135 => {
                c.typ("120");
                c.key("enter");
                Flow::Wait
            }
            136..=140 => Flow::Wait,
            141 => {
                c.key("q");
                c.key("cmd-p");
                Flow::Wait
            }
            _ if !e.sim.done() || c.n < 260 => Flow::Wait,
            _ => {
                let s = e.doc.start.clone().unwrap();
                let f = e.sim.frames.get(s.tick).copied();
                let ok = f.is_some_and(|f| f.position().map(f32::to_bits) == s.pos.map(f32::to_bits));
                c.check(
                    ok,
                    format!("the run arrives at tick {} with the car exactly where it was placed", s.tick),
                );
                let v = glam::Vec3::from(s.velocity).length() * 3.6;
                c.check((v - 120.0).abs() < 0.5, format!("typed start speed {v:.1} km/h along the heading"));
                let after = e
                    .sim
                    .frames
                    .get(s.tick + 50)
                    .map_or(0.0, |f| Vec3::from(f.position()).distance(Vec3::from(s.pos)));
                c.check(after > 5.0, format!("simulated from there: {after:.1} m in 0.5 s"));
                c.shot(e, "20-simulate-from-start");
                c.key("space");
                Flow::Next
            }
        }
    });
    s.add("save the project (\u{2318}S), reopen it", |e, c| match c.n {
        1 => {
            e.project_name = "e2e-script".into();
            c.key("cmd-s");
            Flow::Wait
        }
        2..=6 => Flow::Wait,
        7 => {
            let Some(p) = e.project_path.clone() else { return Flow::Fail("not saved".into()) };
            c.say(format!("  saved {}", p.display()));
            c.f.insert("goals", e.doc.goals.len() as f64);
            c.f.insert("runs", e.doc.runs.len() as f64);
            c.f.insert("bm", e.doc.bookmarks.len() as f64);
            c.v.insert("start", Vec3::from(e.doc.start.as_ref().map_or([0.0; 3], |s| s.pos)));
            c.key("cmd-o");
            Flow::Wait
        }
        8..=10 => Flow::Wait,
        11 => {
            let saved = e.project_path.clone();
            let Some(i) = e.pm_projects().iter().position(|x| Some(&x.path) == saved.as_ref()) else {
                return Flow::Fail("project not listed in Open".into());
            };
            if !c.dbl_click_id(&format!("proj-{i}")) {
                return Flow::Fail(format!("no row proj-{i} on the Projects screen"));
            }
            Flow::Wait
        }
        _ if c.n < 30 || !e.ready() || e.gates.is_empty() => Flow::Wait,
        _ => {
            c.check(
                e.doc.goals.len() as f64 == c.f["goals"],
                format!("reloaded {} goals", e.doc.goals.len()),
            );
            c.check(e.doc.bookmarks.len() as f64 == c.f["bm"], "camera bookmarks reloaded");
            c.check(
                e.doc.start.as_ref().is_some_and(|s| Vec3::from(s.pos) == c.v["start"]),
                "start state reloaded",
            );
            c.check(
                e.sim
                    .frames
                    .get(e.doc.start.as_ref().map_or(0, |s| s.tick))
                    .is_some_and(|f| Vec3::from(f.position()) == c.v["start"]),
                "and simulated from it again",
            );
            c.shot(e, "24-project-reloaded");
            if let Some(dir) = e
                .project_path
                .as_ref()
                .and_then(|p| p.parent())
                .filter(|d| d.ends_with("editor/projects/e2e-script"))
            {
                let _ = std::fs::remove_dir_all(dir);
            }
            Flow::Next
        }
    });
    s.add("shortcut cheat sheet (?)", |e, c| match c.n {
        1 => {
            c.key("shift-/");
            Flow::Wait
        }
        2..=8 => Flow::Wait,
        9 => {
            c.check(e.cheatsheet, "cheat sheet open");
            c.shot(e, "25-cheatsheet");
            c.key("escape");
            Flow::Wait
        }
        _ => Flow::Next,
    });
    s.add("idle frame times and the final view", |e, c| match c.n {
        1 => {
            c.key("home");
            perf_start(e);
            Flow::Wait
        }
        2..=240 => {
            let m = e.vcenter();
            c.move_to(m, None, ALT);
            c.down(m, MouseButton::Left, ALT, 1);
            c.move_to(m + Vec2::new(2.0, 0.0), Some(MouseButton::Left), ALT);
            c.up(m + Vec2::new(2.0, 0.0), MouseButton::Left, ALT, 1);
            Flow::Wait
        }
        _ => {
            perf_end(e, c, "continuous orbit, whole track in view, idle simulation");
            e.sel = Sel::None;
            e.tool = Tool::Select;
            c.shot(e, "26-overview");
            Flow::Next
        }
    });
}

fn stage(c: &mut Ctl) -> Option<u32> {
    if c.n == 1 {
        c.f.insert("stage", 0.0);
        c.f.insert("acted", 0.0);
    }
    (c.n >= c.f["acted"] as u32 + 3).then(|| c.f["stage"] as u32)
}

fn advance(c: &mut Ctl) -> Flow {
    c.f.insert("stage", c.f["stage"] + 1.0);
    c.f.insert("acted", c.n as f64);
    Flow::Wait
}

fn press(c: &mut Ctl, id: &str, missing: &str) -> Flow {
    if c.click_id(id) {
        advance(c)
    } else if c.n > c.f["acted"] as u32 + 120 {
        Flow::Fail(missing.into())
    } else {
        Flow::Wait
    }
}

fn build_smoke(s: &mut Script) {
    s.add("open a map: the 3D view renders", |e, c| {
        if !(e.ready() && !e.gates.is_empty() && e.origin.is_some()) {
            return Flow::Wait;
        }
        let at = *c.f.entry("ready").or_insert(c.n as f64) as u32;
        if c.n < at + 5 {
            return Flow::Wait;
        }
        c.check(
            e.sim.frames.len() == e.doc.len + 1,
            format!("simulated every tick ({} frames for {} ticks)", e.sim.frames.len(), e.doc.len),
        );
        let sd = c.rendered(e, "smoke-1-open");
        c.check(sd > 8.0, format!("the viewport shows the track (luma spread {sd:.1})"));
        Flow::Next
    });
    s.add("import a replay's inputs (\u{2318}I)", |e, c| match stage(c) {
        _ if c.n == 1 => {
            c.key("cmd-i");
            Flow::Wait
        }
        Some(0) => press(c, "menu-0", "no replay in the import menu"),
        Some(_) if matches!(e.doc.run_origin, crate::doc::Origin::Replay(_)) && e.ready() => {
            let fin = e.sim.frames.iter().find_map(|f| f.finish);
            c.check(
                fin.is_some(),
                format!(
                    "the imported run finishes: {}",
                    fin.map_or("no".into(), |t| crate::fmt::fmt_ms(t as i32))
                ),
            );
            Flow::Next
        }
        _ => Flow::Wait,
    });
    s.add("scrub the timeline", |e, c| match stage(c) {
        Some(0) => {
            let (k0, k1) = (e.race_start_tick() + 1200, e.race_start_tick() + 300);
            let vis = e.tl.start..e.tl.start + e.tl.width() as f64 / e.tl.ppt.max(1e-9);
            if !(vis.contains(&(k0 as f64)) && vis.contains(&(k1 as f64))) {
                return Flow::Fail(format!(
                    "ticks {k1}..{k0} are not on the timeline ({:.0}..{:.0})",
                    vis.start, vis.end
                ));
            }
            let y = e.ruler_y();
            let path: Vec<Vec2> = (0..=12)
                .map(|i| Vec2::new(e.tick_x(k0) + (e.tick_x(k1) - e.tick_x(k0)) * i as f32 / 12.0, y))
                .collect();
            c.f.insert("to", k1 as f64);
            c.drag(&path, MouseButton::Left, NONE);
            advance(c)
        }
        Some(_) if e.cursor.abs_diff(c.f["to"] as usize) > 1 && c.n < c.f["acted"] as u32 + 30 => Flow::Wait,
        Some(_) => {
            let to = c.f["to"] as usize;
            c.check(
                e.cursor.abs_diff(to) <= 1,
                format!("dragging the ruler moved the playhead to tick {} (aimed at {to})", e.cursor),
            );
            Flow::Next
        }
        None => Flow::Wait,
    });
    s.add("play (Space)", |e, c| match stage(c) {
        Some(0) => {
            c.f.insert("from", e.cursor as f64);
            c.f.insert("t0", secs());
            c.key("space");
            advance(c)
        }
        Some(1) if secs() - c.f["t0"] < 0.6 || (!e.playing && c.n < c.f["acted"] as u32 + 30) => Flow::Wait,
        Some(1) => {
            c.check(e.playing, "Space plays");
            c.key("space");
            advance(c)
        }
        Some(_) if e.playing && c.n < c.f["acted"] as u32 + 30 => Flow::Wait,
        Some(_) => {
            let moved = e.cursor as f64 - c.f["from"];
            c.check(
                !e.playing && moved >= 20.0,
                format!("played {moved} ticks in {:.2} s, then Space stopped it", secs() - c.f["t0"]),
            );
            Flow::Next
        }
        None => Flow::Wait,
    });
    s.add("search to the first checkpoint, 2 s budget", |e, c| {
        let cp = e
            .sim
            .frames
            .windows(2)
            .find(|w| w[1].waypoints > w[0].waypoints)
            .and_then(|w| w[1].last_crossing)
            .map(|x| x.0);
        let Some(w) = cp else { return Flow::Fail("the run crosses no checkpoint".into()) };
        match stage(c) {
            Some(0) => press(c, "rtab-1", "no Search tab"),
            Some(1) => {
                if hit("num:spec.budget").is_none() && !c.f.contains_key("unfolded") && c.click_id("sec-stop")
                {
                    c.f.insert("unfolded", 1.0);
                }
                press(c, "num:spec.budget", "no budget field under Stop")
            }
            Some(2) => {
                c.typ("2");
                c.key("enter");
                advance(c)
            }
            Some(3) if e.spec.seconds != 2.0 && c.n < c.f["acted"] as u32 + 60 => Flow::Wait,
            Some(3) => {
                c.check(
                    e.spec.by_time && e.spec.seconds == 2.0,
                    format!("2 s typed into the budget ({} s)", e.spec.seconds),
                );
                advance(c)
            }
            Some(4) => {
                if hit(&format!("gate-{w}")).is_none()
                    && !c.f.contains_key("opened")
                    && c.click_id("grp-world")
                {
                    c.f.insert("opened", 1.0);
                }
                press(c, &format!("gate-{w}"), &format!("no gate-{w} row in the Outliner"))
            }
            Some(5) => press(c, "gate-goal", "no Add as goal button on the checkpoint"),
            Some(6) if gate_goal(e).is_none() && c.n < c.f["acted"] as u32 + 60 => Flow::Wait,
            Some(6) => {
                c.f.insert("t0", secs());
                press(c, "goal-obj", "no Search to here button on the goal")
            }
            Some(7) if e.search.running() => Flow::Wait,
            Some(7) if c.n > c.f["acted"] as u32 + 600 => Flow::Fail("the search never ran".into()),
            Some(7) => {
                let l = e.search.live.lock().unwrap();
                let (n, att, err) = (l.candidates.len(), l.attempts, l.error.clone());
                drop(l);
                if att == 0 && err.is_none() {
                    return Flow::Wait;
                }
                c.check(
                    err.is_none(),
                    format!("the search finished without error {}", err.unwrap_or_default()),
                );
                c.check(
                    n > 0,
                    format!("{n} improving candidates from {att} tries in {:.1} s", secs() - c.f["t0"]),
                );
                Flow::Next
            }
            _ => Flow::Wait,
        }
    });
    s.add("preview a candidate, use it", |e, c| {
        let last = e.search.live.lock().unwrap().candidates.last().map(|x| x.id);
        let Some(id) = last else { return Flow::Fail("no candidate".into()) };
        match stage(c) {
            Some(0) if e.sel == Sel::Candidate(id) => advance(c),
            Some(0) => press(c, &format!("cand-{id}"), &format!("no result row cand-{id}")),
            Some(1)
                if (e.sel != Sel::Candidate(id)
                    || !e.runs_frames.contains_key(&crate::background::Key::Candidate(id)))
                    && c.n < c.f["acted"] as u32 + 300 =>
            {
                Flow::Wait
            }
            Some(1) => {
                c.check(e.preview == Some(id), "the candidate previews");
                c.f.insert(
                    "before",
                    gate_goal(e).and_then(|(i, _)| e.reach.get(i)?.ms).unwrap_or(i32::MAX) as f64,
                );
                press(c, &format!("acc-{id}"), "no Use it button")
            }
            Some(_)
                if e.accept_check.is_some()
                    || !e.ready()
                    || !matches!(e.doc.run_origin, crate::doc::Origin::Search(_)) =>
            {
                if c.n > c.f["acted"] as u32 + 600 {
                    return Flow::Fail("Use it did not make the candidate the run".into());
                }
                Flow::Wait
            }
            Some(_) => {
                let msg = e.checks.get(&id).cloned().unwrap_or_default();
                c.check(msg.contains("identical"), format!("the run replays the preview exactly: {msg}"));
                let after = gate_goal(e).and_then(|(i, _)| e.reach.get(i)?.ms);
                c.check(
                    after.is_some_and(|a| a as f64 <= c.f["before"]),
                    format!(
                        "checkpoint time {} \u{2192} {}",
                        crate::fmt::fmt_ms(c.f["before"] as i32),
                        after.map_or("-".into(), crate::fmt::fmt_ms)
                    ),
                );
                let sd = c.rendered(e, "smoke-2-accepted");
                c.check(sd > 8.0, format!("the viewport still renders (luma spread {sd:.1})"));
                Flow::Next
            }
            None => Flow::Wait,
        }
    });
    s.add("save the project (\u{2318}S)", |e, c| match stage(c) {
        Some(0) => {
            c.key("cmd-s");
            advance(c)
        }
        Some(_) if e.project_path.is_none() && c.n < 120 => Flow::Wait,
        Some(_) => {
            let p = e.project_path.clone();
            c.check(
                p.as_ref().is_some_and(|p| p.exists()),
                format!("saved {}", p.map_or("nothing".into(), |p| p.display().to_string())),
            );
            c.check(!e.dirty_doc(), "nothing left unsaved");
            Flow::Next
        }
        None => Flow::Wait,
    });
    s.add("quit (\u{2318}Q)", |_, c| {
        if c.n == 1 {
            c.f.insert("t0", secs());
            c.quit_ok = true;
            c.key("cmd-q");
        }
        if secs() - c.f["t0"] < 3.0 {
            return Flow::Wait;
        }
        c.quit_ok = false;
        Flow::Fail("\u{2318}Q did not quit".into())
    });
}

fn build_record(s: &mut Script, path: PathBuf) {
    s.add("ready", |e, c| {
        if !(e.ready() && !e.gates.is_empty() && !e.doc.inputs.controls.is_empty()) {
            return Flow::Wait;
        }
        c.key("home");
        Flow::Next
    });
    let p2 = path.clone();
    s.add("start recording", move |e, c| {
        if c.n < 30 {
            return Flow::Wait;
        }
        let _ = std::fs::remove_file(&p2);
        perf_start(e);
        let id = e.window_id.unwrap_or(0);
        let child = std::process::Command::new("screencapture")
            .args(["-v", "-V28", "-x", "-k", &format!("-l{id}")])
            .arg(&p2)
            .spawn()
            .ok();
        c.say(format!("recording {} ({})", p2.display(), if child.is_some() { "started" } else { "FAILED" }));
        RECORDER.with(|r| *r.borrow_mut() = child);
        Flow::Next
    });
    s.add("orbit", |e, c| {
        let m = e.vcenter();
        if c.n > 150 {
            return Flow::Next;
        }
        let d = Vec2::new(4.0, if c.n < 75 { -0.6 } else { 0.6 });
        c.move_to(m, None, ALT);
        c.down(m, MouseButton::Left, ALT, 1);
        c.move_to(m + d, Some(MouseButton::Left), ALT);
        c.up(m + d, MouseButton::Left, ALT, 1);
        Flow::Wait
    });
    s.add("dolly in to the cursor", |e, c| {
        let Some(p) = e.sim.frames.get(e.sim.frames.len() / 3).map(|f| Vec3::from(f.position())) else {
            return Flow::Next;
        };
        let m = e.wpt(p).unwrap_or(e.vcenter());
        if c.n > 90 {
            return Flow::Next;
        }
        c.move_to(m, None, NONE);
        if c.n > 5 && c.n % 3 == 0 {
            c.scroll_px(m, Vec2::new(0.0, 0.0), Modifiers { platform: true, ..NONE }, TouchPhase::Moved);
            c.pinch(m, 0.035);
        }
        Flow::Wait
    });
    s.add("pan", |e, c| {
        if c.n > 60 {
            return Flow::Next;
        }
        c.scroll_px(e.vcenter(), Vec2::new(-5.0, 1.5), NONE, TouchPhase::Moved);
        Flow::Wait
    });
    s.add("fly", |e, c| {
        let m = e.vcenter();
        match c.n {
            1 => {
                c.down(m, MouseButton::Right, NONE, 1);
                c.key_down("w");
                Flow::Wait
            }
            2..=110 => {
                c.move_to(
                    m + Vec2::new(c.n as f32 * 0.8, (c.n as f32 * 0.05).sin() * 20.0),
                    Some(MouseButton::Right),
                    NONE,
                );
                Flow::Wait
            }
            _ => {
                c.key_up("w");
                c.up(m, MouseButton::Right, NONE, 1);
                Flow::Next
            }
        }
    });
    s.add("follow playback", |e, c| match c.n {
        1 => {
            c.click(Vec2::new(e.tick_x(e.sim.frames.len() / 5), e.ruler_y()));
            c.key("shift-f");
            Flow::Wait
        }
        2 => {
            c.key("space");
            Flow::Wait
        }
        3..=330 => {
            if c.n % 4 == 0 && (100..200).contains(&c.n) {
                let m = e.vcenter();
                c.drag(&[m, m + Vec2::new(5.0, 0.0)], MouseButton::Middle, NONE);
            }
            Flow::Wait
        }
        _ => {
            c.key("space");
            c.key("shift-f");
            c.key("home");
            Flow::Next
        }
    });
    s.add("search session", |e, c| match c.n {
        1 => {
            let cp = e
                .sim
                .frames
                .windows(2)
                .find(|w| w[1].waypoints > w[0].waypoints)
                .and_then(|w| w[1].last_crossing)
                .map(|x| x.0);
            if let Some(w) = cp {
                c.click_id(&format!("gate-{w}"));
            }
            Flow::Wait
        }
        2..=20 => Flow::Wait,
        21 => {
            c.click_id("gate-goal");
            Flow::Wait
        }
        22..=40 => Flow::Wait,
        41 => {
            c.click_id("goal-obj");
            c.key("f");
            Flow::Wait
        }
        42..=70 => Flow::Wait,
        71 => {
            e.spec.attempts = 320;
            c.click_id("sem-toggle");
            Flow::Wait
        }
        72..=120 => Flow::Wait,
        121 => {
            c.key("cmd-r");
            Flow::Wait
        }
        _ if e.search.running() || c.n < 130 => Flow::Wait,
        _ => Flow::Next,
    });
    s.add("preview and accept", |e, c| {
        let last = e.search.live.lock().unwrap().candidates.last().map(|x| x.id);
        match (c.n, last) {
            (_, None) => Flow::Next,
            (1, Some(id)) => {
                c.click_id(&format!("cand-{id}"));
                Flow::Wait
            }
            (2..=150, _) => Flow::Wait,
            (151, Some(id)) => {
                c.click_id(&format!("acc-{id}"));
                Flow::Wait
            }
            (152..=260, _) => Flow::Wait,
            _ => Flow::Next,
        }
    });
    s.add("stop recording", |e, c| {
        perf_end(e, c, "screen recording (camera work + search session)");
        RECORDER.with(|r| {
            if let Some(mut ch) = r.borrow_mut().take() {
                let _ = std::process::Command::new("kill").args(["-INT", &ch.id().to_string()]).status();
                let _ = ch.wait();
            }
        });
        Flow::Next
    });
    s.record = Some((path, None));
}

thread_local! {
    static RECORDER: std::cell::RefCell<Option<std::process::Child>> = const { std::cell::RefCell::new(None) };
}
