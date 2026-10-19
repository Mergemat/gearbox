use std::time::Instant;

use glam::{Quat, Vec2, Vec3};
use gpui::prelude::FluentBuilder;
use gpui::{
    AnyElement, Bounds, Context, DispatchPhase, FontWeight, Hitbox, HitboxBehavior, InteractiveElement, IntoElement, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent, ParentElement,
    PinchEvent, Pixels, ScrollWheelEvent, StatefulInteractiveElement, Styled, Window, canvas, div, px,
};

use crate::app::*;
use crate::camera::{self, Car};
use crate::doc::{self, GoalKind, PoseSpec, StartState};
use crate::fmt::fmt_ms;
use crate::gfx::renderer::{Output, View};
use crate::gizmo::{self, Gizmo, Handle};
use crate::picker::{Picked, PlaceWhat, Purpose};
use crate::scene::{RefRun, SceneInput};
use crate::theme::{self, c};
use crate::ui::widgets::*;

fn ray_box(o: Vec3, d: Vec3, min: Vec3, max: Vec3) -> Option<f32> {
    let inv = d.recip();
    let (t1, t2) = ((min - o) * inv, (max - o) * inv);
    let (tmin, tmax) = (t1.min(t2).max_element(), t1.max(t2).min_element());
    (tmax >= tmin.max(0.0)).then_some(tmin.max(0.0))
}

impl Editor {
    fn local(&self, p: gpui::Point<Pixels>) -> Vec2 {
        v2(p) - v2(self.viewport.origin)
    }

    pub fn vsize(&self) -> Vec2 {
        Vec2::new(f32::from(self.viewport.size.width), f32::from(self.viewport.size.height))
    }

    fn car_focus(&self) -> Option<Car> {
        self.display_frame().map(|f| Car { pos: Vec3::from(f.position()), forward: Vec3::from(f.axis(2)), vel: Vec3::from(f.velocity()) })
    }

    pub fn gizmo(&self) -> Option<(Gizmo, GizmoTarget)> {
        let eye = self.camera.eye();
        let (target, pos, rot, rotatable) = match self.sel {
            Sel::Goal(id) => match &self.doc.goal(id)?.kind {
                GoalKind::Point { pos, .. } => (GizmoTarget::Goal(id), Vec3::from(*pos), Quat::IDENTITY, false),
                GoalKind::Pose(p) => (GizmoTarget::Goal(id), Vec3::from(p.pos), doc::ypr_rot(p.ypr), true),
                GoalKind::Box { pos, ypr, .. } => (GizmoTarget::Goal(id), Vec3::from(*pos), doc::ypr_rot(*ypr), true),
                GoalKind::Gate(_) => return None,
            },
            Sel::Start => {
                let s = self.doc.start.as_ref()?;
                (GizmoTarget::Start, Vec3::from(s.pos), doc::ypr_rot(s.ypr), true)
            }
            _ => return None,
        };
        let kind = match self.tool {
            Tool::Scale => {
                let (_, box_rot) = match self.sel {
                    Sel::Goal(id) => match &self.doc.goal(id)?.kind {
                        GoalKind::Box { ypr, .. } => ((), doc::ypr_rot(*ypr)),
                        GoalKind::Point { .. } | GoalKind::Pose(_) => ((), Quat::IDENTITY),
                        GoalKind::Gate(_) => return None,
                    },
                    _ => return None,
                };
                return Some((Gizmo::new(pos, eye, gizmo::GizmoMode::Scale, box_rot, false), target));
            }
            Tool::Move => gizmo::GizmoMode::Translate,
            Tool::Rotate if rotatable => gizmo::GizmoMode::Rotate,
            Tool::Select if !rotatable => gizmo::GizmoMode::Lift,
            Tool::Select => gizmo::GizmoMode::Translate,
            _ => return None,
        };
        let center = if rotatable { pos + Vec3::Y * 0.5 } else { pos };
        Some((Gizmo::new(center, eye, kind, rot, true), target))
    }

    fn target_size(&self, t: GizmoTarget) -> Option<Vec3> {
        let GizmoTarget::Goal(id) = t else { return None };
        match &self.doc.goal(id)?.kind {
            GoalKind::Box { size, .. } => Some(Vec3::from(*size)),
            GoalKind::Pose(p) => Some(Vec3::from(p.pos_tol)),
            GoalKind::Point { radius, .. } => Some(Vec3::splat(*radius)),
            GoalKind::Gate(_) => None,
        }
    }

    fn set_target_size(&mut self, t: GizmoTarget, s: Vec3) {
        let GizmoTarget::Goal(id) = t else { return };
        let s = s.max(Vec3::splat(0.1));
        self.edit("size goal", Some(&format!("gizmo-size-{id}")), |d| {
            if let Some(g) = d.goal_mut(id) {
                match &mut g.kind {
                    GoalKind::Box { size, .. } => *size = s.into(),
                    GoalKind::Pose(p) => p.pos_tol = s.into(),
                    GoalKind::Point { radius, .. } => *radius = s.x,
                    GoalKind::Gate(_) => {}
                }
            }
        });
    }

    fn target_pose(&self, t: GizmoTarget) -> Option<(Vec3, Quat)> {
        match t {
            GizmoTarget::Goal(id) => match &self.doc.goal(id)?.kind {
                GoalKind::Point { pos, .. } => Some((Vec3::from(*pos), Quat::IDENTITY)),
                GoalKind::Pose(p) => Some((Vec3::from(p.pos), doc::ypr_rot(p.ypr))),
                GoalKind::Box { pos, ypr, .. } => Some((Vec3::from(*pos), doc::ypr_rot(*ypr))),
                _ => None,
            },
            GizmoTarget::Start => self.doc.start.as_ref().map(|s| (Vec3::from(s.pos), doc::ypr_rot(s.ypr))),
        }
    }

    fn set_target_pose(&mut self, t: GizmoTarget, pos: Vec3, rot: Quat) {
        let ypr = doc::rot_ypr(rot);
        match t {
            GizmoTarget::Goal(id) => self.edit("move goal", Some(&format!("gizmo-{id}")), |d| {
                if let Some(g) = d.goal_mut(id) {
                    match &mut g.kind {
                        GoalKind::Point { pos: p, .. } => *p = pos.into(),
                        GoalKind::Pose(p) => {
                            p.pos = pos.into();
                            p.ypr = ypr;
                        }
                        GoalKind::Box { pos: p, ypr: r, .. } => {
                            *p = pos.into();
                            *r = ypr;
                        }
                        _ => {}
                    }
                }
            }),
            GizmoTarget::Start => self.edit("move start state", Some("gizmo-start"), |d| {
                if let Some(s) = &mut d.start {
                    s.pos = pos.into();
                    s.ypr = ypr;
                }
            }),
        }
    }

    fn velocity_hit(&self, m: Vec2) -> bool {
        let Some(s) = self.doc.start.as_ref().filter(|_| self.sel == Sel::Start) else { return false };
        camera::project(&self.vp, self.vsize(), crate::scene::velocity_tip(s)).is_some_and(|q| q.distance(m) < 14.0)
    }

    fn drag_velocity(&mut self, m: Vec2, vy: f32, shift: bool) {
        let Some(s) = self.doc.start.clone() else { return };
        let p = Vec3::from(s.pos) + Vec3::Y * 0.6;
        let (o, d) = camera::ray(&self.vp, self.vsize(), m);
        if d.y.abs() < 1e-4 {
            return;
        }
        let t = (p.y - o.y) / d.y;
        if t <= 0.0 {
            return;
        }
        let flat = (o + d * t - p) / crate::scene::VEL_ARROW_S;
        let fwd = doc::ypr_rot(s.ypr) * Vec3::Z;
        let fwd_flat = Vec3::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
        let mut speed = flat.length();
        let mut dir = flat.normalize_or_zero();
        if dir.angle_between(fwd_flat).to_degrees() < 4.0 {
            dir = fwd_flat;
        }
        if shift {
            speed = ((speed * 3.6 / 5.0).round() * 5.0) / 3.6;
        }
        let v = dir * speed + Vec3::Y * vy;
        self.edit("start speed", Some("start-velocity-drag"), |d| {
            if let Some(s) = &mut d.start {
                s.velocity = v.into();
            }
        });
    }

    fn request_pick(&mut self, m: Vec2, purpose: Purpose) {
        let (o, d) = camera::ray(&self.vp, self.vsize(), m);
        if let Some(p) = &self.picker {
            p.request(o, d, purpose);
        }
    }

    pub fn picked(&mut self, p: Picked) {
        self.pick_us = Some(p.micros);
        if let Some(e) = &p.error {
            self.error("pick", e.clone());
        }
        match p.purpose {
            Purpose::Hover => self.hover_hit = p.hit.map(|h| (self.mouse, Vec3::from(h.point), Instant::now())),
            Purpose::Wheel { tick, index } => {
                if let Some(f) = self.sim.frames.get(tick) {
                    let centers = wheel_centers(f);
                    let c = self.contacts.get_or_insert((tick, centers.map(|c| (c, None))));
                    if c.0 != tick {
                        *c = (tick, centers.map(|c| (c, None)));
                    }
                    c.1[index as usize].1 = p.hit.map(|h| Vec3::from(h.point));
                }
            }
            Purpose::Place { what: PlaceWhat::Line, .. } => match p.hit {
                Some(h) => self.add_line_point(Vec3::from(h.point)),
                None => self.log(LogLevel::Warn, "goal", "nothing under the cursor"),
            },
            Purpose::Place { what, eye } => match p.hit {
                Some(h) => {
                    let at = Vec3::from(h.point);
                    let yaw = self.heading_near(at).unwrap_or_else(|| {
                        let f = at - eye;
                        f.x.atan2(f.z).to_degrees()
                    });
                    let kind = match what {
                        PlaceWhat::Pose => GoalKind::Pose(PoseSpec::at(at, yaw)),
                        PlaceWhat::Point => GoalKind::Point { pos: at.into(), radius: 3.0 },
                        PlaceWhat::Box => GoalKind::Box { pos: (at + Vec3::Y * 4.0).into(), size: [16.0, 8.0, 2.0], ypr: [yaw, 0.0, 0.0] },
                        PlaceWhat::Line => return,
                    };
                    let id = self.add_goal(kind);
                    self.auto_search(id);
                    let surf = crate::materials::surface_name(h.surface);
                    let label = match what {
                        PlaceWhat::Pose => "pose goal",
                        PlaceWhat::Point => "point goal",
                        PlaceWhat::Box => "box trigger",
                        PlaceWhat::Line => "line point",
                    };
                    self.info("goal", format!("placed a {label} on {surf} at {:.2} {:.2} {:.2} (engine raycast {} \u{00b5}s)", at.x, at.y, at.z, p.micros));
                }
                None => self.log(LogLevel::Warn, "goal", "nothing under the cursor"),
            },
            Purpose::Car { eye, commit } => {
                let Some(h) = p.hit else {
                    self.car_preview = None;
                    return;
                };
                let at = Vec3::from(h.point);
                let n = Vec3::from(h.normal).normalize_or_zero();
                let yaw = match (&self.drag, &self.doc.start) {
                    (Some(Drag::Gizmo { target: GizmoTarget::Start, .. }), Some(s)) => s.ypr[0],
                    _ => self.heading_near(at).unwrap_or_else(|| {
                        let f = at - eye;
                        f.x.atan2(f.z).to_degrees()
                    }),
                };
                let rot = surface_rot(yaw, n);
                if let Some(Drag::Gizmo { target, handle: Handle::Center, .. }) = self.drag {
                    let r = match target {
                        GizmoTarget::Start => rot,
                        _ => self.target_pose(target).map_or(rot, |t| t.1),
                    };
                    self.set_target_pose(target, at, r);
                    return;
                }
                let prev = self.doc.start.clone();
                let s = StartState {
                    tick: prev.as_ref().map_or(self.cursor.max(self.race_start_tick()), |s| s.tick),
                    pos: at.into(),
                    ypr: doc::rot_ypr(rot),
                    velocity: prev.as_ref().map_or([0.0; 3], |s| s.velocity),
                    angular: prev.as_ref().map_or([0.0; 3], |s| s.angular),
                    reset_contacts: prev.as_ref().is_none_or(|s| s.reset_contacts),
                };
                if commit {
                    self.car_preview = None;
                    self.place_start(s);
                    let surf = crate::materials::surface_name(h.surface);
                    self.info("start", format!("start state on {surf} at {:.2} {:.2} {:.2}, tick {} (engine raycast {} \u{00b5}s)", at.x, at.y, at.z, self.doc.start.as_ref().map_or(0, |s| s.tick), p.micros));
                } else if self.tool == Tool::PlaceCar {
                    self.car_preview = Some(s);
                }
            }
            Purpose::Pivot => {
                if let Some(h) = p.hit {
                    let hit = Vec3::from(h.point);
                    let eye = self.camera.eye();
                    let d = (hit - eye).normalize_or_zero();
                    self.camera.go(camera::View { pivot: hit, yaw: d.x.atan2(d.z), pitch: -d.y.asin(), distance: hit.distance(eye) });
                }
            }
        }
        self.dirty = true;
    }

    fn heading_near(&self, p: Vec3) -> Option<f32> {
        let (k, d) = self.sim.frames.iter().enumerate().map(|(k, f)| (k, Vec3::from(f.position()).distance(p))).min_by(|a, b| a.1.total_cmp(&b.1))?;
        if d > 40.0 {
            return None;
        }
        let f = self.sim.frames[k].axis(2);
        Some(f[0].atan2(f[2]).to_degrees())
    }

    fn gate_under(&self, m: Vec2) -> Option<u32> {
        let (o, d) = camera::ray(&self.vp, self.vsize(), m);
        self.gates
            .iter()
            .filter_map(|g| g.gate.boxes.iter().filter_map(|b| ray_box(o, d, Vec3::from(b.min), Vec3::from(b.max))).fold(None, |a: Option<f32>, t| Some(a.map_or(t, |a| a.min(t)))).map(|t| (g.gate.waypoint, t)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
    }

    pub fn object_under_pub(&self, m: Vec2) -> Option<Sel> {
        self.object_under(m)
    }

    fn object_under(&self, m: Vec2) -> Option<Sel> {
        let size = self.vsize();
        let mut best: Option<(Sel, f32)> = None;
        let mut consider = |s: Sel, p: Vec3, r: f32| {
            if let Some(q) = camera::project(&self.vp, size, p) {
                let d = q.distance(m);
                if d < r && best.is_none_or(|b| d < b.1) {
                    best = Some((s, d));
                }
            }
        };
        for g in self.doc.goals.iter().filter(|g| g.visible) {
            match &g.kind {
                GoalKind::Point { pos, .. } => {
                    consider(Sel::Goal(g.id), Vec3::from(*pos), 14.0);
                    consider(Sel::Goal(g.id), Vec3::from(*pos) + Vec3::Y * 3.4, 16.0);
                }
                GoalKind::Pose(p) => consider(Sel::Goal(g.id), Vec3::from(p.pos) + Vec3::Y * 0.5, 26.0),
                GoalKind::Box { pos, .. } => consider(Sel::Goal(g.id), Vec3::from(*pos), 30.0),
                GoalKind::Gate(_) => {}
            }
        }
        if let Some(s) = &self.doc.start {
            consider(Sel::Start, Vec3::from(s.pos) + Vec3::Y * 0.5, 26.0);
        }
        if self.show.setups {
            for st in &self.doc.setups {
                consider(Sel::Setup(st.id), Vec3::from(st.start.pos) + Vec3::Y * 0.5, 24.0);
            }
        }
        if let Some(b) = best {
            return Some(b.0);
        }
        let w = self.gate_under(m)?;
        match self.doc.goals.iter().find(|g| matches!(&g.kind, GoalKind::Gate(x) if x.has(w))) {
            Some(g) => Some(Sel::Goal(g.id)),
            None => Some(Sel::Gate(w)),
        }
    }

    fn viewport_down(&mut self, e: &MouseDownEvent, cx: &mut Context<Self>) {
        self.stats.input();
        self.commit_field(cx);
        self.popup = None;
        let m = self.local(e.position);
        self.mouse = m;
        match e.button {
            MouseButton::Right => {
                self.drag = Some(Drag::Fly { last: m, at: m, t0: Instant::now(), moved: false });
            }
            MouseButton::Middle | MouseButton::Left if e.button == MouseButton::Middle || e.modifiers.alt => {
                self.drag = Some(if e.modifiers.shift {
                    Drag::Pan { last: m }
                } else if e.modifiers.control {
                    Drag::Zoom { last: m, at: self.dolly_target(m) }
                } else {
                    Drag::Orbit { last: m, center: None }
                });
            }
            MouseButton::Left => {
                if self.velocity_hit(m)
                    && let Some(s) = &self.doc.start
                {
                    self.drag = Some(Drag::Velocity { vy: s.velocity[1] });
                    cx.notify();
                    return;
                }
                if let Some((gz, target)) = self.gizmo()
                    && let Some(h) = gz.hit(&self.vp, self.vsize(), m)
                    && let Some((pos0, rot0)) = self.target_pose(target)
                {
                    let scale = gz.kind == gizmo::GizmoMode::Scale;
                    let start = match h {
                        Handle::Axis(k) => gz.axis_param(k, &self.vp, self.vsize(), m),
                        Handle::Ring(k) => gz.ring_angle(k, &self.vp, self.vsize(), m),
                        Handle::Center if scale => crate::camera::project(&self.vp, self.vsize(), gz.center).map(|c| c.distance(m).max(4.0)),
                        Handle::Center => Some(0.0),
                    };
                    if let Some(start) = start {
                        let size0 = self.target_size(target).unwrap_or(Vec3::ONE);
                        self.drag = Some(Drag::Gizmo { handle: h, target, start, pos0, rot0, size0 });
                        cx.notify();
                        return;
                    }
                }
                self.drag = Some(Drag::Press { at: m, double: e.click_count >= 2 });
            }
            _ => {}
        }
        cx.notify();
    }

    pub fn viewport_move(&mut self, e: &MouseMoveEvent, cx: &mut Context<Self>) {
        let m = self.local(e.position);
        let size = self.vsize();
        let last_mouse = self.mouse;
        self.mouse = m;
        match self.drag.clone() {
            Some(Drag::Press { at, .. }) if at.distance(m) > 3.0 => {
                self.drag = Some(Drag::Orbit { last: m, center: None });
                self.camera.orbit(m - at);
            }
            Some(Drag::Orbit { last, center }) => {
                self.camera.orbit_about(m - last, center);
                self.drag = Some(Drag::Orbit { last: m, center });
            }
            Some(Drag::Zoom { last, at }) => {
                self.camera.dolly_drag(m.y - last.y, at);
                self.drag = Some(Drag::Zoom { last: m, at });
            }
            Some(Drag::Pan { last }) => {
                self.camera.pan(m - last, size.y);
                self.drag = Some(Drag::Pan { last: m });
            }
            Some(Drag::Fly { last, at, t0, moved }) => {
                let moved = moved || at.distance(m) > 4.0;
                if moved {
                    self.camera.look(m - last);
                }
                self.drag = Some(Drag::Fly { last: m, at, t0, moved });
            }
            Some(Drag::Velocity { vy }) => self.drag_velocity(m, vy, e.modifiers.shift),
            Some(Drag::Gizmo { handle, target, start, size0, .. }) if self.tool == Tool::Scale => {
                if let Some((gz, _)) = self.gizmo() {
                    let point = matches!(target, GizmoTarget::Goal(id) if matches!(self.doc.goal(id).map(|g| &g.kind), Some(GoalKind::Point { .. })));
                    let snap = |x: f32| if e.modifiers.shift { ((x * 2.0).round() / 2.0).max(0.5) } else { x };
                    let f = match handle {
                        Handle::Axis(k) => gz.axis_param(k, &self.vp, size, m).filter(|_| start.abs() > 1e-3).map(|t| (k, (t / start).max(0.02))),
                        Handle::Center => crate::camera::project(&self.vp, size, gz.center).map(|c| (3, (c.distance(m) / start).max(0.02))),
                        Handle::Ring(_) => None,
                    };
                    if let Some((k, f)) = f {
                        let s = if k == 3 || point { (size0 * f).map(snap) } else { let mut s = size0; s[k] = snap(size0[k] * f); s };
                        self.set_target_size(target, s);
                    }
                }
            }
            Some(Drag::Gizmo { handle, target, start, pos0, rot0, .. }) => {
                if let Some((gz, _)) = self.gizmo() {
                    match handle {
                        Handle::Axis(k) => {
                            let gz0 = Gizmo { center: pos0 + (gz.center - self.target_pose(target).map_or(pos0, |t| t.0)), ..gz };
                            if let Some(t) = gz0.axis_param(k, &self.vp, size, m) {
                                let mut step = t - start;
                                if e.modifiers.shift {
                                    step = (step * 10.0).round() / 10.0;
                                }
                                self.set_target_pose(target, pos0 + Gizmo::AXES[k] * step, rot0);
                            }
                        }
                        Handle::Ring(k) => {
                            let gz0 = Gizmo { rot: rot0, ..gz };
                            if let Some(a) = gz0.ring_angle(k, &self.vp, size, m) {
                                let mut d = a - start;
                                if e.modifiers.shift {
                                    d = (d.to_degrees() / 5.0).round() * 5.0f32.to_radians();
                                }
                                let axis = gz0.ring_axis(k);
                                self.set_target_pose(target, pos0, (Quat::from_axis_angle(axis, d) * rot0).normalize());
                            }
                        }
                        Handle::Center => {
                            let eye = self.camera.eye();
                            self.request_pick(m, Purpose::Car { eye, commit: false });
                        }
                    }
                }
            }
            Some(_) => {}
            None => {
                let in_view = m.x >= 0.0 && m.y >= 0.0 && m.x < size.x && m.y < size.y;
                if !in_view {
                    return;
                }
                let hv = self.velocity_hit(m);
                if hv != self.hover_velocity {
                    self.hover_velocity = hv;
                    self.dirty = true;
                }
                let h = if hv { None } else { self.gizmo().and_then(|(g, _)| g.hit(&self.vp, size, m)) };
                if h != self.hover_handle {
                    self.hover_handle = h;
                    self.dirty = true;
                }
                if m != last_mouse {
                    let eye = self.camera.eye();
                    match self.tool {
                        Tool::PlaceCar => self.request_pick(m, Purpose::Car { eye, commit: false }),
                        _ => self.request_pick(m, Purpose::Hover),
                    }
                    let g = if self.tool == Tool::GateGoal || self.tool == Tool::Select { self.gate_under(m) } else { None };
                    if g != self.hover_gate {
                        self.hover_gate = g;
                        self.dirty = true;
                    }
                }
            }
        }
        if self.drag.is_some() {
            self.stats.input();
            self.dirty = true;
        }
        cx.notify();
    }

    pub fn viewport_up(&mut self, e: &MouseUpEvent, _window: &mut Window, cx: &mut Context<Self>) {
        let drag = self.drag.take();
        if let Some(Drag::Fly { at, t0, moved, .. }) = drag {
            let flew = moved || self.fly_keys.iter().any(|k| *k);
            self.fly_keys = [false; 6];
            if !flew && t0.elapsed().as_millis() < 400 {
                self.open_context_menu(at);
            }
        }
        if let Some(Drag::Press { at, double, .. }) = drag {
            self.click(at, double, e.modifiers.shift);
        }
        if let Some(Drag::Gizmo { target: GizmoTarget::Goal(id), .. }) = drag {
            self.auto_search(id);
        }
        cx.notify();
    }

    pub fn auto_search(&mut self, id: u32) {
        if !self.spec.auto || self.origin.is_none() {
            return;
        }
        let aimed = self.aimed_goals();
        if aimed.is_empty() || self.spec.window.is_empty() {
            return self.search_toward(id);
        }
        if !aimed.iter().any(|g| g.id == id) {
            let at = |g: u32| self.doc.goals.iter().position(|x| x.id == g);
            let i = at(id);
            let before = self.spec.goals.iter().position(|&g| at(g) > i).unwrap_or(self.spec.goals.len());
            self.spec.goals.insert(before, id);
            if let Some(k) = i.and_then(|i| self.reach.get(i)).and_then(|r| r.tick.or(r.closest.map(|c| c.0)))
                && k > self.spec.window.end
            {
                self.spec.window.end = k;
            }
        }
        if self.search.running() {
            return;
        }
        let names = self.aimed_goals().iter().map(|g| g.name.clone()).collect::<Vec<_>>().join(" \u{2192} ");
        self.job = format!("Search to {names}");
        self.start_search();
    }

    fn click(&mut self, at: Vec2, double: bool, shift: bool) {
        let eye = self.camera.eye();
        if double && self.tool == Tool::Line {
            self.set_tool(Tool::Select);
            return;
        }
        if double {
            self.request_pick(at, Purpose::Pivot);
            return;
        }
        match self.tool {
            Tool::Line => self.request_pick(at, Purpose::Place { what: PlaceWhat::Line, eye }),
            Tool::PlaceCar => self.request_pick(at, Purpose::Car { eye, commit: true }),
            Tool::PointGoal => self.request_pick(at, Purpose::Place { what: PlaceWhat::Point, eye }),
            Tool::BoxGoal => self.request_pick(at, Purpose::Place { what: PlaceWhat::Box, eye }),
            Tool::PoseGoal => self.request_pick(at, Purpose::Place { what: PlaceWhat::Pose, eye }),
            Tool::GateGoal => match self.gate_under(at) {
                Some(w) => {
                    if let Some(gate) = self.gate_goal(w) {
                        if self.doc.goals.iter().any(|x| matches!(&x.kind, GoalKind::Gate(gg) if gg.has(w))) {
                            self.log(LogLevel::Warn, "goal", format!("{} is already a goal", self.gate_name(w)));
                        } else {
                            let id = self.add_goal(GoalKind::Gate(gate));
                            self.auto_search(id);
                        }
                    }
                }
                None => self.log(LogLevel::Warn, "goal", "no checkpoint or finish trigger under the cursor"),
            },
            Tool::Select | Tool::Move | Tool::Rotate | Tool::Scale => {
                match self.object_under(at) {
                    Some(s) => self.select(s),
                    None if !shift => self.select(Sel::None),
                    None => {}
                }
            }
        }
    }

    fn viewport_scroll(&mut self, e: &ScrollWheelEvent, cx: &mut Context<Self>) {
        let d = e.delta.pixel_delta(px(16.0));
        let (dx, dy) = (f32::from(d.x), f32::from(d.y));
        self.mouse = self.local(e.position);
        if matches!(self.drag, Some(Drag::Fly { .. })) {
            self.camera.fly_speed_steps(dy.signum());
            let s = self.camera.fly_speed;
            self.info("view", format!("fly speed {s:.1} m/s"));
        } else if e.delta.precise() && !e.modifiers.platform && !e.modifiers.control {
            if e.modifiers.alt != self.layout.trackpad_orbit {
                self.camera.orbit(Vec2::new(-dx, -dy) * 0.8);
            } else {
                self.camera.pan(Vec2::new(dx, dy), self.vsize().y);
            }
        } else {
            let steps = if e.delta.precise() { dy / 40.0 } else { dy / 16.0 };
            let at = self.dolly_target(self.mouse);
            self.camera.dolly(steps, at);
        }
        self.stats.input();
        self.dirty = true;
        cx.notify();
    }

    fn viewport_pinch(&mut self, e: &PinchEvent, cx: &mut Context<Self>) {
        self.mouse = self.local(e.position);
        let at = self.dolly_target(self.mouse);
        self.camera.dolly(e.delta * 9.0, at);
        self.stats.input();
        self.dirty = true;
        cx.notify();
    }

    fn dolly_target(&self, m: Vec2) -> Option<Vec3> {
        if let Some((at, p, t)) = self.hover_hit
            && at.distance(m) < 4.0
            && t.elapsed().as_secs_f32() < 2.0
        {
            return Some(p);
        }
        let (o, d) = camera::ray(&self.vp, self.vsize(), m);
        camera::ray_plane(o, d, self.camera.goal.pivot, self.camera.goal.dir())
    }

    pub fn fly_input(&self) -> Option<(Vec3, camera::FlySpeed)> {
        matches!(self.drag, Some(Drag::Fly { .. })).then(|| {
            let k = self.fly_keys;
            let d = Vec3::new((k[3] as i8 - k[2] as i8) as f32, (k[4] as i8 - k[5] as i8) as f32, (k[0] as i8 - k[1] as i8) as f32);
            (d, if self.shift_down { camera::FlySpeed::Fast } else if self.alt_down { camera::FlySpeed::Slow } else { camera::FlySpeed::Normal })
        })
    }

    pub fn advance_unpainted(&mut self) {
        let now = Instant::now();
        let dt = self.last_paint.map_or(1.0 / 120.0, |t| (now - t).as_secs_f32());
        self.last_paint = Some(now);
        self.playback();
        self.camera.update(dt, self.car_focus(), self.fly_input());
    }

    fn paint_viewport(&mut self, bounds: Bounds<Pixels>, hitbox: &Hitbox, window: &mut Window, cx: &mut Context<Self>) {
        let start = Instant::now();
        self.viewport = bounds;
        self.scale = window.scale_factor();
        let size = self.vsize();
        let now = Instant::now();
        let dt = self.last_paint.map_or(1.0 / 120.0, |t| (now - t).as_secs_f32());
        self.last_paint = Some(now);
        let car = self.display_frame();
        let moving = self.camera.update(dt, self.car_focus(), self.fly_input());
        let aspect = size.x / size.y.max(1.0);
        let vp = self.camera.view_proj(aspect);
        let (eye, vp_rel) = self.camera.view_proj_rel(aspect);
        self.vp = vp;
        if (self.show.physics || self.show.contacts)
            && self.contacts.as_ref().is_none_or(|c| c.0 != self.cursor)
            && self.contacts_requested != Some(self.cursor)
            && let (Some(f), Some(pk)) = (self.sim.frames.get(self.cursor), &self.picker)
        {
            let down = -Vec3::from(f.axis(1));
            for (i, c) in wheel_centers(f).iter().enumerate() {
                pk.request(*c - down * 0.5, down, Purpose::Wheel { tick: self.cursor, index: i as u8 });
            }
            self.contacts_requested = Some(self.cursor);
        }
        let animating = self.playing || self.drag.is_some() || moving;
        if animating {
            window.request_animation_frame();
        }
        if !(self.dirty || self.last_output.is_none() || animating) && self.present_pending {
            let (out, pending) = self.scene.borrow_mut().renderer.latest();
            if let Some(o) = out {
                self.last_output = Some(o);
            }
            self.present_pending = pending;
            if pending {
                window.request_animation_frame();
            }
        } else if self.dirty || self.last_output.is_none() || animating {
            self.dirty = false;
            let gizmo = self.gizmo();
            let active = match &self.drag {
                Some(Drag::Gizmo { handle, .. }) => Some(*handle),
                _ => self.hover_handle,
            };
            let reached: Vec<bool> = self.reach.iter().map(|r| r.tick.is_some()).collect();
            let preview_frames = self.preview.and_then(|id| self.runs_frames.get(&crate::aux::Key::Candidate(id)).cloned());
            let b_frames = match self.compare.1 {
                RunRef::Ghost(_) | RunRef::Current => None,
                r if self.layout.bottom_tab == 1 => {
                    let key = match r {
                        RunRef::Run(id) => Some(crate::aux::Key::Run(id)),
                        RunRef::Previous => self.doc.history.last().map(|h| crate::aux::Key::Run(h.id)),
                        RunRef::Candidate(id) => Some(crate::aux::Key::Candidate(id)),
                        _ => None,
                    };
                    key.and_then(|k| self.runs_frames.get(&k).cloned())
                }
                _ => None,
            };
            let probe_branches = self.sim.probe.as_ref().filter(|p| self.show.branches && p.tick == self.cursor).map(|p| p.branches.clone());
            let mut scene = self.scene.borrow_mut();
            let mut refs = Vec::new();
            if let Some(f) = &preview_frames {
                refs.push(RefRun { frames: f, alpha: 230, car: true });
            }
            if let Some(f) = &b_frames {
                refs.push(RefRun { frames: f, alpha: 150, car: true });
            }
            let pivot = matches!(self.drag, Some(Drag::Orbit { .. }) | Some(Drag::Pan { .. })).then_some(self.camera.cur.pivot);
            let input = SceneInput {
                frames: &self.sim.frames,
                frames_gen: self.frames_gen,
                goals: &self.doc.goals,
                guides: &self.doc.guides,
                selected_guide: if let Sel::Guide(id) = self.sel { Some(id) } else { None },
                reached: &reached,
                selected_goal: if let Sel::Goal(id) = self.sel { Some(id) } else { None },
                start: self.doc.start.as_ref(),
                start_selected: self.sel == Sel::Start,
                car_preview: self.car_preview.as_ref(),
                gates: &self.gates,
                show_gates: self.show.gates,
                hover_gate: self.hover_gate.filter(|_| matches!(self.tool, Tool::GateGoal | Tool::Select)),
                selected_gate: if let Sel::Gate(w) = self.sel { Some(w) } else { None },
                gizmo: gizmo.as_ref().map(|g| (&g.0, active)),
                ghosts: if self.show.ghosts { self.ghosts.iter().filter(|g| g.visible).map(|g| &g.track).collect() } else { Vec::new() },
                refs,
                tries: if self.show.tries { &self.tries } else { &[] },
                branches: probe_branches.as_deref(),
                pivot,
                hover: None,
                trail_width: (self.camera.cur.distance * 0.0045).max(0.6),
                show_trail: self.show.trail,
                trail_from: self.start_tick().unwrap_or(0),
                physics: self.show.physics,
                surroundings: self.show.stadium,
                view_proj: vp,
                car,
                contacts: if self.show.contacts || self.show.physics { self.contacts.as_ref().filter(|c| c.0 == self.cursor).map(|c| c.1) } else { None },
                start_handle: (self.sel == Sel::Start).then_some(self.hover_velocity || matches!(self.drag, Some(Drag::Velocity { .. }))),
                tricks: if self.show.tricks { &self.trick_spans } else { &[] },
                setups: if self.show.setups { self.doc.setups.iter().map(|st| (st, self.sel == Sel::Setup(st.id))).collect() } else { Vec::new() },
                stretch: (self.sel == Sel::Route).then(|| self.route_ticks()).flatten(),
            };
            let draw = scene.draw_list(&input);
            let dev = [(size.x * self.scale).round() as u32, (size.y * self.scale).round() as u32];
            let realistic = !self.show.physics && self.visual_state.is_none();
            let view = View { view_proj: vp_rel, eye, fog: [900.0, 4200.0], exposure: if realistic { 0.66 } else { 1.0 } };
            match scene.renderer.render(dev, &view, &draw) {
                Ok((out, times)) => {
                    if let (Some(Output::Image(old)), Output::Image(_)) = (&self.last_output, &out) {
                        let _ = window.drop_image(old.clone());
                    }
                    self.last_output = Some(out);
                    drop(scene);
                    self.dirty |= times.skipped;
                    self.present_pending = times.pending;
                    if times.pending {
                        window.request_animation_frame();
                    }
                    self.stats.painted(start, &times, animating);
                }
                Err(e) => {
                    drop(scene);
                    self.error("view", format!("render: {e:#}"));
                }
            }
        } else {
            self.stats.idle();
        }
        match &self.last_output {
            Some(Output::Surface(buf)) => window.paint_surface(bounds, buf.clone()),
            Some(Output::Image(img)) => {
                let _ = window.paint_image(bounds, bounds, Default::default(), img.clone(), 0, false);
            }
            None => {}
        }
        let this = cx.entity();
        let hb = hitbox.clone();
        window.on_mouse_event({
            let this = this.clone();
            move |e: &MouseDownEvent, phase, window, cx| {
                if phase == DispatchPhase::Bubble && hb.is_hovered(window) {
                    this.update(cx, |app, cx| app.viewport_down(e, cx));
                }
            }
        });
        let hb = hitbox.clone();
        window.on_mouse_event({
            let this = this.clone();
            move |e: &MouseMoveEvent, phase, window, cx| {
                if phase == DispatchPhase::Bubble && hb.is_hovered(window) && this.read(cx).drag.is_none() {
                    this.update(cx, |app, cx| app.viewport_move(e, cx));
                }
            }
        });
        let hb = hitbox.clone();
        window.on_mouse_event({
            let this = this.clone();
            move |e: &ScrollWheelEvent, phase, window, cx| {
                if phase == DispatchPhase::Bubble && hb.is_hovered(window) {
                    this.update(cx, |app, cx| app.viewport_scroll(e, cx));
                }
            }
        });
        let hb = hitbox.clone();
        window.on_mouse_event(move |e: &PinchEvent, phase, window, cx| {
            if phase == DispatchPhase::Bubble && hb.is_hovered(window) {
                this.update(cx, |app, cx| app.viewport_pinch(e, cx));
            }
        });
    }

    pub fn viewport(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let this = cx.entity();
        let canvas_el = canvas(
            |bounds, window, _| window.insert_hitbox(bounds, HitboxBehavior::Normal),
            move |bounds, hitbox, window, cx| this.update(cx, |app, cx| app.paint_viewport(bounds, &hitbox, window, cx)),
        )
        .absolute()
        .size_full();
        let mut m = div().id("viewport").flex_1().min_h_0().w_full().relative().overflow_hidden().child(canvas_el);
        m = m.children(self.labels());
        if self.map.is_some() && self.open_error.is_none() {
            m = m.child(self.tool_rail(cx));
        }
        m = m.child(self.hud());
        if let Some(l) = &self.loading {
            m = m.child(div().absolute().size_full().flex().items_center().justify_center().child(muted(l.clone()).text_size(px(theme::FS_EMPH))));
        }
        if let Some(e) = &self.open_error {
            m = m.child(div().absolute().size_full().flex().items_center().justify_center().p(px(24.0)).child(open_error_card(e)));
        }
        div().size_full().flex().flex_col().child(self.view_header(cx)).child(m)
    }

    fn tool_rail(&mut self, cx: &mut Context<Self>) -> AnyElement {
        let mut col = div().id("tool-rail").occlude().absolute().top(px(theme::SP3)).left(px(theme::SP3)).p(px(theme::SP1)).flex().flex_col().gap(px(theme::SP1)).rounded(px(theme::R_CTL + 2.0)).bg(theme::alpha(theme::PANEL, 0.94)).border_1().border_color(c(theme::HAIRLINE));
        for (gi, group) in self.tool_groups().into_iter().enumerate() {
            if gi > 0 {
                col = col.child(div().h(px(1.0)).mx(px(theme::SP1)).my(px(2.0)).bg(c(theme::HAIRLINE)));
            }
            for t in group {
                let on = self.tool == t;
                col = col.child(
                    div()
                        .id(gpui::ElementId::Name(format!("tool-{}", t.key()).into()))
                        .relative()
                        .child(track(format!("tool-{}", t.key())))
                        .size(px(30.0))
                        .flex()
                        .items_center()
                        .justify_center()
                        .rounded(px(theme::R_CTL))
                        .cursor_pointer()
                        .when(on, |d| d.bg(c(theme::SELECT)))
                        .when(!on, |d| d.hover(|s| s.bg(c(theme::RAISED))))
                        .child(gpui::svg().path(t.icon()).size(px(16.0)).text_color(c(if on { theme::TEXT } else { theme::MUTED })))
                        .tooltip(tip(format!("{}  {}", t.label(), t.key()), Some(t.hint())))
                        .on_click(cx.listener(move |this, _, _, cx| {
                            this.set_tool(t);
                            cx.notify();
                        })),
                );
            }
        }
        col.into_any_element()
    }

    fn labels(&self) -> Vec<AnyElement> {
        let size = self.vsize();
        if size.x < 1.0 {
            return Vec::new();
        }
        let vp = self.vp;
        let mut out = Vec::new();
        let tag = |s: Vec2, text: String, color: u32, sel: bool| {
            div()
                .absolute()
                .left(px(s.x + 6.0))
                .top(px(s.y - 22.0))
                .px(px(theme::SP1))
                .py(px(1.0))
                .rounded(px(3.0))
                .bg(if sel { theme::alpha(color, 1.0) } else { theme::alpha(theme::PANEL, 0.9) })
                .border_1()
                .border_color(theme::alpha(color, if sel { 1.0 } else { 0.7 }))
                .text_color(c(if sel { theme::BG } else { theme::TEXT }))
                .font_family(theme::MONO)
                .text_size(px(theme::FS_SMALL))
                .child(text)
                .into_any_element()
        };
        let inside = |s: Vec2| s.x > 0.0 && s.y > 0.0 && s.x < size.x && s.y < size.y;
        for (i, g) in self.doc.goals.iter().enumerate().filter(|(_, g)| g.visible) {
            let top = g.pos() + Vec3::Y * if matches!(g.kind, GoalKind::Point { .. }) { 4.4 } else { 2.2 };
            if let Some(s) = camera::project(&vp, size, top).filter(|s| inside(*s)) {
                let r = self.reach.get(i);
                let t = r.and_then(|r| r.ms).map_or(String::new(), |ms| format!(" {}", fmt_ms(ms)));
                out.push(tag(s, format!("{}{t}", g.name), theme::PIN, self.sel == Sel::Goal(g.id)));
            }
        }
        if let Some(st) = &self.doc.start {
            let kmh = Vec3::from(st.velocity).length() * 3.6;
            if self.sel == Sel::Start {
                if let Some(s) = camera::project(&vp, size, crate::scene::velocity_tip(st)).filter(|s| inside(*s)) {
                    out.push(tag(s + Vec2::new(8.0, 0.0), format!("{kmh:.0} km/h"), theme::PIN, matches!(self.drag, Some(Drag::Velocity { .. })) || self.hover_velocity));
                }
            } else if let Some(s) = camera::project(&vp, size, Vec3::from(st.pos) + Vec3::Y * 2.0).filter(|s| inside(*s)) {
                out.push(tag(s, format!("Your car \u{00b7} {kmh:.0} km/h"), theme::PIN, false));
            }
        }
        if self.show.setups {
            for st in &self.doc.setups {
                if self.sel == Sel::Setup(st.id) {
                    continue;
                }
                if let Some(s) = camera::project(&vp, size, Vec3::from(st.start.pos) + Vec3::Y * 2.0).filter(|s| inside(*s)) {
                    out.push(tag(s, st.name.clone(), theme::SETUP, false));
                }
            }
        }
        if self.show.tricks && self.show.trail {
            for t in &self.trick_spans {
                let Some(f) = self.sim.frames.get(t.from) else { continue };
                if let Some(s) = camera::project(&vp, size, Vec3::from(f.position()) + Vec3::Y * 1.5).filter(|s| inside(*s)) {
                    out.push(tag(s, format!("{} {:.2} s", crate::lab::trick_label(t.kind), (t.to - t.from) as f32 / 100.0), crate::lab::trick_color(t.kind), false));
                }
            }
        }
        if self.sel == Sel::Route {
            let stops = self.route_stops();
            for (i, st) in stops.iter().enumerate() {
                let crate::lab::RouteAt::Gate(w) = st.at else { continue };
                let Some(b) = self.gates.iter().find(|g| g.gate.waypoint == w).and_then(|g| g.gate.boxes.first()) else { continue };
                let top = Vec3::new((b.min[0] + b.max[0]) * 0.5, b.max[1] + 1.0, (b.min[2] + b.max[2]) * 0.5);
                if let Some(s) = camera::project(&vp, size, top).filter(|s| inside(*s)) {
                    let mark = if self.route.from == crate::lab::RouteFrom::Stop(i) { "START \u{00b7} " } else if self.route.to == st.at { "END \u{00b7} " } else { "" };
                    let t = st.tick.map_or(String::new(), |k| format!(" {}", fmt_ms(self.race_ms0 + 10 * k as i32)));
                    out.push(tag(s, format!("{mark}{}{t}", st.name), if mark.is_empty() { theme::MUTED } else { theme::PIN }, !mark.is_empty()));
                }
            }
        }
        out
    }

    fn view_header(&mut self, cx: &mut Context<Self>) -> impl IntoElement {
        let realistic_ok = self.scene.borrow().visual.is_some();
        let follow = self.camera.mode == crate::camera::Mode::Follow;
        let physics = self.show.physics || !realistic_ok;
        type Item = (String, Option<String>, std::rc::Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>);
        let mode = segments()
            .child(segment("v-real", !physics, "Realistic").tooltip(tip("Realistic (P)", Some("The textured map and car."))).on_click(cx.listener(|e, _, _, cx| {
                e.show.physics = false;
                e.dirty = true;
                cx.notify();
            })))
            .child(segment("v-phys", physics, "Physics").tooltip(tip("Physics (P)", Some("The collision mesh coloured by surface; effect surfaces tinted."))).on_click(cx.listener(|e, _, _, cx| {
                e.show.physics = true;
                e.dirty = true;
                cx.notify();
            })));
        let show = div()
            .child(btn("v-show", "Show \u{25be}", Btn::Quiet).on_click(cx.listener(|e, ev: &gpui::ClickEvent, _, cx| {
                let s = &e.show;
                let on = |b: bool| b.then(|| "\u{2713}".to_string());
                let items: Vec<Item> = vec![
                    ("Trail (the run's path)".into(), on(s.trail), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.trail = !e.show.trail)),
                    ("Checkpoint and finish triggers".into(), on(s.gates), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.gates = !e.show.gates)),
                    ("Ghosts".into(), on(s.ghosts), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.ghosts = !e.show.ghosts)),
                    ("Search tries".into(), on(s.tries), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.tries = !e.show.tries)),
                    ("Tricks along the run".into(), on(s.tricks), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.tricks = !e.show.tricks)),
                    ("Setups".into(), on(s.setups), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.setups = !e.show.setups)),
                    ("Stadium surroundings  B".into(), on(s.stadium), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.stadium = !e.show.stadium)),
                    ("Wheel contacts  \u{21e7}C".into(), on(s.contacts), std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.show.contacts = !e.show.contacts)),
                ];
                e.popup = Some(Popup { at: ev.position(), items });
                e.dirty = true;
                cx.stop_propagation();
                cx.notify();
            })));
        let cam = div()
            .flex()
            .gap(px(2.0))
            .child(chip("c-follow", follow, "Follow").tooltip(tip("Follow the car (\u{21e7}F)", Some("Orbit around it with a middle drag or \u{2325}+swipe; zoom with the wheel or a pinch. Panning leaves follow."))).on_click(cx.listener(|e, _, _, cx| {
                let on = e.camera.mode != crate::camera::Mode::Follow;
                e.camera.set_follow(on);
                cx.notify();
            })))
            .child(btn("c-view", "View \u{25be}", Btn::Quiet).on_click(cx.listener(|e, ev: &gpui::ClickEvent, _, cx| {
                let mut items: Vec<Item> = vec![
                    ("Frame the track  Home".into(), None, std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.frame_all())),
                    ("Top  \u{2325}1".into(), None, std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.camera.axis_view(e.camera.goal.yaw, crate::camera::PITCH_LIMIT))),
                    ("Front  \u{2325}2".into(), None, std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.camera.axis_view(std::f32::consts::PI, 0.0))),
                    ("Right  \u{2325}3".into(), None, std::rc::Rc::new(|e: &mut Editor, _: &mut Context<Editor>| e.camera.axis_view(-std::f32::consts::FRAC_PI_2, 0.0))),
                ];
                for (i, b) in e.doc.bookmarks.iter().enumerate() {
                    items.push((format!("{}  \u{2303}{}", b.name, i + 1), None, std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| e.recall(i))));
                }
                let n = e.doc.bookmarks.len().min(4);
                items.push((format!("Save this view  \u{2303}\u{21e7}{}", n + 1), None, std::rc::Rc::new(move |e: &mut Editor, _: &mut Context<Editor>| e.save_view(n))));
                e.popup = Some(Popup { at: ev.position(), items });
                cx.stop_propagation();
                cx.notify();
            })));
        div()
            .id("view-header")
            .h(px(theme::H_BAR))
            .flex_none()
            .px(px(theme::SP2))
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .bg(c(theme::PANEL))
            .border_b_1()
            .border_color(c(theme::HAIRLINE))
            .child(mode)
            .child(show)
            .children(self.visual_state.clone().map(|s| div().min_w_0().truncate().text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(s)))
            .child(div().flex_1().min_w_0().flex().justify_center().children((self.tool != Tool::Select).then(|| {
                div().min_w_0().truncate().text_size(px(theme::FS_SMALL)).text_color(c(theme::MUTED)).child(format!("{} \u{00b7} {}", self.tool.label(), self.tool.hint()))
            })))
            .child(cam)
    }

    fn hud(&self) -> impl IntoElement {
        let f = self.display_frame();
        let speed = f.map_or(0.0, |f| f.speed_kmh());
        let t = f.map_or(self.race_ms0, |f| f.race_ms);
        let cam = match self.camera.mode {
            crate::camera::Mode::Follow => "follow".to_string(),
            _ if matches!(self.drag, Some(Drag::Fly { .. })) => format!("fly {:.1} m/s", self.camera.fly_speed),
            _ => format!("orbit \u{00b7} {:.0} m", self.camera.cur.distance),
        };
        let perf = cam;
        let hint = (self.tool != Tool::Select).then(|| format!("{} \u{00b7} {}  (Esc: back to Select)", self.tool.label(), self.tool.hint()));
        div()
            .absolute()
            .left(px(12.0))
            .bottom(px(10.0))
            .right(px(12.0))
            .flex()
            .items_end()
            .gap(px(theme::SP3))
            .child(div().flex().items_baseline().gap(px(theme::SP1)).child(mono(format!("{speed:>3.0}"), theme::FS_DISPLAY)).child(muted("km/h")).child(mono(format!("  {} s", fmt_ms(t)), theme::FS_BODY).text_color(c(theme::MUTED))))
            .child(div().flex_1().flex().justify_center().children(hint.map(|h| div().px(px(theme::PAD)).py(px(theme::SP1)).rounded(px(theme::R_CTL)).bg(theme::alpha(theme::PANEL, 0.9)).border_1().border_color(c(theme::PIN)).text_size(px(theme::FS_BODY)).font_weight(FontWeight::MEDIUM).child(h))))
            .child(mono(perf, theme::FS_SMALL).text_color(c(theme::MUTED)))
    }
}

pub fn surface_rot(yaw: f32, n: Vec3) -> Quat {
    let up = if n.length_squared() > 0.5 { n } else { Vec3::Y };
    let f0 = Vec3::new(yaw.to_radians().sin(), 0.0, yaw.to_radians().cos());
    let f = (f0 - up * f0.dot(up)).normalize_or_zero();
    let f = if f == Vec3::ZERO { Vec3::Z } else { f };
    let s = up.cross(f).normalize();
    Quat::from_mat3(&glam::Mat3::from_cols(s, up, f)).normalize()
}

pub fn wheel_centers(f: &crate::engine::frame::Frame) -> [Vec3; 4] {
    let ax = [Vec3::from(f.axis(0)), Vec3::from(f.axis(1)), Vec3::from(f.axis(2))];
    std::array::from_fn(|i| {
        let p = f.wheels[i].position;
        Vec3::from(f.position()) + ax[0] * p[0] + ax[1] * p[1] + ax[2] * p[2]
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn surface_rotation_keeps_the_heading_and_follows_the_normal() {
        let q = surface_rot(90.0, Vec3::Y);
        assert!(((q * Vec3::Z) - Vec3::X).length() < 1e-5);
        let n = Vec3::new(0.0, 1.0, -0.3).normalize();
        let q = surface_rot(0.0, n);
        assert!(((q * Vec3::Y) - n).length() < 1e-5);
        assert!((q * Vec3::Z).y > 0.0, "facing +Z up a slope that rises towards +Z");
    }

    #[test]
    fn ray_hits_a_box_in_front() {
        assert_eq!(ray_box(Vec3::ZERO, Vec3::Z, Vec3::new(-1.0, -1.0, 5.0), Vec3::new(1.0, 1.0, 6.0)), Some(5.0));
        assert_eq!(ray_box(Vec3::ZERO, Vec3::Z, Vec3::new(2.0, -1.0, 5.0), Vec3::new(3.0, 1.0, 6.0)), None);
    }
}

fn open_error_card(e: &str) -> impl IntoElement {
    let (head, rest) = e.split_once(": ").unwrap_or((e, ""));
    let lines: Vec<&str> = rest.split("; ").filter(|l| !l.is_empty()).collect();
    div()
        .id("open-error")
        .max_w(px(720.0))
        .max_h_full()
        .overflow_y_scroll()
        .flex()
        .flex_col()
        .gap(px(theme::SP2))
        .p(px(16.0))
        .rounded(px(6.0))
        .bg(theme::alpha(theme::PANEL, 0.96))
        .border_1()
        .border_color(c(theme::ERROR))
        .child(text("Could not open this map").text_color(c(theme::ERROR)).text_size(px(theme::FS_TITLE)))
        .child(muted(head.to_string()).text_size(px(theme::FS_BODY)))
        .children(lines.into_iter().map(|l| mono(l.to_string(), theme::FS_BODY).text_color(c(theme::TEXT))))
        .child(muted("The rest of the editor stays usable: open another map with \u{2318}O.").text_size(px(theme::FS_BODY)))
}
