use glam::{Mat4, Vec2, Vec3, Vec4Swizzles};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Free,
    Follow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlySpeed {
    Slow,
    Normal,
    Fast,
}

impl FlySpeed {
    fn factor(self) -> f32 {
        match self {
            Self::Slow => 0.25,
            Self::Normal => 1.0,
            Self::Fast => 4.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct View {
    pub pivot: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl View {
    pub fn dir(&self) -> Vec3 {
        Vec3::new(self.pitch.cos() * self.yaw.sin(), -self.pitch.sin(), self.pitch.cos() * self.yaw.cos())
    }
    pub fn eye(&self) -> Vec3 {
        self.pivot - self.dir() * self.distance
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Car {
    pub pos: Vec3,
    pub forward: Vec3,
    pub vel: Vec3,
}

#[derive(Clone, Copy, Debug)]
struct Follow {
    err: Vec3,
    err_v: Vec3,
    last: Vec3,
    heading: f32,
    heading_v: f32,
    goal: (f32, f32, f32),
    cur: (f32, f32, f32),
}

const POS_OMEGA: f32 = 9.0;
const HEADING_OMEGA: f32 = 6.0;
const RATE: f32 = 25.0;
const RATE_JUMP: f32 = 10.0;
pub const PITCH_LIMIT: f32 = 1.5607;
const MIN_DIST: f32 = 0.5;
const MAX_DIST: f32 = 12000.0;

#[derive(Clone, Debug)]
pub struct Camera {
    pub mode: Mode,
    pub goal: View,
    pub cur: View,
    pub fov_y: f32,
    rate: f32,
    follow: Option<Follow>,
    pub fly_speed: f32,
    fly_vel: Vec3,
    fly_held: f32,
    car: Option<Car>,
}

impl Default for Camera {
    fn default() -> Self {
        let v = View { pivot: Vec3::ZERO, yaw: 0.6, pitch: 0.55, distance: 400.0 };
        Self {
            mode: Mode::Free,
            goal: v,
            cur: v,
            fov_y: 50f32.to_radians(),
            rate: RATE,
            follow: None,
            fly_speed: 12.0,
            fly_vel: Vec3::ZERO,
            fly_held: 0.0,
            car: None,
        }
    }
}

fn wrap(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

fn frame(yaw: f32, pitch: f32) -> glam::Quat {
    glam::Quat::from_rotation_y(yaw) * glam::Quat::from_rotation_x(pitch)
}

fn heading(f: Vec3) -> f32 {
    f.x.atan2(f.z)
}

pub fn damp(rate: f32, dt: f32) -> f32 {
    1.0 - (-rate * dt).exp()
}

impl Camera {
    pub fn update(&mut self, dt: f32, car: Option<Car>, fly: Option<(Vec3, FlySpeed)>) -> bool {
        let dt = dt.clamp(0.0, 0.1);
        self.car = car;
        let held = fly.is_some_and(|f| f.0 != Vec3::ZERO);
        self.fly_held = if held { self.fly_held + dt } else { 0.0 };
        let ramp = 0.3 + 0.7 * (self.fly_held / 1.0).min(1.0);
        let want = fly.map_or(Vec3::ZERO, |(d, speed)| {
            let (f, r) = (self.cur.dir(), self.cur.dir().cross(Vec3::Y).normalize_or_zero());
            (r * d.x + Vec3::Y * d.y + f * d.z).normalize_or_zero() * self.fly_speed * ramp * speed.factor()
        });
        self.fly_vel += (want - self.fly_vel) * damp(12.0, dt);
        if self.fly_vel.length_squared() > 1e-6 {
            if self.mode == Mode::Follow {
                self.leave_follow();
            }
            let d = self.fly_vel * dt;
            self.goal.pivot += d;
            self.cur.pivot += d;
        }
        let mut moving = self.fly_vel.length_squared() > 1e-6;
        match (self.mode, car) {
            (Mode::Follow, Some(c)) => {
                let fl = self.follow.get_or_insert_with(|| Follow::from_view(&self.cur, c));
                moving |= fl.step(c, dt);
                self.cur = fl.view(c);
                self.goal = self.cur;
            }
            _ => {
                let a = damp(self.rate, dt);
                let g = self.goal;
                let c = &mut self.cur;
                c.pivot += (g.pivot - c.pivot) * a;
                c.yaw += wrap(g.yaw - c.yaw) * a;
                c.pitch += (g.pitch - c.pitch) * a;
                c.distance = (c.distance.ln() + (g.distance.ln() - c.distance.ln()) * a).exp();
                let settled = c.pivot.distance(g.pivot) < 1e-3 * g.distance.max(1.0)
                    && wrap(g.yaw - c.yaw).abs() < 1e-4
                    && (g.pitch - c.pitch).abs() < 1e-4
                    && (c.distance / g.distance - 1.0).abs() < 1e-4;
                if settled {
                    *c = g;
                    self.rate = RATE;
                }
                moving |= !settled;
            }
        }
        moving
    }

    pub fn eye(&self) -> Vec3 {
        self.cur.eye()
    }

    fn proj(&self, aspect: f32) -> Mat4 {
        let near = (self.cur.distance * 0.01).clamp(0.05, 0.5);
        glam::camera::rh::proj::directx::perspective_infinite_reverse(self.fov_y, aspect.max(0.01), near)
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let eye = self.cur.eye();
        self.proj(aspect) * glam::camera::rh::view::look_at_mat4(eye, eye + self.cur.dir(), Vec3::Y)
    }

    pub fn view_proj_rel(&self, aspect: f32) -> (glam::DVec3, Mat4) {
        let eye = self.cur.eye();
        (
            eye.as_dvec3(),
            self.proj(aspect) * glam::camera::rh::view::look_at_mat4(Vec3::ZERO, self.cur.dir(), Vec3::Y),
        )
    }

    pub fn orbit(&mut self, d: Vec2) {
        self.orbit_about(d, None);
    }

    pub fn orbit_about(&mut self, d: Vec2, center: Option<Vec3>) {
        let (dy, dp) = (-d.x * 0.0055, d.y * 0.0055);
        if self.mode == Mode::Follow
            && let Some(f) = &mut self.follow
        {
            f.goal.0 += dy;
            f.goal.1 = (f.goal.1 + dp).clamp(-0.6, 1.45);
            return;
        }
        let (yaw, pitch) = (self.goal.yaw, self.goal.pitch);
        self.goal.yaw += dy;
        self.goal.pitch = (pitch + dp).clamp(-PITCH_LIMIT, PITCH_LIMIT);
        if let Some(c) = center {
            let r = frame(self.goal.yaw, self.goal.pitch) * frame(yaw, pitch).inverse();
            self.goal.pivot = c + r * (self.goal.pivot - c);
        }
        self.direct();
    }

    fn direct(&mut self) {
        self.cur = self.goal;
        self.rate = RATE;
    }

    pub fn dolly_drag(&mut self, dy: f32, at: Option<Vec3>) {
        self.dolly(-dy / 25.0, at);
    }

    pub fn pan(&mut self, d: Vec2, h: f32) {
        if self.mode == Mode::Follow {
            self.leave_follow();
        }
        let v = self.goal;
        let f = v.dir();
        let right = f.cross(Vec3::Y).normalize_or_zero();
        let up = right.cross(f);
        let per_pt = 2.0 * v.distance * (self.fov_y * 0.5).tan() / h.max(1.0);
        self.goal.pivot += (-right * d.x + up * d.y) * per_pt;
        self.direct();
    }

    pub fn dolly(&mut self, steps: f32, at: Option<Vec3>) {
        let f = 0.88f32.powf(steps);
        if self.mode == Mode::Follow
            && let Some(fl) = &mut self.follow
        {
            fl.goal.2 = (fl.goal.2 * f).clamp(2.5, 120.0);
            return;
        }
        let v = self.goal;
        let raw = v.distance * f;
        if raw < 1.0 && steps > 0.0 && at.is_none() {
            self.goal.pivot += v.dir() * (v.distance - raw).max(0.25);
            self.direct();
            return;
        }
        let nd = raw.clamp(MIN_DIST, MAX_DIST);
        let f = nd / v.distance;
        if let Some(p) = at {
            self.goal.pivot = p + (v.pivot - p) * f;
        }
        self.goal.distance = nd;
        self.direct();
    }

    pub fn look(&mut self, d: Vec2) {
        if self.mode == Mode::Follow {
            self.leave_follow();
        }
        let eye = self.cur.eye();
        let mut v = self.cur;
        v.yaw -= d.x * 0.0035;
        v.pitch = (v.pitch + d.y * 0.0035).clamp(-PITCH_LIMIT, PITCH_LIMIT);
        v.distance = v.distance.clamp(1.0, 60.0);
        v.pivot = eye + v.dir() * v.distance;
        self.cur = v;
        self.goal = v;
    }

    pub fn fly_speed_steps(&mut self, steps: f32) {
        self.fly_speed = (self.fly_speed * 1.25f32.powf(steps)).clamp(0.5, 1000.0);
    }

    pub fn go(&mut self, v: View) {
        if self.mode == Mode::Follow {
            self.leave_follow();
        }
        self.goal = View {
            pitch: v.pitch.clamp(-PITCH_LIMIT, PITCH_LIMIT),
            distance: v.distance.clamp(MIN_DIST, MAX_DIST),
            ..v
        };
        self.goal.yaw = self.cur.yaw + wrap(self.goal.yaw - self.cur.yaw);
        self.rate = RATE_JUMP;
    }

    pub fn focus(&mut self, center: Vec3, radius: f32) {
        let d = (radius.max(1.0) / (self.fov_y * 0.5).sin()) * 1.1;
        self.go(View { pivot: center, distance: d.max(3.0), ..self.goal });
    }

    pub fn frame(&mut self, min: Vec3, max: Vec3) {
        let r = (max - min).length() * 0.5;
        self.go(View {
            pivot: (min + max) * 0.5,
            distance: (r / (self.fov_y * 0.5).tan()).clamp(20.0, MAX_DIST) * 0.95,
            pitch: 0.75,
            yaw: self.goal.yaw,
        });
    }

    pub fn axis_view(&mut self, yaw: f32, pitch: f32) {
        self.go(View { yaw, pitch, ..self.goal });
    }

    pub fn look_at(&mut self, eye: Vec3, target: Vec3) {
        self.set_follow(false);
        let d = target - eye;
        let distance = d.length().max(MIN_DIST);
        let d = d / distance;
        self.snap_to(View {
            pivot: target,
            yaw: d.x.atan2(d.z),
            pitch: (-d.y).clamp(-1.0, 1.0).asin(),
            distance,
        });
    }

    pub fn snap_to(&mut self, v: View) {
        self.goal = v;
        self.cur = v;
    }

    pub fn set_follow(&mut self, on: bool) {
        match on {
            true if self.mode != Mode::Follow => {
                self.mode = Mode::Follow;
                self.follow = self.car.map(|c| Follow::from_view(&self.cur, c));
            }
            false if self.mode == Mode::Follow => self.leave_follow(),
            _ => {}
        }
    }

    fn leave_follow(&mut self) {
        self.mode = Mode::Free;
        self.follow = None;
        self.goal = self.cur;
        self.rate = RATE;
    }
}

impl Follow {
    fn from_view(v: &View, c: Car) -> Self {
        let h = heading(c.forward);
        let target = c.pos + Vec3::Y * 1.0;
        Self {
            err: v.pivot - target,
            err_v: Vec3::ZERO,
            last: c.pos,
            heading: h,
            heading_v: 0.0,
            goal: (0.0, 0.3, 11.0),
            cur: (wrap(v.yaw - h), v.pitch.clamp(-0.6, 1.45), v.distance),
        }
    }

    fn step(&mut self, c: Car, dt: f32) -> bool {
        let expected = self.last + c.vel * dt;
        self.err += expected - c.pos;
        self.last = c.pos;
        let h = heading(c.forward);
        let mut e = wrap(self.heading - h);
        let n = (dt * 240.0).ceil().max(1.0);
        let sdt = dt / n;
        for _ in 0..n as usize {
            let a = -POS_OMEGA * POS_OMEGA * self.err - 2.0 * POS_OMEGA * self.err_v;
            self.err_v += a * sdt;
            self.err += self.err_v * sdt;
            let ha = -HEADING_OMEGA * HEADING_OMEGA * e - 2.0 * HEADING_OMEGA * self.heading_v;
            self.heading_v += ha * sdt;
            e += self.heading_v * sdt;
        }
        self.heading = wrap(h + e);
        let k = damp(10.0, dt);
        self.cur.0 += wrap(self.goal.0 - self.cur.0) * k;
        self.cur.1 += (self.goal.1 - self.cur.1) * k;
        self.cur.2 += (self.goal.2 - self.cur.2) * k;
        self.err.length() > 1e-3
            || self.heading_v.abs() > 1e-4
            || (self.goal.2 - self.cur.2).abs() > 1e-3
            || wrap(self.goal.0 - self.cur.0).abs() > 1e-4
    }

    fn view(&self, c: Car) -> View {
        View {
            pivot: c.pos + Vec3::Y * 1.0 + self.err,
            yaw: self.heading + self.cur.0,
            pitch: self.cur.1,
            distance: self.cur.2,
        }
    }
}

pub fn project(vp: &Mat4, size: Vec2, p: Vec3) -> Option<Vec2> {
    let c = *vp * p.extend(1.0);
    if c.w <= 0.01 {
        return None;
    }
    let n = c.xyz() / c.w;
    Some(Vec2::new((n.x * 0.5 + 0.5) * size.x, (0.5 - n.y * 0.5) * size.y))
}

pub fn ray(vp: &Mat4, size: Vec2, s: Vec2) -> (Vec3, Vec3) {
    let inv = vp.inverse();
    let nx = s.x / size.x * 2.0 - 1.0;
    let ny = 1.0 - s.y / size.y * 2.0;
    let a = inv * glam::Vec4::new(nx, ny, 1.0, 1.0);
    let b = inv * glam::Vec4::new(nx, ny, 0.5, 1.0);
    let (a, b) = (a.xyz() / a.w, b.xyz() / b.w);
    (a, (b - a).normalize_or_zero())
}

pub fn ray_plane(o: Vec3, d: Vec3, p: Vec3, n: Vec3) -> Option<Vec3> {
    let den = d.dot(n);
    if den.abs() < 1e-6 {
        return None;
    }
    let t = (p - o).dot(n) / den;
    (t > 0.0).then(|| o + d * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(c: &mut Camera, car: Option<Car>, fps: f32, secs: f32) {
        for _ in 0..(fps * secs) as usize {
            c.update(1.0 / fps, car, None);
        }
    }

    #[test]
    fn animated_jumps_are_frame_rate_independent() {
        let run = |fps: f32| {
            let mut c = Camera::default();
            c.snap_to(View { pivot: Vec3::ZERO, yaw: 0.0, pitch: 0.5, distance: 100.0 });
            c.go(View { pivot: Vec3::ZERO, yaw: 1.0, pitch: 0.5, distance: 100.0 });
            for _ in 0..(fps * 0.2).round() as usize {
                c.update(1.0 / fps, None, None);
            }
            c.cur.yaw
        };
        let (a, b) = (run(30.0), run(240.0));
        assert!((a - b).abs() < 0.02, "{a} vs {b}");
        assert!(a > 0.8 && a < 1.0, "most of the way after 0.2 s: {a}");
    }

    #[test]
    fn hand_moves_do_not_drift() {
        let mut c = Camera::default();
        c.snap_to(View { pivot: Vec3::ZERO, yaw: 0.0, pitch: 0.5, distance: 100.0 });
        c.orbit(Vec2::new(-200.0, 0.0));
        c.pan(Vec2::new(30.0, 0.0), 600.0);
        let shown = c.cur;
        assert_eq!(shown, c.goal);
        for _ in 0..60 {
            c.update(1.0 / 60.0, None, None);
        }
        assert_eq!(c.cur, shown);
    }

    #[test]
    fn dolly_keeps_the_point_under_the_cursor() {
        let mut c = Camera::default();
        c.snap_to(View { pivot: Vec3::ZERO, yaw: 0.0, pitch: 0.4, distance: 100.0 });
        let p = Vec3::new(20.0, 0.0, 30.0);
        let (eye0, dir0) = (c.goal.eye(), (p - c.goal.eye()).normalize());
        c.dolly(3.0, Some(p));
        let (eye1, dir1) = (c.goal.eye(), (p - c.goal.eye()).normalize());
        assert!(eye1.distance(p) < eye0.distance(p) * 0.8);
        assert!(dir0.dot(dir1) > 0.99999, "the point stays on the same ray");
        assert!(
            (c.goal.dir() - View { pivot: Vec3::ZERO, yaw: 0.0, pitch: 0.4, distance: 1.0 }.dir()).length()
                < 1e-6,
            "no rotation"
        );
    }

    #[test]
    fn follow_starts_from_the_shown_view_and_settles_behind_the_car() {
        let mut c = Camera::default();
        let start = View { pivot: Vec3::new(50.0, 10.0, 0.0), yaw: 2.0, pitch: 0.8, distance: 300.0 };
        c.snap_to(start);
        let car = Car { pos: Vec3::new(0.0, 0.0, 0.0), forward: Vec3::Z, vel: Vec3::ZERO };
        c.update(1.0 / 120.0, Some(car), None);
        c.set_follow(true);
        c.update(1.0 / 120.0, Some(car), None);
        assert!(c.cur.pivot.distance(start.pivot) < 2.0, "{:?}", c.cur.pivot);
        assert!((c.cur.distance - 300.0).abs() < 30.0, "{}", c.cur.distance);
        settle(&mut c, Some(car), 120.0, 3.0);
        assert!(c.cur.pivot.distance(Vec3::Y) < 0.01);
        assert!(wrap(c.cur.yaw).abs() < 0.01 && (c.cur.distance - 11.0).abs() < 0.05);
    }

    #[test]
    fn follow_tracks_steady_motion_without_lag() {
        let mut c = Camera::default();
        c.set_follow(true);
        let mut car = Car { pos: Vec3::ZERO, forward: Vec3::Z, vel: Vec3::new(0.0, 0.0, 80.0) };
        c.update(1.0 / 60.0, Some(car), None);
        c.set_follow(true);
        for _ in 0..600 {
            car.pos += car.vel / 60.0;
            c.update(1.0 / 60.0, Some(car), None);
        }
        assert!(c.cur.pivot.distance(car.pos + Vec3::Y) < 0.05, "{:?} vs {:?}", c.cur.pivot, car.pos);
    }

    #[test]
    fn fly_starts_slow_then_reaches_the_set_speed() {
        let mut c = Camera::default();
        c.snap_to(View { pivot: Vec3::ZERO, yaw: 0.0, pitch: 0.0, distance: 10.0 });
        c.fly_speed = 10.0;
        let step = |c: &mut Camera| c.update(1.0 / 120.0, None, Some((Vec3::Z, FlySpeed::Normal)));
        let e0 = c.eye();
        for _ in 0..30 {
            step(&mut c);
        }
        let tap = (c.eye() - e0).z;
        assert!(tap > 0.2 && tap < 1.2, "{tap}");
        for _ in 0..240 {
            step(&mut c);
        }
        let e1 = c.eye();
        step(&mut c);
        let v = (c.eye() - e1) * 120.0;
        assert!((v.z - 10.0).abs() < 0.1 && v.x.abs() < 1e-3, "{v:?}");
    }
}
