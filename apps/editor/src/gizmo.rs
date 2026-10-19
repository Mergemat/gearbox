use glam::{Mat4, Quat, Vec2, Vec3};

use crate::camera::{project, ray};
use crate::gfx::mesh::{DrawList, Vertex};
use crate::theme;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Handle {
    Axis(usize),
    Center,
    Ring(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GizmoMode {
    Translate,
    Rotate,
    Lift,
    Scale,
}

pub struct Gizmo {
    pub center: Vec3,
    pub scale: f32,
    pub kind: GizmoMode,
    pub rot: Quat,
    pub full_rotation: bool,
}

const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

fn axis_rgb(k: usize) -> [u8; 3] {
    theme::rgb8([theme::AXIS_X, theme::AXIS_Y, theme::AXIS_Z][k])
}

impl Gizmo {
    pub fn new(center: Vec3, eye: Vec3, kind: GizmoMode, rot: Quat, full_rotation: bool) -> Self {
        Self { center, scale: (center.distance(eye) * 0.12).max(0.4), kind, rot, full_rotation }
    }

    pub fn ring_axis(&self, k: usize) -> Vec3 {
        match k {
            0 => self.rot * Vec3::X,
            1 => Vec3::Y,
            _ => self.rot * Vec3::Z,
        }
    }

    fn ring_points(&self, k: usize) -> Vec<Vec3> {
        let n = self.ring_axis(k);
        let u = n.any_orthonormal_vector();
        let v = n.cross(u);
        let r = self.scale * if k == 1 { 1.2 } else { 1.0 };
        (0..=64).map(|i| {
            let a = i as f32 / 64.0 * std::f32::consts::TAU;
            self.center + (u * a.cos() + v * a.sin()) * r
        }).collect()
    }

    pub fn dir(&self, k: usize) -> Vec3 {
        if self.kind == GizmoMode::Scale { self.rot * AXES[k] } else { AXES[k] }
    }

    fn rings(&self) -> Vec<usize> {
        if self.full_rotation { vec![1, 0, 2] } else { vec![1] }
    }

    fn axes(&self) -> Vec<usize> {
        match self.kind {
            GizmoMode::Lift => vec![1],
            _ => vec![0, 1, 2],
        }
    }

    pub fn hit(&self, vp: &Mat4, size: Vec2, m: Vec2) -> Option<Handle> {
        let seg = |a: Vec2, b: Vec2| {
            let ab = b - a;
            let t = ((m - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
            (a + ab * t).distance(m)
        };
        let c = project(vp, size, self.center)?;
        let mut best: Option<(Handle, f32)> = None;
        let mut consider = |h: Handle, d: f32, lim: f32| {
            if d < lim && best.is_none_or(|b| d < b.1) {
                best = Some((h, d));
            }
        };
        match self.kind {
            GizmoMode::Translate | GizmoMode::Lift | GizmoMode::Scale => {
                for k in self.axes() {
                    if let Some(e) = project(vp, size, self.center + self.dir(k) * self.scale * 1.15) {
                        consider(Handle::Axis(k), seg(c + (e - c) * 0.2, e), 9.0);
                    }
                }
                if matches!(self.kind, GizmoMode::Translate | GizmoMode::Scale) {
                    consider(Handle::Center, c.distance(m), 11.0);
                }
            }
            GizmoMode::Rotate => {
                for k in self.rings() {
                    let pts: Vec<Vec2> = self.ring_points(k).into_iter().filter_map(|p| project(vp, size, p)).collect();
                    for w in pts.windows(2) {
                        consider(Handle::Ring(k), seg(w[0], w[1]), 7.0);
                    }
                }
            }
        }
        best.map(|b| b.0)
    }

    pub fn draw(&self, d: &mut DrawList, active: Option<Handle>) {
        let s = self.scale;
        match self.kind {
            GizmoMode::Scale => {
                let cube = |d: &mut DrawList, at: Vec3, r: f32, c: [u8; 4]| {
                    let (x, y, z) = (self.rot * Vec3::X * r, self.rot * Vec3::Y * r, self.rot * Vec3::Z * r);
                    for (n, u, v) in [(x, y, z), (-x, z, y), (y, z, x), (-y, x, z), (z, x, y), (-z, y, x)] {
                        let f = at + n;
                        for p in [f - u - v, f + u - v, f + u + v, f - u - v, f + u + v, f - u + v] {
                            d.overlay.push(Vertex::flat(p, c));
                        }
                    }
                };
                for k in self.axes() {
                    let a = self.dir(k);
                    let on = active == Some(Handle::Axis(k));
                    let rgb = if on { theme::rgb8(theme::TEXT) } else { axis_rgb(k) };
                    let c = [rgb[0], rgb[1], rgb[2], 255];
                    let u = a.any_orthonormal_vector();
                    let v = a.cross(u);
                    let r = s * if on { 0.03 } else { 0.02 };
                    let (p0, p1) = (self.center + a * s * 0.2, self.center + a * s * 1.05);
                    let q = [u * r + v * r, -u * r + v * r, -u * r - v * r, u * r - v * r];
                    for i in 0..4 {
                        let (e0, e1) = (q[i], q[(i + 1) % 4]);
                        for p in [p0 + e0, p0 + e1, p1 + e1, p0 + e0, p1 + e1, p1 + e0] {
                            d.overlay.push(Vertex::flat(p, c));
                        }
                    }
                    cube(d, self.center + a * s * 1.12, s * if on { 0.085 } else { 0.065 }, c);
                }
                let on = active == Some(Handle::Center);
                let rgb = if on { theme::rgb8(theme::PIN) } else { theme::rgb8(theme::TEXT) };
                cube(d, self.center, s * 0.07, [rgb[0], rgb[1], rgb[2], 230]);
            }
            GizmoMode::Translate | GizmoMode::Lift => {
                for k in self.axes() {
                    let a = AXES[k];
                    let on = active == Some(Handle::Axis(k));
                    let rgb = if on { theme::rgb8(theme::TEXT) } else { axis_rgb(k) };
                    let c = [rgb[0], rgb[1], rgb[2], 255];
                    let (u, v) = (a.any_orthonormal_vector(), a.cross(a.any_orthonormal_vector()));
                    let r = s * if on { 0.035 } else { 0.024 };
                    let (p0, p1) = (self.center + a * s * 0.2, self.center + a * s * 0.92);
                    let q = [u * r + v * r, -u * r + v * r, -u * r - v * r, u * r - v * r];
                    for i in 0..4 {
                        let (e0, e1) = (q[i], q[(i + 1) % 4]);
                        for p in [p0 + e0, p0 + e1, p1 + e1, p0 + e0, p1 + e1, p1 + e0] {
                            d.overlay.push(Vertex::flat(p, c));
                        }
                        let hb = q.map(|e| e * 3.2);
                        let tip = self.center + a * s * 1.15;
                        for p in [p1 + hb[i], p1 + hb[(i + 1) % 4], tip] {
                            d.overlay.push(Vertex::flat(p, c));
                        }
                    }
                }
                if self.kind == GizmoMode::Translate {
                    let on = active == Some(Handle::Center);
                    let rgb = if on { theme::rgb8(theme::PIN) } else { theme::rgb8(theme::TEXT) };
                    let c = [rgb[0], rgb[1], rgb[2], 230];
                    let r = s * 0.09;
                    let pts: Vec<Vec3> = (0..=24).map(|i| {
                        let a = i as f32 / 24.0 * std::f32::consts::TAU;
                        self.center + Vec3::new(a.cos(), 0.0, a.sin()) * r
                    }).collect();
                    for w in pts.windows(2) {
                        for p in [self.center, w[0], w[1]] {
                            d.overlay.push(Vertex::unlit(p, c));
                        }
                    }
                }
            }
            GizmoMode::Rotate => {
                for k in self.rings() {
                    let on = active == Some(Handle::Ring(k));
                    let rgb = if on { theme::rgb8(theme::TEXT) } else { axis_rgb(match k { 0 => 0, 1 => 1, _ => 2 }) };
                    let c = [rgb[0], rgb[1], rgb[2], if on { 255 } else { 220 }];
                    let pts = self.ring_points(k);
                    let n = self.ring_axis(k);
                    let w = s * if on { 0.045 } else { 0.028 };
                    for p in pts.windows(2) {
                        let (a, b) = (p[0], p[1]);
                        let side = (b - a).cross(n).normalize_or_zero() * w;
                        for q in [a - side, a + side, b + side, a - side, b + side, b - side] {
                            d.overlay.push(Vertex::unlit(q, c));
                        }
                    }
                }
            }
        }
    }

    pub fn axis_param(&self, k: usize, vp: &Mat4, size: Vec2, m: Vec2) -> Option<f32> {
        let (o, dir) = ray(vp, size, m);
        let a = self.dir(k);
        let w0 = self.center - o;
        let (b, dd, e) = (a.dot(dir), a.dot(w0), dir.dot(w0));
        let den = 1.0 - b * b;
        (den.abs() > 1e-4).then(|| (b * e - dd) / den)
    }

    pub fn ring_angle(&self, k: usize, vp: &Mat4, size: Vec2, m: Vec2) -> Option<f32> {
        let (o, dir) = ray(vp, size, m);
        let n = self.ring_axis(k);
        let den = dir.dot(n);
        if den.abs() < 1e-4 {
            return None;
        }
        let t = (self.center - o).dot(n) / den;
        if t <= 0.0 {
            return None;
        }
        let p = o + dir * t - self.center;
        let u = n.any_orthonormal_vector();
        let v = n.cross(u);
        Some(p.dot(v).atan2(p.dot(u)))
    }

    pub const AXES: [Vec3; 3] = AXES;
}
