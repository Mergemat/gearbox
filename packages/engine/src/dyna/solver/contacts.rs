use super::super::constants::{MIN_CLIP_LENGTH_SQUARED, SAMPLE_DT, SOLVER_EPSILON};
use super::super::{Body, BodyHandle, BodyInertia, BodyState};
use crate::{
    Vec3,
    math::{maxss, minss},
};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactRow {
    pub body: u32,
    pub point: Vec3,
    pub normal: Vec3,
    pub bias: f32,
    pub mass_factor: f32,
    pub impulse: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpulseContact {
    pub point: Vec3,
    pub normal: Vec3,
    pub other: Option<BodyHandle>,
}

pub fn contact_inverse_mass(k: &BodyInertia, state: &BodyState, p: Vec3, n: Vec3) -> f32 {
    let r = state.pos - p;
    let c0 = n.z * r.y - n.y * r.z;
    let c2 = n.y * r.x - n.x * r.y;
    let c1 = n.x * r.z - n.z * r.x;
    c1 * k.diagonal.y * c1 + c0 * k.diagonal.x * c0 + c2 * k.diagonal.z * c2 + k.inverse_mass
}

pub fn solve_contacts(bodies: &mut [Body], rows: &mut [ContactRow]) -> f32 {
    let mut worst = 0.0;
    for row in rows {
        let body = &mut bodies[row.body as usize];
        let state = &mut body.state;
        let k = &body.inertia;
        let r = row.point - state.pos;
        let v = state.point_velocity(row.point);
        let n = row.normal;
        let vn = v.dot_yxz(n) + row.bias;
        let acc = maxss(0.0, -vn * row.mass_factor + row.impulse);
        let change = acc - row.impulse;
        let d = Vec3::new(n.x * change, n.y * change, n.z * change);
        state.vel.x += k.inverse_mass * d.x;
        state.vel.y += d.y * k.inverse_mass;
        state.vel.z += d.z * k.inverse_mass;
        state.ang_vel.x += (d.z * r.y - d.y * r.z) * k.diagonal.x;
        state.ang_vel.y += (d.x * r.z - d.z * r.x) * k.diagonal.y;
        state.ang_vel.z += (d.y * r.x - d.x * r.y) * k.diagonal.z;
        row.impulse = acc;
        let absolute = change.abs();
        if !(worst >= absolute) {
            worst = absolute;
        }
    }
    worst
}

pub fn clip_sum(radius: f32, displacements: &[Vec3]) -> Vec3 {
    let Some((&first, rest)) = displacements.split_first() else {
        return Vec3::ZERO;
    };
    let mut v = first;
    for p in rest {
        let dot = p.x * v.x + p.y * v.y + p.z * v.z;
        if dot > 0.0 {
            let sq = v.x * v.x + v.y * v.y + v.z * v.z;
            if sq > MIN_CLIP_LENGTH_SQUARED {
                let t = minss(sq, dot) / sq;
                v = Vec3::new(v.x - v.x * t, v.y - v.y * t, v.z - v.z * t);
            }
        }
        v += *p;
    }
    let sq = v.x * v.x + v.y * v.y + v.z * v.z;
    if sq > radius * radius {
        let s = 1.0 / sq.sqrt();
        Vec3::new(v.x - v.x * s * radius, v.y - v.y * s * radius, v.z - v.z * s * radius)
    } else {
        Vec3::ZERO
    }
}

pub fn closest_time_ms(now: i64, max: f32, a: Vec3, b: Vec3) -> i64 {
    let dot = b.dot_yxz(a);
    let sq = b.x * b.x + b.y * b.y + b.z * b.z + SOLVER_EPSILON;
    let mut t = -dot / sq;
    let min = SAMPLE_DT;
    if t < min || t.is_nan() {
        t = min;
    } else if !(max >= t) {
        t = max;
    }
    let d = (max - t) as f64 * -1e9;
    let i = if d.is_nan() || !(-9.223372036854775808e18..9.223372036854775807e18).contains(&d) {
        i64::MIN
    } else {
        d as i64
    };
    now.wrapping_sub(i) / 1_000_000
}
