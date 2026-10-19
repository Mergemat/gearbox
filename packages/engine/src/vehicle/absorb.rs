use super::{
    Tuning, Vehicle,
    forces::common::{EPS, TINY},
};
use crate::{Curve, Vec3};

const SUPPORT_ANGULAR_SCALE: f32 = 0.33333334;
const NS_PER_MS: i64 = 1_000_000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub enum ContactKind {
    #[default]
    Ordinary = 0,
    Side = 1,
    Inside = 2,
    Floor = 3,
    Outside = 4,
}

pub fn extend_deadline(curve: &Curve, limit_ms: u32, speed: f32, now: i64, deadline: &mut u64) {
    let duration = crate::math::cvtt64(curve.eval(speed)) as u32 as i64;
    let base = (*deadline as i64).max(now);
    let candidate = base.wrapping_add(duration * NS_PER_MS);
    let limit = (limit_ms as i64 * NS_PER_MS).wrapping_add(now);
    *deadline = candidate.min(limit) as u64;
}

impl Vehicle {
    pub fn moving_support(&mut self, tuning: &Tuning, point: Vec3, velocity: Vec3, center: Vec3, dt: f32) {
        self.support_contact_count = self.support_contact_count.wrapping_add(1);
        let delta = velocity - self.extra_linear_velocity;
        let speed = (delta.y * delta.y + delta.x * delta.x + delta.z * delta.z).sqrt();
        let scale = tuning.support_by_speed.eval(speed) * dt;
        self.support_linear_x = velocity.x * scale + self.support_linear_x;
        self.support_linear_y = velocity.y * scale + self.support_linear_y;
        self.support_linear_z = velocity.z * scale + self.support_linear_z;
        let delta = delta * scale;
        let radius = point - center;
        self.support_angular_x =
            (radius.y * delta.z - radius.z * delta.y) * SUPPORT_ANGULAR_SCALE + self.support_angular_x;
        self.support_angular_z =
            (radius.x * delta.y - radius.y * delta.x) * SUPPORT_ANGULAR_SCALE + self.support_angular_z;
        self.support_angular_y =
            (radius.z * delta.x - radius.x * delta.z) * SUPPORT_ANGULAR_SCALE + self.support_angular_y;
    }

    pub fn end_collision_step(&mut self, tuning: &Tuning) {
        if self.support_contact_count == 0 {
            let scale = tuning.support_decay;
            self.extra_angular_velocity = scale * self.extra_angular_velocity;
            self.extra_linear_velocity = scale * self.extra_linear_velocity;
        } else {
            let inverse = 1.0 / self.support_contact_count as f32;
            self.extra_linear_velocity = Vec3::new(
                inverse * self.support_linear_x,
                inverse * self.support_linear_y,
                inverse * self.support_linear_z,
            );
            self.extra_angular_velocity = Vec3::new(
                inverse * self.support_angular_x,
                inverse * self.support_angular_y,
                inverse * self.support_angular_z,
            );
        }
        self.support_contact_count = 0;
        self.support_linear_x = 0.0;
        self.support_linear_y = 0.0;
        self.support_linear_z = 0.0;
        self.support_angular_x = 0.0;
        self.support_angular_y = 0.0;
        self.support_angular_z = 0.0;
    }
}

pub fn volume_contact(
    tuning: &Tuning,
    center: Vec3,
    point: Vec3,
    normal: &mut Vec3,
    velocity: &mut Vec3,
) -> ContactKind {
    let project = |normal: Vec3, velocity: &mut Vec3| {
        let dot = normal.x * velocity.x + normal.y * velocity.y + normal.z * velocity.z;
        let scale = if 0.0 <= dot { dot } else { 0.0 };
        *velocity = Vec3::new(normal.x * scale, normal.y * scale, normal.z * scale);
    };
    if point.y < tuning.contact_volume_bottom {
        project(*normal, velocity);
        return ContactKind::Inside;
    }
    let dx = point.x - center.x;
    let dz = point.z - center.z;
    let half_x = tuning.contact_volume_half_width;
    let half_z = tuning.contact_volume_half_depth;
    let floor = tuning.contact_volume_top;
    if dx.abs() < half_x && dz.abs() < half_z {
        if point.y < floor || point.y == floor {
            return ContactKind::Inside;
        }
        project(*normal, velocity);
        return ContactKind::Outside;
    }
    let nearest = |delta: f32, half: f32| {
        if delta <= half && -half <= delta {
            delta
        } else if delta <= half {
            -half
        } else {
            half
        }
    };
    let top = if floor < point.y { floor } else { point.y };
    let delta_y = top - point.y;
    let mut direction = Vec3::new(
        nearest(dx, half_x) - dx,
        if delta_y <= 0.0 { delta_y } else { 0.0 },
        nearest(dz, half_z) - dz,
    );
    let squared = direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
    if TINY < squared && squared < f32::MAX {
        let inverse = 1.0 / squared.sqrt();
        direction.y *= inverse;
        direction.x = inverse * direction.x;
        direction.z = inverse * direction.z;
    }
    *normal = direction;
    if floor < point.y { ContactKind::Outside } else { ContactKind::Floor }
}

pub fn contact_depth(
    tuning: &Tuning,
    wheel: Option<(Vec3, f32)>,
    point: Vec3,
    kind: ContactKind,
) -> (bool, f32) {
    match kind {
        ContactKind::Side | ContactKind::Inside => return (false, 0.0),
        ContactKind::Outside => return (true, 1.0),
        ContactKind::Ordinary => return (false, 1.0),
        ContactKind::Floor => {}
    }
    let depth = if let Some((anchor, radius)) = wheel {
        let top = anchor.y - radius / 3.0;
        if point.y > top { 1.0 } else { 1.0 - (top - point.y) / ((radius + radius) / 3.0) }
    } else {
        let low = tuning.contact_volume_bottom;
        let high = tuning.contact_volume_top;
        let middle = (high - low) * 0.5 + low;
        if -tuning.contact_volume_half_depth > point.z {
            if middle > point.y {
                0.0
            } else if point.y > high {
                1.0
            } else {
                (point.y - middle) / (high - middle)
            }
        } else if low > point.y {
            0.0
        } else if point.y > middle {
            1.0
        } else {
            (point.y - low) / (middle - low)
        }
    };
    (depth > EPS, depth)
}
