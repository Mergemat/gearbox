use super::super::Tuning;
use super::{
    ForceCtx,
    common::{EPS, KMH, clamp01},
};
use crate::{Curve, Mat3, Quat, Vec3};

const NS_PER_MS: i64 = 1_000_000;
const IMMERSION_IMPULSE_SCALE: f32 = 3.0;

pub fn contact_impulse(
    mass: f32,
    inverse_inertia: Mat3,
    restitution: f32,
    velocity: Vec3,
    normal: Vec3,
    radius: Vec3,
) -> Vec3 {
    let (r0, r2, r1) = (radius.x, radius.z, radius.y);
    let [n0, n1, n2] = normal.to_array();
    let cross_y = n0 * r2 - r0 * n2;
    let cross_x = r1 * n2 - r2 * n1;
    let cross_z = r0 * n1 - n0 * r1;
    let inertia = inverse_inertia.to_array();
    let angular_y = cross_y * inertia[4] + cross_x * inertia[3] + cross_z * inertia[5];
    let angular_z = cross_y * inertia[7] + cross_x * inertia[6] + cross_z * inertia[8];
    let angular_x = cross_y * inertia[1] + cross_x * inertia[0] + cross_z * inertia[2];
    let normal_velocity = n1 * velocity.y + n0 * velocity.x + n2 * velocity.z;
    let impulse = (normal_velocity * (restitution - 1.0))
        / ((angular_z * r0 - angular_x * r2) * n1
            + (angular_y * r2 - angular_z * r1) * n0
            + (angular_x * r1 - angular_y * r0) * n2
            + 1.0 / mass);
    Vec3::new(n0 * impulse, n1 * impulse, n2 * impulse)
}

pub fn exchange_ratio(velocity: f32, pending: f32, coefficient: f32) -> f32 {
    if pending.abs() < EPS {
        return 1.0;
    }
    let scaled_velocity = velocity * coefficient;
    if (pending * coefficient) * scaled_velocity > 0.0 {
        return (velocity / (velocity + pending)).abs();
    }
    if scaled_velocity > EPS { 1.0 } else { 0.0 }
}

pub fn direction_drag(tuning: &Tuning, velocity: Vec3, normal: Vec3, vertical: bool) -> f32 {
    if tuning.direction_drag_enabled == 0 {
        return 0.0;
    }
    let [d0, d1, d2] = normal.to_array();
    let [a0, a1, a2] = velocity.to_array();
    let up = d0 * 0.0 + d1 + d2 * 0.0;
    let along = ((a0 * d0 + a1 * d1 + a2 * d2) * KMH).abs();
    let off_axis = (1.0 - up).abs();
    let alignment = tuning.direction_off_axis_drag.eval(off_axis * along);
    let speed = (a0 * a0 + a1 * a1 + a2 * a2).sqrt();
    let speed_response = tuning.direction_speed_drag.eval((speed * KMH - along) * off_axis);
    let vertical_response =
        if vertical { tuning.direction_vertical_drag.eval(up.abs() * along) } else { 0.0 };
    vertical_response + alignment + speed_response
}

pub fn impact_timer(curve: &Curve, limit_ms: u32, speed: f32, now: i64, timer: &mut u64) {
    let duration = crate::math::cvtt64(curve.eval(speed));
    let mut current = *timer as i64;
    if current < now {
        *timer = now as u64;
        current = now;
    }
    let candidate = current.wrapping_add((duration as u32 as i64).wrapping_mul(NS_PER_MS));
    let limit = (limit_ms as i64).wrapping_mul(NS_PER_MS).wrapping_add(now);
    *timer = if candidate <= limit { candidate } else { limit } as u64;
}

pub fn impulse_at_point(ctx: &mut ForceCtx<'_>, impulse: Vec3, point: Vec3, blend: f32) {
    if ctx.vehicle.body == u32::MAX {
        return;
    }
    let state = ctx.body.state();
    let rotation = state.rot.to_array();
    let world_impulse = rotate(impulse, rotation);
    let rotated_point = rotate(point, rotation);
    let world_point = rotated_point + state.pos;
    let [vx, vy, vz] = state.vel.to_array();
    let inverse_mass = 1.0 / ctx.geometry.mass;
    let mut velocity = Vec3::new(
        inverse_mass * world_impulse.x + vx,
        inverse_mass * world_impulse.y + vy,
        inverse_mass * world_impulse.z + vz,
    );
    let previous_squared = (vy * vy + vx * vx) + vz * vz;
    let next_squared = (velocity.y * velocity.y + velocity.x * velocity.x) + velocity.z * velocity.z;
    if next_squared > previous_squared
        && next_squared - previous_squared > ctx.tuning.impulse_speed_squared_limit
    {
        velocity = Vec3::ZERO;
    }
    ctx.body.set_world_velocity(velocity);
    let state = ctx.body.state();
    let center = rotate(ctx.geometry.center_of_mass, state.rot.to_array()) + state.pos;
    let radius = world_point - center;
    let torque = Vec3::new(
        radius.y * world_impulse.z - radius.z * world_impulse.y,
        radius.z * world_impulse.x - radius.x * world_impulse.z,
        radius.x * world_impulse.y - radius.y * world_impulse.x,
    );
    let base = ctx.tuning.impulse_angular_blend;
    let factor = (1.0 - base) * blend + base;
    let inertia = ctx.vehicle.world_inverse_inertia.to_array();
    let angular = Vec3::new(
        ((torque.y * inertia[1] + torque.x * inertia[0]) + torque.z * inertia[2]) * factor,
        ((torque.y * inertia[4] + torque.x * inertia[3]) + torque.z * inertia[5])
            * factor
            * ctx.tuning.impulse_y_blend,
        ((torque.y * inertia[7] + torque.x * inertia[6]) + torque.z * inertia[8]) * factor,
    );
    ctx.body.add_world_angular_velocity(angular);
}

pub fn contact_response(
    ctx: &mut ForceCtx<'_>,
    surface: u8,
    point: Vec3,
    target: Vec3,
    coefficients: Vec3,
    curve: bool,
    scale: f32,
) {
    let vehicle = &*ctx.vehicle;
    let rotation = vehicle.quat;
    let world_point = rotation.rotate(point) + vehicle.pos;
    let world_velocity = ctx.body.velocity_at(world_point);
    let inverse = Quat::new(rotation.w, -rotation.x, -rotation.y, -rotation.z);
    let local_velocity = inverse.rotate(world_velocity);
    let pending = inverse.rotate(vehicle.extra_linear_velocity);
    let impulse = Vec3::new(
        exchange_ratio(local_velocity.x, pending.x, coefficients.x) * target.x,
        exchange_ratio(local_velocity.y, pending.y, coefficients.y) * target.y,
        exchange_ratio(local_velocity.z, pending.z, coefficients.z) * target.z,
    );
    let blend = if surface == b'M' { 1.0 } else { clamp01(vehicle.immersion * IMMERSION_IMPULSE_SCALE) };
    impulse_at_point(ctx, impulse, point, blend);
    let remainder =
        rotation.rotate(Vec3::new(target.x - impulse.x, target.y - impulse.y, target.z - impulse.z));
    let inverse_mass = 1.0 / ctx.geometry.mass;
    let vehicle = &mut *ctx.vehicle;
    vehicle.extra_linear_velocity.x += remainder.x * inverse_mass;
    vehicle.extra_linear_velocity.y += remainder.y * inverse_mass;
    vehicle.extra_linear_velocity.z += remainder.z * inverse_mass;
    let center = ctx.body.center_of_mass();
    let radius = world_point - center;
    let torque = Vec3::new(
        radius.y * remainder.z - radius.z * remainder.y,
        radius.z * remainder.x - radius.x * remainder.z,
        radius.x * remainder.y - radius.y * remainder.x,
    );
    let mut strength = 1.0;
    if curve {
        let length =
            ((remainder.y * remainder.y + remainder.x * remainder.x) + remainder.z * remainder.z).sqrt();
        strength = ctx.tuning.contact_angular_response.eval(length) * scale;
    }
    strength *= ctx.tuning.impulse_angular_blend;
    let inertia = vehicle.world_inverse_inertia.to_array();
    vehicle.extra_angular_velocity.x +=
        ((torque.y * inertia[1] + torque.x * inertia[0]) + torque.z * inertia[2]) * strength;
    vehicle.extra_angular_velocity.z +=
        ((torque.y * inertia[7] + torque.x * inertia[6]) + torque.z * inertia[8]) * strength;
    vehicle.extra_angular_velocity.y += ((torque.x * inertia[3] + inertia[4] * torque.y)
        + inertia[5] * torque.z)
        * strength
        * ctx.tuning.impulse_y_blend;
}

fn rotate(vector: Vec3, rotation: [f32; 9]) -> Vec3 {
    Vec3::new(
        (vector.x * rotation[0] + vector.y * rotation[1]) + vector.z * rotation[2],
        (vector.x * rotation[3] + vector.y * rotation[4]) + vector.z * rotation[5],
        (vector.x * rotation[6] + vector.y * rotation[7]) + vector.z * rotation[8],
    )
}
