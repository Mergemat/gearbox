use crate::{Quat, Vec3};
use crate::math::{angle_approx, fast_exp2, fast_log2};
use super::{ForceCtx, ForceGeometry};
use super::super::Vehicle;
use super::common::{TINY, MS_PER_SECOND, HALF_TURN_DEGREES, clamp01};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Immersion {
    pub surface_height: f32,
    pub fraction: f32,
}

pub fn water_fraction(vehicle: &Vehicle, geometry: &ForceGeometry,
    water: crate::water::WaterManager<'_>) -> Option<Immersion> {
    let bounds = geometry.water_box.transform_quat(vehicle.quat, vehicle.pos);
    let center_y = bounds.center.y;
    let half_height = bounds.half.y.abs();
    let bottom = center_y - half_height;
    let mut depth = 0.0;
    if !water.query(Vec3::new(bounds.center.x, bottom, bounds.center.z),
        half_height + half_height, Some(&mut depth), None) { return None; }
    let fraction = clamp01((depth - bottom) / ((center_y + half_height) - bottom));
    Some(Immersion { surface_height: depth, fraction })
}

pub fn simple_water(ctx: &mut ForceCtx<'_>) {
    let hit = water_fraction(ctx.vehicle, ctx.geometry, ctx.water);
    ctx.vehicle.in_water = hit.is_some() as u32;
    let Some(hit) = hit else { return };
    increase_immersion(ctx);
    apply_buoyancy(ctx, hit.fraction);
}

pub fn water_step(ctx: &mut ForceCtx<'_>) {
    assert!(ctx.tuning.water_enabled != 0, "floating without the water model");
    let hit = water_fraction(ctx.vehicle, ctx.geometry, ctx.water);
    let up_axis = ctx.vehicle.quat.rotate(Vec3::new(0.0, 1.0, 0.0));
    let (uy0, uz0, ux0) = (up_axis.y * 0.0, up_axis.z * 0.0, up_axis.x * 0.0);
    let righting_axis = Vec3::new(uy0 - up_axis.z, uz0 - ux0, up_axis.x - uy0);
    let deadline = ctx.vehicle.water_reentry_until;
    if hit.is_some() || (deadline != u32::MAX && ctx.tick <= deadline) {
        in_water(ctx, hit.map_or(0.0, |h| h.fraction), up_axis, righting_axis);
        return;
    }
    if deadline != u32::MAX {
        ctx.vehicle.water_exit_until = ctx.tick.wrapping_add(ctx.tuning.water_exit_righting_ms);
    }
    let deadline = ctx.vehicle.water_exit_until;
    if ctx.tuning.water_righting_enabled != 0 && deadline > ctx.tick && deadline != u32::MAX {
        let squared_length = (righting_axis.y * righting_axis.y + righting_axis.x * righting_axis.x)
            + righting_axis.z * righting_axis.z;
        if squared_length > TINY && f32::MAX > squared_length {
            let inv = 1.0 / squared_length.sqrt();
            let righting_axis = Vec3::new(righting_axis.x * inv, righting_axis.y * inv,
                righting_axis.z * inv);
            let Vec3 { mut x, mut y, mut z } = ctx.vehicle.ang_vel;
            let squared_length = (y * y + x * x) + z * z;
            let mut angular_speed = 0.0;
            if squared_length > TINY && f32::MAX > squared_length {
                angular_speed = squared_length.sqrt();
                let k = 1.0 / angular_speed;
                (x, y, z) = (x * k, y * k, z * k);
            }
            let projected_rate = ((x * righting_axis.x + y * righting_axis.y) + z * righting_axis.z)
                * angular_speed;
            let righting_response = ctx.tuning.water_exit_righting_by_rate.eval(projected_rate);
            ctx.body.add_torque(Vec3::new(righting_response * righting_axis.x, righting_response
                * righting_axis.y, righting_response * righting_axis.z));
        }
    }
    ctx.vehicle.water_reentry_until = u32::MAX;
    ctx.vehicle.in_water = 0;
}

fn increase_immersion(ctx: &mut ForceCtx<'_>) {
    let dt_ms = ctx.dt * MS_PER_SECOND;
    let immersion = ctx.tuning.immersion_rise_rate / (ctx.tuning.immersion_rise_ms as f32 / dt_ms)
        + ctx.vehicle.immersion;
    ctx.vehicle.immersion = clamp01(immersion);
}

fn apply_buoyancy(ctx: &mut ForceCtx<'_>, fraction: f32) {
    let mut hull = ctx.geometry.water_box;
    hull.center = ctx.geometry.center_of_mass;
    let pose = ctx.body.state().pose();
    let forces = crate::water::buoyancy::apply(Some(ctx.water), ctx.body.dyna, ctx.body.handle,
        hull, hull.transform(&pose), ctx.fluid_tuning, &mut crate::water::buoyancy::CollisionGeometry)
        .expect("force context has a water manager");
    let (force, torque) = (forces.force, forces.torque);
    let buoyancy_scale = fast_exp2(fast_log2(fraction) * ctx.tuning.buoyancy_fraction_power);
    ctx.body.add_force(force * buoyancy_scale);
    ctx.body.add_torque(torque * buoyancy_scale);
}

fn in_water(ctx: &mut ForceCtx<'_>, fraction: f32, up_axis: Vec3, righting_axis: Vec3) {
    let deadline = ctx.vehicle.water_reentry_until;
    if deadline == u32::MAX {
        ctx.vehicle.water_reentry_until = ctx.tuning.immersion_rise_ms.wrapping_add(ctx.tick);
    } else if ctx.tick > deadline {
        ctx.vehicle.water_reentry_until = deadline.wrapping_add(ctx.tuning.water_transition_ms);
    }
    ctx.vehicle.in_water = 1;
    increase_immersion(ctx);
    apply_buoyancy(ctx, fraction);
    if fraction >= 1.0 {
        return;
    }
    if !ctx.vehicle.any_wheel_contact() {
        super::water_wheels::wheel_forces(ctx, fraction);
    }
    let steering = ctx.vehicle.steering;
    let response = ctx.tuning.water_yaw_by_steering.eval(steering.abs());
    let sign = if steering.is_sign_negative() {
        1.0
    } else {
        -1.0
    };
    ctx.body.add_local_torque(Vec3::new(0.0, sign * response, 0.0));
    let angular_velocity = ctx.vehicle.ang_vel;
    let angular_speed = ((angular_velocity.x * angular_velocity.x + angular_velocity.y
        * angular_velocity.y) + angular_velocity.z * angular_velocity.z).sqrt();
    let response = ctx.tuning.water_angular_drag.eval(angular_speed * angular_speed);
    let squared_speed = (angular_velocity.y * angular_velocity.y + angular_velocity.x
        * angular_velocity.x) + angular_velocity.z * angular_velocity.z;
    let mut drag_direction = angular_velocity;
    if squared_speed > TINY && f32::MAX > squared_speed {
        let drag_scale = 1.0 / squared_speed.sqrt();
        drag_direction = Vec3::new(angular_velocity.x * drag_scale, angular_velocity.y * drag_scale,
            angular_velocity.z * drag_scale);
    }
    let orientation = ctx.vehicle.quat;
    let drag_direction = Quat::new(orientation.w, -orientation.x, -orientation.y,
        -orientation.z).rotate(drag_direction);
    let inverse_dt = 1.0 / ctx.dt;
    let drag_scale = -response * angular_speed;
    ctx.body.add_local_torque(Vec3::new((drag_direction.x * drag_scale) * inverse_dt,
        (drag_direction.y * drag_scale) * inverse_dt, (drag_direction.z * drag_scale) * inverse_dt));
    if ctx.tuning.water_righting_enabled == 0 {
        return;
    }
    let (ux0, uy0, uz0) = (up_axis.x * 0.0, up_axis.y * 0.0, up_axis.z * 0.0);
    let (a, b, response) = (ux0 - uz0, up_axis.z - uy0, uy0 - up_axis.x);
    let dot = (up_axis.y + ux0) + uz0;
    let len = ((a * a + b * b) + response * response).sqrt();
    let angle = angle_approx(len, dot);
    let degrees = (angle * HALF_TURN_DEGREES / std::f32::consts::PI).abs();
    let response = -ctx.tuning.water_righting_by_tilt.eval(degrees);
    ctx.body.add_torque(Vec3::new(response * righting_axis.x, response * righting_axis.y, response
        * righting_axis.z));
}
