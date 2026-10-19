use super::super::Vehicle;
use super::{
    ForceCtx,
    common::{KMH, PEDAL_THRESHOLD, PedalMode},
};
use crate::Vec3;

const DRIVE_ANCHOR_RISE: f32 = 0.085;

pub fn start_boost(ctx: &mut ForceCtx<'_>) -> f32 {
    let tuning = ctx.tuning;
    if tuning.slow_start_enabled == 0 {
        return 1.0;
    }
    let vehicle = &mut *ctx.vehicle;
    let forward = ctx.local_velocity.z;
    let gas = vehicle.controls.gas > PEDAL_THRESHOLD || vehicle.effect_mode_is(PedalMode::Full);
    let brake = vehicle.controls.brake > PEDAL_THRESHOLD;
    let previous_gas = vehicle.previous_controls.gas > PEDAL_THRESHOLD;
    let previous_brake = vehicle.previous_controls.brake > PEDAL_THRESHOLD;
    let flags = vehicle.slow_start_flags;
    let braking_armed = flags & 1 != 0;
    let accelerating_armed = flags & 2 != 0;
    vehicle.slow_start_flags = flags & 0xfc;
    if !vehicle.any_wheel_contact() || vehicle.turbo_active != 0 || vehicle.level() != 0 {
        return reset_start_boost(vehicle);
    }
    let cancel = (!gas && previous_gas) || (!brake && previous_brake) || brake == gas;
    let velocity = ctx.local_velocity;
    let speed = (velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z).sqrt() * KMH;
    let limit = tuning.slow_start_speed_limit;
    let start = limit > speed
        && if !gas {
            brake && forward > vehicle.slow_start_forward_threshold && braking_armed
        } else {
            !brake && accelerating_armed
        };
    if start {
        if vehicle.slow_start_begin == u32::MAX {
            vehicle.slow_start_begin = ctx.tick;
            if ctx.tick == u32::MAX {
                return 1.0;
            }
        }
    } else if vehicle.slow_start_begin != u32::MAX {
        if (gas && forward * KMH > limit) || (brake && -limit > forward * KMH) || cancel {
            return reset_start_boost(vehicle);
        }
        if vehicle.slow_start_end == u32::MAX {
            if (brake && !braking_armed) || (gas && !accelerating_armed) {
                vehicle.slow_start_end = tuning.slow_start_release_ms.wrapping_add(ctx.tick);
            }
        } else if ctx.tick > vehicle.slow_start_end {
            return reset_start_boost(vehicle);
        }
    }
    if vehicle.slow_start_begin == u32::MAX {
        return 1.0;
    }
    tuning.slow_start_boost.eval(ctx.tick.wrapping_sub(vehicle.slow_start_begin) as f32) + 1.0
}

fn reset_start_boost(vehicle: &mut Vehicle) -> f32 {
    vehicle.slow_start_flags = 0;
    vehicle.slow_start_begin = u32::MAX;
    vehicle.slow_start_end = u32::MAX;
    1.0
}

pub fn level_drive(ctx: &mut ForceCtx<'_>, scale: f32) {
    let vehicle = &*ctx.vehicle;
    let tuning = ctx.tuning;
    let until = tuning.level_start_push_ms.wrapping_add(vehicle.boost_begin);
    let speed = ctx.local_velocity.z * KMH;
    let thrust = tuning.level_thrust.eval(speed) * vehicle.controls.gas;
    let thrust = thrust - tuning.level_brake.eval(speed) * vehicle.controls.brake;
    let mut multiplier = 1.0;
    if !vehicle.any_wheel_contact() {
        let lateral = ctx.local_velocity.x;
        let damping = tuning.level_lateral_damping.eval((lateral * KMH).abs());
        ctx.body.add_local_force(Vec3::new(-damping * lateral, 0.0, 0.0));
        let forward = ctx.local_velocity.z;
        multiplier = tuning.level_side_slip.eval((forward.abs() - lateral.abs()) / forward);
    }
    let push = multiplier * thrust * scale;
    for definition in &ctx.geometry.wheels {
        let point =
            Vec3::new(definition.anchor.x, definition.anchor.y + DRIVE_ANCHOR_RISE, definition.anchor.z);
        ctx.body.add_local_force_at(Vec3::new(0.0, 0.0, push), point);
        if ctx.tick < until {
            let extra = thrust * tuning.level_start_push * scale;
            ctx.body.add_local_force_at(Vec3::new(0.0, 0.0, extra), point);
        }
    }
}

pub fn level_models(ctx: &mut ForceCtx<'_>) {
    let level = ctx.vehicle.level();
    let maximum = ctx.vehicle.maximum_level();
    if level == 0 || maximum == 0 {
        return;
    }
    let scale = if maximum == 2 { ctx.tuning.level_two_model_scale } else { 1.0 };
    level_drive(ctx, scale);
    match level {
        1 => {
            if maximum == 2 {
                super::level1::hover_springs(ctx);
            }
            super::level1::ground_pull(ctx, scale);
        }
        2 => level2_model(ctx, scale),
        3 => todo!("level 3 model"),
        _ => {}
    }
}

pub fn level2_model(ctx: &mut ForceCtx<'_>, scale: f32) {
    for definition in &ctx.geometry.wheels {
        let contact = ctx.vehicle.any_wheel_contact();
        let coefficient =
            if contact { ctx.tuning.level_two_ground_force } else { ctx.tuning.level_two_air_force };
        let mut force = -scale * coefficient;
        let rotation = ctx.vehicle.quat;
        let inverse = crate::Quat::new(rotation.w, -rotation.x, -rotation.y, -rotation.z);
        let up = inverse.rotate(Vec3::new(0.0, 1.0, 0.0));
        if !contact && -0.5 > up.x * 0.0 + up.y + up.z * 0.0 {
            let origin = rotation.rotate(definition.anchor) + ctx.vehicle.pos;
            let direction = rotation.rotate(Vec3::new(0.0, -ctx.tuning.level_two_down_reach, 0.0));
            if ctx.collision.raycast(origin, direction).is_some() {
                force = force * ctx.tuning.level_two_inverted_force_scale;
            }
        }
        ctx.body.add_local_force_at(Vec3::new(0.0, force, 0.0), definition.anchor);
    }
}
