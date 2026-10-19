use crate::{Quat, Vec3};
use crate::math::maxss;
use super::{ForceCtx, ForceRayHit, common::{KMH, PEDAL_THRESHOLD}};

const GROUND_FORCE_SHARE: f32 = 0.5;
const ROLL_REACH: f32 = 3.0;
const ROLL_SURFACE: u8 = 0x0d;

pub fn hover_springs(ctx: &mut ForceCtx<'_>) {
    for index in 0..ctx.geometry.wheels.len() {
        hover_spring(ctx, index);
        if ctx.tuning.hover_side_enabled != 0 {
            side_push(ctx, index);
        }
    }
}

fn hover_spring(ctx: &mut ForceCtx<'_>, index: usize) {
    let tuning = ctx.tuning;
    let definition = &ctx.geometry.wheels[index];
    if tuning.car_model_kind().clears_hover_suspension() {
        ctx.vehicle.wheels[index].suspension_length = 0.0;
    }
    let rotation = ctx.vehicle.quat;
    let rotated = rotation.rotate(definition.anchor);
    let origin = rotated + ctx.vehicle.pos;
    let direction = rotation.rotate(Vec3::new(0.0, -tuning.hover_down_reach, 0.0));
    let in_water = ctx.vehicle.in_water != 0;
    let hit = if in_water {
        None
    } else {
        ctx.collision.raycast(origin, direction)
    };
    let wheel = &mut ctx.vehicle.wheels[index];
    if !in_water && hit.is_none() {
        wheel.hover_depth = 0.0;
        wheel.boost_contact = 0;
        wheel.suspension_rate = 0.0;
        return;
    }
    let fraction = if in_water {
        tuning.hover_water_fraction
    } else {
        hit.unwrap().fraction
    };
    let depth = maxss(fraction * tuning.hover_down_reach - definition.radius, 0.0);
    let rate = if wheel.boost_contact != 0 {
        (depth - wheel.hover_depth) / ctx.dt
    } else {
        0.0
    };
    wheel.hover_depth = depth;
    wheel.suspension_rate = rate;
    wheel.material = 0;
    wheel.surface = 0;
    wheel.boost_contact = (!in_water) as u32;
    let normal = hit.map_or(Vec3::new(0.0, 1.0, 0.0), |hit| hit.normal);
    wheel.contact_normal_x = normal.x;
    wheel.contact_normal_y = normal.y;
    wheel.contact_normal_z = normal.z;
    let rest = tuning.hover_rest_distance;
    let damping = tuning.mass * tuning.hover_spring_damping * rate;
    let force = maxss((rest - depth) / rest, 0.0) * (tuning.mass * tuning.hover_spring_strength) - damping;
    ctx.body.add_local_force_at(Vec3::new(force * 0.0, force, force * 0.0), definition.anchor);
}

fn side_push(ctx: &mut ForceCtx<'_>, index: usize) {
    let tuning = ctx.tuning;
    let definition = &ctx.geometry.wheels[index];
    let velocity = ctx.body.local_point_velocity(definition.anchor);
    let side = velocity.x.abs() * tuning.hover_side_speed_reach;
    let rotation = ctx.vehicle.quat;
    let origin = rotation.rotate(definition.anchor) + ctx.vehicle.pos;
    let (outward, inward) = if 0.0 > definition.anchor.x {
        (-1.0, 1.0)
    } else {
        (1.0, -1.0)
    };
    let direction = rotation.rotate(Vec3::new(outward * side, 0.0, 0.0));
    if ctx.collision.raycast(origin, direction).is_some() {
        let speed = -((inward * velocity.x + velocity.y * 0.0) + velocity.z * 0.0);
        let force = tuning.hover_side_velocity.eval(speed);
        ctx.body.add_local_force(Vec3::new(force * inward, force * 0.0, force * 0.0));
    }
    let reach = outward * tuning.hover_side_reach;
    let direction = rotation.rotate(Vec3::new(reach, 0.0, 0.0));
    if let Some(hit) = ctx.collision.raycast(origin, direction) {
        let distance = hit.fraction * tuning.hover_side_reach;
        let force = tuning.hover_side_distance.eval(distance) * GROUND_FORCE_SHARE;
        let force = Vec3::new(force * inward, force * 0.0, force * 0.0);
        ctx.body.add_local_force_at(force, definition.anchor);
        ctx.body.add_local_force(force);
    }
}

pub fn ground_pull(ctx: &mut ForceCtx<'_>, scale: f32) {
    let maximum_level = ctx.vehicle.maximum_level();
    let push = maximum_level != 2 || !ctx.vehicle.any_wheel_contact();
    for index in 0..ctx.geometry.wheels.len() {
        let definition = &ctx.geometry.wheels[index];
        if push {
            let mut force = if maximum_level == 2 {
                1.0
            } else {
                scale
            } * ctx.tuning.ground_pull_downforce;
            if ctx.vehicle.in_water != 0 {
                force = force * ctx.tuning.ground_pull_water_scale;
            }
            ctx.body.add_local_force_at(Vec3::new(0.0, force, 0.0), definition.anchor);
        }
        if ctx.vehicle.any_wheel_contact() && ctx.vehicle.in_water == 0 {
            continue;
        }
        let rotation = ctx.vehicle.quat;
        let origin = rotation.rotate(definition.anchor) + ctx.vehicle.pos;
        let direction = rotation.rotate(Vec3::new(0.0, -ctx.tuning.ground_pull_reach, 0.0));
        let Some(hit) = ctx.collision.raycast(origin, direction) else {
            ctx.vehicle.wheels[index].ground_pull_distance = 0.0;
            ctx.vehicle.ground_pull_begin = 0;
            continue;
        };
        ground_response(ctx, index, hit, scale);
        if hit.surface == ROLL_SURFACE {
            ground_roll(ctx, hit);
        }
    }
}

fn ground_response(ctx: &mut ForceCtx<'_>, index: usize, hit: ForceRayHit, scale: f32) {
    let tuning = ctx.tuning;
    let vehicle = &mut *ctx.vehicle;
    if vehicle.ground_pull_begin == 0 {
        vehicle.ground_pull_begin = ctx.tick;
    }
    let distance = maxss(hit.fraction * tuning.ground_pull_reach, 0.0);
    let wheel = &mut vehicle.wheels[index];
    let mut nearest = wheel.ground_pull_distance;
    if nearest == 0.0 || distance > nearest {
        wheel.ground_pull_distance = distance;
        nearest = distance;
    }
    let speed_response = tuning.ground_pull_by_speed.eval(vehicle.vel.y * KMH);
    let normal_y = hit.normal.y.abs();
    let zero = normal_y * 0.0;
    let distance_response = tuning.ground_pull_by_distance.eval(distance / nearest);
    let up = distance_response * (speed_response * tuning.ground_pull_strength)
        * GROUND_FORCE_SHARE * normal_y;
    let force = Vec3::new(zero, up, zero);
    ctx.body.add_local_force(force);
    ctx.body.add_local_force_at(force, ctx.geometry.wheels[index].anchor);
    let inverse_rotation = Quat::new(vehicle.quat.w, -vehicle.quat.x, -vehicle.quat.y, -vehicle.quat.z);
    let normal = inverse_rotation.rotate(hit.normal);
    let elapsed = ctx.tick.wrapping_sub(vehicle.ground_pull_begin);
    let time_response = tuning.ground_push_by_time.eval(elapsed as f32);
    let normal_x_zero = normal.x * 0.0;
    let vertical_response = tuning.ground_push_by_vertical_speed.eval(ctx.local_velocity.y * KMH);
    let distance_response = tuning.ground_push_by_distance.eval(distance / tuning.ground_pull_reach);
    let forward_response = tuning.ground_push_by_forward_speed.eval(ctx.local_velocity.z * KMH);
    let input = vehicle.controls.gas - vehicle.controls.brake;
    let strength = forward_response * (distance_response * vertical_response) * time_response * scale * input;
    ctx.body.add_local_force(Vec3::new(
        (normal.z * 0.0 - normal.y * 0.0) * strength,
        (normal_x_zero - normal.z) * strength,
        (normal.y - normal_x_zero) * strength,
    ));
}

fn ground_roll(ctx: &mut ForceCtx<'_>, hit: ForceRayHit) {
    let vehicle = &mut *ctx.vehicle;
    let steer = vehicle.steering;
    let previous = vehicle.previous_controls.steer;
    let restart = 0.0 > steer * previous
        || (PEDAL_THRESHOLD > previous.abs() && steer.abs() > PEDAL_THRESHOLD);
    let start = if restart || (vehicle.ground_roll_begin == u32::MAX && steer.abs() > PEDAL_THRESHOLD) {
        vehicle.ground_roll_begin = ctx.tick;
        ctx.tick
    } else {
        vehicle.ground_roll_begin
    };
    let start = if start == u32::MAX {
        ctx.tick
    } else {
        start
    };
    let velocity = vehicle.vel;
    let squared_speed = (velocity.x * velocity.x + velocity.y * velocity.y) + velocity.z * velocity.z;
    let speed_response = ctx.tuning.ground_roll_by_speed.eval(squared_speed.sqrt() * KMH);
    let time_response = ctx.tuning.ground_roll_by_time.eval(ctx.tick.wrapping_sub(start) as f32);
    let room = maxss(ROLL_REACH - hit.fraction * ctx.tuning.ground_pull_reach, 0.0);
    let roll = time_response * steer * speed_response * room / ROLL_REACH;
    ctx.body.add_local_torque(Vec3::new(0.0, 0.0, roll));
}
