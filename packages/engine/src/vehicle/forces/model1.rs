use super::common::{DIRECTION_DEAD_ZONE, KMH, PEDAL_THRESHOLD as TENTH, PedalMode, TINY, clamp01};
use super::surface::{SurfaceFactors, SurfaceResponse};
use super::{ForceCtx, adhesion, angular, boost, bumps, control, drag, surface, water, wheel_step, wheels};
use crate::math::{minss, sincos as sincos_value};

const PI: f32 = std::f32::consts::PI;

pub fn forces(ctx: &mut ForceCtx<'_>) {
    let fraction = clamp01((1.0 - ctx.vehicle.grip) / (1.0 - ctx.tuning.effect_minimum_grip));
    let all_boost = ctx.vehicle.wheels.iter().all(|wheel| wheel.has_boost_material());
    let drive_blend = if all_boost { 0.0 } else { ctx.tuning.material_slip.eval(fraction) };
    control::input_overrides(ctx);
    if ctx.tuning.water_enabled != 0 {
        water::simple_water(ctx);
    }
    ctx.vehicle.force_velocity_lateral = ctx.local_velocity.x;
    ctx.vehicle.force_velocity_vertical = ctx.local_velocity.y;
    ctx.vehicle.force_velocity_forward = ctx.local_velocity.z;
    if !ctx.vehicle.level_consistent() {
        for wheel in &mut ctx.vehicle.wheels {
            wheel.contact = wheel.boost_contact;
        }
    }
    control::fold_wheel_samples(ctx);
    if ctx.tuning.car_model_kind().updates_wheel_visuals() {
        bumps::surface_bumps(ctx);
        wheel_step::wheels_engine_step(ctx);
    }
    let (mut grip_slope, mut thrust_slope) = ctx.tuning.slope_factors(ctx.local_force);
    let grip = clamp01(ctx.geometry.grip_scale * ctx.vehicle.grip_scale);
    let remaining = 1.0 - drive_blend;
    grip_slope = grip_slope * (remaining * grip);
    thrust_slope = thrust_slope * remaining;
    let steer = ctx.vehicle.steer_angle(ctx.tuning, ctx.local_velocity);
    let (surface, factors, any) = adhesion::surface_adhesion(ctx, &mut grip_slope, &mut thrust_slope);
    surface::drain_meter(ctx, &surface);
    drag::brake_and_drag(ctx, drive_blend);
    let mut any_slip = false;
    for index in 0..ctx.geometry.wheels.len() {
        if grip_force(ctx, index, grip_slope, factors, steer) {
            any_slip = true;
        }
    }
    if !any || !ctx.tuning.car_model_kind().uses_legacy_drive_thrust() {
        return finish(ctx);
    }
    let velocity = ctx.local_velocity;
    let speed = ((velocity.x * velocity.x + velocity.y * velocity.y) + velocity.z * velocity.z).sqrt();
    select_direction(ctx, speed);
    let capacity = surface.grip * factors.grip;
    for index in 0..ctx.geometry.wheels.len() {
        steering_force(ctx, index, speed, capacity);
    }
    let thrust = engine_thrust(ctx, &surface, factors, capacity);
    braking_thrust(ctx, &surface, thrust, any_slip, thrust_slope);
    finish(ctx);
}

fn select_direction(ctx: &mut ForceCtx<'_>, speed: f32) {
    let velocity = ctx.local_velocity;
    if !ctx.vehicle.effect_mode_is_free() {
        let reverse = !(velocity.z >= 0.0)
            && !(DIRECTION_DEAD_ZONE > velocity.z.abs())
            && !(velocity.x.abs() > DIRECTION_DEAD_ZONE);
        ctx.vehicle.reversing = reverse as u32;
    } else if ctx.tuning.reverse_selection_speed > speed {
        ctx.vehicle.reversing = (ctx.vehicle.controls.brake > TENTH) as u32;
    } else if ctx.vehicle.controls.gas > TENTH {
        ctx.vehicle.reversing = 0;
    }
}

fn set_slip_flags(ctx: &mut ForceCtx<'_>) {
    for wheel in &mut ctx.vehicle.wheels {
        wheel.slipping = 1;
    }
}

fn finish(ctx: &mut ForceCtx<'_>) {
    for wheel in &mut ctx.vehicle.wheels {
        wheel.slip = if wheel.slipping != 0 { 1.0 } else { 0.0 };
    }
    let flagged = ctx
        .vehicle
        .wheels
        .iter()
        .zip(&ctx.geometry.wheels)
        .any(|(wheel, definition)| wheel.contact != 0 && definition.contact_flag);
    if 0.0 > ctx.tuning.lateral_stiffness && flagged {
        ctx.local_velocity.x = 0.0;
        ctx.body.set_local_velocity(ctx.local_velocity);
    }
    let contact = ctx.vehicle.any_wheel_contact();
    angular::angular_control(ctx, contact, flagged);
    boost::level_models(ctx);
}

fn grip_force(
    ctx: &mut ForceCtx<'_>,
    index: usize,
    grip_slope: f32,
    factors: SurfaceFactors,
    steer: f32,
) -> bool {
    let slope_grip = grip_slope;

    let mut slipping = false;

    if ctx.vehicle.level_consistent() {
        wheels::suspension_force(ctx, index);
    }
    let material =
        if ctx.vehicle.effect_flags & 0x10 != 0 { 0x4a } else { ctx.vehicle.wheels[index].material };
    let material = ctx.geometry.material(material);
    if ctx.vehicle.wheels[index].contact == 0 {
        return false;
    }
    let lateral_stiffness = ctx.tuning.lateral_stiffness;
    if !(lateral_stiffness >= 0.0) || !ctx.tuning.car_model_kind().uses_legacy_drive_thrust() {
        return false;
    }
    let slip_scale =
        if ctx.vehicle.wheels[index].slipping != 0 { ctx.tuning.slipping_grip_scale } else { 1.0 };
    let forward_speed = ctx.local_velocity.z;
    let speed_grip = ctx.tuning.grip_by_speed.eval(forward_speed * KMH);
    let immersion = ctx.vehicle.immersion;
    let wet_grip = ctx.tuning.wet_grip.eval(immersion);
    let cap = wet_grip * ((((slope_grip * material.grip_scale) * factors.grip) * speed_grip) * slip_scale);
    let (axis_x, axis_y, axis_z) = grip_axis(ctx, index, steer);
    let velocity = [ctx.local_velocity.x, ctx.local_velocity.y, ctx.local_velocity.z];
    let mut side_force =
        ((axis_y * velocity[1] + axis_x * velocity[0]) + axis_z * velocity[2]) * (-lateral_stiffness * 0.5);
    if side_force.abs() > cap {
        let k = ctx.tuning.grip_excess_blend;
        let cap = if side_force > 0.0 { cap } else { -cap };
        ctx.vehicle.wheels[index].slipping = 1;
        slipping = true;
        side_force = (1.0 - k) * cap + k * side_force;
    } else {
        ctx.vehicle.wheels[index].slipping = 0;
    }
    let (f0, f1, f2) = (axis_x * side_force, axis_y * side_force, axis_z * side_force);
    ctx.body.add_local_force(([f0, f1, f2]).into());
    let forward_speed = ctx.local_velocity.z;
    let torque_by_speed = ctx.tuning.grip_torque_by_speed.eval(forward_speed * KMH);
    let torque_by_normal = ctx.tuning.grip_torque_by_normal.eval(axis_y.abs());
    let torque_scale = -(torque_by_normal * (torque_by_speed * slope_grip));
    let (f00, f10, f20) = (f0 * 0.0, f1 * 0.0, f2 * 0.0);
    let torque = [torque_scale * f2 - f10, f00 - f20, f10 - torque_scale * f0];
    if ctx.vehicle.level_consistent() {
        ctx.body.add_local_torque((torque).into());
    }

    slipping
}

fn steering_force(ctx: &mut ForceCtx<'_>, index: usize, speed: f32, grip_capacity: f32) {
    let axle_span = ctx.geometry.axle_span;

    let steers = ctx.geometry.wheels[index].steers as u32;
    let l1 = ctx.local_velocity.y + 0.0;
    let l2 = ctx.local_velocity.z + 0.0;
    let axle_offset = if steers == 0 { -axle_span } else { axle_span } * 0.5;
    let lateral_speed = axle_offset * ctx.local_angular_velocity.y + ctx.local_velocity.x;
    let top = ctx.tuning.steering_full_response_speed;
    let steering_response = if speed > top { 1.0 } else { sincos_value(speed / top * PI * 0.5).0 };
    let speed_kmh = ctx.local_velocity.z * KMH;
    let speed_grip = ctx.tuning.grip_by_speed.eval(speed_kmh);
    let cap = speed_grip * grip_capacity;
    let stiffness = -ctx.tuning.lateral_stiffness * 0.5;
    let side = ((l1 * 0.0 + lateral_speed) + l2 * 0.0) * stiffness;
    let mut side_force = side;
    if side.abs() > cap {
        let k = ctx.tuning.lateral_excess_blend;
        let r = (1.0 - k) * cap + side.abs() * k;
        side_force = if side > 0.0 { r } else { -r };
    }
    let mut side_force = side_force * ctx.tuning.lateral_torque_scale;
    if steers != 0 {
        let sg = if ctx.vehicle.reversing != 0 { -1.0f32 } else { 1.0 };
        let slip_scale =
            if ctx.vehicle.wheels[index].slipping != 0 { ctx.tuning.slipping_steering_scale } else { 1.0 };
        let speed_steering = ctx.tuning.steering_by_speed.eval(speed_kmh);
        let steering_scale = clamp01(ctx.geometry.steering_scale * ctx.vehicle.steering_scale);
        side_force = side_force
            - ((((steering_scale * sg) * steering_response) * ctx.vehicle.steering) * speed_steering)
                * slip_scale;
    }
    if ctx.tuning.direction_drag_enabled != 0 {
        let x = ctx.vehicle.wheel_meter;
        side_force = side_force * ctx.tuning.meter_steering.eval(x);
    }
    let fs0 = side_force * 0.0;
    let torque = [fs0 * 0.0 - fs0 * axle_offset, side_force * axle_offset - fs0 * 0.0, fs0 * 0.0 - fs0];
    ctx.body.add_local_torque((torque).into());
}

fn engine_thrust(
    ctx: &mut ForceCtx<'_>,
    surface: &SurfaceResponse,
    factors: SurfaceFactors,
    capk: f32,
) -> f32 {
    let (full_pedal, cut_pedal) =
        (ctx.vehicle.effect_mode_is(PedalMode::Full), ctx.vehicle.effect_mode_is(PedalMode::Cut));
    let tick = ctx.tick;
    let forward_speed = ctx.local_velocity.z;
    let speed_kmh = forward_speed * KMH;
    let base_thrust = ctx.tuning.engine_thrust.eval_step(speed_kmh);
    let speed_grip = ctx.tuning.grip_by_speed.eval(speed_kmh);
    let lateral_demand = (ctx.tuning.lateral_stiffness * 0.5 * ctx.local_velocity.x).abs();
    let available_grip = minss(speed_grip * capk, lateral_demand);
    let steering_loss = ctx.tuning.steering_thrust_loss.eval_step(speed_kmh);
    let available_grip = available_grip * ctx.tuning.steering_thrust_loss_scale;
    let lost_thrust = steering_loss * (ctx.vehicle.steering.abs() * available_grip);
    let pedal = if full_pedal {
        1.0
    } else if cut_pedal {
        0.0
    } else {
        let sg = if ctx.vehicle.reversing != 0 { -1.0f32 } else { 0.0 };
        let s4 = surface.thrust;
        s4 * ctx.vehicle.controls.brake * sg + s4 * ctx.vehicle.controls.gas
    };
    let turbo_active = ctx.vehicle.turbo_active != 0;
    let turbo_scale = if turbo_active { ctx.vehicle.turbo_thrust_scale } else { 0.0 };
    let mut thrust = (base_thrust - lost_thrust) * (turbo_scale + pedal) * factors.thrust;
    if cut_pedal {
        thrust = turbo_scale * base_thrust;
    }
    let boost_until = ctx.vehicle.thrust_boost_until;
    if !(tick > boost_until) || boost_until == u32::MAX {
        let boost_elapsed = ctx.vehicle.thrust_boost_elapsed as f32;
        let boost_time_response = ctx.tuning.boost_thrust_by_time.eval(boost_elapsed);
        let absolute_speed_kmh = forward_speed.abs() * KMH;
        let boost_speed_response = ctx.tuning.boost_thrust_by_speed.eval(absolute_speed_kmh);
        thrust = thrust * (boost_speed_response * boost_time_response);
        let drag_time_response = ctx.tuning.boost_drag_by_time.eval(boost_elapsed);
        let velocity = [ctx.local_velocity.x, ctx.local_velocity.y, ctx.local_velocity.z];
        let drag_speed_response = ctx.tuning.boost_drag_by_speed.eval(absolute_speed_kmh);
        let k = -(drag_speed_response * drag_time_response);
        ctx.body.add_local_force(([velocity[0] * k, velocity[1] * k, velocity[2] * k]).into());
    }
    let forward_speed = ctx.local_velocity.z;
    if ctx.tuning.direction_drag_enabled != 0 {
        let x = ctx.vehicle.wheel_meter;
        thrust = thrust * ctx.tuning.meter_thrust.eval(x);
        if ctx.vehicle.any_wheel_contact() {
            let blend = ctx.tuning.meter_ground_drag.eval(forward_speed * KMH);
            ctx.body.add_local_force(([0.0, 0.0, -(blend * x)]).into());
        }
    }
    if ctx.vehicle.in_water != 0 && ctx.tuning.water_thrust_reduction != 0 {
        thrust = thrust * 0.5;
    }

    thrust
}

fn braking_thrust(
    ctx: &mut ForceCtx<'_>,
    surface: &SurfaceResponse,
    thrust: f32,
    any_slip: bool,
    thrust_slope: f32,
) {
    let cut_pedal = ctx.vehicle.effect_mode_is(PedalMode::Cut);
    let forward_speed = ctx.local_velocity.z;
    let surface_brake = surface.brake;
    let mut brake = 0.0f32;
    if forward_speed > 0.0 {
        brake = (forward_speed * ctx.tuning.brake_speed_scale + ctx.tuning.brake_constant)
            * ctx.vehicle.controls.brake;
        let pitch_offset =
            if any_slip { ctx.tuning.brake_slipping_cap } else { ctx.tuning.brake_gripping_cap };
        let immersion = ctx.vehicle.immersion;
        let cap = ctx.tuning.wet_brake.eval(immersion) * (surface_brake * pitch_offset);
        if brake > cap {
            brake = cap;
            set_slip_flags(ctx);
        }
    }
    if 0.0 > forward_speed {
        if cut_pedal {
            brake = (ctx.tuning.brake_constant - forward_speed * ctx.tuning.brake_speed_scale)
                * ctx.vehicle.controls.gas;
            let cap = surface_brake
                * (if any_slip { ctx.tuning.brake_slipping_cap } else { ctx.tuning.brake_gripping_cap });
            if brake > cap {
                brake = cap;
                set_slip_flags(ctx);
            }
            brake = -brake;
            if 1.0 > forward_speed.abs() {
                brake = brake * forward_speed.abs();
            }
        }
    } else if cut_pedal && 1.0 > forward_speed.abs() {
        brake = brake * forward_speed.abs();
    }
    if ctx.vehicle.has_effect_flag(4) {
        brake = 0.0;
    }
    let mut force = thrust - brake;
    let surface_limit = surface.speed_limit;
    let (forward_limit, reverse_limit) =
        (surface_limit * ctx.tuning.forward_speed_limit, surface_limit * ctx.tuning.reverse_speed_limit);
    if forward_speed > forward_limit {
        force = -ctx.tuning.speed_limit_force;
    }
    if -reverse_limit > forward_speed {
        force = ctx.tuning.speed_limit_force;
    }
    let force_scale = clamp01(ctx.geometry.thrust_scale * ctx.vehicle.thrust_scale);
    let force = force * (force_scale * thrust_slope);
    ctx.body.add_local_force(([0.0, 0.0, force]).into());
    let pitch_offset = ctx.tuning.thrust_pitch_offset;
    ctx.body.add_local_torque(([-force * pitch_offset, 0.0, 0.0]).into());
    let force_scale = clamp01(ctx.geometry.gravity_scale * ctx.vehicle.gravity_scale);
    let feedback = (-ctx.tuning.longitudinal_force_feedback * ctx.local_force.z)
        / (force_scale * ctx.tuning.ground_gravity);
    ctx.body.add_local_force(([0.0, 0.0, feedback]).into());
}

fn grip_axis(ctx: &ForceCtx<'_>, index: usize, steer: f32) -> (f32, f32, f32) {
    let (nx, ny, nz) = (
        ctx.vehicle.wheels[index].contact_normal_x,
        ctx.vehicle.wheels[index].contact_normal_y,
        ctx.vehicle.wheels[index].contact_normal_z,
    );
    let nz0 = nz * 0.0;
    let c0 = ny - nz0;
    let c1 = nz0 - nx;
    let c2 = nx * 0.0 - ny * 0.0;
    let l2 = (c1 * c1 + c0 * c0) + c2 * c2;
    let (mut axis_x, mut axis_y, mut axis_z) = if l2 > TINY && f32::MAX > l2 {
        let inv = 1.0 / l2.sqrt();
        (c0 * inv, c1 * inv, c2 * inv)
    } else {
        (1.0, 0.0, 0.0)
    };

    if ctx.geometry.wheels[index].steers as u32 != 0 {
        let (sn, cs) = sincos_value(steer);
        let ns = -sn;
        let ns0 = ns * 0.0;
        axis_x = cs * axis_x + ns0;
        axis_y = cs * axis_y + ns0;
        axis_z = cs * axis_z + ns;
    }
    (axis_x, axis_y, axis_z)
}
