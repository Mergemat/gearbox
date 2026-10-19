use crate::math::{minss, sincos as sincos_value};
use super::gearbox::GearboxState;
use super::ForceCtx;
use super::common::{
    EPS, PEDAL_THRESHOLD, KMH, PI, TINY, HALF_TURN_DEGREES, PedalMode, clamp01, sign_one,
};

const DRIFT_IDLE: u8 = 0;
const DRIFT_LEFT: u8 = 1;
const DRIFT_RIGHT: u8 = 2;
const YAW_IDLE: u8 = 0;
const YAW_FORWARD: u8 = 1;
const YAW_REVERSE: u8 = 2;
const RESET_DRIFT_THRUST: u32 = 0x20000;

const SIDE_THRUST_POWER: f32 = 1.5;

pub fn drive_forces(ctx: &mut ForceCtx<'_>, blend: f32, dt_ms: f32) {
    let tuning = ctx.tuning;
    let wheel_count = ctx.geometry.wheels.len();
    let previous_direction = ctx.vehicle.drift_direction;
    ctx.vehicle.previous_drift_direction = previous_direction;
    if EPS > blend.abs() {
        return;
    }
    ctx.vehicle.update_grip_meter(ctx.tuning.grip_meter_recovery_ms, ctx.tuning.grip_meter_minimum, dt_ms);
    let steering_scale = clamp01(ctx.geometry.steering_scale * ctx.vehicle.steering_scale);
    let average_drive = if ctx.geometry.wheels.is_empty() {
        0.0
    } else {
        ctx.vehicle.wheels.iter().take(ctx.geometry.wheels.len())
            .fold(0.0, |sum, wheel| sum + wheel.drive_coefficient)
            / ctx.geometry.wheels.len() as f32
    };
    let steer = ctx.vehicle.steering;
    let steering_magnitude = steer.abs();
    let steering_grip = average_drive * ctx.vehicle.grip_meter * steering_scale;
    let yaw = if ctx.vehicle.has_effect_flag(8) {
        0.0
    } else {
        tuning.drive_steering_degrees * PI / HALF_TURN_DEGREES * -steer * blend
    };
    update_drift_direction(ctx, steer);
    let grip_scale = clamp01(ctx.geometry.grip_scale * ctx.vehicle.grip_scale);
    for index in 0..wheel_count {
        if ctx.vehicle.wheels[index].contact != 0 {
            drive_wheel(ctx, index, blend, dt_ms, yaw, steering_grip, grip_scale, steering_magnitude);
        }
    }
}

fn update_drift_direction(ctx: &mut ForceCtx<'_>, steer: f32) {
    let tuning = ctx.tuning;
    let tick = ctx.tick;
    if ctx.vehicle.any_wheel_contact() && !ctx.vehicle.has_effect_flag(8) {
        if -PEDAL_THRESHOLD > steer {
            ctx.vehicle.drift_direction = DRIFT_LEFT;
            ctx.vehicle.drift_release_begin = u32::MAX;
        } else if steer > PEDAL_THRESHOLD {
            ctx.vehicle.drift_direction = DRIFT_RIGHT;
            ctx.vehicle.drift_release_begin = u32::MAX;
        } else {
            let mut release_begin = ctx.vehicle.drift_release_begin;
            if release_begin == u32::MAX {
                ctx.vehicle.drift_release_begin = tick;
                release_begin = tick;
            }
            if release_begin.wrapping_add(tuning.drift_release_ms) < tick {
                ctx.vehicle.drift_direction = DRIFT_IDLE;
                ctx.vehicle.drift_thrust_begin = u32::MAX;
            }
        }
        if ctx.vehicle.drift_direction != ctx.vehicle.previous_drift_direction
            || ctx.vehicle.control_flags & RESET_DRIFT_THRUST != 0
        {
            ctx.vehicle.drift_thrust_begin = tick;
            ctx.vehicle.drift_thrust_scale = 1.0;
        }
    }
}

fn drive_wheel(
    ctx: &mut ForceCtx<'_>, index: usize, blend: f32, dt_ms: f32,
    yaw: f32, steering_grip: f32, grip_scale: f32, steering_magnitude: f32,
) {
    let anchor = ctx.geometry.wheels[index].anchor.to_array();
    let point_velocity = ctx.body.local_point_velocity(ctx.geometry.wheels[index].anchor).to_array();
    let velocity = ctx.local_velocity.to_array();
    wheel_drag(ctx, anchor, point_velocity, velocity, grip_scale, blend, steering_magnitude);
    let (right_axis, forward_axis) = yaw_axes(yaw);
    let (mut ux, mut uy, mut uz) = (velocity[0], velocity[1], velocity[2]);
    let l2 = (uy * uy + ux * ux) + uz * uz;
    if l2 > TINY && f32::MAX > l2 {
        let inv = 1.0 / l2.sqrt();
        (ux, uy, uz) = (ux * inv, uy * inv, uz * inv);
    }
    let forward_alignment = (uy * forward_axis[1] + ux * forward_axis[0]) + uz * forward_axis[2];

    if !ctx.vehicle.has_effect_flag(8) {
        wheel_yaw(ctx, velocity, blend, steering_grip, grip_scale, steering_magnitude, forward_alignment);
    }
    if !(blend > EPS) || !ctx.geometry.wheels[index].steers {
        return;
    }
    let yaw_sin_cos = sincos_value(yaw);
    let thrust = pedal_thrust(ctx, blend);
    aligned_thrust(
        ctx, blend, dt_ms, yaw_sin_cos, steering_grip, steering_magnitude,
        forward_alignment, right_axis, forward_axis, velocity, point_velocity, anchor, thrust,
    );
}

fn wheel_drag(
    ctx: &mut ForceCtx<'_>, anchor: [f32; 3], point_velocity: [f32; 3], velocity: [f32; 3],
    grip_scale: f32, blend: f32, steering_magnitude: f32,
) {
    let tuning = ctx.tuning;
    let point_friction = tuning.drive_lateral_friction.eval((point_velocity[0] * KMH).abs());
    let side_sign = sign_one(point_velocity[0]);
    let side_response = tuning.drive_lateral_force_by_side_speed
        .eval(((velocity[1] * 0.0 + velocity[0]) + velocity[2] * 0.0).abs());
    let side_force = side_response * (-side_sign * point_friction) * grip_scale * blend;
    ctx.body.add_local_force_at(([side_force, side_force * 0.0, side_force * 0.0]).into(), (anchor).into());
    let steering_drag = tuning.drive_steering_drag.eval((velocity[2] * KMH).abs());
    let steering_force = -sign_one(velocity[2]) * steering_drag * grip_scale * blend;
    let steering_zero = steering_force * 0.0 * steering_magnitude;
    ctx.body.add_local_force_at(
        [steering_zero, steering_zero, steering_force * steering_magnitude].into(), anchor.into(),
    );
    let longitudinal_drag = tuning.drive_longitudinal_drag.eval((velocity[2] * KMH).abs());
    let longitudinal_force = -(longitudinal_drag * grip_scale) * velocity[2] * blend;
    ctx.body.add_local_force_at(
        [longitudinal_force * 0.0, longitudinal_force * 0.0, longitudinal_force].into(), anchor.into(),
    );
}

fn yaw_axes(yaw: f32) -> ([f32; 3], [f32; 3]) {
    let (qs, qc) = sincos_value(yaw * 0.5);
    let (q0, q1, q2, q3) = (qc, 0.0f32, qs, 0.0f32);
    let (q0z, q1z, q2z, q3z) = (q0 * 0.0, q1 * 0.0, q2 * 0.0, q3 * 0.0);
    let a = (q3 - q1z) + q0z;
    let bb = (q1z - q2) + q0z;
    let c = (q2z - q3z) + q0;
    let e0 = q2 * bb - q3 * a;
    let e1 = q3 * c - q1 * bb;
    let e2 = q1 * a - q2 * c;
    let right_axis = [(e0 + e0) + 1.0, (e2 + e2) + 0.0, (e1 + e1) + 0.0];
    let dp = (q1z - q2z) + q0;
    let ep = (q2 - q3z) + q0z;
    let fp = (q3z - q1) + q0z;
    let g0 = q2 * dp - q3 * fp;
    let gg1 = q3 * ep - q1 * dp;
    let gg2 = q1 * fp - q2 * ep;
    let forward_axis = [(g0 + g0) + 0.0, (gg1 + gg1) + 0.0, (gg2 + gg2) + 1.0];
    (right_axis, forward_axis)
}

fn wheel_yaw(
    ctx: &mut ForceCtx<'_>, velocity: [f32; 3], blend: f32,
    steering_grip: f32, grip_scale: f32, steering_magnitude: f32, forward_alignment: f32,
) {
    let tuning = ctx.tuning;

    let state = if ctx.vehicle.controls.gas > PEDAL_THRESHOLD {
        ctx.vehicle.drift_yaw_state = YAW_FORWARD;
        None
    } else {
        let mut yaw_state = ctx.vehicle.drift_yaw_state;
        if yaw_state == YAW_IDLE {
            if steering_magnitude > PEDAL_THRESHOLD {
                yaw_state = if ctx.vehicle.reversing != 0 && 0.0 > forward_alignment {
                    YAW_REVERSE
                } else {
                    YAW_FORWARD
                };
                ctx.vehicle.drift_yaw_state = yaw_state;
            }
            Some(yaw_state)
        } else if PEDAL_THRESHOLD > steering_magnitude {
            ctx.vehicle.drift_yaw_state = YAW_IDLE;
            None
        } else {
            Some(yaw_state)
        }
    };
    let yaw_response = tuning.drive_yaw_by_speed.eval(velocity[2] * KMH);
    let yaw_sign = if state == Some(YAW_REVERSE) {
        -1.0f32
    } else {
        1.0
    };
    let input_yaw = -yaw_response * (blend * ctx.vehicle.controls.steer * yaw_sign);
    let input_torque = clamp01(ctx.geometry.steering_scale * ctx.vehicle.steering_scale)
        * steering_grip * input_yaw;
    ctx.body.add_local_torque(([0.0, input_torque, 0.0]).into());
    let (mut wx, mut wy, mut wz) = (velocity[0], velocity[1], velocity[2]);
    let l2 = (wy * wy + wx * wx) + wz * wz;
    if l2 > TINY && f32::MAX > l2 {
        let inv = 1.0 / l2.sqrt();
        (wx, wy, wz) = (wx * inv, wy * inv, wz * inv);
    }
    let forward_component = (wy * 0.0 + wx * 0.0) + wz;
    let yaw_rate = ctx.local_angular_velocity.y;
    let rate_response = tuning.drive_yaw_by_rate.eval((yaw_rate * HALF_TURN_DEGREES / PI).abs());
    let alignment_response = tuning.drive_yaw_by_alignment.eval(1.0 - forward_component.abs());
    let damping = alignment_response * (-sign_one(yaw_rate) * rate_response);
    let speed_kmh = ((velocity[0] * velocity[0] + velocity[1] * velocity[1]) + velocity[2]
        * velocity[2]).sqrt() * KMH;
    let speed_response = tuning.drive_yaw_by_total_speed.eval(speed_kmh);
    let damping_torque = speed_response * damping * (grip_scale * blend);
    ctx.body.add_local_torque(([damping_torque * 0.0, damping_torque, damping_torque * 0.0]).into());
}

fn pedal_thrust(ctx: &mut ForceCtx<'_>, blend: f32) -> f32 {
    let tuning = ctx.tuning;
    let velocity = ctx.local_velocity.to_array();
    let tick = ctx.tick;
    let thrust_scale = clamp01(ctx.geometry.thrust_scale * ctx.vehicle.thrust_scale)
        * tuning.effect_minimum_grip;
    let reversing = ctx.vehicle.reversing != 0;
    let thrust_curve = if !reversing && 0.0 > velocity[2] {
        &tuning.drive_transition_thrust
    } else if reversing {
        &tuning.reverse_thrust
    } else {
        &tuning.engine_thrust
    };
    let mut timed_thrust = thrust_curve.eval(velocity[2] * KMH) * thrust_scale;
    let boost_until = ctx.vehicle.thrust_boost_until;
    if !(tick > boost_until) || boost_until == u32::MAX {
        let boost_elapsed = ctx.vehicle.thrust_boost_elapsed as f32;
        let boost_time_response = tuning.boost_thrust_by_time.eval(boost_elapsed);
        let absolute_speed_kmh = velocity[2].abs() * KMH;
        let boost_speed_response = tuning.boost_thrust_by_speed.eval(absolute_speed_kmh);
        timed_thrust = boost_speed_response * boost_time_response * timed_thrust;
        let drag_time_response = tuning.boost_drag_by_time.eval(boost_elapsed);
        let drag_speed_response = tuning.boost_drag_by_speed.eval(absolute_speed_kmh);
        let drag_scale = -(drag_speed_response * drag_time_response);
        ctx.body.add_local_force(([velocity[0] * drag_scale, velocity[1] * drag_scale, velocity[2]
            * drag_scale]).into());
    }
    let mut thrust
        = if GearboxState::from_code(ctx.vehicle.gearbox_state) == Some(GearboxState::Shifting)
        && !reversing {
        0.0
    } else {
        let slip_response = tuning.drive_thrust_by_slip.eval(blend);
        slip_response * (timed_thrust * blend)
    };
    let gas = if ctx.vehicle.effect_mode_is(PedalMode::Full) {
        1.0
    } else if ctx.vehicle.effect_mode_is(PedalMode::Cut) {
        0.0
    } else {
        (if reversing {
            -1.0f32
        } else {
            0.0
        }) * ctx.vehicle.controls.brake + ctx.vehicle.controls.gas
    };
    thrust = thrust * gas;
    if ctx.vehicle.turbo_active != 0 {
        let turbo_response = tuning.engine_thrust.eval(velocity[2] * KMH);
        let turbo_force = turbo_response * tuning.effect_minimum_grip
            * ctx.vehicle.turbo_thrust_scale * blend;
        ctx.body.add_local_force(([0.0, 0.0, turbo_force]).into());
    }
    thrust
}

fn aligned_thrust(
    ctx: &mut ForceCtx<'_>, blend: f32, dt_ms: f32, yaw_sin_cos: (f32, f32), steering_grip: f32,
        steering_magnitude: f32,
    forward_alignment: f32, right_axis: [f32; 3], forward_axis: [f32; 3], velocity: [f32; 3],
    point_velocity: [f32; 3], anchor: [f32; 3], thrust: f32,
) {
    let tuning = ctx.tuning;
    let time_ns = ctx.time_ns;
    let steer = ctx.vehicle.steering;
    let reversing = ctx.vehicle.reversing != 0;
    let (sa, ca) = yaw_sin_cos;
    let cr0 = forward_axis[1] * velocity[2] - forward_axis[2] * velocity[1];
    let cr1 = forward_axis[2] * velocity[0] - forward_axis[0] * velocity[2];
    let cr2 = forward_axis[0] * velocity[1] - forward_axis[1] * velocity[0];
    let l2 = (cr1 * cr1 + cr0 * cr0) + cr2 * cr2;
    let mut side = cr1;
    if l2 > TINY && f32::MAX > l2 {
        side = (1.0 / l2.sqrt()) * cr1;
    }
    let side = side * sign_one(steer);
    let side_boost = if reversing {
        1.0
    } else {
        let dot = (right_axis[2] * velocity[1] + right_axis[0] * velocity[0]) + right_axis[1] * velocity[2];
        let side_speed_response = tuning.drive_thrust_by_side_speed.eval(dot.abs());
        side_speed_response * crate::numeric::powf(steering_magnitude, SIDE_THRUST_POWER) + 1.0
    };
    recover_drift_thrust(ctx, dt_ms, side_boost);
    let side_response = tuning.drive_thrust_by_side.eval(sign_one(side));
    let alignment_boost = side_response * steering_magnitude;
    let side_boost = if reversing {
        0.0
    } else {
        tuning.drive_thrust_by_forward_alignment.eval(forward_alignment) * steering_magnitude
    };
    let side_boost = side_boost * alignment_boost + 1.0;
    let grip_response = tuning.drive_thrust_by_grip.eval(steering_grip);
    let alignment_boost = grip_response * steering_magnitude + 1.0;
    let horizontal_speed = (velocity[0] * velocity[0] + velocity[2] * velocity[2]).sqrt();
    let impact_boost = if time_ns < ctx.vehicle.impact_thrust_until as i64 {
        tuning.impact_thrust_by_speed.eval(horizontal_speed)
    } else {
        1.0
    };
    let contact_boost = if time_ns < ctx.vehicle.contact_impact_thrust_until as i64 {
        tuning.contact_impact_thrust_by_speed.eval(horizontal_speed)
    } else {
        1.0
    };
    let start_boost = super::boost::start_boost(ctx);
    let thrust = thrust * ctx.vehicle.drift_thrust_scale * side_boost * impact_boost * contact_boost
        * alignment_boost * 0.5;
    let scaled_thrust = start_boost * thrust;
    let (ca0, sa0) = (ca * 0.0, sa * 0.0);
    ctx.body.add_local_force(([(ca0 + sa) * scaled_thrust, (ca0 + sa0) * scaled_thrust, (ca + sa0)
        * scaled_thrust]).into());
    let pedal = if reversing {
        ctx.vehicle.controls.gas
    } else {
        -ctx.vehicle.controls.brake
    };
    let point_speed = ((point_velocity[1] * point_velocity[1] + point_velocity[0]
        * point_velocity[0]) + point_velocity[2] * point_velocity[2]).sqrt();
    let brake_response = tuning.drive_brake_by_speed.eval(point_speed * KMH);
    let brake_scale = brake_response * (pedal * 0.5) * blend;
    let brake_force = [brake_scale * point_velocity[0], point_velocity[1] * brake_scale,
        point_velocity[2] * brake_scale];
    ctx.body.add_local_force((brake_force).into());
    ctx.body.add_local_force_at((brake_force).into(), (anchor).into());
}

fn recover_drift_thrust(ctx: &mut ForceCtx<'_>, dt_ms: f32, side_boost: f32) {
    let tuning = ctx.tuning;
    let tick = ctx.tick;
    let slip = ctx.vehicle.drift_thrust_scale;
    let since = ctx.vehicle.drift_thrust_begin;
    let recovered_boost = if side_boost > slip && since != u32::MAX {
        let recovery_ms = tuning.drift_thrust_recovery_ms;
        if since.wrapping_add(recovery_ms) > tick {
            slip
        } else if since.wrapping_add(recovery_ms.wrapping_mul(2)) < tick {
            side_boost
        } else {
            let scaled_thrust = dt_ms / recovery_ms as f32 + slip;
            if scaled_thrust >= 1.0 {
                minss(scaled_thrust, side_boost)
            } else {
                1.0
            }
        }
    } else {
        side_boost
    };
    ctx.vehicle.drift_thrust_scale = recovered_boost;
}
