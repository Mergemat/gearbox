use crate::Vec3;
use super::{ForceCtx, common::{PI, HALF_TURN_DEGREES, sign_one}};

const BOOSTED_DAMPING_SCALE: f32 = 3.0;

pub fn angular_control(ctx: &mut ForceCtx<'_>, no_damping: bool, freeze: bool) {
    let vehicle = &mut *ctx.vehicle;
    let tuning = ctx.tuning;
    let tick = ctx.tick;
    let dt = ctx.dt;

    let water_exit_deadline = vehicle.water_exit_until;
    let held = water_exit_deadline != u32::MAX && water_exit_deadline > tick;
    vehicle.yaw_input_direction = 0.0;
    vehicle.angular_control_scratch = 0;
    vehicle.pitch_input_direction = 0.0;
    let car_kind = tuning.car_model_kind().uses_water_angular_hold();
    if car_kind && (vehicle.in_water != 0 || held) {
        if tuning.water_enabled == 0 {
            return;
        }
        vehicle.angular_control_hold_time = tick;
        vehicle.held_pitch_rate = ctx.local_angular_velocity.x;
        vehicle.held_yaw_rate = ctx.local_angular_velocity.y;
        vehicle.roll_stabilization_active = 0;
        return;
    }
    let state = ctx.body.state();
    let rotation = state.rot.to_array();
    let [pitch_rate, yaw_rate, roll_rate] = ctx.local_angular_velocity.to_array();
    let (mut damping_pitch, mut damping_yaw, mut damping_roll) = (-pitch_rate, -yaw_rate, -roll_rate);
    let eps = super::common::EPS;

    'tail: {
        if freeze || vehicle.angular_freeze != 0 {
            if freeze {
                vehicle.angular_control_hold_time = tick;
            }
            vehicle.held_pitch_rate = pitch_rate;
            vehicle.held_yaw_rate = yaw_rate;
            vehicle.roll_stabilization_active = 0;
            break 'tail;
        }
        if tuning.roll_stabilization_enabled != 0 {
            roll_stabilization(vehicle, tuning, &mut ctx.body, rotation, pitch_rate, tick, state.vel.y);
        }
        let since = vehicle.angular_control_hold_time;
        if since != 0 && tick.wrapping_sub(since) >= tuning.air_control_window {
            break 'tail;
        }
        let steering_disabled = vehicle.has_effect_flag(8);
        let level = vehicle.level();
        let until = tuning.level_air_control_delay_ms.wrapping_add(since);
        let control = tuning.level_air_control != 0 && level != 0 && tick > until && !steering_disabled;
        let (yaw_rate, boosted) = choose_held_yaw(vehicle, tuning, yaw_rate, control);
        let mut pitch_rate = pitch_rate;
        if tuning.roll_stabilization_enabled == 0 && car_kind {
            if vehicle.controls.brake > eps && vehicle.held_pitch_rate > 0.0 && !control {
                pitch_rate = 0.0;
            }
            vehicle.held_pitch_rate = pitch_rate;
        }
        if boosted {
            let k = BOOSTED_DAMPING_SCALE;
            damping_yaw = damping_yaw * k;
            damping_pitch = damping_pitch * k;
            damping_roll = damping_roll * k;
        }
        if !no_damping {
            damping_roll = damping_roll * tuning.roll_damping.eval(roll_rate.abs());
        }
        let velocity = if control {
            apply_air_input(vehicle, tuning, &mut ctx.body, Vec3::new(pitch_rate, yaw_rate, roll_rate), dt)
        } else {
            Vec3::new(pitch_rate, yaw_rate, roll_rate)
        };
        ctx.body.set_local_angular_velocity(velocity);
    }
    damping_torque(&mut ctx.body, tuning, no_damping, damping_pitch, damping_yaw, damping_roll, eps);
}

fn roll_stabilization(
    vehicle: &mut super::super::Vehicle, tuning: &super::super::Tuning, body: &mut super::VehicleBody<'_>,
    rotation: [f32; 9], pitch_rate: f32, tick: u32, vertical_velocity: f32,
) {
    let eps = super::common::EPS;

    let (a, b) = (rotation[7], rotation[1]);
    let len = (a * a + b * b).sqrt();
    let sg = sign_one(rotation[5]);
    let s4 = rotation[4];
    let roll = crate::math::angle_approx(len, s4) * HALF_TURN_DEGREES / PI * -sg;
    let x0 = vehicle.controls.brake;
    if x0 > eps && vehicle.roll_stabilization_active == 0 {
        vehicle.roll_stabilization_start = tick;
        vehicle.roll_stabilization_active = 1;
        vehicle.roll_stabilization_released = 0;
    }
    if vehicle.roll_stabilization_active != 0 {
        let mut tq = -pitch_rate * tuning.roll_angular_damping;
        if x0 > eps && vehicle.roll_stabilization_released == 0 {
            let until = vehicle.roll_stabilization_start.wrapping_add(tuning.roll_stabilization_ms);
            if tick < until && tuning.roll_vertical_speed_limit > vertical_velocity {
                let c = tuning.roll_stabilization_force.eval(roll);
                let k = c * tuning.roll_force_scale;
                let c0 = c + 0.0;
                let fz = k * a + 0.0;
                let fy = k * b + 0.0;
                let out = [
                    (c0 * rotation[3] + fy * rotation[0]) + fz * rotation[6],
                    (c0 * rotation[4] + fy * rotation[1]) + fz * rotation[7],
                    (fy * rotation[2] + c0 * rotation[5]) + fz * rotation[8],
                ];
                body.add_local_force(out.into());
            }
            tq = tq - tuning.roll_stabilization_torque.eval(roll);
        } else {
            vehicle.roll_stabilization_released = 1;
        }
        body.add_local_torque(Vec3::new(tq, 0.0, 0.0));
    }

}

fn limit_pitch(
    vehicle: &mut super::super::Vehicle, tuning: &super::super::Tuning, body: &mut super::VehicleBody<'_>,
    pitch_rate: f32, mut drag: f32,
) -> f32 {
    let (gas, brake) = (vehicle.controls.gas, vehicle.controls.brake);

    let (quat_w, quat_x, quat_y, quat_z) = (vehicle.quat.w, vehicle.quat.x, vehicle.quat.y, vehicle.quat.z);
    let (a0, b0, c0, d0) = (quat_w * 0.0, quat_x * 0.0, quat_y * 0.0, quat_z * 0.0);
    let x1 = (quat_x - c0) + a0;
    let x4 = (b0 - c0) + quat_w;
    let x2 = (c0 - quat_z) + a0;
    let x11 = (d0 - b0) + quat_w;
    let p = x1 * quat_y - x11 * quat_z;
    let r = x11 * quat_x - x2 * quat_y;
    let qq = x2 * quat_z - x1 * quat_x;
    let y1 = (quat_y - d0) + a0;
    let x8 = (d0 - quat_x) + a0;
    let u = x4 * quat_y - quat_z * x8;
    let s2 = quat_x * x8 - y1 * quat_y;
    let t2 = y1 * quat_z - quat_x * x4;
    let fwd = (((p + p) + 0.0) * 0.0 + ((qq + qq) + 1.0)) + ((r + r) + 0.0) * 0.0;
    let up = ((((u + u) + 0.0) * 0.0) + ((t2 + t2) + 0.0)) + ((s2 + s2) + 1.0) * 0.0;
    let pitch_limit = tuning.pitch_limit_degrees * PI / HALF_TURN_DEGREES;
    let negative_limit = -pitch_limit;
    let tenth = super::common::PEDAL_THRESHOLD;
    let (gas_pressed, brake_pressed) = (gas > tenth, brake > tenth);
    let mut torque = 0.0f32;
    if (fwd > negative_limit && pitch_limit > up) || (pitch_limit > fwd && up > negative_limit) {
        if gas_pressed {
            let c = tuning.pitch_limit_scale.eval(up * HALF_TURN_DEGREES / PI);
            drag = drag * c;
        }
        if pitch_rate > 0.0 {
            torque = tuning.pitch_limit_torque.eval(pitch_rate);
        }
    }
    if (fwd > negative_limit && up > negative_limit) || (pitch_limit > fwd && pitch_limit > up) {
        if brake_pressed {
            let c = tuning.pitch_limit_scale.eval(up * HALF_TURN_DEGREES / PI);
            drag = drag * c;
        }
        if 0.0 > pitch_rate {
            torque = tuning.pitch_limit_torque.eval(pitch_rate);
        }
    }
    body.add_local_torque(Vec3::new(torque, 0.0, 0.0));

    drag
}

fn choose_held_yaw(
    vehicle: &mut super::super::Vehicle, tuning: &super::super::Tuning, yaw_rate: f32, control: bool,
) -> (f32, bool) {
    let eps = super::common::EPS;
    let mut boosted = false;
    let selected_yaw;
    if vehicle.axis_torque_enabled != 0 || control {
        selected_yaw = yaw_rate;
        vehicle.held_yaw_rate = selected_yaw;
    } else {
        let steer = vehicle.controls.steer;
        let held_rate = vehicle.held_yaw_rate;
        if (steer > eps && held_rate > 0.0) || (-eps > steer && 0.0 > held_rate) {
            selected_yaw = yaw_rate;
            boosted = true;
            vehicle.held_yaw_rate = selected_yaw;
        } else if steer.abs() > eps {
            if yaw_rate.abs() > tuning.yaw_hold_threshold {
                boosted = true;
                selected_yaw = yaw_rate;
                vehicle.held_yaw_rate = selected_yaw;
            } else {
                selected_yaw = held_rate;
            }
        } else {
            selected_yaw = yaw_rate;
            vehicle.held_yaw_rate = selected_yaw;
        }
    }

    (selected_yaw, boosted)
}

fn apply_air_input(
    vehicle: &mut super::super::Vehicle, tuning: &super::super::Tuning, body: &mut super::VehicleBody<'_>,
    angular_velocity: Vec3, dt: f32,
) -> Vec3 {
    let eps = super::common::EPS;
    let (mut pitch_rate, mut yaw_rate) = (angular_velocity.x, angular_velocity.y);
    let steering_input = -vehicle.controls.steer;
    let response = tuning.yaw_control.eval(sign_one(steering_input * yaw_rate) * yaw_rate.abs());
    yaw_rate = yaw_rate + response * (steering_input * dt);
    if steering_input != 0.0 {
        vehicle.yaw_input_direction = sign_one(steering_input);
    }
    if tuning.pitch_control_enabled != 0 {
        let (gas, brake) = (vehicle.controls.gas, vehicle.controls.brake);
        let pedal_difference = gas - brake;
        vehicle.pitch_input_active = (brake + gas > eps) as u32;
        let response = tuning.pitch_control.eval(sign_one(pedal_difference * pitch_rate) * pitch_rate.abs());
        let mut pitch_delta = response * (pedal_difference * dt);
        if pedal_difference != 0.0 {
            vehicle.pitch_input_direction = sign_one(pedal_difference);
        }
        if tuning.pitch_limit_enabled != 0 {
            pitch_delta = limit_pitch(vehicle, tuning, body, pitch_rate, pitch_delta);
        }
        pitch_rate = pitch_rate + pitch_delta;
    }

    Vec3::new(pitch_rate, yaw_rate, angular_velocity.z)
}

fn damping_torque(
    body: &mut super::VehicleBody<'_>, tuning: &super::super::Tuning,
    no_damping: bool, damping_pitch: f32, damping_yaw: f32, damping_roll: f32, eps: f32,
) {
    if !no_damping {
        let len = (damping_yaw * damping_yaw + damping_pitch * damping_pitch + damping_roll
            * damping_roll).sqrt();
        if len >= eps {
            let inv = 1.0 / len;
            let k = len * tuning.angular_drag_quadratic * len + len * tuning.angular_drag_linear;
            body.add_local_torque(Vec3::new(damping_pitch * inv * k, damping_yaw * inv * k,
                damping_roll * inv * k));
        }
    }
}
