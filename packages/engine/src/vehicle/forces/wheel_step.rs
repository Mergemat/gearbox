use super::{
    ForceCtx,
    common::{KMH, PedalMode, TINY, clamp01},
};
use crate::math::maxss;
use crate::{Iso, Mat3, Vec3};

const SPIN_WRAP: f32 = 1608.4955;
const VISUAL_STEERING_RATE: f32 = 3.0;
const VISUAL_STEERING_DEGREES: f32 = 30.0;
const HALF_TURN: f32 = 180.0;

pub fn wheel_visual(ctx: &mut ForceCtx<'_>, index: usize, speed: f32) {
    let wheel = &mut ctx.vehicle.wheels[index];
    let mut angle = ctx.dt * wheel.spin + wheel.spin_angle;
    if !(angle >= 0.0) || !(SPIN_WRAP > angle) {
        let mut remainder = crate::numeric::fmodf(angle - 0.0, SPIN_WRAP - 0.0);
        if 0.0 > remainder {
            remainder += SPIN_WRAP - 0.0;
        }
        angle = remainder + 0.0;
    }
    wheel.spin_angle = angle;
    let squared = wheel.contact_normal_x * wheel.contact_normal_x
        + wheel.contact_normal_y * wheel.contact_normal_y
        + wheel.contact_normal_z * wheel.contact_normal_z;
    if squared > TINY && f32::MAX > squared {
        let inverse = 1.0 / squared.sqrt();
        wheel.contact_normal_x *= inverse;
        wheel.contact_normal_y *= inverse;
        wheel.contact_normal_z *= inverse;
    }
    visual_steering(ctx, index, speed);
    visual_suspension(ctx, index);
    if let Some(transform_index) = ctx.geometry.wheels[index].transform_index {
        ctx.wheel_transforms[transform_index] = ctx.vehicle.wheel_poses[index].transform;
    }
}

fn visual_steering(ctx: &mut ForceCtx<'_>, index: usize, speed: f32) {
    if !ctx.geometry.wheels[index].steers {
        ctx.vehicle.wheels[index].steering_angle = 0.0;
        return;
    }
    let tuning = ctx.tuning;
    let vehicle = &*ctx.vehicle;
    let wheel = &vehicle.wheels[index];
    let speed = speed.abs() * KMH;
    let to_radians = |degrees| degrees * std::f32::consts::PI / HALF_TURN * -vehicle.steering;
    let ramp = if wheel.contact != 0 && 1.0 > vehicle.front_grip {
        Some(clamp01((1.0 - vehicle.grip) / (1.0 - tuning.effect_minimum_grip)))
    } else {
        None
    };
    let mut rate = VISUAL_STEERING_RATE;
    let angle;
    if tuning.material_visual_steering != 0 {
        let material = ctx.geometry.material(wheel.material);
        rate = material.steering_visual_rate;
        if let Some(ramp) = ramp {
            let minimum = ramp * tuning.low_grip_visual_angle;
            let degrees = material.steering_angle_by_speed.eval(speed);
            angle = to_radians(maxss(minimum, degrees));
            rate = tuning.low_grip_visual_rate;
        } else {
            let degrees = if !tuning.steering_angle_by_speed.keys.is_empty() {
                material.steering_angle_by_speed.eval(speed)
            } else {
                VISUAL_STEERING_DEGREES
            };
            angle = to_radians(degrees);
        }
    } else {
        let degrees = if !tuning.steering_angle_by_speed.keys.is_empty() {
            tuning.steering_angle_by_speed.eval(speed)
        } else {
            VISUAL_STEERING_DEGREES
        };
        angle = if let Some(ramp) = ramp {
            rate = tuning.low_grip_visual_rate;
            to_radians(maxss(ramp * tuning.low_grip_visual_angle, degrees))
        } else {
            to_radians(degrees)
        };
    }
    let wheel = &mut ctx.vehicle.wheels[index];
    let current = wheel.steering_angle;
    let step = rate * ctx.dt;
    wheel.steering_angle = if angle > current {
        let next = current + step;
        if next > angle { angle } else { next }
    } else {
        let next = current - step;
        if angle > next { angle } else { next }
    };
}

fn visual_suspension(ctx: &mut ForceCtx<'_>, index: usize) {
    let tuning = ctx.tuning;
    let wheel = &mut ctx.vehicle.wheels[index];
    let anchor = ctx.geometry.wheels[index].anchor;
    let position = match tuning.suspension_kind {
        3 => {
            let previous = wheel.suspension_length;
            let mut length = previous - wheel.suspension_delta;
            wheel.suspension_delta = 0.0;
            wheel.suspension_length = length;
            if 0.0 > length {
                wheel.suspension_length = 0.0;
                length = 0.0;
            }
            let rest = tuning.suspension_rest_length;
            let fraction = clamp01(length / rest);
            wheel.suspension_rate = (length - previous) / (rest * ctx.dt);
            let position = (1.0 - fraction) * ctx.dt * tuning.suspension_recovery_rate + length;
            wheel.suspension_length = position;
            let negative = -position;
            let side = negative * 0.0;
            let up = tuning.suspension_anchor_offset + anchor.y;
            Vec3::new(anchor.x + side, negative + up, anchor.z + side)
        }
        2 => {
            let previous = wheel.suspension_length;
            let length = previous - wheel.suspension_delta;
            let recovery =
                (tuning.suspension_rest_length - length) * ctx.dt * tuning.suspension_recovery_rate;
            wheel.suspension_delta = 0.0;
            let position = length + recovery;
            wheel.suspension_length = position;
            wheel.suspension_rate = (position - previous) / ctx.dt;
            let up = tuning.suspension_anchor_offset - position;
            let side = up * 0.0;
            Vec3::new(side + anchor.x, anchor.y + up, side + anchor.z)
        }
        _ => return,
    };
    ctx.vehicle.wheel_poses[index].transform = Iso::new(Mat3::IDENTITY, position);
}

pub fn wheels_engine_step(ctx: &mut ForceCtx<'_>) {
    let forward_speed = ctx.body.local_velocity().z;
    let (mut steering_sum, mut contacts) = (0.0, 0);
    for index in 0..ctx.geometry.wheels.len() {
        ctx.vehicle.update_wheel_spin(
            ctx.tuning,
            &ctx.geometry.wheels[index],
            index,
            forward_speed,
            ctx.dt,
            ctx.tick,
        );
        wheel_visual(ctx, index, forward_speed);
        let wheel = &ctx.vehicle.wheels[index];
        if wheel.contact != 0 {
            steering_sum += ctx.geometry.material(wheel.material).steering_rate;
            contacts += 1;
        }
    }
    let mean_steering = if contacts != 0 { steering_sum / contacts as f32 } else { 1.0 };
    let vehicle = &mut *ctx.vehicle;
    let gas = if vehicle.effect_mode_is(PedalMode::Full) {
        1.0
    } else if vehicle.effect_mode_is(PedalMode::Cut) {
        0.0
    } else if vehicle.reversing != 0 {
        vehicle.controls.brake
    } else {
        vehicle.controls.gas
    };
    vehicle.update_engine_gearbox(ctx.tuning, gas, ctx.dt);
    let mut steer = if vehicle.has_effect_flag(8) { 0.0 } else { vehicle.controls.steer };
    let rate = mean_steering * ctx.tuning.steering_rate;
    if !(0.0 >= rate) {
        let current = vehicle.steering;
        let sign = if 0.0 > current - steer { 1.0 } else { -1.0 };
        let next = sign * rate * ctx.dt + current;
        let past = if steer > current { next > steer } else { steer > next };
        if !past {
            steer = next;
        }
    }
    vehicle.steering = steer;
    let gear = vehicle.gear;
    if vehicle.cached_gear != gear {
        vehicle.cached_gear = gear;
    }
}
