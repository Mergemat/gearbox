use super::super::Vehicle;
use super::{
    ForceCtx,
    common::{PedalMode, clamp01},
};
use crate::math::maxss;

const SAMPLE_SCALE: f32 = 100.0;

fn forced_value(v: &Vehicle) -> f32 {
    let a = v.forced_speed_integer as i16 as i32 as f32;
    if a == 0.0 { v.forced_speed } else { a }
}

fn forced_gas(v: &Vehicle) -> f32 {
    let velocity = v.vel;
    let q = crate::Quat::new(v.quat.w, -v.quat.x, -v.quat.y, -v.quat.z);
    let z = q.rotate(velocity).z;
    if z >= forced_value(v) { 0.0 } else { 1.0 }
}

pub fn input_overrides(ctx: &mut ForceCtx<'_>) {
    let v = &mut *ctx.vehicle;
    if v.forced_speed_integer as i16 != 0 || v.forced_speed != 0.0 {
        let g = forced_gas(v);
        v.controls.gas = g;
    }
    if v.has_effect_flag(4) && !v.effect_mode_is(PedalMode::Full) {
        v.controls.brake = 0.0;
    }
    if v.forced_effect_flags & 0x400 != 0 {
        let a = (v.pedal_speed_limit as i16 as i32).wrapping_abs() as u32 as f32;
        if ctx.local_velocity.z > a {
            v.controls.gas = 0.0;
        }
        if -a > ctx.local_velocity.z {
            v.controls.brake = 0.0;
        }
    }
}

pub fn fold_wheel_samples(ctx: &mut ForceCtx<'_>) {
    let v = &mut *ctx.vehicle;
    let model = ctx.tuning;
    let now = ctx.tick;
    if v.forced_effect_flags & 0x200 == 0 && v.wheel_meter_enabled == 0 {
        v.wheel_meter = 0.0;
        return;
    }
    let count = v.wheel_meter_count;
    if count != 0 {
        let value = v.wheel_meter_sum / (count as f32 * SAMPLE_SCALE) + v.wheel_meter;
        v.wheel_meter = clamp01(value);
    }
    v.wheel_meter_sum = 0.0;
    v.wheel_meter_count = 0;
    let mut first = now;
    for wheel in &v.wheels {
        if wheel.sample_time < first && wheel.sample_time != 0 {
            first = wheel.sample_time;
        }
    }
    let Some(expired) = v.wheels.iter().position(|wheel| {
        wheel.sample_time != 0 && now > model.sample_window_ms.wrapping_add(wheel.sample_time)
    }) else {
        return;
    };
    let mut sum = 0.0;
    let mut samples = 0;
    for (index, wheel) in v.wheels.iter_mut().enumerate() {
        if (index == expired || wheel.sample_time != 0) && wheel.sample_count != 0 {
            let average = wheel.sample_sum / wheel.sample_count as f32;
            let elapsed = wheel.sample_time.wrapping_sub(first) as f32 / model.sample_window_ms as f32;
            let expected = model.wheel_sample_expected.eval(elapsed) * model.sample_expected_scale;
            samples += 1;
            wheel.sample_count = 0;
            wheel.sample_sum = 0.0;
            wheel.sample_time = 0;
            sum = sum + (average - expected);
        }
    }
    if samples != 0 {
        let value = maxss(sum, 0.0) / SAMPLE_SCALE + v.wheel_meter;
        v.wheel_meter = clamp01(value);
    }
}

pub fn input_torque(ctx: &mut ForceCtx<'_>) {
    let v = &*ctx.vehicle;
    let k = ctx.tuning.input_torque_scale;
    let mass = ctx.geometry.mass;
    let t = [k * v.controls.pitch * mass, k * v.controls.yaw * mass, k * v.controls.roll * mass];
    ctx.body.add_local_torque(t.into());
}

pub fn post_forces(ctx: &mut ForceCtx<'_>) {
    let vehicle = &*ctx.vehicle;
    if vehicle.post_force_effect_active == 0 {
        return;
    }
    let velocity = ctx.body.state().vel;
    let boost = clamp01(vehicle.post_force_effect_boost) * super::common::PEDAL_THRESHOLD;
    let scale = ctx.geometry.mass * vehicle.post_force_speed_scale + boost;
    ctx.body.add_force(crate::Vec3::new(scale * velocity.x, scale * velocity.y, scale * velocity.z));
}
