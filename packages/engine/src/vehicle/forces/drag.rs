use crate::math::minss;
use super::{ForceCtx, common::PedalMode};

pub fn brake_and_drag(ctx: &mut ForceCtx<'_>, blend: f32) {
    let vehicle = &mut *ctx.vehicle;
    let tuning = ctx.tuning;
    let tick = ctx.tick;
    let grip = vehicle.grip;
    let model_kind = tuning.car_model_kind();
    if model_kind.uses_water_angular_hold() && vehicle.in_water != 0 && !vehicle.any_wheel_contact() {
        return;
    }
    let (normalization_threshold, maximum_length) = (super::common::TINY, f32::MAX);
    let zero_threshold = super::common::EPS;
    if !vehicle.effect_mode_is(PedalMode::Full) {
        let pedal = if vehicle.reversing == 0 {
            vehicle.controls.gas
        } else {
            vehicle.controls.brake
        };
        if zero_threshold > pedal || vehicle.effect_mode_is(PedalMode::Cut) {
            let [x, y, z] = ctx.local_velocity.to_array();
            let l2 = (y * y + x * x) + z * z;
            if l2 > normalization_threshold && maximum_length > l2 {
                let inv = 1.0 / l2.sqrt();
                let c = -tuning.coast_friction;
                let mut f = [inv * x * c, inv * y * c, inv * z * c];
                if !vehicle.effect_mode_is(PedalMode::Cut) {
                    let c2 = -tuning.coast_drag;
                    f = [x * c2 + f[0], y * c2 + f[1], z * c2 + f[2]];
                }
                let b = vehicle.immersion;
                let e = tuning.wet_brake.eval(b);
                let mut g = ctx.geometry.grip_scale * vehicle.grip_scale;
                g = if g >= 0.0 {
                    minss(g, 1.0)
                } else {
                    0.0
                };
                let g = g * grip * e;
                let rest = 1.0 - blend;
                ctx.body.add_local_force(([f[0] * g * rest, f[1] * g * rest, f[2] * g * rest]).into());
            }
        }
    }
    let body_contact = vehicle.body_contact;
    let [x, y, z] = ctx.local_velocity.to_array();
    if model_kind.uses_legacy_contact_drag() {
        if body_contact != 0 {
            let s = -tuning.body_contact_drag.eval_step(z * super::common::KMH);
            ctx.body.add_local_force(([s * x * grip, s * y * grip, z * s * grip]).into());
        }
    } else {
        let base = if body_contact != 0 {
            vehicle.last_body_contact = tick;
            Some(tick)
        } else {
            let b = vehicle.last_body_contact;
            if tick < b {
                None
            } else {
                Some(b)
            }
        };
        if let Some(b) = base {
            if tick - b < tuning.body_contact_drag_ms {
                let len = ((x * x + y * y) + z * z).sqrt();
                if len > zero_threshold {
                    let inv = 1.0 / len;
                    let s = -tuning.body_contact_drag.eval(len * super::common::KMH);
                    ctx.body.add_local_force(([
                        x * inv * s * grip,
                        y * inv * s * grip,
                        z * inv * s * grip,
                    ]).into());
                }
            }
        }
    }
    airborne_drag(ctx);
}

fn airborne_drag(ctx: &mut ForceCtx<'_>) {
    let vehicle = &*ctx.vehicle;
    let tuning = ctx.tuning;
    let (normalization_threshold, maximum_length) = (super::common::TINY, f32::MAX);
    if !vehicle.any_wheel_contact() {
        let (mut wx, mut wy, mut wz) = (vehicle.ang_vel.x, vehicle.ang_vel.y, vehicle.ang_vel.z);
        let l2 = (wy * wy + wx * wx) + wz * wz;
        let mut len = 0.0f32;
        if l2 > normalization_threshold && maximum_length > l2 {
            len = l2.sqrt();
            let inv = 1.0 / len;
            (wx, wy, wz) = (wx * inv, wy * inv, wz * inv);
        }
        let a = -tuning.air_angular_drag.eval(len * len) * len;
        let q = [vehicle.quat.w, -vehicle.quat.x, -vehicle.quat.y, -vehicle.quat.z];
        let tq = crate::Quat::from_array(q).rotate(([wx * a, wy * a, wz * a]).into());
        ctx.body.add_local_torque(tq);
    }
}
