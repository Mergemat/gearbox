use super::{ForceCtx, common::{EPS, TINY, PEDAL_THRESHOLD, PedalMode}};

const NEGATIVE_PEDAL_THRESHOLD: f32 = -PEDAL_THRESHOLD;

const MINIMUM_FORWARD_SPEED: f32 = 6.0;
const SIDE_ALIGNMENT_SCALE: f32 = 20.0;
const FRONT_LATERAL_SCALE: f32 = 1.5;

pub fn steer_grip_forces(ctx: &mut ForceCtx<'_>, blend: f32) {
    if MINIMUM_FORWARD_SPEED >= ctx.local_velocity.z || EPS > blend {
        return;
    }
    let (lateral_speed, vertical_speed, forward_speed) = (ctx.local_velocity.x, ctx.local_velocity.y,
        ctx.local_velocity.z);
    let squared_speed = vertical_speed * vertical_speed + lateral_speed * lateral_speed
        + forward_speed * forward_speed;
    let mut lateral_alignment = lateral_speed;
    if squared_speed > TINY && f32::MAX > squared_speed {
        lateral_alignment = 1.0 / squared_speed.sqrt() * lateral_speed;
    }
    let brake = ctx.vehicle.controls.brake;
    let braking = brake > PEDAL_THRESHOLD;
    let gassing = ctx.vehicle.controls.gas > PEDAL_THRESHOLD;
    if !ctx.vehicle.has_effect_flag(4) && braking {
        let k = blend * NEGATIVE_PEDAL_THRESHOLD;
        ctx.body.add_local_force(([lateral_speed * k, vertical_speed * k, forward_speed * k]).into());
    }
    let full_pedal = ctx.vehicle.effect_mode_is(PedalMode::Full);
    let cut_pedal = ctx.vehicle.effect_mode_is(PedalMode::Cut);
    let (mut front_force, mut rear_force) = ([0f32; 3], [0f32; 3]);
    let pedal = if cut_pedal {
        None
    } else if gassing && !braking {
        Some(if full_pedal {
            1.0
        } else {
            ctx.vehicle.controls.gas
        })
    } else if full_pedal {
        Some(1.0)
    } else {
        None
    };
    if let Some(pedal) = pedal {
        let lateral_scale = pedal * ctx.tuning.special_grip_lateral_scale;
        let steering_scale = pedal * ctx.tuning.special_grip_steering_scale + 1.0;
        let speed_divisor = ctx.local_velocity.z.abs() + 1.0;
        let speed_divisor = speed_divisor * speed_divisor;
        let side_alignment = lateral_alignment.abs();
        let side_scale = side_alignment * SIDE_ALIGNMENT_SCALE + 1.0;
        let (yaw_scale, yaw_damping) = (ctx.tuning.special_grip_yaw_scale,
            ctx.tuning.special_grip_yaw_damping);
        front_force = [(yaw_scale * lateral_alignment) / steering_scale * blend, blend * 0.0,
            lateral_scale * FRONT_LATERAL_SCALE * side_alignment * side_scale * yaw_damping
            / speed_divisor * blend];
        rear_force = [lateral_alignment * -1.0 * yaw_scale / steering_scale * blend, blend * 0.0,
            lateral_scale * side_alignment * side_scale * yaw_damping / speed_divisor * blend];
    }
    for index in 0..ctx.geometry.wheels.len() {
        let braking = ctx.vehicle.controls.brake > PEDAL_THRESHOLD;
        let slipping = ctx.vehicle.wheels[index].slipping != 0;
        let anchor = ctx.geometry.wheels[index].anchor.to_array();
        if index <= 1 && !braking && slipping {
            ctx.body.add_local_force_at((front_force).into(), (anchor).into());
        }
        if (2..=3).contains(&index) && !braking && slipping {
            ctx.body.add_local_force_at((rear_force).into(), (anchor).into());
        }
    }
}
