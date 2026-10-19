use super::{
    ForceCtx,
    common::{KMH, TINY, sign_one},
    contact,
};
use crate::Vec3;
use crate::math::{cvtt64, minss};

const TEXTURE_SIDE: f32 = 128.0;
const TEXTURE_AMPLITUDE: f32 = 255.0;
const NS_PER_MS: i64 = 1_000_000;

pub fn surface_bumps(ctx: &mut ForceCtx<'_>) {
    let speed = ctx.body.local_velocity().z;
    for index in 0..ctx.geometry.wheels.len() {
        let wheel = &ctx.vehicle.wheels[index];
        if wheel.contact == 0 {
            continue;
        }
        let material = ctx.geometry.material(wheel.material);
        if material.bumps_enabled == 0 {
            return;
        }
        let state = ctx.body.state();
        let rotation = state.rot.to_array();
        let anchor = ctx.geometry.wheels[index].anchor;
        let world_z =
            ((anchor.x * rotation[6] + anchor.y * rotation[7]) + anchor.z * rotation[8]) + state.pos.z;
        let row = (crate::numeric::fmod(world_z as f64, material.bump_period_z as f64) as f32).abs()
            / material.bump_period_z
            * TEXTURE_SIDE;
        let row = (cvtt64(row) as u32) << 7;
        let world_x =
            ((anchor.y * rotation[1] + anchor.x * rotation[0]) + anchor.z * rotation[2]) + state.pos.x;
        let column = (crate::numeric::fmod(world_x as f64, material.bump_period_x as f64) as f32).abs()
            / material.bump_period_x
            * TEXTURE_SIDE;
        let texture_index = row.wrapping_add(cvtt64(column) as u32);
        let bump = BUMP[texture_index as usize];
        if bump == 0 {
            continue;
        }
        let force =
            minss(material.bump_limit, bump as i32 as f32 / TEXTURE_AMPLITUDE * speed * material.bump_scale);
        bump_contact(ctx, index, force);
    }
}

fn bump_contact(ctx: &mut ForceCtx<'_>, index: usize, force: f32) {
    let material = ctx.vehicle.wheels[index].material;
    let mut normal = Vec3::new(0.0, 1.0, 0.0);
    let mut point = ctx.geometry.wheels[index].anchor;
    if material == b'D' && ctx.tuning.pipe_side_contact_enabled != 0 {
        if let Some(side_normal) = pipe_side_contact(ctx, index, point) {
            normal = side_normal;
            point = ctx.geometry.wheels[index].anchor;
        }
    }
    let tick = (ctx.time_ns / NS_PER_MS) as u32;
    let wheel = &mut ctx.vehicle.wheels[index];
    wheel.contact |= 1;
    wheel.contact_normal_x = wheel.contact_normal_x + normal.x;
    wheel.contact_normal_y = normal.y + wheel.contact_normal_y;
    wheel.contact_normal_z = wheel.contact_normal_z + normal.z;
    wheel.contact_point_x = point.x;
    wheel.contact_point_y = point.y;
    wheel.contact_point_z = point.z;
    let impact = if wheel.sample_time != 0 {
        true
    } else if wheel.previous_contact != 0 {
        false
    } else {
        let previous = (wheel.last_impact_time_ns as i64 / NS_PER_MS) as u32;
        tick > previous.wrapping_add(ctx.tuning.contact_rearm_ms)
    };
    let velocity = Vec3::new(0.0, -force, 0.0);
    if ctx.vehicle.forced_effect_flags & 0x200 != 0 || ctx.vehicle.wheel_meter_enabled != 0 {
        let sample = contact::direction_drag(ctx.tuning, velocity, normal, impact);
        if sample > 0.0 {
            wheel.sample_count = wheel.sample_count.wrapping_add(1);
            wheel.sample_sum = sample + wheel.sample_sum;
            if wheel.sample_time == 0 {
                wheel.sample_time = tick;
            }
        }
        wheel.last_impact_time_ns = tick as u64 * NS_PER_MS as u64;
    }
    if impact && ctx.tuning.contact_impact_timer_enabled != 0 {
        let speed = -((velocity.y * normal.y + normal.x * velocity.x) + velocity.z * normal.z) * KMH;
        contact::impact_timer(
            &ctx.tuning.contact_impact_duration_by_speed,
            ctx.tuning.contact_impact_limit_ms,
            speed,
            tick as i64 * NS_PER_MS,
            &mut ctx.vehicle.contact_impact_thrust_until,
        );
    }
    if ctx.tuning.suspension_kind != 2 {
        return;
    }
    let restitution =
        if material == 4 { ctx.tuning.dirt_contact_restitution } else { ctx.tuning.contact_restitution };
    let into_surface = (velocity.x * normal.x + velocity.y * normal.y) + velocity.z * normal.z;
    if !(0.0 > into_surface) {
        return;
    }
    let radius = point - ctx.geometry.reference_center;
    let impulse = contact::contact_impulse(
        ctx.geometry.mass,
        ctx.geometry.local_inverse_inertia,
        -restitution,
        velocity,
        normal,
        radius,
    );
    let scale = ctx.vehicle.wheel_grip[index];
    contact::contact_response(ctx, material, point, impulse, velocity, true, scale);
}

pub(crate) fn pipe_side_contact(ctx: &ForceCtx<'_>, index: usize, point: Vec3) -> Option<Vec3> {
    let tuning = ctx.tuning;
    let anchor = ctx.vehicle.wheel_poses[index].transform.pos;
    let (sine, _) = crate::math::sincos(tuning.pipe_side_angle);
    let dx = point.x - anchor.x;
    let mut dy = point.y - anchor.y;
    let dz = point.z - anchor.z;
    if !(ctx.geometry.wheels[index].contact_radius * tuning.pipe_side_width_scale > dx.abs()) {
        return None;
    }
    let mut zero = 0.0;
    let squared = (dy * dy + 0.0) + dz * dz;
    if squared > TINY && f32::MAX > squared {
        let inverse = 1.0 / squared.sqrt();
        dy = dy * inverse;
        zero = inverse * 0.0;
    }
    let normal_y = -dy;
    let mut normal_x = -zero;
    if !(normal_y > sine) {
        return None;
    }
    let mut normal_z = TILT_COS;
    let mut normal_y = sign_one(normal_y) * TILT_SIN;
    let squared = (normal_x * normal_x + normal_y * normal_y) + normal_z * normal_z;
    if squared > TINY && f32::MAX > squared {
        let inverse = 1.0 / squared.sqrt();
        normal_y = normal_y * inverse;
        normal_x = inverse * normal_x;
        normal_z = normal_z * inverse;
    }
    Some(Vec3::new(normal_x, normal_y, normal_z))
}

const BUMP: &[u8; 16384] = include_bytes!("bump.bin");
const TILT_SIN: f32 = 0.99970144;
const TILT_COS: f32 = -0.024432242;
