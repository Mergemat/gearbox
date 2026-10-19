use super::{
    ForceCtx,
    common::{KMH, TINY},
};
use crate::Vec3;

const THRUST_FLOOR: f32 = 1.5;
const THRUST_EASE_START: f32 = 90.0;
const THRUST_EASE_END: f32 = 120.0;
const THRUST_EASE_SPAN: f32 = 30.0;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceResponse {
    pub speed_limit: f32,
    pub thrust: f32,
    pub brake: f32,
    pub grip: f32,
    pub rolling: f32,
    pub sliding: f32,
    pub contact: f32,
    pub lateral_thrust: f32,
    pub lateral_drag: f32,
    pub yaw_drag: f32,
    pub gripping: u32,
    pub immersion_brake_drain: f32,
    pub steering_rate: f32,
    pub visual_steering_rate: f32,
    pub pivot_enabled: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SurfaceFactors {
    pub steering: f32,
    pub thrust: f32,
    pub grip: f32,
    pub alignment: f32,
}

pub fn surface_factors(ctx: &ForceCtx<'_>) -> SurfaceFactors {
    let vehicle = &*ctx.vehicle;
    let speed = ctx.local_velocity.z * KMH;
    let forward = vehicle.quat.rotate(Vec3::new(0.0, 0.0, 1.0));
    let Vec3 { x: mut vx, y: mut vy, z: mut vz } = vehicle.vel;
    let squared = vy * vy + vx * vx + vz * vz;
    if squared > TINY && f32::MAX > squared {
        let inverse = 1.0 / squared.sqrt();
        (vx, vy, vz) = (vx * inverse, vy * inverse, vz * inverse);
    }
    let alignment = 1.0 - (vx * forward.x + vy * forward.y + vz * forward.z);
    let mut result = SurfaceFactors::default();
    let mut count = 0;
    for wheel in &vehicle.wheels {
        if wheel.contact == 0 {
            continue;
        }
        let material = ctx.geometry.material(wheel.material);
        result.steering = material.steering_by_speed.eval(speed) + result.steering;
        result.thrust = material.thrust_by_speed.eval(speed) + result.thrust;
        result.grip = material.grip_by_immersion.eval(vehicle.immersion) + result.grip;
        result.alignment = material.thrust_by_alignment.eval(alignment) + result.alignment;
        count += 1;
    }
    if count == 0 {
        return SurfaceFactors { steering: 1.0, thrust: 1.0, grip: 1.0, alignment: 0.0 };
    }
    result.steering = result.steering / count as f32;
    result.thrust = result.thrust / count as f32;
    result.grip = result.grip / count as f32;
    result.alignment = result.alignment / count as f32;
    result
}

pub fn scale_surface(response: &mut SurfaceResponse, scale: f32) {
    response.speed_limit = scale * response.speed_limit;
    response.brake = scale * response.brake;
    response.grip = scale * response.grip;
    response.rolling = scale * response.rolling;
    response.thrust = scale * response.thrust;
    response.sliding = scale * response.sliding;
    response.contact = scale * response.contact;
    response.gripping = crate::numeric::lroundf(response.gripping as f32 * scale) as u32;
    response.visual_steering_rate = scale * response.visual_steering_rate;
    response.immersion_brake_drain = scale * response.immersion_brake_drain;
    response.steering_rate = scale * response.steering_rate;
    response.yaw_drag = scale * response.yaw_drag;
    if response.lateral_thrust > 0.0 {
        response.lateral_thrust = response.lateral_thrust * scale;
    }
    if response.lateral_drag > 0.0 {
        response.lateral_drag = response.lateral_drag * scale;
    }
}

pub fn average_surfaces(ctx: &mut ForceCtx<'_>) -> (SurfaceResponse, SurfaceFactors, Vec3, u32, bool) {
    let mut response = SurfaceResponse {
        lateral_thrust: -1.0,
        lateral_drag: -1.0,
        pivot_enabled: 1,
        ..SurfaceResponse::default()
    };
    let mut normal = Vec3::ZERO;
    let mut class = 0;
    let mut count = 0;
    let mut gripping = 0;
    for wheel in &ctx.vehicle.wheels {
        if wheel.contact == 0 {
            continue;
        }
        let material = ctx.geometry.material(wheel.material);
        if material.gripping != 0 {
            gripping += 1;
        }
        response.speed_limit = material.speed_limit_scale + response.speed_limit;
        response.thrust = material.thrust_scale + response.thrust;
        response.brake = material.brake_scale + response.brake;
        response.grip = material.grip_scale + response.grip;
        response.rolling = material.rolling_response + response.rolling;
        response.sliding = material.sliding_response + response.sliding;
        response.contact = material.contact_response + response.contact;
        response.gripping = response.gripping.wrapping_add(material.gripping);
        response.visual_steering_rate = material.steering_visual_rate + response.visual_steering_rate;
        response.immersion_brake_drain = material.immersion_brake_drain + response.immersion_brake_drain;
        response.steering_rate = material.steering_rate + response.steering_rate;
        sum_optional(&mut response.lateral_thrust, material.lateral_thrust);
        sum_optional(&mut response.lateral_drag, material.lateral_drag);
        count += 1;
        response.yaw_drag = material.yaw_drag + response.yaw_drag;
        response.pivot_enabled &= material.pivot_enabled;
        normal.x = wheel.contact_normal_x + normal.x;
        normal.y = wheel.contact_normal_y + normal.y;
        normal.z = wheel.contact_normal_z + normal.z;
        if ctx.tuning.adhesion_enabled != 0 {
            class = material_class(wheel.material);
        }
    }
    if count != 0 {
        scale_surface(&mut response, 1.0 / count as f32);
        if gripping < 3 {
            response.gripping = 0;
        }
        normal = world_contact_normal(normal, ctx.body.state().rot.to_array());
    }
    ease_surface_thrust(&mut response, ctx.vehicle.vel);
    (response, surface_factors(ctx), normal, class, count != 0)
}

fn ease_surface_thrust(response: &mut SurfaceResponse, velocity: Vec3) {
    if response.thrust > THRUST_FLOOR {
        let speed =
            ((velocity.x * velocity.x + velocity.y * velocity.y) + velocity.z * velocity.z).sqrt() * KMH;
        let fraction = if THRUST_EASE_START > speed {
            0.0
        } else if speed > THRUST_EASE_END {
            1.0
        } else {
            (speed - THRUST_EASE_START) / THRUST_EASE_SPAN
        };
        response.thrust = (response.thrust - THRUST_FLOOR) * fraction + THRUST_FLOOR;
    }
}

fn sum_optional(sum: &mut f32, value: f32) {
    if !(0.0 >= value) {
        *sum = if 0.0 > *sum { value } else { *sum + value };
    }
}

fn world_contact_normal(normal: Vec3, rotation: [f32; 9]) -> Vec3 {
    let Vec3 { x: mut nx, y: mut ny, z: mut nz } = normal;
    let squared = (nx * nx + ny * ny) + nz * nz;
    if squared > TINY && f32::MAX > squared {
        let inverse = 1.0 / squared.sqrt();
        (nx, ny, nz) = (nx * inverse, ny * inverse, nz * inverse);
    } else {
        (nx, ny, nz) = (0.0, 1.0, 0.0);
    }
    Vec3::new(
        (ny * rotation[1] + nx * rotation[0]) + nz * rotation[2],
        (nx * rotation[3] + ny * rotation[4]) + nz * rotation[5],
        (nx * rotation[6] + ny * rotation[7]) + nz * rotation[8],
    )
}

pub fn material_class(material: u8) -> u32 {
    let shifted = material.wrapping_sub(0x37);
    if !(shifted > 9 || (0x227u32 >> (shifted & 0x1f)) & 1 == 0) {
        return 1;
    }
    if material == 0x3e {
        return 2;
    }
    if material > 0x3f || (0x8001_9390_0000_0000u64 >> (material as u64 & 0x3f)) & 1 == 0 {
        return 0;
    }
    1
}

pub fn drain_meter(ctx: &mut ForceCtx<'_>, response: &SurfaceResponse) {
    use super::common::{EPS, PEDAL_THRESHOLD, clamp01};
    let vehicle = &mut *ctx.vehicle;
    if !vehicle.any_wheel_contact() || vehicle.in_water != 0 {
        return;
    }
    let mut rate = if response.gripping == 0 {
        0.0
    } else {
        1.0 / (response.gripping as f32 / (ctx.dt * MS_PER_SECOND))
    };
    let forward = ctx.local_velocity.z;
    let input = if vehicle.controls.brake > PEDAL_THRESHOLD && forward > EPS {
        Some(vehicle.controls.brake)
    } else if vehicle.controls.gas > PEDAL_THRESHOLD && -EPS > forward {
        Some(if forward > EPS { vehicle.controls.brake } else { vehicle.controls.gas })
    } else {
        None
    };
    if let Some(pedal) = input {
        rate = rate + response.immersion_brake_drain * ctx.dt * pedal;
    }
    let velocity = ctx.local_velocity;
    let speed = (velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z).sqrt() * KMH;
    let scale = ctx.tuning.immersion_drain_by_speed.eval(speed);
    vehicle.immersion = clamp01(vehicle.immersion - scale * rate);
}

const MS_PER_SECOND: f32 = 1000.0;
