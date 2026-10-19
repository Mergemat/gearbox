use crate::Vec3;
use crate::math::acos;
use super::{ForceCtx, surface::{SurfaceFactors, SurfaceResponse, average_surfaces}};
use super::common::{KMH, TINY, clamp01};

const HALF_TURN_DEGREES: f32 = 180.0;
const NS_PER_MS: i64 = 1_000_000;

pub fn angle_from_up_reached(ctx: &ForceCtx<'_>, enabled: bool) -> (bool, f32) {
    let tuning = ctx.tuning;
    if tuning.adhesion_angle_enabled == 0 || !enabled {
        return (false, 1.0);
    }
    let vehicle = &*ctx.vehicle;
    let (mut nx, mut ny, mut nz) = (
        vehicle.adhesion_normal_x,
        vehicle.adhesion_normal_y,
        vehicle.adhesion_normal_z,
    );
    let squared = ny * ny + nx * nx + nz * nz;
    if squared > TINY && f32::MAX > squared {
        let inverse = 1.0 / squared.sqrt();
        (nx, ny, nz) = (nx * inverse, ny * inverse, nz * inverse);
    }
    let angle = acos(nx * 0.0 + ny + nz * 0.0);
    let limit = tuning.adhesion_angle_degrees * std::f32::consts::PI / HALF_TURN_DEGREES;
    (!(limit > angle), angle)
}

pub fn set_gravity(ctx: &mut ForceCtx<'_>, gravity: f32) {
    ctx.vehicle.gravity = gravity;
    if ctx.vehicle.body != u32::MAX {
        ctx.body.set_gravity(gravity);
    }
}

pub fn surface_adhesion(
    ctx: &mut ForceCtx<'_>, grip_slope: &mut f32, thrust_slope: &mut f32,
) -> (SurfaceResponse, SurfaceFactors, bool) {
    let vehicle = &*ctx.vehicle;
    let tuning = ctx.tuning;
    let mut gravity_scale = clamp01(vehicle.gravity_scale * ctx.geometry.gravity_scale);
    if vehicle.vehicle_effect_active != 0 {
        gravity_scale = gravity_scale * vehicle.vehicle_effect_gravity;
    }
    if vehicle.gravity_effect_active != 0 {
        gravity_scale = gravity_scale * vehicle.gravity_effect_scale;
    }
    let contact = vehicle.any_wheel_contact();
    let state = ctx.body.state();
    let damping = if contact {
        0.0
    } else {
        -tuning.air_linear_drag
    };
    ctx.body.add_force(Vec3::new(damping * state.vel.x, damping * state.vel.y, damping * state.vel.z));
    let gravity = if contact {
        tuning.ground_gravity
    } else {
        tuning.air_gravity
    } * gravity_scale;
    set_gravity(ctx, gravity);
    let (response, factors, normal, class, any) = average_surfaces(ctx);
    if any {
        ctx.vehicle.adhesion_class = class;
        ctx.vehicle.adhesion_normal_x = normal.x;
        ctx.vehicle.adhesion_normal_y = normal.y;
        ctx.vehicle.adhesion_normal_z = normal.z;
        ctx.vehicle.adhesion_contact_time = ctx.time_ns as u64;
    }
    if tuning.adhesion_enabled != 0 {
        apply_adhesion(ctx, gravity, grip_slope, thrust_slope);
    }
    (response, factors, any)
}

fn apply_adhesion(ctx: &mut ForceCtx<'_>, gravity: f32, grip_slope: &mut f32, thrust_slope: &mut f32) {
    let tuning = ctx.tuning;
    let last_contact = ctx.vehicle.adhesion_contact_time as i64;
    let recent = last_contact != -1
        && (tuning.adhesion_hold_ms as i64 * NS_PER_MS).wrapping_add(last_contact) > ctx.time_ns;
    let forward = ctx.local_velocity.z;
    let lateral = ctx.local_velocity.x;
    let horizontal_speed = (lateral * lateral + forward * forward).sqrt() * KMH;
    let mass = ctx.geometry.mass;
    let (reached, angle) = angle_from_up_reached(ctx, recent);
    let vehicle = &*ctx.vehicle;
    let normal = Vec3::new(vehicle.adhesion_normal_x, vehicle.adhesion_normal_y, vehicle.adhesion_normal_z);
    let class = vehicle.adhesion_class;
    if reached && class == 0 {
        let response = tuning.adhesion_by_angle.eval(angle * HALF_TURN_DEGREES / std::f32::consts::PI);
        let force = response * ((-mass * gravity) * tuning.adhesion_force_scale);
        let force = force * tuning.adhesion_by_forward_speed.eval(forward * KMH);
        ctx.body.add_force(Vec3::new(force * normal.x, force * normal.y, force * normal.z));
    }
    if recent && class != 0 {
        let negative_mass = -mass;
        let strength = (negative_mass * gravity) * tuning.adhesion_force_scale;
        let mut force = Vec3::new(normal.x * strength, normal.y * strength, normal.z * strength);
        if class >= 2 {
            let response = tuning.adhesion_by_horizontal_speed.eval(horizontal_speed) * negative_mass;
            force = Vec3::new(
                normal.x * response + force.x,
                normal.y * response + force.y,
                normal.z * response + force.z,
            );
        }
        set_gravity(ctx, gravity * tuning.adhesion_gravity_scale);
        ctx.body.add_force(force);
        *grip_slope = 1.0;
        *thrust_slope = 1.0;
        let downforce = -tuning.adhesion_downforce.eval(horizontal_speed);
        ctx.body.add_local_force(Vec3::new(0.0, downforce, 0.0));
    }
}
