use super::super::tuning::CarModelKind;
use super::{ForceCtx, common, control, model1, model5};
use crate::{Vec3, arena::VehicleEventKind};

const FROZEN_STATE: u32 = 1;
const JUMP_ACTION: u32 = 8;
const BOUNCE_ACTION: u32 = 0x10;
const BOOST_ACTION: u32 = 0x200;
const EXTERNAL_JUMP_SCALE: f32 = 1.3333334;

pub fn compute_forces(ctx: &mut ForceCtx<'_>) {
    ctx.local_velocity = ctx.body.local_velocity();
    ctx.local_angular_velocity = ctx.body.local_angular_velocity();
    ctx.local_force = ctx.body.local_force();
    if ctx.vehicle.control_flags & 0xf == FROZEN_STATE || 0.0 > ctx.geometry.water_box.half.x {
        freeze(ctx);
        return;
    }
    spin_up(ctx);
    cap_speed(ctx);
    pre_forces_jumps(ctx);
    bumper_impulse(ctx);
    ctx.vehicle.pre_forces_effects(ctx.tuning, ctx.tick, &ctx.geometry.effect_contacts);
    match ctx.tuning.car_model_kind() {
        CarModelKind::Legacy(_) => model1::forces(ctx),
        CarModelKind::Sport => model5::forces(ctx),
        CarModelKind::Other(kind @ (3 | 4 | 6 | 0xb)) => todo!("vehicle model {kind} forces"),
        CarModelKind::Other(_) => {}
    }
    post_bookkeeping(ctx);
    let control = ctx.vehicle.signed_speed_control;
    if control != 0.0 {
        let force = common::speed_drag(
            ctx.body.state().vel,
            common::asym_scale(control),
            0.0,
            ctx.tuning.control_speed_scale,
            ctx.geometry.mass,
        );
        ctx.body.add_force(force);
    }
    control::post_forces(ctx);
}

fn freeze(ctx: &mut ForceCtx<'_>) {
    ctx.body.set_local_velocity(Vec3::ZERO);
    ctx.body.set_local_angular_velocity(Vec3::ZERO);
    ctx.body.set_local_force(Vec3::ZERO);
    ctx.body.set_local_torque(Vec3::ZERO);
}

fn spin_up(ctx: &mut ForceCtx<'_>) {
    let duration = ctx.vehicle.spin_up_duration;
    if ctx.vehicle.spin_up_begin == u32::MAX {
        ctx.vehicle.spin_up_begin = ctx.tick;
        if duration != 0 {
            post_event(ctx, VehicleEventKind::SpinUp);
        }
    }
    if duration != 0 {
        let start = ctx.vehicle.spin_up_begin;
        let duration = ctx.vehicle.spin_up_duration;
        if ctx.tick >= start && ctx.tick <= start.wrapping_add(duration) {
            let torque =
                (ctx.tick - start) as f32 / duration as f32 * ctx.vehicle.spin_up_torque * ctx.geometry.mass;
            ctx.body.add_local_torque(Vec3::new(torque, 0.0, 0.0));
        }
    }
}

fn cap_speed(ctx: &mut ForceCtx<'_>) {
    let velocity = ctx.local_velocity;
    let cap = ctx.tuning.maximum_speed;
    let squared = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
    if squared > cap * cap && cap > common::EPS {
        let scale = cap / squared.sqrt();
        ctx.local_velocity = Vec3::new(velocity.x * scale, velocity.y * scale, velocity.z * scale);
        ctx.body.set_local_velocity(ctx.local_velocity);
    }
}

pub fn pre_forces_jumps(ctx: &mut ForceCtx<'_>) {
    let vehicle = &mut *ctx.vehicle;
    let tuning = ctx.tuning;
    let mass = ctx.geometry.mass;
    let contact = vehicle.any_wheel_contact();
    let elapsed = ctx.tick.wrapping_sub(vehicle.last_jump);
    let actions = vehicle.controls.actions;
    let previous = vehicle.previous_controls.actions;
    let mut impulse = None;
    match vehicle.jump_kind {
        1 => {
            if contact
                && actions & JUMP_ACTION != 0
                && previous & JUMP_ACTION == 0
                && elapsed > tuning.jump_rearm_ms
            {
                impulse = Some((mass * tuning.jump_forward, mass * tuning.jump_up, true));
            }
        }
        2 if actions & JUMP_ACTION != 0 => {
            let flagged = previous as u8 & JUMP_ACTION as u8 != 0;
            let mut late = flagged;
            if !flagged {
                if elapsed > tuning.jump_rearm_ms && (contact || vehicle.external_jump != 0) {
                    let forward = mass * tuning.held_jump_forward;
                    let scale = if vehicle.external_jump != 0 { EXTERNAL_JUMP_SCALE } else { 1.0 };
                    let up = mass * tuning.held_jump_up * scale;
                    impulse = Some((forward, up, true));
                }
                late = impulse.is_none() && previous as u8 & JUMP_ACTION as u8 != 0;
            }
            if late && elapsed < tuning.jump_repeat_ms {
                impulse = Some((mass * tuning.repeat_jump_forward, mass * tuning.repeat_jump_up, false));
            }
        }
        _ => {}
    }
    if let Some((_, _, stamp)) = impulse {
        if stamp {
            vehicle.last_jump = ctx.tick;
        }
        todo!("jump impulse");
    }
    if actions & BOUNCE_ACTION != 0 && contact {
        todo!("bounce impulse");
    }
    if actions & BOOST_ACTION != 0 && previous & BOOST_ACTION == 0 && contact {
        todo!("boost impulse");
    }
}

fn find_surface_contact(ctx: &ForceCtx<'_>, effect: u8, materials: &[u8]) -> Option<Vec3> {
    let vehicle = &*ctx.vehicle;
    if effect == vehicle.surface_contact_effect {
        let rotation = vehicle.quat;
        let inverse = crate::Quat::new(rotation.w, -rotation.x, -rotation.y, -rotation.z);
        return Some(inverse.rotate(vehicle.surface_contact_normal));
    }
    for wheel in &vehicle.wheels {
        if wheel.contact != 0 && effect == wheel.surface {
            return Some(wheel.surface_normal);
        }
    }
    for wheel in &vehicle.wheels {
        if wheel.contact != 0 && materials.contains(&wheel.material) {
            return Some(wheel.surface_normal);
        }
    }
    None
}

pub fn bumper_impulse(ctx: &mut ForceCtx<'_>) {
    let contact = find_surface_contact(ctx, 10, &[crate::surface::BUMPER])
        .map(|normal| (normal, false))
        .or_else(|| find_surface_contact(ctx, 11, &[crate::surface::BUMPER_2]).map(|normal| (normal, true)));
    let Some((normal, alternate)) = contact else { return };
    let state = ctx.body.state();
    let rotation = state.rot.to_array();
    let offset = ctx.tuning.bumper_offset;
    let center = ctx.geometry.center_of_mass;
    let local_point =
        Vec3::new(normal.x * offset + center.x, normal.y * offset + center.y, normal.z * offset + center.z);
    let point = Vec3::new(
        ((rotation[0] * local_point.x + rotation[1] * local_point.y) + rotation[2] * local_point.z)
            + state.pos.x,
        ((rotation[3] * local_point.x + rotation[4] * local_point.y) + rotation[5] * local_point.z)
            + state.pos.y,
        ((rotation[6] * local_point.x + rotation[7] * local_point.y) + rotation[8] * local_point.z)
            + state.pos.z,
    );
    let (normal_scale, forward_scale) = if alternate {
        (ctx.tuning.bumper_alt_normal, ctx.tuning.bumper_alt_forward)
    } else {
        (ctx.tuning.bumper_normal, ctx.tuning.bumper_forward)
    };
    let normal = Vec3::new(normal.x * normal_scale, normal.y * normal_scale, normal.z * normal_scale);
    let forward = if ctx.tuning.car_model_kind().uses_primary_forward() {
        Vec3::new(ctx.vehicle.adhesion_normal_x, ctx.vehicle.adhesion_normal_y, ctx.vehicle.adhesion_normal_z)
    } else {
        ctx.vehicle.alternate_forward
    };
    let impulse = Vec3::new(
        ((rotation[0] * normal.x + rotation[1] * normal.y) + rotation[2] * normal.z)
            + forward_scale * forward.x,
        ((rotation[3] * normal.x + rotation[4] * normal.y) + rotation[5] * normal.z)
            + forward_scale * forward.y,
        ((rotation[6] * normal.x + rotation[7] * normal.y) + rotation[8] * normal.z)
            + forward_scale * forward.z,
    );
    ctx.body.apply_world_impulse(impulse, point);
    post_event(ctx, VehicleEventKind::Bumper);
}

fn post_event(ctx: &mut ForceCtx<'_>, kind: VehicleEventKind) {
    if let Some(events) = ctx.events.as_deref_mut() {
        if let Some(event) = crate::arena::vehicle_event(ctx.vehicle, ctx.tick, kind) {
            events.push(event);
        }
    }
}

fn post_bookkeeping(ctx: &mut ForceCtx<'_>) {
    let vehicle = &mut *ctx.vehicle;
    if ctx.tuning.suspension_kind.wrapping_sub(5) > 1 && vehicle.level_consistent() {
        for wheel in &mut vehicle.wheels {
            wheel.previous_contact = wheel.contact;
            wheel.contact = 0;
            wheel.contact_normal_x = 0.0;
            wheel.contact_normal_y = 0.0;
            wheel.contact_normal_z = 0.0;
            wheel.contact_point_x = 0.0;
            wheel.contact_point_y = 0.0;
            wheel.contact_point_z = 0.0;
        }
    }
    vehicle.previous_controls = vehicle.controls;
    vehicle.pending_contact_force = Vec3::ZERO;
    vehicle.angular_freeze = 0;
    vehicle.body_contact = 0;
}
