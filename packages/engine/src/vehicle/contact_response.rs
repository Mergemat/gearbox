use super::{
    Vehicle,
    absorb::{self, ContactKind},
    forces::{
        ForceCtx,
        common::{EPS, KMH, TINY},
        contact,
    },
    impacts::{ImpactModel, Summary},
};
use crate::race::countdown::{ControlFlags, ControlPhase};
use crate::{
    ItemId, Vec3,
    collision::{BodyMotion, Contact, Geometry, LocalContact},
    dyna::{BodyHandle, Dyna},
    math::maxss,
};

const WHEEL_CONTACT_ANGLE: f32 = 0.7853982;
const GRIPPING_TRAVEL: f32 = 0.667;
const CONTACT_SIDE: f32 = 0.9;
const ROOF_NORMAL: f32 = -0.75;
const IMPULSE_EPSILON: f32 = 0.00001;

impl BodyMotion for Dyna {
    fn velocity_at(&self, body: u32, point: Vec3) -> Vec3 {
        let body = self.body(BodyHandle(body));
        if self.centered { body.state.point_velocity(point) } else { body.velocity_at(point) }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Absorption {
    pub push: Vec3,
    pub solve: bool,
    pub weight: f32,
    pub volume_contact: bool,
    pub depth: bool,
}

pub fn point_velocity(vehicle: &Vehicle, center: Vec3, point: Vec3) -> Vec3 {
    let radius = point - center;
    let angular = vehicle.extra_angular_velocity;
    let linear = vehicle.extra_linear_velocity;
    Vec3::new(
        (angular.y * radius.z - angular.z * radius.y) + linear.x,
        (angular.z * radius.x - angular.x * radius.z) + linear.y,
        (angular.x * radius.y - angular.y * radius.x) + linear.z,
    )
}

pub fn absorb_contact(ctx: &mut ForceCtx<'_>, model: &ImpactModel, source: &Contact) -> Absorption {
    let geometry = ctx.collision.geometry;
    let vehicle_entry = geometry.entry(source.source_entry());
    let vehicle_item = vehicle_entry.item.and_then(ItemId::dynamic_item).expect("vehicle item");
    let item = &geometry.state.dynamic_item[vehicle_item.index()];
    if matches!(item.class, 0 | u32::MAX) {
        return Absorption::default();
    }
    let mut local = source.to_local(geometry, &*ctx.body.dyna);
    let world_velocity = ctx.body.velocity_at(source.point);
    let pending = point_velocity(ctx.vehicle, ctx.body.center_of_mass(), source.point);
    let rotation = item.iso.rot.to_array();
    local.relative_velocity = Vec3::new(
        pending.y * rotation[3]
            + pending.x * rotation[0]
            + pending.z * rotation[6]
            + local.relative_velocity.x,
        pending.x * rotation[1]
            + pending.y * rotation[4]
            + pending.z * rotation[7]
            + local.relative_velocity.y,
        rotation[5] * pending.y
            + pending.x * rotation[2]
            + pending.z * rotation[8]
            + local.relative_velocity.z,
    );
    let material = local.material_b as u8;
    if material == 13 || material == 23 {
        return Absorption::default();
    }
    if ControlFlags(ctx.vehicle.control_flags).phase() == ControlPhase::Countdown {
        return Absorption {
            push: body_matrix_times(ctx, &local, source, world_velocity, true, false),
            ..Absorption::default()
        };
    }
    let wheel = (vehicle_entry.tag as u32 != u32::MAX).then_some(vehicle_entry.tag as u32 as usize);
    let kind = contact_kind(ctx, &local, wheel);
    let mut result = contact_weight(ctx, &local, wheel, kind);
    mark_contact(ctx, model, &local, wheel, kind, result.depth);
    support_motion(ctx, source, kind, local.normal.y);
    let mut free = true;
    if let Some(index) = wheel.filter(|_| ctx.vehicle.level_consistent()) {
        if (other_layer(geometry, source) == 21 || result.depth) && kind == ContactKind::Floor {
            take_as_is(source, &mut result);
            return result;
        }
        ctx.vehicle.spin_up_duration = 0;
        wheel_contact(ctx, &mut local, index, result.depth);
        free = false;
    } else {
        if body_contact(ctx, &mut local, source, kind, result.depth) {
            take_as_is(source, &mut result);
            return result;
        }
        if ctx.tuning.suspension_kind == 2 {
            free = false;
        }
    }
    result.push = body_matrix_times(ctx, &local, source, world_velocity, free, result.depth);
    result
}

fn other_layer(geometry: Geometry<'_>, source: &Contact) -> u8 {
    geometry.entry_item(source.other_entry()).layer
}

fn take_as_is(source: &Contact, result: &mut Absorption) {
    let normal = source.normal;
    let delta = source.penetration;
    let depth = normal.y * delta.y + normal.x * delta.x + normal.z * delta.z;
    result.push = Vec3::new(normal.x * depth, normal.y * depth, normal.z * depth);
    result.solve = true;
}

pub fn contact_kind(ctx: &ForceCtx<'_>, local: &LocalContact, wheel: Option<usize>) -> ContactKind {
    if ctx.tuning.pipe_side_contact_enabled == 0 {
        return ContactKind::Ordinary;
    }
    let mut normal = local.normal;
    let mut velocity = local.penetration;
    let Some(index) = wheel else {
        return absorb::volume_contact(
            ctx.tuning,
            ctx.geometry.reference_center,
            local.point,
            &mut normal,
            &mut velocity,
        );
    };
    let definition = &ctx.geometry.wheels[index];
    let profile = crate::collision::profile::PipeProfile {
        radius: definition.radius,
        height: definition.contact_radius,
        inner: ctx.tuning.contact_volume_inner_scale,
        outer: ctx.tuning.pipe_side_width_scale,
        tilt: ctx.tuning.pipe_side_angle,
    };
    match profile.contact_normal(
        ctx.geometry.reference_center,
        ctx.vehicle.wheel_poses[index].transform.pos,
        local.point,
        &mut normal,
        &mut velocity,
    ) {
        1 => ContactKind::Side,
        3 => ContactKind::Floor,
        _ => unreachable!(),
    }
}

fn contact_weight(
    ctx: &mut ForceCtx<'_>,
    local: &LocalContact,
    wheel: Option<usize>,
    kind: ContactKind,
) -> Absorption {
    if ctx.tuning.special_contact_depth_enabled != 0 && local.material_b as u8 == b'M' {
        let definition = wheel.map(|index| {
            let wheel = &ctx.geometry.wheels[index];
            (wheel.anchor, wheel.radius)
        });
        let (depth, weight) = absorb::contact_depth(ctx.tuning, definition, local.point, kind);
        let normal = local.normal;
        let velocity = local.relative_velocity;
        let speed = -(normal.x * velocity.x + velocity.y * normal.y + velocity.z * normal.z) * KMH;
        let scale = ctx.tuning.special_contact_depth_by_speed.eval(speed);
        Absorption { weight: scale * weight, volume_contact: false, depth, ..Absorption::default() }
    } else {
        let special = matches!(kind, ContactKind::Floor | ContactKind::Outside);
        if special && ctx.tuning.impact_yaw_enabled == 0 {
            let normal = local.normal;
            let velocity = local.relative_velocity;
            let speed = -(normal.x * velocity.x + velocity.y * normal.y + velocity.z * normal.z) * KMH;
            absorb::extend_deadline(
                &ctx.tuning.volume_impact_duration,
                ctx.tuning.impact_thrust_limit_ms,
                speed,
                ctx.time_ns,
                &mut ctx.vehicle.impact_thrust_until,
            );
        }
        Absorption { volume_contact: special, ..Absorption::default() }
    }
}

fn mark_contact(
    ctx: &mut ForceCtx<'_>,
    model: &ImpactModel,
    local: &LocalContact,
    wheel: Option<usize>,
    kind: ContactKind,
    depth: bool,
) {
    let tick = (ctx.time_ns / 1_000_000) as u32;
    if ctx.tuning.slow_start_enabled != 0 && kind != ContactKind::Side {
        let mut flags = super::flags::SlowStartFlags(ctx.vehicle.slow_start_flags);
        if local.point.z > CONTACT_SIDE {
            flags.insert(super::flags::SlowStartFlags::FRONT);
        }
        if -CONTACT_SIDE > local.point.z {
            flags.insert(super::flags::SlowStartFlags::REAR);
        }
        ctx.vehicle.slow_start_flags = flags.0;
    }
    ctx.vehicle.angular_freeze = 1;
    if kind == ContactKind::Outside && ROOF_NORMAL > local.normal.y && local.point.y > model.ceiling_height {
        ctx.vehicle.last_roof_contact = tick;
    }
    let gripping = wheel.is_some_and(|index| {
        ctx.vehicle.wheels[index].suspension_length >= ctx.tuning.suspension_rest_length * GRIPPING_TRAVEL
    });
    let summary = Summary {
        material: local.material_b,
        point: local.point,
        normal: local.normal,
        velocity: local.relative_velocity,
    };
    ctx.vehicle.accumulate_contact(model, tick, summary, wheel, gripping, depth);
    let start = ctx.vehicle.spin_up_begin;
    ctx.vehicle.impact_wear(model, tick, summary, start);
}

fn support_motion(ctx: &mut ForceCtx<'_>, source: &Contact, kind: ContactKind, normal_y: f32) {
    let geometry = ctx.collision.geometry;
    let item = geometry.entry_item(source.other_entry());
    let velocity = if item.dynamic {
        BodyMotion::velocity_at(&*ctx.body.dyna, item.body, source.point)
    } else {
        Vec3::ZERO
    };
    if kind == ContactKind::Inside || (kind == ContactKind::Side && normal_y > 0.5) {
        let pending = ctx.vehicle.extra_angular_velocity;
        let linear = ctx.vehicle.extra_linear_velocity;
        let squared = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
        let angular_squared = pending.x * pending.x + pending.y * pending.y + pending.z * pending.z;
        let linear_squared = linear.x * linear.x + linear.y * linear.y + linear.z * linear.z;
        if squared + angular_squared + linear_squared > TINY {
            ctx.vehicle.moving_support(
                ctx.tuning,
                source.point,
                velocity,
                ctx.body.center_of_mass(),
                ctx.vehicle.grip,
            );
        }
    }
}

fn body_contact(
    ctx: &mut ForceCtx<'_>,
    local: &mut LocalContact,
    source: &Contact,
    kind: ContactKind,
    depth: bool,
) -> bool {
    let normal = local.normal;
    if ROOF_NORMAL > normal.y && ctx.tuning.model_kind == 5 {
        let delta = local.penetration;
        let scale = delta.y * normal.y + delta.x * normal.x + delta.z * normal.z;
        local.penetration = Vec3::new(normal.x * scale, scale * normal.y, normal.z * scale);
    }
    ctx.vehicle.pivot_blocked = 1;
    if super::flags::ForcedEffects(ctx.vehicle.forced_effect_flags).wheel_meter()
        || ctx.vehicle.wheel_meter_enabled != 0
    {
        let sample = contact::direction_drag(ctx.tuning, local.relative_velocity, normal, true);
        if sample > 0.0 {
            ctx.vehicle.wheel_meter_count = ctx.vehicle.wheel_meter_count.wrapping_add(1);
            ctx.vehicle.wheel_meter_sum = sample + ctx.vehicle.wheel_meter_sum;
        }
    }
    if ctx.tuning.suspension_kind != 2 {
        return false;
    }
    let velocity = local.relative_velocity;
    let speed = velocity.y * normal.y + normal.x * velocity.x + normal.z * velocity.z;
    if !(0.0 > speed) {
        return false;
    }
    if (other_layer(ctx.collision.geometry, source) == 21 || depth)
        && matches!(kind, ContactKind::Floor | ContactKind::Outside)
    {
        return true;
    }
    let dirt = local.material_b as u8 == 4;
    let mut friction = if dirt { ctx.tuning.dirt_body_friction } else { ctx.tuning.body_friction };
    let restitution = if dirt { ctx.tuning.dirt_body_restitution } else { ctx.tuning.body_restitution };
    if normal.y > 0.0 && ctx.tuning.floor_friction_disabled != 0 {
        friction = 0.0;
    }
    let normal_velocity = Vec3::new(normal.x * speed, speed * normal.y, normal.z * speed);
    let mut tangent = velocity - normal_velocity;
    let length = length_y_first(tangent);
    let limit = length_y_first(normal_velocity) * friction;
    if length > limit {
        tangent = (limit / length) * tangent;
    }
    let response = -(normal_velocity + tangent);
    let length = length_y_first(response);
    if length > EPS {
        let direction = (1.0 / length) * response;
        let radius = local.other_point - ctx.geometry.reference_center;
        let impulse = contact::contact_impulse(
            ctx.geometry.mass,
            ctx.geometry.local_inverse_inertia,
            -restitution,
            velocity,
            direction,
            radius,
        );
        contact::contact_response(
            ctx,
            local.material_b as u8,
            local.other_point,
            impulse,
            velocity,
            false,
            1.0,
        );
    }
    false
}

fn length_y_first(vector: Vec3) -> f32 {
    ((vector.y * vector.y + vector.x * vector.x) + vector.z * vector.z).sqrt()
}

pub fn body_matrix_times(
    ctx: &mut ForceCtx<'_>,
    local: &LocalContact,
    source: &Contact,
    velocity: Vec3,
    apply: bool,
    skip: bool,
) -> Vec3 {
    let item = ctx.collision.geometry.entry_item(source.source_entry());
    let rotation = item.iso.rot.to_array();
    let delta = local.penetration;
    let push = Vec3::new(
        delta.y * rotation[1] + delta.x * rotation[0] + delta.z * rotation[2],
        delta.x * rotation[3] + delta.y * rotation[4] + delta.z * rotation[5],
        delta.x * rotation[6] + delta.y * rotation[7] + delta.z * rotation[8],
    );
    if apply && !skip {
        contact_velocity_impulse(ctx, source, velocity);
    }
    push
}

pub fn contact_velocity_impulse(ctx: &mut ForceCtx<'_>, source: &Contact, velocity: Vec3) {
    let material_a = MATERIAL_RESPONSE[source.material_a as u8 as usize];
    let material_b = MATERIAL_RESPONSE[source.material_b as u8 as usize];
    let (first, second) = (material_a[1], material_b[1]);
    let restitution = if first > 0.0 {
        if second > 0.0 { second * first } else { second }
    } else if second > 0.0 {
        first
    } else {
        second + first
    };
    let normal = source.normal;
    let dot = (velocity.x * normal.x + velocity.y * normal.y) + velocity.z * normal.z;
    let normal = Vec3::new(normal.x * dot, normal.y * dot, normal.z * dot);
    let mut tangent = velocity - normal;
    let length = length_y_first(tangent);
    let cap = length_y_first(normal) * (material_a[0] * material_b[0]);
    if length > cap {
        tangent = (cap / length) * tangent;
    }
    let impulse = -(tangent + normal);
    let length = length_y_first(impulse);
    if !(length > IMPULSE_EPSILON) {
        return;
    }
    let direction = (1.0 / length) * impulse;
    let radius = source.point - ctx.body.center_of_mass();
    let cross = Vec3::new(
        radius.y * direction.z - radius.z * direction.y,
        radius.z * direction.x - radius.x * direction.z,
        radius.x * direction.y - radius.y * direction.x,
    );
    let inertia = ctx.vehicle.world_inverse_inertia.to_array();
    let angular = Vec3::new(
        (cross.y * inertia[1] + cross.x * inertia[0]) + cross.z * inertia[2],
        (cross.y * inertia[4] + cross.x * inertia[3]) + cross.z * inertia[5],
        (cross.y * inertia[7] + cross.x * inertia[6]) + cross.z * inertia[8],
    );
    let denominator = (((angular.y * radius.z - angular.z * radius.y) * direction.x
        + (angular.z * radius.x - angular.x * radius.z) * direction.y)
        + (angular.x * radius.y - angular.y * radius.x) * direction.z)
        + 1.0 / ctx.geometry.mass;
    let scale = ((restitution + 1.0) * length) / denominator;
    ctx.body.apply_world_impulse(scale * direction, source.point);
}

pub fn wheel_contact(ctx: &mut ForceCtx<'_>, local: &mut LocalContact, index: usize, no_response: bool) {
    if local.material_b as u8 == b'D' && ctx.tuning.pipe_side_contact_enabled != 0 {
        if let Some(normal) = super::forces::bumps::pipe_side_contact(ctx, index, local.point) {
            local.normal = normal;
            local.other_point = local.point;
        }
    }
    let (limit, _) = crate::math::sincos(WHEEL_CONTACT_ANGLE);
    let contacting = limit > local.normal.x.abs();
    let wheel = &mut ctx.vehicle.wheels[index];
    let mut flags = super::flags::ToggleFlags(wheel.contact);
    if contacting {
        flags.enable();
    }
    wheel.contact = flags.0;
    if contacting {
        wheel.contact_normal_x = wheel.contact_normal_x + local.normal.x;
        wheel.contact_normal_y = local.normal.y + wheel.contact_normal_y;
        wheel.contact_normal_z = wheel.contact_normal_z + local.normal.z;
        wheel.material = local.material_b as u8;
        wheel.surface = (local.material_b >> 8) as u8;
    } else if local.material_b as u8 != b'M' {
        ctx.vehicle.body_contact = 1;
    }
    if local.material_b >> 8 != 0 {
        update_surface_direction(ctx, local, index);
    }
    let tick = (ctx.time_ns / 1_000_000) as u32;
    let wheel = &mut ctx.vehicle.wheels[index];
    wheel.contact_point_x = local.other_point.x;
    wheel.contact_point_y = local.other_point.y;
    wheel.contact_point_z = local.other_point.z;
    let impact = if wheel.sample_time != 0 {
        true
    } else if wheel.previous_contact != 0 {
        false
    } else {
        tick > ((wheel.last_impact_time_ns as i64 / 1_000_000) as u32)
            .wrapping_add(ctx.tuning.contact_rearm_ms)
    };
    if super::flags::ForcedEffects(ctx.vehicle.forced_effect_flags).wheel_meter()
        || ctx.vehicle.wheel_meter_enabled != 0
    {
        let sample = contact::direction_drag(ctx.tuning, local.relative_velocity, local.normal, impact);
        if sample > 0.0 {
            wheel.sample_count = wheel.sample_count.wrapping_add(1);
            wheel.sample_sum = sample + wheel.sample_sum;
            if wheel.sample_time == 0 {
                wheel.sample_time = tick;
            }
        }
        wheel.last_impact_time_ns = tick as u64 * 1_000_000;
    }
    let velocity = local.relative_velocity;
    let normal = local.normal;
    if impact && ctx.tuning.contact_impact_timer_enabled != 0 {
        let speed = -((velocity.y * normal.y + normal.x * velocity.x) + velocity.z * normal.z) * KMH;
        contact::impact_timer(
            &ctx.tuning.contact_impact_duration_by_speed,
            ctx.tuning.contact_impact_limit_ms,
            speed,
            tick as i64 * 1_000_000,
            &mut ctx.vehicle.contact_impact_thrust_until,
        );
    }
    if no_response || ctx.tuning.suspension_kind != 2 {
        return;
    }
    let capped = compress_suspension(ctx, local, index);
    wheel_impulse(ctx, local, index, contacting, capped);
}

fn update_surface_direction(ctx: &mut ForceCtx<'_>, local: &LocalContact, index: usize) {
    let geometry = ctx.collision.geometry;
    let other = geometry.entry(local.b);
    let shape = other.shape.shared().expect("gameplay surface shape");
    let direction = geometry.world.statics.shapes[shape.index()].main_dir;
    let item = geometry.entry_item(local.b);
    let rotation = item.iso.rot.to_array();
    let y = direction.x * rotation[3] + direction.y * rotation[4] + direction.z * rotation[5];
    let x = direction.y * rotation[1] + direction.x * rotation[0] + direction.z * rotation[2];
    let z = direction.x * rotation[6] + direction.y * rotation[7] + direction.z * rotation[8];
    let rotation = geometry.entry(local.a).iso.rot.to_array();
    let normal = Vec3::new(
        y * rotation[3] + x * rotation[0] + z * rotation[6],
        x * rotation[1] + y * rotation[4] + z * rotation[7],
        x * rotation[2] + y * rotation[5] + z * rotation[8],
    );
    ctx.vehicle.wheels[index].surface_normal = normal;
    ctx.vehicle.wheels[index].surface_item = other.item;
}

fn compress_suspension(ctx: &mut ForceCtx<'_>, local: &mut LocalContact, index: usize) -> bool {
    let delta = local.penetration;
    let mut compression = (delta.x * 0.0 + delta.y) + delta.z * 0.0;
    if !(compression > 0.0) {
        return true;
    }
    let mut capped = false;
    let limit = ctx.tuning.suspension_grip_upper_length;
    if !(-EPS > limit) {
        let room = ctx.vehicle.wheels[index].suspension_length - limit;
        if !(room > compression) {
            compression = room;
            capped = true;
        }
    }
    let wheel = &mut ctx.vehicle.wheels[index];
    wheel.suspension_delta = maxss(wheel.suspension_delta, compression);
    let zero = compression * 0.0;
    local.penetration = Vec3::new(delta.x - zero, delta.y - compression, delta.z - zero);
    capped
}

fn wheel_impulse(ctx: &mut ForceCtx<'_>, local: &LocalContact, index: usize, contacting: bool, capped: bool) {
    let material = local.material_b as u8;
    let normal = local.normal;
    let mut velocity = local.relative_velocity;
    let center = ctx.geometry.reference_center;
    if contacting {
        let restitution =
            if material == 4 { ctx.tuning.dirt_contact_restitution } else { ctx.tuning.contact_restitution };
        let speed = (velocity.x * normal.x + velocity.y * normal.y) + velocity.z * normal.z;
        if !(0.0 > speed) {
            return;
        }
        let vertical = (normal.x * speed * 0.0 + normal.y * speed) + normal.z * speed * 0.0;
        let point = if !capped && 0.0 > vertical {
            let zero = vertical * 0.0;
            velocity = Vec3::new(velocity.x - zero, velocity.y - vertical, velocity.z - zero);
            let speed = (velocity.x * normal.x + velocity.y * normal.y) + velocity.z * normal.z;
            if !(0.0 > speed) {
                return;
            }
            ctx.vehicle.wheel_poses[index].transform.pos
        } else {
            local.other_point
        };
        let impulse = contact::contact_impulse(
            ctx.geometry.mass,
            ctx.geometry.local_inverse_inertia,
            -restitution,
            velocity,
            normal,
            point - center,
        );
        let scale = ctx.vehicle.wheel_grip[index];
        contact::contact_response(ctx, material, point, impulse, velocity, true, scale);
    } else {
        let restitution =
            if material == 4 { ctx.tuning.dirt_body_restitution } else { ctx.tuning.body_restitution };
        let speed = (normal.y * velocity.y + normal.x * velocity.x) + normal.z * velocity.z;
        if !(0.0 > speed) {
            return;
        }
        let point = Vec3::new(local.other_point.x, center.y, local.other_point.z);
        let radius = Vec3::new(point.x - center.x, center.y - center.y, point.z - center.z);
        let impulse = contact::contact_impulse(
            ctx.geometry.mass,
            ctx.geometry.local_inverse_inertia,
            -restitution,
            velocity,
            normal,
            radius,
        );
        contact::contact_response(ctx, material, point, impulse, velocity, true, 1.0);
    }
}

pub const MATERIAL_RESPONSE: [[f32; 2]; 80] = [
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [0.0, 0.0],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [0.0, 0.0],
    [0.0, -0.5],
    [0.0, 0.0],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.95],
    [1.0, 0.8],
    [1.0, 0.8],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [0.0, 0.0],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.5],
    [1.0, 0.0],
    [1.0, 0.5],
];
