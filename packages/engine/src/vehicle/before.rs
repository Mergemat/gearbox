use super::{
    BrakeHistory, Vehicle,
    flags::BodyFlags,
    integrate,
    step::{Event, StepCtx},
    vis,
};
use crate::race::countdown::{ControlFlags, ControlPhase};
use crate::{
    Iso, Mat3, ShapeId, Vec3,
    collision::DynaCollision,
    dyna::{BodyHandle, BodyParams, BodyState, ForceModels},
    math::{cvtt, minss},
};

#[derive(Clone, Debug, PartialEq)]
pub struct Scratch {
    pub last_inertia: Mat3,
}
impl Default for Scratch {
    fn default() -> Self {
        Self { last_inertia: Mat3::from_array([0.0; 9]) }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct WaypointEvent {
    pub tick: u32,
    pub tag: u8,
    pub vehicle: u32,
    pub last_inertia: Mat3,
}

pub struct Options {
    pub fast: bool,
    pub recent_event_duration: u32,
    pub zone_source: bool,
    pub pending: u32,
    pub registration: Option<BodyParams>,
}

pub fn update_zones(has_manager_source: bool, has_arena_source: bool) {
    if has_manager_source && has_arena_source {
        todo!("vehicle zone update");
    }
}

pub fn set_body_enabled(ctx: &mut StepCtx<'_>, enabled: bool, registration: Option<BodyParams>) {
    let flags = ctx.vehicle.body_enabled;
    if enabled == BodyFlags(flags).enabled() {
        return;
    }
    ctx.vehicle.body_enabled = BodyFlags(flags).with_enabled(enabled).0;
    if !enabled {
        disable_body(ctx);
        return;
    }
    if ctx.vehicle.body != u32::MAX {
        return;
    }
    let mut params = registration.unwrap_or_else(|| BodyParams {
        model: ctx.assets.body.clone(),
        item: ctx.vehicle.item,
        ..BodyParams::default()
    });
    if let Some(id) = params.item {
        let pose = Iso::from_quat_pos(ctx.vehicle.quat, ctx.vehicle.pos);
        let item = &mut ctx.state.dynamic_item[id.index()];
        item.iso = pose;
        item.quat = ctx.vehicle.quat;
        item.pos = ctx.vehicle.pos;
    }
    params.forces = ForceModels::Vehicle;
    ctx.vehicle.auxiliary_body = 0;
    ctx.vehicle.item = params.item;
    let pose = Iso::from_quat_pos(ctx.vehicle.quat, ctx.vehicle.pos);
    let handle = ctx.dyna.register(
        params,
        pose,
        &mut DynaCollision { world: ctx.world, state: ctx.state, manager: ctx.manager },
    );
    ctx.vehicle.body = handle.0;
    ctx.dyna.body_mut(handle).record.coefficient = ctx.vehicle.gravity;
    ctx.vehicle.world_inverse_inertia = super::spawn::post_product(
        super::spawn::transpose_product(pose.rot, ctx.assets.geometry.local_inverse_inertia),
        pose.rot,
    );
    if ctx.assets.tuning.model_kind == 13 {
        todo!("linked vehicle bodies");
    }
    registration_events(ctx);
}

fn disable_body(ctx: &mut StepCtx<'_>) {
    if ctx.vehicle.body == u32::MAX {
        return;
    }
    if ctx.vehicle.auxiliary_body != 0 {
        todo!("auxiliary body of a disabled vehicle");
    }
    let handle = BodyHandle(ctx.vehicle.body);
    ctx.dyna.remove(handle, &mut DynaCollision { world: ctx.world, state: ctx.state, manager: ctx.manager });
    ctx.vehicle.body = u32::MAX;
}

fn registration_events(ctx: &mut StepCtx<'_>) {
    const VEHICLE_BLOCKED_LAYERS: u32 = 16424;
    let items =
        [ctx.vehicle.item, ctx.vehicle.auxiliary_items[0], ctx.vehicle.auxiliary_items[1], ctx.vehicle.item2];
    for item in items.into_iter().take(ctx.vehicle.collision_items_len as usize) {
        let Some(id) = item else {
            continue;
        };
        let item = &mut ctx.state.dynamic_item[id.index()];
        item.blocked_layers = VEHICLE_BLOCKED_LAYERS;
        let receives_triggers = match ctx.assets.tuning.model_kind {
            0..=6 | 11 | 13..=15 => true,
            9 => todo!("special collision callback"),
            _ => false,
        };
        if receives_triggers {
            item.receives_triggers = true;
        }
    }
}

pub fn wheel_transforms(ctx: &mut StepCtx<'_>) {
    let Some(instance) = ctx.vehicle.shape.and_then(ShapeId::compound) else {
        return;
    };
    let instance = &mut ctx.state.compound_instance[instance.index()];
    let indices = [
        ctx.vehicle.wheel_transform_0,
        ctx.vehicle.wheel_transform_1,
        ctx.vehicle.wheel_transform_2,
        ctx.vehicle.wheel_transform_3,
    ];
    for (wheel, definition) in ctx.assets.geometry.wheels.iter().enumerate() {
        let offset = -ctx.vehicle.wheels[wheel].suspension_length;
        let zero = offset * 0.0;
        let point = Vec3::new(
            zero + definition.anchor.x,
            definition.anchor.y + ctx.assets.tuning.suspension_anchor_offset + offset,
            zero + definition.anchor.z,
        );
        if indices[wheel] != u32::MAX {
            instance.transforms[indices[wheel] as usize] = Iso::new(Mat3::IDENTITY, point);
        }
    }
}

pub fn before_dyna(ctx: &mut StepCtx<'_>, scratch: &mut Scratch, options: Options) {
    if options.zone_source {
        update_zones(true, true);
    }
    if !options.fast {
        vis::vehicle_vis(
            ctx.vehicle,
            ctx.dyna,
            &vis::Config {
                tuning: ctx.assets.tuning,
                geometry: ctx.assets.geometry,
                tick: ctx.tick,
                recent_event_duration: options.recent_event_duration,
            },
            0,
        );
    }
    let frozen = ctx.vehicle.frozen_until != u32::MAX && ctx.tick <= ctx.vehicle.frozen_until;
    let parked = matches!(
        ControlFlags(ctx.vehicle.control_flags).phase(),
        ControlPhase::Forced | ControlPhase::Locked
    );
    if !matches!(ctx.vehicle.zone_mode, 0 | 3) {
        todo!("vehicle in zone mode");
    }
    set_body_enabled(ctx, !(parked || frozen), options.registration);
    if !BodyFlags(ctx.vehicle.body_enabled).enabled() && ctx.vehicle.replica != 0 {
        todo!("disabled vehicle body hook");
    }
    wheel_transforms(ctx);
    if ctx.vehicle.attachment != 0 {
        todo!("vehicle attachment");
    }
    if ctx.vehicle.body != u32::MAX {
        refresh_body(ctx, scratch);
    }
    input_rate_limit(ctx.vehicle);
    if ctx.vehicle.pending_event != 0 {
        todo!("vehicle event 1");
    }
    waypoint_event(ctx, scratch);
    let value = (ctx.vehicle.controls.brake - 0.0) * 255.0;
    let value = if value >= 0.0 { minss(value, 255.0) } else { 0.0 };
    ctx.vehicle.brake_history.advance();
    let ring = &mut ctx.vehicle.brake_history;
    ring.samples[ring.cursor as usize] = cvtt(value) as u8;
    if options.pending != 0 {
        todo!("vehicle manager pending list");
    }
    if ctx.vehicle.auxiliary_body != 0 {
        todo!("vehicle auxiliary-body reset");
    }
    if let Some(id) = ctx.vehicle.item {
        ctx.state.dynamic_item[id.index()].blocked_layers = 0;
    }
}

fn refresh_body(ctx: &mut StepCtx<'_>, scratch: &mut Scratch) {
    let handle = BodyHandle(ctx.vehicle.body);
    ctx.dyna.body_mut(handle).record.coefficient = ctx.vehicle.gravity;
    let pose = Iso::from_quat_pos(ctx.vehicle.quat, ctx.vehicle.pos);
    ctx.dyna.set_state(
        handle,
        BodyState {
            rot: pose.rot,
            pos: pose.pos,
            quat: ctx.vehicle.quat,
            vel: ctx.vehicle.vel,
            ang_vel: ctx.vehicle.ang_vel,
        },
    );
    ctx.vehicle.world_inverse_inertia =
        integrate::world_inertia(pose.rot, ctx.assets.geometry.local_inverse_inertia);
    let inverse = ctx.dyna.body(handle).model.inverse_inertia_matrix;
    scratch.last_inertia = integrate::world_inertia(pose.rot, inverse);
    ctx.dyna.set_diagonal(handle, scratch.last_inertia);
}

fn waypoint_event(ctx: &mut StepCtx<'_>, scratch: &Scratch) {
    let flags = super::race_flags::RaceFlags(ctx.vehicle.flags);
    if !flags.has_waypoints() {
        return;
    }
    ctx.vehicle.flags = flags.clear_waypoints().0;
    if super::race_flags::RaceFlags(ctx.vehicle.flags).contains(super::race_flags::RaceFlags::MUTED)
        || ctx.vehicle.tag == u8::MAX
    {
        return;
    }
    let event = WaypointEvent {
        tick: ctx.tick,
        tag: ctx.vehicle.tag,
        vehicle: ctx.vehicle.key,
        last_inertia: scratch.last_inertia,
    };
    if let Some(events) = &mut ctx.events {
        events.push(Event::Waypoint(event));
    }
}

pub fn input_rate_limit(vehicle: &Vehicle) {
    if vehicle.gear_effect_active != 0 {
        todo!("input rate limiter");
    }
}

impl BrakeHistory {
    pub fn advance(&mut self) {
        self.cursor = self.cursor.wrapping_add(1);
        if self.cursor > 19 {
            self.cursor = 0;
        }
        if self.count < 20 {
            self.count += 1;
        }
    }
}
