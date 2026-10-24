use super::{
    Tuning, Vehicle, contacts, effects,
    forces::{self, ForceCtx, ForceRays, VehicleBody},
    impacts::ImpactModel,
    integrate,
    model::ForceGeometry,
};
use crate::race::countdown::{ControlFlags, ControlPhase};
use crate::{
    CompoundInstanceId, ShapeId, State, Vec3, World,
    collision::{ContactList, DynaCollision, Geometry, Lists, QueryFilter},
    dyna::{BodyHandle, Collision, Dyna},
    math::{cvtt64, cvtt64d},
};

const COLLISION_CLIP: f32 = 0.01;
const SPIN_RESPONSE_THRESHOLD: f32 = 0.1;
const NS_PER_MS: i64 = 1_000_000;
const NS_PER_SECOND: f64 = -1_000_000_000.0;

pub struct Assets<'a> {
    pub tuning: &'a Tuning,
    pub geometry: &'a ForceGeometry,
    pub impacts: &'a ImpactModel,
    pub fluid: &'a crate::water::drag::FluidTuning,
    pub body: &'a crate::dyna::DynaModel,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    Force(crate::arena::VehicleEvent),
    Surface(effects::Event),
    Waypoint(super::before::WaypointEvent),
    Locked { tick: u32, vehicle: u32 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionFrame {
    pub time_ns: i64,
    pub dt: f32,
    pub contacts: ContactList,
    pub output: contacts::Output,
}

pub struct StepCtx<'a> {
    pub world: &'a mut World,
    pub state: &'a mut State,
    pub manager: usize,
    pub dyna: &'a mut Dyna,
    pub vehicle: &'a mut Vehicle,
    pub assets: Assets<'a>,
    pub tick: u32,
    pub dt: f32,
    pub ghost_contacts: &'a [super::ghost::Contact],
    pub events: Option<&'a mut Vec<Event>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Schedule {
    pub remaining: f32,
    pub substep: f32,
    pub clock_step: i64,
    pub repeats: u32,
}

pub fn schedule(vehicle: &Vehicle, tuning: &Tuning, dyna: &Dyna, dt: f32) -> Schedule {
    let body = dyna.body(BodyHandle(vehicle.body));
    let length = |value: Vec3| {
        let squared = (value.x * value.x + value.y * value.y) + value.z * value.z;
        if 0.0 > squared {
            todo!("sqrtf of a negative");
        }
        squared.sqrt()
    };
    let power = vehicle.level_power(tuning);
    let remaining = power * dt;
    let mut distance = length(body.state.vel) + length(body.state.ang_vel);
    distance += length(vehicle.extra_linear_velocity);
    distance += length(vehicle.extra_angular_velocity);
    distance *= remaining;
    distance /= body.model.substep_distance;
    let count = (cvtt64(distance) as u32).wrapping_add(1);
    let maximum = crate::dyna::solver::MAX_SUBSTEPS as u32;
    let (substep, interval, repeats) = if count > maximum {
        let interval = dt / maximum as f32;
        (power * interval, interval, maximum - 1)
    } else if count > 1 {
        let interval = dt / count as f32;
        (interval * power, interval, count - 1)
    } else {
        (power, 0.0, 0)
    };
    Schedule { remaining, substep, repeats, clock_step: cvtt64d(interval as f64 * NS_PER_SECOND) }
}

pub fn substeps(ctx: &mut StepCtx<'_>) -> Vec<CollisionFrame> {
    if ControlFlags(ctx.vehicle.control_flags).phase() == ControlPhase::Forced {
        return Vec::new();
    }
    let mut flags = contacts::ContactFlags(ctx.vehicle.flags);
    flags.set(contacts::ContactFlags::HARD, false);
    flags.set(contacts::ContactFlags::OPPOSING, false);
    ctx.vehicle.flags = flags.0;
    if ctx.vehicle.zone_mode != 0 && ctx.vehicle.zone_mode != 3 {
        ctx.vehicle.previous_controls = ctx.vehicle.controls;
        return Vec::new();
    }
    if ctx.vehicle.body == u32::MAX || !ctx.dyna.is_dynamic_body(BodyHandle(ctx.vehicle.body)) {
        return Vec::new();
    }
    let plan = schedule(ctx.vehicle, ctx.assets.tuning, ctx.dyna, ctx.dt);
    let mut remaining = plan.remaining;
    let mut now = ctx.tick as i64 * NS_PER_MS;
    let mut frames = Vec::with_capacity(plan.repeats as usize + 1);
    for _ in 0..plan.repeats {
        remaining -= plan.substep;
        frames.push(substep(ctx, now, plan.substep, remaining));
        now = now.wrapping_sub(plan.clock_step);
    }
    frames.push(substep(ctx, now, remaining, 0.0));
    let flags = contacts::ContactFlags(ctx.vehicle.flags);
    if flags.contains(contacts::ContactFlags::OPPOSING)
        && flags.contains(contacts::ContactFlags::SOLVED)
        && ctx.vehicle.contact_solve_residual > SPIN_RESPONSE_THRESHOLD
        && !matches!(
            ControlFlags(ctx.vehicle.control_flags).phase(),
            ControlPhase::Forced | ControlPhase::Locked
        )
    {
        lock_vehicle(ctx);
    }
    frames
}

fn lock_vehicle(ctx: &mut StepCtx<'_>) {
    ctx.vehicle.control_flags = ControlFlags(ctx.vehicle.control_flags).locked().0;
    ctx.vehicle.control_entity = ctx.tick;
    if let Some(events) = &mut ctx.events {
        events.push(Event::Locked { tick: ctx.tick, vehicle: ctx.vehicle.key });
    }
}

fn substep(ctx: &mut StepCtx<'_>, now: i64, dt: f32, remaining: f32) -> CollisionFrame {
    let handle = BodyHandle(ctx.vehicle.body);
    if super::ghost::response(
        ctx.assets.geometry.contact_box,
        &mut ctx.dyna.body_mut(handle).state,
        ctx.ghost_contacts,
        remaining,
    ) {
        ctx.vehicle.flags |= super::race_flags::RaceFlags::GHOST_RESPONSE.0;
    }
    ctx.dyna.kinematic_update(
        now,
        &mut DynaCollision { world: ctx.world, state: ctx.state, manager: ctx.manager },
    );
    ctx.dyna.clear_acc(handle);
    let scale = ctx.dyna.body(handle).record.coefficient * ctx.assets.geometry.mass;
    ctx.dyna.add_force(handle, scale * ctx.dyna.gravity);
    if ctx.dyna.body(handle).forces == crate::dyna::ForceModels::Vehicle {
        with_forces(ctx, now, dt, |force, _| forces::compute_forces(force));
    }
    integrate::integrate_body(ctx.vehicle, ctx.assets.geometry, ctx.dyna, dt);
    let frame = collision_step(ctx, now, dt, COLLISION_CLIP);
    let forward = ctx.dyna.local_velocity(handle).z;
    let mut events = Vec::new();
    effects::surface_effects(
        ctx.vehicle,
        ctx.assets.tuning,
        forward,
        ctx.tick,
        dt,
        ctx.events.as_ref().map(|_| &mut events),
    );
    if let Some(output) = &mut ctx.events {
        output.extend(events.into_iter().map(Event::Surface));
    }
    frame
}

pub fn collision_step(ctx: &mut StepCtx<'_>, now: i64, dt: f32, clip: f32) -> CollisionFrame {
    let handle = BodyHandle(ctx.vehicle.body);
    assert!(!ctx.dyna.body(handle).record.has_pre_collision_callback, "body pre-collision callback");
    let run = ControlFlags(ctx.vehicle.control_flags).phase() != ControlPhase::Countdown
        || (ctx.tick.wrapping_sub(ctx.vehicle.race_start) as i32) >= -50;
    let mut list = ContactList::new(contacts::CONTACT_CAPACITY);
    let mut output = contacts::Output::default();
    if run && let Some(vehicle_item) = ctx.vehicle.item {
        let pose = ctx.dyna.state(handle).pose();
        let shape = ctx.state.dynamic_item[vehicle_item.index()].shape;
        let scale = ctx.assets.tuning.collision_scale;
        if scale != 1.0 {
            crate::shape::scale::shape(ctx.world, ctx.state, shape, scale);
        }
        DynaCollision { world: ctx.world, state: ctx.state, manager: ctx.manager }
            .set_placement(vehicle_item, pose);
        let item = &ctx.state.dynamic_item[vehicle_item.index()];
        for (lists, layers) in [
            (Lists::SURFACES, crate::collision::layers::MASKS[item.layer as usize]),
            (Lists::DISCRETE, 1048576),
        ] {
            let filter = QueryFilter {
                lists,
                layers,
                excluded_flags: item.flags,
                ignored: [Some(vehicle_item.into()), None],
                ..Default::default()
            };
            ctx.state.collision_world[ctx.manager].append_contacts_for_item(
                Geometry::new(ctx.world, ctx.state),
                vehicle_item,
                filter,
                &mut list,
            );
        }
        if scale != 1.0 {
            let inverse = 1.0 / scale;
            crate::shape::scale::shape(ctx.world, ctx.state, shape, inverse);
            DynaCollision { world: ctx.world, state: ctx.state, manager: ctx.manager }
                .set_placement(vehicle_item, pose);
            for contact in list.as_mut_slice() {
                contact.rescale(pose, inverse);
            }
        }
        output =
            with_forces(ctx, now, dt, |force, model| contacts::vehicle_contacts(force, model, &mut list));
        let push = crate::dyna::solver::clip_sum(clip, &output.pushes);
        ctx.dyna.add_position(handle, push);
    }
    ctx.vehicle.end_collision_step(ctx.assets.tuning);
    CollisionFrame { time_ns: now, dt, contacts: list, output }
}

fn with_forces<T>(
    ctx: &mut StepCtx<'_>,
    now: i64,
    dt: f32,
    call: impl FnOnce(&mut ForceCtx<'_>, &ImpactModel) -> T,
) -> T {
    let instance = ctx.vehicle.shape.and_then(ShapeId::compound).map(CompoundInstanceId::index);
    let mut transforms =
        instance.map_or_else(Vec::new, |index| ctx.state.compound_instance[index].transforms.clone());
    let rays = ForceRays {
        geometry: Geometry::new(ctx.world, ctx.state),
        manager: &ctx.state.collision_world[ctx.manager],
        vehicle_item: ctx.vehicle.item,
    };
    let mut events = Vec::new();
    let inverse_inertia = ctx.vehicle.world_inverse_inertia;
    let handle = BodyHandle(ctx.vehicle.body);
    let mut force = ForceCtx {
        vehicle: ctx.vehicle,
        tuning: ctx.assets.tuning,
        geometry: ctx.assets.geometry,
        body: VehicleBody { dyna: ctx.dyna, handle, mass: ctx.assets.geometry.mass, inverse_inertia },
        water: crate::water::WaterManager::new(&ctx.world.water),
        fluid_tuning: ctx.assets.fluid,
        collision: &rays,
        wheel_transforms: &mut transforms,
        events: ctx.events.as_ref().map(|_| &mut events),
        tick: ctx.tick,
        time_ns: now,
        dt,
        local_velocity: Vec3::ZERO,
        local_angular_velocity: Vec3::ZERO,
        local_force: Vec3::ZERO,
    };
    let result = call(&mut force, ctx.assets.impacts);
    if let Some(index) = instance {
        ctx.state.compound_instance[index].transforms.copy_from_slice(&transforms);
    }
    if let Some(output) = &mut ctx.events {
        output.extend(events.into_iter().map(Event::Force));
    }
    result
}

pub struct Countdown<'a> {
    pub world: &'a World,
    pub state: &'a mut State,
    pub dyna: &'a mut Dyna,
    pub assets: &'a super::spawn::Assets,
    pub visuals: &'a mut super::vis::Refresh<'a>,
    pub effects: &'a mut super::spawn::EffectRing,
    pub initial_pose: crate::Iso,
    pub parent_pose: Option<crate::Iso>,
    pub waypoints: &'a [crate::race::RaceWaypoint],
}
impl Countdown<'_> {
    pub fn respawn_with(&mut self, vehicle: &mut Vehicle, pose: crate::Iso, reset: super::spawn::Reset) {
        let mut services = super::spawn::Services {
            world: self.world,
            state: self.state,
            visuals: self.visuals,
            ring: self.effects,
            events: None,
        };
        super::spawn::respawn(vehicle, self.assets, self.dyna, pose, reset, &mut services);
    }
}

impl crate::race::countdown::CountdownRespawn for Countdown<'_> {
    fn respawn(&mut self, vehicle: &mut Vehicle, landmark: u32) {
        let pose = super::spawn::arena_pose(self.initial_pose, self.parent_pose, self.waypoints, landmark);
        self.respawn_with(vehicle, pose, super::spawn::Reset::FULL);
    }
}
