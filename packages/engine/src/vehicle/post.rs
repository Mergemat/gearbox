use super::{
    Tuning, Vehicle, VisualWheel,
    flags::{BodyFlags, EffectFlags, ToggleFlags, WheelFlags},
    forces::common::{KMH, PedalMode, TINY, clamp01},
    impacts::{Accumulator, Category},
    model::ForceGeometry,
    vis,
};
use crate::race::countdown::{ControlFlags, ControlPhase};
use crate::{
    Curve, Iso, Quat, Vec3,
    dyna::{BodyHandle, Dyna},
    math::{maxss, minss},
};

const STRONG_IMPACT: f32 = 0.2;
const HARD_BODY_IMPACT: f32 = 0.85;
const LOUD_IMPACT: f32 = 0.95;
const WATER_RADIUS_FRACTION: f32 = 0.01;
const LOW_GRIP_SLIP: f32 = 0.085;
const BUMP_AMPLITUDE_LIMIT: f32 = 0.15;
const BUMP_FOLLOW_TOLERANCE: f32 = 0.05;
const FREE_WHEEL_SHAKE: f32 = 0.45;
const WHEEL_SHAKE: f32 = 0.65;
const SHAKE_SLOPE: f32 = 0.33999997;
const SHAKE_BASE: f32 = 0.66;
const DAMAGE_THRESHOLD: f32 = 0.8;
const SPIN_LIMIT: f32 = 18.849556;
const IDLE_MOVEMENT_SQUARED: f32 = 0.0009;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Model {
    pub wheel_impact_range: [f32; 2],
    pub body_impact_range: [f32; 2],
    pub impact_delays: [u32; 4],
    pub spring_stiffness: f32,
    pub spring_damping: f32,
    pub spring_drive_limit: f32,
    pub spring_drive_scale: f32,
    pub spring_value_limit: f32,
    pub layer_build_by_speed: Curve,
    pub layer_clear_by_speed: Curve,
    pub layer_materials: Vec<bool>,
    pub damage_threshold: f32,
    pub detach_delay: [u32; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub struct ImpactEvent {
    pub category: Category,
    pub tick: u32,
    pub tag: u8,
    pub vehicle: u32,
    pub point: Vec3,
    pub normal: Vec3,
    pub material: u8,
    pub strength: f32,
    pub free: bool,
    pub flags: u32,
}
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    Impact(ImpactEvent),
    Exit(crate::arena::StateExitEvent),
}

pub struct Context<'a> {
    pub tuning: &'a Tuning,
    pub geometry: &'a ForceGeometry,
    pub model: Option<&'a Model>,
    pub dyna: &'a Dyna,
    pub water: crate::water::WaterManager<'a>,
    pub tick: u32,
    pub dt: f32,
    pub rng: &'a mut u32,
    pub parts_count: u32,
    pub events: Option<&'a mut Vec<Event>>,
}

pub fn ramp(value: f32, middle: f32, high: f32) -> f32 {
    let phase = if value <= middle {
        ((value - middle * STRONG_IMPACT) * 0.5) / (middle - middle * STRONG_IMPACT)
    } else {
        ((value - middle) * 0.5) / (high - middle) + 0.5
    };
    let mut result = 0.0;
    if 0.0 < phase {
        result = 1.0;
        if phase < 1.0 {
            result = phase * 3.0 * phase - (phase + phase) * phase * phase;
        }
    }
    result
}

pub fn follow(value: f32, target: f32, tolerance: f32) -> f32 {
    if target + tolerance < value {
        value - tolerance
    } else if value < target - tolerance {
        value + tolerance
    } else {
        target
    }
}

pub fn spring_step(value: &mut f32, target: f32, velocity: &mut f32, stiffness: f32, damping: f32, dt: f32) {
    let acceleration = (target - *value) * stiffness - *velocity * damping;
    *velocity += acceleration * dt;
    *value += *velocity * dt;
}

pub fn idle_update(vehicle: &mut Vehicle, tick: u32) {
    vehicle.suspension_requires_all_contacts = 0;
    let mut last = tick;
    if vehicle.controls.gas != 0.0 || vehicle.controls.brake != 0.0 || vehicle.controls.steer != 0.0 {
        last = tick;
    }
    if tick == tick / 1000 * 1000 {
        vehicle.idle_position_x = vehicle.pos.x;
        vehicle.idle_position_y = vehicle.pos.y;
        vehicle.idle_position_z = vehicle.pos.z;
    }
    let half = tick.wrapping_add(500);
    if half == half / 1000 * 1000 {
        let previous = Vec3::new(vehicle.idle_position_x, vehicle.idle_position_y, vehicle.idle_position_z);
        let delta = vehicle.pos - previous;
        if IDLE_MOVEMENT_SQUARED < (delta.y * delta.y + delta.x * delta.x + delta.z * delta.z) * 4.0 {
            last = tick;
        }
    }
    vehicle.suspension_requires_all_contacts = (1000 < last.wrapping_sub(tick)) as u32;
}

pub fn vehicle_post_step(vehicle: &mut Vehicle, ctx: &mut Context<'_>) {
    if vehicle.body == u32::MAX {
        return;
    }
    idle_update(vehicle, ctx.tick);
    if matches!(ctx.tuning.model_kind, 9 | 15) {
        todo!("post step of model 9/15");
    }
    let target = if vehicle.effect_mode_is(PedalMode::Cut) {
        0.0
    } else if vehicle.effect_mode_is(PedalMode::Full) {
        1.0
    } else {
        let pedal = if vehicle.cached_gear == 0 { vehicle.controls.brake } else { vehicle.controls.gas };
        let enabled = if vehicle.shift_remaining > 0.0 { 0.0 } else { 1.0 };
        enabled * pedal
    };
    vehicle.smoothed_pedal =
        clamp01(((target - vehicle.smoothed_pedal) * 5.0) * ctx.dt + vehicle.smoothed_pedal);
    let Some(model) = ctx.model else {
        return;
    };
    let state = ctx.dyna.state(BodyHandle(vehicle.body));
    let rotation = state.rot.to_array();
    let forward = (state.vel.y * rotation[5] + rotation[2] * state.vel.x) + state.vel.z * rotation[8];
    let strengths = impact_events(vehicle, model, ctx);
    suspension(vehicle, model, state, ctx.dt);
    let wet = wet_wheels(ctx, state.pose());
    surface_summary(vehicle, ctx, forward, &wet);
    shake(vehicle, strengths);
    layers(vehicle, ctx, forward, &wet);
    wear_states(vehicle, ctx);
    super::maintenance::parts_update(ctx.parts_count as usize);
    if (strengths[1] >= LOUD_IMPACT && !crate::surface::SOFT.contains(&vehicle.free_wheel_impact_material))
        || (strengths[0] >= LOUD_IMPACT && !crate::surface::SOFT.contains(&vehicle.wheel_impact_material))
    {
        vehicle.recent_events[0] = ctx.tick;
    }
}

fn impact_events(vehicle: &mut Vehicle, model: &Model, ctx: &mut Context<'_>) -> [f32; 3] {
    let range = model.wheel_impact_range;
    let strengths = [
        ramp(vehicle.wheel_impact_peak * KMH, range[0], range[1]),
        ramp(vehicle.free_wheel_impact_peak * KMH, range[0], range[1]),
        ramp(vehicle.body_impact_peak * KMH, model.body_impact_range[0], model.body_impact_range[1]),
    ];
    let [body_delay, wheel_delay, free_delay, separation] = model.impact_delays;
    let wheel_last = vehicle.wheel_impact_last_event;
    let free_last = vehicle.free_wheel_impact_last_event;
    let conditions = [
        ctx.tick > wheel_delay.wrapping_add(wheel_last) && ctx.tick >= free_last.wrapping_add(separation),
        ctx.tick > free_last.wrapping_add(free_delay) && ctx.tick >= wheel_last.wrapping_add(separation),
        ctx.tick > vehicle.body_impact_last_event.wrapping_add(body_delay),
    ];
    for (index, category) in [Category::Wheel, Category::FreeWheel, Category::Body].into_iter().enumerate() {
        let mut value = vehicle.impact(category);
        if strengths[index] >= STRONG_IMPACT && conditions[index] {
            average(&mut value);
            emit_impact(vehicle, ctx, category, value, strengths[index]);
            match category {
                Category::Wheel => vehicle.wheel_impact_last_event = ctx.tick,
                Category::FreeWheel => vehicle.free_wheel_impact_last_event = ctx.tick,
                Category::Body => vehicle.body_impact_last_event = ctx.tick,
            }
        }
        value.count = 0;
        value.point = Vec3::ZERO;
        value.normal = Vec3::ZERO;
        value.gripping = 0;
        value.peak = 0.0;
        vehicle.set_impact(category, value);
    }
    vehicle.impact_flags = 0;
    vehicle.hard_impact = (strengths[2] >= HARD_BODY_IMPACT && strengths[1] + strengths[0] >= 1.5) as u32;
    strengths
}

fn average(value: &mut Accumulator) {
    let normal = value.normal;
    let squared = (normal.x * normal.x + normal.y * normal.y) + normal.z * normal.z;
    let normal = if squared > TINY && f32::MAX > squared {
        (1.0 / squared.sqrt()) * normal
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    value.normal = -normal;
    let inverse = 1.0 / value.count as f32;
    value.point = inverse * value.point;
}

fn emit_impact(
    vehicle: &Vehicle,
    ctx: &mut Context<'_>,
    category: Category,
    value: Accumulator,
    strength: f32,
) {
    if super::race_flags::RaceFlags(vehicle.flags).contains(super::race_flags::RaceFlags::MUTED)
        || vehicle.tag == u8::MAX
    {
        return;
    }
    let material = if category == Category::Body {
        vehicle.body_impact_material
    } else {
        vehicle.free_wheel_impact_material
    };
    let event = ImpactEvent {
        category,
        tick: ctx.tick,
        vehicle: vehicle.key,
        tag: vehicle.tag,
        point: vehicle.quat.rotate(value.point) + vehicle.pos,
        normal: value.normal,
        material,
        strength,
        free: category != Category::Body && value.gripping == 0,
        flags: vehicle.impact_flags,
    };
    if let Some(events) = &mut ctx.events {
        events.push(Event::Impact(event));
    }
}

fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    if value >= -limit { minss(value, limit) } else { -limit }
}

fn suspension(vehicle: &mut Vehicle, model: &Model, state: &crate::dyna::BodyState, dt: f32) {
    let delta = state.vel - vehicle.previous_body_velocity;
    let rotation = vehicle.suspension_frame.to_array();
    let lateral = (delta.y * rotation[3] + delta.x * rotation[0]) + delta.z * rotation[6];
    let forward = (delta.y * rotation[5] + delta.x * rotation[2]) + delta.z * rotation[8];
    vehicle.previous_body_velocity = state.vel;
    vehicle.suspension_frame = state.rot;
    spring_filter(
        &mut vehicle.lateral_spring_value,
        vehicle.lateral_spring_target,
        &mut vehicle.lateral_spring_velocity,
        lateral,
        model,
        dt,
    );
    spring_filter(
        &mut vehicle.forward_spring_value,
        vehicle.forward_spring_target,
        &mut vehicle.forward_spring_velocity,
        forward,
        model,
        dt,
    );
}

fn spring_filter(value: &mut f32, target: f32, velocity: &mut f32, drive: f32, model: &Model, dt: f32) {
    spring_step(value, target, velocity, model.spring_stiffness, model.spring_damping, dt);
    let input = clamp_symmetric(-drive, model.spring_drive_limit);
    let input = input / model.spring_drive_limit * model.spring_drive_scale;
    *value = clamp_symmetric(*value, model.spring_value_limit);
    *velocity = clamp_symmetric(input + *velocity, model.spring_drive_scale);
}

fn wet_wheels(ctx: &Context<'_>, pose: Iso) -> Vec<bool> {
    ctx.geometry
        .wheels
        .iter()
        .map(|wheel| {
            let point = pose.rot.transform_y_first(wheel.anchor) + pose.pos;
            let bottom = point.y - wheel.radius;
            let height = (wheel.radius + point.y) - bottom;
            let mut depth = 0.0;
            ctx.water.query(Vec3::new(point.x, bottom, point.z), height, Some(&mut depth), None)
                && depth > wheel.radius * WATER_RADIUS_FRACTION + bottom
        })
        .collect()
}

fn surface_summary(vehicle: &mut Vehicle, ctx: &Context<'_>, forward: f32, wet: &[bool]) {
    let tuning = ctx.tuning;
    let records: Vec<_> = ctx
        .geometry
        .wheels
        .iter()
        .enumerate()
        .map(|(index, _)| {
            let wheel = &vehicle.wheels[index];
            let coefficient = vehicle.wheel_grip[index];
            let mut flags = WheelFlags::default();
            flags.set(WheelFlags::WET, wet[index]);
            flags.set(WheelFlags::CONTACT, wheel.contact != 0 && vehicle.level_consistent());
            flags.set(WheelFlags::BOOST, ToggleFlags(wheel.boost_contact).enabled());
            let slip = if tuning.low_grip_threshold >= coefficient { LOW_GRIP_SLIP } else { wheel.slip };
            VisualWheel {
                flags: flags.0,
                slip,
                material: wheel.material as u16 | (wheel.surface as u16) << 8,
                ..VisualWheel::default()
            }
        })
        .collect();
    let fixed = if vehicle.fixed_wheel_spin != 0 {
        true
    } else {
        vehicle.effect_mode_is_free()
            && ControlFlags(vehicle.control_flags).phase() == ControlPhase::Countdown
            && vehicle.in_water == 0
            && forward * KMH > tuning.fixed_spin_minimum_speed
            && tuning.fixed_spin_maximum_speed > forward * KMH
            && vehicle.controls.gas > 0.0
    };
    let mode = fixed && !EffectFlags::FRAGILE.contains_in(vehicle.effect_flags);
    let sides = contact_summary(&ctx.geometry.wheels, &records, false, mode);
    let material = if sides[1].count > sides[0].count { sides[1].material } else { sides[0].material };
    if material == 80 {
        return;
    }
    let material = ctx.geometry.material(material);
    let speed = forward.abs();
    let amplitude = if vehicle.angular_freeze == 0 {
        0.0
    } else {
        minss(speed * material.bump_scale, material.bump_limit)
    };
    let amplitude = amplitude * material.bump_amplitude_scale;
    vehicle.bump_frequency = follow(vehicle.bump_frequency, speed / material.bump_speed_period, 0.5);
    vehicle.bump_amplitude =
        follow(vehicle.bump_amplitude, minss(amplitude, BUMP_AMPLITUDE_LIMIT), BUMP_FOLLOW_TOLERANCE);
}

fn shake(vehicle: &mut Vehicle, strengths: [f32; 3]) {
    let wheel = strengths[1] * FREE_WHEEL_SHAKE + strengths[0] * WHEEL_SHAKE;
    let strength = maxss(strengths[2], wheel);
    if !(strength >= 0.5) {
        return;
    }
    let excess = strength - 0.5;
    let frequency = 4.0 - ((excess + excess) + (excess + excess));
    let amplitude = (excess * SHAKE_SLOPE + excess * SHAKE_SLOPE) + SHAKE_BASE;
    let frequency = if !(frequency >= 4.0) {
        4.0
    } else if 2.0 >= frequency {
        frequency
    } else {
        2.0
    };
    let amplitude = if !(amplitude >= SHAKE_BASE) {
        SHAKE_BASE
    } else if 1.0 >= amplitude {
        amplitude
    } else {
        1.0
    };
    vehicle.bump_frequency = maxss(vehicle.bump_frequency, frequency);
    vehicle.bump_amplitude = maxss(vehicle.bump_amplitude, amplitude);
}

fn layers(vehicle: &mut Vehicle, ctx: &Context<'_>, forward: f32, wet: &[bool]) {
    let model = ctx.model.unwrap();
    let speed = (forward * KMH).abs();
    let build = model.layer_build_by_speed.eval(speed);
    let clear = model.layer_clear_by_speed.eval(speed);
    let mut layers = vehicle.wheel_layers;
    let mut builds = 0;
    for (index, &wet) in wet.iter().enumerate() {
        let wheel = &vehicle.wheels[index];
        let material = if wet {
            13
        } else {
            if wheel.contact == 0 || wheel.material == 255 {
                continue;
            }
            wheel.material
        };
        if model.layer_materials[material as usize] {
            builds += 1;
            layers[index] = clamp01(ctx.dt * build + layers[index]);
        } else {
            layers[index] = clamp01(layers[index] - ctx.dt * clear);
        }
    }
    vehicle.wheel_layers = layers;
    if builds != 0 {
        vehicle.surface_layer = clamp01(ctx.dt * build + vehicle.surface_layer);
    } else if vehicle.in_water != 0 {
        vehicle.surface_layer = clamp01(vehicle.surface_layer - ctx.dt * clear);
    }
}

pub fn grid_reduce(wear: [f32; 9], all: bool) -> [f32; 4] {
    if all {
        let mut maximum = 0.0;
        for value in wear {
            maximum = maxss(maximum, value);
        }
        return [maximum; 4];
    }
    [
        maxss((wear[6] + wear[4]) * 0.5, wear[5]),
        maxss((wear[0] + wear[2]) * 0.5, wear[1]),
        maxss(wear[2], maxss(wear[3], wear[4])),
        maxss(wear[0], maxss(wear[7], wear[6])),
    ]
}

pub fn wheel_flags(wear: [f32; 9], parts: [f32; 4]) -> [u32; 5] {
    let all = DAMAGE_THRESHOLD < wear[8];
    let last = if wear[7] <= DAMAGE_THRESHOLD && !all { 0 } else { 2 };
    [
        if DAMAGE_THRESHOLD < parts[0] || all { 2 } else { 0 },
        if DAMAGE_THRESHOLD < parts[1] || all { 2 } else { 0 },
        if DAMAGE_THRESHOLD < wear[3] || all { 2 } else { 0 },
        last,
        last,
    ]
}

fn wear_states(vehicle: &mut Vehicle, ctx: &mut Context<'_>) {
    let wear = vehicle.wear;
    let parts = grid_reduce(wear, false);
    let flags = wheel_flags(wear, parts);
    let model = ctx.model.unwrap();
    let mut detached = vehicle.detached_parts;
    let mut delays = vehicle.detach_delays;
    for index in 0..4 {
        if detached[index] == 0 && !(model.damage_threshold > parts[index]) {
            detached[index] = 1;
            delays[index] =
                crate::numeric::random_range(ctx.rng, model.detach_delay[0], model.detach_delay[1]);
        }
    }
    vehicle.detached_parts = detached;
    vehicle.detach_delays = delays;
    let previous = vehicle.damage_flags;
    let layer_state = |value| crate::arena::LayerState::from_code(value);
    let events = crate::arena::state_exit_events(
        vehicle,
        ctx.tick,
        ctx.events.is_some(),
        flags.map(layer_state),
        previous.map(layer_state),
    );
    if let Some(output) = &mut ctx.events {
        output.extend(events.into_iter().map(Event::Exit));
    }
    vehicle.damage_flags = flags;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SideSummary {
    pub sum: f32,
    pub count: u32,
    pub material: u8,
    pub weighted: f32,
}
impl Default for SideSummary {
    fn default() -> Self {
        Self { sum: 0.0, count: 0, material: 80, weighted: 0.0 }
    }
}

pub fn contact_summary(
    wheels: &[super::model::WheelDefinition],
    records: &[VisualWheel],
    select_x: bool,
    mode: bool,
) -> [SideSummary; 2] {
    let mut sides = [SideSummary::default(); 2];
    if records.is_empty() || wheels.is_empty() {
        return sides;
    }
    let mut counts = [0; 2];
    let mut last = [0; 2];
    for (wheel, record) in wheels.iter().zip(records) {
        let selected = if select_x { wheel.anchor.x } else { wheel.anchor.z };
        let side = usize::from(!(selected > 0.0));
        counts[side] += 1;
        let summary = &mut sides[side];
        if WheelFlags(record.flags).contains(WheelFlags::CONTACT) {
            summary.count += 1;
            if summary.material != 13 {
                summary.material = record.material as u8;
            }
            summary.sum += record.slip;
            summary.weighted += wheel.radius * record.spin;
        } else {
            last[side] = record.material as u8;
        }
        if WheelFlags(record.flags).contains(WheelFlags::WET) {
            summary.material = 13;
        }
    }
    if mode {
        if sides[1].count != 0 {
            sides[0].sum = 2.0;
        }
        if sides[0].count != 0 {
            sides[1].sum = 2.0;
        }
    } else {
        for side in 0..2 {
            if counts[1 - side] != 0 {
                sides[side].sum *= 1.0 / counts[1 - side] as f32;
            }
        }
    }
    for side in 0..2 {
        if counts[1 - side] != 0 {
            sides[side].weighted *= 1.0 / counts[1 - side] as f32;
        }
        if sides[side].material >= 80 {
            sides[side].material = last[side];
        }
    }
    sides
}

pub fn post(vehicle: &mut Vehicle, ctx: &mut Context<'_>, recent_event_duration: u32, fast: bool) {
    if ControlFlags(vehicle.control_flags).phase() != ControlPhase::Forced
        && matches!(vehicle.zone_mode, 0 | 3)
    {
        vehicle.previous_body_velocity = vehicle.vel;
        let previous = Iso::from_quat_pos(vehicle.quat, vehicle.pos);
        vehicle.suspension_frame = previous.rot;
        vehicle.slow_start_forward_threshold =
            Quat::new(vehicle.quat.w, -vehicle.quat.x, -vehicle.quat.y, -vehicle.quat.z)
                .rotate(vehicle.vel)
                .z;
        let state = ctx.dyna.state(BodyHandle(vehicle.body));
        vehicle.quat = state.quat;
        vehicle.pos = state.pos;
        vehicle.vel = state.vel;
        vehicle.ang_vel = state.ang_vel;
        if matches!(ctx.tuning.model_kind, 9 | 14 | 15) {
            todo!("wheel spin of model 9/14/15");
        }
        let front = 0.0 + vehicle.front_spin;
        let rear = 0.0 + vehicle.rear_spin;
        if !(-SPIN_LIMIT..SPIN_LIMIT).contains(&front) || !(-SPIN_LIMIT..SPIN_LIMIT).contains(&rear) {
            todo!("wheel spin angle wrap");
        }
        vehicle.front_spin = front;
        vehicle.rear_spin = rear;
        vehicle_post_step(vehicle, ctx);
        if vehicle.gear_effect_active != 0 {
            todo!("input override copy");
        }
        if vehicle.replica != 0 && BodyFlags(vehicle.body_enabled).enabled() {
            todo!("vehicle body hook");
        }
    }
    if fast {
        vis::vehicle_vis_fast(vehicle);
    } else {
        vis::vehicle_vis(
            vehicle,
            ctx.dyna,
            &vis::Config {
                tuning: ctx.tuning,
                geometry: ctx.geometry,
                tick: ctx.tick,
                recent_event_duration,
            },
            1,
        );
    }
    if vehicle.zone_mode == 3 && vehicle.zone_bounds != 0 {
        todo!("vehicle zone 3 bounds check");
    }
}
