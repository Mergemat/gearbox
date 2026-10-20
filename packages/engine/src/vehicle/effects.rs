use super::{
    Tuning, Vehicle,
    flags::EffectFlags,
    forces::common::{EPS, KMH, PedalMode},
};
use crate::race::countdown::{ControlFlags, ControlPhase};
use crate::{
    ItemId, Vec3,
    math::{cvtt, cvtt64, minss},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum EventKind {
    Turbo = 2,
    EngineCut = 6,
    Restored = 7,
    Fragile = 11,
    FragileEnded = 12,
    SlowMotion = 13,
    SlowMotionEnded = 14,
    Boost = 15,
    BoostEnded = 16,
    BoostTwo = 18,
    Reactor = 19,
    NoGrip = 21,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub kind: EventKind,
    pub tick: u32,
    pub vehicle: u32,
    pub tag: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceContact {
    pub normal: Vec3,
    pub item: Option<ItemId>,
    pub wheel: bool,
}
impl Default for SurfaceContact {
    fn default() -> Self {
        Self { normal: Vec3::new(0.0, 0.0, 1.0), item: None, wheel: false }
    }
}

impl Vehicle {
    pub fn find_surface_contact(&self, effect: u8, materials: &[u8], out: &mut SurfaceContact) -> bool {
        if effect == self.surface_contact_effect {
            let inverse = crate::Quat::new(self.quat.w, -self.quat.x, -self.quat.y, -self.quat.z);
            *out = SurfaceContact {
                normal: inverse.rotate(self.surface_contact_normal),
                item: self.surface_contact_item,
                wheel: false,
            };
            return true;
        }
        for wheel in &self.wheels {
            if wheel.contact != 0 && effect == wheel.surface {
                *out = SurfaceContact { normal: wheel.surface_normal, item: wheel.surface_item, wheel: true };
                return true;
            }
        }
        for wheel in &self.wheels {
            if wheel.on_material(materials) {
                *out = SurfaceContact { normal: wheel.surface_normal, item: wheel.surface_item, wheel: true };
                return true;
            }
        }
        false
    }

    pub fn effect_active(&self, tuning: &Tuning, tick: u32) -> bool {
        self.reactor_begin != u32::MAX && tick < self.reactor_begin.wrapping_add(tuning.effect_duration_ms)
    }

    pub fn reset_reactor(&mut self) {
        self.wheel_meter_enabled = 0;
        self.wheel_meter = 0.0;
        self.wheel_meter_sum = 0.0;
        self.wheel_meter_count = 0;
        for wheel in &mut self.wheels {
            wheel.drive_coefficient = 1.0;
            wheel.sample_sum = 0.0;
            wheel.sample_time = 0;
        }
    }

    pub fn boost_reset(&mut self) {
        self.boost_level = 0;
        self.boost_maximum_level = 0;
        self.ground_pull_begin = 0;
        for wheel in &mut self.wheels {
            wheel.boost_contact = 0;
        }
    }

    pub fn boost_update(&mut self, tuning: &Tuning, tick: u32, dt: f32) -> (bool, bool) {
        let level = self.level();
        let maximum = self.maximum_level();
        let mut contact = SurfaceContact::default();
        let hits = [12, 18, 14, 19].map(|effect| self.find_surface_contact(effect, &[], &mut contact));
        let any = hits.iter().any(|&hit| hit);
        let direction = if contact.wheel || (!hits[0] && !hits[2]) { contact.normal.z } else { 1.0 };
        let power = self.level_power(tuning);
        if !self.effect_active(tuning, tick) && any {
            let old = self.boost_level;
            self.boost_level = !(direction > 0.0) as u32 + 1;
            self.boost_last_contact = tick;
            self.ground_pull_duration_ms = cvtt64(tuning.turbo_boost_duration_ms as f32 / power) as u32;
            if old == 0 {
                self.boost_begin = tick;
            }
            self.boost_contact = contact.item;
            self.boost_maximum_level = if hits[0] || hits[1] { 1 } else { 2 };
        } else if !any && self.boost_level != 0 && power != self.previous_level_power {
            let duration = self.ground_pull_duration_ms;
            let left = self.boost_last_contact.wrapping_sub(tick).wrapping_add(duration);
            let scaled = cvtt64(self.previous_level_power / power * left as f32) as u32;
            self.ground_pull_duration_ms = scaled.wrapping_sub(left).wrapping_add(duration);
        }
        let end = self.ground_pull_duration_ms.wrapping_add(self.boost_last_contact);
        let ending = !(tick.wrapping_add(400) < end)
            && tick.wrapping_sub(cvtt64(dt * 1000.0) as u32).wrapping_add(400) < end;
        if tick > end && self.boost_level != 0 {
            self.boost_reset();
        }
        let changed = if self.level() == 0 {
            self.boost_contact = None;
            false
        } else {
            level != self.boost_level || maximum != self.boost_maximum_level
        };
        (changed, ending)
    }
}

pub fn timed_boost_phase(race: u32, period: u32) -> f32 {
    let phase = (race % period) as f32 / period as f32;
    if TIMED_BOOST_LOW > phase {
        0.0
    } else if TIMED_BOOST_HIGH > phase {
        0.5
    } else {
        1.0
    }
}

pub fn timed_boost_multiplier(phase: f32) -> f32 {
    phase + 1.0
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
enum PadKind {
    Turbo = 1,
    RedTurbo = 2,
    TimedTurbo = 3,
    Reactor = 4,
    Input = 5,
}

#[derive(Default)]
struct Changes {
    slow_started: bool,
    slow_ended: bool,
    no_grip: bool,
    boost_reset: bool,
    reactor: bool,
    effect: u8,
}

pub fn surface_effects(
    vehicle: &mut Vehicle,
    tuning: &Tuning,
    forward_speed: f32,
    tick: u32,
    dt: f32,
    events: Option<&mut Vec<Event>>,
) {
    let mut events = events;
    let speed = forward_speed * KMH;
    let power = vehicle.level_power(tuning);
    vehicle.previous_level_power = power;
    let mut changes = Changes::default();
    let start = turbo_pads(vehicle, tuning, tick, power, &mut changes, &mut events);
    let mut bits = vehicle.effect_flags;
    reactor(vehicle, tick, power, &mut bits, &mut changes);
    for effect in 20..=23 {
        if vehicle.find_surface_contact(effect, &[], &mut SurfaceContact::default())
            && ControlFlags(vehicle.control_flags).phase() == ControlPhase::Racing
        {
            vehicle.pending_car_changes.push(effect - 20);
        }
    }
    if vehicle.level_power_scale == 1.0 {
        vehicle.slow_motion_gate = u32::MAX;
        vehicle.slow_motion_wheel_contact = None;
        vehicle.slow_motion_body_contact = None;
    }
    let mut contact = SurfaceContact::default();
    if tuning.direction_drag_enabled != 0 && vehicle.find_surface_contact(13, &[], &mut contact) {
        let active = super::flags::ForcedEffects(vehicle.forced_effect_flags).wheel_meter()
            || vehicle.wheel_meter_enabled != 0;
        vehicle.wheel_meter_enabled = 1;
        vehicle.last_effect_contact = contact.item;
        if !active {
            changes.effect = 13;
        }
        changes.no_grip = !active;
    }
    if !vehicle.effect_active(tuning, tick) {
        controls(vehicle, speed, &mut bits, &mut changes);
        slow_motion(vehicle, tuning, tick, &mut changes);
    }
    fragile_wheels(vehicle, tuning, tick, dt);
    rescale_turbo(vehicle, tuning, tick, start);
    let (boost_changed, boost_ended) =
        if tuning.turbo_boost_enabled != 0 { vehicle.boost_update(tuning, tick, dt) } else { (false, false) };
    emit_changes(vehicle, tick, bits, boost_changed, boost_ended, &mut changes, &mut events);
    if vehicle.surface_effect == 0 {
        vehicle.surface_effect = changes.effect;
    }
}

pub(super) fn post(vehicle: &Vehicle, tick: u32, kind: EventKind, events: &mut Option<&mut Vec<Event>>) {
    if !super::race_flags::RaceFlags(vehicle.flags).contains(super::race_flags::RaceFlags::MUTED)
        && vehicle.tag != u8::MAX
        && let Some(events) = events
    {
        events.push(Event { kind, tick, vehicle: vehicle.key, tag: vehicle.tag });
    }
}

fn turbo_pads(
    vehicle: &mut Vehicle,
    tuning: &Tuning,
    tick: u32,
    power: f32,
    changes: &mut Changes,
    events: &mut Option<&mut Vec<Event>>,
) -> u32 {
    let mut contact = SurfaceContact::default();
    let mut pad = None;
    for (effect, materials) in [
        (PadKind::Turbo, &TURBO_PADS[..]),
        (PadKind::RedTurbo, &RED_TURBO_PADS[..]),
        (PadKind::TimedTurbo, &TIMED_TURBO_PADS[..]),
    ] {
        if vehicle.find_surface_contact(effect as u8, materials, &mut contact) {
            pad = Some(effect);
            break;
        }
    }
    let item = if pad.is_some() { contact.item } else { None };
    if pad.is_none() {
        if vehicle.reactor_turbo_begin != u32::MAX
            && (vehicle.turbo_begin == u32::MAX || vehicle.turbo_begin < vehicle.reactor_turbo_begin)
        {
            pad = Some(PadKind::Reactor);
        } else if vehicle.controls.auxiliary_axis > EPS {
            pad = Some(PadKind::Input);
        }
    }
    if let Some(pad) = pad {
        let previous = vehicle.turbo_active;
        if previous != pad as u32 {
            vehicle.timed_turbo_contact = None;
        }
        if vehicle.turbo_contact != item || previous != pad as u32 {
            vehicle.turbo_contact = item;
            vehicle.turbo_begin = tick;
            vehicle.turbo_last_contact = tick;
            vehicle.turbo_end = u32::MAX;
            post(vehicle, tick, EventKind::Turbo, events);
        }
        let scale = if matches!(pad, PadKind::Reactor | PadKind::Input) { 1.0 } else { contact.normal.z };
        apply_pad(vehicle, tuning, tick, pad, item, scale, changes);
        let (duration, fade) = match pad {
            PadKind::Turbo | PadKind::TimedTurbo => (tuning.turbo_duration_ms, tuning.turbo_fade_ms),
            PadKind::RedTurbo => (tuning.red_turbo_duration_ms, tuning.red_turbo_fade_ms),
            PadKind::Reactor => (tuning.reactor_turbo_duration_ms, tuning.reactor_turbo_fade_ms),
            _ => (vehicle.turbo_duration_ms, vehicle.turbo_fade_ms),
        };
        let inverse = 1.0 / power;
        vehicle.turbo_duration_ms = cvtt64(duration as f32 * inverse) as u32;
        vehicle.turbo_fade_ms = cvtt64(fade as f32 * inverse) as u32;
        vehicle.turbo_active = pad as u32;
        vehicle.turbo_last_contact = tick;
    } else {
        vehicle.turbo_contact = None;
        if vehicle.turbo_begin != u32::MAX
            && vehicle.turbo_end == u32::MAX
            && vehicle.turbo_last_contact != u32::MAX
        {
            vehicle.turbo_end = vehicle.turbo_last_contact;
            vehicle.turbo_last_contact = u32::MAX;
        }
    }
    let start = if vehicle.turbo_end != u32::MAX { vehicle.turbo_end } else { tick };
    if vehicle.turbo_active != 0 && tick.wrapping_sub(start) > vehicle.turbo_duration_ms {
        vehicle.turbo_active = 0;
        vehicle.turbo_duration_ms = 0;
    }
    start
}

fn apply_pad(
    vehicle: &mut Vehicle,
    tuning: &Tuning,
    tick: u32,
    pad: PadKind,
    item: Option<ItemId>,
    scale: f32,
    changes: &mut Changes,
) {
    let previous = vehicle.turbo_active;
    match pad {
        PadKind::Turbo | PadKind::TimedTurbo => {
            if pad == PadKind::TimedTurbo {
                let phase = if vehicle.timed_turbo_contact == item {
                    vehicle.turbo_phase
                } else {
                    vehicle.timed_turbo_contact = item;
                    let phase = timed_boost_phase(
                        tick.wrapping_sub(vehicle.race_start),
                        tuning.timed_turbo_period_ms,
                    );
                    vehicle.turbo_phase = phase;
                    phase
                };
                vehicle.turbo_thrust_scale = timed_boost_multiplier(phase) * (scale * tuning.turbo_thrust);
            } else {
                vehicle.turbo_thrust_scale = scale * tuning.turbo_thrust;
            }
            if previous != pad as u32 {
                changes.effect = pad as u8;
            }
        }
        PadKind::RedTurbo => {
            vehicle.turbo_thrust_scale = scale * tuning.red_turbo_thrust;
            changes.effect = if previous == 2 { 0 } else { 2 };
        }
        PadKind::Reactor => {
            vehicle.turbo_thrust_scale = scale * tuning.reactor_turbo_thrust;
        }
        _ => {}
    }
}

fn reactor(vehicle: &mut Vehicle, tick: u32, power: f32, bits: &mut u8, changes: &mut Changes) {
    let previous = vehicle.reactor_ready;
    vehicle.reactor_ready = 0;
    let mut contact = SurfaceContact::default();
    if !vehicle.find_surface_contact(8, &[], &mut contact) && previous == 0 {
        return;
    }
    if contact.item != vehicle.last_effect_contact && previous == 0 {
        changes.effect = 8;
        vehicle.last_effect_contact = contact.item;
    }
    if power != 1.0 {
        changes.slow_ended = true;
    }
    vehicle.reactor_begin = tick;
    vehicle.level_power_scale = 1.0;
    vehicle.forced_speed = 0.0;
    *bits = 0;
    vehicle.control_flags &= !ControlFlags::CRUISE.0;
    if previous == 0 {
        vehicle.reset_reactor();
        if vehicle.boost_level != 0 {
            vehicle.boost_reset();
            vehicle.boost_contact = None;
            changes.boost_reset = true;
        }
    }
    changes.reactor = true;
}

fn controls(vehicle: &mut Vehicle, speed: f32, bits: &mut u8, changes: &mut Changes) {
    let definitions: [(u8, &[u8], bool); 5] = [
        (4, &FREE_WHEELING_PADS, vehicle.effect_mode_is(PedalMode::Cut)),
        (7, &NO_BRAKES_PADS, vehicle.effect_mode_is(PedalMode::Full)),
        (6, &NO_STEERING_PADS, EffectFlags::STEERING_DISABLED.contains_in(vehicle.effect_flags)),
        (5, &NO_GRIP_PADS, EffectFlags::FRAGILE.contains_in(vehicle.effect_flags)),
        (16, &NO_GRIP_PADS, EffectFlags::BRAKES_DISABLED.contains_in(vehicle.effect_flags)),
    ];
    for (effect, materials, active) in definitions {
        let mut contact = SurfaceContact::default();
        if !vehicle.find_surface_contact(effect, materials, &mut contact) {
            continue;
        }
        if vehicle.last_effect_contact != contact.item && !active {
            changes.effect = effect;
        }
        vehicle.last_effect_contact = contact.item;
        match effect {
            4 => *bits = *bits & !EffectFlags::MODE.0 | PedalMode::Cut as u8,
            7 => {
                let next = *bits | EffectFlags::BRAKES_DISABLED.0;
                let mode = if PedalMode::from_code(next) == PedalMode::Cut {
                    PedalMode::Cut
                } else {
                    PedalMode::Full
                };
                *bits = next & !EffectFlags::MODE.0 | mode as u8;
            }
            6 => EffectFlags::STEERING_DISABLED.set_in(bits, true),
            5 => EffectFlags::FRAGILE.set_in(bits, true),
            16 => EffectFlags::BRAKES_DISABLED.set_in(bits, true),
            _ => unreachable!(),
        }
    }
    let mut contact = SurfaceContact::default();
    if vehicle.find_surface_contact(17, &[], &mut contact) {
        if !ControlFlags(vehicle.control_flags).contains(ControlFlags::CRUISE) {
            if (vehicle.forced_speed * KMH - speed).abs() > 1.0 {
                vehicle.forced_speed = cvtt(crate::numeric::ceilf(speed)) as f32 / KMH;
            }
            if vehicle.last_effect_contact != contact.item {
                changes.effect = 17;
            }
        }
        vehicle.last_effect_contact = contact.item;
        vehicle.control_flags = vehicle.control_flags & !ControlFlags::CRUISE.0
            | if vehicle.forced_speed == 0.0 { 0 } else { ControlFlags::CRUISE.0 };
    } else {
        vehicle.control_flags &= !ControlFlags::CRUISE.0;
    }
}

fn slow_motion(vehicle: &mut Vehicle, tuning: &Tuning, tick: u32, changes: &mut Changes) {
    let mut contact = SurfaceContact::default();
    if vehicle.find_surface_contact(9, &[], &mut contact) {
        let mut new = contact.item
            != if contact.wheel {
                vehicle.slow_motion_wheel_contact
            } else {
                vehicle.slow_motion_body_contact
            };
        if !contact.wheel && new {
            new = vehicle.surface_contact_gate != vehicle.slow_motion_gate
                || vehicle.surface_contact_gate == u32::MAX;
        }
        if !new {
            return;
        }
        let previous = vehicle.level_power_scale;
        if previous == 1.0 && 1.0 > tuning.level_power_base {
            if changes.slow_ended {
                changes.slow_ended = false;
            } else {
                changes.slow_started = true;
            }
        }
        let next = vehicle.level_power_scale * tuning.level_power_base;
        let next =
            if next >= tuning.level_power_minimum { minss(next, 1.0) } else { tuning.level_power_minimum };
        vehicle.level_power_scale = next;
        if contact.wheel {
            vehicle.slow_motion_wheel_contact = contact.item;
        } else {
            vehicle.slow_motion_gate = vehicle.surface_contact_gate;
            vehicle.slow_motion_body_contact = contact.item;
        }
        vehicle.slow_motion_until = 0;
        if next != previous {
            changes.effect = 9;
        }
    } else if vehicle.slow_motion_until == 0 {
        vehicle.slow_motion_until = tuning.slow_motion_duration_ms.wrapping_add(tick);
    } else if vehicle.level_power_scale != 1.0 && tick > vehicle.slow_motion_until {
        vehicle.level_power_scale = 1.0;
        changes.slow_ended = true;
        vehicle.slow_motion_until = u32::MAX;
    }
}

fn fragile_wheels(vehicle: &mut Vehicle, tuning: &Tuning, tick: u32, dt: f32) {
    if tuning.special_material_enabled == 0 {
        return;
    }
    let active = super::flags::ForcedEffects(vehicle.forced_effect_flags).wheel_meter()
        || vehicle.wheel_meter_enabled != 0;
    let mut hits = 0;
    for wheel in &mut vehicle.wheels {
        if !wheel.on_material(&WHEEL_METER_GROUND) {
            continue;
        }
        vehicle.thrust_boost_until = u32::MAX;
        hits += 1;
        if active {
            let next = wheel.drive_coefficient - dt * tuning.fragility_rate;
            wheel.drive_coefficient =
                if next >= tuning.fragility_minimum { minss(next, 1.0) } else { tuning.fragility_minimum };
        }
    }
    if hits != 0 {
        vehicle.thrust_boost_elapsed = hits;
    } else if vehicle.thrust_boost_until == u32::MAX {
        vehicle.thrust_boost_until = tuning.fragility_delay_ms.wrapping_add(tick);
    }
}

fn rescale_turbo(vehicle: &mut Vehicle, tuning: &Tuning, tick: u32, start: u32) {
    let power = vehicle.level_power(tuning);
    if power == vehicle.previous_level_power || vehicle.turbo_duration_ms == 0 {
        return;
    }
    let elapsed = tick.wrapping_sub(start);
    let scale = vehicle.previous_level_power / power;
    let rescale = |duration: u32| {
        let left = duration.saturating_sub(elapsed);
        (cvtt64(left as f32 * scale) as u32).wrapping_add(elapsed)
    };
    vehicle.turbo_duration_ms = rescale(vehicle.turbo_duration_ms);
    vehicle.turbo_fade_ms = rescale(vehicle.turbo_fade_ms);
}

fn emit_changes(
    vehicle: &mut Vehicle,
    tick: u32,
    bits: u8,
    boost_changed: bool,
    boost_ended: bool,
    changes: &mut Changes,
    events: &mut Option<&mut Vec<Event>>,
) {
    let previous = vehicle.effect_flags;
    let boost_reset = changes.boost_reset || boost_ended;
    if bits == previous
        && !changes.slow_started
        && !changes.slow_ended
        && !boost_changed
        && !boost_reset
        && !changes.no_grip
    {
        return;
    }
    let started = EffectFlags::FRAGILE.contains_in(bits) && !EffectFlags::FRAGILE.contains_in(previous);
    let ended = !EffectFlags::FRAGILE.contains_in(bits) && EffectFlags::FRAGILE.contains_in(previous);
    let into_cut =
        PedalMode::from_code(bits) == PedalMode::Cut && PedalMode::from_code(previous) != PedalMode::Cut;
    let left_cut =
        PedalMode::from_code(bits) != PedalMode::Cut && PedalMode::from_code(previous) == PedalMode::Cut;
    vehicle.effect_flags = bits;
    let mut event = if ended { EventKind::FragileEnded } else { EventKind::Restored };
    if left_cut {
        event = EventKind::Restored;
    }
    if changes.slow_ended {
        event = EventKind::SlowMotionEnded;
    }
    if boost_reset {
        event = EventKind::BoostEnded;
    }
    if changes.reactor {
        event = EventKind::Reactor;
    }
    if started {
        event = EventKind::Fragile;
    }
    if into_cut {
        event = EventKind::EngineCut;
    }
    if changes.slow_started {
        event = EventKind::SlowMotion;
    }
    if changes.no_grip {
        event = EventKind::NoGrip;
    }
    if boost_changed {
        match vehicle.boost_maximum_level {
            1 => {
                event = EventKind::Boost;
                changes.effect = 18;
            }
            2 => {
                event = EventKind::BoostTwo;
                changes.effect = 19;
            }
            _ => {}
        }
    }
    post(vehicle, tick, event, events);
}

const TIMED_BOOST_LOW: f32 = 0.5714286;
const TIMED_BOOST_HIGH: f32 = 0.85714287;

const TURBO_PADS: [u8; 3] =
    [crate::surface::TURBO, crate::surface::TURBO_WOOD, crate::surface::TURBO_TECH_MAGNETIC];
const RED_TURBO_PADS: [u8; 3] =
    [crate::surface::TURBO_2, crate::surface::TURBO_2_WOOD, crate::surface::TURBO_2_TECH_MAGNETIC];
const TIMED_TURBO_PADS: [u8; 1] = [crate::surface::TURBO_ROULETTE];
const FREE_WHEELING_PADS: [u8; 3] = [
    crate::surface::FREE_WHEELING,
    crate::surface::FREE_WHEELING_WOOD,
    crate::surface::FREE_WHEELING_TECH_MAGNETIC,
];
const NO_GRIP_PADS: [u8; 1] = [crate::surface::NO_GRIP];
const NO_STEERING_PADS: [u8; 1] = [crate::surface::NO_STEERING];
const NO_BRAKES_PADS: [u8; 1] = [crate::surface::NO_BRAKES];
const WHEEL_METER_GROUND: [u8; 3] = [crate::surface::SNOW, crate::surface::SAND, crate::surface::GRASS];
