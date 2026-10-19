use super::{
    Tuning, Vehicle, VisualState,
    flags::{ActionFlags, EffectFlags, SurfaceFlags, ToggleFlags, WheelFlags},
    forces::common::{EPS, KMH},
    model::ForceGeometry,
};
use crate::race::countdown::{ControlFlags, ControlPhase};
use crate::{
    Iso, Mat3, Quat, Vec3,
    dyna::{BodyHandle, Dyna},
    math::{maxss, minss, quantize},
};

const STEP: f32 = 0.001;
const QUAT_STEP: f32 = 1.0e-6;
const BRAKING_THRESHOLD: f32 = 0.9;
const LOW_GRIP_SLIP: f32 = 0.085;

pub struct Config<'a> {
    pub tuning: &'a Tuning,
    pub geometry: &'a ForceGeometry,
    pub tick: u32,
    pub recent_event_duration: u32,
}

pub struct Refresh<'a> {
    pub config: Config<'a>,
}
impl super::spawn::Visuals for Refresh<'_> {
    fn visual(&mut self, vehicle: &mut Vehicle, dyna: &Dyna, _fresh: bool) {
        vehicle_vis(vehicle, dyna, &self.config, 1);
    }
}

struct Flags(u32);
impl Flags {
    const DRIVETRAIN: Self = Self(2);
    const FROZEN: Self = Self(8);
    const RECENT_ROOF_CONTACT: Self = Self(16);
    const FIXED_SPIN: Self = Self(32);
    const CONTACT_EFFECT: Self = Self(64);
    const CONTACT_EFFECT_ON: Self = Self(128);
    const CONTROL_A: Self = Self(256);
    const CONTROL_B: Self = Self(512);
    const CONTROL_C: Self = Self(1024);
    const SPIN_UP: Self = Self(2048);
    const EXTRA_MODEL: Self = Self(4096);
    const BOOST_MATERIAL: Self = Self(8192);
    const RESERVED_EFFECT: Self = Self(32768);
    const ANGULAR_HOLD: Self = Self(65536);
    const HARD_IMPACT: Self = Self(131072);
    const PITCH_INPUT: Self = Self(262144);
    const ADHESION_WINDOW: Self = Self(524288);
    const ANY_WHEEL_CONTACT: Self = Self(1048576);
    const RESERVED_MODE: Self = Self(2097152);
    const ENGINE_RUNNING: Self = Self(8388608);
    const TURBO: Self = Self(16777216);
    const SPIN_UNINITIALIZED: Self = Self(536870912);
    fn set(&mut self, flag: Self, on: bool) {
        self.0 = self.0 & !flag.0 | if on { flag.0 } else { 0 };
    }
}

pub fn vehicle_vis_fast(vehicle: &mut Vehicle) {
    vehicle.visual[1].state = vehicle.model_choice;
}

pub fn vis_state(vehicle: &Vehicle, config: &Config<'_>, velocity: Vec3, visual: &mut VisualState) {
    let tuning = config.tuning;
    let tick = config.tick;
    visual.tick = tick;
    visual.key = vehicle.key;
    visual.model_key = vehicle.model_key;
    visual.state = vehicle.model_choice;
    visual.previous_angular_freeze = vehicle.visual_angular_freeze;
    visual.drift_direction = vehicle.visual_drift_direction;
    visual.previous_race_start = vehicle.visual_race_start;
    visual.forward_speed = quantize(velocity.z, STEP);
    visual.lateral_speed = quantize(velocity.x, STEP);
    visual.speed_kmh = crate::numeric::lroundf(vehicle.forced_speed * KMH) as u32;
    visual.frozen_until = vehicle.frozen_until;
    let mut flags = Flags(visual.flags);
    drivetrain(vehicle, tuning, tick, visual, &mut flags);
    pose(vehicle, visual);
    inputs(vehicle, velocity, visual, &mut flags);
    boost(vehicle, tuning, velocity, tick, visual, &mut flags);
    status(vehicle, tuning, tick, visual, &mut flags);
    wheel_states(vehicle, config, visual);
    visual.model_state =
        [vehicle.model_visual_0, vehicle.model_visual_1, vehicle.model_visual_2, vehicle.model_visual_3];
    visual.wear = vehicle.wear;
    visual.recent_events = [0; 5];
    visual.race_start = vehicle.race_start;
    visual.flags = flags.0;
}

fn drivetrain(vehicle: &Vehicle, tuning: &Tuning, tick: u32, visual: &mut VisualState, flags: &mut Flags) {
    flags.set(Flags::TURBO, vehicle.turbo_duration_ms != 0);
    let mut fraction = 0.0;
    if vehicle.turbo_duration_ms != 0 && vehicle.turbo_end != u32::MAX {
        let ratio = tick.wrapping_sub(vehicle.turbo_end) as f32 / vehicle.turbo_duration_ms as f32;
        fraction = if ratio >= 0.0 { minss(ratio, 1.0) } else { 0.0 };
    }
    visual.turbo_fraction = fraction;
    flags.set(Flags::ENGINE_RUNNING, ToggleFlags(vehicle.engine_flags).enabled());
    visual.rpm = vehicle.rpm;
    visual.gear = vehicle.gear;
    visual.gearbox_state = vehicle.gearbox_state;
    let until = (vehicle.adhesion_contact_time as i64 / 1_000_000) as u32;
    flags.set(Flags::ADHESION_WINDOW, tick < until.wrapping_add(tuning.level_air_control_delay_ms));
    visual.immersion = vehicle.immersion;
    visual.shift_rate = quantize(vehicle.shift_rpm_rate, STEP);
    visual.effect_flags = vehicle.effect_flags;
    flags.set(Flags::FROZEN, ToggleFlags(vehicle.angular_freeze).enabled());
    let mode = match vehicle.turbo_active {
        1 | 4 | 5 => Some(1),
        2 => Some(2),
        3 => Some(if 0.25 > vehicle.turbo_phase {
            3
        } else if 0.75 > vehicle.turbo_phase {
            4
        } else {
            5
        }),
        _ => None,
    };
    if let Some(mode) = mode {
        visual.turbo = mode;
    }
}

fn pose(vehicle: &Vehicle, visual: &mut VisualState) {
    visual.pose = Iso::from_quat_pos(vehicle.quat, vehicle.pos);
    let source = vehicle.quat;
    let quantised = Quat::new(
        quantize(source.w, QUAT_STEP),
        quantize(source.x, QUAT_STEP),
        quantize(source.y, QUAT_STEP),
        quantize(source.z, QUAT_STEP),
    );
    let squared = ((quantised.x * quantised.x + quantised.w * quantised.w) + quantised.y * quantised.y)
        + quantised.z * quantised.z;
    if 0.0 > squared {
        todo!("sqrtf of a negative");
    }
    let inverse = 1.0 / squared.sqrt();
    let rotation = if inverse > EPS && f32::MAX > inverse {
        Quat::new(quantised.w * inverse, quantised.x * inverse, quantised.y * inverse, quantised.z * inverse)
    } else {
        Quat::IDENTITY
    };
    visual.pose.rot = Mat3::from_quat(rotation);
    visual.pose.pos = quantize3(vehicle.pos);
    visual.velocity = quantize3(vehicle.vel + vehicle.extra_linear_velocity);
    visual.angular_velocity = quantize3(vehicle.ang_vel + vehicle.extra_angular_velocity);
}

fn quantize3(vector: Vec3) -> Vec3 {
    Vec3::new(quantize(vector.x, STEP), quantize(vector.y, STEP), quantize(vector.z, STEP))
}

fn inputs(vehicle: &Vehicle, velocity: Vec3, visual: &mut VisualState, flags: &mut Flags) {
    visual.steering = if vehicle.has_effect_flag(8) { 0.0 } else { vehicle.controls.steer };
    visual.gas = vehicle.controls.gas;
    visual.brake = vehicle.controls.brake;
    visual.drivetrain_state = vehicle.drivetrain_state;
    visual.action = ActionFlags(vehicle.controls.actions).contains(ActionFlags::SECONDARY) as u32;
    flags.set(Flags::DRIVETRAIN, ToggleFlags(vehicle.drivetrain_flags).enabled());
    visual.braking = if vehicle.has_effect_flag(4) {
        0
    } else {
        let forward = vehicle.gear == 0 && -2.0 >= velocity.z;
        let pedal = if forward { vehicle.controls.gas } else { vehicle.controls.brake };
        (pedal > BRAKING_THRESHOLD) as u32
    };
    visual.secondary_action = (ToggleFlags(vehicle.controls.auxiliary_bytes[0] as u32).enabled()
        || ActionFlags(vehicle.controls.actions).contains(ActionFlags::TRANSIENT)
        || ToggleFlags(vehicle.controls.auxiliary_words[11] as u32).enabled())
        as u32;
}

fn boost(
    vehicle: &Vehicle,
    tuning: &Tuning,
    velocity: Vec3,
    tick: u32,
    visual: &mut VisualState,
    flags: &mut Flags,
) {
    let fixed_spin = if vehicle.fixed_wheel_spin != 0 {
        true
    } else if vehicle.effect_mode_is_free()
        && ControlFlags(vehicle.control_flags).phase() == ControlPhase::Countdown
        && vehicle.in_water == 0
    {
        let speed = velocity.z * KMH;
        speed > tuning.fixed_spin_minimum_speed
            && tuning.fixed_spin_maximum_speed > speed
            && vehicle.controls.gas > 0.0
    } else {
        false
    };
    flags.set(Flags::FIXED_SPIN, fixed_spin);
    if EffectFlags::FRAGILE.contains_in(vehicle.effect_flags) {
        flags.set(Flags::FIXED_SPIN, false);
    }
    let maximum = vehicle.maximum_level();
    if maximum <= 2 {
        visual.maximum_level = maximum;
    }
    let level = vehicle.level();
    if level <= 3 {
        visual.level = level;
    }
    let mut fade = 0.0;
    if visual.level != 0 {
        let left =
            vehicle.ground_pull_duration_ms.wrapping_add(vehicle.boost_last_contact).wrapping_sub(tick);
        if left < tuning.boost_visual_fade_ms {
            fade = 1.0 - left as f32 / tuning.boost_visual_fade_ms as f32;
        }
    }
    visual.boost_fade = fade;
    flags.set(Flags::ANY_WHEEL_CONTACT, vehicle.any_wheel_contact());
    visual.level_power = vehicle.level_power(tuning);
    flags.set(Flags::BOOST_MATERIAL, vehicle.wheels.iter().any(|wheel| wheel.has_boost_material()));
}

fn status(vehicle: &Vehicle, tuning: &Tuning, tick: u32, visual: &mut VisualState, flags: &mut Flags) {
    flags.set(Flags::RESERVED_MODE, false);
    visual.front_spin = vehicle.front_spin;
    visual.rear_spin = vehicle.rear_spin;
    if matches!(tuning.model_kind, 9 | 15) {
        todo!("visual state of vehicle model 9/15");
    }
    visual.model_extra_state = 0;
    flags.set(Flags::EXTRA_MODEL, false);
    flags.set(
        Flags::CONTACT_EFFECT,
        SurfaceFlags(vehicle.visual_contact_flags).intersects(SurfaceFlags::VISUAL_CONTACTS),
    );
    flags.set(
        Flags::CONTACT_EFFECT_ON,
        SurfaceFlags(vehicle.visual_contact_flags).intersects(SurfaceFlags::CONTACT),
    );
    flags.set(Flags::CONTROL_A, ControlFlags(vehicle.control_flags).contains(ControlFlags::VISUAL_A));
    flags.set(Flags::CONTROL_B, ControlFlags(vehicle.control_flags).contains(ControlFlags::VISUAL_B));
    flags.set(Flags::CONTROL_C, ControlFlags(vehicle.control_flags).contains(ControlFlags::VISUAL_C));
    let power = vehicle.level_power(tuning);
    visual.slow_motion = if power >= 1.0 { 0 } else { 1 + !(power >= tuning.level_power_base) as u8 };
    flags.set(Flags::RESERVED_EFFECT, false);
    visual.state_sequence = vehicle.state_sequence;
    if tuning.model_kind == 6 {
        todo!("visual state of model 6");
    }
    flags.set(
        Flags::ANGULAR_HOLD,
        vehicle.angular_control_hold_time != 0 && vehicle.angular_control_hold_time.wrapping_add(300) <= tick,
    );
    visual.control_phase = ControlFlags(vehicle.control_flags).phase_code();
    visual.respawn_sequence = vehicle.respawn_sequence;
    flags.set(Flags::SPIN_UP, vehicle.spin_up_duration != 0);
    visual.meter = vehicle.engine_meter;
    flags.set(Flags::SPIN_UNINITIALIZED, vehicle.spin_up_begin == u32::MAX);
    flags.set(
        Flags::RECENT_ROOF_CONTACT,
        vehicle.last_roof_contact != u32::MAX && tick < vehicle.last_roof_contact.wrapping_add(2000),
    );
}

fn wheel_states(vehicle: &Vehicle, config: &Config<'_>, visual: &mut VisualState) {
    let tuning = config.tuning;
    let coefficients =
        vehicle.wheel_effect_coefficients(tuning, config.tick, &config.geometry.effect_contacts);
    for index in 0..config.geometry.effect_contacts.len() {
        let wheel = &vehicle.wheels[index];
        let out = &mut visual.wheels[index];
        out.suspension = quantize(wheel.suspension_length - tuning.suspension_anchor_offset, STEP);
        out.spin_angle = quantize(wheel.spin_angle, STEP);
        out.spin = quantize(wheel.spin, STEP);
        out.steering_angle = quantize(wheel.steering_angle, STEP);
        let mut flags = WheelFlags(out.flags);
        flags.set(WheelFlags::CONTACT, wheel.contact != 0 && vehicle.level_consistent());
        flags.set(WheelFlags::BOOST, ToggleFlags(wheel.boost_contact).enabled());
        out.flags = flags.0;
        out.material = wheel.material as u16 | (wheel.surface as u16) << 8;
        let low = tuning.fragility_minimum;
        let fragility = 1.0 - (wheel.drive_coefficient - low) / (1.0 - low) + vehicle.fragility_offset;
        out.fragility = if fragility >= 0.0 { minss(fragility, 1.0) } else { 0.0 };
        out.forced_effect_flags = vehicle.wheel_meter.to_bits();
        let coefficient = coefficients.wheels[index];
        out.slip = if tuning.low_grip_threshold >= coefficient { LOW_GRIP_SLIP } else { wheel.slip };
        let denominator = maxss(1.0 - tuning.effect_minimum_grip, tuning.effect_coefficient_minimum);
        out.effect_loss = (1.0 - coefficient) / denominator;
    }
}

pub fn vehicle_vis(vehicle: &mut Vehicle, dyna: &Dyna, config: &Config<'_>, index: usize) {
    let velocity =
        if vehicle.body != u32::MAX { dyna.local_velocity(BodyHandle(vehicle.body)) } else { Vec3::ZERO };
    let mut visual = vehicle.visual[index].clone();
    vis_state(vehicle, config, velocity, &mut visual);
    if super::race_flags::RaceFlags(vehicle.flags).contains(super::race_flags::RaceFlags::GHOST_RESPONSE) {
        todo!("visual speed from quantised velocity");
    }
    let force = vehicle.pending_contact_force;
    let squared = (force.x * force.x + force.y * force.y) + force.z * force.z;
    if 0.0 > squared {
        todo!("sqrtf of a negative");
    }
    if squared.sqrt() != 0.0 {
        todo!("visual impact time");
    }
    visual.input_direction = [
        vehicle.pitch_input_direction.to_bits(),
        vehicle.yaw_input_direction.to_bits(),
        vehicle.angular_control_scratch,
    ];
    let mut flags = Flags(visual.flags);
    flags.set(Flags::PITCH_INPUT, ToggleFlags(vehicle.pitch_input_active).enabled());
    flags.set(Flags::HARD_IMPACT, ToggleFlags(vehicle.hard_impact).enabled());
    visual.spring_forward = vehicle.forward_spring_value;
    visual.spring_lateral = vehicle.lateral_spring_value;
    visual.layer = vehicle.surface_layer;
    visual.bump_frequency = vehicle.bump_frequency;
    visual.bump_amplitude = vehicle.bump_amplitude;
    copy_history(vehicle, &mut visual);
    for (index, layer) in vehicle.wheel_layers.into_iter().enumerate() {
        visual.wheels[index].layer = layer;
    }
    visual.sound_speed = vehicle.sound_speed;
    let delay = config.tuning.jump_rearm_ms;
    if config.tick.wrapping_sub(vehicle.last_jump) < delay
        || config.tick.wrapping_sub(vehicle.recent_effect_time) < delay
    {
        todo!("visual recent respawn fade");
    }
    for (index, last) in vehicle.recent_events.into_iter().enumerate() {
        visual.recent_events[index] =
            (last != 0 && config.tick < last.wrapping_add(config.recent_event_duration)) as u8;
    }
    visual.flags = flags.0;
    vehicle.visual[index] = visual;
}

fn copy_history(vehicle: &Vehicle, visual: &mut VisualState) {
    visual.damage_states = vehicle.damage_states;
    visual.detached_parts = vehicle.detached_parts;
    visual.history_poses = vehicle.history_poses;
    visual.wheel_flags = vehicle.damage_flags;
}
