use crate::session::Session;
use crate::vehicle::Vehicle;
use crate::vehicle::spawn::Reset;

#[derive(Clone, Debug, Default)]
pub struct Respawns {
    checkpoint: Option<Checkpoint>,
    respawn_at: Option<u32>,
}

#[derive(Clone, Debug)]
struct Checkpoint {
    waypoint: u32,
    taken_at: u32,
    state: Box<Vehicle>,
}

fn delay() -> u32 {
    std::env::var("RESPAWN_DELAY").ok().and_then(|v| v.parse().ok()).unwrap_or(1000)
}

fn taken_offset() -> u32 {
    std::env::var("RESPAWN_TAKEN").ok().and_then(|v| v.parse().ok()).unwrap_or(10)
}

macro_rules! restore_fields {
    ($($field:ident),* $(,)?) => {
        fn restore(vehicle: &mut Vehicle, saved: &Vehicle) {
            $(vehicle.$field = saved.$field.clone();)*
        }
    };
}

restore_fields!(
    controls,
    previous_controls,
    model_key,
    control_flags,
    control_entity,
    race_start,
    frozen_until,
    quat,
    pos,
    vel,
    ang_vel,
    world_inverse_inertia,
    gravity,
    slow_start_begin,
    slow_start_end,
    slow_start_flags,
    extra_linear_velocity,
    extra_angular_velocity,
    wheel_meter_enabled,
    wheel_meter,
    wheel_meter_sum,
    wheel_meter_count,
    effect_flags,
    forced_speed,
    reactor_begin,
    level_power_scale,
    slow_motion_until,
    state_sequence,
    angular_freeze,
    in_water,
    slip_armed,
    reactor_ready,
    water_reentry_until,
    water_exit_until,
    immersion,
    boost_level,
    boost_maximum_level,
    boost_last_contact,
    boost_begin,
    ground_pull_begin,
    ground_pull_duration_ms,
    ground_roll_begin,
    turbo_active,
    turbo_begin,
    turbo_end,
    turbo_last_contact,
    turbo_thrust_scale,
    turbo_duration_ms,
    turbo_fade_ms,
    turbo_phase,
    timed_turbo_contact,
    reactor_turbo_begin,
    turbo_contact,
    body_contact,
    last_slip_armed,
    last_body_contact,
    angular_control_hold_time,
    held_pitch_rate,
    held_yaw_rate,
    force_velocity_lateral,
    force_velocity_vertical,
    force_velocity_forward,
    steering,
    roll_stabilization_active,
    roll_stabilization_released,
    roll_stabilization_start,
    adhesion_contact_time,
    adhesion_class,
    adhesion_normal_x,
    adhesion_normal_y,
    adhesion_normal_z,
    pivot_state,
    force_feedback_lateral,
    force_feedback_vertical,
    force_feedback_forward,
    pivot_center_x,
    pivot_center_y,
    pivot_center_z,
    pivot_support_radius,
    pivot_radius,
    pivot_arm_begin,
    pivot_retire_begin,
    pivot_axis_x,
    pivot_axis_y,
    pivot_axis_z,
    pivot_steering_sign,
    pivot_blocked,
    fixed_wheel_spin,
    grip_meter,
    special_grip_blend_begin,
    special_grip_blend_active,
    special_grip_blend,
    thrust_boost_until,
    thrust_boost_elapsed,
    drift_thrust_begin,
    drift_thrust_scale,
    drift_release_begin,
    drift_yaw_state,
    drift_direction,
    alternate_forward,
    brake_history,
    engine_flags,
    rpm,
    gear,
    rpm_boost,
    shift_remaining,
    gearbox_state,
    shift_rpm_rate,
    reversing,
    target_rpm,
    cached_gear,
    impact_thrust_until,
    contact_impact_thrust_until,
    impact_yaw_change,
    impact_yaw_begin,
    drivetrain_flags,
    drivetrain_state,
    model_visual_0,
    model_visual_1,
    model_visual_2,
    model_visual_3,
    spin_up_duration,
    spin_up_torque,
    spin_up_begin,
    last_roof_contact,
    engine_meter,
    front_spin,
    rear_spin,
    idle_position_x,
    idle_position_y,
    idle_position_z,
    suspension_requires_all_contacts,
    wear,
    thrust_scale,
    steering_scale,
    gravity_scale,
    signed_speed_control,
    grip_scale,
    fragility_offset,
    forced_speed_integer,
    pedal_speed_limit,
    forced_effect_flags,
    visual_race_start,
    visual_drift_direction,
    visual_angular_freeze,
    wheels,
    visual_contact_flags,
    previous_level_power,
    previous_drift_direction,
    slow_start_forward_threshold
);

fn shift_ms(time: &mut u32, delta: i32) {
    if time.wrapping_sub(1) > u32::MAX - 2 {
        return;
    }
    *time = if delta < 0 && *time < delta.unsigned_abs() { 0 } else { time.wrapping_add(delta as u32) };
}

fn shift_ns(time: &mut u64, delta: i32) {
    if *time != i64::MIN as u64 {
        *time = time.wrapping_add((i64::from(delta) * 1_000_000) as u64);
    }
}

fn shift_timers(v: &mut Vehicle, delta: i32) {
    for wheel in &mut v.wheels {
        shift_ns(&mut wheel.last_impact_time_ns, delta);
        shift_ms(&mut wheel.effect_start, delta);
        shift_ms(&mut wheel.sample_time, delta);
    }
    for time in [
        &mut v.spin_up_begin,
        &mut v.reactor_begin,
        &mut v.reactor_turbo_begin,
        &mut v.turbo_last_contact,
        &mut v.turbo_end,
        &mut v.turbo_begin,
        &mut v.slow_start_begin,
        &mut v.slow_start_end,
        &mut v.roll_stabilization_start,
        &mut v.angular_control_hold_time,
        &mut v.last_body_contact,
        &mut v.last_slip_armed,
        &mut v.pivot_retire_begin,
        &mut v.pivot_arm_begin,
        &mut v.special_grip_blend_begin,
        &mut v.thrust_boost_until,
        &mut v.drift_thrust_begin,
        &mut v.drift_release_begin,
        &mut v.boost_last_contact,
        &mut v.boost_begin,
        &mut v.ground_pull_begin,
        &mut v.ground_roll_begin,
        &mut v.water_reentry_until,
        &mut v.water_exit_until,
    ] {
        shift_ms(time, delta);
    }
    for time in [&mut v.impact_thrust_until, &mut v.contact_impact_thrust_until, &mut v.adhesion_contact_time]
    {
        shift_ns(time, delta);
    }
}

impl Session {
    pub fn keep_respawns(&mut self) {
        self.respawns.get_or_insert_with(Default::default);
    }

    pub fn respawn(&mut self, standing: bool) -> bool {
        let Some(respawns) = self.respawns.as_ref() else { return false };
        let now = self.clock.now;
        let pending = respawns.respawn_at.filter(|&at| now < at && at < now + delay());
        match respawns.checkpoint.clone() {
            None => self.respawn_at_start(),
            Some(checkpoint) if standing || pending.is_some() => self.respawn_standing(&checkpoint, pending),
            Some(checkpoint) => self.respawn_at_checkpoint(&checkpoint),
        }
    }

    fn respawn_at_start(&mut self) -> bool {
        let now = self.clock.now;
        let key = self.vehicle.key;
        let Some(landmark) = self
            .race
            .players
            .iter()
            .find(|racer| racer.has_entity(key))
            .map(|racer| racer.player.landmark)
            .filter(|&landmark| landmark != u32::MAX)
        else {
            return false;
        };
        self.respawn_at(now, landmark, Reset::FULL);
        self.vehicle.frozen_until = now;
        true
    }

    fn respawn_standing(&mut self, checkpoint: &Checkpoint, pending: Option<u32>) -> bool {
        let now = self.clock.now;
        let key = self.vehicle.key;
        if let Some(controller) = self
            .race
            .players
            .iter_mut()
            .find(|racer| racer.has_entity(key))
            .and_then(|racer| racer.controller.as_mut())
        {
            let count = controller.start_adjustments;
            if count & 3 != 0 {
                controller.start_adjustments = (count.wrapping_sub(1) ^ count) & 3 ^ count;
            }
        }
        self.vehicle.control_flags = self.vehicle.control_flags & !0xd | 2;
        self.respawn_at(now, checkpoint.waypoint, Reset { keep_effects: true, ..Reset::FULL });
        let saved = &checkpoint.state;
        let vehicle = &mut self.vehicle;
        vehicle.wheel_meter_enabled = saved.wheel_meter_enabled;
        vehicle.wheel_meter = saved.wheel_meter;
        vehicle.wheel_meter_sum = saved.wheel_meter_sum;
        vehicle.wheel_meter_count = saved.wheel_meter_count;
        vehicle.water_reentry_until = saved.water_reentry_until;
        vehicle.water_exit_until = saved.water_exit_until;
        vehicle.immersion = saved.immersion;
        let at = pending.unwrap_or(now + delay());
        vehicle.control_flags &= !0xf;
        vehicle.frozen_until = at;
        if let Some(respawns) = self.respawns.as_mut() {
            respawns.respawn_at = Some(at);
        }
        true
    }

    fn respawn_at_checkpoint(&mut self, checkpoint: &Checkpoint) -> bool {
        let at = self.clock.now + delay();
        let vehicle = &mut self.vehicle;
        let sequence = vehicle.respawn_sequence;
        restore(vehicle, &checkpoint.state);
        vehicle.respawn_sequence = sequence.wrapping_add(1);
        shift_timers(vehicle, at.wrapping_sub(checkpoint.taken_at) as i32);
        vehicle.frozen_until = at;
        vehicle.control_flags &= !0xf;
        vehicle.slow_motion_wheel_contact = None;
        vehicle.slow_motion_body_contact = None;
        vehicle.boost_contact = None;
        vehicle.last_effect_contact = None;
        vehicle.slow_motion_gate = u32::MAX;
        if let Some(respawns) = self.respawns.as_mut() {
            respawns.respawn_at = Some(at);
        }
        true
    }

    pub(crate) fn remember_for_respawn(&mut self, now: u32, crossed_before: usize) {
        let crossings = self.crossing_waypoints();
        let finished = self.race().is_some_and(|r| r.finish.is_some());
        let Some(respawns) = self.respawns.as_mut() else { return };
        let Some(&waypoint) = crossings.last() else { return };
        if crossings.len() == crossed_before || finished {
            return;
        }
        if respawns.checkpoint.as_ref().is_some_and(|c| c.waypoint == waypoint) {
            return;
        }
        let taken_at = now.wrapping_add(taken_offset());
        respawns.checkpoint = Some(Checkpoint { waypoint, taken_at, state: Box::new(self.vehicle.clone()) });
    }
}
