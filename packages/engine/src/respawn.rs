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

const DELAY_MS: u32 = 1000;
const SNAPSHOT_DELAY_MS: u32 = 10;

fn restore(vehicle: &mut Vehicle, saved: &Vehicle) {
    vehicle.copy_state(saved);
    vehicle.model_key = saved.model_key;
    vehicle.control_entity = saved.control_entity;
    vehicle.visual_race_start = saved.visual_race_start;
    vehicle.visual_drift_direction = saved.visual_drift_direction;
    vehicle.visual_angular_freeze = saved.visual_angular_freeze;
    vehicle.visual_contact_flags = saved.visual_contact_flags;
}

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
    pub fn track_respawns(&mut self) {
        self.respawns.get_or_insert_with(Default::default);
    }

    pub fn respawn(&mut self, standing: bool) -> bool {
        let Some(respawns) = self.respawns.as_ref() else { return false };
        let now = self.clock.now;
        let pending = respawns.respawn_at.filter(|&at| now < at && at < now + DELAY_MS);
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
                controller.start_adjustments = count & !3 | (count - 1) & 3;
            }
        }
        self.vehicle.control_flags = self.vehicle.control_flags & !0xd | 2;
        self.respawn_at(now, checkpoint.waypoint, Reset { keep_effects: true, ..Reset::FULL });
        let vehicle = &mut self.vehicle;
        copy_fields! { vehicle, checkpoint.state, [
            wheel_meter_enabled, wheel_meter, wheel_meter_sum, wheel_meter_count, water_reentry_until,
            water_exit_until, immersion,
        ] }
        let at = pending.unwrap_or(now + DELAY_MS);
        vehicle.control_flags &= !0xf;
        vehicle.frozen_until = at;
        if let Some(respawns) = self.respawns.as_mut() {
            respawns.respawn_at = Some(at);
        }
        true
    }

    fn respawn_at_checkpoint(&mut self, checkpoint: &Checkpoint) -> bool {
        let at = self.clock.now + DELAY_MS;
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
        let taken_at = now.wrapping_add(SNAPSHOT_DELAY_MS);
        respawns.checkpoint = Some(Checkpoint { waypoint, taken_at, state: Box::new(self.vehicle.clone()) });
    }
}
