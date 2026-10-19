use super::super::{Tuning, Vehicle};
use super::common::{PEDAL_THRESHOLD, PedalMode, PivotState};
use crate::math::minss;

const LEGACY_SHIFT_SECONDS: f32 = 0.04;
const SHIFT_SECONDS: f32 = 0.025;
const PIVOT_SHIFT_SECONDS: f32 = 0.002;
const MAX_RPM_BOOST: f32 = 1.15;
const UPSHIFT_RPM_RATE: f32 = -5000.0;
const DOWNSHIFT_RPM_RATE: f32 = 5000.0;
const LEGACY_SPEED_SCALE: f32 = 0.2;
const LATERAL_RPM_WEIGHT: f32 = 0.3;
const RPM_BOOST_RATE: f32 = 0.3;
const SHIFT_RPM_DECAY: f32 = 1.9;
const LEGACY_GROUND_RPM_RATE: f32 = 12.0;
const LEGACY_AIR_RPM_RATE: f32 = 3.5;
const DIRECTION_CHANGE_RPM: f32 = 1000.0;
const TOP_GEAR: u32 = 5;

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub(super) enum GearboxState {
    ForwardLaunch = 0,
    Shifting = 2,
    Loaded = 3,
    ReverseLaunch = 4,
    Pivot = 5,
}

impl GearboxState {
    pub(super) fn from_code(code: u32) -> Option<Self> {
        match code {
            0 => Some(Self::ForwardLaunch),
            2 => Some(Self::Shifting),
            3 => Some(Self::Loaded),
            4 => Some(Self::ReverseLaunch),
            5 => Some(Self::Pivot),
            _ => None,
        }
    }
}

impl Vehicle {
    pub fn update_engine_gearbox(&mut self, tuning: &Tuning, gas: f32, dt: f32) {
        if self.effect_mode_is(PedalMode::Cut) {
            self.rpm = 0.0;
            return;
        }
        let throttle = gas > PEDAL_THRESHOLD;
        let low_grip = tuning.low_grip_threshold >= self.grip;
        let airborne = !self.any_wheel_contact();
        let mut timer = self.shift_remaining;
        if timer > 0.0 {
            timer = timer - dt;
            self.shift_remaining = timer;
        }
        let held = airborne || timer > 0.0;
        if tuning.car_model_kind().uses_legacy_gearbox() {
            self.advance_legacy_rpm(tuning, gas, dt, held, timer);
        } else if held {
            self.rpm = if throttle {
                dt * tuning.rpm_air_acceleration + self.rpm
            } else {
                self.rpm - dt * tuning.rpm_air_deceleration
            };
        } else {
            let pivot = PivotState::from_code(self.pivot_state).uses_pivot_rpm();
            self.advance_sport_rpm(tuning, throttle, low_grip, pivot, dt);
            self.select_launch(tuning, throttle, low_grip);
            self.select_gear(tuning, pivot);
        }
        self.finish_rpm(tuning, self.rpm);
    }

    fn advance_legacy_rpm(&mut self, tuning: &Tuning, gas: f32, dt: f32, held: bool, timer: f32) {
        let pedal = gas.abs();
        let scale = tuning.forward_speed_limit * LEGACY_SPEED_SCALE;
        let gear = self.gear;
        let ratio_index = if gear > 1 { gear - 1 } else { 0 } as usize;
        let lateral = self.force_velocity_lateral;
        let forward = self.force_velocity_forward;
        let vertical = self.force_velocity_vertical;
        let down = tuning.downshift_ratios.0[ratio_index];
        let speed = (lateral * lateral * LATERAL_RPM_WEIGHT
            + forward * forward
            + vertical * vertical * PEDAL_THRESHOLD)
            .sqrt();
        let ratio = speed / scale * tuning.gear_ratios.0[ratio_index];
        let (target, rate);
        if !held {
            if self.reversing != 0 {
                if gear != 0 {
                    self.begin_legacy_shift(0);
                }
            } else if gear == 0 {
                self.begin_legacy_shift(1);
            } else if ratio > tuning.upshift_ratios.0[ratio_index] && gear < TOP_GEAR {
                self.begin_legacy_shift(gear + 1);
            } else if down > ratio && gear > 1 {
                self.begin_legacy_shift(gear - 1);
            }
            target = ratio;
            rate = LEGACY_GROUND_RPM_RATE;
        } else {
            if timer >= 0.0 && dt + dt >= timer {
                self.rpm = self.rpm - dt * tuning.maximum_rpm * SHIFT_RPM_DECAY;
            }
            target = pedal;
            rate = LEGACY_AIR_RPM_RATE;
        }
        let current = self.rpm;
        self.rpm = (target * tuning.maximum_rpm - current) * dt * rate + current;
    }

    fn advance_sport_rpm(&mut self, tuning: &Tuning, throttle: bool, low_grip: bool, pivot: bool, dt: f32) {
        if pivot && self.gear != 0 {
            self.gearbox_state = GearboxState::Pivot as u32;
            self.advance_pivot_rpm(tuning, dt);
            return;
        }
        match GearboxState::from_code(self.gearbox_state) {
            Some(GearboxState::Pivot) => {
                self.gearbox_state = GearboxState::Loaded as u32;
                self.advance_loaded_rpm(tuning, throttle, pivot, dt);
            }
            Some(GearboxState::ForwardLaunch) => {
                self.advance_launch_rpm(tuning, throttle, low_grip, false, dt);
            }
            Some(GearboxState::Shifting | GearboxState::Loaded) => {
                self.advance_loaded_rpm(tuning, throttle, pivot, dt);
            }
            Some(GearboxState::ReverseLaunch) => {
                self.advance_launch_rpm(tuning, throttle, low_grip, true, dt);
            }
            None => self.gearbox_state = GearboxState::Loaded as u32,
        }
    }

    fn select_launch(&mut self, tuning: &Tuning, throttle: bool, low_grip: bool) {
        let forward = self.force_velocity_forward;
        let reverse = self.reversing != 0;
        let loaded = GearboxState::from_code(self.gearbox_state) == Some(GearboxState::Loaded);
        let forward_launch = forward > tuning.forward_launch_lower_speed
            && tuning.forward_launch_upper_speed > forward
            && throttle
            && loaded
            && !reverse
            && !low_grip;
        let (reverse_launch, forward_grip_launch) = if forward > tuning.reverse_launch_lower_speed
            && tuning.reverse_launch_upper_speed > forward
            && throttle
            && loaded
            && reverse
            && !low_grip
        {
            (true, false)
        } else {
            (false, low_grip && tuning.forward_hold_speed > forward && !reverse)
        };
        let reverse_grip_launch = low_grip && forward > tuning.reverse_hold_speed && reverse;
        if forward_launch || forward_grip_launch {
            self.gearbox_state = GearboxState::ForwardLaunch as u32;
            if self.gear == 0 {
                self.gear = 1;
                self.shift_remaining = SHIFT_SECONDS;
            }
        } else if reverse_launch || reverse_grip_launch {
            self.gearbox_state = GearboxState::ReverseLaunch as u32;
            if self.gear != 0 {
                self.gear = 0;
                self.shift_remaining = SHIFT_SECONDS;
            }
        }
    }

    fn select_gear(&mut self, tuning: &Tuning, pivot: bool) {
        if !matches!(
            GearboxState::from_code(self.gearbox_state),
            Some(GearboxState::Shifting | GearboxState::Loaded)
        ) {
            return;
        }
        let mut gear = self.gear;
        if self.reversing != 0 {
            if gear != 0 {
                self.gearbox_state = GearboxState::Shifting as u32;
                self.shift_rpm_rate = DOWNSHIFT_RPM_RATE;
                if DIRECTION_CHANGE_RPM > self.rpm {
                    self.gear = 0;
                    self.shift_remaining = if pivot { PIVOT_SHIFT_SECONDS } else { SHIFT_SECONDS };
                }
            }
            return;
        }
        if gear == 0 {
            self.gearbox_state = GearboxState::Shifting as u32;
            self.shift_rpm_rate = UPSHIFT_RPM_RATE;
            if DIRECTION_CHANGE_RPM > self.rpm {
                self.gear = 1;
                gear = 1;
                self.shift_remaining = SHIFT_SECONDS;
            } else {
                return;
            }
        }
        let maximum = tuning.maximum_rpm;
        let target = self.target_rpm;
        if target > maximum * tuning.upshift_ratios.0[gear as usize] && gear < TOP_GEAR {
            self.gear = gear + 1;
            self.shift_remaining = SHIFT_SECONDS;
            self.gearbox_state = GearboxState::Shifting as u32;
            self.shift_rpm_rate = UPSHIFT_RPM_RATE;
        } else if maximum * tuning.downshift_ratios.0[gear as usize] > target && gear > 1 {
            self.gear = gear - 1;
            self.shift_remaining = SHIFT_SECONDS;
            self.gearbox_state = GearboxState::Shifting as u32;
            self.shift_rpm_rate = DOWNSHIFT_RPM_RATE;
        }
    }

    fn advance_pivot_rpm(&mut self, tuning: &Tuning, dt: f32) {
        let maximum = tuning.maximum_rpm;
        let current = self.rpm;
        self.rpm_boost = MAX_RPM_BOOST;
        self.target_rpm = maximum;
        if maximum > current {
            self.rpm = dt * tuning.rpm_pivot_rate + current;
        } else if current > maximum {
            self.rpm = dt * tuning.rpm_air_deceleration + current;
        }
    }

    fn advance_loaded_rpm(&mut self, tuning: &Tuning, throttle: bool, pivot: bool, dt: f32) {
        let armed = self.slip_armed;
        let boost = if armed != 0 && throttle {
            let current = self.rpm_boost;
            let next = if MAX_RPM_BOOST > current {
                (MAX_RPM_BOOST - current) * RPM_BOOST_RATE * dt + current
            } else {
                MAX_RPM_BOOST
            };
            self.rpm_boost = next;
            next
        } else {
            self.rpm_boost = 1.0;
            1.0
        };
        let gear = self.gear;
        let ground_speed = self.ground_speed();
        let mut target = (ground_speed * boost).abs() * tuning.gear_ratios.0[gear as usize];
        self.target_rpm = target;
        if (self.reversing != 0) == (gear != 0) {
            target = 0.0;
            self.target_rpm = 0.0;
        }
        let current = self.rpm;
        if target > current {
            let next = dt * tuning.rpm_loaded_acceleration + current;
            self.rpm = next;
            if next > target {
                self.gearbox_state = GearboxState::Loaded as u32;
            }
        } else {
            let rate = if throttle {
                if armed != 0 && !pivot {
                    tuning.rpm_air_deceleration
                } else {
                    tuning.rpm_loaded_deceleration
                }
            } else {
                tuning.rpm_coast_rate
            };
            let next = current - dt * rate;
            self.rpm = next;
            if target > next {
                self.gearbox_state = GearboxState::Loaded as u32;
            }
        }
    }

    fn advance_launch_rpm(
        &mut self,
        tuning: &Tuning,
        throttle: bool,
        low_grip: bool,
        reverse: bool,
        dt: f32,
    ) {
        let forward = self.force_velocity_forward;
        let coasting = if reverse {
            if low_grip {
                tuning.reverse_hold_speed > forward
            } else {
                tuning.reverse_launch_lower_speed > forward || forward > tuning.reverse_launch_upper_speed
            }
        } else if low_grip {
            forward > tuning.forward_hold_speed
        } else {
            forward > tuning.forward_launch_upper_speed || tuning.forward_launch_lower_speed > forward
        };
        self.rpm_boost = 1.0;
        let ratio_index = if reverse { 0 } else { 1 };
        let target = forward.abs() * tuning.gear_ratios.0[ratio_index];
        self.target_rpm = target;
        if coasting || !throttle {
            let next = self.rpm - dt * tuning.rpm_coast_rate;
            self.rpm = next;
            if target >= next {
                self.rpm = target;
                self.gearbox_state = GearboxState::Loaded as u32;
            }
        } else {
            self.rpm = dt * tuning.rpm_launch_rate + self.rpm;
        }
    }

    fn begin_legacy_shift(&mut self, gear: u32) {
        self.gear = gear;
        self.shift_remaining = LEGACY_SHIFT_SECONDS;
    }

    fn finish_rpm(&mut self, tuning: &Tuning, rpm: f32) {
        self.rpm = if rpm >= 0.0 { minss(rpm, tuning.maximum_rpm) } else { 0.0 };
    }

    pub fn ground_speed(&self) -> f32 {
        if !self.wheels.iter().any(|wheel| wheel.has_boost_material()) {
            return self.force_velocity_forward;
        }
        (self.force_velocity_lateral * self.force_velocity_lateral
            + 0.0
            + self.force_velocity_forward * self.force_velocity_forward)
            .sqrt()
    }
}
