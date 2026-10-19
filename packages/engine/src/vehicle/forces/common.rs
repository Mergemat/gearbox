use super::super::{Tuning, Vehicle};
use crate::Vec3;
use crate::math::{maxss, minss, sincos};

pub(crate) const KMH: f32 = 3.6;
pub(crate) const EPS: f32 = 0.00001;
pub(crate) const TINY: f32 = 1.0e-10;

pub(crate) const PEDAL_THRESHOLD: f32 = 0.1;

pub(crate) const PI: f32 = std::f32::consts::PI;
pub(crate) const HALF_TURN_DEGREES: f32 = 180.0;
pub(crate) const MS_PER_SECOND: f32 = 1000.0;
pub(crate) const PIVOT_UP_THRESHOLD: f32 = 0.75;
pub(crate) const DIRECTION_DEAD_ZONE: f32 = 2.0;
pub(crate) const AMPLIFIED_EFFECT_SCALE: f32 = 1.3333334;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PedalMode {
    Free = 0,
    Cut = 1,
    Full = 2,
    Reserved = 3,
}

impl PedalMode {
    pub fn from_code(code: u8) -> Self {
        match code & 3 {
            0 => Self::Free,
            1 => Self::Cut,
            2 => Self::Full,
            _ => Self::Reserved,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PivotState {
    Idle,
    Armed,
    Turning,
    Retiring,
    Other(u32),
}

impl PivotState {
    pub fn from_code(code: u32) -> Self {
        match code {
            0 => Self::Idle,
            1 => Self::Armed,
            2 => Self::Turning,
            3 => Self::Retiring,
            _ => Self::Other(code),
        }
    }

    pub fn code(self) -> u32 {
        match self {
            Self::Idle => 0,
            Self::Armed => 1,
            Self::Turning => 2,
            Self::Retiring => 3,
            Self::Other(code) => code,
        }
    }

    pub fn uses_pivot_rpm(self) -> bool {
        matches!(self, Self::Armed | Self::Turning)
    }
}

const NEGATIVE_DRAG_SCALE: f32 = -0.25;
const POSITIVE_DRAG_SCALE: f32 = 0.19999999;
const RESPONSE_FLOOR: f32 = 0.7;
const SPEED_DRAG_SCALE: f32 = 0.15;

pub fn asym_scale(x: f32) -> f32 {
    if x >= 0.0 { x * POSITIVE_DRAG_SCALE } else { -x * NEGATIVE_DRAG_SCALE }
}

pub fn sign_one(x: f32) -> f32 {
    if x.to_bits() >> 31 != 0 { -1.0 } else { 1.0 }
}

pub fn clamp01(x: f32) -> f32 {
    if x >= 0.0 { minss(x, 1.0) } else { 0.0 }
}

pub fn timed_ramp(now: u32, duration: u32, start: u32, base: f32, up: bool) -> f32 {
    let sign = if up { 1.0 } else { -1.0 };
    let x = (now.wrapping_sub(start) as f32 * sign) / duration as f32 + base;
    if x < 0.0 {
        return 0.0;
    }
    if 1.0 <= x { 1.0 } else { x }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PiecewiseResponse {
    pub keys: [f32; 5],
    pub values: [f32; 4],
    pub alternate_values: [f32; 4],
}

impl PiecewiseResponse {
    pub fn factor(&self, x: f32, scale: f32, alternate: bool) -> f32 {
        let keys = self.keys;
        let values = if alternate { self.alternate_values } else { self.values };
        let response = if keys[0] > x {
            values[0]
        } else if keys[1] > x {
            values[0] - (values[0] - values[1]) * (x - keys[0]) / (keys[1] - keys[0])
        } else if keys[2] > x {
            values[1] - (values[1] - values[2]) * (x - keys[1]) / (keys[2] - keys[1])
        } else if keys[3] > x {
            values[2]
        } else if keys[4] > x {
            values[2] - (x - keys[3]) * (values[2] - values[3]) / (keys[4] - keys[3])
        } else {
            values[3]
        };
        maxss(response * scale + 1.0, RESPONSE_FLOOR)
    }
}

pub fn speed_drag(velocity: Vec3, x: f32, y: f32, speed_scale: f32, body_scale: f32) -> Vec3 {
    let Vec3 { x: vx, y: vy, z: vz } = velocity;
    let fourth_root = ((vx * vx + vy * vy + vz * vz).sqrt()).sqrt();
    let response = fourth_root * speed_scale * x.abs().sqrt() * x;
    let response = if response >= -1.0 { minss(response, 0.0) } else { -1.0 };
    let scale = y * body_scale + response * SPEED_DRAG_SCALE;
    Vec3::new(vx * scale, vy * scale, vz * scale)
}

impl Vehicle {
    pub fn pivot_state(&self) -> PivotState {
        PivotState::from_code(self.pivot_state)
    }

    pub fn set_pivot_state(&mut self, state: PivotState) {
        self.pivot_state = state.code();
    }

    pub fn effect_mode_is(&self, mode: PedalMode) -> bool {
        PedalMode::from_code(self.forced_effect_flags as u8) == mode
            || PedalMode::from_code(self.effect_flags) == mode
    }

    pub fn effect_mode_is_free(&self) -> bool {
        self.forced_effect_flags as u8 & 3 == 0 && self.effect_flags & 3 == 0
    }

    pub fn has_effect_flag(&self, bit: u8) -> bool {
        (self.forced_effect_flags as u8 | self.effect_flags) & bit != 0
    }

    pub fn any_wheel_contact(&self) -> bool {
        self.wheels.iter().any(|w| w.contact != 0)
    }

    pub fn maximum_level(&self) -> u32 {
        let forced = self.forced_effect_flags >> 4 & 3;
        let current = self.boost_maximum_level;
        if current < forced { forced } else { current }
    }

    pub fn level(&self) -> u32 {
        let flags = self.forced_effect_flags;
        if flags & 0x30 == 0 { self.boost_level } else { (flags & 0x40 != 0) as u32 + 1 }
    }

    pub fn level_consistent(&self) -> bool {
        self.maximum_level() != 2 || self.level() == 2
    }

    pub fn level_power(&self, tuning: &Tuning) -> f32 {
        let exponent = (self.forced_effect_flags >> 7 & 3) as f32;
        let power = crate::math::fast_exp2(crate::math::fast_log2(tuning.level_power_base) * exponent)
            * self.level_power_scale;
        if power >= tuning.level_power_minimum { minss(power, 1.0) } else { tuning.level_power_minimum }
    }

    pub fn steer_angle(&self, tuning: &Tuning, velocity: Vec3) -> f32 {
        let k = velocity.z.abs() * tuning.steering_angle_speed_scale + tuning.steering_angle_constant;
        let angle = if EPS > k { 0.0 } else { crate::math::asin(1.0 / k) };
        if self.has_effect_flag(8) { 0.0 } else { -self.steering * angle }
    }

    pub fn update_grip_meter(&mut self, recovery_ms: u32, minimum: f32, dt: f32) {
        let q = self.quat;
        let v = crate::Quat::new(q.w, -q.x, -q.y, -q.z).rotate(self.vel);
        let Vec3 { mut x, mut y, mut z } = v;
        let sq = y * y + x * x + z * z;
        if sq > TINY && f32::MAX > sq {
            let inv = 1.0 / sq.sqrt();
            (x, y, z) = (x * inv, y * inv, z * inv);
        }
        let rate = dt / recovery_ms as f32;
        let r = if self.controls.gas > PEDAL_THRESHOLD {
            let d = (y * 0.0 + x * 0.0 + z).abs() - 0.5;
            d + d
        } else {
            rate + self.grip_meter
        };
        self.grip_meter = if r >= minimum { minss(r, 1.0) } else { minimum };
    }
}

impl Tuning {
    pub fn slope_factors(&self, direction: Vec3) -> (f32, f32) {
        let Vec3 { x, y, z } = direction;
        let sq = x * x + y * y + z * z;
        if TINY >= sq {
            return (1.0, 1.0);
        }
        let n = (y / sq.sqrt()).abs();
        let ease = |lo: f32, hi: f32| 1.0 - sincos((n - lo) / (hi - lo) * std::f32::consts::PI * 0.5).1;
        let lo = self.grip_slope_lower;
        let hi = self.grip_slope_upper;
        let grip = if lo > n {
            0.0
        } else if n > hi {
            1.0
        } else {
            ease(lo, hi)
        };
        let lo = self.thrust_slope_lower;
        let hi = self.thrust_slope_upper;
        let thrust = if lo > n {
            0.0
        } else if !(n > hi) {
            ease(lo, hi)
        } else {
            1.0
        };
        (grip, thrust)
    }
}
