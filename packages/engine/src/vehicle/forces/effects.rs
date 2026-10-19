use super::super::{Tuning, Vehicle, Wheel};
use super::common::KMH;
use crate::math::{maxss, minss};

const EFFECT_IDLE: u32 = 0;
const EFFECT_RISING: u32 = 1;
const EFFECT_FALLING: u32 = 2;

impl Wheel {
    pub fn on_material(&self, materials: &[u8]) -> bool {
        self.contact != 0 && materials.contains(&self.material)
    }

    pub fn has_boost_material(&self) -> bool {
        matches!(self.material, 3 | 0x4a | 0x15)
    }

    pub fn update_effect(&mut self, active: bool, now: u32, tuning: &Tuning, mut cap: f32, reset: bool) {
        let previous_mode = self.effect_mode;
        if cap < self.effect_floor {
            self.effect_floor = cap;
        }
        if reset {
            self.effect_floor = 0.0;
        }
        let contact = self.contact;
        let material = self.material;
        let boosting = contact != 0
            && (material == 3
                || material == 0x4a
                || (material == 0x15 && tuning.special_material_enabled != 0));
        if previous_mode == EFFECT_RISING {
            if !active {
                self.effect_mode = EFFECT_IDLE;
            } else if contact == 0 || !matches!(material, 3 | 0x4a | 0x15) {
                self.effect_mode = EFFECT_FALLING;
            }
        } else if previous_mode == EFFECT_FALLING {
            if !active {
                self.effect_mode = EFFECT_IDLE;
            } else if boosting {
                self.effect_mode = EFFECT_RISING;
            }
        } else if active {
            self.effect_mode = EFFECT_FALLING - boosting as u32;
        }
        if previous_mode == self.effect_mode && contact == self.previous_contact {
            return;
        }
        let elapsed = now.wrapping_sub(self.effect_start) as f32;
        if previous_mode == EFFECT_RISING {
            let x = elapsed / tuning.effect_rise_ms as f32 + self.effect_level;
            let coefficient = minss(1.0, x);
            cap = minss(coefficient, cap);
            self.effect_floor = cap;
            self.effect_level = coefficient;
        } else if previous_mode == EFFECT_FALLING {
            let duration_ms = if self.previous_contact == 0 {
                tuning.effect_air_fall_ms
            } else {
                tuning.effect_ground_fall_ms
            };
            let x = self.effect_level - elapsed / duration_ms as f32;
            self.effect_level = maxss(self.effect_floor, x);
        } else if previous_mode == EFFECT_IDLE {
            let x = self.effect_level - elapsed / tuning.effect_idle_fall_ms as f32;
            self.effect_level = maxss(self.effect_floor, x);
        }
        if self.effect_mode == EFFECT_RISING || self.effect_level != 0.0 {
            self.effect_start = now;
        }
    }

    pub fn effect_coefficient(&self, tuning: &Tuning, now: u32, with_base: bool) -> f32 {
        let elapsed = now.wrapping_sub(self.effect_start) as f32;
        let mut coefficient = 0.0;
        let mut base = 0.0;
        if self.effect_mode == EFFECT_IDLE {
            if self.effect_start != u32::MAX {
                coefficient =
                    maxss(self.effect_floor, self.effect_level - elapsed / tuning.effect_idle_fall_ms as f32);
            }
        } else if self.effect_mode == EFFECT_RISING {
            coefficient = minss(1.0, elapsed / tuning.effect_rise_ms as f32 + self.effect_level);
        } else if self.effect_mode == EFFECT_FALLING {
            let duration_ms =
                if self.contact != 0 { tuning.effect_ground_fall_ms } else { tuning.effect_air_fall_ms };
            coefficient = maxss(self.effect_floor, self.effect_level - elapsed / duration_ms as f32);
        }
        let minimum_grip = tuning.effect_minimum_grip;
        if with_base {
            let base_loss = 1.0 - tuning.effect_base_grip;
            base = (minimum_grip - base_loss) * coefficient + base_loss;
        }
        let response = (1.0 - (1.0 - minimum_grip) * coefficient) - base;
        maxss(response, tuning.effect_coefficient_minimum)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelEffectCoefficients {
    pub wheels: [f32; 4],
    pub contacting: f32,
    pub free: f32,
    pub all: f32,
    pub boost_material: bool,
}

impl Vehicle {
    pub fn advance_wheel_effects(&mut self, tuning: &Tuning, now: u32) {
        let crate::Vec3 { x, y, z } = self.vel;
        let speed = (x * x + y * y + z * z).sqrt() * KMH;
        let active = speed >= tuning.effect_activation_speed;
        let cap = self.immersion;
        let reset = self.in_water != 0;
        for wheel in &mut self.wheels {
            wheel.update_effect(active, now, tuning, cap, reset);
        }
    }

    pub fn pre_forces_effects(&mut self, tuning: &Tuning, now: u32, contacts: &[bool]) {
        self.advance_wheel_effects(tuning, now);
        let coefficient = self.wheel_effect_coefficients(tuning, now, contacts);
        self.wheel_grip = coefficient.wheels;
        self.front_grip = coefficient.contacting;
        self.rear_grip = coefficient.free;
        self.grip = coefficient.all;
        self.special_material_present = coefficient.boost_material as u32;
    }

    pub fn wheel_effect_coefficients(
        &self,
        tuning: &Tuning,
        now: u32,
        contacts: &[bool],
    ) -> WheelEffectCoefficients {
        assert!(contacts.len() <= self.wheels.len(), "at most four wheels");
        let mut coefficients = WheelEffectCoefficients::default();
        let with_base = self.effect_flags >> 4 & 1 != 0;
        let (mut contacting_count, mut free_count) = (0, 0);
        for (index, &contact) in contacts.iter().enumerate() {
            let wheel = &self.wheels[index];
            let coefficient = wheel.effect_coefficient(tuning, now, with_base);
            coefficients.wheels[index] = coefficient;
            coefficients.all += coefficient;
            if contact {
                contacting_count += 1;
                coefficients.contacting += coefficient;
            } else {
                free_count += 1;
                coefficients.free += coefficient;
            }
            if wheel.has_boost_material() {
                coefficients.boost_material = true;
            }
        }
        if contacting_count != 0 {
            coefficients.contacting /= contacting_count as f32;
        }
        if free_count != 0 {
            coefficients.free /= free_count as f32;
        }
        if contacting_count + free_count != 0 {
            coefficients.all /= (contacting_count + free_count) as f32;
        }
        coefficients
    }
}
