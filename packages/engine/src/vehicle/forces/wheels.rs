use crate::math::{maxss, minss};
use crate::Vec3;
use super::{ForceCtx, WheelDefinition};
use super::super::{Tuning, Vehicle};
use super::common::{EPS, KMH, sign_one};

const AIR_SPIN_DECAY: f32 = 0.995;
const GROUND_SPIN_RETENTION: f32 = 0.95;
const GRIP_LIMIT_EPSILON: f32 = 1.00001;

const PEDAL_SPIN_LIMIT: f32 = 200.0;
const AIR_SPIN_RATE: f32 = 100.0;
const GROUND_SPIN_RATE: f32 = 50.0;
const FIXED_SPIN_SPEED_LIMIT: f32 = 50.0;

impl Vehicle {
    pub fn update_wheel_spin(
        &mut self, tuning: &Tuning, definition: &WheelDefinition, index: usize,
        ground_speed: f32, dt: f32, now: u32,
    ) {
        let lock = self.effect_mode_is(super::common::PedalMode::Cut);
        let grip = if definition.steers {
            self.front_grip
        } else {
            self.rear_grip
        };
        let wheel = &mut self.wheels[index];
        if wheel.contact == 0 {
            airborne_spin(wheel, self.water_exit_until, self.controls.brake, self.controls.gas, lock,
                dt, now);
            return;
        }
        let special = wheel.has_boost_material();
        if self.fixed_wheel_spin != 0 || (special && self.controls.gas > EPS && !lock
            && FIXED_SPIN_SPEED_LIMIT > ground_speed * KMH) {
            wheel.spin = tuning.fixed_spin_speed;
            return;
        }
        let radius = definition.radius;
        let (brake, gas) = (self.controls.brake, self.controls.gas);
        let (mut push, mut target) = (0.0f32, 0.0f32);
        if brake > EPS {
            push = if wheel.spin > 0.0 {
                -PEDAL_SPIN_LIMIT
            } else {
                -GROUND_SPIN_RATE
            };
            target = brake * tuning.reverse_hold_speed / radius;
        } else if gas > EPS && !lock {
            push = if 0.0 > wheel.spin {
                PEDAL_SPIN_LIMIT
            } else {
                GROUND_SPIN_RATE
            };
            target = gas * tuning.forward_hold_speed / radius;
        }
        let rolling = ground_speed / radius;
        let loose = 1.0 - grip;
        if rolling.abs() > target.abs() && sign_one(push * loose) == sign_one(rolling) {
            target = rolling;
        }
        push = push * loose;
        if lock {
            wheel.spin = rolling;
            return;
        }
        let current = wheel.spin;
        let x = if EPS > push.abs() {
            (current - rolling) * GROUND_SPIN_RETENTION + rolling
        } else {
            let x = push * dt + current;
            if push > 0.0 {
                minss(x, target)
            } else {
                maxss(x, target)
            }
        };
        let k = maxss(1.0 - loose / (GRIP_LIMIT_EPSILON - tuning.low_grip_threshold), 0.0);
        wheel.spin = k * (rolling - x) + x;
    }
}

pub fn suspension_forces(ctx: &mut ForceCtx<'_>) {
    for index in 0..ctx.geometry.wheels.len() {
        suspension_force(ctx, index);
    }
}

pub fn suspension_force(ctx: &mut ForceCtx<'_>, index: usize) {
    let vehicle = &*ctx.vehicle;
    if vehicle.suspension_requires_all_contacts != 0
        && vehicle.wheels.iter().any(|wheel| wheel.contact == 0) {
        return;
    }
    let tuning = ctx.tuning;
    if tuning.suspension_kind != 2 {
        return;
    }
    let wheel = &vehicle.wheels[index];
    if wheel.contact == 0 {
        return;
    }
    let damping = tuning.suspension_damping * wheel.suspension_rate;
    let force = (tuning.suspension_rest_length - wheel.suspension_length)
        * tuning.suspension_stiffness - damping;
    ctx.body.add_local_force_at(Vec3::new(0.0, force, 0.0), ctx.geometry.wheels[index].anchor);
}

fn airborne_spin(
    wheel: &mut super::super::Wheel, until: u32, brake: f32, gas: f32, lock: bool, dt: f32, now: u32,
) {
    let (mut limit, mut rate) = (0.0f32, 0.0f32);
    if until != u32::MAX && until > now {
        (limit, rate) = (PEDAL_SPIN_LIMIT, AIR_SPIN_RATE);
    } else if brake > EPS && !lock {
        let x = 1.0 - brake;
        rate = -AIR_SPIN_RATE;
        limit = if x >= 0.0 {
            minss(x, 1.0)
        } else {
            0.0
        };
    } else if gas > EPS && !lock {
        (limit, rate) = (gas * PEDAL_SPIN_LIMIT, AIR_SPIN_RATE);
    } else {
        wheel.spin = wheel.spin * AIR_SPIN_DECAY;
    }
    if !(EPS > rate.abs()) {
        let x = rate * dt + wheel.spin;
        wheel.spin = x;
        if (rate > 0.0 && x > limit) || (0.0 > rate && limit > x) {
            wheel.spin = limit;
        }
    }

}
