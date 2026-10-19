use crate::{Quat, Vec3, math::{maxss, sincos}, vehicle::Vehicle};
use super::{Player, PlayerController};

const INPUT_EPSILON: f32 = 0.00001;
const SEED_FRACTION: f32 = 1.1920929e-7;
const PERTURBATION_BIAS: f32 = 0.1;
const PERTURBATION_SCALE: f32 = 1.1;
const DISPLACEMENT_SCALE: f32 = 0.025;
const YAW_SCALE: f32 = 0.0015;
const MIN_SPEED_SQUARED: f32 = 1.0e-10;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputBits(pub u32);
impl InputBits {
    pub const REPLAY: Self = Self(2);
    pub fn packed(self, buttons: u32) -> u64 {
        let bits = self.0;
        let signed = (((bits << 21) as i32) >> 28) as u32;
        let low = bits & 16 | (signed & 15) << 27 | (buttons & 0x3f_ffff) << 5
            | bits & 15 | (bits >> 5) << 31;
        let high = ((bits >> 12) & 1) * 2 | (bits >> 6) & 1;
        (high as u64) << 32 | low as u64
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InputState {
    pub bits: InputBits,
    pub axes: [f32; 4],
    pub shorts: [u16; 2],
    pub buttons: u32,
}
impl InputState {
    pub fn same(&self, other: &Self) -> bool {
        self.bits.packed(self.buttons) == other.bits.packed(other.buttons)
            && self.shorts == other.shorts
            && self.axes.iter().zip(other.axes).all(|(&left, right)| {
                let limit = maxss(1.0, maxss(left.abs(), right.abs())) * INPUT_EPSILON;
                limit >= (left - right).abs()
            })
    }
}

impl Player {
    pub fn input_history(&self) -> [InputState; 2] {
        [self.previous_input, self.current_input]
    }

    pub fn replay_inputs(&mut self, steer: f32, gas: f32, brake: f32, seed: u32) {
        self.replay_seed = seed;
        self.previous_input = InputState { bits: InputBits::REPLAY,
            axes: [self.current_input.axes[0], self.current_input.axes[1], self.current_input.axes[2], 0.0],
            ..Default::default() };
        self.current_input = InputState { bits: InputBits::REPLAY, axes: [steer, gas, brake, 0.0],
            ..Default::default() };
    }
}

pub fn start_adjustment(player: &Player, vehicle: &mut Vehicle, controller: &mut PlayerController)
    -> Option<u32> {
    let speed = vehicle.vel;
    let squared = speed.x * speed.x + speed.y * speed.y + speed.z * speed.z;
    if !(squared.sqrt() > 1.0) || !vehicle.wheels.iter().any(|wheel| wheel.contact != 0) { return None; }
    let count = controller.start_adjustments;
    let next = (count.wrapping_add(1) ^ count) & 3 ^ count;
    controller.start_adjustments = next;
    if player.replay_seed == 0 { todo!("unseeded start adjustment"); }
    let seed = ((next as u32 & 3).wrapping_mul(0xaa_aaaa).wrapping_sub(0xaa_aaaa)) ^ player.replay_seed;
    let sign = |bit: u32| if seed & (1u32 << bit) != 0 { 1.0 } else { -1.0 };
    let distance = ((((seed % 4999) as f32 / 4999.0 + (seed & 0x7f_ffff) as f32 * SEED_FRACTION)
        * 0.5 + PERTURBATION_BIAS) * sign(12)) / PERTURBATION_SCALE * DISPLACEMENT_SCALE;
    let angle = (((seed % 47777) as f32 / 47777.0 + PERTURBATION_BIAS) * sign(20))
        / PERTURBATION_SCALE * YAW_SCALE;
    let Quat { w, x, y, z } = vehicle.quat;
    let right_w = (z * 0.0 - x * 0.0) + w;
    let right_x = (x - y * 0.0) + w * 0.0;
    let right_y = (y * 0.0 - z) + w * 0.0;
    let right = Vec3::new((z * right_y - right_x * x) * 2.0 + 1.0,
        (right_x * y - right_w * z) * 2.0 + 0.0, (right_w * x - y * right_y) * 2.0 + 0.0);
    vehicle.quat = rotate_y(vehicle.quat, angle);
    let speed = if squared > MIN_SPEED_SQUARED && f32::MAX > squared {
        speed * (1.0 / squared.sqrt())
    } else { speed };
    let delta = Vec3::new(speed.z * right.x - speed.y * right.z,
        speed.x * right.z - speed.z * right.y, speed.y * right.y - speed.x * right.x);
    vehicle.pos = Vec3::new(delta.x * distance + vehicle.pos.x,
        delta.y * distance + vehicle.pos.y, delta.z * distance + vehicle.pos.z);
    Some(player.replay_seed)
}

pub fn rotate_y(quat: Quat, angle: f32) -> Quat {
    let (sine, cosine) = sincos(angle * 0.5);
    let Quat { w, x, y, z } = quat;
    Quat::new(w * cosine - ((sine * y + x * 0.0) + 0.0 * z),
        ((x * cosine + w * 0.0) + z * sine) - y * 0.0,
        ((y * cosine + w * sine) + x * 0.0) - z * 0.0,
        ((z * cosine + w * 0.0) + y * 0.0) - x * sine)
}
