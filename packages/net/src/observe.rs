use anyhow::{Context, Result};
use gearbox_engine::Session;
use gearbox_search::Body;
use gearbox_search::gate::{Gate, gates, target_gate};
use gearbox_search::tricks::Observation;

pub const VERSION: u32 = 1;
pub const AHEAD: usize = 3;
const PER_TARGET: usize = 5;
pub const LEN: usize = 3 + 3 + 3 + 4 + 2 + 3 + AHEAD * PER_TARGET + 1;

#[derive(Clone, Debug, PartialEq)]
pub struct Course {
    pub targets: Vec<[f32; 3]>,
}

impl Course {
    pub fn race(native: &mut Session) -> Result<Self> {
        let mut targets = Vec::new();
        for ordinal in 1.. {
            let Ok(gate) = target_gate(native, ordinal) else { break };
            targets.push(center(&gate));
            if gate.kind == 1 {
                break;
            }
        }
        anyhow::ensure!(!targets.is_empty(), "the map has no checkpoints or finish in a fixed order");
        Ok(Self { targets })
    }

    pub fn through(native: &mut Session, waypoints: &[u32]) -> Result<Self> {
        let all = gates(native)?;
        let targets = waypoints
            .iter()
            .map(|&w| {
                all.iter().find(|g| g.waypoint == w).map(center).with_context(|| format!("no waypoint {w}"))
            })
            .collect::<Result<_>>()?;
        Ok(Self { targets })
    }
}

fn center(gate: &Gate) -> [f32; 3] {
    let b = gate.boxes[0];
    std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Car {
    pub rotation: [f32; 9],
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub wheels: [bool; 4],
    pub body_contact: bool,
    pub wall_contact: bool,
    pub controls: [f32; 3],
}

impl Car {
    pub fn read(native: &mut Session) -> Self {
        let body = Body::from(native.body());
        let seen = Observation::read(native, body);
        let c = &native.vehicle().controls;
        Self {
            rotation: body.rotation(),
            position: body.position(),
            velocity: body.local_velocity(),
            angular_velocity: body.local_angular_velocity(),
            wheels: seen.wheels,
            body_contact: seen.body_contact,
            wall_contact: seen.wall_contact,
            controls: [c.gas, c.brake, c.steer],
        }
    }

    pub fn to_car(&self, world: [f32; 3]) -> [f32; 3] {
        let r = &self.rotation;
        std::array::from_fn(|k| r[k] * world[0] + r[3 + k] * world[1] + r[6 + k] * world[2])
    }

    pub fn features(&self, ahead: &[[f32; 3]], left: usize) -> [f32; LEN] {
        let mut out = [0.0; LEN];
        let mut at = 0;
        let mut put = |values: &[f32]| {
            out[at..at + values.len()].copy_from_slice(values);
            at += values.len();
        };
        put(&self.to_car(self.velocity).map(|v| v / 100.0));
        put(&self.to_car(self.angular_velocity).map(|w| w / 10.0));
        put(&self.to_car([0.0, 1.0, 0.0]));
        put(&self.wheels.map(f32::from));
        put(&[f32::from(self.body_contact), f32::from(self.wall_contact)]);
        put(&self.controls);
        for i in 0..AHEAD {
            match ahead.get(i) {
                Some(target) => {
                    let offset = self.to_car(std::array::from_fn(|k| target[k] - self.position[k]));
                    let distance = offset.iter().map(|v| v * v).sum::<f32>().sqrt();
                    let direction = offset.map(|v| if distance > 1e-3 { v / distance } else { 0.0 });
                    put(&direction);
                    put(&[distance.ln_1p() / 5.0, 1.0]);
                }
                None => put(&[0.0; PER_TARGET]),
            }
        }
        put(&[left as f32 / 10.0]);
        debug_assert_eq!(at, LEN);
        out
    }
}

pub fn observe(native: &mut Session, course: &Course, next: usize) -> [f32; LEN] {
    let ahead = course.targets.get(next..).unwrap_or(&[]);
    Car::read(native).features(ahead, ahead.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn car_facing_world_x(velocity: [f32; 3], position: [f32; 3]) -> Car {
        Car {
            rotation: [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0],
            position,
            velocity,
            angular_velocity: [0.0; 3],
            wheels: [true; 4],
            body_contact: false,
            wall_contact: false,
            controls: [1.0, 0.0, 0.0],
        }
    }

    #[test]
    fn speed_along_the_nose_reads_as_forward_whatever_the_heading() {
        let f = car_facing_world_x([50.0, 0.0, 0.0], [0.0; 3]).features(&[], 0);
        assert_eq!(&f[0..3], &[0.0, 0.0, 0.5]);
        assert_eq!(&f[6..9], &[0.0, 1.0, 0.0]);
    }

    #[test]
    fn a_target_behind_the_car_points_backwards() {
        let car = car_facing_world_x([0.0; 3], [100.0, 10.0, 0.0]);
        let f = car.features(&[[0.0, 10.0, 0.0]], 1);
        let target = &f[18..23];
        assert_eq!(&target[0..3], &[0.0, 0.0, -1.0]);
        assert!((target[3] - 101f32.ln() / 5.0).abs() < 1e-6);
        assert_eq!(target[4], 1.0);
    }

    #[test]
    fn missing_targets_are_marked_absent() {
        let car = car_facing_world_x([0.0; 3], [0.0; 3]);
        let f = car.features(&[[10.0, 0.0, 0.0]], 1);
        assert_eq!(f[18 + 4], 1.0);
        assert_eq!(&f[23..33], &[0.0; 10]);
        assert_eq!(f[LEN - 1], 0.1);
    }
}
