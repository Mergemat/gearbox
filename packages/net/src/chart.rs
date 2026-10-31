use gearbox_engine::Session;
use gearbox_search::gate::gates;

use crate::ground::{self, Ground};
use crate::observe::{Car, hypot};

pub const SIDE: usize = 64;
pub const CHANNELS: usize = crate::atlas::CHANNELS;
pub const LEN: usize = SIDE * SIDE * CHANNELS;
pub const POSE_LEN: usize = 8;
pub const MIN_CELL: f32 = 8.0;
const MARGIN: f32 = 32.0;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Chart {
    pub center: [f32; 2],
    pub extent: f32,
    pub cell: f32,
    pub height: f32,
    pub image: Vec<f32>,
}

impl Chart {
    pub fn build(session: &Session, ground: &Ground) -> Self {
        let start = session.config.initial_pose.pos;
        let mut low = [start.x, start.z];
        let mut high = low;
        for bounds in gates(session).unwrap_or_default().iter().flat_map(|gate| &gate.boxes) {
            for (k, axis) in [0, 2].into_iter().enumerate() {
                low[k] = low[k].min(bounds.min[axis]);
                high[k] = high[k].max(bounds.max[axis]);
            }
        }
        let span = (high[0] - low[0]).max(high[1] - low[1]) + 2.0 * MARGIN;
        let factor =
            ((span / (SIDE as f32 * ground::CELL)).ceil() as usize).max((MIN_CELL / ground::CELL) as usize);
        let cell = factor as f32 * ground::CELL;
        let center = [(low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5];
        let mut chart =
            Self { center, extent: cell * SIDE as f32, cell, height: start.y, image: vec![0.0; LEN] };
        ground.atlas(session).chart(ground, &mut chart, factor);
        chart
    }

    pub fn pose(&self, car: &Car) -> [f32; POSE_LEN] {
        let forward = car.rotation * gearbox_engine::Vec3::new(0.0, 0.0, 1.0);
        let [fx, fz] = ground::unit([forward.x, forward.z]);
        [
            (car.position[0] - self.center[0]) / self.extent,
            (car.position[2] - self.center[1]) / self.extent,
            fx,
            fz,
            (car.position[1] - self.height) / 128.0,
            libm::log2f(self.cell / MIN_CELL) / 4.0,
            self.extent / 4096.0,
            1.0,
        ]
    }
}

pub fn relative(pose: &[f32], side: usize) -> Vec<f32> {
    let extent = pose[6] * 4096.0;
    let [fx, fz] = ground::unit([pose[2], pose[3]]);
    let mut out = Vec::with_capacity(side * side * 4);
    for row in 0..side {
        for col in 0..side {
            let x = ((col as f32 + 0.5) / side as f32 - 0.5 - pose[0]) * extent;
            let z = ((row as f32 + 0.5) / side as f32 - 0.5 - pose[1]) * extent;
            let across = x * fz - z * fx;
            let along = x * fx + z * fz;
            let distance = hypot(across, along);
            out.extend([
                across / 256.0,
                along / 256.0,
                distance / 256.0,
                libm::log1pf(distance / 16.0) / 4.0,
            ]);
        }
    }
    out
}
