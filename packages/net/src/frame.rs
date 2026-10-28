use crate::{Pose, Voxels};
use gearbox_engine::Session;

pub const LEN: usize = 8;

pub fn read(session: &Session, grid: &Voxels) -> [f32; LEN] {
    at(grid, Pose::read(session))
}

pub fn at<B: AsRef<[u8]>>(grid: &Voxels<B>, pose: Pose) -> [f32; LEN] {
    let (origin, extent) = grid.bounds();
    coordinates(pose, origin, extent)
}

fn coordinates(pose: Pose, origin: [f32; 3], extent: [f32; 3]) -> [f32; LEN] {
    let relative: [f32; 3] = std::array::from_fn(|i| (pose.position[i] - origin[i]) / extent[i]);
    let (sin, cos) = pose.yaw.sin_cos();
    [
        relative[0],
        relative[1],
        relative[2],
        sin,
        cos,
        extent[0].ln_1p() / 10.0,
        extent[1].ln_1p() / 10.0,
        extent[2].ln_1p() / 10.0,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_coordinates_retain_scale_heading_and_outside_positions() {
        let pose = Pose { position: [140.0, 10.0, 30.0], yaw: 0.0 };
        let a = coordinates(pose, [100.0, 0.0, -50.0], [80.0, 20.0, 160.0]);
        let b = coordinates(
            Pose { position: pose.position.map(|x| x * 2.0), yaw: std::f32::consts::FRAC_PI_2 },
            [200.0, 0.0, -100.0],
            [160.0, 40.0, 320.0],
        );
        assert_eq!(&a[..3], &[0.5, 0.5, 0.5]);
        assert_eq!(&a[..3], &b[..3]);
        assert_eq!(&a[3..5], &[0.0, 1.0]);
        assert!((b[3] - 1.0).abs() < 1e-7 && b[4].abs() < 1e-7);
        assert!((5..8).all(|i| b[i] > a[i]));
        let outside = coordinates(pose, [200.0, 0.0, 0.0], [10.0; 3]);
        assert_eq!(&outside[..3], &[-6.0, 1.0, 3.0]);
    }
}
