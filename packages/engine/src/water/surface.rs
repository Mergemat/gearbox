use crate::Vec3;
use super::drag::NORMAL_MIN_LENGTH_SQUARED;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Plane {
    pub normal: Vec3,
    pub offset: f32,
}

impl Plane {
    fn through(point: Vec3, first: Vec3, second: Vec3) -> Self {
        let mut normal = Vec3::new(
            second.y * first.z - second.z * first.y,
            first.x * second.z - second.x * first.z,
            second.x * first.y - first.x * second.y,
        );
        let length_squared = normal.dot_yxz(normal);
        if length_squared > NORMAL_MIN_LENGTH_SQUARED && f32::MAX > length_squared {
            normal *= 1.0 / length_squared.sqrt();
        }
        Self { normal, offset: -normal.dot_yxz(point) }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceGrid {
    pub planes: Vec<Plane>,
    pub origin: [f32; 2],
    pub size: [u32; 2],
    pub inverse_cell: [f32; 2],
    pub cell: [f32; 2],
}

impl SurfaceGrid {
    pub fn flat(center: [f32; 2], half: [f32; 2], size: [u32; 2], level: f32) -> Self {
        let [nx, nz] = size;
        assert!(nx > 0 && nz > 0, "water surface needs nonempty cells");
        let origin = [center[0] - half[0], center[1] - half[1]];
        let cell = [(half[0] * 2.0) / nx as f32, (half[1] * 2.0) / nz as f32];
        let columns = nx + 1;
        let point = |index: u32| {
            Vec3::new((index % columns) as f32 * cell[0] + origin[0], level + 0.0,
                (index / columns) as f32 * cell[1] + origin[1])
        };
        let mut planes = Vec::with_capacity((nx * nz * 2) as usize);
        for row in 0..nz {
            let mut first = row * nx;
            for _ in 0..nx {
                let second = first + 1;
                let third = nx + second;
                let fourth = second + columns;
                let [a, b, c, d] = [point(first), point(second), point(third), point(fourth)];
                planes.push(Plane::through(a, b - a, c - a));
                planes.push(Plane::through(b, d - b, c - b));
                first = second;
            }
        }
        Self { planes, origin, size, inverse_cell: [1.0 / cell[0], 1.0 / cell[1]], cell }
    }
}
