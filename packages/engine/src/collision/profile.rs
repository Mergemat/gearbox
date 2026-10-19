use crate::{
    Vec3,
    math::{cvtt, sincos},
    numeric::floorf,
};

const TINY: f32 = 1.0e-10;
const FLOOR_BLEND: f32 = 0.1;

fn normalise(v: Vec3, square: f32) -> Vec3 {
    if square > TINY && f32::MAX > square { v * (1.0 / square.sqrt()) } else { v }
}

#[derive(Clone, Copy, Debug)]
pub struct PipeProfile {
    pub radius: f32,
    pub height: f32,
    pub inner: f32,
    pub outer: f32,
    pub tilt: f32,
}

impl PipeProfile {
    pub fn contact_normal(
        self,
        point: Vec3,
        axis_start: Vec3,
        axis_end: Vec3,
        normal: &mut Vec3,
        velocity: &mut Vec3,
    ) -> u32 {
        let old = *normal;
        let r1 = self.height * self.inner;
        let r2 = self.height * self.outer;
        let (tilt_sine, _) = sincos(self.tilt);
        let delta = axis_end - axis_start;
        let floor = if r1 > delta.x.abs() {
            let n = normalise(Vec3::new(0.0, delta.y, delta.z), delta.y * delta.y + 0.0 + delta.z * delta.z);
            *normal = -n;
            !(normal.y > tilt_sine)
        } else if r2 > delta.x.abs() {
            let n = normalise(Vec3::new(0.0, delta.y, delta.z), delta.y * delta.y + 0.0 + delta.z * delta.z);
            let square = 1.0 - r2 * r2 / (self.height * self.height);
            if 0.0 > square {
                todo!("sqrt of axis_start negative");
            }
            let t = square.sqrt() * self.radius;
            let mut y = n.y * t;
            let z = n.z * t;
            let sign = if delta.x.to_bits() >> 31 == 0 { 1.0 } else { -1.0 };
            if delta.y > 0.0 {
                y = delta.y;
            }
            let n = Vec3::new(sign * r1 - delta.x, y - delta.y, z - delta.z);
            *normal = normalise(n, n.y * n.y + n.x * n.x + n.z * n.z);
            !(normal.y > tilt_sine)
        } else {
            true
        };
        if floor {
            let n = Vec3::new(point.x - axis_end.x, 0.0, (point.z - axis_end.z) * FLOOR_BLEND);
            *normal = normalise(n, n.x * n.x + 0.0 + n.z * n.z);
        }
        let n = *normal;
        let alignment = n.x * old.x + old.y * n.y + n.z * old.z;
        if FLOOR_BLEND > alignment {
            *normal = old;
        } else {
            let vv = n.x * velocity.x + n.y * velocity.y + n.z * velocity.z;
            *velocity = Vec3::new(n.x * vv, vv * n.y, n.z * vv);
        }
        if floor { 3 } else { 1 }
    }
}

pub struct HeightField<'a> {
    pub planes: &'a [[f32; 4]],
    pub origin: [f32; 2],
    pub cells: [u32; 2],
    pub inverse_cell: [f32; 2],
    pub cell_size: [f32; 2],
}

impl HeightField<'_> {
    pub fn height(&self, point: Vec3) -> f32 {
        let fx = point.x - self.origin[0];
        let fz = point.z - self.origin[1];
        let i = cvtt(floorf(fx * self.inverse_cell[0]));
        let i = if i > 0 { i.min(self.cells[0].wrapping_sub(1) as i32) } else { 0 };
        let j = cvtt(floorf(fz * self.inverse_cell[1]));
        let j = if j > 0 { j.min(self.cells[1].wrapping_sub(1) as i32) } else { 0 };
        let lx = fx - i as u32 as f32 * self.cell_size[0];
        let lz = fz - j as u32 as f32 * self.cell_size[1];
        let tri = u32::from(!(lx > lz));
        let index =
            self.cells[0].wrapping_mul(j as u32).wrapping_add(i as u32).wrapping_mul(2).wrapping_add(tri);
        let [a, b, c, d] = self.planes[index as usize];
        point.y - (point.y * b + point.x * a + point.z * c + d) * b
    }
}
