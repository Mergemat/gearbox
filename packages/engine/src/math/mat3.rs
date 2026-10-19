use super::{Quat, Vec3};
use std::ops::Mul;

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Mat3 {
    pub rows: [Vec3; 3],
}

impl Default for Mat3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3::from_array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);

    #[inline]
    pub const fn from_rows(r0: Vec3, r1: Vec3, r2: Vec3) -> Self {
        Self { rows: [r0, r1, r2] }
    }

    #[inline]
    pub const fn from_array(a: [f32; 9]) -> Self {
        Self::from_rows(Vec3::new(a[0], a[1], a[2]), Vec3::new(a[3], a[4], a[5]), Vec3::new(a[6], a[7], a[8]))
    }

    #[inline]
    pub const fn to_array(self) -> [f32; 9] {
        let [a, b, c] = self.rows;
        [a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z]
    }

    #[inline]
    pub const fn diagonal(d: Vec3) -> Self {
        Self::from_rows(Vec3::new(d.x, 0.0, 0.0), Vec3::new(0.0, d.y, 0.0), Vec3::new(0.0, 0.0, d.z))
    }

    #[inline]
    pub fn col(&self, c: usize) -> Vec3 {
        Vec3::new(self.rows[0][c], self.rows[1][c], self.rows[2][c])
    }

    #[inline]
    pub fn transpose(&self) -> Mat3 {
        Mat3::from_rows(self.col(0), self.col(1), self.col(2))
    }

    #[inline]
    pub fn abs(&self) -> Mat3 {
        Mat3::from_rows(self.rows[0].abs(), self.rows[1].abs(), self.rows[2].abs())
    }

    #[inline]
    pub fn transpose_mul_vec(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.col(0).dot(v), self.col(1).dot(v), self.col(2).dot(v))
    }

    pub fn transpose_mul(&self, o: &Mat3) -> Mat3 {
        let (c0, c1, c2) = (o.col(0), o.col(1), o.col(2));
        let row = |r: usize| {
            let b = self.col(r);
            Vec3::new(b.dot(c0), b.dot(c1), b.dot(c2))
        };
        Mat3::from_rows(row(0), row(1), row(2))
    }

    #[inline]
    pub fn transform_y_first(&self, v: Vec3) -> Vec3 {
        let [a, b, c] = self.rows;
        Vec3::new(v.dot_yxz(a), v.dot(b), v.dot(c))
    }

    pub fn from_quat(q: Quat) -> Mat3 {
        let Quat { w, x, y, z } = q;
        let z2 = z + z;
        let y2 = y + y;
        let wx2 = w * (x + x);
        let one_xx = 1.0 - x * (x + x);
        Mat3::from_array([
            (1.0 - y * y2) - z * z2,
            x * y2 - w * z2,
            x * z2 + w * y2,
            x * y2 + w * z2,
            one_xx - z * z2,
            y * z2 - wx2,
            x * z2 - w * y2,
            y * z2 + wx2,
            one_xx - y * y2,
        ])
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, v: Vec3) -> Vec3 {
        Vec3::new(self.rows[0].dot(v), self.rows[1].dot(v), self.rows[2].dot(v))
    }
}

impl Mul for Mat3 {
    type Output = Mat3;
    fn mul(self, b: Mat3) -> Mat3 {
        let (c0, c1, c2) = (b.col(0), b.col(1), b.col(2));
        let row = |r: Vec3| Vec3::new(r.dot(c0), r.dot(c1), r.dot(c2));
        Mat3::from_rows(row(self.rows[0]), row(self.rows[1]), row(self.rows[2]))
    }
}
