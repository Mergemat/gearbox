use super::{Mat3, Quat, Vec3};
use std::ops::Mul;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Iso {
    pub rot: Mat3,
    pub pos: Vec3,
}

impl Iso {
    pub const IDENTITY: Iso = Iso { rot: Mat3::IDENTITY, pos: Vec3::ZERO };

    #[inline]
    pub const fn new(rot: Mat3, pos: Vec3) -> Self {
        Self { rot, pos }
    }

    pub fn from_array(a: [f32; 12]) -> Self {
        Self::new(Mat3::from_array(a[..9].try_into().unwrap()), Vec3::new(a[9], a[10], a[11]))
    }

    pub fn to_array(self) -> [f32; 12] {
        let mut o = [0f32; 12];
        o[..9].copy_from_slice(&self.rot.to_array());
        o[9..].copy_from_slice(&self.pos.to_array());
        o
    }

    #[inline]
    pub fn from_quat_pos(q: Quat, pos: Vec3) -> Self {
        Self::new(Mat3::from_quat(q), pos)
    }

    #[inline]
    pub const fn diagonal(diag: Vec3, pos: Vec3) -> Self {
        Self::new(Mat3::diagonal(diag), pos)
    }

    #[inline]
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        self.rot * p + self.pos
    }

    #[inline]
    pub fn transform_y_first(&self, p: Vec3) -> Vec3 {
        self.rot.transform_y_first(p) + self.pos
    }

    #[inline]
    pub fn inverse_transform_point(&self, p: Vec3) -> Vec3 {
        self.rot.transpose_mul_vec(p - self.pos)
    }

    pub fn inverse(&self) -> Iso {
        let rot = self.rot.transpose();
        Iso::new(rot, rot * -self.pos)
    }

    pub fn inv_mul(&self, o: &Iso) -> Iso {
        let pos = self.rot.transpose_mul_vec(o.pos) + self.rot.transpose_mul_vec(-self.pos);
        Iso::new(self.rot.transpose_mul(&o.rot), pos)
    }
}

impl Mul for Iso {
    type Output = Iso;
    #[inline]
    fn mul(self, a: Iso) -> Iso {
        Iso::new(self.rot * a.rot, self.transform_point(a.pos))
    }
}

impl Mul<Vec3> for Iso {
    type Output = Vec3;
    #[inline]
    fn mul(self, p: Vec3) -> Vec3 {
        self.transform_point(p)
    }
}
