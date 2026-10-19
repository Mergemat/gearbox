use super::{Iso, Quat, Vec3};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Aabb {
    pub center: Vec3,
    pub half: Vec3,
}

impl Aabb {
    pub const EMPTY: Self = Self::new(Vec3::ZERO, Vec3::new(-1.0, -1.0, -1.0));

    #[inline]
    pub const fn new(center: Vec3, half: Vec3) -> Self {
        Self { center, half }
    }

    pub fn transform(&self, t: &Iso) -> Aabb {
        Aabb::new(t.transform_point(self.center), t.rot.abs() * self.half)
    }

    pub fn transform_quat(&self, q: Quat, pos: Vec3) -> Aabb {
        Aabb::new(pos + q.rotate(self.center), q.to_mat3().abs() * self.half)
    }

    pub fn corners(&self) -> [Vec3; 8] {
        let (c, h) = (self.center, self.half);
        let (xs, ys, zs) = ([c.x - h.x, h.x + c.x], [c.y - h.y, h.y + c.y], [c.z - h.z, h.z + c.z]);
        std::array::from_fn(|i| Vec3::new(xs[i & 1], ys[i >> 1 & 1], zs[i >> 2]))
    }

    pub fn corners_transformed(&self, t: &Iso) -> [Vec3; 8] {
        self.corners().map(|p| t.transform_point(p))
    }
}
