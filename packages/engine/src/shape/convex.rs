use crate::{Vec3, static_world::{Convex as Hull, Face}};
use super::Geometry;

pub struct Convex<'a> {
    pub vertices: &'a [Vec3],
    pub faces: &'a [Face],
    indices: &'a [u32],
}

impl<'a> Geometry<'a> {
    pub fn convex(self, shape: &'a Hull) -> Convex<'a> {
        Convex { vertices: &shape.vertices, faces: &shape.faces, indices: &shape.indices }
    }
}

impl Convex<'_> {
    pub fn indices(&self, face: &Face) -> &[u32] {
        &self.indices[face.start as usize..(face.start + face.count) as usize]
    }
}
