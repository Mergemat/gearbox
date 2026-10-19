use super::Mesh;
use crate::{
    Aabb, Vec3,
    arrays::{CookedTriangle, Triangle},
    tree::{AabbTree, Leaf},
};

const MIN_NORMAL_SQUARED: f32 = 1.00000005e-20;

impl Mesh<'_> {
    pub fn cook(vertices: &[Vec3], triangles: &[Triangle]) -> Vec<CookedTriangle> {
        triangles
            .iter()
            .map(|triangle| {
                let [p0, p1, p2] = triangle.verts.map(|i| vertices[i as usize]);
                let (e1, e2) = (p1 - p0, p2 - p0);
                let mut normal = Vec3::new(
                    e1.y * e2.z - e1.z * e2.y,
                    e1.z * e2.x - e1.x * e2.z,
                    e1.x * e2.y - e1.y * e2.x,
                );
                let l2 = normal.y * normal.y + normal.x * normal.x + normal.z * normal.z;
                if l2 > MIN_NORMAL_SQUARED {
                    normal *= 1.0 / l2.sqrt();
                }
                CookedTriangle {
                    normal,
                    offset: -normal.x * p0.x - normal.y * p0.y - normal.z * p0.z,
                    verts: triangle.verts,
                    ids: triangle.surface as u16 | (triangle.gameplay as u16) << 8,
                }
            })
            .collect()
    }

    pub fn build_tree(vertices: &[Vec3], triangles: &[Triangle]) -> AabbTree {
        let mut leaves: Vec<_> = triangles
            .iter()
            .enumerate()
            .map(|(i, triangle)| {
                let [a, b, c] = triangle.verts.map(|i| vertices[i as usize]);
                let mut lo = Vec3::ZERO;
                let mut hi = Vec3::ZERO;
                for k in 0..3 {
                    lo[k] = crate::math::minss(c[k], crate::math::minss(b[k], a[k]));
                    hi[k] = crate::math::maxss(c[k], crate::math::maxss(b[k], a[k]));
                }
                Leaf { bounds: Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5), index: i as u32 }
            })
            .collect();
        AabbTree::build(&mut leaves, 2)
    }
}
