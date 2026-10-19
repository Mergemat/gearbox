use crate::{EntryId, Iso, SurfaceId, Vec3};
use crate::shape::{Geometry, Mesh, Shape};
use crate::tree::{Segment, SegmentQuery};
use super::{Lists, CollisionWorld, QueryFilter, QueryResults, Surface, QUERY_CAPACITY};

const SAFE_DIV: f32 = 3.4028233e32;
const BARY_EPS: f32 = 100_000.0;
pub const FAR: f32 = 1.00001;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ray { pub origin: Vec3, pub direction: Vec3 }

#[derive(Clone, Debug, PartialEq)]
pub struct RayHit {
    pub fraction: f32,
    pub point: Vec3,
    pub normal: Vec3,
    pub material: u16,
    pub entry: EntryId,
    pub triangle: u32,
}

fn safe_div(a: f32, b: f32) -> Option<f32> {
    let ab = b.abs();
    if 1.0 > ab && !(a.abs() < ab * SAFE_DIV) { return None; }
    Some(a / b)
}

fn bary(e1a: f32, e2a: f32, pa: f32, e1b: f32, e2b: f32, pb: f32) -> Option<(f32, f32)> {
    let det = e1a * e2b - e2a * e1b;
    let x8 = pa * e2b - e2a * pb;
    let x2 = e1a * pb - pa * e1b;
    let lim = det.abs() * BARY_EPS;
    if !(lim > x8.abs()) || !(lim > x2.abs()) { return None; }
    let inv = 1.0 / det;
    Some((inv * x8, inv * x2))
}

pub fn triangle(mesh: Mesh<'_>, ray: Ray, idx: u32, two_sided: bool, max: f32) -> Option<f32> {
    let tri = &mesh.cooked[idx as usize];
    let (normal, origin, direction) = (tri.normal, ray.origin, ray.direction);
    let plane_distance = normal.y * origin.y + normal.x * origin.x + normal.z * origin.z + tri.offset;
    if two_sided {
        todo!("two-sided ray vs triangle");
    }
    if 0.0 > plane_distance { return None; }
    let approach = -(normal.x * direction.x + normal.y * direction.y + normal.z * direction.z);
    if plane_distance > approach { return None; }
    let fraction = safe_div(plane_distance, approach)?;
    if fraction > max { return None; }
    let [v0, v1, v2] = tri.verts.map(|i| mesh.vertices[i as usize]);
    let projected = direction * fraction + origin - v0;
    let (edge1, edge2) = (v1 - v0, v2 - v0);
    let absolute_normal = normal.abs();
    let mx = if absolute_normal.y > absolute_normal.x { absolute_normal.y } else { absolute_normal.x };
    let mut axis = usize::from(absolute_normal.y > absolute_normal.x);
    if absolute_normal.z > mx { axis = 2; }
    let (first_axis, second_axis) = ((axis + 1) % 3, (axis + 2) % 3);
    let (u, w) = bary(
        edge1[first_axis], edge2[first_axis], projected[first_axis],
        edge1[second_axis], edge2[second_axis], projected[second_axis],
    )?;
    if !(u >= 0.0) || !(w >= 0.0) || !(1.0 >= u + w) { return None; }
    Some(fraction)
}

pub fn mesh(
    mesh: Mesh<'_>, entry: EntryId, iso: Iso, ray: Ray, offset: Vec3, filter: QueryFilter,
    hit: &mut Option<RayHit>,
) {
    let rotation = iso.rot;
    let (origin, direction) = (ray.origin - iso.pos, ray.direction - offset);
    let local = Ray {
        origin: Vec3::new(
            rotation.rows[0].x * origin.x + origin.y * rotation.rows[1].x + rotation.rows[2].x * origin.z,
            origin.x * rotation.rows[0].y + origin.y * rotation.rows[1].y + origin.z * rotation.rows[2].y,
            origin.x * rotation.rows[0].z + origin.y * rotation.rows[1].z + origin.z * rotation.rows[2].z,
        ),
        direction: Vec3::new(
            direction.y * rotation.rows[1].x + rotation.rows[0].x * direction.x
                + rotation.rows[2].x * direction.z,
            direction.x * rotation.rows[0].y + direction.y * rotation.rows[1].y
                + direction.z * rotation.rows[2].y,
            rotation.rows[0].z * direction.x + direction.y * rotation.rows[1].z
                + rotation.rows[2].z * direction.z,
        ),
    };
    for idx in SegmentQuery::new(mesh.nodes, Segment::new(local.origin, local.direction)) {
        let tri = &mesh.cooked[idx as usize];
        if filter.skip_material.skips(tri.ids as u8) { continue; }
        let limit = hit.as_ref().map_or(FAR, |hit| hit.fraction);
        let Some(fraction) = triangle(mesh, local, idx, filter.two_sided, limit) else { continue; };
        let normal = tri.normal;
        *hit = Some(RayHit {
            fraction, point: fraction * ray.direction + ray.origin,
            normal: Vec3::new(
                rotation.rows[0].y * normal.y + rotation.rows[0].x * normal.x + rotation.rows[0].z * normal.z,
                rotation.rows[1].y * normal.y + rotation.rows[1].x * normal.x + rotation.rows[1].z * normal.z,
                rotation.rows[2].y * normal.y + rotation.rows[2].x * normal.x + rotation.rows[2].z * normal.z,
            ),
            material: tri.ids, entry, triangle: idx,
        });
    }
}

impl CollisionWorld {
    pub fn ray_candidates(&self, geometry: Geometry<'_>, ray: Ray, filter: QueryFilter) -> QueryResults {
        let segment = Segment::new(ray.origin, ray.direction);
        let mut out = QueryResults::default();
        if filter.lists.contains(Lists::SURFACES) {
            self.collect_warp(geometry, segment.bounds(), filter, &mut out.surfaces);
            for (range, nodes) in self.surface_trees(geometry) {
                let start = range.start;
                let count = range.end - range.start;
                for id in SegmentQuery::new(nodes, segment) {
                    assert!(id < count);
                    let r = EntryId::Surface(SurfaceId(start + id));
                    if filter.accepts(geometry, geometry.entry(r).item) {
                        if out.surfaces.len() >= QUERY_CAPACITY { break; }
                        out.surfaces.push(r);
                    }
                }
            }
        }
        if filter.lists.contains(Lists::CONTINUOUS) && !self.continuous_entries.is_empty() {
            todo!("ray vs continuous entries");
        }
        if filter.lists.contains(Lists::DISCRETE) {
            for (r, e) in geometry.placed_entries(self.shape_entries.clone()) {
                if filter.accepts(geometry, e.item) && segment.crosses(e.bounds) {
                    if out.discrete.len() >= QUERY_CAPACITY { break; }
                    out.discrete.push(r);
                }
            }
        }
        out
    }

    pub fn raycast(&self, geometry: Geometry<'_>, ray: Ray, filter: QueryFilter) -> Option<RayHit> {
        let candidates = self.ray_candidates(geometry, ray, filter);
        let mut hit = None;
        if !candidates.is_empty() && self.post_filter != 0 {
            todo!("query post-filter");
        }
        for r in candidates.iter() {
            let Surface { shape, iso, .. } = geometry.entry(r);
            match geometry.shape(shape) {
                Shape::Mesh(m) => mesh(geometry.mesh(m), r, iso, ray, Vec3::ZERO, filter, &mut hit),
                _ => todo!("ray narrowphase for non-mesh shape"),
            }
        }
        hit
    }
}
