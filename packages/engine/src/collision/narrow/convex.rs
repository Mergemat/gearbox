use super::super::{ContactSink, EllipsoidShape, Surface};
use crate::collision::ContactFlags;
use crate::shape::{Geometry, convex::Convex};
use crate::{EntryId, Iso, Mat3, Vec3, math::minss};

const TINY: f32 = 1.0e-10;
const MAX_EDGES: usize = 64;

pub(super) fn unit(v: Vec3) -> Vec3 {
    let l2 = v.y * v.y + v.x * v.x + v.z * v.z;
    if l2 > TINY && f32::MAX > l2 { v * (1.0 / l2.sqrt()) } else { v }
}

pub fn contacts(
    geometry: Geometry<'_>,
    ellipsoid: &EllipsoidShape,
    a_ref: EntryId,
    a: &Surface,
    shape: &crate::static_world::Convex,
    b_ref: EntryId,
    b: &Surface,
    out: &mut impl ContactSink,
) {
    hull(ellipsoid, a_ref, a, geometry.convex(shape), shape.material, b_ref, b, out);
}

pub fn hull(
    ellipsoid: &EllipsoidShape,
    a_ref: EntryId,
    a: &Surface,
    hull: Convex<'_>,
    material: u16,
    b_ref: EntryId,
    b: &Surface,
    out: &mut impl ContactSink,
) {
    let radii = ellipsoid.radii;
    let (normals, local) = unit_frame(ellipsoid, a, b);
    if hull.faces.is_empty() {
        todo!("empty convex hull");
    }
    let mut behind = 0;
    let mut nearest_inside = (f32::MIN, Vec3::new(1.0 / radii.z, 1.0 / radii.z, 1.0 / radii.z));
    let mut edges = Vec::new();
    for face in hull.faces {
        let normal = unit(normals * face.normal);
        let indices = hull.indices(face);
        let mut previous_id = indices[0];
        let mut point = local.transform_point(hull.vertices[previous_id as usize]);
        let distance = -normal.dot(point);
        if distance > 1.0 {
            continue;
        }
        if 0.0 > distance {
            behind += 1;
            if distance >= nearest_inside.0 {
                nearest_inside = (distance, normal);
            }
            continue;
        }
        let projected =
            Vec3::new(0.0 + normal.x * -distance, 0.0 + normal.y * -distance, 0.0 + normal.z * -distance);
        let mut outside = None;
        for &id in indices.iter().rev() {
            let next = local.transform_point(hull.vertices[id as usize]);
            let edge = unit(point - next);
            let side = (edge.z * normal.x - edge.x * normal.z) * (projected.y - next.y)
                + (edge.y * normal.z - edge.z * normal.y) * (projected.x - next.x)
                + (edge.x * normal.y - edge.y * normal.x) * (projected.z - next.z);
            if side > 0.0 {
                outside = Some((id, previous_id, side));
                break;
            }
            previous_id = id;
            point = next;
        }
        if let Some((id, previous_id, side)) = outside {
            if 1.0 > side {
                let edge = [id.min(previous_id), id.max(previous_id)];
                if edges.len() < MAX_EDGES && !edges.contains(&edge) {
                    edges.push(edge);
                }
            }
            continue;
        }
        emit(ellipsoid, a_ref, a, material, b_ref, out, -normal, (1.0 - distance) * normal, projected, true);
        return;
    }
    if behind == hull.faces.len() {
        let (distance, normal) = nearest_inside;
        emit(
            ellipsoid,
            a_ref,
            a,
            material,
            b_ref,
            out,
            -normal,
            (1.0 - distance) * normal,
            -distance * normal,
            true,
        );
        return;
    }
    if let Some((best, point)) = closest_edge(&hull, local, edges) {
        emit(
            ellipsoid,
            a_ref,
            a,
            material,
            b_ref,
            out,
            point,
            (best.sqrt() - 1.0) * unit(point),
            point,
            false,
        );
    }
}

fn unit_frame(ellipsoid: &EllipsoidShape, a: &Surface, b: &Surface) -> (Mat3, Iso) {
    let radii = ellipsoid.radii;
    let (at, bt) = (a.iso.rot, b.iso.rot);
    let normals = Mat3::from_array(std::array::from_fn(|k| {
        let (r, c) = (k / 3, k % 3);
        let x = at.rows[0][r] * bt.rows[0][c];
        let y = at.rows[1][r] * bt.rows[1][c];
        let z = at.rows[2][r] * bt.rows[2][c];
        let xy = if c == 0 || (r == 2 && c == 1) { y + x } else { x + y };
        (xy + z) * radii[r]
    }));
    let mut local = a.iso.inv_mul(&b.iso);
    for row in 0..3 {
        let inv = 1.0 / radii[row];
        local.rot.rows[row] *= inv;
        local.pos[row] *= inv;
    }
    (normals, local)
}

fn closest_edge(hull: &Convex<'_>, local: Iso, edges: Vec<[u32; 2]>) -> Option<(f32, Vec3)> {
    let mut closest = None;
    let mut best = f32::MAX;
    for [i, j] in edges {
        let a = local.transform_point(hull.vertices[i as usize]);
        let b = local.transform_point(hull.vertices[j as usize]);
        let center = (b + a) * 0.5;
        let edge = b - a;
        let l2 = edge.y * edge.y + edge.x * edge.x + edge.z * edge.z;
        let (dir, half) = if l2 > TINY && f32::MAX > l2 {
            let len = l2.sqrt();
            (edge * (1.0 / len), len * 0.5)
        } else {
            todo!("degenerate convex edge");
        };
        let (distance, t) = segment_distance(center, dir, half, Vec3::ZERO);
        if distance > 1.0 {
            continue;
        }
        if !(distance >= best) {
            best = distance;
            closest = Some(dir * t + center);
        }
    }
    closest.map(|point| (best, point))
}

fn emit(
    ellipsoid: &EllipsoidShape,
    a_ref: EntryId,
    a: &Surface,
    material: u16,
    b_ref: EntryId,
    out: &mut impl ContactSink,
    normal: Vec3,
    penetration: Vec3,
    point: Vec3,
    face: bool,
) {
    let radii = ellipsoid.radii;
    let world = a.iso * Iso::diagonal(radii, Vec3::ZERO);
    let normals = a.iso.rot * Mat3::diagonal(Vec3::new(1.0 / radii.x, 1.0 / radii.y, 1.0 / radii.z));
    let mut world_normal = Vec3::new(
        normal.y * normals.rows[0].y + normal.x * normals.rows[0].x + normal.z * normals.rows[0].z,
        normal.y * normals.rows[1].y + normal.x * normals.rows[1].x + normal.z * normals.rows[1].z,
        normal.y * normals.rows[2].y + normal.x * normals.rows[2].x + normal.z * normals.rows[2].z,
    );
    let l2 = world_normal.dot(world_normal);
    if l2 > TINY && f32::MAX > l2 {
        world_normal *= 1.0 / l2.sqrt();
    }
    let contact = out.take_contact();
    contact.a = Some(a_ref);
    contact.b = Some(b_ref);
    contact.material_a = ellipsoid.material;
    contact.material_b = material;
    contact.normal = -world_normal;
    contact.face_normal = -world_normal;
    let displacement = penetration;
    contact.penetration = Vec3::new(
        displacement.y * world.rot.rows[0].y
            + displacement.x * world.rot.rows[0].x
            + displacement.z * world.rot.rows[0].z,
        displacement.y * world.rot.rows[1].y
            + displacement.x * world.rot.rows[1].x
            + displacement.z * world.rot.rows[1].z,
        displacement.y * world.rot.rows[2].y
            + displacement.x * world.rot.rows[2].x
            + displacement.z * world.rot.rows[2].z,
    );
    contact.point = world.transform_point(point);
    contact.flags.set(ContactFlags::OTHER_POINT, false);
    contact.flags.set(ContactFlags::FACE, face);
}

pub(super) fn segment_distance(center: Vec3, dir: Vec3, half: f32, point: Vec3) -> (f32, f32) {
    let delta = point - center;
    let t = delta.y * dir.y + delta.x * dir.x + delta.z * dir.z;
    let t = if -half > t { -half } else { minss(half, t) };
    let v = point - (dir * t + center);
    (v.y * v.y + v.x * v.x + v.z * v.z, t)
}
