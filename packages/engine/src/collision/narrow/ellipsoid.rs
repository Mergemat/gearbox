use crate::collision::ContactFlags;
use crate::{Aabb, EntryId, Iso, Vec3};
use crate::shape::Mesh;
use crate::tree::AabbQuery;
use super::super::{ContactSink, EllipsoidShape, Surface};

const TINY: f32 = 1.0e-10;
const EPS: f32 = 0.00001;

fn sqrt(x: f32) -> f32 {
    if 0.0 > x {
        todo!("sqrtf of a negative");
    }
    x.sqrt()
}

struct Tri { normal: [f32; 3], vertices: [[f32; 3]; 3] }

pub fn mesh(
    ellipsoid: &EllipsoidShape, a_ref: EntryId, a: &Surface, mesh: Mesh<'_>, b_ref: EntryId, b: &Surface,
    out: &mut impl ContactSink,
) {
    let rel = b.iso.inv_mul(&a.iso);
    let bounds = Aabb::new(Vec3::ZERO, ellipsoid.radii).transform(&rel);
    let inverse = rel.inverse();
    let radii = ellipsoid.radii;
    let inv = Vec3::new(1.0 / radii.x, 1.0 / radii.y, 1.0 / radii.z);
    let mut local = inverse;
    for row in 0..3 {
        local.rot.rows[row] *= inv[row];
        local.pos[row] *= inv[row];
    }
    let world = a.iso * Iso::diagonal(radii, Vec3::ZERO);
    let normals = a.iso * Iso::diagonal(inv, Vec3::ZERO);
    for id in AabbQuery::new(mesh.nodes, bounds) {
        let tri = &mesh.cooked[id as usize];
        let vertices = tri.verts.map(|i| local.transform_point(mesh.vertices[i as usize]));
        let size = vertices.iter().flat_map(|v| v.to_array()).fold(1.0f32, |a, v| a.max(v.abs()));
        let edge = 1.0 + 0.01 + size * 1e-4;
        if (0..3).any(|axis| vertices.iter().all(|v| v[axis] > edge)
            || vertices.iter().all(|v| v[axis] < -edge)) { continue; }
        let Some(normal) = crate::math::triangle_normal(vertices[0], vertices[1], vertices[2])
            else { continue; };
        let triangle = Tri { normal: normal.to_array(), vertices: vertices.map(Vec3::to_array) };
        let Some(hit) = touch(&triangle) else { continue; };
        let contact = out.take_contact();
        let direction = Vec3::from(hit.direction);
        contact.point = world.transform_point(-direction);
        contact.other_point = world.transform_point(hit.point.into());
        contact.flags.set(ContactFlags::OTHER_POINT, true);
        contact.normal = normals.rot * direction;
        let l2 = contact.normal.dot(contact.normal);
        if l2 > TINY && f32::MAX > l2 { contact.normal *= 1.0 / sqrt(l2); }
        contact.penetration = world.rot * Vec3::from(hit.penetration);
        contact.material_b = tri.ids;
        contact.flags.set(ContactFlags::FACE, hit.face);
        let rotation = b.iso.rot;
        contact.face_normal = Vec3::new(
            rotation.rows[0].y * tri.normal.y + rotation.rows[0].x * tri.normal.x
                + rotation.rows[0].z * tri.normal.z,
            rotation.rows[1].y * tri.normal.y + rotation.rows[1].x * tri.normal.x
                + rotation.rows[1].z * tri.normal.z,
            rotation.rows[2].y * tri.normal.y + rotation.rows[2].x * tri.normal.x
                + rotation.rows[2].z * tri.normal.z,
        );
        contact.material_a = ellipsoid.material;
        contact.a = Some(a_ref);
        contact.b = Some(b_ref);
    }
}

struct Hit {
    point: [f32; 3],
    direction: [f32; 3],
    penetration: [f32; 3],
    face: bool,
}

fn touch(tr: &Tri) -> Option<Hit> {
    let normal = tr.normal;
    let p0 = tr.vertices[0];
    let distance = (0.0 - p0[1]) * normal[1] + (0.0 - p0[0]) * normal[0] + (0.0 - p0[2]) * normal[2];
    if distance > 1.0 || 0.0 > distance {
        return None;
    }
    let section_radius = sqrt(1.0 - distance * distance);
    let negative_distance = -distance;
    let projected = [
        normal[0] * negative_distance + 0.0,
        normal[1] * negative_distance + 0.0,
        normal[2] * negative_distance + 0.0,
    ];
    for k in 0..3 {
        let pk = tr.vertices[k];
        let pn = tr.vertices[if k == 2 { 0 } else { k + 1 }];
        let edge = [pn[0] - pk[0], pn[1] - pk[1], pn[2] - pk[2]];
        let l2 = edge[0] * edge[0] + edge[1] * edge[1] + edge[2] * edge[2];
        if !(l2 > TINY) || !(f32::MAX > l2) {
            return None;
        }
        let len = sqrt(l2);
        let ac = [projected[0] - pk[0], projected[1] - pk[1], projected[2] - pk[2]];
        let inv = 1.0 / len;
        let edge = [edge[0] * inv, edge[1] * inv, edge[2] * inv];
        let inward = [
            normal[2] * edge[1] - normal[1] * edge[2],
            normal[0] * edge[2] - normal[2] * edge[0],
            normal[1] * edge[0] - normal[0] * edge[1],
        ];
        let dist = ac[1] * inward[1] + ac[0] * inward[0] + ac[2] * inward[2];
        if dist > section_radius {
            return None;
        }
        if !(dist > 0.0) {
            continue;
        }
        let t = ac[0] * edge[0] + ac[1] * edge[1] + ac[2] * edge[2];
        let (point, delta) = boundary_point(projected, edge, inward, pk, pn, dist, t)?;
        return Some(boundary_hit(point, delta, normal));
    }
    let k = 1.0 - distance;
    Some(Hit {
        point: projected,
        direction: normal,
        penetration: [k * normal[0], k * normal[1], k * normal[2]],
        face: true,
    })
}

fn boundary_hit(point: [f32; 3], delta: [f32; 3], normal: [f32; 3]) -> Hit {
    let len = sqrt(delta[1] * delta[1] + delta[0] * delta[0] + delta[2] * delta[2]);
    let inv = 1.0 / len;
    let k = (len - 1.0) * inv;
    let direction = [inv * delta[0], inv * delta[1], inv * delta[2]];
    let depth = -((k * delta[1]) * normal[1] + (k * delta[0]) * normal[0] + (k * delta[2]) * normal[2]);
    Hit {
        point, direction, penetration: [normal[0] * depth, normal[1] * depth, normal[2] * depth],
        face: false,
    }
}

fn boundary_point(
    projected: [f32; 3], edge: [f32; 3], inward: [f32; 3], start: [f32; 3], end: [f32; 3],
    distance: f32, parameter: f32,
) -> Option<([f32; 3], [f32; 3])> {
    let point = if 0.0 > parameter {
        Some(start)
    } else {
        let past_end = (projected[1] - end[1]) * edge[1] + (projected[0] - end[0]) * edge[0]
            + (projected[2] - end[2]) * edge[2];
        if past_end > 0.0 { Some(end) } else { None }
    };
    let pair = match point {
        Some(point) => {
            let delta = [0.0 - point[0], 0.0 - point[1], 0.0 - point[2]];
            let squared = delta[1] * delta[1] + delta[0] * delta[0] + delta[2] * delta[2];
            if !(1.0 >= squared) || !(squared > TINY) { return None; }
            (point, delta)
        }
        None => {
            let negative_distance = -distance;
            let point = [
                negative_distance * inward[0] + projected[0],
                negative_distance * inward[1] + projected[1],
                negative_distance * inward[2] + projected[2],
            ];
            let delta = [0.0 - point[0], 0.0 - point[1], 0.0 - point[2]];
            let squared = delta[1] * delta[1] + delta[0] * delta[0] + delta[2] * delta[2];
            if !(squared > EPS) { return None; }
            (point, delta)
        }
    };
    Some(pair)
}
