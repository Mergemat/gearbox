use crate::collision::ContactFlags;
use crate::{Aabb, EntryId, Vec3};
use crate::shape::Mesh;
use crate::tree::AabbQuery;
use super::super::{ContactSink, SphereShape, Surface};

const TINY: f32 = 1.0e-10;
const EPS: f32 = 0.00001;

fn sqrt(x: f32) -> f32 {
    if 0.0 > x {
        todo!("sqrtf of a negative");
    }
    x.sqrt()
}

pub(super) struct Hit {
    point: [f32; 3],
    direction: [f32; 3],
    penetration: [f32; 3],
    face: bool,
}

fn touch(center: [f32; 3], radius: f32, normal: [f32; 3], vertices: &[[f32; 3]; 3]) -> Option<Hit> {
    let p0 = vertices[0];
    let distance = (center[1] - p0[1]) * normal[1] + (center[0] - p0[0]) * normal[0]
        + (center[2] - p0[2]) * normal[2];
    if distance > radius || 0.0 > distance {
        return None;
    }
    let radius_squared = radius * radius;
    let section_radius = sqrt(radius_squared - distance * distance);
    let negative_distance = -distance;
    let projected = [
        normal[0] * negative_distance + center[0],
        normal[1] * negative_distance + center[1],
        normal[2] * negative_distance + center[2],
    ];
    let towards = |point, ok: &dyn Fn(f32) -> bool| boundary_hit(center, radius, normal, point, ok);
    let vertex = |l2: f32| radius_squared >= l2 && l2 > TINY;
    for index in 0..3 {
        let pk = vertices[index];
        let pn = vertices[if index == 2 { 0 } else { index + 1 }];
        let edge = [pn[0] - pk[0], pn[1] - pk[1], pn[2] - pk[2]];
        let l2 = edge[1] * edge[1] + edge[0] * edge[0] + edge[2] * edge[2];
        if !(l2 > TINY) || !(f32::MAX > l2) {
            return None;
        }
        let len = sqrt(l2);
        let ac = [projected[0] - pk[0], projected[1] - pk[1], projected[2] - pk[2]];
        let inv = 1.0 / len;
        let edge = [edge[0] * inv, edge[1] * inv, edge[2] * inv];
        let inward = [
            edge[1] * normal[2] - edge[2] * normal[1],
            edge[2] * normal[0] - edge[0] * normal[2],
            edge[0] * normal[1] - edge[1] * normal[0],
        ];
        let dist = inward[1] * ac[1] + inward[0] * ac[0] + inward[2] * ac[2];
        if dist > section_radius {
            return None;
        }
        if !(dist > 0.0) {
            continue;
        }
        let t = edge[1] * ac[1] + edge[0] * ac[0] + edge[2] * ac[2];
        if 0.0 > t {
            return towards(pk, &vertex);
        }
        let u = (projected[1] - pn[1]) * edge[1] + (projected[0] - pn[0]) * edge[0]
            + (projected[2] - pn[2]) * edge[2];
        if u > 0.0 {
            return towards(pn, &vertex);
        }
        let nd2 = -dist;
        let point = [
            nd2 * inward[0] + projected[0],
            nd2 * inward[1] + projected[1],
            nd2 * inward[2] + projected[2],
        ];
        return towards(point, &|l2| l2 > EPS);
    }
    let depth = radius - distance;
    Some(Hit {
        point: projected,
        direction: normal,
        penetration: [normal[0] * depth, normal[1] * depth, normal[2] * depth],
        face: true,
    })
}

pub fn mesh(
    sphere: &SphereShape, a_ref: EntryId, a: &Surface, mesh: Mesh<'_>, b_ref: EntryId, b: &Surface,
    out: &mut impl ContactSink,
) {
    let offset = a.iso.pos - b.iso.pos;
    let rotation = b.iso.rot;
    let local = Vec3::new(
        offset.y * rotation.rows[1].x + offset.x * rotation.rows[0].x + offset.z * rotation.rows[2].x,
        offset.y * rotation.rows[1].y + offset.x * rotation.rows[0].y + offset.z * rotation.rows[2].y,
        offset.y * rotation.rows[1].z + offset.x * rotation.rows[0].z + offset.z * rotation.rows[2].z,
    );
    let radius = sphere.radius;
    for id in AabbQuery::new(mesh.nodes, Aabb::new(local, Vec3::new(radius, radius, radius))) {
        let tri = &mesh.cooked[id as usize];
        let p = tri.verts.map(|i| mesh.vertices[i as usize].to_array());
        let Some(hit) = touch(local.to_array(), radius, tri.normal.to_array(), &p) else { continue; };
        let contact = out.take_contact();
        contact.other_point = rotate(b, hit.point.into()) + b.iso.pos;
        contact.flags.set(ContactFlags::OTHER_POINT, true);
        contact.normal = rotate(b, hit.direction.into());
        contact.point = (-radius) * contact.normal + a.iso.pos;
        contact.penetration = rotate(b, hit.penetration.into());
        contact.material_b = tri.ids;
        contact.flags.set(ContactFlags::FACE, hit.face);
        contact.face_normal = rotate(b, tri.normal);
        contact.material_a = sphere.material;
        contact.a = Some(a_ref);
        contact.b = Some(b_ref);
    }
}

fn rotate(b: &Surface, v: Vec3) -> Vec3 {
    let rotation = b.iso.rot;
    Vec3::new(
        v.y * rotation.rows[0].y + v.x * rotation.rows[0].x + v.z * rotation.rows[0].z,
        v.x * rotation.rows[1].x + v.y * rotation.rows[1].y + v.z * rotation.rows[1].z,
        v.x * rotation.rows[2].x + v.y * rotation.rows[2].y + v.z * rotation.rows[2].z,
    )
}

fn boundary_hit(
    center: [f32; 3], radius: f32, normal: [f32; 3], point: [f32; 3], ok: &dyn Fn(f32) -> bool,
) -> Option<Hit> {
    let delta = [center[0] - point[0], center[1] - point[1], center[2] - point[2]];
    let l2 = delta[1] * delta[1] + delta[0] * delta[0] + delta[2] * delta[2];
    if !ok(l2) {
        return None;
    }
    let len = sqrt(l2);
    let inv = 1.0 / len;
    let scale = (len - radius) * inv;
    let direction = [delta[0] * inv, delta[1] * inv, delta[2] * inv];
    let depth = -((delta[1] * scale) * normal[1] + (delta[0] * scale) * normal[0]
        + (delta[2] * scale) * normal[2]);
    Some(Hit {
        point, direction,
        penetration: [normal[0] * depth, normal[1] * depth, normal[2] * depth],
        face: false,
    })
}
