use super::super::{ContactFlags, ContactSink, SphereShape, Surface};
use super::convex::{segment_distance, unit};
use crate::shape::{Geometry, convex::Convex};
use crate::{EntryId, Mat3, Vec3};

pub fn contacts(
    geometry: Geometry<'_>,
    sphere: &SphereShape,
    a_ref: EntryId,
    a: &Surface,
    shape: &crate::static_world::Convex,
    b_ref: EntryId,
    b: &Surface,
    out: &mut impl ContactSink,
) {
    let hull = geometry.convex(shape);
    assert!(!hull.faces.is_empty(), "empty convex hull");
    let center = local_center(a.iso.pos - b.iso.pos, b.iso.rot);
    let radius = sphere.radius;
    let mut behind = 0;
    let mut nearest = (f32::MIN, Vec3::ZERO);
    let mut edges = Vec::new();
    let mut face_contact = None;
    for face in hull.faces {
        let normal = face.normal;
        let distance = center.dot_yxz(normal) + face.offset;
        if distance > radius {
            continue;
        }
        if 0.0 > distance {
            behind += 1;
            if !(nearest.0 > distance) {
                nearest = (distance, normal);
            }
            continue;
        }
        let projected = -distance * normal + center;
        match outside_edge(&hull, face, projected) {
            Some((edge, side)) => {
                if radius > side && edges.len() < 64 && !edges.contains(&edge) {
                    edges.push(edge);
                }
            }
            None => {
                face_contact = Some((distance, normal));
                break;
            }
        }
    }
    if face_contact.is_none() && behind == hull.faces.len() {
        face_contact = Some(nearest);
    }
    let contact = if let Some((distance, normal)) = face_contact {
        let point = -distance * normal + center;
        let [x, y, z] = b.iso.rot.rows;
        let normal = -rotate(-normal, b.iso.rot);
        let contact = out.take_contact();
        contact.flags.set(ContactFlags::FACE, true);
        contact.flags.set(ContactFlags::OTHER_POINT, false);
        contact.normal = normal;
        contact.penetration = (radius - distance) * normal;
        contact.point = Vec3::new(point.dot(x), point.dot_yxz(y), point.dot_yxz(z)) + b.iso.pos;
        contact
    } else if let Some((distance, point)) = closest_edge(&hull, &edges, center, radius) {
        let point = rotate(point, b.iso.rot) + b.iso.pos;
        let normal = -unit(point - a.iso.pos);
        let contact = out.take_contact();
        contact.normal = normal;
        contact.penetration = (radius - distance.sqrt()) * normal;
        contact.point = point;
        contact
    } else {
        return;
    };
    contact.a = Some(a_ref);
    contact.b = Some(b_ref);
    contact.material_a = sphere.material;
    contact.material_b = shape.material;
}

fn outside_edge(
    hull: &Convex<'_>,
    face: &crate::static_world::Face,
    projected: Vec3,
) -> Option<([u32; 2], f32)> {
    let indices = hull.indices(face);
    let mut previous = indices[0];
    let mut point = hull.vertices[previous as usize];
    let normal = face.normal;
    for &id in indices.iter().rev() {
        let next = hull.vertices[id as usize];
        let edge = unit(point - next);
        let delta = projected - next;
        let side = (edge.z * normal.x - edge.x * normal.z) * delta.y
            + (edge.y * normal.z - edge.z * normal.y) * delta.x
            + (edge.x * normal.y - edge.y * normal.x) * delta.z;
        if side > 0.0 {
            return Some(([id.min(previous), id.max(previous)], side));
        }
        previous = id;
        point = next;
    }
    None
}

fn closest_edge(hull: &Convex<'_>, edges: &[[u32; 2]], center: Vec3, radius: f32) -> Option<(f32, Vec3)> {
    let mut best = f32::MAX;
    let mut closest = None;
    for &[i, j] in edges {
        let a = hull.vertices[i as usize];
        let b = hull.vertices[j as usize];
        let midpoint = (b + a) * 0.5;
        let edge = b - a;
        let squared = edge.dot_yxz(edge);
        let (direction, half) = if squared > 1.0e-10 && f32::MAX > squared {
            let length = squared.sqrt();
            (edge * (1.0 / length), length * 0.5)
        } else {
            (edge, 0.0)
        };
        let (distance, t) = segment_distance(midpoint, direction, half, center);
        if distance > radius * radius || distance >= best {
            continue;
        }
        best = distance;
        closest = Some(direction * t + midpoint);
    }
    closest.map(|point| (best, point))
}

fn local_center(delta: Vec3, rotation: Mat3) -> Vec3 {
    Vec3::new(delta.dot_yxz(rotation.col(0)), delta.dot(rotation.col(1)), delta.dot(rotation.col(2)))
}

fn rotate(point: Vec3, rotation: Mat3) -> Vec3 {
    Vec3::from_array(std::array::from_fn(|axis| point.dot_yxz(rotation.rows[axis])))
}
