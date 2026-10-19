use crate::collision::ContactFlags;
use crate::{ShapeId, Vec3};
use crate::shape::{Geometry, Shape};
use super::{Contact, Surface};

fn bytes(a: impl IntoIterator<Item = u8>, b: impl IntoIterator<Item = u8>) -> i32 {
    for (a, b) in a.into_iter().zip(b) {
        if a != b { return if a < b { -1 } else { 1 }; }
    }
    0
}

fn floats(values: impl IntoIterator<Item = f32>) -> impl Iterator<Item = u8> {
    values.into_iter().flat_map(f32::to_le_bytes)
}

pub fn item_tie_compare(geometry: Geometry<'_>, a: ShapeId, b: ShapeId) -> i32 {
    let (ka, kb) = (geometry.shape(a).code(), geometry.shape(b).code());
    if ka != kb { return if ka < kb { 1 } else { -1 }; }
    if a == b { return 0; }
    if !matches!(geometry.shape(a), Shape::Mesh(_)) {
        todo!("non-mesh comparison bounds");
    }
    let (a_box, b_box) = (geometry.local_box(a), geometry.local_box(b));
    let r = bytes(
        floats(a_box.center.to_array().into_iter().chain(a_box.half.to_array())),
        floats(b_box.center.to_array().into_iter().chain(b_box.half.to_array())),
    );
    if r != 0 || ka != 7 { return r; }
    let (Shape::Mesh(a), Shape::Mesh(b)) = (geometry.shape(a), geometry.shape(b))
        else { unreachable!("kind 7 is mesh") };
    let (a, b) = (geometry.mesh(a), geometry.mesh(b));
    if a.triangles.len() != b.triangles.len() {
        return if a.triangles.len() < b.triangles.len() { 1 } else { -1 };
    }
    let tri_bytes = |t: &crate::arrays::Triangle| {
        t.verts.into_iter().flat_map(u32::to_le_bytes)
            .chain([t.surface, t.gameplay]).chain(t.tie_break.to_le_bytes())
    };
    let r = bytes(a.triangles.iter().flat_map(tri_bytes), b.triangles.iter().flat_map(tri_bytes));
    if r != 0 { return r; }
    if a.vertices.len() != b.vertices.len() {
        return if a.vertices.len() < b.vertices.len() { 1 } else { -1 };
    }
    bytes(a.vertices.iter().flat_map(|v| floats(v.to_array())),
        b.vertices.iter().flat_map(|v| floats(v.to_array())))
}

pub fn item_compare(geometry: Geometry<'_>, a: &Surface, b: &Surface) -> i32 {
    for k in 0..3 {
        let (x, y) = (b.iso.pos[k], a.iso.pos[k]);
        if !(x <= y) { return 1; }
        if x < y { return -1; }
    }
    let r = bytes(floats(a.iso.rot.to_array()), floats(b.iso.rot.to_array()));
    if r != 0 { return r; }
    item_tie_compare(geometry, a.shape, b.shape)
}

pub fn contact_compare(geometry: Geometry<'_>, a: &Contact, b: &Contact) -> i32 {
    let vectors = [(a.selected_point(), b.selected_point()),
        (a.normal, b.normal), (a.penetration, b.penetration)];
    for (a, b) in vectors {
        for k in 0..3 {
            if a[k] < b[k] { return 1; }
            if !(a[k] <= b[k]) { return -1; }
        }
    }
    let (fa, fb) = (a.flags.contains(ContactFlags::FACE), b.flags.contains(ContactFlags::FACE));
    if fa != fb { return if !fa { -1 } else { 1 }; }
    for (a, b) in [(a.b, b.b), (a.a, b.a)] {
        if let (Some(a), Some(b)) = (a, b) {
            let ia = geometry.entry_item(a).id as i32;
            let ib = geometry.entry_item(b).id as i32;
            if ia != ib { return ia.wrapping_sub(ib); }
        }
    }
    0
}

pub(super) fn dot_yxz(a: Vec3, b: Vec3) -> f32 { a.y * b.y + a.x * b.x + a.z * b.z }
