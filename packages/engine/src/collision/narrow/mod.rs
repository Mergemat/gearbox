pub mod convex;
pub mod ellipsoid;
pub mod sphere;
mod sphere_convex;

use super::{ContactSink, Surface};
use crate::{
    EntryId,
    shape::{Geometry, Shape},
};

pub fn contacts(
    geometry: Geometry<'_>,
    a_ref: EntryId,
    a: &Surface,
    b_ref: EntryId,
    b: &Surface,
    out: &mut impl ContactSink,
) {
    match (geometry.shape(a.shape), geometry.shape(b.shape)) {
        (Shape::Sphere(s), Shape::Mesh(m)) => sphere::mesh(s, a_ref, a, geometry.mesh(m), b_ref, b, out),
        (Shape::Ellipsoid(shape), Shape::Mesh(mesh)) => {
            ellipsoid::mesh(shape, a_ref, a, geometry.mesh(mesh), b_ref, b, out)
        }
        (Shape::Sphere(s), Shape::Convex(h)) => {
            sphere_convex::contacts(geometry, s, a_ref, a, h, b_ref, b, out)
        }
        (Shape::Ellipsoid(s), Shape::Convex(h)) => convex::contacts(geometry, s, a_ref, a, h, b_ref, b, out),
        (a, b) => todo!("narrowphase pair {}x{}", a.code(), b.code()),
    }
}
