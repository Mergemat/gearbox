use crate::static_world as sw;
use crate::{Iso, ShapeId, State, Vec3, World};

pub fn shape(world: &mut World, state: &mut State, shape_id: ShapeId, scale: f32) {
    match shape_id {
        ShapeId::Sphere(id) => {
            let sphere = &mut state.sphere_shape[id.index()];
            sphere.radius *= scale;
        }
        ShapeId::Ellipsoid(id) => {
            let ellipsoid = &mut state.ellipsoid_shape[id.index()];
            ellipsoid.radii = scale * ellipsoid.radii;
        }
        ShapeId::Shared(id) => match &mut world.statics.shapes[id.index()].geometry {
            sw::Geometry::Mesh(mesh) => {
                for vertex in &mut mesh.vertices {
                    *vertex = scale * *vertex;
                }
            }
            sw::Geometry::Compound(compound) => {
                for location in &mut compound.locations {
                    location.pos = scale * location.pos;
                }
                let children = compound.children.clone();
                for child in children {
                    shape(world, state, child, scale);
                }
            }
            sw::Geometry::Convex(_) => {}
        },
        ShapeId::Compound(id) => {
            let base = state.compound_instance[id.index()].base;
            shape(world, state, base, scale);
            positions(&mut state.compound_instance[id.index()].transforms, scale);
        }
        ShapeId::Box(_) => {}
    }
}

fn positions(transforms: &mut [Iso], scale: f32) {
    for transform in transforms {
        transform.pos = scale * transform.pos;
    }
}

pub fn point(iso: Iso, scale: f32, p: Vec3) -> Vec3 {
    let d = p - iso.pos;
    let r = iso.rot;
    let l = Vec3::new(
        (r.rows[0].x * d.x + r.rows[1].x * d.y + r.rows[2].x * d.z) * scale,
        (r.rows[0].y * d.x + r.rows[1].y * d.y + r.rows[2].y * d.z) * scale,
        (r.rows[0].z * d.x + r.rows[1].z * d.y + r.rows[2].z * d.z) * scale,
    );
    Vec3::new(
        l.y * r.rows[0].y + l.x * r.rows[0].x + l.z * r.rows[0].z + iso.pos.x,
        l.y * r.rows[1].y + l.x * r.rows[1].x + l.z * r.rows[1].z + iso.pos.y,
        l.y * r.rows[2].y + l.x * r.rows[2].x + l.z * r.rows[2].z + iso.pos.z,
    )
}
