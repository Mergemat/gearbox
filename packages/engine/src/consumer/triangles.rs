use crate::{Iso, Session, ShapeId, SharedShapeId};
use crate::static_world::{Geometry, StaticWorld};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicalTriangle {
    pub v: [[f32; 3]; 3],
    pub surface: u8,
    pub gameplay: u8,
}

impl Session {
    pub fn static_triangles(&self) -> Vec<PhysicalTriangle> {
        let world = &self.world.statics;
        let mut triangles = Vec::new();
        for item in &world.items {
            if item.flags.contains(crate::collision::ItemEvents::CONTACT_TIME) { continue; }
            leaves(world, ShapeId::Shared(SharedShapeId(item.shape)), item.iso.to_array(), 0, &mut triangles);
        }
        triangles
    }
}

fn apply(transform: [f32; 12], point: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| transform[3 * i] * point[0] + transform[3 * i + 1] * point[1]
        + transform[3 * i + 2] * point[2] + transform[9 + i])
}

fn compose(parent: [f32; 12], local: Iso) -> [f32; 12] {
    let local = local.to_array();
    let mut result = [0.0; 12];
    for row in 0..3 {
        for column in 0..3 {
            result[3 * row + column] = (0..3)
                .map(|axis| parent[3 * row + axis] * local[3 * axis + column]).sum();
        }
    }
    result[9..].copy_from_slice(&apply(parent, local[9..].try_into().unwrap()));
    result
}

fn leaves(world: &StaticWorld, shape: ShapeId, pose: [f32; 12], depth: usize,
    output: &mut Vec<PhysicalTriangle>)
{
    const MAX_COMPOUND_DEPTH: usize = 16;
    if depth > MAX_COMPOUND_DEPTH { return; }
    let Some(id) = shape.shared() else { return; };
    match &world.shapes[id.index()].geometry {
        Geometry::Compound(compound) => {
            for (&child, &local) in compound.children.iter().zip(&compound.locations) {
                leaves(world, child, compose(pose, local), depth + 1, output);
            }
        }
        Geometry::Mesh(mesh) => {
            for triangle in &mesh.triangles {
                if triangle.verts.iter().any(|&index| index as usize >= mesh.vertices.len()) { continue; }
                output.push(PhysicalTriangle {
                    v: triangle.verts.map(|index| apply(pose, mesh.vertices[index as usize].to_array())),
                    surface: triangle.surface, gameplay: triangle.gameplay,
                });
            }
        }
        Geometry::Convex(_) => {}
    }
}
