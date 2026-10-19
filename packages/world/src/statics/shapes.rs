use std::collections::HashSet;
use gearbox_engine::{Aabb, Iso, ShapeId, SharedShapeId, Vec3};
use gearbox_engine::arrays::{Triangle, CookedTriangle};
use gearbox_engine::math::{minss, maxss};
use gearbox_engine::static_world::{Shape, Geometry, Mesh, Compound, Tree};
use super::tree::{self, Leaf};
mod convex;

pub fn mesh_create(
    shapes: &mut Vec<Shape>, vertices: &[[f32; 3]], triangles: &[[u8; 16]], main_dir: [f32; 3],
) -> Result<u32, String> {
    let id = shapes.len() as u32;
    let triangles = triangles.iter().map(|t| Triangle {
        verts: std::array::from_fn(|i| u32::from_le_bytes(std::array::from_fn(|byte| t[i * 4 + byte]))),
        surface: t[12], gameplay: t[13], tie_break: u16::from_le_bytes([t[14], t[15]]),
    }).collect::<Vec<_>>();
    if triangles.iter().any(|triangle| triangle.verts.iter().any(|&i| i as usize >= vertices.len())) {
        return Err("mesh triangle vertex index out of bounds".into());
    }
    shapes.push(Shape { main_dir: Vec3::from_array(main_dir),
        geometry: Geometry::Mesh(Mesh { vertices: vertices.iter().copied().map(Vec3::from_array).collect(),
            triangles, cooked: Vec::new(), tree: Tree { bounds: Aabb::default(), ..Tree::default() } }) });
    Ok(id)
}

pub fn compound_create(
    shapes: &mut Vec<Shape>, children: &[u32], locs: &[[f32; 12]], joints: &[i16], main_dir: [f32; 3],
) -> u32 {
    let id = shapes.len() as u32;
    shapes.push(Shape { main_dir: Vec3::from_array(main_dir), geometry: Geometry::Compound(Compound {
        children: children.iter().map(|&id| ShapeId::Shared(SharedShapeId(id))).collect(), locations: locs.iter().copied().map(Iso::from_array).collect(),
        joints: joints.to_vec(), tree: Tree::default(),
    }) });
    id
}

pub fn convex_create(
    shapes: &mut Vec<Shape>, raw: &[u8], material: u16, main_dir: [f32; 3],
) -> Result<u32, String> {
    let hull = convex::read(raw, material).map_err(|e| e.to_string())?;
    let id = shapes.len() as u32;
    shapes.push(Shape { main_dir: Vec3::from_array(main_dir), geometry: Geometry::Convex(hull) });
    Ok(id)
}

fn cook(mesh: &mut Mesh) {
    mesh.cooked = mesh.triangles.iter().map(|t| {
        let [p0, p1, p2] = t.verts.map(|i| mesh.vertices[i as usize]);
        let e1 = p1 - p0;
        let e2 = p2 - p0;
        let mut n = cross(e1, e2);
        let len2 = (n.y * n.y + n.x * n.x) + n.z * n.z;
        if len2 > 1.00000005e-20 { n = n * (1.0 / len2.sqrt()); }
        CookedTriangle { normal: n, offset: ((-n.x) * p0.x - n.y * p0.y) - n.z * p0.z,
            verts: t.verts, ids: u16::from_le_bytes([t.surface, t.gameplay]), }
    }).collect();
    let mut boxes = mesh.triangles.iter().enumerate().map(|(index, t)| {
        let [a, b, c] = t.verts.map(|i| mesh.vertices[i as usize]);
        let lo = Vec3::from_array(std::array::from_fn(|k| minss(c[k], minss(b[k], a[k]))));
        let hi = Vec3::from_array(std::array::from_fn(|k| maxss(c[k], maxss(b[k], a[k]))));
        Leaf { bounds: Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5), index: index as u32 }
    }).collect::<Vec<_>>();
    mesh.tree = tree::build(&mut boxes, 2);
}

pub fn prepare(shapes: &mut [Shape], id: u32, prepared: &mut HashSet<u32>) {
    if !prepared.insert(id) { return; }
    let shape = &mut shapes[id as usize];
    match &mut shape.geometry {
        Geometry::Mesh(mesh) => cook(mesh),
        Geometry::Convex(hull) => convex::bounds(hull),
        Geometry::Compound(_) => {}
    }
    if let Geometry::Compound(compound) = &shape.geometry {
        let children = compound.children.clone();
        for child in children { prepare(shapes, child.shared().expect("static child").0, prepared); }
    }
}

pub fn bounds(shape: &Shape) -> Aabb {
    match &shape.geometry {
        Geometry::Mesh(mesh) => mesh.tree.bounds,
        Geometry::Convex(hull) => hull.bounds,
        Geometry::Compound(_) => unreachable!("static surfaces are leaf shapes"),
    }
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}
