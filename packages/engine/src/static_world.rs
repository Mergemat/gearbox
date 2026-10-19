use crate::arrays::{CookedTriangle, TreeNode, Triangle};
use crate::{Aabb, Iso, ShapeId, Vec3};

pub const LAYER_TRIGGER: u8 = 1;
pub const LAYER_STATIC: u8 = 2;
pub const LAYER_GATE: u8 = 0x16;
pub const NO_CHILD_TAG: u64 = u32::MAX as u64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Entity {
    None,
    Waypoint(Option<u32>),
    Gate(u32),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Item {
    pub entity: Entity,
    pub shape: u32,
    pub iso: Iso,
    pub layer: u8,
    pub flags: crate::collision::ItemEvents,
    pub query_flags: crate::collision::ItemFlags,
    pub id: u32,
    pub index: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Surface {
    pub shape: u32,
    pub iso: Iso,
    pub item: u32,
    pub tag: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tree {
    pub nodes: Vec<TreeNode>,
    pub leaves: u32,
    pub depth: u32,
    pub leaf_size: u32,
    pub bounds: Aabb,
}

impl Default for Tree {
    fn default() -> Self {
        Self { nodes: Vec::new(), leaves: 0, depth: 0, leaf_size: 2, bounds: Aabb::EMPTY }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<Triangle>,
    pub cooked: Vec<CookedTriangle>,
    pub tree: Tree,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Compound {
    pub children: Vec<ShapeId>,
    pub locations: Vec<Iso>,
    pub joints: Vec<i16>,
    pub tree: Tree,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    pub normal: Vec3,
    pub offset: f32,
    pub axis: u32,
    pub positive: bool,
    pub start: u32,
    pub count: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Convex {
    pub material: u16,
    pub vertices: Vec<Vec3>,
    pub indices: Vec<u32>,
    pub faces: Vec<Face>,
    pub edges: Vec<[u32; 3]>,
    pub normals: Vec<Vec3>,
    pub directions: Vec<Vec3>,
    pub bounds: Aabb,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    Mesh(Mesh),
    Compound(Compound),
    Convex(Convex),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Shape {
    pub main_dir: Vec3,
    pub geometry: Geometry,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StaticWorld {
    pub items: Vec<Item>,
    pub surfaces: Vec<Surface>,
    pub shapes: Vec<Shape>,
    pub tree: Tree,
    pub material14_alias: bool,
}
