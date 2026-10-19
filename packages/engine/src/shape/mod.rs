use crate::arrays::{CookedTriangle, TreeNode, Triangle};
use crate::collision::*;
use crate::static_world as sw;
use crate::{Aabb, Iso, ItemId, ShapeId, State, Vec3, World};

pub mod convex;
mod mesh;
pub mod scale;
pub mod triangle;

#[derive(Clone, Copy, Debug)]
pub enum Shape<'a> {
    Sphere(&'a SphereShape),
    Ellipsoid(&'a EllipsoidShape),
    Box(&'a BoxShape),
    Mesh(&'a sw::Mesh),
    Compound(&'a sw::Compound),
    CompoundInstance(&'a CompoundInstance),
    Convex(&'a sw::Convex),
}

impl Shape<'_> {
    pub fn code(self) -> u32 {
        match self {
            Self::Sphere(_) => 0,
            Self::Ellipsoid(_) => 1,
            Self::Box(_) => 6,
            Self::Mesh(_) => 7,
            Self::Compound(_) => 13,
            Self::CompoundInstance(_) => 15,
            Self::Convex(_) => 10,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Geometry<'a> {
    pub world: &'a World,
    pub state: &'a State,
}

impl<'a> Geometry<'a> {
    pub const fn new(world: &'a World, state: &'a State) -> Self {
        Self { world, state }
    }

    pub fn shape(self, r: ShapeId) -> Shape<'a> {
        match r {
            ShapeId::Sphere(id) => Shape::Sphere(&self.state.sphere_shape[id.index()]),
            ShapeId::Ellipsoid(id) => Shape::Ellipsoid(&self.state.ellipsoid_shape[id.index()]),
            ShapeId::Box(id) => Shape::Box(&self.world.box_shape[id.index()]),
            ShapeId::Shared(id) => match &self.world.statics.shapes[id.index()].geometry {
                sw::Geometry::Mesh(mesh) => Shape::Mesh(mesh),
                sw::Geometry::Compound(compound) => Shape::Compound(compound),
                sw::Geometry::Convex(convex) => Shape::Convex(convex),
            },
            ShapeId::Compound(id) => Shape::CompoundInstance(&self.state.compound_instance[id.index()]),
        }
    }

    pub fn mesh(self, shape: &'a sw::Mesh) -> Mesh<'a> {
        Mesh {
            vertices: &shape.vertices,
            triangles: &shape.triangles,
            cooked: &shape.cooked,
            nodes: &shape.tree.nodes,
        }
    }

    pub fn local_box(self, shape: ShapeId) -> Aabb {
        match self.shape(shape) {
            Shape::Mesh(s) => {
                if !s.cooked.is_empty() {
                    s.tree.bounds
                } else {
                    Aabb::EMPTY
                }
            }
            Shape::Sphere(s) => Aabb::new(Vec3::ZERO, Vec3::new(s.radius, s.radius, s.radius)),
            Shape::Ellipsoid(s) => Aabb::new(Vec3::ZERO, s.radii),
            Shape::Box(s) => s.bounds,
            Shape::Convex(s) => s.bounds,
            Shape::Compound(s) => self.compound_box(s, &s.locations, None),
            Shape::CompoundInstance(s) => {
                let Shape::Compound(base) = self.shape(s.base) else {
                    panic!("instance base must be compound")
                };
                self.compound_box(base, &s.transforms[..base.children.len()], None)
            }
        }
    }

    pub fn world_box(self, shape: ShapeId, iso: Iso) -> Aabb {
        match self.shape(shape) {
            Shape::Sphere(s) => Aabb::new(iso.pos, Vec3::new(s.radius, s.radius, s.radius)),
            Shape::CompoundInstance(s) => {
                let Shape::Compound(base) = self.shape(s.base) else {
                    panic!("instance base must be compound")
                };
                self.compound_box(base, &s.transforms[..base.children.len()], Some(iso))
            }
            Shape::Compound(_) => todo!("compound world box method"),
            _ => self.local_box(shape).transform(&iso),
        }
    }

    fn compound_box(self, base: &sw::Compound, transforms: &[Iso], pose: Option<Iso>) -> Aabb {
        base.children
            .iter()
            .zip(transforms)
            .map(|(&child, &local)| self.world_box(child, pose.map_or(local, |iso| iso * local)))
            .reduce(crate::tree::box_union)
            .unwrap_or(Aabb::EMPTY)
    }

    pub fn entries(self, shape: ShapeId, iso: Iso, item: Option<ItemId>, tag: u64) -> Vec<ShapeEntry> {
        let mut out = Vec::new();
        self.add_entries(shape, iso, item, tag, &mut out);
        out
    }

    fn add_entries(
        self,
        shape: ShapeId,
        iso: Iso,
        item: Option<ItemId>,
        tag: u64,
        out: &mut Vec<ShapeEntry>,
    ) {
        let (children, transforms, tags) = match self.shape(shape) {
            Shape::Compound(s) => (s.children.as_slice(), s.locations.as_slice(), None),
            Shape::CompoundInstance(s) => {
                let Shape::Compound(base) = self.shape(s.base) else {
                    panic!("instance base must be compound")
                };
                (&base.children[..s.tags.len()], &s.transforms[..s.tags.len()], Some(&s.tags))
            }
            _ => {
                out.push(ShapeEntry { shape, iso, item, tag, bounds: self.world_box(shape, iso) });
                return;
            }
        };
        for (i, (&child, &local)) in children.iter().zip(transforms).enumerate() {
            let child_tag = tags.map_or(tag, |tags| tags[i]);
            self.add_entries(child, iso * local, item, child_tag, out);
        }
    }
}

#[derive(Clone, Copy)]
pub struct Mesh<'a> {
    pub vertices: &'a [Vec3],
    pub triangles: &'a [Triangle],
    pub cooked: &'a [CookedTriangle],
    pub nodes: &'a [TreeNode],
}

impl Geometry<'_> {
    pub fn place_shapes(self, entries: &mut [ShapeEntry], index: u32, shape: ShapeId, iso: Iso) {
        let (children, transforms) = match self.shape(shape) {
            Shape::Compound(s) => (s.children.as_slice(), s.locations.as_slice()),
            Shape::CompoundInstance(s) => {
                let Shape::Compound(base) = self.shape(s.base) else {
                    panic!("instance base must be compound")
                };
                (&base.children[..s.tags.len()], &s.transforms[..s.tags.len()])
            }
            _ => {
                let entry = &mut entries[index as usize];
                entry.iso = iso;
                entry.bounds = self.world_box(entry.shape, iso);
                return;
            }
        };
        for (i, (&child, &local)) in children.iter().zip(transforms).enumerate() {
            self.place_shapes(entries, index.wrapping_add(i as u32), child, iso * local);
        }
    }
}
