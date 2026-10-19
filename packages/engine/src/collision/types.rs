use crate::{Aabb, DynamicItemId, EntryId, ItemId, Iso, Quat, ShapeId, Vec3};
use super::{ContactFlags, ItemFlags, ItemEvents};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionWorld {
    pub merge_pass: u32,
    pub post_filter: u64,
    pub static_dirty: u32,
    pub warp_item: Option<ItemId>,
    pub warp_bounds: Aabb,
    pub warp_surfaces: std::ops::Range<u32>,
    pub warp_boxes: Vec<Aabb>,
    pub dynamic_surfaces: std::ops::Range<u32>,
    pub dynamic_tree_nodes: Vec<crate::arrays::TreeNode>,
    pub updated_surfaces: std::ops::Range<u32>,
    pub updated_tree_nodes: Vec<crate::arrays::TreeNode>,
    pub dynamic_dirty: u32,
    pub dynamic_items: Vec<DynamicItemId>,
    pub discrete_items: Vec<DynamicItemId>,
    pub shape_entries: std::ops::Range<u32>,
    pub continuous_items: Vec<DynamicItemId>,
    pub continuous_entries: std::ops::Range<u32>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct DynamicItem {
    pub class: u32,
    pub kind: u8,
    pub layer: u8,
    pub flags: ItemFlags,
    pub blocked_layers: u32,
    pub id: u32,
    pub event_flags: ItemEvents,
    pub has_contact_callback: bool,
    pub receives_triggers: bool,
    pub shape: ShapeId,
    pub iso: Iso,
    pub entry_start: u16,
    pub entry_count: u16,
    pub quat: Quat,
    pub pos: Vec3,
    pub body: u32,
    pub cached_body: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Contact {
    pub a: Option<EntryId>,
    pub b: Option<EntryId>,
    pub penetration: Vec3,
    pub normal: Vec3,
    pub point: Vec3,
    pub face_normal: Vec3,
    pub other_point: Vec3,
    pub event_time: u32,
    pub material_a: u16,
    pub material_b: u16,
    pub flags: ContactFlags,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Surface {
    pub shape: ShapeId,
    pub iso: Iso,
    pub item: Option<ItemId>,
    pub tag: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ShapeEntry {
    pub shape: ShapeId,
    pub iso: Iso,
    pub item: Option<ItemId>,
    pub tag: u64,
    pub bounds: Aabb,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContinuousEntry {
    pub shape: ShapeId,
    pub iso: Iso,
    pub item: Option<ItemId>,
    pub tag: u64,
    pub bounds: Aabb,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CompoundInstance {
    pub base: ShapeId,
    pub transforms: Vec<Iso>,
    pub tags: Vec<u64>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SphereShape {
    pub material: u16,
    pub radius: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct EllipsoidShape {
    pub material: u16,
    pub radii: Vec3,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoxShape {
    pub material: u16,
    pub bounds: Aabb,
}
