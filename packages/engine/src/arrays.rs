use crate::{Aabb, Vec3};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TreeNode {
    pub bounds: Aabb,
    pub link: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Triangle {
    pub verts: [u32; 3],
    pub surface: u8,
    pub gameplay: u8,
    pub tie_break: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CookedTriangle {
    pub normal: Vec3,
    pub offset: f32,
    pub verts: [u32; 3],
    pub ids: u16,
}
