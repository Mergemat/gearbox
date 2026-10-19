use crate::{Aabb, Vec3};
pub use crate::arrays::TreeNode as Node;
mod build;
pub use build::{Leaf, box_union};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AabbTree {
    pub nodes: Vec<Node>,
    pub bounds: Aabb,
    pub leaf_count: u32,
    pub depth: u32,
    pub leaf_size: u32,
}

pub struct AabbQuery<'a> {
    nodes: &'a [Node],
    cursor: usize,
    bounds: Aabb,
}

impl<'a> AabbQuery<'a> {
    pub fn new(nodes: &'a [Node], bounds: Aabb) -> Self {
        Self { nodes, cursor: 0, bounds }
    }
}

impl Iterator for AabbQuery<'_> {
    type Item = u32;

    fn next(&mut self) -> Option<u32> {
        while let Some(n) = self.nodes.get(self.cursor) {
            let (c, h) = (self.bounds.center, self.bounds.half);
            let (nc, nh) = (n.bounds.center, n.bounds.half);
            let separated = (nh.z + h.z < (nc.z - c.z).abs())
                | (nh.y + h.y < (nc.y - c.y).abs())
                | (nh.x + h.x < (nc.x - c.x).abs());
            let leaf = n.link & 1 != 0;
            self.cursor += if !separated || leaf { 1 } else { (n.link >> 1) as usize };
            if !separated && leaf {
                return Some(n.link >> 1);
            }
        }
        None
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Segment {
    pub center: Vec3,
    pub half: Vec3,
    pub abs_half: Vec3,
}

impl Segment {
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        let half = direction * 0.5;
        Self { center: half + origin, half, abs_half: half.abs() }
    }

    pub fn bounds(self) -> Aabb {
        Aabb::new(self.center, self.abs_half)
    }

    pub fn crosses(self, bounds: Aabb) -> bool {
        let (c, h, a) = (self.center, self.half, self.abs_half);
        let (bc, be) = (bounds.center, bounds.half);
        for k in [0, 2, 1] {
            let d = bc[k] - c[k];
            let r = be[k] + a[k];
            if d > r || -d > r { return false; }
        }
        let d = c - bc;
        if d.x.abs() > be.x + a.x || d.y.abs() > be.y + a.y || d.z.abs() > be.z + a.z { return false; }
        if (h.y * d.z - h.z * d.y).abs() > be.y * a.z + be.z * a.y { return false; }
        if (h.z * d.x - h.x * d.z).abs() > be.x * a.z + be.z * a.x { return false; }
        !((h.x * d.y - h.y * d.x).abs() > be.y * a.x + be.x * a.y)
    }
}

pub struct SegmentQuery<'a> {
    nodes: &'a [Node],
    cursor: usize,
    segment: Segment,
}

impl<'a> SegmentQuery<'a> {
    pub fn new(nodes: &'a [Node], segment: Segment) -> Self {
        Self { nodes, cursor: 0, segment }
    }
}

impl Iterator for SegmentQuery<'_> {
    type Item = u32;

    fn next(&mut self) -> Option<u32> {
        while let Some(n) = self.nodes.get(self.cursor) {
            let hit = self.segment.crosses(n.bounds);
            let leaf = n.link & 1 != 0;
            self.cursor += if hit || leaf { 1 } else { (n.link >> 1) as usize };
            if hit && leaf { return Some(n.link >> 1); }
        }
        None
    }
}

pub fn inside(a: Aabb, b: Aabb) -> bool {
    for k in 0..3 {
        if b.center[k] - b.half[k] > a.center[k] - a.half[k] { return false; }
    }
    for k in 0..3 {
        if a.half[k] + a.center[k] > b.half[k] + b.center[k] { return false; }
    }
    true
}

pub fn overlap(a: Aabb, b: Aabb) -> bool {
    for k in [0, 2, 1] {
        let d = a.center[k] - b.center[k];
        let s = b.half[k] + a.half[k];
        if d > s || -d > s { return false; }
    }
    true
}

pub fn union(boxes: impl IntoIterator<Item = Aabb>) -> Aabb {
    let mut lo = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut hi = Vec3::new(f32::MIN, f32::MIN, f32::MIN);
    for b in boxes {
        for k in 0..3 {
            lo[k] = crate::math::minss(b.center[k] - b.half[k], lo[k]);
            hi[k] = crate::math::maxss(b.center[k] + b.half[k], hi[k]);
        }
    }
    Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5)
}
