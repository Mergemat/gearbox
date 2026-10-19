use super::{AabbTree, Node};
use crate::{Aabb, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Leaf {
    pub bounds: Aabb,
    pub index: u32,
}

impl AabbTree {
    pub fn build(boxes: &mut [Leaf], leaf_size: u32) -> Self {
        assert!(leaf_size != 0, "tree leaf size must be positive");
        let mut tree = Self {
            nodes: Vec::new(),
            bounds: Aabb::EMPTY,
            leaf_count: boxes.len() as u32,
            depth: 0,
            leaf_size,
        };
        if boxes.is_empty() {
            return tree;
        }
        tree.build_range(boxes, 0);
        let mut i = 0;
        let mut bounds = tree.nodes[0].bounds;
        while i < tree.nodes.len() {
            if i != 0 {
                bounds = box_union(bounds, tree.nodes[i].bounds);
            }
            let word = tree.nodes[i].link;
            i += if word & 1 != 0 { 1 } else { (word >> 1) as usize };
        }
        tree.bounds = bounds;
        tree
    }

    fn build_range(&mut self, boxes: &mut [Leaf], depth: u32) {
        self.depth = self.depth.max(depth + 1);
        if boxes.len() <= self.leaf_size as usize {
            self.nodes.extend(
                boxes.iter().map(|leaf| Node { bounds: leaf.bounds, link: leaf.index.wrapping_mul(2) | 1 }),
            );
            return;
        }
        let split = partition(boxes);
        let index = self.nodes.len();
        let mut lo = boxes[0].bounds.center - boxes[0].bounds.half;
        let mut hi = boxes[0].bounds.half + boxes[0].bounds.center;
        for b in &boxes[1..] {
            for k in 0..3 {
                lo[k] = crate::math::minss(b.bounds.center[k] - b.bounds.half[k], lo[k]);
                hi[k] = crate::math::maxss(b.bounds.center[k] + b.bounds.half[k], hi[k]);
            }
        }
        let bounds = if lo.x <= hi.x {
            Aabb::new(
                Vec3::new((hi.x + lo.x) * 0.5, (lo.y + hi.y) * 0.5, (hi.z + lo.z) * 0.5),
                (hi - lo) * 0.5,
            )
        } else {
            crate::Aabb::EMPTY
        };
        self.nodes.push(Node { bounds, link: 0 });
        self.build_range(&mut boxes[..split], depth + 1);
        self.build_range(&mut boxes[split..], depth + 1);
        self.nodes[index].link = (self.nodes.len() as u32).wrapping_sub(index as u32).wrapping_mul(2);
    }
}

fn partition(boxes: &mut [Leaf]) -> usize {
    let mut sum = Vec3::ZERO;
    for b in boxes.iter() {
        sum += b.bounds.center;
    }
    let mean = sum * (1.0 / boxes.len() as u32 as f32);
    let mut variance = Vec3::ZERO;
    for b in boxes.iter() {
        let d = b.bounds.center - mean;
        for k in 0..3 {
            variance[k] += d[k] * d[k];
        }
    }
    let axis = if variance.y > variance.x {
        1 + usize::from(variance.z > variance.y)
    } else if variance.z > variance.x {
        2
    } else {
        0
    };
    let mut count = 0;
    for i in 0..boxes.len() {
        if boxes[i].bounds.center[axis] > mean[axis] {
            boxes.swap(i, count);
            count += 1;
        }
    }
    let third = boxes.len() / 3;
    if third < count && count < (boxes.len() - third).wrapping_sub(1) { count } else { boxes.len() >> 1 }
}

pub fn box_union(a: Aabb, b: Aabb) -> Aabb {
    if a.half.x < 0.0 {
        return b;
    }
    if !(0.0 <= b.half.x) {
        return a;
    }
    let mut lo = Vec3::ZERO;
    let mut hi = Vec3::ZERO;
    for k in 0..3 {
        lo[k] = crate::math::minss(b.center[k] - b.half[k], a.center[k] - a.half[k]);
        hi[k] = crate::math::maxss(b.half[k] + b.center[k], a.center[k] + a.half[k]);
    }
    Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5)
}
