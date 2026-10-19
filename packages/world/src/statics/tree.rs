use gearbox_engine::math::{maxss, minss};
use gearbox_engine::{Aabb, Vec3, arrays::TreeNode, static_world::Tree};

#[derive(Clone, Copy)]
pub struct Leaf {
    pub bounds: Aabb,
    pub index: u32,
}

fn split_axis(boxes: &[Leaf]) -> (usize, Vec3) {
    let mut sum = Vec3::ZERO;
    for b in boxes {
        sum += b.bounds.center;
    }
    let mean = sum * (1.0 / boxes.len() as u32 as u64 as f32);
    let mut variance = Vec3::ZERO;
    for b in boxes {
        let d = b.bounds.center - mean;
        for k in 0..3 {
            variance[k] += d[k] * d[k];
        }
    }
    let axis = if variance.y > variance.x {
        1 + (variance.z > variance.y) as usize
    } else if variance.z > variance.x {
        2
    } else {
        0
    };
    (axis, mean)
}

fn partition(boxes: &mut [Leaf]) -> usize {
    let (axis, mean) = split_axis(boxes);
    let mut split = 0;
    for i in 0..boxes.len() {
        if boxes[i].bounds.center[axis] > mean[axis] {
            boxes.swap(i, split);
            split += 1;
        }
    }
    let third = boxes.len() / 3;
    if third < split && split < (boxes.len() - third).wrapping_sub(1) { split } else { boxes.len() >> 1 }
}

fn union(a: Aabb, b: Aabb) -> Aabb {
    if a.half.x < 0.0 {
        return b;
    }
    if !(0.0 <= b.half.x) {
        return a;
    }
    let lo =
        Vec3::from_array(std::array::from_fn(|k| minss(b.center[k] - b.half[k], a.center[k] - a.half[k])));
    let hi =
        Vec3::from_array(std::array::from_fn(|k| maxss(b.half[k] + b.center[k], a.center[k] + a.half[k])));
    Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5)
}

fn build_range(tree: &mut Tree, boxes: &mut [Leaf], depth: u32) {
    tree.depth = tree.depth.max(depth + 1);
    if boxes.len() <= tree.leaf_size as usize {
        tree.nodes.extend(
            boxes.iter().map(|leaf| TreeNode { bounds: leaf.bounds, link: leaf.index.wrapping_mul(2) | 1 }),
        );
        return;
    }
    let split = partition(boxes);
    let first = boxes[0].bounds;
    let mut lo = first.center - first.half;
    let mut hi = first.half + first.center;
    for b in &boxes[1..] {
        for k in 0..3 {
            lo[k] = minss(b.bounds.center[k] - b.bounds.half[k], lo[k]);
            hi[k] = maxss(b.bounds.center[k] + b.bounds.half[k], hi[k]);
        }
    }
    let bounds =
        if lo.x <= hi.x { Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5) } else { Tree::default().bounds };
    let index = tree.nodes.len();
    tree.nodes.push(TreeNode { bounds, link: 0 });
    let (left, right) = boxes.split_at_mut(split);
    build_range(tree, left, depth + 1);
    build_range(tree, right, depth + 1);
    tree.nodes[index].link = ((tree.nodes.len() - index) as u32).wrapping_mul(2);
}

pub fn build(boxes: &mut [Leaf], leaf_size: u32) -> Tree {
    let mut tree = Tree { leaf_size, leaves: boxes.len() as u32, ..Tree::default() };
    if boxes.is_empty() {
        return tree;
    }
    build_range(&mut tree, boxes, 0);
    tree.bounds = tree.nodes[0].bounds;
    let skip = |word: u32| if word & 1 != 0 { 1 } else { (word >> 1) as usize };
    let mut i = skip(tree.nodes[0].link);
    while i < tree.nodes.len() {
        tree.bounds = union(tree.bounds, tree.nodes[i].bounds);
        i += skip(tree.nodes[i].link);
    }
    tree
}
