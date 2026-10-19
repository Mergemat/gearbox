use crate::{
    Aabb, Vec3,
    math::{maxss, minss},
};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Bounds {
    pub min: Vec3,
    pub max: Vec3,
}

impl Bounds {
    pub fn from_aabb(bounds: Aabb) -> Self {
        Self { min: bounds.center - bounds.half, max: bounds.half + bounds.center }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RadixSorter {
    pub size: u32,
    pub ranks: Vec<u32>,
    pub spare: Vec<u32>,
    pub calls: u32,
    pub hits: u32,
}

impl RadixSorter {
    pub fn sort(&mut self, keys: &[f32]) {
        let n = keys.len() as u32;
        if n == 0 || n & 0x8000_0000 != 0 {
            return;
        }
        self.calls = self.calls.wrapping_add(1);
        let old = self.size & 0x7fff_ffff;
        if n != old {
            if n > old {
                self.ranks = vec![0; n as usize];
                self.spare = vec![0; n as usize];
            }
            self.size = n | 0x8000_0000;
        }
        let invalid = self.size & 0x8000_0000 != 0;
        let byte = |i: usize, b: usize| keys[i].to_bits().to_le_bytes()[b] as usize;
        let mut hist = [[0u32; 256]; 4];
        let mut p = 0usize;
        let mut sorted = true;
        let first = if invalid { 0 } else { self.ranks[0] as usize };
        let mut prev = keys[first];
        while p < keys.len() {
            let index = if invalid { p } else { self.ranks[p] as usize };
            let v = keys[index];
            if prev > v {
                sorted = false;
                break;
            }
            for (b, h) in hist.iter_mut().enumerate() {
                h[byte(p, b)] += 1;
            }
            prev = v;
            p += 1;
        }
        if sorted {
            self.hits = self.hits.wrapping_add(1);
            if invalid {
                for i in 0..n {
                    self.ranks[i as usize] = i;
                }
            }
            return;
        }
        while p < keys.len() {
            for (b, h) in hist.iter_mut().enumerate() {
                h[byte(p, b)] += 1;
            }
            p += 1;
        }
        for (j, h) in hist.iter().enumerate() {
            let first = byte(0, j);
            if j != 3 {
                if h[first] == n {
                    continue;
                }
                let mut link = [0u32; 256];
                for b in 1..256 {
                    link[b] = link[b - 1].wrapping_add(h[b - 1]);
                }
                let invalid = self.size & 0x8000_0000 != 0;
                for i in 0..keys.len() {
                    let id = if invalid { i as u32 } else { self.ranks[i] };
                    let b = byte(id as usize, j);
                    self.spare[link[b] as usize] = id;
                    link[b] = link[b].wrapping_add(1);
                }
                if invalid {
                    self.size &= 0x7fff_ffff;
                }
                std::mem::swap(&mut self.ranks, &mut self.spare);
            } else if h[first] != n {
                let negative = h[128..].iter().fold(0u32, |a, &b| a.wrapping_add(b));
                let mut link = [0u32; 256];
                link[0] = negative;
                for b in 1..128 {
                    link[b] = link[b - 1].wrapping_add(h[b - 1]);
                }
                for r in 0..127 {
                    link[254 - r] = h[255 - r].wrapping_add(link[255 - r]);
                }
                for b in 128..256 {
                    link[b] = link[b].wrapping_add(h[b]);
                }
                let invalid = self.size & 0x8000_0000 != 0;
                for i in 0..keys.len() {
                    let id = if invalid { i as u32 } else { self.ranks[i] };
                    let b = byte(id as usize, 3);
                    if b < 128 {
                        self.spare[link[b] as usize] = id;
                        link[b] = link[b].wrapping_add(1);
                    } else {
                        link[b] = link[b].wrapping_sub(1);
                        self.spare[link[b] as usize] = id;
                    }
                }
                if invalid {
                    self.size &= 0x7fff_ffff;
                }
                std::mem::swap(&mut self.ranks, &mut self.spare);
            } else if first >= 128 {
                let invalid = (self.size as i32) < 0;
                for i in 0..n {
                    self.spare[i as usize] =
                        if invalid { n - i - 1 } else { self.ranks[(n - i - 1) as usize] };
                }
                if invalid {
                    self.size &= 0x7fff_ffff;
                }
                std::mem::swap(&mut self.ranks, &mut self.spare);
            }
        }
    }
}

pub fn union_box(a: &mut Aabb, b: Aabb) {
    if 0.0 > a.half.x {
        *a = b;
        return;
    }
    if 0.0 > b.half.x {
        return;
    }
    let axis = |ac: f32, ah: f32, bc: f32, bh: f32| {
        let lo = minss(bc - bh, ac - ah);
        let hi = maxss(bh + bc, ac + ah);
        ((hi + lo) * 0.5, (hi - lo) * 0.5)
    };
    let x = axis(a.center.x, a.half.x, b.center.x, b.half.x);
    let y = axis(a.center.y, a.half.y, b.center.y, b.half.y);
    let z = axis(a.center.z, a.half.z, b.center.z, b.half.z);
    *a = Aabb::new(Vec3::new(x.0, y.0, z.0), Vec3::new(x.1, y.1, z.1));
}

#[derive(Clone, Debug, Default)]
struct Group {
    keys: Vec<(u32, u32)>,
    yz: Vec<[f32; 4]>,
    ids: Vec<u32>,
}

fn query_lanes(b: [f32; 4]) -> [f32; 4] {
    [b[2] * -1.0, b[3] * -1.0, b[0] * -1.0, b[1] * -1.0]
}
fn yz_overlap(b: [f32; 4], q: [f32; 4]) -> bool {
    (0..4).all(|i| !(b[i] < q[i]))
}

fn complete(group: &Group, pairs: &mut Vec<(u32, u32)>) {
    let n = group.ids.len();
    let mut running = 0;
    for i in 0..n {
        let min = group.keys[i].0;
        loop {
            let k = running;
            running += 1;
            if group.keys[k].0 >= min {
                break;
            }
        }
        let max = group.keys[i].1;
        let query = query_lanes(group.yz[i]);
        let mut j = running;
        while group.keys[j].0 <= max {
            if yz_overlap(group.yz[j], query) {
                pairs.push((group.ids[i], group.ids[j]));
            }
            j += 1;
        }
        if running >= n {
            break;
        }
    }
}

fn bipartite(strict: bool, a: &Group, b: &Group, pairs: &mut Vec<(u32, u32)>) {
    if b.ids.is_empty() {
        return;
    }
    let mut running = 0;
    for i in 0..a.ids.len() {
        let min = a.keys[i].0;
        loop {
            let k = b.keys[running].0;
            let beyond_min = if strict { k >= min } else { k > min };
            if beyond_min {
                break;
            }
            running += 1;
        }
        let max = a.keys[i].1;
        let query = query_lanes(a.yz[i]);
        let mut j = running;
        while b.keys[j].0 <= max {
            if yz_overlap(b.yz[j], query) {
                pairs.push((a.ids[i], b.ids[j]));
            }
            j += 1;
        }
        if running >= b.ids.len() {
            break;
        }
    }
}

pub fn box_pruning(sorter: &mut RadixSorter, boxes: &[Bounds]) -> Vec<(u32, u32)> {
    let Some(first) = boxes.first() else {
        return Vec::new();
    };
    let mut lo = [first.min.y, first.min.z];
    let mut hi = [first.max.y, first.max.z];
    let mut keys = Vec::with_capacity(boxes.len() + 1);
    keys.push(first.min.x);
    for b in &boxes[1..] {
        keys.push(b.min.x);
        lo = [minss(lo[0], b.min.y), minss(lo[1], b.min.z)];
        hi = [maxss(hi[0], b.max.y), maxss(hi[1], b.max.z)];
    }
    keys.push(f32::MAX);
    sorter.sort(&keys);
    let cy = (lo[0] + hi[0]) * 0.5;
    let cz = (lo[1] + hi[1]) * 0.5;
    let mut group_ids = Vec::with_capacity(boxes.len());
    for b in boxes {
        let code = ((cz > b.max.z) as usize) << 3
            | ((b.min.z > cz) as usize) << 2
            | ((cy > b.max.y) as usize) << 1
            | (b.min.y > cy) as usize;
        group_ids.push(SPLIT_GROUP[code] as usize);
    }
    let mut groups: [Group; 5] = std::array::from_fn(|_| Group::default());
    let encode = |v: f32| {
        let bits = v.to_bits();
        bits ^ (((bits as i32) >> 31) as u32 | 0x8000_0000)
    };
    for &rank in &sorter.ranks[..boxes.len()] {
        let b = boxes[rank as usize];
        let g = &mut groups[group_ids[rank as usize]];
        g.keys.push((encode(b.min.x + 0.0), encode(b.max.x + 0.0)));
        g.yz.push([-b.min.y, -b.min.z, b.max.y, b.max.z]);
        g.ids.push(rank);
    }
    for g in &mut groups {
        g.keys.extend([(u32::MAX, 0); 6]);
    }
    let mut pairs = Vec::new();
    for g in &groups {
        complete(g, &mut pairs);
    }
    for g in 0..4 {
        if groups[g].ids.is_empty() || groups[4].ids.is_empty() {
            continue;
        }
        bipartite(true, &groups[g], &groups[4], &mut pairs);
        bipartite(false, &groups[4], &groups[g], &mut pairs);
    }
    pairs
}

const SPLIT_GROUP: [u8; 16] = [4, 4, 4, 0xff, 4, 3, 2, 0xff, 4, 1, 0, 0xff, 0xff, 0xff, 0xff, 0xff];
