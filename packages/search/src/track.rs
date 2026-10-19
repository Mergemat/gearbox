use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::sync::Arc;

pub const CLIMB_COST: f32 = 8.0;

const CLIMBABLE_NY: f32 = 0.33;

pub struct Cells {
    cell: f32,
    lo: [i32; 3],
    dims: [i32; 3],
    grid: Vec<u32>,
    keys: Vec<[i32; 3]>,
    climb: Vec<bool>,
}

impl Cells {
    pub fn build(tris: impl Iterator<Item = [[f32; 3]; 3]>, cell: f32, clearance: i32) -> Self {
        let key = |p: [f32; 3]| p.map(|v| (v / cell).floor() as i32);
        let mut surface: HashMap<[i32; 3], bool> = HashMap::new();
        for t in tris {
            let e1 = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
            let e2 = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
            let len = |e: [f32; 3]| (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt();
            let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
            let climbable = len(n) > 0.0 && (n[1] / len(n)).abs() >= CLIMBABLE_NY;
            let n = ((len(e1).max(len(e2)) / (cell * 0.5)).ceil() as usize).clamp(1, 400);
            for i in 0..=n {
                for j in 0..=(n - i) {
                    let (a, b) = (i as f32 / n as f32, j as f32 / n as f32);
                    let c = surface.entry(key(std::array::from_fn(|k| t[0][k] + a * e1[k] + b * e2[k]))).or_insert(false);
                    *c |= climbable;
                }
            }
        }
        let (mut lo, mut hi) = ([i32::MAX; 3], [i32::MIN; 3]);
        for s in surface.keys() {
            for a in 0..3 {
                lo[a] = lo[a].min(s[a]);
                hi[a] = hi[a].max(s[a] + if a == 1 { clearance } else { 0 });
            }
        }
        if surface.is_empty() {
            return Self { cell, lo: [0; 3], dims: [0; 3], grid: Vec::new(), keys: Vec::new(), climb: Vec::new() };
        }
        let dims = [hi[0] - lo[0] + 1, hi[1] - lo[1] + 1, hi[2] - lo[2] + 1];
        let mut cells = Self { cell, lo, dims, grid: vec![0; (dims[0] * dims[1] * dims[2]) as usize], keys: Vec::new(), climb: Vec::new() };
        let mut keys: Vec<[i32; 3]> = surface.keys().copied().collect();
        keys.sort_unstable();
        for s in keys {
            for up in 0..=clearance {
                let k = [s[0], s[1] + up, s[2]];
                let slot = cells.slot(k).expect("inside the bounds");
                if cells.grid[slot] == 0 {
                    cells.keys.push(k);
                    cells.climb.push(false);
                    cells.grid[slot] = cells.keys.len() as u32;
                }
            }
        }
        for (i, k) in cells.keys.iter().enumerate() {
            cells.climb[i] = surface.get(k).copied().unwrap_or(false);
        }
        cells
    }

    fn slot(&self, k: [i32; 3]) -> Option<usize> {
        let r = [k[0] - self.lo[0], k[1] - self.lo[1], k[2] - self.lo[2]];
        ((0..3).all(|a| (0..self.dims[a]).contains(&r[a]))).then(|| ((r[0] * self.dims[1] + r[1]) * self.dims[2] + r[2]) as usize)
    }

    fn get(&self, k: [i32; 3]) -> Option<u32> {
        self.slot(k).and_then(|s| self.grid[s].checked_sub(1))
    }

    pub fn len(&self) -> usize {
        self.keys.len()
    }

    fn center(&self, k: [i32; 3]) -> [f32; 3] {
        k.map(|v| (v as f32 + 0.5) * self.cell)
    }

    fn key(&self, p: [f32; 3]) -> [i32; 3] {
        p.map(|v| (v / self.cell).floor() as i32)
    }
}

pub struct GoalField {
    cells: Arc<Cells>,
    dist: Vec<f32>,
}

#[derive(PartialEq)]
struct Item(f32, u32);
impl Eq for Item {}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then_with(|| o.1.cmp(&self.1))
    }
}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

impl GoalField {
    pub fn build(cells: Arc<Cells>, center: [f32; 3], inside: impl Fn([f32; 3]) -> bool) -> Self {
        let mut dist = vec![f32::INFINITY; cells.len()];
        let mut heap = BinaryHeap::new();
        for (i, k) in cells.keys.iter().enumerate() {
            if inside(cells.center(*k)) {
                dist[i] = 0.0;
                heap.push(Item(0.0, i as u32));
            }
        }
        if heap.is_empty() {
            let d = |k: &[i32; 3]| {
                let c = cells.center(*k);
                (0..3).map(|a| (c[a] - center[a]).powi(2)).sum::<f32>()
            };
            if let Some((i, _)) = cells.keys.iter().enumerate().min_by(|a, b| d(a.1).total_cmp(&d(b.1))) {
                dist[i] = d(&cells.keys[i]).sqrt();
                heap.push(Item(dist[i], i as u32));
            }
        }
        while let Some(Item(d, i)) = heap.pop() {
            if d > dist[i as usize] {
                continue;
            }
            let k = cells.keys[i as usize];
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let steps = dx * dx + dy * dy + dz * dz;
                        if steps == 0 {
                            continue;
                        }
                        let Some(j) = cells.get([k[0] + dx, k[1] + dy, k[2] + dz]) else { continue };
                        let climb = if dy < 0 && !cells.climb[i as usize] { CLIMB_COST } else { 1.0 };
                        let nd = d + cells.cell * (steps as f32).sqrt() * climb;
                        if nd < dist[j as usize] {
                            dist[j as usize] = nd;
                            heap.push(Item(nd, j));
                        }
                    }
                }
            }
        }
        Self { cells, dist }
    }

    pub fn distance(&self, p: [f32; 3]) -> Option<f64> {
        self.distance_within(p, 1)
    }

    pub fn distance_within(&self, p: [f32; 3], reach: i32) -> Option<f64> {
        let c = &self.cells;
        let k = c.key(p);
        let mut best: Option<f64> = None;
        for r in 0..=reach {
            for dx in -r..=r {
                for dy in -r..=r {
                    for dz in -r..=r {
                        if dx.abs().max(dy.abs()).max(dz.abs()) != r {
                            continue;
                        }
                        let n = [k[0] + dx, k[1] + dy, k[2] + dz];
                        let Some(i) = c.get(n) else { continue };
                        let d = self.dist[i as usize];
                        if !d.is_finite() {
                            continue;
                        }
                        let ctr = c.center(n);
                        let off = (0..3).map(|a| f64::from(p[a] - ctr[a]).powi(2)).sum::<f64>().sqrt();
                        let v = f64::from(d) + off;
                        best = Some(best.map_or(v, |b| b.min(v)));
                    }
                }
            }
            if best.is_some() {
                break;
            }
        }
        best
    }
}

pub struct TrackFields {
    pub goals: Vec<Option<GoalField>>,
    pub reach: i32,
}

impl Default for TrackFields {
    fn default() -> Self {
        Self { goals: Vec::new(), reach: 1 }
    }
}

impl std::fmt::Debug for TrackFields {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TrackFields({} goals)", self.goals.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn u_track() -> Vec<[[f32; 3]; 3]> {
        let quad = |x0: f32, x1: f32, z0: f32, z1: f32| [[[x0, 0.0, z0], [x1, 0.0, z0], [x1, 0.0, z1]], [[x0, 0.0, z0], [x1, 0.0, z1], [x0, 0.0, z1]]];
        let mut t = Vec::new();
        t.extend(quad(0.0, 10.0, 0.0, 100.0));
        t.extend(quad(40.0, 50.0, 0.0, 100.0));
        t.extend(quad(0.0, 50.0, 90.0, 100.0));
        t
    }

    fn platform(ramp: bool) -> Vec<[[f32; 3]; 3]> {
        let quad = |a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3]| [[a, b, c], [a, c, d]];
        let mut t = Vec::new();
        t.extend(quad([-20.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 0.0, 20.0], [-20.0, 0.0, 20.0]));
        t.extend(quad([40.0, 10.0, 0.0], [60.0, 10.0, 0.0], [60.0, 10.0, 20.0], [40.0, 10.0, 20.0]));
        t.extend(quad([40.0, 0.0, 0.0], [40.0, 10.0, 0.0], [40.0, 10.0, 20.0], [40.0, 0.0, 20.0]));
        if ramp {
            t.extend(quad([10.0, 0.0, 0.0], [40.0, 10.0, 0.0], [40.0, 10.0, 20.0], [10.0, 0.0, 20.0]));
        }
        t
    }

    #[test]
    fn a_sheer_wall_cannot_be_climbed_but_a_ramp_can() {
        let goal = |p: [f32; 3]| p[0] > 50.0 && p[1] > 9.0;
        let before = [5.0, 1.0, 10.0];
        let walled = GoalField::build(Arc::new(Cells::build(platform(false).into_iter(), 2.0, 6)), [55.0, 11.0, 10.0], goal);
        let ramped = GoalField::build(Arc::new(Cells::build(platform(true).into_iter(), 2.0, 6)), [55.0, 11.0, 10.0], goal);
        let (w, r) = (walled.distance(before).unwrap(), ramped.distance(before).unwrap());
        assert!(r < 70.0, "{r}");
        assert!(w > r + 40.0, "{w} vs {r}");
        assert!(walled.distance([45.0, 11.0, 10.0]).unwrap() < 10.0);
    }

    #[test]
    fn misses_are_measured_around_the_gap_not_across_it() {
        let cells = Arc::new(Cells::build(u_track().into_iter(), 2.0, 2));
        let f = GoalField::build(cells, [45.0, 0.5, 5.0], |p| (40.0..=50.0).contains(&p[0]) && p[2] < 10.0);
        let left = f.distance([5.0, 0.5, 5.0]).unwrap();
        assert!(left > 150.0, "{left}");
        assert!(f.distance([25.0, 0.5, 95.0]).unwrap() < left);
        assert!(f.distance([25.0, 0.5, 5.0]).is_none());
        assert!(f.distance_within([25.0, 0.5, 5.0], 8).unwrap() > 15.0);
    }
}
