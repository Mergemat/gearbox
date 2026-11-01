use std::collections::HashMap;

use gearbox_engine::collision::ItemEvents;
use gearbox_engine::{Mat3, Session, Vec3, gameplay};

use crate::observe::surface_class;

pub const DESC: usize = 32;
pub const PIECE: usize = 10;
pub const POSE: usize = 13;
pub const CELL: f32 = 32.0;
pub const REACH: i32 = 31;
pub const COARSE_CELL: f32 = 128.0;
pub const COARSE_REACH: i32 = 15;
pub const WAVES: [f32; 4] = [12.0, 48.0, 192.0, 768.0];
pub const FINE: usize = 4096;
pub const COARSE: usize = 512;
const MAX_SIDE: usize = 1024;
const FLOOR_Y: f32 = 0.7;
const DRIVABLE_AREA: f32 = 4.0;
const CLASSES: usize = 17;

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Grid {
    pub origin: [f32; 2],
    pub size: [usize; 2],
    pub cell: f32,
}

impl Grid {
    fn around(low: [f32; 2], high: [f32; 2], cell: f32) -> Self {
        let origin = low.map(|v| (v / cell).floor() * cell);
        let size =
            std::array::from_fn(|k| (((high[k] - origin[k]) / cell).floor() as usize + 1).clamp(1, MAX_SIDE));
        Self { origin, size, cell }
    }

    pub fn cell_of(&self, x: f32, z: f32) -> [i32; 2] {
        [((x - self.origin[0]) / self.cell).floor() as i32, ((z - self.origin[1]) / self.cell).floor() as i32]
    }

    fn index(&self, x: f32, z: f32) -> usize {
        let [i, k] = self.cell_of(x, z);
        let i = i.clamp(0, self.size[0] as i32 - 1) as usize;
        let k = k.clamp(0, self.size[1] as i32 - 1) as usize;
        k * self.size[0] + i
    }

    fn segment(&self, cum: &[u32], [cx, cz]: [i32; 2], r: i32, j: i32) -> (u32, u32) {
        let z = cz + j;
        let x0 = (cx - r).max(0);
        let x1 = (cx + r).min(self.size[0] as i32 - 1);
        if z < 0 || z >= self.size[1] as i32 || x0 > x1 {
            return (0, 0);
        }
        let row = z as usize * self.size[0];
        let start = cum[row + x0 as usize];
        (start, cum[row + x1 as usize + 1] - start)
    }

    fn count(&self, cum: &[u32], at: [i32; 2], r: i32) -> u32 {
        (-r..=r).map(|j| self.segment(cum, at, r, j).1).sum()
    }
}

/// Every placed piece of the map as a shared shape and a pose, and per 128 m cell a pooled summary row. Shapes come
/// first by placement, then one per coarse cell; pieces are the fine ones (drivable, then the rest, each by cell)
/// then the coarse ones.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Tokens {
    pub keys: Vec<u64>,
    pub descs: Vec<[f32; DESC]>,
    pub shapes: usize,
    pub pieces: Vec<[f32; PIECE]>,
    pub fine: usize,
    pub drivable: usize,
    pub grid: Grid,
    pub coarse_grid: Grid,
    pub drivable_cells: Vec<u32>,
    pub other_cells: Vec<u32>,
    pub coarse_cells: Vec<u32>,
}

pub fn shape_key(triangles: &[gearbox_engine::consumer::PhysicalTriangle]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |b: &[u8]| {
        for &x in b {
            h = (h ^ u64::from(x)).wrapping_mul(0x100_0000_01b3);
        }
    };
    for t in triangles {
        for v in t.v {
            for c in v {
                eat(&((c * 1000.0).round() as i32).to_le_bytes());
            }
        }
        eat(&[t.surface, t.gameplay]);
    }
    h
}

pub fn class_of(surface: u8, play: u8) -> usize {
    match play {
        gameplay::TURBO | gameplay::TURBO_2 | gameplay::TURBO_ROULETTE => 12,
        gameplay::FREE_WHEELING => 13,
        gameplay::NO_GRIP => 14,
        gameplay::NO_STEERING => 15,
        _ => surface_class(surface),
    }
}

#[derive(Clone, Copy, Default)]
struct Mix {
    area: f32,
    facing: [f32; 3],
    classes: [f32; CLASSES + 1],
    normal: [f32; 3],
}

impl Mix {
    fn add(&mut self, other: &Mix) {
        self.area += other.area;
        for k in 0..3 {
            self.facing[k] += other.facing[k];
            self.normal[k] += other.normal[k];
        }
        for k in 0..=CLASSES {
            self.classes[k] += other.classes[k];
        }
    }

    fn write(&self, out: &mut [f32]) {
        let share = 1.0 / self.area.max(1e-6);
        for k in 0..3 {
            out[4 + k] = self.facing[k] * share;
            out[25 + k] = self.normal[k] * share;
        }
        for k in 0..=CLASSES {
            out[7 + k] = self.classes[k] * share;
        }
        out[3] = (self.area).ln_1p() / 8.0;
    }
}

struct Shape {
    key: u64,
    center: Vec3,
    desc: [f32; DESC],
    mix: Mix,
    drivable: bool,
}

fn shape(session: &Session, id: u32) -> Option<Shape> {
    let triangles = session.shape_triangles(id);
    if triangles.is_empty() {
        return None;
    }
    let mut low = [f32::INFINITY; 3];
    let mut high = [f32::NEG_INFINITY; 3];
    for t in &triangles {
        for v in t.v {
            for k in 0..3 {
                low[k] = low[k].min(v[k]);
                high[k] = high[k].max(v[k]);
            }
        }
    }
    let center = Vec3::from(std::array::from_fn(|k| (low[k] + high[k]) * 0.5));
    let mut mix = Mix::default();
    let mut centroid = Vec3::ZERO;
    for t in &triangles {
        let [a, b, c] = t.v.map(Vec3::from);
        let n = cross(b - a, c - a);
        let length = n.dot(n).sqrt();
        if !(length > 1e-9) {
            continue;
        }
        let area = length * 0.5;
        let n = n * (1.0 / length);
        mix.area += area;
        mix.facing[if n.y >= FLOOR_Y {
            0
        } else if n.y <= -FLOOR_Y {
            2
        } else {
            1
        }] += area;
        mix.classes[class_of(t.surface, t.gameplay)] += area;
        if t.gameplay != 0 {
            mix.classes[CLASSES] += area;
        }
        for (k, v) in n.to_array().into_iter().enumerate() {
            mix.normal[k] += v * area;
        }
        centroid += (a + b + c) * (area / 3.0);
    }
    let mut desc = [0.0; DESC];
    let size: [f32; 3] = std::array::from_fn(|k| high[k] - low[k]);
    for k in 0..3 {
        desc[k] = size[k].max(0.0).ln_1p() / 4.0;
    }
    mix.write(&mut desc);
    let reach = (size.iter().fold(0.0f32, |m, &s| m.max(s)) * 0.5).max(1.0);
    let offset = (centroid * (1.0 / mix.area.max(1e-6)) - center) * (1.0 / reach);
    desc[28..31].copy_from_slice(&offset.to_array());
    let drivable = mix.facing[0] >= DRIVABLE_AREA;
    desc[31] = f32::from(drivable);
    Some(Shape { key: shape_key(&triangles), center, desc, mix, drivable })
}

fn cumulative(cells: usize, of: impl Iterator<Item = usize>) -> Vec<u32> {
    let mut cum = vec![0u32; cells + 1];
    for cell in of {
        cum[cell + 1] += 1;
    }
    for i in 0..cells {
        cum[i + 1] += cum[i];
    }
    cum
}

#[derive(Default)]
struct Cell {
    count: f32,
    drivable: f32,
    sum: [f32; 3],
    low: [f32; 3],
    high: [f32; 3],
    mix: Mix,
}

impl Tokens {
    pub fn build(session: &Session) -> Self {
        let mut local: HashMap<u32, Option<usize>> = HashMap::new();
        let mut shapes: Vec<Shape> = Vec::new();
        let mut placed: Vec<(usize, [f32; PIECE])> = Vec::new();
        for item in &session.world.statics.items {
            if item.flags.contains(ItemEvents::CONTACT_TIME) {
                continue;
            }
            let index = *local.entry(item.shape).or_insert_with(|| {
                shape(session, item.shape).map(|s| {
                    shapes.push(s);
                    shapes.len() - 1
                })
            });
            let Some(index) = index else { continue };
            let anchor = item.iso.transform_point(shapes[index].center);
            let [a, b] = [item.iso.rot.col(0), item.iso.rot.col(1)];
            let mut row = [0.0; PIECE];
            row[0] = index as f32;
            row[1..4].copy_from_slice(&anchor.to_array());
            row[4..7].copy_from_slice(&a.to_array());
            row[7..10].copy_from_slice(&b.to_array());
            placed.push((index, row));
        }
        let mut low = [f32::INFINITY; 2];
        let mut high = [f32::NEG_INFINITY; 2];
        for (_, row) in &placed {
            for (k, axis) in [1, 3].into_iter().enumerate() {
                low[k] = low[k].min(row[axis]);
                high[k] = high[k].max(row[axis]);
            }
        }
        if placed.is_empty() {
            low = [0.0; 2];
            high = [0.0; 2];
        }
        let grid = Grid::around(low, high, CELL);
        let coarse_grid = Grid::around(low, high, COARSE_CELL);
        let cells = grid.size[0] * grid.size[1];
        let mut order: Vec<(bool, usize, usize)> = placed
            .iter()
            .enumerate()
            .map(|(i, (s, row))| (!shapes[*s].drivable, grid.index(row[1], row[3]), i))
            .collect();
        order.sort_unstable();
        let drivable = order.iter().filter(|o| !o.0).count();
        let drivable_cells = cumulative(cells, order.iter().filter(|o| !o.0).map(|o| o.1));
        let other_cells = cumulative(cells, order.iter().filter(|o| o.0).map(|o| o.1));
        let mut pieces: Vec<[f32; PIECE]> = order.iter().map(|o| placed[o.2].1).collect();
        let fine = pieces.len();

        let mut pooled: std::collections::BTreeMap<usize, Cell> = Default::default();
        for (s, row) in &placed {
            let cell = pooled.entry(coarse_grid.index(row[1], row[3])).or_insert_with(|| Cell {
                low: [f32::INFINITY; 3],
                high: [f32::NEG_INFINITY; 3],
                ..Cell::default()
            });
            let shape = &shapes[*s];
            cell.count += 1.0;
            cell.drivable += f32::from(shape.drivable);
            for k in 0..3 {
                cell.sum[k] += row[1 + k];
                cell.low[k] = cell.low[k].min(row[1 + k]);
                cell.high[k] = cell.high[k].max(row[1 + k]);
            }
            cell.mix.add(&shape.mix);
        }
        let mut keys: Vec<u64> = shapes.iter().map(|s| s.key).collect();
        let mut descs: Vec<[f32; DESC]> = shapes.iter().map(|s| s.desc).collect();
        let first = shapes.len();
        for cell in pooled.values() {
            let mut desc = [0.0; DESC];
            for k in 0..3 {
                desc[k] = (cell.high[k] - cell.low[k]).max(0.0).ln_1p() / 4.0;
            }
            cell.mix.write(&mut desc);
            desc[28] = cell.count.ln_1p() / 8.0;
            desc[29] = cell.drivable.ln_1p() / 8.0;
            desc[30] = (cell.high[1] - cell.low[1]) / 64.0;
            desc[31] = cell.drivable / cell.count;
            let mut row = [0.0; PIECE];
            row[0] = descs.len() as f32;
            for k in 0..3 {
                row[1 + k] = cell.sum[k] / cell.count;
            }
            row[4] = 1.0;
            row[8] = 1.0;
            keys.push(0);
            descs.push(desc);
            pieces.push(row);
        }
        let coarse_cells = cumulative(coarse_grid.size[0] * coarse_grid.size[1], pooled.keys().copied());
        Self {
            keys,
            descs,
            shapes: first,
            pieces,
            fine,
            drivable,
            grid,
            coarse_grid,
            drivable_cells,
            other_cells,
            coarse_cells,
        }
    }

    pub fn bytes(&self) -> usize {
        self.keys.len() * (8 + 4 * DESC)
            + self.pieces.len() * 4 * PIECE
            + 4 * (self.drivable_cells.len() + self.other_cells.len() + self.coarse_cells.len())
    }

    /// The pieces around `position` within `fine` and `coarse` budgets, as rows of `pieces`: the drivable and other
    /// fine pieces of the widest square of cells that fits, the drivable ones out to a wider square while they fit,
    /// then the coarse cells of the widest square that fits.
    pub fn select(&self, position: [f32; 3], fine: usize, coarse: usize, out: &mut Vec<u32>) {
        out.clear();
        let g = &self.grid;
        let at = g.cell_of(position[0], position[2]);
        let (d, o) = (&self.drivable_cells, &self.other_cells);
        let fits = |r: i32| (g.count(d, at, r) + g.count(o, at, r)) as usize <= fine;
        let mut both = 0;
        while both < REACH && fits(both + 1) {
            both += 1;
        }
        let rest = g.count(o, at, both) as usize;
        let mut wide = both;
        while wide < REACH && g.count(d, at, wide + 1) as usize + rest <= fine {
            wide += 1;
        }
        let mut left = fine;
        let take = |start: u32, count: u32, base: usize, left: &mut usize, out: &mut Vec<u32>| {
            let n = (count as usize).min(*left);
            *left -= n;
            out.extend((0..n as u32).map(|k| (base as u32) + start + k));
        };
        for j in -wide..=wide {
            let (start, count) = g.segment(d, at, wide, j);
            take(start, count, 0, &mut left, out);
        }
        for j in -both..=both {
            let (start, count) = g.segment(o, at, both, j);
            take(start, count, self.drivable, &mut left, out);
        }
        let c = &self.coarse_grid;
        let at = c.cell_of(position[0], position[2]);
        let cum = &self.coarse_cells;
        let mut reach = 0;
        while reach < COARSE_REACH && c.count(cum, at, reach + 1) as usize <= coarse {
            reach += 1;
        }
        let mut left = coarse;
        for j in -reach..=reach {
            let (start, count) = c.segment(cum, at, reach, j);
            take(start, count, self.fine, &mut left, out);
        }
    }
}

/// A piece's pose features from the car (position, rotation rows): its offset in metres in the car's frame, then
/// `POSE` features: the offset / 256, its signed log, the log distance and the piece's first two axes in the car's
/// frame.
pub fn relative(
    piece: &[f32; PIECE],
    position: [f32; 3],
    rotation: &Mat3,
    offset: &mut [f32; 3],
    out: &mut [f32; POSE],
) {
    let delta = Vec3::new(piece[1] - position[0], piece[2] - position[1], piece[3] - position[2]);
    let p = rotation.transpose_mul_vec(delta).to_array();
    let a = rotation.transpose_mul_vec(Vec3::new(piece[4], piece[5], piece[6])).to_array();
    let b = rotation.transpose_mul_vec(Vec3::new(piece[7], piece[8], piece[9])).to_array();
    *offset = p;
    let distance = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
    for k in 0..3 {
        out[k] = p[k] / 256.0;
        out[3 + k] = p[k].signum() * (p[k].abs() / 4.0).ln_1p() / 4.0;
        out[7 + k] = a[k];
        out[10 + k] = b[k];
    }
    out[6] = (distance / 4.0).ln_1p() / 4.0;
}

/// The rotary angle of pair `j` of a head for an offset: axis j mod 3, wavelength WAVES[(j / 3) mod 4].
pub fn angle(offset: &[f32; 3], j: usize) -> f32 {
    offset[j % 3] * (std::f32::consts::TAU / WAVES[(j / 3) % WAVES.len()])
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}
