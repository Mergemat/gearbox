//! Observation v12's dots: points on the map's collision surfaces around the car, dense near it and sparser with
//! distance out to the whole course. Each level keeps one point per voxel and facing (up, side, down): the point of
//! the voxel's most central triangle nearest its centre. The near levels are built per tile from the engine's
//! collision trees the first time a car comes near; a map's surfaces are millions of triangles, far too many to
//! voxelise finely up front. The far levels are built once per map over the course.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use gearbox_engine::collision::ItemEvents;
use gearbox_engine::consumer::PhysicalTriangle;
use gearbox_engine::race::Race;
use gearbox_engine::static_world::{Geometry, StaticWorld};
use gearbox_engine::tree::AabbQuery;
use gearbox_engine::{Aabb, Session, Vec3, gameplay};

use crate::ground::{Ground, MIN_NORMAL_Y};
use crate::observe::{Car, Course, passed, surface_class};

pub const FEATURES: usize = 10;
pub const DOTS: usize = 512;
pub const CHECKPOINT: u8 = 17;
pub const FINISH: u8 = 18;
pub const CLASSES: usize = 19;
pub const STEPS: f32 = crate::atlas::STEPS;
/// Voxel sizes in metres, one level per shell. The first `NEAR` build per tile as cars reach them; the rest once per
/// map, those from `COLUMNS` on in tiles that are whole columns.
pub const LEVELS: [f32; 7] = [1.0, 3.0, 8.0, 32.0, 96.0, 192.0, 384.0];
/// Each level's tile side in metres, about half its shell's radius: a car reads few dots outside its shells.
const TILES: [f32; 7] = [8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 2048.0];
const NEAR: usize = 2;
const COLUMNS: usize = 3;
/// Distance shells: outer radius and share of the surface dots. Each reads its own level, sized to offer one to two
/// times its share: thinning a few spread dots keeps them spread. Dots a shell can't fill pass outward, and what the
/// far shells leave (a small map has nothing far) goes back to the inner ones.
pub const SHELLS: [(f32, f32); 7] = [
    (6.0, 0.25),
    (16.0, 0.22),
    (40.0, 0.18),
    (100.0, 0.13),
    (250.0, 0.10),
    (600.0, 0.07),
    (f32::INFINITY, 0.05),
];
/// One dot in this many shows a checkpoint or finish gate (nearest first); surfaces take the rest.
const GATE_SHARE: usize = 16;
const UP: u8 = 0;
const SIDE: u8 = 1;
const DOWN: u8 = 2;
const STRATA: usize = 17 * 3;
/// Metres; far above the rounding in a dot's distance from its voxel's centre on any map.
const SLACK: f32 = 0.02;

/// A point on a surface: its normal ×127 and `kind`, the surface class ×4 plus its facing (up, side, down).
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Dot {
    pub position: [f32; 3],
    pub normal: [i8; 3],
    pub kind: u8,
}

impl Dot {
    pub fn class(&self) -> u8 {
        self.kind >> 2
    }

    fn facing(&self) -> u8 {
        self.kind & 3
    }

    fn stratum(&self) -> usize {
        usize::from(self.class().min(16)) * 3 + usize::from(self.facing())
    }

    fn normal(&self) -> Vec3 {
        Vec3::new(f32::from(self.normal[0]), f32::from(self.normal[1]), f32::from(self.normal[2]))
    }

    fn bits(&self) -> ([u32; 3], [i8; 3], u8) {
        (self.position.map(f32::to_bits), self.normal, self.kind)
    }

    /// The order a shell thins its strata in: a hash of where the dot is, so it holds as the car moves.
    fn key(&self) -> u32 {
        let mut h = u64::from(self.kind);
        for v in self.position.map(f32::to_bits) {
            h = (h ^ u64::from(v)).wrapping_mul(0x9e37_79b9_7f4a_7c15);
            h ^= h >> 29;
        }
        (h >> 32) as u32
    }
}

type Tile = Arc<Block>;

/// A tile's dots, and the box around each run of `RUN` of them in order, so a read skips the runs outside its shell.
#[derive(Debug, Default, PartialEq)]
struct Block {
    dots: Vec<Dot>,
    runs: Vec<[[f32; 3]; 2]>,
}

const RUN: usize = 16;

impl From<Vec<Dot>> for Block {
    fn from(dots: Vec<Dot>) -> Self {
        let runs = dots
            .chunks(RUN)
            .map(|run| {
                let mut bounds = [[f32::INFINITY; 3], [f32::NEG_INFINITY; 3]];
                for dot in run {
                    for k in 0..3 {
                        bounds[0][k] = bounds[0][k].min(dot.position[k]);
                        bounds[1][k] = bounds[1][k].max(dot.position[k]);
                    }
                }
                bounds
            })
            .collect();
        Self { dots, runs }
    }
}
type Fast = std::hash::BuildHasherDefault<Fx>;

/// A multiply-rotate hash for tile keys: std's default costs more than the lookup it serves, every decision.
#[derive(Default)]
struct Fx(u64);

impl std::hash::Hasher for Fx {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut word = [0; 8];
            word[..chunk.len()].copy_from_slice(chunk);
            self.write_u64(u64::from_le_bytes(word));
        }
    }

    fn write_u64(&mut self, v: u64) {
        self.0 = (self.0.rotate_left(5) ^ v).wrapping_mul(0x517c_c1b7_2722_0a95);
    }

    fn write_u32(&mut self, v: u32) {
        self.write_u64(u64::from(v));
    }

    fn write_i32(&mut self, v: i32) {
        self.write_u64(u64::from(v as u32));
    }

    fn write_usize(&mut self, v: usize) {
        self.write_u64(v as u64);
    }
}

/// A map's dots: the far levels whole, the near levels' tiles as cars reach them.
#[derive(Debug, Default)]
pub struct Cloud {
    far: Vec<HashMap<[i32; 3], Tile, Fast>>,
    near: Mutex<HashMap<(usize, [i32; 3]), Tile, Fast>>,
}

impl Clone for Cloud {
    fn clone(&self) -> Self {
        Self { far: self.far.clone(), near: Mutex::default() }
    }
}

impl PartialEq for Cloud {
    fn eq(&self, other: &Self) -> bool {
        self.far == other.far
    }
}

impl serde::Serialize for Cloud {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let levels: Vec<Vec<([i32; 3], &[Dot])>> = self
            .far
            .iter()
            .map(|tiles| {
                let mut list: Vec<_> = tiles.iter().map(|(k, v)| (*k, &v.dots[..])).collect();
                list.sort_by_key(|e| e.0);
                list
            })
            .collect();
        levels.serialize(serializer)
    }
}

impl<'de> serde::Deserialize<'de> for Cloud {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let levels: Vec<Vec<([i32; 3], Vec<Dot>)>> = serde::Deserialize::deserialize(deserializer)?;
        let far = levels
            .into_iter()
            .map(|l| l.into_iter().map(|(k, v)| (k, Tile::new(v.into()))).collect())
            .collect();
        Ok(Self { far, near: Mutex::default() })
    }
}

impl Cloud {
    /// The far levels over the ground's area (the course and its margin), from every static triangle.
    pub fn build(session: &Session, ground: &Ground) -> Self {
        let [x, z] = ground.origin;
        let [w, d] = ground.size.map(|n| n as f32 * crate::ground::CELL);
        let area = [[x, f32::NEG_INFINITY, z], [x + w, f32::INFINITY, z + d]];
        Self::from_triangles(&session.static_triangles(), area)
    }

    /// The far levels from `triangles`, keeping voxels in `area` (low and high corners).
    pub fn from_triangles(triangles: &[PhysicalTriangle], area: [[f32; 3]; 2]) -> Self {
        let mut best = Best::default();
        for triangle in triangles {
            add(&mut best, &triangle.v, triangle.surface, triangle.gameplay, LEVELS[NEAR], area);
        }
        let mut level: Vec<Dot> = best.into_dots();
        let mut far = vec![tiles(&level, NEAR)];
        for (index, &size) in LEVELS.iter().enumerate().skip(NEAR + 1) {
            level = coarser(&level, size);
            far.push(tiles(&level, index));
        }
        Self { far, near: Mutex::default() }
    }

    pub fn bytes(&self) -> usize {
        self.far_bytes()
            + self.near.lock().unwrap().values().map(|t| t.dots.len()).sum::<usize>() * size_of::<Dot>()
    }

    /// The bytes of the levels built once per map.
    pub fn far_bytes(&self) -> usize {
        self.far.iter().flat_map(|tiles| tiles.values()).map(|t| t.dots.len()).sum::<usize>()
            * size_of::<Dot>()
    }

    /// The near tiles built so far.
    pub fn near_tiles(&self) -> usize {
        self.near.lock().unwrap().len()
    }

    /// The level's tiles that may hold dots between `inner` and `outer` of `center`, into `found`.
    fn tiles_around(
        &self,
        statics: &StaticWorld,
        level: usize,
        center: [f32; 3],
        [inner, outer]: [f32; 2],
        found: &mut Vec<Tile>,
    ) {
        found.clear();
        let side = TILES[level];
        let margin = LEVELS[level] * 0.5;
        let reaches = |key: &[i32; 3]| {
            // Dots may lie a little outside their voxel, so tiles grow by half a voxel.
            let (mut near, mut far) = (0.0, 0.0);
            for k in [0, 1, 2] {
                if level >= COLUMNS && k == 1 {
                    continue;
                }
                let low = key[k] as f32 * side - margin - center[k];
                let high = (key[k] + 1) as f32 * side + margin - center[k];
                near += if low > 0.0 {
                    low * low
                } else if high < 0.0 {
                    high * high
                } else {
                    0.0
                };
                far += (low * low).max(high * high);
            }
            near < outer * outer && (level >= COLUMNS || far >= inner * inner)
        };
        let world = statics.tree.bounds;
        let heights = [world.center.y - world.half.y, world.center.y + world.half.y];
        if level >= NEAR {
            let tiles = &self.far[level - NEAR];
            if !outer.is_finite() {
                let mut all: Vec<_> = tiles.iter().filter(|e| reaches(e.0)).collect();
                all.sort_by_key(|e| *e.0);
                found.extend(all.into_iter().map(|e| e.1.clone()));
            } else {
                let keys = if level >= COLUMNS {
                    span([center[0], 0.0, center[2]], outer + margin, side, [0.0, 0.0])
                } else {
                    span(center, outer + margin, side, heights)
                };
                found.extend(keys.filter(reaches).filter_map(|key| tiles.get(&key).cloned()));
            }
            return;
        }
        // In span order whether cached or built now, so the dots' order never depends on what was cached.
        let keys: Vec<[i32; 3]> = span(center, outer + margin, side, heights).filter(reaches).collect();
        let mut missing = Vec::new();
        {
            let near = self.near.lock().unwrap();
            for (at, key) in keys.iter().enumerate() {
                match near.get(&(level, *key)) {
                    Some(tile) => found.push(tile.clone()),
                    None => {
                        missing.push(at);
                        found.push(Tile::default());
                    }
                }
            }
        }
        for at in missing {
            // Built outside the lock: a tile is a pure function of the map, so a race only repeats work.
            let tile = Tile::new(near_tile(statics, level, keys[at]).into());
            found[at] = self.near.lock().unwrap().entry((level, keys[at])).or_insert(tile).clone();
        }
    }

    /// Fills `out` (n dots of `FEATURES`) for the car: gates first, then surfaces shell by shell.
    pub fn read(&self, session: &Session, course: &Course, car: &Car, out: &mut [f32]) {
        self.read_placed(session, course, car, out, &mut []);
    }

    /// `read`, also writing each written dot's exact world position to `placed` (as many as it holds), for viewers:
    /// the features keep a dot's offset only to the log-scaled 1/32 steps of `squash`.
    pub fn read_placed(
        &self,
        session: &Session,
        course: &Course,
        car: &Car,
        out: &mut [f32],
        placed: &mut [[f32; 3]],
    ) {
        assert_eq!(out.len() % FEATURES, 0);
        let n = out.len() / FEATURES;
        let slots = out.as_chunks_mut::<FEATURES>().0;
        let gates = read_gates(course, &session.race, car, &mut slots[..n / GATE_SHARE], placed);
        let surfaces = n - gates;
        let p = Vec3::from(car.position);
        SCRATCH.with_borrow_mut(|Scratch { shells, spare, tiles }| {
            let mut chosen = [0; SHELLS.len()];
            let (mut left, mut carry, mut inner, mut share) = (surfaces, 0, 0.0, 0.0);
            for (level, &(outer, weight)) in SHELLS.iter().enumerate() {
                let before = (surfaces as f32 * share).round() as usize;
                share += weight;
                let budget = ((surfaces as f32 * share).round() as usize - before + carry).min(left);
                let found = &mut shells[level];
                let (low, high) = (inner * inner, outer * outer);
                self.tiles_around(&session.world.statics, level, car.position, [inner, outer], tiles);
                // Branch-free, as about half the dots face away: each dot is written past the kept ones and kept by
                // moving the end. Only surfaces facing the car count; a road's underside can't be driven.
                // A run's box bounds its dots' distances as rounded, so skipping runs keeps exactly the same dots.
                let mut len = 0;
                for tile in tiles.iter() {
                    if found.len() < len + tile.dots.len() {
                        found.resize(2 * (len + tile.dots.len()), Dot::default());
                    }
                    for (bounds, run) in tile.runs.iter().zip(tile.dots.chunks(RUN)) {
                        let (mut near, mut far) = ([0.0; 3], [0.0; 3]);
                        for k in 0..3 {
                            let (below, above) = (bounds[0][k] - p[k], bounds[1][k] - p[k]);
                            near[k] = below.max(-above).max(0.0);
                            far[k] = below.abs().max(above.abs());
                        }
                        let square = |v: [f32; 3]| v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
                        if square(near) >= high || square(far) < low {
                            continue;
                        }
                        for dot in run {
                            let q = Vec3::from(dot.position) - p;
                            let d = q.dot(q);
                            found[len] = *dot;
                            len += usize::from((d >= low) & (d < high) & (dot.normal().dot(q) < 0.0));
                        }
                    }
                }
                found.truncate(len);
                chosen[level] = choose(found, budget, spare);
                carry = budget - chosen[level];
                left -= chosen[level];
                inner = outer;
            }
            for (level, found) in shells.iter_mut().enumerate() {
                let more = choose(&mut found[chosen[level]..], left, spare);
                chosen[level] += more;
                left -= more;
            }
            let mut at = gates;
            for (level, found) in shells.iter().enumerate() {
                for (i, (slot, dot)) in slots[at..].iter_mut().zip(&found[..chosen[level]]).enumerate() {
                    write(slot, car, dot.position, dot.normal() * (1.0 / 127.0), dot.class(), 0.0);
                    if let Some(p) = placed.get_mut(at + i) {
                        *p = dot.position;
                    }
                }
                at += chosen[level];
            }
        });
    }
}

#[derive(Default)]
struct Scratch {
    shells: [Vec<Dot>; SHELLS.len()],
    spare: Vec<(u32, Dot)>,
    tiles: Vec<Tile>,
}

thread_local! {
    static SCRATCH: std::cell::RefCell<Scratch> = Default::default();
}

/// Puts the chosen dots first and returns how many: every stratum (surface class and facing) gets an equal share,
/// those with fewer dots passing what they leave to the rest, and each takes its dots of smallest key. The rest
/// follow, for a later call to choose from.
fn choose(candidates: &mut [Dot], budget: usize, spare: &mut Vec<(u32, Dot)>) -> usize {
    if candidates.len() <= budget || budget == 0 {
        return candidates.len().min(budget);
    }
    let mut counts = [0usize; STRATA];
    for dot in candidates.iter() {
        counts[dot.stratum()] += 1;
    }
    let mut quota = [0usize; STRATA];
    let mut left = budget;
    while left > 0 {
        let open = (0..STRATA).filter(|&s| quota[s] < counts[s]).count();
        let share = left / open.max(1);
        if share == 0 {
            let open: Vec<usize> = (0..STRATA).filter(|&s| quota[s] < counts[s]).take(left).collect();
            for s in open {
                quota[s] += 1;
            }
            break;
        }
        for s in 0..STRATA {
            let more = share.min(counts[s] - quota[s]);
            quota[s] += more;
            left -= more;
        }
    }
    let mut starts = [0usize; STRATA + 1];
    for s in 0..STRATA {
        starts[s + 1] = starts[s] + counts[s];
    }
    spare.clear();
    spare.resize(candidates.len(), (0, Dot::default()));
    let mut fill = starts;
    for dot in candidates.iter() {
        let s = dot.stratum();
        spare[fill[s]] = (dot.key(), *dot);
        fill[s] += 1;
    }
    let (mut first, mut rest) = (0, budget);
    for s in 0..STRATA {
        let group = &mut spare[starts[s]..starts[s + 1]];
        if quota[s] > 0 && quota[s] < group.len() {
            group.select_nth_unstable_by(quota[s] - 1, |a, b| (a.0, a.1.bits()).cmp(&(b.0, b.1.bits())));
        }
        for (i, (_, dot)) in group.iter().enumerate() {
            if i < quota[s] {
                candidates[first] = *dot;
                first += 1;
            } else {
                candidates[rest] = *dot;
                rest += 1;
            }
        }
    }
    budget
}

fn read_gates(
    course: &Course,
    race: &Race,
    car: &Car,
    slots: &mut [[f32; FEATURES]],
    placed: &mut [[f32; 3]],
) -> usize {
    let mut gates: Vec<(f32, [f32; 3], [f32; 3], u8, bool)> = Vec::new();
    let distance = |point: [f32; 3]| (0..3).map(|k| (point[k] - car.position[k]).powi(2)).sum::<f32>();
    for target in &course.checkpoints {
        let done = passed(race, target.group);
        for (&point, &facing) in target.gates.iter().zip(&target.facings) {
            gates.push((distance(point), point, facing, CHECKPOINT, done));
        }
    }
    let open = course.checkpoints.iter().all(|target| passed(race, target.group));
    for (&point, &facing) in course.finishes.iter().zip(&course.finish_facings) {
        gates.push((distance(point), point, facing, FINISH, open));
    }
    let order = |a: &(f32, [f32; 3], [f32; 3], u8, bool), b: &(f32, [f32; 3], [f32; 3], u8, bool)| {
        a.0.total_cmp(&b.0).then_with(|| a.1.map(f32::to_bits).cmp(&b.1.map(f32::to_bits)))
    };
    let count = gates.len().min(slots.len());
    if count == 0 {
        return 0;
    }
    if count < gates.len() {
        gates.select_nth_unstable_by(count - 1, order);
    }
    gates[..count].sort_by(order);
    for (i, (slot, &(_, point, facing, class, flag))) in slots.iter_mut().zip(&gates[..count]).enumerate() {
        write(slot, car, point, Vec3::from(facing), class, f32::from(flag));
        if let Some(p) = placed.get_mut(i) {
            *p = point;
        }
    }
    count
}

fn write(slot: &mut [f32; FEATURES], car: &Car, position: [f32; 3], normal: Vec3, class: u8, flag: f32) {
    let delta = car.to_car(Vec3::from(position) - Vec3::from(car.position));
    let facing = car.to_car(normal);
    *slot = [
        1.0,
        squash(delta.x),
        squash(delta.y),
        squash(delta.z),
        code(facing.x),
        code(facing.y),
        code(facing.z),
        code(normal.y),
        f32::from(class) / STEPS,
        flag,
    ];
}

/// A car-frame offset in metres as log2(1 + |v|) / 3 to the nearest 1/32, with its sign: steps of ~7 cm at the car,
/// 7% of the distance far away, and ±127/32 from 3.6 km. log2 is the float's exponent plus a table over its
/// mantissa's top 7 bits, cheap and the same on every machine.
pub fn squash(v: f32) -> f32 {
    let bits = (1.0 + v.abs()).to_bits();
    let log = ((bits >> 23) as i32 - 127) as f32 + LOG2[(bits >> 16 & 0x7f) as usize];
    (log * (STEPS / 3.0)).round().min(127.0).copysign(v) / STEPS
}

static LOG2: std::sync::LazyLock<[f32; 128]> =
    std::sync::LazyLock::new(|| std::array::from_fn(|i| libm::log2f(1.0 + (i as f32 + 0.5) / 128.0)));

fn code(v: f32) -> f32 {
    (v * STEPS).round().clamp(-127.0, 127.0) / STEPS
}

/// The tiles of `side` within `radius` of `center` on each axis, and between `heights` (the world's).
fn span(center: [f32; 3], radius: f32, side: f32, heights: [f32; 2]) -> impl Iterator<Item = [i32; 3]> {
    let range = |k: usize, low: f32, high: f32| {
        let low = ((center[k] - radius).max(low) / side).floor() as i32;
        let high = ((center[k] + radius).min(high) / side).floor() as i32;
        low..=high
    };
    let (xs, ys) = (range(0, f32::NEG_INFINITY, f32::INFINITY), range(1, heights[0], heights[1]));
    let range = |k: usize| range(k, f32::NEG_INFINITY, f32::INFINITY);
    range(2).flat_map(move |z| {
        let xs = xs.clone();
        ys.clone().flat_map(move |y| xs.clone().map(move |x| [x, y, z]))
    })
}

/// A level's dots in its tiles: whole columns from `COLUMNS` on, as their dots are few in any column.
fn tiles(dots: &[Dot], level: usize) -> HashMap<[i32; 3], Tile, Fast> {
    let mut tiles: HashMap<[i32; 3], Vec<Dot>, Fast> = HashMap::default();
    for dot in dots {
        let mut key = dot.position.map(|v| (v / TILES[level]).floor() as i32);
        if level >= COLUMNS {
            key[1] = 0;
        }
        tiles.entry(key).or_default().push(*dot);
    }
    tiles.into_iter().map(|(k, v)| (k, Tile::new(v.into()))).collect()
}

fn coarser(dots: &[Dot], size: f32) -> Vec<Dot> {
    let mut best = Best::default();
    for dot in dots {
        best.offer(dot.position.map(|v| (v / size).floor() as i32), size, *dot);
    }
    best.into_dots()
}

/// Where `add` offers points: each voxel and facing keeps the dot nearest the voxel's centre (ties to the smaller
/// position bits).
trait Voxels {
    /// The squared distance from the centre a dot must beat to be kept, infinite while the voxel holds none.
    fn bound(&self, voxel: [i32; 3], facing: u8) -> f32;
    fn offer(&mut self, voxel: [i32; 3], size: f32, dot: Dot);
}

#[derive(Default)]
struct Best(HashMap<([i32; 3], u8), (f32, Dot), Fast>);

impl Voxels for Best {
    fn bound(&self, _: [i32; 3], _: u8) -> f32 {
        f32::INFINITY
    }

    fn offer(&mut self, voxel: [i32; 3], size: f32, dot: Dot) {
        keep(self.0.entry((voxel, dot.facing())).or_insert((f32::INFINITY, dot)), voxel, size, dot);
    }
}

impl Best {
    fn into_dots(self) -> Vec<Dot> {
        let mut dots: Vec<_> = self.0.into_iter().map(|(key, (_, dot))| (key, dot)).collect();
        dots.sort_by_key(|e| e.0);
        dots.into_iter().map(|e| e.1).collect()
    }
}

/// `Best` over the voxels `add` may offer for one area, in an array: a near tile's voxels are few and known up
/// front, and hashing them cost more than finding their dots.
struct Grid {
    low: [i32; 3],
    dims: [usize; 3],
    slots: Vec<Option<(f32, Dot)>>,
}

impl Grid {
    fn new(area: [[f32; 3]; 2], size: f32) -> Self {
        let low = area[0].map(|v| (v / size).floor() as i32);
        let high: [i32; 3] = std::array::from_fn(|k| ((area[1][k] - size * 1e-3) / size).floor() as i32);
        let dims = std::array::from_fn(|k| (high[k] - low[k] + 1).max(0) as usize);
        Self { low, dims, slots: vec![None; dims.iter().product::<usize>() * 3] }
    }

    fn slot(&self, voxel: [i32; 3], facing: u8) -> usize {
        let [x, y, z] = std::array::from_fn(|k| (voxel[k] - self.low[k]) as usize);
        ((x * self.dims[1] + y) * self.dims[2] + z) * 3 + usize::from(facing)
    }

    /// In `Best::into_dots`'s order.
    fn into_dots(self) -> Vec<Dot> {
        self.slots.into_iter().flatten().map(|(_, dot)| dot).collect()
    }
}

impl Voxels for Grid {
    fn bound(&self, voxel: [i32; 3], facing: u8) -> f32 {
        self.slots[self.slot(voxel, facing)].map_or(f32::INFINITY, |(d, _)| d)
    }

    fn offer(&mut self, voxel: [i32; 3], size: f32, dot: Dot) {
        let slot = self.slot(voxel, dot.facing());
        keep(self.slots[slot].get_or_insert((f32::INFINITY, dot)), voxel, size, dot);
    }
}

fn keep(entry: &mut (f32, Dot), voxel: [i32; 3], size: f32, dot: Dot) {
    let center: [f32; 3] = std::array::from_fn(|k| (voxel[k] as f32 + 0.5) * size);
    let d = (0..3).map(|k| (dot.position[k] - center[k]).powi(2)).sum::<f32>();
    if d < entry.0 || (d == entry.0 && dot.bits() < entry.1.bits()) {
        *entry = (d, dot);
    }
}

fn class_of(surface: u8, play: u8) -> u8 {
    match play {
        gameplay::TURBO | gameplay::TURBO_2 | gameplay::TURBO_ROULETTE => 12,
        gameplay::FREE_WHEELING => 13,
        gameplay::NO_GRIP => 14,
        gameplay::NO_STEERING => 15,
        _ => surface_class(surface) as u8,
    }
}

/// Offers `triangle`'s points to every voxel of `size` it may touch that lies in `area` (low and high corners).
fn add(best: &mut impl Voxels, v: &[[f32; 3]; 3], surface: u8, play: u8, size: f32, area: [[f32; 3]; 2]) {
    let low: [f32; 3] = std::array::from_fn(|k| v[0][k].min(v[1][k]).min(v[2][k]));
    let high: [f32; 3] = std::array::from_fn(|k| v[0][k].max(v[1][k]).max(v[2][k]));
    let mut lo = low;
    let mut hi = high;
    for k in 0..3 {
        lo[k] = lo[k].max(area[0][k]);
        hi[k] = hi[k].min(area[1][k] - size * 1e-3);
    }
    if (0..3).any(|k| lo[k] > hi[k]) {
        return;
    }
    let [a, b, c] = v.map(Vec3::from);
    let normal = cross(b - a, c - a);
    let length = normal.dot(normal).sqrt();
    if !(length > 1e-9) || v.iter().flatten().any(|x| !x.is_finite()) {
        return;
    }
    let normal = normal * (1.0 / length);
    let facing = if normal.y >= MIN_NORMAL_Y {
        UP
    } else if normal.y <= -MIN_NORMAL_Y {
        DOWN
    } else {
        SIDE
    };
    let dot = Dot {
        position: [0.0; 3],
        normal: normal.to_array().map(|x| (x * 127.0).round() as i8),
        kind: class_of(surface, play) << 2 | facing,
    };
    let n = normal.to_array();
    let along = (0..3).fold(0, |m, k| if n[k].abs() > n[m].abs() { k } else { m });
    let [u, w] = [(along + 1) % 3, (along + 2) % 3];
    let cells = |k: usize| (lo[k] / size).floor() as i32..=(hi[k] / size).floor() as i32;
    let offset = normal.dot(a);
    let reach = 0.75 * size * size;
    for i in cells(u) {
        for j in cells(w) {
            let su = [(i as f32 * size).max(low[u]), ((i + 1) as f32 * size).min(high[u])];
            let sw = [(j as f32 * size).max(low[w]), ((j + 1) as f32 * size).min(high[w])];
            let mut range = [f32::INFINITY, f32::NEG_INFINITY];
            for x in su {
                for y in sw {
                    let t = (offset - n[u] * x - n[w] * y) / n[along];
                    range = [range[0].min(t), range[1].max(t)];
                }
            }
            let first = (range[0].max(lo[along]) / size).floor() as i32;
            let last = (range[1].min(hi[along]) / size).floor() as i32;
            for k in first..=last {
                let mut voxel = [0; 3];
                voxel[u] = i;
                voxel[w] = j;
                voxel[along] = k;
                let center: [f32; 3] = std::array::from_fn(|q| (voxel[q] as f32 + 0.5) * size);
                // The triangle's box is no nearer the centre than its nearest point: a voxel the box can't reach
                // or win skips finding that point. The slack covers rounding.
                let apart = (0..3)
                    .map(|q| (low[q] - center[q]).max(center[q] - high[q]).max(0.0).powi(2))
                    .sum::<f32>()
                    .sqrt();
                if apart > best.bound(voxel, facing).min(reach).sqrt() + SLACK {
                    continue;
                }
                let center = Vec3::from(center);
                let point = closest(center, a, b, c);
                let gap = point - center;
                if gap.dot(gap) <= reach {
                    best.offer(voxel, size, Dot { position: point.to_array(), ..dot });
                }
            }
        }
    }
}

/// A near level's tile from the engine's collision trees: the static surfaces over the tile, then each mesh's
/// triangles there, as `Session::static_triangles` places them.
fn near_tile(statics: &StaticWorld, level: usize, tile: [i32; 3]) -> Vec<Dot> {
    let side = TILES[level];
    let size = LEVELS[level];
    let area = [tile.map(|t| t as f32 * side), tile.map(|t| (t + 1) as f32 * side)];
    let column = Aabb::new(
        Vec3::from(std::array::from_fn(|k| (area[0][k] + area[1][k]) * 0.5)),
        Vec3::new(side * 0.5 + 0.5, side * 0.5 + 0.5, side * 0.5 + 0.5),
    );
    let mut best = Grid::new(area, size);
    for id in AabbQuery::new(&statics.tree.nodes, column) {
        let Some(surface) = statics.surfaces.get(id as usize) else { continue };
        if statics.items[surface.item as usize].flags.contains(ItemEvents::CONTACT_TIME) {
            continue;
        }
        let Geometry::Mesh(mesh) = &statics.shapes[surface.shape as usize].geometry else { continue };
        let iso = surface.iso;
        let local = Aabb::new(
            iso.rot.transpose_mul_vec(column.center - iso.pos),
            iso.rot.transpose().abs() * column.half,
        );
        for t in AabbQuery::new(&mesh.tree.nodes, local) {
            let Some(triangle) = mesh.triangles.get(t as usize) else { continue };
            if triangle.verts.iter().any(|&i| i as usize >= mesh.vertices.len()) {
                continue;
            }
            let v = triangle.verts.map(|i| iso.transform_point(mesh.vertices[i as usize]).to_array());
            add(&mut best, &v, triangle.surface, triangle.gameplay, size, area);
        }
    }
    best.into_dots()
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}

/// The point of triangle abc nearest p (Ericson, Real-Time Collision Detection 5.1.5).
fn closest(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    a + ab * (vb * denom) + ac * (vc * denom)
}
