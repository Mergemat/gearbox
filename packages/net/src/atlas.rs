use gearbox_engine::consumer::PhysicalTriangle;
use gearbox_engine::race::WaypointFlags;
use gearbox_engine::{Session, gameplay};
use gearbox_search::gate::gates;

use crate::chart::{self, Chart};
use crate::ground::{self, Ground, Layer};
use crate::observe::hypot;

pub const SCALES: [usize; 3] = [1, 4, 16];
pub const SIDE: usize = 24;
pub const BEHIND: usize = 8;
pub const CHANNELS: usize = 7;
pub const STEPS: f32 = 32.0;
pub const IMAGE_LEN: usize = SIDE * SIDE * CHANNELS;
pub const LEN: usize = SCALES.len() * IMAGE_LEN;
pub const GROUPS: usize = u16::MAX as usize;

const LEVELS: usize = 4;
const MERGE: [f32; 3] = [0.0, 3.0, 6.0];
const WALL_LOW: f32 = 0.25;
const WALL_HIGH: f32 = 4.0;
const MARK_DEPTH: f32 = 8.0;
const BELOW: f32 = 32.0;
const BOOST: u8 = 1;
const LOSS: u8 = 2;
const OTHER: u8 = 4;
const FINISH: u8 = 8;
const NO_GATE: u16 = u16::MAX;
const REACH: f32 = 0.5;

type Stamp = ([f32; 3], [f32; 3], u8, u16);

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct Level {
    height: f32,
    cover: u16,
    loose: u16,
    slick: u16,
    wall: u16,
    marks: u8,
    gate: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
struct Mark {
    wall: bool,
    marks: u8,
    gate: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
struct Frame {
    origin: [f32; 2],
    cell: f32,
    size: [usize; 2],
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
struct Grid {
    frame: Frame,
    area: f32,
    starts: Vec<u32>,
    levels: Vec<Level>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Atlas {
    marks: Vec<Mark>,
    grids: Vec<Grid>,
    groups: Vec<u32>,
}

impl Frame {
    fn of(ground: &Ground) -> Self {
        Self { origin: ground.origin, cell: ground::CELL, size: ground.size }
    }

    fn index(&self, x: f32, z: f32) -> Option<usize> {
        let i = ((x - self.origin[0]) / self.cell).floor();
        let k = ((z - self.origin[1]) / self.cell).floor();
        (i >= 0.0 && k >= 0.0 && (i as usize) < self.size[0] && (k as usize) < self.size[1])
            .then(|| k as usize * self.size[0] + i as usize)
    }

    fn span(&self, low: f32, high: f32, axis: usize) -> std::ops::Range<usize> {
        let first = ((low - self.origin[axis]) / self.cell).floor().max(0.0);
        let last = ((high - self.origin[axis]) / self.cell).floor().min(self.size[axis] as f32 - 1.0);
        if last >= first { first as usize..last as usize + 1 } else { 0..0 }
    }
}

impl Atlas {
    pub fn build(session: &Session, ground: &Ground, triangles: &[PhysicalTriangle]) -> Self {
        let mut groups = Vec::new();
        let mut stamps = Vec::new();
        let waypoints = &session.race.rules.waypoints;
        for gate in gates(session).unwrap_or_default() {
            let mark = &waypoints[gate.waypoint as usize];
            if !mark.linked || !mark.flags.counts() {
                continue;
            }
            let (bits, index) = if mark.flags.contains(WaypointFlags::GOAL) {
                (FINISH, NO_GATE)
            } else {
                let index = groups.iter().position(|&group| group == mark.group).unwrap_or_else(|| {
                    groups.push(mark.group);
                    groups.len() - 1
                });
                (0, if index < GROUPS { index as u16 } else { NO_GATE })
            };
            stamps.extend(gate.boxes.iter().map(|b| (b.min, b.max, bits, index)));
        }
        for trigger in session.effect_triggers() {
            let bits = effect_marks(trigger.effect);
            if bits != 0 {
                stamps.push((trigger.min, trigger.max, bits, NO_GATE));
            }
        }
        groups.truncate(GROUPS);
        Self::draw(ground, triangles, &stamps, groups)
    }

    fn draw(ground: &Ground, triangles: &[PhysicalTriangle], stamps: &[Stamp], groups: Vec<u32>) -> Self {
        let mut marks = vec![Mark { wall: false, marks: 0, gate: NO_GATE }; ground.layers.len()];
        walls(ground, &mut marks, triangles);
        for &(min, max, bits, gate) in stamps {
            stamp(ground, &mut marks, min, max, bits, gate);
        }
        let grids =
            SCALES[1..].iter().zip(&MERGE[1..]).map(|(&f, &m)| coarse(ground, &marks, f, m)).collect();
        Self { marks, grids, groups }
    }

    pub fn groups(&self) -> &[u32] {
        &self.groups
    }

    pub fn bytes(&self) -> usize {
        self.marks.len() * size_of::<Mark>()
            + self
                .grids
                .iter()
                .map(|g| g.starts.len() * 4 + g.levels.len() * size_of::<Level>())
                .sum::<usize>()
            + self.groups.len() * size_of::<u32>()
    }

    pub fn read(
        &self,
        ground: &Ground,
        position: [f32; 3],
        forward: [f32; 2],
        passed: &[bool],
        out: &mut [f32],
    ) {
        assert_eq!(out.len(), LEN);
        if self.marks.len() != ground.layers.len() {
            return;
        }
        let forward = ground::unit(forward);
        let (fine, rest) = out.split_at_mut(IMAGE_LEN);
        sample(&Frame::of(ground), 1.0, position, forward, passed, fine, |column, limit| {
            let range = ground.starts[column] as usize..ground.starts[column + 1] as usize;
            let layers = &ground.layers[range.clone()];
            let at = layers.iter().position(|layer| layer.height <= limit)?;
            let lower = layers.get(at + 1).is_some_and(|below| below.height >= layers[at].height - BELOW);
            Some((level(&layers[at], self.marks[range.start + at]), lower))
        });
        for (grid, image) in self.grids.iter().zip(rest.as_chunks_mut::<IMAGE_LEN>().0) {
            sample(&grid.frame, grid.area, position, forward, passed, image, |column, limit| {
                let levels = &grid.levels[grid.starts[column] as usize..grid.starts[column + 1] as usize];
                let at = levels.iter().position(|level| level.height <= limit)?;
                let lower = levels.get(at + 1).is_some_and(|below| below.height >= levels[at].height - BELOW);
                Some((levels[at], lower))
            });
        }
    }
}

fn sample(
    frame: &Frame,
    area: f32,
    position: [f32; 3],
    [fx, fz]: [f32; 2],
    passed: &[bool],
    image: &mut [f32],
    pick: impl Fn(usize, f32) -> Option<(Level, bool)>,
) {
    for row in 0..SIDE {
        let along = (row as f32 - BEHIND as f32 + 0.5) * frame.cell;
        for col in 0..SIDE {
            let across = (col as f32 - (SIDE / 2) as f32 + 0.5) * frame.cell;
            let x = position[0] + along * fx + across * fz;
            let z = position[2] + along * fz - across * fx;
            let Some(column) = frame.index(x, z) else { continue };
            let limit = position[1] + 2.0 + 0.25 * along.abs().max(across.abs());
            let Some((level, lower)) = pick(column, limit) else { continue };
            let gate = if level.marks & FINISH != 0 {
                -1.0
            } else if level.gate == NO_GATE {
                0.0
            } else if passed.get(level.gate as usize).copied().unwrap_or(false) {
                0.5
            } else {
                1.0
            };
            let special = special(level.marks);
            let share = |count: f32| (count * STEPS / area).round() / STEPS;
            let cell = (row * SIDE + col) * CHANNELS;
            image[cell..cell + CHANNELS].copy_from_slice(&[
                share(f32::from(level.cover)),
                ((level.height - position[1]) * 2.0).round().clamp(-128.0, 127.0) / STEPS,
                f32::from(lower),
                share(f32::from(level.loose) * 0.5 + f32::from(level.slick)),
                share(f32::from(level.wall)),
                special,
                gate,
            ]);
        }
    }
}

fn level(layer: &Layer, mark: Mark) -> Level {
    Level {
        height: layer.height,
        cover: 1,
        loose: u16::from(matches!(layer.surface, 1 | 2 | 7 | 8)),
        slick: u16::from(matches!(layer.surface, 3 | 9)),
        wall: u16::from(mark.wall),
        marks: mark.marks
            | match layer.special {
                1 => BOOST,
                2..=4 => LOSS,
                _ => 0,
            },
        gate: mark.gate,
    }
}

fn effect_marks(effect: u8) -> u8 {
    match effect {
        gameplay::TURBO
        | gameplay::TURBO_2
        | gameplay::TURBO_ROULETTE
        | gameplay::REACTOR_BOOST_LEGACY
        | gameplay::REACTOR_BOOST_2_LEGACY
        | gameplay::REACTOR_BOOST_ORIENTED
        | gameplay::REACTOR_BOOST_2_ORIENTED => BOOST,
        gameplay::FREE_WHEELING
        | gameplay::NO_GRIP
        | gameplay::NO_STEERING
        | gameplay::NO_BRAKES
        | gameplay::CRUISE
        | gameplay::SLOW_MOTION
        | gameplay::FRAGILE => LOSS,
        gameplay::FORCE_ACCELERATION
        | gameplay::RESET
        | gameplay::BUMPER
        | gameplay::BUMPER_2
        | gameplay::BOUNCY
        | gameplay::VEHICLE_TRANSFORM_RESET
        | gameplay::VEHICLE_TRANSFORM_CAR_SNOW
        | gameplay::VEHICLE_TRANSFORM_CAR_RALLY
        | gameplay::VEHICLE_TRANSFORM_CAR_DESERT => OTHER,
        _ => 0,
    }
}

fn walls(ground: &Ground, marks: &mut [Mark], triangles: &[PhysicalTriangle]) {
    let frame = Frame::of(ground);
    let reach = frame.size.map(|n| n as f32 * frame.cell);
    let mut columns = Vec::new();
    for triangle in triangles {
        let [a, b, c] = triangle.v;
        let bounds = |axis: usize| (a[axis].min(b[axis]).min(c[axis]), a[axis].max(b[axis]).max(c[axis]));
        let (across, height, along) = (bounds(0), bounds(1), bounds(2));
        if !(across.1 >= frame.origin[0]
            && across.0 <= frame.origin[0] + reach[0]
            && along.1 >= frame.origin[1]
            && along.0 <= frame.origin[1] + reach[1])
        {
            continue;
        }
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if !(length > 1e-9) || normal[1].abs() >= ground::MIN_NORMAL_Y * length {
            continue;
        }
        columns.clear();
        let mut touch = |x: f32, z: f32| {
            for k in frame.span(z - REACH, z + REACH, 1) {
                columns.extend(frame.span(x - REACH, x + REACH, 0).map(|i| k * frame.size[0] + i));
            }
        };
        let (first, last) = (frame.span(across.0, across.1, 0), frame.span(along.0, along.1, 1));
        if first.len() <= 2 && last.len() <= 2 {
            touch(across.0, along.0);
            touch(across.1, along.1);
            touch(across.0, along.1);
            touch(across.1, along.0);
        } else {
            for (p, q) in [(a, b), (b, c), (c, a)] {
                let span = hypot(q[0] - p[0], q[2] - p[2]);
                let steps = ((span / (frame.cell * 0.5)).ceil() as usize).clamp(1, 1 << 16);
                for step in 0..=steps {
                    let t = step as f32 / steps as f32;
                    touch(p[0] + (q[0] - p[0]) * t, p[2] + (q[2] - p[2]) * t);
                }
            }
            let edge = |p: [f32; 3], q: [f32; 3], x: f32, z: f32| {
                (q[0] - p[0]) * (z - p[2]) - (q[2] - p[2]) * (x - p[0])
            };
            for k in last {
                let z = frame.origin[1] + (k as f32 + 0.5) * frame.cell;
                for i in first.clone() {
                    let x = frame.origin[0] + (i as f32 + 0.5) * frame.cell;
                    let w = [edge(a, b, x, z), edge(b, c, x, z), edge(c, a, x, z)];
                    if w.iter().all(|&w| w >= 0.0) || w.iter().all(|&w| w <= 0.0) {
                        columns.push(k * frame.size[0] + i);
                    }
                }
            }
        }
        for &column in &columns {
            for at in ground.starts[column] as usize..ground.starts[column + 1] as usize {
                let top = ground.layers[at].height;
                if height.1 > top + WALL_LOW && height.0 < top + WALL_HIGH {
                    marks[at].wall = true;
                }
            }
        }
    }
}

fn stamp(ground: &Ground, marks: &mut [Mark], min: [f32; 3], max: [f32; 3], bits: u8, gate: u16) {
    if !min.iter().chain(&max).all(|v| v.is_finite()) {
        return;
    }
    let frame = Frame::of(ground);
    for k in frame.span(min[2], max[2], 1) {
        for i in frame.span(min[0], max[0], 0) {
            let column = k * frame.size[0] + i;
            let range = ground.starts[column] as usize..ground.starts[column + 1] as usize;
            if let Some(at) = range
                .into_iter()
                .find(|&at| (min[1] - MARK_DEPTH..=max[1]).contains(&ground.layers[at].height))
            {
                marks[at].marks |= bits;
                marks[at].gate = marks[at].gate.min(gate);
            }
        }
    }
}

struct Gather {
    factor: usize,
    merge: f32,
    members: Vec<(Level, usize)>,
    seen: Vec<bool>,
}

impl Gather {
    fn new(factor: usize, merge: f32) -> Self {
        Self { factor, merge, members: Vec::new(), seen: vec![false; factor * factor] }
    }

    fn cell(&mut self, ground: &Ground, marks: &[Mark], first: [isize; 2], levels: &mut Vec<Level>) {
        let factor = self.factor;
        self.members.clear();
        for dk in 0..factor {
            for di in 0..factor {
                let [i, k] = [first[0] + di as isize, first[1] + dk as isize];
                if i < 0 || k < 0 || i as usize >= ground.size[0] || k as usize >= ground.size[1] {
                    continue;
                }
                let column = k as usize * ground.size[0] + i as usize;
                for at in ground.starts[column] as usize..ground.starts[column + 1] as usize {
                    self.members.push((level(&ground.layers[at], marks[at]), dk * factor + di));
                }
            }
        }
        self.members.sort_by(|a, b| b.0.height.total_cmp(&a.0.height));
        let mut at = 0;
        for _ in 0..LEVELS {
            let Some(&(top, _)) = self.members.get(at) else { break };
            self.seen.fill(false);
            let mut level = Level { cover: 0, loose: 0, slick: 0, wall: 0, marks: 0, gate: NO_GATE, ..top };
            while let Some(&(member, id)) =
                self.members.get(at).filter(|m| m.0.height >= top.height - self.merge)
            {
                if !std::mem::replace(&mut self.seen[id], true) {
                    level.cover += member.cover;
                    level.loose += member.loose;
                    level.slick += member.slick;
                    level.wall += member.wall;
                }
                level.marks |= member.marks;
                level.gate = level.gate.min(member.gate);
                at += 1;
            }
            levels.push(level);
        }
    }
}

fn coarse(ground: &Ground, marks: &[Mark], factor: usize, merge: f32) -> Grid {
    let fine = Frame::of(ground);
    let size = fine.size.map(|n| n.div_ceil(factor));
    let mut starts = Vec::with_capacity(size[0] * size[1] + 1);
    let mut levels = Vec::new();
    let mut gather = Gather::new(factor, merge);
    for k in 0..size[1] {
        for i in 0..size[0] {
            starts.push(levels.len() as u32);
            gather.cell(ground, marks, [(i * factor) as isize, (k * factor) as isize], &mut levels);
        }
    }
    starts.push(levels.len() as u32);
    Grid {
        frame: Frame { origin: fine.origin, cell: fine.cell * factor as f32, size },
        area: (factor * factor) as f32,
        starts,
        levels,
    }
}

impl Atlas {
    pub(crate) fn chart(&self, ground: &Ground, chart: &mut Chart, factor: usize) {
        if self.marks.len() != ground.layers.len() {
            return;
        }
        let first = [0, 1].map(|k| {
            ((chart.center[k] - chart.extent * 0.5 - ground.origin[k]) / ground::CELL).round() as isize
        });
        let area = (factor * factor) as f32;
        let mut gather = Gather::new(factor, (chart.cell * 0.25).max(MERGE[1]));
        let mut levels = Vec::with_capacity(LEVELS);
        for (at, cell) in chart.image.as_chunks_mut::<CHANNELS>().0.iter_mut().enumerate() {
            let (row, col) = (at / chart::SIDE, at % chart::SIDE);
            levels.clear();
            let corner = [first[0] + (col * factor) as isize, first[1] + (row * factor) as isize];
            gather.cell(ground, &self.marks, corner, &mut levels);
            let Some(top) = levels.first() else { continue };
            let share = |count: f32| (count * STEPS / area).round() / STEPS;
            let lower = levels.get(1).is_some_and(|below| below.height >= top.height - BELOW);
            cell.copy_from_slice(&[
                share(f32::from(top.cover)),
                (top.height - chart.height).round().clamp(-128.0, 127.0) / STEPS,
                f32::from(lower),
                share(f32::from(top.loose) * 0.5 + f32::from(top.slick)),
                share(f32::from(top.wall)),
                special(top.marks),
                if top.marks & FINISH != 0 { -1.0 } else { f32::from(top.gate != NO_GATE) },
            ]);
        }
    }
}

fn special(marks: u8) -> f32 {
    if marks & LOSS != 0 {
        -1.0
    } else if marks & BOOST != 0 {
        1.0
    } else if marks & OTHER != 0 {
        0.5
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(height: f32, x: [f32; 2], z: [f32; 2]) -> [PhysicalTriangle; 2] {
        let corner = |i: usize, k: usize| [x[i], height, z[k]];
        let triangle = |v| PhysicalTriangle { v, surface: gearbox_engine::surface::ASPHALT, gameplay: 0 };
        [
            triangle([corner(0, 0), corner(0, 1), corner(1, 1)]),
            triangle([corner(0, 0), corner(1, 1), corner(1, 0)]),
        ]
    }

    fn wall(x: f32, z: [f32; 2], top: f32) -> [PhysicalTriangle; 2] {
        let triangle = |v| PhysicalTriangle { v, surface: gearbox_engine::surface::ASPHALT, gameplay: 0 };
        [
            triangle([[x, 0.0, z[0]], [x, top, z[0]], [x, top, z[1]]]),
            triangle([[x, 0.0, z[0]], [x, top, z[1]], [x, 0.0, z[1]]]),
        ]
    }

    fn cell(out: &[f32], scale: usize, row: usize, col: usize) -> &[f32] {
        let at = scale * IMAGE_LEN + (row * SIDE + col) * CHANNELS;
        &out[at..at + CHANNELS]
    }

    #[test]
    fn the_images_show_roads_bridges_walls_and_gates_turned_with_the_heading() {
        let mut triangles = quad(0.0, [-6.0, 6.0], [-200.0, 200.0]).to_vec();
        triangles.extend(quad(12.0, [-100.0, 100.0], [20.0, 26.0]));
        triangles.extend(wall(6.0, [-200.0, 200.0], 3.0));
        let ground = Ground::from_triangles(&triangles, [[-300.0, -300.0], [300.0, 300.0]]);
        let gate = ([-6.0, 0.0, 10.0], [6.0, 8.0, 10.2], 0, 0);
        let atlas = Atlas::draw(&ground, &triangles, &[gate], vec![7]);
        let read = |position: [f32; 3], forward: [f32; 2], passed: bool| {
            let mut out = vec![0.0; LEN];
            atlas.read(&ground, position, forward, &[passed], &mut out);
            out
        };
        let ahead = read([0.0, 0.5, 0.0], [0.0, 1.0], false);
        assert_eq!(cell(&ahead, 0, 13, 11), &[1.0, -1.0 / 32.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(cell(&ahead, 0, 19, 11)[..3], [1.0, -1.0 / 32.0, 0.0]);
        assert_eq!(cell(&ahead, 0, 8, 14)[4], 1.0);
        assert_eq!(cell(&ahead, 0, 8, 13)[4], 0.0);
        assert_eq!(cell(&ahead, 0, 8, 15), &[0.0; CHANNELS]);
        assert_eq!(cell(&read([0.0, 0.5, 0.0], [0.0, 1.0], true), 0, 13, 11)[6], 0.5);
        let deck = read([0.0, 12.5, 23.0], [0.0, 1.0], false);
        assert_eq!(cell(&deck, 0, 8, 11)[..3], [1.0, -1.0 / 32.0, 1.0]);
        assert_eq!(cell(&deck, 0, 8, 2)[..3], [1.0, -1.0 / 32.0, 0.0]);
        assert_eq!(
            [cell(&ahead, 1, 9, 11)[0], cell(&ahead, 1, 9, 12)[0], cell(&ahead, 1, 9, 12)[4]],
            [1.0, 0.25, 0.25]
        );
        let across = read([0.0, 0.5, 0.0], [2.0, 0.0], false);
        assert_eq!(cell(&across, 0, 8, 6)[6], 1.0);
        assert!(
            ahead
                .iter()
                .chain(&deck)
                .all(|v| (v * STEPS).fract() == 0.0 && (-128.0..128.0).contains(&(v * STEPS)))
        );
        assert_eq!(cell(&across, 0, 13, 11), &[0.0; CHANNELS]);
    }
}
