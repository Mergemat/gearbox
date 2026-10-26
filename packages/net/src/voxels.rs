use anyhow::{Context, Result, ensure};
use gearbox_engine::collision::{ItemFlags, Lists, QueryFilter, Ray};
use gearbox_engine::shape::Geometry;
use gearbox_engine::{Session, Vec3};
use gearbox_search::gate::gates;

use crate::observe::surface_class;

pub const CELL: f32 = 4.0;
pub const FORWARD: usize = 24;
pub const BEHIND: usize = 4;
pub const ACROSS: usize = 16;
pub const UP: usize = 8;
pub const CELLS: usize = FORWARD * ACROSS * UP;
pub const CHANNELS: usize = 4;
pub const FEATURES: usize = CHANNELS * CELLS;

const MARGIN: f32 = 256.0;
const MARGIN_UP: f32 = 128.0;
const MAX_ACROSS: usize = 640;
const MAX_UP: usize = 128;
const MAX_HITS: usize = 64;
const MAGIC: &[u8; 4] = b"GVOX";
const FORMAT: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position: [f32; 3],
    pub yaw: f32,
}

impl Pose {
    pub fn read(session: &Session) -> Self {
        let body = session.body();
        let forward = body.rot * Vec3::new(0.0, 0.0, 1.0);
        Self { position: body.pos.to_array(), yaw: forward.x.atan2(forward.z) }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Voxels<B = Vec<u8>> {
    bytes: B,
    origin: [f32; 3],
    size: [usize; 3],
    entries: usize,
}

const HEADER: usize = 4 + 4 + 12 + 12 + 4;

fn code(normal: Vec3, material: u8) -> u8 {
    let up = normal.y.abs();
    let slope = if up > 0.9 {
        0
    } else if up > 0.5 {
        1
    } else {
        2
    };
    let grip = match surface_class(material) {
        0 => 0,
        1 | 2 | 7 => 1,
        3 | 8 | 9 => 2,
        _ => 3,
    };
    1 + slope * 4 + grip
}

const TABLE: [[f32; CHANNELS]; 13] = {
    let mut table = [[0.0; CHANNELS]; 13];
    let mut c = 1;
    while c < 13 {
        let slope = (c - 1) / 4;
        let grip = (c - 1) % 4;
        table[c] = [
            1.0,
            match slope {
                0 => 1.0,
                1 => 0.5,
                _ => 0.0,
            },
            if grip == 1 { 1.0 } else { 0.0 },
            if grip == 2 { 1.0 } else { 0.0 },
        ];
        c += 1;
    }
    table
};

pub fn features(code: u8) -> [f32; CHANNELS] {
    TABLE.get(code as usize).copied().unwrap_or([0.0; CHANNELS])
}

impl Voxels {
    pub fn build(session: &Session) -> Result<Self> {
        let mut low = session.config.initial_pose.pos.to_array();
        let mut high = low;
        for gate in gates(session)? {
            for b in &gate.boxes {
                low = std::array::from_fn(|k| low[k].min(b.min[k]));
                high = std::array::from_fn(|k| high[k].max(b.max[k]));
            }
        }
        let margin = [MARGIN, MARGIN_UP, MARGIN];
        let low = Vec3::from(std::array::from_fn::<f32, 3, _>(|k| low[k] - margin[k]));
        let high = Vec3::from(std::array::from_fn::<f32, 3, _>(|k| high[k] + margin[k]));
        let size = [
            (((high.x - low.x) / CELL).ceil() as usize).min(MAX_ACROSS),
            (((high.y - low.y) / CELL).ceil() as usize).min(MAX_UP),
            (((high.z - low.z) / CELL).ceil() as usize).min(MAX_ACROSS),
        ];
        let origin = [
            (low.x + high.x) * 0.5 - size[0] as f32 * CELL * 0.5,
            (low.y + high.y) * 0.5 - size[1] as f32 * CELL * 0.5,
            (low.z + high.z) * 0.5 - size[2] as f32 * CELL * 0.5,
        ];
        let mut columns: Vec<Vec<(u16, u8)>> = vec![Vec::new(); size[0] * size[2]];
        let mut mark = |p: Vec3, normal: Vec3, material: u8| {
            let cell = [
                ((p.x - origin[0]) / CELL).floor(),
                ((p.y - origin[1]) / CELL).floor(),
                ((p.z - origin[2]) / CELL).floor(),
            ];
            if cell.iter().zip(size).any(|(&c, n)| c < 0.0 || c >= n as f32) {
                return;
            }
            let column = &mut columns[cell[0] as usize * size[2] + cell[2] as usize];
            let y = cell[1] as u16;
            let c = code(normal, material);
            match column.iter_mut().find(|(h, _)| *h == y) {
                Some(slot) if c < slot.1 => slot.1 = c,
                Some(_) => {}
                None => column.push((y, c)),
            }
        };
        let geometry = Geometry::new(&session.world, &session.state);
        let manager = &session.state.collision_world[session.config.manager];
        let filter = QueryFilter {
            lists: Lists::SURFACES,
            layers: 0x12_8002,
            excluded_flags: ItemFlags::NONE,
            ..QueryFilter::default()
        };
        let mut cast_one = |start: Vec3, along: Vec3, length: f32| {
            let mut from = 0.0;
            for _ in 0..MAX_HITS {
                let origin = start + along * from;
                let Some(hit) =
                    manager.raycast(geometry, Ray { origin, direction: along * (length - from) }, filter)
                else {
                    break;
                };
                let travelled = hit.fraction * (length - from);
                mark(origin + along * travelled, hit.normal, hit.material as u8);
                from += travelled + 0.05;
                if from >= length {
                    break;
                }
            }
        };
        let mut cast = |start: Vec3, along: Vec3, length: f32| {
            cast_one(start, along, length);
            cast_one(start + along * length, along * -1.0, length);
        };
        let [nx, ny, nz] = size.map(|n| n as f32 * CELL);
        let centre = |i: usize| (i as f32 + 0.5) * CELL;
        for ix in 0..size[0] {
            for iz in 0..size[2] {
                let top = Vec3::new(origin[0] + centre(ix), origin[1] + ny, origin[2] + centre(iz));
                cast(top, Vec3::new(0.0, -1.0, 0.0), ny);
            }
        }
        for iy in 0..size[1] {
            for iz in 0..size[2] {
                let side = Vec3::new(origin[0], origin[1] + centre(iy), origin[2] + centre(iz));
                cast(side, Vec3::new(1.0, 0.0, 0.0), nx);
            }
            for ix in 0..size[0] {
                let side = Vec3::new(origin[0] + centre(ix), origin[1] + centre(iy), origin[2]);
                cast(side, Vec3::new(0.0, 0.0, 1.0), nz);
            }
        }
        let entries: usize = columns.iter().map(Vec::len).sum();
        let mut bytes = MAGIC.to_vec();
        bytes.extend(FORMAT.to_le_bytes());
        for v in origin {
            bytes.extend(v.to_le_bytes());
        }
        for n in size {
            bytes.extend((n as u32).to_le_bytes());
        }
        bytes.extend((entries as u32).to_le_bytes());
        let mut at = 0u32;
        for column in &mut columns {
            column.sort_unstable();
            bytes.extend(at.to_le_bytes());
            at += column.len() as u32;
        }
        bytes.extend(at.to_le_bytes());
        for (y, _) in columns.iter().flatten() {
            bytes.extend(y.to_le_bytes());
        }
        bytes.extend(columns.iter().flatten().map(|&(_, c)| c));
        Self::read(bytes)
    }
}

impl<B: AsRef<[u8]>> Voxels<B> {
    pub fn read(bytes: B) -> Result<Self> {
        let b = bytes.as_ref();
        ensure!(b.len() >= HEADER && &b[..4] == MAGIC, "not a voxel map");
        let word = |at: usize| u32::from_le_bytes(b[at..at + 4].try_into().unwrap());
        let format = word(4);
        ensure!(format == FORMAT, "voxel map format {format}, this build reads {FORMAT}");
        let origin = [8, 12, 16].map(|at| f32::from_bits(word(at)));
        let size = [20, 24, 28].map(|at| word(at) as usize);
        let entries = word(32) as usize;
        let columns = size[0].checked_mul(size[2]).context("voxel map too large")?;
        let expected = HEADER + 4 * (columns + 1) + 3 * entries;
        ensure!(b.len() == expected, "voxel map is {} bytes, its header says {expected}", b.len());
        ensure!(word(HEADER + 4 * columns) as usize == entries, "voxel map columns don't add up");
        Ok(Self { bytes, origin, size, entries })
    }

    pub fn bytes(&self) -> &[u8] {
        self.bytes.as_ref()
    }

    pub fn occupied(&self) -> usize {
        self.entries
    }

    fn column(&self, index: usize) -> impl Iterator<Item = (u16, u8)> + '_ {
        let b = self.bytes.as_ref();
        let columns = self.size[0] * self.size[2];
        let start =
            |i: usize| u32::from_le_bytes(b[HEADER + 4 * i..HEADER + 4 * i + 4].try_into().unwrap()) as usize;
        let (from, to) = (start(index).min(self.entries), start(index + 1).min(self.entries));
        let heights = HEADER + 4 * (columns + 1);
        let codes = heights + 2 * self.entries;
        (from..to.max(from))
            .map(move |e| (u16::from_le_bytes([b[heights + 2 * e], b[heights + 2 * e + 1]]), b[codes + e]))
    }

    pub fn crop(&self, pose: &Pose) -> [u8; CELLS] {
        let mut out = [0; CELLS];
        let (sin, cos) = pose.yaw.sin_cos();
        let [x, y, z] = pose.position;
        let low = ((y - self.origin[1]) / CELL - UP as f32 * 0.5).floor();
        for f in 0..FORWARD {
            let ahead = (f as f32 - BEHIND as f32 + 0.5) * CELL;
            for a in 0..ACROSS {
                let side = (a as f32 - ACROSS as f32 * 0.5 + 0.5) * CELL;
                let wx = x + sin * ahead + cos * side;
                let wz = z + cos * ahead - sin * side;
                let cx = ((wx - self.origin[0]) / CELL).floor();
                let cz = ((wz - self.origin[2]) / CELL).floor();
                if !(cx >= 0.0 && cz >= 0.0 && cx < self.size[0] as f32 && cz < self.size[2] as f32) {
                    continue;
                }
                for (h, c) in self.column(cx as usize * self.size[2] + cz as usize) {
                    let u = h as f32 - low;
                    if (0.0..UP as f32).contains(&u) {
                        out[(u as usize * ACROSS + a) * FORWARD + f] = c;
                    }
                }
            }
        }
        out
    }
}

pub fn expand(codes: &[u8; CELLS], out: &mut [f32]) {
    for (i, &c) in codes.iter().enumerate() {
        for (channel, value) in features(c).into_iter().enumerate() {
            out[channel * CELLS + i] = value;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floor_under_origin() -> Voxels {
        let size = [40u32, 10, 48];
        let columns = (size[0] * size[2]) as usize;
        let strip: Vec<usize> = (0..columns).filter(|c| c / 48 == 20 && c % 48 >= 21).collect();
        let mut bytes = MAGIC.to_vec();
        bytes.extend(FORMAT.to_le_bytes());
        for v in [-80.0f32, -20.0, -80.0] {
            bytes.extend(v.to_le_bytes());
        }
        for n in size {
            bytes.extend(n.to_le_bytes());
        }
        bytes.extend((strip.len() as u32).to_le_bytes());
        let mut at = 0u32;
        for column in 0..=columns {
            bytes.extend(at.to_le_bytes());
            at += u32::from(strip.contains(&column));
        }
        for _ in &strip {
            bytes.extend(5u16.to_le_bytes());
        }
        bytes.extend(strip.iter().map(|_| 1u8));
        Voxels::read(bytes).unwrap()
    }

    #[test]
    fn a_strip_ahead_of_the_car_shows_ahead_whatever_the_heading() {
        let voxels = floor_under_origin();
        let facing_z = voxels.crop(&Pose { position: [2.0, 2.0, 2.0], yaw: 0.0 });
        let row = |codes: &[u8; CELLS], u: usize, a: usize| -> Vec<u8> {
            (0..FORWARD).map(|f| codes[(u * ACROSS + a) * FORWARD + f]).collect()
        };
        let level = UP / 2;
        let middle = ACROSS / 2 - 1;
        assert!(row(&facing_z, level, middle)[BEHIND..].iter().all(|&c| c == 1));
        assert!(row(&facing_z, level, middle)[..BEHIND].iter().all(|&c| c == 0));
        let facing_back = voxels.crop(&Pose { position: [2.0, 2.0, 2.0], yaw: std::f32::consts::PI });
        assert!(row(&facing_back, level, middle)[BEHIND..].iter().all(|&c| c == 0));
    }

    #[test]
    fn a_map_survives_its_byte_form_and_rejects_damage() {
        let voxels = floor_under_origin();
        let bytes = voxels.bytes();
        let view = Voxels::read(bytes).unwrap();
        let pose = Pose { position: [2.0, 2.0, 2.0], yaw: 0.3 };
        assert_eq!(view.crop(&pose), voxels.crop(&pose));
        assert!(Voxels::read(&bytes[..bytes.len() - 1]).is_err());
        let mut longer = bytes.to_vec();
        longer.push(0);
        assert!(Voxels::read(longer).is_err());
    }

    #[test]
    fn flat_road_wall_and_ice_read_as_different_features() {
        let up = Vec3::new(0.0, 1.0, 0.0);
        let side = Vec3::new(1.0, 0.0, 0.0);
        let road = features(code(up, gearbox_engine::surface::ASPHALT));
        let wall = features(code(side, gearbox_engine::surface::ASPHALT));
        let ice = features(code(up, gearbox_engine::surface::ICE));
        let dirt = features(code(up, gearbox_engine::surface::DIRT));
        assert_eq!(road, [1.0, 1.0, 0.0, 0.0]);
        assert_eq!(wall, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(ice, [1.0, 1.0, 0.0, 1.0]);
        assert_eq!(dirt, [1.0, 1.0, 1.0, 0.0]);
        assert_eq!(features(0), [0.0; CHANNELS]);
    }
}
