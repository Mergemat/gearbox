use anyhow::{Context, Result, ensure};
use gearbox_engine::collision::{ItemFlags, Lists, QueryFilter, Ray};
use gearbox_engine::shape::Geometry;
use gearbox_engine::{Session, Vec3};
use gearbox_search::gate::gates;

use crate::observe::{SURFACES, surface_class};

pub const CELL: f32 = 4.0;
pub const FORWARD: usize = 24;
pub const BEHIND: usize = 4;
pub const ACROSS: usize = 16;
pub const UP: usize = 8;
pub const CELLS: usize = FORWARD * ACROSS * UP;
pub const LOCAL_CHANNELS: usize = 4 + SURFACES;
pub const GLOBAL_CHANNELS: usize = 5;
pub const CHANNELS: usize = LOCAL_CHANNELS + GLOBAL_CHANNELS;
pub const FEATURES: usize = CHANNELS * CELLS;
pub const GLOBAL_GEOMETRY: usize = LOCAL_CHANNELS;
pub const GLOBAL_UNPASSED: usize = LOCAL_CHANNELS + 1;
pub const GLOBAL_PASSED: usize = LOCAL_CHANNELS + 2;
pub const GLOBAL_FINISH: usize = LOCAL_CHANNELS + 3;
pub const GLOBAL_CAR: usize = LOCAL_CHANNELS + 4;

const MARGIN: f32 = 96.0;
const MARGIN_UP: f32 = 32.0;
const MAX_ACROSS: usize = 640;
const MAX_UP: usize = 128;
const MAX_HITS: usize = 64;
const MAGIC: &[u8; 4] = b"GVOX";
const FORMAT: u32 = 2;
const VERTICAL_SAMPLES: [(f32, f32); 3] =
    [(0.0, 0.0), (-CELL * 0.25, -CELL * 0.25), (CELL * 0.25, CELL * 0.25)];

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
    coarse: Vec<f32>,
}

const HEADER: usize = 4 + 4 + 12 + 12 + 4;

fn code(normal: Vec3, material: u8) -> u32 {
    // Separate masks retain concurrent contacts without choosing one winning hit.
    let direction = if normal.y > 0.5 {
        0
    } else if normal.y < -0.5 {
        1
    } else {
        2
    };
    (1 << direction) | (1 << (3 + surface_class(material)))
}

fn add_hit(column: &mut Vec<(u16, u32)>, y: u16, code: u32) {
    match column.iter_mut().find(|(h, _)| *h == y) {
        Some(slot) => slot.1 |= code,
        None => column.push((y, code)),
    }
}

pub fn features(code: u32) -> [f32; LOCAL_CHANNELS] {
    std::array::from_fn(|channel| {
        if channel == 0 { f32::from(code != 0) } else { f32::from(code & (1 << (channel - 1)) != 0) }
    })
}

impl Voxels {
    pub fn build(session: &Session) -> Result<Self> {
        Self::build_with_positions(session, std::iter::empty())
    }

    pub fn build_with_positions(
        session: &Session,
        positions: impl IntoIterator<Item = [f32; 3]>,
    ) -> Result<Self> {
        let mut low = session.config.initial_pose.pos.to_array();
        let mut high = low;
        let mut include = |min: [f32; 3], max: [f32; 3]| {
            low = std::array::from_fn(|k| low[k].min(min[k]));
            high = std::array::from_fn(|k| high[k].max(max[k]));
        };
        for gate in gates(session)? {
            for b in &gate.boxes {
                include(b.min, b.max);
            }
        }
        for position in positions {
            include(position, position);
        }
        let mut include_box = |b: gearbox_engine::Aabb| {
            if b.half.x >= 0.0 && b.half.y >= 0.0 && b.half.z >= 0.0 {
                include((b.center - b.half).to_array(), (b.center + b.half).to_array());
            }
        };
        include_box(session.world.statics.tree.bounds);
        for entry in &session.state.shape_entry {
            include_box(entry.bounds);
        }
        for entry in &session.state.continuous_entry {
            include_box(entry.bounds);
        }
        let margin = [MARGIN, MARGIN_UP, MARGIN];
        let low = Vec3::from(std::array::from_fn::<f32, 3, _>(|k| low[k] - margin[k]));
        let high = Vec3::from(std::array::from_fn::<f32, 3, _>(|k| high[k] + margin[k]));
        let size = [
            ((high.x - low.x) / CELL).ceil() as usize,
            ((high.y - low.y) / CELL).ceil() as usize,
            ((high.z - low.z) / CELL).ceil() as usize,
        ];
        ensure!(
            size[0] <= MAX_ACROSS && size[1] <= MAX_UP && size[2] <= MAX_ACROSS,
            "collision bounds exceed voxel map capacity: {size:?} cells"
        );
        let origin = [
            (low.x + high.x) * 0.5 - size[0] as f32 * CELL * 0.5,
            (low.y + high.y) * 0.5 - size[1] as f32 * CELL * 0.5,
            (low.z + high.z) * 0.5 - size[2] as f32 * CELL * 0.5,
        ];
        let mut columns: Vec<Vec<(u16, u32)>> = vec![Vec::new(); size[0] * size[2]];
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
            add_hit(column, y, c);
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
                for (dx, dz) in VERTICAL_SAMPLES {
                    let top =
                        Vec3::new(origin[0] + centre(ix) + dx, origin[1] + ny, origin[2] + centre(iz) + dz);
                    cast(top, Vec3::new(0.0, -1.0, 0.0), ny);
                }
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
        for (_, c) in columns.iter().flatten() {
            bytes.extend(c.to_le_bytes());
        }
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
        ensure!(
            size.iter().all(|&n| n > 0)
                && size[0] <= MAX_ACROSS
                && size[1] <= MAX_UP
                && size[2] <= MAX_ACROSS,
            "invalid voxel map size {size:?}"
        );
        let entries = word(32) as usize;
        let columns = size[0].checked_mul(size[2]).context("voxel map too large")?;
        let expected = HEADER + 4 * (columns + 1) + 6 * entries;
        ensure!(b.len() == expected, "voxel map is {} bytes, its header says {expected}", b.len());
        ensure!(word(HEADER + 4 * columns) as usize == entries, "voxel map columns don't add up");
        let mut map = Self { bytes, origin, size, entries, coarse: Vec::new() };
        let mut coarse = vec![0.0; CELLS];
        for x in 0..size[0] {
            for z in 0..size[2] {
                for (y, _) in map.column(x * size[2] + z) {
                    let point = [
                        origin[0] + (x as f32 + 0.5) * CELL,
                        origin[1] + (y as f32 + 0.5) * CELL,
                        origin[2] + (z as f32 + 0.5) * CELL,
                    ];
                    if let Some(index) = map.global_index(point) {
                        coarse[index] = 1.0;
                    }
                }
            }
        }
        map.coarse = coarse;
        Ok(map)
    }

    pub fn bytes(&self) -> &[u8] {
        self.bytes.as_ref()
    }

    pub fn occupied(&self) -> usize {
        self.entries
    }

    fn column(&self, index: usize) -> impl Iterator<Item = (u16, u32)> + '_ {
        let b = self.bytes.as_ref();
        let columns = self.size[0] * self.size[2];
        let start =
            |i: usize| u32::from_le_bytes(b[HEADER + 4 * i..HEADER + 4 * i + 4].try_into().unwrap()) as usize;
        let (from, to) = (start(index).min(self.entries), start(index + 1).min(self.entries));
        let heights = HEADER + 4 * (columns + 1);
        let codes = heights + 2 * self.entries;
        (from..to.max(from)).map(move |e| {
            (
                u16::from_le_bytes([b[heights + 2 * e], b[heights + 2 * e + 1]]),
                u32::from_le_bytes(b[codes + 4 * e..codes + 4 * e + 4].try_into().unwrap()),
            )
        })
    }

    pub fn crop(&self, pose: &Pose) -> [u32; CELLS] {
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
                        out[(u as usize * ACROSS + a) * FORWARD + f] |= c;
                    }
                }
            }
        }
        out
    }

    pub(crate) fn global_index(&self, point: [f32; 3]) -> Option<usize> {
        // These bins span the full saved map; local crop cells remain four metres wide.
        let bins = [ACROSS, UP, FORWARD];
        let mut index = [0; 3];
        for k in 0..3 {
            let relative = (point[k] - self.origin[k]) / (self.size[k] as f32 * CELL);
            if !(0.0..1.0).contains(&relative) {
                return None;
            }
            index[k] = (relative * bins[k] as f32) as usize;
        }
        Some((index[1] * ACROSS + index[0]) * FORWARD + index[2])
    }

    pub(crate) fn global(&self, out: &mut [f32]) {
        out[GLOBAL_GEOMETRY * CELLS..(GLOBAL_GEOMETRY + 1) * CELLS].copy_from_slice(&self.coarse);
    }

    pub(crate) fn mark_global(&self, point: [f32; 3], channel: usize, out: &mut [f32]) {
        debug_assert!((GLOBAL_UNPASSED..CHANNELS).contains(&channel));
        if let Some(index) = self.global_index(point) {
            out[channel * CELLS + index] += 1.0;
        }
    }
}

pub fn expand(codes: &[u32; CELLS], out: &mut [f32]) {
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
        for _ in &strip {
            bytes.extend(code(Vec3::new(0.0, 1.0, 0.0), gearbox_engine::surface::ASPHALT).to_le_bytes());
        }
        Voxels::read(bytes).unwrap()
    }

    #[test]
    fn a_strip_ahead_of_the_car_shows_ahead_whatever_the_heading() {
        let voxels = floor_under_origin();
        let facing_z = voxels.crop(&Pose { position: [2.0, 2.0, 2.0], yaw: 0.0 });
        let row = |codes: &[u32; CELLS], u: usize, a: usize| -> Vec<u32> {
            (0..FORWARD).map(|f| codes[(u * ACROSS + a) * FORWARD + f]).collect()
        };
        let level = UP / 2;
        let middle = ACROSS / 2 - 1;
        assert!(row(&facing_z, level, middle)[BEHIND..].iter().all(|&c| c != 0));
        assert!(row(&facing_z, level, middle)[..BEHIND].iter().all(|&c| c == 0));
        let facing_back = voxels.crop(&Pose { position: [2.0, 2.0, 2.0], yaw: std::f32::consts::PI });
        assert!(row(&facing_back, level, middle)[BEHIND..].iter().all(|&c| c == 0));
        let mut global = vec![0.0; FEATURES];
        voxels.global(&mut global);
        let distant = voxels.global_index([2.0, 2.0, 100.0]).unwrap();
        assert_eq!(global[GLOBAL_GEOMETRY * CELLS + distant], 1.0);
    }

    #[test]
    fn a_map_survives_its_byte_form_and_rejects_damage() {
        let voxels = floor_under_origin();
        let bytes = voxels.bytes();
        let view = Voxels::read(bytes).unwrap();
        let pose = Pose { position: [2.0, 2.0, 2.0], yaw: 0.3 };
        assert_eq!(view.crop(&pose), voxels.crop(&pose));
        let mut stale = bytes.to_vec();
        stale[4..8].copy_from_slice(&1u32.to_le_bytes());
        assert!(Voxels::read(stale).is_err());
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
        assert_eq!(&road[..4], &[1.0, 1.0, 0.0, 0.0]);
        assert_eq!(&wall[..4], &[1.0, 0.0, 0.0, 1.0]);
        assert_eq!(&ice[..4], &[1.0, 1.0, 0.0, 0.0]);
        assert_eq!(&dirt[..4], &[1.0, 1.0, 0.0, 0.0]);
        assert_ne!(road, ice);
        assert_ne!(road, dirt);
        assert_eq!(features(0), [0.0; LOCAL_CHANNELS]);
    }

    #[test]
    fn ceiling_and_distinct_hard_surfaces_do_not_alias() {
        let up = Vec3::new(0.0, 1.0, 0.0);
        let down = Vec3::new(0.0, -1.0, 0.0);
        assert_ne!(
            features(code(up, gearbox_engine::surface::ASPHALT)),
            features(code(down, gearbox_engine::surface::ASPHALT))
        );
        let materials = [
            gearbox_engine::surface::ASPHALT,
            gearbox_engine::surface::PLASTIC,
            gearbox_engine::surface::WOOD,
            gearbox_engine::surface::METAL,
        ];
        let distinct: std::collections::HashSet<_> =
            materials.into_iter().map(|material| code(up, material)).collect();
        assert_eq!(distinct.len(), materials.len());
    }

    #[test]
    fn contacts_in_one_cell_keep_both_orientations_and_materials() {
        let mut column = Vec::new();
        add_hit(&mut column, 3, code(Vec3::new(0.0, 1.0, 0.0), gearbox_engine::surface::ASPHALT));
        add_hit(&mut column, 3, code(Vec3::new(0.0, -1.0, 0.0), gearbox_engine::surface::WOOD));
        assert_eq!(column.len(), 1);
        let merged = features(column[0].1);
        assert_eq!(&merged[..4], &[1.0, 1.0, 1.0, 0.0]);
        assert_eq!(merged[4 + surface_class(gearbox_engine::surface::ASPHALT)], 1.0);
        assert_eq!(merged[4 + surface_class(gearbox_engine::surface::WOOD)], 1.0);
    }

    #[test]
    fn offset_rays_reach_a_strip_the_centre_ray_misses() {
        use gearbox_engine::static_world::{
            Convex, Entity, Face, Geometry as StaticGeometry, Item, Shape, Surface,
        };
        use gearbox_engine::tree::{AabbTree, Leaf};
        use gearbox_engine::{Aabb, Iso, State, World};

        let bounds = Aabb::new(Vec3::new(1.0, 0.0, 1.0), Vec3::new(0.25, 0.01, 0.25));
        let mut world = World::default();
        world.statics.shapes.push(Shape {
            main_dir: Vec3::ZERO,
            geometry: StaticGeometry::Convex(Convex {
                material: gearbox_engine::surface::ASPHALT as u16,
                vertices: vec![
                    Vec3::new(0.75, 0.0, 0.75),
                    Vec3::new(0.75, 0.0, 1.25),
                    Vec3::new(1.25, 0.0, 1.25),
                    Vec3::new(1.25, 0.0, 0.75),
                ],
                indices: vec![0, 1, 2, 3],
                faces: vec![Face {
                    normal: Vec3::new(0.0, 1.0, 0.0),
                    offset: 0.0,
                    axis: 1,
                    positive: true,
                    start: 0,
                    count: 4,
                }],
                edges: vec![],
                normals: vec![],
                directions: vec![],
                bounds,
            }),
        });
        world.statics.items.push(Item {
            entity: Entity::None,
            shape: 0,
            iso: Iso::IDENTITY,
            layer: 2,
            flags: Default::default(),
            query_flags: Default::default(),
            id: 0,
            index: 0,
        });
        world.statics.surfaces.push(Surface { shape: 0, iso: Iso::IDENTITY, item: 0, tag: 0 });
        world.statics.tree.nodes = AabbTree::build(&mut [Leaf { bounds, index: 0 }], 1).nodes;
        let state = State::default();
        let geometry = Geometry::new(&world, &state);
        let manager = gearbox_engine::collision::CollisionWorld::default();
        let filter = QueryFilter { lists: Lists::SURFACES, layers: 0x12_8002, ..QueryFilter::default() };
        let hit = |(x, z): (f32, f32)| {
            manager.raycast(
                geometry,
                Ray { origin: Vec3::new(x, 2.0, z), direction: Vec3::new(0.0, -4.0, 0.0) },
                filter,
            )
        };
        assert!(hit(VERTICAL_SAMPLES[0]).is_none());
        assert!(VERTICAL_SAMPLES[1..].iter().copied().any(|sample| hit(sample).is_some()));
    }

    #[test]
    fn exceptional_surface_effects_have_separate_codes() {
        let up = Vec3::new(0.0, 1.0, 0.0);
        let materials = [
            gearbox_engine::surface::ASPHALT,
            gearbox_engine::surface::TURBO,
            gearbox_engine::surface::FREE_WHEELING,
            gearbox_engine::surface::NO_GRIP,
            gearbox_engine::surface::NO_STEERING,
            gearbox_engine::surface::NO_BRAKES,
        ];
        let distinct: std::collections::HashSet<_> =
            materials.into_iter().map(|material| code(up, material)).collect();
        assert_eq!(distinct.len(), materials.len());
    }
}
