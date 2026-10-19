use crate::statics::{
    model::{BlockKey, ModelSource, catalog::GeometrySource},
    place,
};
use std::collections::HashMap;
use anyhow::{Result, Context};
pub type Location = ([f32; 9], [f32; 3]);
const IDENTITY: [f32; 9] = [1., 0., 0., 0., 1., 0., 0., 0., 1.];

pub fn contact(location: Location, clip_type: u32, cell: [f32; 3]) -> Location {
    let [s, h, _] = cell;
    let p = match clip_type {
        0 | 1 => [s * 0.5, h * 0.5, s],
        2 => [s * 0.5, 0., s * 0.5],
        3 => [s * 0.5, h, s * 0.5],
        _ => [0.; 3],
    };
    place::compose_iso((IDENTITY, p), location)
}

pub fn unit_location(
    unit: [i32; 3], clips: &gearbox_packs::blockinfo::UnitClips, face: usize, index: usize, cell: [f32; 3],
) -> Location {
    let dir = if face < 4 {
        (face as u32 + 2) & 3
    } else if index < 5 {
        (clips.directions[face - 4] as u32 >> (index * 2)) & 3
    } else {
        0
    };
    let c = super::neighbour(unit, face as u32);
    place::grid_location_cell([cell[0], cell[1], 0.], c, dir, [1; 3])
}

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    let d = std::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
    (d[1] * d[1] + d[0] * d[0]) + d[2] * d[2]
}

fn same_rotation(a: [f32; 9], b: [f32; 9]) -> bool {
    (0..3).all(|row| {
        let a = std::array::from_fn(|column| a[3 * row + column]);
        let b = std::array::from_fn(|column| b[3 * row + column]);
        distance(a, b) < 1e-10
    })
}

struct Entry<'a> {
    block: &'a gearbox_gbx::map::Block,
    index: usize,
    contact: Location,
    parent: Option<(usize, usize)>,
}
struct Bucket {
    point: [f32; 3],
    entries: Vec<usize>,
}

fn cells(point: [f32; 3], width: f32) -> [[i32; 3]; 8] {
    let inv = 1. / width;
    let p = point.map(|v| v * inv);
    let lo = p.map(|v| gearbox_engine::math::cvtt(gearbox_engine::numeric::floorf(v)));
    let hi = p.map(|v| gearbox_engine::math::cvtt(gearbox_engine::numeric::ceilf(v)));
    std::array::from_fn(|i| [
        if i & 4 == 0 { lo[0] } else { hi[0] },
        if i & 2 == 0 { lo[1] } else { hi[1] },
        if i & 1 == 0 { lo[2] } else { hi[2] },
    ])
}
fn bucket_at(
    buckets: &[Bucket], bins: &HashMap<[i32; 3], Vec<usize>>,
    point: [f32; 3], width: f32, tolerance: f32,
) -> Option<usize> {
    cells(point, width).into_iter().filter_map(|cell| bins.get(&cell)).flatten().copied()
        .find(|&i| distance(buckets[i].point, point) < tolerance)
}

pub(super) fn free_parents(
    map: &gearbox_gbx::Map, src: &dyn ModelSource,
) -> Result<HashMap<usize, (usize, usize)>> {
    let mut entries = Vec::new();
    let mut buckets: Vec<Bucket> = Vec::new();
    let mut tolerance = 0.;
    let mut width = 0.;
    let mut bins: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    for (index, block) in map.baked_blocks.iter().enumerate() {
        let Some((position, angles)) = block.free else { continue };
        let Some(geometry) = src.block_geometry(&BlockKey::of(block)) else { continue };
        if geometry.kind != GeometrySource::Clip {
            continue;
        }
        let t = geometry.cell[0] * 0.001;
        tolerance = t * t;
        width = geometry.cell[0] * 0.5;
        let location = place::free_location_variant(position, angles, Default::default());
        let contact = contact(location, geometry.clip_type, geometry.cell);
        let bucket = match bucket_at(&buckets, &bins, contact.1, width, tolerance) {
            Some(i) => i,
            None => {
                let i = buckets.len();
                buckets.push(Bucket { point: contact.1, entries: Vec::new() });
                for c in cells(contact.1, width) {
                    let list = bins.entry(c).or_default();
                    if list.last() != Some(&i) {
                        list.push(i);
                    }
                }
                i
            }
        };
        buckets[bucket].entries.push(entries.len());
        entries.push(Entry { block, index, contact, parent: None });
    }
    if entries.is_empty() {
        return Ok(HashMap::new());
    }
    for (i, block) in map.blocks.iter().enumerate() {
        let Some((position, angles)) = block.free else { continue };
        let Some(geometry) = src.block_geometry(&BlockKey::of(block)) else { continue };
        let parent_location = place::free_location_variant(position, angles, Default::default());
        for (unit, clips) in geometry.units.iter().zip(geometry.clip_faces.iter()) {
            for (face, names) in clips.faces.iter().enumerate() {
                if names.is_empty() {
                    continue;
                }
                let ground = || -> Result<bool> {
                    if face >= 4 { return Ok(false); }
                    let zone = geometry.ground_heights.iter().find(|(cell, _)| *cell == [unit[0], unit[2]]);
                    let Some((_, height)) = zone
                    else { return Ok(false) };
                    let height = height.with_context(|| format!(
                        "non-flat free clip ground zone on {}", block.name,
                    ))?;
                    Ok(unit[1] == height)
                };
                for (j, name) in names.iter().enumerate() {
                    let Some(name) = name else { continue };
                    let local = unit_location(*unit, clips, face, j, geometry.cell);
                    let placed = place::compose_iso(local, parent_location);
                    let key = BlockKey {
                        name: gearbox_packs::assets::block_name(name).into(),
                        variant: 0, sub_variant: 0, flags: 0,
                    };
                    let Some(clip_geometry) = src.block_geometry(&key)
                    else {
                        continue;
                    };
                    let expected = contact(placed, clip_geometry.clip_type, geometry.cell);
                    let Some(bucket) = bucket_at(&buckets, &bins, expected.1, width, tolerance)
                    else { continue };
                    for &k in buckets[bucket].entries.iter().rev() {
                        let entry = &mut entries[k];
                        if entry.parent.is_none()
                            && entry.block.name.eq_ignore_ascii_case(gearbox_packs::assets::block_name(name))
                            && same_rotation(expected.0, entry.contact.0)
                            && entry.block.is_ground() == ground()?
                        {
                            entry.parent = Some((0, i));
                            break;
                        }
                    }
                }
            }
        }
    }
    Ok(entries.into_iter().filter_map(|entry| Some((entry.index, entry.parent?))).collect())
}
