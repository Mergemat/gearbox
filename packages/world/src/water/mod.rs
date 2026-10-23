mod terrain;

use crate::statics::{BlockKey, MapAssets, place};
use anyhow::{Context, Result, ensure};
use gearbox_engine::world_water::{Column, Entry, Grid, Layer, ModelBox, Volume, VoxelGrid, Water};
use gearbox_engine::{Aabb, Iso, Quat, Vec3};
use gearbox_packs::assets::BlockLookup;
use std::collections::HashMap;

const EMPTY: u32 = u32::MAX;
const MAP_CELL: f32 = 32.0;
const UPRIGHT_TOLERANCE: f32 = 0.00001;

fn map_grid(size: [u32; 2]) -> Grid {
    Grid {
        cell: [MAP_CELL; 2],
        inverse_cell: [1.0 / MAP_CELL; 2],
        origin: [0.0; 2],
        size,
        default_value: EMPTY,
    }
}

pub fn grid(volume: &gearbox_packs::map_model::WaterVolume) -> Result<(VoxelGrid, place::Iso)> {
    ensure!(!volume.boxes.is_empty(), "unsupported empty water volume");
    ensure!(
        volume.boxes.iter().all(|extent| (0..3).all(|k| extent[k] >= 0 && extent[k] <= extent[k + 3])),
        "invalid water voxel extent",
    );
    let lo: [i32; 3] = std::array::from_fn(|k| volume.boxes.iter().map(|extent| extent[k]).min().unwrap());
    let hi: [i32; 3] =
        std::array::from_fn(|k| volume.boxes.iter().map(|extent| extent[k + 3]).max().unwrap());
    let size = [(hi[0] - lo[0] + 1) as u32, (hi[2] - lo[2] + 1) as u32];
    ensure!(size[0] as u64 * size[1] as u64 <= 1 << 20, "water grid too large");
    let angle = volume.yaw * std::f32::consts::PI / 180.0;
    let (sin, cos) = gearbox_engine::math::sincos(angle * 0.5);
    let local = place::Iso {
        q: [cos, 0.0, sin, 0.0],
        p: [volume.position[0], lo[1] as f32 * volume.cell[1] + volume.position[1], volume.position[2]],
    };
    let placement = Iso::from_quat_pos(Quat::from_array(local.q), Vec3::from_array(local.p));
    let mut grid = VoxelGrid {
        material: 0,
        layers: Vec::new(),
        grid: Grid {
            cell: [volume.cell[0], volume.cell[2]],
            inverse_cell: [1.0 / volume.cell[0], 1.0 / volume.cell[2]],
            origin: [lo[0] as f32 * volume.cell[0], lo[2] as f32 * volume.cell[2]],
            size,
            default_value: EMPTY,
        },
        heads: Vec::new(),
        entries: Vec::new(),
        column_count: size[0] * size[1],
        height: volume.cell[1],
        max_y: (hi[1] - lo[1]) as u32,
        placement,
    };
    for z in lo[2]..=hi[2] {
        for x in lo[0]..=hi[0] {
            let mut spans: Vec<_> = volume
                .boxes
                .iter()
                .filter(|extent| x >= extent[0] && x <= extent[3] && z >= extent[2] && z <= extent[5])
                .map(|extent| (extent[1] - lo[1], extent[4] - lo[1]))
                .collect();
            crate::sort_by_key(&mut spans, |span| span.0);
            let mut merged: Vec<(i32, i32)> = Vec::new();
            for (bottom, top) in spans {
                if let Some(last) = merged.last_mut()
                    && bottom <= last.1 + 1
                {
                    last.1 = top;
                } else {
                    merged.push((bottom, top));
                }
            }
            let mut head = EMPTY;
            for (bottom, top) in merged {
                let layer = grid.layers.len() as u32;
                grid.layers.push(Layer { bottom: bottom as u32, top: top as u32 });
                let index = grid.entries.len() as u32;
                grid.entries.push(Entry { value: layer, next: head });
                head = index;
            }
            grid.heads.push(head);
        }
    }
    Ok((grid, local))
}

pub fn build(map: &gearbox_gbx::Map, assets: &MapAssets<'_>) -> Result<Water> {
    let catalog = assets.geometry(map)?;
    let env = crate::statics::packs::environment(map).map_err(anyhow::Error::msg)?;
    let mut water = Water::default();
    let mut cache = HashMap::new();
    let mut boxes = Vec::new();
    for block in map.blocks.iter().chain(&map.baked_blocks) {
        let asset = match assets.assets.block_in(&block.name, env) {
            BlockLookup::Block(asset) => asset,
            BlockLookup::Null => continue,
            BlockLookup::Missing => anyhow::bail!("water block {}", block.name),
        };
        let key = BlockKey::of(block);
        let cache_key = (asset.path.clone(), key.flags & 0x1000 != 0, ((key.flags >> 21) & 0x7f) as usize);
        if !cache.contains_key(&cache_key) {
            let bytes = assets.packs.read_path(&asset.path)?;
            let volumes =
                gearbox_packs::map_model::water_volumes(&bytes, &asset.path, cache_key.1, cache_key.2)
                    .with_context(|| format!("water block {}", asset.path))?;
            let mut models = Vec::new();
            for volume in volumes {
                let (grid, local) = grid(&volume)?;
                let index = water.models.len() as u32;
                water.models.push(grid);
                models.push((index, local));
            }
            cache.insert(cache_key.clone(), models);
        }
        let geometry = catalog.geometry(&key).context("water block geometry missing")?;
        let parent = crate::statics::model::block_iso_with(block, &geometry);
        for &(model, local) in &cache[&cache_key] {
            let transform = place::transform(place::compose(parent, local));
            let placement = Iso::from_array(transform.map(f32::from_bits));
            let [x, y, z] = placement.rot.rows;
            if [x.y, y.x, y.y - 1.0, y.z, z.y].iter().any(|value| value.abs() > UPRIGHT_TOLERANCE) {
                continue;
            }
            boxes.push(ModelBox { model, placement, inverse: placement.inverse() });
        }
    }
    ensure!(map.size[0] > 0 && map.size[2] > 0, "invalid water grid extent");
    let size = [map.size[0] as u32, map.size[2] as u32];
    let cells = size[0] as usize * size[1] as usize;
    let default_column = Column { surface: 0.0, bottom: 1.0, material: EMPTY };
    let columns = terrain::columns(map, assets, &catalog, env, default_column)?;
    if boxes.is_empty() && columns.iter().all(|column| column.surface <= column.bottom) {
        return Ok(water);
    }
    let mut volume = Volume {
        grid: map_grid(size),
        default_column,
        columns,
        box_grid: map_grid(size),
        boxes,
        heads: vec![EMPTY; cells],
        entries: Vec::new(),
    };
    for (i, model_box) in volume.boxes.iter().enumerate() {
        let grid = &water.models[model_box.model as usize].grid;
        let extent = [grid.cell[0] * grid.size[0] as f32, grid.cell[1] * grid.size[1] as f32];
        let half = Vec3::new(extent[0] * 0.5, 0.0, extent[1] * 0.5);
        let center = Vec3::new(grid.origin[0] + half.x, 0.0, grid.origin[1] + half.z);
        let bounds = Aabb::new(center, half).transform(&model_box.placement);
        let x0 = ((bounds.center.x - bounds.half.x) / MAP_CELL).floor() as i32;
        let x1 = ((bounds.center.x + bounds.half.x) / MAP_CELL).floor() as i32;
        let z0 = ((bounds.center.z - bounds.half.z) / MAP_CELL).floor() as i32;
        let z1 = ((bounds.center.z + bounds.half.z) / MAP_CELL).floor() as i32;
        for z in z0.max(0)..=z1.min(map.size[2] - 1) {
            for x in x0.max(0)..=x1.min(map.size[0] - 1) {
                let cell = (z as u32 * size[0] + x as u32) as usize;
                let entry = volume.entries.len() as u32;
                volume.entries.push(Entry { value: i as u32, next: volume.heads[cell] });
                volume.heads[cell] = entry;
            }
        }
    }
    volume.grid.default_value = default_column.surface.to_bits();
    water.volumes.push(volume);
    Ok(water)
}
