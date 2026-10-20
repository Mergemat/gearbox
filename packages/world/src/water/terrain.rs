use anyhow::Result;
use gearbox_engine::world_water::Column;

use crate::statics::{MapAssets, model::Catalog};

pub fn columns(
    map: &gearbox_gbx::Map,
    assets: &MapAssets<'_>,
    catalog: &Catalog,
    environment: &str,
    empty: Column,
) -> Result<Vec<Column>> {
    let water = gearbox_packs::water::terrain(assets.packs, environment)?;
    let mut columns = vec![empty; map.size[0] as usize * map.size[2] as usize];
    for block in &map.blocks {
        if block.free.is_some() || !block.is_ground() || !water.blocks.contains(&block.name) {
            continue;
        }
        let [x, y, z] = block.coord;
        if x < 0 || z < 0 || x >= map.size[0] || z >= map.size[2] {
            continue;
        }
        let height = y as f32 * catalog.cell[1] + catalog.cell[2];
        columns[(z * map.size[0] + x) as usize] =
            Column { surface: height + water.surface, bottom: height + water.bottom, material: 0 };
    }
    Ok(columns)
}
