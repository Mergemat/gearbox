use std::collections::HashMap;

use anyhow::Result;
use gearbox_engine::world_water::Column;

use gearbox_packs::packs::PackSet;

pub fn terrain_columns(
    map: &gearbox_gbx::Map,
    packs: &PackSet,
    cell: [f32; 3],
    environment: &str,
    ground_water: &HashMap<[i32; 3], bool>,
) -> Result<Vec<Column>> {
    let water = gearbox_packs::water::terrain(packs, environment)?;
    let empty = Column { surface: 0.0, bottom: 1.0, material: u32::MAX };
    let mut columns = vec![empty; map.size[0] as usize * map.size[2] as usize];
    for block in map.blocks.iter().chain(&map.baked_blocks) {
        if block.free.is_some()
            || block.is_ghost()
            || !block.is_ground()
            || !water.blocks.contains(&block.name)
        {
            continue;
        }
        let [x, y, z] = block.coord;
        if x < 0 || z < 0 || x >= map.size[0] || z >= map.size[2] {
            continue;
        }
        if ground_water
            .get(&[x, y.max(0), z])
            .or_else(|| ground_water.get(&[x, 0, z]))
            .copied()
            .unwrap_or(false)
        {
            continue;
        }
        let height = y as f32 * cell[1] + cell[2];
        columns[(z * map.size[0] + x) as usize] =
            Column { surface: height + water.surface, bottom: height + water.bottom, material: 0 };
    }
    Ok(columns)
}
