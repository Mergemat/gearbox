use crate::statics::model::{BlockKey, ModelSource};
pub mod clips;
use anyhow::{Result, bail};

pub fn unit_coord(coord: [i32; 3], unit: [i32; 3], dir: u8, size: [u32; 3]) -> Result<[i32; 3]> {
    let [x, y, z] = unit;
    let [sx, _, sz] = size.map(|n| n as i32);
    let [x, z] = match dir {
        0 => [x, z],
        1 => [sz - 1 - z, x],
        2 => [sx - 1 - x, sz - 1 - z],
        3 => [z, sx - 1 - x],
        _ => bail!("non-cardinal block direction {dir}"),
    };
    Ok([coord[0] + x, coord[1] + y, coord[2] + z])
}

pub fn hidden_baked(map: &gearbox_gbx::Map, src: &dyn ModelSource) -> Result<Vec<bool>> {
    let mut ground = std::collections::HashSet::new();
    for block in &map.blocks {
        if block.free.is_some() || !block.is_ground() {
            continue;
        }
        let Some(geometry) = src.block_geometry(&BlockKey::of(block)) else { continue };
        for unit in geometry.units {
            ground.insert(unit_coord(block.coord, unit, block.dir, geometry.size)?);
        }
    }
    Ok(map
        .baked_blocks
        .iter()
        .map(|block| {
            block.free.is_none()
                && src.block_geometry(&BlockKey::of(block)).is_some_and(|geometry| geometry.terrain())
                && ground.contains(&block.coord)
        })
        .collect())
}

pub fn neighbour(c: [i32; 3], dir: u32) -> [i32; 3] {
    let [x, y, z] = c;
    match dir {
        0 => [x, y, z + 1],
        1 => [x - 1, y, z],
        2 => [x, y, z - 1],
        3 => [x + 1, y, z],
        4 => [x, y + 1, z],
        5 => [x, y - 1, z],
        _ => c,
    }
}

pub struct Grid {
    cells: std::collections::HashMap<[i32; 3], Vec<(usize, usize)>>,
    free_parents: std::collections::HashMap<usize, (usize, usize)>,
    geometry: std::collections::HashMap<(usize, usize), crate::statics::model::Geometry>,
}

impl Grid {
    pub fn new(map: &gearbox_gbx::Map, src: &dyn ModelSource) -> Result<Self> {
        let mut cells: std::collections::HashMap<[i32; 3], Vec<(usize, usize)>> =
            std::collections::HashMap::new();

        let mut block_geometry = std::collections::HashMap::new();
        for (list, blocks) in [&map.blocks, &map.baked_blocks].into_iter().enumerate() {
            for (i, block) in blocks.iter().enumerate() {
                let Some(geometry) = src.block_geometry(&BlockKey::of(block)) else { continue };
                if geometry.kind == crate::statics::model::catalog::GeometrySource::Clip {
                    continue;
                }
                if block.free.is_some() {
                    continue;
                }
                for &unit in &geometry.units {
                    let c = unit_coord(block.coord, unit, block.dir, geometry.size)?;
                    cells.entry(c).or_default().push((list, i));
                }
                block_geometry.insert((list, i), geometry);
            }
        }
        Ok(Self { cells, free_parents: clips::free_parents(map, src)?, geometry: block_geometry })
    }

    fn block(map: &gearbox_gbx::Map, (list, i): (usize, usize)) -> &gearbox_gbx::map::Block {
        if list == 0 { &map.blocks[i] } else { &map.baked_blocks[i] }
    }

    fn first_upright<'m>(
        &self,
        map: &'m gearbox_gbx::Map,
        cell: &[(usize, usize)],
    ) -> Option<&'m gearbox_gbx::map::Block> {
        cell.iter().map(|&k| Grid::block(map, k)).find(|block| block.flags & 0x1000_0000 == 0)
    }
}

pub fn clip_parent<'m>(
    map: &'m gearbox_gbx::Map,
    grid: &Grid,
    block: &gearbox_gbx::map::Block,
    geometry: &crate::statics::model::Geometry,
) -> Result<Option<&'m gearbox_gbx::map::Block>> {
    if geometry.kind != crate::statics::model::catalog::GeometrySource::Clip || geometry.clip_type == 0 {
        return Ok(None);
    }
    if block.free.is_some() {
        return Ok(map
            .baked_blocks
            .iter()
            .position(|candidate| std::ptr::eq(candidate, block))
            .and_then(|index| grid.free_parents.get(&index))
            .map(|&k| Grid::block(map, k)));
    }
    let dir = match geometry.clip_type {
        2 => 5,
        3 => 4,
        _ => block.dir as u32,
    };
    let c = neighbour(block.coord, dir);
    let cell = grid.cells.get(&c).map(|v| v.as_slice()).unwrap_or(&[]);
    if block.flags & 0x1000_0000 != 0 {
        let face = match geometry.clip_type {
            2 => 4,
            3 => 5,
            _ => (block.dir as u32 + 2) & 3,
        };
        for &k in cell {
            let parent = Grid::block(map, k);
            if parent.flags & 0x1000_0000 == 0 {
                continue;
            }
            let Some(parent_geometry) = grid.geometry.get(&k) else { continue };
            let local_face = if face < 4 { (face + 4 - parent.dir as u32) & 3 } else { face };
            for (unit, clips) in parent_geometry.units.iter().zip(parent_geometry.clip_faces.iter()) {
                if unit_coord(parent.coord, *unit, parent.dir, parent_geometry.size)? != c {
                    continue;
                }
                for (j, name) in clips.faces[local_face as usize].iter().enumerate() {
                    let Some(name) = name else { continue };
                    if !gearbox_packs::assets::block_name(name).eq_ignore_ascii_case(&block.name) {
                        continue;
                    }
                    let d = if local_face < 4 {
                        (local_face + 2) & 3
                    } else {
                        (clips.directions[(local_face - 4) as usize] as u32 >> (j * 2)) & 3
                    };
                    if (d + parent.dir as u32) & 3 == block.dir as u32 {
                        return Ok(Some(parent));
                    }
                }
            }
        }
        return Ok(None);
    }
    Ok(grid.first_upright(map, cell))
}
