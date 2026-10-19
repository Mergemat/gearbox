use crate::statics::items::ItemSpec;
use crate::statics::model::{BlockKey, ModelSource};
use crate::statics::packs::PackSource;
use crate::statics::place::{self, Iso};

pub const DECO_CELL: [f32; 3] = [256.0, 64.0, -64.0];

pub fn deco_location(size: [i32; 3], deco: [i32; 3]) -> ([f32; 9], [f32; 3]) {
    let x = (size[0] as f32 * 32.0 - deco[0] as f32 * 256.0) * 0.5;
    let z = (size[2] as f32 * 32.0 - deco[2] as f32 * 256.0) * 0.5;
    ([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [x, -184.0, z])
}

fn block(block_entry: &(String, u8, [i32; 3], u32)) -> gearbox_gbx::map::Block {
    let (name, dir, coord, flags) = block_entry.clone();
    gearbox_gbx::map::Block {
        name, dir, coord, flags, variant: (flags & 0x3f) as u8, sub_variant: ((flags >> 6) & 0x3f) as u8,
        waypoint: None, free: None, skin: None,
    }
}

pub fn items(map: &gearbox_gbx::Map, src: &mut PackSource<'_>) -> Result<Vec<Vec<ItemSpec>>, String> {
    let name = map.decoration.as_deref().ok_or("map without a decoration")?;
    let env = src.env.clone();
    let decoration = src.assets.decoration(name, &env)
        .ok_or_else(|| format!("unknown decoration {name}"))?.clone();
    if decoration.map.is_none() {
        return Ok(Vec::new());
    }
    if decoration.collection.as_deref() != Some("25") || env != "Stadium" {
        return Err(format!(
            "decoration {name} (collection {:?}) on {env} not placed yet", decoration.collection,
        ));
    }
    let location = deco_location(map.size, decoration.size);
    let mut out = Vec::new();
    for block_entry in &decoration.blocks {
        let block_entry = block(block_entry);
        let key = BlockKey::of(&block_entry);
        let model = src.block_model(&key)
            .ok_or_else(|| format!(
                "decoration block {}: {:?}", block_entry.name, src.problems.get(&block_entry.name),
            ))?;
        let geometry = src.block_geometry(&key).ok_or("decoration block without metadata")?;
        let iso: Iso = place::grid_iso_in(
            DECO_CELL, Some(location), block_entry.coord, block_entry.dir as u32,
            geometry.size, geometry.offset,
        );
        out.push(
            model
                .parts
                .iter()
                .map(|p| ItemSpec {
                    entity: p.entity,
                    shape: p.shape,
                    transform: gearbox_engine::Iso::from_array(
                        place::transform(place::compose(iso, p.local)).map(f32::from_bits),
                    ),
                    layer: p.layer,
                    flags: 0,
                })
                .collect(),
        );
    }
    Ok(out)
}
