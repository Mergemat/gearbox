use super::packs::LAYER_TRIGGER;
use crate::statics::items::ItemSpec;
use crate::statics::model::{self, ModelSource, PartLocation};
use crate::statics::place;
use anyhow::Result;
use gearbox_engine::collision::ItemEvents;

#[derive(Clone, Debug)]
pub enum Placed {
    Items(Vec<ItemSpec>),
    UnknownModel,
    Hidden,
}

pub fn blocks(map: &gearbox_gbx::Map, src: &mut dyn ModelSource) -> Result<Vec<Placed>> {
    let mut out = Vec::new();
    let hidden = crate::statics::instancing::hidden_baked(map, src)?;
    let grid = crate::statics::instancing::Grid::new(map, src)?;
    for (list, blocks) in [&map.blocks, &map.baked_blocks].into_iter().enumerate() {
        for (i, block) in blocks.iter().enumerate() {
            if list == 1 && hidden[i] {
                out.push(Placed::Hidden);
                continue;
            }
            let Some(model) = src.block_model_at(map, &grid, block) else {
                out.push(Placed::UnknownModel);
                continue;
            };
            let iso = model::block_iso_with(block, &model.geometry);
            let items = model
                .parts
                .iter()
                .map(|p| {
                    let entity = p.entity;
                    let transform = match p.location {
                        PartLocation::Block => {
                            place::transform_matrix(model::block_location_with(block, &model.geometry))
                        }
                        PartLocation::Local(local) => place::transform(place::compose(iso, local)),
                        PartLocation::World(pose) => place::transform(pose),
                    };
                    ItemSpec {
                        entity,
                        shape: p.shape,
                        transform: gearbox_engine::Iso::from_array(transform.map(f32::from_bits)),
                        layer: p.layer,
                        flags: if p.layer == LAYER_TRIGGER { ItemEvents::CONTACT_TIME.bits() } else { 0 },
                    }
                })
                .collect();
            out.push(Placed::Items(items));
        }
    }
    Ok(out)
}

pub const FLAG_MATERIAL14_ALIAS: u32 = 1 << 1;

pub fn runtime_flags(map: &gearbox_gbx::Map) -> u32 {
    let mut flags = map.flags.unwrap_or(5);
    if map.build_version.as_deref().is_some_and(|v| !v.is_empty() && v < "date=2023-10-15") {
        flags |= FLAG_MATERIAL14_ALIAS;
    }
    flags
}
