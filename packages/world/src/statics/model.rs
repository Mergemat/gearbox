pub mod catalog;
pub use catalog::{Catalog, Geometry};

use crate::statics::place::{self, Iso};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct BlockKey {
    pub name: String,
    pub variant: u8,
    pub sub_variant: u8,
    pub flags: u32,
}

impl BlockKey {
    pub fn of(block: &gearbox_gbx::map::Block) -> Self {
        Self {
            name: block.name.clone(),
            variant: block.variant,
            sub_variant: block.sub_variant,
            flags: block.flags & 0x3fe0_1000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Part {
    pub shape: u32,
    pub local: Iso,
    pub block_location: bool,
    pub layer: u8,
    pub entity: gearbox_engine::static_world::Entity,
}

#[derive(Clone, Debug)]
pub struct BlockModel {
    pub geometry: Geometry,
    pub parts: Vec<Part>,
}

pub trait ModelSource {
    fn block_geometry(&self, key: &BlockKey) -> Option<Geometry>;
    fn block_model(&mut self, key: &BlockKey) -> Option<BlockModel>;
    fn block_model_at(
        &mut self, map: &gearbox_gbx::Map, grid: &crate::statics::instancing::Grid, block: &gearbox_gbx::map::Block,
    ) -> Option<BlockModel> {
        let _ = (map, grid);
        self.block_model(&BlockKey::of(block))
    }
}

pub fn block_location_with(block: &gearbox_gbx::map::Block, geometry: &Geometry) -> ([f32; 9], [f32; 3]) {
    if let Some((pos, ang)) = block.free {
        return place::free_location_variant(pos, ang, geometry.offset);
    }
    place::grid_location_in(
        geometry.cell, None, block.coord, block.dir as u32, geometry.size, geometry.offset,
    )
}

pub fn block_iso_with(block: &gearbox_gbx::map::Block, geometry: &Geometry) -> Iso {
    if let Some((pos, ang)) = block.free {
        return place::free_iso_variant(pos, ang, geometry.offset);
    }
    place::grid_iso_in(geometry.cell, None, block.coord, block.dir as u32, geometry.size, geometry.offset)
}
