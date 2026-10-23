use super::BlockKey;
use crate::statics::place::VariantOffset;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometrySource {
    Block,
    Clip,
}

#[derive(Clone, Debug)]
pub struct Geometry {
    pub size: [u32; 3],
    pub units: Vec<[i32; 3]>,
    pub clip_faces: std::rc::Rc<Vec<gearbox_packs::blockinfo::UnitClips>>,
    pub ground: std::rc::Rc<Vec<([i32; 2], gearbox_packs::zones::Ground)>>,
    pub ground_offset: i32,
    pub class: u32,
    pub kind: GeometrySource,
    pub clip_type: u32,
    pub offset: VariantOffset,
    pub cell: [f32; 3],
}

impl Geometry {
    pub fn terrain(&self) -> bool {
        matches!(self.class, 0x0304_f000 | 0x0305_0000 | 0x0314_c000)
    }
}

#[derive(Clone)]
struct Variant {
    units: Vec<[i32; 3]>,
    clip_faces: std::rc::Rc<Vec<gearbox_packs::blockinfo::UnitClips>>,
    ground: std::rc::Rc<Vec<([i32; 2], gearbox_packs::zones::Ground)>>,
    ground_offset: i32,
    size: [u32; 3],
    mobils: Vec<Vec<VariantOffset>>,
}

pub struct Catalog {
    variants: HashMap<(String, bool, u8), (u32, GeometrySource, u32, Variant)>,
    pub cell: [f32; 3],
}

impl Default for Catalog {
    fn default() -> Self {
        Self { variants: HashMap::new(), cell: crate::statics::place::CELL }
    }
}

fn size_of(units: &[[i32; 3]]) -> [u32; 3] {
    std::array::from_fn(|k| units.iter().map(|u| (u[k] as u32).wrapping_add(1)).fold(1, u32::max))
}

impl Catalog {
    pub fn from_assets(assets: &gearbox_packs::assets::Assets, env: &str) -> Self {
        let mut variants = HashMap::new();
        let names: std::collections::BTreeSet<&str> =
            assets.block_names().chain(assets.index.custom_blocks.keys().map(String::as_str)).collect();
        for name in names {
            let Some(block) = assets.block_in(name, env).block() else { continue };
            let kind = if gearbox_packs::blockinfo::is_clip(block.class_id) {
                GeometrySource::Clip
            } else {
                GeometrySource::Block
            };
            for variant in &block.variants {
                let mobils = variant
                    .mobils
                    .iter()
                    .map(|geometry| {
                        geometry
                            .iter()
                            .map(|mobil| VariantOffset {
                                pos: mobil.geom_translation.unwrap_or_default(),
                                deg: mobil.geom_rotation.unwrap_or_default(),
                            })
                            .collect()
                    })
                    .collect();
                let metadata = Variant {
                    clip_faces: Default::default(),
                    ground: Default::default(),
                    ground_offset: 0,
                    units: variant.units.clone(),
                    size: size_of(&variant.units),
                    mobils,
                };
                variants.entry((name.to_ascii_lowercase(), variant.ground, variant.index as u8)).or_insert((
                    block.class_id,
                    kind,
                    block.clip_type,
                    metadata,
                ));
            }
        }
        Self { variants, cell: crate::statics::place::CELL }
    }

    pub fn load_placement_metadata(
        &mut self,
        assets: &gearbox_packs::assets::Assets,
        map: &gearbox_gbx::Map,
        env: &str,
        packs: &gearbox_packs::packs::PackSet,
    ) -> anyhow::Result<()> {
        let mut seen = std::collections::HashSet::new();
        let mut zones = None;
        for block in map.blocks.iter().chain(&map.baked_blocks) {
            if block.free.is_none() && !block.is_ghost() {
                continue;
            }
            let Some(info) = assets.block_in(&block.name, env).block() else { continue };
            if gearbox_packs::blockinfo::is_clip(info.class_id)
                || !seen.insert(block.name.to_ascii_lowercase())
            {
                continue;
            }
            let bytes = packs.read_path(&info.path)?;
            let layout = gearbox_packs::nodes::layout(&bytes)?;
            let parsed = gearbox_packs::blockinfo::read_block_info(&bytes, &layout, &info.path)?;
            for variant in parsed.variants {
                let ground = variant.kind == gearbox_packs::blockinfo::VariantKind::Ground;
                let key = (block.name.to_ascii_lowercase(), ground, variant.index as u8);
                if let Some((_, _, _, metadata)) = self.variants.get_mut(&key) {
                    let mut faces = variant.clip_faces;
                    for name in faces.iter_mut().flat_map(|u| &mut u.faces).flatten().flatten() {
                        *name = assets
                            .index
                            .blocks
                            .get(name)
                            .and_then(|block| block.name.clone())
                            .unwrap_or_else(|| gearbox_packs::assets::block_name(name).to_string());
                    }
                    metadata.clip_faces = std::rc::Rc::new(faces);
                    let (columns, offset) = match variant.ground_archive {
                        Some(at) => gearbox_packs::blockinfo::auto_terrain(&bytes, &layout, &info.path, at)?,
                        None => (Vec::new(), 0),
                    };
                    metadata.ground_offset = offset;
                    metadata.ground = if columns.is_empty() {
                        Default::default()
                    } else {
                        let zones = match zones {
                            Some(ref zones) => zones,
                            None => zones.insert(gearbox_packs::zones::Zones::load(packs)?),
                        };
                        let env = info.path.split('\\').next().unwrap_or(env);
                        std::rc::Rc::new(
                            columns
                                .iter()
                                .filter(|column| column.target.is_some())
                                .map(|column| {
                                    let ground = gearbox_packs::zones::Ground::of(zones, env, column, offset);
                                    ([column.coord[0], column.coord[2]], ground)
                                })
                                .collect(),
                        )
                    };
                }
            }
        }
        Ok(())
    }

    pub fn geometry(&self, key: &BlockKey) -> Option<Geometry> {
        let variant_key =
            (key.name.to_ascii_lowercase(), key.flags & 0x1000 != 0, ((key.flags >> 21) & 0x7f) as u8);
        let (class, kind, clip_type, variant) = self.variants.get(&variant_key)?;
        let offset = variant
            .mobils
            .get(key.variant as usize)
            .and_then(|m| m.get(key.sub_variant as usize).or_else(|| m.first()))
            .copied()
            .unwrap_or_default();
        Some(Geometry {
            size: variant.size,
            units: variant.units.clone(),
            clip_faces: variant.clip_faces.clone(),
            ground: variant.ground.clone(),
            ground_offset: variant.ground_offset,
            class: gearbox_gbx::classid::remap_class_id(*class),
            kind: *kind,
            clip_type: *clip_type,
            cell: self.cell,
            offset: if *kind == GeometrySource::Clip && *clip_type == 0 {
                VariantOffset::default()
            } else {
                offset
            },
        })
    }
}
