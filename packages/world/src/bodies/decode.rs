use super::{Instance, archives::Archives, model};
use crate::statics::{self, items::ItemSpec, packs::PackSource, place};
use anyhow::{Context, Result, ensure};
use gearbox_engine::{
    Iso,
    dyna::{DynaModel, KinematicModel},
};
use gearbox_packs::{
    assets::{Loc, ModifierRef, Role},
    catalog::EntModel,
};
use std::collections::HashMap;

pub(super) struct Decoder<'a> {
    archives: Archives<'a>,
    animation_ids: HashMap<String, u32>,
    models: HashMap<(String, u32, bool), DynaModel>,
    replacements: HashMap<String, HashMap<String, String>>,
    skinned_materials: HashMap<String, Vec<String>>,
    assets: HashMap<String, gearbox_packs::prefab::DynaObject>,
    pub animations: Vec<KinematicModel>,
    pub bodies: Vec<Instance>,
    pub omitted: Vec<usize>,
}

struct Placement<'a> {
    parent: place::Iso,
    phase: f32,
    modifier: Option<&'a str>,
    chain: Vec<Loc>,
}

impl<'a> Decoder<'a> {
    pub fn new(map: &gearbox_gbx::Map, packs: &'a gearbox_packs::packs::PackSet) -> Self {
        Self {
            archives: Archives::new(map, packs),
            animation_ids: HashMap::new(),
            models: HashMap::new(),
            replacements: HashMap::new(),
            skinned_materials: HashMap::new(),
            assets: HashMap::new(),
            animations: Vec::new(),
            bodies: Vec::new(),
            omitted: Vec::new(),
        }
    }

    fn prefab(
        &mut self,
        source: &mut PackSource<'_>,
        path: &str,
        placement: &Placement<'_>,
        specs: &[ItemSpec],
        depth: usize,
        dynamic_index: &mut u32,
    ) -> Result<()> {
        ensure!(depth < 32, "physics prefab nesting too deep: {path}");
        let prefab = source
            .assets
            .index
            .prefabs
            .get(path)
            .with_context(|| format!("physics prefab absent: {path}"))?
            .clone();
        let has_physics = prefab.ents.iter().any(|entity| {
            matches!(&entity.model, EntModel::Dyna { dyna_shape: Some(id), is_static: false, .. }
                if source.assets.place(*id, Role::DynaShape, &[], &[]).surface.surf.is_some())
        });
        let raw = has_physics.then(|| self.archives.prefab(path)).transpose()?;
        for (index, entity) in prefab.ents.iter().enumerate() {
            let mut chain = placement.chain.clone();
            chain.push(Loc { rotation: entity.rotation, position: entity.position });
            let nested = Placement { chain, ..*placement };
            if let EntModel::Prefab(child) = &entity.model {
                self.prefab(source, child, &nested, specs, depth + 1, dynamic_index)?;
            } else if let EntModel::Dyna {
                dyna_shape: Some(id),
                is_static: false,
                static_shape,
                source: asset_path,
                ..
            } = &entity.model
            {
                let modifiers: Vec<_> = placement
                    .modifier
                    .map(|path| ModifierRef { path, folder: None, keep_track_wall: false })
                    .into_iter()
                    .collect();
                let surface = source.assets.place(*id, Role::DynaShape, &[], &modifiers);
                if surface.surface.surf.is_none() {
                    continue;
                }
                let raw = raw.as_ref().context("dynamic prefab archive absent")?;
                let asset_path = asset_path.clone().unwrap_or_else(|| format!("{path}#ent{index}"));
                self.body(
                    source,
                    &asset_path,
                    index,
                    &prefab,
                    raw,
                    &nested,
                    specs,
                    *static_shape,
                    &surface,
                    dynamic_index,
                )?;
            }
        }
        Ok(())
    }

    fn active(&mut self, path: &str, entity: &gearbox_packs::prefab::Ent) -> Result<bool> {
        let params = entity.params.as_ref().context("dynamic instance params absent")?;
        ensure!(params.0 == 0x2f0b_6000, "dynamic params class {:x}: {path}", params.0);
        let mut reader = gearbox_gbx::reader::Reader::new(&params.1);
        ensure!(reader.u32()? == 2, "dynamic instance version: {path}");
        let period = reader.f32()?;
        reader.u32()?;
        let kinematic = reader.u32()? != 0;
        let period_max = reader.f32()?;
        let phase = reader.f32()?;
        let phase_max = reader.f32()?;
        if !self.assets.contains_key(path) {
            let object = match &entity.model {
                gearbox_packs::prefab::NodeLink::Inline(gearbox_packs::prefab::Model::DynaObject(object)) => {
                    object.clone()
                }
                _ => gearbox_packs::map_model::dyna(&self.archives.read(path)?, path)?,
            };
            self.assets.insert(path.to_owned(), object);
        }
        let asset = &self.assets[path].physics;
        if !kinematic && !asset.dynamize_on_spawn {
            return Ok(false);
        }
        ensure!(kinematic, "non-kinematic dynamic item: {path}");
        ensure!(
            period == 1.0 && period_max < 0.0 && phase < 0.0 && phase_max < 0.0,
            "random or overridden animation timing: {path}"
        );
        ensure!(!asset.physical_animation, "legacy physical location animation: {path}");
        Ok(true)
    }

    fn constraint(
        prefab: &gearbox_packs::catalog::Prefab,
        raw: &gearbox_packs::prefab::Prefab,
        ordinal: u32,
    ) -> Result<(usize, String)> {
        for (index, entity) in prefab.ents.iter().enumerate() {
            let Some(params) = raw.ents.get(index).and_then(|entity| entity.params.as_ref()) else {
                continue;
            };
            if params.0 != 0x2f0c_8000 || params.1.len() != 36 {
                continue;
            }
            let words: Vec<_> =
                params.1.as_chunks::<4>().0.iter().map(|bytes| u32::from_le_bytes(*bytes)).collect();
            if words[1] != u32::MAX || words[2] != ordinal {
                continue;
            }
            ensure!(words[3..].iter().all(|&word| word == 0), "offset kinematic constraint");
            let path = match &entity.model {
                EntModel::Other { path: Some(path), .. } => path.clone(),
                _ => String::new(),
            };
            return Ok((index, path));
        }
        anyhow::bail!("dynamic item has no world kinematic constraint")
    }

    fn body(
        &mut self,
        source: &mut PackSource<'_>,
        path: &str,
        index: usize,
        prefab: &gearbox_packs::catalog::Prefab,
        raw: &gearbox_packs::prefab::Prefab,
        placement: &Placement<'_>,
        specs: &[ItemSpec],
        static_shape: Option<usize>,
        surface: &gearbox_packs::assets::PlacedSurface<'_>,
        dynamic_index: &mut u32,
    ) -> Result<()> {
        let entity = raw.ents.get(index).context("dynamic entity metadata absent")?;
        if !self.active(path, entity)? {
            return Ok(());
        }
        let ordinal = *dynamic_index;
        *dynamic_index += 1;
        let shape = source.shape(surface).map_err(anyhow::Error::msg)?;
        let (constraint, animation_path) = Self::constraint(prefab, raw, ordinal)?;
        let animation_path =
            if animation_path.is_empty() { format!("{path}#constraint{constraint}") } else { animation_path };
        if let Some(modifier) = placement.modifier {
            self.modifier(source, modifier)?;
        }
        let animation = self.animation(&animation_path, &raw.ents[constraint], placement.modifier)?;
        let cloned = if let Some(modifier) = placement.modifier {
            let materials = self.archives.visual_materials(path, &self.assets[path].mesh)?;
            materials.iter().any(|path| self.skinned_materials[modifier].contains(&normalize(path)))
        } else {
            false
        };
        let key = (path.to_owned(), shape, cloned);
        let model = if let Some(model) = self.models.get(&key) {
            model.clone()
        } else {
            let model = model::body(&mut source.shapes, shape, self.assets[path].physics.mass, cloned)?;
            self.models.insert(key, model.clone());
            model
        };
        let raw_base = Iso::from_array(
            place::transform(place::compose(placement.parent, statics::packs::chain_iso(&placement.chain)))
                .map(f32::from_bits),
        );
        if let Some(id) = static_shape {
            let modifiers: Vec<_> = placement
                .modifier
                .map(|path| ModifierRef { path, folder: None, keep_track_wall: false })
                .into_iter()
                .collect();
            let surface = source.assets.place(id, Role::DynaStaticShape, &[], &modifiers);
            if surface.surface.surf.is_some() {
                let shape = source.shape(&surface).map_err(anyhow::Error::msg)?;
                let index = specs
                    .iter()
                    .enumerate()
                    .position(|(index, spec)| {
                        !self.omitted.contains(&index)
                            && spec.shape == shape
                            && spec.transform.to_array().map(f32::to_bits)
                                == raw_base.to_array().map(f32::to_bits)
                    })
                    .with_context(|| format!("dynamic static fallback absent: {path}"))?;
                self.omitted.push(index);
            }
        }
        self.bodies.push(Instance {
            model,
            shape,
            animation,
            phase: placement.phase,
            placement: Iso::IDENTITY.inv_mul(&raw_base),
        });
        Ok(())
    }

    fn modifier(&mut self, source: &PackSource<'_>, modifier: &str) -> Result<()> {
        if self.replacements.contains_key(modifier) {
            return Ok(());
        }
        let info = source.assets.index.modifiers.get(modifier).context("dynamic modifier absent")?;
        let skin = info.skin.as_ref().context("inline animation modifier skin")?;
        let files = gearbox_packs::material::skin_files(&self.archives.read(skin)?)?;
        self.skinned_materials.insert(
            modifier.to_owned(),
            files
                .iter()
                .filter(|(class, _, _)| *class == 0x0907_9000)
                .map(|(_, _, path)| normalize(path))
                .collect(),
        );
        self.replacements.insert(
            modifier.to_owned(),
            files
                .into_iter()
                .filter(|(class, _, _)| *class == 0x2f0c_a000)
                .map(|(_, name, path)| {
                    (normalize(&path), format!("{}{name}.KinematicConstraint.Gbx", info.folder))
                })
                .collect(),
        );
        Ok(())
    }

    fn animation(
        &mut self,
        path: &str,
        constraint: &gearbox_packs::prefab::Ent,
        modifier: Option<&str>,
    ) -> Result<u32> {
        let path = modifier
            .and_then(|modifier| self.replacements[modifier].get(&normalize(path)))
            .map_or(path, String::as_str)
            .to_owned();
        if let Some(&index) = self.animation_ids.get(&path) {
            return Ok(index);
        }
        let archive = match &constraint.model {
            gearbox_packs::prefab::NodeLink::Inline(gearbox_packs::prefab::Model::Kinematic(animation)) => {
                animation.clone()
            }
            _ => gearbox_packs::map_model::animation(&self.archives.read(&path)?)
                .with_context(|| format!("kinematic {path}"))?,
        };
        let index = self.animations.len() as u32;
        self.animations.push(model::animation(&archive)?);
        self.animation_ids.insert(path, index);
        Ok(index)
    }

    pub fn traverse(
        &mut self,
        map: &gearbox_gbx::Map,
        source: &mut PackSource<'_>,
        specs: &[ItemSpec],
    ) -> Result<()> {
        self.blocks(map, source, specs)?;
        self.items(map, source, specs)?;
        self.decoration(map, source, specs)
    }

    fn block_prefab(source: &PackSource<'_>, key: &statics::BlockKey) -> Result<Option<String>> {
        let asset = source.assets.block_in(&key.name, &source.env).context("dynamic block asset absent")?;
        let variant = asset
            .variants
            .iter()
            .find(|variant| {
                variant.ground == (key.flags & 0x1000 != 0)
                    && variant.index == ((key.flags >> 21) & 0x7f) as usize
            })
            .context("dynamic block variant absent")?;
        Ok(variant
            .mobils
            .get(key.variant as usize)
            .and_then(|group| group.get(key.sub_variant as usize).or_else(|| group.first()))
            .and_then(|mobil| mobil.prefab.clone()))
    }

    fn blocks(
        &mut self,
        map: &gearbox_gbx::Map,
        source: &mut PackSource<'_>,
        specs: &[ItemSpec],
    ) -> Result<()> {
        for block in map.blocks.iter().chain(&map.baked_blocks) {
            let key = statics::BlockKey::of(block);
            let Some(prefab) = Self::block_prefab(source, &key)? else { continue };
            let geometry = source.catalog.geometry(&key).context("dynamic block placement absent")?;
            let placement = Placement {
                parent: statics::model::block_iso_with(block, &geometry),
                phase: 0.0,
                modifier: None,
                chain: Vec::new(),
            };
            self.prefab(source, &prefab, &placement, specs, 0, &mut 0)?;
        }
        Ok(())
    }

    fn items(
        &mut self,
        map: &gearbox_gbx::Map,
        source: &mut PackSource<'_>,
        specs: &[ItemSpec],
    ) -> Result<()> {
        for item in &map.items {
            let name = item.model.as_deref().context("dynamic item without model")?;
            let path =
                source.assets.find_anchored_item(item, &source.env).context("dynamic item asset absent")?;
            let models = source.assets.index.items[path].models((item.flags >> 8) as usize).to_vec();
            let modifier = source.assets.index.items[path].material_modifier.clone();
            for model in models {
                if let EntModel::Dyna { dyna_shape: Some(id), is_static: false, .. } = &model {
                    ensure!(
                        source.assets.place(*id, Role::DynaShape, &[], &[]).surface.surf.is_none(),
                        "direct dynamic item animation: {name}"
                    );
                }
                if let EntModel::Prefab(prefab) = model {
                    let placement = Placement {
                        parent: statics::anchored::item_iso(item),
                        phase: item.anim_phase as f32 * 0.125,
                        modifier: modifier.as_deref(),
                        chain: Vec::new(),
                    };
                    self.prefab(source, &prefab, &placement, specs, 0, &mut 0)?;
                }
            }
        }
        Ok(())
    }

    fn decoration(
        &mut self,
        map: &gearbox_gbx::Map,
        source: &mut PackSource<'_>,
        specs: &[ItemSpec],
    ) -> Result<()> {
        let decoration = statics::packs::decoration(source.assets, map).cloned();
        let Some(decoration) = decoration.filter(|deco| deco.map.is_some()) else { return Ok(()) };
        let size = decoration.map_size.context("decoration without map dimensions")?;
        let location = statics::decoration::deco_location(size, decoration.size);
        for (name, direction, coord, flags) in decoration.blocks {
            let key = statics::BlockKey {
                name,
                flags,
                variant: (flags & 0x3f) as u8,
                sub_variant: ((flags >> 6) & 0x3f) as u8,
                has_waypoint: false,
            };
            let Some(prefab) = Self::block_prefab(source, &key)? else { continue };
            let geometry = source.catalog.geometry(&key).context("decoration dynamic geometry absent")?;
            let placement = Placement {
                parent: place::grid_iso_in(
                    statics::decoration::DECO_CELL,
                    Some(location),
                    coord,
                    direction as u32,
                    geometry.size,
                    geometry.offset,
                ),
                phase: 0.0,
                modifier: None,
                chain: Vec::new(),
            };
            self.prefab(source, &prefab, &placement, specs, 0, &mut 0)?;
        }
        Ok(())
    }
}

fn normalize(path: &str) -> String {
    path.replace('/', "\\").to_lowercase()
}
