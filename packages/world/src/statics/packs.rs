use std::collections::HashMap;

use gearbox_engine::static_world::{Entity, LAYER_GATE};
use gearbox_packs::assets::{Assets, BlockLookup, ModifierRef, PlacedSurface, Role};
use gearbox_packs::catalog::{Decoration, custom_block_name};
use gearbox_packs::surface::Surf;

use crate::statics::model::{BlockKey, BlockModel, Catalog, Geometry, ModelSource, Part, PartLocation};
use crate::statics::place::Iso;
use crate::statics::veget::Orientation;

pub use gearbox_engine::static_world::{LAYER_STATIC, LAYER_TRIGGER};

pub fn environment(map: &gearbox_gbx::Map) -> Result<&'static str, String> {
    collection_environment(map.collection.as_deref().or(map.decoration_collection.as_deref()))
}

pub fn decoration_environment(map: &gearbox_gbx::Map) -> Result<&'static str, String> {
    collection_environment(map.decoration_collection.as_deref().or(map.collection.as_deref()))
}

pub fn decoration<'a>(assets: &'a Assets, map: &gearbox_gbx::Map) -> Option<&'a Decoration> {
    assets.decoration(map.decoration.as_deref()?, decoration_environment(map).ok()?)
}

fn collection_environment(c: Option<&str>) -> Result<&'static str, String> {
    gearbox_packs::packs::environment(c)
        .ok_or_else(|| format!("collection {c:?} not mapped to an environment yet"))
}

pub fn map_cell(assets: &Assets, map: &gearbox_gbx::Map) -> [f32; 3] {
    let [x, h, y0] = crate::statics::place::CELL;
    let level0 = decoration(assets, map).and_then(|d| d.level0);
    [x, h, level0.map_or(y0, |n| -(n as f32) * h)]
}

pub fn add_map_items(
    assets: &mut Assets,
    map: &gearbox_gbx::Map,
    packs: &gearbox_packs::packs::PackSet,
) -> Vec<String> {
    let mut errors = add_embedded_items(assets, map, packs);
    if let Err(error) = add_block_triggers(assets, map, packs) {
        errors.push(format!("block triggers: {error:#}"));
    }
    errors
}

fn add_embedded_items(
    assets: &mut Assets,
    map: &gearbox_gbx::Map,
    packs: &gearbox_packs::packs::PackSet,
) -> Vec<String> {
    let items: Vec<(String, String)> =
        map.embedded.files.iter().filter_map(|f| Some((f.model_path()?, f.path.clone()))).collect();
    if items.is_empty() {
        return Vec::new();
    }
    let files = gearbox_packs::embedded::Files::new(
        map.embedded.files.iter().map(|file| (file.path.as_str(), file.data.as_slice())),
        packs,
    );
    let mut catalog =
        gearbox_packs::catalog::crawl_embedded(&files, &items, &assets.index, assets.surface_count());
    for file in &map.embedded.files {
        let Some(name) = file.item_model.as_ref().and_then(|model| model.ident[0].as_ref()) else { continue };
        let Some(path) = file.model_path() else { continue };
        if let Some(item) = catalog.index.items.get(&path).cloned() {
            catalog.index.items.entry(name.clone()).or_insert(item);
        }
        if let Some(block) = catalog.index.custom_blocks.get(&custom_block_name(&path)).cloned() {
            catalog.index.custom_blocks.entry(custom_block_name(name)).or_insert(block);
        }
    }
    custom_block_gates(&mut catalog, map);
    let errors = catalog.index.errors.clone();
    assets.add_catalog(catalog);
    errors
}

fn custom_block_gates(catalog: &mut gearbox_packs::catalog::Catalog, map: &gearbox_gbx::Map) {
    use gearbox_packs::catalog::{Ent, EntModel, Prefab};
    let mut prefabs = Vec::new();
    for (name, block) in &mut catalog.index.custom_blocks {
        let Some(block) = block else { continue };
        let Some(map_block) = map.blocks.iter().find(|block| block.name.eq_ignore_ascii_case(name)) else {
            continue;
        };
        if map_block.waypoint.is_some() || block.waypoint.is_some() {
            continue;
        }
        for variant in &mut block.variants {
            let Some(shape) = variant.waypoint_trigger_surface.take() else { continue };
            variant.waypoint_trigger = None;
            for (group, mobils) in variant.mobils.iter_mut().enumerate() {
                for (mobil, mobil_model) in mobils.iter_mut().enumerate() {
                    let Some(original) = mobil_model.prefab.take() else { continue };
                    let key = format!("{name}#gate-{}-{}-{group}-{mobil}", variant.ground, variant.index);
                    let ent = |model| Ent {
                        model,
                        rotation: [0.0, 0.0, 0.0, 1.0],
                        position: [0.0; 3],
                        params_class: None,
                        name: String::new(),
                    };
                    let prefab = Prefab {
                        ents: vec![
                            ent(EntModel::Prefab(original)),
                            ent(EntModel::BlockTrigger { waypoint: None, shape }),
                        ],
                    };
                    prefabs.push((key.clone(), prefab));
                    mobil_model.prefab = Some(key);
                }
            }
        }
    }
    catalog.index.prefabs.extend(prefabs);
}

fn add_block_triggers(
    assets: &mut Assets,
    map: &gearbox_gbx::Map,
    packs: &gearbox_packs::packs::PackSet,
) -> anyhow::Result<()> {
    let env = environment(map).map_err(anyhow::Error::msg)?;
    let mut seen = std::collections::HashSet::new();
    let mut blocks = Vec::new();
    for block in map.blocks.iter().chain(&map.baked_blocks) {
        if assets.index.custom_blocks.contains_key(&block.name.to_ascii_lowercase()) {
            continue;
        }
        let Some(info) = assets.block_in(&block.name, env).block() else { continue };
        if !seen.insert(info.path.clone()) {
            continue;
        }
        let waypoint = crate::gameplay::block_waypoint(block, assets, env)
            .map(|tag| match tag.as_str() {
                "Spawn" => Ok(0),
                "Goal" => Ok(1),
                "Checkpoint" => Ok(2),
                "StartFinish" => Ok(4),
                _ => anyhow::bail!("legacy block waypoint {tag} on {}", block.name),
            })
            .transpose()?;
        blocks.push((info, waypoint));
    }
    let catalog = gearbox_packs::block_triggers::load(packs, &blocks, assets.surface_count())?;
    assets.add_catalog(catalog);
    Ok(())
}

#[derive(Clone)]
struct CachedBlock<'a> {
    model: BlockModel,
    trees: Vec<PlacedSurface<'a>>,
}

pub struct PackSource<'a> {
    pub assets: &'a Assets,
    pub catalog: Catalog,
    pub env: String,
    pub shapes: Vec<gearbox_engine::static_world::Shape>,
    pub gameplay: HashMap<u32, crate::gameplay::gates::Properties>,
    shape_cache: HashMap<(usize, Vec<u8>, u32, bool, bool), u32>,
    tree_scales: HashMap<(usize, i32), f32>,
    models: HashMap<(BlockKey, Mods), Option<CachedBlock<'a>>>,
    pub problems: HashMap<String, String>,
    mods_cache: std::cell::RefCell<HashMap<ModsKey, Mods>>,
}

fn iso_of(l: &gearbox_packs::assets::Loc) -> Iso {
    let [x, y, z, w] = l.rotation;
    Iso { q: [w, x, y, z], p: l.position }
}

fn gameplay_effect(placed: &PlacedSurface<'_>) -> Option<u8> {
    placed
        .materials
        .first()
        .and_then(|material| material.as_ref().map(|(_, ids)| ids.gameplay))
        .or_else(|| placed.surface.material_ids.first().map(|id| (id >> 8) as u8))
}

fn game_triangle(triangle: &gearbox_packs::surface::MeshTri, placed: &PlacedSurface<'_>) -> [u8; 16] {
    let mut bytes = [0u8; 16];
    for k in 0..3 {
        bytes[4 * k..4 * k + 4].copy_from_slice(&triangle.v[k].to_le_bytes());
    }
    bytes[12] = triangle.flags[0];
    bytes[13] = triangle.flags[1];
    if !matches!(placed.role, Role::BlockSpecialTrigger | Role::BlockTrigger(_))
        && let Some(Some((_, ids))) = placed.materials.get(triangle.material as usize)
    {
        bytes[12] = ids.surface;
        bytes[13] = ids.gameplay;
    }
    bytes[14..].copy_from_slice(&triangle.material.to_le_bytes());
    bytes
}

pub fn chain_iso(chain: &[gearbox_packs::assets::Loc]) -> Iso {
    let mut locations: Vec<Iso> = chain.iter().map(iso_of).collect();
    let Some(mut acc) = locations.pop() else { return crate::statics::place::IDENTITY };
    while let Some(parent) = locations.pop() {
        acc = crate::statics::place::compose(parent, acc);
    }
    acc
}

impl<'a> PackSource<'a> {
    pub fn new(assets: &'a Assets, env: &str) -> Self {
        Self {
            assets,
            catalog: Catalog::from_assets(assets, env),
            env: env.to_string(),
            shapes: Vec::new(),
            gameplay: HashMap::new(),
            shape_cache: HashMap::new(),
            tree_scales: HashMap::new(),
            models: HashMap::new(),
            problems: HashMap::new(),
            mods_cache: Default::default(),
        }
    }

    pub fn for_map(
        assets: &'a Assets,
        map: &gearbox_gbx::Map,
        packs: &gearbox_packs::packs::PackSet,
    ) -> Result<Self, String> {
        let mut source = Self::new(assets, environment(map)?);
        source.catalog.cell = map_cell(assets, map);
        source.catalog.load_placement_metadata(assets, map, &source.env, packs).map_err(|e| e.to_string())?;
        Ok(source)
    }

    pub fn shape(&mut self, placed: &PlacedSurface<'_>) -> Result<u32, String> {
        self.shape_scaled(placed, 1.0)
    }

    fn shape_scaled(&mut self, placed: &PlacedSurface<'_>, scale: f32) -> Result<u32, String> {
        let node = placed.surface.surf.as_ref().ok_or("surface without surf")?;
        let ids: Vec<u8> = placed
            .materials
            .iter()
            .flat_map(|material| {
                material.as_ref().map_or([0xff, 0xff], |(_, ids)| [ids.surface, ids.gameplay])
            })
            .collect();
        let preserve_ids = matches!(placed.role, Role::BlockSpecialTrigger | Role::BlockTrigger(_));
        let key = (placed.id, ids, scale.to_bits(), preserve_ids, placed.mergeable);
        if let Some(&shape) = self.shape_cache.get(&key) {
            return Ok(shape);
        }
        let shape = self.surf(node, placed)?;
        if scale != 1.0 {
            if !matches!(node.surf, Surf::Mesh { .. }) {
                return Err("scaled tree collision other than mesh".into());
            }
            let gearbox_engine::static_world::Geometry::Mesh(mesh) =
                &mut self.shapes[shape as usize].geometry
            else {
                unreachable!("Surf::Mesh builds Geometry::Mesh")
            };
            for vertex in &mut mesh.vertices {
                *vertex *= scale;
            }
        }
        if let Some(effect) = gameplay_effect(placed) {
            self.gameplay
                .insert(shape, crate::gameplay::gates::Properties { effect, mergeable: placed.mergeable });
        }
        self.shape_cache.insert(key, shape);
        Ok(shape)
    }

    fn surf(
        &mut self,
        node: &gearbox_packs::surface::SurfNode,
        placed: &PlacedSurface<'_>,
    ) -> Result<u32, String> {
        let dir = node.gameplay_main_dir.unwrap_or([0.0, 0.0, 1.0]);
        match &node.surf {
            Surf::Mesh { vertices, triangles, .. } => {
                let tris: Vec<[u8; 16]> = triangles.iter().map(|t| game_triangle(t, placed)).collect();
                crate::statics::shapes::mesh_create(&mut self.shapes, vertices, &tris, dir)
            }
            Surf::Compound { surfs, locs, joints } => {
                let children = surfs.iter().map(|c| self.surf(c, placed)).collect::<Result<Vec<_>, _>>()?;
                Ok(crate::statics::shapes::compound_create(&mut self.shapes, &children, locs, joints, dir))
            }
            Surf::ConvexPolyhedron { raw, material, .. } => {
                let slot = *material as u16 as usize;
                let ids = placed
                    .materials
                    .get(slot)
                    .and_then(|m| m.as_ref())
                    .map(|(_, ids)| u16::from_le_bytes([ids.surface, ids.gameplay]))
                    .or_else(|| placed.surface.material_ids.get(slot).copied())
                    .ok_or_else(|| format!("convex material slot {slot} absent"))?;
                crate::statics::shapes::convex_create(&mut self.shapes, raw, ids, dir)
            }
            _ => Err(format!("surf kind {} not built yet", node.kind)),
        }
    }

    pub fn modifiers(
        &self,
        map: &gearbox_gbx::Map,
        grid: &crate::statics::instancing::Grid,
        block: &gearbox_gbx::map::Block,
        depth: usize,
    ) -> Result<Mods, String> {
        if depth > 256 {
            return Err("modifier parents nested too deep".into());
        }
        let memo = (
            block.name.clone(),
            block.coord,
            block.dir,
            block.flags,
            block.free.map(|(position, angles)| (position.map(f32::to_bits), angles.map(f32::to_bits))),
        );
        if let Some(modifier) = self.mods_cache.borrow().get(&memo) {
            return Ok(modifier.clone());
        }
        let assets = self.assets;
        let Some(info) = assets.block_in(&block.name, &self.env).block() else { return Ok([None, None]) };
        let key = BlockKey::of(block);
        let is_clip = gearbox_packs::blockinfo::is_clip(info.class_id);
        let geometry = self.catalog.geometry(&key);
        let clip_parent = || match &geometry {
            Some(geometry) => crate::statics::instancing::clip_parent(map, grid, block, geometry)
                .map_err(|error| error.to_string()),
            None => Ok(None),
        };
        let inherited = if is_clip && info.clip_type != 0 && info.material_modifier.is_none() {
            clip_parent()?.map(|p| self.modifiers(map, grid, p, depth + 1)).transpose()?
        } else {
            None
        };
        let [i1, i2] = inherited.unwrap_or([None, None]);
        let own = |modifier: &Option<String>| modifier.clone().map(|modifier| (modifier, None, false));
        let mut slots = [own(&info.material_modifier).or(i1), own(&info.material_modifier2).or(i2)];
        let skin = block.skin.as_ref().and_then(|s| s.first()).filter(|s| !s.is_empty());
        let owned = [info.material_modifier.is_some(), info.material_modifier2.is_some()];
        for (slot, own) in slots.iter_mut().zip(owned) {
            let Some((path, folder @ None, _)) = slot else { continue };
            if own && let Some(s) = skin {
                let dir = &path[..path.rfind('\\').map_or(0, |i| i + 1)];
                *folder = Some(format!("{dir}{s}"));
            }
        }
        if let Some((_, _, keep)) = &mut slots[0] {
            *keep = map.flags.is_some_and(|f| f & 4 == 0);
        }
        self.mods_cache.borrow_mut().insert(memo, slots.clone());
        Ok(slots)
    }

    pub fn surfaces(&self, key: &BlockKey, mods: &Mods) -> Result<Vec<PlacedSurface<'a>>, String> {
        let assets = self.assets;
        let block = assets.block_in(&key.name, &self.env).block().ok_or("unknown block")?;
        let (ground, index) = (key.flags & 0x1000 != 0, ((key.flags >> 21) & 0x7f) as usize);
        let variant = block
            .variants
            .iter()
            .find(|variant| variant.ground == ground && variant.index == index)
            .ok_or("no such variant")?;
        let mut mods: Vec<ModifierRef> = mods
            .iter()
            .flatten()
            .map(|(path, folder, keep)| ModifierRef {
                path,
                folder: folder.as_deref(),
                keep_track_wall: *keep,
            })
            .collect();
        if let Some(p) = assets.index.collection_modifiers.get(&self.env) {
            mods.push(ModifierRef { path: p, folder: None, keep_track_wall: false });
        }
        let prefab = variant
            .mobils
            .get(key.variant as usize)
            .and_then(|group| group.get(key.sub_variant as usize).or_else(|| group.first()))
            .and_then(|mobil| mobil.prefab.as_ref());
        let mut out = match prefab {
            Some(prefab) => assets.prefab_surfaces_with(prefab, &mods).map_err(|e| e.to_string())?,
            None => Vec::new(),
        };
        if let Some(id) = variant.waypoint_trigger_surface {
            let role = if key.has_waypoint || block.waypoint.is_some() {
                Role::Waypoint(u32::MAX)
            } else {
                Role::BlockTrigger(None)
            };
            out.push(assets.place(id, role, &[], &mods));
        } else if variant.waypoint_trigger.is_some() {
            return Err("waypoint trigger shape not extracted".into());
        }
        Ok(out)
    }

    fn build(&mut self, key: &BlockKey, mods: &Mods) -> Result<CachedBlock<'a>, String> {
        let geometry = self.catalog.geometry(key).ok_or("no placement metadata")?;
        let mut parts = Vec::new();
        let mut trees = Vec::new();
        for mut placed in self.surfaces(key, mods)? {
            if placed.role == Role::BlockTrigger(None)
                && ((placed.surface.materials.is_empty() && placed.surface.material_ids.is_empty())
                    || gameplay_effect(&placed) == Some(0))
            {
                continue;
            }
            if placed.role == Role::SpecialTrigger {
                placed.role = Role::BlockSpecialTrigger;
            }
            if self.assets.index.surfaces[placed.id].tree.is_some() {
                trees.push(placed);
                continue;
            }
            let (layer, entity) = match placed.role {
                Role::Collision | Role::DynaStaticShape => (LAYER_STATIC, Entity::None),
                Role::Waypoint(_) => (LAYER_TRIGGER, Entity::Waypoint(None)),
                Role::SpecialTrigger | Role::BlockSpecialTrigger | Role::BlockTrigger(None) => {
                    (LAYER_GATE, Entity::Gate(0))
                }
                Role::BlockTrigger(Some(_)) => (LAYER_TRIGGER, Entity::Waypoint(None)),
                Role::DynaShape => continue,
            };
            let shape = self.shape(&placed)?;
            let local = chain_iso(&placed.chain);
            parts.push(Part {
                shape,
                location: if matches!(placed.role, Role::BlockTrigger(_))
                    || (layer == LAYER_TRIGGER && local == crate::statics::place::IDENTITY)
                {
                    PartLocation::Block
                } else {
                    PartLocation::Local(local)
                },
                layer,
                entity,
            });
        }
        crate::sort_by_key(&mut parts, |p| match p.layer {
            LAYER_STATIC => 0,
            crate::statics::anchored::LAYER_GATE => 1,
            LAYER_TRIGGER => 2,
            _ => unreachable!(),
        });
        Ok(CachedBlock { model: BlockModel { geometry, parts }, trees })
    }

    fn tree_instance(
        &mut self,
        placed: &PlacedSurface<'_>,
        pose: Iso,
        orientation: Orientation,
    ) -> Result<(Iso, u32), String> {
        let params = self.assets.index.surfaces[placed.id].tree.ok_or("tree parameters missing")?;
        let (pose, scale) = crate::statics::veget::placement(pose, params, orientation);
        let bin = gearbox_engine::numeric::lroundf(
            ((scale - params.scale_variation) / (1.0 - params.scale_variation)) * 7.0,
        );
        if !(0..8).contains(&bin) {
            return Err(format!("tree scale cache slot {bin}"));
        }
        let scale = if bin == 7 { 1.0 } else { *self.tree_scales.entry((placed.id, bin)).or_insert(scale) };
        Ok((pose, self.shape_scaled(placed, scale)?))
    }

    pub fn item_parts(
        &mut self,
        anchored: &gearbox_gbx::map::Item,
        iso: Iso,
        location: ([f32; 9], [f32; 3]),
    ) -> Result<Vec<(Role, u32, gearbox_engine::Iso)>, String> {
        let assets = self.assets;
        let name = anchored.model.as_deref().ok_or("item without a model")?;
        let path =
            assets.find_anchored_item(anchored, &self.env).ok_or_else(|| format!("unknown item {name}"))?;
        let variant = (anchored.flags >> 8) as usize;
        let item = &assets.index.items[path];
        let models = item.models(variant);
        if models.iter().any(|model| matches!(model, gearbox_packs::catalog::EntModel::Vehicle { .. })) {
            return Err(format!("{path}: parked vehicle body not simulated"));
        }
        if item.models.is_empty() && item.variants.is_empty() {
            let why = assets
                .index
                .errors
                .iter()
                .find(|error| error.starts_with(path))
                .cloned()
                .unwrap_or_else(|| "item physics model not extracted".into());
            return Err(format!("{path}: {why}"));
        }
        let mut out = Vec::new();
        let mut dynamic = false;
        for placed in assets.item_model_variant(path, variant).map_err(|e| e.to_string())? {
            match placed.role {
                Role::Collision | Role::SpecialTrigger | Role::Waypoint(_) | Role::DynaStaticShape => {}
                Role::BlockSpecialTrigger | Role::BlockTrigger(_) => {
                    return Err("block trigger used as an anchored item".into());
                }
                Role::DynaShape => {
                    dynamic = true;
                    continue;
                }
            }
            let local = (!placed.chain.is_empty()).then(|| chain_iso(&placed.chain));
            let pose = local.map_or(iso, |local| crate::statics::place::compose(iso, local));
            let (transform, shape) = match assets.index.surfaces[placed.id].tree {
                Some(_) => {
                    let orientation = if local.is_none() { Orientation::Random } else { Orientation::Kept };
                    let (pose, shape) = self.tree_instance(&placed, pose, orientation)?;
                    (crate::statics::place::transform(pose), shape)
                }
                None => (
                    local.map_or_else(
                        || crate::statics::place::transform_matrix(location),
                        |_| crate::statics::place::transform(pose),
                    ),
                    self.shape(&placed)?,
                ),
            };
            out.push((placed.role, shape, gearbox_engine::Iso::from_array(transform.map(f32::from_bits))));
        }
        crate::sort_by_key(&mut out, |p| match p.0 {
            Role::Waypoint(_) => 1,
            Role::SpecialTrigger => 2,
            _ => 0,
        });
        let empty = models.iter().all(|m| matches!(m, gearbox_packs::catalog::EntModel::None));
        if out.is_empty() && !dynamic && !empty {
            return Err(format!(
                "{path}: item has no decoded collision parts \
                 (mesh-collidable Solid2Model or unsupported item model)",
            ));
        }
        out.retain(|(role, ..)| !matches!(role, Role::Waypoint(0 | 3 | 5)));
        Ok(out)
    }

    fn model_with(&mut self, key: &BlockKey, mods: Mods) -> Option<CachedBlock<'a>> {
        let cache_key = (key.clone(), mods);
        if let Some(model) = self.models.get(&cache_key) {
            return model.clone();
        }
        let model = match self.build(key, &cache_key.1) {
            Ok(model) => Some(model),
            Err(e) => {
                self.problems.insert(key.name.clone(), e);
                None
            }
        };
        self.models.insert(cache_key, model.clone());
        model
    }
}

pub type Mods = [Option<(String, Option<String>, bool)>; 2];

type ModsKey = (String, [i32; 3], u8, u32, Option<([u32; 3], [u32; 3])>);

impl ModelSource for PackSource<'_> {
    fn covers_terrain(
        &self,
        block: &gearbox_gbx::map::Block,
        unit: [i32; 3],
        terrain: &gearbox_gbx::map::Block,
    ) -> anyhow::Result<bool> {
        self.catalog.covers_terrain(block, unit, terrain)
    }

    fn block_geometry(&self, key: &BlockKey) -> Option<Geometry> {
        self.catalog.geometry(key)
    }

    fn null_block(&self, block: &gearbox_gbx::map::Block) -> bool {
        matches!(self.assets.block_in(&block.name, &self.env), BlockLookup::Null)
    }

    fn block_model(&mut self, key: &BlockKey) -> Option<BlockModel> {
        let info = self.assets.block_in(&key.name, &self.env).block();
        let mods = [
            info.and_then(|block| block.material_modifier.clone()).map(|path| (path, None, false)),
            info.and_then(|block| block.material_modifier2.clone()).map(|path| (path, None, false)),
        ];
        self.model_with(key, mods).map(|cached| cached.model)
    }

    fn block_model_at(
        &mut self,
        map: &gearbox_gbx::Map,
        grid: &crate::statics::instancing::Grid,
        b: &gearbox_gbx::map::Block,
    ) -> Option<BlockModel> {
        let key = BlockKey::of(b);
        let built = (|| {
            let mods = self.modifiers(map, grid, b, 0)?;
            let Some(cached) = self.model_with(&key, mods) else { return Ok(None) };
            let mut model = cached.model;
            let parent = crate::statics::model::block_iso_with(b, &model.geometry);
            for placed in cached.trees {
                let pose = crate::statics::place::compose(parent, chain_iso(&placed.chain));
                let (pose, shape) = self.tree_instance(&placed, pose, Orientation::Kept)?;
                model.parts.push(Part {
                    shape,
                    location: PartLocation::World(pose),
                    layer: LAYER_STATIC,
                    entity: Entity::None,
                });
            }
            Ok(Some(model))
        })();
        match built {
            Ok(model) => model,
            Err(e) => {
                self.problems.insert(key.name.clone(), e);
                None
            }
        }
    }
}
