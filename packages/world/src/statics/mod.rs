pub(crate) mod anchored;
pub(crate) mod build;
pub(crate) mod decoration;
pub(crate) mod instancing;
pub(crate) mod items;
mod mapworld;
pub(crate) mod model;
pub(crate) mod packs;
pub(crate) mod shapes;
mod spawn;
mod tree;
mod veget;

use anyhow::{Result, ensure};
use gearbox_engine::{
    Iso, World,
    static_world::{Entity, Geometry, Item, StaticWorld, Surface},
};
pub use gearbox_packs::placement as place;
use gearbox_packs::{assets::Assets, packs::PackSet};
pub use model::{BlockKey, Catalog, Geometry as BlockGeometry};
pub use spawn::{block_pose, item_poses, item_waypoint_tags};
use std::collections::HashSet;

#[derive(Clone, Debug, Default)]
pub struct Selection {
    pub omitted_items: Vec<usize>,
}

pub struct MapAssets<'a> {
    pub assets: Assets,
    pub packs: &'a PackSet,
}

impl<'a> MapAssets<'a> {
    pub fn from_packs(packs: &'a PackSet, filters: &[&str], map: &gearbox_gbx::Map) -> Result<Self> {
        let catalog =
            gearbox_packs::catalog::crawl_cached(packs, filters, &gearbox_packs::install::cache_dir())?;
        Self::from_catalog(Assets::from_catalog(catalog), packs, map)
    }

    pub fn from_catalog(mut assets: Assets, packs: &'a PackSet, map: &gearbox_gbx::Map) -> Result<Self> {
        let errors = packs::add_map_items(&mut assets, map, packs);
        ensure!(errors.is_empty(), "map assets: {}", errors.join("; "));
        Ok(Self { assets, packs })
    }

    pub fn geometry(&self, map: &gearbox_gbx::Map) -> Result<Catalog> {
        let env = packs::environment(map).map_err(anyhow::Error::msg)?;
        let mut catalog = Catalog::from_assets(&self.assets, env);
        catalog.cell = packs::map_cell(&self.assets, map);
        catalog.load_placement_metadata(&self.assets, map, env, self.packs)?;
        Ok(catalog)
    }
}

fn surfaces(
    shapes: &[gearbox_engine::static_world::Shape],
    out: &mut Vec<Surface>,
    item: u32,
    shape: u32,
    iso: Iso,
) {
    if let Geometry::Compound(compound) = &shapes[shape as usize].geometry {
        for (&child, &local) in compound.children.iter().zip(&compound.locations) {
            surfaces(shapes, out, item, child.shared().expect("static child").0, iso * local);
        }
    } else {
        out.push(Surface { shape, iso, item, tag: gearbox_engine::static_world::NO_CHILD_TAG });
    }
}

#[derive(Default)]
struct GateIds(u32);

impl GateIds {
    fn assign(&mut self, entity: Entity) -> Entity {
        if !matches!(entity, Entity::Gate(_)) {
            return entity;
        }
        let id = self.0;
        self.0 += 1;
        Entity::Gate(id)
    }
}

pub fn build(map: &gearbox_gbx::Map, assets: &MapAssets<'_>, selection: &Selection) -> Result<StaticWorld> {
    let mut source =
        packs::PackSource::for_map(&assets.assets, map, assets.packs).map_err(anyhow::Error::msg)?;
    let built = build::static_items(map, &mut source, assets.packs)?;
    ensure!(built.gaps.is_empty(), "static world incomplete: {}", built.gaps.join("; "));
    finish(map, source, built, selection)
}

pub(crate) fn finish(
    map: &gearbox_gbx::Map,
    mut source: packs::PackSource<'_>,
    built: build::Built,
    selection: &Selection,
) -> Result<StaticWorld> {
    ensure!(selection.omitted_items.iter().all(|&i| i < built.items.len()), "static selection out of range");
    let material14_alias = mapworld::runtime_flags(map) & mapworld::FLAG_MATERIAL14_ALIAS != 0;
    let mut world = StaticWorld { material14_alias, ..StaticWorld::default() };
    let mut gates = GateIds::default();
    let mut prepared = HashSet::new();
    for (i, spec) in built.items.into_iter().enumerate() {
        if selection.omitted_items.contains(&i) {
            continue;
        }
        let entity = gates.assign(spec.entity);
        shapes::prepare(&mut source.shapes, spec.shape, &mut prepared);
        let index = world.items.len() as u32;
        world.items.push(Item {
            entity,
            shape: spec.shape,
            iso: spec.transform,
            layer: spec.layer,
            flags: gearbox_engine::collision::ItemEvents::from_bits(spec.flags),
            query_flags: gearbox_engine::collision::ItemFlags::NONE,
            id: index,
            index,
        });
    }
    world.shapes = source.shapes;
    for (i, item) in world.items.iter().enumerate() {
        surfaces(&world.shapes, &mut world.surfaces, i as u32, item.shape, item.iso);
    }
    let mut boxes = world
        .surfaces
        .iter()
        .enumerate()
        .map(|(i, s)| tree::Leaf {
            bounds: shapes::bounds(&world.shapes[s.shape as usize]).transform(&s.iso),
            index: i as u32,
        })
        .collect::<Vec<_>>();
    world.tree = if boxes.is_empty() {
        gearbox_engine::static_world::Tree { leaf_size: 0, ..Default::default() }
    } else {
        tree::build(&mut boxes, 2)
    };
    Ok(world)
}

pub fn populate(
    world: &mut World,
    map: &gearbox_gbx::Map,
    assets: &MapAssets<'_>,
    selection: &Selection,
) -> Result<()> {
    let statics = build(map, assets, selection)?;
    let water = crate::water::build(map, assets)?;
    world.statics = statics;
    world.water = water;
    Ok(())
}
