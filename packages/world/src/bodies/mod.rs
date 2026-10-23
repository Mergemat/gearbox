mod archives;
mod decode;
mod model;
mod register;

use crate::statics::{self, MapAssets, Selection};
use anyhow::{Context, Result, ensure};
use gearbox_engine::{
    Iso,
    dyna::{Dyna, DynaModel, KinematicModel},
    race::GateRecord,
};

#[derive(Clone, Debug, PartialEq)]
pub struct Instance {
    pub model: DynaModel,
    pub shape: u32,
    pub animation: u32,
    pub phase: f32,
    pub placement: Iso,
}

pub struct Built {
    pub statics: gearbox_engine::static_world::StaticWorld,
    pub selection: Selection,
    pub gates: Vec<GateRecord>,
    pub animations: Vec<KinematicModel>,
    pub instances: Vec<Instance>,
}

pub fn build(map: &gearbox_gbx::Map, assets: &MapAssets<'_>) -> Result<Built> {
    let mut source =
        statics::packs::PackSource::for_map(&assets.assets, map, assets.packs).map_err(anyhow::Error::msg)?;
    let built = statics::build::static_items(map, &mut source, assets.packs)?;
    ensure!(built.gaps.is_empty(), "map objects incomplete: {}", built.gaps.join("; "));
    let mut decoder = decode::Decoder::new(map, assets.packs);
    decoder.traverse(map, &mut source, &built.items).context("map_animations::models")?;
    let selection = Selection { omitted_items: decoder.omitted };
    let effects = std::mem::take(&mut source.gameplay);
    let mut prepared = std::collections::HashSet::new();
    for body in &decoder.bodies {
        statics::shapes::prepare(&mut source.shapes, body.shape, &mut prepared);
    }
    let statics = statics::finish(map, source, built, &selection)?;
    let gates = crate::gameplay::gates(&statics, &effects)?;
    Ok(Built { statics, selection, gates, animations: decoder.animations, instances: decoder.bodies })
}

const REGISTRATION_COEFFICIENT: f32 = 1.9;
const INITIAL_RANDOM_SEED: u32 = 0x4174_7526;

pub fn manager(start: u32) -> Dyna {
    Dyna {
        coefficient: REGISTRATION_COEFFICIENT,
        gravity: gearbox_engine::Vec3::new(0.0, -10.0, 0.0),
        race_origin: start,
        rng: INITIAL_RANDOM_SEED,
        ..Dyna::default()
    }
}
