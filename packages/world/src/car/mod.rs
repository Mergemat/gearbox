mod auxiliary;
mod contacts;
mod geometry;
mod model;
mod vehicle;

use anyhow::{Result, ensure};
use gearbox_packs::{car_model, packs::PackSet};
use gearbox_engine::{ShapeId, State, World, dyna::DynaModel, vehicle::{Tuning, CarShape,
    model::ForceGeometry, spawn}, water::drag::FluidTuning};
pub use auxiliary::Auxiliary;
pub use vehicle::{VehicleStart, new_vehicle, spawn_vehicle};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Car { Stadium, Snow, Rally, Desert }
impl Car {
    pub const ALL: [Self; 4] = [Self::Stadium, Self::Snow, Self::Rally, Self::Desert];
    pub fn pack_name(self) -> &'static str {
        match self {
            Self::Stadium => "CarSport", Self::Snow => "CarSnow",
            Self::Rally => "CarRally", Self::Desert => "CarDesert",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CarAssets {
    pub car: Car,
    pub tunings: Vec<Tuning>,
    pub active_tuning: usize,
    pub fluid: FluidTuning,
    pub shape: CarShape,
    pub impacts: gearbox_engine::vehicle::impacts::ImpactModel,
    pub body: DynaModel,
    pub geometry: ForceGeometry,
    pub spawn: spawn::Assets,
    pub auxiliary: Auxiliary,
    pub source_collision: ShapeId,
    pub template: ShapeId,
    pub water_shape: ShapeId,
}

impl CarAssets {
    pub fn tuning(&self) -> &Tuning { &self.tunings[self.active_tuning] }

    pub fn load(packs: &PackSet, car: Car, world: &mut World, state: &mut State) -> Result<Self> {
        let source = car_model::load(packs, car.pack_name())?;
        let decoded = gearbox_packs::tuning::decode(&packs.read_path(&source.tuning_path)?)?;
        let index = decoded.active;
        let fluid = decoded.fluid(index)?;
        let tunings = decoded.tunings;
        let tuning = &tunings[index];
        ensure!(matches!(tuning.model_kind, 0..=2 | 5),
            "unsupported car force model {}", tuning.model_kind);
        let shape = source.shape.clone();
        let auxiliary = auxiliary::build(source.auxiliary())?;
        let prepared = model::prepare(&source, &shape, tuning, &auxiliary, world, state)?;
        Ok(Self {
            car, tunings, active_tuning: index, fluid, auxiliary, shape, impacts: prepared.impacts,
            body: prepared.body, geometry: prepared.geometry,
            spawn: prepared.spawn, source_collision: prepared.source_collision,
            template: prepared.template, water_shape: prepared.water_shape,
        })
    }
}
