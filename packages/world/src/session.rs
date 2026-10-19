use crate::{
    bodies,
    car::{self, Car, CarAssets, VehicleStart},
    gameplay::{self, Gameplay},
    statics::MapAssets,
};
use anyhow::Result;
use gearbox_engine::{
    Iso, State, World, arena,
    dyna::Dyna,
    race::RaceRules,
    session::{Clock, Session, SessionConfig, VehicleAssets},
    vehicle::{Vehicle, spawn, vis},
};
use gearbox_packs::packs::PackSet;
use std::sync::Arc;

const RECENT_EVENT_DURATION_MS: u32 = 100;
const DEFAULT_INPUT_START_MS: i32 = -1500;
const VEHICLE_CLASS: u32 = 0x0a02_0000;

#[derive(Clone, Copy, Debug)]
pub struct OpenOptions {
    pub t0: i32,
    pub seed: Option<u32>,
    pub car: Option<Car>,
}
impl Default for OpenOptions {
    fn default() -> Self {
        Self { t0: DEFAULT_INPUT_START_MS, seed: None, car: None }
    }
}

pub fn open(map: &gearbox_gbx::Map, packs: &PackSet, options: OpenOptions) -> Result<Session> {
    let clock = Clock::new(options.t0).map_err(anyhow::Error::msg)?;
    let map_assets = MapAssets::from_packs(packs, &gearbox_packs::catalog::PAKS, map)?;
    let gameplay = gameplay::build(map, &map_assets, clock.start, clock.now)?;
    let (mut world, mut state, mut dyna) = build_world(map, &map_assets, clock.start)?;
    let car = load_car(map, packs, options.car, &mut world, &mut state)?;
    let (vehicle, effects, assets) =
        spawn_car(&car, &gameplay, clock.now, &mut world, &mut state, &mut dyna)?;
    state.scene.push(gameplay.scene);
    let config = session_config(assets, car, &gameplay.settings, gameplay.initial_pose);
    Ok(Session {
        world: Arc::new(world),
        state,
        vehicle,
        dyna,
        race: gameplay.race,
        rng: 1,
        config: Arc::new(config),
        effects,
        clock,
        seed: options.seed,
    })
}

fn build_world(map: &gearbox_gbx::Map, assets: &MapAssets<'_>, start: u32) -> Result<(World, State, Dyna)> {
    let mut world = World::default();
    world.water = crate::water::build(map, assets)?;
    let mut state = State::default();
    let mut dyna = bodies::manager(start);
    bodies::build(map, assets)?.register(&mut world, &mut state, &mut dyna)?;
    Ok((world, state, dyna))
}

fn load_car(
    map: &gearbox_gbx::Map,
    packs: &PackSet,
    override_car: Option<Car>,
    world: &mut World,
    state: &mut State,
) -> Result<CarAssets> {
    let car = match override_car {
        Some(car) => car,
        None => Car::ALL
            .into_iter()
            .find(|car| car.pack_name() == map.player_model.as_deref().unwrap_or("CarSport"))
            .ok_or_else(|| anyhow::anyhow!("unsupported session car"))?,
    };
    CarAssets::load(packs, car, world, state)
}

fn spawn_car(
    car: &CarAssets,
    gameplay: &Gameplay,
    now: u32,
    world: &mut World,
    state: &mut State,
    dyna: &mut Dyna,
) -> Result<(Vehicle, spawn::EffectRing, VehicleAssets)> {
    let assets = VehicleAssets {
        tuning: car.tuning().clone(),
        geometry: car.geometry.clone(),
        fluid: car.fluid.clone(),
        impacts: car.impacts(),
        post: car.post_model(),
    };
    let mut visuals = vis::Refresh {
        config: vis::Config {
            tuning: &assets.tuning,
            geometry: &assets.geometry,
            tick: now,
            recent_event_duration: RECENT_EVENT_DURATION_MS,
        },
    };
    let start = VehicleStart {
        pose: gameplay.race.rules.waypoints[gameplay.spawn as usize].pose,
        entity: 0,
        class: VEHICLE_CLASS,
        collision_manager: 0,
    };
    let (vehicle, effects) = car::spawn_vehicle(car, start, world, state, dyna, &mut visuals)?;
    Ok((vehicle, effects, assets))
}

fn session_config(
    assets: VehicleAssets,
    car: CarAssets,
    rules: &RaceRules,
    initial_pose: Iso,
) -> SessionConfig {
    SessionConfig {
        assets,
        manager: 0,
        scene: 0,
        recent_event_duration: RECENT_EVENT_DURATION_MS,
        rules_start: rules.start,
        options: arena::ArenaOptions(rules.flags),
        grid: Default::default(),
        spawn_assets: car.spawn,
        initial_pose,
    }
}

pub fn open_file(map: &std::path::Path, packs: &std::path::Path, options: OpenOptions) -> Result<Session> {
    let map = gearbox_gbx::Map::parse(&std::fs::read(map)?)?;
    let packs = PackSet::open(packs)?;
    open(&map, &packs, options)
}
