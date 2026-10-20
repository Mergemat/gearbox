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
    session::{Clock, Session, SessionConfig, VehicleAssets, VehicleModel},
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
    let (mut vehicle, effects, assets) =
        spawn_car(&car, &gameplay, clock.now, &mut world, &mut state, &mut dyna)?;
    vehicle.model_choice = car.car as u8;
    let models = load_models(packs, car, assets, &mut world, &mut state)?;
    state.scene.push(gameplay.scene);
    let config = session_config(models, &gameplay.settings, gameplay.initial_pose);
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
        steer_limit: 1.0,
        respawns: None,
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
    let assets = vehicle_assets(car);
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

fn vehicle_assets(car: &CarAssets) -> VehicleAssets {
    VehicleAssets {
        tuning: car.tuning().clone(),
        geometry: car.geometry.clone(),
        fluid: car.fluid.clone(),
        impacts: car.impacts(),
        post: car.post_model(),
    }
}

fn load_models(
    packs: &PackSet,
    initial: CarAssets,
    assets: VehicleAssets,
    world: &mut World,
    state: &mut State,
) -> Result<[Option<VehicleModel>; 4]> {
    let mut needed = [false; 4];
    for gate in &world.gate_record {
        if (20..=23).contains(&gate.effect) {
            needed[(gate.effect - 20) as usize] = true;
        }
    }
    for shape in &world.statics.shapes {
        match &shape.geometry {
            gearbox_engine::static_world::Geometry::Mesh(mesh) => {
                for triangle in &mesh.triangles {
                    if (20..=23).contains(&triangle.gameplay) {
                        needed[(triangle.gameplay - 20) as usize] = true;
                    }
                }
            }
            gearbox_engine::static_world::Geometry::Convex(convex) => {
                let effect = convex.material >> 8;
                if (20..=23).contains(&effect) {
                    needed[(effect - 20) as usize] = true;
                }
            }
            gearbox_engine::static_world::Geometry::Compound(_) => {}
        }
    }
    let model = |car: CarAssets, assets, state: &State| VehicleModel {
        assets,
        spawn: car.spawn,
        body: car.body,
        collision: state.compound_instance[car.template.compound().unwrap().index()].clone(),
    };
    let mut models = std::array::from_fn(|_| None);
    let choice = initial.car as usize;
    models[choice] = Some(model(initial, assets, state));
    for car in Car::ALL {
        if needed[car as usize] && models[car as usize].is_none() {
            let loaded = CarAssets::load(packs, car, world, state)?;
            let assets = vehicle_assets(&loaded);
            models[car as usize] = Some(model(loaded, assets, state));
        }
    }
    Ok(models)
}

fn session_config(models: [Option<VehicleModel>; 4], rules: &RaceRules, initial_pose: Iso) -> SessionConfig {
    SessionConfig {
        models,
        manager: 0,
        scene: 0,
        recent_event_duration: RECENT_EVENT_DURATION_MS,
        rules_start: rules.start,
        options: arena::ArenaOptions(rules.flags),
        grid: Default::default(),
        initial_pose,
    }
}

pub fn open_file(map: &std::path::Path, packs: &std::path::Path, options: OpenOptions) -> Result<Session> {
    let map = gearbox_gbx::Map::parse(&std::fs::read(map)?)?;
    let packs = PackSet::open(packs)?;
    open(&map, &packs, options)
}
