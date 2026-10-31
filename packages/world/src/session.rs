use crate::{
    bodies::{self, Built},
    car::{self, Car, CarAssets, VehicleStart},
    gameplay::{self, Gameplay},
    statics::MapAssets,
};
use anyhow::Result;
use gearbox_engine::{
    Iso, State, World, arena,
    dyna::Dyna,
    ghost::Playback,
    race::RaceRules,
    session::{Clock, Session, SessionConfig, VehicleAssets, VehicleModel},
    static_world::{Shape, StaticWorld},
    vehicle::{Vehicle, spawn, vis},
    world_water::Water,
};
use gearbox_packs::packs::PackSet;
use std::sync::Arc;

const RECENT_EVENT_DURATION_MS: u32 = 100;
const DEFAULT_INPUT_START_MS: i32 = -1500;
const VEHICLE_CLASS: u32 = 0x0a02_0000;
const GATE_CARS: [Car; 3] = [Car::Snow, Car::Rally, Car::Desert];

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
    Prepared::new(map, packs)?.open(options)
}

pub struct Prepared<'a> {
    map: &'a gearbox_gbx::Map,
    packs: &'a PackSet,
    layout: gameplay::Layout,
    water: Water,
    bodies: bodies::Built,
}

impl<'a> Prepared<'a> {
    pub fn new(map: &'a gearbox_gbx::Map, packs: &'a PackSet) -> Result<Self> {
        let assets = MapAssets::from_packs(packs, &gearbox_packs::catalog::PAKS, map)?;
        Ok(Self {
            map,
            packs,
            layout: gameplay::layout(map, &assets)?,
            water: crate::water::build(map, &assets)?,
            bodies: bodies::build(map, &assets)?,
        })
    }

    /// What `new` derived from the map and packs, as bytes and the static shapes apart from them (maps share
    /// most shapes, so a cache stores each once); `decode` restores it for the same map, packs and code.
    pub fn encode(&self) -> (Vec<u8>, &[Shape]) {
        let Built { statics, selection, gates, animations, instances } = &self.bodies;
        let StaticWorld { items, surfaces, shapes: _, tree, material14_alias } = statics;
        let rest = (
            &self.layout,
            &self.water,
            items,
            surfaces,
            tree,
            material14_alias,
            selection,
            gates,
            animations,
            instances,
        );
        (postcard::to_stdvec(&rest).expect("a prepared map serializes"), &statics.shapes)
    }

    pub fn decode(
        map: &'a gearbox_gbx::Map,
        packs: &'a PackSet,
        bytes: &[u8],
        shapes: Vec<Shape>,
    ) -> Result<Self> {
        let (layout, water, items, surfaces, tree, material14_alias, selection, gates, animations, instances) =
            postcard::from_bytes(bytes)?;
        let statics = StaticWorld { items, surfaces, shapes, tree, material14_alias };
        let bodies = Built { statics, selection, gates, animations, instances };
        Ok(Self { map, packs, layout, water, bodies })
    }

    pub fn open(&self, options: OpenOptions) -> Result<Session> {
        let clock = Clock::new(options.t0).map_err(anyhow::Error::msg)?;
        let gameplay = self.layout.gameplay(self.map, clock.start, clock.now);
        let mut world = World { water: self.water.clone(), ..World::default() };
        let mut state = State::default();
        let mut dyna = bodies::manager(clock.start);
        self.bodies.clone().register(&mut world, &mut state, &mut dyna)?;
        let car = load_car(self.map, self.packs, options.car, &mut world, &mut state)?;
        let (vehicle, effects, assets) =
            spawn_car(&car, &gameplay, clock.now, &mut world, &mut state, &mut dyna)?;
        let models = load_models(self.packs, car, assets, &mut world, &mut state)?;
        state.scene.push(gameplay.scene);
        let ghosts = crate::ghost::build(self.map)?;
        let config = session_config(models, &gameplay.settings, gameplay.initial_pose, ghosts);
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
            input: Default::default(),
            applied: Default::default(),
            respawns: None,
        })
    }
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
        car: Car::ALL.iter().position(|&known| known == car.car).unwrap_or(0) as u8,
        assets,
        spawn: car.spawn,
        body: car.body,
        collision: state.compound_instance[car.template.compound().unwrap().index()].clone(),
    };
    let mut models = std::array::from_fn(|_| None);
    models[0] = Some(model(initial, assets, state));
    for (choice, car) in GATE_CARS.into_iter().enumerate().map(|(index, car)| (index + 1, car)) {
        if needed[choice] {
            let loaded = CarAssets::load(packs, car, world, state)?;
            let assets = vehicle_assets(&loaded);
            models[choice] = Some(model(loaded, assets, state));
        }
    }
    Ok(models)
}

fn session_config(
    models: [Option<VehicleModel>; 4],
    rules: &RaceRules,
    initial_pose: Iso,
    ghosts: Playback,
) -> SessionConfig {
    SessionConfig {
        models,
        manager: 0,
        scene: 0,
        recent_event_duration: RECENT_EVENT_DURATION_MS,
        rules_start: rules.start,
        options: arena::ArenaOptions(rules.flags),
        grid: Default::default(),
        initial_pose,
        ghosts,
    }
}

pub fn open_file(map: &std::path::Path, packs: &std::path::Path, options: OpenOptions) -> Result<Session> {
    let map = gearbox_gbx::Map::parse(&std::fs::read(map)?)?;
    let packs = PackSet::open(packs)?;
    open(&map, &packs, options)
}
