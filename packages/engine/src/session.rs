use crate::{
    Iso, State, World, arena,
    collision::ByteGrid,
    dyna::{BodyHandle, BodyState, Dyna},
    race::{self, Race, RaceState},
    vehicle::{Tuning, Vehicle, impacts::ImpactModel, model::ForceGeometry, post, spawn, step},
    water::drag::FluidTuning,
};
use std::sync::Arc;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Controls {
    pub gas: f32,
    pub brake: f32,
    pub steer: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clock {
    pub start: u32,
    pub now: u32,
    pub race_ms: i32,
}
impl Clock {
    pub fn new(t0: i32) -> Result<Self, &'static str> {
        if t0 % 10 != 0 {
            return Err("input start must be on a 10-ms tick");
        }
        let race_ms = t0.min(-20);
        let start = u32::try_from(i64::from(race::EPOCH_MS) - i64::from(race_ms))
            .map_err(|_| "input countdown exceeds clock range")?;
        Ok(Self { start, now: race::EPOCH_MS, race_ms })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleAssets {
    pub tuning: Tuning,
    pub geometry: ForceGeometry,
    pub impacts: ImpactModel,
    pub fluid: FluidTuning,
    pub post: post::Model,
}
impl VehicleAssets {
    pub fn view(&self) -> step::Assets<'_> {
        step::Assets {
            tuning: &self.tuning,
            geometry: &self.geometry,
            impacts: &self.impacts,
            fluid: &self.fluid,
        }
    }
}

#[derive(Clone, Debug)]
pub struct VehicleModel {
    pub assets: VehicleAssets,
    pub spawn: spawn::Assets,
    pub body: crate::dyna::DynaModel,
    pub collision: crate::collision::CompoundInstance,
}

#[derive(Clone, Debug)]
pub struct SessionConfig {
    pub models: [Option<VehicleModel>; 4],
    pub analog_steer_limits: Vec<f32>,
    pub manager: usize,
    pub scene: usize,
    pub recent_event_duration: u32,
    pub rules_start: u32,
    pub options: arena::ArenaOptions,
    pub grid: ByteGrid,
    pub initial_pose: Iso,
}

impl SessionConfig {
    pub fn model(&self, choice: u8) -> &VehicleModel {
        self.models[choice as usize].as_ref().expect("session car model")
    }
}

#[derive(Clone, Debug)]
pub struct Session {
    pub world: Arc<World>,
    pub config: Arc<SessionConfig>,
    pub state: State,
    pub vehicle: Vehicle,
    pub dyna: Dyna,
    pub race: Race,
    pub effects: spawn::EffectRing,
    pub rng: u32,
    pub clock: Clock,
    pub seed: Option<u32>,
}

#[derive(Clone, Debug)]
pub struct Savestate(Session);

impl Session {
    pub fn step(&mut self, mut controls: Controls) {
        let index = (self.clock.now.wrapping_sub(race::EPOCH_MS) / 10) as usize;
        if let Some(&limit) = self.config.analog_steer_limits.get(index)
            && self
                .config
                .model(self.vehicle.model_choice)
                .assets
                .tuning
                .car_model_kind()
                .uses_legacy_gearbox()
        {
            controls.steer *= limit;
        }
        self.tick(self.clock.now, controls, self.seed);
        self.clock.now += 10;
        self.clock.race_ms += 10;
    }

    pub fn save(&self) -> Savestate {
        Savestate(self.clone())
    }

    pub fn restore(&mut self, saved: &Savestate) {
        self.clone_from(&saved.0);
    }

    pub fn race_ms(&self) -> i32 {
        self.clock.race_ms
    }

    pub fn body(&self) -> &BodyState {
        self.dyna.state(BodyHandle(self.vehicle.body))
    }

    pub fn vehicle(&self) -> &Vehicle {
        &self.vehicle
    }

    pub fn race(&self) -> Option<RaceState> {
        self.race.state(0)
    }

    pub fn crossing_waypoints(&self) -> Vec<u32> {
        self.race.players.first().map_or_else(Vec::new, |player| {
            self.race.store.entries(player.player.race_run).map(|entry| u32::from(entry.waypoint)).collect()
        })
    }
}

impl Savestate {
    pub fn fork(&self) -> Session {
        self.0.clone()
    }
}
