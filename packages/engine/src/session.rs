use crate::{
    Iso, State, World, arena,
    collision::ByteGrid,
    dyna::{BodyHandle, BodyState, Dyna},
    math::maxss,
    race::{
        self, Race, RaceState,
        replay::{InputBits, InputState},
    },
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

#[derive(Clone, Debug)]
pub struct VehicleModel {
    pub assets: VehicleAssets,
    pub spawn: spawn::Assets,
    pub body: crate::dyna::DynaModel,
    pub collision: crate::collision::CompoundInstance,
}

impl VehicleModel {
    pub fn view(&self) -> step::Assets<'_> {
        step::Assets {
            tuning: &self.assets.tuning,
            geometry: &self.assets.geometry,
            impacts: &self.assets.impacts,
            fluid: &self.assets.fluid,
            body: &self.body,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SessionConfig {
    pub models: [Option<VehicleModel>; 4],
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
    pub input: InputState,
    pub applied: Controls,
    pub respawns: Option<Box<crate::respawn::Respawns>>,
}

#[derive(Clone, Debug)]
pub struct Savestate(Session);

const STEER_LIMIT_KEYS: [(u32, f32); 4] = [(10, 0.2), (12, 0.4), (14, 0.6), (16, 0.8)];
const FULL_STEER_KEY: u32 = 18;
const STEER_LIMIT_EPSILON: f32 = 0.00001;

impl Session {
    pub fn step(&mut self, controls: Controls) {
        let input = InputState {
            bits: InputBits::REPLAY,
            axes: [controls.steer, controls.gas, controls.brake, 0.0],
            ..Default::default()
        };
        self.step_with_input(controls, input);
    }

    pub fn step_with_input(&mut self, mut controls: Controls, input: InputState) {
        let previous = std::mem::replace(&mut self.input, input);
        let tuning = &self.config.model(self.vehicle.model_choice).assets.tuning;
        if tuning.action_key_steer_limits != 0 {
            let limit = &mut self.vehicle.steer_limit;
            if self.clock.race_ms >= 0 {
                let pressed = input.buttons & !previous.buttons;
                for (bit, cap) in STEER_LIMIT_KEYS {
                    if pressed >> bit & 1 != 0 {
                        let scale = maxss(1.0, maxss(limit.abs(), cap));
                        *limit = if (*limit - cap).abs() <= scale * STEER_LIMIT_EPSILON { 1.0 } else { cap };
                    }
                }
                if input.buttons >> FULL_STEER_KEY & 1 != 0 {
                    *limit = 1.0;
                }
            }
            if tuning.steer_limit_clamps == 0 {
                controls.steer *= *limit;
            } else {
                controls.steer = controls.steer.clamp(-*limit, *limit);
            }
        }
        self.applied = controls;
        self.tick(self.clock.now, controls, self.seed, input);
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
