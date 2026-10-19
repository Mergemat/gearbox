pub mod countdown;
pub mod replay;
pub mod store;
pub mod trigger;
mod types;
pub use store::{Chunk, Entry, IdSequence, Store};
pub use trigger::{Contact, Contacts, Phase};
pub use types::*;

use crate::vehicle::race_flags::RaceFlags;
use crate::{Iso, static_world::StaticWorld, vehicle::Vehicle};

pub const EPOCH_MS: u32 = 60_000;
pub const NO_RUN: u32 = u32::MAX;
pub const NO_ENTITY: u32 = 0x0ff0_0000;
pub const INITIAL_SPAWN: u32 = u32::MAX - 1;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RaceState {
    pub checkpoints: Vec<u32>,
    pub laps: u32,
    pub finish: Option<u32>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WaypointFlags(pub u32);
impl WaypointFlags {
    pub const SPAWN: Self = Self(1);
    pub const SPECIAL: Self = Self(2);
    pub const CHECKPOINT: Self = Self(0x80);
    pub const GOAL: Self = Self(0x100);
    pub fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
    pub fn counts(self) -> bool {
        self.0 & (Self::CHECKPOINT.0 | Self::GOAL.0) != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Free,
    Race,
    Laps,
    Disabled,
}
impl Mode {
    const KIND_MASK: u8 = 0x38;
    const RACE: u8 = 0x08;
    const LAPS: u8 = 0x10;
    pub fn from_bits(bits: u8) -> Self {
        match bits & Self::KIND_MASK {
            0 => Self::Free,
            Self::RACE => Self::Race,
            Self::LAPS => Self::Laps,
            _ => Self::Disabled,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaypointKind {
    Ignored,
    Checkpoint,
    Lap,
    Finish,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RaceWaypoint {
    pub flags: WaypointFlags,
    pub group: u32,
    pub order: u32,
    pub pose: Iso,
    pub linked: bool,
    pub respawn: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Rules {
    pub mode: Mode,
    pub checkpoints_per_lap: u32,
    pub lap_race: bool,
    pub laps: u32,
    pub waypoints: Vec<RaceWaypoint>,
    pub clear_untouched: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WaypointState {
    pub arrays: [Vec<u64>; 2],
    pub pairs: [[u32; 2]; 4],
}

#[derive(Clone, Debug, PartialEq)]
pub struct Racer {
    pub player: Player,
    pub controller: Option<PlayerController>,
    pub waypoints: WaypointState,
    pub saved_waypoints: WaypointState,
    pub open_lap: Option<usize>,
    pub closed_lap: Option<usize>,
    pub timed_events: [TimedEvent; 8],
}
impl Racer {
    pub fn controlled_entity(&self) -> u32 {
        for handle in [self.player.override_entity, self.player.seat_entity] {
            if handle != NO_ENTITY {
                return handle;
            }
        }
        if let Some(controller) = &self.controller
            && let Some(&slot) = crate::race::SEAT_SLOTS.get(controller.seat as usize)
            && slot != 6
        {
            return self.player.seats[slot as usize];
        }
        self.player.entity
    }

    pub fn has_entity(&self, handle: u32) -> bool {
        handle != NO_ENTITY
            && ([self.player.override_entity, self.player.seat_entity, self.player.entity].contains(&handle)
                || crate::race::SEAT_SLOTS.iter().any(|&slot| self.player.seats[slot as usize] == handle))
    }

    pub fn save_waypoints(&mut self) {
        if self.controller.is_some() {
            self.saved_waypoints = self.waypoints.clone();
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Race {
    pub rules: Rules,
    pub store: Store,
    pub players: Vec<Racer>,
}
impl Race {
    pub fn waypoint_kind(&self, player: &Racer, waypoint: u32) -> WaypointKind {
        match self.rules.mode {
            Mode::Free => return WaypointKind::Checkpoint,
            Mode::Disabled => return WaypointKind::Ignored,
            _ => {}
        }
        if self.store.run(player.player.race_run).is_some_and(|run| run.finish != NO_RUN) {
            return WaypointKind::Ignored;
        }
        let mark = &self.rules.waypoints[waypoint as usize];
        if !mark.flags.contains(WaypointFlags::GOAL) {
            return if self
                .store
                .entries(player.player.lap_run)
                .any(|entry| self.rules.waypoints[entry.waypoint as usize].group == mark.group)
            {
                WaypointKind::Ignored
            } else {
                WaypointKind::Checkpoint
            };
        }
        if self.store.entry_count(player.player.lap_run) < self.rules.checkpoints_per_lap {
            return WaypointKind::Ignored;
        }
        if !self.rules.lap_race || self.rules.laps == 0 {
            return WaypointKind::Finish;
        }
        if self.rules.mode == Mode::Laps {
            return WaypointKind::Lap;
        }
        let count = self.store.entry_count(player.player.race_run);
        let per_lap = self.rules.checkpoints_per_lap.wrapping_add(1);
        let done = (count / per_lap).min(self.rules.laps - 1);
        if done + 1 >= self.rules.laps { WaypointKind::Finish } else { WaypointKind::Lap }
    }

    pub fn state(&self, index: usize) -> Option<RaceState> {
        let player = self.players.get(index)?;
        let run = self.store.run(player.player.race_run)?;
        let mut state = RaceState::default();
        for entry in self.store.entries(run.id) {
            state.checkpoints.push(entry.time.wrapping_sub(run.start));
            if self.rules.waypoints[entry.waypoint as usize].flags.contains(WaypointFlags::GOAL) {
                state.laps += 1;
            }
        }
        state.finish = (run.finish != NO_RUN).then(|| run.finish.wrapping_sub(run.start));
        Some(state)
    }

    pub fn waypoint(&mut self, index: usize, mut vehicle: Option<&mut Vehicle>, waypoint: u32, time: u32) {
        let Some(mark) = self.rules.waypoints.get(waypoint as usize) else {
            return;
        };
        if !mark.linked {
            return;
        }
        if let Some(vehicle) = &mut vehicle {
            vehicle.reactor_ready = 1;
        }
        let kind = self.waypoint_kind(&self.players[index], waypoint);
        if kind == WaypointKind::Ignored {
            return;
        }
        let racer = &mut self.players[index];
        if self.rules.mode != Mode::Free {
            self.store.record(
                racer.player.tag,
                [racer.player.race_run, racer.player.lap_run],
                Entry { time, waypoint: waypoint as u16 },
                kind == WaypointKind::Finish,
            );
            if kind == WaypointKind::Lap {
                racer.close_lap(&self.store);
                racer.open_lap(&mut self.store, time / 10 * 10);
            }
        }
        racer.save_waypoints();
        if let Some(vehicle) = vehicle {
            vehicle.flags = RaceFlags(vehicle.flags).count_waypoint(kind == WaypointKind::Finish).0;
        }
    }
}

pub fn on_events(
    race: &mut Race,
    statics: &StaticWorld,
    vehicles: &mut [Vehicle],
    now: u32,
    contacts: &[Contact],
) {
    let events = trigger::events(race, statics, vehicles, now, contacts);
    for event in events {
        if !trigger::dispatch(race, vehicles, event) {
            break;
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TimedEvent {
    pub time: u32,
    pub enabled: bool,
}

pub const SEAT_SLOTS: [u8; 4] = [0, 3, 4, 5];
