use super::{NO_RUN, Racer};
use crate::vehicle::Vehicle;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window { pub start: u32, pub epoch: u32, pub end: u32 }
impl Window {
    pub fn start(&self, rules_start: u32) -> u32 {
        if rules_start == NO_RUN || self.start == NO_RUN { NO_RUN } else { self.start.max(rules_start) }
    }
    pub fn contains(&self, now: u32, rules_start: u32) -> bool {
        let start = self.start(rules_start);
        now != NO_RUN && start != NO_RUN && now >= start
            && (self.end == NO_RUN || self.end < self.start || now < self.end)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlPhase { Racing, Countdown, Forced, Locked, Other(u8) }
impl ControlPhase {
    pub fn from_flags(flags: u32) -> Self {
        match flags & 15 { 0 => Self::Racing, 1 => Self::Countdown, 2 => Self::Forced,
            3 => Self::Locked, other => Self::Other(other as u8) }
    }
}

pub trait CountdownRespawn {
    fn respawn(&mut self, vehicle: &mut Vehicle, landmark: u32);
}

pub fn update(racer: &mut Racer, vehicle: &mut Vehicle, rules_start: u32, now: u32,
    respawn: &mut impl CountdownRespawn) {
    let player = &racer.player;
    if player.spawned_at == NO_RUN { return; }
    let window = Window { start: player.race_start, epoch: player.spawned_at, end: player.window_end };
    let start = window.start(rules_start);
    vehicle.race_start = start;
    let phase = ControlFlags(vehicle.control_flags).phase();
    if vehicle.control_entity != NO_RUN {
        if phase != ControlPhase::Locked {
            vehicle.control_flags = ControlFlags(vehicle.control_flags).forced().0;
        }
        return;
    }
    if phase == ControlPhase::Forced { return; }
    let racing = window.contains(now, rules_start) && window.epoch != NO_RUN && window.epoch == window.start;
    let waiting = now < vehicle.frozen_until && vehicle.frozen_until != NO_RUN;
    if !racing {
        if !waiting && (start == NO_RUN || now.wrapping_add(10) < start) && player.landmark != NO_RUN {
            let sequence = vehicle.respawn_sequence;
            respawn.respawn(vehicle, player.landmark);
            vehicle.respawn_sequence = sequence;
        }
        vehicle.control_flags = ControlFlags(vehicle.control_flags).countdown().0;
        return;
    }
    vehicle.control_flags = if racer.controller.is_some() {
        ControlFlags(vehicle.control_flags).racing().0
    } else { ControlFlags(vehicle.control_flags).countdown().0 };
    let Some(controller) = &mut racer.controller else { return; };
    let count = controller.start_adjustments & 3;
    let history = player.input_history();
    if count < 3 && now >= (count as u32 + 1).wrapping_mul(5000).wrapping_add(vehicle.race_start)
        && !history[1].same(&history[0]) {
        super::replay::start_adjustment(player, vehicle, controller);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RaceClock { pub start: u32, pub race_ms: i32 }
impl RaceClock {
    pub fn new(input_start: i32) -> Self {
        let race_ms = input_start.min(-20);
        Self { start: super::EPOCH_MS + race_ms.unsigned_abs(), race_ms }
    }
    pub fn now(self) -> Option<u32> {
        u32::try_from(i64::from(self.start) + i64::from(self.race_ms)).ok()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlFlags(pub u32);
impl ControlFlags {
    pub const CRUISE: Self = Self(65536);
    pub const VOLUME_CONTACT: Self = Self(131072);
    pub const VISUAL_A: Self = Self(512);
    pub const VISUAL_B: Self = Self(2048);
    pub const VISUAL_C: Self = Self(1024);
    pub fn contains(self, flag: Self) -> bool { self.0 & flag.0 == flag.0 }
    pub fn set(&mut self, flag: Self, enabled: bool) {
        if enabled { self.0 |= flag.0; } else { self.0 &= !flag.0; }
    }
    const PHASE: u32 = 15;
    const RESPAWN_TRANSIENT: u32 = 0xe00;
    pub fn phase_code(self) -> u32 { self.0 & Self::PHASE }
    pub fn phase(self) -> ControlPhase { ControlPhase::from_flags(self.0) }
    pub fn forced(self) -> Self { Self(self.0 & !(Self::PHASE & !2) | 2) }
    pub fn countdown(self) -> Self { Self(self.0 & !(Self::PHASE & !1) | 1) }
    pub fn racing(self) -> Self { Self(self.0 & !Self::PHASE) }
    pub fn after_reset(self) -> Self { Self(self.0 & !Self::RESPAWN_TRANSIENT) }
}
