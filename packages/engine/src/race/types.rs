use crate::Vec3;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct GateRecord {
    pub id: u32,
    pub effect: u8,
    pub dir: Vec3,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RaceRules {
    pub flags: u32,
    pub mode_bits: u8,
    pub start: u32,
    pub checkpoints: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RaceRun {
    pub id: u32,
    pub finish: u32,
    pub start: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Player {
    pub previous_input: super::replay::InputState,
    pub current_input: super::replay::InputState,
    pub handled_events: [u32; 8],
    pub keyed: u32,
    pub ghost_active: u32,
    pub tag: u8,
    pub landmark: u32,
    pub race_start: u32,
    pub spawned_at: u32,
    pub window_end: u32,
    pub replay_seed: u32,
    pub entity: u32,
    pub seat_entity: u32,
    pub override_entity: u32,
    pub seats: [u32; 6],
    pub start_order: u8,
    pub race_run: u32,
    pub lap_run: u32,
    pub ghost_start: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlayerController {
    pub seat: u8,
    pub trigger: u32,
    pub start_adjustments: u8,
}
