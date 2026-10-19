use gearbox_engine::race::{*, replay::{InputBits, InputState}};

const FIRST_RUN: u32 = 0x0100_0000;
const LAP_RUN: u32 = FIRST_RUN + 1;

pub(super) fn build(start: u32, landmark: u32, entity: u32) -> Racer {
    let pairs = [[u32::MAX; 2], [0; 2], [u32::MAX; 2], [u32::MAX; 2]];
    let waypoints = WaypointState { arrays: Default::default(), pairs };
    let player = Player {

        previous_input: InputState { bits: InputBits::REPLAY, ..Default::default() },
        current_input: InputState { bits: InputBits::REPLAY, ..Default::default() },
        landmark, race_start: start, spawned_at: start, window_end: NO_RUN,
        entity, seat_entity: NO_ENTITY, override_entity: NO_ENTITY,
        seats: [entity, NO_ENTITY, NO_ENTITY, NO_ENTITY, NO_ENTITY, NO_ENTITY],
        start_order: u8::MAX, race_run: FIRST_RUN, lap_run: LAP_RUN,
        ..Player::default()
    };
    Racer {
        player, controller: Some(PlayerController { trigger: landmark, ..Default::default() }),
        waypoints: waypoints.clone(), saved_waypoints: waypoints,
        open_lap: Some(1), closed_lap: None, timed_events: [TimedEvent::default(); 8],
    }
}

pub(super) fn store(start: u32) -> Store {
    Store {
        runs: [FIRST_RUN, LAP_RUN].map(|id| RaceRun { id, finish: NO_RUN, start }).to_vec(),
        chunks: Vec::new(), ids: vec![IdSequence {
            owner: 0, first: 0, second: 0x1_0000, first_limit: u16::MAX,
            second_limit: u16::MAX, used: 2,
        }],
    }
}
