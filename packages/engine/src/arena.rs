#[derive(Clone, Debug, Default, PartialEq)]
pub struct Scene {
    pub updates_collision: bool,
    pub time: u32,
    pub callbacks_len: u32,
    pub pending_events: u32,
}

pub fn begin_player(vehicle: Option<&mut crate::vehicle::Vehicle>) {
    if let Some(vehicle) = vehicle {
        vehicle.flags |= crate::vehicle::race_flags::RaceFlags::PLAYER_STEP.0;
    }
}

pub fn vehicle_flags(vehicles: &mut [crate::vehicle::Vehicle]) {
    for vehicle in vehicles {
        vehicle.flags = crate::vehicle::race_flags::RaceFlags(vehicle.flags).begin().0;
    }
}

pub fn player_vehicle(racer: &mut crate::race::Racer, vehicle: Option<&mut crate::vehicle::Vehicle>) {
    use crate::vehicle::race_flags::RaceFlags;
    let Some(vehicle) = vehicle else { return; };
    if racer.has_entity(vehicle.key) { vehicle.tag = racer.player.tag; }
    let mut flags = RaceFlags(vehicle.flags);
    flags.set(RaceFlags::PASSIVE, false);
    flags.set(RaceFlags::GHOST, racer.player.ghost_active != 0 && racer.player.ghost_start != u32::MAX
        && racer.player.ghost_start == racer.player.race_start);
    vehicle.flags = flags.0;
}

pub fn mute_seats(racer: &crate::race::Racer, vehicles: &mut [crate::vehicle::Vehicle]) {
    use crate::vehicle::race_flags::RaceFlags;
    let Some(controller) = &racer.controller else { return; };
    let Some(&driven) = crate::race::SEAT_SLOTS.get(controller.seat as usize) else { return; };
    for (index, &handle) in racer.player.seats.iter().enumerate() {
        if handle == crate::race::NO_ENTITY { continue; }
        if let Some(vehicle) = vehicles.iter_mut().find(|vehicle| vehicle.key == handle) {
            let mut flags = RaceFlags(vehicle.flags);
            flags.set(RaceFlags::MUTED, index != driven as usize);
            vehicle.flags = flags.0;
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GhostTarget {
    pub override_window: Option<crate::race::countdown::Window>,
    pub explicit_start: Option<u32>,
    pub followed_window: Option<crate::race::countdown::Window>,
}
impl GhostTarget {
    pub fn origin(self, rules: &crate::race::RaceRules) -> u32 {
        if self.override_window.is_none() {
            if let Some(start) = self.explicit_start { return start; }
        }
        if (rules.mode_bits as i8) < 0
            || ArenaOptions(rules.flags).contains(ArenaOptions::FORCE_RACE_ORIGIN) {
            return if rules.start == u32::MAX { 0 } else { rules.start };
        }
        let start = self.override_window.or(self.followed_window)
            .map_or(u32::MAX, |window| window.start(rules.start));
        if start == u32::MAX { 0 } else { start }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ArenaOptions(pub u32);
impl ArenaOptions {
    pub const SCRIPTED_PLAYERS: Self = Self(0x400);
    pub const FORCE_RACE_ORIGIN: Self = Self(0x800);
    pub fn contains(self, flags: Self) -> bool { self.0 & flags.0 == flags.0 }
}

pub fn player_records(race: &crate::race::Race, vehicles: &[crate::vehicle::Vehicle])
    -> Vec<Option<crate::ids::VehicleId>> {
    race.players.iter().map(|racer| {
        let handle = racer.controlled_entity();
        if handle == crate::race::NO_ENTITY { return None; }
        vehicles.iter().position(|vehicle| vehicle.key == handle)
            .map(|index| crate::ids::VehicleId::new(index as u32))
    }).collect()
}

pub fn begin(race: &mut crate::race::Race, vehicles: &mut [crate::vehicle::Vehicle], options: ArenaOptions) {
    if options.contains(ArenaOptions::SCRIPTED_PLAYERS) { todo!("scripted players"); }
    let records = player_records(race, vehicles);
    for (racer, vehicle) in race.players.iter_mut().zip(records) {
        if racer.player.keyed != 0 { todo!("player vehicle keys"); }
        begin_player(vehicle.map(|id| &mut vehicles[id.index()]));
    }
    vehicle_flags(vehicles);
}

pub fn player_timed_events(racer: &crate::race::Racer, now: u32) {
    let player = &racer.player;
    for (event, handled) in racer.timed_events.iter().zip(player.handled_events) {
        if event.time <= now && event.time != handled && event.enabled {
            todo!("player timed event");
        }
    }
}

pub fn players_vehicles(race: &mut crate::race::Race, vehicles: &mut [crate::vehicle::Vehicle], now: u32) {
    let records = player_records(race, vehicles);
    for (racer, vehicle) in race.players.iter_mut().zip(records) {
        if vehicle.is_some() { player_timed_events(racer, now); }
        player_vehicle(racer, vehicle.map(|id| &mut vehicles[id.index()]));
    }
}

impl Scene {
    pub fn update(&mut self, now: u32, state: &mut crate::State, manager: usize) {
        if self.pending_events != 0 { todo!("scene event dispatch"); }
        self.pending_events = 0;
        self.time = now;
        if self.updates_collision {
            state.collision_world[manager].world_update(&mut state.dynamic_item);
        }
        if self.callbacks_len != 0 { todo!("scene step callbacks"); }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayerState { Inactive, Active, Other(u32) }
impl LayerState {
    pub fn from_code(code: u32) -> Self {
        match code { 0 => Self::Inactive, 2 => Self::Active, other => Self::Other(other) }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehicleEventKind { SpinUp, Bumper }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VehicleEvent {
    pub kind: VehicleEventKind,
    pub time: u32,
    pub tag: u8,
    pub vehicle: u32,
}

pub fn vehicle_event(vehicle: &crate::vehicle::Vehicle, time: u32,
    kind: VehicleEventKind) -> Option<VehicleEvent> {
    use crate::vehicle::race_flags::RaceFlags;
    if vehicle.tag == u8::MAX || RaceFlags(vehicle.flags).contains(RaceFlags::MUTED) {
        return None;
    }
    Some(VehicleEvent { kind, time, tag: vehicle.tag, vehicle: vehicle.key })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StateExitEvent { pub time: u32, pub tag: u8, pub vehicle: u32, pub layer: u8 }

pub fn state_exit_events(vehicle: &crate::vehicle::Vehicle, now: u32, enabled: bool,
    current: [LayerState; 5], previous: [LayerState; 5]) -> Vec<StateExitEvent> {
    use crate::vehicle::race_flags::RaceFlags;
    if !enabled || vehicle.tag == u8::MAX || RaceFlags(vehicle.flags).contains(RaceFlags::MUTED) {
        return Vec::new();
    }
    previous.into_iter().zip(current).enumerate().filter_map(|(layer, (previous, current))| {
        (previous == LayerState::Active && current != LayerState::Active)
            .then_some(StateExitEvent {
                time: now, tag: vehicle.tag, vehicle: vehicle.key, layer: layer as u8,
            })
    }).collect()
}

pub fn car_phy_update(vehicles: &mut [crate::vehicle::Vehicle],
    grid: &crate::collision::ByteGrid) {
    use crate::vehicle::race_flags::RaceFlags;
    for vehicle in vehicles {
        if !RaceFlags(vehicle.flags).contains(RaceFlags::ACTIVE) { continue; }
        if grid.sample(vehicle.pos) != 0 { todo!("vehicle grid event"); }
    }
}

pub fn clear_vehicle_inputs(vehicles: &mut [crate::vehicle::Vehicle]) {
    for vehicle in vehicles { crate::vehicle::spawn::clear_current_inputs(vehicle); }
}

pub fn consume_surface_effects(race: &crate::race::Race, vehicles: &mut [crate::vehicle::Vehicle]) {
    for racer in &race.players {
        if racer.player.keyed != 0 { todo!("surface effects for keyed players"); }
        let handle = racer.controlled_entity();
        if handle == crate::race::NO_ENTITY { continue; }
        if let Some(vehicle) = vehicles.iter_mut().find(|vehicle| vehicle.key == handle) {
            vehicle.surface_effect = 0;
        }
    }
}
