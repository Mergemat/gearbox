use super::{NO_ENTITY, Race, WaypointFlags};
use crate::{
    ids::VehicleId,
    static_world::{Entity, StaticWorld},
    vehicle::{Vehicle, race_flags::RaceFlags},
};

pub const MAX_CONTACTS: usize = 256;
const MAX_EVENT_PLAYERS: usize = 100;
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Phase {
    #[default]
    Idle,
    Collecting,
    Flushing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Contact {
    pub item: u32,
    pub vehicle: VehicleId,
    pub time: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Contacts {
    pub phase: Phase,
    pub contacts: Vec<Contact>,
}
impl Contacts {
    pub fn begin(&mut self) {
        self.phase = Phase::Collecting;
        self.contacts.clear();
    }
    pub fn flush(&mut self, raw: &[Contact]) {
        self.phase = Phase::Flushing;
        if !self.contacts.is_empty() {
            return;
        }
        for &contact in raw {
            if let Some(previous) = self
                .contacts
                .iter_mut()
                .find(|previous| previous.item == contact.item && previous.vehicle == contact.vehicle)
            {
                previous.time = previous.time.min(contact.time);
            } else {
                assert!(self.contacts.len() < MAX_CONTACTS, "trigger contact list overflow");
                self.contacts.push(contact);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub vehicle: VehicleId,
    pub entity: Entity,
    pub now: u32,
    pub time: u32,
}

fn owner(race: &Race, handle: u32) -> Option<usize> {
    if handle == NO_ENTITY {
        return None;
    }
    race.players.iter().position(|racer| {
        [racer.player.override_entity, racer.player.seat_entity, racer.player.entity].contains(&handle)
            || racer.player.seats.contains(&handle)
    })
}

pub fn events(
    race: &mut Race,
    statics: &StaticWorld,
    vehicles: &[Vehicle],
    now: u32,
    contacts: &[Contact],
) -> Vec<Event> {
    let mut out = Vec::new();
    for contact in contacts {
        let entity = statics.items[contact.item as usize].entity;
        let trigger = match entity {
            Entity::Waypoint(Some(waypoint)) => waypoint,
            Entity::None | Entity::Gate(_) | Entity::Waypoint(None) => continue,
        };
        let vehicle = &vehicles[contact.vehicle.index()];
        let Some(index) = owner(race, vehicle.key) else {
            continue;
        };
        let Some(controller) = &mut race.players[index].controller else {
            continue;
        };
        if controller.trigger == trigger {
            continue;
        }
        controller.trigger = trigger;
        if !RaceFlags(vehicle.flags).contains(RaceFlags::GHOST) {
            out.push(Event { vehicle: contact.vehicle, entity, now, time: contact.time });
        }
    }
    if race.rules.clear_untouched && !race.players.is_empty() {
        todo!("clear untouched players' triggers");
    }
    out
}

pub fn dispatch(race: &mut Race, vehicles: &mut [Vehicle], event: Event) -> bool {
    let Entity::Waypoint(Some(waypoint)) = event.entity else {
        return false;
    };
    let handle = vehicles[event.vehicle.index()].key;
    let owner = owner(race, handle);
    let mut indices = Vec::new();
    for (index, racer) in race.players.iter().enumerate() {
        if owner == Some(index) || racer.controlled_entity() == handle || racer.player.seat_entity == handle {
            if indices.len() == MAX_EVENT_PLAYERS {
                break;
            }
            indices.push(index);
        } else if racer.player.seat_entity != NO_ENTITY {
            todo!("arena event players through a seat");
        }
    }
    for index in indices {
        let mark = &race.rules.waypoints[waypoint as usize];
        if mark.flags.contains(WaypointFlags::SPECIAL) {
            todo!("special waypoint flag");
        }
        if !mark.flags.counts() {
            continue;
        }
        let order = race.players[index].player.start_order;
        if mark.flags.contains(WaypointFlags::GOAL)
            && !mark.flags.contains(WaypointFlags::SPAWN)
            && order != u8::MAX
            && mark.order != order as u32
        {
            continue;
        }
        let controlled = race.players[index].controlled_entity();
        let vehicle = vehicles.iter_mut().find(|vehicle| vehicle.key == controlled);
        race.waypoint(index, vehicle, waypoint, event.time);
    }
    true
}

#[derive(Clone, Copy, Debug)]
pub struct CollisionStep {
    pub tick: u32,
    pub now_ns: i64,
    pub dt: f32,
    pub scale: f32,
}

pub fn detect(
    world: &mut crate::World,
    state: &mut crate::State,
    manager: usize,
    dyna: &crate::dyna::Dyna,
    vehicle: &Vehicle,
    id: VehicleId,
    step: CollisionStep,
) -> Vec<Contact> {
    use crate::{
        ItemId,
        collision::{DynaCollision, Geometry, ItemEvents, Lists, QueryFilter},
        dyna::{BodyHandle, Collision},
    };
    let Some(vehicle_item) = vehicle.item else {
        return Vec::new();
    };
    if vehicle.body == u32::MAX
        || (vehicle.control_flags & 0xf == 1 && (step.tick.wrapping_sub(vehicle.race_start) as i32) < -50)
    {
        return Vec::new();
    }
    let handle = BodyHandle(vehicle.body);
    let pose = dyna.state(handle).pose();
    let shape = state.dynamic_item[vehicle_item.index()].shape;
    if step.scale != 1.0 {
        crate::shape::scale::shape(world, state, shape, step.scale);
    }
    DynaCollision { world, state, manager }.set_placement(vehicle_item, pose);
    let item = &state.dynamic_item[vehicle_item.index()];
    let filter = QueryFilter {
        lists: Lists::SURFACES,
        layers: crate::collision::layers::MASKS[item.layer as usize],
        excluded_flags: item.flags,
        ignored: [Some(vehicle_item.into()), None],
        ..Default::default()
    };
    let mut contacts = state.collision_world[manager].contacts_for_item(
        Geometry::new(world, state),
        vehicle_item,
        filter,
        500,
    );
    if step.scale != 1.0 {
        let inverse = 1.0 / step.scale;
        crate::shape::scale::shape(world, state, shape, inverse);
        DynaCollision { world, state, manager }.set_placement(vehicle_item, pose);
        for contact in contacts.as_mut_slice() {
            contact.rescale(pose, inverse);
        }
    }
    let geometry = Geometry::new(world, state);
    contacts.sort(geometry);
    contacts
        .as_slice()
        .iter()
        .filter_map(|contact| {
            let a = geometry.entry(contact.source_entry());
            let b = geometry.entry(contact.other_entry());
            if a.item != Some(vehicle_item.into()) || a.tag != crate::static_world::NO_CHILD_TAG {
                return None;
            }
            let item = b.item.and_then(ItemId::static_item)?.0;
            let trigger = &world.statics.items[item as usize];
            if trigger.layer != crate::static_world::LAYER_TRIGGER {
                return None;
            }
            let time = if trigger.flags.contains(ItemEvents::CONTACT_TIME) {
                dyna.contact_time_ms(
                    handle,
                    step.now_ns,
                    step.dt,
                    contact.point,
                    contact.penetration,
                    trigger.iso,
                ) as u32
            } else {
                step.tick
            };
            Some(Contact { item, vehicle: id, time })
        })
        .collect()
}
