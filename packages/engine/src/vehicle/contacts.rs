use crate::race::countdown::ControlFlags;
use crate::{ItemId, Vec3, collision::{Contact, ContactList, Geometry},
    dyna::{BodyHandle, solver::ImpulseContact}};
use super::{forces::ForceCtx, impacts::ImpactModel, contact_response};

pub const CONTACT_CAPACITY: usize = 500;
const HARD_CONTACT_SQUARED: f32 = 0.010000001;
const NS_PER_MS: i64 = 1_000_000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ContactFlags(pub u32);
impl ContactFlags {
    pub const HARD: Self = Self(512);
    pub const PIPE: Self = Self(1024);
    pub const OPPOSING: Self = Self(2048);
    pub const SOLVED: Self = Self(4096);
    pub fn contains(self, flag: Self) -> bool { self.0 & flag.0 == flag.0 }
    pub fn set(&mut self, flag: Self, enabled: bool) {
        if enabled { self.0 |= flag.0; } else { self.0 &= !flag.0; }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct TriggerContact { pub item: ItemId, pub time: u32, pub contact: Contact }

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Output {
    pub pushes: Vec<Vec3>,
    pub triggers: Vec<TriggerContact>,
}

pub fn vehicle_contacts(ctx: &mut ForceCtx<'_>, model: &ImpactModel, list: &mut ContactList) -> Output {
    let geometry = ctx.collision.geometry;
    list.sort(geometry);
    let initial_yaw = if ctx.tuning.impact_yaw_enabled != 0 { ctx.body.local_angular_velocity().y }
        else { 0.0 };
    let mut out = Output::default();
    let mut contacts = Vec::new();
    let mut weights = Vec::new();
    let (mut solve, mut volume_contact) = (false, false);
    for source in list.as_mut_slice() {
        let other = geometry.entry(source.other_entry()).item.expect("entry without item");
        let item_b = geometry.item(other);
        hard_contact(ctx.vehicle, source.penetration, item_b.layer);
        if let ItemId::Dynamic(id) = other {
            assert!(!geometry.state.dynamic_item[id.index()].has_contact_callback,
                "contact callback of touched item");
        }
        match item_b.layer {
            1 if receives_triggers(geometry, other) => {
                let time = trigger_time(ctx, other, source);
                source.event_time = time;
                out.triggers.push(TriggerContact { item: other, time, contact: source.clone() });
            }
            2 | 16 | 18 | 21 => {
                if ctx.tuning.model_kind > 5 {
                    todo!("vehicle contact absorption");
                }
                let result = contact_response::absorb_contact(ctx, model, source);
                volume_contact |= result.volume_contact;
                solve |= result.solve;
                assert!(out.pushes.len() < CONTACT_CAPACITY, "push-out list overflow (ud2)");
                out.pushes.push(result.push);
                if source.material_b as u8 != 13 {
                    contacts.push(ImpulseContact { point: source.point, normal: source.normal,
                        other: item_b.dynamic.then_some(BodyHandle(item_b.body)) });
                    weights.push(result.weight);
                }
            }
            22 => gate_contact(ctx.vehicle, geometry, other),
            _ => {}
        }
    }
    let mut flags = ContactFlags(ctx.vehicle.flags);
    flags.set(ContactFlags::SOLVED, false);
    ctx.vehicle.contact_solve_residual = 0.0;
    if solve {
        let inertia = ctx.geometry.local_inverse_inertia;
        let inverse = Vec3::new(inertia.rows[0].x, inertia.rows[1].y, inertia.rows[2].z);
        let residual = ctx.body.dyna.contact_impulses(ctx.body.handle, &contacts, &weights,
            1.0 / ctx.geometry.mass, inverse);
        flags.set(ContactFlags::SOLVED, true);
        ctx.vehicle.contact_solve_residual = residual.abs();
    }
    ctx.vehicle.flags = flags.0;
    let mut controls = ControlFlags(ctx.vehicle.control_flags);
    controls.set(ControlFlags::VOLUME_CONTACT, volume_contact);
    ctx.vehicle.control_flags = controls.0;
    spin_bookkeeping(ctx, initial_yaw, volume_contact);
    out
}

fn hard_contact(vehicle: &mut super::Vehicle, displacement: Vec3, other_layer: u8) {
    let squared = displacement.x * displacement.x + displacement.y * displacement.y
        + displacement.z * displacement.z;
    if !(squared > HARD_CONTACT_SQUARED) { return; }
    let mut flags = ContactFlags(vehicle.flags);
    if !flags.contains(ContactFlags::HARD) {
        flags.set(ContactFlags::HARD, true);
        vehicle.contact_normal = displacement;
        flags.set(ContactFlags::PIPE, other_layer == 21);
    } else {
        let normal = vehicle.contact_normal;
        let dot = displacement.y * normal.y + displacement.x * normal.x + displacement.z * normal.z;
        if 0.0 > dot && (other_layer == 21 || flags.contains(ContactFlags::PIPE)) {
            flags.set(ContactFlags::OPPOSING, true);
        }
    }
    vehicle.flags = flags.0;
}

fn receives_triggers(geometry: Geometry<'_>, item: ItemId) -> bool {
    match item {
        ItemId::Static(id) => {
            !matches!(geometry.world.statics.items[id.index()].entity, crate::static_world::Entity::None)
        }
        ItemId::Dynamic(id) => geometry.state.dynamic_item[id.index()].receives_triggers,
    }
}

fn trigger_time(ctx: &ForceCtx<'_>, item: ItemId, source: &Contact) -> u32 {
    let geometry = ctx.collision.geometry;
    let timed = match item {
        ItemId::Static(id) => geometry.world.statics.items[id.index()].flags,
        ItemId::Dynamic(id) => geometry.state.dynamic_item[id.index()].event_flags,
    };
    if timed.contains(crate::collision::ItemEvents::CONTACT_TIME) {
        ctx.body.dyna.contact_time_ms(ctx.body.handle, ctx.time_ns, ctx.dt,
            source.point, source.penetration, geometry.item(item).iso) as u32
    } else { ctx.tick }
}

fn gate_contact(vehicle: &mut super::Vehicle, geometry: Geometry<'_>, item: ItemId) {
    let ItemId::Static(id) = item else { return; };
    let crate::static_world::Entity::Gate(index) = geometry.world.statics.items[id.index()].entity
        else { return; };
    let record = &geometry.world.gate_record[index as usize];
    vehicle.surface_contact_effect = record.effect;
    vehicle.surface_contact_item = Some(item);
    vehicle.surface_contact_normal = record.dir;
    vehicle.surface_contact_gate = record.id;
}

fn spin_bookkeeping(ctx: &mut ForceCtx<'_>, initial_yaw: f32, volume_contact: bool) {
    if ctx.tuning.impact_yaw_enabled != 0 && volume_contact {
        if ctx.vehicle.impact_yaw_begin == u32::MAX { ctx.vehicle.impact_yaw_begin = ctx.tick; }
        let yaw = ctx.body.local_angular_velocity().y;
        ctx.vehicle.impact_yaw_change = (initial_yaw - yaw).abs() + ctx.vehicle.impact_yaw_change;
    }
    let start = ctx.vehicle.impact_yaw_begin;
    if start == u32::MAX || ctx.tick <= start.wrapping_add(400) { return; }
    let base = (ctx.vehicle.impact_thrust_until as i64).max(ctx.time_ns);
    let duration = crate::math::cvtt64(ctx.tuning.impact_yaw_duration.eval(ctx.vehicle.impact_yaw_change));
    let deadline = (duration as u32 as i64).wrapping_mul(NS_PER_MS).wrapping_add(base);
    let limit = (ctx.tuning.impact_thrust_limit_ms as i64).wrapping_mul(NS_PER_MS)
        .wrapping_add(ctx.time_ns);
    ctx.vehicle.impact_thrust_until = deadline.min(limit) as u64;
    ctx.vehicle.impact_yaw_begin = u32::MAX;
    ctx.vehicle.impact_yaw_change = 0.0;
}
