mod contacts;
pub mod islands;
pub mod pruning;

use super::constants::{NULL_INDEX, SOLVER_EPSILON};
use super::{Body, BodyOptions, Collision, Dyna, SolverFlags};
use crate::{Iso, ItemId, Vec3};
pub use contacts::{
    ContactRow, ImpulseContact, clip_sum, closest_time_ms, contact_inverse_mass, solve_contacts,
};

pub const MAX_SUBSTEPS: usize = 1000;
const MAX_WORKERS: usize = 64;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionRecord {
    pub mask: u32,
    pub layer_bit: u32,
    pub item_id: u32,
    pub body_slot: u32,
    pub class: u8,
    pub flags: u8,
    pub boxes: Vec<crate::Aabb>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionContact {
    pub a: Option<ItemId>,
    pub b: Option<ItemId>,
    pub point: Vec3,
    pub normal: Vec3,
    pub depth: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionEntry {
    pub contacts: Vec<CollisionContact>,
    pub indices: Vec<u32>,
    pub pairs: Vec<(u32, u32)>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolverList {
    pub rows: Vec<ContactRow>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolverEntry {
    pub lists: [SolverList; 20],
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StepBuckets {
    pub saved_states: Vec<Iso>,
    pub saved_items: Vec<Iso>,
    pub island_items: Vec<islands::IslandItem>,
}

impl StepBuckets {
    pub fn reset(&mut self) {
        self.saved_states.clear();
        self.saved_items.clear();
        self.island_items.clear();
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolverWork {
    pub contact_pool: crate::collision::ContactList,
    pub collision_entries: Vec<CollisionEntry>,
    pub static_entries: Vec<CollisionEntry>,
    pub narrow_pairs: Vec<(u32, u32)>,
    pub broad_pairs: Vec<(u32, u32)>,
    pub group_offsets: Vec<(u16, u16)>,
    pub group_sizes: Vec<u8>,
    pub solver_entries: Vec<SolverEntry>,
    pub radix: pruning::RadixSorter,
    pub buckets: StepBuckets,
}

impl SolverWork {
    pub fn reset_collision(&mut self, workers: usize) {
        assert!(workers <= MAX_WORKERS, "too many solver workers (ud2)");
        self.narrow_pairs.clear();
        self.broad_pairs.clear();
        self.collision_entries.resize_with(workers, CollisionEntry::default);
        self.static_entries.resize_with(workers, CollisionEntry::default);
        for entries in [&mut self.collision_entries, &mut self.static_entries] {
            for entry in entries.iter_mut() {
                entry.contacts.clear();
                entry.indices.clear();
                entry.pairs.clear();
            }
        }
    }

    pub fn reset_solver(&mut self, workers: usize) {
        assert!(workers <= MAX_WORKERS, "too many solver workers (ud2)");
        self.solver_entries.resize_with(workers, SolverEntry::default);
        for entry in &mut self.solver_entries {
            for list in &mut entry.lists {
                list.rows.clear();
            }
        }
    }

    pub fn update_group_offsets(&mut self) {
        self.group_offsets.resize(self.group_sizes.len(), (0, 0));
        let mut offset = 0u16;
        for (size, pair) in self.group_sizes.iter().zip(&mut self.group_offsets) {
            *pair = (offset, 0);
            offset = offset.wrapping_add(*size as u16);
        }
    }
}

pub fn body_flags(dyna: &mut Dyna, collision: &impl Collision) {
    for body in &mut dyna.bodies {
        let flags = body.flags;
        let mut step = body.record.flags;
        let external = body.model.is_externally_simulated() && !flags.contains(BodyOptions::KINEMATIC);
        step.set(SolverFlags::EXTERNAL, external);
        body.record.parent_slot = if body.record.parent == NULL_INDEX {
            NULL_INDEX
        } else {
            dyna.handles.slot(super::BodyHandle(body.record.parent))
        };
        let fixed = flags.contains(BodyOptions::NO_STATIC_COLLISION)
            || flags.contains(BodyOptions::FIXED)
            || external;
        step.set(SolverFlags::SKIP_INTEGRATION, fixed);
        let water = body.item.is_some_and(|item| collision.is_water(item));
        step.set(SolverFlags::SKIP_FORCES, fixed);
        let can_collide = !fixed || !flags.contains(BodyOptions::FIXED) || water;
        let skip_static = !can_collide
            || external
            || flags.contains(BodyOptions::NO_STATIC_COLLISION)
            || body.item.is_none()
            || body.index.index == u16::MAX;
        step.set(SolverFlags::SKIP_STATIC_COLLISION, skip_static);
        step.set(SolverFlags::SPECIAL_STEP, false);
        body.inertia.inverse_mass = 1.0 / (body.model.mass + body.record.added_mass);
        if flags.contains(BodyOptions::FIXED) && !water {
            body.inertia.inverse_mass = 0.0;
        }
        let special = flags.contains(BodyOptions::SPECIAL);
        step.set(SolverFlags::SPECIAL, special);
        if special {
            step.set(SolverFlags::SKIP_FORCES, true);
            step.set(SolverFlags::SKIP_STATIC_COLLISION, true);
            step.set(SolverFlags::SPECIAL_STEP, true);
            step.set(SolverFlags::SKIP_INTEGRATION, true);
            body.inertia.inverse_mass = 0.0;
        }
        body.record.flags = step;
    }
}

pub fn internal_step(dyna: &mut Dyna, now: u32, cache: &[CollisionRecord], collision: &mut impl Collision) {
    body_flags(dyna, collision);
    save_fixed_poses(dyna, collision);
    place_objects(dyna, collision);
    center_bodies(dyna);
    broad_phase(dyna, cache);
    detect(dyna, cache, collision);
    islands_and_sleep(dyna, now);
    uncenter_bodies(dyna);
    restore_fixed_poses(dyna, collision);
}

fn save_fixed_poses(dyna: &mut Dyna, collision: &impl Collision) {
    for (i, body) in dyna.bodies.iter().enumerate() {
        if body.record.parent_slot != NULL_INDEX && body.record.parent_slot > i as u32 {
            todo!("body joints");
        }
        if body.record.flags.contains(SolverFlags::EXTERNAL) {
            dyna.work.buckets.saved_states.push(body.state.pose());
            if let Some(item) = body.item {
                dyna.work.buckets.saved_items.push(collision.placement(item));
            }
        }
    }
}

fn place_objects(dyna: &Dyna, collision: &mut impl Collision) {
    for owners in [false, true] {
        for body in &dyna.bodies {
            let item = if owners { body.owner_item } else { body.item };
            if let Some(item) = item {
                collision.set_placement(item, body.state.pose());
            }
        }
    }
}

fn center_bodies(dyna: &mut Dyna) {
    for body in &dyna.bodies {
        if !body.record.flags.contains(SolverFlags::SKIP_FORCES) {
            todo!("generic forces/integration");
        }
    }
    for body in &mut dyna.bodies {
        body.place_at_center(false);
    }
    dyna.centered = true;
}

fn broad_phase(dyna: &mut Dyna, cache: &[CollisionRecord]) {
    let count = dyna.bodies.len();
    dyna.work.buckets.island_items =
        (0..count as u32).map(|i| islands::IslandItem { parent: i, link: 0 }).collect();
    dyna.work.reset_collision(1);
    for body in &dyna.bodies {
        if !body.record.flags.contains(SolverFlags::SKIP_STATIC_COLLISION) {
            todo!("static collision item");
        }
    }
    let skip_flagged = dyna.dynamic_registered;
    let mut boxes = Vec::new();
    let mut indices = Vec::new();
    for (i, record) in cache.iter().enumerate() {
        if skip_flagged && record.flags & 2 != 0 {
            continue;
        }
        let mut bounds = crate::Aabb::EMPTY;
        for b in &record.boxes {
            pruning::union_box(&mut bounds, *b);
        }
        boxes.push(pruning::Bounds::from_aabb(bounds));
        indices.push(i as u32);
    }
    dyna.work.broad_pairs = pruning::box_pruning(&mut dyna.work.radix, &boxes);
    dyna.work.narrow_pairs =
        dyna.work.broad_pairs.iter().map(|&(a, b)| (indices[a as usize], indices[b as usize])).collect();
}

fn detect(dyna: &mut Dyna, cache: &[CollisionRecord], collision: &mut impl Collision) {
    collision.detect(cache, dyna.dynamic_registered, &mut dyna.work);
    let has_contacts = dyna
        .work
        .collision_entries
        .iter()
        .chain(&dyna.work.static_entries)
        .any(|entry| !entry.contacts.is_empty() || !entry.pairs.is_empty());
    if has_contacts {
        todo!("per-thread contacts");
    }
}

fn islands_and_sleep(dyna: &mut Dyna, now: u32) {
    rand_step(&mut dyna.rng);
    dyna.work.update_group_offsets();
    let islands = islands::build_item_islands(&mut dyna.work.buckets.island_items, &dyna.active);
    dyna.work.reset_solver(1);
    for body in &dyna.bodies {
        if !body.record.flags.contains(SolverFlags::SKIP_INTEGRATION) {
            todo!("integrate a body");
        }
    }
    let linear = SLEEP_SPEED * SLEEP_SPEED;
    let angular = SLEEP_SPEED * SLEEP_SPEED;
    for members in islands.members() {
        if islands::can_sleep(&mut dyna.bodies, members, now, linear, angular, SLEEP_FRAMES) {
            islands::sleep(&mut dyna.bodies, &mut dyna.active, members);
        }
    }
}

fn uncenter_bodies(dyna: &mut Dyna) {
    for body in &mut dyna.bodies {
        body.place_at_center(true);
    }
    dyna.centered = false;
}

fn restore_fixed_poses(dyna: &mut Dyna, collision: &mut impl Collision) {
    let (mut state_index, mut item_index) = (0, 0);
    for body in &mut dyna.bodies {
        if !body.record.flags.contains(SolverFlags::EXTERNAL) {
            continue;
        }
        let pose = dyna.work.buckets.saved_states[state_index];
        body.state.rot = pose.rot;
        body.state.pos = pose.pos;
        if let Some(item) = body.item {
            collision.restore_placement(item, dyna.work.buckets.saved_items[item_index]);
            item_index += 1;
        }
        state_index += 1;
    }
}

impl Dyna {
    pub fn contact_impulses(
        &mut self,
        handle: super::BodyHandle,
        contacts: &[ImpulseContact],
        weights: &[f32],
        mass: f32,
        inverse: Vec3,
    ) -> f32 {
        if contacts.is_empty() {
            return 0.0;
        }
        assert!(weights.len() >= contacts.len());
        let slot = self.handles.slot(handle) as usize;
        let saved = self.bodies[slot].inertia.clone();
        self.bodies[slot].place_at_center(false);
        let mut max = weights[0];
        for &w in &weights[1..] {
            max = crate::math::maxss(w, max);
        }
        let scale = 1.0 / (max * 50.0 + 1.0);
        self.bodies[slot].inertia = super::BodyInertia {
            diagonal: Vec3::new(scale * inverse.x, scale * inverse.y, scale * inverse.z),
            inverse_mass: mass,
        };
        let mut rows = Vec::with_capacity(contacts.len());
        for (contact, weight) in contacts.iter().zip(weights) {
            let state = &self.bodies[slot].state;
            let velocity = state.point_velocity(contact.point);
            let other = contact.other.map_or(Vec3::ZERO, |h| self.body(h).velocity_at(contact.point));
            let r = velocity - other;
            let n = contact.normal;
            let vn = r.dot_yxz(n);
            if vn > 0.0 {
                continue;
            }
            let on = other.dot_yxz(n);
            rows.push(ContactRow {
                body: slot as u32,
                point: contact.point,
                normal: n,
                bias: vn * weight - on,
                mass_factor: 1.0 / contact_inverse_mass(&self.bodies[slot].inertia, state, contact.point, n),
                impulse: 0.0,
            });
        }
        let mut residual = 0.0;
        for _ in 0..4 {
            residual = solve_contacts(&mut self.bodies, &mut rows);
            if SOLVER_EPSILON > residual.abs() {
                break;
            }
        }
        self.bodies[slot].place_at_center(true);
        self.bodies[slot].inertia = saved;
        residual
    }
}

pub fn rand_step(seed: &mut u32) -> u32 {
    const MULTIPLIER: u32 = 0x3e39_b193;
    let first = 0x3039u32.wrapping_sub(seed.wrapping_mul(MULTIPLIER)) & 0x7fff_ffff;
    let low = (first >> 12) & 0xfff0;
    let second = 0x3039u32.wrapping_sub(first.wrapping_mul(MULTIPLIER));
    let mid = ((second & 0x0fff_0000) | low) >> 4;
    *seed = 0x3039u32.wrapping_sub(second.wrapping_mul(MULTIPLIER)) & 0x7fff_ffff;
    (((*seed >> 16) & 0xff) << 24) | mid
}

const SLEEP_SPEED: f32 = 0.05;
const SLEEP_FRAMES: u32 = 1000;
