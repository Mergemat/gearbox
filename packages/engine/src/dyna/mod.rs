mod bodies;
pub mod constants;
mod flags;
pub mod kinematic;
pub mod solver;
mod types;

pub use bodies::{Body, BodyHandle, BodyParams, ForceModels, Handles, grow_capacity};
pub use flags::{BodyOptions, SolverFlags};
pub use kinematic::KinematicModel;
pub use types::*;

use crate::{DynamicItemId, Iso, Vec3};
use constants::NULL_INDEX;
use solver::{CollisionRecord, SolverWork};

pub trait Collision {
    fn placement(&self, item: DynamicItemId) -> Iso;
    fn set_placement(&mut self, item: DynamicItemId, pose: Iso);
    fn restore_placement(&mut self, item: DynamicItemId, pose: Iso);
    fn set_body(&mut self, item: DynamicItemId, handle: BodyHandle);
    fn is_water(&self, item: DynamicItemId) -> bool;
    fn cache(&mut self, handles: &Handles, bodies: &[Body]) -> Vec<CollisionRecord>;
    fn detect(&mut self, records: &[CollisionRecord], skip_flagged: bool, work: &mut SolverWork);
}

#[derive(Clone, Copy, Debug)]
pub struct Tick {
    pub ms: u32,
    pub dt: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dyna {
    pub bodies: Vec<Body>,
    pub handles: Handles,
    pub active: Vec<u32>,
    pub animations: Vec<kinematic::Motion>,
    pub constraints: Vec<kinematic::KinematicConstraint>,
    pub gravity: Vec3,
    pub coefficient: f32,
    pub dynamic_registered: bool,
    pub race_origin: u32,
    pub step_count: u32,
    pub tick_ms: u32,
    pub rng: u32,
    pub centered: bool,
    pub pending_wakes: Vec<BodyHandle>,
    pub pending_removals: Vec<BodyHandle>,
    pub props_count: u32,
    pub work: SolverWork,
}

impl Dyna {
    pub fn kinematic_update(&mut self, now: i64, collision: &mut impl Collision) {
        let start = if self.race_origin == NULL_INDEX { 0 } else { self.race_origin as i64 * 1_000_000 };
        let t = if now > start { now - start } else { 10_000_000 };
        for motion in &mut self.animations {
            motion.current = motion.model.transform(t, motion.phase);
            motion.past = motion.model.transform(t - 1_000_000, motion.phase);
        }
        for row in &self.constraints {
            let motion = &self.animations[row.animation as usize];
            let current = row.placement * motion.current;
            let past = row.placement * motion.past;
            let body = &mut self.bodies[self.handles.slot(row.body) as usize];
            body.state = BodyState::kinematic(current, past, constants::SAMPLE_DT);
            if let Some(item) = body.item {
                collision.set_placement(item, body.state.pose());
            }
        }
    }

    pub fn update(&mut self, tick: Tick, collision: &mut impl Collision) {
        self.step_count = self.step_count.wrapping_add(1);
        self.work.buckets.reset();
        if !self.pending_wakes.is_empty() {
            todo!("pending wake list");
        }
        let free = self
            .bodies
            .iter()
            .filter(|b| b.model.is_externally_simulated() && b.owner_item.is_none())
            .count();
        self.tick_ms = tick.ms;
        let cache = collision.cache(&self.handles, &self.bodies);
        if free != self.bodies.len() {
            let now = (crate::math::cvtt64(tick.dt * 1000.0) as u32).wrapping_mul(self.step_count);
            solver::internal_step(self, now, &cache, collision);
        }
        for body in &mut self.bodies {
            body.record.added_mass = 0.0;
        }
        if !self.pending_removals.is_empty() {
            todo!("removing bodies");
        }
    }

    pub fn step(&mut self, tick: Tick, collision: &mut impl Collision) {
        self.kinematic_update(tick.ms as i64 * 1_000_000, collision);
        self.update(tick, collision);
        if self.props_count != 0 {
            todo!("attached props");
        }
    }
}
