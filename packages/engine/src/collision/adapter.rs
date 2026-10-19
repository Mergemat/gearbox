use super::{
    Geometry,
    cache::{self, BodyStatus, BodyStatusProvider},
};
use crate::dyna::constants::NULL_INDEX;
use crate::dyna::solver::{CollisionRecord, SolverWork};
use crate::dyna::{self, Body, BodyHandle, Handles, SolverFlags};
use crate::{DynamicItemId, Iso, State, World};

pub struct DynaCollision<'a> {
    pub world: &'a World,
    pub state: &'a mut State,
    pub manager: usize,
}

struct BodyStatuses<'a>(&'a [Body]);
impl BodyStatusProvider for BodyStatuses<'_> {
    fn status(&self, slot: u32) -> BodyStatus {
        let body = &self.0[slot as usize];
        BodyStatus {
            asleep: body.record.flags.contains(SolverFlags::SPECIAL_STEP),
            detached: body.index.index == u16::MAX,
            special: body.record.flags.contains(SolverFlags::SPECIAL),
        }
    }
}

impl dyna::Collision for DynaCollision<'_> {
    fn placement(&self, item: DynamicItemId) -> Iso {
        self.state.dynamic_item[item.index()].iso
    }

    fn set_placement(&mut self, item: DynamicItemId, pose: Iso) {
        let item = &mut self.state.dynamic_item[item.index()];
        item.iso = pose;
        if item.entry_count == 0 {
            return;
        }
        let manager = &self.state.collision_world[self.manager];
        let first = manager.shape_entries.start;
        let start = first + item.entry_start as u32;
        let shape = item.shape;
        let mut entries = std::mem::take(&mut self.state.shape_entry);
        Geometry::new(self.world, self.state).place_shapes(&mut entries, start, shape, pose);
        self.state.shape_entry = entries;
    }

    fn restore_placement(&mut self, item: DynamicItemId, pose: Iso) {
        self.state.dynamic_item[item.index()].iso = pose;
    }

    fn set_body(&mut self, item: DynamicItemId, handle: BodyHandle) {
        self.state.dynamic_item[item.index()].body = handle.0;
    }

    fn is_water(&self, item: DynamicItemId) -> bool {
        self.state.dynamic_item[item.index()].layer == 7
    }

    fn cache(&mut self, handles: &Handles, bodies: &[Body]) -> Vec<CollisionRecord> {
        let manager = &self.state.collision_world[self.manager];
        let order = manager.discrete_items.clone();
        for &id in &order {
            let item = &mut self.state.dynamic_item[id.index()];
            item.cached_body =
                if item.body == NULL_INDEX { NULL_INDEX } else { handles.slot(BodyHandle(item.body)) };
        }
        let geometry = Geometry::new(self.world, self.state);
        let manager = &self.state.collision_world[self.manager];
        cache::create_dyna_cache(
            geometry,
            &order,
            manager.shape_entries.clone(),
            &super::layers::MASKS,
            &BodyStatuses(bodies),
        )
        .into_iter()
        .map(|record| CollisionRecord {
            mask: record.layer_mask,
            layer_bit: record.layer_bit,
            item_id: record.flags.bits(),
            body_slot: record.body_slot,
            class: record.layer,
            flags: record.body_flags,
            boxes: geometry.placed_entries(record.entries.clone()).map(|(_, entry)| entry.bounds).collect(),
        })
        .collect()
    }

    fn detect(&mut self, records: &[CollisionRecord], _: bool, work: &mut SolverWork) {
        for &(a, b) in &work.narrow_pairs {
            let a = records[a as usize].class as usize;
            let b = records[b as usize].class as usize;
            if super::layers::PAIRS[a][b] & 8 != 0 {
                todo!("narrow phase pair");
            }
        }
        work.narrow_pairs.clear();
    }
}
