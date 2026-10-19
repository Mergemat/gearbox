use super::{CollisionWorld, Geometry};

const REBALANCE_THRESHOLD: u32 = 200;

impl CollisionWorld {
    pub fn update_continuous(&mut self, items: &mut [super::DynamicItem]) {
        self.continuous_entries = 0..0;
        for &id in &self.continuous_items {
            let item = &mut items[id.index()];
            item.entry_count = 0;
            if item.layer == 0 {
                item.entry_start = u16::MAX;
            } else {
                todo!("continuous contacts for layered object");
            }
        }
    }

    pub fn rebuild_entries(&self, geometry: Geometry<'_>) -> (Vec<super::ShapeEntry>, Vec<(u32, u16, u16)>) {
        let mut out = Vec::new();
        let mut ranges = Vec::new();
        let mut first = 0u16;
        for &id in &self.discrete_items {
            let item = &geometry.state.dynamic_item[id.index()];
            if item.layer == 0 {
                continue;
            }
            let entries = geometry.entries(item.shape, item.iso, Some(id.into()), u32::MAX as u64);
            let count = entries.len() as u16;
            ranges.push((id.0, first, count));
            first = first.wrapping_add(count);
            out.extend(entries);
        }
        (out, ranges)
    }
}

impl CollisionWorld {
    pub fn dynamic_update(&mut self) {
        if self.dynamic_items.len() > REBALANCE_THRESHOLD as usize && self.dynamic_dirty == 0 {
            todo!("collision world rebalance");
        }
        if self.dynamic_dirty != 0 {
            todo!("collision world rebuild");
        }
        if !self.dynamic_items.is_empty() {
            todo!("dynamic contact rebuild");
        }
        self.updated_surfaces = 0..0;
        self.updated_tree_nodes.clear();
    }

    pub fn world_update(&mut self, items: &mut [super::DynamicItem]) {
        if self.static_dirty != 0 {
            todo!("collision world pending update");
        }
        self.dynamic_update();
        for &id in &self.discrete_items {
            let item = &mut items[id.index()];
            item.pos = item.iso.pos;
            item.quat = crate::Quat::from_mat3(&item.iso.rot);
            item.cached_body = u32::MAX;
        }
        if !self.continuous_items.is_empty() {
            todo!("collision world kinematic objects");
        }
    }
}
