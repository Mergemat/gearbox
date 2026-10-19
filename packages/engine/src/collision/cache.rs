use crate::DynamicItemId;
use crate::shape::Geometry;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BodyStatus { pub asleep: bool, pub detached: bool, pub special: bool }

pub trait BodyStatusProvider { fn status(&self, slot: u32) -> BodyStatus; }

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ItemCache {
    pub entries: std::ops::Range<u32>,
    pub layer_mask: u32,
    pub layer_bit: u32,
    pub flags: super::ItemFlags,
    pub body_slot: u32,
    pub layer: u8,
    pub body_flags: u8,
}

pub fn create_dyna_cache(
    geometry: Geometry<'_>, order: &[DynamicItemId], entries: std::ops::Range<u32>,
    layers: &[u32], bodies: &impl BodyStatusProvider,
) -> Vec<ItemCache> {
    order.iter().map(|&id| {
        let item = &geometry.state.dynamic_item[id.index()];
        let body_flags = if item.cached_body == u32::MAX { 0 } else {
            let status = bodies.status(item.cached_body);
            u8::from(status.asleep || status.detached) | if status.special { 2 } else { 0 }
        };
        let range = if item.entry_count == 0 { 0..0 } else {
            let first = entries.start + item.entry_start as u32;
            first..first + item.entry_count as u32
        };
        ItemCache {
            entries: range,
            layer_mask: layers[item.layer as usize] & !item.blocked_layers,
            layer_bit: if item.layer == 0 { 0 } else { 1u32 << ((item.layer - 1) & 31) },
            flags: item.flags, body_slot: item.cached_body, layer: item.layer, body_flags,
        }
    }).collect()
}
