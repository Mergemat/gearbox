use super::packs::{LAYER_STATIC, LAYER_TRIGGER};
use gearbox_engine::collision::ItemEvents;
use gearbox_engine::static_world::Entity;
use gearbox_packs::assets::Role;

use crate::statics::items::ItemSpec;
use crate::statics::packs::PackSource;
use crate::statics::place::{self, Iso};

pub use gearbox_engine::static_world::LAYER_GATE;

pub fn item_iso(item: &gearbox_gbx::map::Item) -> Result<Iso, String> {
    let (r, p) = item_location(item)?;
    Ok(Iso { q: place::rotation_quat(r), p })
}

pub fn item_location(item: &gearbox_gbx::map::Item) -> Result<([f32; 9], [f32; 3]), String> {
    if item.scale != 1.0 {
        return Err(format!("item scale {}", item.scale));
    }
    let location = place::free_location_variant(item.position, item.yaw_pitch_roll, Default::default());
    Ok(place::compose_iso(([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], item.pivot), location))
}

pub fn items(map: &gearbox_gbx::Map, src: &mut PackSource<'_>) -> Vec<Result<Vec<ItemSpec>, String>> {
    map.items
        .iter()
        .map(|item| {
            let name = item.model.as_deref().ok_or("item without a model")?;
            let iso = item_iso(item)?;
            let location = item_location(item)?;
            let parts = src.item_parts(name, (item.flags >> 8) as usize, iso, location)?;
            Ok(parts
                .into_iter()
                .map(|(role, shape, transform)| {
                    let (entity, layer, flags) = match role {
                        Role::SpecialTrigger => (Entity::Gate(0), LAYER_GATE, 0),
                        Role::Waypoint(_) => {
                            (Entity::Waypoint(None), LAYER_TRIGGER, ItemEvents::CONTACT_TIME.bits())
                        }
                        _ => (Entity::None, LAYER_STATIC, 0),
                    };
                    ItemSpec { entity, shape, transform, layer, flags }
                })
                .collect())
        })
        .collect()
}
