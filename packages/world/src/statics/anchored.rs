use super::packs::{LAYER_STATIC, LAYER_TRIGGER};
use gearbox_engine::collision::ItemEvents;
use gearbox_engine::static_world::Entity;
use gearbox_packs::assets::Role;

use crate::statics::items::ItemSpec;
use crate::statics::packs::PackSource;
use crate::statics::place::{self, Iso};

pub use super::spawn::item_location;
pub use gearbox_engine::static_world::LAYER_GATE;

pub fn item_iso(item: &gearbox_gbx::map::Item) -> Iso {
    let (r, p) = item_location(item);
    Iso { q: place::rotation_quat(r), p }
}

pub fn items(map: &gearbox_gbx::Map, src: &mut PackSource<'_>) -> Vec<Result<Vec<ItemSpec>, String>> {
    map.items
        .iter()
        .map(|item| {
            let iso = item_iso(item);
            let location = item_location(item);
            let parts = src.item_parts(item, iso, location)?;
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
