use crate::statics::items::ItemSpec;
use crate::statics::mapworld::Placed;
use crate::statics::packs::PackSource;
use anyhow::Result;

#[derive(Default)]
pub struct Built {
    pub items: Vec<ItemSpec>,
    pub gaps: Vec<String>,
}

impl Built {
    fn push(&mut self, mut items: Vec<ItemSpec>, waypoint: Option<u32>) {
        for item in &mut items {
            if matches!(item.entity, gearbox_engine::static_world::Entity::Waypoint(_)) {
                item.entity = gearbox_engine::static_world::Entity::Waypoint(waypoint);
            }
        }
        self.items.extend(items);
    }
}

pub fn static_items(map: &gearbox_gbx::Map, src: &mut PackSource<'_>) -> Result<Built> {
    let mut out = Built::default();
    let blocks: Vec<&gearbox_gbx::map::Block> = map.blocks.iter().chain(&map.baked_blocks).collect();
    let mut next_waypoint = 0u32;
    for (i, (block, placed)) in blocks.iter().zip(crate::statics::mapworld::blocks(map, src)?).enumerate() {
        let waypoint = (i < map.blocks.len() && block.waypoint.is_some()).then(|| {
            let id = next_waypoint;
            next_waypoint += 1;
            id
        });
        match placed {
            Placed::Items(items) => out.push(items, waypoint),
            Placed::Hidden => {}
            Placed::UnknownModel => {
                let why = src.problems.get(&block.name).cloned().unwrap_or_default();
                out.gaps.push(format!("block {} at {:?}: unknown model ({why})", block.name, block.coord));
            }
        }
    }
    for (item, result) in map.items.iter().zip(crate::statics::anchored::items(map, src)) {
        let waypoint = item.waypoint.is_some().then(|| {
            let id = next_waypoint;
            next_waypoint += 1;
            id
        });
        match result {
            Ok(items) => out.push(items, waypoint),
            Err(e) => out.gaps.push(format!("item {:?}: {e}", item.model)),
        }
    }
    match crate::statics::decoration::items(map, src) {
        Ok(items) => out.push(items.into_iter().flatten().collect(), None),
        Err(e) => out.gaps.push(format!("decoration: {e}")),
    }
    Ok(out)
}
