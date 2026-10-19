use crate::statics::{
    model::{BlockKey, Catalog},
    place,
};
use anyhow::{Context, Result, ensure};
use gearbox_packs::catalog::EntModel;
use gearbox_packs::{assets::Assets, packs::PackSet};

fn model_file(packs: &PackSet, path: &str) -> Result<Vec<u8>> {
    packs.read_path(path).with_context(|| format!("spawn model {path}"))
}

type Transform = ([f32; 9], [f32; 3]);

pub fn item_location(item: &gearbox_gbx::map::Item) -> Transform {
    let frame = place::free_location_variant(item.position, item.yaw_pitch_roll, Default::default());
    place::compose_iso(([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], item.pivot), frame)
}

fn prefab_spawns(
    assets: &Assets,
    files: &PackSet,
    path: &str,
    stack: &mut Vec<String>,
) -> Result<Vec<Transform>> {
    ensure!(stack.len() < 32 && !stack.iter().any(|p| p == path), "recursive spawn prefab {path}");
    stack.push(path.to_string());
    let prefab = assets.index.prefabs.get(path).with_context(|| format!("spawn prefab {path} missing"))?;
    let mut out = Vec::new();
    for ent in &prefab.ents {
        let [x, y, z, w] = ent.rotation;
        let frame = (quat_matrix([w, x, y, z]), ent.position);
        match &ent.model {
            EntModel::Prefab(child) => {
                for spawn in prefab_spawns(assets, files, child, stack)? {
                    out.push(place::compose_iso(spawn, frame));
                }
            }
            EntModel::Other { class_id: Some(0x0917_a000), .. } => {
                let pose = gearbox_packs::spawn::item_landmark(&model_file(files, path)?)?;
                out.push(place::compose_iso(pose_transform(pose), frame));
            }
            _ => {}
        }
    }
    stack.pop();
    Ok(out)
}

fn nadeo_item_spawn(assets: &Assets, files: &PackSet, name: &str) -> Result<[u32; 12]> {
    let entries: Vec<_> = assets
        .index
        .items
        .iter()
        .filter(|(path, _)| {
            path.starts_with("Stadium\\Items\\") && gearbox_packs::assets::block_name(path) == name
        })
        .collect();
    ensure!(entries.len() == 1, "ambiguous/missing spawn item {name}");
    let mut spawns = Vec::new();
    for model in &entries[0].1.models {
        if let EntModel::Prefab(path) = model {
            spawns.extend(prefab_spawns(assets, files, path, &mut Vec::new())?);
        }
    }
    let (r, p) = spawns.first().copied().unwrap_or(([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0; 3]));
    Ok(std::array::from_fn(|i| if i < 9 { r[i] } else { p[i - 9] }.to_bits()))
}

pub fn map_pose(
    map: &gearbox_gbx::Map,
    assets: &Assets,
    files: &PackSet,
    geometry: &Catalog,
) -> Result<[u32; 12]> {
    let start = |w: &Option<gearbox_gbx::map::Waypoint>| {
        w.as_ref().and_then(|w| w.tag.as_deref()).is_some_and(|tag| matches!(tag, "Spawn" | "StartFinish"))
    };
    let blocks: Vec<_> = map.blocks.iter().filter(|b| start(&b.waypoint)).collect();
    let items: Vec<_> = map.items.iter().filter(|i| start(&i.waypoint)).collect();
    ensure!(blocks.len() + items.len() == 1, "expected one Spawn or StartFinish waypoint");
    if let Some(b) = blocks.first() {
        block_pose(map, b, assets, files, geometry)
    } else {
        item_pose(map, items[0], assets, files)
    }
}

pub fn block_pose(
    map: &gearbox_gbx::Map,
    block: &gearbox_gbx::map::Block,
    assets: &Assets,
    files: &PackSet,
    geometry: &Catalog,
) -> Result<[u32; 12]> {
    let mut model = block.clone();
    let mut custom = None;
    if let Some(name) = block.name.strip_suffix("_CustomBlock") {
        let file = map.embedded.item_file(name)?;
        let (name, pose) = gearbox_packs::spawn::block_item_waypoint(
            &file.data,
            block.is_ground(),
            ((block.flags >> 21) & 0x7f) as usize,
        )?;
        model.name = name;
        custom = pose;
    }
    let paths = assets.block_paths(&model.name);
    let path = paths.first().with_context(|| format!("waypoint block {} absent from assets", model.name))?;
    let block_geometry =
        geometry.geometry(&BlockKey::of(&model)).context("waypoint block placement metadata absent")?;
    let local = match custom {
        Some(pose) => pose_transform(pose),
        None => {
            let local = gearbox_packs::spawn::block_spawn(
                &model_file(files, path)?,
                path,
                model.is_ground(),
                ((model.flags >> 21) & 0x7f) as usize,
            )?;
            let angles = local.yaw_pitch_roll;
            (quat_matrix(place::angles_quat(angles[0], angles[1], angles[2])), local.position)
        }
    };
    let frame = match block.free {
        Some((p, angles)) => (quat_matrix(place::angles_quat(angles[0], angles[1], angles[2])), p),
        None => {
            place::grid_location_cell(block_geometry.cell, block.coord, block.dir as u32, block_geometry.size)
        }
    };
    let frame = if place::variant_applies(block_geometry.offset) {
        place::compose_iso(place::variant_transform(block_geometry.offset), frame)
    } else {
        frame
    };
    pose_bits(place::compose_iso(local, frame))
}

pub fn item_pose(
    map: &gearbox_gbx::Map,
    item: &gearbox_gbx::map::Item,
    assets: &Assets,
    files: &PackSet,
) -> Result<[u32; 12]> {
    let name = item.model.as_deref().context("waypoint item has no model")?;
    let local = if item.author.as_deref() == Some("Nadeo") {
        nadeo_item_spawn(assets, files, name)?
    } else {
        gearbox_packs::spawn::item_landmark(&map.embedded.item_file(name)?.data)?
    };
    pose_bits(place::compose_iso(pose_transform(local), item_location(item)))
}

fn pose_transform(bits: [u32; 12]) -> Transform {
    let pose = bits.map(f32::from_bits);
    (std::array::from_fn(|i| pose[i]), std::array::from_fn(|i| pose[i + 9]))
}

fn pose_bits((r, p): Transform) -> Result<[u32; 12]> {
    Ok(std::array::from_fn(|i| if i < 9 { r[i] } else { p[i - 9] }.to_bits()))
}

fn quat_matrix(q: [f32; 4]) -> [f32; 9] {
    gearbox_engine::Quat::from_array(q).to_mat3().to_array()
}
