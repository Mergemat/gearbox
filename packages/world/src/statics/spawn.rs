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

const IDENTITY: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

pub fn item_location(item: &gearbox_gbx::map::Item) -> Transform {
    let frame = place::free_location_variant(item.position, item.yaw_pitch_roll, Default::default());
    let frame = place::compose_iso(frame, (IDENTITY, [0.0; 3]));
    place::compose_iso((IDENTITY, item.pivot), frame)
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
    let (r, p) = spawns.first().copied().unwrap_or((IDENTITY, [0.0; 3]));
    Ok(std::array::from_fn(|i| if i < 9 { r[i] } else { p[i - 9] }.to_bits()))
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
        let mobil =
            [((block.flags >> 21) & 0x7f) as usize, block.variant as usize, block.sub_variant as usize];
        let (name, pose) = gearbox_packs::spawn::block_item_waypoint(&file.data, block.is_ground(), mobil)?;
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

pub fn item_waypoint_tags(
    map: &gearbox_gbx::Map,
    assets: &Assets,
    files: &PackSet,
) -> Result<Vec<Option<String>>> {
    let mut models = std::collections::HashMap::new();
    map.items
        .iter()
        .map(|item| {
            let name = item.model.as_deref().context("item has no model")?;
            if let Some(tag) = models.get(name) {
                return Ok(Option::clone(tag));
            }
            let file = match map.embedded.item_file(name) {
                Ok(embedded) => embedded.data.clone(),
                Err(_) => files.read_path(&nadeo_item_path(assets, files, name)?)?,
            };
            let tag =
                gearbox_packs::spawn::item_waypoint_tag(&file).with_context(|| format!("item {name}"))?;
            models.insert(name, tag.clone());
            Ok(tag)
        })
        .collect()
}

fn nadeo_item_path(assets: &Assets, files: &PackSet, name: &str) -> Result<String> {
    let indexed =
        assets.index.items.keys().find(|path| gearbox_packs::assets::block_name(path) == name).cloned();
    let packed = || {
        files
            .files()
            .map(|(r, _)| files.paks[r.pak].file_path(r.file))
            .find(|path| gearbox_packs::assets::block_name(path) == name)
    };
    indexed.or_else(packed).with_context(|| format!("item {name} absent from the packs"))
}

pub fn item_poses(
    map: &gearbox_gbx::Map,
    item: &gearbox_gbx::map::Item,
    assets: &Assets,
    files: &PackSet,
) -> Result<([u32; 12], [u32; 12])> {
    let name = item.model.as_deref().context("waypoint item has no model")?;
    let local = if item.author.as_deref() == Some("Nadeo") {
        nadeo_item_spawn(assets, files, name)?
    } else {
        gearbox_packs::spawn::item_landmark(&map.embedded.item_file(name)?.data)?
    };
    let local = pose_transform(local);
    let frame = item_location(item);
    let pose = pose_bits(place::compose_iso(local, frame))?;
    let environment = super::packs::environment(map).map_err(anyhow::Error::msg)?;
    let path = assets
        .find_anchored_item(item, environment)
        .with_context(|| format!("waypoint item {name} absent from assets"))?;
    let models = assets.index.items[path].models(0);
    if !models.iter().any(|model| matches!(model, EntModel::Prefab(_))) {
        return Ok((pose, pose));
    }
    let rotation = gearbox_engine::Mat3::from_array(frame.0);
    let respawn_frame = (gearbox_engine::Quat::from_mat3(&rotation).to_mat3().to_array(), frame.1);
    Ok((pose, pose_bits(place::compose_iso(local, respawn_frame))?))
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
