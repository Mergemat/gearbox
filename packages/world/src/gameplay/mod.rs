mod player;
mod scene;

use crate::statics::MapAssets;
use anyhow::{Context, Result};
use gearbox_engine::{
    Iso,
    race::{self, Race, RaceWaypoint, Rules, WaypointFlags},
};
pub use scene::{EntityBinding, EntityKind};

#[derive(Clone, Debug, PartialEq)]
pub struct Gameplay {
    pub race: Race,
    pub settings: race::RaceRules,
    pub scene: gearbox_engine::arena::Scene,
    pub entities: Vec<EntityBinding>,
    pub ghost: gearbox_engine::arena::GhostTarget,
    pub spawn: u32,
    pub initial_pose: Iso,
}

pub fn build(map: &gearbox_gbx::Map, assets: &MapAssets<'_>, start: u32, now: u32) -> Result<Gameplay> {
    let catalog = assets.geometry(map)?;
    let environment = crate::statics::packs::environment(map).map_err(anyhow::Error::msg)?;
    let mut landmarks = Vec::new();
    for block in &map.blocks {
        let Some(waypoint) = block_waypoint(block, &assets.assets, environment) else { continue };
        let pose = crate::statics::block_pose(map, block, &assets.assets, assets.packs, &catalog)?;
        landmarks.push(landmark(&waypoint, pose)?);
    }
    for item in &map.items {
        let Some(waypoint) = &item.waypoint else { continue };
        let pose = crate::statics::item_pose(map, item, &assets.assets, assets.packs)?;
        landmarks.push(landmark(waypoint, pose)?);
    }
    let spawn = select_spawn(&landmarks)?;
    let (waypoints, checkpoints_per_lap) = groups(landmarks);
    let initial_pose = Iso::from_array([0.0; 12]);
    let settings = race::RaceRules {
        start,
        mode_bits: LOCAL_RACE_MODE,
        checkpoints: checkpoints_per_lap,
        ..Default::default()
    };
    Ok(Gameplay {
        race: Race {
            rules: Rules {
                mode: race::Mode::Race,
                checkpoints_per_lap,
                lap_race: map.lap_race,
                laps: if map.lap_race { map.laps } else { 0 },
                waypoints,
                clear_untouched: false,
            },
            store: player::store(start),
            players: vec![player::build(start, spawn, 0)],
        },
        settings,
        scene: scene::build(now),
        entities: vec![EntityBinding { handle: 0, kind: EntityKind::Vehicle, index: 0 }],
        ghost: Default::default(),
        spawn,
        initial_pose,
    })
}

pub(crate) fn block_waypoint(
    block: &gearbox_gbx::map::Block,
    assets: &gearbox_packs::assets::Assets,
    environment: &str,
) -> Option<gearbox_gbx::map::Waypoint> {
    block.waypoint.clone().or_else(|| {
        let tag = assets.block_in(&block.name, environment)?.waypoint.clone()?;
        Some(gearbox_gbx::map::Waypoint { tag: Some(tag), order: 0 })
    })
}

fn landmark(waypoint: &gearbox_gbx::map::Waypoint, pose: [u32; 12]) -> Result<(RaceWaypoint, bool)> {
    let tag = waypoint.tag.as_deref().context("unnamed waypoint")?;
    let flags = match tag {
        "Spawn" => WaypointFlags::SPAWN,
        "Checkpoint" | "LinkedCheckpoint" => WaypointFlags::CHECKPOINT,
        "Goal" => WaypointFlags::GOAL,
        "StartFinish" => WaypointFlags(WaypointFlags::SPAWN.0 | WaypointFlags::GOAL.0),
        _ => anyhow::bail!("unsupported waypoint tag {tag}"),
    };
    Ok((
        RaceWaypoint {
            flags,
            group: NO_GROUP,
            order: waypoint.order as u32,
            pose: Iso::from_array(pose.map(f32::from_bits)),
            linked: flags.counts(),
            respawn: flags.contains(WaypointFlags::SPAWN) || flags.contains(WaypointFlags::GOAL),
        },
        tag == "LinkedCheckpoint",
    ))
}

const LOCAL_RACE_MODE: u8 = 0x88;
const NO_GROUP: u32 = u32::MAX;
const LINKED_KEY: u32 = 0x8000_0000;

fn select_spawn(landmarks: &[(RaceWaypoint, bool)]) -> Result<u32> {
    landmarks
        .iter()
        .position(|(mark, _)| mark.flags == WaypointFlags::SPAWN)
        .or_else(|| landmarks.iter().position(|(mark, _)| mark.flags.contains(WaypointFlags::SPAWN)))
        .map(|index| index as u32)
        .context("map has no start")
}

fn groups(landmarks: Vec<(RaceWaypoint, bool)>) -> (Vec<RaceWaypoint>, u32) {
    let keys: Vec<_> = landmarks
        .iter()
        .enumerate()
        .map(|(index, (mark, linked))| {
            if mark.flags == WaypointFlags::SPAWN {
                NO_GROUP
            } else if *linked {
                mark.order | LINKED_KEY
            } else {
                index as u32
            }
        })
        .collect();
    let mut unique: Vec<_> = keys.iter().copied().filter(|&key| key != NO_GROUP).collect();
    gearbox_engine::numeric::quicksort(&mut unique, |left, right| match left.cmp(right) {
        std::cmp::Ordering::Less => -1,
        std::cmp::Ordering::Equal => 0,
        std::cmp::Ordering::Greater => 1,
    });
    unique.dedup();
    let mut checkpoints = std::collections::BTreeSet::new();
    let mut waypoints: Vec<_> = landmarks.into_iter().map(|(mark, _)| mark).collect();
    for (mark, key) in waypoints.iter_mut().zip(keys) {
        mark.group = unique.binary_search(&key).map_or(NO_GROUP, |index| index as u32);
        if mark.flags.contains(WaypointFlags::CHECKPOINT) {
            checkpoints.insert(key);
        }
    }
    (waypoints, checkpoints.len() as u32)
}

pub(crate) fn gates(
    world: &gearbox_engine::static_world::StaticWorld,
    effects: &std::collections::HashMap<u32, u8>,
) -> Result<Vec<race::GateRecord>> {
    world
        .items
        .iter()
        .filter(|item| matches!(item.entity, gearbox_engine::static_world::Entity::Gate(_)))
        .map(|item| {
            let effect =
                effects.get(&item.shape).copied().context("gate has no first material gameplay metadata")?;
            let direction = world.shapes[item.shape as usize].main_dir;
            let rows = item.iso.rot.rows;
            let dir = gearbox_engine::Vec3::new(
                direction.y * rows[0].y + direction.x * rows[0].x + direction.z * rows[0].z,
                direction.x * rows[1].x + direction.y * rows[1].y + direction.z * rows[1].z,
                direction.x * rows[2].x + direction.y * rows[2].y + direction.z * rows[2].z,
            );
            Ok(race::GateRecord { effect, dir, id: u32::MAX })
        })
        .collect()
}
