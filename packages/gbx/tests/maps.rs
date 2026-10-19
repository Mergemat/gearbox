use std::path::PathBuf;

use gearbox_gbx::map::BLOCK_SIZE;
use gearbox_gbx::record::CLASS_VEHICLE_VIS;
use gearbox_gbx::{Map, Replay};

fn replays() -> Vec<PathBuf> {
    let Some(root) = std::env::var_os("GBX_TEST_DATA").map(|d| PathBuf::from(d).join("replays")) else {
        return vec![];
    };
    let mut out = vec![];
    let mut dirs = vec![root];
    while let Some(dir) = dirs.pop() {
        let Ok(entries) = std::fs::read_dir(dir) else {
            continue;
        };
        for p in entries.filter_map(|e| Some(e.ok()?.path())) {
            if p.is_dir() {
                dirs.push(p);
            } else if p.to_string_lossy().ends_with(".Replay.Gbx") {
                out.push(p);
            }
        }
    }
    out
}

fn spawns(map: &Map) -> Vec<[f32; 3]> {
    let blocks = map
        .blocks
        .iter()
        .filter(|b| b.waypoint.as_ref().is_some_and(|w| w.tag.as_deref() == Some("Spawn")))
        .map(|b| match b.free {
            Some((pos, _)) => pos,
            None => [
                (b.coord[0] as f32 + 0.5) * BLOCK_SIZE[0],
                (b.coord[1] as f32 - 8.0) * BLOCK_SIZE[1],
                (b.coord[2] as f32 + 0.5) * BLOCK_SIZE[2],
            ],
        });
    let items = map
        .items
        .iter()
        .filter(|i| i.waypoint.as_ref().is_some_and(|w| w.tag.as_deref() == Some("Spawn")))
        .map(|i| i.position);
    blocks.chain(items).collect()
}

#[test]
fn first_car_sample_is_at_the_spawn() {
    let files = replays();
    if files.is_empty() {
        eprintln!("no replays in GBX_TEST_DATA/replays, skipping");
        return;
    }
    for path in files {
        let replay = Replay::parse(&std::fs::read(&path).unwrap()).unwrap();
        let map = Map::parse(replay.map_gbx.as_deref().unwrap()).unwrap();
        let spawns = spawns(&map);
        assert_eq!(spawns.len(), 1, "{path:?}: expected one spawn, got {spawns:?}");
        let spawn = spawns[0];
        for ghost in &replay.ghosts {
            let rec = ghost.record.as_ref().unwrap();
            let car = rec.entities.iter().find(|e| e.class_id == CLASS_VEHICLE_VIS).unwrap();
            let first = car.samples[0].vehicle.as_ref().unwrap().position;
            let horizontal = ((first[0] - spawn[0]).powi(2) + (first[2] - spawn[2]).powi(2)).sqrt();
            let vertical = first[1] - spawn[1];
            assert!(
                horizontal < 16.0 && (0.0..BLOCK_SIZE[1]).contains(&vertical),
                "{path:?}: car starts at {first:?}, spawn is {spawn:?}"
            );
        }
    }
}
