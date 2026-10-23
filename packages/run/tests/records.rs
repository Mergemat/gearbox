use std::path::PathBuf;

use gearbox_gbx::{Map, Replay};
use gearbox_packs::install::Game;
use gearbox_packs::packs::PackSet;
use gearbox_run::Record;

fn bench(id: &str) -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("GBX_TEST_DATA")?).join("bench/maps").join(id);
    dir.join("wr.Replay.Gbx").exists().then_some(dir)
}

fn drives_to_its_finish(id: &str) {
    let Some(dir) = bench(id) else {
        eprintln!("no bench map {id} in GBX_TEST_DATA, skipping");
        return;
    };
    let packs = PackSet::open(&Game::locate(None).unwrap().packs).unwrap();
    let map = Map::parse(&std::fs::read(dir.join("map.Map.Gbx")).unwrap()).unwrap();
    let record = Record::read(&dir.join("wr.Replay.Gbx")).unwrap();
    let driven = record.drive(&map, &packs).unwrap();
    assert_eq!(driven.finish_ms, Some(record.time_ms), "{id}: {driven:?}");
}

#[test]
fn a_record_whose_inputs_start_at_the_race_start_still_drives_the_countdown() {
    let Some(dir) = bench("297170") else { return };
    let record = Record::read(&dir.join("wr.Replay.Gbx")).unwrap();
    assert_eq!(record.run.t0, -1500);
    assert!(record.run.controls[..150].iter().all(|c| *c == Default::default()));
    drives_to_its_finish("297170");
}

#[test]
fn records_with_their_own_countdown_drive_to_their_finish() {
    for id in ["105877", "117913", "126813", "170236", "66836"] {
        drives_to_its_finish(id);
    }
}

#[test]
fn a_record_with_its_own_countdown_keeps_its_start() {
    let Some(data) = std::env::var_os("GBX_TEST_DATA").map(PathBuf::from) else { return };
    let dir = data.join("tmx/effects");
    let Ok(record) = Record::read(&dir.join("179381_213194.Replay.Gbx")) else { return };
    assert_eq!(record.run.t0, -1460);
    let packs = PackSet::open(&Game::locate(None).unwrap().packs).unwrap();
    let map = Map::parse(&std::fs::read(dir.join("179381_No_Grip_Dirt.Map.Gbx")).unwrap()).unwrap();
    assert_eq!(record.drive(&map, &packs).unwrap().finish_ms, Some(record.time_ms));
}

#[test]
fn keyboard_records_with_action_keys_drive_to_their_finish() {
    let Some(data) = std::env::var_os("GBX_TEST_DATA").map(PathBuf::from) else { return };
    let packs = PackSet::open(&Game::locate(None).unwrap().packs).unwrap();
    for id in ["212725", "228722"] {
        let dir = data.join("runs").join(id);
        let Ok(record) = Record::read(&dir.join("record.Replay.Gbx")) else { continue };
        let map = Map::parse(&std::fs::read(dir.join("map.Map.Gbx")).unwrap()).unwrap();
        assert_eq!(record.drive(&map, &packs).unwrap().finish_ms, Some(record.time_ms), "{id}");
    }
}

#[test]
fn a_record_with_respawns_drives_to_its_finish() {
    let Some(data) = std::env::var_os("GBX_TEST_DATA").map(PathBuf::from) else { return };
    let Ok(bytes) = std::fs::read(data.join("runs/river/record.Replay.Gbx")) else { return };
    let replay = Replay::parse(&bytes).unwrap();
    let record = Record::from_replay(&replay).unwrap();
    assert_eq!(record.respawns.len(), 9);
    let packs = PackSet::open(&Game::locate(None).unwrap().packs).unwrap();
    let map = Map::parse(replay.map_gbx.as_deref().unwrap()).unwrap();
    assert_eq!(record.drive(&map, &packs).unwrap().finish_ms, Some(record.time_ms));
}
