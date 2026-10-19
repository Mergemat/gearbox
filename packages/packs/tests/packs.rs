use std::path::PathBuf;

use gearbox_packs::packs::PackSet;

fn packs() -> Option<(PackSet, PathBuf)> {
    let extract = PathBuf::from(std::env::var_os("GBX_TEST_DATA")?).join("game/extract/GameData");
    let set = gearbox_packs::install::Game::locate(None).ok()?.open().ok()?;
    Some((set, extract))
}

#[test]
fn decoded_files_match_the_games_own_extraction() {
    let Some((set, d)) = packs() else {
        eprintln!("no game or GBX_TEST_DATA, skipping");
        return;
    };
    for path in [
        "Vehicles\\Cars\\CarSport\\TuningsSport.VehicleTunings.Gbx",
        "Vehicles\\Cars\\CarSport\\PhyModelSport.VehiclePhyModel.Gbx",
        "Stadium\\Media\\Prefab\\RoadTech\\Straight_Air.Prefab.Gbx",
        "Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoClassic\\RoadTechStraight.EDClassic.Gbx",
    ] {
        let reference = d.join(path.replace('\\', "/"));
        let Ok(reference) = std::fs::read(&reference) else {
            eprintln!("{path}: no reference, skipping");
            continue;
        };
        assert_eq!(set.read_path(path).unwrap(), reference, "{path}");
    }
}

#[test]
fn active_tunings_are_the_documented_ones() {
    let Some((set, _)) = packs() else { return };
    for ((_, file), want) in gearbox_packs::tuning::CARS.iter().zip([27, 0, 10, 11]) {
        let tunings = gearbox_packs::tuning::decode(&set.read_path(file).unwrap()).unwrap();
        assert_eq!(tunings.active, want, "{file}");
    }
}

#[test]
fn vegetation_collision_archives_are_readable() {
    let Some((set, _)) = packs() else { return };
    for (file, _) in set.files().filter(|(_, f)| f.class_id == 0x2f086000) {
        let path = set.paks[file.pak].file_path(file.file);
        let buf = set.read(file).unwrap_or_else(|e| panic!("{path}: {e:#}"));
        let layout = gearbox_packs::nodes::layout(&buf).unwrap_or_else(|e| panic!("{path}: {e:#}"));
        let ctx = gearbox_packs::prefab::Ctx { buf: &buf, layout: &layout, path: &path };
        gearbox_packs::veget::collision(&ctx).unwrap_or_else(|e| panic!("{path}: {e:#}"));
    }
}
