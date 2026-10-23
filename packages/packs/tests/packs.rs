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

#[test]
fn vehicle_items_resolve_car_models_without_static_collision_or_waypoints() {
    use gearbox_packs::{
        assets::Assets,
        catalog::{EntModel, Source},
    };

    let Some((set, _)) = packs() else { return };
    let assets =
        Assets::from_catalog(gearbox_packs::catalog::crawl(&set, &["maniaplanet_modelssport.pak"]).unwrap());
    for car in gearbox_packs::assets::Car::ALL {
        let item = gearbox_gbx::map::Item {
            model: Some(car.name().into()),
            collection: Some("10003".into()),
            author: Some("Nadeo".into()),
            yaw_pitch_roll: [0.0; 3],
            block_coord: [0; 3],
            position: [0.0; 3],
            pivot: [0.0; 3],
            scale: 1.0,
            flags: 0,
            anim_phase: 0,
            waypoint: None,
            placed_on_item: None,
            snapped_on_block: None,
            snapped_on_item: None,
            snapped_on_group: None,
        };
        let path = assets.find_anchored_item(&item, "Stadium").unwrap();
        let [EntModel::Vehicle { physics, visual }] = assets.index.items[path].models.as_slice() else {
            panic!("missing vehicle model for {}", car.name());
        };
        assert_eq!(set.class_of(physics), Some(0x090ea000));
        assert_eq!(set.class_of(visual), Some(0x090e7000));
        assert!(assets.item_model(path).unwrap().is_empty());
        assert_eq!(gearbox_packs::spawn::item_waypoint_tag(&set.read_path(path).unwrap()).unwrap(), None);
    }
}
