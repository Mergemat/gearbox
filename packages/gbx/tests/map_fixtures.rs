use std::path::{Path, PathBuf};

use gearbox_gbx::Map;

fn tmx(relative: &str) -> Option<Map> {
    let Some(root) = std::env::var_os("GBX_TEST_DATA") else {
        eprintln!("GBX_TEST_DATA unset; skipping private fixture {relative}");
        return None;
    };
    fixture(&PathBuf::from(root).join("tmx").join(relative))
}

fn pf(name: &str) -> Option<Map> {
    let root = std::env::var_os("GBX_PF_MAPS")?;
    fixture(&PathBuf::from(root).join(name))
}

fn fixture(path: &Path) -> Option<Map> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    Some(Map::parse(&bytes).unwrap_or_else(|e| panic!("{}: {e:#}", path.display())))
}

#[test]
fn v5_training_items_snap_to_the_road() {
    let Some(map) = tmx("effects/1993_Training_-_09.Map.Gbx") else {
        return;
    };
    assert_eq!(map.name.as_deref(), Some("Training - 09"));
    assert_eq!(map.blocks.len(), 28);
    assert_eq!(map.items.len(), 5);
    let pillar = &map.items[0];
    assert_eq!(pillar.model.as_deref(), Some("ObstaclePillar8m"));
    assert_eq!(pillar.collection.as_deref(), Some("26"));
    assert_eq!(pillar.author.as_deref(), Some("Nadeo"));
    assert_eq!(pillar.block_coord, [255; 3]);
    assert_eq!(pillar.position, [720.0, 10.000004, 872.0]);
    assert_eq!(pillar.pivot, [0.0; 3]);
    assert_eq!(pillar.scale, 1.0);
    assert_eq!(pillar.snapped_on_block, Some(2));
    assert_eq!(pillar.snapped_on_group, Some(0));
    assert!(map.blocks[2].name.starts_with("Road"));
}

#[test]
fn v6_empty_item_list_and_embedded_validation_ghost() {
    let Some(map) = tmx("kacky/25432_LOL_1_-_IZIBUG.Map.Gbx") else {
        return;
    };
    assert_eq!(map.blocks.len(), 2996);
    assert_eq!(map.baked_blocks.len(), 6966);
    assert!(map.items.is_empty());
}

#[test]
fn v7_large_map_with_camera_tracks_keeps_item_transforms() {
    let Some(map) = tmx("desert/177552_NADC_Where_the_Watermelons_Grow.Map.Gbx") else {
        return;
    };
    assert_eq!(map.blocks.len(), 12080);
    assert_eq!(map.baked_blocks.len(), 8398);
    assert_eq!(map.items.len(), 1996);
    assert_eq!(map.items[0].model.as_deref(), Some("SupportConnectorX6"));
    assert_eq!(map.items[0].position, [864.0, 55.0, 603.0]);
    assert_eq!(map.items[1].position, [864.0, 66.0, 596.0]);
    assert_eq!(
        map.items[0].yaw_pitch_roll,
        [1.5707963, std::f32::consts::PI, 0.0]
    );
}

#[test]
fn v7_item_snapping_keeps_item_and_block_parents_separate() {
    let Some(map) = tmx("desert/177803_NADC_-_Gingerbread_Gorge.Map.Gbx") else {
        return;
    };
    assert_eq!(map.items.len(), 2184);
    let screen = &map.items[2099];
    assert_eq!(screen.model.as_deref(), Some("ShowScreen"));
    assert_eq!(screen.flags, 1028);
    assert_eq!(screen.snapped_on_item, Some(2098));
    assert_eq!(screen.snapped_on_block, None);
    assert_eq!(screen.snapped_on_group, Some(2));
    let support = &map.items[2098];
    assert_eq!(support.model.as_deref(), Some("TunnelSupportHalfArch16m"));
    assert_eq!(support.snapped_on_block, Some(7802));
    assert_eq!(support.snapped_on_item, None);
    assert_eq!(support.snapped_on_group, Some(2));
}

#[test]
fn embedded_kacky_models_are_real_gbx_files() {
    let Some(map) = tmx("kacky/126821_Kacky_Reloaded_258.Map.Gbx") else {
        return;
    };
    assert_eq!(map.items.len(), 846);
    assert_eq!(map.embedded.models.len(), 2);
    assert_eq!(map.embedded.files.len(), 2);
    let file = &map.embedded.files[0];
    assert_eq!(
        file.path,
        "Items\\Kacky\\204\\Mini_Pltf_Flat_NoGrip.Item.Gbx"
    );
    let model = file.item_model.as_ref().unwrap();
    assert_eq!(
        model.ident[0].as_deref(),
        Some("Kacky\\204\\Mini_Pltf_Flat_NoGrip.Item.Gbx")
    );
    assert_eq!(model.ident[1].as_deref(), Some("26"));
    assert_eq!(model.ident[2].as_deref(), Some("1hQaX_h5TiG8r96WElL8rQ"));
    assert_eq!(model.item_type, 1);
    assert_eq!(
        gearbox_gbx::Container::parse(&file.data).unwrap().class_id,
        0x2E00_2000
    );
    assert!(
        map.items
            .iter()
            .any(|i| i.model == model.ident[0] && i.author == model.ident[2])
    );
}

#[test]
fn pf_gate_placements_match_the_game() {
    let Some(map) = pf("pf-332174.Map.Gbx") else {
        return;
    };
    assert_eq!(map.items.len(), 9);
    assert_eq!(map.items[0].model.as_deref(), Some("GateStartCenter32m"));
    assert_eq!(
        map.items[0].waypoint.as_ref().unwrap().tag.as_deref(),
        Some("Spawn")
    );
    assert_eq!(map.items[0].position, [782.0, 254.0, 706.0]);
    let expected = [
        ("NoSteering", 111.0),
        ("SlowMotion", 112.0),
        ("Fragile", 112.0),
        ("NoEngine", 112.0),
        ("NoBrake", 110.0),
        ("Cruise", 110.0),
        ("Boost", 109.0),
    ];
    for (item, (effect, y)) in map.items[1..8].iter().zip(expected) {
        assert_eq!(
            item.model.as_deref().unwrap(),
            format!("GateSpecial32m{effect}")
        );
        assert_eq!(item.position, [784.0, y, 706.0]);
    }
    assert_eq!(map.items[8].model.as_deref(), Some("Podium"));
    assert_eq!(map.items[8].position, [781.0, 91.0, 767.0]);
}

#[test]
fn pf_custom_nogrip_gate_matches_embedded_model() {
    let Some(map) = pf("pf-228848.Map.Gbx") else {
        return;
    };
    assert_eq!(map.items.len(), 6);
    let gate = &map.items[0];
    assert_eq!(
        gate.model.as_deref(),
        Some("zzz_ImportedItems\\NoGrip Gates\\NoGripGates\\GateSpecial32mNoGrip.Item.Gbx")
    );
    assert_eq!(gate.position, [668.0, 71.0, 762.0]);
    assert_eq!(map.embedded.files.len(), 1);
    let model = map.embedded.files[0].item_model.as_ref().unwrap();
    assert_eq!(
        model.ident,
        [gate.model.clone(), Some("26".into()), gate.author.clone()]
    );
    assert_eq!(model.item_type, 1);
    assert_eq!(
        map.items[1].model.as_deref(),
        Some("GateCheckpointCenter32mv2")
    );
    assert_eq!(
        map.items[1].waypoint.as_ref().unwrap().tag.as_deref(),
        Some("Checkpoint")
    );
    assert_eq!(map.items[1].position, [668.0, 73.0, 738.0]);
    for mat in &map.items[3..5] {
        assert_eq!(mat.model.as_deref(), Some("InflatableMat1mSlopeBase1x1"));
        assert_eq!(mat.pivot, [-4.0, 0.0, -4.0]);
    }
    assert_eq!(map.items[3].position, [815.0, 59.0, 614.0]);
    assert_eq!(map.items[4].position, [815.0, 59.0, 616.0]);
}
