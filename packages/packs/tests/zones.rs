use gearbox_packs::{blockinfo, zones::Zones};

#[test]
fn ghost_terrain_keeps_a_hill_under_a_flat_zone() {
    let Some(game) = gearbox_packs::install::Game::locate(None).ok() else { return };
    let packs = game.open().unwrap();
    let zones = Zones::load(&packs).unwrap();
    let path =
        "WhiteShore\\GameCtnBlockInfo\\GameCtnBlockInfoClassic\\Stadium\\PlatformIceDiag1.EDClassic.Gbx";
    let buf = packs.read_path(path).unwrap();
    let layout = gearbox_packs::nodes::layout(&buf).unwrap();
    let info = blockinfo::read_block_info(&buf, &layout, path).unwrap();
    let variant =
        info.variants.iter().find(|v| v.kind == blockinfo::VariantKind::Ground && v.index == 0).unwrap();
    let (columns, _) = blockinfo::auto_terrain(&buf, &layout, path, variant.ground_archive.unwrap()).unwrap();
    let column = &columns[0];
    assert!(zones.terrain_matches("WhiteShore", column, 1, "Land", 0, 0).unwrap());
    assert!(!zones.terrain_matches("WhiteShore", column, 1, "LandHill2", 9, 0).unwrap());
    assert!(!zones.terrain_matches("WhiteShore", column, 1, "LandHill1", 9, 0).unwrap());
}
