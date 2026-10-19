use gearbox_packs::blockinfo::{BlockInfo, VariantKind, read_block_info};

fn archive(path: &str) -> Option<Vec<u8>> {
    let set = gearbox_packs::install::Game::locate(None).ok()?.open().ok()?;
    Some(set.read_path(path).unwrap())
}

fn block(path: &str) -> Option<BlockInfo> {
    let buf = archive(path)?;
    let l = gearbox_packs::nodes::layout(&buf).unwrap();
    Some(read_block_info(&buf, &l, path).unwrap())
}

fn variant(b: &BlockInfo, kind: VariantKind, index: usize) -> &gearbox_packs::blockinfo::Variant {
    b.variants.iter().find(|v| v.kind == kind && v.index == index).unwrap()
}

#[test]
fn pillar_with_a_nested_symmetrical_block_info() {
    let Some(b) = block(
        "Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoPillar\\TrackWallChicaneX2LeftPillar.EDClassic.Gbx",
    ) else {
        return;
    };
    assert_eq!(b.variants.len(), 4);
    assert_eq!(variant(&b, VariantKind::Ground, 0).units, [[0, 0, 0], [0, 0, 1], [1, 0, 0], [1, 0, 1]]);
    assert_eq!(variant(&b, VariantKind::Air, 1).units, [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]]);
    assert_eq!(variant(&b, VariantKind::Air, 0).mobils.len(), 8);
    assert_eq!(variant(&b, VariantKind::Ground, 1).mobils.len(), 1);
}

#[test]
fn free_clip_type_and_mobil_offset() {
    let Some(b) =
        block("Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoClip\\PlatformWallStraightFCTRacing.EDClip.Gbx")
    else {
        return;
    };
    assert_eq!(b.clip_type, 2);
    let air = variant(&b, VariantKind::Air, 0);
    assert_eq!(air.units, [[0, 0, 0]]);
    let m = &air.mobils[0][0];
    assert_eq!((m.geom_translation, m.geom_rotation), (Some([0.0, 32.0, 0.0]), Some([90.0, 0.0, 0.0])));
    assert!(variant(&b, VariantKind::Ground, 0).units.is_empty());
}

#[test]
fn clip_mirror_image() {
    let Some(b) = block(
        "GreenCoast\\GameCtnBlockInfo\\GameCtnBlockInfoClip\\Stadium\\DecoWallSlope2End2VFCRight.EDVerticalClip.Gbx",
    ) else {
        return;
    };
    assert_eq!(b.symmetrical_clip.as_deref(), Some("DecoWallSlope2End2VFCLeft"));
    let Some(b) =
        block("Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoClip\\PlatformWallStraightFCTRacing.EDClip.Gbx")
    else {
        return;
    };
    assert_eq!(b.symmetrical_clip.as_deref(), Some("PlatformWallStraightFCB"));
}

#[test]
fn block_collector_id_differs_from_filename() {
    let Some(b) =
        block("Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoClassic\\GateSpecialBoost2.EDClassic.Gbx")
    else {
        return;
    };
    assert_eq!(b.name.as_deref(), Some("GateSpecialBoost2Oriented"));
    let Some(b) = block(
        "Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoClassic\\Deprecated\\GateSpecialBoost2.EDClassic.Gbx",
    ) else {
        return;
    };
    assert_eq!(b.name.as_deref(), Some("GateSpecialBoost2"));
}

#[test]
fn nested_pillar_variants_do_not_supply_water() {
    let path =
        "Stadium\\GameCtnBlockInfo\\GameCtnBlockInfoPillar\\TrackWallChicaneX2LeftPillar.EDClassic.Gbx";
    let Some(buf) = archive(path) else { return };
    for ground in [false, true] {
        for index in 0..2 {
            assert!(gearbox_packs::map_model::water_volumes(&buf, path, ground, index).unwrap().is_empty());
        }
    }
}
