fn fixtures() -> std::path::PathBuf {
    std::env::var_os("GBX_TEST_DATA").expect("GBX_TEST_DATA: a folder with tmx/ and inputs/").into()
}

fn packs() -> std::path::PathBuf {
    gearbox_packs::install::Game::locate(None).unwrap().packs
}

#[test]
#[ignore = "local packs and TMX 126821; run under nice"]
fn reset_gameplay_trigger_is_not_part_of_the_finish() {
    let data = fixtures();
    let mut session = gearbox_search::session::open(
        &packs(),
        &data.join("tmx/kacky/126821_Kacky_Reloaded_258.Map.Gbx"),
        None,
    )
    .unwrap();
    let gate = gearbox_search::gate::target_gate(&mut session, 1).unwrap();
    assert_eq!((gate.waypoint, gate.kind), (1, 1));
    assert_eq!(
        gate.boxes,
        vec![gearbox_search::gate::Bounds {
            min: [1096.5, 150.96196, 1011.462],
            max: [1104.5, 156.20229, 1016.70233],
        }]
    );
}
