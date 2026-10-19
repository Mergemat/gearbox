use gearbox_engine::Controls;
use gearbox_search::input::Timeline;

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn the_cli_pads_inputs_that_start_after_the_session() {
    let data = std::path::PathBuf::from(
        std::env::var_os("GBX_TEST_DATA").expect("GBX_TEST_DATA: a folder with tmx/"),
    );
    let packs = gearbox_packs::install::Game::locate(None).unwrap().packs;
    let dir = std::env::temp_dir().join(format!("tas-pad-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let late =
        Timeline { t0: 0, seed: None, controls: vec![Controls { gas: 1.0, brake: 0.0, steer: 0.25 }; 100] };
    late.write(&dir.join("late.in")).unwrap();
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_gearbox"))
        .arg("search")
        .arg(data.join("tmx/kacky/54999_uberbug.Map.Gbx"))
        .arg(dir.join("late.in"))
        .arg(dir.join("out.in"))
        .args(["--window-start", "500", "--window-end", "1000", "--iterations", "1"])
        .args(["--threads", "1", "speed", "--at-ms", "1000"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let written = Timeline::read(&dir.join("out.in")).unwrap();
    assert_eq!(written.t0, -20);
    assert_eq!(written.controls[..2], [Controls::default(); 2]);
    assert_eq!(written.controls[2..52], late.controls[..50]);
    let mut s =
        gearbox_search::session::open(&packs, &data.join("tmx/kacky/54999_uberbug.Map.Gbx"), Some(&late))
            .unwrap();
    while s.race_ms() < 1000 {
        let c = late.controls.get(((s.race_ms() - late.t0) / 10) as usize).copied().unwrap_or_default();
        s.step(c);
    }
    let v = s.body().vel.to_array();
    let speed = v.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>().sqrt();
    assert!(stdout.contains(&format!("before: speed {speed:.9} m/s")), "{stdout}");
}
