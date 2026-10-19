use std::path::Path;

use gearbox_gbx::Replay;
use gearbox_gbx::record::CLASS_VEHICLE_VIS;

fn replays() -> Vec<std::path::PathBuf> {
    let Some(dir) = std::env::var_os("GBX_TEST_DATA").map(|d| Path::new(&d).join("replays")) else {
        return vec![];
    };
    let Ok(entries) = std::fs::read_dir(dir) else {
        return vec![];
    };
    entries
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|p| p.to_string_lossy().ends_with(".Replay.Gbx"))
        .collect()
}

#[test]
fn recorded_steer_matches_input_stream() {
    let files = replays();
    if files.is_empty() {
        eprintln!("no replays in GBX_TEST_DATA/replays, skipping");
        return;
    }
    for path in files {
        let replay = Replay::parse(&std::fs::read(&path).unwrap()).unwrap();
        for ghost in &replay.ghosts {
            let inputs = ghost.inputs[0].decode().unwrap();
            let samples = &ghost
                .record
                .as_ref()
                .unwrap()
                .entities
                .iter()
                .find(|e| e.class_id == CLASS_VEHICLE_VIS)
                .unwrap()
                .samples;

            let race_time = ghost.race_time_ms.unwrap();
            assert!(
                inputs.start_offset_ms + inputs.ticks.len() as i32 * 10 >= race_time,
                "{path:?}: inputs end before finish"
            );

            let mut worst = 0f32;
            for s in samples {
                let tick = ((s.time_ms - inputs.start_offset_ms) / 10) as usize;
                let recorded = s.vehicle.as_ref().unwrap().steer;
                let input = inputs.ticks[tick].steer as f32 / 127.0;
                worst = worst.max((recorded - input).abs());
            }
            assert!(worst <= 2.0 / 255.0 + 1e-4, "{path:?}: recorded steer differs from inputs by {worst}");
        }
    }
}
