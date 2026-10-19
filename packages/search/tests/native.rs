use std::cmp::Ordering;

use gearbox_search::input::Timeline;
use gearbox_search::mutation::Mutation;
use gearbox_search::objective::Objective;
use gearbox_search::search::{Config, search_from};

fn fixtures() -> std::path::PathBuf {
    std::env::var_os("GBX_TEST_DATA").expect("GBX_TEST_DATA: a folder with tmx/ and inputs/").into()
}

fn packs() -> std::path::PathBuf {
    gearbox_packs::install::Game::locate(None).unwrap().packs
}

#[test]
#[ignore = "local packs, TMX 17165 and recorded inputs; run under nice"]
fn search_is_independent_of_worker_scheduling() {
    let root = fixtures();
    let session = gearbox_search::session::open(
        &packs(),
        &root.join("tmx/kacky/17165_Kacky_Reloaded_26.Map.Gbx"),
        Some(&Timeline::read(&root.join("inputs/k17165s.txt.in")).unwrap()),
    )
    .unwrap();
    let mut base = Timeline::read(&root.join("inputs/k17165s.txt.in")).unwrap();
    let range = base.index(1200).unwrap()..base.index(1300).unwrap();
    for c in &mut base.controls[range] {
        c.gas = 0.0;
        c.brake = 1.0;
    }
    let mut config = Config {
        window_start_ms: 1000,
        window_end_ms: 1800,
        until_ms: 3400,
        objective: Objective::Waypoint(1),
        iterations: 96,
        threads: 1,
        batch_size: 32,
        seed: 42,
        mutation: Mutation::default(),
        method: gearbox_search::search::Method::HillClimb,
        evaluation: Default::default(),
    };
    let one = search_from(&session, base.clone(), &config, |_| {}).unwrap();
    config.threads = 4;
    let four = search_from(&session, base, &config, |_| {}).unwrap();
    assert_eq!(one.after, four.after);
    assert_eq!(one.inputs.controls, four.inputs.controls);
    assert_eq!(one.evaluated_ticks, four.evaluated_ticks);
    assert_eq!(one.improvements, four.improvements);
    assert_eq!(four.after.score.quality_cmp(&four.before.score), Ordering::Greater);
    assert_eq!(one.iterations, 96);
}

#[test]
#[ignore = "local packs, TMX 17165 and recorded inputs; run under nice"]
fn guided_population_and_receding_are_independent_of_worker_count() {
    use gearbox_search::search::{Method, Origin};
    let root = fixtures();
    let origin = gearbox_search::session::open(
        &packs(),
        &root.join("tmx/kacky/17165_Kacky_Reloaded_26.Map.Gbx"),
        Some(&Timeline::read(&root.join("inputs/k17165s.txt.in")).unwrap()),
    )
    .unwrap();
    let gate = gearbox_search::gate::target_gate(&mut origin.native().unwrap(), 1).unwrap();
    let base = Timeline::read(&root.join("inputs/k17165s.txt.in")).unwrap();
    let mut n = origin.native().unwrap();
    n.seed = base.seed;
    let correct = gearbox_search::objective::evaluate(
        &mut n,
        &base,
        &Objective::Gate { ordinal: 1, gate: gate.clone(), time_penalty: 1.0 },
        3400,
    )
    .unwrap();
    assert!(correct.score.reached());
    let mut wrong_gate = gate.clone();
    wrong_gate.waypoint = u32::MAX;
    let mut n = origin.native().unwrap();
    n.seed = base.seed;
    let wrong = gearbox_search::objective::evaluate(
        &mut n,
        &base,
        &Objective::Gate { ordinal: 1, gate: wrong_gate, time_penalty: 1.0 },
        3400,
    )
    .unwrap();
    assert!(!wrong.score.reached());
    for method in [
        Method::Population { width: 4, curve_ticks: 10 },
        Method::Receding { horizon_ticks: 60, commit_ticks: 20, width: 4, curve_ticks: 10 },
    ] {
        let mut config = Config {
            window_start_ms: 1000,
            window_end_ms: 1800,
            until_ms: 3400,
            objective: Objective::Gate { ordinal: 1, gate: gate.clone(), time_penalty: 1.0 },
            iterations: 96,
            threads: 1,
            batch_size: 16,
            seed: 42,
            mutation: Mutation::default(),
            method,
            evaluation: Default::default(),
        };
        let one = gearbox_search::search::search_from(&origin, base.clone(), &config, |_| {}).unwrap();
        config.threads = 4;
        let four = gearbox_search::search::search_from(&origin, base.clone(), &config, |_| {}).unwrap();
        assert_eq!(one.after, four.after);
        assert_eq!(one.inputs.controls, four.inputs.controls);
        assert_eq!(one.evaluated_ticks, four.evaluated_ticks);
    }
}
