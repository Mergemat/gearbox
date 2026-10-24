use std::cmp::Ordering;

use gearbox_engine::Controls;
use gearbox_engine::race::RaceState;
use gearbox_search::input::Timeline;
use gearbox_search::mutation::Mutation;
use gearbox_search::objective::{Objective, Score};
use rand::{SeedableRng, rngs::StdRng};

#[test]
fn malformed_inputs_and_uncovered_times_are_errors() {
    for text in
        ["", "T0 0 1 extra\n1 0 0\n", "T0 0\n1 0\n", "T0 0\n1 0 NaN\n", "T0 0\n1 0 inf\n", "T0 0\n1 0 1.1\n"]
    {
        assert!(Timeline::parse(text).is_err(), "accepted {text:?}");
    }
    let input = Timeline::parse("T0 -1450 4294967295\n1 0 -0.5\n0 1 0.25\n").unwrap();
    assert_eq!((input.t0, input.seed), (-1450, Some(u32::MAX)));
    assert_eq!(input.tick(-1440).unwrap(), 1);
    assert_eq!(input.end_ms(), -1430);
    for ms in [-1460, -1449, -1420] {
        assert!(input.tick(ms).is_err());
    }
}

#[test]
fn neutral_throttle_seeds_are_distinct() {
    let neutral =
        Timeline { actions: Vec::new(), t0: 0, seed: None, controls: vec![Controls::default(); 100] };
    let mut seeds: Vec<Vec<Controls>> = Vec::new();
    for attempt in 0..16 {
        let mut inputs = neutral.clone();
        let mut rng = StdRng::seed_from_u64(attempt as u64);
        Mutation::default().explore(&mut inputs, 10..90, &[], 20, attempt, &mut rng);
        let window = &inputs.controls[10..90];
        if window.iter().all(|c| c.gas == 1.0 && c.brake == 0.0 && *c == window[0]) {
            assert!(!seeds.contains(&window.to_vec()), "attempt {attempt} repeats a seed");
            seeds.push(window.to_vec());
        }
    }
    assert!(seeds.len() >= 3, "{} seeds", seeds.len());
}

#[test]
fn padding_a_late_timeline_keeps_every_tick_on_its_race_time() {
    let late = Timeline::parse("T0 30 5\n1 0 0.5\n0 1 -0.5\n").unwrap();
    let padded = late.starting_at(-20).unwrap();
    assert_eq!((padded.t0, padded.seed, padded.end_ms()), (-20, Some(5), 50));
    for ms in (-20..30).step_by(10) {
        assert_eq!(padded.controls[padded.tick(ms).unwrap()], Controls::default());
    }
    for ms in [30, 40] {
        assert_eq!(padded.controls[padded.tick(ms).unwrap()], late.controls[late.tick(ms).unwrap()]);
    }
    assert!(late.starting_at(40).is_err());
    assert!(late.starting_at(-25).is_err());
}

#[test]
fn race_ranking_keeps_milliseconds_primary_and_rejects_equal_scores() {
    let slow_fast_car = Score::Race { time_ms: 2488, speed_squared: 1_000_000.0 };
    let fast_slow_car = Score::Race { time_ms: 2487, speed_squared: 1.0 };
    assert_eq!(fast_slow_car.quality_cmp(&slow_fast_car), Ordering::Greater);
    assert_eq!(fast_slow_car.quality_cmp(&fast_slow_car), Ordering::Equal);
    assert_eq!(fast_slow_car.quality_cmp(&Score::Unreached), Ordering::Greater);
    let fast_faster_car = Score::Race { time_ms: 2487, speed_squared: 2.0 };
    assert_eq!(fast_faster_car.quality_cmp(&fast_slow_car), Ordering::Greater);
    let race = RaceState { checkpoints: vec![2487, 5001], laps: 1, finish: Some(5001) };
    assert_eq!(Objective::Waypoint(2).race_time(&race), Some(5001));
    assert_eq!(Objective::Waypoint(3).race_time(&race), None);
    assert_eq!(Objective::Finish.race_time(&race), Some(5001));
}

#[test]
fn mutations_cannot_change_the_prefix_suffix_or_control_ranges() {
    let original: Vec<_> = (0..100)
        .map(|i| Controls {
            gas: if i < 40 { 1.0 } else { 0.0 },
            brake: if i < 60 { 0.0 } else { 1.0 },
            steer: if i < 50 { -1.0 } else { 1.0 },
        })
        .collect();
    let mutation = Mutation {
        max_changes: 8,
        max_span_ticks: 1000,
        max_shift_ticks: 1000,
        max_steer_delta: 2.0,
        ..Default::default()
    };
    let mut rng = StdRng::seed_from_u64(2);
    for window in [20..80, 50..51] {
        for _ in 0..200 {
            let mut controls = original.clone();
            mutation.apply(&mut controls, window.clone(), &mut rng);
            assert_eq!(controls[..window.start], original[..window.start]);
            assert_eq!(controls[window.end..], original[window.end..]);
            for c in &controls {
                assert!((0.0..=1.0).contains(&c.gas));
                assert!((0.0..=1.0).contains(&c.brake));
                assert!((-1.0..=1.0).contains(&c.steer));
            }
        }
    }
}

#[test]
fn guided_misses_have_gradient_but_cannot_beat_a_reached_gate() {
    let score = |time_ms, cost, bonus| Score::Guided { time_ms, cost, speed_squared: 100.0, bonus };
    assert_eq!(score(None, 2.0, 0.0).quality_cmp(&score(None, 10.0, 0.0)), Ordering::Greater);
    assert_eq!(score(Some(99999), 99999.0, 0.0).quality_cmp(&score(None, -1e12, 1e12)), Ordering::Greater);
    assert_eq!(score(Some(100), 100.0, 0.0).quality_cmp(&score(Some(101), 101.0, 1e12)), Ordering::Greater);
}

#[test]
fn polyline_projection_respects_turns_and_limits_jumps_at_crossings() {
    use gearbox_search::objective::ReferencePath;
    let path = ReferencePath {
        points: vec![[0., 0., 0.], [10., 0., 0.], [10., 0., 10.]],
        deviation_penalty: 1.0,
        stay_within: None,
    };
    path.validate().unwrap();
    let (progress, deviation) = path.project([12., 0., 5.], f64::INFINITY);
    assert!((progress - 15.).abs() < 1e-6 && (deviation - 2.).abs() < 1e-6);
    let (progress, _) = path.project([10., 0., 5.], 8.);
    assert!(progress <= 8.);
}

#[test]
fn exploration_from_neutral_includes_acceleration_and_preserves_window() {
    let mut inputs =
        Timeline { actions: Vec::new(), t0: -1500, seed: None, controls: vec![Controls::default(); 200] };
    let mut rng = StdRng::seed_from_u64(42);
    Mutation::default().explore(&mut inputs, 40..100, &[], 20, 0, &mut rng);
    assert!(inputs.controls[40..100].iter().all(|c| c.gas == 1.0 && c.steer == 0.0));
    assert!(inputs.controls[..40].iter().chain(&inputs.controls[100..]).all(|c| *c == Controls::default()));
    for attempt in 1..200 {
        Mutation::default().explore(&mut inputs, 40..100, &[], 20, attempt, &mut rng);
        assert!(
            inputs.controls[..40].iter().chain(&inputs.controls[100..]).all(|c| *c == Controls::default())
        );
        assert!(inputs.controls.iter().all(|c| (0.0..=1.0).contains(&c.gas)
            && (0.0..=1.0).contains(&c.brake)
            && (-1.0..=1.0).contains(&c.steer)));
    }
}

#[test]
fn goals_rank_by_progress_then_the_last_time() {
    use gearbox_search::objective::Score;
    use std::cmp::Ordering;
    let goals =
        |times: Vec<i32>, error: f64| Score::Goals { times, total: 3, error, strayed: false, cuts: 0 };
    let early_first =
        Score::Goals { times: vec![1000, 5000, 9000], total: 3, error: 0.0, strayed: false, cuts: 0 };
    let early_last =
        Score::Goals { times: vec![1010, 5100, 8000], total: 3, error: 0.0, strayed: false, cuts: 0 };
    assert_eq!(early_last.quality_cmp(&early_first), Ordering::Greater);
    assert_eq!(goals(vec![1100], 10.0).quality_cmp(&goals(vec![1000], 50.0)), Ordering::Greater);
    assert_eq!(goals(vec![9000, 9500], 900.0).quality_cmp(&goals(vec![1000], 0.1)), Ordering::Greater);
}

#[test]
fn pedal_cuts_have_to_pay_for_themselves() {
    use gearbox_search::objective::Score;
    use std::cmp::Ordering;
    let line = |times: Vec<i32>, error: f64, cuts: u32| Score::Goals {
        times,
        total: 1,
        error,
        strayed: false,
        cuts,
    };
    assert_eq!(line(vec![], 10.0, 0).quality_cmp(&line(vec![], 9.0, 40)), Ordering::Greater);
    assert_eq!(line(vec![], 7.0, 40).quality_cmp(&line(vec![], 10.0, 0)), Ordering::Greater);
    assert_eq!(line(vec![5200], 0.0, 90).quality_cmp(&line(vec![5210], 0.0, 0)), Ordering::Greater);
    assert_eq!(line(vec![5200], 0.0, 0).quality_cmp(&line(vec![5200], 0.0, 30)), Ordering::Greater);
}
