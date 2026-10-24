use gearbox_engine::{Controls, Session};
use gearbox_search::Body;
use gearbox_search::{
    fan,
    goals::{Goal, Goals},
    input::Timeline,
    mutation::Mutation,
    objective::{EvalOptions, Objective, Score, evaluate},
    pose::{Placement, Pose, PoseGoal, place},
    search::{Config, Method, Origin, search_from},
};
use std::cmp::Ordering;

fn fixtures() -> std::path::PathBuf {
    std::env::var_os("GBX_TEST_DATA").expect("GBX_TEST_DATA: a folder with tmx/ and inputs/").into()
}

fn packs() -> std::path::PathBuf {
    gearbox_packs::install::Game::locate(None).unwrap().packs
}

fn open() -> Session {
    let data = fixtures();
    gearbox_search::session::open(&packs(), &data.join("tmx/kacky/54999_uberbug.Map.Gbx"), None).unwrap()
}
fn advance(s: &mut Session, until: i32) {
    while s.race_ms() < until {
        s.step(Controls::default());
    }
}
fn pose_goal(pose: Pose) -> PoseGoal {
    PoseGoal {
        pose,
        position_tolerance: [0.2; 3],
        angle_tolerance: 0.05,
        velocity_tolerance: None,
        position_weights: [1.0; 3],
        angle_weight: 10.0,
        velocity_weights: [0.0; 3],
    }
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn physical_pick_and_placed_car_falls_and_settles_on_road() {
    let mut s = open();
    advance(&mut s, 0);
    let body = Body::from(s.body());
    let p = body.position();
    let ground = gearbox_search::picking::pick(&mut s, [p[0], p[1] + 5.0, p[2]], [0.0, -20.0, 0.0])
        .expect("spawn road");
    eprintln!("spawn {:?}; physical road {:?}", p, ground);
    assert!((ground.point[1] - 58.0).abs() < 1e-4);
    assert!(ground.normal[1] > 0.99);
    assert!(ground.point[1] < p[1] && p[1] - ground.point[1] < 1.0);
    assert_eq!(ground.item, 0);
    assert!(gearbox_search::picking::pick(&mut s, [p[0], p[1] + 5.0, p[2]], [0.0, 20.0, 0.0]).is_none());
    let wall = gearbox_search::picking::pick(&mut s, [780.0, 8.5, 760.0], [-20.0, 0.0, 0.0])
        .expect("lower-ground wall");
    assert!((wall.point[0] - 768.0).abs() < 1e-4);
    assert!(wall.normal[0] > 0.99);
    assert_eq!(wall.surface, 22);
    let before_pick = s.save();
    for _ in 0..100 {
        gearbox_search::picking::pick(&mut s, p, [0.0, -10.0, 0.0]);
    }
    s.step(Controls::default());
    let picked = Body::from(s.body()).0.map(f32::to_bits);
    s.restore(&before_pick);
    s.step(Controls::default());
    assert_eq!(picked, Body::from(s.body()).0.map(f32::to_bits));

    let mut pose = Pose::from_body(body);
    pose.position[1] = ground.point[1] + 6.0;
    pose.velocity = [0.0; 3];
    place(&mut s, &Placement { pose, angular_velocity: [0.0; 3], reset_contacts: true }).unwrap();
    let start = s.save();
    let placed = s.save();
    for _ in 0..20 {
        s.step(Controls::default());
    }
    let falling = Pose::from_body(Body::from(s.body()));
    assert!(falling.position[1] < pose.position[1] - 0.1);
    assert!(falling.velocity[1] < -1.0);
    for _ in 0..380 {
        s.step(Controls::default());
        assert!(Body::from(s.body()).0.iter().all(|v| v.is_finite()));
    }
    let settled = Body::from(s.body());
    eprintln!("settled {:?}, velocity {:?}", settled.position(), settled.local_velocity());
    assert!(settled.position()[1] > ground.point[1]);
    assert!(settled.position()[1] < ground.point[1] + 1.0);
    assert!(settled.local_velocity()[1].abs() < 0.2);
    assert!(gearbox_search::tricks::Observation::read(&mut s, settled).wheels.iter().any(|v| *v));
    let mut fork = start.native().unwrap();
    fork.restore(&placed);
    s.restore(&placed);
    for _ in 0..200 {
        s.step(Controls::default());
        fork.step(Controls::default());
        assert_eq!(Body::from(s.body()).0.map(f32::to_bits), Body::from(fork.body()).0.map(f32::to_bits));
    }
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn fan_is_deterministic_and_snap_inputs_reach_the_returned_pose() {
    let mut s = open();
    advance(&mut s, 0);
    let start = s.save();
    let origin = start.clone();
    let mut config = fan::Config { rollouts: 24, horizon_ticks: 80, sample_every: 7, threads: 1, seed: 42 };
    let one = fan::generate(&origin, &config).unwrap();
    config.threads = 4;
    let four = fan::generate(&origin, &config).unwrap();
    for (a, b) in one.rollouts.iter().zip(&four.rollouts) {
        assert_eq!(a.inputs.controls, b.inputs.controls);
        assert_eq!(a.trajectory, b.trajectory);
    }
    assert_ne!(four.rollouts[0].end, four.rollouts[1].end);
    let mut desired = pose_goal(four.rollouts[0].end);
    desired.pose.position[0] += 0.25;
    let snapped = fan::snap(&desired, &four).unwrap().unwrap();
    assert!(snapped.tick > 0);
    let mut n = origin.native().unwrap();
    for c in snapped.inputs.controls {
        n.step(c);
    }
    assert_eq!(Pose::from_body(Body::from(n.body())), snapped.pose);
    assert!(desired.error(snapped.pose) <= 0.25f64.powi(2) + 1e-9);
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn ordered_mixed_goals_and_receding_prefix_progress_use_real_trajectories() {
    let mut s = open();
    advance(&mut s, 0);
    let initial = Pose::from_body(Body::from(s.body()));
    let start = s.save();
    let origin = start.clone();
    let gas = Controls { gas: 1.0, ..Default::default() };
    for _ in 0..50 {
        s.step(gas);
    }
    let target = Pose::from_body(Body::from(s.body()));
    let base = Timeline { actions: Vec::new(), t0: 0, seed: None, controls: vec![gas; 80] };
    let objective = Objective::Goals(Goals {
        ordered: vec![
            Goal::Point { point: initial.position, radius: 0.5 },
            Goal::Pose { target: pose_goal(target) },
        ],
        fastest: false,
        speed_worth_ms: 0.0,
        conditions: Vec::new(),
        along: None,
        stay_within: None,
        track: None,
    });
    let score = evaluate(&mut origin.native().unwrap(), &base, &objective, 800).unwrap().score;
    assert!(score.reached(), "{score}");
    let reversed = Objective::Goals(Goals {
        ordered: vec![
            Goal::Pose { target: pose_goal(target) },
            Goal::Point { point: initial.position, radius: 0.5 },
        ],
        fastest: false,
        speed_worth_ms: 0.0,
        conditions: Vec::new(),
        along: None,
        stay_within: None,
        track: None,
    });
    assert!(!evaluate(&mut origin.native().unwrap(), &base, &reversed, 800).unwrap().score.reached());
    for method in [
        Method::HillClimb,
        Method::Population { width: 4, curve_ticks: 10 },
        Method::Receding { horizon_ticks: 40, commit_ticks: 20, width: 4, curve_ticks: 10 },
    ] {
        let mut config = Config {
            window_start_ms: 0,
            window_end_ms: 800,
            until_ms: 800,
            objective: objective.clone(),
            iterations: 16,
            threads: 1,
            batch_size: 8,
            seed: 42,
            mutation: Mutation::default(),
            method,
            evaluation: EvalOptions::default(),
        };
        let one = search_from(&origin, base.clone(), &config, |_| {}).unwrap();
        config.threads = 4;
        let four = search_from(&origin, base.clone(), &config, |_| {}).unwrap();
        assert_eq!(one.after, four.after);
        assert_eq!(one.inputs.controls, four.inputs.controls);
        assert!(four.after.score.reached());
        assert!(four.after.score.quality_cmp(&score) != Ordering::Less);
        if let Score::Goals { times, .. } = four.after.score {
            assert_eq!(times[0], 0);
        } else {
            panic!("wrong goal score");
        }
    }
    for _ in 50..70 {
        s.step(gas);
    }
    let far = Pose::from_body(Body::from(s.body()));
    let neutral =
        Timeline { actions: Vec::new(), t0: 0, seed: None, controls: vec![Controls::default(); 90] };
    let tight = Objective::Goals(Goals {
        ordered: vec![
            Goal::Point { point: initial.position, radius: 0.05 },
            Goal::Pose { target: pose_goal(far) },
        ],
        fastest: false,
        speed_worth_ms: 0.0,
        conditions: Vec::new(),
        along: None,
        stay_within: None,
        track: None,
    });
    let config = Config {
        window_start_ms: 0,
        window_end_ms: 900,
        until_ms: 900,
        objective: tight,
        iterations: 144,
        threads: 4,
        batch_size: 16,
        seed: 42,
        mutation: Mutation::default(),
        method: Method::Receding { horizon_ticks: 30, commit_ticks: 10, width: 4, curve_ticks: 10 },
        evaluation: EvalOptions::default(),
    };
    let receding = search_from(&origin, neutral, &config, |_| {}).unwrap();
    assert!(
        matches!(&receding.before.score, Score::Goals { times, .. } if *times == [0]),
        "{}",
        receding.before.score
    );
    assert!(receding.after.score.reached(), "{}", receding.after.score);
    let mut gate = gearbox_search::gate::gates(&mut origin.native().unwrap()).unwrap()[0].clone();
    gate.waypoint = u32::MAX;
    let mixed = Objective::Goals(Goals {
        ordered: vec![
            Goal::Point { point: initial.position, radius: 0.5 },
            Goal::Gate { ordinal: 1, gate, also: Vec::new() },
        ],
        fastest: false,
        speed_worth_ms: 0.0,
        conditions: Vec::new(),
        along: None,
        stay_within: None,
        track: None,
    });
    let result = evaluate(&mut origin.native().unwrap(), &base, &mixed, 800).unwrap();
    assert!(matches!(result.score, Score::Goals {times,..} if times == [0]));
}

#[test]
#[ignore = "local packs, TMX 54999 and recorded inputs; run under nice"]
fn mixed_goals_credit_real_recorded_gate_crossings_in_order() {
    let data = fixtures();
    let file = data.join("inputs/k54999.txt.in");
    let inputs = Timeline::read(&file).unwrap();
    let mut s =
        gearbox_search::session::open(&packs(), &data.join("tmx/kacky/54999_uberbug.Map.Gbx"), Some(&inputs))
            .unwrap();
    let initial = Pose::from_body(Body::from(s.body()));
    let gate = gearbox_search::gate::reference_gate(&mut s, 1, &inputs, inputs.end_ms())
        .unwrap()
        .expect("recorded finish");
    let objective = Objective::Goals(Goals {
        ordered: vec![
            Goal::Point { point: initial.position, radius: 0.5 },
            Goal::Pose { target: pose_goal(initial) },
            Goal::Gate { ordinal: 1, gate, also: Vec::new() },
            Goal::Waypoint { ordinal: 1 },
            Goal::Finish,
        ],
        fastest: false,
        speed_worth_ms: 0.0,
        conditions: Vec::new(),
        along: None,
        stay_within: None,
        track: None,
    });
    let result = evaluate(&mut s, &inputs, &objective, inputs.end_ms()).unwrap();
    eprintln!("recorded mixed goals: {}", result.score);
    assert!(result.score.reached());
    assert!(matches!(result.score, Score::Goals { times, .. } if times == [-1530, -1530, 5509, 5509, 5509]));
}

#[test]
#[ignore = "local packs, TMX 54999 and recorded inputs; run under nice"]
fn a_crossing_in_the_tick_that_reaches_a_point_counts_after_it() {
    let data = fixtures();
    let file = data.join("inputs/k54999.txt.in");
    let inputs = Timeline::read(&file).unwrap();
    let mut s =
        gearbox_search::session::open(&packs(), &data.join("tmx/kacky/54999_uberbug.Map.Gbx"), Some(&inputs))
            .unwrap();
    let start = s.save();
    let (mut at_finish, mut after_finish) = (None, None);
    for c in &inputs.controls {
        s.step(*c);
        let finished = s.race().is_some_and(|r| r.finish.is_some());
        if finished && at_finish.is_none() {
            at_finish = Some((s.race_ms(), Body::from(s.body()).position()));
        } else if finished {
            after_finish = Some((s.race_ms(), Body::from(s.body()).position()));
            break;
        }
    }
    let ((ms, point), (later_ms, later)) = (at_finish.unwrap(), after_finish.unwrap());
    let finish = s.race().unwrap().finish.unwrap() as i32;
    assert!(finish > ms - 10 && finish < ms, "{finish} vs {ms}");
    let score = |point: [f32; 3], s: &mut Session| {
        s.restore(&start);
        let goals = Objective::Goals(Goals {
            ordered: vec![Goal::Point { point, radius: 0.01 }, Goal::Finish],
            fastest: false,
            speed_worth_ms: 0.0,
            conditions: Vec::new(),
            along: None,
            stay_within: None,
            track: None,
        });
        evaluate(s, &inputs, &goals, inputs.end_ms()).unwrap().score
    };
    let same = score(point, &mut s);
    assert!(matches!(&same, Score::Goals { times, .. } if *times == [ms, finish]), "{same}");
    let next = score(later, &mut s);
    assert!(matches!(&next, Score::Goals { times, .. } if *times == [later_ms]), "{next}");
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn placement_velocities_are_world_space_and_rotation_evolves_physically() {
    let mut s = open();
    advance(&mut s, 0);
    let q = std::f32::consts::FRAC_1_SQRT_2;
    let pose =
        Pose { position: [780.0, 30.0, 760.0], rotation: [q, 0.0, q, 0.0], velocity: [7.0, -2.0, 1.0] };
    let clock = s.race_ms();
    place(&mut s, &Placement { pose, angular_velocity: [0.0, 0.5, 0.0], reset_contacts: true }).unwrap();
    assert_eq!(s.race_ms(), clock);
    s.step(Controls::default());
    let after = Pose::from_body(Body::from(s.body()));
    assert!((after.position[0] - pose.position[0] - 0.07).abs() < 0.01, "{after:?}");
    assert!((after.position[2] - pose.position[2] - 0.01).abs() < 0.01, "{after:?}");
    assert!(after.position[1] < pose.position[1] - 0.015);
    let angle = gearbox_search::pose::rotation_distance(pose.rotation, after.rotation);
    assert!((0.001..0.02).contains(&angle), "angle {angle}");
    let pose =
        Pose { position: [800.0, 30.0, 760.0], rotation: [q, 0.0, -q, 0.0], velocity: [-5.0, -1.0, 2.0] };
    place(&mut s, &Placement { pose, angular_velocity: [0.0, 0.0, 0.5], reset_contacts: false }).unwrap();
    s.step(Controls::default());
    let after = Body::from(s.body());
    assert!(after.0.iter().all(|v| v.is_finite()));
    assert!(after.position()[0] < pose.position[0] - 0.04);
    assert!(after.position()[2] > pose.position[2] + 0.015);
}

#[test]
#[ignore = "local packs and TMX 54999; 2000 rollouts, four threads; run under nice"]
fn snapped_variant_refinement_reaches_landing_pose_earlier() {
    let mut s = open();
    advance(&mut s, 0);
    let placement: Placement = serde_json::from_str(include_str!("fixtures/drop-start.json")).unwrap();
    let target: PoseGoal = serde_json::from_str(include_str!("fixtures/drop-goal.json")).unwrap();
    place(&mut s, &placement).unwrap();
    let start = s.save();
    let origin = start.clone();
    let fan = fan::generate(
        &origin,
        &fan::Config { rollouts: 2000, horizon_ticks: 100, sample_every: 5, threads: 4, seed: 42 },
    )
    .unwrap();
    let snap = fan::snap(&target, &fan).unwrap().unwrap();
    let mut seed = snap.inputs;
    seed.controls.resize(100, Controls::default());
    let blank = Timeline { actions: Vec::new(), t0: 0, seed: None, controls: vec![Controls::default(); 100] };
    let config = Config {
        window_start_ms: 0,
        window_end_ms: 1000,
        until_ms: 1000,
        objective: Objective::Goals(Goals {
            ordered: vec![Goal::Pose { target }],
            fastest: false,
            speed_worth_ms: 0.0,
            conditions: Vec::new(),
            along: None,
            stay_within: None,
            track: None,
        }),
        iterations: 256,
        threads: 4,
        batch_size: 64,
        seed: 42,
        mutation: Mutation::default(),
        method: Method::Population { width: 8, curve_ticks: 10 },
        evaluation: EvalOptions { full_rollout: true, ..Default::default() },
    };
    let scratch = search_from(&origin, blank, &config, |_| {}).unwrap();
    let refined = search_from(&origin, seed, &config, |_| {}).unwrap();
    eprintln!(
        "fan {:.1} rollouts/s; scratch {}; refined {}",
        2000.0 / fan.elapsed.as_secs_f64(),
        scratch.after.score,
        refined.after.score
    );
    assert_eq!(scratch.evaluated_ticks, 25600);
    assert_eq!(refined.evaluated_ticks, 25600);
    assert!(scratch.after.score.reached() && refined.after.score.reached());
    assert_eq!(refined.after.score.quality_cmp(&scratch.after.score), Ordering::Greater);
}

#[test]
#[ignore = "local packs, TMX 54999 and recorded inputs; run under nice"]
fn full_rollouts_stop_counting_signatures_at_the_crossing() {
    let data = fixtures();
    let file = data.join("inputs/k54999.txt.in");
    let inputs = Timeline::read(&file).unwrap();
    let mut s =
        gearbox_search::session::open(&packs(), &data.join("tmx/kacky/54999_uberbug.Map.Gbx"), Some(&inputs))
            .unwrap();
    let gate = gearbox_search::gate::reference_gate(&mut s, 1, &inputs, inputs.end_ms())
        .unwrap()
        .expect("recorded finish");
    while s.race_ms() < 5450 {
        let c = inputs.controls[inputs.tick(s.race_ms()).unwrap()];
        s.step(c);
    }
    let mut pose = Pose::from_body(Body::from(s.body()));
    let (h, c) = (0.45f32.sin(), 0.45f32.cos());
    let q = pose.rotation;
    pose.rotation = [q[0] * c - q[1] * h, q[1] * c + q[0] * h, q[2] * c + q[3] * h, q[3] * c - q[2] * h];
    pose.position[1] += 2.0;
    place(&mut s, &Placement { pose, angular_velocity: [0.0; 3], reset_contacts: true }).unwrap();
    let gas = Timeline {
        actions: Vec::new(),
        t0: s.race_ms(),
        seed: inputs.seed,
        controls: vec![Controls { gas: 1.0, ..Default::default() }; 150],
    };
    let start = s.save();
    let objective = Objective::Gate { ordinal: 1, gate, time_penalty: 1.0 };
    let mut run = |full_rollout| {
        s.restore(&start);
        let options = EvalOptions { full_rollout, trick_bonus: 1.0, trace: true, ..Default::default() };
        gearbox_search::objective::evaluate_with(&mut s, &gas, &objective, gas.end_ms(), options).unwrap()
    };
    let first = run(false);
    let full = run(true);
    let Score::Guided { time_ms: Some(hit), .. } = first.score else {
        panic!("gate not reached: {}", first.score);
    };
    let after_hit = full
        .trace
        .iter()
        .filter(|s| {
            let o = s.observation;
            s.ms > hit as i32
                && o.pitch < -35f32.to_radians()
                && (o.wheels[0] || o.wheels[1])
                && !o.wheels[2]
                && !o.wheels[3]
                && o.speed > 5.0
        })
        .count();
    assert!(after_hit > 0);
    assert!(full.ticks > first.ticks);
    assert_eq!(first.score, full.score);
    assert_eq!(first.events, full.events);
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn a_placed_origin_searches_like_a_saved_placed_start() {
    let mut s = open();
    advance(&mut s, 0);
    let placement: Placement = serde_json::from_str(include_str!("fixtures/drop-start.json")).unwrap();
    let start = s.save();
    let unplaced = start.clone();
    let placed_origin =
        gearbox_search::pose::PlacedOrigin { origin: &unplaced, placement: placement.clone() };
    place(&mut s, &placement).unwrap();
    let placed_start = s.save();
    let saved = placed_start.clone();
    let (mut a, mut b) = (placed_origin.native().unwrap(), saved.native().unwrap());
    assert_eq!(a.race_ms(), b.race_ms());
    for _ in 0..100 {
        assert_eq!(Body::from(a.body()).0.map(f32::to_bits), Body::from(b.body()).0.map(f32::to_bits));
        a.step(Controls::default());
        b.step(Controls::default());
    }
    let target: PoseGoal = serde_json::from_str(include_str!("fixtures/drop-goal.json")).unwrap();
    let config = Config {
        window_start_ms: 0,
        window_end_ms: 1000,
        until_ms: 1000,
        objective: Objective::Goals(Goals {
            ordered: vec![Goal::Pose { target }],
            fastest: false,
            speed_worth_ms: 0.0,
            conditions: Vec::new(),
            along: None,
            stay_within: None,
            track: None,
        }),
        iterations: 64,
        threads: 4,
        batch_size: 32,
        seed: 42,
        mutation: Mutation::default(),
        method: Method::Population { width: 8, curve_ticks: 10 },
        evaluation: EvalOptions::default(),
    };
    let blank = Timeline { actions: Vec::new(), t0: 0, seed: None, controls: vec![Controls::default(); 100] };
    let via_placement = search_from(&placed_origin, blank.clone(), &config, |_| {}).unwrap();
    let via_save = search_from(&saved, blank, &config, |_| {}).unwrap();
    assert_eq!(via_placement.after, via_save.after);
    assert_eq!(via_placement.inputs.controls, via_save.inputs.controls);
}

fn open_map(name: &str) -> Session {
    let data = fixtures();
    gearbox_search::session::open(&packs(), &data.join("tmx/kacky").join(name), None).unwrap()
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn an_ordinal_with_alternate_finishes_is_reported_as_ambiguous() {
    let mut s = open_map("54999_uberbug.Map.Gbx");
    let error = gearbox_search::gate::target_gate(&mut s, 1).unwrap_err().to_string();
    assert!(error.contains("4 alternate finishes"), "{error}");
    assert!(error.contains("waypoint IDs 1, 2, 3, 4"), "{error}");
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn a_try_keeps_its_range_then_follows_the_trail_with_the_gas_held() {
    let s = open_map("54999_uberbug.Map.Gbx");
    let start = s.save();
    let origin = start.clone();
    let k0 = (-s.race_ms() / 10) as usize;
    let gas = Controls { gas: 1.0, brake: 0.0, steer: 0.0 };
    let run = Timeline { actions: Vec::new(), t0: s.race_ms(), seed: None, controls: vec![gas; k0 + 400] };
    let (from, to, until) = (k0 + 100, k0 + 150, k0 + 400);
    let trail = gearbox_search::tries::drive_on(&origin, &run, from, until, until, &[]).unwrap().trail;
    let mut line = run.clone();
    for c in &mut line.controls[from..to] {
        c.steer = -1.0;
    }
    let followed = gearbox_search::tries::drive_on(&origin, &line, from, to, until, &trail).unwrap();
    let left_alone = gearbox_search::tries::drive_on(&origin, &line, from, until, until, &[]).unwrap();
    assert_eq!(followed.line.controls[..to], line.controls[..to]);
    assert!(followed.line.controls[to..].iter().all(|c| c.gas == 1.0 && c.brake == 0.0));
    let off = |t: &[[f32; 3]]| {
        let p = t.last().unwrap();
        trail
            .iter()
            .map(|q| ((p[0] - q[0]).powi(2) + (p[2] - q[2]).powi(2)).sqrt())
            .fold(f32::INFINITY, f32::min)
    };
    assert!(
        off(&followed.trail) < 0.5 * off(&left_alone.trail),
        "followed {:.1} m off, left alone {:.1} m",
        off(&followed.trail),
        off(&left_alone.trail)
    );
    assert!(followed.speed_at_end > 0.0);
}

#[test]
#[ignore = "local packs and TMX 54999; run under nice"]
fn the_general_aim_agrees_with_the_speed_objective_and_its_best_is_no_worse() {
    let s = open_map("54999_uberbug.Map.Gbx");
    let start = s.save();
    let origin = start.clone();
    let k0 = (-s.race_ms() / 10) as usize;
    let line = Timeline {
        actions: Vec::new(),
        t0: s.race_ms(),
        seed: None,
        controls: vec![Controls { gas: 1.0, brake: 0.0, steer: 0.3 }; k0 + 300],
    };
    let at_ms = 2000;
    let score = |objective: Objective| {
        let mut n = origin.native().unwrap();
        evaluate(&mut n, &line, &objective, at_ms).unwrap().score
    };
    let Score::Speed { speed_squared } = score(Objective::Speed { at_ms }) else { panic!() };
    let Score::Aim { held, value, .. } =
        score(Objective::Aim { at_ms, aim: gearbox_search::aim::Aim::fastest() })
    else {
        panic!()
    };
    assert!(held);
    assert!(
        (value - speed_squared.sqrt() * 3.6).abs() < 1e-6,
        "{value} km/h against {} km/h",
        speed_squared.sqrt() * 3.6
    );
    let low = |judged| gearbox_search::aim::Aim {
        judged,
        measure: gearbox_search::aim::Measure {
            quantity: gearbox_search::aim::Quantity::Height,
            higher: false,
        },
        conditions: Vec::new(),
        then: None,
    };
    let (Score::Aim { value: end, .. }, Score::Aim { value: best, .. }) = (
        score(Objective::Aim { at_ms, aim: low(gearbox_search::aim::Judged::End) }),
        score(Objective::Aim { at_ms, aim: low(gearbox_search::aim::Judged::Best) }),
    ) else {
        panic!()
    };
    assert!(best >= end, "best {best} below the end's {end}");
}
