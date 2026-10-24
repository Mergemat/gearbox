use std::path::{Path, PathBuf};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use crate::engine::frame::Frame;
use crate::engine::input::InputFile;
use crate::engine::session::{Command, Engine, Event, InputSel, Shared, Source};
use crate::{background, doc, optimize, project};

const TICKS: usize = 500;
const WAIT: Duration = Duration::from_secs(300);
const SCRUB_TICK: usize = 237;
const EDIT_TICK: usize = 257;

struct ProjectDir(PathBuf);

impl Drop for ProjectDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn wait_for(engine: &Engine, wake: &mpsc::Receiver<()>, ready: impl Fn(&Shared) -> bool) {
    let deadline = Instant::now() + WAIT;
    loop {
        {
            let shared = engine.shared.lock().unwrap();
            assert!(shared.stop.is_none(), "simulation stopped: {:?}", shared.stop);
            if ready(&shared) {
                return;
            }
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        assert!(!remaining.is_zero(), "editor session timed out");
        match wake.recv_timeout(remaining.min(Duration::from_millis(20))) {
            Ok(()) | Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => panic!("editor session disconnected"),
        }
    }
}

fn compare(frame: &Frame, expected: &gearbox_trace::TickState) {
    assert_eq!(gearbox_trace::Body::from_floats(frame.body), expected.body, "body at {} ms", frame.race_ms);
    assert_eq!(frame.race_ms, expected.race_ms);
    for (wheel, expected) in frame.wheels.iter().zip(expected.vehicle.wheels) {
        assert_eq!(wheel.contact, expected.contact != 0);
        assert_eq!(wheel.material, expected.material);
        assert_eq!(wheel.spin.to_bits(), expected.spin.0);
        assert_eq!(wheel.steer.to_bits(), expected.steer.0);
        assert_eq!(wheel.position.map(f32::to_bits), expected.position.map(|x| x.0));
    }
    let race = expected.race.as_ref().unwrap();
    assert_eq!(usize::from(frame.waypoints), race.checkpoints.len());
    assert_eq!(frame.finish, race.finish);
    let last = race.waypoints.last().copied().zip(race.checkpoints.last().copied());
    assert_eq!(frame.last_crossing, last);
}

fn check_frames(frames: &[Frame], origin: &gearbox_engine::Savestate, inputs: &InputFile) {
    let mut golden = origin.fork();
    golden.seed = inputs.seed;
    for (tick, frame) in frames.iter().enumerate() {
        compare(frame, &gearbox_trace::typed::state(&golden));
        if tick + 1 < frames.len() {
            golden.step(inputs.at(golden.race_ms()));
        }
    }
}

#[test]
fn project_session_scrub_record_and_search_match_golden_typed_engine() {
    let Ok(game) = gearbox_packs::install::Game::locate(None) else {
        eprintln!("skipped headless check: {}", gearbox_packs::install::NOT_FOUND);
        return;
    };
    let factory = gearbox_trace::typed::TypedFactory::new(game.packs.clone());
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let map = root.join("tests/golden/maps/pf-364816.Map.Gbx").canonicalize().unwrap();
    let trace =
        gearbox_trace::Trace::read(&root.join("tests/golden/pf-364816/rec-pf364816.trace.zst")).unwrap();
    assert!(trace.states.len() > TICKS);
    let inputs = InputFile {
        actions: Vec::new(),
        t0: trace.case.start_ms,
        seed: trace.case.seed,
        controls: trace
            .case
            .steps
            .iter()
            .map(|c| gearbox_engine::Controls { gas: c.gas, brake: c.brake, steer: c.steer })
            .collect(),
    };
    let temp =
        ProjectDir(std::env::temp_dir().join(format!("gearbox-editor-headless-{}", std::process::id())));
    let home = temp.0.join("editor");
    let path = project::new_path(&home, "Typed session");
    let document = doc::Doc::new(inputs, TICKS, doc::Origin::Blank);
    let spec = optimize::Spec {
        window: 150..230,
        by_time: false,
        attempts: 16,
        threads: 1,
        seed: 42,
        ..Default::default()
    };
    project::save(
        &path,
        &home,
        "Typed session",
        &project::Finishes::default(),
        &map,
        &document,
        &spec,
        &crate::lab::RouteSpec::default(),
        &[],
    )
    .unwrap();
    let loaded = project::load(&path, &home).unwrap();
    let mut inputs = loaded.doc.inputs.as_ref().clone();
    let (tx, wake) = mpsc::channel();
    let engine = Engine::spawn(
        game.packs.clone(),
        Arc::new(move || {
            let _ = tx.send(());
        }),
    );
    engine.send(Command::Open {
        source: Source(loaded.map.clone()),
        inputs: InputSel::Memory(inputs.clone(), loaded.name),
        len: Some(loaded.doc.len),
    });
    let Event::Opened { origin, world, world_error, .. } = engine.events.recv_timeout(WAIT).unwrap() else {
        panic!("editor failed to open project map")
    };
    assert!(world_error.is_none(), "{world_error:?}");
    assert!(!world.tris.is_empty());
    let golden = factory.session(&map, inputs.t0, inputs.seed, None).unwrap().save();
    wait_for(&engine, &wake, |s| s.frames.len() == TICKS + 1);
    {
        let shared = engine.shared.lock().unwrap();
        assert_eq!(shared.savestates, 6);
        check_frames(&shared.frames, &golden, &inputs);
        for (frame, expected) in shared.frames.iter().zip(&trace.states) {
            compare(frame, expected);
        }
    }
    engine.send(Command::Probe { tick: SCRUB_TICK, branches: true });
    wait_for(&engine, &wake, |s| s.probe.is_some());
    {
        let shared = engine.shared.lock().unwrap();
        let probe = shared.probe.as_ref().unwrap();
        assert_eq!((probe.tick, probe.stepped), (SCRUB_TICK, 37));
        assert_eq!(probe.branches.len(), 3);
        let saved = background::reach(&origin, &inputs, SCRUB_TICK, None).unwrap();
        for (branch, steer) in probe.branches.iter().zip([-1.0, 0.0, 1.0]) {
            let mut runner = saved.fork();
            let controls = gearbox_engine::Controls { steer, ..inputs.at(runner.race_ms()) };
            assert_eq!(branch.points.len(), 151);
            for point in &branch.points {
                assert_eq!(point.map(f32::to_bits), runner.body().pos.to_array().map(f32::to_bits));
                runner.step(controls);
            }
        }
    }
    let mut recorder = crate::engine::record::Recorder::new(EDIT_TICK);
    recorder.write(
        &mut inputs,
        trace.case.start_ms,
        EDIT_TICK + 20,
        crate::engine::record::Keys { brake: true, right: true, ..Default::default() },
    );
    engine.send(Command::Simulate {
        inputs: Arc::new(inputs.clone()),
        from_tick: EDIT_TICK,
        len: TICKS,
        hold: None,
    });
    wait_for(&engine, &wake, |s| {
        s.frames.len() == TICKS + 1 && s.generation > 3 && s.frames[EDIT_TICK + 1].controls.brake == 1.0
    });
    check_frames(&engine.shared.lock().unwrap().frames, &golden, &inputs);
    inputs.seed = Some(inputs.seed.unwrap_or(0) ^ 12345);
    engine.send(Command::Simulate {
        inputs: Arc::new(inputs.clone()),
        from_tick: TICKS,
        len: TICKS,
        hold: Some(0),
    });
    wait_for(&engine, &wake, |s| s.frames.len() == 1);
    engine.send(Command::Simulate { inputs: Arc::new(inputs.clone()), from_tick: 0, len: TICKS, hold: None });
    wait_for(&engine, &wake, |s| s.frames.len() == TICKS + 1);
    check_frames(&engine.shared.lock().unwrap().frames, &golden, &inputs);
    let start = doc::StartState {
        tick: 245,
        pos: [500.0, 30.0, 500.0],
        ypr: [10.0, 0.0, 0.0],
        velocity: [0.0, 0.0, 20.0],
        angular: [0.0; 3],
        reset_contacts: true,
    };
    engine.send(Command::SetStart { start: Some((start.tick, start.placement())) });
    engine.send(Command::Probe { tick: TICKS, branches: false });
    wait_for(&engine, &wake, |s| s.probe.as_ref().is_some_and(|p| p.tick == TICKS));
    let mut compare_runner = origin.fork();
    let (compared, error) =
        background::simulate(&mut compare_runner, &inputs, TICKS, Some(&(start.tick, start.placement())));
    assert!(error.is_none(), "{error:?}");
    let actual = engine.shared.lock().unwrap().frames.clone();
    for (a, b) in actual.iter().zip(&compared) {
        assert_eq!(a.body.map(f32::to_bits), b.body.map(f32::to_bits));
    }
    let (tx, updates) = mpsc::channel();
    let mut search = optimize::Search::default();
    search.start(
        optimize::Request {
            origin: origin.clone(),
            inputs: inputs.clone(),
            race_ms0: trace.case.start_ms,
            spec: loaded.search.unwrap(),
            objective: gearbox_search::objective::Objective::Position {
                at_ms: trace.case.start_ms + 3000,
                point: [500.0, 20.0, 500.0],
            },
            until_ms: trace.case.start_ms + 3000,
            locks: Vec::new(),
            start: None,
            origin_tick: 150,
            start_line: None,
            world: None,
        },
        Arc::new(move || {
            let _ = tx.send(());
        }),
    );
    let deadline = Instant::now() + WAIT;
    while search.running() {
        updates
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .expect("editor search timed out");
    }
    let live = search.live.lock().unwrap();
    assert!(live.error.is_none(), "{:?}", live.error);
    assert!(live.attempts >= 16);
    assert!(!live.candidates.is_empty(), "search returned no candidates");
    for candidate in &live.candidates {
        let mut runner = origin.fork();
        let (frames, error) = background::simulate(&mut runner, &candidate.inputs, 300, None);
        assert!(error.is_none(), "{error:?}");
        check_frames(&frames, &golden, &candidate.inputs);
    }
    eprintln!(
        "EXACT: 501 golden frames; scrub + 3 branches; recording + seed edits; \
        placed compare run; {} search tries, {} candidate replays",
        live.attempts,
        live.candidates.len()
    );
}
