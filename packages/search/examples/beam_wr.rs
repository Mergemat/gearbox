use gearbox_engine::Session;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use anyhow::{Context, Result};
use gearbox_engine::Controls;
use gearbox_search::beam::{BeamConfig, build_line};
use gearbox_search::goals::{Goal, Goals};
use gearbox_search::input::Timeline;
use gearbox_search::search::Origin;

#[derive(clap::Parser)]
struct Args {
    #[arg(long)]
    game: Option<PathBuf>,
    map: PathBuf,
    inputs: PathBuf,
    mode: String,
    width: Option<usize>,
    chunk: Option<usize>,
    threads: Option<usize>,
    #[arg(long, default_value_t = 1)]
    reach: i32,
    #[arg(long)]
    rest_speed: Option<f64>,
    #[arg(long, value_delimiter = ',')]
    actions: Vec<f32>,
    #[arg(long)]
    first: bool,
    #[arg(long)]
    cap: Option<f64>,
    #[arg(long)]
    rollback: Option<f64>,
    #[arg(long)]
    passes: Option<usize>,
    #[arg(long)]
    explore: Option<usize>,
    #[arg(long, value_delimiter = ',', num_args = 3)]
    coarse: Option<Vec<f32>>,
    #[arg(long, value_delimiter = ',', num_args = 3)]
    cell: Option<Vec<f32>>,
    #[arg(long)]
    trace: bool,
    #[arg(long)]
    section: Option<usize>,
    #[arg(long, default_value_t = 0)]
    from: usize,
    #[arg(long)]
    to: Option<usize>,
    #[arg(long)]
    guide: Option<PathBuf>,
    #[arg(long)]
    finish_only: bool,
    #[arg(long, default_value_t = 1.0)]
    off: f64,
}

fn track(n: &mut Session, goals: &[Goal], reach: i32) -> Arc<gearbox_search::track::TrackFields> {
    let world = n.static_triangles();
    let dry = |t: &&gearbox_engine::consumer::PhysicalTriangle| t.surface != 13;
    let cells = Arc::new(gearbox_search::track::Cells::build(world.iter().filter(dry).map(|t| t.v), 3.0, 4));
    let fields = std::thread::scope(|scope| {
        let jobs: Vec<_> = goals
            .iter()
            .map(|g| {
                let cells = cells.clone();
                scope.spawn(move || match g {
                    Goal::Gate { gate, .. } => {
                        let g = gate.clone();
                        let c = g
                            .boxes
                            .first()
                            .map_or([0.0; 3], |b| std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5));
                        Some(gearbox_search::track::GoalField::build(cells, c, move |p| g.distance(p) <= 1.5))
                    }
                    _ => None,
                })
            })
            .collect();
        jobs.into_iter().map(|j| j.join().unwrap()).collect()
    });
    Arc::new(gearbox_search::track::TrackFields { goals: fields, reach })
}

fn main() -> Result<()> {
    let args: Args = clap::Parser::parse();
    let packs = gearbox_packs::install::Game::locate(args.game.as_deref())?.packs;
    let inputs = Timeline::read(&args.inputs)?;
    let s = gearbox_world::open_file(
        &args.map,
        &packs,
        gearbox_world::OpenOptions { t0: inputs.t0, seed: inputs.seed, car: None },
    )?;
    let sections = args.mode == "sections";
    let mut config = BeamConfig::default();
    if let Some(w) = args.width {
        config.width = w;
    }
    if let Some(c) = args.chunk {
        config.chunk = c;
    }
    if let Some(t) = args.threads {
        config.threads = t;
    }
    if let Some(v) = args.rest_speed {
        config.rest_speed = v;
    }
    if !args.actions.is_empty() {
        config.actions = args.actions.iter().map(|&steer| Controls { gas: 1.0, brake: 0.0, steer }).collect();
    }
    config.first_arrival = args.first;
    if let Some(v) = args.cap {
        config.speed_cap = v;
    }
    if let Some(v) = args.rollback {
        config.rollback_ms = v;
    }
    if let Some(v) = args.passes {
        config.passes = v;
    }
    if let Some(v) = args.explore {
        config.explore = v;
    }
    if let Some(v) = &args.coarse {
        config.coarse = (v[0], v[1] as u32, v[2]);
    }
    if let Some(v) = &args.cell {
        (config.cell_m, config.sectors, config.kmh) = (v[0], v[1] as u32, v[2]);
    }
    let start = s.save();
    let wr = Timeline { t0: inputs.t0, seed: inputs.seed, controls: inputs.controls.clone() };

    let mut n = start.native()?;
    n.seed = wr.seed;
    let wr = if n.race_ms() < wr.t0 { wr.starting_at(n.race_ms())? } else { wr };
    let first = wr.index(n.race_ms())?;
    let mut crossings: Vec<(u32, i32, usize)> = Vec::new();
    let mut finish = None;
    for (k, c) in wr.controls.iter().enumerate().skip(first) {
        n.step(*c);
        let race = n.race().unwrap_or_default();
        while crossings.len() < race.checkpoints.len() {
            let ordinal = crossings.len() + 1;
            let w = gearbox_search::gate::crossing_waypoint(&mut n, ordinal)
                .context("crossing without a waypoint")?;
            crossings.push((w, race.checkpoints[ordinal - 1] as i32, k + 1));
        }
        if let Some(f) = race.finish {
            finish = Some(f as i32);
            break;
        }
    }
    let wr_ms = finish.context("the WR inputs don't finish here")?;
    println!("WR replays to {wr_ms} ms; splits {:?}", crossings.iter().map(|c| c.1).collect::<Vec<_>>());

    let mut n = start.native()?;
    let gates = gearbox_search::gate::gates(&mut n)?;
    let goal_list: Vec<Goal> = crossings
        .iter()
        .enumerate()
        .map(|(i, &(w, _, _))| {
            let mut gate = gates.iter().find(|g| g.waypoint == w).cloned().expect("crossed gate exists");
            let same: Vec<_> =
                gates.iter().filter(|g| g.kind == 1 && gate.kind == 1 && g.waypoint != w).collect();
            gate.boxes.extend(same.iter().flat_map(|g| g.boxes.clone()));
            Goal::Gate { ordinal: i + 1, gate, also: same.iter().map(|g| g.waypoint).collect() }
        })
        .collect();
    let t = Instant::now();
    let fields = track(&mut n, &goal_list, args.reach);
    eprintln!("track measured in {:.1} s", t.elapsed().as_secs_f32());
    if args.trace {
        let mut m = start.native()?;
        m.seed = wr.seed;
        let mut done = 0;
        for (k, c) in wr.controls.iter().enumerate().skip(first) {
            m.step(*c);
            while done < crossings.len() && crossings[done].2 <= k + 1 {
                done += 1;
            }
            if k % 10 == 0 && done < goal_list.len() {
                let b = gearbox_search::Body::from(m.body());
                let p = b.position();
                let d = fields.goals[done].as_ref().and_then(|f| f.distance(p));
                println!(
                    "trace {:6} goal {done} pos {:7.1} {:6.1} {:7.1}  track {:?} v {:?}",
                    m.race_ms(),
                    p[0],
                    p[1],
                    p[2],
                    d.map(|d| d as i32),
                    b.local_velocity()
                );
            }
            if done == goal_list.len() {
                break;
            }
        }
    }
    let state = n.save();
    let origin = state;

    let horizon = wr.controls.len() + 300;
    let base_from = |k: usize| {
        let mut b = wr.clone();
        b.controls.truncate(k);
        b.controls.resize(horizon, Controls { gas: 1.0, brake: 0.0, steer: 0.0 });
        b
    };
    let ranges: Vec<(usize, usize)> = if sections {
        (0..goal_list.len()).filter(|i| args.section.is_none_or(|o| o == *i)).map(|i| (i, i + 1)).collect()
    } else {
        vec![(args.from, args.to.unwrap_or(goal_list.len()))]
    };
    let guide = match &args.guide {
        Some(path) => {
            let g = Timeline::read(path)?;
            let mut g = if g.t0 > wr.t0 { g.starting_at(wr.t0)? } else { g };
            g.controls.resize(horizon, Controls::default());
            let mut m = origin.native()?;
            m.seed = g.seed;
            let mut line = vec![gearbox_search::Body::from(m.body()).position()];
            for c in &g.controls[g.index(m.race_ms())?..] {
                m.step(*c);
                line.push(gearbox_search::Body::from(m.body()).position());
                if m.race().and_then(|r| r.finish).is_some() {
                    break;
                }
            }
            println!("guide finishes at {:?} ms", m.race().and_then(|r| r.finish));
            Some((g, line))
        }
        None => None,
    };
    let mut total_ticks = 0u64;
    let all = Instant::now();
    for (from, to) in ranges {
        let from_k = if from == 0 { first } else { crossings[from - 1].2 };
        let goals = Goals {
            ordered: goal_list[..to].to_vec(),
            along: None,
            stay_within: None,
            track: Some(fields.clone()),
            fastest: false,
            speed_worth_ms: 0.0,
            conditions: Vec::new(),
        };
        let goals = if config.first_arrival && to == goal_list.len() {
            Goals { ordered: vec![Goal::Finish], track: None, ..goals }
        } else {
            goals
        };
        let goals = if args.finish_only {
            let cps = goal_list.len() as u32 - 1;
            Goals {
                ordered: vec![Goal::Finish],
                conditions: vec![gearbox_search::goals::Conditions {
                    min_crossings: Some(cps),
                    ..Default::default()
                }],
                ..goals
            }
        } else {
            goals
        };
        let (goals, base) = match &guide {
            Some((g, line)) => (
                Goals {
                    along: Some(gearbox_search::objective::ReferencePath {
                        points: line.clone(),
                        deviation_penalty: args.off,
                        stay_within: None,
                    }),
                    track: None,
                    ..goals
                },
                g.clone(),
            ),
            None => (goals, base_from(from_k)),
        };
        let t = Instant::now();
        let (mut ticks, mut chunks) = (0, 0);
        let built = build_line(&origin, &base, from_k, horizon, &goals, &config, &|| false, |p| {
            ticks = p.ticks;
            chunks += 1;
            if chunks % 5 == 0 {
                let (g, pos, kmh) = p.lead.unwrap_or_default();
                eprintln!(
                    "  pass {} {:6} ms  kept {:4}  lead eta {:7.0} goals {g} at {:7.1} {:6.1} {:7.1} {kmh:3.0} km/h  best {:?}",
                    p.pass, p.race_ms, p.kept, p.lead_eta_ms, pos[0], pos[1], pos[2], p.best
                );
            }
        })?;
        total_ticks += ticks;
        let wr_at = crossings[to - 1].1;
        let secs = t.elapsed().as_secs_f64();
        match built {
            Some(b) => {
                let at = *b.times.last().unwrap();
                println!(
                    "section {from}->{to}: built {at} ms at {:.0} km/h, WR {wr_at} ms: {:+} ms ({secs:.0} s, {} M ticks)",
                    b.speed_kmh,
                    at - wr_at,
                    ticks / 1_000_000
                );
            }
            None => println!("section {from}->{to}: nothing reached it (WR {wr_at} ms; {secs:.0} s)"),
        }
    }
    println!("total {:.0} s, {} M ticks", all.elapsed().as_secs_f64(), total_ticks / 1_000_000);
    Ok(())
}
