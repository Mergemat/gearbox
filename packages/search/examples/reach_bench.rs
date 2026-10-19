use gearbox_engine::Savestate;
use gearbox_search::Body;
use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context, Result};
use gearbox_engine::Controls;
use gearbox_search::beam::{BeamConfig, build_line};
use gearbox_search::goals::{Goal, Goals};
use gearbox_search::input::Timeline;
use gearbox_search::objective::{EvalOptions, Objective, Score, evaluate_with};
use gearbox_search::pose::{Pose, PoseGoal};
use gearbox_search::search::Origin;

#[derive(clap::Parser)]
struct Args {
    #[arg(long)]
    game: Option<PathBuf>,
    map: PathBuf,
    inputs: PathBuf,
    #[arg(default_value_t = 2.0)]
    ahead: f64,
    #[arg(default_value_t = 3.0)]
    every: f64,
    width: Option<usize>,
    #[arg(long, default_value_t = 2.0)]
    vtol: f32,
    #[arg(long)]
    no_velocity: bool,
    #[arg(long)]
    from: Option<f64>,
    #[arg(long)]
    track: bool,
    #[arg(long)]
    offset: Option<f32>,
    #[arg(long)]
    speed: Option<f32>,
    #[arg(long)]
    trace: Option<i32>,
}

fn target(body: Body, velocity: Option<f32>) -> PoseGoal {
    PoseGoal {
        pose: Pose::from_body(body),
        position_tolerance: [1.0; 3],
        angle_tolerance: 10f32.to_radians(),
        velocity_tolerance: velocity.map(|v| [v; 3]),
        position_weights: [1.0; 3],
        angle_weight: 10.0,
        velocity_weights: [0.1; 3],
    }
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
    let (ahead, every) = (args.ahead, args.every);
    let mut config = BeamConfig { explore: 128, ..Default::default() };
    if let Some(w) = args.width {
        config.width = w;
        config.explore = config.width / 2;
    }
    let velocity = (!args.no_velocity).then_some(args.vtol);
    let start = s.save();
    let run = Timeline { t0: inputs.t0, seed: inputs.seed, controls: inputs.controls.clone() };
    let mut n = start.native()?;
    n.seed = run.seed;
    let run = if n.race_ms() < run.t0 { run.starting_at(n.race_ms())? } else { run };
    let first = run.index(n.race_ms())?;
    let ahead_k = (ahead * 100.0).round() as usize;
    let mut bodies: Vec<Body> = Vec::new();
    let mut saved: Vec<(usize, i32, Savestate)> = Vec::new();
    let only: Option<i32> = args.from.map(|s| (s * 1000.0) as i32);
    let mut next_ms = only.unwrap_or(1000);
    for (k, c) in run.controls.iter().enumerate().skip(first) {
        if n.race_ms() >= next_ms {
            saved.push((k, n.race_ms(), n.save()));
            next_ms = if only.is_some() { i32::MAX } else { next_ms + (every * 1000.0) as i32 };
        }
        n.step(*c);
        bodies.push(gearbox_search::Body::from(n.body()));
        if n.race().and_then(|r| r.finish).is_some() {
            break;
        }
    }
    let body_after = |k: usize| bodies.get(k - first).copied();
    let cells = if args.track {
        let m = start.native()?;
        let world = m.static_triangles();
        let dry = |t: &&gearbox_engine::consumer::PhysicalTriangle| t.surface != 13;
        Some(std::sync::Arc::new(gearbox_search::track::Cells::build(
            world.iter().filter(dry).map(|t| t.v),
            3.0,
            4,
        )))
    } else {
        None
    };
    let (mut reached, mut total, mut late_sum) = (0, 0, 0i64);
    let all = Instant::now();
    for (k, ms, state) in saved {
        let Some(goal_body) = body_after(k + ahead_k - 1) else { break };
        total += 1;
        let mut goal = target(goal_body, velocity);
        if let Some(off) = args.offset {
            let r = goal_body.rotation();
            for i in 0..3 {
                goal.pose.position[i] += r[3 * i] * off;
            }
        }
        if let Some(f) = args.speed {
            goal.pose.velocity = goal.pose.velocity.map(|v| v * f);
        }
        let track = cells.as_ref().map(|c| {
            let (at, tol) = (goal.pose.position, goal.position_tolerance);
            let field = gearbox_search::track::GoalField::build(c.clone(), at, move |p| {
                (0..3).all(|i| (p[i] - at[i]).abs() <= tol[i] + 1.5)
            });
            std::sync::Arc::new(gearbox_search::track::TrackFields { goals: vec![Some(field)], reach: 2 })
        });
        let goals = Goals {
            ordered: vec![Goal::Pose { target: goal }],
            along: None,
            stay_within: None,
            track,
            fastest: false,
            speed_worth_ms: 0.0,
            conditions: Vec::new(),
        };
        let origin = state;
        let until_k = k + 3 * ahead_k;
        let mut base = run.clone();
        base.controls.truncate(k);
        base.controls.resize(until_k, Controls { gas: 1.0, brake: 0.0, steer: 0.0 });
        let t = Instant::now();
        let trace = args.trace == Some(ms / 1000);
        let want_v = Pose::from_body(goal_body).velocity;
        let built = build_line(&origin, &base, k, until_k, &goals, &config, &|| false, |p| {
            if trace && let Some((_, pos, kmh)) = p.lead {
                let tp = Pose::from_body(goal_body).position;
                let d = (0..3).map(|i| (pos[i] - tp[i]).powi(2)).sum::<f32>().sqrt();
                println!(
                    "  {:6} ms  lead {d:6.1} m off ({:+.1} {:+.1} {:+.1}) at {kmh:4.0} km/h (target {:4.0} km/h)  eta {:.0}",
                    p.race_ms,
                    pos[0] - tp[0],
                    pos[1] - tp[1],
                    pos[2] - tp[2],
                    want_v.iter().map(|v| v * v).sum::<f32>().sqrt() * 3.6,
                    p.lead_eta_ms
                );
            }
        })?;
        let secs = t.elapsed().as_secs_f64();
        let want = (ahead * 1000.0).round() as i64;
        match built {
            Some(b) => {
                let took = i64::from(*b.times.last().context("no time")?) - i64::from(ms);
                let mut m = origin.native()?;
                m.seed = b.inputs.seed;
                let check = evaluate_with(
                    &mut m,
                    &b.inputs,
                    &Objective::Goals(goals.clone()),
                    ms + 10 * (3 * ahead_k) as i32,
                    EvalOptions::default(),
                )?;
                let ok = matches!(check.score, Score::Goals { ref times, total, .. } if times.len() == total);
                reached += usize::from(ok);
                late_sum += took - want;
                println!(
                    "from {:6.2} s: reached in {took:5} ms (run {want}) {:+5} ms{}  {secs:.1} s",
                    f64::from(ms) / 1000.0,
                    took - want,
                    if ok { "" } else { "  REPLAY MISSES" }
                );
            }
            None => println!("from {:6.2} s: missed (run {want} ms)  {secs:.1} s", f64::from(ms) / 1000.0),
        }
    }
    println!(
        "reached {reached} of {total}; mean vs run {:+.0} ms; {:.0} s",
        if reached > 0 { late_sum as f64 / reached as f64 } else { 0.0 },
        all.elapsed().as_secs_f64()
    );
    Ok(())
}
