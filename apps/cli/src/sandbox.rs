use anyhow::{Result, ensure};
use clap::{Parser, Subcommand};
use gearbox_search::{Body, Controls};
use gearbox_search::{
    fan,
    goals::{Goal, Goals},
    input::Timeline,
    mutation::Mutation,
    objective::{EvalOptions, Objective},
    pose::{Placement, Pose, PoseGoal},
    search::{Config, Method, search_from},
};
use serde_json::json;
use std::path::PathBuf;

#[derive(Parser)]
struct Args {
    map: PathBuf,
    #[arg(long)]
    inputs: Option<PathBuf>,
    #[arg(long, default_value_t = 0, allow_hyphen_values = true)]
    at_ms: i32,
    #[arg(long)]
    placement: Option<PathBuf>,
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    Inspect {
        #[arg(long, default_value_t = 0)]
        ticks: usize,
        #[arg(long)]
        output: Option<PathBuf>,
    },
    Pick {
        #[arg(long, value_parser = point, allow_hyphen_values = true)]
        origin: [f32; 3],
        #[arg(long, value_parser = point, allow_hyphen_values = true)]
        direction: [f32; 3],
    },
    Fan {
        #[arg(long, default_value_t = 2000)]
        rollouts: usize,
        #[arg(long, default_value_t = 100)]
        ticks: usize,
        #[arg(long, default_value_t = 5)]
        sample_every: usize,
        #[arg(long, default_value_t = 4)]
        threads: usize,
        #[arg(long, default_value_t = 42)]
        seed: u64,
        #[arg(long)]
        output: PathBuf,
        #[arg(long)]
        goal: Option<PathBuf>,
        #[arg(long, default_value_t = 0)]
        refine: usize,
    },
}
fn point(s: &str) -> Result<[f32; 3], String> {
    s.split(',')
        .map(str::parse::<f32>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| e.to_string())?
        .try_into()
        .map_err(|_| "expected X,Y,Z".into())
}
pub fn main(args: Vec<String>) -> Result<()> {
    let args = Args::parse_from(std::iter::once("gearbox sandbox".to_string()).chain(args));
    let inputs = args.inputs.as_ref().map(|p| Timeline::read(p)).transpose()?;
    let mut s = gearbox_search::session::open(&crate::packs()?, &args.map, inputs.as_ref())?;
    ensure!(args.at_ms >= s.race_ms() && (args.at_ms - s.race_ms()) % 10 == 0, "invalid --at-ms");
    let session_t0 = s.race_ms();
    let mut prefix = Vec::new();
    while s.race_ms() < args.at_ms {
        let c = inputs
            .as_ref()
            .map(|t| {
                t.index(s.race_ms()).and_then(|i| {
                    t.controls.get(i).copied().ok_or_else(|| anyhow::anyhow!("prefix exceeds input coverage"))
                })
            })
            .transpose()?
            .unwrap_or_default();
        prefix.push(c);
        s.step(c);
    }
    let from_start = |t: &Timeline| Timeline {
        t0: session_t0,
        seed: t.seed,
        controls: prefix.iter().chain(&t.controls).copied().collect(),
    };
    if let Some(p) = args.placement {
        gearbox_search::pose::place(&mut s, &serde_json::from_slice::<Placement>(&std::fs::read(p)?)?)?;
    }
    match args.command {
        Command::Pick { origin, direction } => println!(
            "{}",
            serde_json::to_string_pretty(&gearbox_search::picking::pick(&mut s, origin, direction))?
        ),
        Command::Inspect { ticks, output } => {
            let mut trajectory = Vec::new();
            for tick in 0..=ticks {
                let pose = Pose::from_body(Body::from(s.body()));
                pose.validate()?;
                let mut ray = pose.position;
                ray[1] += 10.0;
                trajectory.push(json!({"tick":tick,"ms":s.race_ms(),"pose":pose,"ground":gearbox_search::picking::pick(&mut s, ray, [0.0,-1000.0,0.0])}));
                if tick < ticks {
                    s.step(Controls::default());
                }
            }
            let text = serde_json::to_string_pretty(&trajectory)?;
            if let Some(p) = output {
                std::fs::write(p, text)?;
            } else {
                println!("{text}");
            }
        }
        Command::Fan { rollouts, ticks, sample_every, threads, seed, output, goal, refine } => {
            let goal = goal
                .map(|file| -> Result<PoseGoal> {
                    let goal: PoseGoal = serde_json::from_slice(&std::fs::read(file)?)?;
                    goal.validate()?;
                    Ok(goal)
                })
                .transpose()?;
            ensure!(refine == 0 || goal.is_some(), "--refine requires --goal");
            std::fs::create_dir_all(&output)?;
            let origin = s.save();
            let fan = fan::generate(
                &origin,
                &fan::Config { rollouts, horizon_ticks: ticks, sample_every, threads, seed },
            )?;
            let rate = rollouts as f64 / fan.elapsed.as_secs_f64();
            println!(
                "{rollouts} rollouts x {ticks} ticks, {threads} threads: {:.3}s, {rate:.1} rollouts/s, {} simulated ticks (includes probe)",
                fan.elapsed.as_secs_f64(),
                fan.simulated_ticks
            );
            let rows: Vec<_> = fan.rollouts.iter().map(|r| json!({"index":r.index,"family":r.family,"trajectory":r.trajectory,"end":r.end,"t0":r.inputs.t0,"seed":r.inputs.seed,"inputs":r.inputs.controls.iter().map(|c| [c.gas,c.brake,c.steer]).collect::<Vec<_>>()})).collect();
            std::fs::write(output.join("fan.json"), serde_json::to_vec(&rows)?)?;
            if let Some(goal) = goal {
                let snap = fan::snap(&goal, &fan)?.expect("nonempty fan");
                println!(
                    "snap: rollout {}, {} tick {}, error {:.9}, pose {:?}",
                    snap.rollout.index, snap.rollout.family, snap.tick, snap.error, snap.pose
                );
                from_start(&snap.inputs).write(&output.join("snap.in"))?;
                std::fs::write(output.join("snap.json"), serde_json::to_vec_pretty(&snap.pose)?)?;
                if refine > 0 {
                    let t0 = s.race_ms();
                    let until_ms = snap.rollout.inputs.end_ms()?;
                    let config = Config {
                        window_start_ms: t0,
                        window_end_ms: until_ms,
                        until_ms,
                        objective: Objective::Goals(Goals {
                            ordered: vec![Goal::Pose { target: goal }],
                            fastest: false,
                            speed_worth_ms: 0.0,
                            conditions: Vec::new(),
                            along: None,
                            stay_within: None,
                            track: None,
                        }),
                        iterations: refine,
                        threads,
                        batch_size: 64,
                        seed,
                        mutation: Mutation::default(),
                        method: Method::Population { width: 8, curve_ticks: 10 },
                        evaluation: EvalOptions { full_rollout: true, ..Default::default() },
                    };
                    let blank = Timeline { t0, seed: s.seed, controls: vec![Controls::default(); ticks] };
                    let mut snapped = snap.inputs.clone();
                    snapped.controls.resize(ticks, Controls::default());
                    let scratch = search_from(&origin, blank, &config, |_| {})?;
                    println!(
                        "scratch: {} -> {}; {:.3}s, {} ticks",
                        scratch.before.score,
                        scratch.after.score,
                        scratch.elapsed.as_secs_f64(),
                        scratch.evaluated_ticks
                    );
                    from_start(&scratch.inputs).write(&output.join("scratch.in"))?;
                    let refined = search_from(&origin, snapped, &config, |_| {})?;
                    println!(
                        "refined: {} -> {}; {:.3}s, {} ticks",
                        refined.before.score,
                        refined.after.score,
                        refined.elapsed.as_secs_f64(),
                        refined.evaluated_ticks
                    );
                    from_start(&refined.inputs).write(&output.join("refined.in"))?;
                }
            }
        }
    }
    Ok(())
}
