use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Result, ensure};
use clap::{Parser, Subcommand, ValueEnum};
use gearbox_search::input::Timeline;
use gearbox_search::mutation::Mutation;
use gearbox_search::objective::Objective;
use gearbox_search::objective::{EvalOptions, ReferencePath};
use gearbox_search::search::MAX_THREADS;
use gearbox_search::search::{Config, Method, Origin, search_from};

#[derive(Parser)]
#[command(about = "Offline Trackmania guided input search (no game process required)")]
struct Args {
    source: PathBuf,
    inputs: PathBuf,
    output: PathBuf,
    #[arg(long, allow_hyphen_values = true)]
    window_start: i32,
    #[arg(long, allow_hyphen_values = true)]
    window_end: i32,
    #[arg(long, allow_hyphen_values = true)]
    until_ms: Option<i32>,
    #[arg(long, default_value_t = 1000)]
    iterations: usize,
    #[arg(long, default_value_t = 4)]
    threads: usize,
    #[arg(long, default_value_t = 64)]
    batch_size: usize,
    #[arg(long, default_value_t = 1)]
    seed: u64,
    #[arg(long, default_value_t = 2)]
    changes: usize,
    #[arg(long, default_value_t = 10)]
    span_ticks: usize,
    #[arg(long, default_value_t = 5)]
    shift_ticks: usize,
    #[arg(long, default_value_t = 0.1)]
    steer_delta: f32,
    #[arg(long, value_enum, default_value_t = SearchMethod::Population)]
    method: SearchMethod,
    #[arg(long, default_value_t = 8)]
    population: usize,
    #[arg(long, default_value_t = 20)]
    curve_ticks: usize,
    #[arg(long, default_value_t = 200)]
    horizon_ticks: usize,
    #[arg(long, default_value_t = 50)]
    commit_ticks: usize,
    #[arg(long, default_value_t = 0.0)]
    trick_bonus: f64,
    #[arg(long)]
    full_rollouts: bool,
    #[arg(long)]
    start_state: Option<PathBuf>,
    #[command(subcommand)]
    target: Target,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum SearchMethod {
    HillClimb,
    Population,
    Receding,
}

#[derive(Subcommand)]
enum Target {
    Goals {
        #[arg(long)]
        file: PathBuf,
    },
    Waypoint {
        index: usize,
    },
    Finish,
    Gate {
        index: usize,
        #[arg(long)]
        waypoint: Option<u32>,
        #[arg(long, default_value_t = 1.0)]
        time_penalty: f64,
    },
    GateSpeed {
        index: usize,
        #[arg(long)]
        waypoint: Option<u32>,
        #[arg(long, default_value_t = 1.0)]
        time_penalty: f64,
    },
    Path {
        #[arg(long)]
        points: PathBuf,
        #[arg(long, allow_hyphen_values = true)]
        at_ms: i32,
        #[arg(long, default_value_t = 1.0)]
        deviation_penalty: f64,
    },
    Position {
        #[arg(long, allow_hyphen_values = true)]
        at_ms: i32,
        #[arg(long, value_parser = parse_point, allow_hyphen_values = true)]
        point: [f32; 3],
    },
    Speed {
        #[arg(long, allow_hyphen_values = true)]
        at_ms: i32,
    },
}

fn parse_point(value: &str) -> Result<[f32; 3], String> {
    let coordinates =
        value.split(',').map(str::parse::<f32>).collect::<Result<Vec<_>, _>>().map_err(|e| e.to_string())?;
    coordinates.try_into().map_err(|_| "expected three comma-separated coordinates X,Y,Z".into())
}

pub fn main(args: Vec<String>) -> Result<()> {
    let args = Args::parse_from(std::iter::once("gearbox search".to_string()).chain(args));
    let fresh = args.inputs.as_os_str() == "-";
    let recorded = if fresh { None } else { Some(Timeline::read(&args.inputs)?) };
    let session = gearbox_search::session::open(&crate::packs()?, &args.source, recorded.as_ref())?;
    let origin: &dyn Origin = &session;
    let placement = args
        .start_state
        .as_ref()
        .map(|p| -> Result<gearbox_search::pose::Placement> {
            Ok(serde_json::from_slice(&std::fs::read(p)?)?)
        })
        .transpose()?;
    let recorded = recorded
        .map(|t| if t.t0 > session.race_ms() { t.starting_at(session.race_ms()) } else { Ok(t) })
        .transpose()?;
    let gate_speed = matches!(args.target, Target::GateSpeed { .. });
    let objective = match args.target {
        Target::Goals { file } => Objective::Goals(serde_json::from_slice(&std::fs::read(file)?)?),
        Target::Waypoint { index } => Objective::Waypoint(index),
        Target::Finish => Objective::Finish,
        Target::Gate { index, waypoint, time_penalty }
        | Target::GateSpeed { index, waypoint, time_penalty } => {
            let mut n = origin.native()?;
            let gate = if let Some(wp) = waypoint {
                gearbox_search::gate::gates(&mut n)?
                    .into_iter()
                    .find(|g| g.waypoint == wp)
                    .ok_or_else(|| anyhow::anyhow!("unknown waypoint ID {wp}"))?
            } else {
                n.seed = recorded.as_ref().and_then(|t| t.seed);
                let reference = if let Some(inputs) = &recorded {
                    gearbox_search::gate::reference_gate(
                        &mut n,
                        index,
                        inputs,
                        args.until_ms.ok_or_else(|| anyhow::anyhow!("gate requires --until-ms"))?,
                    )?
                } else {
                    None
                };
                match reference {
                    Some(g) => g,
                    None => gearbox_search::gate::target_gate(&mut n, index)?,
                }
            };
            eprintln!("target gate {} kind {}, boxes {:?}", gate.waypoint, gate.kind, gate.boxes);
            if gate_speed {
                Objective::GateSpeed { ordinal: index, gate, time_penalty }
            } else {
                Objective::Gate { ordinal: index, gate, time_penalty }
            }
        }
        Target::Path { points, at_ms, deviation_penalty } => {
            let points = std::fs::read_to_string(points)?
                .lines()
                .filter(|s| !s.trim().is_empty())
                .map(|line| {
                    parse_point(&line.split_whitespace().collect::<Vec<_>>().join(","))
                        .map_err(anyhow::Error::msg)
                })
                .collect::<Result<Vec<_>>>()?;
            Objective::Path { at_ms, path: ReferencePath { points, deviation_penalty, stay_within: None } }
        }
        Target::Position { at_ms, point } => Objective::Position { at_ms, point },
        Target::Speed { at_ms } => Objective::Speed { at_ms },
    };
    let until_ms = match &objective {
        Objective::Speed { at_ms } | Objective::Position { at_ms, .. } | Objective::Path { at_ms, .. } => {
            ensure!(
                args.until_ms.is_none_or(|v| v == *at_ms),
                "--until-ms must equal --at-ms for fixed-time targets"
            );
            *at_ms
        }
        _ => args
            .until_ms
            .ok_or_else(|| anyhow::anyhow!("--until-ms is required for waypoint/finish targets"))?,
    };
    let config = Config {
        window_start_ms: args.window_start,
        window_end_ms: args.window_end,
        until_ms,
        objective,
        iterations: args.iterations,
        threads: args.threads,
        batch_size: args.batch_size,
        seed: args.seed,
        method: match args.method {
            SearchMethod::HillClimb => Method::HillClimb,
            SearchMethod::Population => {
                Method::Population { width: args.population, curve_ticks: args.curve_ticks }
            }
            SearchMethod::Receding => Method::Receding {
                horizon_ticks: args.horizon_ticks,
                commit_ticks: args.commit_ticks,
                width: args.population,
                curve_ticks: args.curve_ticks,
            },
        },
        evaluation: EvalOptions {
            full_rollout: args.full_rollouts,
            trick_bonus: args.trick_bonus,
            trace: true,
            from_ms: None,
            path_every: 0,
            prune: false,
            checkpoint_every: 0,
        },
        mutation: Mutation {
            max_changes: args.changes,
            max_span_ticks: args.span_ticks,
            max_shift_ticks: args.shift_ticks,
            max_steer_delta: args.steer_delta,
            ..Default::default()
        },
    };
    ensure!((1..=MAX_THREADS).contains(&config.threads), "threads must be in 1..={MAX_THREADS}");
    let inputs = if fresh {
        let t0 = origin.native()?.race_ms();
        ensure!(until_ms > t0 && (until_ms - t0) % 10 == 0, "invalid fresh timeline end");
        Timeline {
            actions: Vec::new(),
            t0,
            seed: None,
            controls: vec![Default::default(); ((until_ms - t0) / 10) as usize],
        }
    } else {
        recorded.expect("recorded inputs loaded")
    };
    let placed_state = placement
        .map(|placement| -> Result<_> {
            let mut n = origin.native()?;
            n.seed = inputs.seed;
            let first = inputs.tick(n.race_ms())?;
            let at = inputs.tick(config.window_start_ms)?;
            ensure!(first <= at, "placement precedes origin");
            for c in &inputs.controls[first..at] {
                n.step(*c);
            }
            gearbox_search::pose::place(&mut n, &placement)?;
            Ok(n.save())
        })
        .transpose()?;
    let origin: &dyn Origin = placed_state.as_ref().map_or(origin, |o| o as &dyn Origin);
    let mut last_print = Instant::now();
    let mut last_improvements = 0;
    let result = search_from(origin, inputs, &config, |p| {
        if p.improvements != last_improvements || last_print.elapsed() >= Duration::from_secs(5) {
            eprintln!(
                "{} attempts, {} improvements, {:.1}s: {}",
                p.iterations,
                p.improvements,
                p.elapsed.as_secs_f64(),
                p.score
            );
            last_print = Instant::now();
            last_improvements = p.improvements;
        }
    })?;
    result.inputs.write(&args.output)?;
    println!("before: {}", result.before.score);
    println!("after:  {}", result.after.score);
    println!(
        "{} attempts, {} improvements in {:.3}s: {:.1} iterations/s, {} evaluated ticks, {:.0} ticks/s across {} threads",
        result.iterations,
        result.improvements,
        result.elapsed.as_secs_f64(),
        result.iterations as f64 / result.elapsed.as_secs_f64(),
        result.evaluated_ticks,
        result.evaluated_ticks as f64 / result.elapsed.as_secs_f64(),
        config.threads
    );
    println!("experimental signatures: {:?}", result.after.events);
    println!("winner replay from start state: exact score, trajectory and body match");
    println!("wrote {}", args.output.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_cli_accepts_a_comma_separated_point_and_requires_three_coordinates() {
        let prefix = [
            "tas",
            "snap",
            "in",
            "out",
            "--window-start",
            "0",
            "--window-end",
            "100",
            "position",
            "--at-ms",
            "100",
        ];
        let args = Args::try_parse_from(prefix.into_iter().chain(["--point", "-1.5,2,3"])).unwrap();
        assert!(matches!(args.target, Target::Position { at_ms: 100, point: [-1.5, 2.0, 3.0] }));
        for suffix in [vec![], vec!["--point", "1,2"], vec!["--point", "1,2,3,4"], vec!["--point", "a,2,3"]] {
            assert!(Args::try_parse_from(prefix.into_iter().chain(suffix)).is_err());
        }
    }

    #[test]
    fn fixed_time_targets_accept_countdown_times() {
        let prefix = ["tas", "map", "-", "out", "--window-start", "-1500", "--window-end", "-100"];
        for target in [
            vec!["speed", "--at-ms", "-100"],
            vec!["position", "--at-ms", "-100", "--point", "1,2,3"],
            vec!["path", "--points", "route.txt", "--at-ms", "-100"],
        ] {
            let args = Args::try_parse_from(prefix.into_iter().chain(target.iter().copied()));
            assert!(args.is_ok(), "{target:?}: {}", args.err().unwrap());
        }
    }
}
