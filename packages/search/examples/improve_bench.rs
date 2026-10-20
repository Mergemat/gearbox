use gearbox_search::search::Origin as _;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use anyhow::{Context, Result};
use gearbox_engine::Controls;
use gearbox_search::goals::{Goal, Goals};
use gearbox_search::improve::{Settings, difference, improve};
use gearbox_search::input::Timeline;
use gearbox_search::objective::{Objective, Score};

#[derive(clap::Parser)]
struct Args {
    #[arg(long)]
    game: Option<PathBuf>,
    dir: PathBuf,
    #[arg(default_value_t = 60.0)]
    secs: f32,
    #[arg(default_value_t = 1)]
    seed: u64,
    #[arg(long, default_value_t = 6.0)]
    jitter: f32,
}

fn main() -> Result<()> {
    let args: Args = clap::Parser::parse();
    let packs = gearbox_packs::install::Game::locate(args.game.as_deref())?.packs;
    let (dir, secs, seed) = (args.dir.as_path(), args.secs, args.seed);
    let inputs = Timeline::read(&dir.join("wr.txt"))?;
    let s = gearbox_world::open_file(
        &dir.join("map.Map.Gbx"),
        &packs,
        gearbox_world::OpenOptions { t0: inputs.t0, seed: inputs.seed, car: None },
    )?;
    let start = s.save();
    let origin = start.clone();
    let mut n = start.native()?;
    let wr = Timeline { t0: inputs.t0, seed: inputs.seed, controls: inputs.controls.clone() };
    let wr = if n.race_ms() < wr.t0 { wr.starting_at(n.race_ms())? } else { wr };
    n.seed = wr.seed;
    let k0 = wr.tick(0)?;
    let mut k = wr.tick(n.race_ms())?;
    let (mut path, mut cp) = (Vec::new(), None);
    while k < wr.controls.len() {
        if k >= k0 {
            path.push(gearbox_search::Body::from(n.body()).position());
        }
        n.step(wr.controls[k]);
        k += 1;
        if let Some(t) = n.race().and_then(|r| r.checkpoints.first().copied()) {
            cp = Some((
                t,
                k,
                gearbox_search::gate::crossing_waypoint(&mut n, 1).context("crossing without a waypoint")?,
            ));
            break;
        }
    }
    let (wr_ms, wr_tick, waypoint) = cp.context("the record never crosses a checkpoint")?;
    let mut m = start.native()?;
    let gates = gearbox_search::gate::gates(&mut m)?;
    let gate = gates.iter().find(|g| g.waypoint == waypoint).cloned().context("crossed gate exists")?;
    let world = m.static_triangles();
    let dry = |t: &&gearbox_engine::consumer::PhysicalTriangle| t.surface != 13;
    let cells = Arc::new(gearbox_search::track::Cells::build(world.iter().filter(dry).map(|t| t.v), 3.0, 4));
    let g = gate.clone();
    let centre = g.boxes.first().map_or([0.0; 3], |b| std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5));
    let field = gearbox_search::track::GoalField::build(cells, centre, move |p| g.distance(p) <= 1.5);
    let goals = Goals {
        ordered: vec![Goal::Gate { ordinal: 1, gate, also: Vec::new() }],
        along: None,
        stay_within: None,
        track: Some(Arc::new(gearbox_search::track::TrackFields { goals: vec![Some(field)], reach: 8 })),
        fastest: false,
        speed_worth_ms: 0.0,
        conditions: Vec::new(),
    };
    let objective = Objective::Goals(goals);
    let until = wr_tick + 1000;
    let mut blank = wr.clone();
    blank.controls.truncate(k0);
    blank.controls.resize(until, Controls { gas: 1.0, brake: 0.0, steer: 0.0 });
    let time = |sc: &Score| match sc {
        Score::Goals { times, total, .. } if times.len() == *total => times.last().copied(),
        _ => None,
    };
    let reaches = |line: &Timeline| -> Result<Option<i32>> {
        let mut n = start.native()?;
        n.seed = line.seed;
        Ok(time(
            &gearbox_search::objective::evaluate(&mut n, line, &objective, line.t0 + 10 * until as i32)?
                .score,
        ))
    };
    let jitter = args.jitter;
    let mut shake = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1;
    let mut side = move || {
        shake ^= shake << 13;
        shake ^= shake >> 7;
        shake ^= shake << 17;
        (shake >> 11) as f32 / (1u64 << 53) as f32 * 2.0 - 1.0
    };
    let mut drawn = None;
    for (spacing, jitter) in [
        (30.0, jitter),
        (15.0, jitter),
        (30.0, jitter / 2.0),
        (15.0, jitter / 2.0),
        (30.0, 0.0),
        (15.0, 0.0),
        (8.0, 0.0),
    ] {
        let mut pts = vec![path[0]];
        let mut run = 0.0;
        for w in path.windows(2) {
            run += ((w[1][0] - w[0][0]).powi(2) + (w[1][2] - w[0][2]).powi(2)).sqrt();
            if run >= spacing {
                let (dx, dz) = (w[1][0] - w[0][0], w[1][2] - w[0][2]);
                let l = (dx * dx + dz * dz).sqrt().max(1e-3);
                let o = jitter * side();
                pts.push([w[1][0] - dz / l * o, w[1][1], w[1][2] + dx / l * o]);
                run = 0.0;
            }
        }
        let last = *path.last().unwrap();
        let p = path[path.len().saturating_sub(20)];
        let (dx, dz) = (last[0] - p[0], last[2] - p[2]);
        let l = (dx * dx + dz * dz).sqrt().max(1e-3);
        pts.push(last);
        pts.push([last[0] + 30.0 * dx / l, last[1], last[2] + 30.0 * dz / l]);
        let d = gearbox_search::tries::drive_on(&origin, &blank, k0, k0, until, &pts)?;
        if let Some(t) = reaches(&d.line)? {
            eprintln!(
                "drawn line: a point every {spacing} m, up to {jitter} m off the record, reaches in {t} ms"
            );
            drawn = Some(d);
            break;
        }
    }
    let Some(drawn) = drawn else {
        println!(
            "{}\tdrawn -\tbest -\tWR {wr_ms}\tno drawn line gets there",
            dir.file_name().unwrap().to_string_lossy()
        );
        return Ok(());
    };
    let threads = std::thread::available_parallelism().map_or(8, |n| n.get());
    let settings = Settings {
        threads,
        seed,
        constraints: Default::default(),
        eval: Default::default(),
        placement: None,
    };
    let t = Instant::now();
    let (mut last_print, mut first) = (0.0, None);
    let lines = improve(&origin, &objective, drawn.line.clone(), k0..until, until, &settings, |p| {
        let el = t.elapsed().as_secs_f32();
        let start_ms = first
            .get_or_insert_with(|| p.lines.iter().find(|l| l.change.is_none()).and_then(|l| time(&l.score)));
        if el - last_print >= 10.0 || el >= secs {
            last_print = el;
            let times: Vec<String> = p
                .lines
                .iter()
                .map(|l| time(&l.score).map_or("-".into(), |t| format!("{:.2}", t as f32 / 1000.0)))
                .collect();
            let kept: Vec<String> =
                p.changes.iter().map(|(c, n, k)| format!("{} {k}/{n}", c.label())).collect();
            eprintln!(
                "{el:5.0}s {} tries  lines {}  drawn {:?}  kept {}",
                p.tries,
                times.join(" "),
                start_ms,
                kept.join(", ")
            );
        }
        el < secs
    })?;
    let best = &lines[0];
    let game_cp = |line: &Timeline| -> Result<(Option<i32>, Vec<[f32; 3]>)> {
        let mut n = start.native()?;
        n.seed = line.seed;
        let mut k = line.tick(n.race_ms())?;
        let mut trail = Vec::new();
        while k < line.controls.len() {
            if k >= k0 {
                trail.push(gearbox_search::Body::from(n.body()).position());
            }
            n.step(line.controls[k]);
            k += 1;
            if let Some(t) = n.race().and_then(|r| r.checkpoints.first().copied()) {
                return Ok((Some(t as i32), trail));
            }
        }
        Ok((None, trail))
    };
    let (best_game, best_trail) = game_cp(&best.inputs)?;
    let (_, drawn_trail) = game_cp(&drawn.line)?;
    let drawn_ms = reaches(&drawn.line)?;
    println!(
        "{}\tdrawn {}\tbest {} (game {})\tWR {}\tbest is {:.1} m off drawn, {:.1} m off WR (drawn {:.1})\tlines {}",
        dir.file_name().unwrap().to_string_lossy(),
        drawn_ms.map_or("-".into(), |t| t.to_string()),
        time(&best.score).map_or("-".into(), |t| t.to_string()),
        best_game.map_or("-".into(), |t| t.to_string()),
        wr_ms,
        difference(&best_trail, &drawn_trail),
        difference(&best_trail, &path),
        difference(&drawn_trail, &path),
        lines.iter().filter(|l| time(&l.score).is_some()).count(),
    );
    Ok(())
}
