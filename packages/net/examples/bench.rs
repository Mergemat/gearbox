use std::io::BufRead;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use gearbox_engine::Session;
use gearbox_net::{Course, Ground, LEN, LEN_V9, Net, PERIOD, STEER, observe, rays};
use gearbox_run::Record;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut game = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--game" => game = Some(PathBuf::from(args.next().context("--game needs a folder")?)),
            _ => anyhow::bail!("usage: bench [--game DIR] < lines of 'map record'"),
        }
    }
    let packs = gearbox_packs::install::Game::locate(game.as_deref())?.open()?;
    let mut builds = Vec::new();
    let mut observations = Vec::new();
    let mut wide = Vec::new();
    let mut ray_times = Vec::new();
    let mut ground_times = Vec::new();
    let mut sizes = Vec::new();
    let mut differ = 0;
    let mut maps = 0;
    for line in std::io::stdin().lock().lines() {
        let line = line?;
        let Some((map, record)) = line.split_once(' ') else { continue };
        let parsed = gearbox_gbx::Map::parse(&std::fs::read(map)?)?;
        let record = Record::read(std::path::Path::new(record))?;
        let session = record.open(&parsed, &packs)?;
        let started = Instant::now();
        let ground = Ground::build(&session);
        builds.push(started.elapsed());
        sizes.push(ground.bytes());
        let course = Course::race(&session)?;
        let watched = drive(
            &record,
            session.clone(),
            Some((&course, &ground)),
            &mut observations,
            &mut wide,
            &mut ray_times,
            &mut ground_times,
        );
        let blind =
            drive(&record, session, None, &mut Vec::new(), &mut Vec::new(), &mut Vec::new(), &mut Vec::new());
        let (watched, digest) = watched;
        let (blind, _) = blind;
        println!("digest {map} {digest:016x}");
        if watched != blind {
            differ += 1;
            eprintln!("{map}: observing changed the run: {watched:?} vs {blind:?}");
        }
        maps += 1;
    }
    println!("{maps} maps, {differ} runs changed by observing");
    report("ground build", &mut builds);
    sizes.sort_unstable();
    let at = |q: f64| sizes[((sizes.len() - 1) as f64 * q) as usize] as f64 / 1e6;
    println!("ground MB: median {:.1}, p95 {:.1}, max {:.1}", at(0.5), at(0.95), at(1.0));
    report("observe", &mut observations);
    report("observe v9", &mut wide);
    report("rays", &mut ray_times);
    report("ground ahead", &mut ground_times);
    let mut evals = evaluate();
    report("eval", &mut evals);
    Ok(())
}

fn drive(
    record: &Record,
    mut session: Session,
    watch: Option<(&Course, &Ground)>,
    times: &mut Vec<Duration>,
    wide: &mut Vec<Duration>,
    ray_times: &mut Vec<Duration>,
    ground_times: &mut Vec<Duration>,
) -> ((Option<u32>, [u32; 3]), u64) {
    let mut out = vec![0.0; LEN];
    let mut out_v9 = vec![0.0; LEN_V9];
    let mut digest = 0xcbf2_9ce4_8422_2325u64;
    let mut rays_out = vec![0.0; rays::LEN];
    let mut ground_out = vec![0.0; gearbox_net::ground::LEN];
    let limit = record.run.controls.len() + 100;
    for tick in 0..limit {
        let ms = session.race_ms();
        if let Some((course, ground)) = watch
            && ms >= 0
            && ((ms / 10) as u32).is_multiple_of(PERIOD)
        {
            let started = Instant::now();
            observe(&session, course, ground, &mut out_v9);
            wide.push(started.elapsed());
            let started = Instant::now();
            observe(&session, course, ground, &mut out);
            times.push(started.elapsed());
            assert!(out_v9[..LEN].iter().zip(&out).all(|(a, b)| a.to_bits() == b.to_bits()));
            for v in &out_v9 {
                digest = (digest ^ u64::from(v.to_bits())).wrapping_mul(0x0100_0000_01b3);
            }
            let started = Instant::now();
            rays::read(&session, &mut rays_out, &mut []);
            ray_times.push(started.elapsed());
            let body = session.body();
            let forward = body.rot * gearbox_engine::Vec3::new(0.0, 0.0, 1.0);
            let started = Instant::now();
            ground.ahead(body.pos.to_array(), [forward.x, forward.z], &mut ground_out);
            ground_times.push(started.elapsed());
            std::hint::black_box(&out);
        }
        if !record.step(&mut session, tick) {
            break;
        }
        if session.race().and_then(|r| r.finish).is_some() {
            break;
        }
    }
    ((session.race().and_then(|r| r.finish), session.vehicle().pos.to_array().map(f32::to_bits)), digest)
}

fn evaluate() -> Vec<Duration> {
    let mut bits = 7u32;
    let mut random = move || {
        bits = bits.wrapping_mul(1664525).wrapping_add(1013904223);
        ((bits >> 8) as f32 / 16777216.0 - 0.5) * 0.1
    };
    let mut bytes = b"GNET".to_vec();
    for w in [5, gearbox_net::VERSION, LEN as u32, PERIOD, STEER.len() as u32] {
        bytes.extend(w.to_le_bytes());
    }
    for v in STEER.into_iter().chain([0.0; LEN]).chain([1.0; LEN]) {
        bytes.extend(v.to_le_bytes());
    }
    let sizes = [LEN, 512, 512, 512, STEER.len() + 2];
    bytes.extend((sizes.len() as u32 - 1).to_le_bytes());
    for pair in sizes.windows(2) {
        bytes.extend((pair[0] as u32).to_le_bytes());
        bytes.extend((pair[1] as u32).to_le_bytes());
        for _ in 0..pair[0] * pair[1] + pair[1] {
            bytes.extend(random().to_le_bytes());
        }
    }
    let net = Net::read(&bytes).unwrap();
    let obs: Vec<f32> = (0..LEN).map(|_| random() * 10.0).collect();
    (0..2000)
        .map(|_| {
            let started = Instant::now();
            std::hint::black_box(net.eval(std::hint::black_box(&obs)));
            started.elapsed()
        })
        .collect()
}

fn report(name: &str, times: &mut [Duration]) {
    if times.is_empty() {
        return;
    }
    times.sort_unstable();
    let at = |q: f64| times[((times.len() - 1) as f64 * q) as usize];
    println!("{name}: n {}, median {:?}, p95 {:?}, max {:?}", times.len(), at(0.5), at(0.95), at(1.0));
}
