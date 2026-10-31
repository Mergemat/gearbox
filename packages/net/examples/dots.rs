//! Observation v12's dots on real records: what they cost per decision next to v11, and whether the car's road is
//! among them. Reads lines of `map record` on stdin; per map prints the far build, the decision cost of v11 (once its
//! map layers are built) and of v12 (the first drive builds the near tiles, the second reuses them) and, over
//! decisions with a wheel on the ground, the share with a dot within 2 m of the road under it (where a ray down the
//! car's up axis hits; any dot, and one facing like that road), by how the road faces: up, wall, ceiling. The last
//! line adds the dots' own cost, warm.

use std::io::BufRead;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use gearbox_engine::collision::{Lists, QueryFilter, Ray};
use gearbox_engine::shape::Geometry;
use gearbox_engine::{Session, Vec3};
use gearbox_net::cloud::{self, FEATURES};
use gearbox_net::observe::DOTS_START;
use gearbox_net::{Course, Ground, LEN_V11, LEN_V12, PERIOD, observe};
use gearbox_run::Record;

const NEAR: f32 = 2.0;

#[derive(Clone, Copy)]
enum Watch {
    Observe(usize),
    Dots,
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut game = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--game" => game = Some(PathBuf::from(args.next().context("--game needs a folder")?)),
            _ => anyhow::bail!("usage: dots [--game DIR] < lines of 'map record'"),
        }
    }
    let packs = gearbox_packs::install::Game::locate(game.as_deref())?.open()?;
    let mut all = Tally::default();
    let (mut v11_all, mut cold_all, mut warm_all, mut dots_all) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for line in std::io::stdin().lock().lines() {
        let line = line?;
        let Some((map, record)) = line.split_once(' ') else { continue };
        let parsed = gearbox_gbx::Map::parse(&std::fs::read(map)?)?;
        let record = Record::read(std::path::Path::new(record))?;
        let session = record.open(&parsed, &packs)?;
        let course = Course::race(&session)?;
        let ground = Ground::build(&session);
        let started = Instant::now();
        let far = ground.cloud(&session).far_bytes();
        let build = started.elapsed();
        let (mut v11, mut cold, mut warm) = (Vec::new(), Vec::new(), Vec::new());
        let mut tally = Tally::default();
        let run = |watch, times: &mut Vec<Duration>, tally| {
            drive(&record, session.clone(), &course, &ground, watch, times, tally);
        };
        run(Watch::Observe(LEN_V11), &mut Vec::new(), None);
        run(Watch::Observe(LEN_V11), &mut v11, None);
        run(Watch::Observe(LEN_V12), &mut cold, Some(&mut tally));
        run(Watch::Observe(LEN_V12), &mut warm, None);
        run(Watch::Dots, &mut dots_all, None);
        let tiles = ground.clouded().map_or(0, cloud::Cloud::near_tiles);
        let name = std::path::Path::new(map).file_name().and_then(|n| n.to_str()).unwrap_or(map);
        println!(
            "{name} far {:.0} ms {:.1} MB, {tiles} near tiles {:.1} MB | v11 {} | v12 cold {} warm {} | {}",
            build.as_secs_f64() * 1e3,
            far as f64 / 1e6,
            (ground.clouded().map_or(0, cloud::Cloud::bytes) - far) as f64 / 1e6,
            stats(&mut v11),
            stats(&mut cold),
            stats(&mut warm),
            tally,
        );
        all.add(&tally);
        v11_all.extend(v11);
        cold_all.extend(cold);
        warm_all.extend(warm);
    }
    println!(
        "all | v11 {} | v12 cold {} warm {} | dots alone warm {} | {all}",
        stats(&mut v11_all),
        stats(&mut cold_all),
        stats(&mut warm_all),
        stats(&mut dots_all)
    );
    Ok(())
}

fn drive(
    record: &Record,
    mut session: Session,
    course: &Course,
    ground: &Ground,
    watch: Watch,
    times: &mut Vec<Duration>,
    mut tally: Option<&mut Tally>,
) {
    let mut out = vec![0.0; if let Watch::Observe(len) = watch { len } else { LEN_V12 }];
    for tick in 0..record.run.controls.len() + 100 {
        if session.race_ms() >= 0 && tick % PERIOD as usize == 0 && session.vehicle().body != u32::MAX {
            let started = Instant::now();
            match watch {
                Watch::Observe(_) => observe(&session, course, ground, &mut out),
                Watch::Dots => {
                    let car = gearbox_net::Car::read(&session);
                    ground.cloud(&session).read(&session, course, &car, &mut out[DOTS_START..]);
                }
            }
            times.push(started.elapsed());
            if let Some(tally) = tally.as_deref_mut() {
                tally.see(&session, &out[DOTS_START..]);
            }
        }
        if !record.step(&mut session, tick) || session.race().and_then(|r| r.finish).is_some() {
            break;
        }
    }
}

fn stats(times: &mut [Duration]) -> String {
    if times.is_empty() {
        return "-".into();
    }
    times.sort();
    let mean = times.iter().sum::<Duration>().as_secs_f64() / times.len() as f64;
    let p99 = times[(times.len() * 99 / 100).min(times.len() - 1)];
    format!("mean {:.1} µs p99 {:.0} µs", mean * 1e6, p99.as_secs_f64() * 1e6)
}

/// Decisions with a wheel on the ground, and those with a dot near the contact: [any, same facing], per contact kind.
#[derive(Default)]
struct Tally {
    seen: [usize; 3],
    near: [[usize; 2]; 3],
}

impl Tally {
    fn see(&mut self, session: &Session, dots: &[f32]) {
        let body = session.body();
        if session.vehicle().wheels.iter().all(|w| w.contact == 0) {
            return;
        }
        // The surface under the car: a ray down the car's own up axis, as the wheels touch it.
        let down = body.rot * Vec3::new(0.0, -3.0, 0.0);
        let filter = QueryFilter { lists: Lists::SURFACES, layers: 0x12_8002, ..QueryFilter::default() };
        let geometry = Geometry::new(&session.world, &session.state);
        let ray = Ray { origin: body.pos, direction: down };
        let Some(hit) = session.state.collision_world[session.config.manager].raycast(geometry, ray, filter)
        else {
            return;
        };
        let contact = body.pos + down * hit.fraction;
        let normal = hit.normal;
        let kind = if normal.y >= 0.5 {
            0
        } else if normal.y > -0.5 {
            1
        } else {
            2
        };
        let local = body.rot.transpose_mul_vec(contact - body.pos);
        let facing = body.rot.transpose_mul_vec(normal);
        let length = facing.dot(facing).sqrt().max(1e-6);
        let mut found = [false; 2];
        for dot in dots.as_chunks::<FEATURES>().0 {
            if dot[0] == 0.0 || dot[8] * cloud::STEPS >= f32::from(cloud::CHECKPOINT) {
                continue;
            }
            let at = Vec3::new(unsquash(dot[1]), unsquash(dot[2]), unsquash(dot[3]));
            let gap = at - local;
            if gap.dot(gap) <= NEAR * NEAR {
                found[0] = true;
                found[1] |= Vec3::new(dot[4], dot[5], dot[6]).dot(facing) / length >= 0.7;
            }
        }
        self.seen[kind] += 1;
        for k in 0..2 {
            self.near[kind][k] += usize::from(found[k]);
        }
    }

    fn add(&mut self, other: &Tally) {
        for kind in 0..3 {
            self.seen[kind] += other.seen[kind];
            for k in 0..2 {
                self.near[kind][k] += other.near[kind][k];
            }
        }
    }
}

impl std::fmt::Display for Tally {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let seen: usize = self.seen.iter().sum();
        let near = |k: usize| self.near.iter().map(|n| n[k]).sum::<usize>();
        let share = |n: usize, d: usize| {
            if d == 0 { "-".into() } else { format!("{:.1}%", 100.0 * n as f64 / d as f64) }
        };
        write!(f, "within {NEAR} m: {} ({} facing) of {seen}", share(near(0), seen), share(near(1), seen))?;
        for (kind, name) in ["road", "wall", "ceiling"].iter().enumerate() {
            if self.seen[kind] > 0 {
                write!(
                    f,
                    "; {name} {} ({}) of {}",
                    share(self.near[kind][0], self.seen[kind]),
                    share(self.near[kind][1], self.seen[kind]),
                    self.seen[kind]
                )?;
            }
        }
        Ok(())
    }
}

fn unsquash(v: f32) -> f32 {
    (libm::exp2f(v.abs() * 3.0) - 1.0).copysign(v)
}
