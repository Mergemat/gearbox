use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use gearbox_gbx::inputs::TICK_MS;
use gearbox_gbx::record::CLASS_VEHICLE_VIS;
use gearbox_gbx::{Map, Replay};
use serde_json::json;

pub const USAGE: &str = "usage:
  gearbox replay <file.Replay.Gbx>...           summary of each replay
  gearbox replay-json <file.Replay.Gbx> <out>   inputs + car samples as JSON
  gearbox map <file.Map.Gbx|file.Replay.Gbx>    blocks, items and waypoints of a map
  gearbox map-json <file> <out>                 the map as JSON
  gearbox input-states <file.Replay.Gbx>        full input state words (debug)
  gearbox replay-inputs <file.Replay.Gbx> <out> [map]   first ghost's applied controls as an inputs file
  gearbox body <file.Gbx> <out>                 write the decompressed body";

pub fn main(args: Vec<String>) -> Result<()> {
    match args.first().map(String::as_str) {
        Some("replay") if args.len() > 1 => {
            for path in &args[1..] {
                if let Err(e) = summary(path.into()) {
                    println!("{path}: error: {e:#}\n");
                }
            }
            Ok(())
        }
        Some("replay-json") if args.len() == 3 => dump_json(args[1].clone().into(), args[2].clone().into()),
        Some("map") if args.len() == 2 => map_summary(&load_map(&args[1])?),
        Some("map-json") if args.len() == 3 => {
            std::fs::write(&args[2], serde_json::to_vec(&load_map(&args[1])?)?)?;
            Ok(())
        }
        Some("input-states") if args.len() == 2 => {
            let replay = load(&args[1].clone().into())?;
            for g in &replay.ghosts {
                let d = g.inputs.first().context("no inputs")?.decode()?;
                for (tick, w) in &d.state_words {
                    println!(
                        "tick {tick:5} race {:6} ms  kind {:X}  bits {:034b}",
                        d.start_offset_ms + *tick as i32 * TICK_MS,
                        w & 0xF,
                        w
                    );
                }
                for (tick, w) in &d.extra_words {
                    println!(
                        "tick {tick:5} race {:6} ms  extra {w:#010x}",
                        d.start_offset_ms + *tick as i32 * TICK_MS
                    );
                }
            }
            Ok(())
        }
        Some("replay-inputs") if args.len() == 3 || args.len() == 4 => {
            replay_inputs(args[1].clone().into(), args[2].clone().into(), args.get(3).map(PathBuf::from))
        }
        Some("body") if args.len() == 3 => {
            let bytes = std::fs::read(&args[1])?;
            let c = gearbox_gbx::Container::parse(&bytes)?;
            std::fs::write(&args[2], &c.body)?;
            println!("class 0x{:08X}, {} body bytes", c.class_id, c.body.len());
            Ok(())
        }
        _ => bail!("{USAGE}"),
    }
}

fn load(path: &PathBuf) -> Result<Replay> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    Replay::parse(&bytes)
}

fn load_map(path: &str) -> Result<Map> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {path}"))?;
    if gearbox_gbx::Container::parse(&bytes)?.class_id == gearbox_gbx::map::CLASS_MAP {
        return Map::parse(&bytes);
    }
    let replay = Replay::parse(&bytes)?;
    Map::parse(replay.map_gbx.as_deref().context("replay has no embedded map")?)
}

fn map_summary(map: &Map) -> Result<()> {
    println!(
        "{} ({})  by {}  collection {}  deco {} ({})  car {}  size {:?}  author time {:?} ms  flags {:?}",
        map.name.as_deref().unwrap_or("?"),
        map.uid.as_deref().unwrap_or("?"),
        map.author.as_deref().unwrap_or("?"),
        map.collection.as_deref().unwrap_or("?"),
        map.decoration.as_deref().unwrap_or("?"),
        map.decoration_collection.as_deref().unwrap_or("?"),
        map.player_model.as_deref().unwrap_or("default"),
        map.size,
        map.author_time_ms,
        map.flags
    );
    let free = map.blocks.iter().filter(|b| b.is_free()).count();
    let ghost = map.blocks.iter().filter(|b| b.is_ghost()).count();
    println!(
        "  {} blocks ({free} free, {ghost} ghost)  {} baked  {} items",
        map.blocks.len(),
        map.baked_blocks.len(),
        map.items.len()
    );
    let blocks = map.blocks.iter().filter_map(|b| {
        b.waypoint.as_ref().map(|w| (b.name.as_str(), w, Some(b.coord), b.free.map(|f| f.0)))
    });
    let items = map.items.iter().filter_map(|i| {
        i.waypoint.as_ref().map(|w| (i.model.as_deref().unwrap_or("?"), w, None, Some(i.position)))
    });
    for (name, w, coord, pos) in blocks.chain(items) {
        println!(
            "  {:<12} #{:<3} {name}  coord {coord:?}  pos {pos:?}",
            w.tag.as_deref().unwrap_or("?"),
            w.order
        );
    }
    Ok(())
}

fn summary(path: PathBuf) -> Result<()> {
    let replay = load(&path)?;
    println!("{}", path.display());
    println!(
        "  map {}  embedded map {} bytes",
        replay.map_uid.as_deref().unwrap_or("?"),
        replay.map_gbx.as_ref().map_or(0, Vec::len)
    );
    for g in &replay.ghosts {
        println!(
            "  ghost {:?}  time {:?} ms  respawns {:?}  cps {}  exe {:?}",
            g.nickname.as_deref().unwrap_or("?"),
            g.race_time_ms,
            g.respawns,
            g.checkpoints_ms.len(),
            g.exe_version.as_deref().unwrap_or("?")
        );
        if !g.checkpoints_ms.is_empty() {
            let cps: Vec<String> =
                g.checkpoints_ms.iter().map(|c| c.map_or("-".into(), |t| t.to_string())).collect();
            println!("    checkpoint times (ms): {}", cps.join(" "));
        }
        for inp in &g.inputs {
            match inp.decode() {
                Ok(d) => {
                    let changes = d.ticks.windows(2).filter(|w| w[0] != w[1]).count();
                    let analog = d.ticks.iter().any(|t| t.steer != 0 && t.steer.unsigned_abs() != 127);
                    println!(
                        "    inputs v{}  {} ticks ({} ms)  start offset {} ms  {} changes  {}  respawns {}{}",
                        inp.version,
                        d.ticks.len(),
                        d.ticks.len() as i32 * TICK_MS,
                        d.start_offset_ms,
                        changes,
                        if analog { "analog steer" } else { "digital steer" },
                        d.events
                            .iter()
                            .filter(|e| matches!(e.kind, gearbox_gbx::inputs::EventKind::Respawn))
                            .count(),
                        d.truncated_at.map_or(String::new(), |t| format!("  (stream ends at tick {t})"))
                    );
                }
                Err(e) => println!("    inputs v{}  {} ticks  decode error: {e:#}", inp.version, inp.ticks),
            }
        }
        match &g.record {
            Some(rec) => {
                for ent in g.record_entities().filter(|e| e.class_id == CLASS_VEHICLE_VIS) {
                    let s = &ent.samples;
                    let dt: Vec<i32> = s.windows(2).map(|w| w[1].time_ms - w[0].time_ms).collect();
                    let (lo, hi) = (dt.iter().min(), dt.iter().max());
                    println!(
                        "    record v{}  {} car samples  {}..{} ms  interval {:?}..{:?} ms  sample size {} bytes",
                        rec.version,
                        s.len(),
                        s.first().map_or(0, |x| x.time_ms),
                        s.last().map_or(0, |x| x.time_ms),
                        lo,
                        hi,
                        s.first().map_or(0, |x| x.raw.len())
                    );
                    if let (Some(a), Some(b)) = (
                        s.first().and_then(|x| x.vehicle.as_ref()),
                        s.last().and_then(|x| x.vehicle.as_ref()),
                    ) {
                        println!(
                            "      first pos {:?}  last pos {:?}  last speed {:.1} km/h",
                            a.position,
                            b.position,
                            b.speed * 3.6
                        );
                    }
                }
            }
            None => println!("    no record data"),
        }
    }
    println!();
    Ok(())
}

fn replay_inputs(path: PathBuf, out: PathBuf, map: Option<PathBuf>) -> Result<()> {
    let replay = load(&path)?;
    let record = gearbox_run::Record::from_replay(&replay)?;
    let map = match (map, replay.map_gbx.as_deref()) {
        (Some(map), _) => Map::parse(&std::fs::read(&map)?)?,
        (None, Some(embedded)) => Map::parse(embedded)?,
        (None, None) => bail!("{} has no embedded map; pass the map", path.display()),
    };
    let packs = crate::game()?.open()?;
    let run = record.applied(&map, &packs)?;
    run.write(&out)?;
    println!("wrote {} ticks from {} ms, start seed {:?}", run.controls.len(), run.t0, run.seed);
    Ok(())
}

fn dump_json(path: PathBuf, out: PathBuf) -> Result<()> {
    let replay = load(&path)?;
    let ghosts: Vec<_> = replay
        .ghosts
        .iter()
        .map(|g| {
            let inputs: Vec<_> = g.inputs.iter().map(|i| i.decode().map_err(|e| format!("{e:#}"))).collect();
            let samples = ghost_samples(g);
            json!({ "meta": g, "inputs": inputs, "samples": samples })
        })
        .collect();
    let doc = json!({ "map_uid": replay.map_uid, "ghosts": ghosts });
    std::fs::write(&out, serde_json::to_vec(&doc)?)?;
    println!("wrote {}", out.display());
    Ok(())
}

fn ghost_samples(ghost: &gearbox_gbx::Ghost) -> Vec<&gearbox_gbx::record::Sample> {
    ghost
        .record_entities()
        .filter(|entity| entity.class_id == CLASS_VEHICLE_VIS)
        .flat_map(|entity| &entity.samples)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use gearbox_gbx::{
        Ghost,
        record::{Entity, RecordData, Sample},
    };

    #[test]
    fn samples_follow_the_ghost_across_its_assigned_vehicles() {
        let entity = |class_id, time_ms| Entity {
            class_id,
            start_ms: 0,
            end_ms: 100,
            samples: vec![Sample { time_ms, vehicle: None, raw: vec![] }],
        };
        let ghost = Ghost {
            record: Some(RecordData {
                version: 10,
                start_ms: 0,
                end_ms: 100,
                descs: vec![],
                entities: vec![
                    entity(CLASS_VEHICLE_VIS, 10),
                    entity(CLASS_VEHICLE_VIS, 20),
                    entity(0, 30),
                    entity(CLASS_VEHICLE_VIS, 40),
                ],
            }),
            record_entity_indices: vec![1, 2, 3],
            ..Default::default()
        };
        let samples = serde_json::to_value(ghost_samples(&ghost)).unwrap();
        assert_eq!(
            samples.as_array().unwrap().iter().map(|s| s["time_ms"].as_i64().unwrap()).collect::<Vec<_>>(),
            vec![20, 40]
        );
    }
}
