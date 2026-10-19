use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use gearbox_trace::driver::Defaults;

pub fn golden_dir() -> PathBuf {
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden");
    if source.join("search.tsv").is_file() {
        return source.canonicalize().unwrap_or(source);
    }
    let cwd = std::env::current_dir().unwrap_or_default();
    cwd.ancestors()
        .map(|d| d.join("tests/golden"))
        .find(|d| d.join("search.tsv").is_file())
        .unwrap_or_else(|| PathBuf::from("tests/golden"))
}

struct SearchCase {
    name: &'static str,
    trace: &'static str,
    range: std::ops::Range<usize>,
    until: usize,
    improve: bool,
}

const SEARCHES: [SearchCase; 2] = [
    SearchCase {
        name: "improve",
        trace: "pf-364816/rec-pf364816",
        range: 150..230,
        until: 300,
        improve: true,
    },
    SearchCase {
        name: "population",
        trace: "pf-364816/rec-pf364816",
        range: 150..230,
        until: 300,
        improve: false,
    },
];

fn hash_text(s: &str) -> String {
    format!("{:016x}", gearbox_trace::file::fnv64(s.as_bytes()))
}

fn bits(t: &gearbox_search::input::Timeline) -> String {
    let mut s = format!("{} {:?}", t.t0, t.seed);
    for c in &t.controls {
        s += &format!(" {:x}/{:x}/{:x}", c.gas.to_bits(), c.brake.to_bits(), c.steer.to_bits());
    }
    s
}

fn run_search(
    c: &SearchCase,
    dir: &Path,
    maps: &[PathBuf],
    factory: &gearbox_trace::typed::TypedFactory,
) -> Result<(String, String)> {
    let trace = gearbox_trace::Trace::read(&gearbox_trace::driver::trace_path(dir, c.trace))
        .with_context(|| format!("trace {}", c.trace))?;
    let map = gearbox_trace::engine::find_map(&trace, dir, maps)
        .with_context(|| format!("map {} not found", trace.case.map.file))?;
    let origin =
        factory.session(&map, trace.case.t0, trace.case.seed, None).map_err(|e| anyhow::anyhow!("{e}"))?;
    let timeline = gearbox_search::input::Timeline {
        t0: trace.case.start_ms,
        seed: trace.case.seed,
        controls: trace
            .case
            .steps
            .iter()
            .map(|s| gearbox_engine::Controls { gas: s.gas, brake: s.brake, steer: s.steer })
            .collect(),
    };
    let t0 = timeline.t0;
    let until_ms = t0 + 10 * c.until as i32;
    let p =
        trace.states.get(c.until).context("trace shorter than the search")?.body.position.map(|x| x.get());
    let objective = gearbox_search::objective::Objective::Position {
        at_ms: until_ms,
        point: [p[0] + 6.0, p[1], p[2] + 3.0],
    };
    let mut text = String::new();
    let summary;
    if c.improve {
        let settings = gearbox_search::improve::Settings {
            threads: 4,
            seed: 11,
            constraints: Default::default(),
            eval: Default::default(),
            placement: None,
        };
        let mut rounds = 0;
        let lines = gearbox_search::improve::improve(
            &origin,
            &objective,
            timeline,
            c.range.clone(),
            c.until,
            &settings,
            |_| {
                rounds += 1;
                rounds < 4
            },
        )?;
        for l in &lines {
            text += &format!("{} {:?} {:?} {}\n", l.id, l.change, l.score, bits(&l.inputs));
            for p in &l.trail {
                text += &format!("{:x} {:x} {:x},", p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
            }
            text += "\n";
        }
        summary = format!("{} lines, best {:?}", lines.len(), lines.first().map(|l| &l.score));
    } else {
        let config = gearbox_search::search::Config {
            window_start_ms: t0 + 10 * c.range.start as i32,
            window_end_ms: t0 + 10 * c.range.end as i32,
            until_ms,
            objective,
            iterations: 96,
            threads: 4,
            batch_size: 16,
            seed: 3,
            mutation: Default::default(),
            method: gearbox_search::search::Method::default(),
            evaluation: Default::default(),
        };
        let r = gearbox_search::search::search_from(&origin, timeline, &config, |_| {})?;
        text += &format!(
            "{}\n{:?}\n{:?}\n{} {} {}\n",
            bits(&r.inputs),
            r.before,
            r.after,
            r.iterations,
            r.improvements,
            r.evaluated_ticks
        );
        for s in &r.survivors {
            text += &format!("{}\n", bits(s));
        }
        summary = format!("{} improvements, after {:?}", r.improvements, r.after.score);
    }
    Ok((hash_text(&text), summary))
}

fn search(args: &[String], dir: &Path) -> Result<()> {
    let mut dir = dir.to_path_buf();
    let mut maps: Vec<PathBuf> = Vec::new();
    let mut it = args.iter().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--dir" => dir = it.next().context("--dir DIR")?.into(),
            "--maps" => maps.push(it.next().context("--maps DIR")?.into()),
            _ => anyhow::bail!("unknown argument {a}"),
        }
    }
    let file = dir.join("search.tsv");
    let factory = typed_factory()?;
    match args.first().map(String::as_str) {
        Some("record") => {
            let mut out = String::new();
            for c in &SEARCHES {
                let (h, s) = run_search(c, &dir, &maps, &factory)?;
                println!("{}\t{h}\t{s}", c.name);
                out += &format!("{}\t{}\t{h}\t{s}\n", c.name, c.trace);
            }
            std::fs::write(&file, out)?;
            println!("wrote {}", file.display());
            Ok(())
        }
        Some("check") => {
            let text =
                std::fs::read_to_string(&file).with_context(|| format!("reading {}", file.display()))?;
            let mut bad = 0;
            for line in text.lines() {
                let f: Vec<&str> = line.split('\t').collect();
                let c = SEARCHES
                    .iter()
                    .find(|c| f.first() == Some(&c.name))
                    .with_context(|| format!("unknown search case: {line}"))?;
                let (h, s) = run_search(c, &dir, &maps, &factory)?;
                if f.get(2) == Some(&h.as_str()) {
                    println!("{}\tEXACT {h}", c.name);
                } else {
                    bad += 1;
                    println!("{}\tDIFF {h}, expected {}: {s}", c.name, f.get(2).unwrap_or(&"?"));
                }
            }
            anyhow::ensure!(bad == 0, "{bad} search goldens differ");
            Ok(())
        }
        _ => anyhow::bail!("gearbox golden search record|check [--dir DIR] [--maps DIR]..."),
    }
}

const HELP: &str = "
  search record|check [--dir DIR] [--maps DIR]...
      fixed-seed searches (improve, population) from DIR's
      pf-364816 trace: their output hashes in DIR/search.tsv

Defaults: --dir is gearbox's tests/golden, --corpus DIR/corpus.tsv.";

pub fn typed_factory() -> Result<gearbox_trace::typed::TypedFactory> {
    Ok(gearbox_trace::typed::TypedFactory::new(crate::packs()?))
}

pub fn drive(
    args: Vec<String>,
    command: &[&str],
    factory: &dyn gearbox_trace::Factory,
    defaults: &Defaults,
) -> Result<()> {
    gearbox_trace::driver::main(args, command, factory, defaults).map_err(|e| anyhow::anyhow!("{e}"))
}

pub fn defaults() -> Defaults {
    let dir = golden_dir();
    Defaults { corpus: dir.join("corpus.tsv"), dir }
}

pub fn main(args: Vec<String>) -> Result<()> {
    match args.first().map(String::as_str) {
        None | Some("help" | "--help" | "-h") => {
            println!("{}\n{HELP}", gearbox_trace::driver::USAGE);
            Ok(())
        }
        _ if args.iter().any(|arg| matches!(arg.as_str(), "--help" | "-h")) => {
            println!("{}\n{HELP}", gearbox_trace::driver::USAGE);
            Ok(())
        }
        Some("search") => search(&args[1..], &defaults().dir),
        Some("list" | "show") => {
            drive(args, &["golden"], &gearbox_trace::typed::TypedFactory::new(PathBuf::new()), &defaults())
        }
        _ => {
            let factory = typed_factory()?;
            let packs = factory.packs.display().to_string();
            drive(args, &["--game", &packs, "golden"], &factory, &defaults())
        }
    }
}
