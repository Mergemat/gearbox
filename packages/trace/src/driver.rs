use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::engine::{Error, Factory, Outcome, Perturbed, Setup, check, find_map};
use crate::file::{Case, MapRef, Trace, fnv64};
use crate::inputs::{Timeline, random_controls, start_ms, steps};

pub const USAGE: &str = "golden <command>

  record [--corpus FILE] [--dir DIR] [--case S]... [--jobs N] [--timeout S]
      run the engine over every case of the corpus manifest and write
      DIR/<case>.trace.zst, and DIR/record.tsv
  check [--dir DIR] [--case S]... [--jobs N] [--timeout S] [--packs DIR]
        [--maps DIR]... [--perturb TICK:WORD]
      run the engine over every trace in DIR: EXACT, or the first DIFF
      (tick, field, expected and actual); exits nonzero unless all are
      EXACT. --perturb flips a bit of the engine's state to show a
      difference is caught. Maps are found by their recorded path, in
      DIR/maps, or in --maps directories, by hash
  list [--dir DIR] [--case S]...
      the traces: case, map, ticks, engine
  show TRACE [TICK]...
      a trace's case, and its state at each TICK

--case keeps cases whose id contains S. --jobs defaults to cores - 2. Corpus lines: id, map path, TMX id or -, inputs
(file:PATH or random:SEED[:REPLAY_SEED]), ticks after race 20 ms;
tab-separated, # comments.";

pub struct Defaults {
    pub dir: PathBuf,
    pub corpus: PathBuf,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    pub id: String,
    pub map: PathBuf,
    pub tmx: Option<u32>,
    pub source: String,
    pub ticks: usize,
}

impl Entry {
    pub fn line(&self) -> String {
        let tmx = self.tmx.map_or("-".to_string(), |t| t.to_string());
        format!("{}\t{}\t{tmx}\t{}\t{}", self.id, self.map.display(), self.source, self.ticks)
    }

    pub fn parse(line: &str) -> Result<Self, String> {
        let f: Vec<&str> = line.split('\t').collect();
        let [id, map, tmx, source, ticks] = f[..] else { return Err(format!("corpus line needs 5 fields: {line}")) };
        Ok(Self {
            id: id.into(),
            map: map.into(),
            tmx: (tmx != "-").then(|| tmx.parse()).transpose().map_err(|e| format!("tmx id {tmx}: {e}"))?,
            source: source.into(),
            ticks: ticks.parse().map_err(|e| format!("ticks {ticks}: {e}"))?,
        })
    }
}

pub fn read_corpus(path: &Path) -> Result<Vec<Entry>, Error> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("corpus {}: {e}", path.display()))?;
    Ok(text.lines().filter(|l| !l.trim().is_empty() && !l.starts_with('#')).map(Entry::parse).collect::<Result<_, _>>()?)
}

pub fn timeline(source: &str, ticks: usize) -> Result<Timeline, Error> {
    if let Some(p) = source.strip_prefix("file:") {
        let text = std::fs::read_to_string(p).map_err(|e| format!("inputs {p}: {e}"))?;
        return Ok(Timeline::parse(&text)?);
    }
    if let Some(r) = source.strip_prefix("random:") {
        let mut it = r.split(':');
        let seed: u64 = it.next().unwrap_or("").parse().map_err(|e| format!("random seed: {e}"))?;
        let replay = it.next().map(str::parse::<u32>).transpose().map_err(|e| format!("replay seed: {e}"))?;
        let t0 = -1500;
        let n = ((20 - start_ms(t0)) / 10) as usize + ticks;
        return Ok(Timeline { t0, seed: replay, controls: random_controls(seed, n) });
    }
    Err(format!("inputs source {source}: expected file:PATH or random:SEED").into())
}

pub fn trace_path(dir: &Path, id: &str) -> PathBuf {
    let safe: String = id.chars().map(|c| if c.is_ascii_alphanumeric() || "/_.-".contains(c) { c } else { '_' }).collect();
    dir.join(format!("{safe}.trace.zst"))
}

pub fn traces(dir: &Path) -> Vec<PathBuf> {
    fn walk(d: &Path, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(d) else { return };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                walk(&p, out);
            } else if p.to_string_lossy().ends_with(".trace.zst") {
                out.push(p);
            }
        }
    }
    let mut out = Vec::new();
    walk(dir, &mut out);
    out.sort();
    out
}

pub fn record_case(factory: &dyn Factory, e: &Entry, packs: Option<PathBuf>) -> Result<(Trace, Option<String>), Error> {
    let bytes = std::fs::read(&e.map).map_err(|err| format!("map {}: {err}", e.map.display()))?;
    let t = timeline(&e.source, e.ticks)?;
    let mut steps = steps(&t, e.ticks);
    let setup = Setup { map: e.map.clone(), t0: t.t0, seed: t.seed, packs };
    let mut engine = factory.open(&setup)?;
    let mut states = vec![engine.state()];
    let mut stopped = None;
    for (k, c) in steps.iter().enumerate() {
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            engine.step(*c);
            engine.state()
        }));
        match r {
            Ok(s) => states.push(s),
            Err(p) => {
                let msg = p.downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_default();
                stopped = Some(format!("step {} panicked: {msg}", k + 1));
                break;
            }
        }
    }
    steps.truncate(states.len() - 1);
    let start = start_ms(t.t0);
    if states[0].race_ms != start {
        return Err(format!("the engine starts at race {} ms, not {start}", states[0].race_ms).into());
    }
    let file = e.map.file_name().map_or(String::new(), |n| n.to_string_lossy().into_owned());
    let source = match e.source.strip_prefix("file:") {
        Some(p) => format!("file:{}", Path::new(p).file_name().map_or(p.into(), |n| n.to_string_lossy())),
        None => e.source.clone(),
    };
    let map = MapRef { file, path: e.map.display().to_string(), hash: fnv64(&bytes), len: bytes.len() as u64, tmx: e.tmx };
    let case = Case { id: e.id.clone(), map, source, t0: t.t0, seed: t.seed, start_ms: start, steps };
    Ok((Trace { case, engine: factory.name(), states }, stopped))
}

pub fn check_case(factory: &dyn Factory, path: &Path, dir: &Path, maps: &[PathBuf], packs: Option<PathBuf>, perturb: Option<(usize, usize)>) -> Result<String, Error> {
    let trace = Trace::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let Some(map) = find_map(&trace, dir, maps) else {
        let m = &trace.case.map;
        let hint = m.tmx.map_or(String::new(), |id| format!(" (fetch https://trackmania.exchange/mapgbx/{id} into {}/maps/{})", dir.display(), m.file));
        return Ok(format!("MISSING map {} {:016x}{hint}", m.file, m.hash));
    };
    let setup = Setup { map, t0: trace.case.t0, seed: trace.case.seed, packs };
    let mut engine = factory.open(&setup)?;
    if let Some((at, word)) = perturb {
        engine = Box::new(Perturbed { inner: engine, tick: 0, at, word });
    }
    Ok(match check(&mut *engine, &trace) {
        Outcome::Exact { states } => format!("EXACT {states} states"),
        Outcome::Diff { tick, race_ms, diff } => format!("DIFF tick {tick} (race {race_ms} ms) {diff}"),
    })
}

struct Opts {
    dir: PathBuf,
    corpus: PathBuf,
    cases: Vec<String>,
    jobs: usize,
    timeout: Duration,
    packs: Option<PathBuf>,
    maps: Vec<PathBuf>,
    perturb: Option<(usize, usize)>,
    rest: Vec<String>,
}

fn parse(args: &[String], d: &Defaults) -> Result<Opts, Error> {
    let cores = std::thread::available_parallelism().map_or(4, |n| n.get());
    let mut o = Opts {
        dir: d.dir.clone(),
        corpus: d.corpus.clone(),
        cases: Vec::new(),
        jobs: cores.saturating_sub(2).max(1),
        timeout: Duration::from_secs(900),
        packs: None,
        maps: Vec::new(),
        perturb: None,
        rest: Vec::new(),
    };
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut val = || it.next().cloned().ok_or_else(|| format!("{a} needs a value"));
        match a.as_str() {
            "--dir" => o.dir = val()?.into(),
            "--corpus" => o.corpus = val()?.into(),
            "--case" => o.cases.push(val()?),
            "--jobs" => o.jobs = val()?.parse()?,
            "--timeout" => o.timeout = Duration::from_secs(val()?.parse()?),
            "--packs" => o.packs = Some(val()?.into()),
            "--maps" => o.maps.push(val()?.into()),
            "--perturb" => {
                let v = val()?;
                let (t, w) = v.split_once(':').ok_or("--perturb TICK:WORD")?;
                o.perturb = Some((t.parse()?, w.parse()?));
            }
            _ => o.rest.push(a.clone()),
        }
    }
    o.jobs = o.jobs.max(1);
    Ok(o)
}

fn wanted(o: &Opts, id: &str) -> bool {
    o.cases.is_empty() || o.cases.iter().any(|c| id.contains(c.as_str()))
}

const RESULT: &str = "GOLDEN\t";

fn fan_out(prefix: &[String], jobs: Vec<(String, Vec<String>)>, o: &Opts) -> Vec<(String, String)> {
    let host = std::env::current_exe().expect("the host binary");
    let nice = cfg!(unix) && Command::new("nice").arg("true").status().is_ok_and(|s| s.success());
    let next = Mutex::new(0usize);
    let results = Mutex::new(vec![None; jobs.len()]);
    let done = Mutex::new(0usize);
    let total = jobs.len();
    std::thread::scope(|s| {
        for _ in 0..o.jobs.min(total) {
            s.spawn(|| loop {
                let i = {
                    let mut n = next.lock().unwrap();
                    *n += 1;
                    *n - 1
                };
                let Some((id, args)) = jobs.get(i) else { return };
                let mut cmd = if nice {
                    let mut c = Command::new("nice");
                    c.args(["-n", "10"]).arg(&host);
                    c
                } else {
                    Command::new(&host)
                };
                cmd.args(prefix).args(args).stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
                if let Some(p) = &o.packs {
                    cmd.arg("--packs").arg(p);
                }
                let line = run_child(cmd, o.timeout);
                let mut d = done.lock().unwrap();
                *d += 1;
                println!("[{}/{total}] {id}\t{line}", *d);
                results.lock().unwrap()[i] = Some(line);
            });
        }
    });
    jobs.into_iter().zip(results.into_inner().unwrap()).map(|((id, _), r)| (id, r.unwrap_or_default())).collect()
}

fn run_child(mut cmd: Command, timeout: Duration) -> String {
    let mut child = match cmd.spawn() {
        Ok(c) => c,
        Err(e) => return format!("ERROR spawning: {e}"),
    };
    let read = |mut r: Box<dyn Read + Send>| {
        std::thread::spawn(move || {
            let mut s = String::new();
            let _ = r.read_to_string(&mut s);
            s
        })
    };
    let out = read(Box::new(child.stdout.take().unwrap()));
    let err = read(Box::new(child.stderr.take().unwrap()));
    let began = Instant::now();
    let status = loop {
        match child.try_wait() {
            Ok(Some(s)) => break Some(s),
            Ok(None) if began.elapsed() > timeout => {
                let _ = child.kill();
                let _ = child.wait();
                break None;
            }
            Ok(None) => std::thread::sleep(Duration::from_millis(50)),
            Err(_) => break None,
        }
    };
    let (out, err) = (out.join().unwrap_or_default(), err.join().unwrap_or_default());
    if let Some(l) = out.lines().find_map(|l| l.strip_prefix(RESULT)) {
        return l.to_string();
    }
    let why = err.lines().rev().find(|l| l.contains("panicked") || l.contains("Error") || l.contains("error")).or(err.lines().last()).unwrap_or("").chars().take(300).collect::<String>();
    match status {
        None => format!("ERROR timed out after {} s", timeout.as_secs()),
        Some(s) => format!("ERROR {s}: {why}"),
    }
}

pub fn main(args: Vec<String>, prefix: &[&str], factory: &dyn Factory, defaults: &Defaults) -> Result<(), Error> {
    let prefix: Vec<String> = prefix.iter().map(|s| s.to_string()).collect();
    let Some(cmd) = args.first() else {
        println!("{USAGE}");
        return Ok(());
    };
    let o = parse(&args[1..], defaults)?;
    match cmd.as_str() {
        "record" => {
            let corpus: Vec<Entry> = read_corpus(&o.corpus)?.into_iter().filter(|e| wanted(&o, &e.id)).collect();
            let began = Instant::now();
            let jobs = corpus
                .iter()
                .map(|e| {
                    let tmx = e.tmx.map_or("-".into(), |t| t.to_string());
                    let args = vec!["one-record".into(), o.dir.display().to_string(), e.id.clone(), e.map.display().to_string(), tmx, e.source.clone(), e.ticks.to_string()];
                    (e.id.clone(), args)
                })
                .collect();
            let results = fan_out(&prefix, jobs, &o);
            let ok = results.iter().filter(|(_, r)| r.starts_with("OK")).count();
            let table: String = results.iter().map(|(id, r)| format!("{id}\t{r}\n")).collect();
            std::fs::create_dir_all(&o.dir)?;
            let log = o.dir.join("record.tsv");
            let mut lines: std::collections::BTreeMap<String, String> = std::fs::read_to_string(&log)
                .unwrap_or_default()
                .lines()
                .filter_map(|l| l.split_once('\t').map(|(a, b)| (a.to_string(), b.to_string())))
                .collect();
            for (id, r) in &results {
                lines.insert(id.clone(), r.clone());
            }
            std::fs::write(&log, lines.iter().map(|(a, b)| format!("{a}\t{b}\n")).collect::<String>())?;
            print!("{table}");
            println!("recorded {ok} of {} in {:.0} s ({})", results.len(), began.elapsed().as_secs_f64(), o.dir.display());
            Ok(())
        }
        "check" => {
            let files: Vec<PathBuf> = traces(&o.dir).into_iter().filter(|p| wanted(&o, &p.to_string_lossy())).collect();
            if files.is_empty() {
                return Err(format!("no traces under {}", o.dir.display()).into());
            }
            let began = Instant::now();
            let jobs = files
                .iter()
                .map(|p| {
                    let id = p.strip_prefix(&o.dir).unwrap_or(p).to_string_lossy().trim_end_matches(".trace.zst").to_string();
                    let mut args = vec!["one-check".to_string(), p.display().to_string(), "--dir".into(), o.dir.display().to_string()];
                    for m in &o.maps {
                        args.extend(["--maps".to_string(), m.display().to_string()]);
                    }
                    if let Some((t, w)) = o.perturb {
                        args.extend(["--perturb".to_string(), format!("{t}:{w}")]);
                    }
                    (id, args)
                })
                .collect();
            let results = fan_out(&prefix, jobs, &o);
            let exact = results.iter().filter(|(_, r)| r.starts_with("EXACT")).count();
            for (id, r) in &results {
                println!("{id}\t{r}");
            }
            println!("exact {exact} of {} in {:.0} s ({})", results.len(), began.elapsed().as_secs_f64(), factory.name());
            if exact == results.len() { Ok(()) } else { Err(format!("{} cases not exact", results.len() - exact).into()) }
        }
        "list" => {
            for p in traces(&o.dir).into_iter().filter(|p| wanted(&o, &p.to_string_lossy())) {
                let t = Trace::read(&p)?;
                let tmx = t.case.map.tmx.map_or(String::new(), |id| format!(" tmx {id}"));
                println!("{}\t{}{tmx}\t{}\t{} states\t{}", t.case.id, t.case.map.file, t.case.source, t.states.len(), t.engine);
            }
            Ok(())
        }
        "show" => {
            let [path, ticks @ ..] = &o.rest[..] else { return Err("show TRACE [TICK]...".into()) };
            let t = Trace::read(Path::new(path))?;
            let c = &t.case;
            println!("{} on {} ({:016x}), {}: T0 {} seed {:?}, from race {} ms, {} steps, by {}", c.id, c.map.file, c.map.hash, c.source, c.t0, c.seed, c.start_ms, c.steps.len(), t.engine);
            for k in ticks {
                let k: usize = k.parse()?;
                let s = t.states.get(k).ok_or("tick out of range")?;
                let step = k.checked_sub(1).and_then(|i| c.steps.get(i));
                println!("tick {k} (after {step:?}): {s:#?}");
            }
            Ok(())
        }
        "one-record" => {
            let [dir, id, map, tmx, source, ticks] = &o.rest[..] else { return Err("one-record DIR ID MAP TMX SOURCE TICKS".into()) };
            let e = Entry::parse(&[id.as_str(), map, tmx, source, ticks].join("\t"))?;
            let began = Instant::now();
            let line = match record_case(factory, &e, o.packs.clone()) {
                Ok((t, stopped)) => {
                    let bytes = t.write(&trace_path(Path::new(dir), id))?;
                    let stopped = stopped.map_or(String::new(), |s| format!(", ended early: {}", s.replace('\n', " ")));
                    format!("OK {} states, {bytes} bytes, {:.1} s{stopped}", t.states.len(), began.elapsed().as_secs_f64())
                }
                Err(e) => format!("ERROR {}", e.to_string().replace('\n', " ")),
            };
            println!("{RESULT}{line}");
            Ok(())
        }
        "one-check" => {
            let [path] = &o.rest[..] else { return Err("one-check TRACE".into()) };
            let line = check_case(factory, Path::new(path), &o.dir, &o.maps, o.packs.clone(), o.perturb).unwrap_or_else(|e| format!("ERROR {}", e.to_string().replace('\n', " ")));
            println!("{RESULT}{line}");
            Ok(())
        }
        _ => Err(format!("unknown golden command {cmd}\n{USAGE}").into()),
    }
}
