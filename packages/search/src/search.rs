use std::cmp::Ordering;
use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::{Session, Savestate};
use rand::{Rng, SeedableRng, rngs::StdRng};

use crate::input::Timeline;
use crate::mutation::Mutation;
use crate::objective::{Checkpoint, EvalOptions, Evaluation, Objective, Recorder, Resume, Score, evaluate_resumable, evaluate_with};

const FOCUS_BEFORE: usize = 300;
const FOCUS_AFTER: usize = 50;
const FOCUS_MIN_WINDOW: usize = 600;
const FOCUS_SHARE: f64 = 0.7;

fn goals_reached(s: &Score) -> usize {
    match s {
        Score::Goals { times, .. } => times.len(),
        _ => 0,
    }
}

const TRIES_SHOWN: usize = 12;

pub const MAX_THREADS: usize = 64;

pub fn default_threads() -> usize {
    std::thread::available_parallelism().map_or(4, |n| n.get().saturating_sub(1)).clamp(1, MAX_THREADS)
}

#[derive(Clone, Debug)]
pub enum Method {
    HillClimb,
    Population {
        width: usize,
        curve_ticks: usize,
    },
    Receding {
        horizon_ticks: usize,
        commit_ticks: usize,
        width: usize,
        curve_ticks: usize,
    },
}
impl Default for Method {
    fn default() -> Self {
        Self::Population {
            width: 8,
            curve_ticks: 20,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub window_start_ms: i32,
    pub window_end_ms: i32,
    pub until_ms: i32,
    pub objective: Objective,
    pub iterations: usize,
    pub threads: usize,
    pub batch_size: usize,
    pub seed: u64,
    pub mutation: Mutation,
    pub method: Method,
    pub evaluation: EvalOptions,
}

#[derive(Debug)]
pub struct SearchResult {
    pub inputs: Timeline,
    pub before: Evaluation,
    pub after: Evaluation,
    pub iterations: usize,
    pub improvements: usize,
    pub evaluated_ticks: u64,
    pub elapsed: Duration,
    pub survivors: Vec<Timeline>,
}

pub struct Progress<'a> {
    pub iterations: usize,
    pub improvements: usize,
    pub score: &'a Score,
    pub elapsed: Duration,
    pub tries: &'a [(Vec<[f32; 3]>, bool)],
}

struct Job {
    parents: Arc<Vec<Candidate>>,
    next: Arc<AtomicUsize>,
    end: usize,
    best: Option<Score>,
}

#[derive(Clone)]
struct Candidate {
    attempt: usize,
    inputs: Timeline,
    evaluation: Evaluation,
    checkpoints: Vec<Arc<Checkpoint>>,
}

struct Batch {
    candidates: Vec<Candidate>,
    ticks: u64,
}

pub trait Origin: Sync {
    fn native(&self) -> Result<Session>;
}

impl Origin for Session {
    fn native(&self) -> Result<Session> { Ok(self.clone()) }
}

impl Origin for Savestate {
    fn native(&self) -> Result<Session> { Ok(self.fork()) }
}

pub fn search_from(
    origin: &dyn Origin,
    base: Timeline,
    config: &Config,
    progress: impl FnMut(Progress<'_>),
) -> Result<SearchResult> {
    search_seeded(origin, base, Vec::new(), config, progress)
}

pub fn search_seeded(
    origin: &dyn Origin,
    base: Timeline,
    seeds: Vec<Timeline>,
    config: &Config,
    mut progress: impl FnMut(Progress<'_>),
) -> Result<SearchResult> {
    ensure!(
        (1..=MAX_THREADS).contains(&config.threads),
        "threads must be in 1..={MAX_THREADS}"
    );
    ensure!(
        config.evaluation.trick_bonus.is_finite() && config.evaluation.trick_bonus >= 0.0,
        "invalid trick bonus"
    );
    let evaluation = EvalOptions {
        from_ms: Some(config.window_start_ms),
        trace: !matches!(config.method, Method::HillClimb) || config.evaluation.trace,
        ..config.evaluation
    };
    ensure!(
        config.iterations > 0 && config.batch_size > 0,
        "iterations and batch size must be positive"
    );
    config.mutation.validate()?;
    config
        .objective
        .validate(config.window_start_ms, config.until_ms)?;
    let window = base.index(config.window_start_ms)?..base.index(config.window_end_ms)?;
    ensure!(window.start < window.end, "input window must be nonempty");
    ensure!(
        config.window_end_ms <= config.until_ms,
        "input window extends past evaluation end"
    );
    base.index(config.until_ms)?;
    if let Method::Receding { .. } = config.method {
        return receding(origin, base, config, &mut progress);
    }
    let (width, curve_ticks) = match config.method {
        Method::HillClimb => (1, 1),
        Method::Population { width, curve_ticks } => (width, curve_ticks),
        Method::Receding { .. } => unreachable!(),
    };
    ensure!(
        width > 0 && curve_ticks > 0,
        "population and curve size must be positive"
    );

    let mut native = origin.native()?;
    native.seed = base.seed;
    let start = base.index(native.race_ms())?;
    ensure!(
        start <= window.start,
        "window starts before the start state's race time {}",
        native.race_ms()
    );
    let ticks_to_end = (i64::from(config.until_ms) - i64::from(native.race_ms())) / 10;
    ensure!(
        ticks_to_end >= 0
            && u64::from(native.clock.now) + ticks_to_end as u64 * 10 <= u64::from(u32::MAX),
        "native clock would overflow"
    );
    if !matches!(config.objective, Objective::Goals(_)) {
        for c in &base.controls[start..window.start] {
            native.step(*c);
        }
    }
    if let Some(race) = native.race() {
        ensure!(
            config.objective.race_time(&race).is_none(),
            "target already reached before the mutable window"
        );
    }
    let prefix = native.save();
    let every = evaluation.checkpoint_every as usize;
    let mut rec = Recorder { from: window.start, to: window.end, every, out: Vec::new() };
    let before = evaluate_resumable(
        &mut native,
        &base,
        &config.objective,
        config.until_ms,
        evaluation,
        None,
        None,
        (every > 0).then_some(&mut rec),
    )?;
    let before_checkpoints: Vec<Arc<Checkpoint>> = rec.out.into_iter().map(Arc::new).collect();
    let mut incumbent = base;
    let mut first = vec![Candidate {
        attempt: usize::MAX,
        inputs: incumbent.clone(),
        evaluation: before.clone(),
        checkpoints: before_checkpoints.clone(),
    }];
    if !matches!(config.method, Method::HillClimb) {
        for (i, s) in seeds.into_iter().take(width.saturating_sub(1)).enumerate() {
            if s.t0 != incumbent.t0 || s.controls.len() != incumbent.controls.len() || s.controls == incumbent.controls {
                continue;
            }
            native.restore(&prefix);
            let mut rec = Recorder { from: window.start, to: window.end, every, out: Vec::new() };
            let evaluation = evaluate_resumable(&mut native, &s, &config.objective, config.until_ms, evaluation, None, None, (every > 0).then_some(&mut rec))?;
            first.push(Candidate { attempt: usize::MAX - 1 - i, inputs: s, evaluation, checkpoints: rec.out.into_iter().map(Arc::new).collect() });
        }
    }
    let mut parents = Arc::new(first);
    let mut after = before.clone();
    let mut after_checkpoints = before_checkpoints;
    let (mut iterations, mut improvements, mut evaluated_ticks) = (0, 0, 0);
    let began = Instant::now();

    std::thread::scope(|scope| -> Result<()> {
        let mut workers = Vec::new();
        let mut handles = Vec::new();
        for _ in 0..config.threads {
            let (tx, rx) = mpsc::channel::<Job>();
            let (result_tx, result_rx) = mpsc::channel::<Batch>();
            let prefix = &prefix;
            let window = window.clone();
            handles.push(scope.spawn(move || -> Result<()> {
                let mut native = origin.native()?;
                while let Ok(job) = rx.recv() {
                    native.seed = job.parents[0].inputs.seed;
                    let mut result = Batch {
                        candidates: Vec::new(),
                        ticks: 0,
                    };
                    loop {
                        let attempt = job.next.fetch_add(1, AtomicOrdering::Relaxed);
                        if attempt >= job.end {
                            break;
                        }
                        let parent_index = if attempt % 2 == 0 {
                            0
                        } else {
                            (attempt / 2) % job.parents.len()
                        };
                        let parent = &job.parents[parent_index];
                        let mut inputs = parent.inputs.clone();
                        let mut rng =
                            StdRng::seed_from_u64(config.seed.wrapping_add(attempt as u64));
                        let frontier = parent
                            .evaluation
                            .frontier_ms
                            .and_then(|ms| parent.inputs.index(ms).ok())
                            .map(|f| f.saturating_sub(FOCUS_BEFORE).max(window.start)..(f + FOCUS_AFTER).min(window.end))
                            .filter(|r| window.len() >= FOCUS_MIN_WINDOW && r.len() > 10);
                        let edit = match frontier {
                            Some(f) if rng.random_bool(FOCUS_SHARE) => f,
                            _ => window.clone(),
                        };
                        if matches!(config.method, Method::HillClimb) {
                            config
                                .mutation
                                .apply(&mut inputs.controls, edit, &mut rng);
                        } else {
                            config.mutation.explore(
                                &mut inputs,
                                edit,
                                &parent.evaluation.trace,
                                curve_ticks,
                                attempt,
                                &mut rng,
                            );
                        }
                        config.mutation.constraints.apply(
                            &parent.inputs.controls,
                            &mut inputs.controls,
                            window.clone(),
                        );
                        let first = (window.start..window.end)
                            .find(|&k| inputs.controls[k] != parent.inputs.controls[k])
                            .unwrap_or(window.end);
                        let at = parent.checkpoints.iter().rposition(|c| c.k <= first);
                        let mut rec = Recorder { from: window.start, to: window.end, every, out: Vec::new() };
                        let resume = at.map(|i| Resume { cp: &parent.checkpoints[i], parent: &parent.evaluation });
                        if resume.is_none() {
                            native.restore(prefix);
                        }
                        let evaluation = evaluate_resumable(
                            &mut native,
                            &inputs,
                            &config.objective,
                            config.until_ms,
                            evaluation,
                            job.best.as_ref(),
                            resume,
                            (every > 0).then_some(&mut rec),
                        )?;
                        result.ticks += evaluation.ticks;
                        let mut checkpoints: Vec<Arc<Checkpoint>> = at.map_or(Vec::new(), |i| parent.checkpoints[..=i].to_vec());
                        checkpoints.extend(rec.out.into_iter().map(Arc::new));
                        let candidate = Candidate {
                            attempt,
                            inputs,
                            evaluation,
                            checkpoints,
                        };
                        result.candidates.push(candidate);
                    }
                    if result_tx.send(result).is_err() {
                        break;
                    }
                }
                Ok(())
            }));
            workers.push((tx, result_rx));
        }

        let outcome = (|| -> Result<()> {
            while iterations < config.iterations {
                let count = config.batch_size.min(config.iterations - iterations);
                let active = config.threads.min(count);
                let next = Arc::new(AtomicUsize::new(iterations));
                for (tx, _) in &workers[..active] {
                    tx.send(Job {
                        parents: parents.clone(),
                        next: next.clone(),
                        end: iterations + count,
                        best: after.score.reached().then(|| after.score.clone()),
                    })
                    .context("search worker stopped")?;
                }
                let mut candidates = (*parents).clone();
                for (_, rx) in &workers[..active] {
                    let batch = rx
                        .recv()
                        .context("search worker failed during evaluation")?;
                    evaluated_ticks += batch.ticks;
                    candidates.extend(batch.candidates);
                }
                iterations += count;
                candidates.sort_by(|a, b| {
                    b.evaluation
                        .score
                        .quality_cmp(&a.evaluation.score)
                        .then_with(|| a.attempt.cmp(&b.attempt))
                });
                let tries: Vec<(Vec<[f32; 3]>, bool)> = candidates
                    .iter()
                    .filter(|c| !c.evaluation.path.is_empty())
                    .take(TRIES_SHOWN)
                    .map(|c| (c.evaluation.path.clone(), c.evaluation.score.reached()))
                    .collect();
                let best = &candidates[0];
                if best.evaluation.score.quality_cmp(&after.score) == Ordering::Greater {
                    incumbent = best.inputs.clone();
                    after = best.evaluation.clone();
                    after_checkpoints = best.checkpoints.clone();
                    improvements += 1;
                }
                let lead = goals_reached(&candidates[0].evaluation.score);
                candidates.retain(|c| goals_reached(&c.evaluation.score) >= lead);
                let apart = (window.len() / 50).max(2);
                let mut survivors: Vec<Candidate> = Vec::new();
                for candidate in candidates {
                    let differs = |c: &Candidate| {
                        c.inputs.controls[window.clone()]
                            .iter()
                            .zip(&candidate.inputs.controls[window.clone()])
                            .filter(|(x, y)| x != y)
                            .take(apart)
                            .count()
                            >= apart
                    };
                    if survivors.iter().all(differs) {
                        survivors.push(candidate);
                        if survivors.len() == width {
                            break;
                        }
                    }
                }
                parents = if matches!(config.method, Method::HillClimb) {
                    Arc::new(vec![Candidate {
                        attempt: usize::MAX,
                        inputs: incumbent.clone(),
                        evaluation: after.clone(),
                        checkpoints: after_checkpoints.clone(),
                    }])
                } else {
                    Arc::new(survivors)
                };
                progress(Progress {
                    iterations,
                    improvements,
                    score: &after.score,
                    elapsed: began.elapsed(),
                    tries: &tries,
                });
            }
            Ok(())
        })();
        drop(workers);
        for handle in handles {
            match handle.join() {
                Ok(result) => result.context("native search worker")?,
                Err(_) => bail!("native search worker panicked; no result exported"),
            }
        }
        outcome
    })?;
    let elapsed = began.elapsed();

    verify_replay(
        origin,
        &incumbent,
        &config.objective,
        config.until_ms,
        evaluation,
        &after,
    )?;
    let survivors = parents.iter().filter(|c| c.inputs.controls != incumbent.controls).map(|c| c.inputs.clone()).collect();
    Ok(SearchResult {
        inputs: incumbent,
        before,
        after,
        iterations,
        improvements,
        evaluated_ticks,
        elapsed,
        survivors,
    })
}

fn receding(
    origin: &dyn Origin,
    base: Timeline,
    config: &Config,
    progress: &mut dyn FnMut(Progress<'_>),
) -> Result<SearchResult> {
    let Method::Receding {
        horizon_ticks,
        commit_ticks,
        width,
        curve_ticks,
    } = config.method
    else {
        unreachable!()
    };
    ensure!(
        horizon_ticks > 0 && commit_ticks > 0 && commit_ticks <= horizon_ticks,
        "commit must be in 1..=horizon"
    );
    ensure!(
        config.objective.is_race()
            || matches!(
                config.objective,
                Objective::Path { .. } | Objective::Goals(_)
            ),
        "receding search requires goals, a gate, race or path target"
    );
    let start = base.index(config.window_start_ms)?;
    let end = base.index(config.window_end_ms)?;
    ensure!(start < end, "empty receding window");
    let segments = (end - start).div_ceil(commit_ticks);
    ensure!(
        config.iterations >= segments,
        "need at least one attempt per committed segment"
    );
    let mut n = origin.native()?;
    n.seed = base.seed;
    let original = n.save();
    let eval = EvalOptions {
        from_ms: Some(config.window_start_ms),
        ..config.evaluation
    };
    let before = evaluate_with(&mut n, &base, &config.objective, config.until_ms, eval)?;
    n.restore(&original);
    let initial = base.index(n.race_ms())?;
    ensure!(initial <= start, "window precedes origin");
    for c in &base.controls[initial..start] {
        n.step(*c);
    }
    ensure!(
        n.race()
            .is_none_or(|r| config.objective.race_time(&r).is_none()),
        "target already reached before the mutable window"
    );
    let began = Instant::now();
    let mut inputs = base.clone();
    let (mut iterations, mut improvements, mut evaluated_ticks) = (0, 0, 0);
    for (segment, at) in (start..end).step_by(commit_ticks).enumerate() {
        if n.race()
            .is_some_and(|r| config.objective.race_time(&r).is_some())
        {
            break;
        }
        let mut local = config.clone();
        local.method = Method::Population { width, curve_ticks };
        local.window_start_ms = inputs.t0 + 10 * at as i32;
        local.window_end_ms = inputs.t0 + 10 * (at + horizon_ticks).min(end) as i32;
        local.until_ms = local.window_end_ms;
        if let Objective::Path { at_ms, .. } = &mut local.objective {
            *at_ms = local.until_ms;
        }
        local.iterations =
            config.iterations / segments + usize::from(segment < config.iterations % segments);
        local.seed = config
            .seed
            .wrapping_add((segment as u64).wrapping_mul(0x9e3779b97f4a7c15));
        let saved = if matches!(config.objective, Objective::Goals(_)) {
            original.clone()
        } else {
            n.save()
        };
        let result = search_from(&saved, inputs, &local, |p| {
            progress(Progress {
                iterations: iterations + p.iterations,
                improvements: improvements + p.improvements,
                score: p.score,
                elapsed: began.elapsed(),
                tries: p.tries,
            })
        })?;
        iterations += result.iterations;
        improvements += result.improvements;
        evaluated_ticks += result.evaluated_ticks;
        inputs = result.inputs;
        for c in &inputs.controls[at..(at + commit_ticks).min(end)] {
            n.step(*c);
        }
    }
    n.restore(&original);
    let mut after = evaluate_with(&mut n, &inputs, &config.objective, config.until_ms, eval)?;
    if after.score.quality_cmp(&before.score) == Ordering::Less {
        inputs = base;
        after = before.clone();
        improvements = 0;
    }
    verify_replay(
        origin,
        &inputs,
        &config.objective,
        config.until_ms,
        eval,
        &after,
    )?;
    Ok(SearchResult {
        inputs,
        before,
        after,
        iterations,
        improvements,
        evaluated_ticks,
        elapsed: began.elapsed(),
        survivors: Vec::new(),
    })
}
fn verify_replay(
    origin: &dyn Origin,
    inputs: &Timeline,
    objective: &Objective,
    until: i32,
    options: EvalOptions,
    expected: &Evaluation,
) -> Result<()> {
    let mut n = origin.native()?;
    n.seed = inputs.seed;
    let actual = evaluate_with(&mut n, inputs, objective, until, options)?;
    ensure!(
        actual.score == expected.score
            && actual.digest == expected.digest
            && actual.body.0.map(f32::to_bits) == expected.body.0.map(f32::to_bits),
        "winner differs on fresh replay"
    );
    Ok(())
}
