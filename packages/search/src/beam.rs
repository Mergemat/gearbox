use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Barrier, Mutex};

use crate::Body;
use anyhow::{Result, ensure};
use gearbox_engine::Controls;
use gearbox_engine::Savestate;

use crate::goals::{Goal, Goals, Progress, goal_point};
use crate::input::Timeline;
use crate::pose::{Pose, PoseGoal, rotation_distance};
use crate::search::Origin;

#[derive(Clone, Debug)]
pub struct BeamConfig {
    pub width: usize,
    pub chunk: usize,
    pub fine_chunk: usize,
    pub fine_ms: f64,
    pub threads: usize,
    pub actions: Vec<Controls>,
    pub cell_m: f32,
    pub sectors: u32,
    pub kmh: f32,
    pub explore: usize,
    pub coarse: (f32, u32, f32),
    pub rest_speed: f64,
    pub speed_cap: f64,
    pub passes: usize,
    pub learn: (f32, u32, f32),
    pub rollback_ms: f64,
    pub back: usize,
    pub first_arrival: bool,
    pub first: (f32, u32, f32),
}

impl Default for BeamConfig {
    fn default() -> Self {
        let steers = [-1.0, -0.5, 0.0, 0.5, 1.0];
        let mut actions: Vec<Controls> =
            steers.iter().map(|&steer| Controls { gas: 1.0, brake: 0.0, steer }).collect();
        actions.extend(steers.iter().map(|&steer| Controls { gas: 0.0, brake: 1.0, steer }));
        actions.push(Controls::default());
        Self {
            width: 256,
            chunk: 10,
            fine_chunk: 2,
            fine_ms: 600.0,
            threads: crate::search::default_threads(),
            actions,
            cell_m: 0.5,
            sectors: 360,
            kmh: 2.0,
            explore: 0,
            coarse: (2.0, 64, 5.0),
            rest_speed: 60.0,
            speed_cap: f64::INFINITY,
            passes: 1,
            learn: (3.0, 16, 25.0),
            rollback_ms: 0.0,
            back: 200,
            first_arrival: false,
            first: (4.0, 16, 20.0),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Built {
    pub inputs: Timeline,
    pub times: Vec<i32>,
    pub speed_kmh: f32,
}

#[derive(Clone, Debug)]
pub struct BeamProgress {
    pub race_ms: i32,
    pub kept: usize,
    pub ticks: u64,
    pub best: Option<(i32, f32)>,
    pub pass: usize,
    pub lead_eta_ms: f64,
    pub lead: Option<(usize, [f32; 3], f32)>,
}

#[derive(Clone)]
struct Node {
    state: Arc<Savestate>,
    goals: Progress,
    line: Arc<Vec<Controls>>,
    id: usize,
}

struct TreeNode {
    parent: Option<usize>,
    race_ms: i32,
    key: Key,
    own: f64,
    finished: f64,
    dropped: f64,
}

type Key = (usize, [i32; 3], i32, i32);

struct Child {
    parent: usize,
    action: usize,
    goals: Progress,
    body: Body,
    value: f64,
    measured: f64,
    cps: usize,
}

fn motion(b: Body) -> (f64, f64) {
    let v = b.local_velocity();
    let speed = f64::from(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let r = b.rotation();
    (speed, f64::from(r[2]).atan2(f64::from(r[8])))
}

fn legs(goals: &Goals) -> Vec<f64> {
    let mut rest = vec![0.0; goals.ordered.len()];
    for i in (0..goals.ordered.len().saturating_sub(1)).rev() {
        let (a, b) = (goal_point(&goals.ordered[i]), goal_point(&goals.ordered[i + 1]));
        let field = goals.track.as_ref().and_then(|t| t.goals.get(i + 1)).and_then(Option::as_ref);
        let leg = match (a, field) {
            (Some(a), Some(f)) => f.distance_within(a, 4),
            _ => None,
        };
        let leg = leg.or_else(|| Some(crate::objective::distance(a?, b?))).unwrap_or(0.0);
        rest[i] = leg + rest[i + 1];
    }
    rest
}

fn closing_speed(goals: &Goals, p: &Progress, body: Body) -> f64 {
    let (speed, _) = motion(body);
    let Some(field) = goals
        .track
        .as_ref()
        .and_then(|t| t.goals.get(p.times.len()).and_then(Option::as_ref).map(|f| (f, t.reach)))
    else {
        return speed;
    };
    let (f, reach) = field;
    let at = body.position();
    let h = 3.0f32;
    let mut grad = [0.0f64; 3];
    for (a, g) in grad.iter_mut().enumerate() {
        let (mut lo, mut hi) = (at, at);
        lo[a] -= h;
        hi[a] += h;
        if let (Some(l), Some(r)) = (f.distance_within(lo, reach), f.distance_within(hi, reach)) {
            *g = (r - l) / f64::from(2.0 * h);
        }
    }
    let norm = grad.iter().map(|g| g * g).sum::<f64>().sqrt();
    if norm < 1e-6 {
        return speed;
    }
    let v = body.local_velocity().map(f64::from);
    -(0..3).map(|a| v[a] * grad[a]).sum::<f64>() / norm
}

fn eta(
    goals: &Goals,
    rest: &[f64],
    rest_speed: f64,
    speed_cap: f64,
    p: &Progress,
    race_ms: i32,
    body: Body,
) -> f64 {
    let next = p.times.len();
    if let Some(Goal::Pose { target }) = goals.ordered.get(next) {
        return f64::from(race_ms) + 1000.0 * (pose_time(goals, p, target, body) + rest[next] / rest_speed);
    }
    let speed = closing_speed(goals, p, body);
    f64::from(race_ms) + 1000.0 * (p.now.min(1e5) / speed.clamp(15.0, speed_cap) + rest[next] / rest_speed)
}

const TURN_RATE: f64 = 2.0;
const SPEED_RATE: f64 = 10.0;
const BEND_RATE: f64 = 30.0;

const POSE_NEAR_M: f64 = 15.0;
const POSE_NEAR_COST: f64 = 4.0;

fn pose_time(goals: &Goals, p: &Progress, target: &PoseGoal, body: Body) -> f64 {
    let at = Pose::from_body(body);
    let off: [f64; 3] =
        std::array::from_fn(|k| f64::from(target.pose.position[k]) - f64::from(at.position[k]));
    let excess: [f64; 3] =
        std::array::from_fn(|k| (off[k].abs() - f64::from(target.position_tolerance[k])).max(0.0));
    let straight = excess.iter().map(|e| e * e).sum::<f64>().sqrt();
    let v = at.velocity.map(f64::from);
    let field = goals
        .track
        .as_ref()
        .and_then(|t| t.goals.get(p.times.len()).and_then(Option::as_ref).map(|f| (f, t.reach)));
    let d = off.iter().map(|o| o * o).sum::<f64>().sqrt();
    let (way, closing) = if d < POSE_NEAR_M {
        (straight, if d > 1e-6 { (0..3).map(|k| v[k] * off[k]).sum::<f64>() / d } else { 0.0 })
    } else if goals.along.is_some() {
        (p.now.min(1e5), motion(body).0)
    } else if let Some((f, reach)) = field {
        (
            f.distance_within(at.position, reach).map_or(straight, |d| d.max(straight)),
            closing_speed(goals, p, body),
        )
    } else {
        (straight, (0..3).map(|k| v[k] * off[k]).sum::<f64>() / d)
    };
    let t_way = if way > 0.0 { way / closing.max(5.0) } else { 0.0 };
    let t_turn = (rotation_distance(target.pose.rotation, at.rotation) - f64::from(target.angle_tolerance))
        .max(0.0)
        / TURN_RATE;
    let t_speed = target.velocity_tolerance.filter(|_| d < 2.0 * POSE_NEAR_M).map_or(0.0, |tol| {
        let want = target.pose.velocity.map(f64::from);
        let norm = |x: [f64; 3]| x.iter().map(|c| c * c).sum::<f64>().sqrt();
        let (have, need, slack) = (norm(v), norm(want), norm(tol.map(f64::from)));
        let speed = ((have - need).abs() - slack).max(0.0) / SPEED_RATE;
        let bend = if have > 1.0 && need > 1.0 {
            let cos = ((0..3).map(|k| v[k] * want[k]).sum::<f64>() / (have * need)).clamp(-1.0, 1.0);
            (cos.acos() - (slack / need).atan()).max(0.0) * have / BEND_RATE
        } else {
            0.0
        };
        speed.max(bend)
    });
    let near = if d < POSE_NEAR_M { POSE_NEAR_COST } else { 1.0 };
    t_way.max(near * t_turn).max(near * t_speed)
}

fn learn(tree: &[TreeNode], learned: &mut HashMap<Key, f64>) {
    let mut kids = vec![f64::INFINITY; tree.len()];
    let mut went_on = vec![false; tree.len()];
    let mut backed = vec![f64::INFINITY; tree.len()];
    for id in (0..tree.len()).rev() {
        let t = &tree[id];
        backed[id] = if went_on[id] || t.finished.is_finite() {
            t.finished.min(kids[id])
        } else {
            t.dropped.min(t.own)
        };
        if let Some(p) = t.parent {
            kids[p] = kids[p].min(backed[id]);
            went_on[p] = true;
        }
    }
    let mut fresh: HashMap<Key, f64> = HashMap::new();
    for (t, b) in tree.iter().zip(backed).filter(|(_, b)| b.is_finite()) {
        let ttg = b - f64::from(t.race_ms);
        fresh.entry(t.key).and_modify(|v| *v = v.min(ttg)).or_insert(ttg);
    }
    learned.extend(fresh);
}

const MAX_ROLLBACKS: usize = 20;

fn finished_value(goals: &Goals, times: &[i32], body: Body) -> f64 {
    let t = f64::from(*times.last().unwrap_or(&0));
    let kmh = motion(body).0 * 3.6;
    if goals.fastest { -kmh * 1e3 + t * 1e-3 } else { t - goals.speed_worth_ms * kmh }
}

pub fn build_line(
    origin: &dyn Origin,
    base: &Timeline,
    from_k: usize,
    until_k: usize,
    goals: &Goals,
    config: &BeamConfig,
    stop: &(dyn Fn() -> bool + Sync),
    mut progress: impl FnMut(&BeamProgress),
) -> Result<Option<Built>> {
    goals.validate()?;
    ensure!(
        !config.actions.is_empty() && config.width > 0 && config.chunk > 0,
        "the line builder needs inputs to try, a width and a chunk"
    );
    ensure!(until_k <= base.controls.len() && from_k < until_k, "the range must lie inside the run");
    let mut native = origin.native()?;
    native.seed = base.seed;
    let start = base.index(native.race_ms())?;
    ensure!(start <= from_k, "the range starts before the start state");
    let mut p = Progress::new(native.race_ms());
    let body = Body::from(native.body());
    p.observe(goals, &mut native, body);
    for c in &base.controls[start..from_k] {
        native.step(*c);
        let body = Body::from(native.body());
        p.observe(goals, &mut native, body);
    }
    ensure!(p.times.len() < goals.ordered.len(), "the run already reaches every goal before the range");
    let root_ms = native.race_ms();
    let root_body = Body::from(native.body());
    let root_node = Node { state: Arc::new(native.save()), goals: p, line: Arc::new(Vec::new()), id: 0 };
    drop(native);

    let rest = legs(goals);
    let threads = config.threads.max(1);
    let mut best: Option<(f64, Built)> = None;
    let mut ticks = 0u64;

    struct Work {
        beam: Vec<Node>,
        jobs: Vec<(usize, usize)>,
        next: AtomicUsize,
        children: Mutex<Vec<Child>>,
        finished: Mutex<Vec<(f64, Vec<i32>, Body, usize, usize)>>,
        states: Mutex<Vec<Option<Arc<Savestate>>>>,
        save: bool,
        done: bool,
        chunk: usize,
    }
    let work: Mutex<Option<Arc<Work>>> = Mutex::new(None);
    let (go, back) = (Barrier::new(threads + 1), Barrier::new(threads + 1));
    let failure: Mutex<Option<anyhow::Error>> = Mutex::new(None);

    std::thread::scope(|scope| -> Result<()> {
        for _ in 0..threads {
            let (work, go, back, failure, rest) = (&work, &go, &back, &failure, &rest);
            scope.spawn(move || {
                let mut native = match origin.native() {
                    Ok(n) => Some(n),
                    Err(e) => {
                        *failure.lock().unwrap() = Some(e);
                        None
                    }
                };
                if let Some(n) = native.as_mut() {
                    n.seed = base.seed;
                }
                loop {
                    go.wait();
                    let w = work.lock().unwrap().clone().expect("work set before go");
                    if w.done {
                        break;
                    }
                    if let Some(n) = native.as_mut() {
                        let mut out = Vec::new();
                        let mut fin = Vec::new();
                        let mut saved = Vec::new();
                        loop {
                            let j = w.next.fetch_add(1, Ordering::Relaxed);
                            let Some(&(parent, action)) = w.jobs.get(j) else { break };
                            let node = &w.beam[parent];
                            n.restore(&node.state);
                            let mut g = node.goals.clone();
                            let c = config.actions[action];
                            let mut body = Body::from(n.body());
                            let mut ok = true;
                            for _ in 0..w.chunk {
                                n.step(c);
                                body = Body::from(n.body());
                                if !body.0.iter().all(|v| v.is_finite()) {
                                    ok = false;
                                    break;
                                }
                                g.observe(goals, n, body);
                                if g.times.len() == goals.ordered.len() {
                                    break;
                                }
                            }
                            if w.save {
                                saved.push((j, Arc::new(n.save())));
                                continue;
                            }
                            if !ok {
                                continue;
                            }
                            if g.times.len() == goals.ordered.len() {
                                fin.push((
                                    finished_value(goals, &g.times, body),
                                    g.times.clone(),
                                    body,
                                    parent,
                                    action,
                                ));
                            } else {
                                let value = eta(
                                    goals,
                                    rest,
                                    config.rest_speed,
                                    config.speed_cap,
                                    &g,
                                    n.race_ms(),
                                    body,
                                );
                                let cps = n.race().map_or(0, |r| r.checkpoints.len());
                                out.push(Child {
                                    parent,
                                    action,
                                    goals: g,
                                    body,
                                    value,
                                    measured: value,
                                    cps,
                                });
                            }
                        }
                        w.children.lock().unwrap().extend(out);
                        w.finished.lock().unwrap().extend(fin);
                        let mut s = w.states.lock().unwrap();
                        for (j, d) in saved {
                            s[j] = Some(d);
                        }
                    }
                    drop(w);
                    back.wait();
                }
            });
        }

        let run = |w: Work| -> Work {
            *work.lock().unwrap() = Some(Arc::new(w));
            go.wait();
            back.wait();
            let w = work.lock().unwrap().take().expect("work set");
            Arc::into_inner(w).expect("workers dropped their work")
        };
        let learn_key = |reached: usize, body: Body| -> Key {
            let (cell, sectors, kmh) = config.learn;
            let (speed, heading) = motion(body);
            let sector = ((heading / std::f64::consts::TAU + 0.5) * f64::from(sectors)).floor() as i32;
            (
                reached,
                body.position().map(|v| (v / cell).floor() as i32),
                sector,
                (speed * 3.6 / f64::from(kmh)).floor() as i32,
            )
        };
        let mut learned: HashMap<Key, f64> = HashMap::new();
        let mut earliest: HashMap<Key, i32> = HashMap::new();
        let outcome = (|| -> Result<()> {
            for pass in 0..config.passes.max(1) {
                if stop() {
                    break;
                }
                let mut tree = vec![TreeNode {
                    parent: None,
                    race_ms: root_ms,
                    key: learn_key(root_node.goals.times.len(), root_body),
                    own: f64::INFINITY,
                    finished: f64::INFINITY,
                    dropped: f64::INFINITY,
                }];
                let mut beam = vec![root_node.clone()];
                let mut k = from_k;
                let mut saved: Vec<(usize, Vec<Node>, u32)> = Vec::new();
                let mut recent: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
                let mut tries: HashMap<usize, u32> = HashMap::new();
                let mut went_back = 0usize;
                let mut lead_left = f64::INFINITY;
                while !beam.is_empty() && k < until_k && !stop() {
                    let chunk =
                        if lead_left < config.fine_ms { config.fine_chunk.max(1) } else { config.chunk };
                    let jobs: Vec<(usize, usize)> = (0..beam.len())
                        .flat_map(|i| (0..config.actions.len()).map(move |a| (i, a)))
                        .collect();
                    let w = run(Work {
                        beam: std::mem::take(&mut beam),
                        jobs,
                        next: AtomicUsize::new(0),
                        children: Mutex::new(Vec::new()),
                        finished: Mutex::new(Vec::new()),
                        states: Mutex::new(Vec::new()),
                        save: false,
                        done: false,
                        chunk,
                    });
                    if let Some(e) = failure.lock().unwrap().take() {
                        return Err(e);
                    }
                    ticks += (w.jobs.len() * chunk) as u64;
                    let parents = w.beam;
                    let mut finished = w.finished.into_inner().unwrap();
                    finished.sort_by(|a, b| a.0.total_cmp(&b.0).then((a.3, a.4).cmp(&(b.3, b.4))));
                    for (value, times, body, parent, action) in finished {
                        let t = &mut tree[parents[parent].id];
                        t.finished = t.finished.min(f64::from(*times.last().unwrap_or(&0)));
                        if best.as_ref().is_none_or(|b| value < b.0) {
                            let mut inputs = base.clone();
                            let line = &parents[parent].line;
                            inputs.controls[from_k..from_k + line.len()].copy_from_slice(line);
                            let end = (from_k + line.len() + chunk).min(inputs.controls.len());
                            inputs.controls[from_k + line.len()..end].fill(config.actions[action]);
                            best = Some((
                                value,
                                Built { inputs, times, speed_kmh: (motion(body).0 * 3.6) as f32 },
                            ));
                        }
                    }
                    let mut children = w.children.into_inner().unwrap();
                    let child_ms = root_ms + 10 * (k + chunk - from_k) as i32;
                    let first_key = |c: &Child| {
                        let (cell, sectors, kmh) = config.first;
                        let (speed, heading) = motion(c.body);
                        let sector =
                            ((heading / std::f64::consts::TAU + 0.5) * f64::from(sectors)).floor() as i32;
                        (
                            c.cps,
                            c.body.position().map(|v| (v / cell).floor() as i32),
                            sector,
                            (speed * 3.6 / f64::from(kmh)).floor() as i32,
                        )
                    };
                    for c in &mut children {
                        let reached = if config.first_arrival { c.cps } else { c.goals.times.len() };
                        if config.first_arrival {
                            let late = earliest.get(&first_key(c)).map_or(0, |t| child_ms - t);
                            c.value = -1e9 * c.cps as f64 + f64::from(late) - motion(c.body).0 * 1e-3;
                            c.measured = f64::from(child_ms);
                        }
                        if let Some(ttg) = learned.get(&learn_key(reached, c.body)) {
                            c.value = f64::from(child_ms) + ttg;
                        }
                    }
                    children.sort_by(|a, b| {
                        a.value.total_cmp(&b.value).then((a.parent, a.action).cmp(&(b.parent, b.action)))
                    });
                    let key = |c: &Child, cell: f32, sectors: u32, kmh: f32| {
                        let p = c.body.position();
                        let (speed, heading) = motion(c.body);
                        let sector =
                            ((heading / std::f64::consts::TAU + 0.5) * f64::from(sectors)).floor() as i32;
                        let [x, y, z] = p.map(|v| (v / cell).floor() as i32);
                        (x, y, z, sector, (speed * 3.6 / f64::from(kmh)).floor() as i32)
                    };
                    let mut taken = vec![false; children.len()];
                    let mut seen = HashMap::new();
                    let mut coarse = HashMap::new();
                    for (i, c) in children.iter().enumerate() {
                        if coarse.len() == config.explore.min(config.width) {
                            break;
                        }
                        let (cell, sectors, kmh) = config.coarse;
                        if coarse.insert(key(c, cell, sectors, kmh), ()).is_none() {
                            seen.insert(key(c, config.cell_m, config.sectors, config.kmh), ());
                            taken[i] = true;
                        }
                    }
                    let mut kept_n = coarse.len();
                    for (i, c) in children.iter().enumerate() {
                        if kept_n == config.width {
                            break;
                        }
                        if !taken[i]
                            && seen.insert(key(c, config.cell_m, config.sectors, config.kmh), ()).is_none()
                        {
                            taken[i] = true;
                            kept_n += 1;
                        }
                    }
                    let bound = best
                        .as_ref()
                        .filter(|_| !goals.fastest && goals.speed_worth_ms == 0.0)
                        .map_or(f64::INFINITY, |b| b.0);
                    let mut kept = Vec::new();
                    for (c, t) in children.into_iter().zip(taken) {
                        if t && c.measured < bound {
                            kept.push(c);
                        } else {
                            let t = &mut tree[parents[c.parent].id];
                            t.dropped = t.dropped.min(c.value);
                        }
                    }
                    if config.first_arrival {
                        for c in &kept {
                            earliest.entry(first_key(c)).or_insert(child_ms);
                        }
                    }
                    let lead = kept.first().map_or(f64::INFINITY, |c| c.value);
                    lead_left =
                        kept.iter().map(|c| c.measured).fold(f64::INFINITY, f64::min) - f64::from(child_ms);
                    let lead_measured = kept.iter().map(|c| c.measured).fold(f64::INFINITY, f64::min);
                    let lead_car = kept
                        .first()
                        .map(|c| (c.goals.times.len(), c.body.position(), (motion(c.body).0 * 3.6) as f32));
                    let jobs: Vec<(usize, usize)> = kept.iter().map(|c| (c.parent, c.action)).collect();
                    let n = jobs.len();
                    let w = run(Work {
                        beam: parents,
                        jobs,
                        next: AtomicUsize::new(0),
                        children: Mutex::new(Vec::new()),
                        finished: Mutex::new(Vec::new()),
                        states: Mutex::new(vec![None; n]),
                        save: true,
                        done: false,
                        chunk,
                    });
                    if let Some(e) = failure.lock().unwrap().take() {
                        return Err(e);
                    }
                    ticks += (n * chunk) as u64;
                    let states = w.states.into_inner().unwrap();
                    beam = kept
                        .into_iter()
                        .zip(states)
                        .map(|(c, s)| {
                            let parent = &w.beam[c.parent];
                            let mut line = (*parent.line).clone();
                            line.extend(std::iter::repeat_n(config.actions[c.action], chunk));
                            tree.push(TreeNode {
                                parent: Some(parent.id),
                                race_ms: child_ms,
                                key: learn_key(
                                    if config.first_arrival { c.cps } else { c.goals.times.len() },
                                    c.body,
                                ),
                                own: c.value,
                                finished: f64::INFINITY,
                                dropped: f64::INFINITY,
                            });
                            Node {
                                state: s.expect("every kept car saved"),
                                goals: c.goals,
                                line: Arc::new(line),
                                id: tree.len() - 1,
                            }
                        })
                        .collect();
                    k += chunk;
                    if config.rollback_ms > 0.0 {
                        if (k - from_k).is_multiple_of(100) {
                            saved.push((k, beam.clone(), 0));
                        }
                        let low = recent.iter().copied().fold(f64::INFINITY, f64::min);
                        recent.push_back(lead_measured);
                        if recent.len() > 100 / config.chunk {
                            recent.pop_front();
                        }
                        if lead_measured > low + config.rollback_ms
                            && went_back < MAX_ROLLBACKS
                            && let Some(i) = saved.iter().rposition(|s| {
                                s.0 + config.back <= k && tries.get(&s.0).is_none_or(|t| *t < 2)
                            })
                        {
                            learn(&tree, &mut learned);
                            saved.truncate(i + 1);
                            let s = &saved[i];
                            *tries.entry(s.0).or_default() += 1;
                            went_back += 1;
                            (k, beam) = (s.0, s.1.clone());
                            recent.clear();
                        }
                    }
                    progress(&BeamProgress {
                        race_ms: root_ms + 10 * (k - from_k) as i32,
                        kept: beam.len(),
                        ticks,
                        best: best.as_ref().map(|b| (*b.1.times.last().unwrap_or(&0), b.1.speed_kmh)),
                        pass,
                        lead_eta_ms: lead,
                        lead: lead_car,
                    });
                }
                learn(&tree, &mut learned);
            }
            Ok(())
        })();
        *work.lock().unwrap() = Some(Arc::new(Work {
            beam: Vec::new(),
            jobs: Vec::new(),
            next: AtomicUsize::new(0),
            children: Mutex::new(Vec::new()),
            finished: Mutex::new(Vec::new()),
            states: Mutex::new(Vec::new()),
            save: false,
            done: true,
            chunk: 0,
        }));
        go.wait();
        outcome
    })?;
    Ok(best.map(|b| b.1))
}
