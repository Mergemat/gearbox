use std::ops::Range;

use anyhow::{Result, ensure};
use gearbox_engine::Controls;
use rand::{Rng, RngExt};

#[derive(Clone, Debug)]
pub struct Mutation {
    pub max_changes: usize,
    pub max_span_ticks: usize,
    pub max_shift_ticks: usize,
    pub max_steer_delta: f32,
    pub constraints: Constraints,
}

impl Default for Mutation {
    fn default() -> Self {
        Self {
            max_changes: 2,
            max_span_ticks: 10,
            max_shift_ticks: 5,
            max_steer_delta: 0.1,
            constraints: Constraints::default(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Constraints {
    pub fixed: [bool; 3],
    pub digital_steer: bool,
    pub steer_limit: Option<f32>,
    pub locked: Vec<Range<usize>>,
}

impl Constraints {
    pub fn is_free(&self) -> bool {
        *self == Self::default()
    }

    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.steer_limit.is_none_or(|l| l.is_finite() && (0.0..=1.0).contains(&l)),
            "steer limit must be in [0, 1]"
        );
        ensure!(self.fixed != [true; 3], "every channel is fixed: nothing can change");
        Ok(())
    }

    pub fn apply(&self, parent: &[Controls], child: &mut [Controls], window: Range<usize>) {
        if self.is_free() {
            return;
        }
        let limit = self.steer_limit.unwrap_or(1.0);
        for i in window {
            let (Some(p), Some(c)) = (parent.get(i), child.get_mut(i)) else { break };
            if c == p {
                continue;
            }
            if self.locked.iter().any(|r| r.contains(&i)) {
                *c = *p;
                continue;
            }
            if self.fixed[0] {
                c.gas = p.gas;
            }
            if self.fixed[1] {
                c.brake = p.brake;
            }
            if self.fixed[2] {
                c.steer = p.steer;
            } else {
                if self.digital_steer {
                    c.steer = if c.steer > 0.5 * limit {
                        limit
                    } else if c.steer < -0.5 * limit {
                        -limit
                    } else {
                        0.0
                    };
                }
                c.steer = c.steer.clamp(-limit, limit);
            }
        }
    }
}

impl Mutation {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.max_changes > 0 && self.max_span_ticks > 0, "changes and span must be positive");
        ensure!(
            self.max_steer_delta.is_finite() && (0.0..=2.0).contains(&self.max_steer_delta),
            "steer delta must be finite and in [0, 2]"
        );
        self.constraints.validate()
    }

    pub fn apply(&self, controls: &mut [Controls], window: Range<usize>, rng: &mut impl Rng) {
        let c = &mut controls[window];
        for _ in 0..rng.random_range(1..=self.max_changes) {
            let start = rng.random_range(0..c.len());
            let end = start + rng.random_range(1..=self.max_span_ticks.min(c.len() - start));
            match rng.random_range(0..4) {
                0 => {
                    let delta = rng.random_range(-self.max_steer_delta..=self.max_steer_delta);
                    for c in &mut c[start..end] {
                        c.steer = (c.steer + delta).clamp(-1.0, 1.0);
                    }
                }
                1 => {
                    let gas = rng.random::<bool>();
                    let value = if gas { c[start].gas } else { c[start].brake };
                    let toggled = if value >= 0.5 { 0.0 } else { 1.0 };
                    for c in &mut c[start..end] {
                        if gas {
                            c.gas = toggled;
                        } else {
                            c.brake = toggled;
                        }
                    }
                }
                2 => self.shift_event(c, rng, false),
                _ => self.shift_event(c, rng, true),
            }
        }
    }

    pub fn shift(&self, controls: &mut [Controls], window: Range<usize>, rng: &mut impl Rng) {
        let c = &mut controls[window];
        for _ in 0..rng.random_range(1..=self.max_changes) {
            self.shift_event(c, rng, false);
        }
    }

    fn shift_event(&self, c: &mut [Controls], rng: &mut impl Rng, steer_only: bool) {
        if self.max_shift_ticks == 0 {
            return;
        }
        let channel = if steer_only { 2 } else { rng.random_range(0..3) };
        let get = |c: Controls| match channel {
            0 => c.gas,
            1 => c.brake,
            _ => c.steer,
        };
        let mut chosen = None;
        let mut seen = 0;
        for i in 1..c.len() {
            if get(c[i - 1]) != get(c[i]) {
                seen += 1;
                if rng.random_range(0..seen) == 0 {
                    chosen = Some(i);
                }
            }
        }
        let Some(i) = chosen else {
            return;
        };
        let shift = rng.random_range(1..=self.max_shift_ticks.min(c.len()));
        let (range, value) = if rng.random::<bool>() {
            (i..i + shift.min(c.len() - i), get(c[i - 1]))
        } else {
            (i.saturating_sub(shift)..i, get(c[i]))
        };
        for c in &mut c[range] {
            match channel {
                0 => c.gas = value,
                1 => c.brake = value,
                _ => c.steer = value,
            }
        }
    }
}

impl Mutation {
    pub fn explore(
        &self,
        inputs: &mut crate::input::Timeline,
        window: Range<usize>,
        trace: &[crate::objective::Sample],
        curve_ticks: usize,
        attempt: usize,
        rng: &mut impl Rng,
    ) {
        let t0 = inputs.t0;
        let controls = &mut inputs.controls;
        if attempt % 16 < 3 && controls[window.clone()].iter().all(|c| *c == Controls::default()) {
            let steer = [0.0, -0.1, 0.1][attempt % 16];
            for c in &mut controls[window] {
                *c = Controls { gas: 1.0, brake: 0.0, steer };
            }
            return;
        }
        if rng.random_range(0..3) == 0 {
            let first = rng.random_range(window.start..window.end);
            let last = (first + rng.random_range(1..=curve_ticks * 4)).min(window.end);
            for chunk in controls[first..last].chunks_mut(curve_ticks) {
                let steer = if rng.random_bool(0.5) { rng.random_range(-1.0..=1.0) } else { 0.0 };
                let gas = if rng.random_bool(0.85) { 1.0 } else { 0.0 };
                let brake = if rng.random_bool(0.15) { 1.0 } else { 0.0 };
                chunk.fill(Controls { gas, brake, steer });
            }
        } else {
            self.apply(controls, window.clone(), rng);
        }
        if rng.random_bool(0.5) {
            let kind = rng.random_range(0..4);
            let candidates: Vec<usize> = trace
                .iter()
                .enumerate()
                .filter_map(|(i, s)| {
                    let delta = i64::from(s.ms) - i64::from(t0);
                    if delta < 10 {
                        return None;
                    }
                    let tick = (delta / 10 - 1) as usize;
                    let o = s.observation;
                    let matches = match kind {
                        0 => true,
                        1 => o.sliding(),
                        2 => {
                            i > 0
                                && !trace[i - 1].observation.wheels.iter().any(|w| *w)
                                && o.wheels.iter().any(|w| *w)
                        }
                        _ => o.wall_contact,
                    };
                    (window.contains(&tick) && matches).then_some(tick)
                })
                .collect();
            if candidates.is_empty() {
                return;
            }
            let at = candidates[rng.random_range(0..candidates.len())];
            let at = if kind == 2 { at.saturating_sub(2).max(window.start) } else { at };
            let duration = if kind == 2 { rng.random_range(4..=12) } else { rng.random_range(2..=12) };
            let end = at.saturating_add(duration).min(window.end);
            let sign = if rng.random::<bool>() { 1.0 } else { -1.0 };
            for (j, c) in controls[at..end].iter_mut().enumerate() {
                match kind {
                    0 | 3 => c.steer = if j % 4 < 2 { sign } else { -sign },
                    1 => c.brake = if j < 3 { 1.0 } else { 0.0 },
                    _ => c.gas = 0.0,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(gas: f32, brake: f32, steer: f32) -> Controls {
        Controls { gas, brake, steer }
    }

    #[test]
    fn shifts_move_transitions_without_new_values() {
        use rand::SeedableRng;
        let parent: Vec<Controls> = (0..40)
            .map(|k| c(if (10..20).contains(&k) { 0.0 } else { 1.0 }, 0.0, if k < 25 { -0.5 } else { 0.7 }))
            .collect();
        let m = Mutation { max_changes: 3, max_shift_ticks: 6, ..Default::default() };
        let mut moved = false;
        for seed in 0..200 {
            let mut child = parent.clone();
            m.shift(&mut child, 5..35, &mut rand::rngs::StdRng::seed_from_u64(seed));
            assert_eq!(child[..5], parent[..5]);
            assert_eq!(child[35..], parent[35..]);
            for x in &child {
                assert!(
                    [0.0, 1.0].contains(&x.gas) && x.brake == 0.0 && [-0.5, 0.7].contains(&x.steer),
                    "{x:?}"
                );
            }
            let changes = |f: fn(&Controls) -> f32| child.windows(2).filter(|w| f(&w[0]) != f(&w[1])).count();
            assert!(changes(|x| x.gas) <= 2 && changes(|x| x.steer) <= 1);
            moved |= child != parent;
        }
        assert!(moved);
    }

    #[test]
    fn constraints_restore_fixed_channels_and_locked_ticks() {
        let parent = vec![c(1.0, 0.0, 0.2); 6];
        let mut child = vec![c(0.0, 1.0, -0.9); 6];
        let k = Constraints { fixed: [true, false, false], locked: vec![3..5], ..Default::default() };
        k.apply(&parent, &mut child, 1..6);
        assert_eq!(child[0], c(0.0, 1.0, -0.9));
        assert_eq!(child[1], c(1.0, 1.0, -0.9));
        assert_eq!(child[3], parent[3]);
        assert_eq!(child[4], parent[4]);
        assert_eq!(child[5], c(1.0, 1.0, -0.9));
    }

    #[test]
    fn digital_steer_rounds_to_the_nearest_of_minus_zero_plus_limit() {
        let parent = vec![c(0.0, 0.0, 0.0); 5];
        let mut child: Vec<Controls> = [-0.9, -0.3, 0.2, 0.31, 0.7].iter().map(|&s| c(0.0, 0.0, s)).collect();
        let k = Constraints { digital_steer: true, steer_limit: Some(0.6), ..Default::default() };
        k.apply(&parent, &mut child, 0..5);
        let steer: Vec<f32> = child.iter().map(|c| c.steer).collect();
        assert_eq!(steer, [-0.6, 0.0, 0.0, 0.6, 0.6]);
    }

    #[test]
    fn limits_leave_the_parents_own_inputs_alone() {
        let parent = vec![c(1.0, 0.0, -0.1); 5];
        let mut child = parent.clone();
        child[2].steer = 0.8;
        Constraints { digital_steer: true, ..Default::default() }.apply(&parent, &mut child, 0..5);
        let steer: Vec<f32> = child.iter().map(|c| c.steer).collect();
        assert_eq!(steer, [-0.1, -0.1, 1.0, -0.1, -0.1]);
    }

    #[test]
    fn mutated_candidates_stay_within_the_constraints() {
        use rand::SeedableRng;
        let k = Constraints {
            fixed: [false, true, false],
            steer_limit: Some(0.5),
            locked: vec![10..20],
            ..Default::default()
        };
        let m = Mutation { max_changes: 4, max_span_ticks: 30, constraints: k.clone(), ..Default::default() };
        let parent = vec![c(1.0, 0.0, 0.1); 60];
        for seed in 0..200 {
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
            let mut child = parent.clone();
            m.apply(&mut child, 5..50, &mut rng);
            k.apply(&parent, &mut child, 5..50);
            for (i, (p, ch)) in parent.iter().zip(&child).enumerate() {
                assert_eq!(ch.brake, p.brake, "seed {seed} tick {i}");
                assert!(ch.steer.abs() <= 0.5, "seed {seed} tick {i}: {}", ch.steer);
                if (10..20).contains(&i) || !(5..50).contains(&i) {
                    assert_eq!(ch, p, "seed {seed} tick {i}");
                }
            }
        }
    }
}
