use std::path::Path;

use anyhow::{Context, Result, ensure};
use gearbox_engine::Controls;

#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    pub t0: i32,
    pub seed: Option<u32>,
    pub controls: Vec<Controls>,
}

impl Run {
    pub fn blank(t0: i32) -> Self {
        Self { t0, seed: None, controls: Vec::new() }
    }

    pub fn read(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        Self::parse(&text).with_context(|| format!("parsing {}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Self> {
        let inputs = gearbox_trace::Timeline::parse(text).map_err(anyhow::Error::msg)?;
        let end = i64::from(inputs.t0) + i64::try_from(inputs.controls.len())? * 10;
        ensure!(i32::try_from(end).is_ok(), "input timeline exceeds i32 race time");
        Ok(Self {
            t0: inputs.t0,
            seed: inputs.seed,
            controls: inputs
                .controls
                .into_iter()
                .map(|c| Controls { gas: c.gas, brake: c.brake, steer: c.steer })
                .collect(),
        })
    }

    pub fn to_text(&self) -> String {
        let mut s = match self.seed {
            Some(seed) => format!("T0 {} {seed}\n", self.t0),
            None => format!("T0 {}\n", self.t0),
        };
        for c in &self.controls {
            s.push_str(&format!("{} {} {}\n", c.gas, c.brake, c.steer));
        }
        s
    }

    pub fn write(&self, path: &Path) -> Result<()> {
        std::fs::write(path, self.to_text()).with_context(|| format!("writing {}", path.display()))
    }

    pub fn end_ms(&self) -> i32 {
        self.t0 + 10 * self.controls.len() as i32
    }

    pub fn index(&self, race_ms: i32) -> Option<usize> {
        let d = race_ms - self.t0;
        (d >= 0).then_some((d / 10) as usize)
    }

    pub fn tick(&self, race_ms: i32) -> Result<usize> {
        let delta = i64::from(race_ms) - i64::from(self.t0);
        ensure!(
            delta >= 0 && delta % 10 == 0,
            "time {race_ms} is outside the input tick grid starting at {}",
            self.t0
        );
        let index = usize::try_from(delta / 10)?;
        ensure!(index <= self.controls.len(), "time {race_ms} exceeds input coverage");
        Ok(index)
    }

    pub fn at(&self, race_ms: i32) -> Controls {
        self.index(race_ms).and_then(|i| self.controls.get(i).copied()).unwrap_or_default()
    }

    pub fn set(&mut self, race_ms: i32, c: Controls) {
        let Some(i) = self.index(race_ms) else { return };
        if self.controls.len() <= i {
            self.controls.resize(i + 1, Controls::default());
        }
        self.controls[i] = c;
    }

    pub fn starting_at(&self, t0: i32) -> Result<Self> {
        let gap = i64::from(self.t0) - i64::from(t0);
        ensure!(gap >= 0 && gap % 10 == 0, "inputs at T0 {} cannot start at {t0}", self.t0);
        let mut controls = vec![Controls::default(); usize::try_from(gap / 10)?];
        controls.extend_from_slice(&self.controls);
        Ok(Self { t0, seed: self.seed, controls })
    }

    pub fn trimmed(&self) -> Self {
        let n = self.controls.iter().rposition(|c| *c != Controls::default()).map_or(0, |i| i + 1);
        Self { t0: self.t0, seed: self.seed, controls: self.controls[..n].to_vec() }
    }

    pub fn covering(&self, t0: i32, end_ms: i32) -> Self {
        let front = if t0 < self.t0 && (self.t0 - t0) % 10 == 0 { ((self.t0 - t0) / 10) as usize } else { 0 };
        let mut controls = vec![Controls::default(); front];
        controls.extend_from_slice(&self.controls);
        let mut run = Self { t0: self.t0 - 10 * front as i32, seed: self.seed, controls };
        while run.end_ms() < end_ms {
            run.controls.push(Controls::default());
        }
        run
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_write_round_trip_keeps_seed_and_exact_floats() {
        let text = "T0 -1510 123456\n0 0 0\n1 0 -0.9133858\n1 1 0.007874016\n";
        let run = Run::parse(text).unwrap();
        assert_eq!(run.seed, Some(123456));
        assert_eq!(run.controls[1].steer.to_bits(), (-0.9133858f32).to_bits());
        assert_eq!(run.to_text(), text);
    }

    #[test]
    fn a_run_with_no_ticks_round_trips() {
        let run = Run::parse("T0 -1500\n").unwrap();
        assert_eq!(run, Run::blank(-1500));
        assert_eq!(run.to_text(), "T0 -1500\n");
    }

    #[test]
    fn trimmed_drops_only_trailing_neutral_ticks() {
        let run = Run::parse("T0 -1500 7\n0 0 0\n1 0 0.5\n0 0 0\n0 0 0\n").unwrap();
        assert_eq!(run.trimmed().to_text(), "T0 -1500 7\n0 0 0\n1 0 0.5\n");
        assert!(Run::parse("T0 0\n0 0 0\n").unwrap().trimmed().controls.is_empty());
    }

    #[test]
    fn covering_pads_with_neutral_and_keeps_every_tick() {
        let run = Run::parse("T0 -400 9\n1 0 -1\n0 1 0.25\n").unwrap();
        let wide = run.covering(-1500, 100);
        assert_eq!((wide.t0, wide.seed, wide.end_ms()), (-1500, Some(9), 100));
        for ms in (-1600..200).step_by(10) {
            assert_eq!(wide.at(ms), run.at(ms), "{ms}");
        }
    }

    #[test]
    fn set_before_t0_is_ignored_and_after_end_extends() {
        let mut run = Run::parse("T0 100\n1 0 0\n").unwrap();
        run.set(50, Controls { gas: 1.0, brake: 0.0, steer: 1.0 });
        assert_eq!(run.controls.len(), 1);
        run.set(130, Controls { gas: 0.0, brake: 1.0, steer: -1.0 });
        assert_eq!(run.controls.len(), 4);
        assert_eq!(run.at(110), Controls::default());
        assert_eq!(run.at(135).brake, 1.0);
        assert_eq!(run.at(90), Controls::default());
    }

    #[test]
    fn tick_needs_the_grid_and_coverage_index_does_not() {
        let run = Run::parse("T0 -1450\n1 0 -0.5\n0 1 0.25\n").unwrap();
        assert_eq!(run.tick(-1440).unwrap(), 1);
        for ms in [-1460, -1449, -1420] {
            assert!(run.tick(ms).is_err(), "{ms}");
        }
        assert_eq!(run.index(-1420), Some(3));
        assert_eq!(run.index(-1460), None);
    }
}
