use std::path::Path;

use anyhow::{Context, Result};
use gearbox_engine::Controls;

#[derive(Clone, Debug, PartialEq)]
pub struct InputFile {
    pub t0: i32,
    pub seed: Option<u32>,
    pub controls: Vec<Controls>,
}

impl InputFile {
    pub fn timeline(&self) -> gearbox_search::input::Timeline {
        gearbox_search::input::Timeline { t0: self.t0, seed: self.seed, controls: self.controls.clone() }
    }

    pub fn blank(t0: i32) -> Self {
        Self { t0, seed: None, controls: Vec::new() }
    }

    pub fn trimmed(&self) -> Self {
        let n = self.controls.iter().rposition(|c| *c != Controls::default()).map_or(0, |i| i + 1);
        Self { t0: self.t0, seed: self.seed, controls: self.controls[..n].to_vec() }
    }

    pub fn covering(&self, t0: i32, end_ms: i32) -> Self {
        let front = if t0 < self.t0 && (self.t0 - t0) % 10 == 0 { ((self.t0 - t0) / 10) as usize } else { 0 };
        let mut controls = vec![Controls::default(); front];
        controls.extend_from_slice(&self.controls);
        let mut f = Self { t0: self.t0 - 10 * front as i32, seed: self.seed, controls };
        while f.end_ms() < end_ms {
            f.controls.push(Controls::default());
        }
        f
    }

    pub fn parse(text: &str) -> Result<Self> {
        let timeline = gearbox_trace::Timeline::parse(text).map_err(anyhow::Error::msg)?;
        Ok(Self {
            t0: timeline.t0,
            seed: timeline.seed,
            controls: timeline
                .controls
                .into_iter()
                .map(|c| Controls { gas: c.gas, brake: c.brake, steer: c.steer })
                .collect(),
        })
    }

    pub fn read(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        Self::parse(&text).with_context(|| format!("parsing {}", path.display()))
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

    pub fn index(&self, race_ms: i32) -> Option<usize> {
        let d = race_ms - self.t0;
        (d >= 0).then_some((d / 10) as usize)
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

    pub fn end_ms(&self) -> i32 {
        self.t0 + 10 * self.controls.len() as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_write_round_trip_keeps_seed_and_exact_floats() {
        let text = "T0 -1510 123456\n0 0 0\n1 0 -0.9133858\n1 1 0.007874016\n";
        let f = InputFile::parse(text).unwrap();
        assert_eq!(f.seed, Some(123456));
        assert_eq!(f.controls[1].steer.to_bits(), (-0.9133858f32).to_bits());
        assert_eq!(f.to_text(), text);
    }

    #[test]
    fn trimmed_drops_only_trailing_neutral_ticks() {
        let f = InputFile::parse("T0 -1500 7\n0 0 0\n1 0 0.5\n0 0 0\n0 0 0\n").unwrap();
        assert_eq!(f.trimmed().to_text(), "T0 -1500 7\n0 0 0\n1 0 0.5\n");
        assert!(InputFile::parse("T0 0\n0 0 0\n").unwrap().trimmed().controls.is_empty());
    }

    #[test]
    fn covering_pads_with_neutral_and_keeps_every_tick() {
        let f = InputFile::parse("T0 -400 9\n1 0 -1\n0 1 0.25\n").unwrap();
        let g = f.covering(-1500, 100);
        assert_eq!((g.t0, g.seed, g.end_ms()), (-1500, Some(9), 100));
        for ms in (-1600..200).step_by(10) {
            assert_eq!(g.at(ms), f.at(ms), "{ms}");
        }
    }

    #[test]
    fn set_before_t0_is_ignored_and_after_end_extends() {
        let mut f = InputFile::parse("T0 100\n1 0 0\n").unwrap();
        f.set(50, Controls { gas: 1.0, brake: 0.0, steer: 1.0 });
        assert_eq!(f.controls.len(), 1);
        f.set(130, Controls { gas: 0.0, brake: 1.0, steer: -1.0 });
        assert_eq!(f.controls.len(), 4);
        assert_eq!(f.at(110), Controls::default());
        assert_eq!(f.at(135).brake, 1.0);
        assert_eq!(f.at(90), Controls::default());
    }
}
