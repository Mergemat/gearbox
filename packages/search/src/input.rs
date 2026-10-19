use std::io::Write;
use std::path::Path;

use anyhow::{Context, Result, ensure};
use gearbox_engine::Controls;

#[derive(Clone, Debug)]
pub struct Timeline {
    pub t0: i32,
    pub seed: Option<u32>,
    pub controls: Vec<Controls>,
}

impl Timeline {
    pub fn read(path: &Path) -> Result<Self> {
        Self::parse(&std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?)
    }

    pub fn parse(text: &str) -> Result<Self> {
        let inputs = gearbox_trace::Timeline::parse(text).map_err(anyhow::Error::msg)?;
        ensure!(!inputs.controls.is_empty(), "no input ticks");
        let timeline = Self {
            t0: inputs.t0,
            seed: inputs.seed,
            controls: inputs
                .controls
                .into_iter()
                .map(|c| Controls { gas: c.gas, brake: c.brake, steer: c.steer })
                .collect(),
        };
        timeline.end_ms()?;
        Ok(timeline)
    }

    pub fn starting_at(&self, t0: i32) -> Result<Self> {
        let gap = i64::from(self.t0) - i64::from(t0);
        ensure!(gap >= 0 && gap % 10 == 0, "inputs at T0 {} cannot start at {t0}", self.t0);
        let mut controls = vec![Controls::default(); usize::try_from(gap / 10)?];
        controls.extend_from_slice(&self.controls);
        Ok(Self { t0, seed: self.seed, controls })
    }

    pub fn end_ms(&self) -> Result<i32> {
        let end = i64::from(self.t0) + i64::try_from(self.controls.len())? * 10;
        i32::try_from(end).context("input timeline exceeds i32 race time")
    }

    pub fn index(&self, ms: i32) -> Result<usize> {
        let delta = i64::from(ms) - i64::from(self.t0);
        ensure!(
            delta >= 0 && delta % 10 == 0,
            "time {ms} is outside the input tick grid starting at {}",
            self.t0
        );
        let index = usize::try_from(delta / 10)?;
        ensure!(index <= self.controls.len(), "time {ms} exceeds input coverage");
        Ok(index)
    }

    pub fn write(&self, path: &Path) -> Result<()> {
        let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
        write!(out, "T0 {}", self.t0)?;
        if let Some(seed) = self.seed {
            write!(out, " {seed}")?;
        }
        writeln!(out)?;
        for c in &self.controls {
            writeln!(out, "{} {} {}", c.gas, c.brake, c.steer)?;
        }
        out.flush()?;
        Ok(())
    }
}
