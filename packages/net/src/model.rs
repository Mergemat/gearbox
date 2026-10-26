use std::path::Path;

use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::Controls;

use crate::observe;

const MAGIC: &[u8; 4] = b"GNET";
pub const FORMAT: u32 = 2;

pub const STEER_BINS: usize = 11;
pub const OUTPUTS: usize = STEER_BINS + 3;

#[derive(Clone, Debug)]
struct Layer {
    inputs: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

#[derive(Clone, Debug)]
pub struct Net {
    layers: Vec<Layer>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Verdict {
    pub steer_logits: [f32; STEER_BINS],
    pub gas: f32,
    pub brake: f32,
    pub seconds_left: f32,
}

impl Verdict {
    pub fn controls(&self) -> Controls {
        let bin = (0..STEER_BINS)
            .fold(0, |best, i| if self.steer_logits[i] > self.steer_logits[best] { i } else { best });
        Controls {
            steer: -1.0 + 0.2 * bin as f32,
            gas: f32::from(self.gas > 0.0),
            brake: f32::from(self.brake > 0.0),
        }
    }
}

impl Net {
    pub fn load(path: &Path) -> Result<Self> {
        Self::read(&std::fs::read(path).with_context(|| format!("reading {}", path.display()))?)
            .with_context(|| format!("loading {}", path.display()))
    }

    pub fn read(bytes: &[u8]) -> Result<Self> {
        let mut at = 0;
        let mut word = || -> Result<u32> {
            let b = bytes.get(at..at + 4).context("network file is truncated")?;
            at += 4;
            Ok(u32::from_le_bytes(b.try_into().unwrap()))
        };
        ensure!(word()?.to_le_bytes() == *MAGIC, "not a network file");
        let format = word()?;
        ensure!(format == FORMAT, "network file format {format}, this build reads {FORMAT}");
        let version = word()?;
        if version != observe::VERSION {
            bail!(
                "network was trained on observation version {version}, this build has {}",
                observe::VERSION
            );
        }
        let outputs = word()? as usize;
        ensure!(outputs == OUTPUTS, "network has {outputs} outputs, this build has {OUTPUTS}");
        let count = word()? as usize;
        ensure!(count > 0 && count <= bytes.len() / 8, "invalid layer count {count}");
        let mut layers = Vec::with_capacity(count);
        for _ in 0..count {
            let (inputs, outputs) = (word()? as usize, word()? as usize);
            let mut floats =
                |n: usize| -> Result<Vec<f32>> { (0..n).map(|_| word().map(f32::from_bits)).collect() };
            ensure!(inputs > 0 && outputs > 0, "empty layer");
            let size = inputs.checked_mul(outputs).context("layer size overflow")?;
            ensure!(size <= bytes.len() / 4 && outputs <= bytes.len() / 4, "layer exceeds file size");
            let weights = floats(size)?;
            let bias = floats(outputs)?;
            layers.push(Layer { inputs, weights, bias });
        }
        ensure!(at == bytes.len(), "{} bytes after the last layer", bytes.len() - at);
        let net = Self { layers };
        net.check()?;
        Ok(net)
    }

    fn check(&self) -> Result<()> {
        let first = self.layers.first().context("network has no layers")?;
        ensure!(
            first.inputs == observe::LEN,
            "network reads {} inputs, observations have {}",
            first.inputs,
            observe::LEN
        );
        for pair in self.layers.windows(2) {
            ensure!(pair[0].bias.len() == pair[1].inputs, "layer sizes don't chain");
        }
        let outputs = self.layers.last().unwrap().bias.len();
        ensure!(outputs == OUTPUTS, "network gives {outputs} outputs, expected {}", OUTPUTS);
        Ok(())
    }

    pub fn eval(&self, observation: &[f32; observe::LEN]) -> Verdict {
        let mut x = observation.to_vec();
        for (i, layer) in self.layers.iter().enumerate() {
            let hidden = i + 1 < self.layers.len();
            x = layer
                .bias
                .iter()
                .zip(layer.weights.chunks_exact(layer.inputs))
                .map(|(b, row)| {
                    let y = b + row.iter().zip(&x).map(|(w, v)| w * v).sum::<f32>();
                    if hidden { y.max(0.0) } else { y }
                })
                .collect();
        }
        Verdict {
            steer_logits: std::array::from_fn(|i| x[i]),
            gas: x[STEER_BINS],
            brake: x[STEER_BINS + 1],
            seconds_left: x[STEER_BINS + 2].exp_m1(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn file(version: u32, layers: &[(usize, usize, Vec<f32>, Vec<f32>)]) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        for w in [FORMAT, version, OUTPUTS as u32, layers.len() as u32] {
            out.extend(w.to_le_bytes());
        }
        for (inputs, outputs, weights, bias) in layers {
            out.extend((*inputs as u32).to_le_bytes());
            out.extend((*outputs as u32).to_le_bytes());
            for v in weights.iter().chain(bias) {
                out.extend(v.to_le_bytes());
            }
        }
        out
    }

    fn two_layers() -> Vec<(usize, usize, Vec<f32>, Vec<f32>)> {
        let n = observe::LEN;
        let mut first = vec![0.0; 2 * n];
        first[0] = 2.0;
        first[n + 1] = -1.0;
        let outs = OUTPUTS;
        let mut second = vec![0.0; outs * 2];
        second[3 * 2] = 1.0;
        second[(outs - 1) * 2 + 1] = 3.0;
        vec![(n, 2, first, vec![0.0, 0.5]), (2, outs, second, vec![0.0; outs])]
    }

    #[test]
    fn hidden_layers_clip_at_zero_and_the_last_one_does_not() {
        let net = Net::read(&file(observe::VERSION, &two_layers())).unwrap();
        let mut x = [0.0; observe::LEN];
        x[0] = 1.5;
        x[1] = 2.0;
        let v = net.eval(&x);
        assert_eq!(v.steer_logits[3], 3.0);
        assert!((v.controls().steer + 0.4).abs() < 1e-6);
        assert_eq!(v.seconds_left, 0.0);
        x[1] = -2.0;
        assert_eq!(net.eval(&x).seconds_left, 7.5f32.exp_m1());
    }

    #[test]
    fn a_network_for_another_observation_version_is_refused() {
        let err = Net::read(&file(observe::VERSION + 1, &two_layers())).unwrap_err();
        assert!(format!("{err}").contains("observation version"), "{err}");
    }

    #[test]
    fn layers_that_dont_chain_are_refused() {
        let mut layers = two_layers();
        layers[1].0 = 3;
        layers[1].2 = vec![0.0; 3 * OUTPUTS];
        assert!(Net::read(&file(observe::VERSION, &layers)).is_err());
    }

    #[test]
    fn steering_and_pedals_are_chosen_independently() {
        for (bin, expected) in
            [-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0].into_iter().enumerate()
        {
            for (gas, brake, pedals) in
                [(-1.0, 0.0, [0.0, 0.0]), (0.01, -1.0, [1.0, 0.0]), (1.0, 2.0, [1.0, 1.0])]
            {
                let mut verdict = Verdict { steer_logits: [0.0; STEER_BINS], gas, brake, seconds_left: -1.0 };
                verdict.steer_logits[bin] = 1.0;
                let controls = verdict.controls();
                assert!((controls.steer - expected).abs() < 1e-6);
                assert_eq!([controls.gas, controls.brake], pedals);
            }
        }
    }

    #[test]
    fn old_formats_bad_sizes_and_incomplete_files_are_refused() {
        let valid = file(observe::VERSION, &two_layers());
        for (offset, value) in [(4, 1u32), (12, 12), (16, 0), (20, 34), (24, 0), (24, u32::MAX)] {
            let mut bytes = valid.clone();
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            assert!(Net::read(&bytes).is_err());
        }
        for end in [0, 4, 20, 24, valid.len() - 1] {
            assert!(Net::read(&valid[..end]).is_err());
        }
        let mut trailing = valid;
        trailing.push(0);
        assert!(Net::read(&trailing).is_err());
    }
}
