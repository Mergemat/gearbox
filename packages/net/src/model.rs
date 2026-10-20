use std::path::Path;

use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::Controls;

use crate::observe;

const MAGIC: &[u8; 4] = b"GNET";
const FORMAT: u32 = 1;

pub const ACTIONS: [Controls; 11] = [
    Controls { gas: 1.0, brake: 0.0, steer: -1.0 },
    Controls { gas: 1.0, brake: 0.0, steer: -0.5 },
    Controls { gas: 1.0, brake: 0.0, steer: 0.0 },
    Controls { gas: 1.0, brake: 0.0, steer: 0.5 },
    Controls { gas: 1.0, brake: 0.0, steer: 1.0 },
    Controls { gas: 0.0, brake: 1.0, steer: -1.0 },
    Controls { gas: 0.0, brake: 1.0, steer: -0.5 },
    Controls { gas: 0.0, brake: 1.0, steer: 0.0 },
    Controls { gas: 0.0, brake: 1.0, steer: 0.5 },
    Controls { gas: 0.0, brake: 1.0, steer: 1.0 },
    Controls { gas: 0.0, brake: 0.0, steer: 0.0 },
];

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
    pub logits: [f32; ACTIONS.len()],
    pub seconds_left: f32,
}

impl Verdict {
    pub fn best(&self) -> usize {
        (0..ACTIONS.len()).fold(0, |best, i| if self.logits[i] > self.logits[best] { i } else { best })
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
        let actions = word()? as usize;
        ensure!(actions == ACTIONS.len(), "network has {actions} actions, this build has {}", ACTIONS.len());
        let count = word()? as usize;
        let mut layers = Vec::with_capacity(count);
        for _ in 0..count {
            let (inputs, outputs) = (word()? as usize, word()? as usize);
            let mut floats =
                |n: usize| -> Result<Vec<f32>> { (0..n).map(|_| word().map(f32::from_bits)).collect() };
            let weights = floats(inputs * outputs)?;
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
        ensure!(
            outputs == ACTIONS.len() + 1,
            "network gives {outputs} outputs, expected {}",
            ACTIONS.len() + 1
        );
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
        Verdict { logits: std::array::from_fn(|i| x[i]), seconds_left: x[ACTIONS.len()] }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn file(version: u32, layers: &[(usize, usize, Vec<f32>, Vec<f32>)]) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        for w in [FORMAT, version, ACTIONS.len() as u32, layers.len() as u32] {
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
        let outs = ACTIONS.len() + 1;
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
        assert_eq!(v.logits[3], 3.0);
        assert_eq!(v.best(), 3);
        assert_eq!(v.seconds_left, 0.0);
        x[1] = -2.0;
        assert_eq!(net.eval(&x).seconds_left, 7.5);
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
        layers[1].2 = vec![0.0; 3 * (ACTIONS.len() + 1)];
        assert!(Net::read(&file(observe::VERSION, &layers)).is_err());
    }
}
