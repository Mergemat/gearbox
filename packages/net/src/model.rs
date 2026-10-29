use std::path::Path;

use anyhow::{Context, Result, ensure};
use gearbox_engine::Controls;

use crate::observe;

const MAGIC: &[u8; 4] = b"GNET";
pub const FORMAT: u32 = 5;
pub const PERIOD: u32 = 5;
pub const STEER: [f32; 11] = [-1.0, -0.65, -0.4, -0.2, -0.08, 0.0, 0.08, 0.2, 0.4, 0.65, 1.0];
pub const OUTPUTS: usize = STEER.len() + 2;

#[derive(Clone, Debug)]
struct Dense {
    outputs: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

#[derive(Clone, Debug)]
pub struct Net {
    version: u32,
    period: u32,
    mean: Vec<f32>,
    std: Vec<f32>,
    layers: Vec<Dense>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Verdict {
    pub steer_logits: [f32; STEER.len()],
    pub gas: f32,
    pub brake: f32,
}

impl Verdict {
    pub fn steer_bin(&self) -> usize {
        (0..STEER.len())
            .fold(0, |best, i| if self.steer_logits[i] > self.steer_logits[best] { i } else { best })
    }

    pub fn controls(&self) -> Controls {
        Controls {
            steer: STEER[self.steer_bin()],
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
        let mut r = Reader { bytes, at: 0 };
        ensure!(r.word()?.to_le_bytes() == *MAGIC, "not a network file");
        let format = r.word()?;
        ensure!(format == FORMAT, "network file format {format}, this build reads {FORMAT}");
        let version = r.word()?;
        let expected = observe::len(version).with_context(|| {
            format!("network was trained on observation version {version}, this build has 8 and 9")
        })?;
        let [len, period, steers] = r.words()?;
        ensure!(len == expected, "network reads {len} observations, version {version} has {expected}");
        ensure!(period > 0, "network has no decision period");
        ensure!(steers == STEER.len(), "network has {steers} steering bins, this build has {}", STEER.len());
        ensure!(r.floats(steers)? == STEER, "network steering bins differ from {STEER:?}");
        let mean = r.floats(len)?;
        let std = r.floats(len)?;
        ensure!(std.iter().all(|s| *s > 0.0 && s.is_finite()), "network input scale is not positive");
        let [count] = r.words()?;
        ensure!(count > 0 && count <= 16, "invalid layer count {count}");
        let mut layers: Vec<Dense> = Vec::with_capacity(count);
        let mut inputs = len;
        for _ in 0..count {
            let [i, outputs] = r.words()?;
            ensure!(i == inputs && outputs > 0, "layer sizes don't chain");
            let weights = r.floats(inputs.checked_mul(outputs).context("layer size overflow")?)?;
            let bias = r.floats(outputs)?;
            layers.push(Dense { outputs, weights, bias });
            inputs = outputs;
        }
        ensure!(inputs == OUTPUTS, "network gives {inputs} outputs, expected {OUTPUTS}");
        ensure!(r.at == bytes.len(), "{} bytes after the last layer", bytes.len() - r.at);
        Ok(Self { version, period: period as u32, mean, std, layers })
    }

    pub fn write(&self) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        let words = [FORMAT, self.version, self.inputs() as u32, self.period, STEER.len() as u32];
        out.extend(words.iter().flat_map(|w| w.to_le_bytes()));
        let floats = STEER.iter().chain(&self.mean).chain(&self.std);
        out.extend(floats.flat_map(|v| v.to_le_bytes()));
        out.extend((self.layers.len() as u32).to_le_bytes());
        let mut inputs = self.inputs();
        for layer in &self.layers {
            out.extend([inputs as u32, layer.outputs as u32].iter().flat_map(|w| w.to_le_bytes()));
            out.extend(layer.weights.iter().chain(&layer.bias).flat_map(|v| v.to_le_bytes()));
            inputs = layer.outputs;
        }
        out
    }

    pub fn widen(&self) -> Self {
        let mut net = self.clone();
        if self.version == 8 {
            let added = observe::LEN_V9 - observe::LEN;
            net.version = 9;
            net.mean.resize(observe::LEN_V9, 0.0);
            net.std.resize(observe::LEN_V9, 1.0);
            let first = &mut net.layers[0];
            first.weights.resize(first.weights.len() + added * first.outputs, 0.0);
        }
        net
    }

    pub fn version(&self) -> u32 {
        self.version
    }

    pub fn inputs(&self) -> usize {
        self.mean.len()
    }

    pub fn period(&self) -> u32 {
        self.period
    }

    pub fn eval(&self, observation: &[f32]) -> Verdict {
        assert!(observation.len() >= self.inputs(), "observation length");
        let observation = &observation[..self.inputs()];
        let mut x: Vec<f32> =
            observation.iter().zip(&self.mean).zip(&self.std).map(|((v, m), s)| (v - m) / s).collect();
        let mut y = Vec::new();
        for (i, layer) in self.layers.iter().enumerate() {
            y.clear();
            y.resize(layer.outputs, 0.0);
            for (&value, row) in x.iter().zip(layer.weights.chunks_exact(layer.outputs)) {
                if value != 0.0 {
                    for (sum, &weight) in y.iter_mut().zip(row) {
                        *sum += value * weight;
                    }
                }
            }
            let hidden = i + 1 < self.layers.len();
            for (sum, &bias) in y.iter_mut().zip(&layer.bias) {
                *sum += bias;
                if hidden {
                    *sum = sum.max(0.0);
                }
            }
            std::mem::swap(&mut x, &mut y);
        }
        Verdict {
            steer_logits: std::array::from_fn(|i| x[i]),
            gas: x[STEER.len()],
            brake: x[STEER.len() + 1],
        }
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn word(&mut self) -> Result<u32> {
        let b = self.bytes.get(self.at..self.at + 4).context("network file is truncated")?;
        self.at += 4;
        Ok(u32::from_le_bytes(b.try_into().unwrap()))
    }

    fn words<const N: usize>(&mut self) -> Result<[usize; N]> {
        let mut out = [0; N];
        for v in &mut out {
            *v = self.word()? as usize;
        }
        Ok(out)
    }

    fn floats(&mut self, n: usize) -> Result<Vec<f32>> {
        ensure!(n <= (self.bytes.len() - self.at) / 4, "network file is truncated");
        (0..n).map(|_| self.word().map(f32::from_bits)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn file(layers: &[(usize, usize, Vec<f32>, Vec<f32>)], mean: f32, std: f32) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        let words = [FORMAT, observe::VERSION, observe::LEN as u32, PERIOD, STEER.len() as u32];
        for w in words {
            out.extend(w.to_le_bytes());
        }
        let header = STEER.iter().copied().chain([mean; observe::LEN]).chain([std; observe::LEN]);
        for v in header {
            out.extend(v.to_le_bytes());
        }
        out.extend((layers.len() as u32).to_le_bytes());
        for (inputs, outputs, weights, bias) in layers {
            out.extend((*inputs as u32).to_le_bytes());
            out.extend((*outputs as u32).to_le_bytes());
            for v in weights.iter().chain(bias) {
                out.extend(v.to_le_bytes());
            }
        }
        out
    }

    fn values(count: usize, seed: u32) -> Vec<f32> {
        let mut bits = seed;
        (0..count)
            .map(|_| {
                bits = bits.wrapping_mul(1664525).wrapping_add(1013904223);
                ((bits >> 8) as f32 / 16777216.0 - 0.5) * 0.2
            })
            .collect()
    }

    fn two_layers() -> Vec<(usize, usize, Vec<f32>, Vec<f32>)> {
        let hidden = 24;
        vec![
            (observe::LEN, hidden, values(observe::LEN * hidden, 3), values(hidden, 5)),
            (hidden, OUTPUTS, values(hidden * OUTPUTS, 7), values(OUTPUTS, 11)),
        ]
    }

    #[test]
    fn eval_normalises_inputs_and_matches_a_plain_layer_by_layer_sum() {
        let layers = two_layers();
        let net = Net::read(&file(&layers, 0.5, 2.0)).unwrap();
        assert_eq!(net.period(), PERIOD);
        let obs = values(observe::LEN, 13).iter().map(|v| v * 40.0).collect::<Vec<_>>();
        let mut x: Vec<f64> = obs.iter().map(|&v| (f64::from(v) - 0.5) / 2.0).collect();
        for (k, (inputs, outputs, weights, bias)) in layers.iter().enumerate() {
            x = (0..*outputs)
                .map(|o| {
                    let sum = (0..*inputs).map(|i| x[i] * f64::from(weights[i * outputs + o])).sum::<f64>();
                    let y = f64::from(bias[o]) + sum;
                    if k == 0 { y.max(0.0) } else { y }
                })
                .collect();
        }
        let verdict = net.eval(&obs);
        let got: Vec<f32> =
            verdict.steer_logits.iter().copied().chain([verdict.gas, verdict.brake]).collect();
        for (a, b) in got.iter().zip(&x) {
            assert!((f64::from(*a) - b).abs() < 1e-5, "{a} vs {b}");
        }
    }

    #[test]
    fn controls_take_the_first_best_steering_bin_and_independent_pedals() {
        for bin in 0..STEER.len() {
            for (gas, brake, pedals) in
                [(-1.0, 0.0, [0.0, 0.0]), (0.01, -1.0, [1.0, 0.0]), (1.0, 2.0, [1.0, 1.0])]
            {
                let mut verdict = Verdict { steer_logits: [0.0; STEER.len()], gas, brake };
                verdict.steer_logits[bin] = 1.0;
                verdict.steer_logits[(bin + 3) % STEER.len()] = 1.0;
                let controls = verdict.controls();
                assert_eq!(controls.steer, STEER[bin.min((bin + 3) % STEER.len())]);
                assert_eq!([controls.gas, controls.brake], pedals);
            }
        }
    }

    #[test]
    fn a_widened_v8_network_reads_v9_and_decides_as_before() {
        let net = Net::read(&file(&two_layers(), 0.5, 2.0)).unwrap();
        let wide = net.widen();
        assert_eq!([wide.version(), wide.inputs() as u32], [9, observe::LEN_V9 as u32]);
        let again = Net::read(&wide.write()).unwrap();
        assert_eq!(again.write(), wide.write());
        let obs = values(observe::LEN_V9, 17).iter().map(|v| v * 40.0).collect::<Vec<_>>();
        let verdict = net.eval(&obs);
        assert_eq!(verdict, net.eval(&obs[..observe::LEN]));
        assert_eq!(verdict, again.eval(&obs));
    }

    #[test]
    fn other_formats_versions_shapes_and_damaged_files_are_refused() {
        let valid = file(&two_layers(), 0.0, 1.0);
        assert!(Net::read(&valid).is_ok());
        for offset in [0, 4, 8, 12, 20, 24] {
            let mut bytes = valid.clone();
            bytes[offset] ^= 1;
            assert!(Net::read(&bytes).is_err(), "byte {offset}");
        }
        assert!(Net::read(&file(&two_layers(), 0.0, 0.0)).is_err());
        let mut short = two_layers();
        short.pop();
        assert!(Net::read(&file(&short, 0.0, 1.0)).is_err());
        for end in [0, 4, 20, valid.len() - 1] {
            assert!(Net::read(&valid[..end]).is_err());
        }
        let mut trailing = valid;
        trailing.push(0);
        assert!(Net::read(&trailing).is_err());
    }
}
