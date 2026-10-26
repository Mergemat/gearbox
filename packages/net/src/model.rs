use std::path::Path;

use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::Controls;

use crate::observe::{self, VECTOR_LEN};
use crate::voxels::{ACROSS, CHANNELS, FORWARD, UP};

const MAGIC: &[u8; 4] = b"GNET";
pub const FORMAT: u32 = 3;

pub const STEER_BINS: usize = 11;
pub const OUTPUTS: usize = STEER_BINS + 3;
const KERNEL: usize = 3;

#[derive(Clone, Debug)]
struct Conv {
    inputs: usize,
    outputs: usize,
    stride: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

#[derive(Clone, Debug)]
struct Dense {
    inputs: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

#[derive(Clone, Debug)]
pub struct Net {
    convs: Vec<Conv>,
    dense: Vec<Dense>,
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

fn shrink(n: usize, stride: usize) -> usize {
    (n + 2 - KERNEL) / stride + 1
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
        if version != observe::VERSION {
            bail!(
                "network was trained on observation version {version}, this build has {}",
                observe::VERSION
            );
        }
        let outputs = r.word()? as usize;
        ensure!(outputs == OUTPUTS, "network has {outputs} outputs, this build has {OUTPUTS}");
        let shape: [usize; 5] = r.words()?;
        ensure!(
            shape == [VECTOR_LEN, CHANNELS, UP, ACROSS, FORWARD],
            "network reads inputs shaped {shape:?}, observations are {:?}",
            [VECTOR_LEN, CHANNELS, UP, ACROSS, FORWARD]
        );
        let [conv_count] = r.words()?;
        ensure!(conv_count <= 8, "invalid convolution count {conv_count}");
        let mut convs = Vec::with_capacity(conv_count);
        for _ in 0..conv_count {
            let [inputs, outputs, kernel, stride] = r.words()?;
            ensure!(kernel == KERNEL && stride > 0 && inputs > 0 && outputs > 0, "invalid convolution layer");
            let weights = r.floats(outputs * inputs * KERNEL * KERNEL)?;
            let bias = r.floats(outputs)?;
            convs.push(Conv { inputs, outputs, stride, weights, bias });
        }
        let [dense_count] = r.words()?;
        ensure!(dense_count > 0 && dense_count <= 16, "invalid layer count {dense_count}");
        let mut dense = Vec::with_capacity(dense_count);
        for _ in 0..dense_count {
            let [inputs, outputs] = r.words()?;
            ensure!(inputs > 0 && outputs > 0, "empty layer");
            let weights = r.floats(inputs.checked_mul(outputs).context("layer size overflow")?)?;
            let bias = r.floats(outputs)?;
            dense.push(Dense { inputs, weights, bias });
        }
        ensure!(r.at == bytes.len(), "{} bytes after the last layer", bytes.len() - r.at);
        let net = Self { convs, dense };
        net.check()?;
        Ok(net)
    }

    fn check(&self) -> Result<()> {
        let mut channels = CHANNELS * UP;
        let mut dims = [ACROSS, FORWARD];
        for conv in &self.convs {
            ensure!(conv.inputs == channels, "convolutions don't chain");
            channels = conv.outputs;
            dims = dims.map(|n| shrink(n, conv.stride));
        }
        let first = self.dense.first().context("network has no layers")?;
        let grid = if self.convs.is_empty() { 0 } else { channels * dims.iter().product::<usize>() };
        ensure!(
            first.inputs == VECTOR_LEN + grid,
            "network reads {} inputs, observations give {}",
            first.inputs,
            VECTOR_LEN + grid
        );
        for pair in self.dense.windows(2) {
            ensure!(pair[0].bias.len() == pair[1].inputs, "layer sizes don't chain");
        }
        let outputs = self.dense.last().unwrap().bias.len();
        ensure!(outputs == OUTPUTS, "network gives {outputs} outputs, expected {}", OUTPUTS);
        Ok(())
    }

    pub fn eval(&self, observation: &[f32]) -> Verdict {
        assert_eq!(observation.len(), observe::LEN, "observation length");
        let mut grid = observation[VECTOR_LEN..].to_vec();
        let mut dims = [ACROSS, FORWARD];
        for conv in &self.convs {
            let next = dims.map(|n| shrink(n, conv.stride));
            grid = convolve(conv, &grid, dims, next);
            dims = next;
        }
        let mut x = observation[..VECTOR_LEN].to_vec();
        if !self.convs.is_empty() {
            x.extend(grid);
        }
        for (i, layer) in self.dense.iter().enumerate() {
            let hidden = i + 1 < self.dense.len();
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

fn convolve(conv: &Conv, input: &[f32], [h, w]: [usize; 2], [oh, ow]: [usize; 2]) -> Vec<f32> {
    let k = KERNEL;
    let mut out = vec![0.0; conv.outputs * oh * ow];
    for o in 0..conv.outputs {
        for y in 0..oh {
            for x in 0..ow {
                let mut sum = conv.bias[o];
                for i in 0..conv.inputs {
                    for ky in 0..k {
                        let Some(iy) = (y * conv.stride + ky).checked_sub(1).filter(|&v| v < h) else {
                            continue;
                        };
                        for kx in 0..k {
                            let Some(ix) = (x * conv.stride + kx).checked_sub(1).filter(|&v| v < w) else {
                                continue;
                            };
                            sum += conv.weights[((o * conv.inputs + i) * k + ky) * k + kx]
                                * input[(i * h + iy) * w + ix];
                        }
                    }
                }
                out[(o * oh + y) * ow + x] = sum.max(0.0);
            }
        }
    }
    out
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
        ensure!(n <= self.bytes.len() / 4, "layer exceeds file size");
        (0..n).map(|_| self.word().map(f32::from_bits)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::voxels::CELLS;

    struct File {
        convs: Vec<(usize, usize, usize, Vec<f32>, Vec<f32>)>,
        dense: Vec<(usize, usize, Vec<f32>, Vec<f32>)>,
    }

    impl File {
        fn bytes(&self, format: u32) -> Vec<u8> {
            let mut out = MAGIC.to_vec();
            let header = [format, observe::VERSION, OUTPUTS as u32];
            let shape = [VECTOR_LEN, CHANNELS, UP, ACROSS, FORWARD].map(|n| n as u32);
            for w in header.into_iter().chain(shape).chain([self.convs.len() as u32]) {
                out.extend(w.to_le_bytes());
            }
            for (inputs, outputs, stride, weights, bias) in &self.convs {
                for w in [*inputs, *outputs, KERNEL, *stride] {
                    out.extend((w as u32).to_le_bytes());
                }
                for v in weights.iter().chain(bias) {
                    out.extend(v.to_le_bytes());
                }
            }
            out.extend((self.dense.len() as u32).to_le_bytes());
            for (inputs, outputs, weights, bias) in &self.dense {
                out.extend((*inputs as u32).to_le_bytes());
                out.extend((*outputs as u32).to_le_bytes());
                for v in weights.iter().chain(bias) {
                    out.extend(v.to_le_bytes());
                }
            }
            out
        }
    }

    const PICKED: [usize; 2] = [5, 7];

    fn picker() -> File {
        let taps = KERNEL * KERNEL;
        let mut conv = vec![0.0; CHANNELS * UP * taps];
        conv[taps / 2] = 1.0;
        let dims = [ACROSS, FORWARD].map(|n| shrink(n, 2));
        let grid = dims.iter().product::<usize>();
        let inputs = VECTOR_LEN + grid;
        let mut weights = vec![0.0; OUTPUTS * inputs];
        let [y, x] = PICKED;
        weights[4 * inputs + VECTOR_LEN + y * dims[1] + x] = 3.0;
        weights[STEER_BINS * inputs] = 1.0;
        File {
            convs: vec![(CHANNELS * UP, 1, 2, conv, vec![0.0])],
            dense: vec![(inputs, OUTPUTS, weights, vec![0.0; OUTPUTS])],
        }
    }

    #[test]
    fn the_grid_part_reads_the_voxel_the_layout_says() {
        let net = Net::read(&picker().bytes(FORMAT)).unwrap();
        let mut obs = vec![0.0; observe::LEN];
        obs[0] = 2.0;
        let [y, x] = PICKED.map(|n| n * 2);
        obs[VECTOR_LEN + y * FORWARD + x] = 1.0;
        let verdict = net.eval(&obs);
        assert_eq!(verdict.steer_logits[4], 3.0);
        assert_eq!(verdict.gas, 2.0);
        obs[VECTOR_LEN + CELLS + y * FORWARD + x] = 1.0;
        assert_eq!(net.eval(&obs).steer_logits[4], 3.0);
        obs[VECTOR_LEN + y * FORWARD + x] = -1.0;
        assert_eq!(net.eval(&obs).steer_logits[4], 0.0);
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
    fn old_formats_wrong_shapes_and_damaged_files_are_refused() {
        let file = picker();
        assert!(Net::read(&file.bytes(2)).is_err());
        let valid = file.bytes(FORMAT);
        for offset in [8, 16, 20, 24] {
            let mut bytes = valid.clone();
            bytes[offset] ^= 1;
            assert!(Net::read(&bytes).is_err());
        }
        for end in [0, 4, 20, valid.len() - 1] {
            assert!(Net::read(&valid[..end]).is_err());
        }
        let mut trailing = valid;
        trailing.push(0);
        assert!(Net::read(&trailing).is_err());
    }
}
