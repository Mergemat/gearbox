use std::path::Path;

use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::Controls;

use crate::observe::{self, VECTOR_LEN};
use crate::voxels::{ACROSS, CHANNELS, FORWARD, UP};

const MAGIC: &[u8; 4] = b"GNET";
pub const FORMAT: u32 = 4;

pub const STEER_COMPONENTS: usize = 5;
pub const OUTPUTS: usize = 14;
const KERNEL: usize = 3;

#[derive(Clone, Debug)]
struct Conv {
    inputs: usize,
    outputs: usize,
    stride: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

impl Conv {
    fn new(inputs: usize, outputs: usize, stride: usize, weights: Vec<f32>, bias: Vec<f32>) -> Self {
        let taps = inputs * KERNEL * KERNEL;
        let mut packed = vec![0.0; weights.len()];
        for (o, row) in weights.chunks_exact(taps).enumerate() {
            for (tap, &weight) in row.iter().enumerate() {
                packed[tap * outputs + o] = weight;
            }
        }
        Self { inputs, outputs, stride, weights: packed, bias }
    }
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
    pub steer_logits: [f32; STEER_COMPONENTS],
    pub steer_means: [f32; 3],
    pub steer_log_stds: [f32; 3],
    pub gas: f32,
    pub brake: f32,
    pub seconds_left: f32,
}

impl Verdict {
    pub fn controls(&self) -> Controls {
        let component = (0..STEER_COMPONENTS)
            .fold(0, |best, i| if self.steer_logits[i] > self.steer_logits[best] { i } else { best });
        Controls {
            steer: match component {
                0 => -1.0,
                1 => 1.0,
                i => self.steer_means[i - 2].clamp(-4.0, 4.0).tanh(),
            },
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
            convs.push(Conv::new(inputs, outputs, stride, weights, bias));
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
            steer_means: std::array::from_fn(|i| x[5 + i]),
            steer_log_stds: std::array::from_fn(|i| x[8 + i]),
            gas: x[11],
            brake: x[12],
            seconds_left: x[13].exp_m1(),
        }
    }
}

fn convolve(conv: &Conv, input: &[f32], [h, w]: [usize; 2], [oh, ow]: [usize; 2]) -> Vec<f32> {
    let mut out = vec![0.0; conv.outputs * oh * ow];
    let mut sums = conv.bias.clone();
    for y in 0..oh {
        for x in 0..ow {
            sums.copy_from_slice(&conv.bias);
            for i in 0..conv.inputs {
                for ky in 0..KERNEL {
                    let Some(iy) = (y * conv.stride + ky).checked_sub(1).filter(|&v| v < h) else {
                        continue;
                    };
                    for kx in 0..KERNEL {
                        let Some(ix) = (x * conv.stride + kx).checked_sub(1).filter(|&v| v < w) else {
                            continue;
                        };
                        let value = input[(i * h + iy) * w + ix];
                        let start = ((i * KERNEL + ky) * KERNEL + kx) * conv.outputs;
                        let weights = &conv.weights[start..start + conv.outputs];
                        for (sum, weight) in sums.iter_mut().zip(weights) {
                            *sum += weight * value;
                        }
                    }
                }
            }
            for (o, &sum) in sums.iter().enumerate() {
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
        weights[11 * inputs] = 1.0;
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
        for (component, expected) in
            [-1.0, 1.0, (-0.5_f32).tanh(), 0.5_f32.tanh(), 4.0_f32.tanh()].into_iter().enumerate()
        {
            for (gas, brake, pedals) in
                [(-1.0, 0.0, [0.0, 0.0]), (0.01, -1.0, [1.0, 0.0]), (1.0, 2.0, [1.0, 1.0])]
            {
                let mut verdict = Verdict {
                    steer_logits: [0.0; STEER_COMPONENTS],
                    steer_means: [-0.5, 0.5, 8.0],
                    steer_log_stds: [0.0; 3],
                    gas,
                    brake,
                    seconds_left: -1.0,
                };
                verdict.steer_logits[component] = 1.0;
                let controls = verdict.controls();
                assert_eq!(controls.steer, expected);
                assert_eq!([controls.gas, controls.brake], pedals);
            }
        }
    }

    #[test]
    fn format_four_outputs_keep_their_meaning() {
        let mut file = picker();
        file.convs.clear();
        let outputs = [-2.0, -1.0, 0.0, 1.0, 3.0, -2.0, 0.25, 7.0, -1.0, 0.0, 1.0, 0.1, -0.1, 2.0];
        file.dense = vec![(VECTOR_LEN, OUTPUTS, vec![0.0; OUTPUTS * VECTOR_LEN], outputs.to_vec())];
        let verdict = Net::read(&file.bytes(FORMAT)).unwrap().eval(&vec![0.0; observe::LEN]);
        assert_eq!(verdict.steer_logits, outputs[..5]);
        assert_eq!(verdict.steer_means, outputs[5..8]);
        assert_eq!(verdict.steer_log_stds, outputs[8..11]);
        assert_eq!(verdict.gas, 0.1);
        assert_eq!(verdict.brake, -0.1);
        assert_eq!(verdict.seconds_left, 2.0_f32.exp_m1());
        assert_eq!(verdict.controls().steer, 4.0_f32.tanh());
        assert_eq!(verdict.controls().gas, 1.0);
        assert_eq!(verdict.controls().brake, 0.0);
    }

    #[test]
    fn old_formats_wrong_shapes_and_damaged_files_are_refused() {
        let file = picker();
        assert!(Net::read(&file.bytes(3)).is_err());
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

    #[test]
    fn convolutions_preserve_each_output_accumulation_order() {
        fn values(count: usize, seed: u32) -> Vec<f32> {
            let mut bits = seed;
            (0..count)
                .map(|_| {
                    bits = bits.wrapping_mul(1664525).wrapping_add(1013904223);
                    ((bits >> 8) as f32 / 16777216.0 - 0.5) * 3.0
                })
                .collect()
        }
        for outputs in [1, 3, 8, 16, 17, 19] {
            for stride in [1, 2, 3, 5] {
                for dims in [[1, 1], [2, 3], [7, 11], [16, 24]] {
                    let inputs = 5;
                    let conv = Conv {
                        inputs,
                        outputs,
                        stride,
                        weights: values(outputs * inputs * KERNEL * KERNEL, 7),
                        bias: values(outputs, 13),
                    };
                    let input = values(inputs * dims.iter().product::<usize>(), 29);
                    let next = dims.map(|n| shrink(n, stride));
                    let packed = Conv::new(inputs, outputs, stride, conv.weights.clone(), conv.bias.clone());
                    let actual = convolve(&packed, &input, dims, next);
                    let expected = scalar_convolve(&conv, &input, dims, next);
                    assert_eq!(
                        actual.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                        expected.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
                        "{outputs} outputs, stride {stride}, dimensions {dims:?}"
                    );
                }
            }
        }
    }

    fn scalar_convolve(conv: &Conv, input: &[f32], [h, w]: [usize; 2], [oh, ow]: [usize; 2]) -> Vec<f32> {
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
                                let Some(ix) = (x * conv.stride + kx).checked_sub(1).filter(|&v| v < w)
                                else {
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
}
