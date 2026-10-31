use anyhow::{Context, Result, bail, ensure};

use crate::chart::{self, Chart};
use crate::model::{Conv, Reader};

const INPUT: u32 = 0;
const CHART: u32 = 1;
const NORMALIZE: u32 = 2;
const CONV: u32 = 3;
const DENSE: u32 = 4;
const FLATTEN: u32 = 5;
const CONCAT: u32 = 6;
const ADD: u32 = 7;
const LAYER_NORM: u32 = 8;
const RELU: u32 = 9;
const POOL: u32 = 10;
const ATTEND: u32 = 11;
const SELECT: u32 = 12;
const ONE_HOT: u32 = 13;
pub const RELATIVE: usize = 4;

#[derive(Clone, Debug)]
struct Dense {
    outputs: usize,
    relu: bool,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

#[derive(Clone, Debug)]
struct Attend {
    query: usize,
    pose: usize,
    keys: usize,
    values: usize,
    heads: usize,
    relative_keys: Dense,
    relative_values: Dense,
}

#[derive(Clone, Debug)]
enum Op {
    Input { offset: usize },
    Chart,
    Normalize { source: usize, mean: Vec<f32>, std: Vec<f32> },
    Conv { source: usize, conv: Conv },
    Dense { source: usize, dense: Dense },
    Flatten { source: usize },
    Concat { sources: Vec<usize> },
    Add { left: usize, right: usize },
    LayerNorm { source: usize, gain: Vec<f32>, bias: Vec<f32>, epsilon: f32 },
    Relu { source: usize },
    Pool { source: usize },
    Attend(Box<Attend>),
    Select { source: usize, column: usize },
    OneHot { source: usize, column: usize, scale: f32 },
}

#[derive(Clone, Debug)]
struct Program {
    ops: Vec<(Op, [usize; 3])>,
}

#[derive(Clone, Debug)]
pub struct Graph {
    memory: Program,
    tick: Program,
}

#[derive(Clone, Debug, Default)]
pub struct Memory {
    buffers: Vec<Vec<f32>>,
}

impl Dense {
    fn read(r: &mut Reader<'_>, inputs: usize) -> Result<Self> {
        let [i, outputs, relu] = r.words()?;
        ensure!(i == inputs && outputs > 0 && relu <= 1, "dense layer reads {i} of {inputs} values");
        let weights = r.floats(inputs.checked_mul(outputs).context("layer size overflow")?)?;
        Ok(Self { outputs, relu: relu == 1, weights, bias: r.floats(outputs)? })
    }

    fn write(&self, out: &mut Vec<u8>) {
        let inputs = self.weights.len() / self.outputs;
        words(out, &[inputs, self.outputs, usize::from(self.relu)]);
        floats(out, self.weights.iter().chain(&self.bias));
    }

    fn apply(&self, x: &[f32], out: &mut [f32]) {
        for (&value, row) in x.iter().zip(self.weights.chunks_exact(self.outputs)) {
            if value != 0.0 {
                for (sum, &weight) in out.iter_mut().zip(row) {
                    *sum += value * weight;
                }
            }
        }
        for (sum, &bias) in out.iter_mut().zip(&self.bias) {
            *sum += bias;
            if self.relu {
                *sum = sum.max(0.0);
            }
        }
    }
}

fn words(out: &mut Vec<u8>, values: &[usize]) {
    out.extend(values.iter().flat_map(|&w| (w as u32).to_le_bytes()));
}

fn floats<'a>(out: &mut Vec<u8>, values: impl IntoIterator<Item = &'a f32>) {
    out.extend(values.into_iter().flat_map(|v| v.to_le_bytes()));
}

impl Program {
    fn read(r: &mut Reader<'_>, memory: Option<&Program>, len: usize) -> Result<Self> {
        let [count] = r.words()?;
        ensure!(count <= 256 && (count > 0 || memory.is_none()), "invalid operation count {count}");
        let mut ops: Vec<(Op, [usize; 3])> = Vec::with_capacity(count);
        for _ in 0..count {
            let shape_of = |ops: &[(Op, [usize; 3])], at: usize| {
                ops.get(at)
                    .map(|op| op.1)
                    .with_context(|| format!("operation reads buffer {at} before it exists"))
            };
            let kind = r.word()?;
            let op = match kind {
                INPUT => {
                    ensure!(memory.is_some(), "the per-race program reads the chart, not the observation");
                    let [offset, height, width, channels] = r.words()?;
                    let size = height * width * channels;
                    ensure!(size > 0 && offset + size <= len, "input past the observation");
                    (Op::Input { offset }, [height, width, channels])
                }
                CHART => {
                    ensure!(memory.is_none(), "only the per-race program reads the chart");
                    (Op::Chart, [chart::SIDE, chart::SIDE, chart::CHANNELS])
                }
                NORMALIZE => {
                    let [source] = r.words()?;
                    let shape = shape_of(&ops, source)?;
                    let n = shape[2];
                    let (mean, std) = (r.floats(n)?, r.floats(n)?);
                    ensure!(std.iter().all(|s| *s > 0.0 && s.is_finite()), "input scale is not positive");
                    (Op::Normalize { source, mean, std }, shape)
                }
                CONV => {
                    let [source] = r.words()?;
                    let shape = shape_of(&ops, source)?;
                    let conv = Conv::read(r, shape)?;
                    let out = conv.shape(shape);
                    (Op::Conv { source, conv }, out)
                }
                DENSE => {
                    let [source] = r.words()?;
                    let [h, w, c] = shape_of(&ops, source)?;
                    let dense = Dense::read(r, c)?;
                    let out = [h, w, dense.outputs];
                    (Op::Dense { source, dense }, out)
                }
                FLATTEN => {
                    let [source] = r.words()?;
                    let [h, w, c] = shape_of(&ops, source)?;
                    (Op::Flatten { source }, [1, 1, h * w * c])
                }
                CONCAT => {
                    let [n] = r.words()?;
                    ensure!(n > 0 && n <= 64, "invalid concatenation of {n}");
                    let sources = (0..n).map(|_| r.word().map(|w| w as usize)).collect::<Result<Vec<_>>>()?;
                    let mut total = 0;
                    for &source in &sources {
                        let [h, w, c] = shape_of(&ops, source)?;
                        ensure!(h == 1 && w == 1, "only vectors concatenate");
                        total += c;
                    }
                    (Op::Concat { sources }, [1, 1, total])
                }
                ADD => {
                    let [left, right] = r.words()?;
                    let shape = shape_of(&ops, left)?;
                    ensure!(shape == shape_of(&ops, right)?, "added buffers differ in shape");
                    (Op::Add { left, right }, shape)
                }
                LAYER_NORM => {
                    let [source] = r.words()?;
                    let shape = shape_of(&ops, source)?;
                    let (gain, bias) = (r.floats(shape[2])?, r.floats(shape[2])?);
                    let epsilon = f32::from_bits(r.word()?);
                    ensure!(epsilon > 0.0 && epsilon.is_finite(), "invalid layer norm epsilon");
                    (Op::LayerNorm { source, gain, bias, epsilon }, shape)
                }
                RELU => {
                    let [source] = r.words()?;
                    (Op::Relu { source }, shape_of(&ops, source)?)
                }
                POOL => {
                    let [source] = r.words()?;
                    let [_, _, c] = shape_of(&ops, source)?;
                    (Op::Pool { source }, [1, 1, 2 * c])
                }
                ATTEND => {
                    let memory = memory.context("the per-race program can't attend")?;
                    let [query, pose, keys, values, heads] = r.words()?;
                    let [_, _, width] = shape_of(&ops, query)?;
                    ensure!(
                        shape_of(&ops, pose)? == [1, 1, chart::POSE_LEN],
                        "attention reads the car's pose"
                    );
                    let [kh, kw, kc] = shape_of(&memory.ops, keys)?;
                    ensure!(
                        kh == kw && shape_of(&memory.ops, values)? == [kh, kw, kc],
                        "keys and values differ"
                    );
                    ensure!(heads > 0 && width == kc && kc % heads == 0, "attention widths don't match");
                    let relative_keys = Dense::read(r, RELATIVE)?;
                    let relative_values = Dense::read(r, RELATIVE)?;
                    ensure!(
                        relative_keys.outputs == kc && relative_values.outputs == kc,
                        "relative widths differ"
                    );
                    let attend = Attend { query, pose, keys, values, heads, relative_keys, relative_values };
                    (Op::Attend(Box::new(attend)), [1, 1, kc])
                }
                SELECT => {
                    let [source, column] = r.words()?;
                    let shape = shape_of(&ops, source)?;
                    ensure!(column < shape[2], "selection by column {column} of {}", shape[2]);
                    (Op::Select { source, column }, shape)
                }
                ONE_HOT => {
                    let [source, column, classes] = r.words()?;
                    let scale = f32::from_bits(r.word()?);
                    let [h, w, c] = shape_of(&ops, source)?;
                    ensure!(column < c && classes > 0 && classes <= 256, "one-hot of column {column} of {c}");
                    ensure!(scale > 0.0 && scale.is_finite(), "invalid one-hot scale");
                    (Op::OneHot { source, column, scale }, [h, w, c - 1 + classes])
                }
                _ => bail!("unknown operation {kind}"),
            };
            ops.push(op);
        }
        Ok(Self { ops })
    }

    fn write(&self, out: &mut Vec<u8>) {
        words(out, &[self.ops.len()]);
        for (op, shape) in &self.ops {
            match op {
                Op::Input { offset } => words(out, &[INPUT as usize, *offset, shape[0], shape[1], shape[2]]),
                Op::Chart => words(out, &[CHART as usize]),
                Op::Normalize { source, mean, std } => {
                    words(out, &[NORMALIZE as usize, *source]);
                    floats(out, mean.iter().chain(std));
                }
                Op::Conv { source, conv } => {
                    words(out, &[CONV as usize, *source]);
                    conv.write(out);
                }
                Op::Dense { source, dense } => {
                    words(out, &[DENSE as usize, *source]);
                    dense.write(out);
                }
                Op::Flatten { source } => words(out, &[FLATTEN as usize, *source]),
                Op::Concat { sources } => {
                    words(out, &[CONCAT as usize, sources.len()]);
                    words(out, sources);
                }
                Op::Add { left, right } => words(out, &[ADD as usize, *left, *right]),
                Op::LayerNorm { source, gain, bias, epsilon } => {
                    words(out, &[LAYER_NORM as usize, *source]);
                    floats(out, gain.iter().chain(bias).chain([epsilon]));
                }
                Op::Relu { source } => words(out, &[RELU as usize, *source]),
                Op::Pool { source } => words(out, &[POOL as usize, *source]),
                Op::Select { source, column } => words(out, &[SELECT as usize, *source, *column]),
                Op::OneHot { source, column, scale } => {
                    let classes = shape[2] + 1 - self.ops[*source].1[2];
                    words(out, &[ONE_HOT as usize, *source, *column, classes]);
                    floats(out, [scale]);
                }
                Op::Attend(a) => {
                    words(out, &[ATTEND as usize, a.query, a.pose, a.keys, a.values, a.heads]);
                    a.relative_keys.write(out);
                    a.relative_values.write(out);
                }
            }
        }
    }

    fn run(&self, observation: &[f32], chart: Option<&Chart>, memory: Option<&Memory>) -> Vec<Vec<f32>> {
        let mut buffers: Vec<Vec<f32>> = Vec::with_capacity(self.ops.len());
        for (op, shape) in &self.ops {
            let size = shape.iter().product::<usize>();
            let value = match op {
                Op::Input { offset } => observation[*offset..*offset + size].to_vec(),
                Op::Chart => chart.map_or_else(|| vec![0.0; size], |chart| chart.image.clone()),
                Op::Normalize { source, mean, std } => buffers[*source]
                    .chunks_exact(mean.len())
                    .flat_map(|row| row.iter().zip(mean).zip(std).map(|((v, m), s)| (v - m) / s))
                    .collect(),
                Op::Conv { source, conv } => {
                    let [h, w, c] = self.ops[*source].1;
                    conv.apply(&buffers[*source], [h, w, c])
                }
                Op::Dense { source, dense } => {
                    let inputs = self.ops[*source].1[2];
                    let mut out = vec![0.0; buffers[*source].len() / inputs * dense.outputs];
                    for (x, y) in
                        buffers[*source].chunks_exact(inputs).zip(out.chunks_exact_mut(dense.outputs))
                    {
                        dense.apply(x, y);
                    }
                    out
                }
                Op::Flatten { source } => buffers[*source].clone(),
                Op::Concat { sources } => sources.iter().flat_map(|&s| buffers[s].iter().copied()).collect(),
                Op::Add { left, right } => {
                    buffers[*left].iter().zip(&buffers[*right]).map(|(a, b)| a + b).collect()
                }
                Op::LayerNorm { source, gain, bias, epsilon } => {
                    let n = gain.len();
                    let mut out = Vec::with_capacity(size);
                    for row in buffers[*source].chunks_exact(n) {
                        let mean = row.iter().sum::<f32>() / n as f32;
                        let variance = row.iter().map(|v| (v - mean) * (v - mean)).sum::<f32>() / n as f32;
                        let scale = 1.0 / (variance + epsilon).sqrt();
                        out.extend(
                            row.iter().zip(gain).zip(bias).map(|((v, g), b)| (v - mean) * scale * g + b),
                        );
                    }
                    out
                }
                Op::Relu { source } => buffers[*source].iter().map(|v| v.max(0.0)).collect(),
                Op::Pool { source } => {
                    let c = shape[2] / 2;
                    let rows = buffers[*source].chunks_exact(c);
                    let count = rows.len().max(1) as f32;
                    let mut sum = vec![0.0f32; c];
                    let mut most = vec![f32::NEG_INFINITY; c];
                    for row in rows {
                        for ((s, m), &v) in sum.iter_mut().zip(&mut most).zip(row) {
                            *s += v;
                            *m = m.max(v);
                        }
                    }
                    let most = most.into_iter().map(|m| if m == f32::NEG_INFINITY { 0.0 } else { m });
                    sum.iter().map(|s| s / count).chain(most).collect()
                }
                Op::Select { source, column } => {
                    let c = shape[2];
                    buffers[*source]
                        .chunks_exact(c)
                        .filter(|row| row[*column] != 0.0)
                        .flatten()
                        .copied()
                        .collect()
                }
                Op::OneHot { source, column, scale } => {
                    let c = self.ops[*source].1[2];
                    let classes = shape[2] + 1 - c;
                    let mut out = Vec::with_capacity(buffers[*source].len() / c * shape[2]);
                    for row in buffers[*source].chunks_exact(c) {
                        let index = (row[*column] * scale).round();
                        out.extend(row.iter().enumerate().filter(|&(k, _)| k != *column).map(|(_, v)| *v));
                        let at = out.len();
                        out.resize(at + classes, 0.0);
                        if index >= 0.0 && (index as usize) < classes {
                            out[at + index as usize] = 1.0;
                        }
                    }
                    out
                }
                Op::Attend(a) => match memory {
                    Some(memory) => a.apply(&buffers[a.query], &buffers[a.pose], memory),
                    None => vec![0.0; size],
                },
            };
            buffers.push(value);
        }
        buffers
    }
}

impl Attend {
    fn apply(&self, query: &[f32], pose: &[f32], memory: &Memory) -> Vec<f32> {
        let keys = &memory.buffers[self.keys];
        let values = &memory.buffers[self.values];
        let width = self.relative_keys.outputs;
        let tokens = keys.len() / width;
        let side = (tokens as f32).sqrt() as usize;
        let dim = width / self.heads;
        let scale = 1.0 / (dim as f32).sqrt();
        let relative = chart::relative(pose, side);
        let mut key = vec![0.0; width];
        let mut value = vec![0.0; tokens * width];
        let mut scores = vec![0.0; tokens * self.heads];
        for t in 0..tokens {
            key.fill(0.0);
            self.relative_keys.apply(&relative[t * RELATIVE..][..RELATIVE], &mut key);
            let row = &mut value[t * width..][..width];
            self.relative_values.apply(&relative[t * RELATIVE..][..RELATIVE], row);
            for ((v, &m), (k, &mk)) in
                row.iter_mut().zip(&values[t * width..]).zip(key.iter_mut().zip(&keys[t * width..]))
            {
                *v += m;
                *k += mk;
            }
            for h in 0..self.heads {
                let q = &query[h * dim..][..dim];
                let k = &key[h * dim..][..dim];
                scores[h * tokens + t] = q.iter().zip(k).fold(0.0, |s, (a, b)| s + a * b) * scale;
            }
        }
        let mut out = vec![0.0; width];
        for h in 0..self.heads {
            let row = &mut scores[h * tokens..][..tokens];
            let most = row.iter().fold(f32::NEG_INFINITY, |m, &v| m.max(v));
            let mut total = 0.0;
            for s in row.iter_mut() {
                *s = libm::expf(*s - most);
                total += *s;
            }
            let o = &mut out[h * dim..][..dim];
            for (t, &w) in row.iter().enumerate() {
                for (sum, &v) in o.iter_mut().zip(&value[t * width + h * dim..][..dim]) {
                    *sum += w / total * v;
                }
            }
        }
        out
    }
}

impl Graph {
    pub(crate) fn read(r: &mut Reader<'_>, len: usize) -> Result<Self> {
        let memory = Program::read(r, None, 0)?;
        let tick = Program::read(r, Some(&memory), len)?;
        Ok(Self { memory, tick })
    }

    pub(crate) fn write(&self, out: &mut Vec<u8>) {
        self.memory.write(out);
        self.tick.write(out);
    }

    pub(crate) fn outputs(&self) -> usize {
        self.tick.ops.last().map_or(0, |op| op.1.iter().product())
    }

    pub(crate) fn reads_chart(&self) -> bool {
        self.tick.ops.iter().any(|op| matches!(op.0, Op::Attend(_)))
    }

    pub(crate) fn prepare(&self, chart: &Chart) -> Memory {
        Memory { buffers: self.memory.run(&[], Some(chart), None) }
    }

    pub(crate) fn eval(&self, observation: &[f32], memory: Option<&Memory>) -> Vec<f32> {
        self.tick.run(observation, None, memory).pop().unwrap_or_default()
    }
}
