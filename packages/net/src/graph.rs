use anyhow::{Context, Result, bail, ensure};

use std::sync::Arc;

use gearbox_engine::Mat3;
use gearbox_engine::Vec3;

use crate::chart::{self, Chart};
use crate::model::{Conv, Reader};
use crate::tokens::{self, DESC, POSE, Tokens};

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
const PIECES: u32 = 14;
const EMBED: u32 = 15;
const CROSS: u32 = 16;
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
struct Cross {
    query: usize,
    keys: usize,
    values: usize,
    heads: usize,
    pose: usize,
    fine: usize,
    coarse: usize,
    relative_keys: Dense,
    relative_values: Dense,
}

struct CrossGeometry {
    turns: Vec<[f32; 2]>,
    poses: Vec<[f32; POSE]>,
    shapes: Vec<usize>,
}

#[derive(Clone, Debug)]
struct Embed {
    column: usize,
    table: Vec<f32>,
    dense: Dense,
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
    Pieces { vocab: Vec<u64> },
    Embed { source: usize, embed: Box<Embed> },
    Cross(Box<Cross>),
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
    tokens: Option<Arc<Tokens>>,
}

struct Map<'a> {
    chart: Option<&'a Chart>,
    tokens: Option<&'a Tokens>,
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
        self.linear(x, out);
        if self.relu {
            for sum in out.iter_mut() {
                *sum = sum.max(0.0);
            }
        }
    }

    fn linear(&self, x: &[f32], out: &mut [f32]) {
        for (&value, row) in x.iter().zip(self.weights.chunks_exact(self.outputs)) {
            if value != 0.0 {
                for (sum, &weight) in out.iter_mut().zip(row) {
                    *sum += value * weight;
                }
            }
        }
        for (sum, &bias) in out.iter_mut().zip(&self.bias) {
            *sum += bias;
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
                PIECES => {
                    ensure!(memory.is_none(), "only the per-race program reads the map's pieces");
                    let [count] = r.words()?;
                    ensure!(count < 1 << 20, "invalid vocabulary of {count}");
                    let vocab = (0..count)
                        .map(|_| Ok(u64::from(r.word()?) | u64::from(r.word()?) << 32))
                        .collect::<Result<Vec<_>>>()?;
                    (Op::Pieces { vocab }, [1, 1, 1 + DESC])
                }
                EMBED => {
                    let [source, column, classes] = r.words()?;
                    let [h, w, c] = shape_of(&ops, source)?;
                    ensure!(
                        column < c && classes > 0 && classes < 1 << 20,
                        "embedding of column {column} of {c}"
                    );
                    let [outputs] = r.words()?;
                    let table = r.floats(classes.checked_mul(outputs).context("embedding size overflow")?)?;
                    let dense = Dense::read(r, c - 1)?;
                    ensure!(dense.outputs == outputs, "embedding widths differ");
                    (Op::Embed { source, embed: Box::new(Embed { column, table, dense }) }, [h, w, outputs])
                }
                CROSS => {
                    let memory = memory.context("the per-race program can't attend")?;
                    let [query, keys, values, heads, pose, fine, coarse] = r.words()?;
                    let [_, _, width] = shape_of(&ops, query)?;
                    ensure!(pose + 12 <= len, "the car's pose is past the observation");
                    ensure!(
                        memory.ops.iter().any(|op| matches!(op.0, Op::Pieces { .. })),
                        "attention over pieces needs the map's pieces"
                    );
                    let [_, _, kc] = shape_of(&memory.ops, keys)?;
                    ensure!(shape_of(&memory.ops, values)?[2] == kc, "keys and values differ");
                    ensure!(
                        heads > 0 && width == kc && kc % heads == 0 && (kc / heads) % 2 == 0,
                        "attention widths don't match"
                    );
                    ensure!(fine <= 1 << 20 && coarse <= 1 << 16, "invalid token budgets");
                    let relative_keys = Dense::read(r, POSE)?;
                    let relative_values = Dense::read(r, POSE)?;
                    ensure!(
                        relative_keys.outputs == kc && relative_values.outputs == kc,
                        "relative widths differ"
                    );
                    let cross = Cross {
                        query,
                        keys,
                        values,
                        heads,
                        pose,
                        fine,
                        coarse,
                        relative_keys,
                        relative_values,
                    };
                    (Op::Cross(Box::new(cross)), [1, 1, kc])
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
                Op::Pieces { vocab } => {
                    words(out, &[PIECES as usize, vocab.len()]);
                    for &key in vocab {
                        out.extend((key as u32).to_le_bytes());
                        out.extend(((key >> 32) as u32).to_le_bytes());
                    }
                }
                Op::Embed { source, embed } => {
                    let classes = embed.table.len() / embed.dense.outputs;
                    words(out, &[EMBED as usize, *source, embed.column, classes, embed.dense.outputs]);
                    floats(out, &embed.table);
                    embed.dense.write(out);
                }
                Op::Cross(c) => {
                    words(
                        out,
                        &[CROSS as usize, c.query, c.keys, c.values, c.heads, c.pose, c.fine, c.coarse],
                    );
                    c.relative_keys.write(out);
                    c.relative_values.write(out);
                }
            }
        }
    }

    fn run(&self, observation: &[f32], map: &Map<'_>, memory: Option<&Memory>) -> Vec<Vec<f32>> {
        let mut buffers: Vec<Vec<f32>> = Vec::with_capacity(self.ops.len());
        let mut cross_geometry = None;
        for (op, shape) in &self.ops {
            let size = shape.iter().product::<usize>();
            let value = match op {
                Op::Input { offset } => observation[*offset..*offset + size].to_vec(),
                Op::Chart => map.chart.map_or_else(|| vec![0.0; size], |chart| chart.image.clone()),
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
                Op::Pieces { vocab } => map.tokens.map_or_else(Vec::new, |tokens| pieces(vocab, tokens)),
                Op::Embed { source, embed } => {
                    let c = self.ops[*source].1[2];
                    let outputs = embed.dense.outputs;
                    let classes = embed.table.len() / outputs;
                    let mut out = vec![0.0; buffers[*source].len() / c * outputs];
                    let mut rest = vec![0.0; c - 1];
                    for (row, y) in buffers[*source].chunks_exact(c).zip(out.chunks_exact_mut(outputs)) {
                        let mut k = 0;
                        for (i, &v) in row.iter().enumerate() {
                            if i != embed.column {
                                rest[k] = v;
                                k += 1;
                            }
                        }
                        embed.dense.linear(&rest, y);
                        let index = row[embed.column].round();
                        if index >= 0.0 && (index as usize) < classes {
                            for (sum, &e) in y.iter_mut().zip(&embed.table[index as usize * outputs..]) {
                                *sum += e;
                            }
                        }
                        if embed.dense.relu {
                            for sum in y.iter_mut() {
                                *sum = sum.max(0.0);
                            }
                        }
                    }
                    out
                }
                Op::Cross(c) => match memory {
                    Some(memory) if memory.tokens.is_some() => {
                        let key = (c.pose, c.fine, c.coarse, c.relative_keys.outputs / c.heads / 2);
                        if cross_geometry.as_ref().is_none_or(|(cached, _)| *cached != key) {
                            cross_geometry =
                                Some((key, c.geometry(observation, memory.tokens.as_deref().unwrap())));
                        }
                        c.apply(&buffers[c.query], &cross_geometry.as_ref().unwrap().1, memory)
                    }
                    _ => vec![0.0; size],
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

    pub(crate) fn reads_tokens(&self) -> bool {
        self.tick.ops.iter().any(|op| matches!(op.0, Op::Cross(_)))
    }

    pub(crate) fn prepare(&self, chart: &Chart) -> Memory {
        Memory {
            buffers: self.memory.run(&[], &Map { chart: Some(chart), tokens: None }, None),
            tokens: None,
        }
    }

    pub(crate) fn prepare_tokens(&self, tokens: Arc<Tokens>) -> Memory {
        let buffers = self.memory.run(&[], &Map { chart: None, tokens: Some(&tokens) }, None);
        Memory { buffers, tokens: Some(tokens) }
    }

    pub(crate) fn eval(&self, observation: &[f32], memory: Option<&Memory>) -> Vec<f32> {
        self.tick.run(observation, &Map { chart: None, tokens: None }, memory).pop().unwrap_or_default()
    }
}

fn pieces(vocab: &[u64], tokens: &Tokens) -> Vec<f32> {
    let index: std::collections::HashMap<u64, usize> =
        vocab.iter().enumerate().map(|(i, &k)| (k, i)).collect();
    let mut out = Vec::with_capacity(tokens.keys.len() * (1 + DESC));
    for (s, (key, desc)) in tokens.keys.iter().zip(&tokens.descs).enumerate() {
        let id =
            if s >= tokens.shapes { vocab.len() + 1 } else { index.get(key).copied().unwrap_or(vocab.len()) };
        out.push(id as f32);
        out.extend(desc);
    }
    out
}

impl Cross {
    fn geometry(&self, observation: &[f32], tokens: &Tokens) -> CrossGeometry {
        let pairs = self.relative_keys.outputs / self.heads / 2;
        let pose = &observation[self.pose..self.pose + 12];
        let position = [pose[0], pose[1], pose[2]];
        let rotation = Mat3 {
            rows: [
                Vec3::new(pose[3], pose[4], pose[5]),
                Vec3::new(pose[6], pose[7], pose[8]),
                Vec3::new(pose[9], pose[10], pose[11]),
            ],
        };
        let mut selected = Vec::new();
        tokens.select(position, self.fine, self.coarse, &mut selected);
        let n = selected.len();
        let mut turns = vec![[0.0f32; 2]; n * pairs];
        let mut poses = vec![[0.0f32; POSE]; n];
        let mut shapes = vec![0usize; n];
        let (mut offset, mut psi) = ([0.0; 3], [0.0; POSE]);
        for (t, &row) in selected.iter().enumerate() {
            let piece = &tokens.pieces[row as usize];
            tokens::relative(piece, position, &rotation, &mut offset, &mut psi);
            let s = piece[0] as usize;
            shapes[t] = s;
            poses[t] = psi;
            for j in 0..pairs {
                let (sin, cos) = tokens::angle(&offset, j).sin_cos();
                turns[t * pairs + j] = [cos, sin];
            }
        }
        CrossGeometry { turns, poses, shapes }
    }

    fn apply(&self, query: &[f32], geometry: &CrossGeometry, memory: &Memory) -> Vec<f32> {
        let CrossGeometry { turns, poses, shapes } = geometry;
        let keys = &memory.buffers[self.keys];
        let values = &memory.buffers[self.values];
        let width = self.relative_keys.outputs;
        let (heads, dim) = (self.heads, width / self.heads);
        let pairs = dim / 2;
        let scale = 1.0 / (dim as f32).sqrt();
        let n = shapes.len();
        if n == 0 {
            return vec![0.0; width];
        }
        let mut seen = vec![0.0; heads * POSE];
        let weights = &self.relative_keys.weights;
        for h in 0..heads {
            for f in 0..POSE {
                seen[h * POSE + f] =
                    (0..dim).map(|d| query[h * dim + d] * weights[f * width + h * dim + d]).sum::<f32>();
            }
        }
        let mut scores = vec![0.0; heads * n];
        for (t, &s) in shapes.iter().enumerate() {
            let key = &keys[s * width..][..width];
            for h in 0..heads {
                let q = &query[h * dim..][..dim];
                let k = &key[h * dim..][..dim];
                let mut sum = 0.0;
                for j in 0..pairs {
                    let [cos, sin] = turns[t * pairs + j];
                    let (q0, q1, k0, k1) = (q[2 * j], q[2 * j + 1], k[2 * j], k[2 * j + 1]);
                    sum += cos * (q0 * k0 + q1 * k1) + sin * (q1 * k0 - q0 * k1);
                }
                sum += (0..POSE).map(|f| seen[h * POSE + f] * poses[t][f]).sum::<f32>();
                scores[h * n + t] = sum * scale;
            }
        }
        let mut out = vec![0.0; width];
        let mut pooled = vec![0.0; heads * POSE];
        for h in 0..heads {
            let row = &mut scores[h * n..][..n];
            let most = row.iter().fold(f32::NEG_INFINITY, |m, &v| m.max(v));
            let mut total = 0.0;
            for s in row.iter_mut() {
                *s = libm::expf(*s - most);
                total += *s;
            }
            let o = &mut out[h * dim..][..dim];
            for t in 0..n {
                let w = row[t] / total;
                let v = &values[shapes[t] * width + h * dim..][..dim];
                for j in 0..pairs {
                    let [cos, sin] = turns[t * pairs + j];
                    o[2 * j] += w * (v[2 * j] * cos - v[2 * j + 1] * sin);
                    o[2 * j + 1] += w * (v[2 * j] * sin + v[2 * j + 1] * cos);
                }
                for f in 0..POSE {
                    pooled[h * POSE + f] += w * poses[t][f];
                }
            }
        }
        let weights = &self.relative_values.weights;
        for h in 0..heads {
            for d in h * dim..(h + 1) * dim {
                out[d] += self.relative_values.bias[d]
                    + (0..POSE).map(|f| pooled[h * POSE + f] * weights[f * width + d]).sum::<f32>();
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tokens::Grid;

    #[test]
    fn shared_geometry_matches_independent_layers_across_observations_maps_and_budgets() {
        let width = 8;
        let dense = |seed: usize| Dense {
            outputs: width,
            relu: false,
            weights: (0..POSE * width).map(|i| ((i + seed) % 17) as f32 / 19.0 - 0.4).collect(),
            bias: vec![0.1; width],
        };
        let layers: Vec<_> =
            [(0, 3, 1, 2), (0, 3, 1, 2), (12, 3, 1, 2), (12, 1, 1, 2), (12, 1, 0, 2), (12, 1, 0, 1)]
                .into_iter()
                .enumerate()
                .map(|(query, (pose, fine, coarse, heads))| Cross {
                    query,
                    keys: 0,
                    values: 1,
                    heads,
                    pose,
                    fine,
                    coarse,
                    relative_keys: dense(query),
                    relative_values: dense(query + 3),
                })
                .collect();
        let mut ops = vec![(Op::Input { offset: 24 }, [1, 1, width])];
        ops.extend(layers.iter().cloned().map(|c| (Op::Cross(Box::new(c)), [1, 1, width])));
        let program = Program { ops };
        let tokens = Tokens {
            pieces: (0..4)
                .map(|i| [i as f32, 16.0 + 32.0 * i as f32, 0.0, 16.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
                .collect(),
            fine: 3,
            drivable: 3,
            grid: Grid { origin: [0.0; 2], size: [3, 1], cell: 32.0 },
            coarse_grid: Grid { origin: [0.0; 2], size: [1, 1], cell: 128.0 },
            drivable_cells: vec![0, 1, 2, 3],
            other_cells: vec![0; 4],
            coarse_cells: vec![0, 1],
            ..Tokens::default()
        };
        let map = Map { chart: None, tokens: None };
        let mut previous = None;
        for shift in [0.0, 7.0] {
            let mut tokens = tokens.clone();
            for piece in &mut tokens.pieces {
                piece[2] += shift;
            }
            let memory = Memory {
                buffers: (0..2)
                    .map(|j| (0..4 * width).map(|i| (i + j) as f32 / 23.0 - 0.5).collect())
                    .collect(),
                tokens: Some(Arc::new(tokens)),
            };
            for x in [16.0, 48.0, 10000.0] {
                let mut observation = vec![
                    x,
                    0.0,
                    16.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    x + 32.0,
                    0.0,
                    16.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    1.0,
                    0.0,
                    -1.0,
                    0.0,
                    0.0,
                ];
                observation.extend((0..width).map(|i| i as f32 / 7.0));
                let got = program.run(&observation, &map, Some(&memory));
                let mut expected = observation[24..].to_vec();
                for (i, layer) in layers.iter().enumerate() {
                    let geometry = layer.geometry(&observation, memory.tokens.as_deref().unwrap());
                    expected = layer.apply(&expected, &geometry, &memory);
                    assert_eq!(
                        got[i + 1].iter().map(|x| x.to_bits()).collect::<Vec<_>>(),
                        expected.iter().map(|x| x.to_bits()).collect::<Vec<_>>()
                    );
                }
                if x < 10000.0 {
                    assert!(expected.iter().all(|v| v.is_finite()));
                    assert!(expected.iter().any(|v| *v != 0.0));
                    assert_ne!(previous.as_ref(), Some(&expected));
                    previous = Some(expected);
                } else {
                    assert_eq!(expected, vec![0.0; width]);
                }
            }
        }
    }
}
