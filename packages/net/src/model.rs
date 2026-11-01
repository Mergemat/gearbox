use std::path::Path;

use anyhow::{Context, Result, ensure};
use gearbox_engine::{Controls, Session};

use crate::chart::Chart;
use crate::graph::{Graph, Memory};
use crate::ground::Ground;
use crate::observe;

const MAGIC: &[u8; 4] = b"GNET";
pub const FORMAT: u32 = 5;
pub const FORMAT_RESPAWN: u32 = 6;
pub const FORMAT_IMAGES: u32 = 7;
pub const FORMAT_PLAN: u32 = 8;
pub const FORMAT_GRAPH: u32 = 9;
const HEAD_BINS: usize = 0;
const HEAD_PLAN: usize = 1;
pub const HOLD: u32 = 100;
pub const COMPONENTS: u32 = 2;
pub const HOLD_GIVEN: usize = 4;
pub const PERIOD: u32 = 5;
pub const STEER: [f32; 11] = [-1.0, -0.65, -0.4, -0.2, -0.08, 0.0, 0.08, 0.2, 0.4, 0.65, 1.0];
pub const OUTPUTS: usize = STEER.len() + 2;
pub const RESPAWNS: usize = 3;

#[derive(Clone, Debug)]
struct Dense {
    outputs: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

#[derive(Clone, Debug)]
pub(crate) struct Conv {
    outputs: usize,
    kernel: usize,
    stride: usize,
    padding: usize,
    weights: Vec<f32>,
    bias: Vec<f32>,
}

impl Conv {
    pub(crate) fn read(r: &mut Reader<'_>, [height, width, channels]: [usize; 3]) -> Result<Self> {
        let [outputs, kernel, stride, padding] = r.words()?;
        ensure!(
            outputs > 0
                && kernel > 0
                && stride > 0
                && height + 2 * padding >= kernel
                && width + 2 * padding >= kernel,
            "invalid convolution"
        );
        let taps = kernel * kernel * channels;
        let weights = r.floats(taps * outputs)?;
        let weights = (0..outputs * taps).map(|i| weights[(i % taps) * outputs + i / taps]).collect();
        let bias = r.floats(outputs)?;
        Ok(Self { outputs, kernel, stride, padding, weights, bias })
    }

    pub(crate) fn write(&self, out: &mut Vec<u8>) {
        let words = [self.outputs, self.kernel, self.stride, self.padding];
        out.extend(words.iter().flat_map(|&w| (w as u32).to_le_bytes()));
        let taps = self.weights.len() / self.outputs;
        let weights =
            (0..self.weights.len()).map(|i| self.weights[(i % self.outputs) * taps + i / self.outputs]);
        out.extend(weights.chain(self.bias.iter().copied()).flat_map(|v| v.to_le_bytes()));
    }

    pub(crate) fn shape(&self, [height, width, _]: [usize; 3]) -> [usize; 3] {
        let [rows, cols] = [height, width].map(|n| (n + 2 * self.padding - self.kernel) / self.stride + 1);
        [rows, cols, self.outputs]
    }

    pub(crate) fn apply(&self, x: &[f32], [height, width, channels]: [usize; 3]) -> Vec<f32> {
        let (k, stride, padding) = (self.kernel, self.stride, self.padding);
        let [rows, cols, _] = self.shape([height, width, channels]);
        let taps = k * k * channels;
        let mut patch = vec![0.0; taps];
        let mut y = vec![0.0; rows * cols * self.outputs];
        for (at, out) in y.chunks_exact_mut(self.outputs).enumerate() {
            let (row, col) = (at / cols, at % cols);
            patch.fill(0.0);
            for ky in 0..k {
                let Some(iy) = (row * stride + ky).checked_sub(padding).filter(|&v| v < height) else {
                    continue;
                };
                for kx in 0..k {
                    let Some(ix) = (col * stride + kx).checked_sub(padding).filter(|&v| v < width) else {
                        continue;
                    };
                    let to = (ky * k + kx) * channels;
                    patch[to..to + channels].copy_from_slice(&x[(iy * width + ix) * channels..][..channels]);
                }
            }
            for ((value, weights), &bias) in
                out.iter_mut().zip(self.weights.chunks_exact(taps)).zip(&self.bias)
            {
                *value = (dot(&patch, weights) + bias).max(0.0);
            }
        }
        y
    }
}

#[derive(Clone, Debug)]
struct Tower {
    image: [usize; 4],
    convs: Vec<Conv>,
}

#[derive(Clone, Debug, PartialEq)]
enum Head {
    Bins { period: u32 },
    Plan { hold: u32, components: u32, given: Vec<f32> },
}

#[derive(Clone, Debug)]
pub struct Net {
    format: u32,
    version: u32,
    inputs: usize,
    head: Head,
    mean: Vec<f32>,
    std: Vec<f32>,
    towers: Vec<Tower>,
    layers: Vec<Dense>,
    graph: Option<Graph>,
    respawns: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Verdict {
    pub steer_logits: [f32; STEER.len()],
    pub gas: f32,
    pub brake: f32,
    pub respawn_logits: [f32; RESPAWNS],
}

#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub controls: Vec<Controls>,
    pub respawn: Respawn,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Respawn {
    None,
    Moving,
    Standing,
}

impl Respawn {
    pub const ALL: [Respawn; RESPAWNS] = [Respawn::None, Respawn::Moving, Respawn::Standing];
}

fn argmax(values: &[f32]) -> usize {
    (0..values.len()).fold(0, |best, i| if values[i] > values[best] { i } else { best })
}

impl Verdict {
    pub fn steer_bin(&self) -> usize {
        argmax(&self.steer_logits)
    }

    pub fn controls(&self) -> Controls {
        Controls {
            steer: STEER[self.steer_bin()],
            gas: f32::from(self.gas > 0.0),
            brake: f32::from(self.brake > 0.0),
        }
    }

    pub fn respawn(&self) -> Respawn {
        Respawn::ALL[argmax(&self.respawn_logits)]
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
        ensure!(
            matches!(format, FORMAT | FORMAT_RESPAWN | FORMAT_IMAGES | FORMAT_PLAN | FORMAT_GRAPH),
            "network file format {format}, this build reads {FORMAT} to {FORMAT_GRAPH}"
        );
        let version = r.word()?;
        let expected = observe::len(version).with_context(|| {
            format!("network was trained on observation version {version}, this build has 8 to 13")
        })?;
        let [len] = r.words()?;
        ensure!(
            len == expected || (version == 12 && observe::dots(len).is_some()),
            "network reads {len} observations, version {version} has {expected}"
        );
        let plan = match format {
            FORMAT_GRAPH => match r.words()? {
                [HEAD_BINS] => false,
                [HEAD_PLAN] => true,
                [head] => anyhow::bail!("network head {head}, this build reads 0 (bins) and 1 (plan)"),
            },
            _ => format == FORMAT_PLAN,
        };
        let (mut head, plan_respawns) = if plan {
            let [respawns, hold, components] = r.words()?;
            ensure!(respawns <= 1, "invalid respawn flag {respawns}");
            ensure!(hold > 0 && hold <= 10_000, "invalid longest hold {hold}");
            ensure!(components > 0 && components <= 16, "invalid steer component count {components}");
            let given = Vec::new();
            (Head::Plan { hold: hold as u32, components: components as u32, given }, respawns == 1)
        } else {
            let [period, steers] = r.words()?;
            ensure!(period > 0, "network has no decision period");
            ensure!(
                steers == STEER.len(),
                "network has {steers} steering bins, this build has {}",
                STEER.len()
            );
            ensure!(r.floats(steers)? == STEER, "network steering bins differ from {STEER:?}");
            (Head::Bins { period: period as u32 }, false)
        };
        if format == FORMAT_GRAPH {
            let graph = Graph::read(&mut r, len)?;
            if let Head::Plan { hold, given, .. } = &mut head {
                *given = r.floats(HOLD_GIVEN * *hold as usize)?;
            }
            let outputs = graph.outputs();
            let respawns = if plan { plan_respawns } else { outputs == OUTPUTS + RESPAWNS };
            let (mean, std, towers, layers, graph) =
                (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Some(graph));
            let net = Self { format, version, inputs: len, head, mean, std, towers, layers, graph, respawns };
            ensure!(
                outputs == net.outputs_len(),
                "format 9 network gives {outputs} outputs, expected {}",
                net.outputs_len()
            );
            ensure!(r.at == bytes.len(), "{} bytes after the network", bytes.len() - r.at);
            return Ok(net);
        }
        let images = observe::images(version);
        let towered = matches!(format, FORMAT_IMAGES | FORMAT_PLAN);
        let flat = if towered { r.words::<1>()?[0] } else { len };
        let mean = r.floats(flat)?;
        let std = r.floats(flat)?;
        ensure!(std.iter().all(|s| *s > 0.0 && s.is_finite()), "network input scale is not positive");
        let mut towers = Vec::new();
        if towered {
            ensure!(
                flat == images.first().map_or(len, |image| image[0]),
                "network reads {flat} plain inputs"
            );
            let [count] = r.words()?;
            ensure!(count <= 8, "invalid image count {count}");
            for _ in 0..count {
                towers.push(Tower { image: r.words()?, convs: Vec::new() });
            }
            ensure!(
                towers.iter().map(|tower| tower.image).eq(images.iter().copied()),
                "network reads images {:?}, version {version} has {images:?}",
                towers.iter().map(|tower| tower.image).collect::<Vec<_>>()
            );
        }
        let mut inputs = flat;
        for tower in &mut towers {
            let [_, height, width, channels] = tower.image;
            let mut shape = [height, width, channels];
            let [count] = r.words()?;
            ensure!(count > 0 && count <= 8, "invalid convolution count {count}");
            for _ in 0..count {
                let conv = Conv::read(&mut r, shape)?;
                shape = conv.shape(shape);
                tower.convs.push(conv);
            }
            inputs += shape.iter().product::<usize>();
        }
        let [count] = r.words()?;
        ensure!(count > 0 && count <= 16, "invalid layer count {count}");
        let mut layers: Vec<Dense> = Vec::with_capacity(count);
        for _ in 0..count {
            let [i, outputs] = r.words()?;
            ensure!(i == inputs && outputs > 0, "layer sizes don't chain");
            let weights = r.floats(inputs.checked_mul(outputs).context("layer size overflow")?)?;
            let bias = r.floats(outputs)?;
            layers.push(Dense { outputs, weights, bias });
            inputs = outputs;
        }
        let respawns = match format {
            FORMAT_IMAGES => inputs == OUTPUTS + RESPAWNS,
            FORMAT_PLAN => plan_respawns,
            _ => format == FORMAT_RESPAWN,
        };
        if let Head::Plan { hold, given, .. } = &mut head {
            *given = r.floats(HOLD_GIVEN * *hold as usize)?;
        }
        let net =
            Self { format, version, inputs: len, head, mean, std, towers, layers, graph: None, respawns };
        let outputs = net.outputs_len();
        ensure!(inputs == outputs, "format {format} network gives {inputs} outputs, expected {outputs}");
        ensure!(r.at == bytes.len(), "{} bytes after the network", bytes.len() - r.at);
        Ok(net)
    }

    fn outputs_len(&self) -> usize {
        let head = match &self.head {
            Head::Bins { .. } => OUTPUTS,
            Head::Plan { hold, components, .. } => (hold + 3 * components + 2) as usize,
        };
        head + if self.respawns { RESPAWNS } else { 0 }
    }

    pub fn write(&self) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        let mut words = vec![self.format, self.version, self.inputs as u32];
        if self.graph.is_some() {
            words.push(if self.period().is_some() { HEAD_BINS } else { HEAD_PLAN } as u32);
        }
        match &self.head {
            Head::Bins { period } => words.extend([*period, STEER.len() as u32]),
            Head::Plan { hold, components, .. } => {
                words.extend([u32::from(self.respawns), *hold, *components])
            }
        }
        out.extend(words.iter().flat_map(|w| w.to_le_bytes()));
        if let Head::Bins { .. } = self.head {
            out.extend(STEER.iter().flat_map(|v| v.to_le_bytes()));
        }
        if let Some(graph) = &self.graph {
            graph.write(&mut out);
            if let Head::Plan { given, .. } = &self.head {
                out.extend(given.iter().flat_map(|v| v.to_le_bytes()));
            }
            return out;
        }
        let towered = matches!(self.format, FORMAT_IMAGES | FORMAT_PLAN);
        if towered {
            out.extend((self.mean.len() as u32).to_le_bytes());
        }
        out.extend(self.mean.iter().chain(&self.std).flat_map(|v| v.to_le_bytes()));
        if towered {
            out.extend((self.towers.len() as u32).to_le_bytes());
            for tower in &self.towers {
                out.extend(tower.image.iter().flat_map(|&w| (w as u32).to_le_bytes()));
            }
            for tower in &self.towers {
                out.extend((tower.convs.len() as u32).to_le_bytes());
                for conv in &tower.convs {
                    conv.write(&mut out);
                }
            }
        }
        out.extend((self.layers.len() as u32).to_le_bytes());
        let mut inputs = self.layers[0].weights.len() / self.layers[0].outputs;
        for layer in &self.layers {
            out.extend([inputs as u32, layer.outputs as u32].iter().flat_map(|w| w.to_le_bytes()));
            out.extend(layer.weights.iter().chain(&layer.bias).flat_map(|v| v.to_le_bytes()));
            inputs = layer.outputs;
        }
        if let Head::Plan { given, .. } = &self.head {
            out.extend(given.iter().flat_map(|v| v.to_le_bytes()));
        }
        out
    }

    pub fn widen(&self) -> Self {
        let mut net = self.clone();
        if self.version == 8 && self.towers.is_empty() && self.graph.is_none() {
            let added = observe::LEN_V9 - observe::LEN;
            net.version = 9;
            net.inputs = observe::LEN_V9;
            net.mean.resize(observe::LEN_V9, 0.0);
            net.std.resize(observe::LEN_V9, 1.0);
            let first = &mut net.layers[0];
            first.weights.resize(first.weights.len() + added * first.outputs, 0.0);
        }
        net
    }

    pub fn respawns(&self) -> bool {
        self.respawns
    }

    pub fn version(&self) -> u32 {
        self.version
    }

    pub fn inputs(&self) -> usize {
        self.inputs
    }

    pub fn period(&self) -> Option<u32> {
        match self.head {
            Head::Bins { period } => Some(period),
            Head::Plan { .. } => None,
        }
    }

    pub fn hold(&self) -> Option<u32> {
        match &self.head {
            Head::Bins { .. } => None,
            Head::Plan { hold, .. } => Some(*hold),
        }
    }

    /// A planning head's mixture components and the (4, hold) weights its hold logits take from the held controls.
    pub fn plan_head(&self) -> Option<(u32, &[f32])> {
        match &self.head {
            Head::Bins { .. } => None,
            Head::Plan { components, given, .. } => Some((*components, given)),
        }
    }

    pub fn reads_chart(&self) -> bool {
        self.graph.as_ref().is_some_and(Graph::reads_chart)
    }

    pub fn prepare(&self, chart: &Chart) -> Option<Memory> {
        self.graph.as_ref().filter(|graph| graph.reads_chart()).map(|graph| graph.prepare(chart))
    }

    pub fn reads_tokens(&self) -> bool {
        self.graph.as_ref().is_some_and(Graph::reads_tokens)
    }

    pub fn prepare_tokens(&self, tokens: std::sync::Arc<crate::tokens::Tokens>) -> Option<Memory> {
        self.graph.as_ref().filter(|graph| graph.reads_tokens()).map(|graph| graph.prepare_tokens(tokens))
    }

    pub fn memory(&self, ground: &Ground, session: &Session) -> Option<Memory> {
        if self.reads_tokens() {
            return self.prepare_tokens(std::sync::Arc::new(ground.tokens(session).clone()));
        }
        self.reads_chart().then(|| self.prepare(ground.chart(session))).flatten()
    }

    pub fn plan(&self, observation: &[f32]) -> Plan {
        self.plan_with(observation, None)
    }

    pub fn plan_with(&self, observation: &[f32], memory: Option<&Memory>) -> Plan {
        self.plan_from(&self.outputs_with(observation, memory))
    }

    /// The plan for outputs already computed (`outputs_with`), so a caller that keeps them evaluates once.
    pub fn plan_from(&self, x: &[f32]) -> Plan {
        let Head::Plan { hold, components, given } = &self.head else {
            let verdict = self.verdict(x);
            let period = self.period().unwrap_or(PERIOD) as usize;
            return Plan { controls: vec![verdict.controls(); period], respawn: verdict.respawn() };
        };
        let (h, c) = (*hold as usize, *components as usize);
        let steer = x[h + c + argmax(&x[h..h + c])].clamp(-1.0, 1.0);
        let pedals = h + 3 * c;
        let controls = Controls {
            steer,
            gas: f32::from(u8::from(x[pedals] > 0.0)),
            brake: f32::from(u8::from(x[pedals + 1] > 0.0)),
        };
        let features = [controls.steer, controls.steer.abs(), controls.gas, controls.brake];
        let logits: Vec<f32> = (0..h)
            .map(|i| features.iter().enumerate().fold(x[i], |sum, (j, f)| sum + f * given[j * h + i]))
            .collect();
        let respawn = match self.respawns {
            true => Respawn::ALL[argmax(&x[pedals + 2..pedals + 2 + RESPAWNS])],
            false => Respawn::None,
        };
        Plan { controls: vec![controls; argmax(&logits) + 1], respawn }
    }

    pub fn eval(&self, observation: &[f32]) -> Verdict {
        self.eval_with(observation, None)
    }

    pub fn eval_with(&self, observation: &[f32], memory: Option<&Memory>) -> Verdict {
        assert!(self.period().is_some(), "a planning network has no single verdict");
        self.verdict(&self.outputs_with(observation, memory))
    }

    fn verdict(&self, x: &[f32]) -> Verdict {
        Verdict {
            steer_logits: std::array::from_fn(|i| x[i]),
            gas: x[STEER.len()],
            brake: x[STEER.len() + 1],
            respawn_logits: match self.respawns {
                true => std::array::from_fn(|i| x[OUTPUTS + i]),
                false => [0.0, f32::NEG_INFINITY, f32::NEG_INFINITY],
            },
        }
    }

    pub fn outputs(&self, observation: &[f32]) -> Vec<f32> {
        self.outputs_with(observation, None)
    }

    pub fn outputs_with(&self, observation: &[f32], memory: Option<&Memory>) -> Vec<f32> {
        assert!(observation.len() >= self.inputs(), "observation length");
        let observation = &observation[..self.inputs];
        if let Some(graph) = &self.graph {
            return graph.eval(observation, memory);
        }
        let mut x: Vec<f32> =
            observation.iter().zip(&self.mean).zip(&self.std).map(|((v, m), s)| (v - m) / s).collect();
        for tower in &self.towers {
            x.extend(tower.eval(observation));
        }
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
        x
    }
}

impl Tower {
    fn eval(&self, observation: &[f32]) -> Vec<f32> {
        let [offset, height, width, channels] = self.image;
        let mut shape = [height, width, channels];
        let mut x = observation[offset..offset + height * width * channels].to_vec();
        for conv in &self.convs {
            x = conv.apply(&x, shape);
            shape = conv.shape(shape);
        }
        x
    }
}

pub(crate) fn dot(a: &[f32], b: &[f32]) -> f32 {
    let mut lanes = [0.0f32; 8];
    let (chunks, rest) = a.as_chunks::<8>();
    for (x, y) in chunks.iter().zip(b.as_chunks::<8>().0) {
        for i in 0..8 {
            lanes[i] += x[i] * y[i];
        }
    }
    let tail = rest.iter().zip(&b[chunks.len() * 8..]).fold(0.0, |sum, (x, y)| sum + x * y);
    let [a, b, c, d, e, f, g, h] = lanes;
    ((a + b) + (c + d)) + ((e + f) + (g + h)) + tail
}

pub(crate) struct Reader<'a> {
    pub(crate) bytes: &'a [u8],
    pub(crate) at: usize,
}

impl Reader<'_> {
    pub(crate) fn word(&mut self) -> Result<u32> {
        let b = self.bytes.get(self.at..self.at + 4).context("network file is truncated")?;
        self.at += 4;
        Ok(u32::from_le_bytes(b.try_into().unwrap()))
    }

    pub(crate) fn words<const N: usize>(&mut self) -> Result<[usize; N]> {
        let mut out = [0; N];
        for v in &mut out {
            *v = self.word()? as usize;
        }
        Ok(out)
    }

    pub(crate) fn floats(&mut self, n: usize) -> Result<Vec<f32>> {
        ensure!(n <= (self.bytes.len() - self.at) / 4, "network file is truncated");
        (0..n).map(|_| self.word().map(f32::from_bits)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn file(layers: &[(usize, usize, Vec<f32>, Vec<f32>)], mean: f32, std: f32) -> Vec<u8> {
        file_as(FORMAT, layers, mean, std)
    }

    fn file_as(format: u32, layers: &[(usize, usize, Vec<f32>, Vec<f32>)], mean: f32, std: f32) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        let words = [format, observe::VERSION, observe::LEN as u32, PERIOD, STEER.len() as u32];
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
        layers_to(OUTPUTS)
    }

    fn layers_to(outputs: usize) -> Vec<(usize, usize, Vec<f32>, Vec<f32>)> {
        let hidden = 24;
        vec![
            (observe::LEN, hidden, values(observe::LEN * hidden, 3), values(hidden, 5)),
            (hidden, outputs, values(hidden * outputs, 7), values(outputs, 11)),
        ]
    }

    #[test]
    fn eval_normalises_inputs_and_matches_a_plain_layer_by_layer_sum() {
        let layers = two_layers();
        let net = Net::read(&file(&layers, 0.5, 2.0)).unwrap();
        assert_eq!(net.period(), Some(PERIOD));
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
                let mut verdict =
                    Verdict { steer_logits: [0.0; STEER.len()], gas, brake, respawn_logits: [0.0; RESPAWNS] };
                verdict.steer_logits[bin] = 1.0;
                verdict.steer_logits[(bin + 3) % STEER.len()] = 1.0;
                let controls = verdict.controls();
                assert_eq!(controls.steer, STEER[bin.min((bin + 3) % STEER.len())]);
                assert_eq!([controls.gas, controls.brake], pedals);
            }
        }
    }

    #[test]
    fn a_respawn_head_adds_three_outputs_and_networks_without_it_never_respawn() {
        let obs = values(observe::LEN, 13).iter().map(|v| v * 40.0).collect::<Vec<_>>();
        let plain = Net::read(&file(&two_layers(), 0.5, 2.0)).unwrap();
        assert!(!plain.respawns());
        assert_eq!(plain.eval(&obs).respawn(), Respawn::None);
        let mut layers = layers_to(OUTPUTS + RESPAWNS);
        let head = Net::read(&file_as(FORMAT_RESPAWN, &layers, 0.5, 2.0)).unwrap();
        assert!(head.respawns() && head.widen().respawns());
        assert_eq!(Net::read(&head.write()).unwrap().write(), head.write());
        let driving = |v: &Verdict| (v.steer_logits, v.gas, v.brake);
        let first = driving(&head.eval(&obs));
        for (choice, respawn) in Respawn::ALL.into_iter().enumerate() {
            for (k, bias) in layers[1].3[OUTPUTS..].iter_mut().enumerate() {
                *bias = if k == choice { 1e4 } else { -1e4 };
            }
            let verdict = Net::read(&file_as(FORMAT_RESPAWN, &layers, 0.5, 2.0)).unwrap().eval(&obs);
            assert_eq!(verdict.respawn(), respawn);
            assert_eq!(driving(&verdict), first);
        }
        assert!(Net::read(&file_as(FORMAT_RESPAWN, &two_layers(), 0.5, 2.0)).is_err());
        assert!(Net::read(&file(&layers, 0.5, 2.0)).is_err());
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

    fn plan_file(respawns: u32, layers: &[(usize, usize, Vec<f32>, Vec<f32>)], given: &[f32]) -> Vec<u8> {
        let len = observe::LEN as u32;
        let words = [FORMAT_PLAN, observe::VERSION, len, respawns, HOLD, COMPONENTS, len];
        let mut out = MAGIC.to_vec();
        out.extend(words.iter().flat_map(|w| w.to_le_bytes()));
        out.extend(
            [0.5f32; observe::LEN].iter().chain(&[2.0f32; observe::LEN]).flat_map(|v| v.to_le_bytes()),
        );
        out.extend(0u32.to_le_bytes());
        out.extend((layers.len() as u32).to_le_bytes());
        for (inputs, outputs, weights, bias) in layers {
            out.extend([*inputs as u32, *outputs as u32].iter().flat_map(|w| w.to_le_bytes()));
            out.extend(weights.iter().chain(bias).flat_map(|v| v.to_le_bytes()));
        }
        out.extend(given.iter().flat_map(|v| v.to_le_bytes()));
        out
    }

    #[test]
    fn a_plan_holds_the_likeliest_component_s_steer_and_the_pedals_for_the_likeliest_number_of_ticks_given_them()
     {
        let (h, c) = (HOLD as usize, COMPONENTS as usize);
        let mut layers = layers_to(h + 3 * c + 2 + RESPAWNS);
        layers[1].2.iter_mut().for_each(|w| *w = 0.0);
        let bias = &mut layers[1].3;
        bias.iter_mut().for_each(|b| *b = -1.0);
        bias[36] = 2.0;
        bias[h..h + 2 * c].copy_from_slice(&[0.2, 0.7, -0.4, 1.8]);
        bias[h + 3 * c] = 0.5;
        bias[h + 3 * c + 2 + 2] = 5.0;
        let mut given = vec![0.0; HOLD_GIVEN * h];
        given[2 * h + 9] = 4.0;
        given[3 * h + 70] = 100.0;
        let net = Net::read(&plan_file(1, &layers, &given)).unwrap();
        assert_eq!((net.period(), net.hold()), (None, Some(HOLD)));
        assert_eq!(Net::read(&net.write()).unwrap().write(), net.write());
        let plan = net.plan(&values(observe::LEN, 13));
        assert_eq!(plan.controls, vec![Controls { steer: 1.0, gas: 1.0, brake: 0.0 }; 10]);
        assert_eq!(plan.respawn, Respawn::Standing);
        assert!(Net::read(&plan_file(0, &layers, &given)).is_err());
        assert!(Net::read(&plan_file(1, &layers, &given[1..])).is_err());
        let binned = Net::read(&file(&two_layers(), 0.5, 2.0)).unwrap();
        let obs = values(observe::LEN, 17);
        let verdict = binned.eval(&obs);
        assert_eq!(binned.plan(&obs).controls, vec![verdict.controls(); PERIOD as usize]);
    }
}
