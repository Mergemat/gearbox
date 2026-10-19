use std::io;
use std::path::Path;

use crate::state::{Controls, Race, TickState, WORDS};

pub const MAGIC: &[u8; 8] = b"TMTRACE\0";
pub const FORMAT_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MapRef {
    pub file: String,
    pub path: String,
    pub hash: u64,
    pub len: u64,
    pub tmx: Option<u32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Case {
    pub id: String,
    pub map: MapRef,
    pub source: String,
    pub t0: i32,
    pub seed: Option<u32>,
    pub start_ms: i32,
    pub steps: Vec<Controls>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Trace {
    pub case: Case,
    pub engine: String,
    pub states: Vec<TickState>,
}

pub fn fnv64(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

struct W(Vec<u8>);

impl W {
    fn u8(&mut self, x: u8) {
        self.0.push(x);
    }
    fn u32(&mut self, x: u32) {
        self.0.extend_from_slice(&x.to_le_bytes());
    }
    fn u64(&mut self, x: u64) {
        self.0.extend_from_slice(&x.to_le_bytes());
    }
    fn opt(&mut self, x: Option<u32>) {
        self.u8(x.is_some() as u8);
        self.u32(x.unwrap_or(0));
    }
    fn str(&mut self, s: &str) {
        self.u32(s.len() as u32);
        self.0.extend_from_slice(s.as_bytes());
    }
    fn list(&mut self, v: &[u32]) {
        self.u32(v.len() as u32);
        for x in v {
            self.u32(*x);
        }
    }
}

struct R<'a> {
    b: &'a [u8],
    i: usize,
}

fn bad(what: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, format!("trace: {what}"))
}

impl R<'_> {
    fn take(&mut self, n: usize) -> io::Result<&[u8]> {
        let s = self.b.get(self.i..self.i + n).ok_or_else(|| bad("truncated"))?;
        self.i += n;
        Ok(s)
    }
    fn u8(&mut self) -> io::Result<u8> {
        Ok(self.take(1)?[0])
    }
    fn u32(&mut self) -> io::Result<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> io::Result<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn opt(&mut self) -> io::Result<Option<u32>> {
        let some = self.u8()? != 0;
        let x = self.u32()?;
        Ok(some.then_some(x))
    }
    fn str(&mut self) -> io::Result<String> {
        let n = self.u32()? as usize;
        String::from_utf8(self.take(n)?.to_vec()).map_err(|_| bad("string is not UTF-8"))
    }
    fn list(&mut self) -> io::Result<Vec<u32>> {
        let n = self.u32()? as usize;
        (0..n).map(|_| self.u32()).collect()
    }
}

fn put_case(w: &mut W, c: &Case) {
    w.str(&c.id);
    w.str(&c.map.file);
    w.str(&c.map.path);
    w.u64(c.map.hash);
    w.u64(c.map.len);
    w.opt(c.map.tmx);
    w.str(&c.source);
    w.u32(c.t0 as u32);
    w.opt(c.seed);
    w.u32(c.start_ms as u32);
    w.u32(c.steps.len() as u32);
    for s in &c.steps {
        for b in s.bits() {
            w.u32(b);
        }
    }
}

fn get_case(r: &mut R) -> io::Result<Case> {
    let id = r.str()?;
    let map = MapRef { file: r.str()?, path: r.str()?, hash: r.u64()?, len: r.u64()?, tmx: r.opt()? };
    let source = r.str()?;
    let t0 = r.u32()? as i32;
    let seed = r.opt()?;
    let start_ms = r.u32()? as i32;
    let n = r.u32()? as usize;
    let steps =
        (0..n).map(|_| Ok(Controls::from_bits([r.u32()?, r.u32()?, r.u32()?]))).collect::<io::Result<_>>()?;
    Ok(Case { id, map, source, t0, seed, start_ms, steps })
}

impl Trace {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = W(Vec::new());
        w.0.extend_from_slice(MAGIC);
        w.u32(FORMAT_VERSION);
        w.u32(crate::STATE_VERSION);
        put_case(&mut w, &self.case);
        w.str(&self.engine);
        w.u32(self.states.len() as u32);
        let mut prev = [0u32; WORDS];
        let mut prev_race: Option<&Option<Race>> = None;
        for s in &self.states {
            let words = s.words();
            for (x, p) in words.iter().zip(&prev) {
                w.u32(x ^ p);
            }
            prev = words;
            match (&s.race, prev_race) {
                (r, Some(p)) if p == r => w.u8(0),
                (None, _) => w.u8(1),
                (Some(r), _) => {
                    w.u8(2);
                    w.list(&r.checkpoints);
                    w.u32(r.laps);
                    w.opt(r.finish);
                    w.list(&r.waypoints);
                }
            }
            prev_race = Some(&s.race);
        }
        w.0
    }

    pub fn decode(bytes: &[u8]) -> io::Result<Self> {
        let mut r = R { b: bytes, i: 0 };
        if r.take(8)? != MAGIC {
            return Err(bad("not a trace file"));
        }
        let (format, state) = (r.u32()?, r.u32()?);
        if format != FORMAT_VERSION || state != crate::STATE_VERSION {
            return Err(bad(&format!(
                "format {format} state {state}; this build reads format {FORMAT_VERSION} state {}",
                crate::STATE_VERSION
            )));
        }
        let case = get_case(&mut r)?;
        let engine = r.str()?;
        let n = r.u32()? as usize;
        let mut states: Vec<TickState> = Vec::with_capacity(n.min(1 << 20));
        let mut prev = [0u32; WORDS];
        for _ in 0..n {
            for p in prev.iter_mut() {
                *p ^= r.u32()?;
            }
            let race = match r.u8()? {
                0 => states.last().ok_or_else(|| bad("first state repeats a race"))?.race.clone(),
                1 => None,
                2 => Some(Race {
                    checkpoints: r.list()?,
                    laps: r.u32()?,
                    finish: r.opt()?,
                    waypoints: r.list()?,
                }),
                _ => return Err(bad("race tag")),
            };
            states.push(TickState::from_words(&prev, race));
        }
        if r.i != bytes.len() {
            return Err(bad("trailing bytes"));
        }
        Ok(Self { case, engine, states })
    }

    pub fn write(&self, path: &Path) -> io::Result<u64> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let z = zstd::encode_all(&self.encode()[..], 12)?;
        let tmp = path.with_extension("tmp");
        std::fs::write(&tmp, &z)?;
        std::fs::rename(&tmp, path)?;
        Ok(z.len() as u64)
    }

    pub fn read(path: &Path) -> io::Result<Self> {
        let z = std::fs::read(path)?;
        Self::decode(&zstd::decode_all(&z[..])?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::F32;

    #[test]
    fn trace_round_trips_with_nan_bits_and_race_changes() {
        let mut states = Vec::new();
        for k in 0..5 {
            let mut s = TickState { race_ms: -20 + 10 * k, ..TickState::default() };
            s.body.position[1] = F32(0x7fc0_0001);
            s.body.linear_velocity[2] = F32::new(k as f32 * 0.1);
            s.vehicle.wheels[1].material = 255;
            s.race = match k {
                0 => None,
                1 | 2 => Some(Race::default()),
                _ => Some(Race {
                    checkpoints: vec![30],
                    laps: 0,
                    finish: (k == 4).then_some(30),
                    waypoints: vec![7],
                }),
            };
            states.push(s);
        }
        let t = Trace {
            case: Case {
                id: "m/rand-1".into(),
                map: MapRef {
                    file: "m.Map.Gbx".into(),
                    path: "/x/m.Map.Gbx".into(),
                    hash: 9,
                    len: 3,
                    tmx: Some(42),
                },
                source: "random:1".into(),
                t0: -1500,
                seed: None,
                start_ms: -1500,
                steps: vec![Controls { gas: 1.0, brake: 0.0, steer: -0.5 }; 4],
            },
            engine: "test".into(),
            states,
        };
        let bytes = t.encode();
        assert_eq!(Trace::decode(&bytes).unwrap(), t);
        assert!(Trace::decode(&bytes[..bytes.len() - 1]).is_err());
        let mut other = bytes.clone();
        other[8] = 99;
        assert!(Trace::decode(&other).unwrap_err().to_string().contains("format 99"));
    }
}
