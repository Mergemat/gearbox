use serde::{Deserialize, Serialize};
use std::fmt;

pub const STATE_VERSION: u32 = 1;

#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct F32(pub u32);

impl F32 {
    pub fn new(x: f32) -> Self {
        Self(x.to_bits())
    }
    pub fn get(self) -> f32 {
        f32::from_bits(self.0)
    }
}

impl From<f32> for F32 {
    fn from(x: f32) -> Self {
        Self::new(x)
    }
}

impl fmt::Debug for F32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?} ({:#010x})", self.get(), self.0)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Controls {
    pub gas: f32,
    pub brake: f32,
    pub steer: f32,
}

impl Controls {
    pub fn bits(self) -> [u32; 3] {
        [self.gas.to_bits(), self.brake.to_bits(), self.steer.to_bits()]
    }
    pub fn from_bits(b: [u32; 3]) -> Self {
        Self { gas: f32::from_bits(b[0]), brake: f32::from_bits(b[1]), steer: f32::from_bits(b[2]) }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Body {
    pub rotation: [F32; 9],
    pub position: [F32; 3],
    pub quaternion: [F32; 4],
    pub linear_velocity: [F32; 3],
    pub angular_velocity: [F32; 3],
}

impl Body {
    pub fn from_floats(f: [f32; 22]) -> Self {
        let g = |i: usize| F32::new(f[i]);
        Self {
            rotation: std::array::from_fn(|i| g(i)),
            position: std::array::from_fn(|i| g(9 + i)),
            quaternion: std::array::from_fn(|i| g(12 + i)),
            linear_velocity: std::array::from_fn(|i| g(16 + i)),
            angular_velocity: std::array::from_fn(|i| g(19 + i)),
        }
    }

    pub fn floats(&self) -> [f32; 22] {
        let mut f = [0f32; 22];
        let all = self
            .rotation
            .iter()
            .chain(&self.position)
            .chain(&self.quaternion)
            .chain(&self.linear_velocity)
            .chain(&self.angular_velocity);
        for (o, v) in f.iter_mut().zip(all) {
            *o = v.get();
        }
        f
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Wheel {
    pub contact: u32,
    pub material: u8,
    pub spin: F32,
    pub spin_rate: F32,
    pub steer: F32,
    pub suspension: F32,
    pub position: [F32; 3],
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Vehicle {
    pub contact_flags: u32,
    pub contact_normal: [F32; 3],
    pub rpm: F32,
    pub gear: u32,
    pub wheels: [Wheel; 4],
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Race {
    pub checkpoints: Vec<u32>,
    pub laps: u32,
    pub finish: Option<u32>,
    pub waypoints: Vec<u32>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TickState {
    pub race_ms: i32,
    pub body: Body,
    pub vehicle: Vehicle,
    pub race: Option<Race>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValueType {
    F32,
    U32,
    I32,
    U8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Name {
    pub path: &'static str,
    pub index: Option<u8>,
    pub leaf: &'static str,
}

impl fmt::Display for Name {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.path)?;
        if let Some(i) = self.index {
            write!(f, "[{i}]")?;
        }
        if !self.leaf.is_empty() {
            write!(f, ".{}", self.leaf)?;
        }
        Ok(())
    }
}

pub const WORDS: usize = 65;

const XYZ: [&str; 3] = ["x", "y", "z"];
const POS: [&str; 3] = ["position.x", "position.y", "position.z"];
const M: [&str; 9] = ["m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22"];
const Q: [&str; 4] = ["q0", "q1", "q2", "q3"];

fn name(path: &'static str, index: Option<u8>, leaf: &'static str) -> Name {
    Name { path, index, leaf }
}

impl TickState {
    pub fn walk(&mut self, f: &mut dyn FnMut(Name, ValueType, &mut u32)) {
        let mut w = self.race_ms as u32;
        f(name("race_ms", None, ""), ValueType::I32, &mut w);
        self.race_ms = w as i32;
        let b = &mut self.body;
        for (i, v) in b.rotation.iter_mut().enumerate() {
            f(name("body.rotation", None, M[i]), ValueType::F32, &mut v.0);
        }
        for (i, v) in b.position.iter_mut().enumerate() {
            f(name("body.position", None, XYZ[i]), ValueType::F32, &mut v.0);
        }
        for (i, v) in b.quaternion.iter_mut().enumerate() {
            f(name("body.quaternion", None, Q[i]), ValueType::F32, &mut v.0);
        }
        for (path, a) in [
            ("body.linear_velocity", &mut b.linear_velocity),
            ("body.angular_velocity", &mut b.angular_velocity),
        ] {
            for (i, v) in a.iter_mut().enumerate() {
                f(name(path, None, XYZ[i]), ValueType::F32, &mut v.0);
            }
        }
        let v = &mut self.vehicle;
        f(name("vehicle.contact_flags", None, ""), ValueType::U32, &mut v.contact_flags);
        for (i, x) in v.contact_normal.iter_mut().enumerate() {
            f(name("vehicle.contact_normal", None, XYZ[i]), ValueType::F32, &mut x.0);
        }
        f(name("vehicle.rpm", None, ""), ValueType::F32, &mut v.rpm.0);
        f(name("vehicle.gear", None, ""), ValueType::U32, &mut v.gear);
        for (k, w) in v.wheels.iter_mut().enumerate() {
            let k = Some(k as u8);
            f(name("wheels", k, "contact"), ValueType::U32, &mut w.contact);
            let mut m = w.material as u32;
            f(name("wheels", k, "material"), ValueType::U8, &mut m);
            w.material = m as u8;
            f(name("wheels", k, "spin"), ValueType::F32, &mut w.spin.0);
            f(name("wheels", k, "spin_rate"), ValueType::F32, &mut w.spin_rate.0);
            f(name("wheels", k, "steer"), ValueType::F32, &mut w.steer.0);
            f(name("wheels", k, "suspension"), ValueType::F32, &mut w.suspension.0);
            for (i, x) in w.position.iter_mut().enumerate() {
                f(name("wheels", k, POS[i]), ValueType::F32, &mut x.0);
            }
        }
    }

    pub fn words(&self) -> [u32; WORDS] {
        let mut out = [0u32; WORDS];
        let mut i = 0;
        self.clone().walk(&mut |_, _, w| {
            out[i] = *w;
            i += 1;
        });
        out
    }

    pub fn from_words(words: &[u32; WORDS], race: Option<Race>) -> Self {
        let mut s = Self { race, ..Self::default() };
        let mut i = 0;
        s.walk(&mut |_, _, w| {
            *w = words[i];
            i += 1;
        });
        s
    }

    pub fn names() -> Vec<(Name, ValueType)> {
        let mut out = Vec::with_capacity(WORDS);
        Self::default().walk(&mut |n, k, _| out.push((n, k)));
        out
    }
}

fn show(kind: ValueType, w: u32) -> String {
    match kind {
        ValueType::F32 => format!("{:?}", F32(w)),
        ValueType::U32 => format!("{w} ({w:#x})"),
        ValueType::I32 => format!("{}", w as i32),
        ValueType::U8 => format!("{w}"),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FieldDiff {
    pub field: String,
    pub expected: String,
    pub actual: String,
}

impl fmt::Display for FieldDiff {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: expected {}, actual {}", self.field, self.expected, self.actual)
    }
}

pub fn first_diff(expected: &TickState, actual: &TickState) -> Option<FieldDiff> {
    if expected == actual {
        return None;
    }
    let (e, a) = (expected.words(), actual.words());
    for ((n, k), (x, y)) in TickState::names().into_iter().zip(e.iter().zip(&a)) {
        if x != y {
            return Some(FieldDiff { field: n.to_string(), expected: show(k, *x), actual: show(k, *y) });
        }
    }
    let diff =
        |field: &str, x: String, y: String| Some(FieldDiff { field: field.into(), expected: x, actual: y });
    match (&expected.race, &actual.race) {
        (Some(x), Some(y)) => {
            if x.checkpoints != y.checkpoints {
                return diff(
                    "race.checkpoints",
                    format!("{:?}", x.checkpoints),
                    format!("{:?}", y.checkpoints),
                );
            }
            if x.laps != y.laps {
                return diff("race.laps", x.laps.to_string(), y.laps.to_string());
            }
            if x.finish != y.finish {
                return diff("race.finish", format!("{:?}", x.finish), format!("{:?}", y.finish));
            }
            diff("race.waypoints", format!("{:?}", x.waypoints), format!("{:?}", y.waypoints))
        }
        (x, y) => diff("race", format!("{x:?}"), format!("{y:?}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walk_covers_every_word_once_with_distinct_names() {
        let names = TickState::names();
        assert_eq!(names.len(), WORDS);
        let shown: std::collections::HashSet<String> = names.iter().map(|(n, _)| n.to_string()).collect();
        assert_eq!(shown.len(), WORDS);
        let words: [u32; WORDS] = std::array::from_fn(|i| 0x1000 + i as u32);
        let words = {
            let mut w = words;
            for (i, (n, k)) in names.iter().enumerate() {
                if *k == ValueType::U8 {
                    assert!(n.to_string().ends_with(".material"));
                    w[i] = 200 + i as u32 % 50;
                }
            }
            w
        };
        assert_eq!(TickState::from_words(&words, None).words(), words);
    }

    #[test]
    fn body_floats_keep_integrator_order() {
        let f: [f32; 22] = std::array::from_fn(|i| i as f32);
        let b = Body::from_floats(f);
        assert_eq!(b.position[0].get(), 9.0);
        assert_eq!(b.quaternion[3].get(), 15.0);
        assert_eq!(b.linear_velocity[0].get(), 16.0);
        assert_eq!(b.angular_velocity[2].get(), 21.0);
        assert_eq!(b.floats(), f);
    }

    #[test]
    fn first_diff_names_the_field_and_ignores_nothing_bitwise() {
        let a = TickState { race_ms: 20, race: Some(Race::default()), ..TickState::default() };
        let mut b = a.clone();
        b.vehicle.wheels[2].spin = F32::new(-0.0);
        let d = first_diff(&a, &b).unwrap();
        assert_eq!(d.field, "wheels[2].spin");
        assert!(d.actual.contains("0x80000000"), "{d}");
        let mut c = a.clone();
        c.race.as_mut().unwrap().waypoints.push(3);
        assert_eq!(first_diff(&a, &c).unwrap().field, "race.waypoints");
        let mut e = a.clone();
        e.body.position[1] = F32(1);
        e.race = None;
        assert_eq!(first_diff(&a, &e).unwrap().field, "body.position.y");
        assert_eq!(first_diff(&a, &a.clone()), None);
    }
}
