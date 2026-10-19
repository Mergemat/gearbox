use crate::Body;
use serde::{Deserialize, Serialize};

use crate::tricks::{Events, Observation, TrickKind};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Quantity {
    Speed,
    Height,
    Distance { to: [f32; 3] },
    Angle { yaw: Option<f32>, pitch: Option<f32>, roll: Option<f32> },
    Yaw,
    Pitch,
    Roll,
    Trick(TrickKind),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Wheels {
    Front,
    Rear,
    All,
    Any,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Condition {
    Above(Quantity, f32),
    Below(Quantity, f32),
    Wheels { which: Wheels, touching: bool },
    Inside { min: [f32; 3], max: [f32; 3] },
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Measure {
    pub quantity: Quantity,
    pub higher: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Judged {
    End,
    Best,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Aim {
    pub judged: Judged,
    pub measure: Measure,
    pub conditions: Vec<Condition>,
    pub then: Option<Measure>,
}

pub struct State<'a> {
    pub body: Body,
    pub observation: Observation,
    pub events: &'a Events,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Verdict {
    pub held: bool,
    pub value: f64,
    pub tie: f64,
}

impl Verdict {
    pub fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.held
            .cmp(&other.held)
            .then(self.value.total_cmp(&other.value))
            .then(self.tie.total_cmp(&other.tie))
    }
}

pub fn yaw_pitch_roll(body: &Body) -> [f32; 3] {
    let r = body.rotation();
    let col = |j: usize| [r[j], r[3 + j], r[6 + j]];
    let (side, up, fwd) = (col(0), col(1), col(2));
    [
        fwd[0].atan2(fwd[2]).to_degrees(),
        fwd[1].clamp(-1.0, 1.0).asin().to_degrees(),
        (-side[1]).atan2(up[1]).to_degrees(),
    ]
}

fn off(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(360.0);
    d.min(360.0 - d)
}

impl Quantity {
    pub fn value(&self, s: &State) -> f64 {
        let p = s.body.position();
        let ypr = yaw_pitch_roll(&s.body);
        match *self {
            Self::Speed => s.observation.speed * 3.6,
            Self::Height => f64::from(p[1]),
            Self::Distance { to } => (0..3).map(|k| f64::from(p[k] - to[k]).powi(2)).sum::<f64>().sqrt(),
            Self::Angle { yaw, pitch, roll } => [yaw, pitch, roll]
                .iter()
                .zip(ypr)
                .filter_map(|(t, a)| t.map(|t| f64::from(off(a, t)).powi(2)))
                .sum::<f64>()
                .sqrt(),
            Self::Yaw => f64::from(ypr[0]),
            Self::Pitch => f64::from(ypr[1]),
            Self::Roll => f64::from(ypr[2]),
            Self::Trick(kind) => f64::from(s.events.ticks(kind)),
        }
    }
}

impl Condition {
    pub fn holds(&self, s: &State) -> bool {
        match *self {
            Self::Above(q, v) => q.value(s) > f64::from(v),
            Self::Below(q, v) => q.value(s) < f64::from(v),
            Self::Wheels { which, touching } => {
                let w = s.observation.wheels;
                let on = |i: &[usize]| i.iter().all(|&k| w[k] == touching);
                match which {
                    Wheels::Front => on(&[0, 1]),
                    Wheels::Rear => on(&[2, 3]),
                    Wheels::All => on(&[0, 1, 2, 3]),
                    Wheels::Any => w.contains(&touching),
                }
            }
            Self::Inside { min, max } => {
                let p = s.body.position();
                (0..3).all(|k| min[k] <= p[k] && p[k] <= max[k])
            }
        }
    }
}

impl Measure {
    fn oriented(&self, s: &State) -> f64 {
        let v = self.quantity.value(s);
        if self.higher { v } else { -v }
    }
}

impl Aim {
    pub fn judge(&self, s: &State) -> Verdict {
        Verdict {
            held: self.conditions.iter().all(|c| c.holds(s)),
            value: self.measure.oriented(s),
            tie: self.then.map_or(0.0, |m| m.oriented(s)),
        }
    }

    pub fn fastest() -> Self {
        Self {
            judged: Judged::End,
            measure: Measure { quantity: Quantity::Speed, higher: true },
            conditions: Vec::new(),
            then: None,
        }
    }

    pub fn longest_trick(kind: TrickKind) -> Self {
        Self {
            judged: Judged::End,
            measure: Measure { quantity: Quantity::Trick(kind), higher: true },
            conditions: Vec::new(),
            then: Some(Measure { quantity: Quantity::Speed, higher: true }),
        }
    }

    pub fn nose_position() -> Self {
        Self {
            judged: Judged::End,
            measure: Measure {
                quantity: Quantity::Angle { yaw: None, pitch: Some(-90.0), roll: None },
                higher: false,
            },
            conditions: vec![
                Condition::Above(Quantity::Speed, 390.0),
                Condition::Wheels { which: Wheels::Front, touching: true },
            ],
            then: Some(Measure { quantity: Quantity::Speed, higher: true }),
        }
    }

    pub fn land_flat() -> Self {
        Self {
            judged: Judged::End,
            measure: Measure {
                quantity: Quantity::Angle { yaw: None, pitch: Some(0.0), roll: Some(0.0) },
                higher: false,
            },
            conditions: vec![Condition::Wheels { which: Wheels::All, touching: true }],
            then: Some(Measure { quantity: Quantity::Speed, higher: true }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(pitch_deg: f32, speed: f32) -> Body {
        let (s, c) = pitch_deg.to_radians().sin_cos();
        let mut b = [0.0f32; 22];
        let rot = [1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c];
        b[..9].copy_from_slice(&rot);
        b[10] = 5.0;
        b[18] = speed;
        Body(b)
    }

    fn state<'a>(b: Body, wheels: [bool; 4], events: &'a Events) -> State<'a> {
        let v = b.local_velocity();
        let speed = v.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>().sqrt();
        State {
            body: b,
            observation: Observation {
                speed,
                slip: 0.0,
                pitch: 0.0,
                wheels,
                body_contact: false,
                wall_contact: false,
            },
            events,
        }
    }

    #[test]
    fn the_nose_angle_reads_as_pitch_and_wraps_the_short_way() {
        let e = Events::default();
        let s = state(body(-80.0, 120.0), [true, true, false, false], &e);
        assert!((Quantity::Pitch.value(&s) + 80.0).abs() < 1e-3);
        assert!(
            (Quantity::Angle { yaw: None, pitch: Some(-90.0), roll: None }.value(&s) - 10.0).abs() < 1e-3
        );
        assert_eq!(off(179.0, -179.0), 2.0);
    }

    #[test]
    fn conditions_rank_before_the_measure() {
        let e = Events::default();
        let aim = Aim::nose_position();
        let slow = aim.judge(&state(body(-89.0, 100.0), [true, true, false, false], &e));
        let fast = aim.judge(&state(body(-70.0, 120.0), [true, true, false, false], &e));
        assert!(!slow.held && fast.held);
        assert_eq!(fast.cmp(&slow), std::cmp::Ordering::Greater);
        let closer = aim.judge(&state(body(-85.0, 110.0), [true, true, false, false], &e));
        assert_eq!(closer.cmp(&fast), std::cmp::Ordering::Greater);
        let flat = aim.judge(&state(body(-85.0, 120.0), [true, true, true, true], &e));
        assert!(flat.held, "front wheels touching holds with the rear down too");
        assert!(!Condition::Wheels { which: Wheels::Rear, touching: false }.holds(&state(
            body(-85.0, 120.0),
            [true, true, true, true],
            &e
        )));
    }
}
