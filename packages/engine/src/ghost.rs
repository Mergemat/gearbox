mod trig;

pub use trig::sin_cos;

use crate::{Iso, Mat3, Quat, Vec3, vehicle::ghost::Contact};

#[derive(Clone, Debug, Default)]
pub struct Playback {
    pub samples: Vec<Sample>,
    pub clones: Vec<CloneConfig>,
    pub start_ms: i32,
    pub end_ms: i32,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub time_ms: i32,
    pub sequence: u8,
    pub contact: Contact,
}

#[derive(Clone, Copy, Debug)]
pub struct CloneConfig {
    pub start: f32,
    pub speed: f32,
}

impl Playback {
    pub fn contacts(&self, time: f32) -> Vec<Contact> {
        self.clones
            .iter()
            .filter_map(|clone| {
                if time < clone.start {
                    return None;
                }
                let local = ((time - clone.start) * clone.speed).max(0.0);
                let ms = ((local as f64 * 1_000_000_000.0) as i64 / 1_000_000) as i32;
                if ms < self.start_ms || self.end_ms < ms {
                    return None;
                }
                let mut contact = self.sample(ms, local)?;
                contact.velocity = contact.velocity * clone.speed;
                Some(contact)
            })
            .collect()
    }

    fn sample(&self, ms: i32, time: f32) -> Option<Contact> {
        let last = self.samples.len().checked_sub(1)?;
        let b = self.samples.partition_point(|s| s.time_ms <= ms).min(last);
        let a = if self.samples[b].time_ms > ms { b.saturating_sub(1) } else { b };
        let prev = a.saturating_sub(1);
        let next = (b + 1).min(last);
        let [p, a, b, n] = [prev, a, b, next].map(|i| self.samples[i]);
        if a.sequence != b.sequence {
            return Some(b.contact);
        }
        let k = if a.time_ms == b.time_ms {
            0.0
        } else {
            (time - a.time_ms as f32 * 0.001) / ((b.time_ms - a.time_ms) as f32 * 0.001)
        };
        let ap = a.contact.pose.pos;
        let bp = b.contact.pose.pos;
        let mut position = (bp - ap) * k + ap;
        if [p, a, b, n].iter().all(|s| s.sequence == b.sequence) {
            let da = (bp - p.contact.pose.pos) * 0.5;
            let db = (n.contact.pose.pos - ap) * 0.5;
            let k2 = k * k;
            let k3 = k2 * k;
            position = Vec3::from_array(std::array::from_fn(|i| {
                (((bp[i] - ap[i]) * 3.0 - (da[i] + da[i])) - db[i]) * k2
                    + (da[i] * k + ap[i])
                    + ((ap[i] - bp[i]) + (ap[i] - bp[i]) + da[i] + db[i]) * k3
            }));
        }
        Some(Contact {
            pose: Iso::new(rotation(a.contact.pose.rot, b.contact.pose.rot, k), position),
            velocity: (b.contact.velocity - a.contact.velocity) * k + a.contact.velocity,
        })
    }
}

fn rotation(a: Mat3, b: Mat3, k: f32) -> Mat3 {
    let original = Quat::from_mat3(&a).to_array();
    let mut qa = original;
    let qb = Quat::from_mat3(&b).to_array();
    let mut dot = ((qa[2] * qb[2] + qb[1] * qa[1]) + qa[3] * qb[3]) + qa[0] * qb[0];
    if dot < 0.0 {
        dot = -dot;
        qa = qa.map(|v| -v);
    }
    let mut left = 1.0 - k;
    let mut right = k;
    if 0.00001 < k.abs().max(left.abs()) * (1.0 - dot) {
        let angle = (dot as f64).acos() as f32;
        let sin = sin_cos(angle).0;
        if 0.00001 <= sin.abs() {
            let inv = 1.0 / sin;
            left = sin_cos(angle * left).0 * inv;
            right = sin_cos(angle * right).0 * inv;
        }
    }
    let q = std::array::from_fn(|i| right * qb[i] + left * qa[i]);
    if (0..4).all(|i| (original[i] - q[i]).abs() <= original[i].abs().max(q[i].abs()).max(1.0) * 0.00001) {
        a
    } else {
        Quat::from_array(q).to_mat3()
    }
}
