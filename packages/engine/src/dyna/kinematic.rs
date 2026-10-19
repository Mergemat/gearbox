use crate::{Iso, Quat, Vec3, math};
use super::BodyHandle;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Easing {
    #[default]
    Constant,
    Linear,
    Square,
    Decelerate,
    InOut,
    Unknown(u8),
}

impl Easing {
    pub fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Constant,
            1 => Self::Linear,
            2 => Self::Square,
            3 => Self::Decelerate,
            4 => Self::InOut,
            n => Self::Unknown(n),
        }
    }

    pub fn code(self) -> u8 {
        match self {
            Self::Constant => 0,
            Self::Linear => 1,
            Self::Square => 2,
            Self::Decelerate => 3,
            Self::InOut => 4,
            Self::Unknown(n) => n,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Segment {
    pub easing: Easing,
    pub reversed: u8,
    pub duration: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct MotionCurve {
    pub segments: Vec<Segment>,
    pub cached_duration: u32,
}

impl MotionCurve {
    pub fn duration(&mut self) -> u32 {
        if self.segments.is_empty() {
            return 1000;
        }
        if self.cached_duration == 0 {
            self.cached_duration = self.segments.iter().fold(0u32, |a, s| a.wrapping_add(s.duration));
        }
        self.cached_duration
    }

    pub fn eval(&self, time: f32) -> f32 {
        let mut start = 0u32;
        for segment in &self.segments {
            let end = segment.duration.wrapping_add(start);
            if end as f32 > time {
                let x = (time - start as f32) / segment.duration as f32;
                let y = match segment.easing {
                    Easing::Constant => 0.0,
                    Easing::Linear => x,
                    Easing::Square => x * x,
                    Easing::Decelerate => -((x - 2.0) * x),
                    Easing::InOut => math::ease_in_out(x),
                    Easing::Unknown(_) => {
                        start = end;
                        continue;
                    }
                };
                return if segment.reversed != 0 {
                    1.0 - y
                } else {
                    y
                };
            }
            start = end;
        }
        0.0
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct KinematicModel {
    pub translation: MotionCurve,
    pub translation_axis: u8,
    pub translation_start: f32,
    pub translation_end: f32,
    pub rotation: MotionCurve,
    pub rotation_axis: u8,
    pub rotation_start: f32,
    pub rotation_end: f32,
}

impl KinematicModel {
    pub fn transform(&mut self, now: i64, speed: f32) -> Iso {
        let mut pos = Vec3::ZERO;
        let d = self.translation.duration();
        if d != 0 {
            let value = self.translation.eval(phase(now, d, speed));
            pos[self.translation_axis as usize] =
                (self.translation_end - self.translation_start) * value + self.translation_start;
        }
        let d = self.rotation.duration();
        let quat = if d == 0 {
            Quat::IDENTITY
        } else {
            let value = self.rotation.eval(phase(now, d, speed));
            let mut axis = Vec3::ZERO;
            axis[self.rotation_axis as usize] = 1.0;
            let degrees = (self.rotation_end - self.rotation_start) * value + self.rotation_start;
            let angle = degrees / 180.0 * std::f32::consts::PI;
            let (s, c) = math::sincos_precise(angle * 0.5);
            Quat::new(c, s * axis.x, s * axis.y, s * axis.z)
        };
        Iso::from_quat_pos(quat, pos)
    }
}

pub fn phase(now: i64, duration: u32, speed: f32) -> f32 {
    let x = (duration as f32 * speed / 1000.0) as f64 * -1e9;
    let offset = if x.is_nan() || x >= 9.223372036854775807e18 || x < -9.223372036854775808e18 {
        i64::MIN
    } else {
        x as i64
    };
    let rem = now.wrapping_sub(offset).wrapping_rem(duration as i64 * 1_000_000);
    (rem as f64 * 1e-9) as f32 * 1000.0
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Motion {
    pub model: KinematicModel,
    pub phase: f32,
    pub current: Iso,
    pub past: Iso,
}

#[derive(Clone, Debug, PartialEq)]
pub struct KinematicConstraint {
    pub body: BodyHandle,
    pub placement: Iso,
    pub animation: u32,
}
