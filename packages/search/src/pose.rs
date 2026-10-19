use crate::Body;
use anyhow::{Result, ensure};
use gearbox_engine::Session;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub velocity: [f32; 3],
}
impl Pose {
    pub fn from_body(body: Body) -> Self {
        Self {
            position: body.position(),
            rotation: body.0[12..16].try_into().unwrap(),
            velocity: body.local_velocity(),
        }
    }
    pub fn validate(&self) -> Result<()> {
        ensure!(self.position.iter().chain(&self.velocity).all(|v| v.is_finite()), "non-finite pose");
        unit(self.rotation)?;
        Ok(())
    }
}

fn unit(q: [f32; 4]) -> Result<[f32; 4]> {
    let n = q.iter().map(|&v| f64::from(v).powi(2)).sum::<f64>().sqrt();
    ensure!(n.is_finite() && n > 1e-12, "invalid quaternion");
    Ok(q.map(|v| (f64::from(v) / n) as f32))
}
pub fn rotation_distance(a: [f32; 4], b: [f32; 4]) -> f64 {
    let norm = |q: [f32; 4]| q.iter().map(|&v| f64::from(v).powi(2)).sum::<f64>().sqrt();
    let n = norm(a) * norm(b);
    if !n.is_finite() || n <= 1e-24 {
        return f64::INFINITY;
    }
    let dot = (0..4).map(|k| f64::from(a[k]) * f64::from(b[k])).sum::<f64>();
    2.0 * (dot.abs() / n).clamp(0.0, 1.0).acos()
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseGoal {
    pub pose: Pose,
    pub position_tolerance: [f32; 3],
    pub angle_tolerance: f32,
    pub velocity_tolerance: Option<[f32; 3]>,
    pub position_weights: [f32; 3],
    pub angle_weight: f32,
    pub velocity_weights: [f32; 3],
}
impl PoseGoal {
    pub fn validate(&self) -> Result<()> {
        self.pose.validate()?;
        ensure!(
            self.position_tolerance
                .iter()
                .chain(&self.position_weights)
                .chain(&self.velocity_weights)
                .chain(self.velocity_tolerance.iter().flatten())
                .chain([&self.angle_tolerance, &self.angle_weight])
                .all(|v| v.is_finite() && *v >= 0.0),
            "invalid pose tolerance/weight"
        );
        ensure!(
            self.position_weights.iter().any(|v| *v > 0.0)
                || self.angle_weight > 0.0
                || (self.velocity_tolerance.is_some() && self.velocity_weights.iter().any(|v| *v > 0.0)),
            "pose metric has no positive weight"
        );
        Ok(())
    }
    pub fn error(&self, actual: Pose) -> f64 {
        let mut error =
            f64::from(self.angle_weight) * rotation_distance(self.pose.rotation, actual.rotation).powi(2);
        for k in 0..3 {
            error += f64::from(self.position_weights[k])
                * (f64::from(actual.position[k]) - f64::from(self.pose.position[k])).powi(2);
            if self.velocity_tolerance.is_some() {
                error += f64::from(self.velocity_weights[k])
                    * (f64::from(actual.velocity[k]) - f64::from(self.pose.velocity[k])).powi(2);
            }
        }
        if error.is_finite() { error } else { f64::INFINITY }
    }
    pub fn contains(&self, actual: Pose) -> bool {
        (0..3).all(|k| (actual.position[k] - self.pose.position[k]).abs() <= self.position_tolerance[k])
            && rotation_distance(self.pose.rotation, actual.rotation) <= f64::from(self.angle_tolerance)
            && self
                .velocity_tolerance
                .is_none_or(|t| (0..3).all(|k| (actual.velocity[k] - self.pose.velocity[k]).abs() <= t[k]))
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Placement {
    pub pose: Pose,
    pub angular_velocity: [f32; 3],
    pub reset_contacts: bool,
}
pub fn place(native: &mut Session, placement: &Placement) -> Result<()> {
    placement.pose.validate()?;
    ensure!(placement.angular_velocity.iter().all(|v| v.is_finite()), "non-finite angular velocity");
    let q = unit(placement.pose.rotation)?;
    let rotation = gearbox_engine::Mat3::from_quat(gearbox_engine::Quat::new(q[0], q[1], q[2], q[3]));
    native.place_car(
        gearbox_engine::Iso::new(rotation, placement.pose.position.into()),
        placement.pose.velocity.into(),
        placement.angular_velocity.into(),
        placement.reset_contacts,
    );
    Ok(())
}

pub struct PlacedOrigin<'a> {
    pub origin: &'a dyn crate::search::Origin,
    pub placement: Placement,
}
impl crate::search::Origin for PlacedOrigin<'_> {
    fn native(&self) -> Result<Session> {
        let mut n = self.origin.native()?;
        place(&mut n, &self.placement)?;
        Ok(n)
    }
}
