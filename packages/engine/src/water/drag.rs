use crate::{Curve, Vec3, dyna::ForceAcc, math::{fast_exp2, fast_log2}};

pub const NORMAL_MIN_LENGTH_SQUARED: f32 = 1.0e-10;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FluidTuning {
    pub reference_speed: f32,
    pub scale: f32,
    pub facing: Curve,
    pub facing_exponent: f32,
    pub trailing: Curve,
    pub trailing_exponent: f32,
    pub reference_length: f32,
    pub tangential: Curve,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FluidForces {
    pub hydrostatic: ForceAcc,
    pub facing: ForceAcc,
    pub trailing: ForceAcc,
    pub tangential: ForceAcc,
}

impl FluidForces {
    pub fn total(&self) -> ForceAcc {
        let mut total = ForceAcc::default();
        for lane in [&self.hydrostatic, &self.facing, &self.trailing, &self.tangential] {
            total.force = Vec3::new(total.force.x + lane.force.x,
                total.force.y + lane.force.y, total.force.z + lane.force.z);
            total.torque = Vec3::new(total.torque.x + lane.torque.x,
                lane.torque.y + total.torque.y, lane.torque.z + total.torque.z);
        }
        total
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SurfaceElement {
    pub velocity: Vec3,
    pub normal: Vec3,
    pub point: Vec3,
    pub center: Vec3,
}

fn direction(velocity: Vec3) -> (Vec3, f32) {
    let length_squared = velocity.dot_yxz(velocity);
    if length_squared > NORMAL_MIN_LENGTH_SQUARED && f32::MAX > length_squared {
        let speed = length_squared.sqrt();
        (velocity * (1.0 / speed), speed)
    } else {
        (velocity, 0.0)
    }
}

fn accumulate(acc: &mut ForceAcc, force: Vec3, relative: Vec3) {
    acc.force = force + acc.force;
    acc.torque.x = (relative.y * force.z - relative.z * force.y) + acc.torque.x;
    acc.torque.y = (force.x * relative.z - relative.x * force.z) + acc.torque.y;
    acc.torque.z = (relative.x * force.y - force.x * relative.y) + acc.torque.z;
}

pub fn surface_drag(element: SurfaceElement, pressure_scale: f32, area_scale: f32,
    friction: f32, tuning: &FluidTuning, forces: &mut FluidForces)
{
    let (unit_velocity, speed) = direction(element.velocity);
    let relative = element.point - element.center;
    let ratio = speed / tuning.reference_speed;
    let dot = element.normal.dot(unit_velocity);
    let facing = dot >= 0.0;
    let (exponent, curve) = if facing {
        (tuning.facing_exponent, &tuning.facing)
    } else {
        (tuning.trailing_exponent, &tuning.trailing)
    };
    let power = fast_exp2(fast_log2(dot.abs()) * exponent);
    let mut magnitude = power * (curve.eval(ratio) * pressure_scale) * tuning.scale;
    if facing { magnitude = magnitude * -1.0; }
    let force = element.normal * magnitude;
    let lane = if facing { &mut forces.facing } else { &mut forces.trailing };
    accumulate(lane, force, relative);
    let normal_velocity = element.normal.dot(element.velocity);
    let tangent = element.velocity - element.normal * normal_velocity;
    let (unit_tangent, _) = direction(tangent);
    let scale = tuning.tangential.eval(ratio)
        * (area_scale * 0.5 * friction * pressure_scale) * tuning.scale;
    let force = (unit_tangent * -1.0) * scale;
    accumulate(&mut forces.tangential, force, relative);
}
