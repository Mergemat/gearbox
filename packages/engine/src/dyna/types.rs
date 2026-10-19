use super::SolverFlags;
use crate::{Mat3, Quat, Vec3};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct DynaBody {
    pub flags: SolverFlags,
    pub added_mass: f32,
    pub coefficient: f32,
    pub has_pre_collision_callback: bool,
    pub parent: u32,
    pub parent_slot: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyState {
    pub rot: Mat3,
    pub pos: Vec3,
    pub quat: Quat,
    pub vel: Vec3,
    pub ang_vel: Vec3,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyInertia {
    pub diagonal: Vec3,
    pub inverse_mass: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyIndex {
    pub rest_since: u32,
    pub index: u16,
    pub wake_disabled: u8,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ForceAcc {
    pub force: Vec3,
    pub torque: Vec3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynaModel {
    pub mass: f32,
    pub inverse_inertia_matrix: crate::Mat3,
    pub center_x: f32,
    pub center_y: f32,
    pub center_z: f32,
    pub max_angular_velocity: f32,
    pub externally_simulated: u32,
    pub substep_distance: f32,
    pub kind: u8,
}

impl Default for DynaModel {
    fn default() -> Self {
        Self {
            mass: 0.0,
            inverse_inertia_matrix: crate::Mat3::diagonal(crate::Vec3::ZERO),
            center_x: 0.0,
            center_y: 0.0,
            center_z: 0.0,
            max_angular_velocity: 0.0,
            externally_simulated: 0,
            substep_distance: 0.0,
            kind: 0,
        }
    }
}
