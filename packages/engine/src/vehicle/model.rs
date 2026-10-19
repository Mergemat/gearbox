use crate::{Aabb, Vec3};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfaceMaterial {
    pub speed_limit_scale: f32,
    pub thrust_scale: f32,
    pub brake_scale: f32,
    pub grip_scale: f32,
    pub rolling_response: f32,
    pub sliding_response: f32,
    pub contact_response: f32,
    pub lateral_drag: f32,
    pub lateral_thrust: f32,
    pub yaw_drag: f32,
    pub gripping: u32,
    pub immersion_brake_drain: f32,
    pub steering_rate: f32,
    pub steering_visual_rate: f32,
    pub pivot_enabled: u32,
    pub bump_scale: f32,
    pub bump_limit: f32,
    pub bumps_enabled: u32,
    pub bump_period_x: f32,
    pub bump_period_z: f32,
    pub index: u8,
    pub thrust_by_speed: crate::Curve,
    pub steering_angle_by_speed: crate::Curve,
    pub steering_by_speed: crate::Curve,
    pub grip_by_immersion: crate::Curve,
    pub thrust_by_alignment: crate::Curve,
    pub bump_speed_period: f32,
    pub bump_amplitude_scale: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CarShape {
    pub bounds: Aabb,
    pub wheel_radius: [f32; 2],
    pub wheel_width: [f32; 2],
    pub water_bounds: Aabb,
    pub manual_wheel_positions: u32,
    pub wheel_height: f32,
    pub front_axle: f32,
    pub rear_axle: f32,
    pub front_half_track: f32,
    pub rear_half_track: f32,
    pub lateral_offset: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelDefinition {
    pub steers: bool,
    pub radius: f32,
    pub anchor: Vec3,
    pub contact_flag: bool,
    pub transform_index: Option<usize>,
    pub contact_radius: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ForceGeometry {
    pub mass: f32,
    pub center_of_mass: Vec3,
    pub water_box: Aabb,
    pub wheels: Vec<WheelDefinition>,
    pub materials: Vec<SurfaceMaterial>,
    pub material14_alias: bool,
    pub thrust_scale: f32,
    pub steering_scale: f32,
    pub gravity_scale: f32,
    pub grip_scale: f32,
    pub reference_center: Vec3,
    pub local_inverse_inertia: crate::Mat3,
    pub axle_span: f32,
    pub effect_contacts: Vec<bool>,
}

impl ForceGeometry {
    pub fn material(&self, material: u8) -> &SurfaceMaterial {
        let index = if self.material14_alias && material == 14 {
            0
        } else {
            material as usize
        };
        &self.materials[index]
    }
}
