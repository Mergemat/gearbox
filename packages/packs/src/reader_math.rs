pub use gearbox_engine::math::{angle_approx, cvtt, maxss, minss, sincos_precise, sincos as sincos_value};

pub fn matrix_to_quat_values(r: [f32; 9]) -> [f32; 4] {
    gearbox_engine::Quat::from_mat3(&gearbox_engine::Mat3::from_array(r)).to_array()
}
pub fn quat_matrix(q: [f32; 4]) -> [f32; 9] {
    gearbox_engine::Quat::from_array(q).to_mat3().to_array()
}
pub fn angles_quat(a: f32, b: f32, c: f32) -> [f32; 4] {
    gearbox_engine::Quat::from_angles(a, b, c).to_array()
}
