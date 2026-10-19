use super::{Vehicle, forces::common::TINY, model::ForceGeometry};
use crate::{
    Mat3, Vec3,
    dyna::{BodyHandle, BodyState, Dyna},
};

pub fn integrate_body(vehicle: &mut Vehicle, geometry: &ForceGeometry, dyna: &mut Dyna, dt: f32) {
    let handle = BodyHandle(vehicle.body);
    let body = dyna.body_mut(handle);
    let state = &mut body.state;
    let pending = vehicle.extra_linear_velocity;
    state.pos.x = (pending.x + state.vel.x) * dt + state.pos.x;
    state.pos.y = (state.vel.y + pending.y) * dt + state.pos.y;
    state.pos.z = (pending.z + state.vel.z) * dt + state.pos.z;
    let angular = vehicle.extra_angular_velocity + state.ang_vel;
    rotate_about_center(state, body.model.center(), angular, dt);
    let scale = dt / geometry.mass;
    state.vel.x = body.acc.force.x * scale + state.vel.x;
    state.vel.y = body.acc.force.y * scale + state.vel.y;
    state.vel.z = body.acc.force.z * scale + state.vel.z;
    let torque = body.acc.torque;
    let inertia = vehicle.world_inverse_inertia.to_array();
    let delta = Vec3::new(
        ((torque.y * inertia[1] + torque.x * inertia[0]) + torque.z * inertia[2]) * dt,
        ((torque.y * inertia[4] + torque.x * inertia[3]) + torque.z * inertia[5]) * dt,
        ((torque.y * inertia[7] + torque.x * inertia[6]) + torque.z * inertia[8]) * dt,
    );
    dyna.add_angular_velocity(handle, delta);
    let body = dyna.body(handle);
    vehicle.world_inverse_inertia = world_inertia(body.state.rot, geometry.local_inverse_inertia);
    let inertia = world_inertia(body.state.rot, body.model.inverse_inertia_matrix);
    dyna.set_diagonal(handle, inertia);
}

fn rotate_about_center(state: &mut BodyState, center: Vec3, angular: Vec3, dt: f32) {
    let squared = (angular.y * angular.y + angular.x * angular.x) + angular.z * angular.z;
    if !(squared > TINY) {
        return;
    }
    let previous = state.rot.to_array();
    state.quat = state.quat.integrate(angular, dt);
    state.rot = Mat3::from_quat(state.quat);
    let next = state.rot.to_array();
    let new_center = Vec3::new(
        (center.x * next[0] + center.y * next[1]) + center.z * next[2],
        (center.x * next[3] + center.y * next[4]) + center.z * next[5],
        (center.x * next[6] + center.y * next[7]) + center.z * next[8],
    );
    let old_center = Vec3::new(
        (center.x * previous[0] + center.y * previous[1]) + center.z * previous[2],
        (center.x * previous[3] + center.y * previous[4]) + center.z * previous[5],
        (center.y * previous[7] + center.x * previous[6]) + center.z * previous[8],
    );
    state.pos = state.pos - (new_center - old_center);
}

pub fn premultiply(rotation: Mat3, value: Mat3) -> Mat3 {
    let left = rotation.to_array();
    let right = value.to_array();
    Mat3::from_array(std::array::from_fn(|index| {
        let (row, column) = (index / 3, index % 3);
        let term = |axis: usize| right[3 * axis + column] * left[3 * row + axis];
        if row == 0 { (term(0) + term(1)) + term(2) } else { (term(1) + term(0)) + term(2) }
    }))
}

pub fn world_inertia(rotation: Mat3, local: Mat3) -> Mat3 {
    premultiply(rotation, premultiply(local, rotation.transpose()))
}
