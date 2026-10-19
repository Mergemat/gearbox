use crate::{Vec3, dyna::BodyState};

pub struct VehicleBody<'a> {
    pub dyna: &'a mut crate::dyna::Dyna,
    pub handle: crate::dyna::BodyHandle,
    pub mass: f32,
    pub inverse_inertia: crate::Mat3,
}

impl VehicleBody<'_> {
    pub fn state(&self) -> BodyState {
        self.dyna.state(self.handle).clone()
    }
    pub fn center_of_mass(&self) -> Vec3 {
        self.dyna.body(self.handle).center_of_mass()
    }
    pub fn set_gravity(&mut self, gravity: f32) {
        self.dyna.body_mut(self.handle).record.coefficient = gravity;
    }
    pub fn apply_world_impulse(&mut self, impulse: Vec3, point: Vec3) {
        self.dyna.apply_impulse(self.handle, impulse, point, self.mass, self.inverse_inertia);
    }
}

impl VehicleBody<'_> {
    pub fn local_velocity(&self) -> Vec3 {
        self.dyna.local_velocity(self.handle)
    }
    pub fn local_angular_velocity(&self) -> Vec3 {
        self.dyna.local_angular_velocity(self.handle)
    }
    pub fn local_force(&self) -> Vec3 {
        self.dyna.local_force(self.handle)
    }
    pub fn set_local_velocity(&mut self, velocity: Vec3) {
        self.dyna.set_local_velocity(self.handle, velocity)
    }
    pub fn set_local_angular_velocity(&mut self, velocity: Vec3) {
        self.dyna.set_local_angular_velocity(self.handle, velocity)
    }
    pub fn set_local_force(&mut self, force: Vec3) {
        self.dyna.set_local_force(self.handle, force)
    }
    pub fn set_local_torque(&mut self, torque: Vec3) {
        self.dyna.set_local_torque(self.handle, torque)
    }
    pub fn add_force(&mut self, force: Vec3) {
        self.dyna.add_force(self.handle, force)
    }
    pub fn add_torque(&mut self, torque: Vec3) {
        self.dyna.add_torque(self.handle, torque)
    }
    pub fn add_local_force(&mut self, force: Vec3) {
        self.dyna.add_local_force(self.handle, force)
    }
    pub fn add_local_torque(&mut self, torque: Vec3) {
        self.dyna.add_local_torque(self.handle, torque)
    }
    pub fn add_local_force_at(&mut self, force: Vec3, point: Vec3) {
        self.dyna.add_local_force_at(self.handle, force, point)
    }
    pub fn velocity_at(&self, point: Vec3) -> Vec3 {
        self.dyna.body(self.handle).velocity_at(point)
    }
    pub fn local_point_velocity(&self, point: Vec3) -> Vec3 {
        self.dyna.local_point_velocity(self.handle, point)
    }
    pub fn set_world_velocity(&mut self, velocity: Vec3) {
        self.dyna.set_velocity(self.handle, velocity)
    }
    pub fn add_world_angular_velocity(&mut self, velocity: Vec3) {
        self.dyna.add_angular_velocity(self.handle, velocity)
    }
}
