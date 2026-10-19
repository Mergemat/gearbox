use super::{Vehicle, Wheel, race_flags::RaceFlags};
use crate::{Aabb, DynamicItemId, Iso, Mat3, Quat, ShapeId, Vec3, dyna::{BodyHandle, Dyna}, math::minss};
#[path = "spawn_fields.rs"]
mod fields;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelModel { pub position: Vec3, pub shape: u32 }
#[derive(Clone, Debug, PartialEq)]
pub struct Assets {
    pub wheels: Vec<WheelModel>,
    pub water_bounds: Aabb,
    pub travel: f32,
    pub water_travel: f32,
    pub airborne_height: f32,
    pub gravity_coefficient: f32,
    pub model_gravity: f32,
    pub inverse_inertia: Mat3,
    pub effect_defaults: [f32; 3],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reset {
    pub full: bool,
    pub keep_effects: bool,
    pub spin_up_torque: f32,
    pub airborne: bool,
}
impl Reset {
    pub const FULL: Self = Self { full: true, keep_effects: false, spin_up_torque: 0.0, airborne: false };
}

pub trait Visuals {
    fn visual(&mut self, vehicle: &mut Vehicle, dyna: &Dyna, fresh: bool);
}

pub struct Services<'a, V: Visuals> {
    pub world: &'a crate::World,
    pub state: &'a mut crate::State,
    pub visuals: &'a mut V,
    pub ring: &'a mut EffectRing,
}
impl<V: Visuals> Services<'_, V> {
    fn submerged(&self, vehicle: &Vehicle, bounds: Aabb) -> bool {
        let bounds = bounds.transform_quat(vehicle.quat, vehicle.pos);
        let half = bounds.half.y.abs();
        let point = Vec3::new(bounds.center.x, bounds.center.y - half, bounds.center.z);
        let mut depth = 0.0;
        crate::water::WaterManager::new(&self.world.water)
            .query(point, half + half, Some(&mut depth), None)
    }

    fn place_item(&mut self, id: DynamicItemId, pose: Iso) {
        let item = &mut self.state.dynamic_item[id.index()];
        item.iso = pose;
        item.pos = pose.pos;
        item.quat = Quat::from_mat3(&pose.rot);
    }

    fn wheel_pose(&mut self, compound: Option<ShapeId>, shape: u32, pose: Iso) {
        let id = compound.and_then(ShapeId::compound).expect("vehicle compound instance");
        self.state.compound_instance[id.index()].transforms[shape as usize] = pose;
    }
}

pub fn arena_pose(initial: Iso, parent: Option<Iso>, landmarks: &[super::super::race::RaceWaypoint],
    landmark: u32) -> Iso {
    if landmark == crate::race::INITIAL_SPAWN {
        parent.map_or(initial, |parent| Iso::new(transpose_product(initial.rot, parent.rot),
            parent.rot.transform_y_first(initial.pos) + parent.pos))
    } else { landmarks[landmark as usize].pose }
}

pub fn clear_inputs(vehicle: &mut Vehicle) {
    clear_current_inputs(vehicle);
    fields::clear_previous_controls(vehicle);
    vehicle.previous_controls.control_word = super::race_flags::InputFlags(vehicle.previous_controls.control_word).cleared().0;
}

pub fn clear_current_inputs(vehicle: &mut Vehicle) {
    fields::clear_current_controls(vehicle);
    vehicle.controls.control_word = super::race_flags::InputFlags(vehicle.controls.control_word).cleared().0;
}

pub fn wheel_state(wheel: &mut Wheel) {
    wheel.suspension_length = 0.0; wheel.suspension_rate = 0.0; wheel.suspension_delta = 0.0;
    wheel.contact_point_x = 0.0;
    wheel.contact_point_y = 0.0; wheel.contact_point_z = 0.0; wheel.spin = 0.0;
    wheel.previous_contact = 0; wheel.contact = 0; wheel.last_impact_time_ns = u64::MAX;
    wheel.material = 0;
    wheel.surface = 0;
    wheel.contact_normal_x = 0.0; wheel.contact_normal_y = 0.0; wheel.contact_normal_z = 0.0; wheel.hover_depth = 0.0;
    wheel.boost_contact = 0; wheel.surface_normal.x = 0.0; wheel.surface_normal.y = 0.0; wheel.surface_normal.z = 0.0;
    wheel.surface_item = None; wheel.slipping = 0;
    wheel.slip = 0.0; wheel.spin_angle = 0.0; wheel.steering_angle = 0.0;
    wheel.sample_time = 0; wheel.sample_count = 0; wheel.sample_sum = 0.0; wheel.drive_coefficient = 1.0;
    wheel.effect_mode = 0; wheel.effect_start = u32::MAX;
    wheel.effect_level = 0.0; wheel.effect_floor = 0.0; wheel.ground_pull_distance = 0.0;
}

pub fn transpose_product(left: Mat3, right: Mat3) -> Mat3 {
    let left = left.to_array(); let right = right.to_array();
    Mat3::from_array(std::array::from_fn(|index| {
        let (row, column) = (index / 3, index % 3);
        (left[3 + column] * right[3 * row + 1] + left[column] * right[3 * row])
            + left[6 + column] * right[3 * row + 2]
    }))
}

pub fn post_product(left: Mat3, right: Mat3) -> Mat3 {
    let left = left.to_array(); let right = right.to_array();
    Mat3::from_array(std::array::from_fn(|index| {
        let (row, column) = (index / 3, index % 3);
        let term = |axis: usize| right[3 * axis + row] * left[3 * axis + column];
        if column == 0 || index == 7 { (term(1) + term(0)) + term(2) }
        else { (term(0) + term(1)) + term(2) }
    }))
}

fn effect_state(vehicle: &mut Vehicle, assets: &Assets, keep: bool, services: &mut Services<impl Visuals>) {
    vehicle.gear_effect_thrust = 1.0;
    let ring = &mut services.ring;
    ring.reset();
    vehicle.last_jump = 0;
    vehicle.recent_effect_time = 0;
    vehicle.vehicle_effect_thrust = assets.effect_defaults[0];
    vehicle.vehicle_effect_speed_scale = assets.effect_defaults[1];
    vehicle.vehicle_effect_gravity = assets.effect_defaults[2];
    vehicle.gravity_effect_scale = 1.0; vehicle.post_force_effect_active = 0;
    if !keep {
        vehicle.gear_effect_active = 0; vehicle.gravity_effect_active = 0; vehicle.jump_kind = 0;
        vehicle.axis_torque_enabled = 0; vehicle.vehicle_effect_active = 0; vehicle.gravity_effect_thrust = 1.0; vehicle.vehicle_effect_amplified = 0;
        vehicle.external_jump = 0;
    }
}

fn contact_state(vehicle: &mut Vehicle, full: bool) {
    fields::clear_contact_history(vehicle);
    vehicle.suspension_frame = Mat3::IDENTITY;
    if full {
        fields::clear_full_contact_history(vehicle);
        set_history_poses(vehicle);
    }
}

fn set_history_poses(vehicle: &mut Vehicle) {
    vehicle.history_poses = [[1.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0].map(f32::to_bits); 4];
}

fn wheels(vehicle: &mut Vehicle, assets: &Assets, airborne: bool, services: &mut Services<impl Visuals>) {
    let submerged = services.submerged(vehicle, assets.water_bounds);
    vehicle.in_water = submerged as u32;
    let travel = if submerged { assets.water_travel } else { assets.travel };
    let height = if airborne { assets.airborne_height } else { travel };
    for (index, model) in assets.wheels.iter().enumerate() {
        let wheel = &mut vehicle.wheels[index];
        wheel_state(wheel);
        wheel.suspension_length = height;
        let position = model.position;
        let pose = Iso::new(Mat3::IDENTITY, Vec3::new(position.x + -height * 0.0,
            (position.y + travel) + -height, position.z + -height * 0.0));
        vehicle.wheel_poses[index].transform = pose;
        if model.shape != u32::MAX { services.wheel_pose(vehicle.shape, model.shape, pose); }
    }
}

pub fn reset(vehicle: &mut Vehicle, assets: &Assets, dyna: &mut Dyna, params: Reset,
    services: &mut Services<impl Visuals>) {
    clear_inputs(vehicle);
    fields::clear_reset_state(vehicle);
    let sequences = [vehicle.state_sequence, vehicle.respawn_sequence];
    let (quat, pos, count) = (vehicle.quat, vehicle.pos, vehicle.engine_flags);
    let history = history(vehicle);
    super::defaults::reset_wheel_coefficients(vehicle);
    vehicle.slow_motion_gate = u32::MAX;
    super::defaults::reset_state(vehicle);
    if !params.full { restore_history(vehicle, history); }
    vehicle.state_sequence = sequences[0]; vehicle.respawn_sequence = sequences[1];
    vehicle.spin_up_torque = params.spin_up_torque; vehicle.spin_up_duration = params.airborne as u32;
    vehicle.flags = RaceFlags(vehicle.flags).reset().0;
    vehicle.quat = quat; vehicle.pos = pos; vehicle.engine_flags = count;
    if params.full {
        vehicle.control_entity = u32::MAX;
        if vehicle.detached_wheel_count != 0 { todo!("detached wheel bodies"); }
    }
    vehicle.control_flags = crate::race::countdown::ControlFlags(vehicle.control_flags).after_reset().0;
    let mut shapes = [u32::MAX; 4];
    for (index, model) in assets.wheels.iter().enumerate() { shapes[index] = model.shape; }
    [vehicle.wheel_transform_0, vehicle.wheel_transform_1,
        vehicle.wheel_transform_2, vehicle.wheel_transform_3] = shapes;
    wheels(vehicle, assets, params.airborne, services);
    if vehicle.body != u32::MAX {
        let handle = BodyHandle(vehicle.body);
        let body = dyna.body_mut(handle);
        body.state.vel = Vec3::ZERO; body.state.ang_vel = Vec3::ZERO;
        dyna.clear_acc(handle);
    }
    let gravity = assets.model_gravity * vehicle.gravity_scale;
    let gravity = if gravity >= 0.0 { minss(gravity, 1.0) } else { 0.0 };
    vehicle.gravity = gravity * assets.gravity_coefficient;
    if vehicle.body != u32::MAX {
        dyna.body_mut(BodyHandle(vehicle.body)).record.coefficient = vehicle.gravity;
    }
    let rotation = Mat3::from_quat(quat);
    vehicle.world_inverse_inertia = post_product(transpose_product(rotation, assets.inverse_inertia), rotation);
    effect_state(vehicle, assets, params.keep_effects, services);
    reset_sentinels(vehicle);
    contact_state(vehicle, params.full);
    services.visuals.visual(vehicle, dyna, false);
    vehicle.visual[0] = vehicle.visual[1].clone();
}

fn history(vehicle: &Vehicle) -> [u32; 9] { vehicle.wear.map(f32::to_bits) }
fn restore_history(vehicle: &mut Vehicle, history: [u32; 9]) { vehicle.wear = history.map(f32::from_bits); }
fn reset_sentinels(vehicle: &mut Vehicle) {
    vehicle.sound_speed = -1.0;
}

pub fn reset_at_pose(vehicle: &mut Vehicle, assets: &Assets, dyna: &mut Dyna, params: Reset,
    services: &mut Services<impl Visuals>) {
    if params.keep_effects && vehicle.effect_flags != 0 { todo!("respawn event"); }
    reset(vehicle, assets, dyna, params, services);
    vehicle.visual[1] = super::defaults::VISUAL_STATE.clone();
    services.visuals.visual(vehicle, dyna, true);
    vehicle.visual[0] = vehicle.visual[1].clone();
    if vehicle.replica != 0 { todo!("replica reset"); }
}

pub fn respawn(vehicle: &mut Vehicle, assets: &Assets, dyna: &mut Dyna, pose: Iso, params: Reset,
    services: &mut Services<impl Visuals>) {
    vehicle.respawn_sequence = vehicle.respawn_sequence.wrapping_add(1);
    vehicle.pos = pose.pos;
    vehicle.quat = Quat::from_mat3(&pose.rot);
    if vehicle.body != u32::MAX {
        dyna.set_pose(BodyHandle(vehicle.body), pose);
    }
    if let Some(item) = vehicle.item { services.place_item(item, pose); }
    reset_at_pose(vehicle, assets, dyna, params, services);
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EffectRing { pub values: Vec<u32>, pub head: u32, pub len: u32 }
impl EffectRing {
    pub fn reset(&mut self) {
        if self.values.is_empty() { self.values.push(0); }
        self.len = 1;
        self.head = self.values.len() as u32 - 1;
        self.values[self.head as usize] = u32::MAX;
    }
}

pub fn constructed(entity: u32) -> Vehicle {
    const INVALID_TIME: u32 = u32::MAX;
    let mut vehicle = Vehicle::default();
    super::defaults::reset_full_state(&mut vehicle);
    vehicle.visual = [super::defaults::VISUAL_STATE.clone(), super::defaults::VISUAL_STATE.clone()];
    vehicle.key = entity;
    vehicle.control_flags = 1;
    vehicle.body = INVALID_TIME;
    vehicle.collision_items_len = 1;
    vehicle.tag = u8::MAX;
    vehicle.slow_motion_gate = INVALID_TIME;
    vehicle.surface_contact_gate = INVALID_TIME;
    vehicle.previous_level_power = vehicle.level_power_scale;
    clear_current_inputs(&mut vehicle);
    vehicle
}
