use super::{CarAssets, geometry};
use anyhow::{Result, ensure};
use gearbox_engine::{
    DynamicItemId, Iso, ShapeId, State, World,
    collision::{DynaCollision, DynamicItem},
    dyna::{BodyParams, Dyna, ForceModels},
    vehicle::{Vehicle, spawn},
};

const VEHICLE_LAYER: u8 = 4;
const NO_ENTRY: u16 = u16::MAX;

#[derive(Clone, Debug)]
pub struct VehicleStart {
    pub pose: Iso,
    pub entity: u32,
    pub class: u32,
    pub collision_manager: usize,
}

pub fn new_vehicle(
    assets: &CarAssets,
    start: VehicleStart,
    world: &mut World,
    state: &mut State,
    dyna: &mut Dyna,
) -> Result<Vehicle> {
    ensure!(start.collision_manager < state.collision_world.len(), "missing vehicle collision manager");
    let template = assets
        .template
        .compound()
        .and_then(|id| state.compound_instance.get(id.index()))
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("missing car compound template"))?;
    let transforms = template.transforms;
    let tags = template.tags;
    let shape = geometry::instance(template.base, transforms, tags, state);
    let mut vehicle = spawn::constructed(start.entity);
    vehicle.shape = Some(shape);
    vehicle.quat = gearbox_engine::Quat::from_mat3(&start.pose.rot);
    vehicle.pos = start.pose.pos;
    vehicle.wheel_count = 4;
    vehicle.thrust_scale = assets.geometry.thrust_scale;
    vehicle.steering_scale = assets.geometry.steering_scale;
    vehicle.gravity_scale = assets.geometry.gravity_scale;
    vehicle.grip_scale = assets.geometry.grip_scale;
    let item = collision_item(shape, &start, world, state)?;
    vehicle.item = Some(item);
    let params = BodyParams {
        model: assets.body.clone(),
        item: Some(item),
        forces: ForceModels::Vehicle,
        ..Default::default()
    };
    let handle = dyna.register(
        params,
        Iso::new(gearbox_engine::Mat3::from_quat(vehicle.quat), vehicle.pos),
        &mut DynaCollision { world, state, manager: start.collision_manager },
    );
    vehicle.body = handle.0;
    vehicle.body_enabled = 1;
    dyna.body_mut(handle).record.coefficient = vehicle.gravity;
    for (index, wheel) in assets.geometry.wheels.iter().enumerate() {
        vehicle.wheel_poses[index].transform = Iso::new(gearbox_engine::Mat3::IDENTITY, wheel.anchor);
    }
    vehicle.world_inverse_inertia = spawn::post_product(
        spawn::transpose_product(
            gearbox_engine::Mat3::from_quat(vehicle.quat),
            assets.geometry.local_inverse_inertia,
        ),
        gearbox_engine::Mat3::from_quat(vehicle.quat),
    );
    Ok(vehicle)
}

fn collision_item(
    shape: ShapeId,
    start: &VehicleStart,
    world: &mut World,
    state: &mut State,
) -> Result<DynamicItemId> {
    let result = DynamicItemId(state.dynamic_item.len() as u32);
    let item = DynamicItem {
        class: start.class,
        kind: 2,
        layer: VEHICLE_LAYER,
        shape,
        iso: Iso::new(
            gearbox_engine::Mat3::from_quat(gearbox_engine::Quat::from_mat3(&start.pose.rot)),
            start.pose.pos,
        ),
        quat: gearbox_engine::Quat::from_mat3(&start.pose.rot),
        pos: start.pose.pos,
        id: (world.statics.items.len() + state.dynamic_item.len()) as u32,
        entry_start: NO_ENTRY,
        receives_triggers: true,
        ..Default::default()
    };
    state.dynamic_item.push(item);
    let manager = &mut state.collision_world[start.collision_manager];
    manager.discrete_items.push(result);
    Ok(result)
}

pub fn spawn_vehicle(
    assets: &CarAssets,
    start: VehicleStart,
    world: &mut World,
    state: &mut State,
    dyna: &mut Dyna,
    visuals: &mut impl spawn::Visuals,
) -> Result<(Vehicle, spawn::EffectRing)> {
    let pose = start.pose;
    let mut vehicle = new_vehicle(assets, start, world, state, dyna)?;
    let mut ring = spawn::EffectRing::default();
    spawn::respawn(
        &mut vehicle,
        &assets.spawn,
        dyna,
        pose,
        spawn::Reset::FULL,
        &mut spawn::Services { world, state, visuals, ring: &mut ring },
    );
    Ok((vehicle, ring))
}
