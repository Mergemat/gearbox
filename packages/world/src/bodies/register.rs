use super::Built;
use anyhow::{Result, ensure};
use gearbox_engine::collision::{DynaCollision, DynamicItem};
use gearbox_engine::dyna::{
    BodyHandle, BodyParams, Dyna,
    kinematic::{KinematicConstraint, Motion},
};
use gearbox_engine::{DynamicItemId, Iso, Quat, ShapeId, SharedShapeId, State, World};

const MAP_BODY_CLASS: u32 = 0x3009_8000;
const DISCRETE_ITEM: u8 = 2;
const MAP_BODY_LAYER: u8 = 21;
const NO_BODY: u32 = u32::MAX;

impl Built {
    pub fn register(self, world: &mut World, state: &mut State, dyna: &mut Dyna) -> Result<Vec<BodyHandle>> {
        ensure!(
            dyna.bodies.is_empty() && state.dynamic_item.is_empty(),
            "map bodies must register before vehicles"
        );
        if state.collision_world.is_empty() {
            state.collision_world.push(Default::default());
        }
        ensure!(state.collision_world.len() == 1, "multiple collision managers");
        world.statics = self.statics;
        world.gate_record = self.gates;
        let mut motions: Vec<(u32, u32)> = Vec::new();
        let mut handles = Vec::new();
        for body in &self.instances {
            ensure!((body.shape as usize) < world.statics.shapes.len(), "missing kinematic shape");
            super::model::validate_bounds(&world.statics.shapes, body.shape)?;
            ensure!((body.animation as usize) < self.animations.len(), "missing kinematic animation");
            let key = (body.animation, body.phase.to_bits());
            let animation = if let Some(index) = motions.iter().position(|&entry| entry == key) {
                index as u32
            } else {
                let mut model = self.animations[body.animation as usize].clone();
                let current = model.transform(0, body.phase);
                let past = model.transform(-1_000_000, body.phase);
                dyna.animations.push(Motion { model, phase: body.phase, current, past });
                motions.push(key);
                motions.len() as u32 - 1
            };
            let pose = body.placement * dyna.animations[animation as usize].current;
            let item = create_item(world, state, body.shape, pose);
            let params = BodyParams {
                model: body.model.clone(),
                item: Some(item),
                fixed: true,
                ..BodyParams::default()
            };
            let handle = dyna.register(params, pose, &mut DynaCollision { world, state, manager: 0 });
            dyna.constraints.push(KinematicConstraint { body: handle, placement: body.placement, animation });
            handles.push(handle);
        }
        Ok(handles)
    }
}

fn create_item(world: &mut World, state: &mut State, shape: u32, pose: Iso) -> DynamicItemId {
    let index = state.dynamic_item.len() as u32;
    state.dynamic_item.push(DynamicItem {
        class: MAP_BODY_CLASS,
        kind: DISCRETE_ITEM,
        layer: MAP_BODY_LAYER,
        id: world.statics.items.len() as u32 + index,
        shape: ShapeId::Shared(SharedShapeId(shape)),
        iso: pose,
        entry_start: u16::MAX,
        quat: Quat::from_mat3(&pose.rot),
        pos: pose.pos,
        body: NO_BODY,
        ..Default::default()
    });
    let item = DynamicItemId(index);
    state.collision_world[0].discrete_items.push(item);
    item
}
