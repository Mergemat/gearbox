use crate::{arena, collision, race};

#[derive(Clone, Debug, Default)]
pub struct World {
    pub statics: crate::static_world::StaticWorld,
    pub water: crate::world_water::Water,
    pub gate_record: Vec<race::GateRecord>,
    pub box_shape: Vec<collision::BoxShape>,
}

#[derive(Clone, Debug, Default)]
pub struct State {
    pub compound_instance: Vec<collision::CompoundInstance>,
    pub scene: Vec<arena::Scene>,
    pub collision_world: Vec<collision::CollisionWorld>,
    pub dynamic_item: Vec<collision::DynamicItem>,
    pub continuous_entry: Vec<collision::ContinuousEntry>,
    pub shape_entry: Vec<collision::ShapeEntry>,
    pub sphere_shape: Vec<collision::SphereShape>,
    pub ellipsoid_shape: Vec<collision::EllipsoidShape>,
}
