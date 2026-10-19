use crate::Vec3;
use super::{VehicleBody, ForceRays};
use super::super::{Tuning, Vehicle, model::ForceGeometry};

pub struct ForceCtx<'a> {
    pub vehicle: &'a mut Vehicle,
    pub tuning: &'a Tuning,
    pub geometry: &'a ForceGeometry,
    pub body: VehicleBody<'a>,
    pub water: crate::water::WaterManager<'a>,
    pub fluid_tuning: &'a crate::water::drag::FluidTuning,
    pub collision: &'a ForceRays<'a>,
    pub wheel_transforms: &'a mut [crate::Iso],
    pub events: Option<&'a mut Vec<crate::arena::VehicleEvent>>,
    pub tick: u32,
    pub time_ns: i64,
    pub dt: f32,
    pub local_velocity: Vec3,
    pub local_angular_velocity: Vec3,
    pub local_force: Vec3,
}
