use crate::{DynamicItemId, Vec3};

pub struct ForceRays<'a> {
    pub geometry: crate::shape::Geometry<'a>,
    pub manager: &'a crate::collision::CollisionWorld,
    pub vehicle_item: Option<DynamicItemId>,
}

impl ForceRays<'_> {
    pub fn raycast(&self, origin: Vec3, direction: Vec3) -> Option<ForceRayHit> {
        use crate::collision::{ItemFlags, QueryFilter, Ray};
        let filter = QueryFilter {
            layers: 0x12_8002,
            excluded_flags: ItemFlags::NONE,
            ignored: [self.vehicle_item.map(Into::into), None],
            ..QueryFilter::default()
        };
        self.manager.raycast(self.geometry, Ray { origin, direction }, filter).map(|hit| {
            ForceRayHit { fraction: hit.fraction, normal: hit.normal, surface: hit.material as u8 }
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForceRayHit {
    pub fraction: f32,
    pub normal: Vec3,
    pub surface: u8,
}
