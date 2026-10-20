use crate::collision::{Geometry, ItemFlags, Lists, QueryFilter, Ray};
use crate::dyna::BodyHandle;
use crate::race::WaypointFlags;
use crate::vehicle::{spawn, vis};
use crate::{Iso, ItemId, Quat, Session, ShapeId, SharedShapeId, Vec3};

mod triangles;
pub use triangles::PhysicalTriangle;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaypointTrigger {
    pub waypoint: u32,
    pub kind: u32,
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StaticHit {
    pub point: Vec3,
    pub normal: Vec3,
    pub material: u16,
    pub item: u32,
    pub triangle: u32,
    pub fraction: f32,
}

impl Session {
    pub fn crossing_waypoint(&self, ordinal: usize) -> Option<u32> {
        let index = ordinal.checked_sub(1)?;
        let player = self.race.players.first()?;
        self.race.store.entries(player.player.race_run).nth(index).map(|entry| u32::from(entry.waypoint))
    }

    pub fn waypoint_order(&self, waypoint: u32) -> Option<u32> {
        self.race.rules.waypoints.get(waypoint as usize).map(|mark| mark.order)
    }

    pub fn waypoint_triggers(&self) -> Vec<WaypointTrigger> {
        use crate::static_world::Entity;
        let geometry = Geometry::new(&self.world, &self.state);
        let mut items = std::collections::BTreeMap::<u32, WaypointTrigger>::new();
        for surface in &self.world.statics.surfaces {
            let item = &self.world.statics.items[surface.item as usize];
            let Entity::Waypoint(Some(waypoint)) = item.entity else { continue };
            if !item.flags.contains(crate::collision::ItemEvents::CONTACT_TIME) {
                continue;
            }
            let flags = self.race.rules.waypoints[waypoint as usize].flags;
            let kind = if flags.contains(WaypointFlags::GOAL) {
                if flags.contains(WaypointFlags::SPAWN) { 4 } else { 1 }
            } else if flags.contains(WaypointFlags::CHECKPOINT) {
                2
            } else {
                0
            };
            let bounds = geometry.world_box(ShapeId::Shared(SharedShapeId(surface.shape)), surface.iso);
            let min = (bounds.center - bounds.half).to_array();
            let max = (bounds.center + bounds.half).to_array();
            items
                .entry(surface.item)
                .and_modify(|trigger| {
                    for k in 0..3 {
                        trigger.min[k] = trigger.min[k].min(min[k]);
                        trigger.max[k] = trigger.max[k].max(max[k]);
                    }
                })
                .or_insert(WaypointTrigger { waypoint, kind, min, max });
        }
        items.into_values().collect()
    }

    pub fn pick_static(&self, origin: Vec3, direction: Vec3) -> Option<StaticHit> {
        if !origin.to_array().iter().chain(&direction.to_array()).all(|v| v.is_finite())
            || direction == Vec3::ZERO
        {
            return None;
        }
        const WHEEL_RAY_MASK: u32 = 0x12_8002;
        let geometry = Geometry::new(&self.world, &self.state);
        let hit = self.state.collision_world[self.config.manager].raycast(
            geometry,
            Ray { origin, direction },
            QueryFilter {
                lists: Lists::SURFACES,
                layers: WHEEL_RAY_MASK,
                excluded_flags: ItemFlags::from_bits(WHEEL_RAY_MASK),
                ..Default::default()
            },
        )?;
        let item = geometry.entry(hit.entry).item.and_then(ItemId::static_item)?;
        Some(StaticHit {
            point: hit.point,
            normal: hit.normal,
            material: hit.material,
            item: self.world.statics.items[item.index()].id,
            triangle: hit.triangle,
            fraction: hit.fraction,
        })
    }

    pub fn place_car(&mut self, pose: Iso, velocity: Vec3, angular: Vec3, reset_contacts: bool) {
        let model = self.config.model(self.vehicle.model_choice);
        let handle = BodyHandle(self.vehicle.body);
        assert_ne!(handle.0, u32::MAX, "vehicle has no active body");
        let mut visuals = vis::Refresh {
            config: vis::Config {
                tuning: &model.assets.tuning,
                geometry: &model.assets.geometry,
                tick: self.state.scene[self.config.scene].time,
                recent_event_duration: self.config.recent_event_duration,
            },
        };
        if reset_contacts {
            spawn::respawn(
                &mut self.vehicle,
                &model.spawn,
                &mut self.dyna,
                pose,
                spawn::Reset::FULL,
                &mut spawn::Services {
                    world: &self.world,
                    state: &mut self.state,
                    visuals: &mut visuals,
                    ring: &mut self.effects,
                    events: None,
                },
            );
        } else {
            self.dyna.set_pose(handle, pose);
            self.vehicle.quat = Quat::from_mat3(&pose.rot);
            self.vehicle.pos = pose.pos;
            if let Some(id) = self.vehicle.item {
                let item = &mut self.state.dynamic_item[id.index()];
                item.iso = pose;
                item.pos = pose.pos;
                item.quat = self.vehicle.quat;
            }
            self.dyna.clear_acc(handle);
            self.vehicle.world_inverse_inertia = spawn::post_product(
                spawn::transpose_product(pose.rot, model.spawn.inverse_inertia),
                pose.rot,
            );
        }
        let body = &mut self.dyna.body_mut(handle).state;
        body.vel = velocity;
        body.ang_vel = angular;
        self.vehicle.vel = velocity;
        self.vehicle.ang_vel = angular;
        vis::vehicle_vis(&mut self.vehicle, &self.dyna, &visuals.config, 1);
        self.vehicle.visual[0] = self.vehicle.visual[1].clone();
    }
}
