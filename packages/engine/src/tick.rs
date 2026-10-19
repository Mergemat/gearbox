use std::sync::Arc;
use crate::{
    arena, race,
    collision::{Geometry, DynaCollision},
    dyna::Tick,
    session::{Session, Controls},
    vehicle::{before, post, vis, step},
};

pub const DT: f32 = 0.010000001;

impl Session {
    pub(crate) fn tick(&mut self, now: u32, controls: Controls, seed: Option<u32>) {
        self.replay_inputs(controls, seed);
        let scene = self.scene_update(now);
        self.arena_begin(now);
        self.apply_controls(now, controls);
        self.before_dyna(now);
        let frames = self.vehicle_substeps(now);
        self.dyna_step(now);
        self.after_dyna();
        self.post(now);
        self.race_contacts(now, &frames);
        self.state.scene.insert(self.config.scene, scene);
    }

    fn replay_inputs(&mut self, controls: Controls, seed: Option<u32>) {
        if let Some(seed) = seed {
            for racer in &mut self.race.players {
                if racer.has_entity(self.vehicle.key) {
                    racer.player.replay_inputs(controls.steer, controls.gas, controls.brake, seed);
                }
            }
        }
    }

    fn scene_update(&mut self, now: u32) -> arena::Scene {
        let mut scene = self.state.scene.remove(self.config.scene);
        scene.update(now, &mut self.state, self.config.manager);
        scene
    }

    fn arena_begin(&mut self, now: u32) {
        self.vehicle.reset_step();
        let vehicles = std::slice::from_mut(&mut self.vehicle);
        arena::begin(&mut self.race, vehicles, self.config.options);
        arena::players_vehicles(&mut self.race, vehicles, now);
        arena::clear_vehicle_inputs(vehicles);
    }

    fn apply_controls(&mut self, now: u32, controls: Controls) {
        self.countdown(now);
        arena::car_phy_update(
            std::slice::from_mut(&mut self.vehicle),
            &self.config.grid,
        );
        self.vehicle.controls.gas = controls.gas;
        self.vehicle.controls.brake = controls.brake;
        self.vehicle.controls.steer = controls.steer;
    }

    fn before_dyna(&mut self, now: u32) {
        let mut scratch = before::Scratch::default();
        let options = before::Options {
            fast: false,
            recent_event_duration: self.config.recent_event_duration,
            zone_source: false,
            pending: 0,
            registration: None,
        };
        before::before_dyna(&mut self.step_context(now), &mut scratch, options);
        let vehicles = std::slice::from_mut(&mut self.vehicle);
        for racer in &self.race.players {
            arena::mute_seats(racer, vehicles);
        }
    }

    fn step_context(&mut self, now: u32) -> step::StepCtx<'_> {
        step::StepCtx {
            world: Arc::make_mut(&mut self.world),
            state: &mut self.state,
            manager: self.config.manager,
            dyna: &mut self.dyna,
            vehicle: &mut self.vehicle,
            assets: self.config.assets.view(),
            tick: now,
            dt: DT,
            ghost_contacts: 0,
            events: None,
        }
    }

    fn vehicle_substeps(&mut self, now: u32) -> Vec<step::CollisionFrame> {
        self.continuous();
        self.rebuild_entries();
        step::substeps(&mut self.step_context(now))
    }

    fn dyna_step(&mut self, now: u32) {
        self.dyna.step(
            Tick { ms: now, dt: DT },
            &mut DynaCollision {
                world: &self.world,
                state: &mut self.state,
                manager: self.config.manager,
            },
        );
    }

    fn after_dyna(&mut self) {
        crate::vehicle::maintenance::process_contact_points(0, 0, 0);
        self.continuous();
        arena::consume_surface_effects(&self.race, std::slice::from_mut(&mut self.vehicle));
    }

    fn post(&mut self, now: u32) {
        let mut context = post::Context {
            tuning: &self.config.assets.tuning,
            geometry: &self.config.assets.geometry,
            model: Some(&self.config.assets.post),
            dyna: &self.dyna,
            water: crate::water::WaterManager::new(&self.world.water),
            tick: now,
            dt: DT,
            rng: &mut self.rng,
            parts_count: 0,
            events: None,
        };
        post::post(&mut self.vehicle, &mut context, self.config.recent_event_duration, false);
    }

    fn countdown(&mut self, now: u32) {
        let mut visuals = vis::Refresh {
            config: vis::Config {
                tuning: &self.config.assets.tuning,
                geometry: &self.config.assets.geometry,
                tick: now,
                recent_event_duration: self.config.recent_event_duration,
            },
        };
        let mut respawn = step::Countdown {
            world: &self.world,
            state: &mut self.state,
            dyna: &mut self.dyna,
            assets: &self.config.spawn_assets,
            visuals: &mut visuals,
            effects: &mut self.effects,
            initial_pose: self.config.initial_pose,
            parent_pose: None,
            waypoints: &self.race.rules.waypoints,
        };
        for racer in &mut self.race.players {
            race::countdown::update(racer, &mut self.vehicle, self.config.rules_start, now, &mut respawn);
        }
    }

    fn continuous(&mut self) {
        self.state.collision_world[self.config.manager]
            .update_continuous(&mut self.state.dynamic_item);
    }

    fn rebuild_entries(&mut self) {
        let (entries, ranges) = self.state.collision_world[self.config.manager]
            .rebuild_entries(Geometry::new(&self.world, &self.state));
        self.state.collision_world[self.config.manager].shape_entries = 0..entries.len() as u32;
        self.state.shape_entry = entries;
        for (index, start, count) in ranges {
            let item = &mut self.state.dynamic_item[index as usize];
            item.entry_start = start;
            item.entry_count = count;
        }
    }

    fn race_contacts(&mut self, now: u32, frames: &[step::CollisionFrame]) {
        let geometry = Geometry::new(&self.world, &self.state);
        let mut raw = Vec::new();
        for frame in frames {
            for contact in &frame.output.triggers {
                if geometry.entry(contact.contact.source_entry()).tag != crate::static_world::NO_CHILD_TAG {
                    continue;
                }
                let Some(item) = contact.item.static_item() else {
                    continue;
                };
                raw.push(race::trigger::Contact {
                    item: item.0,
                    vehicle: crate::ids::VehicleId::new(0),
                    time: contact.time,
                });
            }
        }
        let mut contacts = race::Contacts::default();
        contacts.begin();
        contacts.flush(&raw);
        race::on_events(
            &mut self.race,
            &self.world.statics,
            std::slice::from_mut(&mut self.vehicle),
            now,
            &contacts.contacts,
        );
    }
}
