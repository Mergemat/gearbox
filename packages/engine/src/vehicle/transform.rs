use super::{Vehicle, spawn, vis};
use crate::{Iso, Mat3, Quat, Vec3, dyna::BodyHandle, session::Session};

impl Session {
    pub(crate) fn transform_car(&mut self, now: u32) {
        let pending = std::mem::take(&mut self.vehicle.pending_car_changes);
        if let Some(choice) = pending.into_iter().find(|&choice| choice != self.vehicle.model_choice) {
            self.change_car(choice, now);
        }
    }

    pub(crate) fn select_car(&mut self, choice: u8) {
        if choice == self.vehicle.model_choice {
            return;
        }
        let model = self.config.model(choice);
        let compound =
            self.vehicle.shape.and_then(|shape| shape.compound()).expect("vehicle collision compound");
        self.state.compound_instance[compound.index()] = model.collision.clone();
        if self.vehicle.body != u32::MAX {
            let body = self.dyna.body_mut(BodyHandle(self.vehicle.body));
            body.model = model.body.clone();
            body.inertia.diagonal = body.model.inverse_inertia();
        }
        let mut shapes = [u32::MAX; 4];
        for (shape, wheel) in shapes.iter_mut().zip(&model.spawn.wheels) {
            *shape = wheel.shape;
        }
        [
            self.vehicle.wheel_transform_0,
            self.vehicle.wheel_transform_1,
            self.vehicle.wheel_transform_2,
            self.vehicle.wheel_transform_3,
        ] = shapes;
        self.vehicle.model_choice = choice;
    }

    fn change_car(&mut self, choice: u8, now: u32) {
        if choice == self.vehicle.model_choice {
            return;
        }
        let tuning = &self.config.model(choice).assets.tuning;
        let old_tuning = &self.config.model(self.vehicle.model_choice).assets.tuning;
        let height = (tuning.transform_suspension_length - tuning.transform_height_offset)
            - (old_tuning.transform_suspension_length - old_tuning.transform_height_offset);
        let rotation = Mat3::from_quat(self.vehicle.quat);
        let pose =
            Iso::new(rotation, self.vehicle.pos + rotation.transform_y_first(Vec3::new(0.0, height, 0.0)));
        self.select_car(choice);
        let model = self.config.model(choice);
        let tuning = &model.assets.tuning;
        let previous = std::mem::take(&mut self.vehicle);
        let mut vehicle = spawn::constructed(previous.key);
        vehicle.body_enabled = previous.body_enabled;
        vehicle.item = previous.item;
        vehicle.shape = previous.shape;
        vehicle.wheel_count = 4;
        vehicle.model_choice = choice;
        vehicle.tag = previous.tag;
        let mut visuals = vis::Refresh {
            config: vis::Config {
                tuning,
                geometry: &model.assets.geometry,
                tick: now,
                recent_event_duration: self.config.recent_event_duration,
            },
        };
        spawn::respawn(
            &mut vehicle,
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
        vehicle.body = previous.body;
        vehicle.transfer_motion(&previous);
        vehicle.pos = pose.pos;
        vehicle.quat = Quat::from_mat3(&pose.rot);
        vehicle.control_flags &= !15;
        vehicle.pivot_state = 0;
        vehicle.fixed_wheel_spin = 0;
        for wheel in &mut vehicle.wheels {
            wheel.suspension_length = tuning.transform_suspension_length;
        }
        if vehicle.surface_effect == 0 {
            vehicle.surface_effect = choice + crate::gameplay::VEHICLE_TRANSFORM_RESET;
        }
        vis::vehicle_vis(&mut vehicle, &self.dyna, &visuals.config, 1);
        self.vehicle = vehicle;
    }
}

impl Vehicle {
    fn transfer_motion(&mut self, previous: &Self) {
        self.copy_state(previous);
        self.visual[0] = previous.visual[0].clone();
        self.visual[0].key = self.key;
        self.visual[0].state = self.model_choice;
    }

    pub(crate) fn copy_state(&mut self, previous: &Self) {
        copy_fields! { self, previous, [
            alternate_forward, controls, previous_controls, control_flags, race_start, frozen_until,
            quat, pos, vel, ang_vel, world_inverse_inertia, gravity, slow_start_begin, slow_start_end,
            slow_start_flags, extra_linear_velocity, extra_angular_velocity, wheel_meter_enabled,
            wheel_meter, wheel_meter_sum, wheel_meter_count, effect_flags, forced_speed, reactor_begin,
            level_power_scale, slow_motion_until, state_sequence, respawn_sequence, angular_freeze,
            in_water, slip_armed, reactor_ready, water_reentry_until, water_exit_until, immersion,
            boost_level, boost_maximum_level, boost_last_contact, boost_begin, ground_pull_begin,
            ground_pull_duration_ms, ground_roll_begin, turbo_active, turbo_begin, turbo_end,
            turbo_last_contact, turbo_thrust_scale, turbo_duration_ms, turbo_fade_ms, turbo_phase,
            timed_turbo_contact, reactor_turbo_begin, turbo_contact, body_contact, last_slip_armed,
            last_body_contact, angular_control_hold_time, held_pitch_rate, held_yaw_rate,
            force_velocity_lateral, force_velocity_vertical, force_velocity_forward, steering,
            roll_stabilization_active, roll_stabilization_released, roll_stabilization_start,
            adhesion_contact_time, adhesion_class, adhesion_normal_x, adhesion_normal_y,
            adhesion_normal_z, pivot_state, force_feedback_lateral, force_feedback_vertical,
            force_feedback_forward, pivot_center_x, pivot_center_y, pivot_center_z,
            pivot_support_radius, pivot_radius, pivot_arm_begin, pivot_retire_begin, pivot_axis_x,
            pivot_axis_y, pivot_axis_z, pivot_steering_sign, pivot_blocked, fixed_wheel_spin,
            grip_meter, special_grip_blend_begin, special_grip_blend_active, special_grip_blend,
            thrust_boost_until, thrust_boost_elapsed, drift_thrust_begin, drift_thrust_scale,
            drift_release_begin, drift_yaw_state, drift_direction, engine_flags, rpm, gear, rpm_boost,
            shift_remaining, gearbox_state, shift_rpm_rate, reversing, target_rpm, cached_gear,
            impact_thrust_until, contact_impact_thrust_until, impact_yaw_change, impact_yaw_begin,
            drivetrain_flags, drivetrain_state, model_visual_0, model_visual_1, model_visual_2,
            model_visual_3, spin_up_duration, spin_up_torque, spin_up_begin, last_roof_contact,
            engine_meter, front_spin, rear_spin, idle_position_x, idle_position_y, idle_position_z,
            suspension_requires_all_contacts, steer_limit, wear, thrust_scale, steering_scale, gravity_scale,
            signed_speed_control, grip_scale, fragility_offset, forced_speed_integer,
            pedal_speed_limit, forced_effect_flags, previous_level_power, previous_drift_direction,
            slow_start_forward_threshold,
        ] }
        self.brake_history = previous.brake_history.clone();
        self.wheels = previous.wheels.clone();
    }
}
