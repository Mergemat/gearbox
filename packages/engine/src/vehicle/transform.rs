use super::{Vehicle, spawn, vis};
use crate::{Iso, Mat3, Quat, dyna::BodyHandle, session::Session};

impl Session {
    pub(crate) fn transform_car(&mut self, now: u32) {
        for choice in std::mem::take(&mut self.vehicle.pending_car_changes) {
            if choice == self.vehicle.model_choice {
                continue;
            }
            let model = self.config.model(choice);
            let previous_model = self.config.model(self.vehicle.model_choice);
            let tuning = &model.assets.tuning;
            let old_tuning = &previous_model.assets.tuning;
            let height = (tuning.transform_suspension_length - tuning.transform_height_offset)
                - (old_tuning.transform_suspension_length - old_tuning.transform_height_offset);
            let rotation = Mat3::from_quat(self.vehicle.quat);
            let rows = rotation.rows;
            let mut pose = Iso::new(rotation, self.vehicle.pos);
            pose.pos.x += (height * rows[0].y + rows[0].x * 0.0) + rows[0].z * 0.0;
            pose.pos.y += (height * rows[1].y + rows[1].x * 0.0) + rows[1].z * 0.0;
            pose.pos.z += (height * rows[2].y + rows[2].x * 0.0) + rows[2].z * 0.0;
            let previous = std::mem::take(&mut self.vehicle);
            let shape = previous.shape.expect("vehicle collision shape");
            let compound = shape.compound().expect("vehicle collision compound");
            self.state.compound_instance[compound.index()] = model.collision.clone();
            let mut vehicle = spawn::constructed(previous.key);
            vehicle.body = previous.body;
            vehicle.body_enabled = 1;
            vehicle.item = previous.item;
            vehicle.shape = Some(shape);
            vehicle.wheel_count = 4;
            vehicle.model_choice = choice;
            vehicle.tag = previous.tag;
            let handle = BodyHandle(vehicle.body);
            {
                let body = self.dyna.body_mut(handle);
                body.model = model.body.clone();
                body.inertia.diagonal = body.model.inverse_inertia();
            }
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
                },
            );
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
                vehicle.surface_effect = choice + 20;
            }
            vis::vehicle_vis(&mut vehicle, &self.dyna, &visuals.config, 1);
            self.vehicle = vehicle;
        }
    }
}

impl Vehicle {
    fn transfer_motion(&mut self, previous: &Self) {
        self.alternate_forward = previous.alternate_forward;
        self.controls = previous.controls;
        self.previous_controls = previous.previous_controls;
        self.control_flags = previous.control_flags;
        self.race_start = previous.race_start;
        self.frozen_until = previous.frozen_until;
        self.quat = previous.quat;
        self.pos = previous.pos;
        self.vel = previous.vel;
        self.ang_vel = previous.ang_vel;
        self.world_inverse_inertia = previous.world_inverse_inertia;
        self.gravity = previous.gravity;
        self.slow_start_begin = previous.slow_start_begin;
        self.slow_start_end = previous.slow_start_end;
        self.slow_start_flags = previous.slow_start_flags;
        self.extra_linear_velocity = previous.extra_linear_velocity;
        self.extra_angular_velocity = previous.extra_angular_velocity;
        self.wheel_meter_enabled = previous.wheel_meter_enabled;
        self.wheel_meter = previous.wheel_meter;
        self.wheel_meter_sum = previous.wheel_meter_sum;
        self.wheel_meter_count = previous.wheel_meter_count;
        self.effect_flags = previous.effect_flags;
        self.forced_speed = previous.forced_speed;
        self.reactor_begin = previous.reactor_begin;
        self.level_power_scale = previous.level_power_scale;
        self.slow_motion_until = previous.slow_motion_until;
        self.state_sequence = previous.state_sequence;
        self.respawn_sequence = previous.respawn_sequence;
        self.angular_freeze = previous.angular_freeze;
        self.in_water = previous.in_water;
        self.slip_armed = previous.slip_armed;
        self.reactor_ready = previous.reactor_ready;
        self.water_reentry_until = previous.water_reentry_until;
        self.water_exit_until = previous.water_exit_until;
        self.immersion = previous.immersion;
        self.boost_level = previous.boost_level;
        self.boost_maximum_level = previous.boost_maximum_level;
        self.boost_last_contact = previous.boost_last_contact;
        self.boost_begin = previous.boost_begin;
        self.ground_pull_begin = previous.ground_pull_begin;
        self.ground_pull_duration_ms = previous.ground_pull_duration_ms;
        self.ground_roll_begin = previous.ground_roll_begin;
        self.turbo_active = previous.turbo_active;
        self.turbo_begin = previous.turbo_begin;
        self.turbo_end = previous.turbo_end;
        self.turbo_last_contact = previous.turbo_last_contact;
        self.turbo_thrust_scale = previous.turbo_thrust_scale;
        self.turbo_duration_ms = previous.turbo_duration_ms;
        self.turbo_fade_ms = previous.turbo_fade_ms;
        self.turbo_phase = previous.turbo_phase;
        self.timed_turbo_contact = previous.timed_turbo_contact;
        self.reactor_turbo_begin = previous.reactor_turbo_begin;
        self.turbo_contact = previous.turbo_contact;
        self.body_contact = previous.body_contact;
        self.last_slip_armed = previous.last_slip_armed;
        self.last_body_contact = previous.last_body_contact;
        self.angular_control_hold_time = previous.angular_control_hold_time;
        self.held_pitch_rate = previous.held_pitch_rate;
        self.held_yaw_rate = previous.held_yaw_rate;
        self.force_velocity_lateral = previous.force_velocity_lateral;
        self.force_velocity_vertical = previous.force_velocity_vertical;
        self.force_velocity_forward = previous.force_velocity_forward;
        self.steering = previous.steering;
        self.roll_stabilization_active = previous.roll_stabilization_active;
        self.roll_stabilization_released = previous.roll_stabilization_released;
        self.roll_stabilization_start = previous.roll_stabilization_start;
        self.adhesion_contact_time = previous.adhesion_contact_time;
        self.adhesion_class = previous.adhesion_class;
        self.adhesion_normal_x = previous.adhesion_normal_x;
        self.adhesion_normal_y = previous.adhesion_normal_y;
        self.adhesion_normal_z = previous.adhesion_normal_z;
        self.pivot_state = previous.pivot_state;
        self.force_feedback_lateral = previous.force_feedback_lateral;
        self.force_feedback_vertical = previous.force_feedback_vertical;
        self.force_feedback_forward = previous.force_feedback_forward;
        self.pivot_center_x = previous.pivot_center_x;
        self.pivot_center_y = previous.pivot_center_y;
        self.pivot_center_z = previous.pivot_center_z;
        self.pivot_support_radius = previous.pivot_support_radius;
        self.pivot_radius = previous.pivot_radius;
        self.pivot_arm_begin = previous.pivot_arm_begin;
        self.pivot_retire_begin = previous.pivot_retire_begin;
        self.pivot_axis_x = previous.pivot_axis_x;
        self.pivot_axis_y = previous.pivot_axis_y;
        self.pivot_axis_z = previous.pivot_axis_z;
        self.pivot_steering_sign = previous.pivot_steering_sign;
        self.pivot_blocked = previous.pivot_blocked;
        self.fixed_wheel_spin = previous.fixed_wheel_spin;
        self.grip_meter = previous.grip_meter;
        self.special_grip_blend_begin = previous.special_grip_blend_begin;
        self.special_grip_blend_active = previous.special_grip_blend_active;
        self.special_grip_blend = previous.special_grip_blend;
        self.thrust_boost_until = previous.thrust_boost_until;
        self.thrust_boost_elapsed = previous.thrust_boost_elapsed;
        self.drift_thrust_begin = previous.drift_thrust_begin;
        self.drift_thrust_scale = previous.drift_thrust_scale;
        self.drift_release_begin = previous.drift_release_begin;
        self.drift_yaw_state = previous.drift_yaw_state;
        self.drift_direction = previous.drift_direction;
        self.brake_history = previous.brake_history.clone();
        self.engine_flags = previous.engine_flags;
        self.rpm = previous.rpm;
        self.gear = previous.gear;
        self.rpm_boost = previous.rpm_boost;
        self.shift_remaining = previous.shift_remaining;
        self.gearbox_state = previous.gearbox_state;
        self.shift_rpm_rate = previous.shift_rpm_rate;
        self.reversing = previous.reversing;
        self.target_rpm = previous.target_rpm;
        self.cached_gear = previous.cached_gear;
        self.impact_thrust_until = previous.impact_thrust_until;
        self.contact_impact_thrust_until = previous.contact_impact_thrust_until;
        self.impact_yaw_change = previous.impact_yaw_change;
        self.impact_yaw_begin = previous.impact_yaw_begin;
        self.drivetrain_flags = previous.drivetrain_flags;
        self.drivetrain_state = previous.drivetrain_state;
        self.model_visual_0 = previous.model_visual_0;
        self.model_visual_1 = previous.model_visual_1;
        self.model_visual_2 = previous.model_visual_2;
        self.model_visual_3 = previous.model_visual_3;
        self.spin_up_duration = previous.spin_up_duration;
        self.spin_up_torque = previous.spin_up_torque;
        self.spin_up_begin = previous.spin_up_begin;
        self.last_roof_contact = previous.last_roof_contact;
        self.engine_meter = previous.engine_meter;
        self.front_spin = previous.front_spin;
        self.rear_spin = previous.rear_spin;
        self.idle_position_x = previous.idle_position_x;
        self.idle_position_y = previous.idle_position_y;
        self.idle_position_z = previous.idle_position_z;
        self.suspension_requires_all_contacts = previous.suspension_requires_all_contacts;
        self.wear = previous.wear;
        self.thrust_scale = previous.thrust_scale;
        self.steering_scale = previous.steering_scale;
        self.gravity_scale = previous.gravity_scale;
        self.signed_speed_control = previous.signed_speed_control;
        self.grip_scale = previous.grip_scale;
        self.fragility_offset = previous.fragility_offset;
        self.forced_speed_integer = previous.forced_speed_integer;
        self.pedal_speed_limit = previous.pedal_speed_limit;
        self.forced_effect_flags = previous.forced_effect_flags;
        self.wheels = previous.wheels.clone();
        self.previous_level_power = previous.previous_level_power;
        self.previous_drift_direction = previous.previous_drift_direction;
        self.slow_start_forward_threshold = previous.slow_start_forward_threshold;
        self.visual[0] = previous.visual[0].clone();
        self.visual[0].key = self.key;
        self.visual[0].state = self.model_choice;
    }
}
