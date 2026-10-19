use super::Vehicle;
use crate::Vec3;

pub(super) fn clear_previous_controls(vehicle: &mut Vehicle) {
    let flags = vehicle.previous_controls.control_word;
    vehicle.previous_controls = crate::vehicle::VehicleInputs::default();
    vehicle.previous_controls.control_word = flags;
}

pub(super) fn clear_current_controls(vehicle: &mut Vehicle) {
    let flags = vehicle.controls.control_word;
    vehicle.controls = crate::vehicle::VehicleInputs::default();
    vehicle.controls.control_word = flags;
}

pub(super) fn clear_contact_history(vehicle: &mut Vehicle) {
    vehicle.body_impact_count = 0;
    vehicle.body_impact_point_x = 0.0;
    vehicle.body_impact_point_y = 0.0;
    vehicle.body_impact_point_z = 0.0;
    vehicle.body_impact_normal_x = 0.0;
    vehicle.body_impact_normal_y = 0.0;
    vehicle.body_impact_normal_z = 0.0;
    vehicle.wheel_impact_count = 0;
    vehicle.wheel_impact_point_x = 0.0;
    vehicle.wheel_impact_point_y = 0.0;
    vehicle.wheel_impact_point_z = 0.0;
    vehicle.wheel_impact_normal_x = 0.0;
    vehicle.wheel_impact_normal_y = 0.0;
    vehicle.wheel_impact_normal_z = 0.0;
    vehicle.wheel_impact_gripping = 0;
    vehicle.free_wheel_impact_count = 0;
    vehicle.free_wheel_impact_point_x = 0.0;
    vehicle.free_wheel_impact_point_y = 0.0;
    vehicle.free_wheel_impact_point_z = 0.0;
    vehicle.free_wheel_impact_normal_x = 0.0;
    vehicle.free_wheel_impact_normal_y = 0.0;
    vehicle.free_wheel_impact_normal_z = 0.0;
    vehicle.free_wheel_impact_gripping = 0;
    vehicle.body_impact_peak = 0.0;
    vehicle.wheel_impact_peak = 0.0;
    vehicle.free_wheel_impact_peak = 0.0;
    vehicle.impact_flags = 0;
    vehicle.body_impact_material = 0;
    vehicle.wheel_impact_material = 0;
    vehicle.free_wheel_impact_material = 0;
    vehicle.body_impact_last_event = 0;
    vehicle.wheel_impact_last_event = 0;
    vehicle.free_wheel_impact_last_event = 0;
    vehicle.smoothed_pedal = 0.0;
    vehicle.recent_events[0] = 0;
    vehicle.recent_events[1] = 0;
    vehicle.recent_events[2] = 0;
    vehicle.recent_events[3] = 0;
    vehicle.recent_events[4] = 0;
    vehicle.forward_spring_value = 0.0;
    vehicle.forward_spring_target = 0.0;
    vehicle.forward_spring_velocity = 0.0;
    vehicle.lateral_spring_value = 0.0;
    vehicle.lateral_spring_target = 0.0;
    vehicle.lateral_spring_velocity = 0.0;
    vehicle.previous_body_velocity = Vec3::ZERO;
    vehicle.hard_impact = 0;
    vehicle.bump_frequency = 0.0;
    vehicle.bump_amplitude = 0.0;
}

pub(super) fn clear_full_contact_history(vehicle: &mut Vehicle) {
    vehicle.surface_layer = 0.0;
    vehicle.wheel_layers[0] = 0.0;
    vehicle.wheel_layers[1] = 0.0;
    vehicle.wheel_layers[2] = 0.0;
    vehicle.wheel_layers[3] = 0.0;
    vehicle.damage_flags[0] = 0;
    vehicle.damage_flags[1] = 0;
    vehicle.damage_flags[2] = 0;
    vehicle.damage_flags[3] = 0;
    vehicle.damage_flags[4] = 0;
    vehicle.detach_delays[0] = 0;
    vehicle.detach_delays[1] = 0;
    vehicle.detach_delays[2] = 0;
    vehicle.detach_delays[3] = 0;
    vehicle.damage_states[0] = 0;
    vehicle.damage_states[1] = 0;
    vehicle.damage_states[2] = 0;
    vehicle.damage_states[3] = 0;
    vehicle.damage_states[4] = 0;
    vehicle.damage_states[5] = 0;
    vehicle.damage_states[6] = 0;
    vehicle.damage_states[7] = 0;
    vehicle.damage_states[8] = 0;
    vehicle.damage_states[9] = 0;
    vehicle.damage_states[10] = 0;
    vehicle.damage_states[11] = 0;
    vehicle.damage_states[12] = 0;
    vehicle.damage_states[13] = 0;
    vehicle.damage_states[14] = 0;
    vehicle.damage_states[15] = 0;
    vehicle.detached_parts[0] = 0;
    vehicle.detached_parts[1] = 0;
    vehicle.detached_parts[2] = 0;
    vehicle.detached_parts[3] = 0;
}

pub(super) fn clear_reset_state(vehicle: &mut Vehicle) {
    vehicle.pending_event = 0;
    vehicle.post_force_speed_scale = 0.0;
    vehicle.slow_motion_wheel_contact = None;
    vehicle.slow_motion_body_contact = None;
    vehicle.boost_contact = None;
    vehicle.last_effect_contact = None;
    vehicle.support_contact_count = 0;
    vehicle.support_linear_x = 0.0;
    vehicle.support_linear_y = 0.0;
    vehicle.support_linear_z = 0.0;
    vehicle.support_angular_x = 0.0;
    vehicle.support_angular_y = 0.0;
    vehicle.support_angular_z = 0.0;
}
