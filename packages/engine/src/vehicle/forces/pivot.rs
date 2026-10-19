use super::common::PivotState;
use super::{ForceCtx, surface::SurfaceResponse};
use super::common::{EPS, KMH, PI, TINY, PEDAL_THRESHOLD, PIVOT_UP_THRESHOLD, sign_one};
use crate::math::{angle_approx, fast_exp2};

const LOG2_E: f32 = 1.442695;

struct PivotMotion {
    radial: crate::Vec3,
    tangent: crate::Vec3,
    lateral_speed: f32,
    distance: f32,
}

pub fn pivot_turn(ctx: &mut ForceCtx<'_>, steer: f32, surface: &SurfaceResponse) {
    let state = ctx.body.state();
    if ctx.tuning.low_grip_threshold >= ctx.vehicle.grip || ctx.vehicle.special_material_present != 0 {
        return;
    }
    if !ctx.vehicle.effect_mode_is_free() || surface.pivot_enabled == 0 {
        return;
    }
    let tick = ctx.tick;
    let brake = ctx.vehicle.controls.brake > PEDAL_THRESHOLD;
    let gas_input = ctx.vehicle.controls.gas;
    let gas = gas_input > PEDAL_THRESHOLD;
    if brake && PEDAL_THRESHOLD > gas_input && ctx.vehicle.pivot_state() == PivotState::Armed {
        ctx.vehicle.pivot_retire_begin = tick;
        ctx.vehicle.set_pivot_state(PivotState::Retiring);
    }
    let vertical_alignment = state.rot.rows[1].y;
    if !(gas && brake) {
        return;
    }
    let speed = ctx.local_velocity.z;
    if ctx.tuning.pivot_minimum_speed > speed && vertical_alignment > PIVOT_UP_THRESHOLD {
        ctx.vehicle.set_pivot_state(PivotState::Armed);
        ctx.vehicle.fixed_wheel_spin = 1;
        ctx.vehicle.pivot_arm_begin = tick;
    }
    let speed = ctx.local_velocity.z;
    if !(ctx.tuning.pivot_maximum_speed > speed && speed > ctx.tuning.pivot_minimum_speed) {
        return;
    }
    if EPS > steer.abs() {
        return;
    }
    construct_pivot(ctx, steer, state);
}

pub fn pivot_forces(ctx: &mut ForceCtx<'_>, active: bool, steer: f32, dt: f32) {
    let state = ctx.body.state();
    let velocity = [ctx.local_velocity.x, ctx.local_velocity.y, ctx.local_velocity.z];
    let blocked = ctx.tuning.low_grip_threshold >= ctx.vehicle.grip
        || ctx.vehicle.special_material_present != 0 || dt > EPS;
    let eligible = !(PEDAL_THRESHOLD > ctx.vehicle.controls.gas)
        && !(PEDAL_THRESHOLD > ctx.vehicle.controls.brake)
        && !(EPS > steer.abs())
        && sign_one(steer) == ctx.vehicle.pivot_steering_sign
        && ctx.vehicle.body_contact == 0
        && ctx.vehicle.pivot_blocked as i32 <= 0
        && active
        && ctx.vehicle.in_water == 0
        && ctx.vehicle.effect_mode_is_free()
        && !blocked;
    active_pivot_forces(ctx, steer, state, velocity, eligible);
    if ctx.vehicle.pivot_state() != PivotState::Turning {
        ctx.vehicle.set_pivot_state(PivotState::Armed);
        let tick = ctx.tick;
        ctx.vehicle.pivot_arm_begin = tick;
    }
}

fn construct_pivot(ctx: &mut ForceCtx<'_>, steer: f32, state: crate::dyna::BodyState) {
    ctx.vehicle.set_pivot_state(PivotState::Turning);
    let steering_sign = sign_one(steer);
    ctx.vehicle.pivot_steering_sign = steering_sign;

    let (normal_x, normal_y, normal_z) = initialize_axis(ctx);
    store_support_radius(ctx);
    let (velocity_x, velocity_y, velocity_z) = (ctx.local_velocity.x, ctx.local_velocity.y,
        ctx.local_velocity.z);
    let mut tangent_x = (velocity_z * normal_y - velocity_y * normal_z) * steering_sign;
    let mut tangent_y = (velocity_x * normal_z - velocity_z * normal_x) * steering_sign;
    let mut tangent_z = (velocity_y * normal_x - velocity_x * normal_y) * steering_sign;
    let squared_length = (tangent_y * tangent_y + tangent_x * tangent_x) + tangent_z * tangent_z;
    if squared_length > TINY && f32::MAX > squared_length {
        let inv = 1.0 / squared_length.sqrt();
        tangent_x = inv * tangent_x;
        tangent_y = inv * tangent_y;
        tangent_z = inv * tangent_z;
    }
    let rotation = state.rot.to_array();
    let axis = crate::Vec3::new(
        (normal_y * rotation[1] + normal_x * rotation[0]) + normal_z * rotation[2],
        (normal_x * rotation[3] + normal_y * rotation[4]) + normal_z * rotation[5],
        (normal_x * rotation[6] + normal_y * rotation[7]) + normal_z * rotation[8],
    );
    ctx.vehicle.pivot_axis_x = axis.x;
    ctx.vehicle.pivot_axis_y = axis.y;
    ctx.vehicle.pivot_axis_z = axis.z;
    if PIVOT_UP_THRESHOLD > ctx.vehicle.pivot_axis_y {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
        ctx.vehicle.pivot_axis_x = 0.0;
        ctx.vehicle.pivot_axis_y = 1.0; ctx.vehicle.pivot_axis_z = 0.0;
    }
    let (a0, b0, c0) = (tangent_x * 0.0, tangent_y * 0.0, tangent_z * 0.0);
    let p1 = tangent_x - c0;
    let p2 = c0 - tangent_y;
    let p3_ = b0 - a0;
    let dot = (a0 + b0) + tangent_z;
    let distance = ((p1 * p1 + p2 * p2) + p3_ * p3_).sqrt();
    let mut angle = crate::math::angle_approx(distance, dot);
    if !((p2 * 0.0 + p1) + p3_ * 0.0 >= 0.0) {
        angle = -angle;
    }
    if !(steer >= 0.0) {
        angle = -angle;
    }
    if angle > ctx.tuning.pivot_positive_angle_limit || -ctx.tuning.pivot_negative_angle_limit > angle {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
        ctx.vehicle.fixed_wheel_spin = 0;
        return;
    }
    let transform = crate::Iso::new(state.rot, state.pos).to_array();
    let (center_x, center_y, center_z) = (ctx.geometry.center_of_mass.x,
        ctx.geometry.center_of_mass.y, ctx.geometry.center_of_mass.z);
    let radius = ctx.tuning.pivot_radius_by_speed.eval(velocity_z * KMH) + ctx.vehicle.pivot_support_radius;
    ctx.vehicle.pivot_radius = radius;
    let pivot_x = (((center_x * transform[0] + center_y * transform[1]) + center_z * transform[2])
        + transform[9]) + ((transform[0] * tangent_x + transform[1] * tangent_y) + transform[2]
        * tangent_z) * radius;
    ctx.vehicle.pivot_center_x = pivot_x;
    let pivot_y = (((center_x * transform[3] + center_y * transform[4]) + center_z * transform[5])
        + transform[10]) + ((transform[3] * tangent_x + transform[4] * tangent_y) + transform[5]
        * tangent_z) * radius;
    ctx.vehicle.pivot_center_y = pivot_y;
    let pivot_z = ((transform[6] * tangent_x + transform[7] * tangent_y) + transform[8] * tangent_z)
        * radius + (((center_x * transform[6] + center_y * transform[7]) + center_z * transform[8])
        + transform[11]);
    ctx.vehicle.pivot_center_z = pivot_z;
    let pivoting = ctx.vehicle.pivot_state() == PivotState::Turning;
    ctx.vehicle.fixed_wheel_spin = pivoting as u32;
}

fn initialize_axis(ctx: &mut ForceCtx<'_>) -> (f32, f32, f32) {
    ctx.vehicle.pivot_axis_y = 0.0; ctx.vehicle.pivot_axis_z = 0.0;
    ctx.vehicle.pivot_axis_x = 0.0;
    let (mut normal_x, mut normal_y, mut normal_z) = (0.0f32, 0.0f32, 0.0f32);
    for wheel in &ctx.vehicle.wheels {
        if wheel.contact != 0 {
            normal_x = normal_x + wheel.contact_normal_x;
            normal_y = normal_y + wheel.contact_normal_y;
            normal_z = normal_z + wheel.contact_normal_z;
        }
    }
    let squared_length = (normal_y * normal_y + normal_x * normal_x) + normal_z * normal_z;
    if squared_length > TINY && f32::MAX > squared_length {
        let inv = 1.0 / squared_length.sqrt();
        normal_z = normal_z * inv;
        normal_y = normal_y * inv;
        normal_x = normal_x * inv;
    } else {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
        (normal_x, normal_y, normal_z) = (0.0, 1.0, 0.0);
    }
    ctx.vehicle.pivot_axis_x = normal_x;
    ctx.vehicle.pivot_axis_y = normal_y;
    ctx.vehicle.pivot_axis_z = normal_z;
    (normal_x, normal_y, normal_z)
}

fn store_support_radius(ctx: &mut ForceCtx<'_>) {
    let bounds = ctx.geometry.water_box;
    let k = (bounds.half.y * 0.0 + bounds.half.x * 0.0) + bounds.half.z;
    let dz = (bounds.center.z - ctx.geometry.center_of_mass.z) + k;
    let k0 = k * 0.0;
    let dy = (bounds.center.y - ctx.geometry.center_of_mass.y) + k0;
    let dx = (bounds.center.x - ctx.geometry.center_of_mass.x) + k0;
    let dist = ((dy * dy + dx * dx) + dz * dz).sqrt();
    ctx.vehicle.pivot_support_radius = dist;
}

fn active_pivot_forces(
    ctx: &mut ForceCtx<'_>, steer: f32, state: crate::dyna::BodyState, velocity: [f32; 3], eligible: bool,
) {

    if !eligible {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
        return;
    }
    let sum = |axis: usize| {
        let normal = |index: usize| {
            let wheel = &ctx.vehicle.wheels[index];
            [wheel.contact_normal_x, wheel.contact_normal_y, wheel.contact_normal_z][axis]
        };
        (((normal(0) + 0.0) + normal(1)) + normal(2)) + normal(3)
    };
    let (mut normal_x, mut normal_y, mut normal_z) = (sum(0), sum(1), sum(2));
    let squared_length = (normal_x * normal_x + normal_y * normal_y) + normal_z * normal_z;
    if squared_length > TINY && f32::MAX > squared_length {
        let inv = 1.0 / squared_length.sqrt();
        (normal_x, normal_y, normal_z) = (normal_x * inv, normal_y * inv, normal_z * inv);
    } else {
        (normal_x, normal_y, normal_z) = (0.0, 1.0, 0.0);
    }
            let rotation = state.rot.to_array();
    let world_x = (normal_y * rotation[1] + normal_x * rotation[0]) + normal_z * rotation[2];
    let world_y = (normal_x * rotation[3] + normal_y * rotation[4]) + normal_z * rotation[5];
    let world_z = (normal_x * rotation[6] + normal_y * rotation[7]) + normal_z * rotation[8];
    let (held_x, held_y, held_z) = (ctx.vehicle.pivot_axis_x, ctx.vehicle.pivot_axis_y,
        ctx.vehicle.pivot_axis_z);
    let c0 = held_z * world_y - held_y * world_z;
    let c1 = held_x * world_z - held_z * world_x;
    let c2 = held_y * world_x - held_x * world_y;
    let l = ((c1 * c1 + c0 * c0) + c2 * c2).sqrt();
    let dot = (held_y * world_y + held_x * world_x) + held_z * world_z;
    if angle_approx(l, dot) > ctx.tuning.pivot_normal_angle_limit {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
        return;
    }
    let motion = radial_force(ctx, state, velocity, [normal_x, normal_y, normal_z]);
    let wheel_count = ctx.geometry.wheels.len();
    for index in 0..wheel_count {
        super::wheels::suspension_force(ctx, index);
        ctx.vehicle.wheels[index].slipping = 0;
    }
    let radius_response = ctx.tuning.pivot_lateral_force_by_radius.eval(motion.distance) / KMH;
    let force_scale = (-sign_one(steer) * radius_response - motion.lateral_speed)
        * ctx.tuning.pivot_lateral_stiffness;
    ctx.body.add_local_force(motion.tangent * force_scale);
    pivot_yaw(ctx, steer, &motion);
    let c3 = ctx.tuning.pivot_torque_by_side_speed.eval(velocity[0].abs() * KMH);
    let t3 = -sign_one(velocity[0]) * c3;
    ctx.body.add_local_torque(([t3 * 0.0, t3 * 0.0, t3]).into());
    let c4 = ctx.tuning.pivot_torque_by_forward_speed.eval(velocity[2] * KMH);
    ctx.body.add_local_torque(([c4, 0.0, 0.0]).into());
}

fn radial_force(
    ctx: &mut ForceCtx<'_>, state: crate::dyna::BodyState, velocity: [f32; 3], normal: [f32; 3],
) -> PivotMotion {
    let [normal_x, normal_y, normal_z] = normal;
    let inverse_rotation = state.rot.transpose().to_array();
    let negative = crate::Vec3::new(-state.pos.x, -state.pos.y, -state.pos.z);
    let inverse_transform = [inverse_rotation[0], inverse_rotation[1], inverse_rotation[2],
        inverse_rotation[3], inverse_rotation[4], inverse_rotation[5],
        inverse_rotation[6], inverse_rotation[7], inverse_rotation[8],
        negative.x * inverse_rotation[0] + negative.y * inverse_rotation[1] + negative.z
            * inverse_rotation[2],
        negative.x * inverse_rotation[3] + negative.y * inverse_rotation[4] + negative.z
            * inverse_rotation[5],
        negative.x * inverse_rotation[6] + negative.y * inverse_rotation[7] + negative.z
            * inverse_rotation[8]];
    let (pivot_x, pivot_y, pivot_z) = (ctx.vehicle.pivot_center_x, ctx.vehicle.pivot_center_y,
        ctx.vehicle.pivot_center_z);
    let offset_x = (((pivot_x * inverse_transform[0] + pivot_y * inverse_transform[1]) + pivot_z
        * inverse_transform[2]) + inverse_transform[9]) - ctx.geometry.center_of_mass.x;
    let offset_y = (((pivot_x * inverse_transform[3] + pivot_y * inverse_transform[4]) + pivot_z
        * inverse_transform[5]) + inverse_transform[10]) - ctx.geometry.center_of_mass.y;
    let offset_z = (((pivot_x * inverse_transform[6] + pivot_y * inverse_transform[7]) + pivot_z
        * inverse_transform[8]) + inverse_transform[11]) - ctx.geometry.center_of_mass.z;
    let squared_length = (offset_y * offset_y + offset_x * offset_x) + offset_z * offset_z;
    let distance = squared_length.sqrt();
    let (mut radial_x, mut radial_y, mut radial_z) = (offset_x, offset_y, offset_z);
    if squared_length > TINY && f32::MAX > squared_length {
        let inv = 1.0 / distance;
        radial_z = offset_z * inv;
        radial_x = offset_x * inv;
        radial_y = offset_y * inv;
    }
    let tangent_x = radial_z * normal_y - radial_y * normal_z;
    let tangent_y = radial_x * normal_z - radial_z * normal_x;
    let tangent_z = radial_y * normal_x - radial_x * normal_y;
    let lateral_speed = (tangent_y * velocity[1] + tangent_x * velocity[0]) + tangent_z * velocity[2];
    if lateral_speed.abs() > ctx.tuning.pivot_maximum_lateral_speed {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
    }
    if distance > ctx.vehicle.pivot_support_radius || ctx.tuning.pivot_minimum_radius > distance {
        let stretch = (distance - ctx.vehicle.pivot_radius) * ctx.tuning.pivot_radial_stiffness;
        let radial = ((radial_y * velocity[1] + radial_x * velocity[0]) + radial_z * velocity[2])
            * ctx.tuning.pivot_radial_damping;
        let exponential = fast_exp2((stretch - radial) * LOG2_E);
        let force = exponential * (lateral_speed * lateral_speed / distance);
        ctx.body.add_local_force(([force * radial_x, force * radial_y, force * radial_z]).into());
    } else {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
    }
    PivotMotion {
        radial: crate::Vec3::new(radial_x, radial_y, radial_z),
        tangent: crate::Vec3::new(tangent_x, tangent_y, tangent_z),
        lateral_speed,
        distance,
    }
}

fn pivot_yaw(ctx: &mut ForceCtx<'_>, steer: f32, motion: &PivotMotion) {
    let [radial_x, radial_y, radial_z] = motion.radial.to_array();
    let lateral_speed = motion.lateral_speed;
    let distance = motion.distance;
    let a = radial_x - radial_z * 0.0;
    let b = radial_z * 0.0 - radial_y;
    let cc = radial_y * 0.0 - radial_x * 0.0;
    let dt2 = (radial_x * 0.0 + radial_y * 0.0) + radial_z;
    let mut angle = angle_approx(((a * a + b * b) + cc * cc).sqrt(), dt2);
    if !((b * 0.0 + a) + cc * 0.0 >= 0.0) {
        angle = -angle;
    }
    let angle = angle / PI;
    let steering_sign = sign_one(steer);
    let in_range = crate::numeric::is_finite(angle)
        && !(-ctx.tuning.pivot_negative_angle_limit > steering_sign * angle * PI)
        && !(angle * steering_sign * PI > ctx.tuning.pivot_positive_angle_limit);
    if in_range {
        let yaw = ctx.local_angular_velocity.y;
        let pulled = if angle > 0.0 {
            0.0 > yaw
        } else {
            yaw > 0.0
        };
        let t1 = if pulled {
            -ctx.tuning.pivot_return_yaw_damping * yaw
        } else {
            let x = if angle > 0.0 {
                angle - 1.0
            } else {
                angle + 1.0
            };
            -ctx.tuning.pivot_outer_yaw_damping * yaw * (x * x)
        };
        let lr = lateral_speed / distance;
        let against = if angle > 0.0 {
            0.0 > lr
        } else {
            lr > 0.0
        };
        let t2 = if against {
            -ctx.tuning.pivot_lateral_yaw_damping * lr
        } else {
            0.0
        };
        let torque = (angle * ctx.tuning.pivot_angle_stiffness + t1) + t2;
        ctx.body.add_local_torque([torque * 0.0, torque, torque * 0.0].into());
    } else {
        ctx.vehicle.set_pivot_state(PivotState::Idle);
    }
}
