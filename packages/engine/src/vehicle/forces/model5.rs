use super::gearbox::GearboxState;
use super::common::PivotState;
use super::{ForceCtx, adhesion, angular, boost, bumps, control, drag, drive, grip, pivot, surface, water,
    wheel_step, wheels};
use super::surface::{SurfaceFactors, SurfaceResponse};
use super::common::{
    EPS, KMH, TINY, PEDAL_THRESHOLD as TENTH, PedalMode, AMPLIFIED_EFFECT_SCALE, PIVOT_UP_THRESHOLD,
    DIRECTION_DEAD_ZONE, MS_PER_SECOND, clamp01, sign_one, timed_ramp, asym_scale,
};
use crate::math::{minss, sincos as sincos_value};

const STEERING_MINIMUM_SPEED: f32 = 0.7;
const PI: f32 = std::f32::consts::PI;

struct ModelStep {
    surface: SurfaceResponse,
    factors: SurfaceFactors,
    grip_slope: f32,
    thrust_slope: f32,
    steer: f32,
    blend: f32,
    remaining: f32,
    brake_on: bool,
    gas_on: bool,
    all_special: bool,
}

pub fn forces(ctx: &mut ForceCtx<'_>) {
    let (step, any) = prepare(ctx);
    let mut armed = 0;
    if ctx.vehicle.pivot_state() == PivotState::Turning {
        pivot::pivot_forces(ctx, any, step.steer, step.blend);
        return finish(ctx);
    }
    update_pivot_timers(ctx, &mut armed);
    let mut any_slip = false;
    for index in 0..ctx.geometry.wheels.len() {
        if grip_force(ctx, index, &step) {
            any_slip = true;
        }
    }
    if !any {
        if ctx.vehicle.pivot_state() == PivotState::Armed {
            ctx.vehicle.pivot_retire_begin = ctx.tick;
            ctx.vehicle.set_pivot_state(PivotState::Retiring);
        }
        ctx.vehicle.slip_armed = armed;
        return finish(ctx);
    }
    let velocity = ctx.local_velocity;
    let speed = ((velocity.x * velocity.x + velocity.y * velocity.y) + velocity.z * velocity.z).sqrt();
    pivot::pivot_turn(ctx, step.steer, &step.surface);
    select_direction(ctx, &step);
    let response = ctx.tuning.side_speed_roll_torque
        .eval(velocity.x * velocity.x / (velocity.z.abs() + 1.0) * KMH);
    ctx.body.add_local_torque([0.0, 0.0, sign_one(-velocity.x) * response].into());
    let (mut demand, mut capacity) = (0.0, 0.0);
    for index in 0..ctx.geometry.wheels.len() {
        lateral_force(ctx, index, &step, speed, &mut demand, &mut capacity, &mut armed);
    }
    if armed != 0 {
        ctx.vehicle.last_slip_armed = ctx.tick;
    }
    let thrust = engine_thrust(ctx, &step, capacity, demand);
    braking_thrust(ctx, &step, thrust, any_slip, &mut armed);
    ctx.vehicle.slip_armed = armed;
    finish(ctx);
}

fn prepare(ctx: &mut ForceCtx<'_>) -> (ModelStep, bool) {
    control::input_overrides(ctx);
    let brake_on = ctx.vehicle.controls.brake > TENTH;
    let gas_on = ctx.vehicle.controls.gas > TENTH;
    cache_velocity(ctx);
    if !ctx.vehicle.level_consistent() {
        for wheel in &mut ctx.vehicle.wheels {
            wheel.contact = wheel.boost_contact;
        }
    }
    control::fold_wheel_samples(ctx);
    if gas_on {
    }
    let fraction = clamp01((1.0 - ctx.vehicle.grip) / (1.0 - ctx.tuning.effect_minimum_grip));
    let boost_material = ctx.vehicle.wheels.iter().any(|wheel| wheel.has_boost_material());
    let blend = if boost_material {
        ctx.tuning.special_material_slip.eval(fraction)
    } else {
        ctx.tuning.material_slip.eval(fraction)
    };
    if ctx.tuning.car_model_kind().updates_wheel_visuals() {
        bumps::surface_bumps(ctx);
        wheel_step::wheels_engine_step(ctx);
    }
    let remaining = 1.0 - blend;
    let (mut grip_slope, mut thrust_slope) = ctx.tuning.slope_factors(ctx.local_force);
    grip_slope = grip_slope * remaining;
    thrust_slope = thrust_slope * remaining;
    let steer = ctx.vehicle.steer_angle(ctx.tuning, ctx.local_velocity);
    drive::drive_forces(ctx, blend, ctx.dt * MS_PER_SECOND);
    let (response, factors, any) = adhesion::surface_adhesion(ctx, &mut grip_slope, &mut thrust_slope);
    drag::brake_and_drag(ctx, blend);
    water::water_step(ctx);
    surface::drain_meter(ctx, &response);
    let all_special = ctx.vehicle.wheels.iter().all(|wheel| wheel.contact != 0
        && (wheel.material == 6 || (wheel.material == 5 && ctx.tuning.special_material_enabled != 0)));
    (ModelStep {
        surface: response,
        factors,
        grip_slope,
        thrust_slope,
        steer,
        blend,
        remaining,
        brake_on,
        gas_on,
        all_special,
    }, any)
}

fn update_pivot_timers(ctx: &mut ForceCtx<'_>, armed: &mut u32) {
    let mut retire = false;
    if ctx.vehicle.pivot_state() == PivotState::Armed {
        let since = ctx.vehicle.pivot_arm_begin;
        if ctx.tick >= since && ctx.tick - since < ctx.tuning.pivot_arm_ms {
            *armed = 1;
        } else {
            ctx.vehicle.pivot_retire_begin = ctx.tick;
            ctx.vehicle.set_pivot_state(PivotState::Retiring);
            retire = true;
        }
    } else if ctx.vehicle.pivot_state() == PivotState::Retiring {
        retire = true;
    }
    if retire {
        let since = ctx.vehicle.pivot_retire_begin;
        if ctx.tick >= since && ctx.tick - since < ctx.tuning.pivot_retire_ms {
            set_slip_flags(ctx);
        } else {
            ctx.vehicle.set_pivot_state(PivotState::Idle);
            ctx.vehicle.fixed_wheel_spin = 0;
        }
    }
}

fn cache_velocity(ctx: &mut ForceCtx<'_>) {
    ctx.vehicle.force_velocity_lateral = ctx.local_velocity.x;
    ctx.vehicle.force_velocity_vertical = ctx.local_velocity.y;
    ctx.vehicle.force_velocity_forward = ctx.local_velocity.z;
}

fn finish(ctx: &mut ForceCtx<'_>) {
    for wheel in &mut ctx.vehicle.wheels { wheel.slip = if wheel.slipping != 0 {
        1.0
    } else {
        0.0
    }; }
    cache_velocity(ctx);
    if !ctx.vehicle.any_wheel_contact() && ctx.vehicle.axis_torque_enabled != 0 {
        control::input_torque(ctx);
    }
    let flagged = ctx.vehicle.wheels.iter().zip(&ctx.geometry.wheels)
        .any(|(wheel, definition)| wheel.contact != 0 && definition.contact_flag);
    angular::angular_control(ctx, ctx.vehicle.any_wheel_contact(), flagged);
    boost::level_models(ctx);
    if ctx.tuning.hover_forces_enabled != 0 {
        todo!("hover forces");
    }
    let force = ctx.body.local_force();
    let inverse_mass = 1.0 / ctx.tuning.force_feedback_mass;
    ctx.vehicle.force_feedback_forward = inverse_mass * force.z;
    ctx.vehicle.force_feedback_lateral = inverse_mass * force.x;
    ctx.vehicle.force_feedback_vertical = inverse_mass * force.y;
    ctx.vehicle.pivot_blocked = 0;
}

fn average_drive(ctx: &ForceCtx<'_>) -> f32 {
    let count = ctx.geometry.wheels.len();
    if count == 0 {
        return 0.0;
    }
    ctx.vehicle.wheels.iter().take(count).fold(0.0, |sum, wheel| sum + wheel.drive_coefficient) / count as f32
}

fn slip_product(ctx: &ForceCtx<'_>, coefficient: f32) -> f32 {
    let mut product = 1.0;
    for (index, wheel) in ctx.vehicle.wheels.iter().enumerate() {
        if wheel.slipping != 0 { product = if index == 0 {
            coefficient
        } else {
            product * coefficient
        }; }
    }
    product
}

fn set_slip_flags(ctx: &mut ForceCtx<'_>) {
    for wheel in &mut ctx.vehicle.wheels {
        wheel.slipping = 1;
    }
}

fn select_direction(ctx: &mut ForceCtx<'_>, step: &ModelStep) {
    let velocity = ctx.local_velocity.to_array();
    let (brake_on, gas_on) = (step.brake_on, step.gas_on);
    let blend = step.blend;
    let tuning = ctx.tuning;
    let direction_dead_zone = DIRECTION_DEAD_ZONE;
    if ctx.vehicle.effect_mode_is(PedalMode::Full) {
        let forward_speed = velocity[2];
        let reversing = !(forward_speed >= 0.0) && !(direction_dead_zone > forward_speed.abs())
            && !(velocity[0].abs() > direction_dead_zone);
        ctx.vehicle.reversing = reversing as u32;
    } else if blend > 0.5 {
        if brake_on && tuning.reverse_selection_speed > velocity[2] {
            ctx.vehicle.reversing = 1;
        }
        if gas_on {
            ctx.vehicle.reversing = 0;
        }
        if TENTH > ctx.vehicle.controls.gas && TENTH > ctx.vehicle.controls.brake {
            let reversing = !(velocity[2] >= 0.0) && !(direction_dead_zone > velocity[2].abs());
            ctx.vehicle.reversing = reversing as u32;
        }
    } else if ctx.vehicle.pivot_state() != PivotState::Idle {
        ctx.vehicle.reversing = 0;
    } else {
        let blocked = tuning.low_grip_threshold >= ctx.vehicle.grip
            || ctx.vehicle.special_material_present != 0;
        if brake_on && tuning.reverse_selection_speed > velocity[2]
            && direction_dead_zone > velocity[0].abs() {
            ctx.vehicle.reversing = 1;
        }
        if gas_on && (velocity[2] > 0.0 || velocity[0].abs() > direction_dead_zone || blocked) {
            ctx.vehicle.reversing = 0;
        }
        if TENTH > ctx.vehicle.controls.gas && TENTH > ctx.vehicle.controls.brake {
            let reversing = !(velocity[2] >= 0.0) && !(direction_dead_zone > velocity[2].abs());
            ctx.vehicle.reversing = reversing as u32;
        }
        if velocity[2] > 0.0 && ctx.vehicle.turbo_active != 0 {
            ctx.vehicle.reversing = 0;
        }
    }
}

fn grip_force(ctx: &mut ForceCtx<'_>, index: usize, step: &ModelStep) -> bool {
    let remaining = step.remaining;
    let steer = step.steer;
    let brake_on = step.brake_on;
    let all_special = step.all_special;
    let tick = ctx.tick;
    let velocity = ctx.local_velocity.to_array();
    let mut slipping = false;

    if ctx.vehicle.level_consistent() {
        wheels::suspension_force(ctx, index);
    }
    let material = ctx.geometry.material(ctx.vehicle.wheels[index].material);
    let wheel_grip = ctx.vehicle.wheel_grip[index];
    if ctx.vehicle.wheels[index].contact == 0 || !(ctx.tuning.lateral_stiffness >= 0.0) {
        return false;
    }
    let (axis_x, axis_y, axis_z, axis_speed) = grip_axis_and_feedback(ctx, index, steer, remaining, velocity);
    let mut ramp = 1.0f32;
    if ctx.vehicle.pivot_state() == PivotState::Armed {
        let elapsed = tick.wrapping_sub(ctx.vehicle.pivot_arm_begin) as f32;
        let span = ctx.tuning.pivot_arm_ms.wrapping_add(ctx.tuning.pivot_arm_ms) as f32;
        let (_, cosine) = sincos_value(elapsed * PI / span);
        ramp = (ctx.tuning.pivot_arm_grip_scale - 1.0) * cosine + 1.0;
    }
    let (upper_length, lower_length) = (ctx.tuning.suspension_grip_upper_length,
        ctx.tuning.suspension_grip_lower_length);
    let suspension_fraction = if upper_length == lower_length {
        0.0
    } else {
        (ctx.vehicle.wheels[index].suspension_length - lower_length) / (upper_length - lower_length)
    };
    let suspension_response = ctx.tuning.grip_by_suspension.eval(suspension_fraction);
    let immersion = ctx.vehicle.immersion;
    let wet_grip = ctx.tuning.wet_grip.eval(immersion);
    let grip_scale = clamp01(ctx.geometry.grip_scale * ctx.vehicle.grip_scale);
    let average_drive_coefficient = average_drive(ctx);
    let grip = average_drive_coefficient * grip_scale * remaining * wet_grip;
    let (slip_scale, brake_slip_scale) = if ctx.vehicle.wheels[index].slipping != 0 {
        (ctx.tuning.slipping_grip_scale, if brake_on {
            ctx.tuning.slipping_brake_grip_scale
        } else {
            1.0
        })
    } else {
        (1.0, 1.0)
    };
    let speed_grip = ctx.tuning.grip_by_speed.eval(velocity[2] * KMH);
    let cap = speed_grip * (step.grip_slope * material.grip_scale * step.factors.grip * grip)
        * slip_scale * brake_slip_scale * suspension_response;
    let mut side_force = -ctx.tuning.lateral_stiffness * 0.5 * axis_speed * ramp * wheel_grip;
    if side_force.abs() * grip > cap {
        let k = ctx.tuning.grip_excess_blend;
        let cap = if side_force > 0.0 {
            cap
        } else {
            -cap
        };
        ctx.vehicle.wheels[index].slipping = 1;
        slipping = true;
        side_force = (1.0 - k) * cap + k * side_force;
    } else {
        ctx.vehicle.wheels[index].slipping = 0;
    }
    let side_force = side_force * grip;
    let force = [side_force * axis_x, side_force * axis_y, side_force * axis_z];
    special_grip(ctx, all_special, tick);
    ctx.body.add_local_force((force).into());

    slipping
}

fn lateral_force(ctx: &mut ForceCtx<'_>, index: usize, step: &ModelStep, speed: f32,
    demand: &mut f32, capacity: &mut f32, armed: &mut u32)
{
    let remaining = step.remaining;
    let speed = speed;
    let axle_span = ctx.geometry.axle_span;

    let steers = ctx.geometry.wheels[index].steers as u32;
    let velocity = ctx.local_velocity.to_array();
    let axle_offset = if steers == 0 {
        -axle_span
    } else {
        axle_span
    } * 0.5;
    let lateral_speed = axle_offset * ctx.local_angular_velocity.y + velocity[0];
    let steering_response = if STEERING_MINIMUM_SPEED > speed.abs() {
        0.0
    } else {
        let top = ctx.tuning.steering_full_response_speed;
        if speed > top {
            1.0
        } else {
            sincos_value(speed / top * PI * 0.5).0
        }
    };
    let forward_speed = velocity[2];
    let speed_grip = ctx.tuning.grip_by_speed.eval(forward_speed * KMH);
    let vehicle_grip_scale = ctx.vehicle.grip_scale;
    let combined_grip_scale = clamp01(vehicle_grip_scale * ctx.geometry.grip_scale);
    let average_drive_coefficient = average_drive(ctx);
    let cap = average_drive_coefficient * step.surface.grip * combined_grip_scale
        * ctx.vehicle.wheel_grip[index] * speed_grip;
    let side = (((velocity[1] + 0.0) * 0.0 + lateral_speed) + (forward_speed + 0.0) * 0.0)
        * (-ctx.tuning.lateral_stiffness * 0.5) * remaining;
    let mut side_force = side;
    if side.abs() > cap {
        let k = ctx.tuning.lateral_excess_blend;
        *capacity = *capacity + cap;
        *demand = *demand + side.abs();
        let lim = (1.0 - k) * cap + k * side.abs();
        side_force = sign_one(side) * lim;
        let mat = ctx.vehicle.wheels[index].material;
        *armed = !matches!(mat, 0x4a | 0x3 | 0x15) as u32;
    }
    let grip_scale = clamp01(vehicle_grip_scale * ctx.geometry.grip_scale);
    let mut side_force = side_force * ctx.tuning.lateral_torque_scale * grip_scale;
    if steers != 0 {
        side_force = side_force - lateral_steering(ctx, index, step, forward_speed, steering_response);
    }
    let g = clamp01(ctx.geometry.steering_scale * ctx.vehicle.steering_scale);
    let side_force = side_force * (remaining * g);
    {
        let zero = side_force * 0.0;
        ctx.body.add_local_torque([0.0 * zero - axle_offset * zero, side_force * axle_offset - 0.0 * zero,
            0.0 * zero - side_force * 0.0].into());
    }
    if ctx.vehicle.immersion > EPS {
        immersed_lateral_force(ctx, step);
    }
}

fn engine_thrust(ctx: &mut ForceCtx<'_>, step: &ModelStep, capacity: f32, demand: f32) -> f32 {
    let tick = ctx.tick;
    let time_ns = ctx.time_ns;
    let blocked = ctx.tuning.low_grip_threshold >= ctx.vehicle.grip
        || ctx.vehicle.special_material_present != 0;
    let mut pedal_thrust = 1.0f32;
    if !blocked && tick == ctx.vehicle.last_slip_armed && capacity > EPS {
        let demand_fraction = (demand - capacity) / capacity / ctx.tuning.slip_thrust_demand_scale;
        let demand_fraction = if demand_fraction >= 0.0 {
            minss(demand_fraction, 1.0)
        } else {
            0.0
        };
        pedal_thrust = 1.0 - demand_fraction;
    }
    let forward_speed = ctx.local_velocity.z;
    let speed_kmh = forward_speed * KMH;
    let slip_response = ctx.tuning.slip_thrust.eval(speed_kmh);
    let reversing = ctx.vehicle.reversing != 0;
    let pivot_scale = slip_response * ctx.tuning.slip_thrust_scale;
    let base_thrust = (if reversing {
        &ctx.tuning.reverse_thrust
    } else {
        &ctx.tuning.engine_thrust
    }).eval(speed_kmh);
    let loaded_thrust
        = if GearboxState::from_code(ctx.vehicle.gearbox_state) == Some(GearboxState::Shifting) {
        0.0
    } else {
        (1.0 - pedal_thrust) * pivot_scale + pedal_thrust * base_thrust
    };
    let steering_loss = ctx.tuning.steering_thrust_loss.eval(speed_kmh);
    let lost_thrust = steering_loss * (ctx.vehicle.steering.abs() * ctx.tuning.steering_thrust_loss_scale);
    let (pivot_scale, pivot_thrust) = pivot_thrust_envelope(ctx, tick);
    let mut pedal_thrust = if ctx.vehicle.effect_mode_is(PedalMode::Full) {
        1.0
    } else if ctx.vehicle.effect_mode_is(PedalMode::Cut) {
        0.0
    } else {
        let surface_thrust = step.surface.thrust;
        surface_thrust * ctx.vehicle.controls.brake * (if reversing {
            -1.0f32
        } else {
            0.0
        }) + surface_thrust * ctx.vehicle.controls.gas
    };
    pedal_thrust = pedal_thrust * loaded_thrust * step.factors.thrust;
    pedal_thrust = effect_thrust(ctx, speed_kmh, pedal_thrust);
    let lx = ctx.local_velocity.x;
    let impact_scale = if time_ns < ctx.vehicle.impact_thrust_until as i64 {
        ctx.tuning.impact_thrust_by_speed.eval((lx * lx + forward_speed * forward_speed).sqrt())
    } else {
        1.0
    };
    let turbo_scale = if ctx.vehicle.turbo_active != 0 {
        ctx.vehicle.turbo_thrust_scale
    } else {
        0.0
    };
    let direction_sign = if reversing {
        -1.0f32
    } else {
        1.0
    };
    let thrust_scale = clamp01(ctx.geometry.thrust_scale * ctx.vehicle.thrust_scale);
    let mut thrust = ((((turbo_scale * base_thrust) + pedal_thrust) * pivot_scale - lost_thrust
        * direction_sign) + pivot_thrust) * impact_scale * thrust_scale;
    thrust = timed_thrust_boost(ctx, tick, forward_speed, thrust);
    finish_engine_thrust(ctx, thrust, base_thrust)
}

fn braking_thrust(ctx: &mut ForceCtx<'_>, step: &ModelStep, thrust: f32, any_slip: bool, armed: &mut u32) {
    let gas_on = step.gas_on;
    let brake_on = step.brake_on;
    let remaining = step.remaining;
    let tick = ctx.tick;
    let mut brake = 0.0f32;
    brake = forward_brake(ctx, step, any_slip, armed, brake);
    let forward_speed = ctx.local_velocity.z;
    if 0.0 > forward_speed && gas_on && step.surface.pivot_enabled != 0 {
        let up = ctx.body.state().rot.rows[1].y;
        if ctx.vehicle.effect_mode_is_free() && !ctx.vehicle.has_effect_flag(4)
            && ctx.vehicle.grip >= ctx.tuning.low_grip_threshold {
            let x = -thrust * ctx.tuning.force_feedback_mass * forward_speed;
            if x > ctx.tuning.reverse_pivot_power_threshold && up > PIVOT_UP_THRESHOLD {
                ctx.vehicle.pivot_arm_begin = tick;
                ctx.vehicle.set_pivot_state(PivotState::Armed);
                ctx.vehicle.fixed_wheel_spin = 1;
            }
        }
        let k = slip_product(ctx, ctx.tuning.slipping_brake_scale);
        brake = if ctx.vehicle.has_effect_flag(4) {
            0.0
        } else {
            (ctx.tuning.brake_constant - forward_speed * ctx.tuning.brake_speed_scale)
                * ctx.vehicle.controls.gas * k
        };
        let reverse_cap = if any_slip {
            ctx.tuning.reverse_brake_slipping_cap
        } else {
            ctx.tuning.reverse_brake_gripping_cap
        };
        let cap = step.surface.brake * ctx.vehicle.grip * reverse_cap;
        if brake > cap {
            brake = cap;
            *armed = 1;
            set_slip_flags(ctx);
        }
    }
    if ctx.tuning.low_grip_threshold >= ctx.vehicle.grip {
        let forward_speed = ctx.local_velocity.z;
        let hold = (forward_speed > ctx.tuning.reverse_hold_speed && brake_on)
            || (ctx.tuning.forward_hold_speed > forward_speed && gas_on);
        if hold {
            *armed = 1;
            set_slip_flags(ctx);
        }
    }
    let forward_speed = ctx.local_velocity.z;
    let brake = brake * sign_one(forward_speed);
    let surface_limit = step.surface.speed_limit;
    let (forward_limit, reverse_limit) = (surface_limit * ctx.tuning.forward_speed_limit,
        surface_limit * ctx.tuning.reverse_speed_limit);
    let mut force = thrust - brake;
    if forward_speed > forward_limit {
        let k = ctx.tuning.speed_limit_force;
        force = if 0.0 > force {
            force - k
        } else {
            -k
        };
    }
    if -reverse_limit > forward_speed {
        let k = ctx.tuning.speed_limit_force;
        force = if force > 0.0 {
            force + k
        } else {
            k
        };
    }
    let force = force * step.thrust_slope * remaining;
    ctx.body.add_local_force(([0.0, 0.0, force]).into());
    longitudinal_feedback(ctx, remaining);
}

fn in_range(squared: f32) -> bool { squared > TINY && f32::MAX > squared }

fn special_grip(ctx: &mut ForceCtx<'_>, all_special: bool, tick: u32) {
    if ctx.tuning.special_grip_blend_enabled == 0 {
        if all_special {
            grip::steer_grip_forces(ctx, 1.0);
        }
    } else {
        let mut flag = ctx.vehicle.special_grip_blend_active;
        if all_special == (flag == 0) {
            let x = timed_ramp(tick, ctx.tuning.special_grip_transition_ms,
                ctx.vehicle.special_grip_blend_begin, ctx.vehicle.special_grip_blend, flag != 0);
            ctx.vehicle.special_grip_blend = x;
            flag = (flag == 0) as u32;
            ctx.vehicle.special_grip_blend_active = flag;
            ctx.vehicle.special_grip_blend_begin = tick;
        }
        let x = timed_ramp(tick, ctx.tuning.special_grip_blend_ms,
            ctx.vehicle.special_grip_blend_begin, ctx.vehicle.special_grip_blend, flag != 0);
        grip::steer_grip_forces(ctx, x);
    }
}

fn grip_axis_and_feedback(
    ctx: &mut ForceCtx<'_>, index: usize, steer: f32, remaining: f32, velocity: [f32; 3],
) -> (f32, f32, f32, f32) {
    let (nx, ny, nz) = (ctx.vehicle.wheels[index].contact_normal_x,
        ctx.vehicle.wheels[index].contact_normal_y, ctx.vehicle.wheels[index].contact_normal_z);
    let cv = ctx.geometry.center_of_mass.to_array();
    let rx = ctx.vehicle.wheels[index].contact_point_x - cv[0];
    let ry = ctx.vehicle.wheels[index].contact_point_y - cv[1];
    let rz = ctx.vehicle.wheels[index].contact_point_z - cv[2];
    let nz0 = nz * 0.0;
    let c0 = ny - nz0;
    let c1 = nz0 - nx;
    let c2 = nx * 0.0 - ny * 0.0;
    let l2 = (c1 * c1 + c0 * c0) + c2 * c2;
    let (mut axis_x, mut axis_y, mut axis_z) = if in_range(l2) {
        let inv = 1.0 / l2.sqrt();
        (inv * c0, inv * c1, inv * c2)
    } else {
        (1.0, 0.0, 0.0)
    };
    if ctx.geometry.wheels[index].steers as u32 != 0 {
        let (sn, cs) = sincos_value(steer);
        let ns = -sn;
        let ns0 = ns * 0.0;
        axis_x = axis_x * cs + ns0;
        axis_y = axis_y * cs + ns0;
        axis_z = axis_z * cs + ns;
    }
    let k = -ctx.tuning.force_feedback_mass;
    let (velocity_x, velocity_y, forward_speed) = (remaining * velocity[0], remaining * velocity[1],
        remaining * velocity[2]);
    let (mut feedback_x, mut feedback_y, mut feedback_z) = (k * ctx.vehicle.force_feedback_lateral, k
        * ctx.vehicle.force_feedback_vertical, k * ctx.vehicle.force_feedback_forward);
    let axis_speed = (velocity_y * axis_y + velocity_x * axis_x) + forward_speed * axis_z;
    let feedback_length = ((feedback_y * feedback_y + feedback_x * feedback_x) + feedback_z
        * feedback_z).sqrt();
    if ctx.tuning.force_feedback_threshold > feedback_length {
        (feedback_x, feedback_y, feedback_z) = (0.0, 0.0, 0.0);
    }
    let pitch_torque = (feedback_z * ry - feedback_y * rz) * -1.0 * ctx.tuning.force_feedback_pitch_scale;
    let roll_torque = (feedback_y * rx - feedback_x * ry) * -1.0 * ctx.tuning.force_feedback_roll_scale;
    ctx.body.add_local_torque(([pitch_torque, 0.0, roll_torque]).into());
    if ctx.vehicle.pivot_state() == PivotState::Armed {
        let blend = ctx.tuning.pivot_torque_by_forward_speed.eval(velocity[2] * KMH);
        ctx.body.add_local_torque(([blend, 0.0, 0.0]).into());
    }
    (axis_x, axis_y, axis_z, axis_speed)
}

fn immersed_lateral_force(ctx: &mut ForceCtx<'_>, step: &ModelStep) {

    let (quat_w, quat_x, quat_y, quat_z) = (ctx.vehicle.quat.w, ctx.vehicle.quat.x,
        ctx.vehicle.quat.y, ctx.vehicle.quat.z);
    let (q00, q10, q20, q30) = (quat_w * 0.0, quat_x * 0.0, quat_y * 0.0, quat_z * 0.0);
    let a = (quat_y - q30) + q00;
    let b = (q30 - quat_x) + q00;
    let cc = (q10 - q20) + quat_w;
    let e1 = b * quat_x - a * quat_y;
    let e0 = a * quat_z - cc * quat_x;
    let e2 = cc * quat_y - b * quat_z;
    let forward_axis = [(e0 + e0) + 0.0, (e2 + e2) + 0.0, (e1 + e1) + 1.0];
    let (mut velocity_x, mut velocity_y, mut velocity_z) = (ctx.vehicle.vel.x, ctx.vehicle.vel.y,
        ctx.vehicle.vel.z);
    let l2 = (velocity_y * velocity_y + velocity_x * velocity_x) + velocity_z * velocity_z;
    if in_range(l2) {
        let inv = 1.0 / l2.sqrt();
        (velocity_x, velocity_y, velocity_z) = (velocity_x * inv, velocity_y * inv, velocity_z * inv);
    }
    let mut lateral_thrust = step.surface.lateral_thrust;
    if lateral_thrust == -1.0 {
        lateral_thrust = 0.0;
    }
    let lateral_thrust = lateral_thrust * ctx.local_velocity.x;
    let alignment = step.factors.alignment;
    let aligned_thrust = (alignment * lateral_thrust).abs();
    ctx.body.add_local_force(([aligned_thrust * 0.0, aligned_thrust * 0.0, aligned_thrust]).into());
    let yaw_drag = (step.surface.yaw_drag * ctx.local_angular_velocity.y * (1.0 - alignment)).abs();
    ctx.body.add_local_force(([yaw_drag * 0.0, yaw_drag * 0.0, yaw_drag * -1.0]).into());
    let forward_alignment = ((velocity_y * forward_axis[0] + velocity_x * forward_axis[1])
        + velocity_z * forward_axis[2]).abs();
    let side_drag = (1.0 - forward_alignment) * lateral_thrust;
    ctx.body.add_local_force(([side_drag * -1.0, side_drag * 0.0, side_drag * 0.0]).into());
}

fn pivot_thrust_envelope(ctx: &ForceCtx<'_>, tick: u32) -> (f32, f32) {
    let mut extra_thrust = 0.0f32;
    let state = ctx.vehicle.pivot_state();
    let mut scale = 1.0f32;
    if state == PivotState::Armed {
        let elapsed = tick.wrapping_sub(ctx.vehicle.pivot_arm_begin) as f32;
        let (sine, _) = sincos_value(elapsed * PI / ctx.tuning.pivot_arm_ms as f32);
        scale = (ctx.tuning.pivot_arm_thrust_scale - 1.0) * sine + 1.0;
    } else if state == PivotState::Retiring {
        let duration = ctx.tuning.pivot_retire_ms;
        let elapsed = tick.wrapping_sub(ctx.vehicle.pivot_retire_begin);
        let (sine, _) = sincos_value(elapsed as f32 * PI / duration as f32);
        let q = (elapsed / duration) as f32 - 1.0;
        extra_thrust = q * q * ctx.tuning.pivot_retire_extra_thrust;
        scale = (ctx.tuning.pivot_retire_thrust_scale - 1.0) * sine + 1.0;
    }
    (scale, extra_thrust)
}

fn timed_thrust_boost(ctx: &mut ForceCtx<'_>, tick: u32, forward_speed: f32, mut thrust: f32) -> f32 {
    let boost_until = ctx.vehicle.thrust_boost_until;
    if !(tick > boost_until) || boost_until == u32::MAX {
        let boost_elapsed = ctx.vehicle.thrust_boost_elapsed as f32;
        let thrust_time_response = ctx.tuning.boost_thrust_by_time.eval(boost_elapsed);
        let absolute_speed_kmh = forward_speed.abs() * KMH;
        let thrust_speed_response = ctx.tuning.boost_thrust_by_speed.eval(absolute_speed_kmh);
        thrust = thrust * (thrust_speed_response * thrust_time_response);
        let drag_time_response = ctx.tuning.boost_drag_by_time.eval(boost_elapsed);
        let velocity = ctx.local_velocity.to_array();
        let drag_speed_response = ctx.tuning.boost_drag_by_speed.eval(absolute_speed_kmh);
        let drag_scale = -(drag_speed_response * drag_time_response);
        ctx.body.add_local_force(([velocity[0] * drag_scale, velocity[1] * drag_scale, velocity[2]
            * drag_scale]).into());
    }
    thrust
}

fn longitudinal_feedback(ctx: &mut ForceCtx<'_>, remaining: f32) {
    let mut g = clamp01(ctx.geometry.gravity_scale * ctx.vehicle.gravity_scale);
    if ctx.vehicle.vehicle_effect_active != 0 {
        g = g * ctx.vehicle.vehicle_effect_gravity;
    }
    if ctx.vehicle.gravity_effect_active != 0 {
        g = g * ctx.vehicle.gravity_effect_scale;
    }
    let x0 = remaining * 0.0;
    let fz = -ctx.tuning.longitudinal_force_feedback * ctx.local_force.z / (g
        * ctx.tuning.ground_gravity) * remaining;
    ctx.body.add_local_force(([x0, x0, fz]).into());
}

fn effect_thrust(ctx: &ForceCtx<'_>, speed_kmh: f32, mut pedal_thrust: f32) -> f32 {
    if ctx.vehicle.gear_effect_active != 0 && ctx.vehicle.gear != 0 {
        pedal_thrust = pedal_thrust * ctx.vehicle.gear_effect_thrust;
    }
    let speed_control = ctx.vehicle.signed_speed_control;
    if speed_control != 0.0 {
        let a = asym_scale(speed_control);
        pedal_thrust = pedal_thrust * ctx.tuning.control_response.factor(speed_kmh, a, false);
    }
    if ctx.vehicle.gravity_effect_active != 0 {
        pedal_thrust = pedal_thrust * ctx.vehicle.gravity_effect_thrust;
    }
    if ctx.vehicle.vehicle_effect_active != 0 {
        let mut k = ctx.vehicle.vehicle_effect_thrust;
        if ctx.vehicle.vehicle_effect_amplified != 0 {
            k = k * AMPLIFIED_EFFECT_SCALE;
        }
        pedal_thrust = pedal_thrust * k;
    }
    pedal_thrust
}

fn lateral_steering(
    ctx: &ForceCtx<'_>, index: usize, step: &ModelStep, forward_speed: f32, steering_response: f32,
) -> f32 {

    let mut steering_speed = forward_speed;
    if ctx.vehicle.vehicle_effect_active != 0 {
        let mut k = ctx.vehicle.vehicle_effect_speed_scale;
        if ctx.vehicle.vehicle_effect_amplified != 0 {
            k = k * AMPLIFIED_EFFECT_SCALE;
        }
        steering_speed = steering_speed / k;
    }
    let grip_blend = if ctx.vehicle.turbo_active != 0 {
        ctx.tuning.boost_steering_grip_blend
    } else {
        ctx.tuning.steering_grip_blend
    };
    let direction_sign = if ctx.vehicle.reversing != 0 {
        -1.0f32
    } else {
        1.0
    };
    let slip_scale = if ctx.vehicle.wheels[index].slipping != 0 {
        ctx.tuning.slipping_steering_scale
    } else {
        1.0
    };
    let speed_steering = ctx.tuning.steering_by_speed.eval(steering_speed * KMH);
    let a = direction_sign * steering_response * ctx.vehicle.steering;
    let b = (1.0 - grip_blend) * (1.0 - ctx.vehicle.wheel_grip[index]);
    let mut steering_torque = speed_steering * a * step.factors.steering * slip_scale * (1.0 - b);
    if ctx.tuning.direction_drag_enabled != 0 {
        let x = ctx.vehicle.wheel_meter;
        steering_torque = steering_torque * ctx.tuning.meter_steering.eval(x);
    }
    steering_torque
}

fn forward_brake(
    ctx: &mut ForceCtx<'_>, step: &ModelStep, any_slip: bool, armed: &mut u32, mut brake: f32,
) -> f32 {
    let forward_speed = ctx.local_velocity.z;
    let grip = ctx.vehicle.grip;
    if forward_speed > 0.0 {
        let k = slip_product(ctx, ctx.tuning.slipping_brake_scale);
        brake = if ctx.vehicle.has_effect_flag(4) {
            0.0
        } else {
            (forward_speed * ctx.tuning.brake_speed_scale + ctx.tuning.brake_constant)
                * ctx.vehicle.controls.brake * k
        };
        let brake_cap = if any_slip {
            ctx.tuning.brake_slipping_cap
        } else {
            ctx.tuning.brake_gripping_cap
        };
        let immersion = ctx.vehicle.immersion;
        let cap = ctx.tuning.wet_brake.eval(immersion) * (step.surface.brake * grip * brake_cap);
        if brake > cap {
            brake = cap;
            *armed = 1;
            set_slip_flags(ctx);
        }
    }
    brake
}

fn finish_engine_thrust(ctx: &mut ForceCtx<'_>, mut thrust: f32, base_thrust: f32) -> f32 {
    let forward_speed = ctx.local_velocity.z;
    if ctx.vehicle.in_water != 0 && ctx.tuning.water_thrust_reduction != 0 {
        thrust = thrust * 0.5;
    }
    if ctx.vehicle.effect_mode_is(PedalMode::Cut) {
        thrust = (if ctx.vehicle.turbo_active != 0 {
            ctx.vehicle.turbo_thrust_scale
        } else {
            0.0
        }) * base_thrust;
    }
    if ctx.tuning.direction_drag_enabled != 0 {
        let x = ctx.vehicle.wheel_meter;
        thrust = thrust * ctx.tuning.meter_thrust.eval(x);
        if ctx.vehicle.any_wheel_contact() {
            let meter_drag = ctx.tuning.meter_ground_drag.eval(forward_speed * KMH);
            ctx.body.add_local_force(([0.0, 0.0, -(meter_drag * x)]).into());
        }
    }
    thrust = thrust * boost::start_boost(ctx);

    thrust
}
