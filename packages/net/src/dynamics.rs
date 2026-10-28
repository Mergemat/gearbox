use gearbox_engine::{Session, Vec3};

pub const LEN: usize = 83;

pub fn read(session: &Session) -> Vec<f32> {
    let v = session.vehicle();
    let now = session.clock.now;
    let age = |begin: u32| {
        if begin == u32::MAX { -1.0 } else { now.wrapping_sub(begin).min(10_000) as f32 * 0.001 }
    };
    let remaining = |until: u32| (until.wrapping_sub(now) as i32).clamp(0, 10_000) as f32 * 0.001;
    let mut out = vec![
        session.applied.steer,
        session.applied.gas,
        session.applied.brake,
        v.steering,
        v.gearbox_state as f32 / 4.0,
        v.shift_remaining,
        f32::from(v.reversing != 0),
        v.grip_meter,
        v.special_grip_blend,
        v.drift_thrust_scale,
        v.drift_direction as f32 / 2.0,
        v.drift_yaw_state as f32 / 2.0,
        f32::from(v.slip_armed != 0),
        f32::from(v.in_water != 0),
        v.immersion,
        v.forced_speed / 100.0,
        v.steering_scale,
        v.grip_scale,
        v.vehicle_effect_speed_scale,
        v.gravity_effect_scale,
        session.race_ms() as f32 / 60_000.0,
        age(v.drift_release_begin),
        age(v.drift_thrust_begin),
        remaining(v.thrust_boost_until),
        f32::from(v.special_grip_blend_active != 0),
        f32::from(v.body_contact != 0),
        v.brake_history.count as f32 / 20.0,
    ];
    let rotation = session.body().rot;
    for wheel in &v.wheels {
        out.extend([
            wheel.steering_angle,
            wheel.spin / (1.0 + wheel.spin.abs()),
            wheel.suspension_rate / (1.0 + wheel.suspension_rate.abs()),
            wheel.drive_coefficient,
            f32::from(wheel.slipping != 0),
            wheel.effect_level,
        ]);
        let normal = Vec3::new(wheel.contact_normal_x, wheel.contact_normal_y, wheel.contact_normal_z);
        out.extend(rotation.transpose_mul_vec(normal).to_array());
    }
    let history = &v.brake_history;
    for lag in 0..20 {
        out.push(if lag < history.count as usize {
            f32::from(history.samples[(history.cursor as usize + 20 - lag) % 20]) / 255.0
        } else {
            0.0
        });
    }
    debug_assert_eq!(out.len(), LEN);
    out
}
