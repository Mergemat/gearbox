use super::ForceCtx;
use super::common::{KMH, TINY};
use crate::math::{fast_exp2, fast_log2, maxss};
use crate::{Quat, Vec3};

const HALF_PI: f32 = 1.5707964;

pub fn wheel_forces(ctx: &mut ForceCtx<'_>, fraction: f32) {
    let wheel_count = ctx.geometry.wheels.len();
    let inverse_count = 1.0 / wheel_count as f32;
    for definition in &ctx.geometry.wheels {
        let vehicle = &*ctx.vehicle;
        let tuning = ctx.tuning;
        let orientation = vehicle.quat;
        let a = orientation.rotate(definition.anchor);
        let y = a.y + vehicle.pos.y;
        let x = a.x + vehicle.pos.x;
        let z = a.z + vehicle.pos.z;
        let lo = y - definition.radius;
        let span = (y + definition.radius) - lo;
        let mut depth = 0.0;
        if !ctx.water.query(Vec3::new(x, lo, z), span, Some(&mut depth), None) {
            continue;
        }
        let Vec3 { x: vx, y: vy, z: vz } = ctx.body.velocity_at(Vec3::new(x, y, z));
        let (vx2, vz2) = (vx * vx, vz * vz);
        let speed_lift = tuning.water_lift_by_speed.eval((vx2 + vz2).sqrt() * KMH);
        let vertical_lift = tuning.water_lift_by_vertical_speed.eval(vy * KMH);
        let forward_axis = orientation.rotate(Vec3::new(0.0, 0.0, 1.0));
        let l2 = (vy * vy + vx2) + vz2;
        let (nx, ny, nz) = if l2 > TINY && f32::MAX > l2 {
            let k = 1.0 / l2.sqrt();
            (k * vx, vy * k, vz * k)
        } else {
            (vx, vy, vz)
        };
        let dot = (ny * forward_axis.y + nx * forward_axis.x) + nz * forward_axis.z;
        let heading_lift =
            tuning.water_lift_by_heading.eval(dot * (vehicle.controls.gas - vehicle.controls.brake));
        let up = heading_lift * (vertical_lift * speed_lift) * fraction * inverse_count;
        let inverse_orientation = Quat::new(orientation.w, -orientation.x, -orientation.y, -orientation.z);
        ctx.body.add_local_force(inverse_orientation.rotate(Vec3::new(0.0, up, 0.0)));
        let speed_drag = tuning.water_wheel_drag.eval(l2.sqrt() * KMH);
        let immersion_drag = fast_exp2(fast_log2(vehicle.immersion) * tuning.water_drag_immersion_power);
        let drag_scale = speed_drag * -fraction * inverse_count * immersion_drag;
        let drag_force =
            inverse_orientation.rotate(Vec3::new(drag_scale * vx, drag_scale * vy, drag_scale * vz));
        let inverse_dt = 1.0 / ctx.dt;
        ctx.body.add_local_force(Vec3::new(
            drag_force.x * inverse_dt,
            drag_force.y * inverse_dt,
            drag_force.z * inverse_dt,
        ));
        let steers = definition.steers && !vehicle.has_effect_flag(8);
        let steering_angle = if steers { -vehicle.steering * HALF_PI } else { 0.0 };
        let steering_rotation = Quat::about_y(steering_angle);
        let side_axis = steering_rotation.rotate(Vec3::new(1.0, 0.0, 0.0));
        let forward_axis = steering_rotation.rotate(Vec3::new(0.0, 0.0, 1.0));
        let Vec3 { x: l0, y: l1, z: l2 } = ctx.local_velocity;
        let side = (side_axis.y * l1 + side_axis.x * l0) + side_axis.z * l2;
        let ll = (l1 * l1 + l0 * l0) + l2 * l2;
        let a = (side * KMH).abs();
        let velocity_direction = if ll > TINY && f32::MAX > ll {
            let k = 1.0 / ll.sqrt();
            Vec3::new(l0 * k, l1 * k, l2 * k)
        } else {
            Vec3::new(l0, l1, l2)
        };
        let lateral_response = tuning.water_lateral_force.eval(a);
        let forward_force = lateral_response * a * inverse_count;
        let forward_drag = forward_axis * forward_force;
        let k = if side.is_sign_negative() { 1.0 } else { -1.0 } * forward_force;
        let side_drag = side_axis * k;
        let backward_drag = ((side_drag.y + forward_drag.y) * velocity_direction.y
            + (side_drag.x + forward_drag.x) * velocity_direction.x)
            + (side_drag.z + forward_drag.z) * velocity_direction.z;
        let backward_drag = -maxss(backward_drag, 0.0);
        ctx.body.add_local_force(velocity_direction * backward_drag);
        ctx.body.add_local_force(forward_drag);
        ctx.body.add_local_force(side_drag);
    }
}
