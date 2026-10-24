use crate::{Aabb, Iso, Vec3, dyna::BodyState};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Contact {
    pub pose: Iso,
    pub velocity: Vec3,
}

fn overlap(bounds: Aabb, a: Iso, b: Iso) -> Option<(Vec3, f32)> {
    let ac = a.transform_y_first(bounds.center);
    let bc = b.transform_point(bounds.center);
    let delta = bc - ac;
    let axes_a = [a.rot.col(0), a.rot.col(1), a.rot.col(2)];
    let axes_b = [b.rot.col(0), b.rot.col(1), b.rot.col(2)];
    let r: [[f32; 3]; 3] = std::array::from_fn(|i| std::array::from_fn(|j| axes_a[i].dot_yxz(axes_b[j])));
    let mut abs = r.map(|row| row.map(f32::abs));
    let t = axes_a.map(|axis| axis.dot_yxz(delta));
    let h = bounds.half;
    let mut best = f32::MIN;
    let mut normal = Vec3::ZERO;
    for i in 0..3 {
        let separation = t[i].abs() - (h.x * abs[i][0] + h[i] + h.y * abs[i][1] + h.z * abs[i][2]);
        if 0.0 < separation {
            return None;
        }
        if best < separation {
            best = separation;
            normal = if t[i] < 0.0 { -axes_a[i] } else { axes_a[i] };
        }
    }
    for j in 0..3 {
        let d = axes_b[j].dot_yxz(delta);
        let separation = d.abs() - (h.y * abs[1][j] + h.x * abs[0][j] + h.z * abs[2][j] + h[j]);
        if 0.0 < separation {
            return None;
        }
        if best < separation {
            best = separation;
            normal = if d < 0.0 { -axes_b[j] } else { axes_b[j] };
        }
    }
    const EPS: f32 = 0.00001;
    for row in &mut abs {
        for x in row {
            *x += EPS;
        }
    }
    for i in 0..3 {
        let p = (i + 1) % 3;
        let q = (i + 2) % 3;
        for j in 0..3 {
            let u = (j + 1) % 3;
            let v = (j + 2) % 3;
            let d = t[q] * r[p][j] - t[p] * r[q][j];
            let radius = h[p] * abs[q][j] + h[q] * abs[p][j] + h[u] * abs[i][v] + h[v] * abs[i][u];
            let separation = d.abs() - radius;
            if EPS < separation {
                return None;
            }
            let mut axis = Vec3::ZERO;
            axis[p] = -r[q][j];
            axis[q] = r[p][j];
            let length = axis.dot(axis).sqrt();
            if length > EPS {
                let separation = separation / length;
                if best < separation * 1.05 {
                    axis = Vec3::new(axis.x / length, axis.y / length, axis.z / length);
                    normal = a.rot.transform_y_first(axis);
                    if d < 0.0 {
                        normal = -normal;
                    }
                    best = separation;
                }
            }
        }
    }
    Some((normal, -best))
}

pub fn response(bounds: Aabb, body: &mut BodyState, contacts: &[Contact], remaining: f32) -> bool {
    if contacts.is_empty() {
        return false;
    }
    let speed = body.vel.dot(body.vel).sqrt();
    let up = body.rot.col(1);
    let forward = body.rot.col(2);
    let mut sum = Vec3::ZERO;
    let mut turn = 0.0;
    let mut total = 0.0;
    let mut maximum = 0.0;
    for contact in contacts {
        let ghost_speed = contact.velocity.dot(contact.velocity).sqrt();
        let weight = if ghost_speed <= speed {
            1.0
        } else {
            (speed - ghost_speed * 0.9) / (ghost_speed - ghost_speed * 0.9)
        };
        if weight <= 0.00001 {
            continue;
        }
        let pose = Iso::new(contact.pose.rot, contact.pose.pos - contact.velocity * remaining);
        let Some((normal, depth)) = overlap(bounds, Iso::new(body.rot, body.pos), pose) else {
            continue;
        };
        let weight = weight * if depth / 0.15 >= 1.0 { 1.0 } else { depth / 0.15 };
        let normal = normal - up * normal.dot_yxz(up);
        let correction = normal * (-speed * 0.1);
        let ghost_forward = contact.pose.rot.col(2);
        let ghost_forward = ghost_forward - up * ghost_forward.dot_yxz(up);
        let forward = forward - up * forward.dot_yxz(up);
        let cross = Vec3::new(
            forward.y * ghost_forward.z - forward.z * ghost_forward.y,
            forward.z * ghost_forward.x - forward.x * ghost_forward.z,
            forward.x * ghost_forward.y - forward.y * ghost_forward.x,
        );
        let length = cross.dot(cross).sqrt();
        let mut angle = if length > 0.00001 {
            crate::math::angle_approx(length, forward.dot_yxz(ghost_forward))
        } else {
            0.0
        };
        if cross.dot_yxz(up) < 0.0 {
            angle = -angle;
        }
        total += weight;
        turn += angle * 0.1 * weight;
        sum += (correction + contact.velocity) * weight;
        if maximum < weight {
            maximum = weight;
        }
    }
    if total > 0.001 {
        let inverse = 1.0 / total;
        let mut velocity = (sum * inverse - body.vel) * maximum + body.vel;
        let length = velocity.dot(velocity).sqrt();
        if speed < length {
            velocity *= speed / length;
        }
        body.vel = velocity;
        body.ang_vel += up * (turn * inverse);
        true
    } else {
        maximum != 0.0
    }
}
