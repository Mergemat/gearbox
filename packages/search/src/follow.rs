use gearbox_engine::Controls;


const STEER_GAIN: f32 = 2.0;

const PASSED_M: f32 = 6.0;

pub fn steer_toward(body: &crate::Body, aim: [f32; 3]) -> Controls {
    let (p, r) = (body.position(), body.rotation());
    let heading = r[2].atan2(r[8]);
    let mut err = (aim[0] - p[0]).atan2(aim[2] - p[2]) - heading;
    while err > std::f32::consts::PI {
        err -= std::f32::consts::TAU;
    }
    while err < -std::f32::consts::PI {
        err += std::f32::consts::TAU;
    }
    Controls { gas: 1.0, brake: 0.0, steer: (-STEER_GAIN * err).clamp(-1.0, 1.0) }
}

pub(crate) fn next_point(p: [f32; 3], pts: &[[f32; 3]], mut i: usize) -> usize {
    while i + 1 < pts.len() {
        let (a, b) = (pts[i], pts[i + 1]);
        let (to, on) = ([p[0] - a[0], p[2] - a[2]], [b[0] - a[0], b[2] - a[2]]);
        let near = (to[0] * to[0] + to[1] * to[1]).sqrt() < PASSED_M;
        if near || to[0] * on[0] + to[1] * on[1] > 0.0 {
            i += 1;
        } else {
            break;
        }
    }
    i
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn points_pass_once_the_car_is_level_with_them() {
        let pts = [[0.0, 0.0, 0.0], [0.0, 0.0, 50.0], [20.0, 0.0, 100.0]];
        assert_eq!(next_point([0.0, 0.0, 30.0], &pts, 1), 1);
        assert_eq!(next_point([10.0, 0.0, 51.0], &pts, 1), 2);
    }
}
