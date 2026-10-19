use crate::Body;
use anyhow::Result;

use crate::follow::{next_point, steer_toward};
use crate::input::Timeline;
use crate::search::Origin;

pub struct Tried {
    pub line: Timeline,
    pub trail: Vec<[f32; 3]>,
    pub speed_at_end: f64,
}

pub fn drive_on(
    origin: &dyn Origin,
    line: &Timeline,
    from: usize,
    to: usize,
    until: usize,
    follow: &[[f32; 3]],
) -> Result<Tried> {
    let mut n = origin.native()?;
    n.seed = line.seed;
    let mut line = line.clone();
    if line.controls.len() < until {
        let last = line.controls.last().copied().unwrap_or_default();
        line.controls.resize(until, last);
    }
    let (mut trail, mut speed_at_end, mut i) = (Vec::new(), 0.0, 0);
    let mut k = line.index(n.race_ms())?;
    while k < until {
        let body = Body::from(n.body());
        if k >= to && !follow.is_empty() {
            i = next_point(body.position(), follow, i);
            line.controls[k] = steer_toward(&body, follow[i]);
        }
        if k >= from {
            if k % 5 == 0 {
                trail.push(body.position());
            }
            if k == to {
                speed_at_end =
                    body.local_velocity().iter().map(|v| f64::from(*v).powi(2)).sum::<f64>().sqrt();
            }
        }
        n.step(line.controls[k]);
        k += 1;
        if n.race().is_some_and(|r| r.finish.is_some()) {
            break;
        }
    }
    Ok(Tried { line, trail, speed_at_end })
}
