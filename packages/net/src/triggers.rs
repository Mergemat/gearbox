use gearbox_engine::consumer::EffectTrigger;
use gearbox_engine::{Session, Vec3};

use crate::observe::Car;

pub const NEAREST: usize = 4;
pub const EFFECTS: usize = 23;
pub const FEATURES: usize = 7 + EFFECTS;
pub const LEN: usize = NEAREST * FEATURES;
pub const RANGE: f32 = 400.0;
const TOUCH: f32 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trigger {
    pub center: [f32; 3],
    pub direction: Vec3,
    pub effect: u8,
}

pub fn build(session: &Session) -> Vec<Trigger> {
    let mut parts: Vec<EffectTrigger> = session
        .effect_triggers()
        .into_iter()
        .filter(|trigger| (1..=EFFECTS as u8).contains(&trigger.effect))
        .collect();
    let mut i = 0;
    while i < parts.len() {
        let mut grew = false;
        let mut j = i + 1;
        while j < parts.len() {
            if joined(&parts[i], &parts[j]) {
                let other = parts.swap_remove(j);
                for k in 0..3 {
                    parts[i].min[k] = parts[i].min[k].min(other.min[k]);
                    parts[i].max[k] = parts[i].max[k].max(other.max[k]);
                }
                grew = true;
            } else {
                j += 1;
            }
        }
        if !grew {
            i += 1;
        }
    }
    parts
        .into_iter()
        .map(|part| Trigger {
            center: std::array::from_fn(|k| (part.min[k] + part.max[k]) * 0.5),
            direction: part.direction,
            effect: part.effect,
        })
        .collect()
}

fn joined(a: &EffectTrigger, b: &EffectTrigger) -> bool {
    a.effect == b.effect
        && a.direction.dot(b.direction) > 0.99
        && (0..3).all(|k| a.min[k] <= b.max[k] + TOUCH && b.min[k] <= a.max[k] + TOUCH)
}

pub fn read(triggers: &[Trigger], car: &Car, travel: [f32; 2], out: &mut [f32]) {
    assert_eq!(out.len(), LEN);
    let mut nearest: [(f32, Option<&Trigger>); NEAREST] = [(RANGE * RANGE, None); NEAREST];
    for trigger in triggers {
        let delta: [f32; 3] = std::array::from_fn(|k| trigger.center[k] - car.position[k]);
        if delta[0] * travel[0] + delta[2] * travel[1] < 0.0 {
            continue;
        }
        let distance = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
        if let Some(index) = nearest.iter().position(|entry| distance < entry.0) {
            nearest.copy_within(index..NEAREST - 1, index + 1);
            nearest[index] = (distance, Some(trigger));
        }
    }
    for (slot, (_, trigger)) in out.as_chunks_mut::<FEATURES>().0.iter_mut().zip(nearest) {
        slot.fill(0.0);
        let Some(trigger) = trigger else { continue };
        let target = car.target(trigger.center, 0.0);
        slot[..3].copy_from_slice(&target[..3]);
        slot[3] = target[3] * 5.0 / 6.0;
        slot[4..7].copy_from_slice(&car.to_car(trigger.direction).to_array());
        slot[6 + trigger.effect as usize] = 1.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gearbox_engine::Mat3;

    fn car(position: [f32; 3]) -> Car {
        Car { rotation: Mat3::IDENTITY, position, velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO }
    }

    fn trigger(center: [f32; 3], effect: u8) -> Trigger {
        Trigger { center, direction: Vec3::new(0.0, 0.0, 1.0), effect }
    }

    fn effects(out: &[f32]) -> Vec<Option<u8>> {
        out.as_chunks::<FEATURES>()
            .0
            .iter()
            .map(|slot| slot[7..].iter().position(|&v| v == 1.0).map(|i| i as u8 + 1))
            .collect()
    }

    #[test]
    fn the_nearest_triggers_ahead_of_travel_are_typed_by_their_gameplay_effect() {
        let triggers = [
            trigger([0.0, 0.0, 300.0], 18),
            trigger([0.0, 0.0, -5.0], 9),
            trigger([3.0, 0.0, 10.0], 22),
            trigger([0.0, 0.0, 401.0], 1),
            trigger([0.0, 50.0, 40.0], 13),
            trigger([-20.0, 0.0, 30.0], 23),
            trigger([0.0, 0.0, 200.0], 5),
        ];
        let mut out = [0.0; LEN];
        read(&triggers, &car([0.0; 3]), [0.0, 1.0], &mut out);
        assert_eq!(effects(&out), [Some(22), Some(23), Some(13), Some(5)]);
        assert!(out.as_chunks::<FEATURES>().0.iter().all(|slot| slot[7..].iter().sum::<f32>() == 1.0));
        read(&triggers, &car([0.0; 3]), [0.0, -1.0], &mut out);
        assert_eq!(effects(&out), [Some(9), None, None, None]);
        assert!(out[FEATURES..].iter().all(|&v| v == 0.0));
        let slot = &out[..FEATURES];
        assert_eq!(&slot[..3], &[0.0, 0.0, -1.0]);
        assert!((slot[3] - libm::log1pf(5.0) / 6.0).abs() < 1e-6);
        assert_eq!(&slot[4..7], &[0.0, 0.0, 1.0]);
    }

    #[test]
    fn trigger_and_gate_directions_turn_with_the_car() {
        let car = Car {
            rotation: Mat3::from_array([0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]),
            ..car([0.0; 3])
        };
        let mut out = [0.0; LEN];
        read(&[trigger([100.0, 0.0, 0.0], 20)], &car, [1.0, 0.0], &mut out);
        assert_eq!(&out[..3], &[0.0, 0.0, 1.0]);
        assert_eq!(&out[4..7], &[-1.0, 0.0, 0.0]);
        assert_eq!(effects(&out)[0], Some(20));
    }
}
