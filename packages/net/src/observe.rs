use anyhow::{Result, ensure};
use gearbox_engine::race::{Race, WaypointFlags};
use gearbox_engine::vehicle::forces::{ForceRays, common::PedalMode};
use gearbox_engine::{Mat3, Session, Vec3, surface};
use gearbox_search::gate::gates;

pub const VERSION: u32 = 3;
pub const AHEAD: usize = 4;
pub const SURFACES: usize = 12;
pub const CAR_LEN: usize = 17;
pub const WHEEL_LEN: usize = SURFACES + 3;
pub const EFFECTS_START: usize = CAR_LEN + 4 * WHEEL_LEN;
pub const COURSE_START: usize = EFFECTS_START + 9;
pub const TRACK_START: usize = COURSE_START + AHEAD * 5 + 7;
pub const ROAD_DISTANCES: [f32; 8] = [2.0, 4.0, 8.0, 16.0, 24.0, 40.0, 64.0, 96.0];
pub const LEN: usize = TRACK_START + 16 + ROAD_DISTANCES.len() * (1 + SURFACES);

#[derive(Clone, Debug, PartialEq)]
pub struct Target {
    pub group: u32,
    pub gates: Vec<[f32; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Course {
    pub checkpoints: Vec<Target>,
    pub finishes: Vec<[f32; 3]>,
    pub laps: u32,
}

impl Course {
    pub fn race(session: &Session) -> Result<Self> {
        let mut checkpoints: Vec<Target> = Vec::new();
        let mut finishes = Vec::new();
        for gate in gates(session)? {
            let mark = &session.race.rules.waypoints[gate.waypoint as usize];
            if !mark.linked || !mark.flags.counts() {
                continue;
            }
            let centers = gate.boxes.iter().map(|b| std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5));
            if mark.flags.contains(WaypointFlags::GOAL) {
                finishes.extend(centers);
            } else if let Some(target) = checkpoints.iter_mut().find(|target| target.group == mark.group) {
                target.gates.extend(centers);
            } else {
                checkpoints.push(Target { group: mark.group, gates: centers.collect() });
            }
        }
        ensure!(!finishes.is_empty(), "the map has no finish gate");
        let laps = if session.race.rules.lap_race { session.race.rules.laps.max(1) } else { 1 };
        Ok(Self { checkpoints, finishes, laps })
    }

    pub fn checkpoints_left(&self, session: &Session) -> usize {
        self.checkpoints.iter().filter(|target| !passed(&session.race, target.group)).count()
    }

    fn features(&self, car: &Car, race: &Race) -> [f32; AHEAD * 5 + 7] {
        let mut nearest = [(f32::INFINITY, [0.0; 3]); AHEAD];
        let mut left = 0;
        for target in &self.checkpoints {
            if passed(race, target.group) {
                continue;
            }
            left += 1;
            let candidate = closest(car.position, &target.gates);
            if let Some(index) = nearest.iter().position(|entry| candidate.0 < entry.0) {
                nearest.copy_within(index..AHEAD - 1, index + 1);
                nearest[index] = candidate;
            }
        }
        let mut out = [0.0; AHEAD * 5 + 7];
        for (slot, (distance, point)) in out[..AHEAD * 5].as_chunks_mut::<5>().0.iter_mut().zip(nearest) {
            if distance.is_finite() {
                slot.copy_from_slice(&car.target(point, 1.0));
            }
        }
        let completed = race.players.first().map_or(0, |player| {
            entries(race, player.player.race_run)
                .filter(|entry| {
                    race.rules.waypoints[entry.waypoint as usize].flags.contains(WaypointFlags::GOAL)
                })
                .count() as u32
        });
        let laps_left = self.laps.saturating_sub(completed);
        let finish = closest(car.position, &self.finishes).1;
        out[AHEAD * 5..AHEAD * 5 + 5]
            .copy_from_slice(&car.target(finish, f32::from(left == 0 && laps_left > 0)));
        out[AHEAD * 5 + 5] = left as f32 / 10.0;
        out[AHEAD * 5 + 6] = laps_left as f32;
        out
    }
}

fn entries(race: &Race, run: u32) -> impl Iterator<Item = &gearbox_engine::race::Entry> {
    race.store
        .chunks
        .iter()
        .filter(move |chunk| run != gearbox_engine::race::NO_RUN && chunk.run == run)
        .flat_map(|chunk| chunk.entries.iter().take_while(|entry| entry.time != gearbox_engine::race::NO_RUN))
}

fn passed(race: &Race, group: u32) -> bool {
    race.players.first().is_some_and(|player| {
        entries(race, player.player.lap_run)
            .any(|entry| race.rules.waypoints[entry.waypoint as usize].group == group)
    })
}

fn closest(position: [f32; 3], points: &[[f32; 3]]) -> (f32, [f32; 3]) {
    points
        .iter()
        .map(|&point| {
            let distance = (0..3).map(|k| (point[k] - position[k]).powi(2)).sum::<f32>();
            (distance, point)
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap_or((f32::INFINITY, position))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Car {
    pub rotation: Mat3,
    pub position: [f32; 3],
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
}

impl Car {
    pub fn read(session: &Session) -> Self {
        let body = session.body();
        Self {
            rotation: body.rot,
            position: body.pos.to_array(),
            velocity: body.vel,
            angular_velocity: body.ang_vel,
        }
    }

    pub fn to_car(&self, world: Vec3) -> Vec3 {
        self.rotation.transpose_mul_vec(world)
    }

    fn target(&self, point: [f32; 3], flag: f32) -> [f32; 5] {
        let delta = self.to_car(Vec3::from(point) - Vec3::from(self.position));
        let distance = delta.dot(delta).sqrt();
        let direction = if distance > 1e-3 { delta * (1.0 / distance) } else { Vec3::ZERO };
        [direction.x, direction.y, direction.z, distance.ln_1p() / 5.0, flag]
    }
}

fn surface_class(material: u8) -> usize {
    match material {
        surface::CONCRETE
        | surface::PAVEMENT
        | surface::ASPHALT
        | surface::WET_ASPHALT
        | surface::WET_PAVEMENT
        | surface::TURBO
        | surface::TURBO_2
        | surface::TURBO_ROULETTE
        | surface::FREE_WHEELING
        | surface::NO_GRIP
        | surface::NO_STEERING
        | surface::NO_BRAKES => 0,
        surface::DIRT | surface::DIRT_ROAD | surface::WET_DIRT_ROAD | surface::GRAVEL => 1,
        surface::GRASS | surface::WET_GRASS => 2,
        surface::ICE | surface::ROAD_ICE => 3,
        surface::PLASTIC | surface::ROAD_SYNTHETIC => 4,
        surface::WOOD
        | surface::SLIDING_WOOD
        | surface::TURBO_WOOD
        | surface::TURBO_2_WOOD
        | surface::FREE_WHEELING_WOOD => 5,
        surface::METAL
        | surface::RESONANT_METAL
        | surface::METAL_TRANS
        | surface::METAL_FENCE
        | surface::TECH_MAGNETIC
        | surface::TURBO_TECH_MAGNETIC
        | surface::TURBO_2_TECH_MAGNETIC
        | surface::FREE_WHEELING_TECH_MAGNETIC
        | surface::TECH_SUPER_MAGNETIC => 6,
        surface::SAND => 7,
        surface::SNOW => 8,
        surface::WATER => 9,
        surface::RUBBER | surface::SLIDING_RUBBER | surface::RUBBER_BAND => 10,
        _ => 11,
    }
}

pub fn observe(session: &Session, course: &Course) -> [f32; LEN] {
    let car = Car::read(session);
    let vehicle = session.vehicle();
    let tuning = &session.config.model(vehicle.model_choice).assets.tuning;
    let velocity = car.to_car(car.velocity);
    let mut out = [0.0; LEN];
    out[0..3].copy_from_slice(&(velocity * 0.01).to_array());
    out[3..6].copy_from_slice(&(car.to_car(car.angular_velocity) * 0.1).to_array());
    out[6..9].copy_from_slice(&car.to_car(Vec3::new(0.0, 1.0, 0.0)).to_array());
    out[9] = velocity.x / velocity.x.hypot(velocity.z).max(1.0);
    out[10] = vehicle.rpm / 10_000.0;
    out[11] = vehicle.gear as f32 / 6.0;
    out[12] = (vehicle.wheels[0].steering_angle + vehicle.wheels[1].steering_angle) / std::f32::consts::PI;
    out[13 + vehicle.model_choice as usize] = 1.0;
    for (slot, wheel) in
        out[CAR_LEN..EFFECTS_START].as_chunks_mut::<WHEEL_LEN>().0.iter_mut().zip(&vehicle.wheels)
    {
        slot[0] = f32::from(wheel.contact != 0);
        if wheel.contact != 0 {
            slot[1 + surface_class(wheel.material)] = 1.0;
        }
        slot[1 + SURFACES] = wheel.slip / (1.0 + wheel.slip.abs());
        slot[2 + SURFACES] = if tuning.suspension_rest_length > 0.0 {
            (1.0 - wheel.suspension_length / tuning.suspension_rest_length).clamp(0.0, 1.0)
        } else {
            0.0
        };
    }
    let turbo_left = if vehicle.turbo_active == 0 {
        0.0
    } else if vehicle.turbo_end == u32::MAX {
        1.0
    } else {
        vehicle.turbo_duration_ms.saturating_sub(session.clock.now.wrapping_sub(vehicle.turbo_end)) as f32
            / vehicle.turbo_duration_ms.max(1) as f32
    };
    out[EFFECTS_START..COURSE_START].copy_from_slice(&[
        f32::from(vehicle.turbo_active != 0),
        turbo_left,
        vehicle.boost_level as f32 / 2.0,
        f32::from(vehicle.effect_mode_is(PedalMode::Cut)),
        f32::from(vehicle.has_effect_flag(8)),
        f32::from(vehicle.has_effect_flag(4)),
        f32::from(vehicle.forced_speed != 0.0 || vehicle.forced_speed_integer != 0),
        f32::from(vehicle.has_effect_flag(16)),
        (1.0 - vehicle.level_power_scale).clamp(0.0, 1.0),
    ]);
    out[COURSE_START..TRACK_START].copy_from_slice(&course.features(&car, &session.race));
    let rays = ForceRays {
        geometry: gearbox_engine::shape::Geometry::new(&session.world, &session.state),
        manager: &session.state.collision_world[session.config.manager],
        vehicle_item: vehicle.item,
    };
    let origin = Vec3::from(car.position);
    for (slot, degrees) in out[TRACK_START..TRACK_START + 16].iter_mut().zip([
        -90.0_f32, -75.0, -60.0, -45.0, -30.0, -20.0, -10.0, -5.0, 5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 75.0,
        90.0,
    ]) {
        let (sin, cos) = degrees.to_radians().sin_cos();
        let direction = car.rotation * Vec3::new(sin * 100.0, 0.0, cos * 100.0);
        *slot = rays.raycast(origin, direction).map_or(1.0, |hit| hit.fraction);
    }
    for (slot, distance) in
        out[TRACK_START + 16..].as_chunks_mut::<{ 1 + SURFACES }>().0.iter_mut().zip(ROAD_DISTANCES)
    {
        let start = origin + car.rotation * Vec3::new(0.0, 16.0, distance);
        slot[0] = -1.0;
        if let Some(hit) = rays.raycast(start, car.rotation * Vec3::new(0.0, -80.0, 0.0)) {
            slot[0] = (16.0 - hit.fraction * 80.0) / 64.0;
            slot[1 + surface_class(hit.surface)] = 1.0;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use gearbox_engine::Iso;
    use gearbox_engine::race::{
        IdSequence, Mode, NO_RUN, Player, RaceRun, RaceWaypoint, Racer, Rules, Store, WaypointState,
    };

    fn race(laps: u32) -> Race {
        let waypoints = [(0, false), (1, false), (1, false), (2, true)]
            .map(|(group, finish)| RaceWaypoint {
                flags: if finish { WaypointFlags::GOAL } else { WaypointFlags::CHECKPOINT },
                group,
                order: u32::MAX,
                pose: Iso::default(),
                respawn_pose: Iso::default(),
                linked: true,
                respawn: false,
            })
            .to_vec();
        Race {
            rules: Rules {
                mode: Mode::Race,
                checkpoints_per_lap: 2,
                lap_race: laps > 1,
                laps,
                waypoints,
                clear_untouched: false,
            },
            store: Store {
                runs: vec![
                    RaceRun { id: 1, finish: NO_RUN, start: 0 },
                    RaceRun { id: 2, finish: NO_RUN, start: 0 },
                ],
                chunks: vec![],
                ids: vec![IdSequence {
                    owner: 0,
                    first: 10,
                    second: 1000,
                    first_limit: 1000,
                    second_limit: 1000,
                    used: 0,
                }],
            },
            players: vec![Racer {
                player: Player { race_run: 1, lap_run: 2, ..Default::default() },
                controller: None,
                waypoints: WaypointState::default(),
                saved_waypoints: WaypointState::default(),
                open_lap: None,
                closed_lap: None,
                timed_events: Default::default(),
            }],
        }
    }

    fn course(laps: u32) -> Course {
        Course {
            checkpoints: vec![
                Target { group: 0, gates: vec![[100.0, 0.0, 0.0]] },
                Target { group: 1, gates: vec![[0.0, 0.0, 10.0], [0.0, 0.0, -200.0]] },
            ],
            finishes: vec![[0.0, 0.0, 20.0]],
            laps,
        }
    }

    fn car() -> Car {
        Car {
            rotation: Mat3::IDENTITY,
            position: [0.0; 3],
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }

    #[test]
    fn nearest_unpassed_targets_ignore_the_order_they_were_built_in() {
        let mut race = race(1);
        let course = course(1);
        let before = course.features(&car(), &race);
        assert_eq!(&before[..3], &[0.0, 0.0, 1.0]);
        assert_eq!(&before[5..8], &[1.0, 0.0, 0.0]);
        race.waypoint(0, None, 2, 10);
        let after = course.features(&car(), &race);
        assert_eq!(&after[..3], &[1.0, 0.0, 0.0]);
        assert_eq!(&after[5..20], &[0.0; 15]);
        assert_eq!(after[25], 0.1);
    }

    #[test]
    fn either_linked_gate_counts_the_group_once() {
        for waypoint in [1, 2] {
            let mut race = race(1);
            race.waypoint(0, None, waypoint, 10);
            race.waypoint(0, None, 3 - waypoint, 20);
            assert_eq!(course(1).features(&car(), &race)[25], 0.1);
            assert_eq!(race.state(0).unwrap().checkpoints.len(), 1);
        }
    }

    #[test]
    fn only_the_four_nearest_groups_fill_the_target_slots() {
        let mut course = course(1);
        for (group, distance) in [(2, 2.0), (3, 30.0), (4, 5.0), (5, 1.0)] {
            course.checkpoints.push(Target { group, gates: vec![[0.0, 0.0, distance]] });
        }
        let features = course.features(&car(), &race(1));
        for (slot, distance) in features[..20].as_chunks::<5>().0.iter().zip([1.0_f32, 2.0, 5.0, 10.0]) {
            assert_eq!(&slot[..3], &[0.0, 0.0, 1.0]);
            assert!((slot[3] - distance.ln_1p() / 5.0).abs() < 1e-6);
            assert_eq!(slot[4], 1.0);
        }
        assert_eq!(features[25], 0.6);
    }

    #[test]
    fn finish_opens_once_every_checkpoint_of_the_lap_is_passed() {
        let mut race = race(2);
        let course = course(2);
        for lap in 0..2 {
            assert_eq!(course.features(&car(), &race)[26], (2 - lap) as f32);
            assert_eq!(course.features(&car(), &race)[24], 0.0);
            race.waypoint(0, None, 0, lap * 100 + 10);
            assert_eq!(course.features(&car(), &race)[24], 0.0);
            race.waypoint(0, None, 1, lap * 100 + 20);
            assert_eq!(course.features(&car(), &race)[24], 1.0);
            race.waypoint(0, None, 3, lap * 100 + 30);
        }
        let finished = course.features(&car(), &race);
        assert_eq!(finished[24], 0.0);
        assert_eq!(finished[26], 0.0);
        assert_eq!(race.state(0).unwrap().finish, Some(130));
    }

    #[test]
    fn heading_rotates_targets_and_motion_into_the_car_frame() {
        let car = Car { rotation: Mat3::from_array([0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]), ..car() };
        assert_eq!(car.to_car(Vec3::new(50.0, 0.0, 0.0)).to_array(), [0.0, 0.0, 50.0]);
        assert_eq!(&car.target([-100.0, 0.0, 0.0], 1.0)[..3], &[0.0, 0.0, -1.0]);
        assert_eq!(&car.target([0.0; 3], 1.0)[..3], &[0.0; 3]);
    }
}
