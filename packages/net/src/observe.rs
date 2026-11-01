use anyhow::{Result, ensure};
use gearbox_engine::race::{Race, WaypointFlags};
use gearbox_engine::vehicle::forces::common::PedalMode;
use gearbox_engine::{Mat3, Session, Vec3, surface};
use gearbox_search::gate::gates;

use crate::ground::{self, Ground};
use crate::triggers::{self, Trigger};
use crate::{atlas, chart, cloud, dynamics, rays};

pub const VERSION: u32 = 8;
pub const AHEAD: usize = 4;
pub const SURFACES: usize = 17;
pub const CAR_LEN: usize = 16;
pub const WHEEL_LEN: usize = SURFACES + 3;
pub const EFFECTS_START: usize = CAR_LEN + 4 * WHEEL_LEN;
pub const DYNAMICS_START: usize = EFFECTS_START + 9;
pub const TARGETS_START: usize = DYNAMICS_START + dynamics::LEN;
pub const GROUND_START: usize = TARGETS_START + AHEAD * 5 + 7;
pub const RAYS_START: usize = GROUND_START + ground::LEN;
pub const LEN: usize = RAYS_START + rays::LEN;
pub const TRAVEL_START: usize = LEN;
pub const TRIGGERS_START: usize = TRAVEL_START + ground::TRAVEL_LEN;
pub const RAYS_3D_START: usize = TRIGGERS_START + triggers::LEN;
pub const STATE_START: usize = RAYS_3D_START + rays::EXTRA_LEN;
pub const STATE_LEN: usize = 15;
pub const LEN_V9: usize = STATE_START + STATE_LEN;
pub const EVERY: usize = 32;
pub const GATE_LEN: usize = 10;
pub const DRIVEN_START: usize = LEN;
pub const EVERY_START: usize = DRIVEN_START + 4;
pub const IMAGES_START: usize = EVERY_START + (EVERY + 1) * GATE_LEN + 4;
pub const LEN_V10: usize = IMAGES_START + atlas::LEN;
pub const EVERY_V11: usize = 512;
pub const POSE_START: usize = EVERY_START + (EVERY_V11 + 1) * GATE_LEN + 4;
pub const IMAGES_START_V11: usize = POSE_START + chart::POSE_LEN;
pub const LEN_V11: usize = IMAGES_START_V11 + atlas::LEN;
pub const DOTS_START: usize = POSE_START;
pub const LEN_V12: usize = DOTS_START + cloud::DOTS * cloud::FEATURES;
pub const CAR_POSE_START: usize = POSE_START;
pub const LEN_V13: usize = CAR_POSE_START + 12;

const _: () = assert!(LEN == 677 && LEN_V9 == 1006 && LEN_V10 == 13111 && LEN_V11 == 17919);

pub fn len(version: u32) -> Option<usize> {
    match version {
        8 => Some(LEN),
        9 => Some(LEN_V9),
        10 => Some(LEN_V10),
        11 => Some(LEN_V11),
        12 => Some(LEN_V12),
        13 => Some(LEN_V13),
        _ => None,
    }
}

/// The length of observation `version`, with `dots` dots for v12.
pub fn len_dots(version: u32, dots: usize) -> Option<usize> {
    if version == 12 { (dots > 0).then_some(DOTS_START + dots * cloud::FEATURES) } else { len(version) }
}

/// How many dots an observation of `len` values holds, if it is a v12 one (any count of at least one dot).
pub fn dots(len: usize) -> Option<usize> {
    let rest = len.checked_sub(DOTS_START)?;
    (rest > 0 && rest % cloud::FEATURES == 0 && len != LEN_V10 && len != LEN_V11 && len != LEN_V13)
        .then_some(rest / cloud::FEATURES)
}

/// Where an observation's values that are int8 codes (multiples of 1/32 in [-4, 4)) start: v10's and v11's images,
/// v12's dots.
pub fn codes_start(len: usize) -> Option<usize> {
    match len {
        LEN_V10 => Some(IMAGES_START),
        LEN_V11 => Some(IMAGES_START_V11),
        _ => dots(len).map(|_| DOTS_START),
    }
}

pub fn images(version: u32) -> Vec<[usize; 4]> {
    let start = match version {
        10 => IMAGES_START,
        11 => IMAGES_START_V11,
        _ => return Vec::new(),
    };
    (0..atlas::SCALES.len())
        .map(|k| [start + k * atlas::IMAGE_LEN, atlas::SIDE, atlas::SIDE, atlas::CHANNELS])
        .collect()
}

#[derive(Clone, Debug, PartialEq)]
pub struct Target {
    pub group: u32,
    pub gates: Vec<[f32; 3]>,
    pub facings: Vec<[f32; 3]>,
    pub links: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Course {
    pub checkpoints: Vec<Target>,
    pub finishes: Vec<[f32; 3]>,
    pub finish_facings: Vec<[f32; 3]>,
    pub finish_links: u32,
    pub laps: u32,
    pub triggers: Vec<Trigger>,
}

impl Course {
    pub fn race(session: &Session) -> Result<Self> {
        let mut checkpoints: Vec<Target> = Vec::new();
        let mut finish = Target { group: 0, gates: Vec::new(), facings: Vec::new(), links: 0 };
        for gate in gates(session)? {
            let mark = &session.race.rules.waypoints[gate.waypoint as usize];
            if !mark.linked || !mark.flags.counts() {
                continue;
            }
            let centers = gate.boxes.iter().map(|b| std::array::from_fn(|k| (b.min[k] + b.max[k]) * 0.5));
            let target = if mark.flags.contains(WaypointFlags::GOAL) {
                &mut finish
            } else if let Some(at) = checkpoints.iter().position(|target| target.group == mark.group) {
                &mut checkpoints[at]
            } else {
                checkpoints.push(Target {
                    group: mark.group,
                    gates: Vec::new(),
                    facings: Vec::new(),
                    links: 0,
                });
                checkpoints.last_mut().unwrap()
            };
            target.gates.extend(centers);
            let facing = (mark.pose.rot * Vec3::new(0.0, 0.0, 1.0)).to_array();
            target.facings.resize(target.gates.len(), facing);
            target.links += 1;
        }
        ensure!(!finish.gates.is_empty(), "the map has no finish gate");
        let laps = if session.race.rules.lap_race { session.race.rules.laps.max(1) } else { 1 };
        Ok(Self {
            checkpoints,
            finishes: finish.gates,
            finish_facings: finish.facings,
            finish_links: finish.links,
            laps,
            triggers: triggers::build(session),
        })
    }

    pub fn checkpoints_left(&self, session: &Session) -> usize {
        self.checkpoints.iter().filter(|target| !passed(&session.race, target.group)).count()
    }

    pub fn task_potential(&self, session: &Session) -> f32 {
        self.potential_at(session.vehicle().pos.to_array(), &session.race)
    }

    pub fn gates_left(&self, session: &Session) -> usize {
        let laps = self.laps_left(&session.race);
        if laps == 0 {
            return 0;
        }
        (laps - 1) as usize * (self.checkpoints.len() + 1) + self.checkpoints_left(session) + 1
    }

    fn laps_left(&self, race: &Race) -> u32 {
        let completed = race.players.first().map_or(0, |player| {
            entries(race, player.player.race_run)
                .filter(|entry| {
                    race.rules.waypoints[entry.waypoint as usize].flags.contains(WaypointFlags::GOAL)
                })
                .count() as u32
        });
        self.laps.saturating_sub(completed)
    }

    fn potential_at(&self, position: [f32; 3], race: &Race) -> f32 {
        let laps_left = self.laps_left(race);
        if laps_left == 0 {
            return 0.0;
        }
        let mut left = 0;
        let mut distance = f32::INFINITY;
        for target in &self.checkpoints {
            if !passed(race, target.group) {
                left += 1;
                distance = distance.min(closest(position, &target.gates).0);
            }
        }
        if left == 0 {
            distance = closest(position, &self.finishes).0;
        }
        let distance = distance.sqrt();
        let proximity = if distance.is_finite() { distance / (distance + 100.0) } else { 1.0 };
        let per_lap = self.checkpoints.len() + 1;
        let remaining = (laps_left - 1) as usize * per_lap + left;
        -(remaining as f32 + proximity) / (self.laps as usize * per_lap) as f32
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
        let laps_left = self.laps_left(race);
        let finish = closest(car.position, &self.finishes).1;
        out[AHEAD * 5..AHEAD * 5 + 5]
            .copy_from_slice(&car.target(finish, f32::from(left == 0 && laps_left > 0)));
        out[AHEAD * 5 + 5] = left as f32 / 10.0;
        out[AHEAD * 5 + 6] = laps_left as f32;
        out
    }

    fn every(&self, car: &Car, race: &Race, out: &mut [f32]) {
        let count = (out.len() - 4) / GATE_LEN - 1;
        assert_eq!(out.len(), (count + 1) * GATE_LEN + 4);
        let gate = |gates: &[[f32; 3]], facings: &[[f32; 3]], flag: bool, links: u32| {
            gates
                .iter()
                .zip(facings)
                .map(|(&point, &facing)| {
                    let target = car.target(point, 0.0);
                    let facing = car.to_car(Vec3::from(facing)).to_array();
                    let mut slot = [0.0; GATE_LEN];
                    slot[..4].copy_from_slice(&target[..4]);
                    slot[4..7].copy_from_slice(&facing);
                    slot[7..].copy_from_slice(&[f32::from(flag), links as f32 / 4.0, 1.0]);
                    slot
                })
                .min_by(order)
                .unwrap_or_default()
        };
        let mut slots: Vec<[f32; GATE_LEN]> = self
            .checkpoints
            .iter()
            .map(|target| gate(&target.gates, &target.facings, passed(race, target.group), target.links))
            .collect();
        slots.sort_by(order);
        let (every, rest) = out.split_at_mut(count * GATE_LEN);
        for (out, slot) in every.as_chunks_mut::<GATE_LEN>().0.iter_mut().zip(&slots) {
            *out = *slot;
        }
        let left = self.checkpoints.iter().filter(|target| !passed(race, target.group)).count();
        let laps_left = self.laps_left(race);
        let open = left == 0 && laps_left > 0;
        rest[..GATE_LEN].copy_from_slice(&gate(
            &self.finishes,
            &self.finish_facings,
            open,
            self.finish_links,
        ));
        rest[GATE_LEN..].copy_from_slice(&[
            left as f32 / self.checkpoints.len().max(1) as f32,
            self.checkpoints.len() as f32 / EVERY as f32,
            laps_left as f32,
            self.laps as f32,
        ]);
    }
}

fn order(a: &[f32; GATE_LEN], b: &[f32; GATE_LEN]) -> std::cmp::Ordering {
    a[3].total_cmp(&b[3])
        .then_with(|| a.iter().zip(b).fold(std::cmp::Ordering::Equal, |o, (x, y)| o.then(x.total_cmp(y))))
}

fn entries(race: &Race, run: u32) -> impl Iterator<Item = &gearbox_engine::race::Entry> {
    race.store
        .chunks
        .iter()
        .filter(move |chunk| run != gearbox_engine::race::NO_RUN && chunk.run == run)
        .flat_map(|chunk| chunk.entries.iter().take_while(|entry| entry.time != gearbox_engine::race::NO_RUN))
}

pub(crate) fn passed(race: &Race, group: u32) -> bool {
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

    pub fn travel(&self) -> ([f32; 2], f32) {
        let speed = hypot(self.velocity.x, self.velocity.z);
        if speed >= ground::TRAVEL_MIN_SPEED {
            return ([self.velocity.x, self.velocity.z], speed);
        }
        let forward = self.rotation * Vec3::new(0.0, 0.0, 1.0);
        ([forward.x, forward.z], speed)
    }

    pub(crate) fn target(&self, point: [f32; 3], flag: f32) -> [f32; 5] {
        let delta = self.to_car(Vec3::from(point) - Vec3::from(self.position));
        let distance = delta.dot(delta).sqrt();
        let direction = if distance > 1e-3 { delta * (1.0 / distance) } else { Vec3::ZERO };
        [direction.x, direction.y, direction.z, libm::log1pf(distance) / 5.0, flag]
    }
}

pub(crate) fn hypot(x: f32, y: f32) -> f32 {
    let [x, y] = [x, y].map(f64::from);
    (x * x + y * y).sqrt() as f32
}

pub fn surface_class(material: u8) -> usize {
    match material {
        surface::TURBO
        | surface::TURBO_2
        | surface::TURBO_ROULETTE
        | surface::TURBO_WOOD
        | surface::TURBO_2_WOOD
        | surface::TURBO_TECH_MAGNETIC
        | surface::TURBO_2_TECH_MAGNETIC => 12,
        surface::FREE_WHEELING | surface::FREE_WHEELING_WOOD | surface::FREE_WHEELING_TECH_MAGNETIC => 13,
        surface::NO_GRIP => 14,
        surface::NO_STEERING => 15,
        surface::NO_BRAKES => 16,
        surface::CONCRETE
        | surface::PAVEMENT
        | surface::ASPHALT
        | surface::WET_ASPHALT
        | surface::WET_PAVEMENT => 0,
        surface::DIRT | surface::DIRT_ROAD | surface::WET_DIRT_ROAD | surface::GRAVEL => 1,
        surface::GRASS | surface::WET_GRASS => 2,
        surface::ICE | surface::ROAD_ICE => 3,
        surface::PLASTIC | surface::ROAD_SYNTHETIC => 4,
        surface::WOOD | surface::SLIDING_WOOD => 5,
        surface::METAL
        | surface::RESONANT_METAL
        | surface::METAL_TRANS
        | surface::METAL_FENCE
        | surface::TECH_MAGNETIC
        | surface::TECH_SUPER_MAGNETIC => 6,
        surface::SAND => 7,
        surface::SNOW => 8,
        surface::WATER => 9,
        surface::RUBBER | surface::SLIDING_RUBBER | surface::RUBBER_BAND => 10,
        _ => 11,
    }
}

pub fn observe(session: &Session, course: &Course, ground: &Ground, out: &mut [f32]) {
    let dots = dots(out.len());
    assert!(
        matches!(out.len(), LEN | LEN_V9 | LEN_V10 | LEN_V11 | LEN_V13) || dots.is_some(),
        "observation length"
    );
    out.fill(0.0);
    let (out, extra) = out.split_at_mut(LEN);
    let car = Car::read(session);
    let vehicle = session.vehicle();
    let tuning = &session.config.model(vehicle.model_choice).assets.tuning;
    let velocity = car.to_car(car.velocity);
    out[0..3].copy_from_slice(&(velocity * 0.01).to_array());
    out[3..6].copy_from_slice(&(car.to_car(car.angular_velocity) * 0.1).to_array());
    out[6..9].copy_from_slice(&car.to_car(Vec3::new(0.0, 1.0, 0.0)).to_array());
    out[9] = velocity.x / hypot(velocity.x, velocity.z).max(1.0);
    out[10] = vehicle.rpm / 10_000.0;
    out[11] = vehicle.gear as f32 / 6.0;
    out[12 + vehicle.model_choice as usize] = 1.0;
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
    out[EFFECTS_START..DYNAMICS_START].copy_from_slice(&[
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
    dynamics::read(session, &mut out[DYNAMICS_START..TARGETS_START]);
    out[TARGETS_START..GROUND_START].copy_from_slice(&course.features(&car, &session.race));
    let forward = car.rotation * Vec3::new(0.0, 0.0, 1.0);
    ground.ahead(car.position, [forward.x, forward.z], &mut out[GROUND_START..RAYS_START]);
    if extra.len() != LEN_V9 - LEN {
        rays::read(session, &mut out[RAYS_START..], &mut []);
        if !extra.is_empty() {
            let (driven, extra) = extra.split_at_mut(EVERY_START - DRIVEN_START);
            driven[session.config.model(vehicle.model_choice).car as usize] = 1.0;
            if extra.len() == LEN_V13 - EVERY_START {
                let (every, pose) = extra.split_at_mut(CAR_POSE_START - EVERY_START);
                course.every(&car, &session.race, every);
                pose[..3].copy_from_slice(&car.position);
                for r in 0..3 {
                    pose[3 + 3 * r..6 + 3 * r].copy_from_slice(&car.rotation.rows[r].to_array());
                }
                return;
            }
            if dots.is_some() {
                let (every, dots) = extra.split_at_mut(DOTS_START - EVERY_START);
                course.every(&car, &session.race, every);
                ground.cloud(session).read(session, course, &car, dots);
                return;
            }
            let images = if extra.len() == LEN_V11 - EVERY_START {
                let (every, extra) = extra.split_at_mut(POSE_START - EVERY_START);
                course.every(&car, &session.race, every);
                let (pose, images) = extra.split_at_mut(chart::POSE_LEN);
                pose.copy_from_slice(&ground.chart(session).pose(&car));
                images
            } else {
                let (every, images) = extra.split_at_mut(IMAGES_START - EVERY_START);
                course.every(&car, &session.race, every);
                images
            };
            let map = ground.atlas(session);
            let passed_groups: Vec<bool> =
                map.groups().iter().map(|&group| passed(&session.race, group)).collect();
            map.read(ground, car.position, [forward.x, forward.z], &passed_groups, images);
        }
        return;
    }
    let (travel, extra) = extra.split_at_mut(TRIGGERS_START - LEN);
    let (near, extra) = extra.split_at_mut(RAYS_3D_START - TRIGGERS_START);
    let (rays_3d, state) = extra.split_at_mut(STATE_START - RAYS_3D_START);
    rays::read(session, &mut out[RAYS_START..], rays_3d);
    let (direction, speed) = car.travel();
    ground.travel(car.position, direction, speed, travel);
    triggers::read(&course.triggers, &car, direction, near);
    read_state(session, state);
}

fn read_state(session: &Session, out: &mut [f32]) {
    let vehicle = session.vehicle();
    let tuning = &session.config.model(vehicle.model_choice).assets.tuning;
    let now = session.clock.now;
    out[..3].copy_from_slice(&[vehicle.controls.steer, vehicle.controls.gas, vehicle.controls.brake]);
    out[3..7].copy_from_slice(&vehicle.wheel_layers);
    for (slot, wheel) in out[7..11].iter_mut().zip(&vehicle.wheels) {
        *slot = f32::from(wheel.contact != 0 && magnetic(wheel.material));
    }
    let power = vehicle.level_power(tuning);
    out[11] = if power >= 1.0 {
        0.0
    } else if power >= tuning.level_power_base {
        0.5
    } else {
        1.0
    };
    let duration = vehicle.ground_pull_duration_ms;
    if vehicle.boost_level != 0 && duration > 0 {
        let left = vehicle.boost_last_contact.wrapping_add(duration).wrapping_sub(now) as i32;
        out[12] = left.clamp(0, duration as i32) as f32 / duration as f32;
    }
    if vehicle.frozen_until != u32::MAX {
        out[13] = (vehicle.frozen_until.wrapping_sub(now) as i32).clamp(0, 1000) as f32 * 0.001;
    }
    out[14] = (session.respawn_count() as f32 * 0.1).min(1.0);
}

fn magnetic(material: u8) -> bool {
    matches!(
        material,
        surface::TECH_MAGNETIC
            | surface::TURBO_TECH_MAGNETIC
            | surface::TURBO_2_TECH_MAGNETIC
            | surface::FREE_WHEELING_TECH_MAGNETIC
            | surface::TECH_SUPER_MAGNETIC
            | surface::TECH_MAGNETIC_ACCEL
    )
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

    fn target(group: u32, gates: Vec<[f32; 3]>) -> Target {
        let facings = (0..gates.len()).map(|k| [k as f32, 0.0, 1.0]).collect();
        Target { group, links: gates.len() as u32, gates, facings }
    }

    fn course(laps: u32) -> Course {
        Course {
            checkpoints: vec![
                target(0, vec![[100.0, 0.0, 0.0]]),
                target(1, vec![[0.0, 0.0, 10.0], [0.0, 0.0, -200.0]]),
            ],
            finishes: vec![[0.0, 0.0, 20.0]],
            finish_facings: vec![[0.0, 0.0, 1.0]],
            finish_links: 1,
            laps,
            triggers: Vec::new(),
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
    fn task_potential_counts_valid_groups_and_completed_laps() {
        let course = course(2);
        let mut race = race(2);
        let mut previous = course.potential_at([0.0; 3], &race);
        assert!((-1.0..0.0).contains(&previous));
        for lap in 0..2 {
            for waypoint in [2, 0, 3] {
                race.waypoint(0, None, waypoint, lap * 100 + waypoint * 10 + 10);
                let next = course.potential_at([0.0; 3], &race);
                assert!(next > previous);
                previous = next;
            }
        }
        assert_eq!(previous, 0.0);
    }

    #[test]
    fn task_potential_approaches_any_unpassed_linked_gate_without_counting_it_twice() {
        let course = course(1);
        let mut race = race(1);
        let start = course.potential_at([0.0; 3], &race);
        assert!(course.potential_at([0.0, 0.0, -199.0], &race) > start);
        race.waypoint(0, None, 2, 100);
        let passed = course.potential_at([0.0; 3], &race);
        race.waypoint(0, None, 1, 200);
        assert_eq!(course.potential_at([0.0; 3], &race), passed);
        assert!(course.potential_at([90.0, 0.0, 0.0], &race) > passed);
        assert!(course.potential_at([-1000.0; 3], &race) < passed);
    }

    #[test]
    fn task_potential_ignores_gate_timestamps_and_waiting() {
        let course = course(1);
        let mut early = race(1);
        let mut late = race(1);
        early.waypoint(0, None, 2, 10);
        late.waypoint(0, None, 2, 240_000);
        assert_eq!(course.potential_at([0.0; 3], &early), course.potential_at([0.0; 3], &late));
    }

    #[test]
    fn only_the_four_nearest_groups_fill_the_target_slots() {
        let mut course = course(1);
        for (group, distance) in [(2, 2.0), (3, 30.0), (4, 5.0), (5, 1.0)] {
            course.checkpoints.push(target(group, vec![[0.0, 0.0, distance]]));
        }
        let features = course.features(&car(), &race(1));
        for (slot, distance) in features[..20].as_chunks::<5>().0.iter().zip([1.0_f32, 2.0, 5.0, 10.0]) {
            assert_eq!(&slot[..3], &[0.0, 0.0, 1.0]);
            assert!((slot[3] - libm::log1pf(distance) / 5.0).abs() < 1e-6);
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
    fn every_checkpoint_shows_in_distance_order_whatever_order_the_map_lists_them() {
        let mut course = course(1);
        for (group, at) in
            [(2, [0.0, 0.0, 2.0]), (3, [-30.0, 0.0, 0.0]), (4, [5.0, 3.0, 0.0]), (5, [0.0, 0.0, -2.0])]
        {
            course.checkpoints.push(target(group, vec![at, [at[0], at[1], at[2] + 500.0]]));
        }
        let mut race = race(1);
        race.waypoint(0, None, 1, 10);
        let every = |course: &Course| {
            let mut out = vec![0.0; IMAGES_START - EVERY_START];
            course.every(&car(), &race, &mut out);
            out
        };
        let seen = every(&course);
        let slots = seen.as_chunks::<GATE_LEN>().0;
        assert_eq!(slots[..6].iter().map(|slot| slot[3]).collect::<Vec<_>>(), {
            let mut d = [2.0_f32, 2.0, 5.83, 10.0, 30.0, 100.0].map(|d| libm::log1pf(d) / 5.0).to_vec();
            d[2] = libm::log1pf(34.0_f32.sqrt()) / 5.0;
            d
        });
        assert_eq!(&slots[3][..], &[0.0, 0.0, 1.0, libm::log1pf(10.0) / 5.0, 0.0, 0.0, 1.0, 1.0, 0.5, 1.0]);
        assert!(slots[6..EVERY].iter().all(|slot| slot == &[0.0; GATE_LEN]));
        assert_eq!(&seen[(EVERY + 1) * GATE_LEN..], &[5.0 / 6.0, 6.0 / 32.0, 1.0, 1.0]);
        for turn in 1..course.checkpoints.len() {
            let mut shuffled = course.clone();
            shuffled.checkpoints.rotate_left(turn);
            shuffled.checkpoints.reverse();
            for target in &mut shuffled.checkpoints {
                target.gates.reverse();
                target.facings.reverse();
            }
            assert_eq!(every(&shuffled), seen);
        }
    }

    #[test]
    fn heading_rotates_targets_and_motion_into_the_car_frame() {
        let car = Car { rotation: Mat3::from_array([0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]), ..car() };
        assert_eq!(car.to_car(Vec3::new(50.0, 0.0, 0.0)).to_array(), [0.0, 0.0, 50.0]);
        assert_eq!(&car.target([-100.0, 0.0, 0.0], 1.0)[..3], &[0.0, 0.0, -1.0]);
        assert_eq!(&car.target([0.0; 3], 1.0)[..3], &[0.0; 3]);
    }

    #[test]
    fn travel_follows_the_motion_once_moving_and_the_nose_before() {
        let reversing = Car { velocity: Vec3::new(0.0, -30.0, -20.0), ..car() };
        assert_eq!(reversing.travel(), ([0.0, -20.0], 20.0));
        let creeping = Car { velocity: Vec3::new(-3.0, 0.0, -4.0), ..car() };
        assert_eq!(creeping.travel(), ([0.0, 1.0], 5.0));
    }

    #[test]
    #[ignore = "requires a local real map, record and game packs"]
    fn observing_every_tick_leaves_the_run_unchanged() -> Result<()> {
        let (record, start) = open("GEARBOX_TEST")?;
        let course = Course::race(&start)?;
        let ground = Ground::build(&start);
        let drive = |watch: usize| {
            let mut session = start.clone();
            let mut out = vec![0.0; watch];
            let mut seen = Vec::new();
            for tick in 0..record.run.controls.len() + 100 {
                if watch > 0 {
                    observe(&session, &course, &ground, &mut out);
                    seen.push(out.iter().map(|v| v.to_bits()).collect::<Vec<_>>());
                }
                if !record.step(&mut session, tick) || session.race().and_then(|r| r.finish).is_some() {
                    break;
                }
            }
            let body = session.body();
            let state = [body.pos, body.vel, body.ang_vel].map(|v| v.to_array().map(f32::to_bits));
            (session.race().and_then(|r| r.finish), state, seen)
        };
        let (blind, blind_state, _) = drive(0);
        assert!(blind.is_some());
        for len in [LEN_V9, LEN_V10, LEN_V12] {
            let (finish, state, seen) = drive(len);
            assert_eq!(finish, blind);
            assert_eq!(state, blind_state);
            assert_eq!(seen, drive(len).2);
            if len != LEN_V9 {
                assert_eq!(seen.iter().map(|o| o[..LEN].to_vec()).collect::<Vec<_>>(), drive(LEN).2);
            }
        }
        Ok(())
    }

    fn open(prefix: &str) -> Result<(gearbox_run::Record, Session)> {
        use anyhow::Context;
        let var = |name: &str| std::env::var_os(name).map(std::path::PathBuf::from).context(name.to_owned());
        let map = gearbox_gbx::Map::parse(&std::fs::read(var(&format!("{prefix}_MAP"))?)?)?;
        let record = gearbox_run::Record::read(&var(&format!("{prefix}_RECORD"))?)?;
        let packs = gearbox_packs::install::Game::locate(Some(&var("GEARBOX_TEST_GAME")?))?.open()?;
        let start = record.open(&map, &packs)?;
        Ok((record, start))
    }

    #[test]
    #[ignore = "requires a local real map with effect gates, its record and game packs"]
    fn every_effect_gate_shows_its_type_a_second_before_the_car_touches_it() -> Result<()> {
        let (record, mut session) = open("GEARBOX_TEST_EFFECT")?;
        let course = Course::race(&session)?;
        let ground = Ground::build(&session);
        let mut out = vec![0.0; LEN_V9];
        let mut seen = Vec::new();
        let mut touched = 0;
        for tick in 0..record.run.controls.len() + 100 {
            observe(&session, &course, &ground, &mut out);
            let slots = out[TRIGGERS_START..RAYS_3D_START].as_chunks::<{ triggers::FEATURES }>().0;
            seen.push(
                slots.iter().flat_map(|slot| slot[7..].iter().position(|&v| v == 1.0)).collect::<Vec<_>>(),
            );
            let before = session.vehicle().surface_contact_effect;
            if !record.step(&mut session, tick) || session.race().and_then(|r| r.finish).is_some() {
                break;
            }
            let effect = session.vehicle().surface_contact_effect;
            if effect != 0 && effect != before && session.race_ms() >= 1000 {
                touched += 1;
                let early = &seen[tick + 1 - 100];
                assert!(early.contains(&(effect as usize - 1)), "gate {effect} at {} ms", session.race_ms());
            }
        }
        assert!(touched > 0);
        Ok(())
    }

    #[test]
    #[ignore = "requires a local real map with a loop, its record and game packs"]
    fn upward_rays_reach_the_far_side_of_a_loop() -> Result<()> {
        let (record, mut session) = open("GEARBOX_TEST_LOOP")?;
        let course = Course::race(&session)?;
        let ground = Ground::build(&session);
        let mut out = vec![0.0; LEN_V9];
        let (mut inverted, mut hits) = (0, 0);
        for tick in 0..record.run.controls.len() + 100 {
            let up = session.body().rot * Vec3::new(0.0, 1.0, 0.0);
            if up.y < -0.5 && session.vehicle().wheels.iter().all(|wheel| wheel.contact != 0) {
                observe(&session, &course, &ground, &mut out);
                inverted += 1;
                hits += usize::from(
                    out[RAYS_3D_START + 2 * 12 + 1] == 1.0 && out[RAYS_3D_START + 2 * 12 + 6] == 1.0,
                );
            }
            if !record.step(&mut session, tick) || session.race().and_then(|r| r.finish).is_some() {
                break;
            }
        }
        assert!(inverted > 0 && hits == inverted, "{hits} of {inverted} inverted ticks");
        Ok(())
    }
}
