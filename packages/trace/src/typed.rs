use std::path::{Path, PathBuf};
use crate::{Engine, Factory, Setup, Error, Controls, TickState, Body, Vehicle, Wheel, Race, F32};

pub struct TypedFactory {
    pub packs: PathBuf,
}
impl TypedFactory {
    pub fn new(packs: PathBuf) -> Self { Self { packs } }

    pub fn session(&self, map: &Path, t0: i32, seed: Option<u32>, packs: Option<&Path>)
        -> Result<gearbox_engine::Session, Error>
    {
        Ok(gearbox_world::open_file(map, packs.unwrap_or(&self.packs),
            gearbox_world::OpenOptions { t0, seed, car: None })?)
    }
}
impl Factory for TypedFactory {
    fn name(&self) -> String { "typed".into() }
    fn open(&self, setup: &Setup) -> Result<Box<dyn Engine>, Error> {
        Ok(Box::new(self.session(&setup.map, setup.t0, setup.seed, setup.packs.as_deref())?))
    }
}

impl Engine for gearbox_engine::Session {
    fn step(&mut self, controls: Controls) {
        self.step(gearbox_engine::Controls {
            gas: controls.gas, brake: controls.brake, steer: controls.steer,
        });
    }

    fn state(&mut self) -> TickState {
        state(self)
    }
}

pub fn state(session: &gearbox_engine::Session) -> TickState {
    let body = session.body();
    let car = session.vehicle();
    let wheels = std::array::from_fn(|index| {
        let wheel = &car.wheels[index];
        Wheel {
            contact: wheel.contact, material: wheel.material,
            spin: wheel.spin_angle.into(), spin_rate: wheel.spin.into(),
            steer: wheel.steering_angle.into(), suspension: wheel.suspension_length.into(),
            position: car.wheel_poses[index].transform.pos.to_array().map(F32::new),
        }
    });
    let race = session.race().map(|race| Race {
        checkpoints: race.checkpoints, laps: race.laps, finish: race.finish,
        waypoints: session.crossing_waypoints(),
    });
    TickState {
        race_ms: session.race_ms(),
        body: Body {
            rotation: body.rot.to_array().map(F32::new),
            position: body.pos.to_array().map(F32::new),
            quaternion: body.quat.to_array().map(F32::new),
            linear_velocity: body.vel.to_array().map(F32::new),
            angular_velocity: body.ang_vel.to_array().map(F32::new),
        },
        vehicle: Vehicle { contact_flags: car.flags,
            contact_normal: car.contact_normal.to_array().map(F32::new),
            rpm: car.rpm.into(), gear: car.gear, wheels },
        race,
    }
}
