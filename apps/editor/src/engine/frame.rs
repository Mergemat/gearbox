use gearbox_engine::{Controls, Session};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wheel {
    pub contact: bool,
    pub material: u8,
    pub spin: f32,
    pub steer: f32,
    pub position: [f32; 3],
}

#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub race_ms: i32,
    pub body: [f32; 22],
    pub wheels: [Wheel; 4],
    pub waypoints: u16,
    pub finish: Option<u32>,
    pub last_crossing: Option<(u32, u32)>,
    pub controls: Controls,
}

impl Frame {
    pub fn read(runner: &Session, controls: Controls) -> Self {
        let body = gearbox_search::Body::from(runner.body()).0;
        let car = runner.vehicle();
        let wheels = std::array::from_fn(|i| {
            let w = &car.wheels[i];
            Wheel {
                contact: w.contact != 0,
                material: w.material,
                spin: w.spin_angle,
                steer: w.steering_angle,
                position: car.wheel_poses[i].transform.pos.to_array(),
            }
        });
        let race = runner.race();
        let (waypoints, finish, last_ms) = race.map_or((0, None, None), |r| {
            (r.checkpoints.len() as u16, r.finish, r.checkpoints.last().copied())
        });
        let last_crossing = last_ms.and_then(|t| {
            runner.crossing_waypoint(waypoints as usize).map(|w| (w, t))
        });
        Self { race_ms: runner.race_ms(), body, wheels, waypoints, finish, last_crossing, controls }
    }

    pub fn position(&self) -> [f32; 3] {
        [self.body[9], self.body[10], self.body[11]]
    }

    pub fn rotation(&self) -> [f32; 9] {
        self.body[..9].try_into().unwrap()
    }

    pub fn velocity(&self) -> [f32; 3] {
        [self.body[16], self.body[17], self.body[18]]
    }

    pub fn angular_velocity(&self) -> [f32; 3] {
        [self.body[19], self.body[20], self.body[21]]
    }

    pub fn speed(&self) -> f32 {
        let v = self.velocity();
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }

    pub fn speed_kmh(&self) -> f32 {
        self.speed() * 3.6
    }

    pub fn axis(&self, k: usize) -> [f32; 3] {
        let r = self.rotation();
        [r[k], r[3 + k], r[6 + k]]
    }

    pub fn yaw_pitch_roll(&self) -> [f32; 3] {
        let f = self.axis(2);
        let u = self.axis(1);
        let yaw = f[0].atan2(f[2]).to_degrees();
        let pitch = f[1].clamp(-1.0, 1.0).asin().to_degrees();
        let s = self.axis(0);
        let roll = (-s[1]).atan2(u[1]).to_degrees();
        [yaw, pitch, roll]
    }

}
