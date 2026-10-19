use crate::Body;
use gearbox_engine::Session;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Events {
    pub bugslide_ticks: u32,
    pub nosebug_ticks: u32,
    pub uberbug_ticks: u32,
}
impl Events {
    pub fn total(self) -> u32 {
        self.bugslide_ticks + self.nosebug_ticks + self.uberbug_ticks
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum TrickKind {
    Nosebug,
    Bugslide,
    Uberbug,
    Any,
}

impl Events {
    pub fn ticks(self, kind: TrickKind) -> u32 {
        match kind {
            TrickKind::Nosebug => self.nosebug_ticks,
            TrickKind::Bugslide => self.bugslide_ticks,
            TrickKind::Uberbug => self.uberbug_ticks,
            TrickKind::Any => self.total(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Observation {
    pub speed: f64,
    pub slip: f64,
    pub pitch: f32,
    pub wheels: [bool; 4],
    pub body_contact: bool,
    pub wall_contact: bool,
}
impl Observation {
    pub fn read(native: &mut Session, body: Body) -> Self {
        let v = body.local_velocity();
        let speed = v.iter().map(|v| f64::from(*v).powi(2)).sum::<f64>().sqrt();
        let heading = f64::from(body.0[2]).atan2(f64::from(body.0[8]));
        let travel = f64::from(v[0]).atan2(f64::from(v[2]));
        let slip = (travel - heading).sin().atan2((travel - heading).cos()).abs();
        let car = native.vehicle();
        let wheels = std::array::from_fn(|i| car.wheels[i].contact != 0);
        const HARD_CONTACT: u32 = 0x200;
        let body_contact = car.flags & HARD_CONTACT != 0;
        let normal = car.contact_normal.to_array();
        let norm = normal.iter().map(|v| v * v).sum::<f32>().sqrt();
        let wall_contact = body_contact && normal[1].abs() < norm * 0.5;
        Self { speed, slip, wall_contact, pitch: body.0[5].clamp(-1.0, 1.0).asin(), wheels, body_contact }
    }
    pub fn sliding(self) -> bool {
        self.speed > 15.0 && self.slip > 40f64.to_radians() && self.wheels.iter().any(|v| *v)
    }
}

#[derive(Clone, Default)]
pub struct Detector {
    previous: Option<Observation>,
    sliding_ticks: u32,
    slide_start_speed: f64,
    pub events: Events,
}
impl Detector {
    pub fn observe(&mut self, now: Observation) {
        if let Some(prev) = self.previous {
            if now.sliding() {
                if self.sliding_ticks == 0 {
                    self.slide_start_speed = prev.speed;
                }
                self.sliding_ticks += 1;
                if self.sliding_ticks >= 10 && now.speed >= self.slide_start_speed * 0.98 {
                    self.events.bugslide_ticks += 1;
                }
            } else {
                self.sliding_ticks = 0;
            }
            if now.pitch < -35f32.to_radians()
                && (now.wheels[0] || now.wheels[1])
                && !now.wheels[2]
                && !now.wheels[3]
                && now.speed > 5.0
            {
                self.events.nosebug_ticks += 1;
            }
            if (now.body_contact || now.wheels.iter().any(|w| *w))
                && now.speed - prev.speed > 5.0
                && now.speed > prev.speed * 1.10
            {
                self.events.uberbug_ticks += 1;
            }
        }
        self.previous = Some(now);
    }
}
