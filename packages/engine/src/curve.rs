const EPS: f32 = 0.00001;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Key {
    pub x: f32,
    pub y: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Curve {
    pub flags: u32,
    pub keys: Vec<Key>,
}

impl Curve {
    pub const STEP: u32 = 1;
    pub const SMOOTH: u32 = 2;
    pub const SPLINE: u32 = 0x10;

    pub fn new(flags: u32, keys: Vec<Key>) -> Self {
        Self { flags, keys }
    }

    pub fn eval(&self, x: f32) -> f32 {
        let k = &self.keys;
        match k.len() {
            0 => return 0.0,
            1 => return k[0].y,
            _ => {}
        }
        if !(k[0].x + EPS <= x) {
            return k[0].y;
        }
        let last = k.len() - 1;
        if !(x <= k[last].x - EPS) {
            return k[last].y;
        }
        let mut i = 1;
        let mut x0 = k[0].x;
        let mut x1 = k[1].x;
        while x1 + EPS < x {
            i += 1;
            x0 = x1;
            x1 = k[i].x;
        }
        if EPS <= (x1 - x0).abs() && self.flags & Self::STEP == 0 {
            let mut t = (x - x0) / (x1 - x0);
            if self.flags & Self::SMOOTH == 0 {
                assert!(self.flags & Self::SPLINE == 0, "spline curves are not supported");
            } else {
                t = t * 3.0 * t - (t + t) * t * t;
            }
            return (1.0 - t) * k[i - 1].y + t * k[i].y;
        }
        k[i - 1].y
    }

    pub fn eval_step(&self, x: f32) -> f32 {
        let k = &self.keys;
        match k.len() {
            0 => return 0.0,
            1 => return k[0].y,
            _ => {}
        }
        if k[0].x + EPS > x {
            return k[0].y;
        }
        let last = k.len() - 1;
        if x > k[last].x - EPS {
            return k[last].y;
        }
        let mut i = 1;
        while x > k[i].x + EPS {
            i += 1;
        }
        k[i - 1].y
    }
}
