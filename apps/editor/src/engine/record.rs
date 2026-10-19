use gearbox_engine::Controls;

use crate::engine::input::InputFile;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Keys {
    pub gas: bool,
    pub brake: bool,
    pub left: bool,
    pub right: bool,
    pub half: bool,
}

impl Keys {
    pub fn controls(self) -> Controls {
        let lock = if self.half { 64.0 / 127.0 } else { 1.0 };
        let steer = match (self.left, self.right) {
            (true, false) => -lock,
            (false, true) => lock,
            _ => 0.0,
        };
        Controls { gas: self.gas as u8 as f32, brake: self.brake as u8 as f32, steer }
    }
}

#[derive(Clone, Debug)]
pub struct Recorder {
    pub next: usize,
}

impl Recorder {
    pub fn new(tick: usize) -> Self {
        Self { next: tick }
    }

    pub fn write(&mut self, inputs: &mut InputFile, race_ms0: i32, until: usize, keys: Keys) -> Option<usize> {
        if until <= self.next {
            return None;
        }
        let first = self.next;
        let c = keys.controls();
        for k in first..until {
            inputs.set(race_ms0 + 10 * k as i32, c);
        }
        self.next = until;
        Some(first)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_map_to_digital_controls() {
        let k = |gas, brake, left, right, half| Keys { gas, brake, left, right, half }.controls();
        assert_eq!(k(true, false, false, false, false), Controls { gas: 1.0, brake: 0.0, steer: 0.0 });
        assert_eq!(k(false, true, true, false, false), Controls { gas: 0.0, brake: 1.0, steer: -1.0 });
        assert_eq!(k(true, false, false, true, true).steer, 64.0 / 127.0);
        assert_eq!(k(false, false, true, true, false).steer, 0.0);
    }

    #[test]
    fn recording_overwrites_from_the_cursor_and_extends_the_track() {
        let mut f = InputFile::parse("T0 -100\n0 1 0\n0 1 0\n0 1 0\n").unwrap();
        let mut r = Recorder::new(2);
        let gas_right = Keys { gas: true, right: true, ..Default::default() };
        assert_eq!(r.write(&mut f, -100, 4, gas_right), Some(2));
        assert_eq!(r.write(&mut f, -100, 4, Keys::default()), None);
        assert_eq!(r.write(&mut f, -100, 6, Keys { brake: true, ..Default::default() }), Some(4));
        assert_eq!(f.to_text(), "T0 -100\n0 1 0\n0 1 0\n1 0 1\n1 0 1\n0 1 0\n0 1 0\n");
        assert_eq!(r.next, 6);
    }
}
