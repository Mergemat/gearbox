use super::{Vehicle, race_flags::RaceFlags};

impl Vehicle {
    pub fn reset_step(&mut self) {
        self.tag = u8::MAX;
        self.zone_mode = 0;
        self.zone_bounds = 0;
        self.flags = RaceFlags(self.flags).reset_step().0;
        self.pitch_input_active = 0;
        self.angular_control_scratch = 0;
        self.pitch_input_direction = 0.0;
        self.yaw_input_direction = 0.0;
        self.surface_contact_effect = 0;
        self.surface_contact_item = None;
        self.controls.actions &= !super::flags::ActionFlags::TRANSIENT.0;
        self.controls.auxiliary_bytes[0] &= !(super::flags::ToggleFlags::ENABLED.0 as u8);
    }
}

pub fn process_contact_points(points: usize, events: usize, impacts: usize) {
    if points != 0 {
        todo!("queued contact points");
    }
    if events != 0 {
        todo!("contact events");
    }
    if impacts != 0 {
        todo!("impacts");
    }
}

pub fn parts_update(count: usize) {
    if count != 0 {
        todo!("breakable parts");
    }
}
