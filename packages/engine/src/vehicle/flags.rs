#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct EffectFlags(pub u8);
impl EffectFlags {
    pub const MODE: Self = Self(3);
    pub const BRAKES_DISABLED: Self = Self(4);
    pub const STEERING_DISABLED: Self = Self(8);
    pub const FRAGILE: Self = Self(16);
    pub fn contains_in(self, bits: u8) -> bool { bits & self.0 == self.0 }
    pub fn set_in(self, bits: &mut u8, enabled: bool) {
        if enabled { *bits |= self.0; } else { *bits &= !self.0; }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct BodyFlags(pub u8);
impl BodyFlags {
    const ENABLED: Self = Self(1);
    pub fn enabled(self) -> bool { self.0 & Self::ENABLED.0 != 0 }
    pub fn with_enabled(self, enabled: bool) -> Self {
        Self(self.0 & !Self::ENABLED.0 | if enabled { Self::ENABLED.0 } else { 0 })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct WheelFlags(pub u32);
impl WheelFlags {
    pub const WET: Self = Self(2);
    pub const CONTACT: Self = Self(4);
    pub const BOOST: Self = Self(8);
    pub fn contains(self, flag: Self) -> bool { self.0 & flag.0 == flag.0 }
    pub fn set(&mut self, flag: Self, enabled: bool) {
        if enabled { self.0 |= flag.0; } else { self.0 &= !flag.0; }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct ForcedEffects(pub u32);
impl ForcedEffects {
    const WHEEL_METER: Self = Self(512);
    pub fn wheel_meter(self) -> bool { self.0 & Self::WHEEL_METER.0 != 0 }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct SlowStartFlags(pub u8);
impl SlowStartFlags {
    pub const FRONT: Self = Self(1);
    pub const REAR: Self = Self(2);
    pub fn insert(&mut self, flag: Self) { self.0 |= flag.0; }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct ToggleFlags(pub u32);
impl ToggleFlags {
    pub const ENABLED: Self = Self(1);
    pub fn enabled(self) -> bool { self.0 & Self::ENABLED.0 != 0 }
    pub fn enable(&mut self) { self.0 |= Self::ENABLED.0; }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct ActionFlags(pub u32);
impl ActionFlags {
    pub const SECONDARY: Self = Self(2);
    pub const TRANSIENT: Self = Self(1024);
    pub fn contains(self, flag: Self) -> bool { self.0 & flag.0 == flag.0 }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct SurfaceFlags(pub u8);
impl SurfaceFlags {
    pub const CONTACT: Self = Self(4);
    pub const VISUAL_CONTACTS: Self = Self(12);
    pub fn intersects(self, flags: Self) -> bool { self.0 & flags.0 != 0 }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct ImpactFlags(pub u32);
impl ImpactFlags {
    pub const DEPTH: Self = Self(1);
    pub fn insert(&mut self, flag: Self) { self.0 |= flag.0; }
}
