#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct BodyOptions(u8);

impl BodyOptions {
    pub const FIXED: Self = Self(1);
    pub const SPECIAL: Self = Self(2);
    pub const NO_STATIC_COLLISION: Self = Self(4);
    pub const KINEMATIC: Self = Self(8);

    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }

    pub fn set(&mut self, flag: Self, enabled: bool) {
        if enabled {
            self.0 |= flag.0;
        } else {
            self.0 &= !flag.0;
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct SolverFlags(u8);

impl SolverFlags {
    pub const EXTERNAL: Self = Self(1);
    pub const SKIP_FORCES: Self = Self(2);
    pub const SKIP_STATIC_COLLISION: Self = Self(4);
    pub const SPECIAL_STEP: Self = Self(8);
    pub const SKIP_INTEGRATION: Self = Self(16);
    pub const SPECIAL: Self = Self(32);
    pub const IN_WATER: Self = Self(64);

    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }

    pub fn set(&mut self, flag: Self, enabled: bool) {
        if enabled {
            self.0 |= flag.0;
        } else {
            self.0 &= !flag.0;
        }
    }
}
