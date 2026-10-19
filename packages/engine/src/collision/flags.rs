macro_rules! flags {
    ($(#[$doc:meta])* $name:ident, $bits:ty) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        #[repr(transparent)]
        pub struct $name($bits);
        impl $name {
            pub const fn from_bits(bits: $bits) -> Self { Self(bits) }
            pub const fn bits(self) -> $bits { self.0 }
            pub const fn contains(self, flag: Self) -> bool { self.0 & flag.0 == flag.0 }
            pub const fn intersects(self, flags: Self) -> bool { self.0 & flags.0 != 0 }
            pub fn set(&mut self, flag: Self, enabled: bool) {
                if enabled { self.0 |= flag.0; } else { self.0 &= !flag.0; }
            }
        }
    };
}

flags!(Lists, u8);
impl Lists {
    pub const SURFACES: Self = Self(1);
    pub const CONTINUOUS: Self = Self(2);
    pub const DISCRETE: Self = Self(4);
    pub const ALL: Self = Self(7);
}

flags!(ItemFlags, u32);
impl ItemFlags {
    pub const NONE: Self = Self(0);
}

flags!(ItemEvents, u8);
impl ItemEvents {
    pub const NONE: Self = Self(0);
    pub const CONTACT_TIME: Self = Self(2);
}

flags!(ContactFlags, u32);
impl ContactFlags {
    pub const FACE: Self = Self(1);
    pub const OTHER_POINT: Self = Self(2);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct SkipMaterial(u8);
impl SkipMaterial {
    pub const NONE: Self = Self(255);
    pub const fn from_id(material: u8) -> Self {
        Self(material)
    }
    pub const fn id(self) -> u8 {
        self.0
    }
    pub const fn skips(self, material: u8) -> bool {
        self.0 == material
    }
}
