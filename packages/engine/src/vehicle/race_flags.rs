#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RaceFlags(pub u32);
impl RaceFlags {
    pub const PLAYER: Self = Self(1);
    pub const PASSIVE: Self = Self(2);
    pub const MUTED: Self = Self(4);
    pub const ACTIVE: Self = Self(8);
    pub const STEP: Self = Self(16);
    pub const FINISHED: Self = Self(0x100);
    pub const PLAYER_STEP: Self = Self(0x2000);
    pub const GHOST: Self = Self(0x4000);
    pub const GHOST_RESPONSE: Self = Self(0x1_0000);
    const WAYPOINTS: u32 = 0xe0;
    pub fn has_waypoints(self) -> bool { self.0 & Self::WAYPOINTS != 0 }
    pub fn clear_waypoints(self) -> Self { Self(self.0 & !(Self::WAYPOINTS | Self::FINISHED.0)) }
    pub fn contains(self, flags: Self) -> bool { self.0 & flags.0 == flags.0 }
    pub fn set(&mut self, flags: Self, enabled: bool) {
        if enabled { self.0 |= flags.0; } else { self.0 &= !flags.0; }
    }
    pub fn count_waypoint(self, finish: bool) -> Self {
        Self((self.0.wrapping_add(0x20) & Self::WAYPOINTS)
            | if finish { Self::FINISHED.0 } else { 0 }
            | self.0 & !(Self::WAYPOINTS | Self::FINISHED.0))
    }
    pub fn begin(self) -> Self {
        Self((self.0 & !(Self::PLAYER.0 | Self::PASSIVE.0 | Self::MUTED.0 | Self::GHOST.0))
            | Self::ACTIVE.0 | Self::STEP.0)
    }
    pub fn reset_step(self) -> Self {
        const TRANSIENT: RaceFlags = RaceFlags(122907);
        Self(self.0 & !TRANSIENT.0)
    }
    pub fn reset(self) -> Self {
        Self(self.0 & !(Self::MUTED.0 | Self::WAYPOINTS | Self::FINISHED.0 | Self::GHOST_RESPONSE.0))
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputFlags(pub u32);
impl InputFlags {
    const TRANSIENT: Self = Self(3);
    pub fn cleared(self) -> Self { Self(self.0 & !Self::TRANSIENT.0) }
}
