pub mod buoyancy;
pub mod drag;
mod query;
pub mod surface;

use crate::world_water::Water;

#[derive(Clone, Copy, Debug)]
pub struct WaterManager<'a> {
    pub(crate) water: &'a Water,
}

impl<'a> WaterManager<'a> {
    pub const fn new(water: &'a Water) -> Self {
        Self { water }
    }
}
