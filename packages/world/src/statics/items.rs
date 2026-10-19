#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ItemSpec {
    pub entity: gearbox_engine::static_world::Entity,
    pub shape: u32,
    pub transform: gearbox_engine::Iso,
    pub layer: u8,
    pub flags: u8,
}
