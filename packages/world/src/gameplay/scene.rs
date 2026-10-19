use gearbox_engine::arena::Scene;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntityKind {
    Node,
    Vehicle,
    Character,
}
impl EntityKind {
    pub fn is_kind(self, expected: Self) -> bool {
        self == expected || self == Self::Character && expected == Self::Node
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EntityBinding {
    pub handle: u32,
    pub kind: EntityKind,
    pub index: u32,
}

pub(super) fn build(now: u32) -> Scene {
    Scene { updates_collision: true, time: now, ..Scene::default() }
}
