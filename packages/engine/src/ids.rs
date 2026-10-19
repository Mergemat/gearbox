macro_rules! ids {
    ($($name:ident),* $(,)?) => {$(
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(pub u32);

        impl $name {
            pub const fn new(index: u32) -> Self { Self(index) }
            pub const fn index(self) -> usize { self.0 as usize }
        }
    )*};
}

ids!(
    VehicleId,
    SharedShapeId,
    SphereId,
    EllipsoidId,
    BoxId,
    CompoundInstanceId,
    StaticItemId,
    DynamicItemId,
    SurfaceId,
    ShapeEntryId,
    ContinuousEntryId
);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShapeId {
    Shared(SharedShapeId),
    Sphere(SphereId),
    Ellipsoid(EllipsoidId),
    Box(BoxId),
    Compound(CompoundInstanceId),
}

impl Default for ShapeId {
    fn default() -> Self {
        Self::Shared(SharedShapeId::default())
    }
}

impl ShapeId {
    pub fn shared(self) -> Option<SharedShapeId> {
        if let Self::Shared(id) = self { Some(id) } else { None }
    }

    pub fn compound(self) -> Option<CompoundInstanceId> {
        if let Self::Compound(id) = self { Some(id) } else { None }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ItemId {
    Static(StaticItemId),
    Dynamic(DynamicItemId),
}

impl ItemId {
    pub fn static_item(self) -> Option<StaticItemId> {
        if let Self::Static(id) = self { Some(id) } else { None }
    }

    pub fn dynamic_item(self) -> Option<DynamicItemId> {
        if let Self::Dynamic(id) = self { Some(id) } else { None }
    }
}

impl From<StaticItemId> for ItemId {
    fn from(id: StaticItemId) -> Self {
        Self::Static(id)
    }
}

impl From<DynamicItemId> for ItemId {
    fn from(id: DynamicItemId) -> Self {
        Self::Dynamic(id)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EntryId {
    Surface(SurfaceId),
    Discrete(ShapeEntryId),
    Continuous(ContinuousEntryId),
}
