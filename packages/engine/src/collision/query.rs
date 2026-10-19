use super::{CollisionWorld, ContactList, ContactPool, ItemFlags, Lists, ShapeEntry, SkipMaterial, Surface};
use crate::shape::Geometry;
use crate::tree::{AabbQuery, overlap};
use crate::{
    Aabb, ContinuousEntryId, DynamicItemId, EntryId, Iso, ItemId, ShapeEntryId, ShapeId, SharedShapeId,
    StaticItemId, SurfaceId,
};

pub const QUERY_CAPACITY: usize = 1000;
const CLOSE_MERGE_LIMIT: usize = 40;
const CLOSE_MERGE_MIN_DOT: f32 = 0.9396926;
const CLOSE_MERGE_DISTANCE: f32 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QueryFilter {
    pub skip_material: SkipMaterial,
    pub two_sided: bool,
    pub lists: Lists,
    pub layers: u32,
    pub excluded_flags: ItemFlags,
    pub ignored: [Option<ItemId>; 2],
}

impl Default for QueryFilter {
    fn default() -> Self {
        Self {
            skip_material: SkipMaterial::NONE,
            two_sided: false,
            lists: Lists::ALL,
            layers: u32::MAX,
            excluded_flags: ItemFlags::NONE,
            ignored: [None; 2],
        }
    }
}

impl QueryFilter {
    pub fn accepts(self, geometry: Geometry<'_>, item: Option<ItemId>) -> bool {
        let Some(id) = item else {
            return false;
        };
        let item_view = geometry.item(id);
        let bit = if item_view.layer == 0 { 0 } else { 1u32 << ((item_view.layer - 1) & 31) };
        self.layers & bit != 0
            && !self.excluded_flags.intersects(item_view.flags)
            && !self.ignored.contains(&item)
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct QueryResults {
    pub surfaces: Vec<EntryId>,
    pub continuous: Vec<EntryId>,
    pub discrete: Vec<EntryId>,
}

impl QueryResults {
    pub fn is_empty(&self) -> bool {
        self.surfaces.is_empty() && self.continuous.is_empty() && self.discrete.is_empty()
    }
    pub fn iter(&self) -> impl Iterator<Item = EntryId> + '_ {
        self.surfaces.iter().chain(&self.continuous).chain(&self.discrete).copied()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactShape {
    pub shape: ShapeId,
    pub first_entry: ShapeEntryId,
    pub capacity: usize,
}

#[derive(Clone, Copy)]
pub struct ItemView {
    pub layer: u8,
    pub flags: ItemFlags,
    pub id: u32,
    pub iso: Iso,
    pub body: u32,
    pub dynamic: bool,
}

impl<'a> Geometry<'a> {
    pub fn item(self, id: ItemId) -> ItemView {
        match id {
            ItemId::Static(id) => {
                let item = &self.world.statics.items[id.index()];
                ItemView {
                    layer: item.layer,
                    flags: item.query_flags,
                    id: item.id,
                    iso: item.iso,
                    body: u32::MAX,
                    dynamic: false,
                }
            }
            ItemId::Dynamic(id) => {
                let item = &self.state.dynamic_item[id.index()];
                ItemView {
                    layer: item.layer,
                    flags: item.flags,
                    id: item.id,
                    iso: item.iso,
                    body: item.body,
                    dynamic: item.kind != 0,
                }
            }
        }
    }

    pub fn entry(self, id: EntryId) -> Surface {
        match id {
            EntryId::Surface(id) => {
                let surface = &self.world.statics.surfaces[id.index()];
                Surface {
                    shape: ShapeId::Shared(SharedShapeId(surface.shape)),
                    iso: surface.iso,
                    item: Some(StaticItemId(surface.item).into()),
                    tag: surface.tag,
                }
            }
            EntryId::Discrete(id) => {
                let entry = &self.state.shape_entry[id.index()];
                Surface { shape: entry.shape, iso: entry.iso, item: entry.item, tag: entry.tag }
            }
            EntryId::Continuous(id) => {
                let entry = &self.state.continuous_entry[id.index()];
                Surface { shape: entry.shape, iso: entry.iso, item: entry.item, tag: entry.tag }
            }
        }
    }

    pub fn entry_item(self, id: EntryId) -> ItemView {
        self.item(self.entry(id).item.expect("entry without item"))
    }

    pub(crate) fn placed_entries(
        self,
        range: std::ops::Range<u32>,
    ) -> impl Iterator<Item = (EntryId, &'a ShapeEntry)> {
        let entries = &self.state.shape_entry;
        range.map(move |index| (EntryId::Discrete(ShapeEntryId(index)), &entries[index as usize]))
    }
}

impl CollisionWorld {
    pub fn query_all(&self, geometry: Geometry<'_>, bounds: Aabb, filter: QueryFilter) -> Vec<EntryId> {
        let mut out = Vec::new();
        self.collect_warp(geometry, bounds, filter, &mut out);
        for (range, nodes) in self.surface_trees(geometry) {
            let start = range.start;
            let count = range.end - range.start;
            for id in AabbQuery::new(nodes, bounds) {
                assert!(id < count, "tree leaf index outside surface list");
                let r = EntryId::Surface(SurfaceId(start + id));
                if filter.accepts(geometry, geometry.entry(r).item) {
                    if out.len() >= QUERY_CAPACITY {
                        break;
                    }
                    out.push(r);
                }
            }
        }
        out
    }

    pub fn query_box(&self, geometry: Geometry<'_>, bounds: Aabb, filter: QueryFilter) -> QueryResults {
        let surfaces = if filter.lists.contains(Lists::SURFACES) {
            self.query_all(geometry, bounds, filter)
        } else {
            Vec::new()
        };
        let mut continuous = Vec::new();
        if filter.lists.contains(Lists::CONTINUOUS) && !self.continuous_entries.is_empty() {
            for i in self.continuous_entries.clone() {
                let entry = &geometry.state.continuous_entry[i as usize];
                if filter.accepts(geometry, entry.item) && overlap(entry.bounds, bounds) {
                    if continuous.len() >= QUERY_CAPACITY {
                        break;
                    }
                    continuous.push(EntryId::Continuous(ContinuousEntryId(i)));
                }
            }
        }
        let mut discrete = Vec::new();
        if filter.lists.contains(Lists::DISCRETE) {
            for (r, entry) in geometry.placed_entries(self.shape_entries.clone()) {
                if filter.accepts(geometry, entry.item) && overlap(entry.bounds, bounds) {
                    if discrete.len() >= QUERY_CAPACITY {
                        break;
                    }
                    discrete.push(r);
                }
            }
        }
        QueryResults { surfaces, continuous, discrete }
    }

    pub fn shape_contacts(
        &self,
        geometry: Geometry<'_>,
        source: EntryId,
        shape: &Surface,
        candidates: &QueryResults,
        out: &mut ContactList,
    ) {
        if self.post_filter != 0 {
            todo!("narrowphase post-filter");
        }
        let mut pool = ContactList::default();
        {
            let mut sink = ContactPool::new(&mut pool, &mut out.overflow);
            for other in candidates.iter() {
                let b = geometry.entry(other);
                super::narrow::contacts(geometry, source, shape, other, &b, &mut sink);
            }
        }
        out.merge(pool.as_mut_slice(), self.merge_pass);
    }

    pub fn contacts(
        &self,
        geometry: Geometry<'_>,
        shape: ContactShape,
        iso: Iso,
        filter: QueryFilter,
    ) -> ContactList {
        let entries = geometry.entries(shape.shape, iso, None, u32::MAX as u64);
        if entries.is_empty() {
            return ContactList::new(shape.capacity);
        }
        let first = shape.first_entry.0;
        let entries = entries
            .iter()
            .enumerate()
            .map(|(index, entry)| (EntryId::Discrete(ShapeEntryId(first + index as u32)), entry));
        self.contacts_from_entries(geometry, entries, filter, shape.capacity)
    }

    pub fn contacts_for_item(
        &self,
        geometry: Geometry<'_>,
        item: DynamicItemId,
        filter: QueryFilter,
        capacity: usize,
    ) -> ContactList {
        let mut out = ContactList::new(capacity);
        self.append_contacts_for_item(geometry, item, filter, &mut out);
        out
    }

    pub fn append_contacts_for_item(
        &self,
        geometry: Geometry<'_>,
        item: DynamicItemId,
        filter: QueryFilter,
        out: &mut ContactList,
    ) {
        let item = &geometry.state.dynamic_item[item.index()];
        let first =
            if item.entry_count == 0 { 0 } else { self.shape_entries.start + item.entry_start as u32 };
        let entries = geometry.placed_entries(first..first + item.entry_count as u32);
        self.append_contacts_from_entries(geometry, entries, filter, out);
    }

    fn contacts_from_entries<'a>(
        &self,
        geometry: Geometry<'_>,
        entries: impl Iterator<Item = (EntryId, &'a ShapeEntry)>,
        filter: QueryFilter,
        capacity: usize,
    ) -> ContactList {
        let mut out = ContactList::new(capacity);
        self.append_contacts_from_entries(geometry, entries, filter, &mut out);
        out
    }

    fn append_contacts_from_entries<'a>(
        &self,
        geometry: Geometry<'_>,
        entries: impl Iterator<Item = (EntryId, &'a ShapeEntry)>,
        filter: QueryFilter,
        out: &mut ContactList,
    ) {
        let before = out.len();
        let entries: Vec<_> = entries.collect();
        let bounds = crate::tree::union(entries.iter().map(|(_, entry)| entry.bounds));
        let mut candidates = self.query_box(geometry, bounds, filter);
        crate::numeric::quicksort(&mut candidates.surfaces, |a, b| {
            super::item_compare(geometry, &geometry.entry(*a), &geometry.entry(*b))
        });
        for (source, entry) in entries {
            let surface = Surface { shape: entry.shape, iso: entry.iso, item: entry.item, tag: entry.tag };
            self.shape_contacts(geometry, source, &surface, &candidates, out);
        }
        if out.len() != before && out.len() < CLOSE_MERGE_LIMIT {
            out.merge_close(geometry, CLOSE_MERGE_MIN_DOT, CLOSE_MERGE_DISTANCE);
        }
    }

    pub(crate) fn collect_warp(
        &self,
        geometry: Geometry<'_>,
        bounds: Aabb,
        filter: QueryFilter,
        out: &mut Vec<EntryId>,
    ) {
        if !filter.accepts(geometry, self.warp_item) || crate::tree::inside(bounds, self.warp_bounds) {
            return;
        }
        let first = self.warp_surfaces.start;
        let count = self.warp_surfaces.len();
        for (i, &warp) in self.warp_boxes[..count].iter().enumerate() {
            if overlap(warp, bounds) {
                if out.len() >= QUERY_CAPACITY {
                    break;
                }
                out.push(EntryId::Surface(SurfaceId(first + i as u32)));
            }
        }
    }

    pub(crate) fn surface_trees<'a>(
        &'a self,
        geometry: Geometry<'a>,
    ) -> [(std::ops::Range<u32>, &'a [crate::arrays::TreeNode]); 3] {
        [
            (0..geometry.world.statics.surfaces.len() as u32, &geometry.world.statics.tree.nodes),
            (self.dynamic_surfaces.clone(), &self.dynamic_tree_nodes),
            (self.updated_surfaces.clone(), &self.updated_tree_nodes),
        ]
    }
}
