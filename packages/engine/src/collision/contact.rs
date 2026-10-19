use crate::collision::ContactFlags;
use crate::{EntryId, Iso, Vec3};
use super::{Contact, Geometry};

pub const CONTACT_CAPACITY: usize = 0x400;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactSlot { Record(usize), Overflow }

#[derive(Clone, Debug, PartialEq)]
pub struct ContactList {
    records: Vec<Contact>,
    used: usize,
    capacity: usize,
    pub(super) overflow: Contact,
}

pub struct ContactPool<'a> {
    list: &'a mut ContactList,
    overflow: &'a mut Contact,
}

impl<'a> ContactPool<'a> {
    pub fn new(list: &'a mut ContactList, overflow: &'a mut Contact) -> Self { Self { list, overflow } }
    pub fn len(&self) -> usize { self.list.len() }
    pub fn is_empty(&self) -> bool { self.list.is_empty() }
    pub fn clear(&mut self) { self.list.clear(); }

    pub fn take(&mut self) -> ContactSlot { self.list.take() }
    pub fn get(&self, slot: ContactSlot) -> &Contact {
        match slot { ContactSlot::Record(i) => &self.list.records[i], ContactSlot::Overflow => self.overflow }
    }
    pub fn get_mut(&mut self, slot: ContactSlot) -> &mut Contact {
        match slot {
            ContactSlot::Record(index) => &mut self.list.records[index],
            ContactSlot::Overflow => self.overflow,
        }
    }
}

pub trait ContactSink { fn take_contact(&mut self) -> &mut Contact; }

impl ContactSink for ContactList {
    fn take_contact(&mut self) -> &mut Contact { let slot = self.take(); self.get_mut(slot) }
}
impl ContactSink for ContactPool<'_> {
    fn take_contact(&mut self) -> &mut Contact { let slot = self.take(); self.get_mut(slot) }
}

impl Default for ContactList {
    fn default() -> Self { Self::new(CONTACT_CAPACITY) }
}

impl ContactList {
    pub fn new(capacity: usize) -> Self {
        Self { records: Vec::new(), used: 0, capacity, overflow: Contact::default() }
    }

    pub fn as_slice(&self) -> &[Contact] { &self.records[..self.used] }
    pub fn as_mut_slice(&mut self) -> &mut [Contact] { &mut self.records[..self.used] }
    pub fn len(&self) -> usize { self.used }
    pub fn is_empty(&self) -> bool { self.used == 0 }
    pub fn capacity(&self) -> usize { self.capacity }
    pub fn overflow(&self) -> &Contact { &self.overflow }
    pub fn clear(&mut self) { self.used = 0; }

    pub fn take(&mut self) -> ContactSlot {
        if self.used >= self.capacity { return ContactSlot::Overflow; }
        let slot = ContactSlot::Record(self.used);
        if self.used == self.records.len() { self.records.push(Contact::default()); }
        self.used += 1;
        slot
    }

    pub fn get_mut(&mut self, slot: ContactSlot) -> &mut Contact {
        match slot {
            ContactSlot::Record(index) => &mut self.records[index],
            ContactSlot::Overflow => &mut self.overflow,
        }
    }

    pub(crate) fn push(&mut self, contact: Contact) -> bool {
        if self.len() >= self.capacity { return false; }
        if self.used == self.records.len() { self.records.push(contact); }
        else { self.records[self.used] = contact; }
        self.used += 1;
        true
    }

    pub fn remove_swap(&mut self, index: usize) {
        assert!(index < self.used, "contact index outside used pool");
        self.used -= 1;
        if index < self.used { self.records[index] = self.records[self.used].clone(); }
    }

    pub fn sort(&mut self, geometry: Geometry<'_>) {
        crate::numeric::quicksort(self.as_mut_slice(), |a, b| super::contact_compare(geometry, a, b));
    }
}

impl Contact {
    pub fn selected_point(&self) -> Vec3 {
        if self.flags.contains(ContactFlags::OTHER_POINT) { self.other_point } else { self.point }
    }

    pub fn source_entry(&self) -> EntryId { self.a.expect("contact without source entry") }
    pub fn other_entry(&self) -> EntryId { self.b.expect("contact without other entry") }

    pub fn to_local(&self, geometry: Geometry<'_>, motion: &impl BodyMotion) -> LocalContact {
        let source_item = geometry.entry_item(self.source_entry());
        let other_item = geometry.entry_item(self.other_entry());
        let iso = source_item.iso;
        let va = motion.velocity_at(source_item.body, self.point);
        let vb = if other_item.dynamic { motion.velocity_at(other_item.body, self.point) } else { Vec3::ZERO };
        let rotation = iso.rot;
        let rotate_to_local = |v: Vec3| Vec3::new(
            v.y * rotation.rows[1].x + v.x * rotation.rows[0].x + v.z * rotation.rows[2].x,
            v.y * rotation.rows[1].y + v.x * rotation.rows[0].y + v.z * rotation.rows[2].y,
            v.x * rotation.rows[0].z + v.y * rotation.rows[1].z + v.z * rotation.rows[2].z,
        );
        let relative_velocity = va - vb;
        LocalContact {
            a: self.source_entry(), b: self.other_entry(),
            material_a: self.material_a, material_b: self.material_b,
            normal: rotate_to_local(self.normal), point: iso.inverse_transform_point(self.point),
            other_point: iso.inverse_transform_point(self.selected_point()),
            relative_velocity: Vec3::new(
                relative_velocity.x * rotation.rows[0].x
                    + relative_velocity.y * rotation.rows[1].x + relative_velocity.z * rotation.rows[2].x,
                relative_velocity.x * rotation.rows[0].y
                    + relative_velocity.y * rotation.rows[1].y + relative_velocity.z * rotation.rows[2].y,
                relative_velocity.y * rotation.rows[1].z
                    + relative_velocity.x * rotation.rows[0].z + relative_velocity.z * rotation.rows[2].z,
            ),
            penetration: rotate_to_local(self.penetration),
        }
    }

    pub fn rescale(&mut self, iso: Iso, scale: f32) {
        self.point = crate::shape::scale::point(iso, scale, self.point);
        self.penetration = scale * self.penetration;
        if self.flags.contains(ContactFlags::OTHER_POINT) {
            self.other_point = crate::shape::scale::point(iso, scale, self.other_point);
        }
    }
}

pub trait BodyMotion { fn velocity_at(&self, body: u32, point: Vec3) -> Vec3; }

#[derive(Clone, Debug, PartialEq)]
pub struct LocalContact {
    pub a: EntryId,
    pub b: EntryId,
    pub material_a: u16,
    pub material_b: u16,
    pub normal: Vec3,
    pub point: Vec3,
    pub other_point: Vec3,
    pub relative_velocity: Vec3,
    pub penetration: Vec3,
}
