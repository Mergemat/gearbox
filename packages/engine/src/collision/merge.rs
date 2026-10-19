use super::compare::dot_yxz;
use super::{ContactList, Geometry};
use crate::collision::ContactFlags;
use crate::{Vec3, numeric};

const COS10: f32 = 0.9848077;
const COS30: f32 = 0.8660254;
const MAX_ISLANDS: usize = 32;
const MAX_REMOVALS: usize = 1024;
const TINY: f32 = 1.0e-10;

impl ContactList {
    pub fn merge(&mut self, src: &mut [super::Contact], single_pass: u32) {
        let before = self.len();
        let mut count = before;
        let mut full = false;
        for contact in src.iter() {
            if contact.flags.contains(ContactFlags::FACE) {
                count = self.len();
                if !self.push(contact.clone()) {
                    full = true;
                    break;
                }
            }
        }
        if !full && !src.is_empty() {
            count = self.len();
        }
        if single_pass == 0 && before == count && count > 1 && !src.is_empty() {
            for contact in src.iter_mut() {
                if COS10 < dot_yxz(contact.normal, contact.face_normal) {
                    contact.normal = contact.face_normal;
                    contact.flags.set(ContactFlags::FACE, true);
                    self.push(contact.clone());
                    break;
                }
            }
            count = self.len();
        } else {
            count = self.len();
        }
        for contact in src.iter() {
            if contact.flags.contains(ContactFlags::FACE) {
                continue;
            }
            if self.as_slice()[before..count].iter().any(|d| {
                (COS10 < dot_yxz(contact.face_normal, d.face_normal)
                    || COS30 < dot_yxz(contact.normal, d.normal))
                    && d.material_b as u8 != 13
            }) {
                continue;
            }
            if !self.push(contact.clone()) {
                return;
            }
        }
    }

    pub fn merge_close(&mut self, geometry: Geometry<'_>, min_dot: f32, distance: f32) {
        let d2 = distance * distance;
        let mut islands: Vec<Vec<u32>> = Vec::new();
        let records = self.as_slice();
        for i in 0..records.len() {
            for j in i + 1..records.len() {
                let (a, b) = (&records[i], &records[j]);
                let d = b.selected_point() - a.selected_point();
                if d.y * d.y + d.x * d.x + d.z * d.z > d2 {
                    continue;
                }
                let dot = a.normal.y * b.normal.y + a.normal.x * b.normal.x + b.normal.z * a.normal.z;
                if min_dot > dot {
                    continue;
                }
                if geometry.entry_item(a.other_entry()).layer == 1
                    || geometry.entry_item(b.other_entry()).layer == 1
                {
                    continue;
                }
                add_pair(&mut islands, i as u32, j as u32);
            }
        }
        let mut removals = Vec::new();
        for island in islands {
            let (&last, rest) = island.split_last().expect("islands contain a pair");
            let mut tag = self.as_slice()[last as usize].material_b;
            let (mut normal, mut point, mut penetration) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
            let mut count = 0;
            for &id in &island {
                let contact = &self.as_slice()[id as usize];
                if contact.material_b as u8 == 13 {
                    continue;
                }
                count += 1;
                point += contact.selected_point();
                if tag as u8 == 13 {
                    tag = contact.material_b;
                }
                normal += if contact.normal.dot(contact.face_normal) > min_dot {
                    contact.face_normal
                } else {
                    contact.normal
                };
                penetration += contact.penetration;
            }
            if count != 0 {
                let l2 = normal.y * normal.y + normal.x * normal.x + normal.z * normal.z;
                if l2 > TINY && f32::MAX > l2 {
                    normal *= 1.0 / l2.sqrt();
                }
                let target = &mut self.as_mut_slice()[last as usize];
                target.normal = normal;
                target.flags.set(ContactFlags::OTHER_POINT, false);
                target.point =
                    Vec3::new(point.x / count as f32, point.y / count as f32, point.z / count as f32);
                target.material_b = tag;
                target.penetration = Vec3::new(
                    penetration.x / count as f32,
                    penetration.y / count as f32,
                    penetration.z / count as f32,
                );
            }
            assert!(removals.len() + rest.len() <= MAX_REMOVALS, "contact removal list overflow");
            removals.extend_from_slice(rest);
        }
        numeric::quicksort(&mut removals, |a, b| {
            if a < b {
                1
            } else if a > b {
                -1
            } else {
                0
            }
        });
        for id in removals {
            self.remove_swap(id as usize);
        }
    }
}

fn add_pair(islands: &mut Vec<Vec<u32>>, a: u32, b: u32) {
    let ga = islands.iter().position(|v| v.contains(&a));
    let gb = islands.iter().position(|v| v.contains(&b));
    match (ga, gb) {
        (None, None) => {
            assert!(islands.len() < MAX_ISLANDS, "too many contact islands");
            islands.push(vec![a, b]);
        }
        (None, Some(geometry)) => push_id(&mut islands[geometry], a),
        (Some(geometry), None) => push_id(&mut islands[geometry], b),
        (Some(a), Some(b)) if a != b => {
            let other = std::mem::take(&mut islands[b]);
            assert!(islands[a].len() + other.len() <= MAX_REMOVALS, "contact island overflow");
            islands[a].extend(other);
            islands.swap_remove(b);
        }
        _ => {}
    }
}

fn push_id(island: &mut Vec<u32>, id: u32) {
    assert!(island.len() < MAX_REMOVALS, "contact island overflow");
    island.push(id);
}
