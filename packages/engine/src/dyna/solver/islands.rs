use super::super::{SolverFlags, constants::NULL_INDEX};
use super::Body;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IslandItem {
    pub parent: u32,
    pub link: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Islands {
    pub ids: Vec<u32>,
    pub ends: Vec<u32>,
}

impl Islands {
    pub fn members(&self) -> impl Iterator<Item = &[u32]> {
        self.ends.iter().scan(0usize, |start, &end| {
            let members = &self.ids[*start..end as usize];
            *start = end as usize;
            Some(members)
        })
    }
}

pub fn build_item_islands(items: &mut [IslandItem], ids: &[u32]) -> Islands {
    const END: u32 = 0x7fff_ffff;
    let n = ids.len();
    for i in 0..n {
        items[i].parent = items[items[i].parent as usize].parent;
        items[i].link = END;
    }
    for i in (0..n).rev() {
        let parent = items[i].parent as usize;
        if parent != i {
            items[i].link = items[parent].link;
            items[parent].link = i as u32;
        }
    }
    let mut out = Islands::default();
    for i in 0..n {
        if (items[i].link as i32) < 0 {
            continue;
        }
        let mut current = i;
        loop {
            out.ids.push(ids[current]);
            let next = items[current].link;
            items[current].link = out.ends.len() as u32 | 0x8000_0000;
            items[current].parent = current as u32;
            if next == END {
                break;
            }
            current = next as usize;
        }
        out.ends.push(out.ids.len() as u32);
    }
    out
}

pub fn can_sleep(bodies: &mut [Body], ids: &[u32], now: u32, linear: f32, angular: f32, frames: u32) -> bool {
    let mut all = true;
    for &i in ids {
        let b = &mut bodies[i as usize];
        let mut t = b.index.rest_since;
        if t == NULL_INDEX {
            t = now;
        }
        if b.record.flags.contains(SolverFlags::EXTERNAL) {
            t = now;
        }
        let ok = match b.model.kind {
            0 => false,
            2 | 3 => todo!("sleep test for item kind"),
            _ => {
                let v = b.state.vel;
                let w = b.state.ang_vel;
                let moving = b.record.flags.contains(SolverFlags::IN_WATER)
                    || v.x * v.x + v.y * v.y + v.z * v.z > linear
                    || w.x * w.x + w.y * w.y + w.z * w.z > angular;
                if moving {
                    t = now;
                }
                now.wrapping_sub(t) > frames
            }
        };
        b.index.rest_since = t;
        all &= ok;
    }
    all
}

pub fn sleep(bodies: &mut [Body], active: &mut Vec<u32>, ids: &[u32]) {
    for &id in ids {
        let index = bodies[id as usize].index.index;
        if index == u16::MAX {
            continue;
        }
        let last = active.len() - 1;
        if index as usize != last {
            let replacement = active[last];
            active[index as usize] = replacement;
            bodies[replacement as usize].index.index = index;
        }
        bodies[id as usize].index.index = u16::MAX;
        bodies[id as usize].state.vel = crate::Vec3::ZERO;
        bodies[id as usize].state.ang_vel = crate::Vec3::ZERO;
        active.pop();
    }
}
