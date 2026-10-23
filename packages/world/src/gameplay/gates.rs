use std::collections::HashMap;

use anyhow::{Context, Result};
use gearbox_engine::{
    Aabb, Quat, Vec3,
    race::GateRecord,
    static_world::{Entity, StaticWorld},
};

#[derive(Clone, Copy)]
pub(crate) struct Properties {
    pub effect: u8,
    pub mergeable: bool,
}

struct Mergeable {
    index: usize,
    bounds: Aabb,
    inverse: Quat,
}

pub(crate) fn build(world: &StaticWorld, properties: &HashMap<u32, Properties>) -> Result<Vec<GateRecord>> {
    let mut gates = Vec::new();
    let mut mergeable = Vec::new();
    for item in world.items.iter().filter(|item| matches!(item.entity, Entity::Gate(_))) {
        let property = properties.get(&item.shape).context("gate has no first material gameplay metadata")?;
        let shape = &world.shapes[item.shape as usize];
        let direction = shape.main_dir;
        let rows = item.iso.rot.rows;
        let dir = Vec3::new(
            direction.y * rows[0].y + direction.x * rows[0].x + direction.z * rows[0].z,
            direction.x * rows[1].x + direction.y * rows[1].y + direction.z * rows[1].z,
            direction.x * rows[2].x + direction.y * rows[2].y + direction.z * rows[2].z,
        );
        if property.mergeable && property.effect == 9 {
            let rotation = Quat::from_mat3(&item.iso.rot);
            mergeable.push(Mergeable {
                index: gates.len(),
                bounds: crate::statics::shapes::bounds(shape).transform(&item.iso),
                inverse: Quat::new(rotation.w, -rotation.x, -rotation.y, -rotation.z),
            });
        }
        gates.push(GateRecord { effect: property.effect, dir, id: u32::MAX });
    }
    merge(&mut gates, &mergeable);
    Ok(gates)
}

fn merge(gates: &mut [GateRecord], mergeable: &[Mergeable]) {
    let mut count = 0;
    let mut joins = Vec::new();
    for (i, first) in mergeable.iter().enumerate() {
        let half = first.inverse.rotate(first.bounds.half);
        for second in &mergeable[i + 1..] {
            let other_half = first.inverse.rotate(second.bounds.half);
            let delta = first.inverse.rotate(second.bounds.center - first.bounds.center);
            let adjacent = delta.z.abs() <= 0.01
                && delta.x.abs() <= (half.x + other_half.x).abs() + 0.01
                && delta.y.abs() <= (half.y + other_half.y).abs() + 0.01
                && (delta.x.abs() <= 0.01 || delta.y.abs() <= 0.01);
            if !adjacent {
                continue;
            }
            let left = gates[first.index].id;
            let right = gates[second.index].id;
            match (left, right) {
                (u32::MAX, u32::MAX) => {
                    gates[first.index].id = count;
                    gates[second.index].id = count;
                    count += 1;
                }
                (u32::MAX, _) => gates[first.index].id = right,
                (_, u32::MAX) => gates[second.index].id = left,
                _ if left != right => joins.push((left.max(right), left.min(right))),
                _ => {}
            }
        }
    }
    let mut parents: Vec<u32> = (0..count).collect();
    joins.sort_unstable();
    for (left, right) in joins {
        let left = root(&parents, left);
        let right = root(&parents, right);
        parents[left.max(right) as usize] = left.min(right);
    }
    for gate in gates.iter_mut().filter(|gate| gate.id != u32::MAX) {
        gate.id = root(&parents, gate.id);
    }
}

fn root(parents: &[u32], mut index: u32) -> u32 {
    while parents[index as usize] < index {
        index = parents[index as usize];
    }
    index
}
