use crate::crystal::Crystal;

type Point = (bool, usize, usize);
pub(crate) type Selection = Vec<Point>;

pub(crate) fn select(
    c: &Crystal,
    layers: &[(Option<String>, Vec<u32>, bool, usize)],
    refs: &[(u32, Option<String>)],
) -> Vec<Selection> {
    selected_groups(layers, refs)
        .into_iter()
        .map(|(trigger, index, group)| {
            let g = if trigger { &c.triggers[index] } else { &c.geometry[index] };
            let mut points = Vec::new();
            for ((face, _), &fg) in g.faces.iter().zip(&g.face_groups) {
                if fg != group || !creatable(face) {
                    continue;
                }
                for &v in face {
                    let p = (trigger, index, v as usize);
                    if !points.contains(&p) {
                        points.push(p);
                    }
                }
            }
            points
        })
        .collect()
}

pub(crate) struct Chaos {
    pub from: f32,
    pub to: f32,
    pub seed: u32,
}

struct Lcg(u32);

impl Lcg {
    fn between(&mut self, from: f32, to: f32) -> f32 {
        self.0 = 0x3039u32.wrapping_sub(self.0.wrapping_mul(0x3e39_b193)) & 0x7fff_ffff;
        (self.0 >> 16) as f32 / 32767.0 * (to - from) + from
    }
}

type Layer = (Option<String>, Vec<u32>, bool, usize);

fn selected_groups(layers: &[Layer], refs: &[(u32, Option<String>)]) -> Vec<(bool, usize, usize)> {
    refs.iter()
        .filter_map(|(group, id)| {
            layers.iter().find_map(|(layer, groups, trigger, index)| {
                let position = groups.iter().position(|g| g == group).filter(|_| layer == id)?;
                Some((*trigger, *index, position))
            })
        })
        .collect()
}

fn creatable(face: &[u32]) -> bool {
    (3..=4).contains(&face.len())
}

pub(crate) fn chaos(c: &mut Crystal, layers: &[Layer], refs: &[(u32, Option<String>)], chaos: &Chaos) {
    let groups = selected_groups(layers, refs);
    let mut points = Vec::new();
    for selection in select(c, layers, refs) {
        for p in selection {
            if !points.contains(&p) {
                points.push(p);
            }
        }
    }
    for trigger in [false, true] {
        let geometry = if trigger { &mut c.triggers } else { &mut c.geometry };
        let mut random = Lcg(chaos.seed);
        let selected = |index: usize, group: usize| groups.contains(&(trigger, index, group));
        let mut fixed = std::collections::HashSet::new();
        let mut adjacent = std::collections::HashMap::<usize, Vec<Vec<usize>>>::new();
        for (index, g) in geometry.iter().enumerate() {
            let mut sides = std::collections::HashMap::<(u32, u32), u32>::new();
            let mut faces_of = vec![Vec::new(); g.positions.len()];
            for (f, (face, _)) in g.faces.iter().enumerate().filter(|(_, (face, _))| creatable(face)) {
                for (i, &v) in face.iter().enumerate() {
                    let w = face[(i + 1) % face.len()];
                    *sides.entry((v.min(w), v.max(w))).or_default() += 1;
                    faces_of[v as usize].push(f);
                }
            }
            for ((face, _), &group) in g.faces.iter().zip(&g.face_groups) {
                if !creatable(face) || !selected(index, group) {
                    continue;
                }
                for (i, &v) in face.iter().enumerate() {
                    let w = face[(i + 1) % face.len()];
                    if sides[&(v.min(w), v.max(w))] == 1 {
                        fixed.insert((index, v as usize));
                        fixed.insert((index, w as usize));
                    }
                }
            }
            adjacent.insert(index, faces_of);
        }
        for &(t, index, vertex) in &points {
            if t != trigger || fixed.contains(&(index, vertex)) {
                continue;
            }
            let g = &geometry[index];
            let mut sum = [0.0f32; 3];
            let mut inside = true;
            for &f in &adjacent[&index][vertex] {
                for i in 0..3 {
                    sum[i] += g.normals[f][i];
                }
                if !selected(index, g.face_groups[f]) {
                    inside = false;
                    break;
                }
            }
            if !inside {
                continue;
            }
            let n = sum[1] * sum[1] + sum[0] * sum[0] + sum[2] * sum[2];
            if n > 1e-10 && n < f32::MAX {
                let inv = 1.0 / n.sqrt();
                sum = sum.map(|v| v * inv);
            }
            let distance = random.between(chaos.from, chaos.to);
            let p = &mut geometry[index].positions[vertex];
            for i in 0..3 {
                p[i] += sum[i] * distance;
            }
        }
        for (index, g) in geometry.iter_mut().enumerate() {
            for f in 0..g.faces.len() {
                if creatable(&g.faces[f].0) && selected(index, g.face_groups[f]) {
                    g.normals[f] = crate::crystal_deformation::face_normal(&g.positions, &g.faces[f].0);
                }
            }
        }
    }
}

pub fn centroid(positions: impl IntoIterator<Item = [f32; 3]>) -> [f32; 3] {
    let mut sum = [0.0; 3];
    let mut count = 0.0f32;
    for p in positions {
        count += 1.0;
        for i in 0..3 {
            sum[i] += p[i];
        }
    }
    if count != 0.0 {
        let reciprocal = 1.0 / count;
        sum = sum.map(|v| reciprocal * v);
    }
    sum.map(|v| {
        let scale = crate::reader_math::maxss(1.0, crate::reader_math::maxss(v.abs(), 0.0));
        if scale * 0.00001 >= (v - 0.0).abs() { 0.0 } else { v }
    })
}

pub fn scale(positions: &mut [[f32; 3]], value: [f32; 3]) {
    let center = centroid(positions.iter().copied());
    for p in positions {
        for i in 0..3 {
            p[i] = (p[i] - center[i]) * value[i] + center[i];
        }
    }
}

pub fn translate(positions: &mut [[f32; 3]], value: [f32; 3]) {
    for p in positions {
        p[0] += value[0];
        p[1] += value[1];
        p[2] += value[2];
    }
}

pub(crate) fn apply(c: &mut Crystal, selections: &[Selection], value: [f32; 3], scaling: bool) {
    let mut points = Vec::new();
    for selection in selections {
        for &p in selection {
            if !points.contains(&p) {
                points.push(p);
            }
        }
    }
    for trigger in [false, true] {
        let selected: Vec<_> = points.iter().copied().filter(|p| p.0 == trigger).collect();
        let mut positions: Vec<_> = selected
            .iter()
            .map(|&(_, g, p)| if trigger { c.triggers[g].positions[p] } else { c.geometry[g].positions[p] })
            .collect();
        if scaling {
            scale(&mut positions, value);
        } else {
            translate(&mut positions, value);
        }
        for ((_, g, p), position) in selected.into_iter().zip(positions) {
            if trigger {
                c.triggers[g].positions[p] = position;
            } else {
                c.geometry[g].positions[p] = position;
            }
        }
    }
}
