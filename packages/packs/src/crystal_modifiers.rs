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
            for ((face, _), &fg) in g.faces.iter().zip(&g.face_groups).rev() {
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

pub(crate) struct Mirror {
    pub axis: usize,
    pub distance: f32,
    pub independently: bool,
}

fn bounds(positions: impl IntoIterator<Item = [f32; 3]>) -> Option<([f32; 3], [f32; 3])> {
    let mut low = [f32::MAX; 3];
    let mut high = [-f32::MAX; 3];
    let mut any = false;
    for p in positions {
        any = true;
        for i in 0..3 {
            low[i] = crate::reader_math::minss(p[i], low[i]);
            high[i] = crate::reader_math::maxss(p[i], high[i]);
        }
    }
    any.then_some((low, high))
}

pub(crate) fn mirror(
    c: &mut Crystal,
    layers: &mut Vec<Layer>,
    layer_id: &Option<String>,
    refs: &[(u32, Option<String>)],
    mirror: &Mirror,
) -> anyhow::Result<()> {
    let batches: Vec<Vec<(u32, Option<String>)>> = if mirror.independently {
        refs.iter().map(|r| vec![r.clone()]).collect()
    } else {
        vec![refs.to_vec()]
    };
    for trigger in [false, true] {
        for batch in &batches {
            let groups: Vec<_> =
                selected_groups(layers, batch).into_iter().filter(|g| g.0 == trigger).collect();
            if groups.is_empty() {
                continue;
            }
            let geometry = if trigger { &mut c.triggers } else { &mut c.geometry };
            let mut faces = Vec::new();
            let mut points = Vec::new();
            for &(_, index, group) in &groups {
                let g = &geometry[index];
                anyhow::ensure!(g.group_parents[group] == u32::MAX, "mirror of nested crystal groups");
                for (f, ((face, _), &fg)) in g.faces.iter().zip(&g.face_groups).enumerate().rev() {
                    if fg != group || !creatable(face) {
                        continue;
                    }
                    faces.push((index, f));
                    for &v in face {
                        if !points.contains(&(index, v as usize)) {
                            points.push((index, v as usize));
                        }
                    }
                }
            }
            let at = |p: &(usize, usize)| geometry[p.0].positions[p.1];
            let (low, high) = bounds(points.iter().map(at)).unwrap_or(([1.0; 3], [-1.0; 3]));
            let center: [f32; 3] = std::array::from_fn(|i| (low[i] + high[i]) * 0.5);
            let half: [f32; 3] = std::array::from_fn(|i| (high[i] - low[i]) * 0.5);
            let a = mirror.axis;
            let edge = if 0.0 <= mirror.distance { center[a] + half[a] } else { center[a] - half[a] };
            let offset = -(edge + mirror.distance);
            let normal: [f32; 3] = std::array::from_fn(|i| if i == a { 1.0 } else { 0.0 });

            let mut clip = points.clone();
            clip.sort_unstable_by(|a, b| b.cmp(a));
            let (low, high) = bounds(clip.iter().map(at)).unwrap_or(([0.0; 3], [0.0; 3]));
            let middle: [f32; 3] = std::array::from_fn(|i| (high[i] - low[i]) * 0.5 + low[i]);
            let base = middle.map(|m| m + 0.0);
            let positions: Vec<[f32; 3]> = clip
                .iter()
                .map(|p| {
                    let q = at(p);
                    let mut p: [f32; 3] = std::array::from_fn(|i| (q[i] - middle[i]) + base[i]);
                    let d = ((p[0] * normal[0] + p[1] * normal[1]) + p[2] * normal[2]) + offset;
                    let d = d + d;
                    for i in 0..3 {
                        p[i] -= normal[i] * d;
                    }
                    p
                })
                .collect();

            let mut sources = Vec::new();
            let mut new_faces = Vec::new();
            let mut new_uvs = Vec::new();
            let mut new_groups = Vec::new();
            for &(index, f) in &faces {
                let g = &geometry[index];
                let source = (index, g.face_groups[f]);
                let group = match sources.iter().position(|s| *s == source) {
                    Some(i) => i,
                    None => {
                        sources.push(source);
                        sources.len() - 1
                    }
                };
                let (face, material) = &g.faces[f];
                let mut vertices: Vec<u32> = face
                    .iter()
                    .map(|&v| clip.iter().position(|p| *p == (index, v as usize)).unwrap() as u32)
                    .collect();
                let mut uvs = g.face_uvs[f].clone();
                let last = vertices.len() - 1;
                vertices.swap(1, last);
                if uvs.len() > last {
                    uvs.swap(1, last);
                }
                new_faces.push((vertices, *material));
                new_uvs.push(uvs);
                new_groups.push(group);
            }
            let normals = new_faces
                .iter()
                .map(|(f, _)| crate::crystal_deformation::face_normal(&positions, f))
                .collect();
            let mirrored = crate::crystal::Geometry {
                positions,
                faces: new_faces,
                face_uvs: new_uvs,
                face_groups: new_groups,
                normals,
                group_parents: vec![u32::MAX; sources.len()],
                archive_version: geometry[groups[0].1].archive_version,
            };
            geometry.push(mirrored);
            let index = geometry.len() - 1;
            let first: u32 = geometry[..index].iter().map(|g| g.group_parents.len() as u32).sum();
            layers.push((layer_id.clone(), (first..first + sources.len() as u32).collect(), trigger, index));
        }
    }
    Ok(())
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
