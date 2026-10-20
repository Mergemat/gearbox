use crate::crystal::Crystal;

type Point = (bool, usize, usize);
pub(crate) type Selection = Vec<Point>;

pub(crate) fn select(
    c: &Crystal,
    layers: &[(Option<String>, Vec<u32>, bool, usize)],
    refs: &[(u32, Option<String>)],
) -> Vec<Selection> {
    refs.iter()
        .map(|(group, id)| {
            let mut points = Vec::new();
            for (layer, groups, trigger, index) in layers.iter().rev() {
                if layer != id {
                    continue;
                }
                let Some(group) = groups.iter().position(|g| g == group) else {
                    continue;
                };
                let g = if *trigger { &c.triggers[*index] } else { &c.geometry[*index] };
                for ((face, _), &fg) in g.faces.iter().zip(&g.face_groups).rev() {
                    if fg != group {
                        continue;
                    }
                    for &v in face {
                        let p = (*trigger, *index, v as usize);
                        if !points.contains(&p) {
                            points.push(p);
                        }
                    }
                }
            }
            points
        })
        .collect()
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
