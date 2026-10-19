use crate::reader_math::{angle_approx, matrix_to_quat_values, maxss, minss, quat_matrix};

#[derive(Clone, Copy, Debug)]
struct Frame {
    rotation: [f32; 9],
    position: [f32; 3],
    parameter: f32,
}
const EPS: f32 = 0.00001;
const TINY: f32 = 1.0e-10;
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn length(p: [f32; 3]) -> f32 {
    (p[1] * p[1] + p[0] * p[0] + p[2] * p[2]).sqrt()
}
fn difference(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn near(a: f32, b: f32) -> bool {
    (a - b).abs() <= maxss(1.0, maxss(a.abs(), b.abs())) * EPS
}
fn normalize(mut p: [f32; 3], y_first: bool) -> [f32; 3] {
    let n = if y_first {
        p[1] * p[1] + p[0] * p[0] + p[2] * p[2]
    } else {
        p[0] * p[0] + p[1] * p[1] + p[2] * p[2]
    };
    if n > TINY && n < f32::MAX {
        let inv = 1.0 / n.sqrt();
        p = p.map(|v| v * inv);
    }
    p
}
fn basis(forward: [f32; 3], up: [f32; 3]) -> [f32; 9] {
    let right = cross(up, forward);
    let n = right[1] * right[1] + right[0] * right[0] + right[2] * right[2];
    if !(n > TINY && n < f32::MAX) {
        todo!("degenerate crystal deformation frame");
    }
    let right = normalize(right, true);
    let forward = normalize(forward, false);
    let up = cross(forward, right);
    [right[0], up[0], forward[0], right[1], up[1], forward[1], right[2], up[2], forward[2]]
}
fn column(r: [f32; 9], i: usize) -> [f32; 3] {
    [r[i], r[3 + i], r[6 + i]]
}
fn point(r: [f32; 9], p: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| (r[i * 3 + 1] * p[1] + r[i * 3] * p[0]) + r[i * 3 + 2] * p[2])
}
fn tangent(angle: f32) -> f32 {
    let mut x = angle.abs();
    let mut n = (x * 1.2732395).floor() as i32;
    if n & 1 != 0 {
        n += 1;
    }
    let nf = n as f32;
    x = ((x - nf * 0.7853981) - nf * 3.7748947e-8) - nf * 2.6951515e-15;
    let z = x * x;
    if z > 1.0e-14 {
        let numerator = ((1153516.6 - z * 13093.694) * z - 17956526.0) * z;
        let denominator = (((z + 13681.296) * z - 1320892.4) * z + 25008380.0) * z - 53869576.0;
        x = (numerator / denominator) * x + x;
    }
    if n & 2 != 0 {
        x = -1.0 / x;
    }
    if angle < 0.0 { -x } else { x }
}
fn handles(a: Frame, b: Frame, limit: f32) -> ([f32; 3], [f32; 3]) {
    let f0 = column(a.rotation, 2);
    let f1 = column(b.rotation, 2);
    let axis = cross(f1, f0);
    let axis_n = axis[1] * axis[1] + axis[0] * axis[0] + axis[2] * axis[2];
    let (h0, h1) = if axis_n <= TINY {
        let h = length(difference(b.position, a.position)) * 0.3;
        (h, h)
    } else {
        let u = cross(f0, axis);
        let v = cross(f1, axis);
        let d = difference(b.position, a.position);
        let delta = difference(u, v).map(f32::abs);
        let (det, n0, n1) = if delta[0] >= delta[2] && delta[1] >= delta[2] {
            (v[1] * u[0] - v[0] * u[1], d[0] * v[1] - d[1] * v[0], d[1] * u[0] - d[0] * u[1])
        } else if delta[0] >= delta[1] && delta[2] >= delta[1] {
            (v[2] * u[0] - v[0] * u[2], d[0] * v[2] - d[2] * v[0], d[2] * u[0] - d[0] * u[2])
        } else {
            (v[2] * u[1] - v[1] * u[2], d[1] * v[2] - d[2] * v[1], d[2] * u[1] - d[1] * u[2])
        };
        if !(det.abs() * 100000.0 > n0.abs() && det.abs() * 100000.0 > n1.abs()) {
            todo!("singular crystal deformation handles");
        }
        let inv = 1.0 / det;
        let h0 = minss((inv * n0).abs(), limit);
        let h1 = minss((inv * n1).abs(), limit);
        let c = cross(f1, f0);
        let dot = (f0[0] * f1[0] + f0[1] * f1[1]) + f0[2] * f1[2];
        let c_len = (c[1] * c[1] + c[0] * c[0] + c[2] * c[2]).sqrt();
        let factor = tangent(angle_approx(c_len, dot) * 0.25) * 1.3333334;
        ((factor * h1).abs(), (factor * h0).abs())
    };
    (std::array::from_fn(|i| a.position[i] + f0[i] * h0), std::array::from_fn(|i| b.position[i] - f1[i] * h1))
}
fn transform_frames(frames: &mut [Frame], guides: &[Frame], limit: f32) {
    if guides.is_empty() {
        return;
    }
    let first = frames[0];
    let mut path = vec![Frame { rotation: quat_matrix(matrix_to_quat_values(first.rotation)), ..first }];
    path.extend_from_slice(guides);
    let mut distances = vec![0.0f32];
    for pair in path.windows(2) {
        distances.push(distances.last().unwrap() + length(difference(pair[1].position, pair[0].position)));
    }
    let total = *distances.last().unwrap();
    let distances: Vec<_> = distances.iter().map(|v| v / total).collect();
    for f in frames {
        let (segment, t) = if f.parameter <= 0.0 {
            (1, 0.0)
        } else if f.parameter >= 1.0 {
            (path.len() - 1, 1.0)
        } else {
            let i = distances.partition_point(|&d| d <= f.parameter).min(path.len() - 1);
            (i, (f.parameter - distances[i - 1]) / (distances[i] - distances[i - 1]))
        };
        let (a, b) = (path[segment - 1], path[segment]);
        let (h0, h1) = handles(a, b, limit);
        let s = 1.0 - t;
        f.position = if t.abs() < EPS {
            a.position
        } else if s.abs() < EPS {
            b.position
        } else {
            let ratio = t / s;
            let s3 = (s * s) * s;
            let a1 = (s3 * ratio) * 3.0;
            let a2 = ((s3 * ratio) * ratio) * 3.0;
            let a3 = ratio * ((s3 * ratio) * ratio);
            std::array::from_fn(|i| {
                (((a.position[i] * s3 + 0.0) + a1 * h0[i]) + a2 * h1[i]) + b.position[i] * a3
            })
        };
        let a0 = (s * s) * -3.0;
        let mix = (s * 6.0) * t;
        let a1 = (s * s) * 3.0 - mix;
        let a3 = (t * t) * 3.0;
        let a2 = mix - a3;
        let tangent = std::array::from_fn(|i| {
            (((a.position[i] * a0 + 0.0) + h0[i] * a1) + h1[i] * a2) + b.position[i] * a3
        });
        let tangent = normalize(tangent, false);
        let up0 = column(a.rotation, 1);
        let up1 = column(b.rotation, 1);
        let up = normalize(std::array::from_fn(|i| (up1[i] - up0[i]) * t + up0[i]), true);
        f.rotation = basis(tangent, up);
    }
}

fn bounds(positions: &[[f32; 3]]) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::MAX; 3];
    let mut hi = [-f32::MAX; 3];
    for p in positions {
        for i in 0..3 {
            lo[i] = minss(p[i], lo[i]);
            hi[i] = maxss(p[i], hi[i]);
        }
    }
    let half = std::array::from_fn(|i| (hi[i] - lo[i]) * 0.5);
    (std::array::from_fn(|i| (lo[i] + hi[i]) * 0.5), half)
}
#[derive(Clone, Copy, Debug)]
struct LocalPoint {
    position: [f32; 3],
    frame: usize,
}
fn source_frames(
    positions: &[[f32; 3]],
    anchor: [f32; 3],
    direction: [f32; 3],
    width: f32,
) -> (Vec<Frame>, Vec<LocalPoint>, f32) {
    let (mut center, mut half) = bounds(positions);
    for i in 0..3 {
        if half[i] == 0.0 && direction[i] != 0.0 {
            half[i] = 0.5;
            center[i] += direction[i] * 0.5;
        }
    }
    let mut start = std::array::from_fn::<_, 3, _>(|i| center[i] - half[i]);
    let mut end = std::array::from_fn::<_, 3, _>(|i| center[i] + half[i]);
    let axis = if direction[0] == 0.0 { 2 } else { 0 };
    if direction[axis] < 0.0 {
        std::mem::swap(&mut start[axis], &mut end[axis]);
    }
    let delta = std::array::from_fn::<_, 3, _>(|i| if i == axis { end[i] - start[i] } else { 0.0 });
    let origin = std::array::from_fn::<_, 3, _>(|i| if i == axis { start[i] } else { anchor[i] });
    let len = (delta[2] * delta[2] + delta[0] * delta[0]).sqrt();
    let inv = 1.0 / (len * len);
    let coefficients = [inv * delta[0], inv * delta[2]];
    let rotation = basis(direction, [0.0, 1.0, 0.0]);
    let transpose = std::array::from_fn(|i| rotation[(i % 3) * 3 + i / 3]);
    let mut frames = Vec::<Frame>::new();
    let mut points = Vec::new();
    for &p in positions {
        let t = (p[2] - start[2]) * coefficients[1] + (p[0] - start[0]) * coefficients[0];
        let plane = [delta[0] * t + origin[0], t * 0.0 + origin[1], t * delta[2] + origin[2]];
        let index =
            frames.iter().position(|f| (0..3).all(|i| near(f.position[i], plane[i]))).unwrap_or_else(|| {
                let i = frames.len();
                frames.push(Frame { rotation, position: plane, parameter: t });
                i
            });
        points.push(LocalPoint { position: point(transpose, difference(p, plane)), frame: index });
    }
    let mut order: Vec<_> = (0..frames.len()).collect();
    order.sort_by(|&a, &b| frames[a].parameter.partial_cmp(&frames[b].parameter).unwrap());
    let mut remap = vec![0; frames.len()];
    for (new, &old) in order.iter().enumerate() {
        remap[old] = new;
    }
    frames = order.iter().map(|&i| frames[i]).collect();
    for p in &mut points {
        p.frame = remap[p.frame];
    }
    let first = frames[0].position;
    let length = (delta[0] * delta[0] + 0.0 + delta[2] * delta[2]).sqrt();
    for f in &mut frames {
        let scale = width / length;
        f.position = [
            (scale * delta[0]) * f.parameter + first[0],
            (scale * 0.0) * f.parameter + first[1],
            (scale * delta[2]) * f.parameter + first[2],
        ];
    }
    (frames, points, len)
}
fn arc_length(frames: &[Frame]) -> f32 {
    frames.windows(2).fold(0.0, |sum, p| sum + length(difference(p[0].position, p[1].position)))
}
fn apply_frames(frames: &[Frame], points: &[LocalPoint]) -> Vec<[f32; 3]> {
    points
        .iter()
        .map(|p| {
            let f = frames[p.frame];
            let v = point(f.rotation, p.position);
            std::array::from_fn(|i| v[i] + f.position[i])
        })
        .collect()
}

pub(crate) struct Deformation {
    center: [f32; 3],
    half: [f32; 3],
    transform: [f32; 12],
    guides: Vec<[f32; 12]>,
}
pub(crate) fn read(r: &mut gearbox_gbx::reader::Reader<'_>) -> anyhow::Result<Deformation> {
    anyhow::ensure!(r.u32()? == 0, "crystal deformation version");
    let center = r.vec3()?;
    let half = r.vec3()?;
    let mut transform = [0.; 12];
    for v in &mut transform {
        *v = r.f32()?;
    }
    let mut guides = Vec::new();
    for _ in 0..r.len()? {
        let mut g = [0.; 12];
        for v in &mut g {
            *v = r.f32()?;
        }
        guides.push(g);
    }
    Ok(Deformation { center, half, transform, guides })
}
pub(crate) fn apply_layers(
    geometry: &mut [crate::crystal::Geometry],
    layers: &[(Option<String>, Vec<u32>, bool, usize)],
    refs: &[(u32, Option<String>)],
    modifier: &Deformation,
    trigger: bool,
) {
    let mut selected = Vec::<(usize, Vec<usize>)>::new();
    for (group, id) in refs {
        if let Some((_, groups, _, index)) = layers.iter().find(|l| &l.0 == id && l.2 == trigger) {
            if let Some(group) = groups.iter().position(|g| g == group) {
                let i = selected.iter().position(|s| s.0 == *index).unwrap_or_else(|| {
                    selected.push((*index, Vec::new()));
                    selected.len() - 1
                });
                if !selected[i].1.contains(&group) {
                    selected[i].1.push(group);
                }
            }
        }
    }
    if selected.len() > 1 {
        todo!("crystal deformation spanning geometry layers");
    }
    for (i, groups) in selected {
        apply(&mut geometry[i], &groups, modifier);
    }
}
fn apply(g: &mut crate::crystal::Geometry, groups: &[usize], modifier: &Deformation) {
    if g.positions.is_empty() {
        return;
    }
    if modifier.guides.len() != 1 {
        todo!("multi-guide or implicit crystal deformation");
    }
    if g.archive_version != 37 {
        todo!("legacy crystal deformation topology");
    }
    if g.group_parents.iter().any(|&p| p != u32::MAX) {
        todo!("nested crystal deformation groups");
    }
    if g.face_groups.iter().any(|i| !groups.contains(i)) {
        todo!("partial-layer crystal deformation");
    }
    let mut ids = Vec::new();
    for &group in groups {
        for ((face, _), &fg) in g.faces.iter().zip(&g.face_groups) {
            if fg == group {
                for &id in face {
                    if !ids.contains(&id) {
                        ids.push(id);
                    }
                }
            }
        }
    }
    let positions: Vec<_> = ids.iter().map(|&i| g.positions[i as usize]).collect();
    let (mut center, mut half) = bounds(&positions);
    let forward = column(modifier.transform[..9].try_into().unwrap(), 2);
    for i in 0..3 {
        if half[i] == 0.0 && forward[i] != 0.0 {
            half[i] = 0.05;
            center[i] += forward[i] * 0.05;
        }
    }
    let mut guides = modifier.guides.clone();
    if modifier.half[0] >= 0.0
        && !(0..3).all(|i| near(modifier.center[i], center[i]))
        && !(0..3).all(|i| near(modifier.half[i], half[i]))
    {
        let scale: [f32; 3] = std::array::from_fn(|i| half[i] / modifier.half[i]);
        for guide in &mut guides {
            for i in 0..3 {
                guide[9 + i] = (guide[9 + i] - modifier.center[i]) * scale[i] + center[i];
            }
        }
    }
    let axis = if forward[2].abs() > forward[0].abs() { 2 } else { 0 };
    let mut direction = [0.; 3];
    direction[axis] = if forward[axis].is_sign_negative() { -1.0 } else { 1.0 };
    let anchor = std::array::from_fn(|i| center[i] - half[i] * direction[i]);
    let width = ((half[1] + half[1]) * direction[1]
        + (half[0] + half[0]) * direction[0]
        + (half[2] + half[2]) * direction[2])
        .abs();
    let guides: Vec<_> = guides
        .iter()
        .map(|g| Frame {
            rotation: quat_matrix(matrix_to_quat_values(g[..9].try_into().unwrap())),
            position: g[9..].try_into().unwrap(),
            parameter: 0.,
        })
        .collect();
    let (mut frames, _, limit) = source_frames(&positions, anchor, direction, width);
    transform_frames(&mut frames, &guides, limit);
    let copies = if width == 0.0 { 1 } else { (arc_length(&frames) / width).round() as usize };
    if copies == 0 {
        todo!("zero-repeat crystal deformation");
    }
    let mut lo = [f32::MAX; 3];
    let mut hi = [-f32::MAX; 3];
    for p in &positions {
        for i in 0..3 {
            lo[i] = minss(p[i], lo[i]);
            hi[i] = maxss(p[i], hi[i]);
        }
    }
    let export_center: [f32; 3] = std::array::from_fn(|i| (hi[i] - lo[i]) * 0.5 + lo[i]);
    let mut source_ids = ids.clone();
    source_ids.sort_unstable();
    let mut source_map = vec![0; g.positions.len()];
    let source: Vec<_> = source_ids
        .iter()
        .enumerate()
        .map(|(new, &old)| {
            source_map[old as usize] = new as u32;
            g.positions[old as usize]
        })
        .collect();
    let mut repeated = Vec::with_capacity(source.len() * copies);
    for copy in 0..copies {
        let step = direction.map(|v| width * v);
        let offset = step.map(|v| v * (copy as f32));
        let origin: [f32; 3] = std::array::from_fn(|i| export_center[i] + offset[i]);
        for p in &source {
            repeated.push(std::array::from_fn(|i| (p[i] - export_center[i]) + origin[i]));
        }
    }
    let (mut frames, points, limit) = source_frames(&repeated, anchor, direction, width);
    transform_frames(&mut frames, &guides, limit);
    let positions = apply_frames(&frames, &points);
    let mut faces = Vec::new();
    let mut face_uvs = Vec::new();
    let mut face_groups = Vec::new();
    for copy in 0..copies {
        for &group in groups {
            for (i, (face, material)) in g.faces.iter().enumerate() {
                if g.face_groups[i] != group {
                    continue;
                }
                faces.push((
                    face.iter()
                        .map(|&v| source_map[v as usize] + (copy * source.len()) as u32)
                        .collect::<Vec<_>>(),
                    *material,
                ));
                face_uvs.push(g.face_uvs[i].clone());
                face_groups.push(group);
            }
        }
    }
    check_cleanup(&positions, &faces, &face_groups, groups);
    let mut remap: Vec<_> = (0..positions.len()).collect();
    for &group in groups {
        let mut points = Vec::new();
        for ((face, _), &fg) in faces.iter().zip(&face_groups).rev() {
            if fg == group {
                for &v in face {
                    if !points.contains(&(v as usize)) {
                        points.push(v as usize);
                    }
                }
            }
        }
        if points.len() >= 3000 {
            continue;
        }
        let mut i = 0;
        while i < points.len() {
            let mut j = i + 1;
            while j < points.len() {
                let d = difference(positions[points[i]], positions[points[j]]);
                if (d[1] * d[1] + d[0] * d[0]) + d[2] * d[2] < EPS {
                    remap[points[j]] = points[i];
                    points.swap_remove(j);
                } else {
                    j += 1;
                }
            }
            i += 1;
        }
    }
    for (face, _) in &mut faces {
        for v in face {
            *v = remap[*v as usize] as u32;
        }
    }
    g.positions = positions;
    g.faces = faces;
    g.face_uvs = face_uvs;
    g.face_groups = face_groups;
}

fn face_normal(positions: &[[f32; 3]], face: &[u32]) -> [f32; 3] {
    let mut normal = [0.0; 3];
    let origin = positions[face[0] as usize];
    for pair in face[1..].windows(2) {
        let a = difference(positions[pair[0] as usize], origin);
        let b = difference(positions[pair[1] as usize], origin);
        let n = cross(a, b);
        for i in 0..3 {
            normal[i] += n[i];
        }
    }
    normalize(normal, false)
}

fn on_edge(p: [f32; 3], a: [f32; 3], b: [f32; 3]) -> bool {
    let d = difference(b, a);
    let len = length(d);
    let inv = 1.0 / len;
    let v = difference(p, a);
    let projection = ((inv * d[1]) * v[1] + (inv * d[0]) * v[0]) + (inv * d[2]) * v[2];
    projection >= -EPS && projection <= len + EPS && (length(v) - projection).abs() <= EPS
}

fn check_cleanup(positions: &[[f32; 3]], faces: &[(Vec<u32>, i32)], face_groups: &[usize], groups: &[usize]) {
    for &group in groups {
        let part: Vec<_> =
            faces.iter().zip(face_groups).filter(|(_, g)| **g == group).map(|(f, _)| &f.0).collect();
        if part.len() < 500 {
            let normals: Vec<_> = part.iter().map(|f| face_normal(positions, f)).collect();
            for (i, a) in part.iter().enumerate() {
                let n = normals[i];
                if n[0] * n[0] + n[1] * n[1] + n[2] * n[2] <= TINY {
                    continue;
                }
                let p = positions[a[0] as usize];
                let plane = (n[1] * p[1] + n[0] * p[0]) + n[2] * p[2];
                for (j, b) in part[..i].iter().enumerate() {
                    if !(0..3).all(|k| near(n[k], -normals[j][k])) {
                        continue;
                    }
                    let p = positions[b[0] as usize];
                    if (((n[1] * p[1] + n[0] * p[0]) + n[2] * p[2]) - plane).abs() <= EPS {
                        todo!("crystal deformation coplanar face surgery");
                    }
                }
            }
        }
        let mut points = Vec::new();
        let mut edges = Vec::new();
        for face in part {
            for i in 0..face.len() {
                let a = face[i];
                let b = face[(i + 1) % face.len()];
                if !points.contains(&a) {
                    points.push(a);
                }
                let edge = if a < b { (a, b) } else { (b, a) };
                if !edges.contains(&edge) {
                    edges.push(edge);
                }
            }
        }
        if points.len() >= 1000 {
            continue;
        }
        for v in points {
            let p = positions[v as usize];
            for &(a, b) in &edges {
                if v == a || v == b {
                    continue;
                }
                let a = positions[a as usize];
                let b = positions[b as usize];
                if (0..3).all(|i| near(p[i], a[i])) || (0..3).all(|i| near(p[i], b[i])) {
                    continue;
                }
                if on_edge(p, a, b) {
                    todo!("crystal deformation point/edge surgery");
                }
            }
        }
    }
}
