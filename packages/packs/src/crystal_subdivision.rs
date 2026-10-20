use crate::crystal::{Crystal, Geometry, next_age};
use crate::crystal_deformation::face_normal;
use crate::crystal_modifiers::{Layer, creatable, selected_groups};
use std::collections::{HashMap, HashSet};

type Uv = [f32; 2];
type Made = (Vec<u32>, i32, Vec<Uv>, usize, (u32, u32));

fn pair(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

fn halfway(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| (b[i] - a[i]) * 0.5 + a[i])
}

fn halfway_uv(a: Uv, b: Uv) -> Uv {
    std::array::from_fn(|i| (b[i] - a[i]) * 0.5 + a[i])
}

fn squared(d: [f32; 3]) -> f32 {
    (d[0] * d[0] + d[1] * d[1]) + d[2] * d[2]
}

fn difference(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}

struct Edges {
    ends: Vec<[u32; 2]>,
    by_pair: HashMap<(u32, u32), usize>,
}

impl Edges {
    fn of(g: &Geometry) -> Self {
        let mut edges = Self { ends: Vec::new(), by_pair: HashMap::new() };
        for &[a, b] in &g.edges {
            edges.add(a, b);
        }
        for (face, _) in g.faces.iter().filter(|(face, _)| creatable(face)) {
            edges.sides(face);
        }
        edges
    }
    fn without(self, removed: &HashSet<usize>) -> Vec<[u32; 2]> {
        let kept = self.ends.into_iter().enumerate().filter(|(e, _)| !removed.contains(e));
        kept.map(|(_, ends)| ends).collect()
    }
    fn add(&mut self, a: u32, b: u32) -> usize {
        self.ends.push([a, b]);
        *self.by_pair.entry(pair(a, b)).or_insert(self.ends.len() - 1)
    }
    fn find(&mut self, a: u32, b: u32) -> usize {
        match self.by_pair.get(&pair(a, b)) {
            Some(&e) => e,
            None => self.add(a, b),
        }
    }
    fn sides(&mut self, face: &[u32]) {
        for (i, &a) in face.iter().enumerate() {
            self.find(a, face[(i + 1) % face.len()]);
        }
    }
}

struct Split {
    edges: Edges,
    middles: HashMap<usize, u32>,
    split: HashSet<usize>,
    removed: Vec<usize>,
    made: Vec<Made>,
}

impl Split {
    fn of(g: &Geometry) -> Self {
        Self {
            edges: Edges::of(g),
            middles: HashMap::new(),
            split: HashSet::new(),
            removed: Vec::new(),
            made: Vec::new(),
        }
    }
}

struct Work<'a> {
    g: &'a mut Geometry,
    s: &'a mut Split,
    age: (u32, u32),
}

impl Work<'_> {
    fn vertex(&mut self, p: [f32; 3]) -> u32 {
        self.g.positions.push(p);
        self.g.positions.len() as u32 - 1
    }
    fn middle(&mut self, edge: usize) -> u32 {
        if let Some(&m) = self.s.middles.get(&edge) {
            return m;
        }
        let [a, b] = self.s.edges.ends[edge];
        let m = self.vertex(halfway(self.g.positions[a as usize], self.g.positions[b as usize]));
        self.s.edges.add(m, a);
        self.s.edges.add(m, b);
        self.s.middles.insert(edge, m);
        self.s.split.insert(edge);
        m
    }
    fn face(&mut self, corners: Vec<u32>, uvs: Vec<Uv>, f: usize) {
        self.s.edges.sides(&corners);
        let (_, material) = self.g.faces[f];
        self.s.made.push((corners, material, uvs, self.g.face_groups[f], self.age));
        self.age.1 += 1;
    }
}

fn corner_uv(uvs: &[Uv], face: &[u32], v: u32) -> Uv {
    face.iter().position(|&c| c == v).and_then(|i| uvs.get(i)).copied().unwrap_or_default()
}

fn subdivide_face(w: &mut Work, f: usize) -> bool {
    let face = w.g.faces[f].0.clone();
    let uvs = w.g.face_uvs[f].clone();
    let n = face.len();
    let edges: Vec<usize> = (0..n).map(|i| w.s.edges.find(face[i], face[(i + 1) % n])).collect();
    let at = |w: &Work, v: u32| w.g.positions[v as usize];
    let ends = |w: &Work, e: usize| {
        let [a, b] = w.s.edges.ends[e];
        (at(w, a), at(w, b))
    };
    if !edges.iter().any(|&e| {
        let (a, b) = ends(w, e);
        squared(difference(b, a)).sqrt() > 0.0
    }) {
        return false;
    }
    let uv_of = |w: &Work, e: usize| {
        let [a, b] = w.s.edges.ends[e];
        halfway_uv(corner_uv(&uvs, &face, a), corner_uv(&uvs, &face, b))
    };
    let corner_uv = |c: usize| uvs.get(c).copied().unwrap_or_default();
    if n == 3 {
        let m: Vec<[f32; 3]> = edges
            .iter()
            .map(|&e| {
                let (a, b) = ends(w, e);
                halfway(a, b)
            })
            .collect();
        let a = squared(difference(m[0], m[1]));
        let b = squared(difference(m[0], m[2]));
        let c = squared(difference(m[1], m[2]));
        let short = if a > b * 4.0 {
            Some(1)
        } else if b > a * 4.0 {
            Some(2)
        } else if b > c * 4.0 || a > c * 4.0 {
            Some(0)
        } else {
            None
        };
        if let Some(short) = short {
            for j in (0..3).filter(|&j| j != short) {
                w.middle(edges[j]);
            }
            let c = (short + 2) % 3;
            let (next, prev) = (edges[c], edges[(c + 2) % 3]);
            let (mn, mp) = (w.s.middles[&next], w.s.middles[&prev]);
            let (un, up) = (uv_of(w, next), uv_of(w, prev));
            w.face(vec![face[c], mn, mp], vec![corner_uv(c), un, up], f);
            let (c1, c2) = ((c + 1) % 3, (c + 2) % 3);
            w.face(vec![mn, face[c1], face[c2], mp], vec![un, corner_uv(c1), corner_uv(c2), up], f);
        } else {
            let mids: Vec<u32> = edges.iter().map(|&e| w.middle(e)).collect();
            let mid_uvs: Vec<Uv> = edges.iter().map(|&e| uv_of(w, e)).collect();
            let corners = [face[0], mids[0], face[1], mids[1], face[2], mids[2]];
            let corner_uvs = [corner_uv(0), mid_uvs[0], corner_uv(1), mid_uvs[1], corner_uv(2), mid_uvs[2]];
            for t in [[0, 1, 5], [1, 2, 3], [3, 4, 5], [1, 3, 5]] {
                w.face(t.map(|i| corners[i]).to_vec(), t.map(|i| corner_uvs[i]).to_vec(), f);
            }
        }
    } else {
        let mids: Vec<u32> = edges.iter().map(|&e| w.middle(e)).collect();
        let mid_uvs: Vec<Uv> = edges.iter().map(|&e| uv_of(w, e)).collect();
        let center = scaled(sum(mids.iter().map(|&m| at(w, m))), 0.25);
        let z = w.vertex(center);
        for &m in &mids {
            w.s.edges.add(m, z);
        }
        let points: [[f32; 3]; 4] = std::array::from_fn(|i| at(w, face[i]));
        let face_uvs: [Uv; 4] = std::array::from_fn(corner_uv);
        let z_uv = quad_uv(points, face_uvs, center);
        let mut corners = Vec::new();
        let mut corner_uvs = Vec::new();
        for c in 0..4 {
            corners.extend([face[c], mids[c]]);
            corner_uvs.extend([face_uvs[c], mid_uvs[c]]);
        }
        corners.push(z);
        corner_uvs.push(z_uv);
        for q in [[0, 1, 8, 7], [1, 2, 3, 8], [3, 4, 5, 8], [5, 6, 7, 8]] {
            w.face(q.map(|i| corners[i]).to_vec(), q.map(|i| corner_uvs[i]).to_vec(), f);
        }
    }
    true
}

fn triangle_uv(p: [[f32; 3]; 3], uv: [Uv; 3], q: [f32; 3]) -> Uv {
    let u = difference(p[1], p[2]);
    let w = difference(p[0], p[2]);
    let d = difference(q, p[2]);
    let uu = (u[1] * u[1] + u[0] * u[0]) + u[2] * u[2];
    let uw = (u[1] * w[1] + u[0] * w[0]) + u[2] * w[2];
    let du = (d[0] * u[0] + u[1] * d[1]) + u[2] * d[2];
    let dw = (d[0] * w[0] + w[1] * d[1]) + w[2] * d[2];
    let ww = (w[1] * w[1] + w[0] * w[0]) + w[2] * w[2];
    let s = (dw * uu - du * uw) / (ww * uu - uw * uw);
    let t = (du - uw * s) / uu;
    std::array::from_fn(|i| ((uv[1][i] - uv[2][i]) * t + (uv[0][i] - uv[2][i]) * s) + uv[2][i])
}

fn quad_uv(p: [[f32; 3]; 4], uv: [Uv; 4], q: [f32; 3]) -> Uv {
    let a = difference(p[3], p[0]);
    let b = difference(p[1], p[0]);
    let c = difference(p[3], p[1]);
    let nx = a[2] * b[1] - a[1] * b[2];
    let nz = a[1] * b[0] - a[0] * b[1];
    let ny = a[0] * b[2] - a[2] * b[0];
    let d = difference(q, p[1]);
    let side =
        ((nz * c[0] - nx * c[2]) * d[1] + (ny * c[2] - nz * c[1]) * d[0]) + (nx * c[1] - ny * c[0]) * d[2];
    if side < 0.0 {
        triangle_uv([p[2], p[3], p[1]], [uv[2], uv[3], uv[1]], q)
    } else {
        triangle_uv([p[0], p[1], p[3]], [uv[0], uv[1], uv[3]], q)
    }
}

fn selected_faces(
    geometry: &[Geometry],
    groups: &[(bool, usize, usize)],
    trigger: bool,
) -> Vec<(usize, usize)> {
    let mut faces = Vec::new();
    for &(t, index, group) in groups {
        if t != trigger {
            continue;
        }
        let g = &geometry[index];
        for f in (0..g.faces.len()).rev() {
            if g.face_groups[f] == group && creatable(&g.faces[f].0) {
                faces.push((index, f));
            }
        }
    }
    faces
}

fn add_faces(g: &mut Geometry, made: Vec<Made>) {
    for (corners, material, uvs, group, age) in made {
        g.normals.push(face_normal(&g.positions, &corners));
        g.faces.push((corners, material));
        g.face_uvs.push(uvs);
        g.face_groups.push(group);
        g.ages.push(age);
    }
}

fn sides_in_use(g: &Geometry) -> HashSet<(u32, u32)> {
    let mut used = HashSet::new();
    for (face, _) in g.faces.iter().filter(|(face, _)| creatable(face)) {
        for (i, &a) in face.iter().enumerate() {
            used.insert(pair(a, face[(i + 1) % face.len()]));
        }
    }
    used
}

pub(crate) fn subdivide(c: &mut Crystal, layers: &[Layer], refs: &[(u32, Option<String>)]) {
    let groups = selected_groups(layers, refs);
    for trigger in [false, true] {
        let geometry = if trigger { &mut c.triggers } else { &mut c.geometry };
        let faces = selected_faces(geometry, &groups, trigger);
        let mut age = (next_age(geometry), 0);
        let mut splits: HashMap<usize, Split> = HashMap::new();
        for (index, f) in faces {
            let s = splits.entry(index).or_insert_with(|| Split::of(&geometry[index]));
            let mut w = Work { g: &mut geometry[index], s, age };
            if subdivide_face(&mut w, f) {
                w.s.removed.push(f);
            }
            age = w.age;
        }
        for (index, s) in splits {
            let g = &mut geometry[index];
            g.remove_faces(&s.removed);
            add_faces(g, s.made);
            let used = sides_in_use(g);
            let unused: HashSet<usize> = s
                .split
                .into_iter()
                .filter(|&e| {
                    let [a, b] = s.edges.ends[e];
                    !used.contains(&pair(a, b))
                })
                .collect();
            g.edges = s.edges.without(&unused);
        }
    }
}

pub(crate) fn smooth(c: &mut Crystal, layers: &[Layer], refs: &[(u32, Option<String>)], levels: u32) {
    for _ in 0..levels {
        let groups = selected_groups(layers, refs);
        for trigger in [false, true] {
            let geometry = if trigger { &mut c.triggers } else { &mut c.geometry };
            let faces = selected_faces(geometry, &groups, trigger);
            let major = next_age(geometry);
            let mut ages = vec![(0, 0); faces.len()];
            let mut next = 0;
            for (p, &(index, f)) in faces.iter().enumerate().rev() {
                ages[p] = (major, next);
                next += geometry[index].faces[f].0.len() as u32;
            }
            let mut indices: Vec<usize> = faces.iter().map(|&(i, _)| i).collect();
            indices.sort_unstable();
            indices.dedup();
            for index in indices {
                let selected: Vec<(usize, (u32, u32))> = faces
                    .iter()
                    .zip(&ages)
                    .filter(|((i, _), _)| *i == index)
                    .map(|(&(_, f), &age)| (f, age))
                    .collect();
                smooth_geometry(&mut geometry[index], &selected);
            }
        }
    }
}

fn sum(points: impl IntoIterator<Item = [f32; 3]>) -> [f32; 3] {
    let mut s = [0.0f32; 3];
    for p in points {
        for i in 0..3 {
            s[i] += p[i];
        }
    }
    s
}

fn scaled(p: [f32; 3], k: f32) -> [f32; 3] {
    p.map(|v| v * k)
}

fn smooth_geometry(g: &mut Geometry, selected: &[(usize, (u32, u32))]) {
    let mut edges = Edges::of(g);
    let chosen: HashSet<usize> = selected.iter().map(|&(f, _)| f).collect();
    let sides: Vec<Vec<usize>> = g
        .faces
        .iter()
        .map(|(face, _)| {
            if !creatable(face) {
                return Vec::new();
            }
            (0..face.len()).map(|i| edges.find(face[i], face[(i + 1) % face.len()])).collect()
        })
        .collect();
    let mut faces_of_edge = vec![Vec::new(); edges.ends.len()];
    let mut faces_of = vec![Vec::new(); g.positions.len()];
    for (f, (face, _)) in g.faces.iter().enumerate() {
        for &e in &sides[f] {
            faces_of_edge[e].push(f);
        }
        if creatable(face) {
            for &v in face {
                faces_of[v as usize].push(f);
            }
        }
    }
    let mut edges_of = vec![Vec::new(); g.positions.len()];
    for (e, &[a, b]) in edges.ends.iter().enumerate() {
        edges_of[a as usize].push(e);
        edges_of[b as usize].push(e);
    }
    let mut edge_list = Vec::new();
    let mut vertex_list = Vec::new();
    for &(f, _) in selected {
        for &e in &sides[f] {
            if !edge_list.contains(&e) {
                edge_list.push(e);
            }
        }
        for &v in &g.faces[f].0 {
            if !vertex_list.contains(&v) {
                vertex_list.push(v);
            }
        }
    }
    let in_edges: HashSet<usize> = edge_list.iter().copied().collect();
    let at = |g: &Geometry, v: u32| g.positions[v as usize];
    let mut face_points = HashMap::new();
    for &(f, _) in selected {
        let face = &g.faces[f].0;
        let c = scaled(sum(face.iter().map(|&v| at(g, v))), 1.0 / face.len() as f32);
        g.positions.push(c);
        face_points.insert(f, g.positions.len() as u32 - 1);
    }
    let mut edge_points = HashMap::new();
    let mut boundary = HashSet::new();
    for &e in &edge_list {
        let [a, b] = edges.ends[e];
        let (pa, pb) = (at(g, a), at(g, b));
        let mut s: [f32; 3] = std::array::from_fn(|i| pa[i] + pb[i]);
        let around: Vec<usize> = faces_of_edge[e].iter().copied().filter(|f| chosen.contains(f)).collect();
        let mut divisor = 2.0f32;
        if around.len() == 2 {
            for f in around {
                let p = at(g, face_points[&f]);
                for i in 0..3 {
                    s[i] += p[i];
                }
            }
            divisor = 4.0;
        } else if around.len() == 1 {
            boundary.insert(e);
        }
        g.positions.push(scaled(s, 1.0 / divisor));
        edge_points.insert(e, g.positions.len() as u32 - 1);
    }
    for &(f, _) in selected {
        for &e in &sides[f] {
            edges.add(face_points[&f], edge_points[&e]);
        }
    }
    let mut moved = Vec::new();
    for &v in &vertex_list {
        let around = &faces_of[v as usize];
        let spokes = &edges_of[v as usize];
        let p = at(g, v);
        let interior = around.iter().all(|f| chosen.contains(f))
            && spokes.iter().all(|e| in_edges.contains(e))
            && around.len() == spokes.len();
        let q = if interior {
            if around.len() < 3 {
                continue;
            }
            let faces = scaled(sum(around.iter().map(|f| at(g, face_points[f]))), 1.0 / around.len() as f32);
            let middles = sum(spokes.iter().map(|&e| {
                let [a, b] = edges.ends[e];
                let (pa, pb) = (at(g, a), at(g, b));
                std::array::from_fn(|i| (pa[i] + pb[i]) * 0.5)
            }));
            let k = spokes.len() as f32;
            let inverse = 1.0 / k;
            let weight = (spokes.len() - 3) as f32;
            std::array::from_fn(|i| {
                let m = inverse * middles[i];
                ((weight * p[i] + faces[i]) + (m + m)) * inverse
            })
        } else {
            let rim: Vec<u32> =
                spokes.iter().filter(|e| boundary.contains(e)).map(|e| edge_points[e]).collect();
            let mut s = p;
            for &m in &rim {
                let pm = at(g, m);
                for i in 0..3 {
                    s[i] += pm[i];
                }
            }
            let inverse = 1.0 / (rim.len() + 1) as f32;
            s.map(|v| inverse * v)
        };
        moved.push((v, q));
    }
    for (v, q) in moved {
        g.positions[v as usize] = q;
    }
    let mut kept = HashSet::new();
    for &e in edge_list.iter().rev() {
        let [a, b] = edges.ends[e];
        let m = edge_points[&e];
        edges.add(a, m);
        edges.add(m, b);
        if boundary.contains(&e) && faces_of_edge[e].iter().any(|f| !chosen.contains(f)) {
            kept.insert(e);
        }
    }
    let mut made = Vec::new();
    for &(f, (major, first)) in selected.iter().rev() {
        let (face, material) = g.faces[f].clone();
        let uvs = &g.face_uvs[f];
        let n = face.len();
        let average = if uvs.is_empty() {
            [0.0; 2]
        } else {
            let mut s = [0.0f32; 2];
            for uv in uvs {
                s[0] += uv[0];
                s[1] += uv[1];
            }
            let inverse = 1.0 / n as f32;
            s.map(|v| v * inverse)
        };
        for i in 0..n {
            let next = sides[f][i];
            let prev = sides[f][(i + n - 1) % n];
            let corners = vec![face[i], edge_points[&next], face_points[&f], edge_points[&prev]];
            edges.sides(&corners);
            let corner_uvs = if uvs.is_empty() {
                Vec::new()
            } else {
                let mean = |a: Uv, b: Uv| -> Uv { [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5] };
                let before = ((i as u64).wrapping_sub(1) % n as u64) as usize;
                vec![uvs[i], mean(uvs[i], uvs[(i + 1) % n]), average, mean(uvs[i], uvs[before])]
            };
            made.push((corners, material, corner_uvs, g.face_groups[f], (major, first + i as u32)));
        }
    }
    let removed: Vec<usize> = selected.iter().map(|&(f, _)| f).collect();
    g.remove_faces(&removed);
    add_faces(g, made);
    let deleted: HashSet<usize> = edge_list.into_iter().filter(|e| !kept.contains(e)).collect();
    g.edges = edges.without(&deleted);
}
