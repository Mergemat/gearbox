use crate::crystal::Geometry;
use anyhow::{Context, Result, ensure};
use gearbox_gbx::reader::Reader;

#[path = "crystal_voxel_cases.rs"]
mod cases;

struct Case {
    points: &'static [[f32; 3]],
    links: [i8; 6],
    faces: [i8; 3],
}

struct Voxel {
    material: i32,
    index: usize,
    group: usize,
}

pub(crate) struct Voxels {
    size: [usize; 3],
    width: f32,
    origin: [f32; 3],
    cells: Vec<Voxel>,
    pub visible: bool,
    pub collidable: bool,
    pub groups: usize,
}

fn index(p: [usize; 3], size: [usize; 3]) -> usize {
    (p[0] * size[1] + p[1]) * size[2] + p[2]
}

impl Voxels {
    pub(crate) fn read(r: &mut Reader<'_>) -> Result<Self> {
        let version = r.u32()?;
        ensure!((6..=7).contains(&version), "crystal voxel version {version}");
        let size = [r.len()?, r.len()?, r.len()?];
        ensure!(size.iter().all(|&n| n > 0), "crystal voxel dimensions");
        let width = r.f32()?;
        let origin = r.vec3()?;
        r.u32()?;
        let mut cells = Vec::new();
        for _ in 0..r.len()? {
            let material = (r.u32()? & 0x7fff_ffff) as i32;
            for _ in 0..r.len()? {
                let index = r.u32()? as usize;
                ensure!(index < size.iter().product(), "crystal voxel index");
                r.u32()?;
                if version >= 7 {
                    r.skip(4)?;
                }
                cells.push(Voxel { material, index, group: usize::MAX });
            }
        }
        let visible = r.bool32()?;
        let collidable = r.bool32()?;
        let groups = r.len()?;
        let mut out = Self { size, width, origin, cells, visible, collidable, groups };
        let lookup = out.lookup();
        for group in 0..groups {
            let start = r.u32()? as usize;
            if start != u32::MAX as usize {
                for cell in out.component(start, &lookup) {
                    out.cells[cell].group = group;
                }
            }
        }
        if out.groups == 0 && !out.cells.is_empty() {
            for i in 0..out.cells.len() {
                if out.cells[i].group == usize::MAX {
                    for cell in out.component(out.cells[i].index, &lookup) {
                        out.cells[cell].group = out.groups;
                    }
                    out.groups += 1;
                }
            }
        }
        Ok(out)
    }

    fn lookup(&self) -> Vec<usize> {
        let mut lookup = vec![usize::MAX; self.size.iter().product()];
        for (i, cell) in self.cells.iter().enumerate() {
            lookup[cell.index] = i;
        }
        lookup
    }

    fn component(&self, start: usize, lookup: &[usize]) -> Vec<usize> {
        if start >= lookup.len() || lookup[start] == usize::MAX {
            return Vec::new();
        }
        let mut visited = vec![false; lookup.len()];
        let mut queue = vec![start];
        visited[start] = true;
        let mut at = 0;
        while at < queue.len() {
            let i = queue[at];
            let p = [i / (self.size[1] * self.size[2]), i / self.size[2] % self.size[1], i % self.size[2]];
            for axis in 0..3 {
                for direction in [-1, 1] {
                    let mut q = p;
                    let n = p[axis] as isize + direction;
                    if n < 0 || n >= self.size[axis] as isize {
                        continue;
                    }
                    q[axis] = n as usize;
                    let j = index(q, self.size);
                    if lookup[j] != usize::MAX && !visited[j] {
                        visited[j] = true;
                        queue.push(j);
                    }
                }
            }
            at += 1;
        }
        queue.into_iter().map(|i| lookup[i]).collect()
    }

    pub(crate) fn compile(&self) -> Result<Geometry> {
        let size = self.size.map(|n| n + 1);
        let lookup = self.lookup();
        let mut lattice = vec![Vec::<(u32, [f32; 3])>::new(); size.iter().product()];
        let mut positions = Vec::new();
        let mut adjacent = Vec::<Vec<(u32, usize)>>::new();
        let mut edges = Vec::<[u32; 2]>::new();
        let mut faces = Vec::new();
        let mut face_groups = Vec::new();
        for x in 0..size[0] {
            for y in 0..size[1] {
                for z in 0..size[2] {
                    let p = [x, y, z];
                    let mut mask = 0;
                    for (bit, delta) in [
                        [-1, -1, -1],
                        [0, -1, -1],
                        [0, -1, 0],
                        [-1, -1, 0],
                        [-1, 0, -1],
                        [0, 0, -1],
                        [0, 0, 0],
                        [-1, 0, 0],
                    ]
                    .iter()
                    .enumerate()
                    {
                        let q: [isize; 3] = std::array::from_fn(|a| p[a] as isize + delta[a]);
                        if (0..3).all(|a| q[a] >= 0 && q[a] < self.size[a] as isize)
                            && lookup[index(q.map(|n| n as usize), self.size)] != usize::MAX
                        {
                            mask |= 1 << bit;
                        }
                    }
                    if mask == 0 || mask == 255 {
                        continue;
                    }
                    let case = &cases::CASES[mask];
                    let cell = index(p, size);
                    for &label in case.points {
                        let id = positions.len() as u32;
                        let local = [
                            (x as f32 - self.size[0] as f32 * 0.5) * self.width,
                            y as f32 * self.width,
                            (z as f32 - self.size[2] as f32 * 0.5) * self.width,
                        ];
                        let world = [
                            ((local[1] * 0.0 + local[0] * 1.0) + local[2] * 0.0) + 0.0 + self.origin[0],
                            ((local[1] * 1.0 + local[0] * 0.0) + local[2] * 0.0) + 0.0 + self.origin[1],
                            ((local[1] * 0.0 + local[0] * 0.0) + local[2] * 1.0) + 0.0 + self.origin[2],
                        ];
                        positions.push(world);
                        adjacent.push(Vec::new());
                        lattice[cell].push((id, label));
                    }
                    for (side, &link) in case.links.iter().enumerate() {
                        let axis = side % 3;
                        if link < 0 || p[axis] == 0 {
                            continue;
                        }
                        let mut q = p;
                        q[axis] -= 1;
                        let label = case.points[link as usize];
                        let mut shifted = label;
                        shifted[axis] += 1.0;
                        let previous = lattice[index(q, size)]
                            .iter()
                            .min_by(|a, b| {
                                let distance = |v: &[f32; 3]| {
                                    let mut v = *v;
                                    v[axis] -= 1.0;
                                    ((shifted[1] - v[1]).abs() + (shifted[0] - v[0]).abs())
                                        + (shifted[2] - v[2]).abs()
                                };
                                distance(&a.1).total_cmp(&distance(&b.1))
                            })
                            .context("crystal voxel connection")?
                            .0;
                        let current = lattice[cell]
                            .iter()
                            .find(|(_, v)| {
                                (0..3).all(|a| {
                                    near(v[a], if a == axis { shifted[a] - 1.0 } else { shifted[a] })
                                })
                            })
                            .context("crystal voxel point")?
                            .0;
                        if !adjacent[previous as usize].iter().any(|n| n.0 == current) {
                            let delta: [f32; 3] = std::array::from_fn(|a| {
                                (positions[current as usize][a] - positions[previous as usize][a]).abs()
                            });
                            let direction = if delta[1] > delta[0] {
                                if delta[1] < delta[2] { 2 } else { 1 }
                            } else if delta[0] < delta[2] {
                                2
                            } else {
                                0
                            };
                            adjacent[previous as usize].push((current, direction));
                            adjacent[current as usize].push((previous, direction));
                            edges.push([previous, current]);
                        }
                    }
                    for axis in 0..3 {
                        let sign = case.faces[axis];
                        if sign == 0 {
                            continue;
                        }
                        let representative = lattice[cell]
                            .iter()
                            .min_by(|a, b| {
                                (a.1[axis] - sign as f32).abs().total_cmp(&(b.1[axis] - sign as f32).abs())
                            })
                            .context("crystal voxel face point")?
                            .0;
                        let neighbors = &adjacent[representative as usize];
                        let a = (axis + 1) % 3;
                        let b = (axis + 2) % 3;
                        let first =
                            neighbors.iter().rev().find(|n| n.1 == a).context("crystal voxel face edge")?.0;
                        let last =
                            neighbors.iter().rev().find(|n| n.1 == b).context("crystal voxel face edge")?.0;
                        let diagonal = neighbors
                            .iter()
                            .find_map(|&(v, direction)| {
                                let other = if direction == a {
                                    b
                                } else if direction == b {
                                    a
                                } else {
                                    return None;
                                };
                                adjacent[v as usize].iter().find(|n| n.1 == other).map(|n| n.0)
                            })
                            .context("crystal voxel face corner")?;
                        let vertices = if sign > 0 {
                            vec![representative, last, diagonal, first]
                        } else {
                            vec![representative, first, diagonal, last]
                        };
                        let mut source = p;
                        for n in &mut source {
                            *n = n.wrapping_sub(1);
                        }
                        if sign > 0 {
                            source[axis] = p[axis];
                        }
                        let source =
                            *lookup.get(index(source, self.size)).context("crystal voxel face source")?;
                        let voxel = self.cells.get(source).context("crystal voxel face cell")?;
                        faces.push((vertices, voxel.material));
                        face_groups.push(if voxel.group == usize::MAX { 0 } else { voxel.group });
                    }
                }
            }
        }
        positions.reverse();
        faces.reverse();
        face_groups.reverse();
        edges.reverse();
        let remap = |v: &mut u32| *v = positions.len() as u32 - 1 - *v;
        for (face, _) in &mut faces {
            face.iter_mut().for_each(remap);
        }
        for edge in &mut edges {
            edge.iter_mut().for_each(remap);
        }
        let normals =
            faces.iter().map(|(f, _)| crate::crystal_deformation::face_normal(&positions, f)).collect();
        let face_uvs = faces.iter().map(|(f, _)| vec![[0.0; 2]; f.len()]).collect();
        Ok(Geometry {
            positions,
            faces,
            face_uvs,
            face_groups,
            normals,
            group_parents: vec![u32::MAX; self.groups],
            edges,
            ages: Vec::new(),
            archive_version: 37,
        })
    }
}

fn near(a: f32, b: f32) -> bool {
    (a - b).abs() <= crate::reader_math::maxss(1.0, crate::reader_math::maxss(a.abs(), b.abs())) * 0.00001
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voxel_meshes() {
        for (data, vertices, faces, expected) in [
            (
                include_bytes!("../tests/data/voxel-components.bin").as_slice(),
                736,
                716,
                0x03bedec35f486095u64,
            ),
            (include_bytes!("../tests/data/voxel-connected.bin").as_slice(), 380, 378, 0xc549da478ee669e5u64),
        ] {
            let mut r = Reader::new(data);
            let g = Voxels::read(&mut r).unwrap().compile().unwrap();
            assert_eq!(r.pos(), data.len());
            assert_eq!((g.positions.len(), g.faces.len()), (vertices, faces));
            let mut bytes = Vec::new();
            for ((face, material), &group) in g.faces.iter().zip(&g.face_groups).rev() {
                bytes.extend((group as u32).to_le_bytes());
                bytes.extend(material.to_le_bytes());
                bytes.extend((face.len() as u32).to_le_bytes());
                for &v in face {
                    bytes.extend(g.positions[v as usize].into_iter().flat_map(f32::to_le_bytes));
                }
            }
            let hash = bytes
                .into_iter()
                .fold(0xcbf29ce484222325u64, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3));
            assert_eq!(hash, expected);
        }
    }
}
