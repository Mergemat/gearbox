use anyhow::{Result, ensure};
use gearbox_engine::{Aabb, Vec3, math::{minss, maxss}, static_world::{Convex, Face}};

fn dot(a: Vec3, b: Vec3) -> f32 { (a.y * b.y + a.x * b.x) + a.z * b.z }

fn axis(axes: &mut Vec<Vec3>, n: Vec3) -> u32 {
    if let Some(i) = axes.iter().position(|&a| {
        let delta = (1.0 - dot(n, a).abs()).abs();
        delta.is_nan() || delta < 0.00001
    }) { i as u32 }
    else { axes.push(n); axes.len() as u32 - 1 }
}

pub(super) fn bounds(hull: &mut Convex) {
    let mut lo = hull.vertices[0];
    let mut hi = lo;
    for v in &hull.vertices[1..] {
        for k in 0..3 {
            lo[k] = minss(v[k], lo[k]);
            hi[k] = maxss(v[k], hi[k]);
        }
    }
    hull.bounds = Aabb::new((hi + lo) * 0.5, (hi - lo) * 0.5);
}

pub(super) fn read(raw: &[u8], material: u16) -> Result<Convex> {
    let mut reader = gearbox_gbx::reader::Reader::new(raw);
    ensure!(reader.u32()? == 0, "unsupported convex archive version");
    ensure!(reader.u32()? == 0, "unsupported procedural convex archive");
    let center = Vec3::from_array(reader.vec3()?);
    let half = Vec3::from_array(reader.vec3()?);
    let n = reader.u32()?;
    let vertices = (0..n).map(|_| reader.vec3().map(Vec3::from_array)).collect::<Result<Vec<_>>>()?;
    ensure!(!vertices.is_empty(), "unsupported empty convex hull");
    let n = reader.u32()?;
    let indices = (0..n).map(|_| reader.u32()).collect::<Result<Vec<_>>>()?;
    let n = reader.u32()?;
    let ranges = (0..n).map(|_| Ok((reader.u32()?, reader.u32()?))).collect::<Result<Vec<_>>>()?;
    ensure!(indices.iter().all(|&i| (i as usize) < vertices.len()), "convex vertex index out of bounds");
    let mut hull = Convex { material, vertices, indices, faces: Vec::new(), edges: Vec::new(),
        normals: Vec::new(), directions: Vec::new(), bounds: Aabb::new(center, half) };
    for (start, count) in ranges {
        ensure!(
            count >= 3 && start as u64 + count as u64 <= hull.indices.len() as u64,
            "convex face range out of bounds",
        );
        let ids = &hull.indices[start as usize..(start + count) as usize];
        let [v0, v1, v2] = [ids[0], ids[1], ids[2]].map(|i| hull.vertices[i as usize]);
        let mut normal = cross(v1 - v0, v2 - v0);
        let length = (normal.x * normal.x + normal.y * normal.y) + normal.z * normal.z;
        if length > 1.00000005e-20 {
            let inv = 1.0 / length.sqrt();
            normal = normal * inv;
        }
        let id = axis(&mut hull.normals, normal);
        hull.faces.push(Face { normal, offset: ((-normal.x * v0.x) - normal.y * v0.y) - normal.z * v0.z,
            axis: id, positive: dot(normal, hull.normals[id as usize]) > 0.0, start, count });
        for k in 0..ids.len() {
            let (a, b) = (ids[k], ids[(k + 1) % ids.len()]);
            if !hull.edges.iter().any(|e| (e[0] == a && e[1] == b) || (e[0] == b && e[1] == a)) {
                hull.edges.push([a, b, 0]);
            }
        }
    }
    for e in &mut hull.edges {
        let mut d = hull.vertices[e[1] as usize] - hull.vertices[e[0] as usize];
        let length = dot(d, d);
        if length > 1.0e-10 && f32::MAX > length { d = d * (1.0 / length.sqrt()); }
        e[2] = axis(&mut hull.directions, d);
    }
    Ok(hull)
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}
