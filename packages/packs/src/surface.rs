use anyhow::{Result, bail, ensure};
use gearbox_gbx::reader::Reader;
use serde::{Deserialize, Serialize};

pub type Vec3 = [f32; 3];
pub type Iso4 = [f32; 12];

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshTri {
    pub v: [i32; 3],
    pub flags: [u8; 2],
    pub material: i16,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Surf {
    Sphere { radius: f32, material: Option<i16> },
    Ellipsoid { size: Vec3, material: Option<i16> },
    Box { bounds: [f32; 6], material: Option<i16> },
    Mesh { version: i32, vertices: Vec<Vec3>, triangles: Vec<MeshTri> },
    VCylinder { radius: f32, height: f32, material: i16 },
    MultiSphere { spheres: Vec<(f32, Vec3)>, material: i16 },
    ConvexPolyhedron { version: i32, representation: i32, raw: Vec<u8>, material: i16 },
    Capsule { center: Vec3, dir: Vec3, length: f32, material: i16 },
    Circle { center: Vec3, normal: Vec3, radius: f32 },
    Compound { surfs: Vec<SurfNode>, locs: Vec<Iso4>, joints: Vec<i16> },
    SphereLocated { center: Vec3, radius: f32, material: i16 },
    CompoundInstance,
    Cylinder { radius_y: f32, radius_xz: f32, material: i16 },
    SphericalShell { inner: f32, outer: f32, skip_in_to_out: bool, material: i16 },
    Voxel,
    Diggable { bounds: [f32; 6], u: [u32; 5] },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfNode {
    pub kind: i32,
    pub surf: Surf,
    pub gameplay_main_dir: Option<Vec3>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SurfMaterial {
    Material(u32),
    SurfaceId(i16),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Surface {
    pub version: u32,
    pub surf_version: u32,
    pub surf: Option<SurfNode>,
    pub materials: Vec<SurfMaterial>,
    pub material_ids: Vec<u16>,
}

fn vec3(r: &mut Reader) -> Result<Vec3> {
    r.vec3()
}

fn f6(r: &mut Reader) -> Result<[f32; 6]> {
    let mut b = [0f32; 6];
    for x in &mut b {
        *x = r.f32()?;
    }
    Ok(b)
}

fn count(r: &mut Reader, max: usize) -> Result<usize> {
    let n = r.u32()? as usize;
    ensure!(n <= max, "count {n} too large at {}", r.pos() - 4);
    Ok(n)
}

fn i16_(r: &mut Reader) -> Result<i16> {
    r.i16()
}

pub fn read_surf(r: &mut Reader, surf_version: u32) -> Result<Option<SurfNode>> {
    let kind = r.i32()?;
    if kind == -1 {
        return Ok(None);
    }
    let v1 = surf_version >= 1;
    let opt_mat = |r: &mut Reader| -> Result<Option<i16>> { if v1 { Ok(Some(r.i16()?)) } else { Ok(None) } };
    let surf = match kind {
        0 => Surf::Sphere { radius: r.f32()?, material: opt_mat(r)? },
        1 => Surf::Ellipsoid { size: vec3(r)?, material: opt_mat(r)? },
        6 => Surf::Box { bounds: f6(r)?, material: opt_mat(r)? },
        7 => {
            let version = r.i32()?;
            ensure!(version == 6 || version == 7, "mesh version {version} not supported");
            let nv = count(r, 1 << 22)?;
            let mut vertices = Vec::with_capacity(nv);
            for _ in 0..nv {
                vertices.push(vec3(r)?);
            }
            let nt = count(r, 1 << 22)?;
            let mut triangles = Vec::with_capacity(nt);
            for _ in 0..nt {
                let v = [r.i32()?, r.i32()?, r.i32()?];
                let flags = [r.u8()?, r.u8()?];
                triangles.push(MeshTri { v, flags, material: i16_(r)? });
            }
            Surf::Mesh { version, vertices, triangles }
        }
        8 => Surf::VCylinder { radius: r.f32()?, height: r.f32()?, material: i16_(r)? },
        9 => {
            let n = count(r, 10)?;
            let mut spheres = Vec::with_capacity(n);
            for _ in 0..n {
                spheres.push((r.f32()?, vec3(r)?));
            }
            Surf::MultiSphere { spheres, material: i16_(r)? }
        }
        10 => {
            let start = r.pos();
            let version = r.i32()?;
            let representation = r.i32()?;
            if representation != 0 {
                vec3(r)?;
            } else {
                f6(r)?;
                let n = count(r, 1 << 20)?;
                r.skip(12 * n)?;
                let n = count(r, 1 << 20)?;
                r.skip(4 * n)?;
                let n = count(r, 1 << 20)?;
                r.skip(8 * n)?;
            }
            let end = r.pos();
            let material = i16_(r)?;
            let raw = r_slice(r, start, end);
            Surf::ConvexPolyhedron { version, representation, raw, material }
        }
        11 => Surf::Capsule { center: vec3(r)?, dir: vec3(r)?, length: r.f32()?, material: i16_(r)? },
        12 => Surf::Circle { center: vec3(r)?, normal: vec3(r)?, radius: r.f32()? },
        13 => {
            let n = count(r, 1 << 16)?;
            let mut surfs = Vec::with_capacity(n);
            for _ in 0..n {
                surfs
                    .push(read_surf(r, surf_version)?.ok_or_else(|| anyhow::anyhow!("null compound child"))?);
            }
            let mut locs = Vec::with_capacity(n);
            for _ in 0..n {
                let mut l = [0f32; 12];
                for x in &mut l {
                    *x = r.f32()?;
                }
                locs.push(l);
            }
            let mut joints = Vec::new();
            if surf_version >= 1 {
                let k = count(r, 1 << 16)?;
                for _ in 0..k {
                    joints.push(r.i16()?);
                }
            }
            Surf::Compound { surfs, locs, joints }
        }
        14 => Surf::SphereLocated { center: vec3(r)?, radius: r.f32()?, material: i16_(r)? },
        15 => Surf::CompoundInstance,
        16 => Surf::Cylinder { radius_y: r.f32()?, radius_xz: r.f32()?, material: i16_(r)? },
        17 => Surf::SphericalShell {
            inner: r.f32()?,
            outer: r.f32()?,
            skip_in_to_out: r.bool32()?,
            material: i16_(r)?,
        },
        18 => Surf::Voxel,
        19 => {
            let bounds = f6(r)?;
            Surf::Diggable { bounds, u: [r.u32()?, r.u32()?, r.u32()?, r.u32()?, r.u32()?] }
        }
        k => bail!("unknown surf kind {k}"),
    };
    let gameplay_main_dir = if surf_version >= 2 { Some(vec3(r)?) } else { None };
    Ok(Some(SurfNode { kind, surf, gameplay_main_dir }))
}

fn skip_id(r: &mut Reader) -> Result<()> {
    let mut v = r.u32()?;
    if v == 3 {
        v = r.u32()?;
    }
    if v != u32::MAX && v & 0xc000_0000 != 0 && v & 0x3fff_ffff == 0 {
        r.string()?;
    }
    Ok(())
}

fn r_slice(r: &Reader, start: usize, end: usize) -> Vec<u8> {
    r.buf()[start..end].to_vec()
}

pub fn read_node(
    buf: &[u8],
    pos: usize,
    refs: &mut dyn FnMut(&mut Reader, u32) -> Result<()>,
) -> Result<(Surface, usize)> {
    let mut r = Reader::new(buf);
    r.skip(pos)?;
    let mut out = None;
    loop {
        let id = r.u32()?;
        match id {
            0xfacade01 => break,
            0x0900c003 => {
                let version = r.u32()?;
                ensure!(version <= 5, "CPlugSurface chunk version {version}");
                let surf_version = if version >= 2 { r.u32()? } else { 0 };
                let surf = read_surf(&mut r, if version == 1 { 1 } else { surf_version })?;
                let n = count(&mut r, 1 << 16)?;
                let mut materials = Vec::with_capacity(n);
                for _ in 0..n {
                    if r.bool32()? {
                        let idx = r.u32()?;
                        refs(&mut r, idx)?;
                        materials.push(SurfMaterial::Material(idx));
                    } else {
                        materials.push(SurfMaterial::SurfaceId(r.i16()?));
                    }
                }
                let mut material_ids = Vec::new();
                if version < 3 {
                    r.data()?;
                } else {
                    let primary = version >= 4 || n == 0;
                    if primary {
                        let k = count(&mut r, 1 << 16)?;
                        for _ in 0..k {
                            material_ids.push(r.u16()?);
                        }
                    }
                    if !primary || material_ids.is_empty() {
                        let k = count(&mut r, 1 << 16)?;
                        for _ in 0..k {
                            material_ids.push(r.u16()?);
                        }
                    }
                }
                if version >= 1 {
                    let idx = r.u32()?;
                    refs(&mut r, idx)?;
                }
                if version >= 5 {
                    let k = count(&mut r, 1 << 12)?;
                    for _ in 0..k {
                        skip_id(&mut r)?;
                    }
                }
                out = Some(Surface { version, surf_version, surf, materials, material_ids });
            }
            other => bail!("CPlugSurface: unexpected chunk {other:#010x} at {}", r.pos() - 4),
        }
    }
    let end = r.pos();
    Ok((out.ok_or_else(|| anyhow::anyhow!("CPlugSurface without chunk 0x0900C003"))?, end))
}

#[derive(Default)]
pub struct Writer(pub Vec<u8>);

impl Writer {
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn i16(&mut self, v: i16) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn fs(&mut self, v: &[f32]) {
        for &x in v {
            self.f32(x);
        }
    }
}

pub fn write_surf(w: &mut Writer, n: Option<&SurfNode>, surf_version: u32) {
    let Some(n) = n else {
        w.i32(-1);
        return;
    };
    w.i32(n.kind);
    let v1 = surf_version >= 1;
    let opt = |w: &mut Writer, m: &Option<i16>| {
        if v1 {
            w.i16(m.unwrap_or(0));
        }
    };
    match &n.surf {
        Surf::Sphere { radius, material } => {
            w.f32(*radius);
            opt(w, material);
        }
        Surf::Ellipsoid { size, material } => {
            w.fs(size);
            opt(w, material);
        }
        Surf::Box { bounds, material } => {
            w.fs(bounds);
            opt(w, material);
        }
        Surf::Mesh { version, vertices, triangles } => {
            w.i32(*version);
            w.u32(vertices.len() as u32);
            for v in vertices {
                w.fs(v);
            }
            w.u32(triangles.len() as u32);
            for t in triangles {
                for i in t.v {
                    w.i32(i);
                }
                w.u8(t.flags[0]);
                w.u8(t.flags[1]);
                w.i16(t.material);
            }
        }
        Surf::VCylinder { radius, height, material } => {
            w.fs(&[*radius, *height]);
            w.i16(*material);
        }
        Surf::MultiSphere { spheres, material } => {
            w.u32(spheres.len() as u32);
            for (r, c) in spheres {
                w.f32(*r);
                w.fs(c);
            }
            w.i16(*material);
        }
        Surf::ConvexPolyhedron { raw, material, .. } => {
            w.0.extend_from_slice(raw);
            w.i16(*material);
        }
        Surf::Capsule { center, dir, length, material } => {
            w.fs(center);
            w.fs(dir);
            w.f32(*length);
            w.i16(*material);
        }
        Surf::Circle { center, normal, radius } => {
            w.fs(center);
            w.fs(normal);
            w.f32(*radius);
        }
        Surf::Compound { surfs, locs, joints } => {
            w.u32(surfs.len() as u32);
            for c in surfs {
                write_surf(w, Some(c), surf_version);
            }
            for l in locs {
                w.fs(l);
            }
            if surf_version >= 1 {
                w.u32(joints.len() as u32);
                for &j in joints {
                    w.i16(j);
                }
            }
        }
        Surf::SphereLocated { center, radius, material } => {
            w.fs(center);
            w.f32(*radius);
            w.i16(*material);
        }
        Surf::CompoundInstance | Surf::Voxel => {}
        Surf::Cylinder { radius_y, radius_xz, material } => {
            w.fs(&[*radius_y, *radius_xz]);
            w.i16(*material);
        }
        Surf::SphericalShell { inner, outer, skip_in_to_out, material } => {
            w.fs(&[*inner, *outer]);
            w.u32(*skip_in_to_out as u32);
            w.i16(*material);
        }
        Surf::Diggable { bounds, u } => {
            w.fs(bounds);
            for &x in u {
                w.u32(x);
            }
        }
    }
    if surf_version >= 2 {
        w.fs(&n.gameplay_main_dir.unwrap_or([0.0; 3]));
    }
}

pub fn encode(s: &Surface) -> Vec<u8> {
    let mut w = Writer::default();
    w.u32(s.version);
    w.u32(s.surf_version);
    write_surf(&mut w, s.surf.as_ref(), if s.version == 1 { 1 } else { s.surf_version });
    w.u32(s.materials.len() as u32);
    for m in &s.materials {
        match m {
            SurfMaterial::Material(i) => {
                w.u32(1);
                w.u32(*i);
            }
            SurfMaterial::SurfaceId(id) => {
                w.u32(0);
                w.i16(*id);
            }
        }
    }
    w.u32(s.material_ids.len() as u32);
    for &m in &s.material_ids {
        w.u16(m);
    }
    w.0
}

pub fn decode(b: &[u8]) -> Result<Surface> {
    let mut r = Reader::new(b);
    let version = r.u32()?;
    let surf_version = r.u32()?;
    let surf = read_surf(&mut r, if version == 1 { 1 } else { surf_version })?;
    let n = count(&mut r, 1 << 16)?;
    let mut materials = Vec::with_capacity(n);
    for _ in 0..n {
        materials.push(if r.u32()? != 0 {
            SurfMaterial::Material(r.u32()?)
        } else {
            SurfMaterial::SurfaceId(r.i16()?)
        });
    }
    let k = count(&mut r, 1 << 16)?;
    let mut material_ids = Vec::with_capacity(k);
    for _ in 0..k {
        material_ids.push(r.u16()?);
    }
    ensure!(r.remaining() == 0, "trailing bytes in surface");
    Ok(Surface { version, surf_version, surf, materials, material_ids })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoding_reads_back() {
        let mesh = SurfNode {
            kind: 7,
            surf: Surf::Mesh {
                version: 7,
                vertices: vec![[0.0, 1.0, 2.0], [3.5, -4.0, 5.25], [6.0, 7.0, 8.0]],
                triangles: vec![MeshTri { v: [0, 1, 2], flags: [16, 0], material: 1 }],
            },
            gameplay_main_dir: Some([0.0, 0.0, 1.0]),
        };
        let sphere = SurfNode {
            kind: 0,
            surf: Surf::Sphere { radius: 2.0, material: Some(3) },
            gameplay_main_dir: Some([1.0, 0.0, 0.0]),
        };
        let s = Surface {
            version: 4,
            surf_version: 2,
            surf: Some(SurfNode {
                kind: 13,
                surf: Surf::Compound {
                    surfs: vec![mesh, sphere],
                    locs: vec![[1.0; 12], [2.0; 12]],
                    joints: vec![0, 1],
                },
                gameplay_main_dir: Some([0.0, 1.0, 0.0]),
            }),
            materials: vec![SurfMaterial::Material(7), SurfMaterial::SurfaceId(16)],
            material_ids: vec![16, 6],
        };
        assert_eq!(decode(&encode(&s)).unwrap(), s);
    }
}
