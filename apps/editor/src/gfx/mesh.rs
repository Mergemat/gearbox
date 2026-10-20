use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3};

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable, PartialEq)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub normal: [i8; 4],
    pub color: [u8; 4],
    pub uv: [f32; 2],
}

pub const FLAT: [i8; 4] = [0, 0, 0, 0];
pub const UNLIT: [i8; 4] = [0, 0, 0, -127];

impl Vertex {
    pub fn flat(pos: Vec3, color: [u8; 4]) -> Self {
        Self { pos: pos.into(), normal: FLAT, color, uv: [0.0; 2] }
    }
    pub fn unlit(pos: Vec3, color: [u8; 4]) -> Self {
        Self { pos: pos.into(), normal: UNLIT, color, uv: [0.0; 2] }
    }
    pub fn smooth(pos: Vec3, n: Vec3, color: [u8; 4], uv: [f32; 2]) -> Self {
        let n = n.normalize_or_zero() * 127.0;
        Self { pos: pos.into(), normal: [n.x as i8, n.y as i8, n.z as i8, 127], color, uv }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct LineVertex {
    pub pos: [f32; 3],
    pub color: [u8; 4],
}

#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl Mesh {
    pub fn unit_box() -> Self {
        let mut m = Mesh::default();
        for (k, s) in [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0), (2, 1.0), (2, -1.0)] {
            let mut n = Vec3::ZERO;
            n[k] = s;
            let (mut u, mut v) = (Vec3::ZERO, Vec3::ZERO);
            u[(k + 1) % 3] = 1.0;
            v[(k + 2) % 3] = 1.0;
            let base = m.vertices.len() as u32;
            for (a, b) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                m.vertices.push(Vertex::smooth(
                    n + u * a + v * b,
                    n,
                    [255; 4],
                    [0.5 + a * 0.5, 0.5 + b * 0.5],
                ));
            }
            m.indices.extend([0, 1, 2, 0, 2, 3].map(|i| base + i));
        }
        m
    }

    pub fn octahedron() -> Self {
        let p = [Vec3::X, Vec3::NEG_X, Vec3::Y, Vec3::NEG_Y, Vec3::Z, Vec3::NEG_Z];
        let faces = [[2, 0, 4], [2, 4, 1], [2, 1, 5], [2, 5, 0], [3, 4, 0], [3, 1, 4], [3, 5, 1], [3, 0, 5]];
        let mut m = Mesh::default();
        for f in faces {
            let base = m.vertices.len() as u32;
            for i in f {
                m.vertices.push(Vertex::flat(p[i], [255; 4]));
            }
            m.indices.extend([base, base + 1, base + 2]);
        }
        m
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MeshId(pub u32);
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct TextureId(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Pass {
    Auto,
    Opaque,
    Cutout,
    Decal,
    Glass,
    Holo,
    Glow,
}

#[derive(Clone, Copy, Debug)]
pub struct Instance {
    pub mesh: MeshId,
    pub texture: TextureId,
    pub transform: Mat4,
    pub tint: [f32; 4],
    pub pass: Pass,
}

impl Instance {
    pub fn pass(&self) -> Pass {
        match self.pass {
            Pass::Auto if self.tint[3] < 1.0 => Pass::Glass,
            Pass::Auto => Pass::Opaque,
            p => p,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LineSetId(pub u32);

#[derive(Default)]
pub struct DrawList {
    pub instances: Vec<Instance>,
    pub statics: Vec<u32>,
    pub line_sets: Vec<LineSetId>,
    pub tris: Vec<Vertex>,
    pub glass: Vec<Vertex>,
    pub overlay: Vec<Vertex>,
    pub lines: Vec<LineVertex>,
    pub overlay_lines: Vec<LineVertex>,
}

impl DrawList {
    pub fn line(&mut self, a: Vec3, b: Vec3, color: [u8; 4]) {
        self.lines.push(LineVertex { pos: a.into(), color });
        self.lines.push(LineVertex { pos: b.into(), color });
    }
    pub fn overlay_line(&mut self, a: Vec3, b: Vec3, color: [u8; 4]) {
        self.overlay_lines.push(LineVertex { pos: a.into(), color });
        self.overlay_lines.push(LineVertex { pos: b.into(), color });
    }
}
