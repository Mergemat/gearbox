use std::collections::HashMap;

use crate::engine::display::World;
use crate::engine::frame::{Frame, Wheel};
use crate::replay::GhostTrack;
use glam::{Mat4, Quat, Vec3};

use crate::background::GateInfo;
use crate::doc::{Goal, GoalKind, StartState};
use crate::gfx::mesh::{DrawList, Instance, LineSetId, LineVertex, Mesh, MeshId, Pass, TextureId, Vertex};
use crate::gfx::renderer::Renderer;
use crate::gizmo::{Gizmo, Handle};
use crate::theme;
use crate::visual::MatKind;

pub const CAR_HALF: Vec3 = Vec3::new(0.95, 0.42, 2.15);
const WHEELS: [[f32; 3]; 4] =
    [[0.86, -0.35, 1.38], [-0.86, -0.35, 1.38], [-0.86, -0.35, -1.42], [0.86, -0.35, -1.42]];

pub fn build_world(w: &World, hidden: &[crate::materials::Family]) -> Mesh {
    let mut mesh = Mesh::default();
    mesh.indices.reserve(w.tris.len() * 3);
    let mut seen: HashMap<([u32; 3], [u8; 3]), u32> = HashMap::with_capacity(w.tris.len());
    for t in &w.tris {
        if t.gameplay == 0 && hidden.contains(&crate::materials::family(t.surface)) {
            continue;
        }
        let c = theme::surface_rgb(t.surface, t.gameplay);
        for p in t.v {
            let key = (p.map(f32::to_bits), c);
            let i = *seen.entry(key).or_insert_with(|| {
                mesh.vertices.push(Vertex::flat(Vec3::from(p), [c[0], c[1], c[2], 255]));
                mesh.vertices.len() as u32 - 1
            });
            mesh.indices.push(i);
        }
    }
    mesh
}

fn trail_mesh(frames: &[Frame], half_width: f32) -> Mesh {
    let mut m = Mesh::default();
    let lift = Vec3::Y * 0.08;
    for k in 1..frames.len() {
        let (a, b) = (Vec3::from(frames[k - 1].position()) + lift, Vec3::from(frames[k].position()) + lift);
        let side = (b - a).cross(Vec3::Y).normalize_or_zero() * half_width;
        if side == Vec3::ZERO {
            continue;
        }
        let [r, g, bl] = theme::speed_rgb(frames[k].speed_kmh());
        let c = [r, g, bl, 255];
        let base = m.vertices.len() as u32;
        for p in [a - side, a + side, b + side, b - side] {
            m.vertices.push(Vertex::unlit(p, c));
        }
        m.indices.extend([0, 1, 2, 0, 2, 3].map(|i| base + i));
    }
    m
}

fn box_transform(c: Vec3, ax: [Vec3; 3], half: Vec3) -> Mat4 {
    Mat4::from_cols(
        (ax[0] * half.x).extend(0.0),
        (ax[1] * half.y).extend(0.0),
        (ax[2] * half.z).extend(0.0),
        c.extend(1.0),
    )
}

fn tint(rgb: [u8; 3], a: f32) -> [f32; 4] {
    [rgb[0] as f32 / 255.0, rgb[1] as f32 / 255.0, rgb[2] as f32 / 255.0, a]
}

fn rgba(hex: u32, a: u8) -> [u8; 4] {
    theme::rgba8(hex, a)
}

pub fn lerp_frame(a: &Frame, b: &Frame, t: f32) -> Frame {
    use glam::Mat3;
    let mut f = *a;
    let rot = |f: &Frame| {
        Quat::from_mat3(&Mat3::from_cols(Vec3::from(f.axis(0)), Vec3::from(f.axis(1)), Vec3::from(f.axis(2))))
            .normalize()
    };
    let m = Mat3::from_quat(rot(a).slerp(rot(b), t));
    for (k, c) in [m.x_axis, m.y_axis, m.z_axis].iter().enumerate() {
        f.body[k] = c.x;
        f.body[3 + k] = c.y;
        f.body[6 + k] = c.z;
    }
    for k in 9..22 {
        if !(12..16).contains(&k) {
            f.body[k] = a.body[k] + (b.body[k] - a.body[k]) * t;
        }
    }
    let ang = |x: f32, y: f32| {
        let d = (y - x + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
        x + d * t
    };
    for (w, (wa, wb)) in f.wheels.iter_mut().zip(a.wheels.iter().zip(&b.wheels)) {
        w.position = std::array::from_fn(|k| wa.position[k] + (wb.position[k] - wa.position[k]) * t);
        w.steer = ang(wa.steer, wb.steer);
        w.spin = ang(wa.spin, wb.spin);
    }
    f.race_ms = a.race_ms + (10.0 * t).round() as i32;
    f
}

pub struct Shared<T>(pub std::sync::Arc<T>);

impl<T> PartialEq for Shared<T> {
    fn eq(&self, o: &Self) -> bool {
        std::sync::Arc::ptr_eq(&self.0, &o.0)
    }
}

struct LineCache<K> {
    key: Option<K>,
    set: Option<LineSetId>,
}

impl<K> Default for LineCache<K> {
    fn default() -> Self {
        Self { key: None, set: None }
    }
}

impl<K: PartialEq> LineCache<K> {
    fn fresh(&self, key: &K) -> bool {
        self.key.as_ref() == Some(key)
    }

    fn store(&mut self, r: &mut Renderer, key: K, v: &[LineVertex]) {
        if let Some(s) = self.set.take() {
            r.remove_lines(s);
        }
        self.set = r.add_lines(v);
        self.key = Some(key);
    }
}

type Tries = Vec<(Vec<[f32; 3]>, crate::optimize::Mark)>;

pub struct Scene {
    pub renderer: Renderer,
    early_trail: LineCache<(u64, usize)>,
    ghost_set: LineCache<Vec<(usize, usize)>>,
    ref_set: LineCache<Vec<(Shared<Vec<Frame>>, u8)>>,
    tries_set: LineCache<Shared<Tries>>,
    world: Option<MeshId>,
    pub bounds: Option<(Vec3, Vec3)>,
    unit_box: MeshId,
    octa: MeshId,
    quad: MeshId,
    blob: TextureId,
    trail: Option<MeshId>,
    trail_key: (u64, u32, usize),
    ghost_lines: Vec<(usize, Vec<crate::gfx::mesh::LineVertex>)>,
    pub visual: Option<VisualGpu>,
}

struct VisualPart {
    pass: Pass,
    mesh: MeshId,
    texture: TextureId,
    tint: [f32; 4],
    transform: Mat4,
    center: Vec3,
    radius: f32,
    surroundings: bool,
}

impl VisualPart {
    fn instance(&self) -> Instance {
        Instance {
            mesh: self.mesh,
            texture: self.texture,
            transform: self.transform,
            tint: self.tint,
            pass: self.pass,
        }
    }
}

pub struct VisualGpu {
    parts: Vec<VisualPart>,
    car_body: Vec<(MeshId, TextureId, [f32; 4], Mat4)>,
    wheels: Vec<(Vec3, Vec<(MeshId, TextureId, [f32; 4], Mat4)>)>,
}

fn gameplay_tint(g: Option<u8>) -> [f32; 3] {
    match g {
        None | Some(0) => [1.0, 1.0, 1.0],
        Some(1..=3 | 10 | 11 | 16..=19) => [1.0, 0.84, 0.66],
        Some(8 | 20..=23) => [0.80, 0.96, 0.95],
        _ => [0.88, 0.84, 1.0],
    }
}

fn part_sphere(m: &crate::visual::MeshData, first: u32, count: u32) -> (Vec3, f32) {
    let idx = &m.indices[first as usize..(first + count) as usize];
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for &i in idx {
        let p = Vec3::from(m.vertices[i as usize].pos);
        lo = lo.min(p);
        hi = hi.max(p);
    }
    if lo.x > hi.x {
        return (Vec3::ZERO, 0.0);
    }
    ((lo + hi) * 0.5, (hi - lo).length() * 0.5)
}

impl VisualGpu {
    pub fn upload(r: &mut Renderer, v: &crate::visual::VisualWorld) -> Self {
        let textures: Vec<Option<TextureId>> = v
            .textures
            .iter()
            .map(|t| {
                t.as_ref().map(|t| {
                    let mips: Vec<&[u8]> = t.mips.iter().map(Vec::as_slice).collect();
                    r.add_texture_mips(t.format, t.width, t.height, &mips)
                })
            })
            .collect();
        let mut ids: Vec<Vec<MeshId>> = Vec::with_capacity(v.meshes.len());
        let mut spheres: Vec<Vec<(Vec3, f32)>> = Vec::with_capacity(v.meshes.len());
        for m in &v.meshes {
            let parts: Vec<(u32, u32)> = m.parts.iter().map(|p| (p.0, p.1)).collect();
            spheres.push(parts.iter().map(|&(f, c)| part_sphere(m, f, c)).collect());
            ids.push(if m.indices.is_empty() {
                Vec::new()
            } else {
                r.add_mesh_parts(&m.vertices, &m.indices, &parts)
            });
        }
        let look = |p: &crate::visual::Placed| -> Option<(MeshId, TextureId, [f32; 4])> {
            let mesh = *ids.get(p.mesh)?.get(p.part)?;
            let mat = v.materials.get(p.material);
            if mat.is_some_and(|m| m.kind == MatKind::Skip) {
                return None;
            }
            let tex = mat
                .and_then(|m| m.texture)
                .and_then(|t| textures.get(t).copied().flatten())
                .unwrap_or(TextureId(0));
            let base = mat.map_or([1.0; 4], |m| m.base);
            let kind = mat.map(|m| m.kind);
            let g = if matches!(kind, Some(MatKind::Holo | MatKind::Glow)) {
                [1.0; 3]
            } else {
                gameplay_tint(mat.and_then(|m| m.gameplay))
            };
            let mut tint = [
                base[0] * g[0],
                base[1] * g[1],
                base[2] * g[2],
                if matches!(kind, Some(MatKind::Glass | MatKind::Holo | MatKind::Glow)) {
                    base[3]
                } else {
                    1.0
                },
            ];
            if p.surroundings {
                for c in &mut tint[..3] {
                    *c *= 0.55;
                }
            }
            Some((mesh, tex, tint))
        };
        let mut parts = Vec::with_capacity(v.placed.len());
        for p in &v.placed {
            let Some((mesh, texture, tint)) = look(p) else { continue };
            let (c, rad) = spheres[p.mesh][p.part];
            let scale = p
                .transform
                .x_axis
                .truncate()
                .length()
                .max(p.transform.y_axis.truncate().length())
                .max(p.transform.z_axis.truncate().length());
            let pass = match v.materials.get(p.material).map(|m| m.kind) {
                Some(MatKind::Cutout) => Pass::Cutout,
                Some(MatKind::Decal) => Pass::Decal,
                Some(MatKind::Glass) => Pass::Glass,
                Some(MatKind::Holo) => Pass::Holo,
                Some(MatKind::Glow) => Pass::Glow,
                _ => Pass::Opaque,
            };
            parts.push(VisualPart {
                pass,
                mesh,
                texture,
                tint,
                transform: p.transform,
                center: p.transform.transform_point3(c),
                radius: rad * scale,
                surroundings: p.surroundings,
            });
        }
        let key = |p: &VisualPart| (p.instance().pass(), p.mesh, p.texture);
        parts.sort_by_key(key);
        r.set_statics(&parts.iter().map(VisualPart::instance).collect::<Vec<_>>());
        let (car_body, wheels) = match &v.car {
            Some(c) => (
                c.body.iter().filter_map(|p| look(p).map(|(m, t, tint)| (m, t, tint, p.transform))).collect(),
                c.wheels
                    .iter()
                    .map(|w| {
                        (
                            w.pivot.w_axis.truncate(),
                            w.parts
                                .iter()
                                .filter_map(|p| look(p).map(|(m, t, tint)| (m, t, tint, p.transform)))
                                .collect(),
                        )
                    })
                    .collect(),
            ),
            None => (Vec::new(), Vec::new()),
        };
        Self { parts, car_body, wheels }
    }
}

fn frustum(vp: &Mat4) -> [glam::Vec4; 5] {
    let r = [vp.row(0), vp.row(1), vp.row(2), vp.row(3)];
    let mut p = [r[3] + r[0], r[3] - r[0], r[3] + r[1], r[3] - r[1], r[3] - r[2]];
    for q in &mut p {
        *q /= q.truncate().length();
    }
    p
}

pub struct RefRun<'a> {
    pub frames: &'a std::sync::Arc<Vec<Frame>>,
    pub alpha: u8,
    pub car: bool,
}

pub struct SceneInput<'a> {
    pub frames: &'a [Frame],
    pub frames_gen: u64,
    pub goals: &'a [Goal],
    pub guides: &'a [crate::doc::Guide],
    pub selected_guide: Option<u32>,
    pub reached: &'a [bool],
    pub selected_goal: Option<u32>,
    pub start: Option<&'a StartState>,
    pub start_selected: bool,
    pub car_preview: Option<&'a StartState>,
    pub gates: &'a [GateInfo],
    pub show_gates: bool,
    pub hover_gate: Option<u32>,
    pub selected_gate: Option<u32>,
    pub gizmo: Option<(&'a Gizmo, Option<Handle>)>,
    pub ghosts: Vec<&'a GhostTrack>,
    pub refs: Vec<RefRun<'a>>,
    pub tries: Option<&'a std::sync::Arc<Tries>>,
    pub branches: Option<&'a [crate::engine::session::Branch]>,
    pub pivot: Option<Vec3>,
    pub hover: Option<Vec3>,
    pub trail_width: f32,
    pub show_trail: bool,
    pub trail_from: usize,
    pub physics: bool,
    pub surroundings: bool,
    pub view_proj: Mat4,
    pub car: Option<Frame>,
    pub contacts: Option<[(Vec3, Option<Vec3>); 4]>,
    pub start_handle: Option<bool>,
    pub tricks: &'a [crate::lab::TrickSpan],
    pub setups: Vec<(&'a crate::doc::Setup, bool)>,
    pub stretch: Option<(usize, usize)>,
}

pub const VEL_ARROW_S: f32 = 0.15;

pub fn velocity_tip(s: &StartState) -> Vec3 {
    let p = Vec3::from(s.pos) + Vec3::Y * 0.6;
    let v = Vec3::from(s.velocity);
    if v.length() * VEL_ARROW_S > 3.0 {
        p + v * VEL_ARROW_S
    } else {
        p + crate::doc::ypr_rot(s.ypr) * Vec3::Z * 3.0
    }
}

fn box_lines(d: &mut DrawList, min: Vec3, max: Vec3, c: [u8; 4], overlay: bool) {
    let p = |i: usize| {
        Vec3::new(
            if i & 1 == 0 { min.x } else { max.x },
            if i & 2 == 0 { min.y } else { max.y },
            if i & 4 == 0 { min.z } else { max.z },
        )
    };
    for (a, b) in
        [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
    {
        if overlay {
            d.overlay_line(p(a), p(b), c);
        } else {
            d.line(p(a), p(b), c);
        }
    }
}

fn oriented_box_lines(d: &mut DrawList, c: Vec3, h: Vec3, q: Quat, col: [u8; 4]) {
    let p = |i: usize| {
        c + q * Vec3::new(
            if i & 1 == 0 { -h.x } else { h.x },
            if i & 2 == 0 { -h.y } else { h.y },
            if i & 4 == 0 { -h.z } else { h.z },
        )
    };
    for (a, b) in
        [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
    {
        d.line(p(a), p(b), col);
    }
}

fn circle(d: &mut DrawList, c: Vec3, n: Vec3, r: f32, col: [u8; 4]) {
    let u = n.any_orthonormal_vector();
    let v = n.cross(u);
    let pts: Vec<Vec3> = (0..=48)
        .map(|i| {
            let a = i as f32 / 48.0 * std::f32::consts::TAU;
            c + (u * a.cos() + v * a.sin()) * r
        })
        .collect();
    for w in pts.windows(2) {
        d.line(w[0], w[1], col);
    }
}

impl Scene {
    pub fn new(mut renderer: Renderer) -> Self {
        let unit_box = renderer.add_mesh(&Mesh::unit_box());
        let octa = renderer.add_mesh(&Mesh::octahedron());
        let quad = renderer.add_mesh(&Mesh {
            vertices: [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
                .map(|(x, z)| Vertex::unlit(Vec3::new(x, 0.0, z), [255; 4]))
                .into_iter()
                .zip([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
                .map(|(mut v, uv)| {
                    v.uv = uv;
                    v
                })
                .collect(),
            indices: vec![0, 1, 2, 0, 2, 3],
        });
        let n = 64u32;
        let blob_px: Vec<u8> = (0..n * n)
            .flat_map(|i| {
                let (x, y) = (
                    (i % n) as f32 / (n - 1) as f32 * 2.0 - 1.0,
                    (i / n) as f32 / (n - 1) as f32 * 2.0 - 1.0,
                );
                let r = (x * x + y * y).sqrt();
                let a = ((1.0 - r) / 0.6).clamp(0.0, 1.0);
                [0, 0, 0, (a * a * (3.0 - 2.0 * a) * 255.0) as u8]
            })
            .collect();
        let blob = renderer.add_texture(&blob_px, n, n);
        Self {
            renderer,
            world: None,
            bounds: None,
            unit_box,
            octa,
            quad,
            blob,
            trail: None,
            trail_key: (u64::MAX, 0, 0),
            early_trail: LineCache::default(),
            ghost_set: LineCache::default(),
            ref_set: LineCache::default(),
            tries_set: LineCache::default(),
            ghost_lines: Vec::new(),
            visual: None,
        }
    }

    pub fn set_world(&mut self, mesh: &Mesh, min: Vec3, max: Vec3) {
        if let Some(w) = self.world.take() {
            self.renderer.remove_mesh(w);
        }
        self.world = (!mesh.indices.is_empty()).then(|| self.renderer.add_mesh(mesh));
        self.bounds = Some((min, max));
    }

    fn ghost_path(&mut self, g: &GhostTrack) -> &[crate::gfx::mesh::LineVertex] {
        let key = g as *const GhostTrack as usize;
        if !self.ghost_lines.iter().any(|(k, _)| *k == key) {
            let c = theme::rgba8(theme::GHOST, 220);
            let mut v = Vec::new();
            for s in g.samples.windows(2) {
                for p in [s[0].position, s[1].position] {
                    v.push(crate::gfx::mesh::LineVertex {
                        pos: (Vec3::from(p) + Vec3::Y * 0.25).into(),
                        color: c,
                    });
                }
            }
            self.ghost_lines.push((key, v));
            if self.ghost_lines.len() > 16 {
                self.ghost_lines.remove(0);
            }
        }
        &self.ghost_lines.iter().find(|(k, _)| *k == key).unwrap().1
    }

    fn car(
        &self,
        d: &mut DrawList,
        body: Mat4,
        wheels: Option<&[Wheel; 4]>,
        tint_rgba: Option<[f32; 4]>,
        realistic: bool,
        shadow: f32,
    ) {
        if realistic && let Some(v) = self.visual.as_ref().filter(|v| !v.car_body.is_empty()) {
            if shadow > 0.0 {
                let s = body
                    * Mat4::from_translation(Vec3::new(0.0, 0.015, 0.25))
                    * Mat4::from_scale(Vec3::new(1.35, 1.0, 2.6));
                d.instances.push(Instance {
                    mesh: self.quad,
                    texture: self.blob,
                    transform: s,
                    tint: [1.0, 1.0, 1.0, 0.55 * shadow],
                    pass: Pass::Decal,
                });
            }
            let mix = |t: [f32; 4]| match tint_rgba {
                Some(k) => {
                    [t[0] * 0.35 + k[0] * 0.65, t[1] * 0.35 + k[1] * 0.65, t[2] * 0.35 + k[2] * 0.65, k[3]]
                }
                None => t,
            };
            for &(mesh, texture, t, m) in &v.car_body {
                d.instances.push(Instance {
                    mesh,
                    texture,
                    transform: body * m,
                    tint: mix(t),
                    pass: Pass::Auto,
                });
            }
            for (i, (pivot, parts)) in v.wheels.iter().enumerate() {
                let w = wheels.and_then(|w| w.get(i).copied()).unwrap_or_default();
                let at = if w.position == [0.0; 3] { *pivot } else { Vec3::from(w.position) };
                let local = Mat4::from_translation(at)
                    * Mat4::from_rotation_y(w.steer)
                    * Mat4::from_rotation_x(w.spin)
                    * Mat4::from_translation(-*pivot);
                for &(mesh, texture, t, m) in parts {
                    d.instances.push(Instance {
                        mesh,
                        texture,
                        transform: body * local * m,
                        tint: mix(t),
                        pass: Pass::Auto,
                    });
                }
            }
            return;
        }
        let ax = [body.x_axis.truncate(), body.y_axis.truncate(), body.z_axis.truncate()];
        let c = body.w_axis.truncate();
        let base = tint_rgba.unwrap_or(tint(theme::rgb8(theme::TEXT), 1.0));
        d.instances.push(Instance {
            mesh: self.unit_box,
            texture: TextureId(0),
            transform: box_transform(c + ax[1] * 0.45, ax, CAR_HALF),
            tint: base,
            pass: Pass::Auto,
        });
        let dark = [0.18, 0.2, 0.24, base[3]];
        d.instances.push(Instance {
            mesh: self.unit_box,
            texture: TextureId(0),
            transform: box_transform(c + ax[1] * 0.95 - ax[2] * 0.3, ax, Vec3::new(0.7, 0.18, 0.9)),
            tint: dark,
            pass: Pass::Auto,
        });
        for (i, w) in WHEELS.iter().enumerate() {
            let wc = c + ax[0] * w[0] + ax[1] * (w[1] + 0.45) + ax[2] * w[2];
            let contact = wheels.is_some_and(|ws| ws[i].contact);
            let rgb = if contact || wheels.is_none() { [34, 37, 42] } else { [90, 60, 70] };
            d.instances.push(Instance {
                mesh: self.unit_box,
                texture: TextureId(0),
                transform: box_transform(wc, ax, Vec3::new(0.2, 0.36, 0.36)),
                tint: tint(rgb, base[3]),
                pass: Pass::Auto,
            });
        }
    }

    fn start_marker(&self, d: &mut DrawList, s: &StartState, selected: bool, preview: bool, realistic: bool) {
        let q = crate::doc::ypr_rot(s.ypr);
        let body = Mat4::from_rotation_translation(q, Vec3::from(s.pos));
        let col = if preview { theme::GHOST } else { theme::PIN };
        let rgb = theme::rgb8(col);
        let a = if preview {
            0.45
        } else if selected {
            0.7
        } else {
            0.55
        };
        self.car(d, body, None, Some(tint(rgb, a)), realistic, 0.0);
        let p = Vec3::from(s.pos) + Vec3::Y * 0.6;
        let v = Vec3::from(s.velocity);
        if v.length() > 0.05 {
            let e = p + v * VEL_ARROW_S;
            let c = rgba(col, 255);
            d.overlay_line(p, e, c);
            let dir = v.normalize();
            let side = dir.cross(Vec3::Y).normalize_or_zero() * 0.5;
            d.overlay_line(e, e - dir * 0.9 + side, c);
            d.overlay_line(e, e - dir * 0.9 - side, c);
        }
        let w = Vec3::from(s.angular);
        if w.length() > 0.5 {
            let n = w.normalize();
            let u = n.any_orthonormal_vector();
            let vv = n.cross(u);
            let sweep = (w.length().to_radians() * 0.5).min(5.5);
            let pts: Vec<Vec3> = (0..=24)
                .map(|i| {
                    let t = i as f32 / 24.0 * sweep;
                    p + (u * t.cos() + vv * t.sin()) * 1.6
                })
                .collect();
            for seg in pts.windows(2) {
                d.overlay_line(seg[0], seg[1], rgba(theme::WARN, 230));
            }
        }
        if selected {
            circle(d, Vec3::from(s.pos) + Vec3::Y * 0.05, Vec3::Y, 2.8, rgba(theme::PIN, 255));
        }
    }

    pub fn draw_list(&mut self, s: &SceneInput<'_>) -> DrawList {
        let mut d = DrawList::default();
        let realistic = !s.physics && self.visual.is_some();
        if realistic {
            let planes = frustum(&s.view_proj);
            let v = self.visual.as_ref().unwrap();
            for (i, p) in v.parts.iter().enumerate() {
                if (p.surroundings && !s.surroundings)
                    || planes.iter().any(|pl| pl.truncate().dot(p.center) + pl.w < -p.radius)
                {
                    continue;
                }
                d.statics.push(i as u32);
            }
        } else if let Some(w) = self.world {
            d.instances.push(Instance {
                mesh: w,
                texture: TextureId(0),
                transform: Mat4::IDENTITY,
                tint: [1.0; 4],
                pass: Pass::Auto,
            });
        }
        if let Some((min, max)) = self.bounds.filter(|_| !realistic) {
            let y = min.y - 0.05;
            let g = theme::rgba8(theme::HAIRLINE, 140);
            let (x0, x1) = ((min.x / 32.0).floor() * 32.0, (max.x / 32.0).ceil() * 32.0);
            let (z0, z1) = ((min.z / 32.0).floor() * 32.0, (max.z / 32.0).ceil() * 32.0);
            let mut x = x0;
            while x <= x1 {
                d.line(Vec3::new(x, y, z0), Vec3::new(x, y, z1), g);
                x += 32.0;
            }
            let mut z = z0;
            while z <= z1 {
                d.line(Vec3::new(x0, y, z), Vec3::new(x1, y, z), g);
                z += 32.0;
            }
        }
        if s.show_trail {
            let step = (s.trail_width * 8.0).round().clamp(2.0, 40.0) as u32;
            let from = s.trail_from.min(s.frames.len());
            if self.trail_key != (s.frames_gen, step, from) {
                if let Some(t) = self.trail.take() {
                    self.renderer.remove_mesh(t);
                }
                let m = trail_mesh(&s.frames[from..], step as f32 / 16.0);
                self.trail = (!m.indices.is_empty()).then(|| self.renderer.add_mesh(&m));
                self.trail_key = (s.frames_gen, step, from);
            }
            if let Some(t) = self.trail {
                d.instances.push(Instance {
                    mesh: t,
                    texture: TextureId(0),
                    transform: Mat4::IDENTITY,
                    tint: [1.0; 4],
                    pass: Pass::Auto,
                });
            }
            let key = (s.frames_gen, from);
            if !self.early_trail.fresh(&key) {
                let mut v = DrawList::default();
                for w in s.frames[..from.min(s.frames.len())].windows(2).step_by(2) {
                    v.line(
                        Vec3::from(w[0].position()) + Vec3::Y * 0.1,
                        Vec3::from(w[1].position()) + Vec3::Y * 0.1,
                        rgba(theme::MUTED, 90),
                    );
                }
                self.early_trail.store(&mut self.renderer, key, &v.lines);
            }
            d.line_sets.extend(self.early_trail.set);
        }
        let race_ms = s.car.as_ref().map(|f| f.race_ms);
        let key: Vec<(usize, usize)> =
            s.ghosts.iter().map(|g| (*g as *const GhostTrack as usize, g.samples.len())).collect();
        if !self.ghost_set.fresh(&key) {
            let mut v = Vec::new();
            for g in &s.ghosts {
                v.extend_from_slice(self.ghost_path(g));
            }
            self.ghost_set.store(&mut self.renderer, key, &v);
        }
        d.line_sets.extend(self.ghost_set.set);
        for g in &s.ghosts {
            if let Some(t) = race_ms
                && let Some((p, q, _)) = g.at(t)
            {
                let body = Mat4::from_rotation_translation(Quat::from_array(q), Vec3::from(p));
                self.car(&mut d, body, None, Some(tint(theme::rgb8(theme::GHOST), 0.42)), realistic, 0.0);
            }
        }
        let key: Vec<(Shared<Vec<Frame>>, u8)> =
            s.refs.iter().map(|r| (Shared(r.frames.clone()), r.alpha)).collect();
        if !self.ref_set.fresh(&key) {
            let mut v = DrawList::default();
            for r in &s.refs {
                let c = rgba(theme::GHOST, r.alpha);
                for w in r.frames.windows(2) {
                    v.line(
                        Vec3::from(w[0].position()) + Vec3::Y * 0.3,
                        Vec3::from(w[1].position()) + Vec3::Y * 0.3,
                        c,
                    );
                }
            }
            self.ref_set.store(&mut self.renderer, key, &v.lines);
        }
        d.line_sets.extend(self.ref_set.set);
        for r in &s.refs {
            if r.car
                && let Some(t) = race_ms
                && let Some(f) = r.frames.iter().find(|f| f.race_ms >= t)
            {
                let ax = [Vec3::from(f.axis(0)), Vec3::from(f.axis(1)), Vec3::from(f.axis(2))];
                let body = Mat4::from_cols(
                    ax[0].extend(0.0),
                    ax[1].extend(0.0),
                    ax[2].extend(0.0),
                    Vec3::from(f.position()).extend(1.0),
                );
                self.car(
                    &mut d,
                    body,
                    Some(&f.wheels),
                    Some(tint(theme::rgb8(theme::GHOST), 0.5)),
                    realistic,
                    0.0,
                );
            }
        }
        for g in s.guides {
            let sel = s.selected_guide == Some(g.id);
            let col = rgba(theme::WARN, if sel { 255 } else { 170 });
            for w in g.points.windows(2) {
                d.overlay_line(Vec3::from(w[0]), Vec3::from(w[1]), col);
                d.line(Vec3::from(w[0]), Vec3::from(w[1]), col);
            }
            let sz = if sel { 0.45 } else { 0.3 };
            for p in &g.points {
                d.instances.push(Instance {
                    mesh: self.octa,
                    texture: TextureId(0),
                    transform: Mat4::from_scale_rotation_translation(
                        Vec3::splat(sz),
                        Quat::IDENTITY,
                        Vec3::from(*p),
                    ),
                    tint: tint(theme::rgb8(theme::WARN), 1.0),
                    pass: Pass::Auto,
                });
            }
        }
        if let Some(tries) = s.tries {
            let key = Shared(tries.clone());
            if !self.tries_set.fresh(&key) {
                let mut v = DrawList::default();
                let n = tries.len().max(1) as f32;
                for (i, (path, mark)) in tries.iter().enumerate() {
                    let age = (i as f32 + 1.0) / n;
                    let c = match mark {
                        crate::optimize::Mark::Reached => rgba(theme::OK, (70.0 + 150.0 * age) as u8),
                        crate::optimize::Mark::Missed => rgba(theme::TEXT, (22.0 + 70.0 * age) as u8),
                    };
                    for w in path.windows(2) {
                        v.line(Vec3::from(w[0]) + Vec3::Y * 0.25, Vec3::from(w[1]) + Vec3::Y * 0.25, c);
                    }
                }
                self.tries_set.store(&mut self.renderer, key, &v.lines);
            }
            d.line_sets.extend(self.tries_set.set);
        }
        if let Some(bs) = s.branches {
            for b in bs {
                for w in b.points.windows(2) {
                    d.overlay_line(
                        Vec3::from(w[0]) + Vec3::Y * 0.3,
                        Vec3::from(w[1]) + Vec3::Y * 0.3,
                        rgba(theme::WARN, 200),
                    );
                }
            }
        }
        let goal_gates: Vec<u32> = s
            .goals
            .iter()
            .filter_map(|g| if let GoalKind::Gate(x) = &g.kind { Some(x) } else { None })
            .flat_map(|x| std::iter::once(x.waypoint).chain(x.also.iter().copied()))
            .collect();
        if s.show_gates {
            for g in s.gates {
                let is_goal = goal_gates.contains(&g.gate.waypoint);
                let hot = s.hover_gate == Some(g.gate.waypoint) || s.selected_gate == Some(g.gate.waypoint);
                let c = if hot {
                    rgba(theme::TEXT, 255)
                } else if is_goal {
                    rgba(theme::PIN, 0)
                } else if g.gate.kind == 1 {
                    rgba(theme::WARN, 150)
                } else {
                    rgba(theme::MUTED, 120)
                };
                if c[3] == 0 {
                    continue;
                }
                for b in &g.gate.boxes {
                    box_lines(&mut d, Vec3::from(b.min), Vec3::from(b.max), c, hot);
                }
            }
        }
        for (i, g) in s.goals.iter().enumerate() {
            if !g.visible {
                continue;
            }
            let sel = s.selected_goal == Some(g.id);
            let reached = s.reached.get(i).copied().unwrap_or(false);
            let pin = theme::rgb8(theme::PIN);
            let a = if sel { 255 } else { 190 };
            match &g.kind {
                GoalKind::Gate(gate) => {
                    for b in &gate.boxes {
                        box_lines(
                            &mut d,
                            Vec3::from(b.min),
                            Vec3::from(b.max),
                            [pin[0], pin[1], pin[2], a],
                            sel,
                        );
                        let (mn, mx) = (Vec3::from(b.min), Vec3::from(b.max));
                        d.instances.push(Instance {
                            mesh: self.unit_box,
                            texture: TextureId(0),
                            transform: Mat4::from_scale_rotation_translation(
                                (mx - mn) * 0.5,
                                Quat::IDENTITY,
                                (mn + mx) * 0.5,
                            ),
                            tint: tint(pin, if sel { 0.16 } else { 0.08 }),
                            pass: Pass::Glass,
                        });
                    }
                }
                GoalKind::Point { pos, radius } => {
                    let p = Vec3::from(*pos);
                    circle(&mut d, p, Vec3::Y, *radius, [pin[0], pin[1], pin[2], a]);
                    circle(&mut d, p, Vec3::X, *radius, [pin[0], pin[1], pin[2], a / 2]);
                    circle(&mut d, p, Vec3::Z, *radius, [pin[0], pin[1], pin[2], a / 2]);
                    let top = p + Vec3::Y * 2.6;
                    d.line(p, top, [pin[0], pin[1], pin[2], 255]);
                    d.overlay_line(p, top, [pin[0], pin[1], pin[2], 70]);
                    let sz = if sel { 0.7 } else { 0.55 };
                    let col = if reached { pin } else { theme::rgb8(theme::MUTED) };
                    d.instances.push(Instance {
                        mesh: self.octa,
                        texture: TextureId(0),
                        transform: Mat4::from_scale_rotation_translation(
                            Vec3::new(sz, sz * 1.4, sz),
                            Quat::IDENTITY,
                            top + Vec3::Y * sz * 1.4,
                        ),
                        tint: tint(col, 1.0),
                        pass: Pass::Auto,
                    });
                }
                GoalKind::Box { pos, size, ypr } => {
                    let (c, h, q) = (Vec3::from(*pos), Vec3::from(*size) * 0.5, crate::doc::ypr_rot(*ypr));
                    let col = [pin[0], pin[1], pin[2], a];
                    oriented_box_lines(&mut d, c, h, q, col);
                    d.instances.push(Instance {
                        mesh: self.unit_box,
                        texture: TextureId(0),
                        transform: Mat4::from_scale_rotation_translation(h, q, c),
                        tint: tint(
                            if reached { pin } else { theme::rgb8(theme::MUTED) },
                            if sel { 0.22 } else { 0.12 },
                        ),
                        pass: Pass::Glass,
                    });
                }
                GoalKind::Pose(p) => {
                    let q = crate::doc::ypr_rot(p.ypr);
                    let body = Mat4::from_rotation_translation(q, Vec3::from(p.pos));
                    self.car(
                        &mut d,
                        body,
                        None,
                        Some(tint(pin, if sel { 0.5 } else { 0.32 })),
                        realistic,
                        0.0,
                    );
                    let c = Vec3::from(p.pos);
                    let t = Vec3::from(p.pos_tol);
                    box_lines(
                        &mut d,
                        c - t,
                        c + t,
                        [pin[0], pin[1], pin[2], if sel { 200 } else { 90 }],
                        false,
                    );
                    let f = q * Vec3::Z;
                    let side = q * Vec3::X;
                    let (a0, b0) = (c + Vec3::Y * 0.1 + f * 2.6, c + Vec3::Y * 0.1 + f * 4.4);
                    d.line(a0, b0, [pin[0], pin[1], pin[2], 255]);
                    d.line(b0, b0 - f * 0.8 + side * 0.6, [pin[0], pin[1], pin[2], 255]);
                    d.line(b0, b0 - f * 0.8 - side * 0.6, [pin[0], pin[1], pin[2], 255]);
                }
            }
        }
        if let Some(st) = s.start {
            self.start_marker(&mut d, st, s.start_selected, false, realistic);
            if let Some(hot) = s.start_handle {
                let tip = velocity_tip(st);
                let sz = if hot { 0.75 } else { 0.5 };
                d.instances.push(Instance {
                    mesh: self.octa,
                    texture: TextureId(0),
                    transform: Mat4::from_scale_rotation_translation(Vec3::splat(sz), Quat::IDENTITY, tip),
                    tint: tint(theme::rgb8(if hot { theme::TEXT } else { theme::PIN }), 1.0),
                    pass: Pass::Auto,
                });
                if Vec3::from(st.velocity).length() * VEL_ARROW_S <= 3.0 {
                    d.overlay_line(Vec3::from(st.pos) + Vec3::Y * 0.6, tip, rgba(theme::PIN, 120));
                }
            }
        }
        if let Some((a, b)) = s.stretch {
            let (a, b) = (a.min(s.frames.len()), b.min(s.frames.len().saturating_sub(1)));
            if a < b {
                for w in s.frames[a..=b].windows(2) {
                    let (p, q) = (
                        Vec3::from(w[0].position()) + Vec3::Y * 0.35,
                        Vec3::from(w[1].position()) + Vec3::Y * 0.35,
                    );
                    d.overlay_line(p, q, rgba(theme::PIN, 150));
                }
            }
        }
        if s.show_trail {
            for t in s.tricks {
                let col = rgba(crate::lab::trick_color(t.kind), 255);
                let to = t.to.min(s.frames.len());
                for k in t.from..to.saturating_sub(1) {
                    let (p, q) = (Vec3::from(s.frames[k].position()), Vec3::from(s.frames[k + 1].position()));
                    let side = (q - p).cross(Vec3::Y).normalize_or_zero() * 0.35;
                    for o in [Vec3::ZERO, side, -side, Vec3::Y * 0.3] {
                        d.overlay_line(p + Vec3::Y * 0.45 + o, q + Vec3::Y * 0.45 + o, col);
                    }
                }
            }
        }
        for (st, sel) in &s.setups {
            let rgb = theme::rgb8(theme::SETUP);
            let q = crate::doc::ypr_rot(st.start.ypr);
            self.car(
                &mut d,
                Mat4::from_rotation_translation(q, Vec3::from(st.start.pos)),
                None,
                Some(tint(rgb, if *sel { 0.7 } else { 0.45 })),
                realistic,
                0.0,
            );
            let qe = crate::doc::ypr_rot(st.end_ypr);
            self.car(
                &mut d,
                Mat4::from_rotation_translation(qe, Vec3::from(st.end_pos)),
                None,
                Some(tint(rgb, if *sel { 0.35 } else { 0.18 })),
                realistic,
                0.0,
            );
            let (a, b) = (Vec3::from(st.start.pos) + Vec3::Y * 0.7, Vec3::from(st.end_pos) + Vec3::Y * 0.7);
            let n = ((b - a).length() / 1.5).ceil().max(1.0) as usize;
            for i in (0..n).step_by(2) {
                d.overlay_line(
                    a.lerp(b, i as f32 / n as f32),
                    a.lerp(b, ((i + 1) as f32 / n as f32).min(1.0)),
                    rgba(theme::SETUP, if *sel { 255 } else { 160 }),
                );
            }
            if *sel {
                circle(
                    &mut d,
                    Vec3::from(st.start.pos) + Vec3::Y * 0.05,
                    Vec3::Y,
                    2.8,
                    rgba(theme::SETUP, 255),
                );
            }
        }
        if let Some(st) = s.car_preview {
            self.start_marker(&mut d, st, false, true, realistic);
        }
        if let Some(c) = &s.contacts {
            let col = theme::rgba8(theme::GHOST, 255);
            for (center, hit) in c {
                d.overlay_line(
                    *center - Vec3::X * 0.12,
                    *center + Vec3::X * 0.12,
                    theme::rgba8(theme::TEXT, 255),
                );
                d.overlay_line(
                    *center - Vec3::Z * 0.12,
                    *center + Vec3::Z * 0.12,
                    theme::rgba8(theme::TEXT, 255),
                );
                if let Some(h) = hit {
                    d.overlay_line(*center, *h, col);
                    for a in [Vec3::X, Vec3::Z] {
                        d.overlay_line(*h - a * 0.25, *h + a * 0.25, col);
                    }
                }
            }
        }
        if let Some((gz, active)) = s.gizmo {
            gz.draw(&mut d, active);
        }
        if let Some(h) = s.hover {
            d.overlay_line(h, h + Vec3::Y * 1.5, theme::rgba8(theme::PIN, 160));
        }
        if let Some(p) = s.pivot {
            for a in [Vec3::X, Vec3::Y, Vec3::Z] {
                d.overlay_line(p - a * 0.6, p + a * 0.6, theme::rgba8(theme::TEXT, 200));
            }
        }
        if let Some(f) = s.car.as_ref() {
            let ax = [Vec3::from(f.axis(0)), Vec3::from(f.axis(1)), Vec3::from(f.axis(2))];
            let body = Mat4::from_cols(
                ax[0].extend(0.0),
                ax[1].extend(0.0),
                ax[2].extend(0.0),
                Vec3::from(f.position()).extend(1.0),
            );
            let touching = f.wheels.iter().filter(|w| w.contact).count() as f32 / 4.0;
            self.car(&mut d, body, Some(&f.wheels), None, realistic, touching);
        }
        d
    }
}
