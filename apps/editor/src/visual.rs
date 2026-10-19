use std::path::Path;
use std::time::Instant;

use anyhow::{Context, Result};
use gearbox_packs::visual::{self as tv, TextureData, VisualCache};
use glam::{Mat4, Vec3};

use crate::gfx::mesh::Vertex;

pub struct TexData {
    pub width: u32,
    pub height: u32,
    pub format: wgpu::TextureFormat,
    pub mips: Vec<Vec<u8>>,
}

pub struct MeshData {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub parts: Vec<(u32, u32, usize)>,
}

#[derive(Clone, Copy)]
pub struct Placed {
    pub mesh: usize,
    pub part: usize,
    pub material: usize,
    pub transform: Mat4,
    pub surroundings: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MatKind {
    Opaque,
    Cutout,
    Decal,
    Glass,
    Holo,
    Glow,
    Skip,
}

pub struct MaterialData {
    pub kind: MatKind,
    pub texture: Option<usize>,
    pub base: [f32; 4],
    pub gameplay: Option<u8>,
}

pub struct CarData {
    pub body: Vec<Placed>,
    pub wheels: Vec<WheelData>,
}

pub struct WheelData {
    pub pivot: Mat4,
    pub parts: Vec<Placed>,
}

pub struct VisualWorld {
    pub meshes: Vec<MeshData>,
    pub materials: Vec<MaterialData>,
    pub textures: Vec<Option<TexData>>,
    pub placed: Vec<Placed>,
    pub car: Option<CarData>,
    pub report: String,
}

pub fn mat4(t: &tv::Transform) -> Mat4 {
    Mat4::from_cols_array(&[
        t[0], t[3], t[6], 0.0, t[1], t[4], t[7], 0.0, t[2], t[5], t[8], 0.0, t[9], t[10], t[11], 1.0,
    ])
}

pub fn track_box(map: &gearbox_gbx::Map) -> Option<(Vec3, Vec3)> {
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for b in &map.blocks {
        let p = match b.free {
            Some((pos, _)) => Vec3::from(pos),
            None => {
                Vec3::new(b.coord[0] as f32 * 32.0, b.coord[1] as f32 * 8.0 - 64.0, b.coord[2] as f32 * 32.0)
            }
        };
        lo = lo.min(p);
        hi = hi.max(p + Vec3::new(32.0, 8.0, 32.0));
    }
    for it in &map.items {
        lo = lo.min(Vec3::from(it.position));
        hi = hi.max(Vec3::from(it.position));
    }
    (lo.x <= hi.x).then(|| (lo - Vec3::new(48.0, 96.0, 48.0), hi + Vec3::new(48.0, 160.0, 48.0)))
}

pub fn load(packs: &Path, map: &Path, bc: bool) -> Result<VisualWorld> {
    let t0 = Instant::now();
    let set = gearbox_packs::packs::PackSet::open(packs)?;
    let assets = gearbox_packs::assets::Assets::from_catalog(gearbox_packs::catalog::cached(&set)?);
    let parsed = gearbox_gbx::Map::parse(&std::fs::read(map)?)
        .with_context(|| format!("parsing {}", map.display()))?;
    let track = track_box(&parsed);
    let mut cache = VisualCache::new(set);
    let scene = tv::visual_scene(&parsed, &assets, &mut cache)?;
    let car_name = parsed.player_model.as_deref().unwrap_or("CarSport");
    let car = match car_name {
        "CarSnow" => gearbox_packs::assets::Car::Snow,
        "CarRally" => gearbox_packs::assets::Car::Rally,
        "CarDesert" => gearbox_packs::assets::Car::Desert,
        _ => gearbox_packs::assets::Car::Sport,
    };
    let car_visual = cache.car(car).ok();
    let t_scene = t0.elapsed();

    let t1 = Instant::now();
    let meshes: Vec<MeshData> = cache.meshes.iter().map(mesh_data).collect();
    let mut materials: Vec<MaterialData> = cache
        .materials
        .iter()
        .map(|m| MaterialData {
            kind: MatKind::Opaque,
            texture: m.textures.iter().find(|b| b.role == tv::TextureRole::Albedo).map(|b| b.texture),
            base: m.base_color.unwrap_or([1.0; 4]),
            gameplay: m.gameplay_id,
        })
        .collect::<Vec<_>>();
    let n = cache.textures.len();
    let mut textures: Vec<Option<TexData>> = (0..n).map(|_| None).collect();
    let chunk = n.div_ceil(4).max(1);
    std::thread::scope(|s| {
        for (part, out) in cache.textures.chunks(chunk).zip(textures.chunks_mut(chunk)) {
            s.spawn(move || {
                for (t, o) in part.iter().zip(out.iter_mut()) {
                    *o = tex_data(t, bc).ok();
                }
            });
        }
    });
    for (i, m) in cache.materials.iter().enumerate() {
        let md = &mut materials[i];
        md.kind = classify(m, &cache.textures);
        md.texture = albedo(m, &cache.textures).map(|b| b.texture);
        let emissive = m.textures.iter().find(|b| b.role == tv::TextureRole::Emissive).map(|b| b.texture);
        match md.kind {
            MatKind::Decal => {
                let opacity = m.textures.iter().find(|b| b.sampler.eq_ignore_ascii_case("TOpacity"));
                if let (Some(a), Some(o)) = (md.texture, opacity) {
                    match combine_opacity(&cache.textures[a], &cache.textures[o.texture]) {
                        Ok(t) => {
                            textures.push(Some(t));
                            md.texture = Some(textures.len() - 1);
                        }
                        Err(_) => md.kind = MatKind::Skip,
                    }
                }
            }
            MatKind::Glass => {
                md.base = if m.name.to_ascii_lowercase().contains("water") {
                    [0.32, 0.42, 0.52, 0.72]
                } else {
                    [0.7, 0.75, 0.8, 0.35]
                }
            }
            MatKind::Holo => {
                md.texture = emissive;
                let key = std::iter::once(m.name.as_str())
                    .chain(m.textures.iter().map(|b| cache.textures[b.texture].name.as_str()))
                    .collect::<String>();
                let [r, g, b] = holo_rgb(&key);
                md.base = [r, g, b, 0.6];
            }
            MatKind::Glow => {
                md.texture = emissive;
                md.base = [1.0, 1.0, 1.0, 0.75];
            }
            _ => {}
        }
    }
    let t_tex = t1.elapsed();

    let expand =
        |inst: &tv::Instance, extra: Mat4, surroundings: &dyn Fn(Mat4, usize) -> bool| -> Vec<Placed> {
            let tr = extra * mat4(&inst.transform);
            let m = &cache.meshes[inst.mesh_id];
            let sur = surroundings(tr, inst.mesh_id);
            m.submeshes
                .iter()
                .enumerate()
                .map(|(k, sm)| Placed {
                    mesh: inst.mesh_id,
                    part: k,
                    material: inst.material_overrides.get(&sm.material).copied().unwrap_or(sm.material),
                    transform: tr,
                    surroundings: sur,
                })
                .collect()
        };
    let centers: Vec<Vec3> = meshes
        .iter()
        .map(|m| {
            if m.vertices.is_empty() {
                return Vec3::ZERO;
            }
            let (lo, hi) =
                m.vertices.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), v| {
                    (lo.min(Vec3::from(v.pos)), hi.max(Vec3::from(v.pos)))
                });
            (lo + hi) * 0.5
        })
        .collect();
    let is_sur = |tr: Mat4, mesh: usize| match track {
        Some((lo, hi)) => {
            let c = tr.transform_point3(centers[mesh]);
            c.cmplt(lo).any() || c.cmpgt(hi).any()
        }
        None => false,
    };
    let mut placed = Vec::new();
    for inst in &scene.instances {
        placed.extend(expand(inst, Mat4::IDENTITY, &is_sur));
    }
    let car = car_visual.map(|cv| {
        let wheel_set: std::collections::HashSet<usize> =
            cv.wheels.iter().flat_map(|w| w.instances.iter().copied()).collect();
        let none = |_: Mat4, _: usize| false;
        let body = cv
            .instances
            .iter()
            .enumerate()
            .filter(|(i, _)| !wheel_set.contains(i))
            .flat_map(|(_, inst)| expand(inst, Mat4::IDENTITY, &none))
            .collect();
        let wheels = cv
            .wheels
            .iter()
            .map(|w| {
                let parts = w
                    .instances
                    .iter()
                    .flat_map(|&i| expand(&cv.instances[i], Mat4::IDENTITY, &none))
                    .collect();
                WheelData { pivot: mat4(&w.transform), parts }
            })
            .collect();
        CarData { body, wheels }
    });
    let triangles: usize = placed.iter().map(|p| meshes[p.mesh].parts[p.part].1 as usize / 3).sum();
    let sur = placed.iter().filter(|p| p.surroundings).count();
    let report = format!(
        "visual: {} placements → {} instanced parts ({} surroundings), {} meshes, {} materials, {} textures ({} BC, {} failed), {:.2}M triangles; scene {:.2?}, textures {:.2?}, {} diagnostics",
        scene.instances.len(),
        placed.len(),
        sur,
        meshes.len(),
        cache.materials.len(),
        n,
        textures.iter().flatten().filter(|t| t.format.is_compressed()).count(),
        textures.iter().filter(|t| t.is_none()).count(),
        triangles as f32 / 1e6,
        t_scene,
        t_tex,
        scene.diagnostics.len()
    );
    Ok(VisualWorld { meshes, materials, textures, placed, car, report })
}

pub fn albedo<'a>(m: &'a tv::Material, textures: &[tv::Texture]) -> Option<&'a tv::TextureBinding> {
    let mask = |b: &tv::TextureBinding| {
        b.sampler.to_ascii_lowercase().contains("huemask")
            || textures[b.texture].name.to_ascii_lowercase().contains("huemask")
    };
    let mut it = m.textures.iter().filter(|b| b.role == tv::TextureRole::Albedo);
    it.clone().find(|b| !mask(b)).or_else(|| it.next())
}

fn has_alpha(t: &tv::Texture) -> bool {
    use image_dds::ImageFormat as I;
    match &t.data {
        TextureData::Dds { format, .. } => !matches!(
            format,
            I::BC1RgbaUnorm
                | I::BC1RgbaUnormSrgb
                | I::BC4RUnorm
                | I::BC4RSnorm
                | I::BC5RgUnorm
                | I::BC5RgSnorm
                | I::BC6hRgbUfloat
                | I::BC6hRgbSfloat
        ),
        TextureData::Encoded { .. } => true,
    }
}

fn file_name(path: &str) -> String {
    path.rsplit(['\\', '/']).next().unwrap_or(path).to_ascii_lowercase()
}

pub fn classify(m: &tv::Material, textures: &[tv::Texture]) -> MatKind {
    let lname = m.name.to_ascii_lowercase();
    let base = file_name(&m.name);
    let albedo = albedo(m, textures);
    let albedo_file = albedo.map(|b| file_name(&textures[b.texture].name)).unwrap_or_default();
    let emissive = m.textures.iter().find(|b| b.role == tv::TextureRole::Emissive);
    let opacity = m.textures.iter().any(|b| b.sampler.eq_ignore_ascii_case("TOpacity"));
    if lname.contains("invisible") {
        MatKind::Skip
    } else if lname.contains("triggerfx")
        || (albedo.is_none()
            && emissive.is_some_and(|e| file_name(&textures[e.texture].name).contains("triggerfx")))
    {
        MatKind::Holo
    } else if base.starts_with("decal") || albedo_file.starts_with("decal") {
        match albedo {
            Some(a) if opacity || has_alpha(&textures[a.texture]) => MatKind::Decal,
            _ => MatKind::Skip,
        }
    } else if base.starts_with("water.mat")
        || lname.contains("glass")
        || m.textures.iter().any(|b| b.sampler == "Transmittance")
    {
        MatKind::Glass
    } else if albedo.is_none() && emissive.is_some() {
        MatKind::Glow
    } else if albedo.is_some_and(|a| a.sampler.ends_with("Op"))
        || (lname.contains("#tree-material") && albedo.is_some_and(|a| has_alpha(&textures[a.texture])))
    {
        MatKind::Cutout
    } else {
        MatKind::Opaque
    }
}

fn holo_rgb(key: &str) -> [f32; 3] {
    let k = key.to_ascii_lowercase();
    if k.contains("checkpoint") {
        [0.40, 0.62, 0.92]
    } else if k.contains("finish") {
        [0.92, 0.42, 0.46]
    } else if k.contains("start") || k.contains("multilap") {
        [0.42, 0.82, 0.58]
    } else if k.contains("turbo") || k.contains("boost") {
        [0.92, 0.68, 0.38]
    } else if k.contains("reset") {
        [0.36, 0.80, 0.74]
    } else {
        [0.66, 0.58, 0.92]
    }
}

fn mesh_data(m: &tv::Mesh) -> MeshData {
    let uv = m.uvs.first();
    let vertices = m
        .positions
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let n = m.normals.get(i).copied().map(Vec3::from).unwrap_or(Vec3::ZERO);
            let t = uv.and_then(|u| u.get(i)).copied().map_or([0.0; 2], |[u, v]| [u, 1.0 - v]);
            if n == Vec3::ZERO {
                Vertex { pos: *p, normal: crate::gfx::mesh::FLAT, color: [255; 4], uv: t }
            } else {
                Vertex::smooth(Vec3::from(*p), n, [255; 4], t)
            }
        })
        .collect();
    let parts = m.submeshes.iter().map(|s| (s.first_index, s.index_count, s.material)).collect();
    MeshData { vertices, indices: m.indices.clone(), parts }
}

fn tex_data(t: &tv::Texture, bc: bool) -> Result<TexData> {
    if bc
        && let TextureData::Dds { format, mipmaps, layers: 1, depth: 1, blocks } = &t.data
        && let Some(f) = bc_format(*format)
    {
        let (bw, bh) = f.block_dimensions();
        let bytes = f.block_copy_size(None).context("block size")?;
        let mut mips = Vec::new();
        let mut off = 0usize;
        for level in 0..*mipmaps {
            let (w, h) = ((t.width >> level).max(1), (t.height >> level).max(1));
            let len = (w.div_ceil(bw) * h.div_ceil(bh) * bytes) as usize;
            if off + len > blocks.len() {
                break;
            }
            mips.push(blocks[off..off + len].to_vec());
            off += len;
        }
        let mut keep = 0;
        for level in 0..mips.len() as u32 {
            let (w, h) = ((t.width >> level).max(1), (t.height >> level).max(1));
            if level > 0 && (w % bw != 0 || h % bh != 0) {
                break;
            }
            keep += 1;
        }
        mips.truncate(keep.max(1));
        if t.width.is_multiple_of(bw) && t.height.is_multiple_of(bh) {
            return Ok(TexData { width: t.width, height: t.height, format: f, mips });
        }
    }
    Ok(rgba_mips(t.rgba8()?))
}

fn rgba_mips(img: image::RgbaImage) -> TexData {
    let (w, h) = img.dimensions();
    let mut mips = vec![img.clone().into_raw()];
    let mut cur = img;
    while cur.width() > 1 || cur.height() > 1 {
        let (nw, nh) = ((cur.width() / 2).max(1), (cur.height() / 2).max(1));
        cur = image::imageops::resize(&cur, nw, nh, image::imageops::FilterType::Triangle);
        mips.push(cur.as_raw().clone());
    }
    TexData { width: w, height: h, format: wgpu::TextureFormat::Rgba8UnormSrgb, mips }
}

fn combine_opacity(albedo: &tv::Texture, opacity: &tv::Texture) -> Result<TexData> {
    let mut a = albedo.rgba8()?;
    let mut o = opacity.rgba8()?;
    if o.dimensions() != a.dimensions() {
        o = image::imageops::resize(&o, a.width(), a.height(), image::imageops::FilterType::Triangle);
    }
    for (p, q) in a.pixels_mut().zip(o.pixels()) {
        p.0[3] = q.0[0];
    }
    Ok(rgba_mips(a))
}

fn bc_format(f: image_dds::ImageFormat) -> Option<wgpu::TextureFormat> {
    use image_dds::ImageFormat as I;
    use wgpu::TextureFormat as W;
    Some(match f {
        I::BC1RgbaUnorm => W::Bc1RgbaUnormSrgb,
        I::BC1RgbaUnormSrgb => W::Bc1RgbaUnormSrgb,
        I::BC2RgbaUnorm => W::Bc2RgbaUnormSrgb,
        I::BC2RgbaUnormSrgb => W::Bc2RgbaUnormSrgb,
        I::BC3RgbaUnorm => W::Bc3RgbaUnormSrgb,
        I::BC3RgbaUnormSrgb => W::Bc3RgbaUnormSrgb,
        I::BC4RUnorm => W::Bc4RUnorm,
        I::BC4RSnorm => W::Bc4RSnorm,
        I::BC5RgUnorm => W::Bc5RgUnorm,
        I::BC5RgSnorm => W::Bc5RgSnorm,
        I::BC6hRgbUfloat => W::Bc6hRgbUfloat,
        I::BC6hRgbSfloat => W::Bc6hRgbFloat,
        I::BC7RgbaUnorm => W::Bc7RgbaUnormSrgb,
        I::BC7RgbaUnormSrgb => W::Bc7RgbaUnormSrgb,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_fallback(m: &tv::Material, kind: MatKind, textures: &[tv::Texture]) -> bool {
        matches!(kind, MatKind::Opaque | MatKind::Cutout)
            && albedo(m, textures).is_none()
            && m.base_color.is_none()
    }

    #[test]
    fn gates_have_no_opaque_placeholders() {
        let (Some(data), Ok(game)) =
            (std::env::var_os("GBX_TEST_DATA"), gearbox_packs::install::Game::locate(None))
        else {
            eprintln!("skipped: needs the game and GBX_TEST_DATA");
            return;
        };
        let map = Path::new(&data).join("tmx/effects/179381_No_Grip_Dirt.Map.Gbx");
        let set = game.open().unwrap();
        let assets =
            gearbox_packs::assets::Assets::from_catalog(gearbox_packs::catalog::cached(&set).unwrap());
        let parsed = gearbox_gbx::Map::parse(&std::fs::read(&map).unwrap()).unwrap();
        let mut cache = VisualCache::new(set);
        let scene = tv::visual_scene(&parsed, &assets, &mut cache).unwrap();
        let mut holo = 0;
        for inst in scene.instances.iter().filter(|i| i.name.starts_with("Gate")) {
            for sm in &cache.meshes[inst.mesh_id].submeshes {
                let m = &cache.materials
                    [inst.material_overrides.get(&sm.material).copied().unwrap_or(sm.material)];
                let kind = classify(m, &cache.textures);
                assert!(
                    !is_fallback(m, kind, &cache.textures),
                    "{} draws {} as an opaque placeholder",
                    inst.name,
                    m.name
                );
                if m.name.contains("RaceTriggerFXCheckpoint") {
                    assert_eq!(kind, MatKind::Holo, "{}", m.name);
                    holo += 1;
                }
            }
        }
        assert!(holo > 0, "no checkpoint hologram on the map");
    }
}
