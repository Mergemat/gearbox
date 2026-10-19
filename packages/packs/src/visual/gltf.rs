use super::*;
use anyhow::Result;
use serde_json::{Value, json};
use std::io::Cursor;

#[derive(Default)]
struct Buffer {
    bytes: Vec<u8>,
    views: Vec<Value>,
    accessors: Vec<Value>,
}
impl Buffer {
    fn view(&mut self, data: &[u8], target: Option<u32>) -> usize {
        while !self.bytes.len().is_multiple_of(4) {
            self.bytes.push(0);
        }
        let offset = self.bytes.len();
        self.bytes.extend_from_slice(data);
        let i = self.views.len();
        let mut v = json!({"buffer":0,"byteOffset":offset,"byteLength":data.len()});
        if let Some(target) = target {
            v["target"] = json!(target);
        }
        self.views.push(v);
        i
    }
    fn floats<const N: usize>(&mut self, values: &[[f32; N]], kind: &str, position: bool) -> usize {
        let bytes: Vec<u8> = values.iter().flatten().flat_map(|f| f.to_le_bytes()).collect();
        let view = self.view(&bytes, Some(34962));
        let i = self.accessors.len();
        let mut a = json!({"bufferView":view,"componentType":5126,"count":values.len(),"type":kind});
        if position {
            let b = bounds(values.iter().map(|v| [v[0], v[1], v[2]])).unwrap();
            a["min"] = json!(b[0]);
            a["max"] = json!(b[1]);
        }
        self.accessors.push(a);
        i
    }
    fn indices(&mut self, values: &[u32]) -> usize {
        let bytes: Vec<_> = values.iter().flat_map(|i| i.to_le_bytes()).collect();
        let view = self.view(&bytes, Some(34963));
        let i = self.accessors.len();
        self.accessors
            .push(json!({"bufferView":view,"componentType":5125,"count":values.len(),"type":"SCALAR"}));
        i
    }
}
pub fn export(path: &std::path::Path, scene: &Scene, cache: &VisualCache) -> Result<()> {
    let mut b = Buffer::default();
    let mut meshes = Vec::new();
    let mut nodes = Vec::new();
    let mut mesh_variants = HashMap::new();
    let mut attributes = HashMap::new();
    let mut textures = Vec::new();
    let mut images = Vec::new();
    let mut texture_map = HashMap::new();
    let used: std::collections::BTreeSet<_> = scene
        .instances
        .iter()
        .flat_map(|i| {
            cache.meshes[i.mesh_id]
                .submeshes
                .iter()
                .map(|s| i.material_overrides.get(&s.material).copied().unwrap_or(s.material))
        })
        .collect();
    for &id in &used {
        for binding in selected_textures(&cache.materials[id]) {
            if texture_map.contains_key(&binding.texture) {
                continue;
            }
            let tex = &cache.textures[binding.texture];
            match tex.rgba8() {
                Ok(mut image) => {
                    if binding.role == TextureRole::Normal {
                        let bc5 = matches!(
                            tex.data,
                            TextureData::Dds {
                                format: image_dds::ImageFormat::BC5RgUnorm
                                    | image_dds::ImageFormat::BC5RgSnorm,
                                ..
                            }
                        );
                        for pixel in image.pixels_mut() {
                            let [r, g, b, a] = pixel.0;
                            let (x, y) = if bc5 {
                                (r, g)
                            } else {
                                match binding.bitmap_usage {
                                    Some(19 | 20) => (r, a),
                                    Some(26 | 27) => (a, g),
                                    _ => (r, g),
                                }
                            };
                            let z = if bc5 || matches!(binding.bitmap_usage, Some(11)) {
                                let x = x as f32 / 127.5 - 1.;
                                let y = y as f32 / 127.5 - 1.;
                                (((1. - x * x - y * y).max(0.).sqrt() * 0.5 + 0.5) * 255.).round() as u8
                            } else {
                                b
                            };
                            pixel.0 = [x, y, z, 255];
                        }
                    }
                    let mut png = Cursor::new(Vec::new());
                    image.write_to(&mut png, image::ImageFormat::Png)?;
                    let view = b.view(png.get_ref(), None);
                    let i = images.len();
                    images.push(json!({"name":tex.name,"bufferView":view,"mimeType":"image/png"}));
                    texture_map.insert(binding.texture, i);
                    textures.push(json!({"source":i,"sampler":0}));
                }
                Err(e) => anyhow::bail!("GLB texture {}: {e:#}", tex.name),
            }
        }
    }
    let mut materials = Vec::new();
    let mut material_map = HashMap::new();
    for id in used {
        let m = &cache.materials[id];
        let mut pbr = json!({"baseColorFactor":m.base_color.unwrap_or([1.;4]),"metallicFactor":0.0,"roughnessFactor":0.8});
        let mut material = json!({"name":m.name,"doubleSided":true,"extras":{"surfaceId":m.surface_id,"gameplayId":m.gameplay_id}});
        for binding in selected_textures(m) {
            let Some(&texture) = texture_map.get(&binding.texture) else {
                continue;
            };
            match binding.role {
                TextureRole::Albedo => pbr["baseColorTexture"] = json!({"index":texture}),
                TextureRole::Normal => material["normalTexture"] = json!({"index":texture}),
                TextureRole::Emissive => {
                    material["emissiveTexture"] = json!({"index":texture});
                    material["emissiveFactor"] = json!([1., 1., 1.]);
                }
                _ => {}
            }
        }
        material["pbrMetallicRoughness"] = pbr;
        material_map.insert(id, materials.len());
        materials.push(material);
    }
    for instance in &scene.instances {
        let key =
            (instance.mesh_id, instance.material_overrides.iter().map(|(a, b)| (*a, *b)).collect::<Vec<_>>());
        let mesh_index = if let Some(&i) = mesh_variants.get(&key) {
            i
        } else {
            let m = &cache.meshes[instance.mesh_id];
            let attrs = attributes
                .entry(instance.mesh_id)
                .or_insert_with(|| {
                    let mut a = json!({"POSITION":b.floats(&m.positions,"VEC3",true)});
                    if m.normals.len() == m.positions.len() {
                        a["NORMAL"] = json!(b.floats(&m.normals, "VEC3", false));
                    }
                    for (ch, uv) in m.uvs.iter().enumerate() {
                        if uv.len() == m.positions.len() {
                            a[format!("TEXCOORD_{ch}")] = json!(b.floats(uv, "VEC2", false));
                        }
                    }
                    if let Some(c) = m.colors.first().filter(|c| c.len() == m.positions.len()) {
                        a["COLOR_0"] = json!(b.floats(c, "VEC4", false));
                    }
                    a
                })
                .clone();
            let mut primitives = Vec::new();
            for s in &m.submeshes {
                let mat = instance.material_overrides.get(&s.material).copied().unwrap_or(s.material);
                let first = s.first_index as usize;
                let indices = b.indices(&m.indices[first..first + s.index_count as usize]);
                primitives.push(
                    json!({"attributes":attrs,"indices":indices,"material":material_map[&mat],"mode":4}),
                );
            }
            let i = meshes.len();
            meshes.push(json!({"name":m.name,"primitives":primitives}));
            mesh_variants.insert(key, i);
            i
        };
        let t = instance.transform;
        nodes.push(json!({"name":instance.name,"mesh":mesh_index,"matrix":[t[0],t[3],t[6],0.,t[1],t[4],t[7],0.,t[2],t[5],t[8],0.,t[9],t[10],t[11],1.]}));
    }
    let mut doc = serde_json::to_vec(
        &json!({"asset":{"version":"2.0","generator":"tmassets"},"scene":0,"scenes":[{"nodes":(0..nodes.len()).collect::<Vec<_>>()}],"nodes":nodes,"meshes":meshes,"materials":materials,"textures":textures,"images":images,"samplers":[{"magFilter":9729,"minFilter":9987,"wrapS":10497,"wrapT":10497}],"buffers":[{"byteLength":b.bytes.len()}],"bufferViews":b.views,"accessors":b.accessors,"extras":{"diagnostics":scene.diagnostics}}),
    )?;
    while doc.len() % 4 != 0 {
        doc.push(b' ');
    }
    while b.bytes.len() % 4 != 0 {
        b.bytes.push(0);
    }
    let mut out = Vec::new();
    out.extend_from_slice(&0x46546c67u32.to_le_bytes());
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&((12 + 8 + doc.len() + 8 + b.bytes.len()) as u32).to_le_bytes());
    out.extend_from_slice(&(doc.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x4e4f534au32.to_le_bytes());
    out.extend_from_slice(&doc);
    out.extend_from_slice(&(b.bytes.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x004e4942u32.to_le_bytes());
    out.extend_from_slice(&b.bytes);
    std::fs::write(path, out)?;
    Ok(())
}

fn selected_textures(material: &Material) -> Vec<&TextureBinding> {
    [TextureRole::Albedo, TextureRole::Normal, TextureRole::Emissive]
        .into_iter()
        .filter_map(|role| {
            material.textures.iter().filter(|b| b.role == role).max_by_key(|b| {
                let name = b.sampler.to_ascii_lowercase();
                if name.starts_with('t') && (name.contains("basecolor") || name.contains("normal")) {
                    5
                } else if name == "basecolor" || name == "normal" || name == "selfillum" {
                    4
                } else if name.starts_with("py") || name.starts_with("pxz") {
                    1
                } else {
                    3
                }
            })
        })
        .collect()
}
