use super::*;
use anyhow::Context;
use serde::Serialize;
#[derive(Clone, Debug, Serialize)]
pub struct WheelPivot {
    pub name: String,
    pub transform: Transform,
    pub steering: bool,
    pub driven: bool,
    pub instances: Vec<usize>,
    pub physics_transform: Transform,
}
#[derive(Clone, Debug, Serialize)]
pub struct Bone {
    pub name: String,
    pub bind_transform: Transform,
    pub parent: i16,
}
#[derive(Clone, Debug, Serialize)]
pub struct CarVisual {
    pub car: String,
    pub instances: Vec<Instance>,
    pub wheels: Vec<WheelPivot>,
    pub skeleton: Vec<Bone>,
    pub diagnostics: Vec<Diagnostic>,
}
pub(super) fn load(cache: &mut VisualCache, car: crate::assets::Car) -> Result<CarVisual> {
    let begin = cache.diagnostics.len();
    let model = crate::car_model::load(&cache.packs, car.name())?;
    let skel = model.collision.skeleton.as_ref().context("car skeleton absent")?;
    let mut wheels: Vec<WheelPivot> = model
        .wheels
        .iter()
        .map(|w| {
            let i = skel.names.iter().position(|n| n == &w.name).context("wheel pivot absent")?;
            Ok(WheelPivot {
                name: w.name.clone(),
                transform: skel.transforms[i].map(f32::from_bits),
                steering: w.steering != 0,
                driven: w.driven != 0,
                instances: Vec::new(),
                physics_transform: skel.transforms[i].map(f32::from_bits),
            })
        })
        .collect::<Result<_>>()?;
    let folder = match car {
        crate::assets::Car::Sport => "Stadium/Standard",
        crate::assets::Car::Snow => "Snow",
        crate::assets::Car::Rally => "Rally",
        crate::assets::Car::Desert => "Desert",
    };
    let path = format!("Skins/Models/CarSport/{folder}/MainBody.Mesh.gbx");
    let parts = cache.file_parts(&path, 0, 0)?;
    let texture_folder = if car == crate::assets::Car::Sport { "Stadium/Common" } else { folder };
    let skeleton_path = format!("Skins/Models/CarSport/{texture_folder}/MainBody.Skel.Gbx");
    let bytes = cache.read(&skeleton_path)?;
    let layout = crate::nodes::layout(&bytes)?;
    let mut reader = super::decode::cursor(&bytes, layout.head.body_start)?;
    anyhow::ensure!(reader.u32()? == 0x090ba000 && reader.u32()? == 20, "visual skeleton archive");
    reader.id()?;
    let mut skeleton = Vec::new();
    for _ in 0..reader.u16()? {
        let name = reader.id()?.context("visual bone name absent")?;
        let parent = reader.i16()?;
        let mut bind_transform = IDENTITY;
        for value in &mut bind_transform {
            *value = reader.f32()?;
        }
        skeleton.push(Bone { name, parent, bind_transform });
    }
    for wheel in &mut wheels {
        let visual_name = format!("{}Wheel", wheel.name.strip_suffix("Surf").unwrap_or(&wheel.name));
        wheel.transform = skeleton
            .iter()
            .find(|b| b.name == visual_name)
            .with_context(|| format!("visual wheel joint {visual_name} absent"))?
            .bind_transform;
    }
    let mut overrides = BTreeMap::new();
    for part in &parts {
        for s in cache.meshes[part.mesh].submeshes.clone() {
            let name = cache.materials[s.material].name.clone();
            let group = match name.rsplit('_').next().unwrap_or("").to_ascii_lowercase().as_str() {
                "skin" => "Skin",
                "details" => "Details",
                "wheels" => "Wheels",
                "glass" => "Glass",
                _ => continue,
            };
            let mut material = cache.materials[s.material].clone();
            for (suffix, role) in [
                ("B", TextureRole::Albedo),
                ("N", TextureRole::Normal),
                ("R", TextureRole::Specular),
                ("I", TextureRole::Emissive),
            ] {
                let path = format!("Skins/Models/CarSport/{texture_folder}/{group}_{suffix}.dds");
                if cache.packs.find(&path).is_none() {
                    continue;
                }
                match cache.texture(&path) {
                    Ok(texture) => material.textures.push(TextureBinding {
                        sampler: format!("{group}_{suffix}"),
                        role,
                        texture,
                        bitmap_usage: None,
                    }),
                    Err(e) => cache.report("car-texture", &path, format!("{e:#}")),
                }
            }
            if let std::collections::btree_map::Entry::Vacant(e) = overrides.entry(s.material) {
                let id = cache.materials.len();
                cache.materials.push(material);
                e.insert(id);
            }
        }
    }
    let mut instances = Vec::new();
    for part in parts {
        let mesh = cache.meshes[part.mesh].clone();
        let mut wheel_tris = BTreeMap::<usize, Vec<u32>>::new();
        let mut body = Vec::new();
        for tri in mesh.indices.as_chunks::<3>().0 {
            let wheel = if mesh.joint_indices.len() == mesh.positions.len() && mesh.blend_weights.is_empty() {
                let j = mesh.joint_indices[tri[0] as usize][0];
                if tri.iter().all(|&i| mesh.joint_indices[i as usize][0] == j) {
                    mesh.skin_bones.get(j as usize).and_then(|name| {
                        wheels.iter().position(|w| {
                            name == &format!("{}Wheel", w.name.strip_suffix("Surf").unwrap_or(&w.name))
                        })
                    })
                } else {
                    None
                }
            } else {
                None
            };
            if let Some(wheel) = wheel {
                wheel_tris.entry(wheel).or_default().extend_from_slice(tri);
            } else {
                body.extend_from_slice(tri);
            }
        }
        if !body.is_empty() {
            let id = if wheel_tris.is_empty() {
                part.mesh
            } else {
                let id = cache.meshes.len();
                cache.meshes.push(subset(&mesh, &body, IDENTITY));
                id
            };
            instances.push(Instance {
                name: format!("{}/body/{}", car.name(), mesh.name),
                mesh_id: id,
                transform: transform(chain(&part.chain)),
                material_overrides: overrides.clone(),
            });
        }
        for (wheel, indices) in wheel_tris {
            let pivot = wheels[wheel].transform;
            let inv = inverse(pivot);
            let id = cache.meshes.len();
            cache.meshes.push(subset(&mesh, &indices, inv));
            wheels[wheel].instances.push(instances.len());
            instances.push(Instance {
                name: format!("{}/{}", car.name(), wheels[wheel].name),
                mesh_id: id,
                transform: pivot,
                material_overrides: overrides.clone(),
            });
        }
    }
    for wheel in &wheels {
        if wheel.instances.is_empty() {
            cache.report("car-wheel", car.name(), format!("no rigid triangles assigned to {}", wheel.name));
        }
    }
    Ok(CarVisual {
        car: car.name().into(),
        instances,
        wheels,
        skeleton,
        diagnostics: cache.diagnostics[begin..].to_vec(),
    })
}
fn inverse(t: Transform) -> Transform {
    let mut out = IDENTITY;
    for i in 0..3 {
        for j in 0..3 {
            out[i * 3 + j] = t[j * 3 + i];
        }
    }
    for i in 0..3 {
        out[9 + i] = -(out[i * 3] * t[9] + out[i * 3 + 1] * t[10] + out[i * 3 + 2] * t[11]);
    }
    out
}
fn subset(source: &Mesh, indices: &[u32], t: Transform) -> Mesh {
    let mut mesh = Mesh {
        name: source.name.clone(),
        uvs: vec![Vec::new(); source.uvs.len()],
        colors: vec![Vec::new(); source.colors.len()],
        ..Default::default()
    };
    let mut remap = HashMap::new();
    let rotate =
        |v: [f32; 3]| std::array::from_fn(|i| t[i * 3] * v[0] + t[i * 3 + 1] * v[1] + t[i * 3 + 2] * v[2]);
    for &index in indices {
        let mapped = *remap.entry(index).or_insert_with(|| {
            let i = index as usize;
            let mapped = mesh.positions.len() as u32;
            mesh.positions.push(point(t, source.positions[i]));
            if source.normals.len() == source.positions.len() {
                mesh.normals.push(rotate(source.normals[i]));
            }
            if source.tangent_u.len() == source.positions.len() {
                mesh.tangent_u.push(rotate(source.tangent_u[i]));
            }
            if source.tangent_v.len() == source.positions.len() {
                mesh.tangent_v.push(rotate(source.tangent_v[i]));
            }
            for (out, uv) in mesh.uvs.iter_mut().zip(&source.uvs) {
                if uv.len() == source.positions.len() {
                    out.push(uv[i]);
                }
            }
            for (out, color) in mesh.colors.iter_mut().zip(&source.colors) {
                if color.len() == source.positions.len() {
                    out.push(color[i]);
                }
            }
            mapped
        });
        mesh.indices.push(mapped);
    }
    mesh.submeshes = source
        .submeshes
        .iter()
        .map(|s| Submesh {
            first_index: 0,
            index_count: indices.len() as u32,
            material: s.material,
            lod_mask: s.lod_mask,
        })
        .collect();
    mesh
}
