mod car;
mod decode;
pub mod gltf;
mod material;
mod scene;
mod texture;
mod tree_pose;
pub use crate::placement;

pub use car::{Bone, CarVisual, WheelPivot};
pub use scene::visual_scene;
pub use texture::{Texture, TextureData};

use crate::packs::PackSet;
use anyhow::Result;
use serde::Serialize;
use std::collections::{BTreeMap, HashMap};

pub type MeshId = usize;
pub type MaterialId = usize;
pub type TextureId = usize;
pub type Transform = [f32; 12];
pub const IDENTITY: Transform = [1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0.];

#[derive(Clone, Debug, Default, Serialize)]
pub struct Mesh {
    pub name: String,
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<Vec<[f32; 2]>>,
    pub tangent_u: Vec<[f32; 3]>,
    pub tangent_v: Vec<[f32; 3]>,
    pub colors: Vec<Vec<[f32; 4]>>,
    pub joint_indices: Vec<[u8; 4]>,
    pub blend_weights: Vec<[f32; 2]>,
    pub skin_bones: Vec<String>,
    pub vegetation: Option<crate::veget::Params>,
    pub indices: Vec<u32>,
    pub submeshes: Vec<Submesh>,
    pub lod: u32,
}
#[derive(Clone, Debug, Serialize)]
pub struct Submesh {
    pub first_index: u32,
    pub index_count: u32,
    pub material: MaterialId,
    pub lod_mask: u32,
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct Material {
    pub name: String,
    pub surface_id: Option<u8>,
    pub gameplay_id: Option<u8>,
    pub textures: Vec<TextureBinding>,
    pub base_color: Option<[f32; 4]>,
}
#[derive(Clone, Debug, Serialize)]
pub struct TextureBinding {
    pub sampler: String,
    pub role: TextureRole,
    pub texture: TextureId,
    pub bitmap_usage: Option<u8>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum TextureRole {
    Albedo,
    Normal,
    Specular,
    Emissive,
    Other,
}
#[derive(Clone, Debug, Serialize)]
pub struct Instance {
    pub name: String,
    pub mesh_id: MeshId,
    pub transform: Transform,
    pub material_overrides: BTreeMap<MaterialId, MaterialId>,
}
#[derive(Clone, Debug, Serialize)]
pub struct Diagnostic {
    pub feature: String,
    pub asset: String,
    pub message: String,
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct Scene {
    pub instances: Vec<Instance>,
    pub diagnostics: Vec<Diagnostic>,
    pub bounds_checks: Vec<BoundsCheck>,
    pub placements: usize,
    pub decoded_placements: usize,
}
#[derive(Clone, Debug, Serialize)]
pub struct BoundsCheck {
    pub block: String,
    pub visual: [[f32; 3]; 2],
    pub collision: [[f32; 3]; 2],
    pub overlap: bool,
}
#[derive(Clone, Debug)]
pub(crate) struct Part {
    pub mesh: MeshId,
    pub chain: Vec<crate::assets::Loc>,
}

struct Decoded {
    result: std::result::Result<Vec<Part>, String>,
    diagnostics: Vec<Diagnostic>,
}
pub struct VisualCache {
    pub meshes: Vec<Mesh>,
    pub materials: Vec<Material>,
    pub textures: Vec<Texture>,
    pub diagnostics: Vec<Diagnostic>,
    packs: PackSet,
    cars: HashMap<crate::assets::Car, CarVisual>,
    files: HashMap<String, Vec<u8>>,
    decoded: HashMap<String, Decoded>,
    material_ids: HashMap<String, MaterialId>,
    texture_ids: HashMap<String, TextureId>,
    embedded: HashMap<String, Vec<u8>>,
    embedded_models: HashMap<String, String>,
}
impl VisualCache {
    pub fn new(packs: PackSet) -> Self {
        Self {
            packs,
            cars: HashMap::new(),
            meshes: Vec::new(),
            materials: Vec::new(),
            textures: Vec::new(),
            diagnostics: Vec::new(),
            files: HashMap::new(),
            decoded: HashMap::new(),
            material_ids: HashMap::new(),
            texture_ids: HashMap::new(),
            embedded: HashMap::new(),
            embedded_models: HashMap::new(),
        }
    }
    pub fn model(&mut self, path: &str, variant: usize) -> Result<Vec<Instance>> {
        Ok(self
            .file_parts(path, variant, 0)?
            .into_iter()
            .map(|p| Instance {
                name: path.into(),
                mesh_id: p.mesh,
                transform: transform(chain(&p.chain)),
                material_overrides: BTreeMap::new(),
            })
            .collect())
    }
    pub fn car(&mut self, car: crate::assets::Car) -> Result<CarVisual> {
        if let Some(visual) = self.cars.get(&car) {
            self.diagnostics.extend(visual.diagnostics.clone());
            return Ok(visual.clone());
        }
        let visual = car::load(self, car)?;
        self.cars.insert(car, visual.clone());
        Ok(visual)
    }
    fn report(&mut self, feature: &str, asset: &str, message: impl ToString) {
        self.diagnostics.push(Diagnostic {
            feature: feature.into(),
            asset: asset.into(),
            message: message.to_string(),
        });
    }
    fn read(&mut self, path: &str) -> Result<Vec<u8>> {
        let key = norm(path);
        if let Some(bytes) = self.files.get(&key) {
            return Ok(bytes.clone());
        }
        let bytes = match self.embedded.get(&key) {
            Some(b) if b.starts_with(b"GBX") => crate::embedded::plain_gbx(b)?,
            Some(b) => b.clone(),
            None => self.packs.read_path(path)?,
        };
        self.files.insert(key, bytes.clone());
        Ok(bytes)
    }
}
fn norm(path: &str) -> String {
    path.replace('/', "\\").to_ascii_lowercase()
}
pub(crate) fn transform(loc: crate::assets::Loc) -> Transform {
    let mut out = IDENTITY;
    let [x, y, z, w] = loc.rotation;
    out[..9].copy_from_slice(&crate::reader_math::quat_matrix([w, x, y, z]));
    out[9..].copy_from_slice(&loc.position);
    out
}
pub(crate) fn chain(chain: &[crate::assets::Loc]) -> crate::assets::Loc {
    let mut it = chain.iter().rev();
    let mut acc = it.next().copied().unwrap_or(crate::assets::Loc::IDENTITY);
    for parent in it {
        acc = scene::iso_loc(placement::compose(scene::loc_iso(*parent), scene::loc_iso(acc)));
    }
    acc
}
pub fn point(t: Transform, p: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| (t[i * 3] * p[0] + t[i * 3 + 1] * p[1]) + t[i * 3 + 2] * p[2] + t[9 + i])
}
pub fn bounds(points: impl IntoIterator<Item = [f32; 3]>) -> Option<[[f32; 3]; 2]> {
    let mut b = [[f32::INFINITY; 3], [f32::NEG_INFINITY; 3]];
    let mut any = false;
    for p in points {
        any = true;
        for k in 0..3 {
            b[0][k] = b[0][k].min(p[k]);
            b[1][k] = b[1][k].max(p[k]);
        }
    }
    any.then_some(b)
}
