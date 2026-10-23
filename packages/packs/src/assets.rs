use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail, ensure};

use crate::catalog::{Block, Catalog, EntModel, Index};
use crate::material::MaterialIds;
use crate::surface::{Surf, Surface};

const MAGIC: &[u8; 8] = b"TMASURF1";

pub fn save(cat: &Catalog, dir: &Path) -> Result<()> {
    std::fs::create_dir_all(dir)?;
    std::fs::write(dir.join("index.json"), serde_json::to_vec(&cat.index)?)?;
    let mut b = MAGIC.to_vec();
    b.extend_from_slice(&(cat.surfaces.len() as u32).to_le_bytes());
    for s in &cat.surfaces {
        let e = crate::surface::encode(s);
        b.extend_from_slice(&(e.len() as u32).to_le_bytes());
        b.extend_from_slice(&e);
    }
    std::fs::write(dir.join("surfaces.bin"), b)?;
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Loc {
    pub rotation: [f32; 4],
    pub position: [f32; 3],
}

impl Loc {
    pub const IDENTITY: Loc = Loc { rotation: [0.0, 0.0, 0.0, 1.0], position: [0.0; 3] };

    pub fn then(&self, child: &Loc) -> Loc {
        let [ax, ay, az, aw] = self.rotation;
        let [bx, by, bz, bw] = child.rotation;
        let rotation = [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ];
        let p = self.rotate(child.position);
        Loc {
            rotation,
            position: [p[0] + self.position[0], p[1] + self.position[1], p[2] + self.position[2]],
        }
    }

    fn rotate(&self, v: [f32; 3]) -> [f32; 3] {
        let m = self.matrix();
        [
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
        ]
    }

    pub fn matrix(&self) -> [f32; 9] {
        let [x, y, z, w] = self.rotation;
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Collision,
    DynaShape,
    DynaStaticShape,
    Waypoint(u32),
    SpecialTrigger,
    BlockSpecialTrigger,
    BlockTrigger(Option<u32>),
}

#[derive(Clone, Debug)]
pub struct PlacedSurface<'a> {
    pub id: usize,
    pub role: Role,
    pub surface: &'a Surface,
    pub loc: Loc,
    pub chain: Vec<Loc>,
    pub materials: Vec<Option<(String, MaterialIds)>>,
    pub modified: bool,
    pub vertex_loc: Option<[f32; 12]>,
}

impl PlacedSurface<'_> {
    pub fn game_triangles(&self) -> Option<Vec<[u8; 16]>> {
        let Surf::Mesh { triangles, .. } = &self.surface.surf.as_ref()?.surf else { return None };
        Some(
            triangles
                .iter()
                .map(|t| {
                    let mut b = [0u8; 16];
                    for k in 0..3 {
                        b[4 * k..4 * k + 4].copy_from_slice(&t.v[k].to_le_bytes());
                    }
                    b[12] = t.flags[0];
                    b[13] = t.flags[1];
                    if let Some(Some((_, ids))) = self.materials.get(t.material as usize) {
                        b[12] = ids.surface;
                        b[13] = ids.gameplay;
                    }
                    b[14..].copy_from_slice(&t.material.to_le_bytes());
                    b
                })
                .collect(),
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModifierRef<'a> {
    pub path: &'a str,
    pub folder: Option<&'a str>,
    pub keep_track_wall: bool,
}

pub const TRACK_WALL: &str = "Stadium\\Media\\Material\\TrackWall.Material.Gbx";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Car {
    Sport,
    Snow,
    Rally,
    Desert,
}

impl Car {
    pub const ALL: [Car; 4] = [Car::Sport, Car::Snow, Car::Rally, Car::Desert];

    pub fn name(self) -> &'static str {
        match self {
            Car::Sport => "CarSport",
            Car::Snow => "CarSnow",
            Car::Rally => "CarRally",
            Car::Desert => "CarDesert",
        }
    }

    pub fn from_name(name: &str) -> Option<Car> {
        Car::ALL.into_iter().find(|c| c.name().eq_ignore_ascii_case(name))
    }
}

#[derive(Clone)]
pub struct Assets {
    pub dir: PathBuf,
    pub index: Index,
    surfaces: Vec<Surface>,
    by_name: HashMap<String, Vec<String>>,
}

pub fn block_name(path: &str) -> &str {
    let f = path.rsplit(['\\', '/']).next().unwrap_or(path);
    f.split('.').next().unwrap_or(f)
}

impl Assets {
    pub fn from_catalog(cat: Catalog) -> Self {
        let mut by_name: HashMap<String, Vec<String>> = HashMap::new();
        for (path, block) in &cat.index.blocks {
            by_name
                .entry(block.name.as_deref().unwrap_or_else(|| block_name(path)).to_ascii_lowercase())
                .or_default()
                .push(path.clone());
        }
        for paths in by_name.values_mut() {
            paths.sort_by_key(|p| (p.contains("\\Deprecated\\"), !p.starts_with("Stadium\\"), p.clone()));
        }
        Self { dir: PathBuf::new(), index: cat.index, surfaces: cat.surfaces, by_name }
    }

    pub fn open(dir: &Path) -> Result<Assets> {
        let index: Index =
            serde_json::from_slice(&std::fs::read(dir.join("index.json")).context("index.json")?)?;
        let b = std::fs::read(dir.join("surfaces.bin")).context("surfaces.bin")?;
        ensure!(b.starts_with(MAGIC), "surfaces.bin: bad magic");
        let n = u32::from_le_bytes(b[8..12].try_into().unwrap()) as usize;
        let mut surfaces = Vec::with_capacity(n);
        let mut p = 12;
        for i in 0..n {
            let len = u32::from_le_bytes(b.get(p..p + 4).context("truncated")?.try_into().unwrap()) as usize;
            surfaces.push(
                crate::surface::decode(b.get(p + 4..p + 4 + len).context("truncated")?)
                    .with_context(|| format!("surface {i}"))?,
            );
            p += 4 + len;
        }
        let mut by_name: HashMap<String, Vec<String>> = HashMap::new();
        for (path, block) in &index.blocks {
            by_name
                .entry(block.name.as_deref().unwrap_or_else(|| block_name(path)).to_ascii_lowercase())
                .or_default()
                .push(path.clone());
        }
        for v in by_name.values_mut() {
            v.sort_by_key(|p| (p.contains("\\Deprecated\\"), !p.starts_with("Stadium\\"), p.clone()));
        }
        Ok(Assets { dir: dir.to_owned(), index, surfaces, by_name })
    }

    pub fn surface(&self, id: usize) -> &Surface {
        &self.surfaces[id]
    }

    pub fn surface_count(&self) -> usize {
        self.surfaces.len()
    }

    pub fn add_catalog(&mut self, cat: Catalog) {
        self.surfaces.extend(cat.surfaces);
        self.index.surfaces.extend(cat.index.surfaces);
        self.index.blocks.extend(cat.index.blocks);
        self.index.prefabs.extend(cat.index.prefabs);
        self.index.custom_blocks.extend(cat.index.custom_blocks);
        self.index.items.extend(cat.index.items);
        self.index.materials.extend(cat.index.materials);
        self.index.modifiers.extend(cat.index.modifiers);
        self.index.errors.extend(cat.index.errors);
    }

    pub fn block(&self, name: &str) -> Option<&Block> {
        self.block_in(name, "Stadium")
    }

    pub fn block_in(&self, name: &str, env: &str) -> Option<&Block> {
        if let Some((_, block)) =
            self.index.custom_blocks.iter().find(|(key, _)| key.eq_ignore_ascii_case(name))
        {
            return block.as_ref();
        }
        let paths = self.by_name.get(&name.to_ascii_lowercase())?;
        let own = format!("{env}\\");
        let p = paths
            .iter()
            .filter(|p| p.starts_with(&own) || p.starts_with("Stadium\\") || p.starts_with("Stadium256\\"))
            .min_by_key(|p| {
                (p.contains("\\Deprecated\\"), !p.starts_with(&own), !p.starts_with("Stadium\\"), p.as_str())
            })?;
        self.index.blocks.get(p)
    }

    pub fn block_names(&self) -> impl Iterator<Item = &str> {
        self.by_name.keys().map(String::as_str)
    }

    pub fn block_is_null(&self, name: &str) -> bool {
        self.index.custom_blocks.iter().any(|(key, block)| key.eq_ignore_ascii_case(name) && block.is_none())
    }

    pub fn block_paths(&self, name: &str) -> &[String] {
        self.by_name.get(&name.to_ascii_lowercase()).map(|v| v.as_slice()).unwrap_or(&[])
    }

    pub fn material_ids(&self, path: &str) -> Option<MaterialIds> {
        self.index.materials.get(path).map(|&(surface, gameplay)| MaterialIds { surface, gameplay })
    }

    pub fn block_model(
        &self,
        name: &str,
        ground: bool,
        variant: usize,
        group: usize,
        mobil: usize,
    ) -> Result<Vec<PlacedSurface<'_>>> {
        let b = self.block(name).with_context(|| format!("unknown block {name}"))?;
        let v = b.variants.iter().find(|v| v.ground == ground && v.index == variant).with_context(|| {
            format!("{name}: no {} variant {variant}", if ground { "ground" } else { "air" })
        })?;
        let m = v
            .mobils
            .get(group)
            .and_then(|g| g.get(mobil))
            .with_context(|| format!("{name}: no mobil {group}/{mobil}"))?;
        let Some(prefab) = &m.prefab else { return Ok(Vec::new()) };
        let modifiers: Vec<&str> =
            [&b.material_modifier, &b.material_modifier2].into_iter().flatten().map(|s| s.as_str()).collect();
        self.prefab_surfaces(prefab, &modifiers)
    }

    pub fn prefab_surfaces(&self, prefab: &str, modifiers: &[&str]) -> Result<Vec<PlacedSurface<'_>>> {
        let mods: Vec<ModifierRef> = modifiers
            .iter()
            .map(|&path| ModifierRef { path, folder: None, keep_track_wall: false })
            .collect();
        self.prefab_surfaces_with(prefab, &mods)
    }

    pub fn prefab_surfaces_with(
        &self,
        prefab: &str,
        mods: &[ModifierRef<'_>],
    ) -> Result<Vec<PlacedSurface<'_>>> {
        let mut out = Vec::new();
        self.walk(prefab, &mut Vec::new(), mods, &mut out, 0)?;
        Ok(out)
    }

    pub fn decoration(&self, name: &str, env: &str) -> Option<&crate::catalog::Decoration> {
        let own = format!("{env}\\");
        self.index
            .decorations
            .iter()
            .filter(|(p, d)| {
                d.name.eq_ignore_ascii_case(name) && (p.starts_with(&own) || p.starts_with("Stadium\\"))
            })
            .min_by_key(|(p, _)| (!p.starts_with(&own), p.as_str()))
            .map(|(_, d)| d)
    }

    pub fn find_anchored_item(&self, item: &gearbox_gbx::map::Item, env: &str) -> Option<&str> {
        let name = item.model.as_deref()?;
        self.index.items.get_key_value(name).map(|(path, _)| path.as_str()).or_else(|| {
            let env = crate::packs::environment(item.collection.as_deref()).unwrap_or(env);
            self.find_item(name, env)
        })
    }

    pub fn find_item(&self, name: &str, env: &str) -> Option<&str> {
        let file = format!("\\{}.item.gbx", name.to_ascii_lowercase());
        let own = format!("{env}\\Items\\");
        self.index
            .items
            .keys()
            .filter(|p| {
                p.to_ascii_lowercase().ends_with(&file)
                    && (p.starts_with(&own) || p.starts_with("Stadium\\Items\\"))
            })
            .min_by_key(|p| (!p.starts_with(&own), p.as_str()))
            .map(|p| p.as_str())
    }

    pub fn item_model(&self, path: &str) -> Result<Vec<PlacedSurface<'_>>> {
        self.item_model_variant(path, 0)
    }

    pub fn item_model_variant(&self, path: &str, variant: usize) -> Result<Vec<PlacedSurface<'_>>> {
        let it = self.index.items.get(path).with_context(|| format!("unknown item {path}"))?;
        let mods: Vec<ModifierRef> = it
            .material_modifier
            .iter()
            .map(|s| ModifierRef { path: s, folder: None, keep_track_wall: false })
            .collect();
        let mods = mods.as_slice();
        let mut out = Vec::new();
        for m in it.models(variant) {
            match m {
                EntModel::Other { class_id: Some(0x2f086000), path: source } => {
                    bail!("{path}: vegetation collision model not decoded ({source:?})")
                }
                EntModel::Other { class_id: Some(0x09144000), path: source } => {
                    bail!("{path}: dynamic collision model not decoded ({source:?})")
                }
                EntModel::Prefab(x) => self.walk(x, &mut Vec::new(), mods, &mut out, 0)?,
                EntModel::Static { shape: None, mesh_collidable: true, .. } => {
                    bail!("{path}: mesh collision not decoded")
                }
                EntModel::Static { shape: Some(id), .. } | EntModel::Surface(id) => {
                    out.push(self.place(*id, Role::Collision, &[], mods))
                }
                EntModel::Dyna { dyna_shape, static_shape, .. } => {
                    if let Some(id) = dyna_shape {
                        out.push(self.place(*id, Role::DynaShape, &[], mods));
                    }
                    if let Some(id) = static_shape {
                        out.push(self.place(*id, Role::DynaStaticShape, &[], mods));
                    }
                }
                EntModel::BlockTrigger { waypoint, shape } => {
                    out.push(self.place(*shape, Role::BlockTrigger(*waypoint), &[], mods))
                }
                EntModel::Trigger { waypoint, shape: Some(id), loc } => {
                    let mut ps = self.place(
                        *id,
                        waypoint.map(Role::Waypoint).unwrap_or(Role::SpecialTrigger),
                        &[],
                        mods,
                    );
                    ps.vertex_loc = *loc;
                    out.push(ps);
                }
                _ => {}
            }
        }
        Ok(out)
    }

    fn walk<'a>(
        &'a self,
        prefab: &str,
        chain: &mut Vec<Loc>,
        mods: &[ModifierRef<'_>],
        out: &mut Vec<PlacedSurface<'a>>,
        depth: usize,
    ) -> Result<()> {
        if depth > 16 {
            bail!("prefab nesting too deep at {prefab}");
        }
        let p = self.index.prefabs.get(prefab).with_context(|| format!("prefab {prefab} not extracted"))?;
        for e in &p.ents {
            let loc = Loc { rotation: e.rotation, position: e.position };
            chain.push(loc);
            match &e.model {
                EntModel::Other { class_id: Some(0x2f086000), path: source } => {
                    bail!("{prefab}: vegetation collision model not decoded ({source:?})")
                }
                EntModel::Other { class_id: Some(0x09144000), path: source } => {
                    bail!("{prefab}: dynamic collision model not decoded ({source:?})")
                }
                EntModel::Prefab(x) => self.walk(x, chain, mods, out, depth + 1)?,
                EntModel::Static { shape: None, mesh_collidable: true, .. } => {
                    bail!("{prefab}: mesh collision not decoded")
                }
                EntModel::Static { shape: Some(id), .. } | EntModel::Surface(id) => {
                    out.push(self.place(*id, Role::Collision, chain, mods))
                }
                EntModel::Dyna { dyna_shape, static_shape, .. } => {
                    if let Some(id) = dyna_shape {
                        out.push(self.place(*id, Role::DynaShape, chain, mods));
                    }
                    if let Some(id) = static_shape {
                        out.push(self.place(*id, Role::DynaStaticShape, chain, mods));
                    }
                }
                EntModel::BlockTrigger { waypoint, shape } => {
                    out.push(self.place(*shape, Role::BlockTrigger(*waypoint), chain, mods))
                }
                EntModel::Trigger { waypoint, shape: Some(id), loc } => {
                    let role = waypoint.map(Role::Waypoint).unwrap_or(Role::SpecialTrigger);
                    let mut ps = self.place(*id, role, chain, mods);
                    ps.vertex_loc = *loc;
                    out.push(ps);
                }
                _ => {}
            }
            chain.pop();
        }
        Ok(())
    }

    pub fn place(&self, id: usize, role: Role, chain: &[Loc], mods: &[ModifierRef<'_>]) -> PlacedSurface<'_> {
        let loc = chain.iter().fold(Loc::IDENTITY, |a, b| a.then(b));
        let info = &self.index.surfaces[id];
        let mut modified = false;
        let materials = info
            .materials
            .iter()
            .map(|m| {
                let m = m.as_ref()?;
                let mut path = m.clone();
                for md in mods {
                    if md.keep_track_wall && path.eq_ignore_ascii_case(TRACK_WALL) {
                        continue;
                    }
                    if let Some(mo) = self.index.modifiers.get(md.path)
                        && let Some(r) =
                            crate::material::replacement(&path, &mo.folder, md.folder, &mo.skinned)
                        && self.index.materials.contains_key(&r)
                    {
                        path = r;
                        modified = true;
                    }
                }
                let ids = self.material_ids(&path)?;
                Some((path, ids))
            })
            .collect();
        PlacedSurface {
            id,
            role,
            surface: &self.surfaces[id],
            loc,
            chain: chain.to_vec(),
            materials,
            modified,
            vertex_loc: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Loc;

    fn yaw90() -> Loc {
        let h = std::f32::consts::FRAC_1_SQRT_2;
        Loc { rotation: [0.0, h, 0.0, h], position: [0.0; 3] }
    }

    fn apply(m: &[f32; 9], v: [f32; 3]) -> [f32; 3] {
        [
            m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
        ]
    }

    #[test]
    fn quarter_turn_about_y_maps_x_to_minus_z() {
        let r = apply(&yaw90().matrix(), [1.0, 0.0, 0.0]);
        assert!(r[0].abs() < 1e-6 && r[1].abs() < 1e-6 && (r[2] + 1.0).abs() < 1e-6, "{r:?}");
    }

    #[test]
    fn composition_rotates_the_child_offset() {
        let parent = Loc { position: [10.0, 0.0, 0.0], ..yaw90() };
        let child = Loc { rotation: [0.0, 0.0, 0.0, 1.0], position: [1.0, 2.0, 0.0] };
        let c = parent.then(&child);
        assert!(
            (c.position[0] - 10.0).abs() < 1e-5
                && (c.position[1] - 2.0).abs() < 1e-5
                && (c.position[2] + 1.0).abs() < 1e-5,
            "{:?}",
            c.position
        );
        let twice = yaw90().then(&yaw90()).matrix();
        assert!((twice[0] + 1.0).abs() < 1e-5 && (twice[8] + 1.0).abs() < 1e-5);
    }
}
