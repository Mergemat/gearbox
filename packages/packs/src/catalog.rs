use std::collections::{BTreeMap, VecDeque};
use std::hash::{Hash, Hasher};
use std::path::Path;

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};

use crate::blockinfo::VariantKind;
use crate::material::MaterialIds;
use crate::packs::PackSet;
use crate::prefab::{Ctx, Model, NodeLink, StaticObject};
use crate::surface::{SurfMaterial, Surface};

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Block {
    #[serde(default)]
    pub name: Option<String>,
    pub path: String,
    pub class_id: u32,
    pub variants: Vec<Variant>,
    pub material_modifier: Option<String>,
    pub material_modifier2: Option<String>,
    #[serde(default)]
    pub clip_type: u32,
    #[serde(default)]
    pub symmetrical_clip: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Variant {
    pub ground: bool,
    pub index: usize,
    pub mobils: Vec<Vec<Mobil>>,
    #[serde(default)]
    pub units: Vec<[i32; 3]>,
    #[serde(default)]
    pub unit_clips: Vec<Vec<String>>,
    #[serde(default)]
    pub waypoint_trigger: Option<String>,
    #[serde(default)]
    pub waypoint_trigger_surface: Option<usize>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Mobil {
    pub prefab: Option<String>,
    pub geom_translation: Option<[f32; 3]>,
    pub geom_rotation: Option<[f32; 3]>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum EntModel {
    None,
    Prefab(String),
    Static {
        shape: Option<usize>,
        mesh_collidable: bool,
        source: Option<String>,
    },
    Surface(usize),
    Trigger {
        waypoint: Option<u32>,
        shape: Option<usize>,
        #[serde(default)]
        loc: Option<[f32; 12]>,
    },
    BlockTrigger {
        waypoint: Option<u32>,
        shape: usize,
    },
    Dyna {
        dyna_shape: Option<usize>,
        static_shape: Option<usize>,
        is_static: bool,
        source: Option<String>,
    },
    Other {
        class_id: Option<u32>,
        path: Option<String>,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Ent {
    pub model: EntModel,
    pub rotation: [f32; 4],
    pub position: [f32; 3],
    pub params_class: Option<u32>,
    pub name: String,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
pub struct Prefab {
    pub ents: Vec<Ent>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SurfaceInfo {
    pub source: String,
    pub materials: Vec<Option<String>>,
    #[serde(default)]
    pub extra_refs: u32,
    #[serde(default)]
    pub tree: Option<crate::veget::Params>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Modifier {
    pub skin: Option<String>,
    pub folder: String,
    pub skinned: Vec<(String, String)>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
pub struct Index {
    pub blocks: BTreeMap<String, Block>,
    #[serde(skip)]
    pub custom_blocks: BTreeMap<String, Block>,
    pub prefabs: BTreeMap<String, Prefab>,
    pub surfaces: Vec<SurfaceInfo>,
    pub materials: BTreeMap<String, (u8, u8)>,
    pub modifiers: BTreeMap<String, Modifier>,
    #[serde(default)]
    pub items: BTreeMap<String, Item>,
    #[serde(default)]
    pub decorations: BTreeMap<String, Decoration>,
    #[serde(default)]
    pub collection_modifiers: BTreeMap<String, String>,
    pub errors: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
pub struct Decoration {
    pub name: String,
    pub map: Option<String>,
    pub collection: Option<String>,
    pub size: [i32; 3],
    pub blocks: Vec<(String, u8, [i32; 3], u32)>,
    #[serde(default)]
    pub level0: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
pub struct Item {
    pub models: Vec<EntModel>,
    #[serde(default)]
    pub variants: Vec<Vec<EntModel>>,
    #[serde(default)]
    pub material_modifier: Option<String>,
}

#[derive(Serialize, Deserialize)]
pub struct Catalog {
    pub index: Index,
    pub surfaces: Vec<Surface>,
}

fn norm(p: &str) -> String {
    p.replace('/', "\\").to_lowercase()
}

pub trait Source {
    fn read_path(&self, path: &str) -> Result<Vec<u8>>;
    fn class_of(&self, path: &str) -> Option<u32>;
}

impl Source for PackSet {
    fn read_path(&self, path: &str) -> Result<Vec<u8>> {
        PackSet::read_path(self, path)
    }
    fn class_of(&self, path: &str) -> Option<u32> {
        self.find(path).map(|r| self.paks[r.pak].files[r.file].class_id)
    }
}

struct Crawler<'a> {
    set: &'a dyn Source,
    cat: Catalog,
    prefab_queue: VecDeque<String>,
    seen: std::collections::HashSet<String>,
    shape_files: BTreeMap<String, Option<usize>>,
    static_files: BTreeMap<String, EntModel>,
    id_base: usize,
}

impl Crawler<'_> {
    fn custom_block(&mut self, key: &str, path: &str, known: &Index) -> Result<bool> {
        let (buf, l) = self.read(path)?;
        let Some(n) = l.nodes.iter().find(|n| n.class_id == 0x2e025000) else { return Ok(false) };
        let ctx = Ctx { buf: &buf, layout: &l, path };
        let item = crate::custom_block::read(&ctx, n.start)?;
        anyhow::ensure!(matches!(item.collection, 25 | 26), "custom block collection {}", item.collection);
        let (_, archetype) = known
            .blocks
            .iter()
            .filter(|(p, _)| {
                crate::assets::block_name(p).eq_ignore_ascii_case(&item.archetype)
                    && p.starts_with("Stadium\\")
            })
            .min_by_key(|(p, _)| (p.contains("\\Deprecated\\"), *p))
            .ok_or_else(|| anyhow::anyhow!("custom block archetype {} not extracted", item.archetype))?;
        let mut block = archetype.clone();
        let (archetype_buf, archetype_layout) = self.read(&archetype.path)?;
        let original = crate::blockinfo::read_block_info(&archetype_buf, &archetype_layout, &archetype.path)?;
        let customized: std::collections::HashSet<_> =
            item.mobils.iter().filter(|m| m.crystal.is_some()).map(|m| m.id).collect();
        for mobil in item.mobils {
            let (ground, index, group, sub) = crate::custom_block::mobil_key(mobil.id);
            let Some(v) = block.variants.iter_mut().find(|v| v.ground == ground && v.index == index) else {
                continue;
            };
            if v.mobils.get(group).and_then(|g| g.get(sub)).is_none() {
                continue;
            }
            let mut trigger = None;
            let mut replace_trigger = mobil.trigger.is_some();
            let model = if let Some(object) = mobil.object {
                Some(match object {
                    NodeLink::Null => EntModel::None,
                    NodeLink::Inline(so) => {
                        self.static_object(so, path, &l.head, format!("{path}#mobil{}", mobil.id))
                    }
                    NodeLink::External(p) => self.static_file(&p),
                    NodeLink::Earlier(_) => bail!("shared custom mobil static object"),
                })
            } else if let Some(start) = mobil.crystal.filter(|_| !item.compiled) {
                let crystal = crate::crystal::read(&ctx, start, |p| self.crystal_material(p))?;
                if !crystal.triggers.is_empty() {
                    replace_trigger = true;
                    let s = crate::crystal::trigger(&crystal)?;
                    trigger = s
                        .surf
                        .is_some()
                        .then(|| self.add_surface(s, format!("{path}#trigger{}", mobil.id), &l.head, path));
                }
                let s = crate::crystal::collision(&crystal)?;
                Some(if s.surf.is_none() {
                    EntModel::None
                } else {
                    EntModel::Static {
                        shape: Some(self.add_surface(s, format!("{path}#mobil{}", mobil.id), &l.head, path)),
                        mesh_collidable: true,
                        source: None,
                    }
                })
            } else {
                None
            };
            if let Some(t) = mobil.trigger {
                trigger = match t {
                    NodeLink::Inline(s) => {
                        Some(self.add_surface(s, format!("{path}#trigger{}", mobil.id), &l.head, path))
                    }
                    NodeLink::External(p) => self.shape_file(&p),
                    NodeLink::Null => None,
                    NodeLink::Earlier(i) => Some(self.add_surface(
                        self.inline_surface(path, i)?,
                        format!("{path}#trigger{}", mobil.id),
                        &l.head,
                        path,
                    )),
                };
            }
            if let Some(model) = model {
                let prefab = format!("{path}#mobil{}", mobil.id);
                let ent = |model| Ent {
                    model,
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    position: [0.0; 3],
                    params_class: None,
                    name: String::new(),
                };
                let mut ents = vec![ent(model)];
                if let Some(shape) = trigger {
                    ents.push(ent(EntModel::BlockTrigger { waypoint: None, shape }));
                }
                self.cat.index.prefabs.insert(prefab.clone(), Prefab { ents });
                let m = &mut v.mobils[group][sub];
                m.prefab = Some(prefab);
                m.geom_translation = None;
                m.geom_rotation = None;
                let ov = original
                    .variants
                    .iter()
                    .find(|ov| (ov.kind == VariantKind::Ground) == ground && ov.index == index)
                    .context("custom archetype variant")?;
                let og = &ov.mobils[group];
                if og[sub].solid_frequency == 0
                    && let Some(companion) =
                        (1..og.len()).map(|i| (sub + i) % og.len()).find(|&i| og[i].solid_frequency != 0)
                {
                    let id = mobil.id - sub as u32 + companion as u32;
                    if !customized.contains(&id) {
                        v.mobils[group][companion] = v.mobils[group][sub].clone();
                    }
                }
            }
            if (v.waypoint_trigger.is_some() || v.waypoint_trigger_surface.is_some()) && replace_trigger {
                v.waypoint_trigger_surface = trigger;
                v.waypoint_trigger = None;
            }
        }
        self.cat.index.custom_blocks.insert(format!("{key}_CustomBlock"), block);
        Ok(true)
    }
    fn read(&self, path: &str) -> Result<(Vec<u8>, crate::nodes::Layout)> {
        let buf = self.set.read_path(path)?;
        let l = crate::nodes::layout(&buf)?;
        Ok((buf, l))
    }

    fn add_surface(&mut self, s: Surface, source: String, head: &crate::unpack::GbxHead, own: &str) -> usize {
        let materials = s
            .materials
            .iter()
            .map(|m| match m {
                SurfMaterial::Material(i) => head.ref_path(own, *i),
                SurfMaterial::SurfaceId(_) => None,
            })
            .collect();
        self.cat.surfaces.push(s);
        self.cat.index.surfaces.push(SurfaceInfo { source, materials, extra_refs: 0, tree: None });
        self.id_base + self.cat.surfaces.len() - 1
    }

    fn shape_file(&mut self, path: &str) -> Option<usize> {
        if let Some(&id) = self.shape_files.get(&norm(path)) {
            return id;
        }
        let r = (|| -> Result<usize> {
            let (buf, l) = self.read(path)?;
            if l.head.class_id != 0x0900c000 {
                bail!("shape file of class {:#010x}", l.head.class_id);
            }
            let ctx = Ctx { buf: &buf, layout: &l, path };
            let mut rd = gearbox_gbx::reader::Reader::new(&buf);
            rd.skip(l.head.body_start)?;
            let s = ctx.surface(&mut rd)?;
            Ok(self.add_surface(s, path.to_string(), &l.head, path))
        })();
        let id = match r {
            Ok(id) => Some(id),
            Err(e) => {
                self.cat.index.errors.push(format!("{path}: {e}"));
                None
            }
        };
        self.shape_files.insert(norm(path), id);
        id
    }

    fn static_object(
        &mut self,
        so: StaticObject,
        owner: &str,
        head: &crate::unpack::GbxHead,
        source: String,
    ) -> EntModel {
        if so.mesh_collidable && so.mesh == NodeLink::Null {
            return EntModel::None;
        }
        let shape = match so.shape {
            NodeLink::Inline(s) => Some(self.add_surface(s, source, head, owner)),
            NodeLink::External(p) => self.shape_file(&p),
            NodeLink::Earlier(index) => {
                let result = self.inline_surface(owner, index);
                match result {
                    Ok(s) => Some(self.add_surface(s, source, head, owner)),
                    Err(e) => {
                        self.cat.index.errors.push(format!("{owner}: {e:#}"));
                        None
                    }
                }
            }
            NodeLink::Null if so.mesh_collidable => match self.mesh_collision(owner, so.mesh) {
                Ok(s) if s.surf.is_some() => Some(self.add_surface(s, source, head, owner)),
                Ok(_) => return EntModel::None,
                Err(e) => {
                    self.cat.index.errors.push(format!("{owner}: mesh collision: {e:#}"));
                    None
                }
            },
            NodeLink::Null => return EntModel::None,
        };
        EntModel::Static { shape, mesh_collidable: so.mesh_collidable, source: None }
    }

    fn inline_surface(&self, path: &str, index: u32) -> Result<Surface> {
        let (buf, l) = self.read(path)?;
        let n = l
            .nodes
            .iter()
            .find(|n| n.index == index)
            .ok_or_else(|| anyhow::anyhow!("missing shape node {index}"))?;
        anyhow::ensure!(n.class_id == 0x0900c000, "shared shape class {:#x}", n.class_id);
        let ctx = Ctx { buf: &buf, layout: &l, path };
        let mut r = gearbox_gbx::reader::Reader::new(&buf);
        r.skip(n.start)?;
        ctx.surface(&mut r)
    }

    fn mesh_collision(&self, owner: &str, mesh: NodeLink<usize>) -> Result<Surface> {
        let path = match &mesh {
            NodeLink::External(p) => p.as_str(),
            _ => owner,
        };
        let (buf, l) = self.read(path)?;
        let start = match mesh {
            NodeLink::Inline(start) => start,
            NodeLink::External(_) => l.head.body_start,
            NodeLink::Earlier(index) => {
                l.nodes
                    .iter()
                    .find(|n| n.index == index)
                    .ok_or_else(|| anyhow::anyhow!("missing mesh node {index}"))?
                    .start
            }
            NodeLink::Null => bail!("collidable static object has no mesh"),
        };
        let ctx = Ctx { buf: &buf, layout: &l, path };
        crate::solid::collision(&ctx, start)
    }

    fn dyna(
        &mut self,
        d: crate::prefab::DynaObject,
        owner: &str,
        head: &crate::unpack::GbxHead,
        source: String,
        file: Option<String>,
    ) -> EntModel {
        let shape = |c: &mut Self, r: NodeLink<Surface>, tag: &str| match r {
            NodeLink::Inline(s) => Some(c.add_surface(s, format!("{source}#{tag}"), head, owner)),
            NodeLink::External(p) => c.shape_file(&p),
            _ => None,
        };
        let dyna_shape = shape(self, d.dyna_shape, "dyna");
        let static_shape = shape(self, d.static_shape, "static");
        EntModel::Dyna { dyna_shape, static_shape, is_static: d.is_static, source: file }
    }

    fn dyna_file(&mut self, path: &str) -> EntModel {
        let r = (|| -> Result<(crate::prefab::DynaObject, crate::unpack::GbxHead)> {
            let (buf, l) = self.read(path)?;
            if l.head.class_id != 0x09144000 {
                bail!("dyna object file of class {:#010x}", l.head.class_id);
            }
            let ctx = Ctx { buf: &buf, layout: &l, path };
            let mut rd = gearbox_gbx::reader::Reader::new(&buf);
            rd.skip(l.head.body_start)?;
            Ok((ctx.dyna_object(&mut rd)?, l.head))
        })();
        match r {
            Ok((d, head)) => self.dyna(d, path, &head, path.to_string(), Some(path.to_string())),
            Err(e) => {
                self.cat.index.errors.push(format!("{path}: {e}"));
                EntModel::Other { class_id: Some(0x09144000), path: Some(path.to_string()) }
            }
        }
    }

    fn static_file(&mut self, path: &str) -> EntModel {
        if let Some(model) = self.static_files.get(&norm(path)) {
            return model.clone();
        }
        let r = (|| -> Result<(StaticObject, crate::unpack::GbxHead)> {
            let (buf, l) = self.read(path)?;
            if l.head.class_id != 0x09159000 {
                bail!("static object file of class {:#010x}", l.head.class_id);
            }
            let ctx = Ctx { buf: &buf, layout: &l, path };
            let mut rd = gearbox_gbx::reader::Reader::new(&buf);
            rd.skip(l.head.body_start)?;
            Ok((ctx.static_object(&mut rd)?, l.head))
        })();
        let mut model = match r {
            Ok((so, head)) => self.static_object(so, path, &head, format!("{path}#shape")),
            Err(e) => {
                self.cat.index.errors.push(format!("{path}: {e}"));
                EntModel::Other { class_id: Some(0x09159000), path: Some(path.to_string()) }
            }
        };
        if let EntModel::Static { source, .. } = &mut model {
            *source = Some(path.to_string());
        }
        self.static_files.insert(norm(path), model.clone());
        model
    }

    fn veget_file(&mut self, path: &str) -> EntModel {
        if let Some(&id) = self.shape_files.get(&norm(path)) {
            return id.map(EntModel::Surface).unwrap_or(EntModel::None);
        }
        let result = (|| -> Result<Option<usize>> {
            let (buf, l) = self.read(path)?;
            let ctx = Ctx { buf: &buf, layout: &l, path };
            let (surface, params) = crate::veget::collision(&ctx)?;
            Ok(surface.map(|s| {
                let id = self.add_surface(s, path.to_string(), &l.head, path);
                self.cat.index.surfaces[id - self.id_base].extra_refs = 1;
                self.cat.index.surfaces[id - self.id_base].tree = Some(params);
                id
            }))
        })();
        let id = match result {
            Ok(id) => id,
            Err(e) => {
                self.cat.index.errors.push(format!("{path}: {e}"));
                None
            }
        };
        self.shape_files.insert(norm(path), id);
        id.map(EntModel::Surface).unwrap_or(EntModel::None)
    }

    fn prefab(&mut self, path: &str) -> Result<()> {
        let (buf, l) = self.read(path)?;
        let p = crate::prefab::read_prefab(&buf, &l, path)?;
        self.store_prefab(path, p, path, &l.head);
        Ok(())
    }

    fn store_prefab(
        &mut self,
        name: &str,
        p: crate::prefab::Prefab,
        owner: &str,
        head: &crate::unpack::GbxHead,
    ) -> EntModel {
        let path = owner;
        let mut empty = true;
        let mut ents = Vec::with_capacity(p.ents.len());
        for (k, e) in p.ents.into_iter().enumerate() {
            let shared = matches!(e.model, NodeLink::Earlier(_));
            let model = match e.model {
                NodeLink::Null | NodeLink::Earlier(_) => EntModel::None,
                NodeLink::External(x) => self.external_model(&x),
                NodeLink::Inline(Model::Prefab(p)) => {
                    let nested = format!("{name}#ent{k}#prefab");
                    self.store_prefab(&nested, *p, owner, head)
                }
                NodeLink::Inline(Model::StaticObject(so)) => {
                    self.static_object(so, path, head, format!("{name}#ent{k}"))
                }
                NodeLink::Inline(Model::Surface(s)) => {
                    EntModel::Surface(self.add_surface(s, format!("{name}#ent{k}"), head, path))
                }
                NodeLink::Inline(Model::DynaObject(d)) => {
                    self.dyna(d, path, head, format!("{name}#ent{k}"), None)
                }
                NodeLink::Inline(Model::Trigger { waypoint, shape }) => {
                    let shape = match shape {
                        NodeLink::Inline(s) => {
                            Some(self.add_surface(s, format!("{name}#ent{k}#trigger"), head, path))
                        }
                        NodeLink::External(p) => self.shape_file(&p),
                        _ => None,
                    };
                    EntModel::Trigger { waypoint, shape, loc: None }
                }
                NodeLink::Inline(Model::Kinematic(_)) => {
                    EntModel::Other { class_id: Some(0x2f0ca000), path: None }
                }
                NodeLink::Inline(Model::Other(c)) => EntModel::Other { class_id: Some(c), path: None },
            };
            empty &= !shared && matches!(model, EntModel::None);
            ents.push(Ent {
                model,
                rotation: e.rotation,
                position: e.position,
                params_class: e.params.map(|p| p.0),
                name: e.name,
            });
        }
        self.cat.index.prefabs.insert(name.to_string(), Prefab { ents });
        if empty { EntModel::None } else { EntModel::Prefab(name.to_string()) }
    }

    fn item(&mut self, path: &str) -> Result<Item> {
        let (buf, l) = self.read(path)?;
        let ctx = Ctx { buf: &buf, layout: &l, path };
        let mut models = Vec::new();
        if let Some(n) = l.nodes.iter().find(|n| n.class_id == 0x2e026000) {
            let mut r = crate::crystal::cursor(&buf, n.start)?;
            anyhow::ensure!(r.u32()? == 0x2e026000, "item edition chunk");
            anyhow::ensure!(r.u32()? == 7, "item edition version");
            anyhow::ensure!(r.u32()? == 1, "non-ornament crystal item");
            let start = ctx.noderef(&mut r, |_, r, class| {
                anyhow::ensure!(class == 0x09003000, "edition crystal class {class:#x}");
                Ok(r.pos())
            })?;
            let NodeLink::Inline(start) = start else {
                bail!("non-inline edition crystal");
            };
            let crystal = crate::crystal::read(&ctx, start, |p| self.crystal_material(p))?;
            let s = crate::crystal::collision(&crystal)?;
            let model = if s.surf.is_none() {
                EntModel::None
            } else {
                EntModel::Static {
                    shape: Some(self.add_surface(s, format!("{path}#crystal"), &l.head, path)),
                    mesh_collidable: true,
                    source: None,
                }
            };
            let mut models = vec![model];
            if !crystal.triggers.is_empty() {
                let s = crate::crystal::trigger(&crystal)?;
                if s.surf.is_some() {
                    let shape = self.add_surface(s, format!("{path}#trigger"), &l.head, path);
                    models.push(EntModel::Trigger {
                        waypoint: Some(u32::MAX),
                        shape: Some(shape),
                        loc: None,
                    });
                }
            }
            return Ok(Item { models, ..Default::default() });
        }
        if let Some(n) = l.nodes.iter().find(|n| n.class_id == 0x2f0bc000) {
            let mut r = gearbox_gbx::reader::Reader::new(&buf);
            r.skip(n.start)?;
            let version = r.u32()?;
            anyhow::ensure!(version <= 1, "item variant list version {version}");
            let count = r.u32()?;
            anyhow::ensure!(count < 1 << 16, "item variant count {count}");
            let mut variants = Vec::new();
            for _ in 0..count {
                let tags = r.u32()?;
                anyhow::ensure!(tags < 1 << 12, "item variant tag count {tags}");
                for _ in 0..tags {
                    r.string()?;
                    r.string()?;
                }
                let m = ctx.noderef(&mut r, |c, r, class| c.model(r, class))?;
                let model = match m {
                    NodeLink::Null => EntModel::None,
                    NodeLink::External(x) => self.external_model(&x),
                    NodeLink::Inline(Model::StaticObject(so)) => {
                        self.static_object(so, path, &l.head, format!("{path}#variant{}", variants.len()))
                    }
                    NodeLink::Inline(Model::Surface(s)) => EntModel::Surface(self.add_surface(
                        s,
                        format!("{path}#variant{}", variants.len()),
                        &l.head,
                        path,
                    )),
                    NodeLink::Inline(Model::Prefab(prefab)) => {
                        let name = format!("{path}#variant{}", variants.len());
                        self.store_prefab(&name, *prefab, path, &l.head)
                    }
                    _ => bail!("unsupported inline item variant"),
                };
                if version >= 1 {
                    r.u32()?;
                }
                variants.push(vec![model]);
            }
            return Ok(Item { variants, ..Default::default() });
        }
        for n in l.nodes.iter().filter(|n| n.class_id == 0x2e027000) {
            let w = 0x2e027000u32.to_le_bytes();
            if buf.get(n.start..n.start + 4) != Some(&w[..]) {
                continue;
            }
            let mut r = gearbox_gbx::reader::Reader::new(&buf);
            r.skip(n.start + 4)?;
            if r.u32()? < 4 {
                continue;
            }
            let so = ctx.noderef(&mut r, |c, r, class| {
                anyhow::ensure!(class == 0x09159000, "entity static object of class {class:#010x}");
                c.static_object(r)
            })?;
            match so {
                NodeLink::Inline(so) => {
                    let m = self.static_object(so, path, &l.head, format!("{path}#entity"));
                    models.push(m);
                }
                NodeLink::External(x) => models.push(self.static_file(&x)),
                _ => {}
            }
            let trigger = ctx.noderef(&mut r, |c, r, class| {
                anyhow::ensure!(class == 0x0900c000, "entity trigger shape of class {class:#010x}");
                c.surface(r)
            })?;
            let shape = match trigger {
                NodeLink::Inline(s) => Some(self.add_surface(s, format!("{path}#trigger"), &l.head, path)),
                NodeLink::External(x) => self.shape_file(&x),
                _ => None,
            };
            if let Some(shape) = shape {
                let mut loc = [0f32; 12];
                for x in &mut loc {
                    *x = r.f32()?;
                }
                models.push(EntModel::Trigger {
                    waypoint: Some(u32::MAX),
                    shape: Some(shape),
                    loc: Some(loc),
                });
            }
        }
        let mut prefab_end = 0;
        for n in l.nodes.iter().filter(|n| n.class_id == 0x09145000 && n.index != 0) {
            if n.start < prefab_end {
                continue;
            }
            let mut r = gearbox_gbx::reader::Reader::new(&buf);
            r.skip(n.start)?;
            let p = ctx.prefab(&mut r)?;
            prefab_end = r.pos();
            let name = format!("{path}#prefab{}", n.index);
            models.push(self.store_prefab(&name, p, path, &l.head));
        }
        let mut refs: Vec<String> =
            l.head.ref_files.keys().filter_map(|&i| l.head.ref_path(path, i)).collect();
        refs.sort();
        let mut material_modifier = None;
        for x in refs {
            let lx = x.to_lowercase();
            if lx.ends_with(".prefab.gbx") {
                self.enqueue(&x);
                models.push(EntModel::Prefab(x));
            } else if lx.ends_with(".staticobject.gbx") {
                models.push(self.static_file(&x));
            } else if lx.ends_with(".dynaobject.gbx") || self.is_dyna_file(&x) {
                models.push(self.dyna_file(&x));
            } else if lx.ends_with(".shape.gbx") {
                if let Some(id) = self.shape_file(&x) {
                    models.push(EntModel::Surface(id));
                }
            } else if self.set.class_of(&x) == Some(0x2f086000) {
                models.push(self.veget_file(&x));
            } else if lx.contains("\\modifier\\") && !lx.ends_with(".placeparam.gbx") {
                self.modifier(&x);
                material_modifier = Some(x);
            }
        }
        Ok(Item { models, material_modifier, variants: Vec::new() })
    }

    fn external_model(&mut self, path: &str) -> EntModel {
        let lower = path.to_lowercase();
        if lower.ends_with(".prefab.gbx") {
            self.enqueue(path);
            EntModel::Prefab(path.to_string())
        } else if self.set.class_of(path) == Some(0x2f086000) {
            self.veget_file(path)
        } else if lower.ends_with(".staticobject.gbx") {
            self.static_file(path)
        } else if lower.ends_with(".dynaobject.gbx") || self.is_dyna_file(path) {
            self.dyna_file(path)
        } else if lower.ends_with(".shape.gbx") {
            self.shape_file(path)
                .map(EntModel::Surface)
                .unwrap_or_else(|| EntModel::Other { class_id: None, path: Some(path.to_string()) })
        } else {
            EntModel::Other { class_id: self.set.class_of(path), path: Some(path.to_string()) }
        }
    }

    fn is_dyna_file(&self, path: &str) -> bool {
        self.set.class_of(path) == Some(0x09144000)
    }

    fn enqueue(&mut self, path: &str) {
        if self.seen.insert(norm(path)) {
            self.prefab_queue.push_back(path.to_string());
        }
    }

    fn crystal_material(&mut self, path: &str) -> Result<[u8; 2]> {
        let key = norm(path);
        if let Some((_, &(surface, gameplay))) = self.cat.index.materials.iter().find(|(p, _)| norm(p) == key)
        {
            return Ok([surface, gameplay]);
        }
        let buf = self.set.read_path(path)?;
        let head = crate::unpack::parse_head(&buf)
            .ok_or_else(|| anyhow::anyhow!("truncated crystal material {path}"))??;
        let ids = match head.class_id {
            0x09079000 => crate::material::material_ids(&buf)?,
            0x090fd000 => crate::material::user_material_ids(&buf, head.body_start)?,
            class => bail!("crystal material {path} class {class:#x}"),
        };
        self.cat.index.materials.insert(path.to_owned(), (ids.surface, ids.gameplay));
        Ok([ids.surface, ids.gameplay])
    }

    fn material(&mut self, path: &str) {
        if self.cat.index.materials.contains_key(path) {
            return;
        }
        match self.set.read_path(path).and_then(|b| crate::material::material_ids(&b)) {
            Ok(MaterialIds { surface, gameplay }) => {
                self.cat.index.materials.insert(path.to_string(), (surface, gameplay));
            }
            Err(e) => self.cat.index.errors.push(format!("{path}: {e}")),
        }
    }

    fn modifier(&mut self, path: &str) {
        if self.cat.index.modifiers.contains_key(path) || !self.seen.insert(format!("modifier:{path}")) {
            return;
        }
        let r = (|| -> Result<Modifier> {
            let m = crate::material::modifier(&self.set.read_path(path)?, path)?;
            let skinned = match &m.skin {
                Some(s) => crate::material::skin_materials(&self.set.read_path(s)?)?,
                None => m.inline_skinned.clone(),
            };
            Ok(Modifier { skin: m.skin, folder: m.folder, skinned })
        })();
        match r {
            Ok(m) => {
                let repl: Vec<String> = m
                    .skinned
                    .iter()
                    .filter_map(|(_, p)| crate::material::replacement(p, &m.folder, None, &m.skinned))
                    .collect();
                self.cat.index.modifiers.insert(path.to_string(), m);
                for r in repl {
                    if self.set.class_of(&r).is_some() {
                        self.material(&r);
                    }
                }
            }
            Err(e) => self.cat.index.errors.push(format!("{path}: {e}")),
        }
    }
}

fn decoration(set: &PackSet, path: &str) -> Result<Decoration> {
    let buf = set.read_path(path)?;
    let head = match crate::unpack::parse_head(&buf) {
        Some(h) => h?,
        None => bail!("truncated header"),
    };
    let stem = crate::assets::block_name(path);
    let id = buf[..head.body_start].windows(8).position(|w| w == [3, 0, 0, 0, 0, 0, 0, 0x40]).and_then(|p| {
        let n = u32::from_le_bytes(buf.get(p + 8..p + 12)?.try_into().ok()?) as usize;
        buf.get(p + 12..p + 12 + n).map(|s| String::from_utf8_lossy(s).into_owned())
    });
    let name = id.unwrap_or_else(|| stem.strip_prefix("Base").unwrap_or(stem).to_string());
    let refs: Vec<String> = head.ref_files.keys().filter_map(|&i| head.ref_path(path, i)).collect();
    let map = refs.iter().find(|p| p.to_lowercase().ends_with(".map.gbx")).cloned();
    let level0 = refs
        .iter()
        .find(|p| p.to_lowercase().ends_with(".decorationsize.gbx"))
        .and_then(|p| set.read_path(p).ok())
        .and_then(|b| level0(&b));
    let mut d = Decoration { name, map: map.clone(), level0, ..Default::default() };
    if let Some(m) = map {
        let parsed = gearbox_gbx::Map::parse(&set.read_path(&m)?)?;
        d.collection = parsed.collection.clone();
        d.size = parsed.size;
        d.blocks = parsed.blocks.iter().map(|b| (b.name.clone(), b.dir, b.coord, b.flags)).collect();
    }
    Ok(d)
}

fn level0(b: &[u8]) -> Option<u32> {
    let at = |p: usize| b.get(p..p + 4).map(|x| u32::from_le_bytes(x.try_into().unwrap()));
    let find = |id: u32| b.windows(4).position(|w| w == id.to_le_bytes());
    let p = find(0x0303b002)?;
    let (a, flag) = (at(p + 4)?, at(p + 20)?);
    let base = match find(0x0303b003) {
        Some(q) if b.get(q + 4..q + 8) == Some(b"PIKS") => at(q + 16)?,
        _ => 0,
    };
    Some(if flag != 0 { a + 1 + base } else { base })
}

pub fn crawl(set: &PackSet, paks: &[&str]) -> Result<Catalog> {
    let mut c = Crawler {
        set,
        cat: Catalog { index: Index::default(), surfaces: Vec::new() },
        prefab_queue: VecDeque::new(),
        seen: Default::default(),
        shape_files: Default::default(),
        static_files: Default::default(),
        id_base: 0,
    };
    let mut block_files = Vec::new();
    let mut item_files = Vec::new();
    let mut material_files = Vec::new();
    let mut decoration_files = Vec::new();
    let mut collection_files = Vec::new();
    for (r, f) in set.files() {
        let pname = set.paks[r.pak].path.file_name().unwrap().to_string_lossy().to_lowercase();
        if !paks.iter().any(|p| pname.contains(p)) {
            continue;
        }
        if crate::blockinfo::is_block_info(f.class_id) {
            block_files.push(set.paks[r.pak].file_path(r.file));
        }
        let fp = set.paks[r.pak].file_path(r.file);
        if f.class_id == 0x2e002000 && fp.to_lowercase().ends_with(".item.gbx") {
            item_files.push(fp);
        } else if f.class_id == 0x09079000 && fp.to_lowercase().ends_with(".material.gbx") {
            material_files.push(fp);
        } else if f.class_id == 0x03038000 && fp.to_lowercase().ends_with(".decoration.gbx") {
            decoration_files.push(fp);
        } else if f.class_id == 0x03033000 && fp.to_lowercase().ends_with(".collection.gbx") {
            collection_files.push(fp);
        }
    }
    for path in collection_files {
        let Ok(buf) = set.read_path(&path) else { continue };
        let Some(Ok(head)) = crate::unpack::parse_head(&buf) else { continue };
        let modifier = head
            .ref_files
            .keys()
            .filter_map(|&i| head.ref_path(&path, i))
            .find(|p| p.to_lowercase().ends_with(".terrainmodifier.gbx"));
        if let Some(m) = modifier {
            c.modifier(&m);
            c.cat
                .index
                .collection_modifiers
                .insert(crate::assets::block_name(&path).split('.').next().unwrap_or("").to_string(), m);
        }
    }
    for path in decoration_files {
        match decoration(set, &path) {
            Ok(d) => {
                c.cat.index.decorations.insert(path, d);
            }
            Err(e) => c.cat.index.errors.push(format!("{path}: {e}")),
        }
    }
    for path in block_files {
        let r = (|| -> Result<Block> {
            let (buf, l) = c.read(&path)?;
            let b = crate::blockinfo::read_block_info(&buf, &l, &path)?;
            Ok(Block {
                name: b.name,
                path: path.clone(),
                class_id: b.class_id,
                variants: b
                    .variants
                    .into_iter()
                    .map(|v| Variant {
                        ground: v.kind == VariantKind::Ground,
                        index: v.index,
                        mobils: v
                            .mobils
                            .into_iter()
                            .map(|g| {
                                g.into_iter()
                                    .map(|m| Mobil {
                                        prefab: m.prefab,
                                        geom_translation: m.geom_translation,
                                        geom_rotation: m.geom_rotation,
                                    })
                                    .collect()
                            })
                            .collect(),
                        units: v.units,
                        unit_clips: v
                            .unit_clips
                            .iter()
                            .map(|c| c.iter().map(|p| crate::assets::block_name(p).to_string()).collect())
                            .collect(),
                        waypoint_trigger: v.waypoint_trigger,
                        waypoint_trigger_surface: None,
                    })
                    .collect(),
                material_modifier: b.material_modifier,
                material_modifier2: b.material_modifier2,
                clip_type: b.clip_type,
                symmetrical_clip: b.symmetrical_clip,
            })
        })();
        match r {
            Ok(mut b) => {
                for v in &mut b.variants {
                    if let Some(p) = &v.waypoint_trigger {
                        v.waypoint_trigger_surface = c.shape_file(p);
                    }
                }
                for v in &b.variants {
                    for g in &v.mobils {
                        for m in g {
                            if let Some(p) = &m.prefab {
                                c.enqueue(p);
                            }
                        }
                    }
                }
                for m in [&b.material_modifier, &b.material_modifier2].into_iter().flatten() {
                    c.modifier(m);
                }
                c.cat.index.blocks.insert(path.clone(), b);
            }
            Err(e) => c.cat.index.errors.push(format!("{path}: {e}")),
        }
    }
    for path in item_files {
        let r = c.item(&path);
        match r {
            Ok(it) => {
                if let Some(modifier) = &it.material_modifier {
                    c.modifier(modifier);
                }
                c.cat.index.items.insert(path, it);
            }
            Err(e) => c.cat.index.errors.push(format!("{path}: {e}")),
        }
    }
    while let Some(p) = c.prefab_queue.pop_front() {
        if let Err(e) = c.prefab(&p) {
            c.cat.index.errors.push(format!("{p}: {e}"));
        }
    }
    let mats: Vec<String> =
        c.cat.index.surfaces.iter().flat_map(|s| s.materials.iter().flatten().cloned()).collect();
    for m in mats.into_iter().chain(material_files) {
        c.material(&m);
    }
    Ok(c.cat)
}

pub fn crawl_embedded(
    src: &dyn Source,
    items: &[(String, String)],
    known: &Index,
    id_base: usize,
) -> Catalog {
    let mut c = Crawler {
        set: src,
        cat: Catalog { index: Index::default(), surfaces: Vec::new() },
        prefab_queue: VecDeque::new(),
        seen: known.prefabs.keys().map(|p| norm(p)).collect(),
        shape_files: known
            .surfaces
            .iter()
            .enumerate()
            .filter(|(_, s)| !s.source.contains('#'))
            .map(|(i, s)| (norm(&s.source), Some(i)))
            .collect(),
        static_files: Default::default(),
        id_base,
    };
    for p in known.prefabs.values() {
        for e in &p.ents {
            if let EntModel::Static { source: Some(s), .. } = &e.model {
                c.static_files.insert(norm(s), e.model.clone());
            }
        }
    }
    c.cat.index.modifiers = known.modifiers.clone();
    c.cat.index.materials = known.materials.clone();
    for (key, path) in items {
        match c.custom_block(key, path, known) {
            Ok(true) => continue,
            Ok(false) => {}
            Err(e) => {
                c.cat.index.errors.push(format!("{path}: {e}"));
                continue;
            }
        }
        match c.item(path) {
            Ok(it) => {
                c.cat.index.items.insert(key.clone(), it);
            }
            Err(e) => c.cat.index.errors.push(format!("{path}: {e}")),
        }
    }
    while let Some(p) = c.prefab_queue.pop_front() {
        if let Err(e) = c.prefab(&p) {
            c.cat.index.errors.push(format!("{p}: {e}"));
        }
    }
    let mats: Vec<String> =
        c.cat.index.surfaces.iter().flat_map(|s| s.materials.iter().flatten().cloned()).collect();
    for m in mats {
        if !known.materials.contains_key(&m) {
            c.material(&m);
        }
    }
    for m in known.modifiers.keys() {
        c.cat.index.modifiers.remove(m);
    }
    c.cat
}

const KEEP_CATALOGS: usize = 4;

pub const PAKS: [&str; 6] =
    ["stadium.pak", "stadium256.pak", "greencoast.pak", "redisland.pak", "bluebay.pak", "whiteshore.pak"];

pub fn cached(set: &PackSet) -> Result<Catalog> {
    crawl_cached(set, &PAKS, &crate::install::cache_dir())
}

pub fn crawl_cached(set: &PackSet, paks: &[&str], cache: &Path) -> Result<Catalog> {
    let file = cache.join(format!("catalog-{:016x}.bin", cache_key(set, paks)));
    if let Ok(bytes) = std::fs::read(&file)
        && let Ok(catalog) = postcard::from_bytes(&bytes)
    {
        let _ = std::fs::File::options()
            .append(true)
            .open(&file)
            .and_then(|f| f.set_modified(std::time::SystemTime::now()));
        return Ok(catalog);
    }
    let catalog = crawl(set, paks)?;
    if std::fs::create_dir_all(cache).is_ok()
        && let Ok(bytes) = postcard::to_stdvec(&catalog)
    {
        let partial = file.with_extension(format!("{}.tmp", std::process::id()));
        if std::fs::write(&partial, bytes).is_ok() && std::fs::rename(&partial, &file).is_ok() {
            remove_stale(cache, &file);
        }
    }
    Ok(catalog)
}

fn cache_key(set: &PackSet, paks: &[&str]) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    paks.hash(&mut hasher);
    for pak in &set.paks {
        (&pak.path, pak.len, pak.checksum).hash(&mut hasher);
    }
    let binary = std::env::current_exe().and_then(std::fs::metadata);
    if let Ok(meta) = binary {
        (meta.len(), meta.modified().ok()).hash(&mut hasher);
    }
    hasher.finish()
}

fn remove_stale(cache: &Path, current: &Path) {
    let mut kept: Vec<(std::time::SystemTime, std::path::PathBuf)> = std::fs::read_dir(cache)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            let name = p.file_name().and_then(|n| n.to_str()).unwrap_or_default();
            p != current && name.starts_with("catalog-") && name.ends_with(".bin")
        })
        .map(|p| (std::fs::metadata(&p).and_then(|m| m.modified()).unwrap_or(std::time::UNIX_EPOCH), p))
        .collect();
    kept.sort_by(|a, b| b.0.cmp(&a.0));
    for (_, path) in kept.into_iter().skip(KEEP_CATALOGS - 1) {
        let _ = std::fs::remove_file(&path);
    }
}
