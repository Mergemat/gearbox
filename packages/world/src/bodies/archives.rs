use anyhow::{Context, Result};
use std::collections::HashMap;
use gearbox_packs::{
    catalog::Source,
    prefab::{Ctx, Model, Prefab, NodeLink},
};

pub struct Archives<'a> {
    packs: &'a gearbox_packs::packs::PackSet,
    embedded: gearbox_packs::embedded::Files<'a>,
    aliases: HashMap<String, String>,
    prefabs: HashMap<String, Prefab>,
    materials: HashMap<(String, usize), Vec<String>>,
}
impl<'a> Archives<'a> {
    pub fn new(map: &gearbox_gbx::Map, packs: &'a gearbox_packs::packs::PackSet) -> Self {
        Self {
            packs,
            embedded: gearbox_packs::embedded::Files::new(
                map.embedded
                    .files
                    .iter()
                    .map(|f| (f.path.as_str(), f.data.as_slice())),
                packs,
            ),
            aliases: map
                .embedded
                .files
                .iter()
                .flat_map(|f| {
                    let name = f.item_model.as_ref().and_then(|m| m.ident[0].clone());
                    std::iter::once(f.path.clone())
                        .chain(name)
                        .map(|p| (p.replace('/', "\\").to_lowercase(), f.path.clone()))
                })
                .collect(),
            prefabs: HashMap::new(),
            materials: HashMap::new(),
        }
    }
    pub fn read(&self, path: &str) -> Result<Vec<u8>> {
        if let Some(file) = self.aliases.get(&path.replace('/', "\\").to_lowercase()) {
            self.embedded.read_path(file)
        } else {
            self.packs.read_path(path)
        }
    }
    pub fn visual_materials(&mut self, owner: &str, visual: &NodeLink<usize>) -> Result<Vec<String>> {
        let (path, start) = match visual {
            NodeLink::Null => return Ok(Vec::new()),
            NodeLink::External(path) => (path.as_str(), 0),
            NodeLink::Inline(start) => (owner.split('#').next().unwrap_or(owner), *start),
            NodeLink::Earlier(_) => {
                anyhow::bail!("earlier dynamic visual modifier traversal: {owner}")
            }
        };
        let key = (path.to_owned(), start);
        if !self.materials.contains_key(&key) {
            let buf = self.read(path)?;
            let layout = gearbox_packs::nodes::layout(&buf)?;
            let ctx = Ctx {
                buf: &buf,
                layout: &layout,
                path,
            };
            let start = if start == 0 {
                layout.head.body_start
            } else {
                start
            };
            let materials = gearbox_packs::solid::external_materials(&ctx, start)
                .with_context(|| format!("dynamic visual materials: {path}"))?;
            self.materials.insert(key.clone(), materials);
        }
        Ok(self.materials[&key].clone())
    }
    fn store(&mut self, path: &str, prefab: Prefab) {
        for (index, entity) in prefab.ents.iter().enumerate() {
            if let NodeLink::Inline(Model::Prefab(child)) = &entity.model {
                self.store(&format!("{path}#ent{index}#prefab"), *child.clone());
            }
        }
        self.prefabs.insert(path.to_owned(), prefab);
    }
    pub fn prefab(&mut self, path: &str) -> Result<Prefab> {
        if !self.prefabs.contains_key(path) {
            let owner = path.split('#').next().unwrap_or(path);
            let buf = self
                .read(owner)
                .with_context(|| format!("dynamic prefab {path}"))?;
            let layout = gearbox_packs::nodes::layout(&buf)?;
            let ctx = Ctx {
                buf: &buf,
                layout: &layout,
                path: owner,
            };
            if layout.head.class_id == 0x09145000 {
                self.store(owner, gearbox_packs::prefab::read_prefab(&buf, &layout, owner)?);
            } else {
                let mut end = 0;
                for node in layout
                    .nodes
                    .iter()
                    .filter(|n| n.class_id == 0x09145000 && n.index != 0)
                {
                    if node.start < end {
                        continue;
                    }
                    let mut r = gearbox_gbx::reader::Reader::new(&buf);
                    r.skip(node.start)?;
                    let p = ctx.prefab(&mut r)?;
                    end = r.pos();
                    self.store(&format!("{owner}#prefab{}", node.index), p);
                }
            }
        }
        self.prefabs
            .get(path)
            .cloned()
            .with_context(|| format!("dynamic prefab not found: {path}"))
    }
}
