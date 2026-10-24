use anyhow::{Context, Result};
use gearbox_packs::{
    catalog::Source,
    prefab::{Ctx, Model, NodeLink, Prefab},
};
use std::collections::HashMap;

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
                map.embedded.files.iter().map(|f| (f.path.as_str(), f.data.as_slice())),
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
            NodeLink::Inline(start) => (archive_path(owner), *start),
            NodeLink::Earlier(_) => {
                anyhow::bail!("earlier dynamic visual modifier traversal: {owner}")
            }
        };
        let key = (path.to_owned(), start);
        if !self.materials.contains_key(&key) {
            let buf = self.read(path)?;
            let layout = gearbox_packs::nodes::layout(&buf)?;
            let ctx = Ctx { buf: &buf, layout: &layout, path };
            let start = if start == 0 { layout.head.body_start } else { start };
            let materials = gearbox_packs::solid::external_materials(&ctx, start)
                .with_context(|| format!("dynamic visual materials: {path}"))?;
            self.materials.insert(key.clone(), materials);
        }
        Ok(self.materials[&key].clone())
    }
    fn store(&mut self, path: &str, mut prefab: Prefab, ctx: &Ctx<'_>) -> Result<()> {
        for entity in &mut prefab.ents {
            if let NodeLink::Earlier(index) = entity.model {
                let node = ctx
                    .layout
                    .nodes
                    .iter()
                    .find(|node| node.index == index)
                    .with_context(|| format!("shared prefab model {index} absent: {path}"))?;
                let mut reader = gearbox_gbx::reader::Reader::new(ctx.buf);
                reader.skip(node.start)?;
                entity.model = NodeLink::Inline(ctx.model(&mut reader, node.class_id)?);
            }
        }
        for (index, entity) in prefab.ents.iter().enumerate() {
            if let NodeLink::Inline(Model::Prefab(child)) = &entity.model {
                self.store(&format!("{path}#ent{index}#prefab"), *child.clone(), ctx)?;
            }
        }
        self.prefabs.insert(path.to_owned(), prefab);
        Ok(())
    }
    pub fn prefab(&mut self, path: &str) -> Result<Prefab> {
        if !self.prefabs.contains_key(path) {
            let owner = archive_path(path);
            let buf = self.read(owner).with_context(|| format!("dynamic prefab {path}"))?;
            let layout = gearbox_packs::nodes::layout(&buf)?;
            let ctx = Ctx { buf: &buf, layout: &layout, path: owner };
            if layout.head.class_id == 0x09145000 {
                self.store(owner, gearbox_packs::prefab::read_prefab(&buf, &layout, owner)?, &ctx)?;
            } else if let Some(node) = layout.nodes.iter().find(|n| n.class_id == 0x2f0bc000) {
                for (index, model) in ctx.item_variants(node.start)?.into_iter().enumerate() {
                    if let NodeLink::Inline(Model::Prefab(prefab)) = model {
                        self.store(&format!("{owner}#variant{index}"), *prefab, &ctx)?;
                    }
                }
            } else {
                let mut end = 0;
                for node in layout.nodes.iter().filter(|n| n.class_id == 0x09145000 && n.index != 0) {
                    if node.start < end {
                        continue;
                    }
                    let mut r = gearbox_gbx::reader::Reader::new(&buf);
                    r.skip(node.start)?;
                    let p = ctx.prefab(&mut r)?;
                    end = r.pos();
                    self.store(&format!("{owner}#prefab{}", node.index), p, &ctx)?;
                }
            }
        }
        self.prefabs.get(path).cloned().with_context(|| format!("dynamic prefab not found: {path}"))
    }
}

fn archive_path(path: &str) -> &str {
    path.to_ascii_lowercase().find(".gbx#").map_or(path, |end| &path[..end + 4])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_dynamic_models_keep_each_entitys_parameters() {
        let Some(root) = std::env::var_os("GBX_ITEM_MAPS") else { return };
        let Some(game) = std::env::var_os("GBX_TEST_DATA") else { return };
        let path = std::path::PathBuf::from(root).join("juxzJrfRESL1DHrYxULYVVjPKT2.Map.Gbx");
        let map = gearbox_gbx::Map::parse(&std::fs::read(path).unwrap()).unwrap();
        let packs = gearbox_packs::install::Game::locate(Some(&std::path::PathBuf::from(game).join("game")))
            .unwrap()
            .open()
            .unwrap();
        let file = map.embedded.files.iter().find(|f| f.path.contains("Firework200")).unwrap();
        let buf = gearbox_packs::embedded::plain_gbx(&file.data).unwrap();
        let layout = gearbox_packs::nodes::layout(&buf).unwrap();
        let ctx = Ctx { buf: &buf, layout: &layout, path: &file.path };
        let node = layout.nodes.iter().find(|node| node.class_id == 0x09145000).unwrap();
        let mut reader = gearbox_gbx::reader::Reader::new(&buf);
        reader.skip(node.start).unwrap();
        let raw = ctx.prefab(&mut reader).unwrap();
        let mut archives = Archives::new(&map, &packs);
        let prefab = archives.prefab(&format!("{}#prefab1", file.path)).unwrap();
        assert!(matches!(prefab.ents[6].model, NodeLink::Inline(Model::DynaObject(_))));
        assert_eq!(prefab.ents[6].model, prefab.ents[4].model);
        assert_eq!(prefab.ents[6].params, raw.ents[6].params);
        assert_eq!(prefab.ents[6].rotation, raw.ents[6].rotation);
        assert_eq!(prefab.ents[6].position, raw.ents[6].position);
    }
}
