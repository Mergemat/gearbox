use super::decode::{chunk, cursor};
use super::*;
use crate::prefab::{Ctx, NodeLink};
use anyhow::{Context, ensure};

fn role(s: &str) -> TextureRole {
    let name = s.to_ascii_lowercase();
    if name.contains("normal") || name == "n" || name.ends_with("_n.dds") {
        TextureRole::Normal
    } else if name.contains("diffuse")
        || name.contains("albedo")
        || name.contains("basecolor")
        || name == "d"
        || name == "basediffuse"
        || name.ends_with("_d.dds")
    {
        TextureRole::Albedo
    } else if name.contains("specular")
        || name.contains("roughness")
        || name.contains("roughmetal")
        || name == "s"
        || name.ends_with("_s.dds")
    {
        TextureRole::Specular
    } else if name.contains("emiss") || name.contains("illum") || name == "i" || name.ends_with("_i.dds") {
        TextureRole::Emissive
    } else {
        TextureRole::Other
    }
}
impl VisualCache {
    pub fn material(&mut self, path: &str) -> Result<MaterialId> {
        let key = norm(path);
        if let Some(&i) = self.material_ids.get(&key) {
            return Ok(i);
        }
        let b = self.read(path)?;
        let ids = crate::material::material_ids(&b).ok();
        let mut mat = Material {
            name: path.into(),
            surface_id: ids.map(|i| i.surface),
            gameplay_id: ids.map(|i| i.gameplay),
            ..Default::default()
        };
        self.texture_graph(path, &mut mat, 0, &mut std::collections::HashSet::new())?;
        let i = self.materials.len();
        self.materials.push(mat);
        self.material_ids.insert(key, i);
        Ok(i)
    }
    pub(super) fn texture_graph(
        &mut self,
        path: &str,
        mat: &mut Material,
        depth: usize,
        seen: &mut std::collections::HashSet<String>,
    ) -> Result<()> {
        ensure!(depth < 24, "material graph recursion");
        if !seen.insert(norm(path)) {
            return Ok(());
        }
        let b = self.read(path)?;
        let l = crate::nodes::layout(&b)?;
        let ctx = Ctx { buf: &b, layout: &l, path };
        let mut bitmap_usage = None;
        if l.head.class_id == 0x09011000 {
            for id in [0x09011014, 0x09011015, 0x09011018, 0x09011022] {
                if let Ok(p) = chunk(&ctx, l.head.body_start, id) {
                    let mut r = cursor(&b, p)?;
                    ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
                    bitmap_usage = Some((r.u32()? >> 8) as u8);
                    break;
                }
            }
        }
        let mut named = std::collections::HashMap::new();
        for n in &l.nodes {
            if n.class_id != 0x0903a000 {
                continue;
            }
            for id in [0x0903a013, 0x0903a006] {
                let Ok(p) = chunk(&ctx, n.start, id) else { continue };
                let mut r = cursor(&b, p)?;
                if id == 0x0903a013 {
                    r.u32()?;
                }
                for _ in 0..r.len()? {
                    let (raw, name) = crate::crystal::id(&mut r)?;
                    r.u32()?;
                    let reference = ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
                    if let NodeLink::External(p) = reference {
                        named.insert(norm(&p), name.unwrap_or_else(|| format!("sampler-id-{raw:#x}")));
                    }
                    if id == 0x0903a013 {
                        r.skip(8)?;
                    }
                }
            }
        }
        let mut refs: Vec<_> = l.head.ref_files.keys().filter_map(|&i| l.head.ref_path(path, i)).collect();
        refs.sort();
        refs.dedup();
        for p in refs {
            let lower = norm(&p);
            let sampler =
                named.get(&lower).cloned().unwrap_or_else(|| crate::assets::block_name(&p).to_string());
            let texture = lower.ends_with(".dds") || lower.ends_with(".png") || lower.ends_with(".webp");
            if texture {
                match self.texture(&p) {
                    Ok(texture) => {
                        let mut kind = role(&sampler);
                        if kind == TextureRole::Other {
                            kind = role(&p);
                        }
                        if !mat.textures.iter().any(|b| b.texture == texture && b.sampler == sampler) {
                            mat.textures.push(TextureBinding { sampler, role: kind, texture, bitmap_usage });
                        }
                    }
                    Err(e) => self.report("texture", &p, format!("{e:#}")),
                }
            } else {
                let cid = self.packs.find(&p).map(|r| self.packs.paks[r.pak].files[r.file].class_id);
                if matches!(
                    cid,
                    Some(0x09011000 | 0x09002000 | 0x09079000 | 0x0903a000 | 0x090f0000 | 0x0914c000)
                ) || lower.ends_with(".bitmap.gbx")
                    || lower.ends_with(".mat.gbx")
                {
                    let mut child = Material::default();
                    match self.texture_graph(&p, &mut child, depth + 1, seen) {
                        Ok(()) => {
                            for mut binding in child.textures {
                                if named.contains_key(&lower) {
                                    if !sampler.starts_with("sampler-id-") {
                                        binding.role = role(&sampler);
                                    }
                                    binding.sampler = sampler.clone();
                                }
                                if !mat
                                    .textures
                                    .iter()
                                    .any(|b| b.texture == binding.texture && b.sampler == binding.sampler)
                                {
                                    mat.textures.push(binding);
                                }
                            }
                        }
                        Err(e) => self.report("material-graph", &p, format!("{e:#}")),
                    }
                }
            }
        }
        Ok(())
    }
    pub(super) fn material_ref(&mut self, ctx: &Ctx<'_>, m: NodeLink<usize>) -> Result<MaterialId> {
        match m {
            NodeLink::External(p) => self.material_or_diagnostic(&p),
            NodeLink::Null => Ok(self.intern_material(format!("{}#null", ctx.path), Material::default())),
            NodeLink::Inline(p) => self.inline_material(ctx, p),
            NodeLink::Earlier(i) => self.inline_material(
                ctx,
                ctx.layout.nodes.iter().find(|n| n.index == i).context("material node")?.start,
            ),
        }
    }
    fn inline_material(&mut self, ctx: &Ctx<'_>, p: usize) -> Result<MaterialId> {
        let n = ctx.layout.nodes.iter().find(|n| n.start == p).context("inline material")?;
        if n.class_id == 0x090fd000 {
            return self.user_material(ctx, p);
        }
        let end = n.end.context("material end")?;
        let ids = crate::material::material_ids(&ctx.buf[p..end]).ok();
        Ok(self.intern_material(
            format!("{}#material{p}", ctx.path),
            Material {
                surface_id: ids.map(|i| i.surface),
                gameplay_id: ids.map(|i| i.gameplay),
                ..Default::default()
            },
        ))
    }
    pub(super) fn material_or_diagnostic(&mut self, path: &str) -> Result<MaterialId> {
        match self.material(path) {
            Ok(i) => Ok(i),
            Err(e) => {
                self.report("material", path, format!("{e:#}"));
                Ok(self.intern_material(path.into(), Material::default()))
            }
        }
    }
    pub(super) fn named_material(&mut self, name: String) -> MaterialId {
        self.intern_material(name.clone(), Material { name, ..Default::default() })
    }
    pub(super) fn intern_unassigned(&mut self, path: &str) -> MaterialId {
        self.intern_material(format!("{path}#unassigned"), Material::default())
    }
    pub(super) fn intern_material(&mut self, key: String, mut mat: Material) -> MaterialId {
        let key = norm(&key);
        if let Some(&i) = self.material_ids.get(&key) {
            return i;
        }
        if mat.name.is_empty() {
            mat.name = key.clone();
        }
        let i = self.materials.len();
        self.materials.push(mat);
        self.material_ids.insert(key, i);
        i
    }
    pub(super) fn user_material(&mut self, ctx: &Ctx<'_>, start: usize) -> Result<MaterialId> {
        let key = format!("{}#user{start}", ctx.path);
        if let Some(&i) = self.material_ids.get(&norm(&key)) {
            return Ok(i);
        }
        let mut r = cursor(ctx.buf, start)?;
        ensure!(r.u32()? == 0x090fd000, "user material chunk");
        let version = r.u32()?;
        ensure!((10..=11).contains(&version), "user material version {version}");
        let game = version >= 11 && r.bool8()?;
        let (_, name) = crate::crystal::id(&mut r)?;
        let (_, model) = crate::crystal::id(&mut r)?;
        let base = r.string()?;
        let surface_id = Some(r.u8()?);
        let gameplay_id = Some(r.u8()?);
        let link = if game || version == 10 { Some(r.string()?) } else { crate::crystal::id(&mut r)?.1 };
        let mut mat = Material {
            name: name.unwrap_or_else(|| key.clone()),
            surface_id,
            gameplay_id,
            ..Default::default()
        };
        for _ in 0..r.len()? {
            crate::crystal::id(&mut r)?;
            crate::crystal::id(&mut r)?;
            r.u32()?;
        }
        let colors = (0..r.len()?).map(|_| r.i32()).collect::<Result<Vec<_>>>()?;
        if colors.len() == 3 && colors.iter().all(|&v| (0..=255).contains(&v)) {
            mat.base_color =
                Some([colors[0] as f32 / 255., colors[1] as f32 / 255., colors[2] as f32 / 255., 1.]);
        }
        if let Some(link) = link.filter(|s| !s.is_empty()) {
            let path = crate::material::material_link_path(&link);
            match self.material(&path) {
                Ok(i) => mat.textures = self.materials[i].textures.clone(),
                Err(e) => self.report("user-material", &key, format!("link {path}: {e:#}")),
            }
        } else if !base.is_empty() {
            self.report(
                "user-material",
                &key,
                format!("custom texture base {base:?}, model {model:?}; user texture slots not decoded"),
            );
        }
        Ok(self.intern_material(key, mat))
    }
}
