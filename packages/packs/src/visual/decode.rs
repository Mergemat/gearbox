use super::*;
use crate::prefab::{Ctx, NodeLink};
use anyhow::{Context, bail, ensure};
use gearbox_gbx::reader::Reader;

pub(super) fn cursor(b: &[u8], p: usize) -> Result<Reader<'_>> {
    let mut r = Reader::new(b);
    r.skip(p)?;
    Ok(r)
}
pub(super) fn chunk(ctx: &Ctx<'_>, start: usize, id: u32) -> Result<usize> {
    let end =
        ctx.layout.nodes.iter().find(|n| n.start == start).and_then(|n| n.end).context("node without end")?;
    ctx.buf
        .get(start..end)
        .context("node bounds")?
        .windows(4)
        .position(|b| b == id.to_le_bytes())
        .map(|p| start + p + 4)
        .with_context(|| format!("chunk {id:#x} absent"))
}
fn node<'a>(ctx: &'a Ctx<'_>, index: u32) -> Result<&'a crate::nodes::NodeSpan> {
    ctx.layout.nodes.iter().find(|n| n.index == index).with_context(|| format!("node {index} absent"))
}
fn reference(ctx: &Ctx<'_>, r: &mut Reader) -> Result<NodeLink<usize>> {
    ctx.noderef(r, |c, r, class| {
        let start = r.pos();
        if matches!(class, 0x09159000 | 0x09144000 | 0x09145000) {
            c.model(r, class)?;
        } else {
            c.skip_node(r)?;
        }
        Ok(start)
    })
}
fn resolve_start(ctx: &Ctx<'_>, r: NodeLink<usize>) -> Result<usize> {
    match r {
        NodeLink::Inline(p) => Ok(p),
        NodeLink::Earlier(i) => Ok(node(ctx, i)?.start),
        _ => bail!("external/null vertex stream"),
    }
}
fn packed(v: u32) -> [f32; 3] {
    std::array::from_fn(|k| (((v >> (10 * k)) as i32 & 1023) << 22 >> 22) as f32 / 511.)
}
fn rgba(v: u32) -> [f32; 4] {
    [
        ((v >> 16) & 255) as f32 / 255.,
        ((v >> 8) & 255) as f32 / 255.,
        (v & 255) as f32 / 255.,
        (v >> 24) as f32 / 255.,
    ]
}

fn stream(ctx: &Ctx<'_>, start: usize, mesh: &mut Mesh, depth: usize) -> Result<()> {
    ensure!(depth < 16, "shared stream recursion");
    let mut r = cursor(ctx.buf, start)?;
    ensure!(r.u32()? == 0x09056000, "vertex stream chunk");
    let version = r.u32()?;
    ensure!((1..=2).contains(&version), "vertex stream version {version}");
    let count = r.len()?;
    r.u32()?;
    let shared = reference(ctx, &mut r)?;
    if !matches!(shared, NodeLink::Null) {
        return stream(ctx, resolve_start(ctx, shared)?, mesh, depth + 1);
    }
    let mut decls = Vec::new();
    for _ in 0..r.len()? {
        let flags = r.u32()?;
        let flags2 = r.u32()?;
        if flags2 & 0xffc != 0 {
            r.skip(4)?;
        }
        decls.push((flags & 511, flags >> 9 & 511));
    }
    r.bool32()?;
    for (semantic, ty) in decls {
        match (semantic, ty) {
            (4, 5) => {
                mesh.joint_indices = (0..count).map(|_| Ok(r.u32()?.to_le_bytes())).collect::<Result<_>>()?;
            }
            (3, 1) => {
                mesh.blend_weights = (0..count).map(|_| Ok([r.f32()?, r.f32()?])).collect::<Result<_>>()?;
            }
            (0, 2) => mesh.positions = (0..count).map(|_| r.vec3()).collect::<Result<_>>()?,
            (5 | 6, 2 | 14) | (18 | 19 | 20 | 21, 14) => {
                let values = (0..count).map(|_| Ok(packed(r.u32()?))).collect::<Result<Vec<_>>>()?;
                match semantic {
                    5 => mesh.normals = values,
                    18 => mesh.tangent_u = values,
                    20 => mesh.tangent_v = values,
                    _ => {}
                }
            }
            (10..=17, 1) => {
                let channel = (semantic - 10) as usize;
                mesh.uvs.resize_with(mesh.uvs.len().max(channel + 1), Vec::new);
                mesh.uvs[channel] = (0..count).map(|_| Ok([r.f32()?, r.f32()?])).collect::<Result<_>>()?;
            }
            (8 | 9 | 22, 4) => {
                let channel = match semantic {
                    8 => 0,
                    9 => 1,
                    _ => 2,
                };
                mesh.colors.resize_with(mesh.colors.len().max(channel + 1), Vec::new);
                mesh.colors[channel] = (0..count).map(|_| Ok(rgba(r.u32()?))).collect::<Result<_>>()?;
            }
            _ => {
                let size = match ty {
                    1 => 8,
                    2 => 12,
                    3 => 16,
                    4 | 5 | 14 => 4,
                    _ => bail!("vertex type {ty}, semantic {semantic}"),
                };
                r.skip(size * count)?;
            }
        }
    }
    Ok(())
}
fn visual(ctx: &Ctx<'_>, start: usize) -> Result<Mesh> {
    let mut mesh = Mesh::default();
    let mut r = cursor(ctx.buf, chunk(ctx, start, 0x0900600f)?)?;
    let version = r.u32()?;
    ensure!((4..=6).contains(&version), "visual version {version}");
    let flags = r.u32()?;
    let uv_count = r.len()?;
    let count = r.len()?;
    let streams = r.len()?;
    for _ in 0..streams {
        let s = reference(ctx, &mut r)?;
        stream(ctx, resolve_start(ctx, s)?, &mut mesh, 0)?;
    }
    for channel in 0..uv_count {
        let v = r.u32()?;
        let uv_flags = if v >= 3 {
            ensure!(r.len()? == count, "UV count");
            r.u32()?
        } else {
            0
        };
        let mut values = Vec::new();
        for _ in 0..count {
            values.push([r.f32()?, r.f32()?]);
            if v == 1 {
                r.skip(4)?;
            }
            if v == 2 {
                r.skip(8)?;
            }
        }
        if v >= 3 {
            r.skip(count * (uv_flags as usize & 255) * 4)?;
        }
        mesh.uvs.resize_with(mesh.uvs.len().max(channel + 1), Vec::new);
        mesh.uvs[channel] = values;
    }
    if flags & 7 != 0 {
        let skin_version = 2 + version;
        r.bool32()?;
        r.u32()?;
        if skin_version >= 5 {
            ensure!(!r.bool32()?, "unsupported skin deformation");
            r.bool32()?;
        }
        let bones = r.len()?;
        if skin_version == 2 {
            let n = r.len()?;
            r.skip(n * 48)?;
        }
        for _ in 0..bones {
            let (raw, name) = crate::crystal::id(&mut r)?;
            mesh.skin_bones.push(name.unwrap_or_else(|| format!("bone-id-{raw:#x}")));
        }
        if skin_version != 3 {
            let n = r.len()?;
            r.skip(n * 4)?;
        }
    }
    if streams == 0 {
        let mut r = cursor(ctx.buf, chunk(ctx, start, 0x0902c004)?)?;
        let normals = flags & 0x200 == 0 || flags & 0x20 != 0;
        let colors = flags & 0x200 == 0 || flags & 0x40 != 0;
        for _ in 0..count {
            mesh.positions.push(r.vec3()?);
            if normals {
                mesh.normals.push(if flags & 0x80 != 0 { packed(r.u32()?) } else { r.vec3()? });
            }
            if colors {
                if mesh.colors.is_empty() {
                    mesh.colors.push(Vec::new());
                }
                mesh.colors[0].push(if flags & 0x100 != 0 {
                    rgba(r.u32()?)
                } else {
                    [r.f32()?, r.f32()?, r.f32()?, r.f32()?]
                });
            }
        }
        for target in [&mut mesh.tangent_u, &mut mesh.tangent_v] {
            let n = r.len()?;
            ensure!(n == 0 || n == count, "tangent count");
            for _ in 0..n {
                target.push(if flags & 0x80 != 0 { packed(r.u32()?) } else { r.vec3()? });
            }
        }
    }
    ensure!(mesh.positions.len() == count, "visual/vertex count {} != {count}", mesh.positions.len());
    let mut r = cursor(ctx.buf, chunk(ctx, start, 0x0906a001)?)?;
    ensure!(r.bool32()?, "visual without index buffer");
    let kind = r.u32()?;
    ensure!(matches!(kind, 0x09057000 | 0x09057001), "index chunk {kind:#x}");
    r.u32()?;
    let mut last = 0u16;
    for _ in 0..r.len()? {
        let value = r.u16()?;
        last = if kind == 0x09057001 { last.wrapping_add(value) } else { value };
        mesh.indices.push(last as u32);
    }
    ensure!(
        mesh.indices.len() % 3 == 0 && mesh.indices.iter().all(|&i| (i as usize) < count),
        "invalid visual indices"
    );
    if flags & 7 != 0 {
        mesh.name = format!("bind-pose skin flags {}", flags & 7);
    }
    Ok(mesh)
}

impl VisualCache {
    pub(super) fn solid(&mut self, ctx: &Ctx<'_>, start: usize) -> Result<Vec<Part>> {
        let key = format!("{}#solid{start}", norm(ctx.path));
        if let Some(result) = self.decoded.get(&key) {
            self.diagnostics.extend(result.diagnostics.clone());
            return result.result.clone().map_err(anyhow::Error::msg);
        }
        let begin = self.diagnostics.len();
        let result = self.solid_inner(ctx, start);
        self.decoded.insert(
            key,
            Decoded {
                result: result.as_ref().map(Clone::clone).map_err(|e| format!("{e:#}")),
                diagnostics: self.diagnostics[begin..].to_vec(),
            },
        );
        result
    }
    fn solid_inner(&mut self, ctx: &Ctx<'_>, start: usize) -> Result<Vec<Part>> {
        let mut r = cursor(ctx.buf, start)?;
        ensure!(r.u32()? == 0x090bb000, "Solid2 chunk");
        let version = r.u32()?;
        ensure!((6..=34).contains(&version), "Solid2 version {version}");
        crate::crystal::id(&mut r)?;
        let mut geoms = Vec::new();
        for _ in 0..r.len()? {
            let vi = r.u32()?;
            let mi = r.u32()?;
            r.u32()?;
            let lod = r.u32()?;
            if version >= 32 {
                r.u32()?;
            }
            geoms.push((vi, mi, lod));
        }
        ensure!(r.u32()? == 10, "visual array version");
        let mut visuals = Vec::new();
        for _ in 0..r.len()? {
            let v = reference(ctx, &mut r)?;
            visuals.push(resolve_start(ctx, v)?);
        }
        let mut material_names = Vec::new();
        for _ in 0..r.len()? {
            material_names.push(crate::crystal::id(&mut r)?.1);
        }
        let custom = if version >= 29 { r.len()? } else { 0 };
        let mut materials = Vec::new();
        if custom == 0 {
            ensure!(r.u32()? == 10, "material array version");
            for _ in 0..r.len()? {
                let m = reference(ctx, &mut r)?;
                materials.push(self.material_ref(ctx, m)?);
            }
        }
        let mut global_joints = Vec::new();
        if version >= 29 && (custom > 0 || (materials.is_empty() && !material_names.is_empty())) {
            reference(ctx, &mut r)?;
            let n = r.len()?;
            r.skip(n * 4)?;
            r.u32()?;
            if r.bool32()? {
                let v = r.u32()?;
                r.skip(4 + 4 + 4 + 8 * 4 + 8)?;
                let n = r.len()?;
                r.skip(n * 24)?;
                if v >= 1 {
                    let n = r.len()?;
                    r.skip(n * 16)?;
                }
            }
            r.skip(8)?;
            r.string()?;
            r.string()?;
            r.string()?;
            for _ in 0..r.len()? {
                crate::crystal::id(&mut r)?;
                if r.bool32()? {
                    reference(ctx, &mut r)?;
                } else {
                    r.string()?;
                }
                r.skip(48 + 6 * 4)?;
                if r.bool32()? {
                    r.skip(3 * 4)?;
                }
            }
            for _ in 0..r.len()? {
                reference(ctx, &mut r)?;
            }
            let n = r.len()?;
            r.skip(n * 8)?;
            r.skip(3 * 4)?;
            r.string()?;
            if version >= 30 {
                r.u32()?;
            }
            for _ in 0..custom {
                let name = r.string()?;
                let material = if name.is_empty() {
                    let m = reference(ctx, &mut r)?;
                    self.material_ref(ctx, m)?
                } else {
                    self.material_or_diagnostic(&name)?
                };
                materials.push(material);
            }
            for _ in 0..r.len()? {
                let (raw, name) = crate::crystal::id(&mut r)?;
                global_joints.push(name.unwrap_or_else(|| {
                    if raw == u32::MAX { String::new() } else { format!("bone-id-{raw:#x}") }
                }));
            }
        }
        if materials.is_empty() && !material_names.is_empty() {
            for (i, name) in material_names.into_iter().enumerate() {
                let name = name.with_context(|| format!("named material {i} uses unresolved lookback"))?;
                materials.push(self.named_material(name));
            }
        }
        let mut out = Vec::new();
        for (vi, mi, lod) in geoms {
            if lod != 0 && lod & 1 == 0 {
                continue;
            }
            let material = *materials
                .get(mi as usize)
                .with_context(|| format!("material slot {mi}/{}", materials.len()))?;
            let mut mesh = visual(ctx, *visuals.get(vi as usize).context("visual index")?)?;
            if mesh.indices.is_empty() {
                continue;
            }
            if mesh.skin_bones.is_empty() {
                mesh.skin_bones = global_joints.clone();
            }
            if !mesh.blend_weights.is_empty() {
                self.report("skinning", ctx.path, "weighted skin retained in bind pose");
            }
            mesh.name = format!("{}#visual{vi}/material{mi}", ctx.path);
            mesh.submeshes.push(Submesh {
                first_index: 0,
                index_count: mesh.indices.len() as u32,
                material,
                lod_mask: lod,
            });
            let mesh_id = self.meshes.len();
            self.meshes.push(mesh);
            out.push(Part { mesh: mesh_id, chain: Vec::new() });
        }
        Ok(out)
    }
    pub(super) fn mesh_ref(&mut self, ctx: &Ctx<'_>, m: NodeLink<usize>, depth: usize) -> Result<Vec<Part>> {
        match m {
            NodeLink::Null => Ok(Vec::new()),
            NodeLink::External(p) => self.file_parts(&p, 0, depth + 1),
            m => self.solid(ctx, resolve_start(ctx, m)?),
        }
    }
    fn at(
        &mut self,
        ctx: &Ctx<'_>,
        start: usize,
        class: u32,
        variant: usize,
        depth: usize,
    ) -> Result<Vec<Part>> {
        ensure!(depth < 64, "model recursion limit");
        match class {
            0x090bb000 => self.solid(ctx, start),
            0x09159000 => {
                let mut r = cursor(ctx.buf, start)?;
                let so = ctx.static_object(&mut r)?;
                self.mesh_ref(ctx, so.mesh, depth)
            }
            0x09144000 => {
                let mut r = cursor(ctx.buf, start)?;
                r.u32()?;
                r.skip(8)?;
                let m = reference(ctx, &mut r)?;
                self.mesh_ref(ctx, m, depth)
            }
            0x09145000 => {
                let mut r = cursor(ctx.buf, start)?;
                let p = ctx.prefab(&mut r)?;
                let mut out = Vec::new();
                for e in p.ents {
                    let loc = crate::assets::Loc { rotation: e.rotation, position: e.position };
                    let result = match e.model {
                        NodeLink::External(p) => self.file_parts(&p, 0, depth + 1),
                        NodeLink::Earlier(i) => {
                            let n = node(ctx, i)?;
                            self.at(ctx, n.start, n.class_id, 0, depth + 1)
                        }
                        NodeLink::Inline(crate::prefab::Model::StaticObject(so)) => {
                            self.mesh_ref(ctx, so.mesh, depth + 1)
                        }
                        NodeLink::Inline(crate::prefab::Model::Prefab(p)) => {
                            self.prefab_value(ctx, *p, depth + 1)
                        }
                        NodeLink::Inline(crate::prefab::Model::DynaObject(d)) => {
                            self.mesh_ref(ctx, d.mesh, depth + 1)
                        }
                        NodeLink::Inline(crate::prefab::Model::Other(c))
                            if !matches!(c, 0x09051000 | 0x09003000 | 0x2f086000) =>
                        {
                            continue;
                        }
                        NodeLink::Null
                        | NodeLink::Inline(crate::prefab::Model::Kinematic(_))
                        | NodeLink::Inline(crate::prefab::Model::Trigger { .. })
                        | NodeLink::Inline(crate::prefab::Model::Surface(_)) => {
                            continue;
                        }
                        _ => Err(anyhow::anyhow!("prefab entity visual class unsupported")),
                    };
                    match result {
                        Ok(parts) => {
                            for mut p in parts {
                                p.chain.insert(0, loc);
                                out.push(p);
                            }
                        }
                        Err(error) => {
                            self.report("prefab", &format!("{}:{}", ctx.path, e.name), format!("{error:#}"))
                        }
                    }
                }
                Ok(out)
            }
            0x2f0bc000 => {
                let mut r = cursor(ctx.buf, start)?;
                let version = r.u32()?;
                ensure!(version <= 1, "item variant version {version}");
                let n = r.len()?;
                ensure!(variant < n, "item variant {variant}/{n}");
                let mut selected = None;
                for i in 0..n {
                    for _ in 0..r.len()? {
                        r.string()?;
                        r.string()?;
                    }
                    let m = reference(ctx, &mut r)?;
                    if i == variant {
                        selected = Some(m);
                    }
                    if version >= 1 {
                        r.u32()?;
                    }
                }
                self.model_ref(ctx, selected.context("item variant")?, depth)
            }
            0x09003000 => self.crystal(ctx, start),
            0x2f086000 => self.tree(ctx),
            0x0915c000 | 0x09160000 => Ok(Vec::new()),
            0x2f0ca000 => {
                self.report(
                    "animation",
                    ctx.path,
                    "kinematic motion is not evaluated; mesh uses archived prefab transform",
                );
                Ok(Vec::new())
            }
            _ => bail!("visual model class {class:#010x}"),
        }
    }
    fn prefab_value(&mut self, ctx: &Ctx<'_>, p: crate::prefab::Prefab, depth: usize) -> Result<Vec<Part>> {
        let mut out = Vec::new();
        for e in p.ents {
            let mut parts = match e.model {
                NodeLink::External(p) => self.file_parts(&p, 0, depth + 1)?,
                NodeLink::Inline(crate::prefab::Model::StaticObject(so)) => {
                    self.mesh_ref(ctx, so.mesh, depth + 1)?
                }
                NodeLink::Inline(crate::prefab::Model::Prefab(p)) => self.prefab_value(ctx, *p, depth + 1)?,
                NodeLink::Inline(crate::prefab::Model::DynaObject(d)) => {
                    self.mesh_ref(ctx, d.mesh, depth + 1)?
                }
                NodeLink::Earlier(i) => {
                    let n = node(ctx, i)?;
                    self.at(ctx, n.start, n.class_id, 0, depth + 1)?
                }
                _ => continue,
            };
            for p in &mut parts {
                p.chain.insert(0, crate::assets::Loc { rotation: e.rotation, position: e.position });
            }
            out.extend(parts);
        }
        Ok(out)
    }
    fn model_ref(&mut self, ctx: &Ctx<'_>, m: NodeLink<usize>, depth: usize) -> Result<Vec<Part>> {
        match m {
            NodeLink::External(p) => self.file_parts(&p, 0, depth + 1),
            NodeLink::Null => Ok(Vec::new()),
            r => {
                let start = resolve_start(ctx, r)?;
                let n = ctx.layout.nodes.iter().find(|n| n.start == start).context("model node")?;
                self.at(ctx, start, n.class_id, 0, depth + 1)
            }
        }
    }
    pub(super) fn file_parts(&mut self, path: &str, variant: usize, depth: usize) -> Result<Vec<Part>> {
        let key = format!("{}#variant{variant}", norm(path));
        if let Some(r) = self.decoded.get(&key) {
            self.diagnostics.extend(r.diagnostics.clone());
            return r.result.clone().map_err(anyhow::Error::msg);
        }
        ensure!(depth < 64, "file recursion limit");
        let begin = self.diagnostics.len();
        let result = (|| {
            let b = self.read(path)?;
            let l = crate::nodes::layout(&b)?;
            let ctx = Ctx { buf: &b, layout: &l, path };
            if l.head.class_id == 0x2e002000 {
                if let Some(n) = l.nodes.iter().find(|n| n.class_id == 0x2f0bc000) {
                    return self.at(&ctx, n.start, n.class_id, variant, depth);
                }
                if let Some(n) = l.nodes.iter().find(|n| n.class_id == 0x2e027000) {
                    let mut r = cursor(&b, chunk(&ctx, n.start, 0x2e027000)?)?;
                    ensure!(r.u32()? >= 4, "common item archive before v4");
                    let m = reference(&ctx, &mut r)?;
                    return self.model_ref(&ctx, m, depth);
                }
                if let Some(n) =
                    l.nodes.iter().find(|n| matches!(n.class_id, 0x09145000 | 0x09159000 | 0x09003000))
                {
                    return self.at(&ctx, n.start, n.class_id, variant, depth);
                }
                let mut refs: Vec<_> = l
                    .head
                    .ref_files
                    .keys()
                    .filter_map(|&i| l.head.ref_path(path, i))
                    .filter(|p| {
                        self.packs.find(p).is_some_and(|r| {
                            matches!(
                                self.packs.paks[r.pak].files[r.file].class_id,
                                0x09145000 | 0x09159000 | 0x09144000 | 0x090bb000 | 0x2f086000
                            )
                        })
                    })
                    .collect();
                refs.sort();
                refs.dedup();
                ensure!(!refs.is_empty(), "item without visual entity reference");
                let mut out = Vec::new();
                for p in refs {
                    out.extend(self.file_parts(&p, variant, depth + 1)?);
                }
                return Ok(out);
            }
            self.at(&ctx, l.head.body_start, l.head.class_id, variant, depth)
        })();
        self.decoded.insert(
            key,
            Decoded {
                result: result.as_ref().map(Clone::clone).map_err(|e| format!("{e:#}")),
                diagnostics: self.diagnostics[begin..].to_vec(),
            },
        );
        result
    }
}
impl VisualCache {
    pub(super) fn crystal(&mut self, ctx: &Ctx<'_>, start: usize) -> Result<Vec<Part>> {
        let key = format!("{}#crystal{start}", norm(ctx.path));
        if let Some(cached) = self.decoded.get(&key) {
            self.diagnostics.extend(cached.diagnostics.clone());
            return cached.result.clone().map_err(anyhow::Error::msg);
        }
        let begin = self.diagnostics.len();
        let result = self.crystal_inner(ctx, start);
        self.decoded.insert(
            key,
            Decoded {
                result: result.as_ref().map(Clone::clone).map_err(|e| format!("{e:#}")),
                diagnostics: self.diagnostics[begin..].to_vec(),
            },
        );
        result
    }
    fn crystal_inner(&mut self, ctx: &Ctx<'_>, start: usize) -> Result<Vec<Part>> {
        let crystal = crate::crystal::read(ctx, start, |_| Ok([0, 0]))?;
        let mut materials = Vec::new();
        if let Ok(p) = chunk(ctx, start, 0x09003003) {
            let mut r = cursor(ctx.buf, p)?;
            ensure!(r.u32()? == 2, "crystal materials version");
            for _ in 0..r.len()? {
                let name = r.string()?;
                let id = if !name.is_empty() {
                    self.material_or_diagnostic(&name)?
                } else {
                    let m = reference(ctx, &mut r)?;
                    self.material_ref(ctx, m)?
                };
                materials.push(id);
            }
        }
        let default = self.intern_unassigned(ctx.path);
        let mut meshes = BTreeMap::<usize, Mesh>::new();
        for g in &crystal.visual_geometry {
            for (fi, (face, slot)) in g.faces.iter().enumerate() {
                ensure!(
                    face.len() == 3 || face.len() == 4,
                    "crystal polygon {} requires compilation",
                    face.len()
                );
                let material = if *slot < 0 {
                    default
                } else {
                    *materials.get(*slot as usize).context("crystal material slot")?
                };
                let mesh = meshes.entry(material).or_insert_with(|| Mesh {
                    name: format!("{}#crystal{start}/material{slot}", ctx.path),
                    uvs: vec![Vec::new()],
                    ..Default::default()
                });
                let tris: &[[usize; 3]] =
                    if face.len() == 3 { &[[0, 1, 2]] } else { &[[0, 1, 3], [1, 2, 3]] };
                for tri in tris {
                    let [a, b, c] = tri.map(|j| g.positions[face[j] as usize]);
                    let ab = std::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
                    let ac = std::array::from_fn::<_, 3, _>(|i| c[i] - a[i]);
                    let normal = [
                        ab[1] * ac[2] - ab[2] * ac[1],
                        ab[2] * ac[0] - ab[0] * ac[2],
                        ab[0] * ac[1] - ab[1] * ac[0],
                    ];
                    let len = normal.iter().map(|v| v * v).sum::<f32>().sqrt();
                    let normal = normal.map(|v| if len > 0. { v / len } else { 0. });
                    for &j in tri {
                        mesh.indices.push(mesh.positions.len() as u32);
                        mesh.positions.push(g.positions[face[j] as usize]);
                        mesh.normals.push(normal);
                        mesh.uvs[0].push(g.face_uvs[fi][j]);
                    }
                }
            }
        }
        let mut parts = Vec::new();
        for (material, mut mesh) in meshes {
            mesh.submeshes.push(Submesh {
                first_index: 0,
                index_count: mesh.indices.len() as u32,
                material,
                lod_mask: 1,
            });
            let id = self.meshes.len();
            self.meshes.push(mesh);
            parts.push(Part { mesh: id, chain: Vec::new() });
        }
        Ok(parts)
    }
}

impl VisualCache {
    fn tree_material(&mut self, path: &str, slot: usize, refs: &[NodeLink<usize>]) -> MaterialId {
        let mut mat = Material::default();
        for (k, sampler, role) in [
            (0, "Diffuse", TextureRole::Albedo),
            (1, "Normal", TextureRole::Normal),
            (2, "Roughness", TextureRole::Specular),
        ] {
            let mut found = None;
            for r in [refs.get(k + 3), refs.get(k)].into_iter().flatten() {
                let NodeLink::External(p) = r else { continue };
                let lower = p.to_ascii_lowercase();
                found = if lower.ends_with(".dds") || lower.ends_with(".png") || lower.ends_with(".webp") {
                    self.texture(p).ok()
                } else {
                    let mut child = Material::default();
                    self.texture_graph(p, &mut child, 1, &mut Default::default())
                        .ok()
                        .and_then(|()| child.textures.first().map(|b| b.texture))
                };
                if found.is_some() {
                    break;
                }
            }
            match found {
                Some(texture) => mat.textures.push(TextureBinding {
                    sampler: sampler.into(),
                    role,
                    texture,
                    bitmap_usage: None,
                }),
                None if k == 0 => {
                    self.report("tree-material", path, format!("slot {slot}: no diffuse texture"))
                }
                None => {}
            }
        }
        self.intern_material(format!("{path}#tree-material{slot}"), mat)
    }
    fn tree(&mut self, ctx: &Ctx<'_>) -> Result<Vec<Part>> {
        let mut r = cursor(ctx.buf, ctx.layout.head.body_start)?;
        ensure!(r.u32()? == 21, "vegetation visual version");
        r.u32()?;
        let lods = r.len()?;
        ensure!(lods <= 3, "tree LOD count");
        let leaves = r.len()?;
        let mut materials = Vec::new();
        for _ in 0..r.len()? {
            let textures = (0..8).map(|_| reference(ctx, &mut r)).collect::<Result<Vec<_>>>()?;
            let m = reference(ctx, &mut r)?;
            materials.push(if matches!(m, NodeLink::Null) {
                self.tree_material(ctx.path, materials.len(), &textures)
            } else {
                self.material_ref(ctx, m)?
            });
            r.u8()?;
        }
        for _ in 0..r.len()? {
            crate::crystal::id(&mut r)?;
        }
        let mut meshes = Vec::new();
        for lod in 0..lods {
            for _ in 0..r.len()? {
                let slot = r.u16()? as usize;
                let v = reference(ctx, &mut r)?;
                r.u8()?;
                if lod == 0 && !matches!(v, NodeLink::Null) {
                    let mut mesh = visual(ctx, resolve_start(ctx, v)?)?;
                    mesh.name = format!("{}#tree{}", ctx.path, meshes.len());
                    mesh.submeshes.push(Submesh {
                        first_index: 0,
                        index_count: mesh.indices.len() as u32,
                        material: *materials.get(slot).context("tree material index")?,
                        lod_mask: 1,
                    });
                    meshes.push(mesh);
                }
            }
        }
        r.skip(leaves * 4 + 1 + 4 + 1 + 1 + 8)?;
        let params =
            crate::veget::Params { tilt_degrees: r.f32()?, scale_variation: r.f32()?, random_y: r.bool32()? };
        let mut out = Vec::new();
        for mut mesh in meshes {
            mesh.vegetation = Some(params);
            let id = self.meshes.len();
            self.meshes.push(mesh);
            out.push(Part { mesh: id, chain: Vec::new() });
        }
        Ok(out)
    }
}
