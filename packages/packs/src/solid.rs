use anyhow::{Context, Result, bail, ensure};
use gearbox_gbx::reader::Reader;

use crate::prefab::{Ctx, NodeLink};
use crate::surface::{MeshTri, Surf, SurfNode, Surface};

fn cursor<'a>(b: &'a [u8], p: usize) -> Result<Reader<'a>> {
    let mut r = Reader::new(b);
    r.skip(p)?;
    Ok(r)
}

fn chunk(b: &[u8], start: usize, end: usize, id: u32) -> Result<usize> {
    b.get(start..end)
        .context("node bounds")?
        .windows(4)
        .position(|w| w == id.to_le_bytes())
        .map(|p| start + p + 4)
        .with_context(|| format!("missing chunk {id:#010x}"))
}

fn skip_id(r: &mut Reader) -> Result<()> {
    let mut id = r.u32()?;
    if id == 3 {
        id = r.u32()?;
    }
    if id != u32::MAX && id & 0xc0000000 != 0 && id & 0x3fffffff == 0 {
        r.string()?;
    }
    Ok(())
}

fn node<'a>(ctx: &'a Ctx<'_>, index: u32) -> Result<&'a crate::nodes::NodeSpan> {
    ctx.layout.nodes.iter().find(|n| n.index == index).with_context(|| format!("missing visual node {index}"))
}

fn reference(r: &mut Reader, ctx: &Ctx<'_>) -> Result<u32> {
    let index = r.u32()?;
    let n = node(ctx, index)?;
    if r.pos() + 4 == n.start {
        r.skip(n.end.context("visual node without end")? - r.pos())?;
    }
    Ok(index)
}

fn positions(ctx: &Ctx<'_>, index: u32) -> Result<Vec<[f32; 3]>> {
    let n = node(ctx, index)?;
    ensure!(n.class_id == 0x09056000, "position stream class {:#x}", n.class_id);
    let mut r = cursor(ctx.buf, n.start)?;
    ensure!(r.u32()? == 0x09056000, "vertex stream chunk");
    let version = r.u32()?;
    ensure!((1..=2).contains(&version), "vertex stream version {version}");
    let count = r.len()?;
    r.u32()?;
    ensure!(r.u32()? == u32::MAX, "shared vertex stream not decoded");
    let mut decls = Vec::new();
    for _ in 0..r.len()? {
        let flags = r.u32()?;
        let flags2 = r.u32()?;
        if flags2 & 0xffc != 0 {
            r.skip(4)?;
        }
        decls.push((flags & 0x1ff, flags >> 9 & 0x1ff));
    }
    r.bool32()?;
    let mut out = None;
    for (semantic, ty) in decls {
        if semantic == 0 && ty == 2 {
            out = Some((0..count).map(|_| r.vec3()).collect::<Result<_>>()?);
        } else {
            let size = match ty {
                1 => 8,
                2 if matches!(semantic, 5 | 6) => 4,
                2 => 12,
                4 | 5 | 14 => 4,
                _ => bail!("vertex declaration type {ty}, semantic {semantic}"),
            };
            r.skip(size * count)?;
        }
    }
    out.context("vertex stream without positions")
}

fn visual(ctx: &Ctx<'_>, index: u32) -> Result<(Vec<[f32; 3]>, Vec<u32>)> {
    let n = node(ctx, index)?;
    ensure!(n.class_id == 0x0901e000, "collision visual class {:#x}", n.class_id);
    let end = n.end.context("visual without end")?;
    let mut r = cursor(ctx.buf, chunk(ctx.buf, n.start, end, 0x0900600f)?)?;
    let version = r.u32()?;
    ensure!((4..=6).contains(&version), "visual version {version}");
    let flags = r.u32()?;
    ensure!(flags & 7 == 0, "skinned collision visual");
    r.u32()?;
    let count = r.len()?;
    if count == 0 {
        return Ok((Vec::new(), Vec::new()));
    }
    let streams = r.len()?;
    ensure!(streams == 1, "collision visual has {streams} vertex streams");
    let stream = reference(&mut r, ctx)?;
    let vertices = positions(ctx, stream)?;
    ensure!(vertices.len() == count, "visual/stream vertex count differs");
    let mut r = cursor(ctx.buf, chunk(ctx.buf, n.start, end, 0x0906a001)?)?;
    ensure!(r.bool32()?, "collision visual without index buffer");
    let kind = r.u32()?;
    ensure!(matches!(kind, 0x09057000 | 0x09057001), "index buffer chunk {kind:#x}");
    r.u32()?;
    let mut indices = Vec::new();
    let mut last = 0i32;
    for _ in 0..r.len()? {
        let v = if kind == 0x09057001 { last.wrapping_add(r.i16()? as i32) } else { r.u16()? as i32 };
        last = v;
        indices.push(v as u16 as u32);
    }
    ensure!(
        indices.len() % 3 == 0 && indices.iter().all(|&i| (i as usize) < vertices.len()),
        "invalid visual triangle indices"
    );
    Ok((vertices, indices))
}

pub fn collision(
    ctx: &Ctx<'_>,
    start: usize,
    mut resolve: impl FnMut(&str) -> Result<[u8; 2]>,
) -> Result<Surface> {
    let mut r = cursor(ctx.buf, start)?;
    ensure!(r.u32()? == 0x090bb000, "Solid2 chunk");
    let version = r.u32()?;
    ensure!((29..=34).contains(&version), "Solid2 version {version}");
    skip_id(&mut r)?;
    let mut geoms = Vec::new();
    for _ in 0..r.len()? {
        let visual = r.u32()?;
        let material = r.u32()?;
        let flags = r.u32()?;
        r.u32()?;
        if version >= 32 {
            r.u32()?;
        }
        geoms.push((visual, material, flags));
    }
    ensure!(r.u32()? == 10, "visual array version");
    let mut visuals = Vec::new();
    for _ in 0..r.len()? {
        visuals.push(reference(&mut r, ctx)?);
    }
    for _ in 0..r.len()? {
        skip_id(&mut r)?;
    }
    let material_count = r.len()?;
    let refs = if material_count > 0 {
        custom_materials(ctx, &mut r, version, material_count)?
    } else {
        ensure!(r.u32()? == 10, "material array version");
        (0..r.len()?).map(|_| node_link(ctx, &mut r)).collect::<Result<Vec<_>>>()?
    };
    let materials =
        refs.into_iter().map(|m| material_ids(ctx, m, &mut resolve)).collect::<Result<Vec<_>>>()?;
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    let mut has_geometry = false;
    for (vi, mi, flags) in geoms {
        if flags & 1 == 0 {
            continue;
        }
        let ids = *materials.get(mi as usize).context("shaded geometry material index")?;
        if ids[0] == 28 {
            continue;
        }
        let (vs, indices) = visual(ctx, *visuals.get(vi as usize).context("shaded geometry visual index")?)?;
        has_geometry = true;
        let base = vertices.len() as i32;
        vertices.extend(vs.into_iter().map(|[x, y, z]| {
            [
                ((y * 0.0 + x * 1.0) + z * 0.0) + 0.0,
                ((x * 0.0 + y * 1.0) + z * 0.0) + 0.0,
                ((x * 0.0 + y * 0.0) + z * 1.0) + 0.0,
            ]
        }));
        triangles.extend(indices.as_chunks::<3>().0.iter().map(|t| MeshTri {
            v: [base + t[0] as i32, base + t[1] as i32, base + t[2] as i32],
            flags: ids,
            material: 0,
        }));
    }
    Ok(Surface {
        version: 4,
        surf_version: 2,
        surf: has_geometry.then_some(SurfNode {
            kind: 7,
            surf: Surf::Mesh { version: 7, vertices, triangles },
            gameplay_main_dir: Some([0.0, 0.0, 1.0]),
        }),
        materials: Vec::new(),
        material_ids: Vec::new(),
    })
}

fn material_ids(
    ctx: &Ctx<'_>,
    material: NodeLink<usize>,
    resolve: &mut impl FnMut(&str) -> Result<[u8; 2]>,
) -> Result<[u8; 2]> {
    let start = match material {
        NodeLink::External(path) => return resolve(&path),
        NodeLink::Inline(start) => start,
        NodeLink::Earlier(index) => node(ctx, index)?.start,
        NodeLink::Null => bail!("null collision material"),
    };
    let n = ctx.layout.nodes.iter().find(|n| n.start == start).context("collision material node")?;
    let ids = match n.class_id {
        0x090fd000 => crate::material::user_material_ids(ctx.buf, start)?,
        0x09079000 => crate::material::material_ids(&ctx.buf[start..n.end.context("material end")?])?,
        class => bail!("collision material class {class:#x}"),
    };
    Ok([ids.surface, ids.gameplay])
}

pub fn external_materials(ctx: &Ctx<'_>, start: usize) -> Result<Vec<String>> {
    let mut r = cursor(ctx.buf, start)?;
    ensure!(r.u32()? == 0x090bb000, "Solid2 chunk");
    let version = r.u32()?;
    ensure!((29..=34).contains(&version), "Solid2 version {version}");
    skip_id(&mut r)?;
    let n = r.len()?;
    r.skip(n * if version >= 32 { 20 } else { 16 })?;
    ensure!(r.u32()? == 10, "visual array version");
    for _ in 0..r.len()? {
        reference(&mut r, ctx)?;
    }
    for _ in 0..r.len()? {
        skip_id(&mut r)?;
    }
    if r.u32()? != 0 {
        return Ok(Vec::new());
    }
    ensure!(r.u32()? == 10, "material array version");
    let mut out = Vec::new();
    for _ in 0..r.len()? {
        match ctx.noderef(&mut r, |c, r, _| c.skip_node(r))? {
            crate::prefab::NodeLink::External(p) => out.push(p),
            crate::prefab::NodeLink::Null => {}
            _ => bail!("unsupported inline Solid2 material modifier traversal"),
        }
    }
    Ok(out)
}

fn node_link(ctx: &Ctx<'_>, r: &mut Reader) -> Result<NodeLink<usize>> {
    ctx.noderef(r, |c, r, _| {
        let start = r.pos();
        c.skip_node(r)?;
        Ok(start)
    })
}

fn skip_reference(ctx: &Ctx<'_>, r: &mut Reader) -> Result<()> {
    ctx.noderef(r, |c, r, _| c.skip_node(r))?;
    Ok(())
}

pub(crate) fn custom_materials(
    ctx: &Ctx<'_>,
    r: &mut Reader,
    version: u32,
    count: usize,
) -> Result<Vec<NodeLink<usize>>> {
    skip_skeleton(ctx, r, version)?;
    (0..count)
        .map(|_| {
            let name = r.string()?;
            if name.is_empty() { node_link(ctx, r) } else { Ok(NodeLink::External(name)) }
        })
        .collect()
}

fn skip_skeleton(ctx: &Ctx<'_>, r: &mut Reader, version: u32) -> Result<()> {
    skip_reference(ctx, r)?;
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
        crate::crystal::id(r)?;
        if r.bool32()? {
            skip_reference(ctx, r)?;
        } else {
            r.string()?;
        }
        r.skip(48 + 6 * 4)?;
        if r.bool32()? {
            r.skip(3 * 4)?;
        }
    }
    for _ in 0..r.len()? {
        skip_reference(ctx, r)?;
    }
    let n = r.len()?;
    r.skip(n * 8)?;
    r.skip(3 * 4)?;
    r.string()?;
    if version >= 30 {
        r.u32()?;
    }
    Ok(())
}
