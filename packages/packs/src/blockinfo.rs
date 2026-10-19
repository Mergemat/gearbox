use anyhow::{Context, Result, ensure};
use gearbox_gbx::reader::Reader;

use crate::nodes::Layout;
use crate::prefab::{Ctx, NodeLink};

#[derive(Clone, Debug, PartialEq)]
pub struct Mobil {
    pub index: u32,
    pub solid_frequency: u32,
    pub geom_translation: Option<[f32; 3]>,
    pub geom_rotation: Option<[f32; 3]>,
    pub prefab: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum VariantKind {
    Ground,
    Air,
}

#[derive(Clone, Debug, PartialEq, Default)]
pub struct UnitClips {
    pub faces: [Vec<Option<String>>; 6],
    pub directions: [u16; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub struct AutoTerrain {
    pub coord: [i32; 3],
    pub steps: Vec<Option<String>>,
    pub target: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Variant {
    pub kind: VariantKind,
    pub index: usize,
    pub mobils: Vec<Vec<Mobil>>,
    pub units: Vec<[i32; 3]>,
    pub unit_clips: Vec<Vec<String>>,
    pub clip_faces: Vec<UnitClips>,
    pub ground_archive: Option<usize>,
    pub waypoint_trigger: Option<String>,
    pub legacy_waypoint_trigger: Option<usize>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BlockInfo {
    pub name: Option<String>,
    pub class_id: u32,
    pub variants: Vec<Variant>,
    pub material_modifier: Option<String>,
    pub material_modifier2: Option<String>,
    pub clip_type: u32,
    pub symmetrical_clip: Option<String>,
}

pub fn is_clip(class_id: u32) -> bool {
    matches!(class_id, 0x03053000 | 0x03340000 | 0x0335b000)
}

pub fn is_block_info(class_id: u32) -> bool {
    matches!(class_id >> 12, 0x0304f | 0x03050 | 0x0314c | 0x03051 | 0x03052 | 0x03053 | 0x03054 | 0x03340 | 0x0335a | 0x0335b)
}

fn symmetrical_clip(b: &[u8], p: usize) -> Option<String> {
    if b.get(p + 4..p + 8)? != b"PIKS" {
        return None;
    }
    let mut q = p + 16;
    let first = u32_at(b, q)?;
    q += 4;
    if first != u32::MAX && first & 0xc000_0000 != 0 && first & 0x3fff_ffff == 0 {
        q += 4 + u32_at(b, q)? as usize;
    }
    let id = u32_at(b, q)?;
    if id == u32::MAX || id & 0xc000_0000 == 0 || id & 0x3fff_ffff != 0 {
        return None;
    }
    let n = u32_at(b, q + 4)? as usize;
    b.get(q + 8..q + 8 + n).map(|s| String::from_utf8_lossy(s).into_owned())
}

fn u32_at(b: &[u8], p: usize) -> Option<u32> {
    b.get(p..p + 4).map(|s| u32::from_le_bytes(s.try_into().unwrap()))
}

fn find_all(b: &[u8], from: usize, word: u32) -> Vec<usize> {
    let w = word.to_le_bytes();
    let mut v = Vec::new();
    let mut p = from;
    while let Some(o) = b.get(p..).and_then(|s| s.windows(4).position(|x| x == w)) {
        v.push(p + o);
        p += o + 1;
    }
    v
}

fn path_of(r: NodeLink<()>) -> Option<String> {
    match r {
        NodeLink::External(p) => Some(p),
        _ => None,
    }
}

fn mobil(ctx: &Ctx, index: u32, start: usize, end: usize) -> Result<Mobil> {
    let b = ctx.buf;
    let at = find_all(&b[..end.min(b.len())], start, 0x03122003).into_iter().next();
    let mut m = Mobil { index, solid_frequency: 0, geom_translation: None, geom_rotation: None, prefab: None };
    let Some(at) = at else { return Ok(m) };
    let mut r = Reader::new(b);
    r.skip(at + 4)?;
    let version = r.u32()?;
    ensure!(version >= 14 && version < 64, "mobil chunk version {version}");
    m.solid_frequency = r.u32()?;
    if r.u8()? != 0 {
        m.geom_translation = Some(r.vec3()?);
        m.geom_rotation = Some(r.vec3()?);
    }
    let solid = ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
    if solid == NodeLink::Null {
        ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
    }
    m.prefab = path_of(ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?);
    Ok(m)
}

fn ref_span(ctx: &Ctx, r: &mut Reader) -> Result<Option<(usize, u32)>> {
    Ok(match ctx.noderef(r, |ctx, r, class| {
        let start = r.pos();
        ctx.skip_node(r)?;
        Ok((start, class))
    })? {
        NodeLink::Inline(span) => Some(span),
        NodeLink::Earlier(index) => ctx.layout.nodes.iter().find(|n| n.index == index).map(|n| (n.start, n.class_id)),
        _ => None,
    })
}

fn new_id(r: &mut Reader) -> Result<Option<String>> {
    let id = r.u32()?;
    if id != u32::MAX && id & 0xc000_0000 != 0 && id & 0x3fff_ffff == 0 {
        Ok(Some(r.string()?))
    } else { Ok(None) }
}

pub fn auto_terrain(buf: &[u8], layout: &Layout, path: &str, at: usize) -> Result<(Vec<AutoTerrain>, i32)> {
    let ctx = &Ctx { buf, layout, path };
    let mut r = Reader::new(ctx.buf);
    r.skip(at + 4)?;
    ensure!(r.u32()? == 2, "unsupported ground variant archive version");
    ensure!(r.u32()? == 10, "unsupported automatic terrain array version");
    let n = r.u32()?;
    ensure!(n < 65536, "automatic terrain count {n}");
    let mut out = Vec::new();
    for _ in 0..n {
        let span = ref_span(ctx, &mut r)?;
        let Some((p, 0x03120000)) = span else { anyhow::bail!("automatic terrain node not decoded"); };
        let mut a = Reader::new(ctx.buf);
        a.skip(p)?;
        ensure!(a.u32()? == 0x03120001, "unsupported automatic terrain archive");
        let coord = [a.i32()?, a.i32()?, a.i32()?];
        let Some((p, 0x0311d000)) = ref_span(ctx, &mut a)? else { anyhow::bail!("automatic terrain genealogy not decoded"); };
        let mut g = Reader::new(ctx.buf);
        g.skip(p)?;
        ensure!(g.u32()? == 0x0311d002, "unsupported zone genealogy archive");
        let count = g.u32()?;
        ensure!(count < 256, "zone genealogy length {count}");
        let steps = (0..count).map(|_| new_id(&mut g)).collect::<Result<_>>()?;
        g.u32()?;
        g.u32()?;
        let target = new_id(&mut g)?;
        out.push(AutoTerrain { coord, steps, target });
    }
    let offset = r.i32()?;
    Ok((out, offset))
}

fn variant(ctx: &Ctx, kind: VariantKind, index: usize, start: usize, end: usize, mobils: &std::collections::HashMap<u32, Mobil>, own: impl Fn(usize) -> bool) -> Result<Variant> {
    let (buf, layout) = (ctx.buf, ctx.layout);
    let unit_nodes: Vec<&crate::nodes::NodeSpan> = layout
        .nodes
        .iter()
        .filter(|n| n.class_id == 0x03036000 && n.start > start && n.start < end && own(n.start))
        .filter(|n| u32_at(buf, n.start) == Some(0x03036000))
        .collect();
    let units = unit_nodes.iter().map(|n| std::array::from_fn(|k| u32_at(buf, n.start + 16 + 4 * k).unwrap_or(0) as i32)).collect();
    let mut clip_faces = Vec::new();
    for n in &unit_nodes {
        let mut clips = UnitClips::default();
        let mut r = Reader::new(buf);
        r.skip(n.start + 28)?;
        let k = r.u32()?;
        ensure!(k == 0 || k == 4, "{k} legacy unit clips");
        for face in 0..k as usize {
            clips.faces[face].push(path_of(ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?));
        }
        let node_end = n.end.unwrap_or(end).min(end);
        if let Some(p) = find_all(&buf[..node_end], n.start, 0x0303600b).into_iter().next() {
            let mut r = Reader::new(buf);
            r.skip(p + 8)?;
            for face in [5, 4] {
                clips.faces[face] = vec![path_of(ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?)];
            }
            let bottom = r.u32()? as u16;
            let top = r.u32()? as u16;
            clips.directions = [top, bottom];
        }
        if let Some(p) = find_all(&buf[..node_end], n.start, 0x0303600c).into_iter().next() {
            let mut r = Reader::new(buf);
            r.skip(p + 4)?;
            let version = r.u32()?;
            ensure!(version == 2, "unit clips chunk version {version}");
            let counts = r.u32()?;
            for face in 0..6 {
                clips.faces[face].clear();
                for _ in 0..(counts >> (3 * face)) & 7 {
                    clips.faces[face].push(path_of(ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?));
                }
            }
            let bottom = r.u16()?;
            let top = r.u16()?;
            clips.directions = [top, bottom];
        }
        clip_faces.push(clips);
    }
    let unit_clips = clip_faces.iter().map(|c| c.faces.iter().flatten().flatten().cloned().collect()).collect();
    let mut groups = Vec::new();
    {
        let mut q = start + 8;
        let ngroups = u32_at(buf, q).unwrap_or(0) as usize;
        q += 4;
        ensure!(ngroups < 64, "{ngroups} mobil groups");
        for _ in 0..ngroups {
            let n = u32_at(buf, q).context("mobil count")? as usize;
            ensure!(n < 1024, "{n} mobils");
            q += 4;
            let mut list = Vec::new();
            for _ in 0..n {
                let idx = u32_at(buf, q).context("mobil ref")?;
                match layout.by_start.get(&(q + 8)).map(|&i| &layout.nodes[i]) {
                    Some(node) if node.index == idx => q = node.end.context("mobil node without end")?,
                    _ => q += 4,
                }
                if idx != u32::MAX {
                    list.push(mobils.get(&idx).cloned().with_context(|| format!("mobil node {idx}"))?);
                }
            }
            groups.push(list);
        }
    }
    let mut waypoint_trigger = None;
    let mut legacy_waypoint_trigger = None;
    if let Some(p) = find_all(&buf[..end.min(buf.len())], start, 0x0315b006).into_iter().find(|&p| own(p)) {
        let mut r = Reader::new(buf);
        r.skip(p + 4)?;
        let version = r.u32()?;
        ensure!((9..=11).contains(&version), "variant trigger chunk version {version}");
        let mut refs = Vec::new();
        for _ in 0..if version >= 11 { 4 } else { 2 } {
            refs.push(ctx.noderef(&mut r, |c, r, class| {
                let start = r.pos();
                c.skip_node(r)?;
                Ok((class, start))
            })?);
        }
        legacy_waypoint_trigger = match &refs[1] {
            NodeLink::Null => None,
            NodeLink::Inline((0x09005000, start)) => Some(*start),
            NodeLink::Earlier(index) => {
                let node = layout.nodes.iter().find(|n| n.index == *index).context("legacy trigger node")?;
                ensure!(node.class_id == 0x09005000, "legacy trigger solid class {:#x}", node.class_id);
                Some(node.start)
            }
            _ => anyhow::bail!("legacy trigger solid reference not decoded"),
        };
        waypoint_trigger = refs.get(3).and_then(|r| match r { NodeLink::External(p) => Some(p.clone()), _ => None });
    }
    let ground_chunk = find_all(&buf[..end.min(buf.len())], start, 0x0315c001).into_iter().find(|&p| own(p));
    Ok(Variant { kind, index, mobils: groups, units, unit_clips, clip_faces, ground_archive: ground_chunk, waypoint_trigger, legacy_waypoint_trigger })
}

pub(crate) fn nested_blocks(buf: &[u8], layout: &Layout) -> Vec<(usize, usize)> {
    layout.nodes[1..]
        .iter()
        .filter(|n| is_block_info(n.class_id))
        .filter_map(|n| {
            let c = find_all(buf, n.start, 0x0304e031).into_iter().next()?;
            let e = buf[c..].windows(4).position(|w| w == 0xfacade01u32.to_le_bytes())?;
            Some((n.start, c + e + 4))
        })
        .collect()
}

pub fn read_block_info(buf: &[u8], layout: &Layout, path: &str) -> Result<BlockInfo> {
    let ctx = Ctx { buf, layout, path };
    let body = layout.head.body_start;
    let mobil_nodes: Vec<(u32, usize)> =
        layout.nodes.iter().filter(|n| n.class_id == 0x03122000).map(|n| (n.index, n.start)).collect();
    let bound = |start: usize| -> usize {
        layout.nodes.iter().map(|n| n.start).filter(|&s| s > start).min().unwrap_or(buf.len())
    };
    let mut mobils = std::collections::HashMap::new();
    for &(i, s) in &mobil_nodes {
        mobils.insert(i, mobil(&ctx, i, s, bound(s))?);
    }
    let nested = nested_blocks(buf, layout);
    let own = |p: usize| !nested.iter().any(|&(s, e)| s <= p && p < e);
    let count_at = |chunk: u32| -> usize {
        find_all(buf, body, chunk)
            .into_iter()
            .find(|&p| own(p))
            .and_then(|p| match u32_at(buf, p + 4)? {
                10 => u32_at(buf, p + 8),
                n => Some(n),
            })
            .unwrap_or(0) as usize
    };
    let extra_ground = count_at(0x0304e027);
    let extra_air = count_at(0x0304e02c);
    let mut kinds = vec![(VariantKind::Ground, 0), (VariantKind::Air, 0)];
    kinds.extend((1..=extra_ground).map(|i| (VariantKind::Ground, i)));
    kinds.extend((1..=extra_air).map(|i| (VariantKind::Air, i)));
    let v005: Vec<usize> = find_all(buf, body, 0x0315b005).into_iter().filter(|&p| own(p)).collect();
    ensure!(v005.len() == kinds.len(), "{} variant mobil chunks for {} variants", v005.len(), kinds.len());
    let mut variants = Vec::new();
    for (k, &(kind, index)) in kinds.iter().enumerate() {
        let end = v005.get(k + 1).copied().unwrap_or(buf.len());
        variants.push(variant(&ctx, kind, index, v005[k], end, &mobils, own)?);
    }
    let (mut mm, mut mm2) = (None, None);
    if let Some(p) = find_all(buf, body, 0x0304e031).into_iter().filter(|&p| own(p)).last() {
        let mut r = Reader::new(buf);
        r.skip(p + 4)?;
        let v = r.u32()?;
        if v <= 4 {
            ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
            mm = path_of(ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?);
            if v >= 1 {
                mm2 = path_of(ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?);
            }
        }
    }
    let clip_type = if is_clip(layout.head.class_id) {
        find_all(buf, body, 0x03053005).into_iter().find(|&p| own(p)).and_then(|p| u32_at(buf, p + 4)).unwrap_or(0)
    } else {
        0
    };
    let symmetrical_clip = if is_clip(layout.head.class_id) {
        find_all(buf, body, 0x03053008).into_iter().find(|&p| own(p)).and_then(|p| symmetrical_clip(buf, p))
    } else {
        None
    };
    let container = gearbox_gbx::Container::parse(buf)?;
    let name = if let Some(chunk) = container.header_chunk(0x2e001003) {
        let mut r = Reader::new(chunk);
        r.ident()?[0].clone()
    } else { None };
    Ok(BlockInfo { name, class_id: layout.head.class_id, variants, material_modifier: mm, material_modifier2: mm2, clip_type, symmetrical_clip })
}
