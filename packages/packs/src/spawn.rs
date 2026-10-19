use anyhow::{Context, Result, ensure};
use gearbox_gbx::reader::Reader;
#[path = "spawn_crystal.rs"]
mod crystal;
use crate::{blockinfo::{self, VariantKind}, nodes, prefab::Ctx};

#[derive(Clone, Copy, Debug)]
pub struct BlockSpawn {
    pub position: [f32; 3],
    pub yaw_pitch_roll: [f32; 3],
}

pub fn block_spawn(file: &[u8], path: &str, ground: bool, index: usize) -> Result<BlockSpawn> {
    let buf = crate::embedded::plain_gbx(file)?;
    let layout = nodes::layout(&buf)?;
    let block = blockinfo::read_block_info(&buf, &layout, path)?;
    let kind = if ground { VariantKind::Ground } else { VariantKind::Air };
    let selected = block.variants.iter().position(|v| v.kind == kind && v.index == index).context("spawn variant absent")?;
    let chunk = 0x0315_b008u32.to_le_bytes();
    let chunks: Vec<usize> = buf[layout.head.body_start..].windows(4).enumerate().filter(|(_, w)| *w == chunk).map(|(i, _)| layout.head.body_start + i).collect();
    ensure!(chunks.len() == block.variants.len(), "spawn/unit chunks {} != variants {}", chunks.len(), block.variants.len());
    let mut r = Reader::new(&buf);
    r.skip(chunks[selected] + 4)?;
    let version = r.u32()?;
    ensure!(version <= 2, "spawn/unit chunk version {version}");
    let ctx = Ctx { buf: &buf, layout: &layout, path };
    let n = r.u32()?;
    ensure!(n <= 4096, "spawn/unit count {n}");
    for _ in 0..n { ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?; }
    r.u32()?;
    for _ in 0..4 { ensure!(r.u32()? <= 1, "invalid symmetry flag"); }
    let position = r.vec3()?;
    let yaw_pitch_roll = [r.f32()?, r.f32()?, if version >= 2 { r.f32()? } else { 0.0 }];
    Ok(BlockSpawn { position, yaw_pitch_roll })
}

pub fn item_landmark(file: &[u8]) -> Result<[u32; 12]> {
    item_waypoint(file)
}

const IDENTITY: [u32; 12] = [
    1.0f32.to_bits(), 0, 0, 0, 1.0f32.to_bits(), 0, 0, 0, 1.0f32.to_bits(), 0, 0, 0,
];

fn item_waypoint(file: &[u8]) -> Result<[u32; 12]> {
    let buf = crate::embedded::plain_gbx(file)?;
    let layout = nodes::layout(&buf)?;
    let models: Vec<_> = layout.nodes.iter().filter(|n| n.class_id == 0x0917_a000).collect();
    if models.is_empty() && layout.head.class_id == 0x2e00_2000 {
        let chunk = 0x2e00_201fu32.to_le_bytes();
        let chunks: Vec<_> = buf[layout.head.body_start..].windows(4).enumerate().filter(|(_, w)| *w == chunk).map(|(i, _)| layout.head.body_start + i).collect();
        ensure!(chunks.len() == 1, "expected one item waypoint chunk");
        let mut r = Reader::new(&buf);
        r.skip(chunks[0] + 4)?;
        if r.peek_u32() == Some(0x534b_4950) { r.skip(8)?; }
        let version = r.u32()?;
        ensure!(matches!(version, 11..=13), "unsupported item waypoint chunk version {version}");
        let kind = r.u32()?;
        ensure!(kind <= 5, "item waypoint type {kind}");
        let common: Vec<_> = layout.nodes.iter().filter(|n| n.class_id == 0x2e02_7000).collect();
        if common.is_empty() {
            let edition = layout.nodes.iter().any(|n| n.class_id == 0x2e02_6000);
            ensure!(edition, "item has no supported entity model");
            let source = layout.nodes.iter().find(|n| n.class_id == 0x0900_3000)
                .context("edition waypoint has no crystal")?;
            let ctx = Ctx { buf: &buf, layout: &layout, path: "" };
            return Ok(crystal::spawn(&ctx, source.start)?.unwrap_or(IDENTITY));
        }
        ensure!(common.len() == 1, "expected one common item entity model");
        r = Reader::new(&buf);
        r.skip(common[0].start)?;
        ensure!(r.u32()? == 0x2e02_7000, "common item entity chunk");
        let version = r.u32()?;
        ensure!(matches!(version, 4..=6), "unsupported common item entity version {version}");
        let ctx = Ctx { buf: &buf, layout: &layout, path: "" };
        ctx.noderef(&mut r, |c, r, class| c.model(r, class).map(|_| ()))?;
        ctx.noderef(&mut r, |c, r, class| c.model(r, class).map(|_| ()))?;
        let mut trigger_pose = [0; 12];
        for x in &mut trigger_pose { *x = r.u32()?; }
        return Ok(trigger_pose);
    }
    ensure!(models.len() == 1, "expected one inline spawn model, found {} (classes {:x?})", models.len(), layout.nodes.iter().map(|n| n.class_id).collect::<Vec<_>>());
    let mut r = Reader::new(&buf);
    r.skip(models[0].start)?;
    ensure!(r.u32()? == models[0].class_id, "spawn model chunk");
    let version = r.u32()?;
    ensure!(version <= 3, "spawn model version {version}");
    let mut pose = [0; 12];
    for x in &mut pose { *x = r.u32()?; }
    Ok(pose)
}

pub fn block_item_waypoint(file: &[u8], ground: bool, variant: usize) -> Result<(String, Option<[u32; 12]>)> {
    let buf = crate::embedded::plain_gbx(file)?;
    let layout = nodes::layout(&buf)?;
    let node = layout.nodes.iter().find(|n| n.class_id == 0x2e02_5000)
        .context("custom block has no block-item model")?;
    let mut r = Reader::new(&buf);
    r.skip(node.start)?;
    ensure!(r.u32()? == 0x2e02_5000, "block-item chunk");
    let version = r.u32()?;
    ensure!(version <= 1, "block-item version {version}");
    let name = r.u32()?;
    ensure!(matches!(name, 0x4000_0000 | 0x8000_0000), "unsupported block archetype lookback {name:#x}");
    let name = r.string()?;
    let collection = r.u32()?;
    ensure!(collection == 26, "custom block archetype collection {collection}");
    let ctx = Ctx { buf: &buf, layout: &layout, path: "" };
    let n = r.u32()?;
    ensure!(n < 4096, "custom block mobil count {n}");
    let mut selected = None;
    for _ in 0..n {
        let id = r.u32()?;
        let source = ctx.noderef(&mut r, |c, r, class| {
            ensure!(class == 0x0900_3000, "custom block crystal class {class:#x}");
            let start = r.pos();
            c.skip_node(r)?;
            Ok(start)
        })?;
        if (id / 1_000_000 % 10 == 0) == ground && (id / 1000 % 1000) as usize == variant {
            ensure!(selected.is_none(), "multiple custom block mobils for a waypoint");
            selected = Some(source);
        }
    }
    ensure!(version == 0 || r.u8()? == 0, "unsupported custom block waypoint properties");
    let source = match selected.context("custom block waypoint mobil missing")? {
        crate::prefab::NodeLink::Inline(p) => Some(p),
        crate::prefab::NodeLink::Earlier(i) => Some(layout.nodes.iter().find(|n| n.index == i).context("custom block shared crystal")?.start),
        crate::prefab::NodeLink::Null => None,
        crate::prefab::NodeLink::External(p) => anyhow::bail!("external custom block crystal {p}"),
    };
    Ok((name, source.map(|p| crystal::spawn(&ctx, p)).transpose()?.flatten()))
}
