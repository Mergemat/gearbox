use crate::prefab::{Ctx, NodeLink, StaticObject};
use anyhow::{Context, Result, ensure};

pub struct Mobil {
    pub id: u32,
    pub crystal: Option<usize>,
    pub object: Option<NodeLink<StaticObject>>,
    pub trigger: Option<NodeLink<crate::surface::Surface>>,
}
pub struct BlockItem {
    pub archetype: String,
    pub collection: u32,
    pub mobils: Vec<Mobil>,
    pub compiled: bool,
}

pub fn mobil_key(id: u32) -> (bool, usize, usize, usize) {
    let v = id / 1_000_000;
    (v % 10 == 0, (id / 1000 % 1000) as usize, (v / 10 % 100) as usize, (id % 1000) as usize)
}

pub fn read(ctx: &Ctx<'_>, start: usize) -> Result<BlockItem> {
    let mut r = crate::crystal::cursor(ctx.buf, start)?;
    ensure!(r.u32()? == 0x2e025000, "block item chunk");
    let version = r.u32()?;
    ensure!(version <= 1, "block item version {version}");
    let (_, archetype) = crate::crystal::id(&mut r)?;
    let archetype = archetype.context("block archetype id lookback not decoded")?;
    let (collection, _) = crate::crystal::id(&mut r)?;
    let mut mobils = Vec::new();
    for _ in 0..r.len()? {
        let id = r.u32()?;
        let crystal = ctx.noderef(&mut r, |c, r, class| {
            ensure!(class == 0x09003000, "block crystal class {class:#x}");
            let start = r.pos();
            c.skip_node(r)?;
            Ok(start)
        })?;
        let crystal = match crystal {
            NodeLink::Null => None,
            NodeLink::Inline(p) => Some(p),
            NodeLink::Earlier(i) => {
                Some(ctx.layout.nodes.iter().find(|n| n.index == i).context("block crystal node")?.start)
            }
            NodeLink::External(_) => anyhow::bail!("external block crystal"),
        };
        mobils.push(Mobil { id, crystal, object: None, trigger: None });
    }
    let compiled = version >= 1 && r.bool8()?;
    if compiled {
        for mobil in &mut mobils {
            let flags = r.u8()?;
            if flags & 1 != 0 {
                mobil.object = Some(ctx.noderef(&mut r, |c, r, class| {
                    ensure!(class == 0x09159000, "custom mobil static class {class:#x}");
                    c.static_object(r)
                })?);
            }
            if flags & 2 != 0 {
                mobil.trigger = Some(ctx.noderef(&mut r, |c, r, class| {
                    ensure!(class == 0x0900c000, "custom mobil trigger class {class:#x}");
                    c.surface(r)
                })?);
            }
            if flags & 4 != 0 {
                r.skip(24)?;
            }
            if flags & 8 != 0 {
                r.skip(12)?;
            }
            ensure!(flags & !15 == 0, "custom mobil flags {flags:#x}");
        }
    }
    Ok(BlockItem { archetype, collection, mobils, compiled })
}
