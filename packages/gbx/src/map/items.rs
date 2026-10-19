use super::{CLASS_WAYPOINT, Item, waypoint_property};
use crate::reader::Reader;
use anyhow::{Result, bail, ensure};
const CLASS_ITEM: u32 = 0x0310_1000;

pub(super) fn read(r: &mut Reader, block_count: usize) -> Result<Vec<Item>> {
    let version = r.u32()?;
    ensure!(version <= 8, "items chunk version {version}");
    let mut e = r.encapsulated()?;
    ensure!(e.u32()? == 10, "unexpected item list version");
    let mut items = Vec::new();
    for _ in 0..e.len()? {
        let class = e.u32()?;
        ensure!(class == CLASS_ITEM, "item class 0x{class:08X}");
        items.push(anchored_object(&mut e)?);
    }
    if version >= 1 && version != 5 && version < 8 {
        for _ in 0..e.len()? {
            let parent = index(e.i32()?, items.len())?;
            let child = index(e.i32()?, items.len())?.ok_or_else(|| anyhow::anyhow!("null placed item"))?;
            items[child].placed_on_item = parent;
        }
    }
    if version >= 5 {
        let blocks = indices(&mut e, block_count)?;
        let early_groups = if version < 7 { Some(ints(&mut e)?) } else { None };
        let parents = if version >= 6 { Some(indices(&mut e, items.len())?) } else { None };
        let groups = match early_groups {
            Some(g) => g,
            None => ints(&mut e)?,
        };
        if version != 6 {
            let reserved = ints(&mut e)?;
            ensure!(reserved.iter().all(|&x| x == -1), "unsupported item snap metadata");
        }
        let snapped = indices(&mut e, blocks.len())?;
        ensure!(snapped.len() == items.len(), "item snapping count differs from item count");
        ensure!(groups.len() == blocks.len(), "item snap group count differs from block count");
        if let Some(p) = &parents {
            ensure!(p.len() == blocks.len(), "item snap parent count differs from block count");
        }
        for (item, snap) in items.iter_mut().zip(snapped) {
            if let Some(snap) = snap {
                item.snapped_on_block = blocks[snap];
                item.snapped_on_item = parents.as_ref().and_then(|p| p[snap]);
                item.snapped_on_group = Some(groups[snap]);
            }
        }
    }
    ensure!(e.remaining() == 0, "unread items data: {} bytes", e.remaining());
    Ok(items)
}

fn index(value: i32, count: usize) -> Result<Option<usize>> {
    ensure!(
        value == -1 || (value >= 0 && (value as usize) < count),
        "item reference {value} outside {count} entries"
    );
    Ok((value != -1).then_some(value as usize))
}
fn indices(r: &mut Reader, count: usize) -> Result<Vec<Option<usize>>> {
    (0..r.len()?).map(|_| index(r.i32()?, count)).collect()
}
fn ints(r: &mut Reader) -> Result<Vec<i32>> {
    (0..r.len()?).map(|_| r.i32()).collect()
}

fn anchored_object(r: &mut Reader) -> Result<Item> {
    let mut item = None;
    r.chunks("CGameCtnAnchoredObject", |r, id, _| {
        match id {
            0x0310_1002 => {
                let version = r.u32()?;
                ensure!((4..=8).contains(&version), "item version {version}");
                let [model, collection, author] = r.ident()?;
                let yaw_pitch_roll = r.vec3()?;
                let block_coord = [r.u8()?, r.u8()?, r.u8()?];
                r.id()?;
                let position = r.vec3()?;
                let waypoint = match r.u32()? {
                    u32::MAX => None,
                    CLASS_WAYPOINT => Some(waypoint_property(r)?),
                    class => bail!("item waypoint class 0x{class:08X}"),
                };
                let flags = r.u16()?;
                let pivot = if version >= 5 { r.vec3()? } else { [0.0; 3] };
                let scale = if version >= 6 { r.f32()? } else { 1.0 };
                if version >= 7 && flags & 4 != 0 {
                    r.pack_desc()?;
                }
                if version >= 8 {
                    r.vec3()?;
                    r.vec3()?;
                }
                item = Some(Item {
                    model,
                    collection,
                    author,
                    yaw_pitch_roll,
                    block_coord,
                    position,
                    pivot,
                    scale,
                    flags,
                    anim_phase: 0,
                    waypoint,
                    placed_on_item: None,
                    snapped_on_block: None,
                    snapped_on_item: None,
                    snapped_on_group: None,
                });
            }
            _ => return Ok(false),
        }
        Ok(true)
    })?;
    item.ok_or_else(|| anyhow::anyhow!("item without chunk 0x03101002"))
}
