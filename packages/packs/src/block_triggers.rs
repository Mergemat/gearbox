use crate::{
    blockinfo::{BlockInfo, VariantKind},
    catalog::{Block, Catalog, Ent, EntModel, Index, Prefab, SurfaceInfo, surface_materials},
    packs::PackSet,
    prefab::Ctx,
    surface::{Surf, SurfNode, Surface},
};
use anyhow::{Context, Result, ensure};
use gearbox_gbx::reader::Reader;

fn solid_surface(ctx: &Ctx<'_>, start: usize) -> Result<Surface> {
    let solid = ctx.layout.nodes.iter().find(|n| n.start == start).context("trigger solid span")?;
    let end = solid.end.context("trigger solid without end")?;
    let nodes: Vec<_> = ctx.layout.nodes.iter().filter(|n| n.start > start && n.start < end).collect();
    let trees: Vec<_> = nodes.iter().filter(|n| n.class_id == 0x0904f000).collect();
    let surfaces: Vec<_> = nodes.iter().filter(|n| n.class_id == 0x0900c000).collect();
    ensure!(trees.len() == 1 && surfaces.len() == 1, "legacy trigger solid tree layout not decoded");
    let mut r = Reader::new(ctx.buf);
    r.skip(trees[0].start)?;
    ensure!(
        r.u32()? == 0x0904f006 && r.u32()? == 10 && r.u32()? == 0,
        "legacy trigger tree transform not decoded"
    );
    ensure!(
        surfaces[0].start < trees[0].end.context("trigger tree without end")?,
        "trigger surface outside tree"
    );
    let mut r = Reader::new(ctx.buf);
    r.skip(surfaces[0].start)?;
    let mut surface = ctx.surface(&mut r)?;
    let child = surface.surf.take().context("legacy trigger without surface")?;
    surface.surf = Some(SurfNode {
        kind: 13,
        surf: Surf::Compound {
            surfs: vec![child],
            locs: vec![[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]],
            joints: vec![0],
        },
        gameplay_main_dir: Some([0.0, 0.0, 1.0]),
    });
    Ok(surface)
}

pub(crate) struct LegacyTrigger {
    pub ground: bool,
    pub index: usize,
    pub start: usize,
    pub surface: Surface,
}

pub(crate) fn legacy_triggers(ctx: &Ctx<'_>, info: &BlockInfo) -> Result<Vec<LegacyTrigger>> {
    info.variants
        .iter()
        .filter(|v| v.waypoint_trigger.is_none())
        .filter_map(|v| Some((v, v.legacy_waypoint_trigger?)))
        .map(|(v, start)| {
            let surface = solid_surface(ctx, start)
                .with_context(|| format!("{} variant {:?}/{}", ctx.path, v.kind, v.index))?;
            Ok(LegacyTrigger { ground: v.kind == VariantKind::Ground, index: v.index, start, surface })
        })
        .collect()
}

pub fn load(set: &PackSet, blocks: &[(&Block, Option<u32>)], id_base: usize) -> Result<Catalog> {
    let mut cat = Catalog { index: Index::default(), surfaces: Vec::new() };
    for &(block, waypoint) in blocks {
        let buf = set.read_path(&block.path)?;
        let layout = crate::nodes::layout(&buf)?;
        let parsed = crate::blockinfo::read_block_info(&buf, &layout, &block.path)?;
        let ctx = Ctx { buf: &buf, layout: &layout, path: &block.path };
        let mut actual = block.clone();
        let mut changed = false;
        for trigger in legacy_triggers(&ctx, &parsed)? {
            let Some(variant) =
                actual.variants.iter_mut().find(|a| a.ground == trigger.ground && a.index == trigger.index)
            else {
                continue;
            };
            let kind = if trigger.ground { VariantKind::Ground } else { VariantKind::Air };
            let shape = id_base + cat.surfaces.len();
            cat.index.surfaces.push(SurfaceInfo {
                source: format!("{}#legacy-trigger-{:x}", block.path, trigger.start),
                materials: surface_materials(&trigger.surface, &layout.head, &block.path),
                extra_refs: 0,
                tree: None,
            });
            cat.surfaces.push(trigger.surface);
            for (group, mobils) in variant.mobils.iter_mut().enumerate() {
                for (mobil, m) in mobils.iter_mut().enumerate() {
                    let Some(original) = m.prefab.take() else {
                        continue;
                    };
                    let key = format!("{}#trigger-{kind:?}-{}-{group}-{mobil}", block.path, trigger.index);
                    let ent = |model| Ent {
                        model,
                        rotation: [0.0, 0.0, 0.0, 1.0],
                        position: [0.0; 3],
                        params_class: None,
                        name: String::new(),
                    };
                    cat.index.prefabs.insert(
                        key.clone(),
                        Prefab {
                            ents: vec![
                                ent(EntModel::Prefab(original)),
                                ent(EntModel::BlockTrigger { waypoint, shape }),
                            ],
                        },
                    );
                    m.prefab = Some(key);
                }
            }
            changed = true;
        }
        if changed {
            cat.index.blocks.insert(block.path.clone(), actual);
        }
    }
    Ok(cat)
}
