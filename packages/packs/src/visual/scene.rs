use super::*;
use crate::assets::{Assets, Loc};
use crate::surface::Surf;
use anyhow::{Context, ensure};
use placement::{Iso, VariantOffset};

pub fn visual_scene(map: &gearbox_gbx::Map, assets: &Assets, cache: &mut VisualCache) -> Result<Scene> {
    let begin = cache.diagnostics.len();
    let embedded: HashMap<_, _> =
        map.embedded.files.iter().map(|f| (norm(&f.path), f.data.clone())).collect();
    if cache.embedded != embedded {
        let changed: std::collections::HashSet<_> =
            cache.embedded.keys().chain(embedded.keys()).cloned().collect();
        let stale = |key: &str| {
            changed.iter().any(|p| key == p || key.strip_prefix(p).is_some_and(|tail| tail.starts_with('#')))
        };
        cache.files.retain(|key, _| !stale(key));
        cache.decoded.retain(|key, _| !stale(key));
        cache.material_ids.retain(|key, _| !stale(key));
        cache.texture_ids.retain(|key, _| !stale(key));
        cache.cars.clear();
        cache.embedded = embedded;
        cache.embedded_models.clear();
        for file in &map.embedded.files {
            if let Some(name) = file.item_model.as_ref().and_then(|m| m.ident[0].as_deref()) {
                cache.embedded_models.insert(norm(name), file.path.clone());
                cache.embedded_models.insert(norm(&format!("{name}_CustomBlock")), file.path.clone());
            }
        }
    }
    let env = match map.decoration_collection.as_deref().or(map.collection.as_deref()) {
        Some("26" | "Stadium") => "Stadium",
        Some("25") => "Stadium256",
        Some("29") => "WhiteShore",
        v => anyhow::bail!("visual environment {v:?}"),
    };
    let decoration = map.decoration.as_deref().and_then(|name| assets.decoration(name, env));
    let cell = [32., 8., decoration.and_then(|d| d.level0).map_or(-64., |v| -(v as f32) * 8.)];
    let mut scene = Scene::default();
    let mut checked = std::collections::HashSet::new();
    let mut ground = std::collections::HashSet::new();
    for b in &map.blocks {
        if b.free.is_some() || !b.is_ground() {
            continue;
        }
        if let Ok((_, v, _, _)) = selection(assets, b, env) {
            let size = size(&v.units);
            for &u in &v.units {
                ground.insert(unit(b.coord, u, b.dir, size));
            }
        }
    }
    let mut material_grid = HashMap::<[i32; 3], Vec<&gearbox_gbx::map::Block>>::new();
    for b in map.blocks.iter().chain(&map.baked_blocks) {
        if b.free.is_some() {
            continue;
        }
        if let Ok((info, v, _, _)) = selection(assets, b, env) {
            if crate::blockinfo::is_clip(info.class_id) {
                continue;
            }
            for &u in &v.units {
                material_grid.entry(unit(b.coord, u, b.dir, size(&v.units))).or_default().push(b);
            }
        }
    }
    for (b, baked) in map.blocks.iter().map(|b| (b, false)).chain(map.baked_blocks.iter().map(|b| (b, true)))
    {
        if baked
            && b.free.is_none()
            && ground.contains(&b.coord)
            && assets
                .block_in(&b.name, env)
                .is_some_and(|b| matches!(b.class_id, 0x0304f000 | 0x03050000 | 0x0314c000))
        {
            continue;
        }
        scene.placements += 1;
        let result = (|| -> Result<_> {
            let embedded = cache.embedded_models.get(&norm(&b.name)).cloned();
            let (info, v, m, offset) = if let Some(path) = &embedded {
                let bytes = cache.read(path)?;
                let layout = crate::nodes::layout(&bytes)?;
                let ctx = crate::prefab::Ctx { buf: &bytes, layout: &layout, path };
                let n = layout
                    .nodes
                    .iter()
                    .find(|n| n.class_id == 0x2e025000)
                    .context("custom block archive absent")?;
                let custom = crate::custom_block::read(&ctx, n.start)?;
                let mut archetype = b.clone();
                archetype.name = custom.archetype;
                let (info, v, m, offset) = selection(assets, &archetype, env)?;
                let overridden = custom
                    .mobils
                    .iter()
                    .find(|m| {
                        crate::custom_block::mobil_key(m.id)
                            == (
                                b.is_ground(),
                                ((b.flags >> 21) & 127) as usize,
                                b.variant as usize,
                                b.sub_variant as usize,
                            )
                    })
                    .is_some_and(|m| m.object.is_some() || (!custom.compiled && m.crystal.is_some()));
                (info, v, m, if overridden { Default::default() } else { offset })
            } else {
                selection(assets, b, env)?
            };
            let iso = if let Some((p, a)) = b.free {
                placement::free_iso_variant(p, a, offset)
            } else {
                placement::grid_iso_in(cell, None, b.coord, b.dir as u32, size(&v.units), offset)
            };
            let parts = if let Some(path) = embedded {
                match cache.custom_block_parts(&path, b)? {
                    Some(parts) => parts,
                    None => match &m.prefab {
                        Some(p) => cache.file_parts(p, 0, 0)?,
                        None => Vec::new(),
                    },
                }
            } else if let Some(path) = &m.prefab {
                cache.file_parts(path, 0, 0)?
            } else {
                Vec::new()
            };
            let mut mods: Vec<_> = block_modifiers(map, assets, env, b, info, &material_grid, cache, 0)?
                .into_iter()
                .flatten()
                .collect();
            if let Some(p) = assets.index.collection_modifiers.get(env) {
                mods.push((p.clone(), None, false));
            }
            let overrides = cache.overrides(&parts, &mods, assets)?;
            let start = scene.instances.len();
            append(&mut scene, cache, &b.name, &parts, iso, &overrides, 1.);
            if checked.insert((b.name.clone(), b.flags & 0x0fe01000, b.variant, b.sub_variant))
                && let (Some(visual), Ok(collision)) = (
                    instance_bounds(cache, &scene.instances[start..]),
                    assets.block_model(
                        &b.name,
                        b.is_ground(),
                        ((b.flags >> 21) & 127) as usize,
                        b.variant as usize,
                        b.sub_variant as usize,
                    ),
                )
            {
                let mut points = Vec::new();
                for ps in collision {
                    if ps.role != crate::assets::Role::Collision {
                        continue;
                    }
                    let local = iso_loc(placement::compose(iso, loc_iso(chain(&ps.chain))));
                    surf_points(&ps.surface.surf, transform(local), &mut points);
                }
                if let Some(collision) = bounds(points) {
                    scene.bounds_checks.push(BoundsCheck {
                        block: b.name.clone(),
                        visual,
                        collision,
                        overlap: (0..3).all(|k| {
                            visual[0][k] <= collision[1][k] + 0.05 && collision[0][k] <= visual[1][k] + 0.05
                        }),
                    });
                }
            }
            Ok(())
        })();
        match result {
            Ok(()) => scene.decoded_placements += 1,
            Err(e) => cache.report("block", &format!("{} {:?}", b.name, b.coord), format!("{e:#}")),
        }
    }
    for it in &map.items {
        scene.placements += 1;
        let result = (|| -> Result<_> {
            let name = it.model.as_deref().context("item model absent")?;
            let path = cache
                .embedded_models
                .get(&norm(name))
                .map(String::as_str)
                .or_else(|| assets.find_item(name, env))
                .with_context(|| format!("unknown item {name}"))?
                .to_string();
            let parts = cache.file_parts(&path, (it.flags >> 8) as usize, 0)?;
            let loc = placement::free_location_variant(it.position, it.yaw_pitch_roll, Default::default());
            let loc = placement::compose_iso(([1., 0., 0., 0., 1., 0., 0., 0., 1.], it.pivot), loc);
            let iso = Iso { q: placement::rotation_quat(loc.0), p: loc.1 };
            let mods = assets
                .index
                .items
                .get(&path)
                .and_then(|i| i.material_modifier.as_ref())
                .map(|p| vec![(p.clone(), None, false)])
                .unwrap_or_default();
            let overrides = cache.overrides(&parts, &mods, assets)?;
            append(&mut scene, cache, name, &parts, iso, &overrides, it.scale);
            Ok(())
        })();
        match result {
            Ok(()) => scene.decoded_placements += 1,
            Err(e) => cache.report("item", it.model.as_deref().unwrap_or("<none>"), format!("{e:#}")),
        }
    }
    if let Some(d) = decoration
        && let Some(path) = &d.map
    {
        match cache.read(path).and_then(|b| gearbox_gbx::Map::parse(&b)) {
            Ok(deco) => {
                for b in &deco.blocks {
                    let result = (|| -> Result<_> {
                        let (_, v, m, offset) = selection(assets, b, env)?;
                        let parts = match &m.prefab {
                            Some(p) => cache.file_parts(p, 0, 0)?,
                            None => Vec::new(),
                        };
                        let location = (
                            [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                            [
                                (map.size[0] as f32 * 32. - d.size[0] as f32 * 256.) * 0.5,
                                -184.,
                                (map.size[2] as f32 * 32. - d.size[2] as f32 * 256.) * 0.5,
                            ],
                        );
                        let iso = placement::grid_iso_in(
                            [256., 64., -64.],
                            Some(location),
                            b.coord,
                            b.dir as u32,
                            size(&v.units),
                            offset,
                        );
                        append(
                            &mut scene,
                            cache,
                            &format!("decoration/{}", b.name),
                            &parts,
                            iso,
                            &BTreeMap::new(),
                            1.,
                        );
                        Ok(())
                    })();
                    if let Err(e) = result {
                        cache.report("decoration", &b.name, format!("{e:#}"));
                    }
                }
            }
            Err(e) => cache.report("decoration", path, format!("{e:#}")),
        }
    }
    scene.diagnostics = cache.diagnostics[begin..].to_vec();
    Ok(scene)
}
fn unit(coord: [i32; 3], u: [i32; 3], dir: u8, size: [u32; 3]) -> [i32; 3] {
    let [x, y, z] = u;
    let [x, z] = match dir {
        0 => [x, z],
        1 => [size[2] as i32 - 1 - z, x],
        2 => [size[0] as i32 - 1 - x, size[2] as i32 - 1 - z],
        3 => [z, size[0] as i32 - 1 - x],
        _ => [x, z],
    };
    [coord[0] + x, coord[1] + y, coord[2] + z]
}
fn size(units: &[[i32; 3]]) -> [u32; 3] {
    std::array::from_fn(|k| units.iter().map(|u| (u[k] as u32).wrapping_add(1)).fold(1, u32::max))
}
fn selection<'a>(
    assets: &'a Assets,
    b: &gearbox_gbx::map::Block,
    env: &str,
) -> Result<(&'a crate::catalog::Block, &'a crate::catalog::Variant, &'a crate::catalog::Mobil, VariantOffset)>
{
    let info = assets.block_in(&b.name, env).with_context(|| format!("block {} absent", b.name))?;
    let v = info
        .variants
        .iter()
        .find(|v| v.ground == b.is_ground() && v.index == ((b.flags >> 21) & 127) as usize)
        .context("block variant absent")?;
    let group = v.mobils.get(b.variant as usize).context("mobil group absent")?;
    let m = group.get(b.sub_variant as usize).or_else(|| group.first()).context("mobil absent")?;
    let offset = if crate::blockinfo::is_clip(info.class_id) && info.clip_type == 0 {
        VariantOffset::default()
    } else {
        VariantOffset {
            pos: m.geom_translation.unwrap_or_default(),
            deg: m.geom_rotation.unwrap_or_default(),
        }
    };
    Ok((info, v, m, offset))
}
pub(super) fn loc_iso(l: Loc) -> Iso {
    let [x, y, z, w] = l.rotation;
    Iso { q: [w, x, y, z], p: l.position }
}
pub(super) fn iso_loc(l: Iso) -> Loc {
    let [w, x, y, z] = l.q;
    Loc { rotation: [x, y, z, w], position: l.p }
}
fn append(
    scene: &mut Scene,
    cache: &VisualCache,
    name: &str,
    parts: &[Part],
    iso: Iso,
    overrides: &BTreeMap<usize, usize>,
    scale: f32,
) {
    for p in parts {
        let mut local = loc_iso(chain(&p.chain));
        local.p = local.p.map(|x| x * scale);
        let pose = placement::compose(iso, local);
        let (pose, veget_scale) = cache.meshes[p.mesh]
            .vegetation
            .map_or((pose, 1.), |params| super::tree_pose::placement(pose, params));
        let mut t = placement::transform(pose).map(f32::from_bits);
        for x in &mut t[..9] {
            *x *= scale * veget_scale;
        }
        if !cache.meshes[p.mesh].indices.is_empty() {
            scene.instances.push(Instance {
                name: name.into(),
                mesh_id: p.mesh,
                transform: t,
                material_overrides: overrides.clone(),
            });
        }
    }
}
pub fn instance_bounds(cache: &VisualCache, instances: &[Instance]) -> Option<[[f32; 3]; 2]> {
    bounds(
        instances
            .iter()
            .flat_map(|i| cache.meshes[i.mesh_id].positions.iter().map(move |&p| point(i.transform, p))),
    )
}
fn surf_points(s: &Option<crate::surface::SurfNode>, t: Transform, out: &mut Vec<[f32; 3]>) {
    if let Some(s) = s
        && let Surf::Mesh { vertices, .. } = &s.surf
    {
        out.extend(vertices.iter().map(|&p| point(t, p)))
    }
}
impl VisualCache {
    fn overrides(
        &mut self,
        parts: &[Part],
        mods: &[(String, Option<String>, bool)],
        assets: &Assets,
    ) -> Result<BTreeMap<usize, usize>> {
        let ids: std::collections::BTreeSet<_> =
            parts.iter().flat_map(|p| self.meshes[p.mesh].submeshes.iter().map(|s| s.material)).collect();
        let mut out = BTreeMap::new();
        for id in ids {
            let mut path = self.materials[id].name.clone();
            for (modifier, folder, keep) in mods {
                let Some(m) = assets.index.modifiers.get(modifier) else {
                    self.report("material-modifier", modifier, "modifier absent from asset catalog");
                    continue;
                };
                let entries: Vec<_> = m
                    .skinned
                    .iter()
                    .filter(|(_, p)| !(*keep && p.to_ascii_lowercase().ends_with("\\trackwall.material.gbx")))
                    .cloned()
                    .collect();
                if let Some(p) = crate::material::replacement(&path, &m.folder, folder.as_deref(), &entries) {
                    path = p;
                }
            }
            if path != self.materials[id].name {
                match self.material(&path) {
                    Ok(m) => {
                        out.insert(id, m);
                    }
                    Err(e) => {
                        self.report("material-modifier", &path, format!("replacement kept original: {e:#}"))
                    }
                }
            }
        }
        Ok(out)
    }
    fn custom_block_parts(&mut self, path: &str, b: &gearbox_gbx::map::Block) -> Result<Option<Vec<Part>>> {
        let bytes = self.read(path)?;
        let l = crate::nodes::layout(&bytes)?;
        let ctx = crate::prefab::Ctx { buf: &bytes, layout: &l, path };
        let n = l.nodes.iter().find(|n| n.class_id == 0x2e025000).context("custom block archive absent")?;
        let custom = crate::custom_block::read(&ctx, n.start)?;
        let Some(mobil) = custom.mobils.iter().find(|m| {
            crate::custom_block::mobil_key(m.id)
                == (
                    b.is_ground(),
                    ((b.flags >> 21) & 127) as usize,
                    b.variant as usize,
                    b.sub_variant as usize,
                )
        }) else {
            return Ok(None);
        };
        if let Some(object) = &mobil.object {
            let parts = match object {
                crate::prefab::NodeLink::Null => Vec::new(),
                crate::prefab::NodeLink::Inline(so) => self.mesh_ref(&ctx, so.mesh.clone(), 0)?,
                crate::prefab::NodeLink::External(path) => self.file_parts(path, 0, 0)?,
                crate::prefab::NodeLink::Earlier(i) => {
                    let node =
                        l.nodes.iter().find(|n| n.index == *i).context("shared custom mobil object")?;
                    let mut r = super::decode::cursor(&bytes, node.start)?;
                    let so = ctx.static_object(&mut r)?;
                    self.mesh_ref(&ctx, so.mesh, 0)?
                }
            };
            return Ok(Some(parts));
        }
        if !custom.compiled
            && let Some(start) = mobil.crystal
        {
            return self.crystal(&ctx, start).map(Some);
        }
        Ok(None)
    }
}

type Modifier = (String, Option<String>, bool);
fn block_modifiers(
    map: &gearbox_gbx::Map,
    assets: &Assets,
    env: &str,
    b: &gearbox_gbx::map::Block,
    info: &crate::catalog::Block,
    grid: &HashMap<[i32; 3], Vec<&gearbox_gbx::map::Block>>,
    cache: &mut VisualCache,
    depth: usize,
) -> Result<[Option<Modifier>; 2]> {
    ensure!(depth < 64, "clip material recursion");
    let mut inherited = [None, None];
    if crate::blockinfo::is_clip(info.class_id) && info.clip_type != 0 && info.material_modifier.is_none() {
        if b.free.is_some() || b.flags & 0x10000000 != 0 {
            cache.report(
                "clip-material-inheritance",
                &b.name,
                "free/flipped clip parent face lookup not decoded by visual assembler",
            );
        } else {
            let dir = match info.clip_type {
                2 => 5,
                3 => 4,
                _ => b.dir as u32,
            };
            let delta = match dir {
                0 => [0, 0, 1],
                1 => [-1, 0, 0],
                2 => [0, 0, -1],
                3 => [1, 0, 0],
                4 => [0, 1, 0],
                _ => [0, -1, 0],
            };
            let coord = std::array::from_fn(|k| b.coord[k] + delta[k]);
            if let Some(parent) =
                grid.get(&coord).and_then(|cell| cell.iter().find(|b| b.flags & 0x10000000 == 0))
                && let Some(info) = assets.block_in(&parent.name, env)
            {
                inherited = block_modifiers(map, assets, env, parent, info, grid, cache, depth + 1)?;
            }
        }
    }
    let own = [&info.material_modifier, &info.material_modifier2];
    for i in 0..2 {
        if let Some(p) = own[i] {
            let folder = b
                .skin
                .as_ref()
                .and_then(|s| s.first())
                .filter(|s| !s.is_empty())
                .map(|s| format!("{}{s}", &p[..p.rfind('\\').map_or(0, |i| i + 1)]));
            inherited[i] = Some((p.clone(), folder, false));
        }
    }
    if let Some((_, _, keep)) = &mut inherited[0] {
        *keep = map.flags.is_some_and(|f| f & 4 == 0);
    }
    Ok(inherited)
}
