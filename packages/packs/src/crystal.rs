use crate::prefab::{Ctx, NodeLink};
use crate::surface::{MeshTri, Surf, SurfNode, Surface};
use anyhow::{Context, Result, bail, ensure};
use gearbox_gbx::reader::Reader;

#[derive(Clone, Debug)]
pub struct Geometry {
    pub positions: Vec<[f32; 3]>,
    pub faces: Vec<(Vec<u32>, i32)>,
    pub face_uvs: Vec<Vec<[f32; 2]>>,
    pub face_groups: Vec<usize>,
    pub(crate) normals: Vec<[f32; 3]>,
    pub(crate) group_parents: Vec<u32>,
    pub(crate) edges: Vec<[u32; 2]>,
    pub(crate) ages: Vec<(u32, u32)>,
    pub(crate) archive_version: u32,
}

impl Geometry {
    pub(crate) fn remove_faces(&mut self, removed: &[usize]) {
        let mut gone = vec![false; self.faces.len()];
        for &f in removed {
            gone[f] = true;
        }
        fn keep<T>(items: &mut Vec<T>, gone: &[bool]) {
            let mut i = 0;
            items.retain(|_| {
                i += 1;
                !gone[i - 1]
            });
        }
        keep(&mut self.faces, &gone);
        keep(&mut self.face_uvs, &gone);
        keep(&mut self.face_groups, &gone);
        keep(&mut self.normals, &gone);
        keep(&mut self.ages, &gone);
    }
}

pub(crate) fn next_age(list: &[Geometry]) -> u32 {
    list.iter().flat_map(|g| &g.ages).map(|a| a.0 + 1).max().unwrap_or(0)
}

pub(crate) fn push_geometry(list: &mut Vec<Geometry>, mut g: Geometry) {
    let age = next_age(list);
    g.ages = (0..g.faces.len() as u32).map(|i| (age, i)).collect();
    list.push(g);
}

#[derive(Clone, Debug, Default)]
pub struct Crystal {
    pub materials: Vec<[u8; 2]>,
    pub geometry: Vec<Geometry>,
    pub triggers: Vec<Geometry>,
    pub visual_geometry: Vec<Geometry>,
}

pub(crate) fn cursor(b: &[u8], p: usize) -> Result<Reader<'_>> {
    let mut r = Reader::new(b);
    r.skip(p)?;
    Ok(r)
}

pub(crate) fn id(r: &mut Reader) -> Result<(u32, Option<String>)> {
    let mut v = r.u32()?;
    if v == 3 {
        v = r.u32()?;
    }
    let name =
        if v != u32::MAX && v & 0xc0000000 != 0 && v & 0x3fffffff == 0 { Some(r.string()?) } else { None };
    Ok((v, name))
}

fn optimized(r: &mut Reader, bound: usize) -> Result<u32> {
    Ok(if bound > 65535 {
        r.u32()?
    } else if bound > 255 {
        r.u16()? as u32
    } else {
        r.u8()? as u32
    })
}
fn optimized_array(r: &mut Reader, count: usize, bound: usize) -> Result<Vec<u32>> {
    (0..count)
        .map(|_| {
            Ok(if bound >= 65535 {
                r.u32()?
            } else if bound >= 255 {
                r.u16()? as u32
            } else {
                r.u8()? as u32
            })
        })
        .collect()
}

fn geometry(r: &mut Reader, materials: usize) -> Result<Geometry> {
    let v = r.u32()?;
    ensure!((25..=37).contains(&v), "crystal geometry version {v}");
    r.u32()?;
    let n = r.len()?;
    r.skip(n * 8)?;
    let n = r.len()?;
    for _ in 0..n {
        r.skip(8 + 48)?;
        r.string()?;
        r.u32()?;
    }
    let groups = r.len()?;
    let mut group_parents = Vec::with_capacity(groups);
    for _ in 0..groups {
        if v >= 31 {
            r.u32()?;
        }
        if v >= 36 {
            r.u8()?;
        } else {
            r.u32()?;
        }
        group_parents.push(r.u32()?);
        r.string()?;
        r.u32()?;
        let n = r.len()?;
        r.skip(n * 4)?;
    }
    if v < 29 {
        r.bool32()?;
        r.bool32()?;
    }
    let embedded = if v >= 34 { r.bool8()? } else { r.bool32()? };
    ensure!(embedded, "external crystal geometry");
    if v >= 33 {
        r.skip(8)?;
    }
    let positions = (0..r.len()?).map(|_| r.vec3()).collect::<Result<Vec<_>>>()?;
    let n = r.len()?;
    let edges = if v < 35 {
        (0..n).map(|_| Ok([r.u32()?, r.u32()?])).collect::<Result<Vec<_>>>()?
    } else {
        let count = r.len()?;
        (0..count)
            .map(|_| {
                Ok(if positions.len() >= 65535 {
                    [r.u32()?, r.u32()?]
                } else if positions.len() >= 255 {
                    [r.u16()? as u32, r.u16()? as u32]
                } else {
                    [r.u8()? as u32, r.u8()? as u32]
                })
            })
            .collect::<Result<Vec<_>>>()?
    };
    let count = r.len()?;
    let (uv_table, uv_indices) = if v >= 37 {
        let uv = (0..r.len()?).map(|_| Ok([r.f32()?, r.f32()?])).collect::<Result<Vec<_>>>()?;
        let n = r.len()?;
        (uv, optimized_array(r, n, n)?)
    } else {
        (Vec::new(), Vec::new())
    };
    let mut uv_index = 0;
    let mut faces = Vec::with_capacity(count);
    let mut face_uvs = Vec::with_capacity(count);
    let mut face_groups = Vec::with_capacity(count);
    for _ in 0..count {
        let n = if v >= 35 { r.u8()? as usize + 3 } else { r.len()? };
        let indices = if v >= 34 {
            optimized_array(r, n, positions.len())?
        } else {
            (0..n).map(|_| r.u32()).collect::<Result<_>>()?
        };
        let uvs = if v < 27 {
            let uv = r.len()?.min(n);
            let mut values = (0..uv).map(|_| Ok([r.f32()?, r.f32()?])).collect::<Result<Vec<_>>>()?;
            values.resize(n, [0.0; 2]);
            r.skip(12)?;
            values
        } else if v < 37 {
            (0..n).map(|_| Ok([r.f32()?, r.f32()?])).collect::<Result<Vec<_>>>()?
        } else {
            let mut values = Vec::new();
            for _ in 0..n {
                let index = *uv_indices.get(uv_index).context("crystal UV corner")? as usize;
                values.push(*uv_table.get(index).context("crystal UV table index")?);
                uv_index += 1;
            }
            values
        };
        face_uvs.push(uvs);
        let material = if v >= 33 && materials > 0 { optimized(r, materials)? as i32 } else { r.i32()? };
        let group = if v >= 33 { optimized(r, groups)? as usize } else { r.u32()? as usize };
        ensure!(group < groups, "crystal face group {group}/{groups}");
        ensure!(indices.iter().all(|&i| (i as usize) < positions.len()), "crystal face position index");
        faces.push((indices, material));
        face_groups.push(group);
    }
    if v < 30 {
        r.skip(count * 4)?;
    }
    if v < 29 {
        r.skip(positions.len() * 4)?;
    }
    r.u32()?;
    if v < 32 {
        ensure!(r.len()? == 0, "crystal links");
        r.u32()?;
        r.string()?;
        if v < 30 {
            let n = r.len()?;
            r.skip(n * 4)?;
        }
    }
    if v < 36 {
        let (f, e, p) = (r.len()?, r.len()?, r.len()?);
        r.skip((f + e + p) * 4)?;
        r.u32()?;
    }
    let normals =
        faces.iter().map(|(face, _)| crate::crystal_deformation::face_normal(&positions, face)).collect();
    Ok(Geometry {
        positions,
        faces,
        face_uvs,
        face_groups,
        normals,
        group_parents,
        edges,
        ages: Vec::new(),
        archive_version: v,
    })
}

struct LayerIds {
    names: Vec<String>,
    complete: bool,
}
impl LayerIds {
    fn collector(ctx: &Ctx<'_>, start: usize) -> Result<Self> {
        let container = gearbox_gbx::Container::parse(ctx.buf)?;
        let mut ids = Self { names: Vec::new(), complete: true };
        if let Some(header) = container.header_chunk(0x2e001003) {
            let mut r = Reader::new(header);
            for _ in 0..3 {
                ids.read(&mut r)?;
            }
        }
        for node in ctx.layout.nodes.iter().filter(|node| node.start < start) {
            match node.class_id {
                0x2e025000 => ids.block(ctx, node.start)?,
                0x09003000 => {
                    read_layers(ctx, node.start, &mut |_| Ok([0, 0]), &mut ids, false)?;
                }
                _ => {}
            }
        }
        Ok(ids)
    }
    fn block(&mut self, ctx: &Ctx<'_>, start: usize) -> Result<()> {
        let mut r = cursor(ctx.buf, start)?;
        ensure!(r.u32()? == 0x2e025000, "block item chunk");
        ensure!(r.u32()? <= 1, "block item version");
        self.read(&mut r)?;
        self.read(&mut r)?;
        Ok(())
    }

    fn read(&mut self, r: &mut Reader) -> Result<Option<String>> {
        let (token, name) = id(r)?;
        if let Some(name) = name {
            self.names.push(name.clone());
            return Ok(Some(name));
        }
        if token == u32::MAX {
            return Ok(None);
        }
        if token & 0xc0000000 == 0 {
            return Ok(Some(token.to_string()));
        }
        let name = self.names.get((token & 0x3fffffff) as usize - 1).cloned();
        self.complete &= name.is_some();
        Ok(name)
    }
}

fn material(
    ctx: &Ctx<'_>,
    r: &mut Reader,
    ids: &mut LayerIds,
    resolve: &mut impl FnMut(&str) -> Result<[u8; 2]>,
) -> Result<[u8; 2]> {
    let name = r.string()?;
    if !name.is_empty() {
        return resolve(&name);
    }
    match ctx.noderef(r, |c, r, class| {
        ensure!(class == 0x090fd000, "crystal material class {class:#x}");
        ensure!(r.u32()? == 0x090fd000, "user material chunk");
        let version = r.u32()?;
        ensure!((10..=11).contains(&version), "user material version {version}");
        let link_name = version < 11 || r.bool8()?;
        ids.read(r)?;
        ids.read(r)?;
        r.string()?;
        let physics = [r.u8()?, r.u8()?];
        if link_name {
            r.string()?;
        } else {
            ids.read(r)?;
        }
        for _ in 0..r.len()? {
            ids.read(r)?;
            ids.read(r)?;
            r.u32()?;
        }
        let colors = r.len()?;
        r.skip(colors * 4)?;
        for _ in 0..r.len()? {
            ids.read(r)?;
            ids.read(r)?;
            r.skip(12)?;
            ids.read(r)?;
        }
        for _ in 0..r.len()? {
            ids.read(r)?;
        }
        for _ in 0..r.len()? {
            r.u32()?;
            r.string()?;
        }
        ids.read(r)?;
        let start = c
            .layout
            .nodes
            .iter()
            .find(|n| n.start <= r.pos() && n.class_id == class && n.end.is_some_and(|end| r.pos() < end))
            .context("user material node")?;
        r.skip(start.end.unwrap() - r.pos())?;
        Ok(physics)
    })? {
        NodeLink::Inline(ids) => Ok(ids),
        NodeLink::External(p) => resolve(&p),
        NodeLink::Null => Ok([0, 0]),
        NodeLink::Earlier(i) => {
            let n = ctx.layout.nodes.iter().find(|n| n.index == i).context("shared crystal material")?;
            let mut rd = cursor(ctx.buf, n.start + 8)?;
            let version = u32::from_le_bytes(ctx.buf[n.start + 4..n.start + 8].try_into().unwrap());
            if version >= 11 {
                rd.bool8()?;
            }
            id(&mut rd)?;
            id(&mut rd)?;
            rd.string()?;
            Ok([rd.u8()?, rd.u8()?])
        }
    }
}

pub fn read(
    ctx: &Ctx<'_>,
    start: usize,
    mut resolve: impl FnMut(&str) -> Result<[u8; 2]>,
) -> Result<Crystal> {
    let mut ids = LayerIds::collector(ctx, start)?;
    read_layers(ctx, start, &mut resolve, &mut ids, true)
}

fn modifier_refs(r: &mut Reader, ids: &mut LayerIds) -> Result<Vec<(u32, Option<String>)>> {
    ensure!(r.u32()? == 0, "crystal modifier version");
    (0..r.len()?).map(|_| Ok((r.u32()?, ids.read(r)?))).collect()
}

fn read_layers(
    ctx: &Ctx<'_>,
    start: usize,
    resolve: &mut impl FnMut(&str) -> Result<[u8; 2]>,
    ids: &mut LayerIds,
    compile: bool,
) -> Result<Crystal> {
    let mut layers = Vec::<(Option<String>, Vec<u32>, bool, usize)>::new();
    let mut visual_layers = Vec::new();
    let n = ctx.layout.nodes.iter().find(|n| n.start == start).context("crystal node")?;
    let mut r = cursor(ctx.buf, start)?;
    let mut out = Crystal::default();
    loop {
        let chunk = r.u32()?;
        match chunk {
            0xfacade01 => break,
            0x09051000 => {
                ensure!(r.u32()? == 1, "tree generator version");
            }
            0x09003003 => {
                let v = r.u32()?;
                ensure!(v == 2, "crystal materials version {v}");
                for _ in 0..r.len()? {
                    out.materials.push(material(ctx, &mut r, ids, resolve)?);
                }
            }
            0x09003005 => {
                ensure!(r.u32()? == 0, "crystal layer archive version");
                for _ in 0..r.len()? {
                    let kind = r.u32()?;
                    let v = r.u32()?;
                    r.bool32()?;
                    let layer_id = ids.read(&mut r)?;
                    r.string()?;
                    let enabled = (if v >= 1 { r.bool32()? } else { true }) && compile;
                    match kind {
                        13 => {
                            let voxels = crate::crystal_voxels::Voxels::read(&mut r)?;
                            if enabled && (voxels.visible || voxels.collidable) {
                                let g = voxels.compile()?;
                                for (geometry, layers, include) in [
                                    (&mut out.visual_geometry, &mut visual_layers, voxels.visible),
                                    (&mut out.geometry, &mut layers, voxels.collidable),
                                ] {
                                    if include {
                                        let first = geometry
                                            .iter()
                                            .map(|g| g.group_parents.len() as u32)
                                            .sum::<u32>();
                                        layers.push((
                                            layer_id.clone(),
                                            (first..first + g.group_parents.len() as u32).collect(),
                                            false,
                                            geometry.len(),
                                        ));
                                        push_geometry(geometry, g.clone());
                                    }
                                }
                            }
                        }
                        0 => {
                            let gv = r.u32()?;
                            let g = geometry(&mut r, out.materials.len())?;
                            let group_ids = (0..r.len()?).map(|_| r.u32()).collect::<Result<Vec<_>>>()?;
                            let (visible, collidable) =
                                if gv >= 1 { (r.bool32()?, r.bool32()?) } else { (true, true) };
                            if enabled && visible {
                                visual_layers.push((
                                    layer_id.clone(),
                                    group_ids.clone(),
                                    false,
                                    out.visual_geometry.len(),
                                ));
                                push_geometry(&mut out.visual_geometry, g.clone());
                            }
                            if enabled && collidable {
                                layers.push((layer_id, group_ids, false, out.geometry.len()));
                                push_geometry(&mut out.geometry, g);
                            }
                        }
                        14 => {
                            let tv = r.u32()?;
                            let g = geometry(&mut r, out.materials.len())?;
                            let group_ids = if tv >= 1 {
                                (0..r.len()?).map(|_| r.u32()).collect::<Result<Vec<_>>>()?
                            } else {
                                Vec::new()
                            };
                            if enabled {
                                layers.push((layer_id, group_ids, true, out.triggers.len()));
                                push_geometry(&mut out.triggers, g);
                            }
                        }
                        2 | 4 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            ensure!(r.u32()? == 0, "crystal transform version");
                            let value = r.vec3()?;
                            let individually = kind == 4 && r.bool32()?;
                            if enabled {
                                if !ids.complete && (!out.geometry.is_empty() || !out.triggers.is_empty()) {
                                    todo!("crystal modifier with an enclosing/parameter id table");
                                }
                                let selections = crate::crystal_modifiers::select(&out, &layers, &refs);
                                if individually {
                                    todo!("individual-group crystal scale");
                                }
                                crate::crystal_modifiers::apply(&mut out, &selections, |positions| {
                                    if kind == 4 {
                                        crate::crystal_modifiers::scale(positions, value);
                                    } else {
                                        crate::crystal_modifiers::translate(positions, value);
                                    }
                                });
                            }
                        }
                        3 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            ensure!(r.u32()? == 0, "crystal rotation version");
                            let angle = r.f32()?;
                            let axis = r.u32()? as usize;
                            ensure!(axis < 3, "crystal rotation axis {axis}");
                            let individually = r.bool32()?;
                            if enabled {
                                ensure!(
                                    ids.complete || (out.geometry.is_empty() && out.triggers.is_empty()),
                                    "crystal rotation with an enclosing/parameter id table"
                                );
                                let selections = crate::crystal_modifiers::select(&out, &layers, &refs);
                                let rotate = |positions: &mut [[f32; 3]]| {
                                    crate::crystal_modifiers::rotate(positions, angle, axis);
                                };
                                let size = if individually { 1 } else { selections.len().max(1) };
                                for part in selections.chunks(size) {
                                    crate::crystal_modifiers::apply(&mut out, part, rotate);
                                }
                            }
                        }
                        12 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            let modifier = crate::crystal_deformation::read(&mut r)?;
                            if enabled {
                                if !ids.complete {
                                    todo!("crystal deformation with an enclosing/parameter id table");
                                }
                                crate::crystal_deformation::apply_layers(
                                    &mut out.geometry,
                                    &layers,
                                    &refs,
                                    &modifier,
                                    false,
                                );
                                crate::crystal_deformation::apply_layers(
                                    &mut out.triggers,
                                    &layers,
                                    &refs,
                                    &modifier,
                                    true,
                                );
                                crate::crystal_deformation::apply_layers(
                                    &mut out.visual_geometry,
                                    &visual_layers,
                                    &refs,
                                    &modifier,
                                    false,
                                );
                            }
                        }
                        15 | 18 => {
                            r.u32()?;
                            for _ in 0..r.len()? {
                                r.u32()?;
                                ids.read(&mut r)?;
                            }
                            let version = r.u32()?;
                            if kind == 15 {
                                r.skip(20)?;
                                if version >= 1 {
                                    r.f32()?;
                                }
                            } else {
                                for _ in 0..r.len()? {
                                    ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
                                }
                                let n = r.len()?;
                                r.skip(n * 52)?;
                            }
                        }
                        1 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            let version = r.u32()?;
                            ensure!(version == 0, "crystal smooth subdivision version {version}");
                            let levels = r.u32()?;
                            if enabled {
                                crate::crystal_subdivision::smooth(&mut out, &layers, &refs, levels);
                            }
                        }
                        8 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            let version = r.u32()?;
                            ensure!(version == 0, "crystal subdivision version {version}");
                            r.u32()?;
                            if enabled {
                                crate::crystal_subdivision::subdivide(&mut out, &layers, &refs);
                            }
                        }
                        9 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            let version = r.u32()?;
                            ensure!(version == 1, "crystal chaos version {version}");
                            let from = r.f32()?;
                            let seed = r.u32()?;
                            let to = r.f32()?;
                            if enabled {
                                let chaos = crate::crystal_modifiers::Chaos { from, to, seed };
                                crate::crystal_modifiers::chaos(&mut out, &layers, &refs, &chaos);
                            }
                        }
                        5 => {
                            let refs = modifier_refs(&mut r, ids)?;
                            let version = r.u32()?;
                            ensure!(version == 0, "crystal mirror version {version}");
                            let axis = r.u32()?;
                            ensure!(axis < 3, "crystal mirror axis {axis}");
                            let distance = r.f32()?;
                            let independently = r.bool32()?;
                            if enabled {
                                let mirror = crate::crystal_modifiers::Mirror {
                                    axis: axis as usize,
                                    distance,
                                    independently,
                                };
                                crate::crystal_modifiers::mirror(
                                    &mut out,
                                    &mut layers,
                                    &layer_id,
                                    &refs,
                                    &mirror,
                                )?;
                            }
                        }
                        other => bail!("crystal modifier layer {other} needs compilation"),
                    }
                }
            }
            0x09003000 => {
                r.u32()?;
                let g = geometry(&mut r, out.materials.len())?;
                push_geometry(&mut out.visual_geometry, g.clone());
                push_geometry(&mut out.geometry, g);
            }
            0x09003007 => {
                r.u32()?;
                let n = r.len()?;
                r.skip(n * 4)?;
                let n = r.len()?;
                r.skip(n * 4)?;
            }
            0x09003006 => {
                let v = r.u32()?;
                let n = r.len()?;
                r.skip(n * if v == 0 { 8 } else { 4 })?;
                if v >= 2 {
                    let n = r.len()?;
                    optimized_array(&mut r, n, n)?;
                }
            }
            _ => {
                ensure!(r.u32()? == 0x534b4950, "crystal chunk {chunk:#x} not decoded");
                let n = r.len()?;
                r.skip(n)?;
            }
        }
        ensure!(r.pos() <= n.end.context("crystal without end")?, "crystal crossed node end");
    }
    Ok(out)
}

pub fn trigger(c: &Crystal) -> Result<Surface> {
    collision(&Crystal {
        materials: c.materials.clone(),
        geometry: c.triggers.clone(),
        triggers: Vec::new(),
        visual_geometry: Vec::new(),
    })
}

pub fn collision(c: &Crystal) -> Result<Surface> {
    let mut vertices = Vec::<[f32; 3]>::new();
    let mut triangles = Vec::new();
    let mut material_slots = Vec::new();
    let mut order: Vec<_> = c
        .geometry
        .iter()
        .flat_map(|g| g.faces.iter().zip(&g.ages).map(move |(face, age)| (*age, g, face)))
        .collect();
    order.sort_by(|a, b| b.0.cmp(&a.0));
    for (_, g, (face, material)) in order {
        let ids = if *material < 0 {
            [0, 0]
        } else {
            *c.materials.get(*material as usize).context("crystal face material")?
        };
        let slot = material_slots.iter().position(|m| m == material).unwrap_or_else(|| {
            material_slots.push(*material);
            material_slots.len() - 1
        }) as i16;
        if ids[0] == 28 {
            continue;
        }
        let tris: &[[usize; 3]] = match face.len() {
            3 => &[[0, 1, 2]],
            4 => &[[0, 1, 3], [1, 2, 3]],
            n => bail!("crystal polygon with {n} vertices needs compilation"),
        };
        for tri in tris {
            let mut indices = [0; 3];
            for (dst, &src) in indices.iter_mut().zip(tri) {
                let p = g.positions[face[src] as usize];
                *dst = vertices.len() as i32;
                vertices.push(p);
            }
            triangles.push(MeshTri { v: indices, flags: ids, material: slot });
        }
    }
    let (welded, remap) = crate::crystal_mesh::weld(&vertices);
    for t in &mut triangles {
        t.v = t.v.map(|i| remap[i as usize] as i32);
    }
    triangles.retain(|t| {
        crate::crystal_mesh::keep_triangle(
            welded[t.v[0] as usize],
            welded[t.v[1] as usize],
            welded[t.v[2] as usize],
        )
    });
    let mut used = vec![None; welded.len()];
    vertices.clear();
    for t in &mut triangles {
        for i in &mut t.v {
            *i = *used[*i as usize].get_or_insert_with(|| {
                let index = vertices.len() as i32;
                vertices.push(welded[*i as usize]);
                index
            });
        }
    }
    Ok(Surface {
        version: 4,
        surf_version: 2,
        surf: (!triangles.is_empty()).then_some(SurfNode {
            kind: 7,
            surf: Surf::Mesh { version: 7, vertices, triangles },
            gameplay_main_dir: Some([0.0, 0.0, 1.0]),
        }),
        materials: Vec::new(),
        material_ids: material_slots
            .iter()
            .map(|&slot| {
                let ids = if slot < 0 { [0, 0] } else { c.materials[slot as usize] };
                u16::from_le_bytes(ids)
            })
            .collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crystal_modifiers::Layer;

    type Run = dyn Fn(&mut Crystal, &mut Vec<Layer>, &[(u32, Option<String>)], &[&str]);

    fn faces_newest_first(c: &Crystal) -> Vec<u8> {
        let mut firsts = Vec::new();
        let mut first = 0u32;
        for g in &c.geometry {
            firsts.push(first);
            first += g.group_parents.len() as u32;
        }
        let mut order = Vec::new();
        for (g, first) in c.geometry.iter().zip(firsts) {
            for (f, age) in g.ages.iter().enumerate() {
                order.push((*age, g, first, f));
            }
        }
        order.sort_by(|a, b| b.0.cmp(&a.0));
        let mut out = Vec::new();
        for (_, g, first, f) in order {
            let face = &g.faces[f].0;
            if !(3..=4).contains(&face.len()) {
                continue;
            }
            out.extend((first + g.face_groups[f] as u32).to_le_bytes());
            out.extend((face.len() as u32).to_le_bytes());
            for &v in face {
                out.extend(g.positions[v as usize].into_iter().flat_map(f32::to_le_bytes));
            }
        }
        out
    }

    fn matches_recorded(layer: &str, run: &Run) {
        let Some(dir) =
            std::env::var_os("GBX_TEST_DATA").map(|d| std::path::Path::new(&d).join("crystal").join(layer))
        else {
            return;
        };
        let Ok(entries) = std::fs::read_dir(&dir) else { return };
        let mut checked = 0;
        for entry in entries.flatten() {
            let path = entry.path();
            let params = std::fs::read_to_string(path.join("params.txt")).unwrap();
            let lines: Vec<&str> = params.lines().collect();
            let bytes = std::fs::read(path.join("geometry.bin")).unwrap();
            let g = geometry(&mut Reader::new(&bytes), lines[2].parse().unwrap()).unwrap();
            let all: Vec<u32> = (0..g.group_parents.len() as u32).collect();
            let groups: Vec<u32> = match lines[1] {
                "all" => all.clone(),
                list => list.split_whitespace().map(|s| s.parse().unwrap()).collect(),
            };
            let mut crystal = Crystal::default();
            push_geometry(&mut crystal.geometry, g);
            let mut layers = vec![(Some("L".to_string()), all, false, 0)];
            let refs: Vec<_> = groups.iter().map(|&j| (j, Some("L".to_string()))).collect();
            run(&mut crystal, &mut layers, &refs, &lines[0].split_whitespace().collect::<Vec<_>>());
            let got = faces_newest_first(&crystal);
            let expected = std::fs::read(path.join("expected.bin")).unwrap();
            if got != expected {
                let at = got
                    .iter()
                    .zip(&expected)
                    .position(|(a, b)| a != b)
                    .unwrap_or(got.len().min(expected.len()));
                panic!(
                    "{}: {} vs {} bytes, first difference at {at}",
                    path.display(),
                    got.len(),
                    expected.len()
                );
            }
            checked += 1;
        }
        assert!(checked > 0);
    }

    #[test]
    fn chaos_matches_the_recorded_layers() {
        matches_recorded("chaos", &|c, layers, refs, head| {
            let chaos = crate::crystal_modifiers::Chaos {
                from: head[0].parse().unwrap(),
                to: head[1].parse().unwrap(),
                seed: head[2].parse().unwrap(),
            };
            crate::crystal_modifiers::chaos(c, layers, refs, &chaos);
        });
    }

    #[test]
    fn mirror_matches_the_recorded_layers() {
        matches_recorded("mirror", &|c, layers, refs, head| {
            let mirror = crate::crystal_modifiers::Mirror {
                axis: head[0].parse().unwrap(),
                distance: head[1].parse().unwrap(),
                independently: false,
            };
            crate::crystal_modifiers::mirror(c, layers, &Some("M".to_string()), refs, &mirror).unwrap();
        });
    }

    #[test]
    fn smooth_subdivision_matches_the_recorded_layers() {
        matches_recorded("smooth", &|c, layers, refs, head| {
            crate::crystal_subdivision::smooth(c, layers, refs, head[0].parse().unwrap());
        });
    }

    #[test]
    fn subdivision_matches_the_recorded_layers() {
        matches_recorded("subdivide", &|c, layers, refs, _| {
            crate::crystal_subdivision::subdivide(c, layers, refs);
        });
    }
}
