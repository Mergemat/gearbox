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
    pub(crate) group_parents: Vec<u32>,
    pub(crate) archive_version: u32,
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
    let edges = r.len()?;
    if v < 35 {
        r.skip(edges * 8)?;
    } else {
        let count = r.len()?;
        let stride = if positions.len() >= 65535 {
            8
        } else if positions.len() >= 255 {
            4
        } else {
            2
        };
        r.skip(count * stride)?;
    }
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
    Ok(Geometry { positions, faces, face_uvs, face_groups, group_parents, archive_version: v })
}

struct LayerIds {
    names: Vec<String>,
    complete: bool,
}
impl LayerIds {
    fn collector(ctx: &Ctx<'_>, start: usize) -> Result<Self> {
        let container = gearbox_gbx::Container::parse(ctx.buf)?;
        let mut ids = Self {
            names: Vec::new(),
            complete: !ctx
                .layout
                .nodes
                .iter()
                .any(|n| n.class_id == 0x2e025000 || (n.class_id == 0x09003000 && n.start < start)),
        };
        if let Some(header) = container.header_chunk(0x2e001003) {
            let mut r = Reader::new(header);
            for _ in 0..3 {
                ids.read(&mut r)?;
            }
        }
        Ok(ids)
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
        ids.complete &= r.len()? == 0;
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
                    out.materials.push(material(ctx, &mut r, &mut ids, &mut resolve)?);
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
                    let enabled = if v >= 1 { r.bool32()? } else { true };
                    match kind {
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
                                out.visual_geometry.push(g.clone());
                            }
                            if enabled && collidable {
                                layers.push((layer_id, group_ids, false, out.geometry.len()));
                                out.geometry.push(g);
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
                                out.triggers.push(g);
                            }
                        }
                        2 | 4 => {
                            ensure!(r.u32()? == 0, "crystal modifier version");
                            let mut refs = Vec::new();
                            for _ in 0..r.len()? {
                                refs.push((r.u32()?, ids.read(&mut r)?));
                            }
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
                                crate::crystal_modifiers::apply(&mut out, &selections, value, kind == 4);
                            }
                        }
                        12 => {
                            ensure!(r.u32()? == 0, "crystal modifier version");
                            let mut refs = Vec::new();
                            for _ in 0..r.len()? {
                                refs.push((r.u32()?, ids.read(&mut r)?));
                            }
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
                        other => bail!("crystal modifier layer {other} needs compilation"),
                    }
                }
            }
            0x09003000 => {
                r.u32()?;
                let g = geometry(&mut r, out.materials.len())?;
                out.visual_geometry.push(g.clone());
                out.geometry.push(g);
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
    for g in c.geometry.iter().rev() {
        for (face, material) in g.faces.iter().rev() {
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
