use anyhow::{Result, bail, ensure};
use gearbox_gbx::reader::Reader;

use crate::nodes::Layout;
use crate::surface::Surface;

#[derive(Clone, Debug, PartialEq)]
pub enum NodeLink<T> {
    Null,
    External(String),
    Earlier(u32),
    Inline(T),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Model {
    Prefab(Box<Prefab>),
    StaticObject(StaticObject),
    DynaObject(DynaObject),
    Kinematic(crate::map_model::Animation),
    Trigger { waypoint: Option<u32>, shape: NodeLink<Surface> },
    Surface(Surface),
    Other(u32),
}

#[derive(Clone, Debug, PartialEq)]
pub struct StaticObject {
    pub version: u32,
    pub mesh: NodeLink<usize>,
    pub mesh_collidable: bool,
    pub shape: NodeLink<Surface>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynaObject {
    pub version: u32,
    pub mesh: NodeLink<usize>,
    pub is_static: bool,
    pub dyna_shape: NodeLink<Surface>,
    pub static_shape: NodeLink<Surface>,
    pub physics: crate::map_model::Dyna,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Ent {
    pub model: NodeLink<Model>,
    pub rotation: [f32; 4],
    pub position: [f32; 3],
    pub params: Option<(u32, Vec<u8>)>,
    pub name: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Prefab {
    pub version: u32,
    pub url: String,
    pub ents: Vec<Ent>,
}

pub struct Ctx<'a> {
    pub buf: &'a [u8],
    pub layout: &'a Layout,
    pub path: &'a str,
}

impl Ctx<'_> {
    pub fn noderef<T>(
        &self,
        r: &mut Reader,
        inline: impl FnOnce(&Self, &mut Reader, u32) -> Result<T>,
    ) -> Result<NodeLink<T>> {
        let idx = r.u32()?;
        if idx == u32::MAX {
            return Ok(NodeLink::Null);
        }
        if self.layout.head.refs.contains(&idx) {
            let p = self.layout.head.ref_path(self.path, idx).unwrap_or_else(|| format!("<resource {idx}>"));
            return Ok(NodeLink::External(p));
        }
        let start = r.pos() + 4;
        if let Some(&ni) = self.layout.by_start.get(&start)
            && self.layout.nodes[ni].index == idx
        {
            let class = self.layout.nodes[ni].class_id;
            r.skip(4)?;
            return Ok(NodeLink::Inline(inline(self, r, class)?));
        }
        Ok(NodeLink::Earlier(idx))
    }

    pub fn skip_node(&self, r: &mut Reader) -> Result<()> {
        let ni =
            *self.layout.by_start.get(&r.pos()).ok_or_else(|| anyhow::anyhow!("no node at {}", r.pos()))?;
        let Some(end) = self.layout.nodes[ni].end else {
            bail!(
                "cannot skip node of class {:#010x} archived without chunks",
                self.layout.nodes[ni].class_id
            )
        };
        r.skip(end - r.pos())
    }

    pub fn dyna_object(&self, r: &mut Reader) -> Result<DynaObject> {
        let version = r.u32()?;
        let is_static = r.u32()? != 0;
        let dynamize_on_spawn = r.u32()? != 0;
        let mesh = self.noderef(r, |c, r, class| {
            ensure!(class == 0x090bb000, "dyna mesh of class {class:#010x}");
            let start = r.pos();
            c.skip_node(r)?;
            Ok(start)
        })?;
        let surface = |c: &Self, r: &mut Reader, class: u32| {
            ensure!(class == 0x0900c000, "dyna shape of class {class:#010x}");
            c.surface(r)
        };
        let dyna_shape = self.noderef(r, surface)?;
        let static_shape = self.noderef(r, surface)?;
        ensure!(version == 13, "dyna archive version {version}: {}", self.path);
        r.f32()?;
        let mass = r.f32()?;
        r.skip(8 + 8 + 2 + 8 + 1 + 8)?;
        let animation = match self.noderef(r, |c, r, _| c.skip_node(r))? {
            NodeLink::Null => None,
            NodeLink::External(p) => Some(p),
            _ => bail!("unsupported inline legacy location animation: {}", self.path),
        };
        r.u32()?;
        let physical_animation = r.u32()? != 0;
        ensure!(r.u32()? == u32::MAX, "unsupported dyna water model: {}", self.path);
        let physics = crate::map_model::Dyna { dynamize_on_spawn, mass, animation, physical_animation };
        Ok(DynaObject { version, mesh, is_static, dyna_shape, static_shape, physics })
    }

    pub fn surface(&self, r: &mut Reader) -> Result<Surface> {
        let mut skip = |rd: &mut Reader, _idx: u32| -> Result<()> {
            if let Some(c) = rd.peek_u32()
                && crate::unpack::is_class(c)
            {
                rd.skip(4)?;
                self.skip_node(rd)?;
            }
            Ok(())
        };
        let (s, end) = crate::surface::read_node(self.buf, r.pos(), &mut skip)?;
        r.skip(end - r.pos())?;
        Ok(s)
    }

    pub fn static_object(&self, r: &mut Reader) -> Result<StaticObject> {
        let version = r.u32()?;
        ensure!(version <= 3, "static object version {version}");
        let mesh = self.noderef(r, |c, r, class| {
            ensure!(class == 0x090bb000, "static object mesh of class {class:#010x}");
            let start = r.pos();
            c.skip_node(r)?;
            Ok(start)
        })?;
        let mesh_collidable = r.u8()? != 0;
        let shape = if mesh_collidable {
            NodeLink::Null
        } else {
            self.noderef(r, |c, r, class| {
                ensure!(class == 0x0900c000, "static object shape of class {class:#010x}");
                c.surface(r)
            })?
        };
        Ok(StaticObject { version, mesh, mesh_collidable, shape })
    }

    pub fn model(&self, r: &mut Reader, class: u32) -> Result<Model> {
        match class {
            0x09145000 => Ok(Model::Prefab(Box::new(self.prefab(r)?))),
            0x2f0ca000 => Ok(Model::Kinematic(crate::map_model::animation_node(r)?)),
            0x09159000 => Ok(Model::StaticObject(self.static_object(r)?)),
            0x09144000 => {
                let d = self.dyna_object(r)?;
                Ok(Model::DynaObject(d))
            }
            0x09178000 => {
                r.u32()?;
                let kind = r.u32()?;
                let shape = self.noderef(r, |c, r, _| c.surface(r))?;
                r.u32()?;
                Ok(Model::Trigger { waypoint: Some(kind), shape })
            }
            0x09179000 => {
                let v = r.u32()?;
                let shape = self.noderef(r, |c, r, _| c.surface(r))?;
                if v >= 2 {
                    r.u32()?;
                }
                Ok(Model::Trigger { waypoint: None, shape })
            }
            0x0917b000 => {
                r.u32()?;
                self.noderef(r, |c, r, class| c.model(r, class).map(|_| ()))?;
                Ok(Model::Other(0x0917b000))
            }
            0x0900c000 => Ok(Model::Surface(self.surface(r)?)),
            c => {
                self.skip_node(r)?;
                Ok(Model::Other(c))
            }
        }
    }

    pub fn prefab(&self, r: &mut Reader) -> Result<Prefab> {
        let version = r.u32()?;
        ensure!(version >= 9 && version <= 11, "prefab version {version}");
        r.skip(8)?;
        let url = r.string()?;
        r.i32()?;
        let n = r.u32()? as usize;
        ensure!(n < 1 << 16, "bad ent count {n}");
        r.i32()?;
        let mut ents = Vec::with_capacity(n);
        for _ in 0..n {
            let model = self.noderef(r, |c, r, class| c.model(r, class))?;
            let rotation = [r.f32()?, r.f32()?, r.f32()?, r.f32()?];
            let position = r.vec3()?;
            let pc = r.u32()?;
            let params = if pc == u32::MAX {
                None
            } else {
                let start = r.pos();
                skip_params(r, pc)?;
                Some((pc, r.buf()[start..r.pos()].to_vec()))
            };
            let name = r.string()?;
            ents.push(Ent { model, rotation, position, params, name });
        }
        Ok(Prefab { version, url, ents })
    }
}

fn skip_params(r: &mut Reader, class: u32) -> Result<()> {
    fn placement(r: &mut Reader) -> Result<()> {
        r.u32()?;
        r.i32()?;
        let n = r.u32()? as usize;
        ensure!(n < 1 << 12, "bad option count");
        for _ in 0..n {
            let k = r.u32()? as usize;
            ensure!(k < 1 << 12, "bad tag count");
            for _ in 0..k {
                r.string()?;
                r.string()?;
            }
        }
        Ok(())
    }
    match class {
        0x2f0a9000 => placement(r)?,
        0x2f0b6000 => {
            let v = r.u32()?;
            r.f32()?;
            r.i32()?;
            r.u32()?;
            if v >= 1 {
                r.skip(12)?;
                if v >= 2 {
                    r.u32()?;
                }
            }
        }
        0x2f0c8000 => {
            r.u32()?;
            r.skip(8 + 24)?;
        }
        0x2f0d8000 => {
            r.u32()?;
            let n = r.u32()? as usize;
            ensure!(n < 1 << 12, "bad placement count");
            for _ in 0..n {
                placement(r)?;
            }
            let k = r.u32()? as usize;
            r.skip(2 * k)?;
            let k = r.u32()? as usize;
            r.skip(28 * k)?;
        }
        0x2f0d9000 => {
            r.u32()?;
            r.f32()?;
        }
        c => bail!("unknown ent params class {c:#010x}"),
    }
    Ok(())
}

pub fn read_prefab(buf: &[u8], layout: &Layout, path: &str) -> Result<Prefab> {
    ensure!(layout.head.class_id == 0x09145000, "not a prefab (class {:#010x})", layout.head.class_id);
    let ctx = Ctx { buf, layout, path };
    let mut r = Reader::new(buf);
    r.skip(layout.head.body_start)?;
    ctx.prefab(&mut r)
}
