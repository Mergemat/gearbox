use crate::{
    packs::PackSet,
    surface::{self, Surface},
    unpack::GbxHead,
};
use anyhow::{Context, Result, ensure};
use gearbox_engine::{Aabb, Vec3, vehicle::CarShape};
use gearbox_gbx::reader::{FACADE, Reader};

#[derive(Debug)]
pub struct Skeleton {
    pub names: Vec<String>,
    pub transforms: Vec<[u32; 12]>,
}
#[derive(Debug)]
pub struct Collision {
    pub surface: Surface,
    pub skeleton: Option<Skeleton>,
}
#[derive(Debug)]
pub struct Wheel {
    pub name: String,
    pub driven: u32,
    pub steering: u32,
}
#[derive(Debug)]
pub struct CarModel {
    pub car: String,
    aux: crate::car_aux::Aux,
    pub path: String,
    pub shape: CarShape,
    pub wheels: Vec<Wheel>,
    pub collision: Collision,
    pub tuning_path: String,
}

impl CarModel {
    pub fn auxiliary(&self) -> &crate::car_aux::Aux { &self.aux }
}

fn head(buf: &[u8]) -> Result<GbxHead> {
    crate::unpack::parse_head(buf).context("truncated model header")?
}

fn skeleton(r: &mut Reader<'_>) -> Result<Skeleton> {
    ensure!(r.u32()? == 0x090ba000, "skeleton chunk");
    ensure!(r.u32()? == 20, "unsupported skeleton version");
    r.id()?;
    let n = r.u16()? as usize;
    ensure!(n < 1024, "skeleton joint count {n}");
    let mut names = Vec::new();
    let mut transforms = Vec::new();
    for _ in 0..n {
        names.push(r.id()?.context("null joint name")?);
        r.i16()?;
        let mut t = [0; 12];
        for v in &mut t {
            *v = r.u32()?;
        }
        transforms.push(t);
    }
    ensure!(r.u32()? == 0, "unsupported skeleton joint expression path");
    ensure!(r.u32()? == 0, "unsupported skeleton sockets");
    ensure!(r.u32()? == 0, "unsupported skeleton constraints");
    for _ in 0..3 {
        let n = r.u32()? as usize;
        r.skip(n)?;
    }
    let n = r.u32()? as usize;
    r.skip(n * 12)?;
    r.u8()?;
    r.u32()?;
    ensure!(r.u32()? == FACADE, "skeleton end");
    Ok(Skeleton { names, transforms })
}

fn collision(buf: &[u8], pos: usize) -> Result<(Collision, usize)> {
    let mut skel = None;
    let (surface, end) = surface::read_node(buf, pos, &mut |r, idx| {
        if idx == u32::MAX {
            return Ok(());
        }
        ensure!(r.u32()? == 0x090ba000, "collision reference class");
        skel = Some(skeleton(r)?);
        Ok(())
    })?;
    Ok((
        Collision {
            surface,
            skeleton: skel,
        },
        end,
    ))
}

struct Archive<'a> {
    r: Reader<'a>,
    h: GbxHead,
    path: &'a str,
    packs: &'a PackSet,
}
impl Archive<'_> {
    fn collision_ref(&mut self) -> Result<Collision> {
        let idx = self.r.u32()?;
        if let Some(path) = self.h.ref_path(self.path, idx) {
            let b = self.packs.read_path(&path)?;
            let h = head(&b)?;
            ensure!(h.class_id == 0x0900c000, "{path}: not surface");
            return Ok(collision(&b, h.body_start)?.0);
        }
        ensure!(idx != u32::MAX, "missing collision shape");
        ensure!(self.r.u32()? == 0x0900c000, "inline collision class");
        let (c, end) = collision(self.r.buf(), self.r.pos())?;
        let l = crate::nodes::layout(self.r.buf())?;
        if let Some(s) = l
            .nodes
            .iter()
            .find(|s| s.class_id == 0x090ba000 && s.start >= self.r.pos() && s.start < end)
        {
            self.r.skip(s.start - self.r.pos())?;
            let _ = skeleton(&mut self.r)?;
        }
        self.r.skip(end - self.r.pos())?;
        Ok(c)
    }
    fn shape(&mut self) -> Result<(CarShape, Vec<Wheel>, Collision)> {
        ensure!(self.r.u32()? == 0x0910e000, "car shape class");
        ensure!(self.r.u32()? == 0x0910e000, "car shape chunk");
        ensure!(self.r.u32()? == 6, "unsupported car shape archive version");
        let mut shape = CarShape { lateral_offset: self.r.f32()?, ..CarShape::default() };
        ensure!(self.r.u32()? == u32::MAX, "unsupported legacy solid reference");
        let collision = self.collision_ref()?;
        let n = self.r.u32()? as usize;
        ensure!(n <= 4, "wheel count {n}");
        let mut wheels = Vec::new();
        for _ in 0..n {
            let driven = self.r.u32()?;
            let steering = self.r.u32()?;
            let name = self.r.id()?.context("null wheel name")?;
            wheels.push(Wheel { name, driven, steering });
        }
        shape.bounds = self.bounds()?;
        shape.manual_wheel_positions = self.r.u32()?;
        shape.wheel_height = self.r.f32()?;
        shape.front_axle = self.r.f32()?;
        shape.rear_axle = self.r.f32()?;
        shape.front_half_track = self.r.f32()?;
        shape.rear_half_track = self.r.f32()?;
        shape.wheel_radius = [self.r.f32()?, self.r.f32()?];
        shape.wheel_width = [self.r.f32()?, self.r.f32()?];
        ensure!(self.r.u32()? == 0, "unsupported shape identifier array");
        shape.water_bounds = self.bounds()?;
        self.r.skip(4 * 16)?;
        ensure!(self.r.u32()? == u32::MAX, "unsupported auxiliary collision");
        let n = self.r.u32()?;
        ensure!(n == 6, "shape point count {n}");
        self.r.skip(4 * 18)?;
        for _ in 0..3 {
            ensure!(self.r.u32()? == u32::MAX, "unsupported additional car collision reference");
        }
        ensure!(self.r.u32()? == FACADE, "car shape end");
        Ok((shape, wheels, collision))
    }

    fn bounds(&mut self) -> Result<Aabb> {
        let center = Vec3::new(self.r.f32()?, self.r.f32()?, self.r.f32()?);
        let half = Vec3::new(self.r.f32()?, self.r.f32()?, self.r.f32()?);
        Ok(Aabb::new(center, half))
    }
}

pub fn load(packs: &PackSet, car: &str) -> Result<CarModel> {
    let (path, visual_path) = crate::car_aux::selection(packs, car)?;
    let path = path.as_str();
    let bytes = packs.read_path(path)?;
    let h = head(&bytes)?;
    ensure!(h.class_id == 0x090ea000, "not vehicle model");
    let mut a = Archive {
        r: Reader::new(&bytes),
        h,
        path,
        packs,
    };
    a.r.skip(a.h.body_start)?;
    ensure!(a.r.u32()? == 0x090ea003, "vehicle tuning chunk");
    let ti = a.r.u32()?;
    let tuning_path = a.h.ref_path(path, ti).context("tuning reference")?;
    ensure!(a.r.u32()? == 0x090ea008, "vehicle shape chunk");
    ensure!(a.r.u32()? == 14, "unsupported vehicle model version");
    a.r.u32()?;
    let (shape, wheels, collision) = a.shape()?;
    ensure!(a.r.u32()? == 0, "unsupported extra physics model objects");
    let n = a.r.u32()? as usize;
    ensure!(
        n == 0,
        "unsupported additional contact definitions"
    );
    a.r.u32()?;
    ensure!(
        a.r.u32()? == 0x090ea009 && a.r.u32()? == 1,
        "material reference chunk"
    );
    let mut material_paths = [None, None];
    for p in &mut material_paths {
        let idx = a.r.u32()?;
        if idx != u32::MAX {
            *p = Some(a.h.ref_path(path, idx).context("material reference")?);
        }
    }
    ensure!(
        a.r.u32()? == FACADE && a.r.remaining() == 0,
        "model trailing data at {:x}",
        a.r.pos()
    );
    let mut aux = crate::car_aux::load(packs, &visual_path, &material_paths)?;
    let tunings = crate::tuning::decode(&packs.read_path(&tuning_path)?)?;
    if let Some(materials) = tunings.active_materials() {
        aux.materials.extend(materials.iter().cloned());
    }
    Ok(CarModel {
        aux,
        car: car.into(),
        path: path.into(),
        shape,
        wheels,
        collision,
        tuning_path,
    })
}

pub use crate::car_aux::Aux;

