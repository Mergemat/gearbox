use crate::{nodes, packs::PackSet, unpack::GbxHead};
use anyhow::{Context, Result, ensure};
use gearbox_engine::{Curve, Key, vehicle::model::SurfaceMaterial};
use gearbox_gbx::reader::{FACADE, Reader};

#[derive(Clone, Debug, PartialEq)]
pub struct Aux {
    pub wheel_impact_max: f32,
    pub wheel_impact_min: f32,
    pub body_impact_max: f32,
    pub body_impact_min: f32,
    pub curves: Vec<Curve>,
    pub groups: Vec<Vec<u32>>,
    pub times: [u32; 4],
    pub materials: Vec<SurfaceMaterial>,
}

fn head(b: &[u8]) -> Result<GbxHead> {
    crate::unpack::parse_head(b).context("truncated car asset")?
}
fn chunk(b: &[u8], class: u32, id: u32) -> Result<Reader<'_>> {
    let l = nodes::layout(b)?;
    ensure!(l.head.class_id == class, "wrong car auxiliary class");
    let matches: Vec<_> = b[l.head.body_start..]
        .windows(4)
        .enumerate()
        .filter(|(_, v)| *v == id.to_le_bytes())
        .map(|(i, _)| i + l.head.body_start)
        .filter(|&p| !l.nodes.iter().skip(1).any(|n| n.start <= p && n.end.is_some_and(|e| p < e)))
        .collect();
    ensure!(matches.len() == 1, "expected one car chunk {id:08x}, got {}", matches.len());
    let mut r = Reader::new(b);
    r.skip(matches[0] + 4)?;
    Ok(r)
}
fn curve(r: &mut Reader<'_>) -> Result<Curve> {
    ensure!(r.u32()? == 3, "car simple curve version");
    let n = r.u32()? as usize;
    ensure!(n < 65536, "car curve count");
    let flags = u32::from(r.u16()?);
    let keys = (0..n).map(|_| Ok(Key { x: r.f32()?, y: r.f32()? })).collect::<Result<_>>()?;
    Ok(Curve::new(flags, keys))
}

pub fn selection(packs: &PackSet, car: &str) -> Result<(String, String)> {
    let own = format!("Vehicles/Items/Cars/{car}.Item.Gbx");
    let b = packs.read_path(&own)?;
    let l = nodes::layout(&b)?;
    let n = l.nodes.iter().find(|n| n.class_id == 0x2e01c000).context("VehicleStruct absent")?;
    selection_at(&crate::prefab::Ctx { buf: &b, layout: &l, path: &own }, n.start)
}

pub(crate) fn selection_at(ctx: &crate::prefab::Ctx<'_>, start: usize) -> Result<(String, String)> {
    let mut r = Reader::new(ctx.buf);
    r.skip(start)?;
    ensure!(r.u32()? == 0x2e01c000 && r.u32()? == 3, "unsupported VehicleStruct version");
    let model = r.u32()?;
    let visual = r.u32()?;
    ensure!(r.u32()? == 0 && r.u32()? == 0 && r.u32()? == FACADE, "unsupported VehicleStruct extra objects");
    Ok((
        ctx.layout.head.ref_path(ctx.path, model).context("vehicle physics reference")?,
        ctx.layout.head.ref_path(ctx.path, visual).context("vehicle visual reference")?,
    ))
}

pub fn load(packs: &PackSet, visual: &str, paths: &[Option<String>; 2]) -> Result<Aux> {
    let b = packs.read_path(visual)?;
    let h = head(&b)?;
    let mut r = chunk(&b, 0x090e7000, 0x090e7000)?;
    ensure!(r.u32()? == 18, "unsupported visual model version");
    let shared_path = h.ref_path(visual, r.u32()?).context("shared visual reference")?;
    let mut r = chunk(&b, 0x090e7000, 0x090e7002)?;
    ensure!(r.u32()? == 5, "unsupported visual thresholds version");
    let wheel_impact_max = r.f32()?;
    let wheel_impact_min = r.f32()?;
    let body_impact_max = r.f32()?;
    let body_impact_min = r.f32()?;
    let b = packs.read_path(&shared_path)?;
    let mut r = chunk(&b, 0x090e8000, 0x090e8021)?;
    ensure!(r.u32()? == 5, "unsupported shared visual curves version");
    let mut curves = Vec::new();
    for _ in 0..4 {
        curves.push(curve(&mut r)?);
    }
    let mut r = chunk(&b, 0x090e8000, 0x090e8012)?;
    ensure!(r.u32()? == 10, "material group array version");
    let n = r.u32()?;
    ensure!(n < 256, "material group count");
    let mut groups = Vec::new();
    for _ in 0..n {
        r.u32()?;
        ensure!(r.u32()? == 0x090e9000 && r.u32()? == 0x090e9000, "material group class/chunk");
        let count = r.u32()?;
        ensure!(count < 256, "material group size");
        let mut ids = Vec::new();
        for _ in 0..count {
            ids.push(r.u32()?);
        }
        ensure!(r.u32()? == FACADE, "material group end");
        groups.push(ids);
    }
    let mut r = chunk(&b, 0x090e8000, 0x090e8023)?;
    ensure!(r.u32()? == 1, "visual impact timers version");
    r.u32()?;
    r.u32()?;
    let times = [r.u32()?, r.u32()?, r.u32()?, r.u32()?];
    let mut materials = Vec::new();
    for path in paths.iter().flatten() {
        materials.extend(read_materials(&packs.read_path(path)?)?);
    }
    Ok(Aux {
        wheel_impact_max,
        wheel_impact_min,
        body_impact_max,
        body_impact_min,
        curves,
        groups,
        times,
        materials,
    })
}

fn read_materials(b: &[u8]) -> Result<Vec<SurfaceMaterial>> {
    let h = head(b)?;
    ensure!(h.class_id == 0x01026000, "not material RefBuffer");
    let mut r = Reader::new(b);
    r.skip(h.body_start)?;
    read_materials_body(&mut r)
}

pub(crate) fn read_materials_body(r: &mut Reader<'_>) -> Result<Vec<SurfaceMaterial>> {
    ensure!(
        r.u32()? == 0x01026000 && r.u32()? == 0x090f1000 && r.u32()? == 1 && r.u32()? == 10,
        "material RefBuffer header"
    );
    let n = r.u32()?;
    ensure!(n < 256, "material count");
    (0..n).map(|_| material(r)).collect()
}

fn material(r: &mut Reader<'_>) -> Result<SurfaceMaterial> {
    r.u32()?;
    ensure!(r.u32()? == 0x090f1000, "material class");
    ensure!(r.u32()? == 0x090f100f, "material scalar chunk");
    let bump_speed_period = r.f32()?;
    let bump_amplitude_scale = r.f32()?;
    ensure!(r.u32()? == 0x090f1010 && r.u32()? == 14, "material v14 chunk");
    let speed_limit_scale = r.f32()?;
    let grip_scale = r.f32()?;
    let thrust_scale = r.f32()?;
    let brake_scale = r.f32()?;
    let index = r.u8()?;
    let bump_scale = r.f32()?;
    let bump_limit = r.f32()?;
    r.skip(16)?;
    let rolling_response = r.f32()?;
    let sliding_response = r.f32()?;
    let contact_response = r.f32()?;
    let bumps_enabled = r.u32()?;
    let bump_period_x = r.f32()?;
    let bump_period_z = r.f32()?;
    let lateral_thrust = r.f32()?;
    let lateral_drag = r.f32()?;
    let gripping = r.u32()?;
    let thrust_by_speed = curve(r)?;
    let steering_angle_by_speed = curve(r)?;
    let steering_visual_rate = r.f32()?;
    let steering_by_speed = curve(r)?;
    let grip_by_immersion = curve(r)?;
    let immersion_brake_drain = r.f32()?;
    let steering_rate = r.f32()?;
    let pivot_enabled = r.u32()?;
    let thrust_by_alignment = curve(r)?;
    let yaw_drag = r.f32()?;
    ensure!(r.u32()? == FACADE, "material end");
    Ok(SurfaceMaterial {
        speed_limit_scale,
        thrust_scale,
        brake_scale,
        grip_scale,
        rolling_response,
        sliding_response,
        contact_response,
        lateral_drag,
        lateral_thrust,
        yaw_drag,
        gripping,
        immersion_brake_drain,
        steering_rate,
        steering_visual_rate,
        pivot_enabled,
        bump_scale,
        bump_limit,
        bumps_enabled,
        bump_period_x,
        bump_period_z,
        index,
        thrust_by_speed,
        steering_angle_by_speed,
        steering_by_speed,
        grip_by_immersion,
        thrust_by_alignment,
        bump_speed_period,
        bump_amplitude_scale,
    })
}
