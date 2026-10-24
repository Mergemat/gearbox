use std::f32::consts::PI;
use std::io::Read;

use anyhow::{Context, Result, ensure};
use serde::Serialize;

use crate::reader::Reader;

pub const CLASS_VEHICLE_VIS: u32 = 0x0A01_8000;

#[derive(Debug, Clone, Serialize)]
pub struct RecordData {
    pub version: u32,
    pub start_ms: i32,
    pub end_ms: i32,
    pub descs: Vec<u32>,
    pub entities: Vec<Entity>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Entity {
    pub class_id: u32,
    pub start_ms: i32,
    pub end_ms: i32,
    pub samples: Vec<Sample>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Sample {
    pub time_ms: i32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle: Option<VehicleSample>,
    #[serde(skip)]
    pub raw: Vec<u8>,
}

#[derive(Debug, Clone, Serialize)]
pub struct VehicleSample {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub speed: f32,
    pub velocity: [f32; 3],
    pub side_speed: f32,
    pub steer: f32,
    pub gas: f32,
    pub brake: f32,
    pub rpm: u8,
    pub gear: u8,
    pub turbo_time: f32,
    pub is_turbo: bool,
    pub ground_contact: bool,
    pub top_contact: bool,
    pub wheel_materials: [u8; 4],
    pub wheel_damper: [f32; 4],
    pub wheel_slip: [bool; 4],
    pub wheel_icing: [f32; 4],
    pub wheel_dirt: [f32; 4],
    pub reactor_flags: u8,
    pub sim_time_coef: f32,
}

pub fn parse_node_chunks(r: &mut Reader) -> Result<RecordData> {
    let mut out = None;
    r.chunks("CPlugEntRecordData", |r, id, _| {
        if id != 0x0911_F000 {
            return Ok(false);
        }
        let version = r.u32()?;
        ensure!(version >= 5, "record data version {version} is pre-TM2020 and not supported");
        let uncompressed = r.len()?;
        let compressed = r.data()?;
        let mut payload = Vec::with_capacity(uncompressed);
        flate2::read::ZlibDecoder::new(compressed).read_to_end(&mut payload).context("zlib")?;
        ensure!(payload.len() == uncompressed, "zlib size mismatch");
        out = Some(parse_payload(&payload, version).context("record payload")?);
        Ok(true)
    })?;
    out.context("record data chunk missing")
}

fn parse_payload(buf: &[u8], version: u32) -> Result<RecordData> {
    let mut r = Reader::new(buf);
    let start_ms = r.i32()?;
    let end_ms = r.i32()?;

    let n = r.len()?;
    let mut descs = Vec::with_capacity(n);
    for _ in 0..n {
        descs.push(r.u32()?);
        r.i32()?;
        r.i32()?;
        r.i32()?;
        r.data()?;
        r.i32()?;
    }
    let notices = r.len()?;
    for _ in 0..notices {
        r.i32()?;
        r.i32()?;
        if version >= 4 {
            r.u32()?;
        }
    }

    let mut entities = Vec::new();
    let mut has_next = r.bool8()?;
    while has_next {
        let ty = r.len()?;
        let class_id = *descs.get(ty).context("entity type out of range")?;
        r.i32()?;
        let start_ms = r.i32()?;
        let end_ms = r.i32()?;
        if version >= 6 {
            r.i32()?;
        }
        let raw = if version < 11 { read_deltas(&mut r)? } else { read_columnar(&mut r)? };
        let samples = raw
            .into_iter()
            .map(|(time_ms, raw)| {
                let vehicle =
                    (class_id == CLASS_VEHICLE_VIS && raw.len() >= 103).then(|| decode_vehicle(&raw));
                Sample { time_ms, vehicle, raw }
            })
            .collect();
        entities.push(Entity { class_id, start_ms, end_ms, samples });

        has_next = r.bool8()?;
        if version >= 2 {
            while r.bool8()? {
                r.i32()?;
                r.i32()?;
                r.data()?;
            }
        }
    }
    Ok(RecordData { version, start_ms, end_ms, descs, entities })
}

fn read_deltas(r: &mut Reader) -> Result<Vec<(i32, Vec<u8>)>> {
    let mut out = Vec::new();
    while r.bool8()? {
        let t = r.i32()?;
        out.push((t, r.data()?.to_vec()));
    }
    Ok(out)
}

fn read_columnar(r: &mut Reader) -> Result<Vec<(i32, Vec<u8>)>> {
    let count = r.len()?;
    if count == 0 {
        return Ok(Vec::new());
    }
    let size = r.len()?;
    let mut time = 0;
    let mut out: Vec<(i32, Vec<u8>)> = Vec::with_capacity(count);
    for _ in 0..count {
        time += r.i32()?;
        out.push((time, vec![0; size]));
    }
    for col in 0..size {
        let slice = r.bytes(count)?;
        let mut acc = 0u8;
        for (row, d) in slice.iter().enumerate() {
            acc = acc.wrapping_add(*d);
            out[row].1[col] = acc;
        }
    }
    Ok(out)
}

fn decode_vehicle(d: &[u8]) -> VehicleSample {
    let u8_at = |i: usize| d[i];
    let f = |i: usize| f32::from_le_bytes(d[i..i + 4].try_into().unwrap());
    let i16_at = |i: usize| i16::from_le_bytes(d[i..i + 2].try_into().unwrap());
    let unit = |b: u8| b as f32 / 255.0;
    let signed = |b: u8| (b as f32 / 255.0 - 0.5) * 2.0;

    let position = [f(47), f(51), f(55)];
    let angle = u16::from_le_bytes([d[59], d[60]]) as f32 * PI / u16::MAX as f32;
    let axis_heading = i16_at(61) as f32 * PI / i16::MAX as f32;
    let axis_pitch = i16_at(63) as f32 / i16::MAX as f32 * PI / 2.0;
    let speed = (i16_at(65) as f64 / 1000.0).exp() as f32;
    let vel_heading = (d[67] as i8) as f32 / i8::MAX as f32 * PI;
    let vel_pitch = (d[68] as i8) as f32 / i8::MAX as f32 * PI / 2.0;
    let (sa, ca) = angle.sin_cos();
    let rotation = [
        sa * axis_pitch.cos() * axis_heading.cos(),
        sa * axis_pitch.cos() * axis_heading.sin(),
        sa * axis_pitch.sin(),
        ca,
    ];
    let velocity = [
        speed * vel_pitch.cos() * vel_heading.cos(),
        speed * vel_pitch.cos() * vel_heading.sin(),
        speed * vel_pitch.sin(),
    ];

    let slip1 = u8_at(32);
    let slip2 = u8_at(33);
    VehicleSample {
        position,
        rotation,
        speed,
        velocity,
        side_speed: ((u16::from_le_bytes([d[2], d[3]]) as f64 / 65536.0 - 0.5) * 2000.0) as f32,
        steer: signed(u8_at(14)),
        gas: unit(u8_at(15)) + unit(u8_at(18)),
        brake: unit(u8_at(18)),
        rpm: u8_at(5),
        gear: u8_at(91),
        turbo_time: unit(u8_at(21)),
        is_turbo: u8_at(31) & 0x82 != 0,
        ground_contact: u8_at(89) & 0x1 != 0,
        top_contact: u8_at(76) & 0x20 != 0,
        wheel_materials: [u8_at(24), u8_at(26), u8_at(28), u8_at(30)],
        wheel_damper: [23, 25, 27, 29].map(|i| signed(u8_at(i)) * 2.0),
        wheel_slip: [slip1 & 0x40 != 0, slip2 & 0x1 != 0, slip2 & 0x4 != 0, slip2 & 0x10 != 0],
        wheel_icing: [81, 82, 83, 84].map(|i| unit(u8_at(i))),
        wheel_dirt: [93, 95, 97, 99].map(|i| unit(u8_at(i))),
        reactor_flags: u8_at(89) & !0x1,
        sim_time_coef: unit(u8_at(102)),
    }
}
