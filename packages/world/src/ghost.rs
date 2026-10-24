use anyhow::{Result, ensure};
use gearbox_engine::{
    Iso, Quat, Vec3,
    ghost::{CloneConfig, Playback, Sample},
    math::sincos_precise,
    vehicle::ghost::Contact,
};
use gearbox_gbx::{Map, record::CLASS_VEHICLE_VIS};
use std::f32::consts::{FRAC_PI_2, PI};

pub fn build(map: &Map) -> Result<Playback> {
    let mut playback = Playback::default();
    if !map.clones_enabled || map.author_clones.is_empty() {
        return Ok(playback);
    }
    let Some(record) = map.author_ghost.as_ref().and_then(|ghost| ghost.record.as_ref()) else {
        return Ok(playback);
    };
    let Some(entity) = record.entities.iter().find(|entity| entity.class_id == CLASS_VEHICLE_VIS) else {
        return Ok(playback);
    };
    playback.start_ms = entity.start_ms;
    playback.end_ms = entity.end_ms;
    for sample in &entity.samples {
        let raw = &sample.raw;
        ensure!(raw.len() >= 73, "author vehicle sample too short");
        let short = |at| i16::from_le_bytes(raw[at..at + 2].try_into().unwrap());
        let float = |at| f32::from_le_bytes(raw[at..at + 4].try_into().unwrap());
        let heading = short(61) as f32 * PI / 32767.0;
        let pitch = short(63) as f32 * FRAC_PI_2 / 32767.0;
        let angle = u16::from_le_bytes(raw[59..61].try_into().unwrap()) as f32 * PI / 65535.0;
        let (sh, ch) = sincos_precise(heading);
        let (sp, cp) = sincos_precise(pitch);
        let (sa, ca) = sincos_precise(angle);
        let rotation = Quat::new(ca, sa * (ch * cp), sa * (sh * cp), sa * sp).to_mat3();
        let heading = raw[67] as i8 as f32 * PI / 127.0;
        let pitch = raw[68] as i8 as f32 * FRAC_PI_2 / 127.0;
        let (sh, ch) = sincos_precise(heading);
        let (sp, cp) = sincos_precise(pitch);
        let magnitude = short(65);
        let magnitude =
            if magnitude == i16::MIN { 0.0 } else { ((magnitude as f32 / 1000.0) as f64).exp() as f32 };
        playback.samples.push(Sample {
            time_ms: sample.time_ms,
            sequence: raw[43] >> 4,
            contact: Contact {
                pose: Iso::new(rotation, Vec3::new(float(47), float(51), float(55))),
                velocity: Vec3::new(magnitude * (ch * cp), magnitude * (sh * cp), magnitude * sp),
            },
        });
    }
    for clone in map.author_clones.iter().take(128) {
        let start = (clone.offset_ms + 10) as f32 * 0.001;
        playback.clones.push(CloneConfig { start, speed: clone.speed.clamp(0.0, 1.0) });
    }
    Ok(playback)
}
