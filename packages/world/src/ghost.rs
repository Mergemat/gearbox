use anyhow::{Context, Result};
use gearbox_engine::{
    Iso, Quat, Vec3,
    ghost::{CloneConfig, Playback, Sample, sin_cos},
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
        let motion = sample.motion().context("author vehicle sample too short")?;
        let heading = motion.axis_heading as f32 * PI / 32767.0;
        let pitch = motion.axis_pitch as f32 * FRAC_PI_2 / 32767.0;
        let angle = motion.angle as f32 * PI / 65535.0;
        let (sh, ch) = sin_cos(heading);
        let (sp, cp) = sin_cos(pitch);
        let (sa, ca) = sin_cos(angle);
        let rotation = Quat::new(ca, sa * (ch * cp), sa * (sh * cp), sa * sp).to_mat3();
        let heading = motion.velocity_heading as f32 * PI / 127.0;
        let pitch = motion.velocity_pitch as f32 * FRAC_PI_2 / 127.0;
        let (sh, ch) = sin_cos(heading);
        let (sp, cp) = sin_cos(pitch);
        let magnitude =
            if motion.speed == i16::MIN { 0.0 } else { ((motion.speed as f32 / 1000.0) as f64).exp() as f32 };
        playback.samples.push(Sample {
            time_ms: sample.time_ms,
            sequence: motion.sequence,
            contact: Contact {
                pose: Iso::new(rotation, Vec3::from_array(motion.position)),
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
