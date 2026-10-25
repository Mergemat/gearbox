use anyhow::{Context, Result};
use gearbox_engine::{
    Iso, Quat, Vec3,
    ghost::{CloneConfig, Playback, Sample, Track, sin_cos},
    vehicle::ghost::Contact,
};
use gearbox_gbx::{Map, record::CLASS_VEHICLE_VIS};
use std::f32::consts::{FRAC_PI_2, PI};

pub fn build(map: &Map) -> Result<Playback> {
    let mut playback = Playback::default();
    if !map.clones_enabled || map.author_clones.is_empty() {
        return Ok(playback);
    }
    let Some(ghost) = map.author_ghost.as_ref() else {
        return Ok(playback);
    };
    for entity in ghost.record_entities().filter(|entity| entity.class_id == CLASS_VEHICLE_VIS) {
        let mut track = Track { start_ms: entity.start_ms, end_ms: entity.end_ms, ..Default::default() };
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
            let magnitude = if motion.speed == i16::MIN {
                0.0
            } else {
                gearbox_engine::numeric::expf(motion.speed as f32 / 1000.0)
            };
            track.samples.push(Sample {
                time_ms: sample.time_ms,
                sequence: motion.sequence,
                contact: Contact {
                    pose: Iso::new(rotation, Vec3::from_array(motion.position)),
                    velocity: Vec3::new(magnitude * (ch * cp), magnitude * (sh * cp), magnitude * sp),
                },
            });
        }
        playback.tracks.push(track);
    }
    for clone in map.author_clones.iter().take(128) {
        let start = (clone.offset_ms + 10) as f32 * 0.001;
        playback.clones.push(CloneConfig { start, speed: clone.speed.clamp(0.0, 1.0) });
    }
    Ok(playback)
}

#[cfg(test)]
mod tests {
    use super::*;
    use gearbox_gbx::{
        Ghost,
        map::AuthorClone,
        record::{Entity, RecordData},
    };

    fn vehicle(start_ms: i32, end_ms: i32, x: f32) -> Entity {
        let mut raw = vec![0; 69];
        raw[47..51].copy_from_slice(&x.to_le_bytes());
        raw[65..67].copy_from_slice(&i16::MIN.to_le_bytes());
        Entity {
            class_id: CLASS_VEHICLE_VIS,
            start_ms,
            end_ms,
            samples: [start_ms, end_ms]
                .map(|time_ms| gearbox_gbx::record::Sample { time_ms, vehicle: None, raw: raw.clone() })
                .into(),
        }
    }

    #[test]
    fn clones_decode_quantized_velocity() {
        let mut entity = vehicle(0, 100, 0.0);
        for sample in &mut entity.samples {
            sample.raw[65..67].copy_from_slice(&2768i16.to_le_bytes());
            sample.raw[67] = 55;
            sample.raw[68] = 12;
        }
        let map = Map {
            clones_enabled: true,
            author_clones: vec![AuthorClone { offset_ms: -10, speed: 1.0 }],
            author_ghost: Some(Ghost {
                record: Some(RecordData {
                    version: 11,
                    start_ms: 0,
                    end_ms: 100,
                    descs: vec![CLASS_VEHICLE_VIS],
                    entities: vec![entity],
                }),
                record_entity_indices: vec![0],
                ..Default::default()
            }),
            ..Default::default()
        };
        let playback = build(&map).unwrap();
        assert_eq!(playback.contacts(0.05)[0].velocity, Vec3::new(3.2876534, 15.404726, 2.3552055));
    }

    #[test]
    fn clones_follow_assigned_vehicles_across_car_changes() {
        let mut map = Map {
            clones_enabled: true,
            author_clones: vec![AuthorClone { offset_ms: -10, speed: 1.0 }],
            author_ghost: Some(Ghost {
                record: Some(RecordData {
                    version: 11,
                    start_ms: 0,
                    end_ms: 300,
                    descs: vec![CLASS_VEHICLE_VIS],
                    entities: vec![vehicle(0, 300, -100.0), vehicle(0, 100, 10.0), vehicle(110, 300, 20.0)],
                }),
                record_entity_indices: vec![1, 2],
                ..Default::default()
            }),
            ..Default::default()
        };
        let playback = build(&map).unwrap();
        assert_eq!(playback.contacts(0.05)[0].pose.pos.x, 10.0);
        assert_eq!(playback.contacts(0.2)[0].pose.pos.x, 20.0);
        assert!(playback.contacts(0.105).is_empty());
        assert!(playback.contacts(0.31).is_empty());
        map.clones_enabled = false;
        assert!(build(&map).unwrap().contacts(0.2).is_empty());
    }
}
