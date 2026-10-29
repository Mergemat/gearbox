use gearbox_engine::collision::{CollisionWorld, Lists, QueryFilter, Ray};
use gearbox_engine::shape::Geometry;
use gearbox_engine::{Iso, Session, Vec3};

pub const FEATURES: usize = 3;
pub const LEN: usize = DIRECTIONS.len() * FEATURES;
pub const EXTRA_FEATURES: usize = 5;
pub const EXTRA_LEN: usize = DIRECTIONS.len() * 2 + EXTRA.len() * EXTRA_FEATURES;

const PITCH_SIN: f32 = 0.052_335_956;
const PITCH_COS: f32 = 0.998_629_5;
const AROUND: [(f32, f32); 9] = [
    (-1.0, 0.0),
    (-0.866_025_4, 0.5),
    (-0.642_787_6, 0.766_044_4),
    (-0.342_020_14, 0.939_692_6),
    (0.0, 1.0),
    (0.342_020_14, 0.939_692_6),
    (0.642_787_6, 0.766_044_4),
    (0.866_025_4, 0.5),
    (1.0, 0.0),
];

const fn around(i: usize) -> Vec3 {
    let (sin, cos) = AROUND[i];
    Vec3 { x: sin * PITCH_COS * 80.0, y: PITCH_SIN * 80.0, z: cos * PITCH_COS * 80.0 }
}

const DIRECTIONS: [Vec3; 12] = [
    around(0),
    around(1),
    around(2),
    around(3),
    around(4),
    around(5),
    around(6),
    around(7),
    around(8),
    Vec3 { x: 0.0, y: -10.0, z: 0.0 },
    Vec3 { x: 0.0, y: 20.0, z: 0.0 },
    Vec3 { x: 0.0, y: -0.342_020_14 * 40.0, z: 0.939_692_6 * 40.0 },
];

const EXTRA: [Vec3; 6] = [
    Vec3 { x: 0.0, y: 0.258_819_04 * 80.0, z: 0.965_925_8 * 80.0 },
    Vec3 { x: 0.0, y: 0.573_576_4 * 80.0, z: 0.819_152_05 * 80.0 },
    Vec3 { x: 0.0, y: 0.866_025_4 * 80.0, z: 0.5 * 80.0 },
    Vec3 { x: 0.707_106_8 * 40.0, y: 0.707_106_8 * 40.0, z: 0.0 },
    Vec3 { x: -0.707_106_8 * 40.0, y: 0.707_106_8 * 40.0, z: 0.0 },
    Vec3 { x: 0.0, y: -0.139_173_1 * 80.0, z: 0.990_268_1 * 80.0 },
];

pub fn read(session: &Session, out: &mut [f32], extra: &mut [f32]) {
    let body = session.body();
    measure(
        Geometry::new(&session.world, &session.state),
        &session.state.collision_world[session.config.manager],
        Iso { pos: body.pos, rot: body.rot },
        out,
        extra,
    );
}

pub fn measure(
    geometry: Geometry<'_>,
    manager: &CollisionWorld,
    pose: Iso,
    out: &mut [f32],
    extra: &mut [f32],
) {
    assert_eq!(out.len(), LEN);
    assert!(extra.is_empty() || extra.len() == EXTRA_LEN);
    let filter = QueryFilter { lists: Lists::SURFACES, layers: 0x12_8002, ..QueryFilter::default() };
    let up = pose.rot * Vec3::new(0.0, 1.0, 0.0);
    let cast = |direction: Vec3| {
        let ray = Ray { origin: pose.pos, direction: pose.rot * direction };
        manager.raycast(geometry, ray, filter).map(|hit| (hit.fraction, hit.normal))
    };
    let (sides, around) = extra.split_at_mut(extra.len().min(DIRECTIONS.len() * 2));
    for (i, (slot, &direction)) in out.as_chunks_mut::<FEATURES>().0.iter_mut().zip(&DIRECTIONS).enumerate() {
        let hit = cast(direction);
        *slot = match hit {
            Some((fraction, normal)) => [fraction, 1.0, normal.dot(up)],
            None => [1.0, 0.0, 0.0],
        };
        if let Some(side) = sides.get_mut(2 * i..2 * i + 2) {
            let normal = hit.map_or(Vec3::ZERO, |(_, normal)| pose.rot.transpose_mul_vec(normal));
            side.copy_from_slice(&[normal.x, normal.z]);
        }
    }
    for (slot, &direction) in around.as_chunks_mut::<EXTRA_FEATURES>().0.iter_mut().zip(&EXTRA) {
        *slot = match cast(direction) {
            Some((fraction, normal)) => {
                let [x, y, z] = pose.rot.transpose_mul_vec(normal).to_array();
                [fraction, 1.0, x, y, z]
            }
            None => [1.0, 0.0, 0.0, 0.0, 0.0],
        };
    }
}
