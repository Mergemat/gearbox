use gearbox_engine::collision::{CollisionWorld, Lists, QueryFilter, Ray};
use gearbox_engine::shape::Geometry;
use gearbox_engine::{Iso, Session, Vec3};

pub const FEATURES: usize = 3;
pub const LEN: usize = DIRECTIONS.len() * FEATURES;

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

pub fn read(session: &Session, out: &mut [f32]) {
    let body = session.body();
    measure(
        Geometry::new(&session.world, &session.state),
        &session.state.collision_world[session.config.manager],
        Iso { pos: body.pos, rot: body.rot },
        out,
    );
}

pub fn measure(geometry: Geometry<'_>, manager: &CollisionWorld, pose: Iso, out: &mut [f32]) {
    assert_eq!(out.len(), LEN);
    let filter = QueryFilter { lists: Lists::SURFACES, layers: 0x12_8002, ..QueryFilter::default() };
    let up = pose.rot * Vec3::new(0.0, 1.0, 0.0);
    for (slot, &direction) in out.as_chunks_mut::<FEATURES>().0.iter_mut().zip(&DIRECTIONS) {
        let ray = Ray { origin: pose.pos, direction: pose.rot * direction };
        *slot = match manager.raycast(geometry, ray, filter) {
            Some(hit) => [hit.fraction, 1.0, hit.normal.dot(up)],
            None => [1.0, 0.0, 0.0],
        };
    }
}
