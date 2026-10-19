use crate::statics::place::Iso;
use gearbox_packs::veget::Params;

fn instance_hash(iso: Iso) -> u32 {
    let mut h = 28 ^ 0x5748_9862u32;
    for word in iso.q.into_iter().chain(iso.p).map(f32::to_bits) {
        let k = word.wrapping_mul(0x5bd1_e995);
        h = h.wrapping_mul(0x5bd1_e995) ^ (k ^ (k >> 24)).wrapping_mul(0x5bd1_e995);
    }
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^ (h >> 15)
}

fn next(seed: &mut u32) -> u32 {
    *seed = 0x3039u32.wrapping_sub(seed.wrapping_mul(0x3e39_b193)) & 0x7fff_ffff;
    *seed >> 16
}

fn range(seed: &mut u32, lo: f32, hi: f32) -> f32 {
    (next(seed) as f32 / 32767.0) * (hi - lo) + lo
}

fn rotate(q: [f32; 4], angle: f32, axis: usize) -> [f32; 4] {
    let (s, c) = gearbox_engine::math::sincos(angle * 0.5);
    let mut p = [c, 0.0, 0.0, 0.0];
    p[axis] = s;
    let [w, x, y, z] = q;
    let [c, ax, ay, az] = p;
    [
        w * c - ((ay * y + x * ax) + az * z),
        ((x * c + w * ax) + z * ay) - y * az,
        ((y * c + w * ay) + x * az) - z * ax,
        ((z * c + w * az) + y * ax) - x * ay,
    ]
}

pub fn placement(mut iso: Iso, params: Params) -> (Iso, f32) {
    let mut seed = instance_hash(iso);
    let mut scale_seed = seed;
    let bin = (next(&mut scale_seed) as u64 * 8) >> 15;
    let scale = 1.0 - (bin as f32 / 7.0) * params.scale_variation;
    let tilt = (params.tilt_degrees * std::f32::consts::PI) / 180.0;
    if params.random_y || tilt != 0.0 {
        let yaw = if params.random_y {
            range(&mut seed, 0.0, (360.0 * std::f32::consts::PI) / 180.0)
        } else {
            0.0
        };
        let pitch = range(&mut seed, -tilt, tilt);
        let roll = range(&mut seed, -tilt, tilt);
        iso.q = rotate(iso.q, yaw, 2);
        iso.q = rotate(iso.q, pitch, 1);
        iso.q = rotate(iso.q, roll, 3);
    }
    (iso, scale)
}
