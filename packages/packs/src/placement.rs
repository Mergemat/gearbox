use crate::reader_math::{matrix_to_quat_values, quat_matrix};

pub const CELL: [f32; 3] = [32.0, 8.0, -64.0];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Iso {
    pub q: [f32; 4],
    pub p: [f32; 3],
}

pub fn dir_rotation(dir: u32) -> u32 {
    match dir {
        0 => 0,
        1 => 3,
        2 => 2,
        3 => 1,
        _ => u32::MAX,
    }
}

pub fn quarter_turn(k: u32) -> [f32; 9] {
    const T: [f32; 4] = [1.0, 0.0, -1.0, 0.0];
    let c = T[(k & 3) as usize];
    let s = T[(k.wrapping_sub(1) & 3) as usize];
    [c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c]
}

pub fn grid_location(coord: [i32; 3], dir: u32, size: [u32; 3]) -> ([f32; 9], [f32; 3]) {
    grid_location_cell(CELL, coord, dir, size)
}

pub fn grid_location_cell(cell: [f32; 3], coord: [i32; 3], dir: u32, size: [u32; 3]) -> ([f32; 9], [f32; 3]) {
    let (fx, fy, y0) = (cell[0], cell[1], cell[2]);
    let fz = fx;
    let px = coord[0] as f32 * fx;
    let pz = coord[2] as f32 * fz;
    let py = coord[1] as f32 * fy + y0;
    let (mut x, mut z) = (px, pz);
    match dir {
        1 => x = (size[2] as u64 as f32) * fx + px,
        2 => {
            x = (size[0] as u64 as f32) * fx + px;
            z = (size[2] as u64 as f32) * fz + pz;
        }
        3 => z = (size[0] as u64 as f32) * fx + pz,
        _ => {}
    }
    (quarter_turn(dir_rotation(dir)), [x, py, z])
}

pub fn rotation_quat(r: [f32; 9]) -> [f32; 4] {
    matrix_to_quat_values(r)
}

pub fn quat_mul(b: [f32; 4], p: [f32; 4]) -> [f32; 4] {
    let [bw, bx, by, bz] = b;
    let [w, x, y, z] = p;
    [
        w * bw - ((x * bx + y * by) + z * bz),
        ((w * bx + x * bw) + z * by) - y * bz,
        ((w * by + y * bw) + x * bz) - z * bx,
        ((w * bz + z * bw) + y * bx) - x * by,
    ]
}

pub fn quat_rotate(v: [f32; 3], q: [f32; 4]) -> [f32; 3] {
    let [vx, vy, vz] = v;
    let [w, qx, qy, qz] = q;
    let a = (vz * qy - vy * qz) + vx * w;
    let b = (vx * qz - qx * vz) + w * vy;
    let c = (qx * vy - vx * qy) + w * vz;
    let ox = c * qy - b * qz;
    let oy = a * qz - c * qx;
    let oz = b * qx - a * qy;
    [(ox + ox) + vx, (oy + oy) + vy, (oz + oz) + vz]
}

pub fn compose(block: Iso, part: Iso) -> Iso {
    let q = quat_mul(block.q, part.q);
    let r = quat_rotate(part.p, block.q);
    Iso { q, p: [r[0] + block.p[0], r[1] + block.p[1], r[2] + block.p[2]] }
}

pub fn transform(t: Iso) -> [u32; 12] {
    let r = quat_matrix(t.q);
    let mut out = [0u32; 12];
    for i in 0..9 {
        out[i] = r[i].to_bits();
    }
    for i in 0..3 {
        out[9 + i] = t.p[i].to_bits();
    }
    out
}

pub fn grid_iso(coord: [i32; 3], dir: u32, size: [u32; 3]) -> Iso {
    let (r, p) = grid_location(coord, dir, size);
    Iso { q: rotation_quat(r), p }
}

pub const IDENTITY: Iso = Iso { q: [1.0, 0.0, 0.0, 0.0], p: [0.0, 0.0, 0.0] };

pub fn angles_quat(a: f32, b: f32, c: f32) -> [f32; 4] {
    crate::reader_math::angles_quat(a, b, c)
}

pub fn free_iso(pos: [f32; 3], angles: [f32; 3]) -> Iso {
    free_iso_variant(pos, angles, VariantOffset::default())
}

pub fn free_iso_variant(pos: [f32; 3], angles: [f32; 3], v: VariantOffset) -> Iso {
    let (r, p) = free_location_variant(pos, angles, v);
    Iso { q: rotation_quat(r), p }
}

pub fn free_location_variant(pos: [f32; 3], angles: [f32; 3], v: VariantOffset) -> ([f32; 9], [f32; 3]) {
    let q = angles_quat(angles[0], angles[1], angles[2]);
    let l = (quat_matrix(q), pos);
    if variant_applies(v) { compose_iso(variant_transform(v), l) } else { l }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VariantOffset {
    pub pos: [f32; 3],
    pub deg: [f32; 3],
}

pub fn variant_transform(v: VariantOffset) -> ([f32; 9], [f32; 3]) {
    use crate::reader_math::sincos_precise;
    const PI: f32 = 3.1415927;
    let rad = |d: f32| (d * PI) / 180.0;
    let mut o = [0f32; 9];
    let (s1, c1) = sincos_precise(rad(v.deg[0]));
    o[4] = c1;
    o[5] = -s1;
    o[7] = s1;
    o[8] = c1;
    o[0] = 1.0;
    let (s2, c2) = sincos_precise(rad(v.deg[1]));
    let n2 = -s2;
    o[6] = c2 * 0.0 + n2;
    let o0 = s2 * 0.0 + c2;
    o[0] = o0;
    let o1 = s1 * s2 + c2 * 0.0;
    o[1] = o1;
    o[7] = s1 * c2 + n2 * 0.0;
    let o2 = c1 * s2 + 0.0 * c2;
    o[2] = o2;
    o[8] = c1 * c2 + 0.0 * n2;
    let (s3, c3) = sincos_precise(rad(v.deg[2]));
    let n3 = -s3;
    o[0] = o0 * c3 + n3 * 0.0;
    o[3] = o[3] * c3 + o0 * s3;
    let (r4, r5) = (o[4], o[5]);
    o[4] = r4 * c3 + o1 * s3;
    o[1] = r4 * n3 + o1 * c3;
    o[5] = r5 * c3 + o2 * s3;
    o[2] = r5 * n3 + o2 * c3;
    (o, v.pos)
}

pub fn compose_iso(a: ([f32; 9], [f32; 3]), b: ([f32; 9], [f32; 3])) -> ([f32; 9], [f32; 3]) {
    let (ar, ap) = a;
    let (br, bp) = b;
    let [a0, a1, a2, a3, a4, a5, a6, a7, a8] = ar;
    let r = [
        a3 * br[1] + a0 * br[0] + a6 * br[2],
        a4 * br[1] + a1 * br[0] + a7 * br[2],
        a5 * br[1] + a2 * br[0] + a8 * br[2],
        a3 * br[4] + a0 * br[3] + a6 * br[5],
        a4 * br[4] + a1 * br[3] + a7 * br[5],
        a5 * br[4] + a2 * br[3] + a8 * br[5],
        a3 * br[7] + a0 * br[6] + a6 * br[8],
        a4 * br[7] + a1 * br[6] + a7 * br[8],
        a5 * br[7] + a2 * br[6] + a8 * br[8],
    ];
    let p = [
        br[1] * ap[1] + ap[0] * br[0] + br[2] * ap[2] + bp[0],
        br[4] * ap[1] + ap[0] * br[3] + br[5] * ap[2] + bp[1],
        br[7] * ap[1] + br[6] * ap[0] + br[8] * ap[2] + bp[2],
    ];
    (r, p)
}

pub fn variant_applies(v: VariantOffset) -> bool {
    v.pos.iter().chain(&v.deg).any(|&x| x.abs() > x.abs().max(1.0) * 0.00001)
}

pub fn grid_iso_variant(coord: [i32; 3], dir: u32, size: [u32; 3], v: VariantOffset) -> Iso {
    grid_iso_in(CELL, None, coord, dir, size, v)
}

pub fn grid_iso_in(cell: [f32; 3], loc: Option<([f32; 9], [f32; 3])>, coord: [i32; 3], dir: u32, size: [u32; 3], v: VariantOffset) -> Iso {
    let (r, p) = grid_location_in(cell, loc, coord, dir, size, v);
    Iso { q: rotation_quat(r), p }
}

pub fn grid_location_in(cell: [f32; 3], loc: Option<([f32; 9], [f32; 3])>, coord: [i32; 3], dir: u32, size: [u32; 3], v: VariantOffset) -> ([f32; 9], [f32; 3]) {
    let mut l = grid_location_cell(cell, coord, dir, size);
    if let Some(loc) = loc {
        l = compose_iso(l, loc);
    }
    if variant_applies(v) { compose_iso(variant_transform(v), l) } else { l }
}

pub fn transform_matrix(l: ([f32; 9], [f32; 3])) -> [u32; 12] {
    let mut out = [0u32; 12];
    for i in 0..9 {
        out[i] = l.0[i].to_bits();
    }
    for i in 0..3 {
        out[9 + i] = l.1[i].to_bits();
    }
    out
}
