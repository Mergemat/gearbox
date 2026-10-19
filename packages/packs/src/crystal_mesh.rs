pub fn position_eq(a: [f32; 3], b: [f32; 3]) -> bool {
    (0..3).all(|i| {
        let scale = crate::reader_math::maxss(1.0, crate::reader_math::maxss(a[i].abs(), b[i].abs()));
        scale * 0.00001 >= (a[i] - b[i]).abs()
    })
}

pub fn grid_hash(p: [i32; 3]) -> u32 {
    let mut h = 12 ^ 0x57489862u32;
    for v in p {
        let k = (v as u32).wrapping_mul(0x5bd1e995);
        h = h.wrapping_mul(0x5bd1e995) ^ (k ^ (k >> 24)).wrapping_mul(0x5bd1e995);
    }
    h = (h ^ (h >> 13)).wrapping_mul(0x5bd1e995);
    h ^ (h >> 15)
}

fn table_size(n: usize) -> usize {
    if n * 4 < 33 {
        return 11;
    }
    let mut p = (n * 4 / 3) | 1;
    while (3..).step_by(2).take_while(|&d| d <= p / d).any(|d| p % d == 0) {
        p += 2;
    }
    p
}

pub fn weld(positions: &[[f32; 3]]) -> (Vec<[f32; 3]>, Vec<u32>) {
    let size = table_size(positions.len() * 2);
    let mut table = vec![u32::MAX; size];
    let mut representatives = Vec::<[f32; 3]>::new();
    let mut remap = Vec::with_capacity(positions.len());
    for &p in positions {
        let scaled = p.map(|v| v * 1000.0);
        let a = grid_hash(scaled.map(|v| crate::reader_math::cvtt(v.floor())));
        let b = grid_hash(scaled.map(|v| crate::reader_math::cvtt((v + 0.02).floor())));
        let hashes = if a == b { vec![a] } else { vec![a, b] };
        let mut slots = Vec::new();
        let mut found = None;
        for hash in hashes {
            let mut slot = hash as usize % size;
            let step = hash as usize % (size - 2) + 1;
            while table[slot] != u32::MAX {
                let i = table[slot];
                if position_eq(representatives[i as usize], p) {
                    found = Some(i);
                    break;
                }
                slot = (slot + step) % size;
            }
            if found.is_some() {
                break;
            }
            slots.push(slot);
        }
        let index = found.unwrap_or_else(|| {
            let index = representatives.len() as u32;
            representatives.push(p);
            for slot in slots {
                table[slot] = index;
            }
            index
        });
        remap.push(index);
    }
    (representatives, remap)
}

pub fn keep_triangle(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> bool {
    let ab: [f32; 3] = std::array::from_fn(|i| b[i] - a[i]);
    let ac: [f32; 3] = std::array::from_fn(|i| c[i] - a[i]);
    let bc: [f32; 3] = std::array::from_fn(|i| c[i] - b[i]);
    let len2 = |p: [f32; 3]| (p[1] * p[1] + p[0] * p[0]) + p[2] * p[2];
    let ab2 = len2(ab);
    let ac2 = len2(ac);
    let lengths = [ab2.sqrt(), ac2.sqrt(), len2(bc).sqrt()];
    let i_min = if lengths[(lengths[1] < lengths[0]) as usize] <= lengths[2] {
        (lengths[1] < lengths[0]) as usize
    } else {
        2
    };
    let i_max = if lengths[2] <= lengths[(lengths[0] < lengths[1]) as usize] {
        (lengths[0] < lengths[1]) as usize
    } else {
        2
    };
    let normal =
        [ac[2] * ab[1] - ac[1] * ab[2], ac[0] * ab[2] - ac[2] * ab[0], ac[1] * ab[0] - ac[0] * ab[1]];
    let area2 = len2(normal);
    !(lengths[i_min] < lengths[i_max] * 0.00001
        || area2 <= (ab2 * 1.0e-10) * ac2
        || area2 < 1.0e-10 * 1.0e-10)
}
