fn d(bits: u64) -> f64 {
    f64::from_bits(bits)
}

pub(super) fn acos(x: f32) -> f32 {
    let ax = x.abs();
    if ax < f32::from_bits(0x3280_0000) {
        return std::f32::consts::FRAC_PI_2;
    }
    if ax >= 1.0 || x.is_nan() {
        return if x == 1.0 {
            0.0
        } else if x == -1.0 {
            std::f32::consts::PI
        } else {
            f32::NAN
        };
    }
    let z = if ax >= 0.5 { (1.0 - ax) * 0.5 } else { ax * ax };
    let numerator =
        (f32::from_bits(0xbc5b_3fe1) - z * f32::from_bits(0x3b81_ce6b)) * z - f32::from_bits(0x3d67_8bdd);
    let numerator = (numerator * z + f32::from_bits(0x3e3c_94dc)) * z;
    let denominator = f32::from_bits(0x3f8d_6fa5) - z * f32::from_bits(0x3f56_1f0d);
    let r = numerator / denominator;
    let tail = d(0x3c91_a626_3314_5c07);
    if ax < 0.5 {
        (std::f64::consts::FRAC_PI_2 - (x as f64 - (tail - (r * x) as f64))) as f32
    } else {
        let s = z.sqrt();
        if x < 0.0 {
            let correction = ((r * s) as f64 - tail) + s as f64;
            (std::f64::consts::PI - (correction + correction)) as f32
        } else {
            let head = f32::from_bits(s.to_bits() & 0xffff_0000);
            let correction = (z - head * head) / (head + s);
            ((s + s) * r + (correction + correction)) + head * 2.0
        }
    }
}

fn sine(x: f64) -> f64 {
    let z = x * x;
    let z2 = z * z;
    let a = d(0x3ec7_1de3_a556_c734) * z + d(0xbf2a_01a0_1a01_a01a);
    let b = d(0x3f81_1111_1111_1111) * z + d(0xbfc5_5555_5555_5555);
    (a * z2 + b) * (x * z) + x
}

fn cosine(x: f64) -> f64 {
    let z = x * x;
    let z2 = z * z;
    let a = d(0xbe92_7e4f_b778_9f5c) * z + d(0x3efa_01a0_1a01_a019);
    let b = d(0xbf56_c16c_16c1_6c16) * z + d(0x3fa5_5555_5555_5555);
    (d(0xbfe0_0000_0000_0000) * z + 1.0) + (a * z2 + b) * z2
}

pub fn sin_cos(angle: f32) -> (f32, f32) {
    let x = angle as f64;
    let ax = x.abs();
    if ax <= d(0x3fe9_21fb_5444_2d18) {
        let sin = if ax < d(0x3f20_0000_0000_0000) {
            x
        } else if ax < d(0x3f80_0000_0000_0000) {
            x - ((x * x) * x) * d(0x3fc5_5555_5555_5555)
        } else {
            sine(x)
        };
        return (sin as f32, cosine(x) as f32);
    }
    let quadrant = (ax * d(0x3fe4_5f30_6dc9_c883) + 0.5) as i32;
    let q = quadrant as f64;
    let mut head = ax - d(0x3ff9_21fb_5440_0000) * q;
    let mut r = head - d(0x3dd0_b461_1a62_6331) * q;
    let exponent = (r.to_bits() >> 52) & 0x7ff;
    if ((ax.to_bits() >> 52) & 0x7ff) as i32 - exponent as i32 > 15 {
        let previous = head;
        let correction = d(0x3dd0_b461_1a60_0000) * q;
        head -= correction;
        let tail = d(0x3ba3_198a_2e03_7073) * q - ((previous - head) - correction);
        r = head - tail;
    }
    let (s, c) = if exponent < 0x3f2 {
        if exponent <= 0x3de {
            (r, 1.0)
        } else {
            let z = r * r;
            (r - (d(0x3fc5_5555_5555_5555) * r) * z, 1.0 - z * 0.5)
        }
    } else {
        (sine(r), cosine(r))
    };
    let (s, c) = match quadrant & 3 {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        _ => (-c, s),
    };
    ((if angle < 0.0 { -s } else { s }) as f32, c as f32)
}

#[cfg(test)]
mod tests {
    use super::{acos, sin_cos};

    #[test]
    fn small_rotation_angles() {
        for (dot, expected) in [(0.9995362, 0x3cf9_8150), (0.9994528, 0x3d07_8158)] {
            assert_eq!(acos(dot).to_bits(), expected);
        }
    }

    #[test]
    fn compressed_rotation_angles() {
        for (angle, expected) in [
            (0x3f4b_a463, [0x3f36_d5a8, 0x3f33_2f8a]),
            (0xbfca_22c1, [0xbf7f_fdb2, 0xbc09_72c5]),
            (0xbbd4_1061, [0xbbd4_1000, 0x3f7f_fea1]),
        ] {
            let (s, c) = sin_cos(f32::from_bits(angle));
            assert_eq!([s.to_bits(), c.to_bits()], expected);
        }
    }
}
