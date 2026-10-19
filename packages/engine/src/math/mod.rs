mod aabb;
mod iso;
mod mat3;
mod quat;
mod vec3;

pub use aabb::Aabb;
pub use iso::Iso;
pub use mat3::Mat3;
pub use quat::Quat;
pub use vec3::Vec3;

const fn d(bits: u64) -> f64 {
    f64::from_bits(bits)
}

#[inline]
pub fn maxss(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

#[inline]
pub fn minss(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

#[inline]
pub fn cvtt(x: f32) -> i32 {
    if x.is_nan() || !(-2147483648.0..2147483648.0).contains(&x) { i32::MIN } else { x as i32 }
}

#[inline]
pub fn cvtt64(x: f32) -> i64 {
    if x.is_nan() || !(-9.223372e18..9.223372e18).contains(&x) { i64::MIN } else { x as i64 }
}

#[inline]
pub fn cvtt64d(x: f64) -> i64 {
    if x.is_nan() || !(-9.223372036854775808e18..9.223372036854775808e18).contains(&x) {
        i64::MIN
    } else {
        x as i64
    }
}

pub fn sincos(mut a: f32) -> (f32, f32) {
    const INV_TWO_PI: f32 = 0.15915494;
    const TWO_PI: f32 = 6.2831855;
    const HALF_PI: f32 = 1.5707964;
    const PI: f32 = 3.1415927;
    let q = if a < 0.0 { a * INV_TWO_PI - 0.5 } else { a * INV_TWO_PI + 0.5 };
    a -= cvtt(q) as f32 * TWO_PI;
    let sign;
    if a <= HALF_PI {
        if a < -HALF_PI {
            a = -PI - a;
            sign = -1.0;
        } else {
            sign = 1.0;
        }
    } else {
        a = PI - a;
        sign = -1.0;
    }
    let x2 = a * a;
    let t = x2 * 2.6051615e-7;
    let sin = (((((2.7525562e-6 - x2 * 2.3889859e-8) * x2 - 0.00019840874) * x2 + 0.008333331) * x2
        - 0.16666667)
        * x2
        + 1.0)
        * a;
    let cos =
        (((((0.000024760495 - t) * x2 - 0.0013888378) * x2 + 0.041666638) * x2 - 0.5) * x2 + 1.0) * sign;
    (sin, cos)
}

pub fn sincos_precise(x: f32) -> (f32, f32) {
    let mut sin_neg = 0.0 > x;
    let ax = x.abs() as f64;
    let mut q = crate::numeric::floorf((ax * d(0x3ff4_5f30_6dc9_c882)) as f32);
    let e = cvtt(q);
    if e & 1 != 0 {
        q += 1.0;
    }
    let mut j = if e & 1 != 0 { e.wrapping_add(1) } else { e } & 7;
    let mut swapped = false;
    if j > 3 {
        j -= 4;
        swapped = true;
        sin_neg = !sin_neg;
    }
    let cos_neg = if j <= 1 { swapped } else { !swapped };
    let qd = q as f64;
    let r =
        ax - qd * d(0x3fe9_21fb_4000_0000) - qd * d(0x3e64_442d_0000_0000) - qd * d(0x3ce8_4698_98cc_5170);
    let z = r * r;
    let zr = z * r;
    let mut s = z * d(0x3de5_d8fd_1fd1_9ccd) - d(0x3e5a_e5e5_a929_1f5d);
    s = s * z + d(0x3ec7_1de3_567d_48a1);
    s = s * z - d(0x3f2a_01a0_19bf_df03);
    s = s * z + d(0x3f81_1111_1110_f7d0);
    s = s * z - d(0x3fc5_5555_5555_5548);
    s = s * zr + r;
    let s = s as f32;
    let mut c = d(0x3e21_ee9d_7b4e_3f05) - z * d(0x3da8_fa49_a086_1a9b);
    c = c * z - d(0x3e92_7e4f_7eac_4bc6);
    c = c * z + d(0x3efa_01a0_19c8_44f5);
    c = c * z - d(0x3f56_c16c_16c1_4f91);
    c *= z;
    let c =
        (c + d(0x3fa5_5555_5555_554b)) * (z * z) + (d(0x3ff0_0000_0000_0000) - z * d(0x3fe0_0000_0000_0000));
    let c = c as f32;
    let neg = |v: f32, n: bool| if n { -v } else { v };
    if (j as u32).wrapping_sub(1) > 1 {
        (neg(s, sin_neg), neg(c, cos_neg))
    } else {
        (neg(c, sin_neg), neg(s, cos_neg))
    }
}

fn asin_core(ax: f32) -> f32 {
    const HALF_PI: f32 = 1.5707963;
    let t = maxss(1.0 - ax, 0.0);
    t.sqrt()
        * (((((((0.00667009 - ax * 0.0012624911) * ax - 0.017088126) * ax + 0.03089188) * ax - 0.050174303)
            * ax
            + 0.08897899)
            * ax
            - 0.2145988)
            * ax
            + HALF_PI)
}

pub fn asin(x: f32) -> f32 {
    const HALF_PI: f32 = 1.5707963;
    let p = asin_core(x.abs());
    if x < 0.0 { p - HALF_PI } else { HALF_PI - p }
}

pub fn acos(x: f32) -> f32 {
    let p = asin_core(x.abs());
    if x >= 0.0 { p } else { 3.1415925 - p }
}

pub fn angle_approx(mut a: f32, mut b: f32) -> f32 {
    let (half, nhalf) = (1.5707964, -1.5707964);
    let (k1, k2, k3) = (0.678484, 0.08595542, 0.43157974);
    if 1.0e-9 > a.abs() + b.abs() {
        b *= 1.0e12;
        a *= 1.0e12;
    }
    let (bb, aa) = (b * b, a * a);
    let sa = if 0.0 > a { nhalf } else { half };
    if aa > bb {
        let den = (bb * k1 + aa) * (bb * k2 + aa);
        if den == 0.0 {
            return sa;
        }
        return sa - (bb * k3 + aa) * (a * b) / den;
    }
    let den = (aa * k1 + bb) * (aa * k2 + bb);
    let ab = a * b;
    let sab = if 0.0 > ab { nhalf } else { half };
    if den == 0.0 {
        return sa - sab;
    }
    (aa * k3 + bb) * ab / den + sa - sab
}

pub fn fast_exp2(x: f32) -> f32 {
    let one = if 0.0 > x { 1.0 } else { 0.0 };
    let c = maxss(-126.0, x);
    let i = cvtt(c);
    let frac = (c - i as f32) + one;
    let c2 = c + 121.274055;
    let a = 27.728024 / (4.8425255 - frac) + c2;
    let r = (a - frac * 1.4901291) * 8388608.0;
    f32::from_bits(cvtt64(r) as u32)
}

pub fn fast_log2(x: f32) -> f32 {
    let bits = x.to_bits();
    let mant = f32::from_bits(bits & 0x7f_ffff | 0x3f00_0000);
    ((bits as f32 * 1.1920929e-7 - 124.22552) - mant * 1.4980303) - 1.72588 / (mant + 0.35208872)
}

pub fn ease_in_out(x: f32) -> f32 {
    let a = (x + x) * x;
    if 0.5 > x { a } else { x * 4.0 - a - 1.0 }
}

#[inline]
pub fn quantize(x: f32, step: f32) -> f32 {
    crate::numeric::lroundf(x / step) as f32 * step
}

pub fn triangle_normal(a: Vec3, b: Vec3, c: Vec3) -> Option<Vec3> {
    let (ca, ba) = (c - a, b - a);
    let n = Vec3::new(ca.z * ba.y - ca.y * ba.z, ca.x * ba.z - ca.z * ba.x, ca.y * ba.x - ca.x * ba.y);
    let l2 = n.dot(n);
    if 1.00000005e-20 <= l2 { Some(n * (1.0 / l2.sqrt())) } else { None }
}
