mod powf_tables;

use crate::math::cvtt;

pub fn lroundf(x: f32) -> i32 {
    if x == 0.0 || !x.is_finite() {
        return 0;
    }
    let r = x.round();
    if -2147483648.0 <= r && r <= 2147483648.0 { cvtt(r) } else { 0 }
}

#[inline]
pub fn roundf(x: f32) -> f32 {
    x.round()
}

#[inline]
pub fn fmodf(x: f32, y: f32) -> f32 {
    x % y
}

pub fn floorf(x: f32) -> f32 {
    let bits = x.to_bits();
    let a = bits & 0x7fff_ffff;
    let neg = bits != a;
    if a < 0x4b80_0000 {
        if a < 0x3f80_0000 {
            return if a == 0 {
                x
            } else if neg {
                -1.0
            } else {
                0.0
            };
        }
        let shift = 0x96u32.wrapping_sub(bits >> 23) & 31;
        let r = bits & (u32::MAX << shift);
        let f = f32::from_bits(r);
        return if neg && bits != r { f - 1.0 } else { f };
    }
    if a <= 0x7f80_0000 { x } else { f32::from_bits(bits | 0x40_0000) }
}

pub fn ceilf(x: f32) -> f32 {
    let bits = x.to_bits();
    let a = bits & 0x7fff_ffff;
    let neg = bits != a;
    if a < 0x4b80_0000 {
        if a < 0x3f80_0000 {
            return if a == 0 {
                x
            } else if neg {
                -0.0
            } else {
                1.0
            };
        }
        let shift = 0x96u32.wrapping_sub(bits >> 23) & 31;
        let r = bits & (u32::MAX << shift);
        let f = f32::from_bits(r);
        return if !neg && bits != r { f + 1.0 } else { f };
    }
    if a <= 0x7f80_0000 { x } else { f32::from_bits(bits | 0x40_0000) }
}

#[inline]
pub fn isnan(x: f64) -> bool {
    x.to_bits() & 0x7fff_ffff_ffff_ffff > 0x7ff0_0000_0000_0000
}

#[inline]
pub fn finite(x: f64) -> bool {
    x.to_bits() & 0x7fff_ffff_ffff_ffff < 0x7ff0_0000_0000_0000
}

#[inline]
pub fn is_finite(x: f32) -> bool {
    let d = x as f64;
    !isnan(d) && finite(d)
}

fn cvtss2si(x: f32) -> i32 {
    x.round_ties_even() as i32
}

fn pow_sign(y: f32, ay: u32, x_sign: u32) -> Option<u32> {
    if ay & 0x7f80_0000 > 0x4b00_0000 {
        return Some(0);
    }
    let n = cvtss2si(y);
    if n as f32 != y {
        return None;
    }
    Some(if n & 1 != 0 { x_sign } else { 0 })
}

pub fn powf(x: f32, y: f32) -> f32 {
    use powf_tables::{EXP2_64, LOG, LOG_INV};
    let d = f64::from_bits;
    const THIRD: u64 = 0x3fd5_5555_5555_5555;
    const LN2: u64 = 0x3fe6_2e42_fefa_39ef;
    const MAX_Z: u64 = 0x4056_2e43_0000_0000;
    const MIN_Z: u64 = 0xc059_d1da_0000_0000;
    const LN2_64: u64 = 0x3f86_2e42_fefa_39ef;
    const SIXTH: u64 = 0x3fc5_5555_5555_5555;
    const INV_LN2_64: u64 = 0x4057_1547_652b_82fe;
    const A: [u64; 2] = [0x3f89_9999_99ba_c6d4, 0x3f3c_8034_c85d_fff0];
    const B: [u64; 2] = [0x3fb5_5555_5555_54e6, 0x3f62_4923_07f1_519f];
    let (xb, yb) = (x.to_bits(), y.to_bits());
    let (ax, ay) = (xb & 0x7fff_ffff, yb & 0x7fff_ffff);
    let one = 0x3f80_0000u32;
    let inf = 0x7f80_0000u32;
    let f = f32::from_bits;
    if ay >= inf {
        if ay > inf {
            if ax <= inf && xb == one && ay >= 0x7fc0_0000 {
                return f(one);
            }
            todo!("powf: NaN y (errno)");
        }
        if ax > inf {
            todo!("powf: NaN x (errno)");
        }
        if ax == one {
            return f(one);
        }
        if yb >> 31 != 0 {
            if ax == 0 {
                return f(inf);
            }
            return f(if ax < one { inf } else { 0 });
        }
        return f(if ax < one { 0 } else { inf });
    }
    if (ay as i32) <= one as i32 {
        if ay == 0 {
            if ax > inf && ax < 0x7fc0_0000 {
                todo!("powf: signalling NaN x (errno)");
            }
            return f(one);
        }
        if yb == one {
            if ax > inf {
                todo!("powf: NaN x (errno)");
            }
            return x;
        }
    }
    if ax >= inf {
        if ay == 0 {
            return f(one);
        }
        if xb == inf {
            return f(if (yb as i32) < 0 { 0 } else { inf });
        }
        if xb == 0xff80_0000 {
            let mag = if (yb as i32) < 0 {
                0
            } else if (yb as i32) > 0 {
                inf
            } else {
                yb
            };
            let s = pow_sign(y, ay, 0x8000_0000).unwrap_or(0);
            return f(s | mag);
        }
        todo!("powf: NaN x (errno)");
    }
    let xd = x as f64;
    let yd = y as f64;
    let mut sign = 0u32;
    let mut near_one = false;
    if (xb as i32) < 0x3f88_0000 {
        if (xb as i32) <= 0 {
            if ax == 0 {
                let (mag, pole) = if (yb as i32) < 0 {
                    (inf, true)
                } else if (yb as i32) > 0 {
                    (0, false)
                } else {
                    (yb, false)
                };
                let s = pow_sign(y, ay, xb & 0x8000_0000).unwrap_or(0);
                if pole {
                    todo!("powf: zero to a negative power (errno)");
                }
                return f(s | mag);
            }
            match pow_sign(y, ay, 0x8000_0000) {
                None => todo!("powf: negative x, non-integer y (errno)"),
                Some(s) => sign = s,
            }
        }
        near_one = ((xd - 1.0).to_bits() & !(1 << 63)) < 0x3fb0_0000_0000_0000;
    }
    let l = if near_one {
        let u = xd - 1.0;
        let s = u / (u + 2.0);
        let su = s * u;
        let v = s + s;
        let v2 = v * v;
        let (p0, p1) = (v2 * d(A[0]) + d(B[0]), v2 * d(A[1]) + d(B[1]));
        let v3 = v * v2;
        let v7 = v2 * v2 * v3;
        u + ((v3 * p0 + v7 * p1) - su)
    } else {
        let bits = xd.to_bits();
        let mant = bits & 0x000f_ffff_ffff_ffff;
        let e = ((bits & 0x7ff0_0000_0000_0000) >> 52) as i64 - 0x3ff;
        let i = (mant >> 44) + ((mant >> 43) & 1);
        let fi = f64::from_bits((i | 0x3fe00) << 44);
        let mm = f64::from_bits(mant | 0x3fe0_0000_0000_0000);
        let r = (fi - mm) * d(LOG_INV[i as usize]);
        let poly = r * r * (d(THIRD) * r + 0.5) + r;
        (e as i32 as f64 * d(LN2) + d(LOG[i as usize])) - poly
    };
    let z = yd * l;
    if z > d(MAX_Z) {
        todo!("powf: overflow (errno)");
    }
    if !(z > d(MIN_Z)) {
        todo!("powf: underflow (errno)");
    }
    let n = (z * d(INV_LN2_64)).round_ties_even() as i32;
    let rr = z - n as f64 * d(LN2_64);
    let q = rr * rr * (d(SIXTH) * rr + 0.5) + rr;
    let t = d(EXP2_64[(n & 0x3f) as usize]);
    let p = q * t + t;
    let bits = p.to_bits().wrapping_add(((n >> 6) as i64 as u64) << 52);
    f((f64::from_bits(bits) as f32).to_bits() | sign)
}

pub fn fmod(x: f64, y: f64) -> f64 {
    const ABS: u64 = 0x7fff_ffff_ffff_ffff;
    const EXP: u64 = 0x7ff0_0000_0000_0000;
    const SIGN: u64 = 0x8000_0000_0000_0000;
    let hi = |v: f64| f64::from_bits(v.to_bits() & 0xffff_ffff_f800_0000);
    let (xb, yb) = (x.to_bits(), y.to_bits());
    let (ax, ay) = (xb & ABS, yb & ABS);
    if ay > EXP {
        return y;
    }
    if ax > EXP {
        return x;
    }
    if ax == EXP {
        todo!("fmod: infinite x (errno)");
    }
    if ay == 0 {
        todo!("fmod: zero y (errno)");
    }
    if ax == ay {
        return f64::from_bits(xb & SIGN);
    }
    let (axd, ayd) = (f64::from_bits(ax), f64::from_bits(ay));
    if !(axd > ayd) {
        return x;
    }
    let ex = ((xb & EXP) >> 52) as i64;
    let ey = ((yb & EXP) >> 52) as i64;
    if ex == 0 || ey == 0 || ex - ey >= 52 {
        return x % y;
    }
    let qd = ((axd / ayd) as i64) as f64;
    let eq = ((qd.to_bits() & EXP) >> 52) as i64 - 0x3ff;
    if eq + (ey - 0x3ff) >= 0x3ff {
        return x % y;
    }
    let p = qd * ayd;
    let (hy, hq) = (hi(ayd), hi(qd));
    let (ly, lq) = (ayd - hy, qd - hq);
    let e = lq * ly + (((hy * hq - p) + hy * lq) + hq * ly);
    let t1 = axd - p;
    let mut r = (((axd - t1) - p) - e) + t1;
    if !(r >= 0.0) {
        r = r + ayd;
    }
    if x > 0.0 { r } else { f64::from_bits(r.to_bits() | SIGN) }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rand {
    pub state: u32,
}

impl Rand {
    pub const fn new(seed: u32) -> Self {
        Self { state: seed }
    }

    pub fn next(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(0x343fd).wrapping_add(0x269ec3);
        self.state >> 16 & 0x7fff
    }

    pub fn range(&mut self, lo: u32, hi: u32) -> u32 {
        let r = self.next() as u64;
        let n = hi.wrapping_sub(lo).wrapping_add(1) as u64;
        ((r.wrapping_mul(n) >> 15) as u32).wrapping_add(lo)
    }
}

pub fn quicksort<T>(v: &mut [T], mut cmp: impl FnMut(&T, &T) -> i32) {
    if v.len() < 2 {
        return;
    }
    let mut c = |v: &[T], a: isize, b: isize| cmp(&v[a as usize], &v[b as usize]);
    let swap = |v: &mut [T], a: isize, b: isize| v.swap(a as usize, b as usize);
    let mut stack: Vec<(isize, isize)> = Vec::new();
    let (mut lo, mut hi) = (0isize, v.len() as isize - 1);
    loop {
        let size = hi - lo + 1;
        if size <= 8 {
            let mut h = hi;
            while lo < h {
                let mut max = lo;
                let mut p = lo + 1;
                while p <= h {
                    if c(v, p, max) > 0 {
                        max = p;
                    }
                    p += 1;
                }
                swap(v, max, h);
                h -= 1;
            }
        } else {
            let mut mid = lo + size / 2;
            if c(v, lo, mid) > 0 {
                swap(v, lo, mid);
            }
            if c(v, lo, hi) > 0 {
                swap(v, lo, hi);
            }
            if c(v, mid, hi) > 0 {
                swap(v, mid, hi);
            }
            let (mut loguy, mut higuy) = (lo, hi);
            loop {
                if mid > loguy {
                    loop {
                        loguy += 1;
                        if !(loguy < mid && c(v, loguy, mid) <= 0) {
                            break;
                        }
                    }
                }
                if mid <= loguy {
                    loop {
                        loguy += 1;
                        if !(loguy <= hi && c(v, loguy, mid) <= 0) {
                            break;
                        }
                    }
                }
                loop {
                    higuy -= 1;
                    if !(higuy > mid && c(v, higuy, mid) > 0) {
                        break;
                    }
                }
                if higuy < loguy {
                    break;
                }
                swap(v, loguy, higuy);
                if mid == higuy {
                    mid = loguy;
                }
            }
            higuy += 1;
            if mid < higuy {
                loop {
                    higuy -= 1;
                    if !(higuy > mid && c(v, higuy, mid) == 0) {
                        break;
                    }
                }
            }
            if mid >= higuy {
                loop {
                    higuy -= 1;
                    if !(higuy > lo && c(v, higuy, mid) == 0) {
                        break;
                    }
                }
            }
            if higuy - lo >= hi - loguy {
                if lo < higuy {
                    stack.push((lo, higuy));
                }
                if loguy < hi {
                    lo = loguy;
                    continue;
                }
            } else {
                if loguy < hi {
                    stack.push((loguy, hi));
                }
                if lo < higuy {
                    hi = higuy;
                    continue;
                }
            }
        }
        match stack.pop() {
            Some((l, h)) => {
                lo = l;
                hi = h;
            }
            None => return,
        }
    }
}
pub fn next_random(seed: &mut u32) -> u32 {
    *seed = seed.wrapping_mul(214013).wrapping_add(2531011);
    *seed >> 16 & 32767
}

pub fn random_range(seed: &mut u32, low: u32, high: u32) -> u32 {
    let random = next_random(seed) as u64;
    let width = high.wrapping_sub(low).wrapping_add(1) as u64;
    ((random.wrapping_mul(width) >> 15) as u32).wrapping_add(low)
}
