use super::{Mat3, Vec3, quantize, sincos, sincos_precise};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

const MIN_INV_NORM: f32 = 0.00001;

impl Quat {
    pub const IDENTITY: Quat = Quat::new(1.0, 0.0, 0.0, 0.0);

    #[inline]
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    #[inline]
    pub const fn from_array([w, x, y, z]: [f32; 4]) -> Self {
        Self { w, x, y, z }
    }

    #[inline]
    pub const fn to_array(self) -> [f32; 4] {
        [self.w, self.x, self.y, self.z]
    }

    #[inline]
    pub fn to_mat3(self) -> Mat3 {
        Mat3::from_quat(self)
    }

    pub fn from_mat3(m: &Mat3) -> Quat {
        let e = m.to_array();
        let m8 = e[8];
        let tr = e[0] + e[4] + m8;
        let mut q = [0f32; 4];
        if 0.0 < tr {
            let s = (tr + 1.0).sqrt();
            let r = 0.5 / s;
            q[0] = s * 0.5;
            q[1] = (e[7] - e[5]) * r;
            q[2] = (e[2] - e[6]) * r;
            q[3] = (e[3] - e[1]) * r;
            return Quat::from_array(q);
        }
        const NEXT: [usize; 3] = [1, 2, 0];
        let mut i = (e[0] < e[4]) as usize;
        let mi = e[i * 4];
        if mi <= m8 && m8 != mi {
            i = 2;
        }
        let j = NEXT[i];
        let kk = NEXT[j];
        let s = ((e[i * 4] - (e[kk * 4] + e[j * 4])) + 1.0).sqrt();
        let r = 0.5 / s;
        q[i + 1] = s * 0.5;
        q[0] = (e[kk * 3 + j] - e[j * 3 + kk]) * r;
        q[j + 1] = (e[i * 3 + j] + e[j * 3 + i]) * r;
        q[kk + 1] = (e[i * 3 + kk] + e[kk * 3 + i]) * r;
        Quat::from_array(q)
    }

    pub fn rotate(self, v: Vec3) -> Vec3 {
        let Quat { w: q0, x: q1, y: q2, z: q3 } = self;
        let Vec3 { x: v0, y: v1, z: v2 } = v;
        let t8 = (v2 * q2 - v1 * q3) + v0 * q0;
        let t9 = (v0 * q3 - q1 * v2) + q0 * v1;
        let t6 = (q1 * v1 - v0 * q2) + q0 * v2;
        let c10 = t9 * q1 - t8 * q2;
        let c7 = t6 * q2 - t9 * q3;
        let c5 = t8 * q3 - t6 * q1;
        Vec3::new(c7 + c7 + v0, c5 + c5 + v1, c10 + c10 + v2)
    }

    pub fn integrate(self, w: Vec3, dt: f32) -> Quat {
        let Vec3 { x: wx, y: wy, z: wz } = w;
        let Quat { w: a, x: b, y: c, z: d } = self;
        let n0 = (((-wx) * b - wy * c) - wz * d) * 0.5 * dt + a;
        let n1 = ((d * wy + a * wx) - wz * c) * 0.5 * dt + b;
        let n2 = ((a * wy - d * wx) + wz * b) * 0.5 * dt + c;
        let n3 = ((c * wx - wy * b) + a * wz) * 0.5 * dt + d;
        let inv = 1.0 / (n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3).sqrt();
        if inv <= MIN_INV_NORM || f32::MAX <= inv {
            Quat::IDENTITY
        } else {
            Quat::new(n0 * inv, n1 * inv, inv * n2, inv * n3)
        }
    }

    pub fn normalised_or_identity(self) -> Quat {
        let Quat { w, x, y, z } = self;
        let n = x * x + w * w + y * y + z * z;
        let inv = 1.0 / n.sqrt();
        if inv > MIN_INV_NORM && f32::MAX > inv {
            Quat::new(w * inv, x * inv, y * inv, z * inv)
        } else {
            Quat::IDENTITY
        }
    }

    pub fn angular_velocity_to(self, to: Quat, dt: f32) -> Vec3 {
        let Quat { w: a0, x: a1, y: a2, z: a3 } = self.normalised_or_identity();
        let Quat { w: b0, x: b1, y: b2, z: b3 } = to.normalised_or_identity();
        let s = 2.0 / dt;
        let (d0, d1, d2, d3) = ((b0 - a0) * s, (b1 - a1) * s, (b2 - a2) * s, (b3 - a3) * s);
        let (n1, n2, n3) = (-a1, -a2, -a3);
        Vec3::new(
            d1 * a0 + d0 * n1 + d2 * n3 - d3 * n2,
            d2 * a0 + d0 * n2 + d3 * n1 - d1 * n3,
            d3 * a0 + d0 * n3 + d1 * n2 - d2 * n1,
        )
    }

    pub fn about_y(angle: f32) -> Quat {
        let (s, c) = sincos(angle * 0.5);
        Quat::new(c, 0.0, s, 0.0)
    }

    pub fn from_angles(a: f32, b: f32, c: f32) -> Quat {
        let (sa, ca) = sincos_precise(a * 0.5);
        let (sc, cc) = sincos_precise(c * 0.5);
        let (sb, cb) = sincos_precise(b * 0.5);
        let scsa = sc * sa;
        let ccca = cc * ca;
        let ccsa = cc * sa;
        let scca = sc * ca;
        Quat::new(
            scsa * sb - ccca * cb,
            (-scsa) * cb - ccca * sb,
            (-scca) * sb - ccsa * cb,
            ccsa * sb - scca * cb,
        )
    }

    pub fn quantize(self, step: f32) -> Quat {
        Quat::new(
            quantize(self.w, step),
            quantize(self.x, step),
            quantize(self.y, step),
            quantize(self.z, step),
        )
    }
}
