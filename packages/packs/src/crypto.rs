use crate::bf_consts;

#[derive(Clone)]
pub struct Blowfish {
    p: [u32; 18],
    s: [[u32; 256]; 4],
}

const ROUNDS: usize = 8;

impl Blowfish {
    pub fn new(key: &[u8]) -> Self {
        assert!(!key.is_empty());
        let mut bf = Blowfish { p: bf_consts::P, s: bf_consts::S };
        let mut j = 0;
        for i in 0..ROUNDS + 2 {
            let mut data = 0u32;
            for k in 0..4 {
                data |= (key[j] as u32) << (k * 8);
                j = (j + 1) % key.len();
            }
            bf.p[i] ^= data;
        }
        let (mut l, mut r) = (0u32, 0u32);
        for i in (0..ROUNDS + 2).step_by(2) {
            bf.encrypt_lr(&mut l, &mut r);
            bf.p[i] = l;
            bf.p[i + 1] = r;
        }
        for i in 0..4 {
            for j in (0..256).step_by(2) {
                bf.encrypt_lr(&mut l, &mut r);
                bf.s[i][j] = l;
                bf.s[i][j + 1] = r;
            }
        }
        bf.p[..10].reverse();
        bf
    }

    #[inline(always)]
    fn f(&self, x: u32) -> u32 {
        let (a, b, c, d) =
            ((x >> 24) as usize, (x >> 16 & 0xff) as usize, (x >> 8 & 0xff) as usize, (x & 0xff) as usize);
        (self.s[0][a].wrapping_add(self.s[1][b]) ^ self.s[2][c]).wrapping_add(self.s[3][d])
    }

    #[inline(always)]
    fn encrypt_lr(&self, l: &mut u32, r: &mut u32) {
        let (mut a, mut b) = (*l, *r);
        for i in 0..ROUNDS {
            a ^= self.p[i];
            b ^= self.f(a);
            std::mem::swap(&mut a, &mut b);
        }
        std::mem::swap(&mut a, &mut b);
        b ^= self.p[ROUNDS];
        a ^= self.p[ROUNDS + 1];
        *l = a;
        *r = b;
    }

    #[inline(always)]
    pub fn block(&self, c: u64) -> u64 {
        let (mut l, mut r) = (c as u32, (c >> 32) as u32);
        self.encrypt_lr(&mut l, &mut r);
        l as u64 | (r as u64) << 32
    }
}

pub fn iv_xor_feed(mut x: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        x = x.rotate_left(13) ^ (b | 0xaa) as u64;
    }
    x
}

#[derive(Clone)]
pub struct Cipher<'k> {
    bf: &'k Blowfish,
    pub iv: u64,
    pub iv_xor: u64,
    buffer_index: usize,
    total: usize,
}

impl<'k> Cipher<'k> {
    pub fn new(bf: &'k Blowfish, iv: u64) -> Self {
        Cipher { bf, iv, iv_xor: 0, buffer_index: 0, total: 0 }
    }

    #[inline]
    pub fn next_block(&mut self, c: u64) -> u64 {
        if self.total == 0 {
            self.iv ^= self.iv_xor;
            self.iv_xor = 0;
        }
        if self.buffer_index == 0x100 {
            self.iv ^= self.iv_xor;
            self.iv_xor = 0;
            self.buffer_index = 0;
        }
        let p = self.bf.block(c) ^ self.iv;
        self.iv = (self.iv >> 0x2f) ^ self.iv.wrapping_mul(9) ^ c;
        self.buffer_index += 8;
        self.total += 8;
        p
    }

    pub fn dummy_write(&mut self, bytes: &[u8]) {
        self.iv_xor = iv_xor_feed(self.iv_xor, bytes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iv_xor_matches_wiki_formulation() {
        let input = [0x00u8, 0x50, 0x01, 0x09, 0xff, 0x13, 0x77, 0xa0];
        let mut x: u64 = 0x0123_4567_89ab_cdef;
        let reference = {
            let mut v = x;
            for &b in &input {
                let lo = v as u32;
                let hi = (v >> 32) as u32;
                let lo2 = (b as u32 | 0xaa) ^ ((lo << 13) | (hi >> 19));
                let hi2 = ((v << 13) >> 32) as u32;
                v = (hi2 as u64) << 32 | lo2 as u64;
            }
            v
        };
        x = iv_xor_feed(x, &input);
        assert_eq!(x, reference);
    }
}
