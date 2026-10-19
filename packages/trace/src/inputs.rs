use crate::state::Controls;

#[derive(Clone, Debug, PartialEq)]
pub struct Timeline {
    pub t0: i32,
    pub seed: Option<u32>,
    pub controls: Vec<Controls>,
}

impl Timeline {
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let header =
            lines.next().and_then(|l| l.strip_prefix("T0 ")).ok_or("expected a `T0 <ms> [seed]` header")?;
        let mut h = header.split_whitespace();
        let t0: i32 = h.next().ok_or("missing T0")?.parse().map_err(|e| format!("T0: {e}"))?;
        if t0 % 10 != 0 {
            return Err("T0 must be on a 10-ms tick".into());
        }
        let seed = h.next().map(|s| s.parse::<u32>().map_err(|e| format!("seed: {e}"))).transpose()?;
        if h.next().is_some() {
            return Err("unexpected header field".into());
        }
        let controls = lines
            .enumerate()
            .map(|(i, line)| {
                let v: Vec<f32> = line
                    .split_whitespace()
                    .map(str::parse)
                    .collect::<Result<_, _>>()
                    .map_err(|e| format!("line {}: {e}", i + 2))?;
                match v[..] {
                    [gas, brake, steer]
                        if (0.0..=1.0).contains(&gas)
                            && (0.0..=1.0).contains(&brake)
                            && (-1.0..=1.0).contains(&steer) =>
                    {
                        Ok(Controls { gas, brake, steer })
                    }
                    _ => {
                        Err(format!("line {}: expected gas and brake in [0, 1] and steer in [-1, 1]", i + 2))
                    }
                }
            })
            .collect::<Result<_, String>>()?;
        Ok(Self { t0, seed, controls })
    }

    pub fn at(&self, ms: i32) -> Controls {
        let offset = i64::from(ms) - i64::from(self.t0);
        if offset < 0 {
            return Controls::default();
        }
        self.controls.get((offset / 10) as usize).copied().unwrap_or_default()
    }
}

pub fn start_ms(t0: i32) -> i32 {
    t0.min(-20)
}

pub fn steps(t: &Timeline, ticks: usize) -> Vec<Controls> {
    let start = start_ms(t.t0);
    let n = ((20 - start) / 10) as usize + ticks;
    (0..n).map(|k| t.at(start + 10 * k as i32)).collect()
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

pub fn random_controls(seed: u64, n: usize) -> Vec<Controls> {
    let mut r = Rng(seed);
    let mut out = Vec::with_capacity(n);
    let mut steer = 0f32;
    while out.len() < n {
        let len = 3 + r.below(58) as usize;
        let gas = if r.unit() < 0.85 { 1.0 } else { 0.0 };
        let brake = if r.unit() < 0.08 { 1.0 } else { 0.0 };
        let x = r.unit();
        let target = if x < 0.4 {
            [-1.0, -0.5, 0.0, 0.5, 1.0][r.below(5) as usize]
        } else if x < 0.75 {
            (r.below(255) as i32 - 127) as f32 / 127.0
        } else {
            (steer + (r.below(21) as i32 - 10) as f32 / 100.0).clamp(-1.0, 1.0)
        };
        let ramp = r.unit() < 0.3;
        for k in 0..len {
            let s = if ramp { steer + (target - steer) * (k + 1) as f32 / len as f32 } else { target };
            out.push(Controls { gas, brake, steer: s.clamp(-1.0, 1.0) });
        }
        steer = target;
    }
    out.truncate(n);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_cover_the_countdown_then_ticks_after_20_ms() {
        let t = Timeline::parse("T0 -40 7\n1 0 0\n0 1 0.5\n").unwrap();
        assert_eq!(t.seed, Some(7));
        let s = steps(&t, 2);
        assert_eq!(s.len(), 8);
        assert_eq!(s[0].gas, 1.0);
        assert_eq!(s[1].steer, 0.5);
        assert_eq!(s[2], Controls::default());
        let late = Timeline::parse("T0 0\n1 0 0\n").unwrap();
        let s = steps(&late, 0);
        assert_eq!((start_ms(0), s.len()), (-20, 4));
        assert_eq!((s[1].gas, s[2].gas), (0.0, 1.0));
        assert!(Timeline::parse("T0 -15\n").is_err());
        assert!(Timeline::parse("T0 0\n2 0 0\n").is_err());
    }

    #[test]
    fn random_driving_is_stable_and_search_like() {
        let a = random_controls(1, 3000);
        assert_eq!(a, random_controls(1, 3000));
        assert_ne!(a, random_controls(2, 3000));
        let changes = a.windows(2).filter(|w| w[0] != w[1]).count();
        let gas = a.iter().filter(|c| c.gas == 1.0).count();
        assert!(changes > 50, "{changes} changes");
        assert!(gas > 2000 && gas < 3000, "{gas} ticks of gas");
        assert!(a.iter().any(|c| c.brake == 1.0));
        assert!(a.iter().all(|c| (-1.0..=1.0).contains(&c.steer)));
    }
}
