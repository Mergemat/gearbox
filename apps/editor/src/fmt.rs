pub fn fmt_ms(ms: i32) -> String {
    let neg = ms < 0;
    let a = ms.unsigned_abs();
    let (m, s, f) = (a / 60000, (a / 1000) % 60, a % 1000);
    let sign = if neg { "-" } else { "" };
    if m > 0 { format!("{sign}{m}:{s:02}.{f:03}") } else { format!("{sign}{s}.{f:03}") }
}

pub fn fmt_s(ms: i32) -> String {
    let sign = if ms < 0 { "-" } else { "" };
    let a = ms.unsigned_abs();
    format!("{sign}{}.{:03}", a / 1000, a % 1000)
}

pub fn fmt_delta(ms: i32) -> String {
    let sign = if ms > 0 { "+" } else if ms < 0 { "\u{2212}" } else { "\u{00b1}" };
    let a = ms.unsigned_abs();
    format!("{sign}{}.{:03}", a / 1000, a % 1000)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn times_format_with_fixed_millis() {
        assert_eq!(fmt_ms(7443), "7.443");
        assert_eq!(fmt_ms(65012), "1:05.012");
        assert_eq!(fmt_ms(-1460), "-1.460");
        assert_eq!(fmt_s(65012), "65.012");
        assert_eq!(fmt_delta(123), "+0.123");
        assert_eq!(fmt_delta(-2045), "\u{2212}2.045");
        assert_eq!(fmt_delta(0), "\u{00b1}0.000");
    }
}
