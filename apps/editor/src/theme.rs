use std::borrow::Cow;

use gpui::{App, Hsla, Rgba, rgb};

pub const BG: u32 = 0x14171C;
pub const PANEL: u32 = 0x1B1F25;
pub const RAISED: u32 = 0x22272E;
pub const HAIRLINE: u32 = 0x2C323A;
pub const TEXT: u32 = 0xE6E9EE;
pub const MUTED: u32 = 0x8C95A1;
pub const PIN: u32 = 0xFF4D85;
pub const GHOST: u32 = 0x2FB8B4;
pub const SETUP: u32 = 0xA48BFF;
pub const WELL: u32 = 0x101317;
pub const SELECT: u32 = 0x2A303A;
pub const FAINT: u32 = 0x5C6570;
pub const SPEED: u32 = 0xE6C88A;
pub const STEER: u32 = 0x7A9CFF;
pub const OK: u32 = 0x4CC38A;
pub const WARN: u32 = 0xE8A33D;
pub const ERROR: u32 = 0xF2555A;
pub const AXIS_X: u32 = 0xE0605F;
pub const AXIS_Y: u32 = 0x7CC46C;
pub const AXIS_Z: u32 = 0x5B8DEF;

pub const FS_SMALL: f32 = 10.5;
pub const FS_BODY: f32 = 11.5;
pub const FS_EMPH: f32 = 12.5;
pub const FS_TITLE: f32 = 15.0;
pub const FS_DISPLAY: f32 = 22.0;
pub const SP1: f32 = 4.0;
pub const SP2: f32 = 8.0;
pub const SP3: f32 = 12.0;
pub const SP4: f32 = 16.0;
pub const SP5: f32 = 24.0;
pub const PAD: f32 = 14.0;
pub const H_CTL: f32 = 26.0;
pub const H_ROW: f32 = 30.0;
pub const H_BAR: f32 = 38.0;

pub const SANS: &str = "Instrument Sans";
pub const MONO: &str = "Martian Mono";
pub const RADIUS: f32 = 8.0;
pub const R_CTL: f32 = 4.0;

pub fn c(hex: u32) -> Rgba {
    rgb(hex)
}

pub fn alpha(hex: u32, a: f32) -> Hsla {
    let mut c: Hsla = rgb(hex).into();
    c.a = a;
    c
}

pub fn rgb8(hex: u32) -> [u8; 3] {
    [(hex >> 16) as u8, (hex >> 8) as u8, hex as u8]
}

pub fn rgba8(hex: u32, a: u8) -> [u8; 4] {
    let [r, g, b] = rgb8(hex);
    [r, g, b, a]
}

pub fn load_fonts(cx: &App) -> anyhow::Result<()> {
    let fonts: Vec<Cow<'static, [u8]>> = vec![
        Cow::Borrowed(include_bytes!("../assets/fonts/InstrumentSans-Regular.ttf")),
        Cow::Borrowed(include_bytes!("../assets/fonts/InstrumentSans-Medium.ttf")),
        Cow::Borrowed(include_bytes!("../assets/fonts/InstrumentSans-SemiBold.ttf")),
        Cow::Borrowed(include_bytes!("../assets/fonts/MartianMono-StdRg.ttf")),
        Cow::Borrowed(include_bytes!("../assets/fonts/MartianMono-StdMd.ttf")),
    ];
    cx.text_system().add_fonts(fonts)
}

pub fn speed_rgb(kmh: f32) -> [u8; 3] {
    let t = (kmh / 400.0).clamp(0.0, 1.0);
    let stops: [(f32, [f32; 3]); 3] = [(0.0, [61.0, 90.0, 168.0]), (0.55, [201.0, 138.0, 58.0]), (1.0, [244.0, 227.0, 181.0])];
    for w in stops.windows(2) {
        let ((a, ca), (b, cb)) = (w[0], w[1]);
        if t <= b {
            let u = (t - a) / (b - a);
            return [0, 1, 2].map(|k| (ca[k] + (cb[k] - ca[k]) * u) as u8);
        }
    }
    [244, 227, 181]
}

pub fn surface_rgb(surface: u8, gameplay: u8) -> [u8; 3] {
    use crate::materials::{Family, family};
    if gameplay != 0 {
        return [118, 100, 74];
    }
    match family(surface) {
        Family::Road => [86, 91, 100],
        Family::Ice => [86, 102, 114],
        Family::Dirt => [104, 90, 76],
        Family::Grass => [74, 90, 76],
        Family::Plastic => [100, 84, 98],
        Family::Wood => [98, 88, 74],
        Family::Metal => [78, 84, 94],
        Family::Water => [58, 78, 102],
        Family::Other => [70, 74, 80],
    }
}
