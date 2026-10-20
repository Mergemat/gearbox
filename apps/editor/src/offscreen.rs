use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use gpui::{AppContext, HeadlessAppContext, px, size};

use crate::app::{self, Editor, Options};
use crate::scene::Scene;

const FRAME: Duration = Duration::from_micros(8_333);

pub fn run(opts: Options, game: gearbox_packs::install::Game, home: PathBuf, scene: Scene) -> Result<()> {
    let text = gpui_platform::current_platform(true).text_system();
    let mut cx = HeadlessAppContext::with_platform(
        text,
        Arc::new(crate::theme_icons::Icons),
        gpui_platform::current_headless_renderer,
    );
    cx.allow_parking();
    cx.update(|cx| {
        crate::theme::load_fonts(cx)?;
        app::bind_keys(cx);
        anyhow::Ok(())
    })?;
    let [w, h] = opts.size.unwrap_or([1600.0, 980.0]);
    let window = cx.open_window(size(px(w), px(h)), |window, cx| {
        cx.new(|cx| Editor::new(opts, game, home, scene, window, cx))
    })?;
    let mut last = Instant::now();
    while !app::quitting() {
        let now = Instant::now();
        cx.advance_clock(now - last);
        last = now;
        cx.run_until_parked();
        cx.update_window(window.into(), |_, window, cx| {
            window.simulate_next_frame(cx);
        })?;
        std::thread::sleep((now + FRAME).saturating_duration_since(Instant::now()));
    }
    window.update(&mut cx, |e, _, _| e.quit_script())?;
    Ok(())
}
