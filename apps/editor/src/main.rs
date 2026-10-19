mod app;
mod background;
mod bundle;
mod camera;
mod catalog;
mod doc;
mod engine;
mod first_run;
mod fmt;
mod gfx;
mod gizmo;
#[cfg(test)]
mod headless;
mod lab;
mod mapcards;
mod materials;
mod optimize;
mod picker;
mod project;
mod replay;
mod scene;
mod script;
mod sim;
mod stats;
mod theme;
mod theme_icons;
mod ui;
mod visual;

use std::path::PathBuf;

use anyhow::{Context as _, Result, bail};
use gpui::{App, AppContext, Bounds, TitlebarOptions, WindowBounds, WindowOptions, px, size};

use app::{Editor, Options};
use gfx::renderer::{Composite, Gpu, Renderer};

fn parse() -> Result<Options> {
    let mut o = Options::default();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut val = || it.next().with_context(|| format!("{a} needs a value"));
        match a.as_str() {
            "--inputs" => o.inputs = Some(Some(PathBuf::from(val()?))),
            "--new" => o.inputs = Some(None),
            "--project" => o.project = Some(PathBuf::from(val()?)),
            "--ghost" => o.ghost = Some(PathBuf::from(val()?)),
            "--composite" => {
                o.composite = Some(match val()?.as_str() {
                    "surface" => Composite::Surface,
                    "image" => Composite::Image,
                    v => bail!("--composite surface|image, not {v}"),
                })
            }
            "--shot" => o.shot = Some(PathBuf::from(val()?)),
            "--exit" => o.exit = true,
            "--size" => {
                let v = val()?;
                let (w, h) = v.split_once('x').context("--size WxH")?;
                o.size = Some([w.parse()?, h.parse()?]);
            }
            "--sync" => o.sync = true,
            "--eye" | "--look" => {
                let v = val()?;
                let p: Vec<f32> = v.split(',').map(str::parse).collect::<Result<_, _>>().with_context(|| format!("{a} x,y,z"))?;
                let p: [f32; 3] = p.try_into().ok().with_context(|| format!("{a} x,y,z"))?;
                if a == "--eye" {
                    o.eye = Some(p);
                } else {
                    o.look = Some(p);
                }
            }
            "--physics" => o.physics = true,
            "--stadium" => o.stadium = true,
            "--pick-map" => o.pick_map = true,
            "--script" => o.script = Some(PathBuf::from(val()?)),
            "--smoke" => o.smoke = true,
            "--game" => o.game = Some(PathBuf::from(val()?)),
            "--record" => o.record = Some(PathBuf::from(val()?)),
            s if s.starts_with("--") => bail!("unknown option {s}"),
            _ => o.map = Some(PathBuf::from(a)),
        }
    }
    if o.script.is_some() && o.map.is_none() && o.project.is_none() {
        bail!("--script needs a map or --project");
    }
    Ok(o)
}

fn window_number(window: &gpui::Window) -> Option<isize> {
    use raw_window_handle::{HasWindowHandle, RawWindowHandle};
    let RawWindowHandle::AppKit(h) = HasWindowHandle::window_handle(window).ok()?.as_raw() else { return None };
    let view = h.ns_view.as_ptr() as *mut objc2::runtime::AnyObject;
    unsafe {
        let win: *mut objc2::runtime::AnyObject = objc2::msg_send![&*view, window];
        if win.is_null() {
            return None;
        }
        Some(objc2::msg_send![&*win, windowNumber])
    }
}

fn keep_on_screen(window: &gpui::Window) {
    use raw_window_handle::{HasWindowHandle, RawWindowHandle};
    const ALL_SPACES: usize = 1 << 0;
    const FULL_SCREEN_AUXILIARY: usize = 1 << 8;
    let Ok(handle) = HasWindowHandle::window_handle(window) else { return };
    let RawWindowHandle::AppKit(h) = handle.as_raw() else { return };
    let view = h.ns_view.as_ptr() as *mut objc2::runtime::AnyObject;
    unsafe {
        let win: *mut objc2::runtime::AnyObject = objc2::msg_send![&*view, window];
        if win.is_null() {
            return;
        }
        let _: () = objc2::msg_send![&*win, setCollectionBehavior: ALL_SPACES | FULL_SCREEN_AUXILIARY];
        let _: () = objc2::msg_send![&*win, setHidesOnDeactivate: false];
        let _: () = objc2::msg_send![&*win, orderFrontRegardless];
    }
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|a| a == "bundle") {
        return bundle::run(&args[1..]);
    }
    let opts = parse()?;
    let game = match gearbox_packs::install::Game::locate(opts.game.as_deref()) {
        Ok(game) => Some(game),
        Err(e) if opts.game.is_some() || opts.script.is_some() => return Err(e),
        Err(_) => None,
    };
    let home = match &opts.script {
        Some(dir) => {
            let home = std::path::absolute(dir)?.join("editor");
            let _ = std::fs::remove_dir_all(&home);
            home
        }
        None => gearbox_packs::install::data_dir().join("editor"),
    };
    let composite = opts.composite.unwrap_or(Composite::Surface);
    let gpu = Gpu::new()?;
    eprintln!("gpu: {} (timestamps {}), composite {:?}", gpu.adapter, gpu.timestamps, composite);
    let mut renderer = Renderer::new(gpu, composite, theme::rgb8(theme::BG));
    renderer.pipelined = !opts.sync;
    let scene = scene::Scene::new(renderer);
    let (opened_tx, opened_rx) = futures::channel::mpsc::unbounded();
    let application = gpui_platform::application().with_assets(theme_icons::Icons);
    application.on_open_urls(move |urls| {
        for url in urls {
            if let Some(path) = url.strip_prefix("file://").map(percent_decoded) {
                let _ = opened_tx.unbounded_send(PathBuf::from(path));
            }
        }
    });
    application.run(move |cx: &mut App| {
        if let Err(e) = theme::load_fonts(cx) {
            eprintln!("fonts: {e:#}");
        }
        app::bind_keys(cx);
        let start = move |game, cx: &mut App| open_editor(opts, game, home, scene, opened_rx, cx);
        match game {
            Some(game) => start(game, cx),
            None => first_run::open(cx, start),
        }
        cx.activate(true);
    });
    Ok(())
}

fn open_editor(opts: Options, game: gearbox_packs::install::Game, home: PathBuf, scene: scene::Scene,
    mut opened: futures::channel::mpsc::UnboundedReceiver<PathBuf>, cx: &mut App)
{
    let [w, h] = opts.size.unwrap_or([1600.0, 980.0]);
    let bounds = Bounds::centered(None, size(px(w), px(h)), cx);
    let scripted = opts.script.is_some() || opts.record.is_some() || opts.shot.is_some();
    let handle = cx
        .open_window(
            WindowOptions {
                window_bounds: Some(WindowBounds::Windowed(bounds)),
                titlebar: Some(TitlebarOptions { title: Some("Gearbox".into()), appears_transparent: true, traffic_light_position: Some(gpui::point(px(14.0), px(13.0))) }),
                focus: true,
                show: true,
                kind: if scripted { gpui::WindowKind::Floating } else { gpui::WindowKind::Normal },
                inactive_frame_interval: if scripted { None } else { Some(std::time::Duration::from_micros(33_333)) },
                ..Default::default()
            },
            move |window, cx| cx.new(|cx| Editor::new(opts, game, home, scene, window, cx)),
        )
        .expect("opening the window");
    let _ = handle.update(cx, |app, window, _| {
        app.window_id = window_number(window);
        if scripted {
            keep_on_screen(window);
        }
    });
    cx.spawn(async move |cx| {
        use futures::StreamExt;
        while let Some(path) = opened.next().await {
            let _ = handle.update(cx, |editor, _, cx| editor.open_file(&path, cx));
        }
    })
    .detach();
}

fn percent_decoded(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        let hex = bytes.get(i + 1..i + 3).and_then(|h| std::str::from_utf8(h).ok()).and_then(|h| u8::from_str_radix(h, 16).ok());
        match (bytes[i], hex) {
            (b'%', Some(byte)) => {
                out.push(byte);
                i += 3;
            }
            (byte, _) => {
                out.push(byte);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}
