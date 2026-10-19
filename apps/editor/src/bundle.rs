use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};

const ICON: &[u8] = include_bytes!("../assets/Gearbox.icns");
const NAME: &str = "Gearbox";
const IDENTIFIER: &str = "dev.gearbox.editor";

pub fn run(args: &[String]) -> Result<()> {
    let mut install = false;
    let mut out = None;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--install" => install = true,
            "--out" => out = Some(PathBuf::from(it.next().context("--out DIR")?)),
            _ => bail!("usage: gearbox-editor bundle [--install] [--out DIR]"),
        }
    }
    let exe = std::env::current_exe()?;
    let dir = match (out, install) {
        (Some(dir), _) => dir,
        (None, true) => home()?.join("Applications"),
        (None, false) => exe.parent().context("executable folder")?.to_path_buf(),
    };
    let app = dir.join(format!("{NAME}.app"));
    build(&exe, &app)?;
    println!("{}", app.display());
    Ok(())
}

fn home() -> Result<PathBuf> {
    std::env::var_os("HOME").map(PathBuf::from).context("no home folder")
}

fn build(exe: &Path, app: &Path) -> Result<()> {
    if app.exists() {
        std::fs::remove_dir_all(app).with_context(|| format!("replacing {}", app.display()))?;
    }
    let contents = app.join("Contents");
    let macos = contents.join("MacOS");
    let resources = contents.join("Resources");
    std::fs::create_dir_all(&macos)?;
    std::fs::create_dir_all(&resources)?;
    std::fs::copy(exe, macos.join(NAME)).with_context(|| format!("copying {}", exe.display()))?;
    std::fs::write(resources.join(format!("{NAME}.icns")), ICON)?;
    std::fs::write(contents.join("Info.plist"), info_plist())?;
    let signed = Command::new("codesign").args(["--force", "--sign", "-"]).arg(app).status();
    if !signed.is_ok_and(|status| status.success()) {
        eprintln!("warning: ad-hoc signing failed; macOS may refuse to open the app");
    }
    Ok(())
}

fn info_plist() -> String {
    let version = env!("CARGO_PKG_VERSION");
    format!(r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key>
    <string>{NAME}</string>
    <key>CFBundleDisplayName</key>
    <string>{NAME}</string>
    <key>CFBundleIdentifier</key>
    <string>{IDENTIFIER}</string>
    <key>CFBundleExecutable</key>
    <string>{NAME}</string>
    <key>CFBundleIconFile</key>
    <string>{NAME}</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleShortVersionString</key>
    <string>{version}</string>
    <key>CFBundleVersion</key>
    <string>{version}</string>
    <key>LSMinimumSystemVersion</key>
    <string>13.0</string>
    <key>LSApplicationCategoryType</key>
    <string>public.app-category.developer-tools</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>CFBundleDocumentTypes</key>
    <array>
        <dict>
            <key>CFBundleTypeName</key>
            <string>Trackmania map or replay</string>
            <key>CFBundleTypeRole</key>
            <string>Viewer</string>
            <key>LSHandlerRank</key>
            <string>Alternate</string>
            <key>CFBundleTypeExtensions</key>
            <array>
                <string>gbx</string>
                <string>Gbx</string>
            </array>
        </dict>
    </array>
</dict>
</plist>
"#)
}
