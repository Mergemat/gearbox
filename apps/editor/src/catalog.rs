use std::path::{Path, PathBuf};
pub use gearbox_world::paths::map_stem;

fn collect_maps(dir: &Path, depth: u32, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    let mut entries: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
    entries.sort();
    for path in entries {
        if path.is_dir() && depth > 0 {
            collect_maps(&path, depth - 1, out);
        } else if map_stem(&path).is_some() {
            out.push(path);
        }
    }
}

pub fn test_maps() -> Option<&'static Path> {
    static DIR: std::sync::OnceLock<Option<PathBuf>> = std::sync::OnceLock::new();
    DIR.get_or_init(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/maps").canonicalize().ok()).as_deref()
}

pub fn scan_maps(game: &gearbox_packs::install::Game) -> Vec<PathBuf> {
    let mut maps = Vec::new();
    if let Some(dir) = test_maps() {
        collect_maps(dir, 0, &mut maps);
    }
    for dir in gearbox_packs::install::maps_dirs(Some(game)) {
        collect_maps(&dir, 4, &mut maps);
    }
    if let Some(dir) = dirs::download_dir() {
        collect_maps(&dir, 0, &mut maps);
    }
    maps
}

pub fn inputs_for_map(home: &Path, map: &Path) -> Option<PathBuf> {
    let stem = map_stem(map)?;
    let dir = home.join("inputs");
    let exact = dir.join(format!("{stem}.txt.in"));
    if exact.exists() {
        return Some(exact);
    }
    if stem.starts_with("pf-") {
        let p = dir.join("pf_gas.txt.in");
        return p.exists().then_some(p);
    }
    let id: String = stem.chars().take_while(char::is_ascii_digit).collect();
    if id.len() < 3 {
        return None;
    }
    let mut found: Vec<PathBuf> = std::fs::read_dir(&dir)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            let n = p.file_name().unwrap_or_default().to_string_lossy().into_owned();
            n.strip_suffix(".txt.in")
                .and_then(|s| s.strip_suffix(id.as_str()))
                .is_some_and(|prefix| !prefix.is_empty() && prefix.chars().all(|c| c.is_ascii_alphabetic()))
        })
        .collect();
    found.sort();
    found.into_iter().next()
}
