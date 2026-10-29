use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::path::{Path, PathBuf};

fn sources(dir: &Path, hasher: &mut DefaultHasher) {
    let mut paths: Vec<PathBuf> =
        std::fs::read_dir(dir).expect("source directory").flatten().map(|entry| entry.path()).collect();
    paths.sort();
    for path in paths {
        if path.is_dir() {
            sources(&path, hasher);
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            path.hash(hasher);
            std::fs::read(&path).expect("source file").hash(hasher);
        }
    }
}

fn main() {
    let mut hasher = DefaultHasher::new();
    for dir in ["src", "../gbx/src", "../engine/src"] {
        println!("cargo:rerun-if-changed={dir}");
        sources(Path::new(dir), &mut hasher);
    }
    println!("cargo:rustc-env=GEARBOX_CATALOG_SOURCE={:016x}", hasher.finish());
}
