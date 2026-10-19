use std::path::{Path, PathBuf};

use anyhow::Result;
use gearbox_trace::driver::Defaults;

pub fn golden_dir() -> PathBuf {
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden");
    if source.join("maps").is_dir() {
        return source.canonicalize().unwrap_or(source);
    }
    let cwd = std::env::current_dir().unwrap_or_default();
    cwd.ancestors()
        .map(|d| d.join("tests/golden"))
        .find(|d| d.join("maps").is_dir())
        .unwrap_or_else(|| PathBuf::from("tests/golden"))
}

const HELP: &str = "
Defaults: --dir is gearbox's tests/golden, --corpus DIR/corpus.tsv.";

pub fn typed_factory() -> Result<gearbox_trace::typed::TypedFactory> {
    Ok(gearbox_trace::typed::TypedFactory::new(crate::packs()?))
}

pub fn drive(
    args: Vec<String>,
    command: &[&str],
    factory: &dyn gearbox_trace::Factory,
    defaults: &Defaults,
) -> Result<()> {
    gearbox_trace::driver::main(args, command, factory, defaults).map_err(|e| anyhow::anyhow!("{e}"))
}

pub fn defaults() -> Defaults {
    let dir = golden_dir();
    Defaults { corpus: dir.join("corpus.tsv"), dir }
}

pub fn main(args: Vec<String>) -> Result<()> {
    match args.first().map(String::as_str) {
        None | Some("help" | "--help" | "-h") => {
            println!("{}\n{HELP}", gearbox_trace::driver::USAGE);
            Ok(())
        }
        _ if args.iter().any(|arg| matches!(arg.as_str(), "--help" | "-h")) => {
            println!("{}\n{HELP}", gearbox_trace::driver::USAGE);
            Ok(())
        }
        Some("list" | "show") => {
            drive(args, &["golden"], &gearbox_trace::typed::TypedFactory::new(PathBuf::new()), &defaults())
        }
        _ => {
            let factory = typed_factory()?;
            let packs = factory.packs.display().to_string();
            drive(args, &["--game", &packs, "golden"], &factory, &defaults())
        }
    }
}
