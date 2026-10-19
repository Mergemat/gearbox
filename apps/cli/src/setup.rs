use std::time::Instant;

use anyhow::{Result, bail};
use gearbox_packs::install::{self, Game, Origin};

pub const USAGE: &str = "usage: gearbox setup [--game DIR]

Finds Trackmania (DIR is the game folder or its Packs folder; without it,
the saved one or a standard install), saves it for every gearbox command,
and builds the pack catalog cache. Safe to run again.";

pub fn main(args: Vec<String>) -> Result<()> {
    if args.iter().any(|a| matches!(a.as_str(), "help" | "--help" | "-h")) {
        println!("{USAGE}");
        return Ok(());
    }
    if let Some(a) = args.first() {
        bail!("unknown argument {a}\n{USAGE}");
    }
    let game = match crate::given_game() {
        Some(dir) => Game::at(dir, Origin::Given)?,
        None => Game::locate(None).or_else(|e| install::detect().ok_or(e))?,
    };
    let how = match game.origin {
        Origin::Given => "as given",
        Origin::Saved => "as saved before",
        Origin::Detected => "found in a standard install",
    };
    println!("Trackmania packs: {} ({how})", game.packs.display());
    let config = game.save()?;
    println!("saved to {}", config.display());
    let began = Instant::now();
    let set = game.open()?;
    let catalog = gearbox_packs::catalog::cached(&set)?;
    println!("pack catalog: {} blocks, {} items, {} surfaces in {:.1} s, cached in {}",
        catalog.index.blocks.len(), catalog.index.items.len(), catalog.surfaces.len(),
        began.elapsed().as_secs_f64(), install::cache_dir().display());
    println!("editor projects and layout: {}", install::data_dir().join("editor").display());
    for maps in install::maps_dirs(Some(&game)) {
        println!("your maps: {}", maps.display());
    }
    println!("ready: try `gearbox golden check`");
    Ok(())
}
