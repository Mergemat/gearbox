pub mod assets;
pub mod gbx;
pub mod golden;
pub mod probe;
pub mod sandbox;
pub mod search;
pub mod setup;

use std::path::PathBuf;
use std::sync::OnceLock;

use anyhow::Result;
use gearbox_packs::install::Game;

static GIVEN: OnceLock<Option<PathBuf>> = OnceLock::new();
static FOUND: OnceLock<Game> = OnceLock::new();

pub fn take_game(args: &mut Vec<String>) -> Result<Option<PathBuf>> {
    let mut given = None;
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--game" {
            args.remove(i);
            anyhow::ensure!(i < args.len(), "--game needs the Trackmania folder");
            given = Some(PathBuf::from(args.remove(i)));
        } else if let Some(dir) = args[i].strip_prefix("--game=") {
            given = Some(PathBuf::from(dir));
            args.remove(i);
        } else {
            i += 1;
        }
    }
    Ok(given)
}

pub fn select_game(given: Option<PathBuf>) {
    let _ = GIVEN.set(given);
}

pub fn given_game() -> Option<&'static std::path::Path> {
    GIVEN.get().and_then(Option::as_deref)
}

pub fn game() -> Result<&'static Game> {
    if let Some(game) = FOUND.get() {
        return Ok(game);
    }
    let game = Game::locate(given_game())?;
    Ok(FOUND.get_or_init(|| game))
}

pub fn packs() -> Result<PathBuf> {
    Ok(game()?.packs.clone())
}
