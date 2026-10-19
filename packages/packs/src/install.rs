use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};

use crate::pak::Pak;
use crate::packs::PackSet;

pub const NOT_FOUND: &str = "Trackmania not found. Run: gearbox setup --game <path to Trackmania>";

const INSTALLS: [&str; 8] = [
    "Program Files (x86)/Ubisoft/Ubisoft Game Launcher/games/Trackmania",
    "Program Files/Ubisoft/Ubisoft Game Launcher/games/Trackmania",
    "Program Files/Epic Games/TrackmaniaNext",
    "Program Files/Epic Games/Trackmania",
    "Program Files (x86)/Steam/steamapps/common/Trackmania",
    "Program Files/Steam/steamapps/common/Trackmania",
    "Program Files/Trackmania",
    "Program Files (x86)/Trackmania",
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    Given,
    Saved,
    Detected,
}

#[derive(Clone, Debug)]
pub struct Game {
    pub packs: PathBuf,
    pub origin: Origin,
}

#[derive(Serialize, Deserialize)]
struct Config {
    game: PathBuf,
}

impl Game {
    pub fn locate(given: Option<&Path>) -> Result<Game> {
        if let Some(path) = given {
            return Game::at(path, Origin::Given);
        }
        if let Some(file) = config_file()
            && let Some(saved) = read_config(&file)?
        {
            return Game::at(&saved, Origin::Saved).with_context(|| {
                format!("the game saved in {} is gone. Run: gearbox setup --game <path to Trackmania>", file.display())
            });
        }
        detect().context(NOT_FOUND)
    }

    pub fn at(path: &Path, origin: Origin) -> Result<Game> {
        let Some(packs) = packs_dir(path) else {
            bail!("{}: no Stadium.pak there or in its Packs folder; pass the Trackmania folder or its Packs folder", path.display());
        };
        let stadium = child(&packs, "Stadium.pak").context("Stadium.pak")?;
        let pak = Pak::open(&stadium, None)?;
        ensure!(pak.check_key(&crate::packs::key("stadium").context("Stadium key")?),
            "{}: does not open with the Stadium key; is it a Trackmania 2020 install?", stadium.display());
        Ok(Game { packs: std::fs::canonicalize(&packs)?, origin })
    }

    pub fn open(&self) -> Result<PackSet> {
        PackSet::open(&self.packs)
    }

    pub fn save(&self) -> Result<PathBuf> {
        let file = config_file().context("no configuration folder on this system")?;
        write_config(&file, &self.packs)?;
        Ok(file)
    }
}

pub fn config_file() -> Option<PathBuf> {
    Some(dirs::config_dir()?.join("gearbox").join("config.json"))
}

pub fn cache_dir() -> PathBuf {
    dirs::cache_dir().unwrap_or_else(std::env::temp_dir).join("gearbox")
}

pub fn data_dir() -> PathBuf {
    dirs::data_dir().unwrap_or_else(std::env::temp_dir).join("gearbox")
}

pub fn maps_dirs(game: Option<&Game>) -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = dirs::document_dir().map(|d| d.join("Trackmania").join("Maps")).into_iter().collect();
    if let Some(drive) = game.and_then(|g| g.packs.ancestors().find(|a| a.file_name().is_some_and(|n| n.eq_ignore_ascii_case("drive_c")))) {
        let users = list(&drive.join("users"));
        for user in users {
            for docs in ["Documents", "My Documents"] {
                dirs.push(user.join(docs).join("Trackmania").join("Maps"));
            }
        }
    }
    dirs.retain(|d| d.is_dir());
    dirs.dedup();
    dirs
}

pub fn read_config(file: &Path) -> Result<Option<PathBuf>> {
    let text = match std::fs::read_to_string(file) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e).with_context(|| format!("reading {}", file.display())),
    };
    let config: Config = serde_json::from_str(&text).with_context(|| format!("{}: not a gearbox configuration", file.display()))?;
    Ok(Some(config.game))
}

pub fn write_config(file: &Path, packs: &Path) -> Result<()> {
    let dir = file.parent().context("configuration file has no folder")?;
    std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
    let text = serde_json::to_string_pretty(&Config { game: packs.to_path_buf() })? + "\n";
    let partial = file.with_extension(format!("{}.tmp", std::process::id()));
    std::fs::write(&partial, text).with_context(|| format!("writing {}", partial.display()))?;
    std::fs::rename(&partial, file).with_context(|| format!("writing {}", file.display()))?;
    Ok(())
}

pub fn detect() -> Option<Game> {
    let home = dirs::home_dir()?;
    let drives: Vec<PathBuf> = ["C:\\", "D:\\", "E:\\"].into_iter().map(PathBuf::from).collect();
    candidates(Os::current(), &home, &drives).into_iter().find_map(|c| Game::at(&c, Origin::Detected).ok())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Os {
    Windows,
    Linux,
    Mac,
}

impl Os {
    pub fn current() -> Os {
        if cfg!(windows) {
            Os::Windows
        } else if cfg!(target_os = "macos") {
            Os::Mac
        } else {
            Os::Linux
        }
    }
}

pub fn candidates(os: Os, home: &Path, drives: &[PathBuf]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    match os {
        Os::Windows => {
            for drive in drives {
                out.extend(windows_installs(drive));
            }
        }
        Os::Linux => {
            for steam in [".steam/steam", ".steam/root", ".local/share/Steam", ".var/app/com.valvesoftware.Steam/.local/share/Steam"] {
                out.extend(steam_installs(&home.join(steam)));
            }
            let mut prefixes = vec![home.join(".wine")];
            for root in ["Games", ".local/share/lutris/prefixes", ".local/share/bottles/bottles", ".var/app/com.usebottles.bottles/data/bottles/bottles"] {
                prefixes.extend(list(&home.join(root)));
            }
            for prefix in prefixes {
                out.extend(windows_installs(&prefix.join("drive_c")));
            }
        }
        Os::Mac => {
            let mut drives = vec![home.join(".wine/drive_c")];
            find_drives(&home.join("Library/Application Support"), 4, &mut drives);
            find_drives(&home.join("Library/Containers/com.isaacmarovitz.Whisky/Data/Library/Application Support"), 4, &mut drives);
            find_drives(&home.join("Library/Containers/com.isaacmarovitz.Whisky/Bottles"), 2, &mut drives);
            for drive in drives {
                out.extend(windows_installs(&drive));
            }
        }
    }
    out.dedup();
    out
}

fn windows_installs(drive: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = INSTALLS.iter().map(|i| drive.join(i)).collect();
    for steam in ["Program Files (x86)/Steam", "Program Files/Steam"] {
        out.extend(steam_installs(&drive.join(steam)));
    }
    for manifest in list(&drive.join("ProgramData/Epic/EpicGamesLauncher/Data/Manifests")) {
        if let Some(at) = epic_install(&manifest) {
            out.push(at);
        }
    }
    out
}

fn steam_installs(steam: &Path) -> Vec<PathBuf> {
    let mut libraries = vec![steam.to_path_buf()];
    if let Ok(text) = std::fs::read_to_string(steam.join("steamapps/libraryfolders.vdf")) {
        libraries.extend(text.lines().filter_map(vdf_path));
    }
    libraries.iter().map(|l| l.join("steamapps").join("common").join("Trackmania")).collect()
}

fn vdf_path(line: &str) -> Option<PathBuf> {
    let rest = line.trim().strip_prefix("\"path\"")?.trim();
    let value = rest.strip_prefix('"')?.strip_suffix('"')?;
    Some(PathBuf::from(value.replace("\\\\", "\\")))
}

fn epic_install(manifest: &Path) -> Option<PathBuf> {
    if !manifest.extension().is_some_and(|e| e.eq_ignore_ascii_case("item")) {
        return None;
    }
    let item: serde_json::Value = serde_json::from_slice(&std::fs::read(manifest).ok()?).ok()?;
    let at = item.get("InstallLocation")?.as_str()?;
    at.to_lowercase().contains("trackmania").then(|| PathBuf::from(at))
}

fn find_drives(dir: &Path, depth: u32, out: &mut Vec<PathBuf>) {
    for entry in list(dir) {
        if !entry.is_dir() {
            continue;
        }
        if entry.file_name().is_some_and(|n| n.eq_ignore_ascii_case("drive_c")) {
            out.push(entry);
        } else if depth > 0 {
            find_drives(&entry, depth - 1, out);
        }
    }
}

fn list(dir: &Path) -> Vec<PathBuf> {
    let mut entries: Vec<PathBuf> = std::fs::read_dir(dir).into_iter().flatten().flatten().map(|e| e.path()).collect();
    entries.sort();
    entries
}

fn child(dir: &Path, name: &str) -> Option<PathBuf> {
    list(dir).into_iter().find(|p| p.file_name().is_some_and(|n| n.eq_ignore_ascii_case(name)))
}

pub fn packs_dir(path: &Path) -> Option<PathBuf> {
    if child(path, "Stadium.pak").is_some_and(|p| p.is_file()) {
        return Some(path.to_path_buf());
    }
    let packs = child(path, "Packs")?;
    child(&packs, "Stadium.pak").is_some_and(|p| p.is_file()).then_some(packs)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Temp(PathBuf);

    impl Temp {
        fn new(name: &str) -> Temp {
            let dir = std::env::temp_dir().join(format!("gearbox-install-{name}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).unwrap();
            Temp(dir)
        }

        fn file(&self, rel: &str, text: &str) -> PathBuf {
            let p = self.0.join(rel);
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::fs::write(&p, text).unwrap();
            p
        }
    }

    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn mac_finds_games_in_crossover_and_whisky_bottles() {
        let home = Temp::new("mac");
        let crossover = "Library/Application Support/CrossOver/Bottles/Trackmania/drive_c";
        let whisky = "Library/Containers/com.isaacmarovitz.Whisky/Bottles/5D9A/drive_c";
        let other = "Library/Application Support/Sikarugir/Wrapper/drive_c";
        for drive in [crossover, whisky, other] {
            std::fs::create_dir_all(home.0.join(drive)).unwrap();
        }
        let found = candidates(Os::Mac, &home.0, &[]);
        for want in [
            home.0.join(crossover).join("Program Files (x86)/Ubisoft/Ubisoft Game Launcher/games/Trackmania"),
            home.0.join(whisky).join("Program Files/Epic Games/TrackmaniaNext"),
            home.0.join(other).join("Program Files (x86)/Steam/steamapps/common/Trackmania"),
        ] {
            assert!(found.contains(&want), "{} not in {found:#?}", want.display());
        }
        assert!(found.iter().all(|p| p.starts_with(&home.0)));
    }

    #[test]
    fn linux_follows_steam_library_folders_and_wine_prefixes() {
        let home = Temp::new("linux");
        home.file(".local/share/Steam/steamapps/libraryfolders.vdf",
            "\"libraryfolders\"\n{\n\t\"1\"\n\t{\n\t\t\"path\"\t\t\"/mnt/games/SteamLibrary\"\n\t}\n}\n");
        std::fs::create_dir_all(home.0.join("Games/trackmania/drive_c")).unwrap();
        let found = candidates(Os::Linux, &home.0, &[]);
        for want in [
            PathBuf::from("/mnt/games/SteamLibrary/steamapps/common/Trackmania"),
            home.0.join(".local/share/Steam/steamapps/common/Trackmania"),
            home.0.join(".wine/drive_c/Program Files (x86)/Ubisoft/Ubisoft Game Launcher/games/Trackmania"),
            home.0.join("Games/trackmania/drive_c/Program Files/Epic Games/TrackmaniaNext"),
        ] {
            assert!(found.contains(&want), "{} not in {found:#?}", want.display());
        }
    }

    #[test]
    fn windows_reads_epic_manifests_and_steam_libraries() {
        let drive = Temp::new("windows");
        drive.file("ProgramData/Epic/EpicGamesLauncher/Data/Manifests/A1.item",
            r#"{"DisplayName": "Trackmania", "InstallLocation": "D:\\Epic\\TrackmaniaNext"}"#);
        drive.file("ProgramData/Epic/EpicGamesLauncher/Data/Manifests/B2.item",
            r#"{"DisplayName": "Other", "InstallLocation": "D:\\Epic\\Other"}"#);
        drive.file("Program Files (x86)/Steam/steamapps/libraryfolders.vdf", "\t\t\"path\"\t\t\"E:\\\\SteamLibrary\"\n");
        let found = candidates(Os::Windows, Path::new("/nowhere"), std::slice::from_ref(&drive.0));
        assert!(found.contains(&PathBuf::from("D:\\Epic\\TrackmaniaNext")), "{found:#?}");
        assert!(!found.contains(&PathBuf::from("D:\\Epic\\Other")));
        assert!(found.contains(&Path::new("E:\\SteamLibrary").join("steamapps").join("common").join("Trackmania")), "{found:#?}");
        assert!(found.contains(&drive.0.join("Program Files (x86)/Ubisoft/Ubisoft Game Launcher/games/Trackmania")));
    }

    #[test]
    fn the_game_folder_and_its_packs_folder_both_resolve_to_the_packs() {
        let game = Temp::new("packs");
        assert_eq!(packs_dir(&game.0), None);
        game.file("packs/STADIUM.PAK", "");
        let packs = game.0.join("packs");
        assert_eq!(packs_dir(&game.0), Some(packs.clone()));
        assert_eq!(packs_dir(&packs), Some(packs));
    }

    #[test]
    fn a_pack_that_does_not_open_with_the_stadium_key_is_rejected() {
        let game = Temp::new("invalid");
        game.file("Packs/Stadium.pak", &"x".repeat(0x5000));
        assert!(Game::at(&game.0, Origin::Given).is_err());
        let missing = Game::at(&game.0.join("Packs/nothing"), Origin::Given).unwrap_err().to_string();
        assert!(missing.contains("no Stadium.pak"), "{missing}");
    }

    #[test]
    fn the_saved_game_round_trips_and_a_missing_config_is_none() {
        let dir = Temp::new("config");
        let file = dir.0.join("gearbox/config.json");
        assert_eq!(read_config(&file).unwrap(), None);
        write_config(&file, Path::new("/games/Trackmania/Packs")).unwrap();
        assert_eq!(read_config(&file).unwrap(), Some(PathBuf::from("/games/Trackmania/Packs")));
        std::fs::write(&file, "game = 1").unwrap();
        assert!(read_config(&file).is_err());
    }
}
