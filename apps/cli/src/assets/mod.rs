use std::path::{Path, PathBuf};

mod extract;
mod visual;

use anyhow::{Result, bail};
use gearbox_packs::pak::Pak;

pub const USAGE: &str = "usage: gearbox assets <command>   (on the packs of `gearbox setup` or --game)

  gltf <map> <out.glb>                     visual map + car export
  gltf-car <CarSport|CarSnow|CarRally|CarDesert> <out.glb>
  extract <out> [pak filter...]            crawl packs into an asset dir
  verify <extract/GameData>                decoding vs the game's extraction
  check [pak filter]                       decode every file, summarise
  find <game path>                         pack, key, index of a file
  paths [substring]                        game paths of all packed files
  cat <game path> <out>                    decode one file by game path
  unpack <pak> <key> <index> <out> [ref]   decode one file
  embedded <map> <path substring> <out>    a file embedded in a map
  list <pak> [key]                         file table
  info <pak>...                            pack headers";

pub fn main(args: Vec<String>) -> Result<()> {
    if args.is_empty() || args[0] == "help" || args[0] == "--help" {
        println!("{USAGE}");
        return Ok(());
    }
    let cmd = &args[0];
    match cmd.as_str() {
        "gltf" | "gltf-car" => return visual::run(&args),
        "info" => {
            for p in &args[1..] {
                let pak = Pak::open(Path::new(p), None)?;
                println!(
                    "{p}: v{} flags {:#x} header max {:#x} title {:?} subdir {:?} build {:?} checksum {}",
                    pak.version,
                    pak.header_flags,
                    pak.header_max_size,
                    pak.title_id,
                    pak.usage_sub_dir,
                    pak.creation_build_info,
                    hex(&pak.checksum)
                );
            }
        }
        "list" => {
            let key = args.get(2).map(|k| parse_key(k)).transpose()?.unwrap_or([0; 16]);
            let pak = Pak::open(Path::new(&args[1]), Some(key))?;
            println!("{} folders, {} files, flags {:#x}", pak.folders.len(), pak.files.len(), pak.flags);
            for (i, f) in pak.files.iter().enumerate() {
                println!(
                    "{i} {} class {:08x} size {} comp {} off {:#x} flags {:#x} unk {:#x}",
                    pak.file_path(i),
                    f.class_id,
                    f.uncompressed_size,
                    f.compressed_size,
                    f.offset,
                    f.flags,
                    f.unknown
                );
            }
        }
        "unpack" => {
            let pak = Pak::open(Path::new(&args[1]), Some(parse_key(&args[2])?))?;
            let i: usize = args[3].parse()?;
            let out = gearbox_packs::unpack::unpack(&pak, i)?;
            std::fs::write(&args[4], &out)?;
            println!("{} bytes", out.len());
            if let Some(r) = args.get(5) {
                let r = std::fs::read(r)?;
                let m = out.iter().zip(&r).position(|(a, b)| a != b);
                println!("reference {} bytes, first mismatch {:?}", r.len(), m);
            }
        }
        "find" => {
            let set = crate::game()?.open()?;
            let r = set.find(arg(&args, 1, "find <game path>")?).ok_or_else(|| anyhow::anyhow!("not found"))?;
            let pak = &set.paks[r.pak];
            println!("{} {} {}", pak.path.file_name().unwrap().to_string_lossy(), hex(&pak.key().unwrap()), r.file);
        }
        "check" => {
            use std::collections::BTreeMap;
            let set = crate::game()?.open()?;
            let filter = args.get(1).map(|s| s.to_lowercase());
            let mut stats: BTreeMap<(String, u32), [usize; 3]> = BTreeMap::new();
            for (r, f) in set.files() {
                let pak = set.paks[r.pak].path.file_name().unwrap().to_string_lossy().into_owned();
                if filter.as_ref().is_some_and(|x| !pak.to_lowercase().contains(x.as_str())) {
                    continue;
                }
                let st = stats.entry((pak.clone(), f.class_id)).or_default();
                match set.read(r) {
                    Ok(b) => {
                        let ok = !b.starts_with(b"GBX") || b.ends_with(&0xfacade01u32.to_le_bytes());
                        st[if ok { 0 } else { 1 }] += 1;
                    }
                    Err(e) => {
                        st[2] += 1;
                        println!("ERR {pak} {} class {:08x}: {e:#}", set.paks[r.pak].file_path(r.file), f.class_id);
                    }
                }
            }
            for ((p, c), s) in &stats {
                println!("{p} {c:08x}: {} ok, {} no end marker, {} errors", s[0], s[1], s[2]);
            }
            let decoded: usize = stats.values().map(|s| s[0] + s[1]).sum();
            let failed: usize = stats.values().map(|s| s[2]).sum();
            println!("total: {decoded} decoded, {failed} failures");
            anyhow::ensure!(failed == 0, "{failed} pack files failed to decode");
        }
        "paths" => {
            let set = crate::game()?.open()?;
            let sub = args.get(1).map(|s| s.to_lowercase());
            for (r, f) in set.files() {
                let p = set.paks[r.pak].file_path(r.file);
                if sub.as_ref().is_none_or(|s| p.to_lowercase().contains(s)) {
                    println!("{:08x} {p}", f.class_id);
                }
            }
        }
        "cat" => {
            let set = crate::game()?.open()?;
            let r = set.find(arg(&args, 1, "cat <game path> <out>")?).ok_or_else(|| anyhow::anyhow!("not found"))?;
            let b = set.read(r)?;
            let b = if b.starts_with(b"GBX") { gearbox_packs::embedded::plain_gbx(&b)? } else { b };
            std::fs::write(arg(&args, 2, "cat <game path> <out>")?, &b)?;
            println!("{} bytes", b.len());
        }
        "embedded" => {
            let map = gearbox_gbx::Map::parse(&std::fs::read(&args[1])?)?;
            let f = map.embedded.files.iter().find(|f| f.path.contains(args[2].as_str())).ok_or_else(|| anyhow::anyhow!("not found"))?;
            let b = if f.data.starts_with(b"GBX") { gearbox_packs::embedded::plain_gbx(&f.data)? } else { f.data.clone() };
            std::fs::write(&args[3], &b)?;
            println!("{} {} bytes", f.path, b.len());
        }
        "extract" => extract::extract(&crate::game()?.packs, Path::new(arg(&args, 1, "extract <out> [pak filter...]")?), &args[2..])?,
        "verify" => verify(&crate::game()?.packs, Path::new(arg(&args, 1, "verify <extract/GameData>")?))?,
        _ => bail!("unknown command {cmd}"),
    }
    Ok(())
}

fn arg<'a>(args: &'a [String], i: usize, usage: &str) -> Result<&'a str> {
    args.get(i).map(String::as_str).ok_or_else(|| anyhow::anyhow!("usage: gearbox assets {usage}"))
}

fn parse_key(s: &str) -> Result<[u8; 16]> {
    if s.len() != 32 {
        bail!("key must be 32 hex digits");
    }
    let mut k = [0u8; 16];
    for i in 0..16 {
        k[i] = u8::from_str_radix(&s[2 * i..2 * i + 2], 16)?;
    }
    Ok(k)
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    if let Ok(rd) = std::fs::read_dir(dir) {
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                walk(&p, out);
            } else {
                out.push(p);
            }
        }
    }
}

fn verify(packs: &Path, gamedata: &Path) -> Result<()> {
    use std::collections::BTreeMap;
    let set = gearbox_packs::packs::PackSet::open(packs)?;
    let mut files = Vec::new();
    walk(gamedata, &mut files);
    files.sort();
    let mut stats: BTreeMap<String, [usize; 4]> = BTreeMap::new();
    let mut missing = 0;
    for f in &files {
        let rel = f.strip_prefix(gamedata).unwrap().to_string_lossy().replace('/', "\\");
        if rel.ends_with(".DS_Store") {
            continue;
        }
        let Some(r) = set.find(&rel) else {
            missing += 1;
            println!("MISSING {rel}");
            continue;
        };
        let pak = set.paks[r.pak].path.file_name().unwrap().to_string_lossy().into_owned();
        let st = stats.entry(pak).or_default();
        let reference = std::fs::read(f)?;
        match set.read(r) {
            Ok(out) if out == reference => st[0] += 1,
            Ok(out) if reference.len() < out.len() && out.starts_with(&reference) => st[1] += 1,
            Ok(out) => {
                st[2] += 1;
                let m = out.iter().zip(&reference).position(|(a, b)| a != b);
                let facade = |b: &[u8]| b.ends_with(&0xfacade01u32.to_le_bytes());
                println!(
                    "DIFF {rel} ({} vs {} bytes, first diff {m:?}, ends with FACADE01: ours {} reference {})",
                    out.len(),
                    reference.len(),
                    facade(&out),
                    facade(&reference)
                );
            }
            Err(e) => {
                st[3] += 1;
                println!("ERR {rel}: {e}");
            }
        }
    }
    for (p, s) in &stats {
        println!("{p}: {} identical, {} reference truncated, {} differ, {} errors", s[0], s[1], s[2], s[3]);
    }
    println!("{missing} reference files not found in packs");
    Ok(())
}
