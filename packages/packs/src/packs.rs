use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use anyhow::{Context, Result};

use crate::pak::{Pak, key_from_base};

pub fn name_hash(name: &str) -> String {
    let lower = name.to_lowercase();
    let b = lower.as_bytes();
    let mut d = vec![b.len() as u8];
    d.extend_from_slice(&crate::md5::md5(b));
    const H: &[u8; 16] = b"0123456789ABCDEF";
    d.iter().flat_map(|&x| [H[(x % 16) as usize] as char, H[(x / 16) as usize] as char]).collect()
}

pub fn environment(collection: Option<&str>) -> Option<&'static str> {
    match collection? {
        "26" | "Stadium" => Some("Stadium"),
        "25" => Some("Stadium256"),
        "15" => Some("GreenCoast"),
        "16" => Some("RedIsland"),
        "28" => Some("BlueBay"),
        "29" => Some("WhiteShore"),
        _ => None,
    }
}

const BASE_KEYS: [(&str, &str); 12] = [
    ("Stadium", "5B985F4CC43013C458A4C99A4B285E1E"),
    ("Maniaplanet", "24283C7980351AAB426F880FBA61817C"),
    ("Maniaplanet_Core", "24283C7980351AAB426F880FBA61817C"),
    ("Maniaplanet_ModelsSport", "24283C7980351AAB426F880FBA61817C"),
    ("Skins_StadiumPrestige", "24283C7980351AAB426F880FBA61817C"),
    ("BlueBay", "105C22EB2CFAC821DF2282E634B453BD"),
    ("GreenCoast", "105C22EB2CFAC821DF2282E634B453BD"),
    ("RedIsland", "105C22EB2CFAC821DF2282E634B453BD"),
    ("WhiteShore", "105C22EB2CFAC821DF2282E634B453BD"),
    ("Resource", "087480148E51B70DD83AF2D86F974FEF"),
    ("Titles", "29E6D9E44FB38ED1BC1F21E400FF273F"),
    ("Trackmania.Title.Pack", "50106CD3E964B316CAA0780F4949737D"),
];

pub fn keys() -> &'static HashMap<String, [u8; 16]> {
    static KEYS: OnceLock<HashMap<String, [u8; 16]>> = OnceLock::new();
    KEYS.get_or_init(|| {
        BASE_KEYS
            .iter()
            .map(|(name, hex)| {
                let base: [u8; 16] =
                    std::array::from_fn(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap());
                (name.to_lowercase(), key_from_base(&base))
            })
            .collect()
    })
}

pub fn key(pack: &str) -> Option<[u8; 16]> {
    keys().get(&pack.to_lowercase()).copied()
}

fn norm(p: &str) -> String {
    p.replace('/', "\\").to_lowercase()
}

#[derive(Clone, Copy, Debug)]
pub struct FileRef {
    pub pak: usize,
    pub file: usize,
}

pub struct PackSet {
    pub paks: Vec<Pak>,
    index: HashMap<String, FileRef>,
}

impl PackSet {
    pub fn open(dir: &Path) -> Result<PackSet> {
        let mut entries: Vec<PathBuf> = std::fs::read_dir(dir)
            .with_context(|| format!("reading {}", dir.display()))?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| {
                let n = p.file_name().and_then(|n| n.to_str()).unwrap_or("").to_lowercase();
                n.ends_with(".pak") || n.ends_with(".pack.gbx")
            })
            .collect();
        entries.sort();
        let mut paks = Vec::new();
        for p in entries {
            let fname = p.file_name().unwrap().to_str().unwrap().to_lowercase();
            let stem = fname.trim_end_matches(".pak").trim_end_matches(".gbx").to_string();
            let mut pak = Pak::open(&p, None)?;
            match keys().get(&stem) {
                Some(k) => pak.set_key(*k).with_context(|| format!("{}", p.display()))?,
                None => {
                    if pak.header_flags & 1 == 0 && pak.header_flags & 3 != 0 {
                        pak.set_key([0; 16]).ok();
                    } else {
                        continue;
                    }
                }
            }
            paks.push(pak);
        }
        let mut index = HashMap::new();
        for (pi, pak) in paks.iter().enumerate() {
            for fi in 0..pak.files.len() {
                index.entry(norm(&pak.file_path(fi))).or_insert(FileRef { pak: pi, file: fi });
            }
        }
        Ok(PackSet { paks, index })
    }

    pub fn find(&self, path: &str) -> Option<FileRef> {
        let p = norm(path);
        if let Some(r) = self.index.get(&p) {
            return Some(*r);
        }
        let parts: Vec<&str> = p.split('\\').collect();
        for k in (0..parts.len()).rev() {
            let dir: String = parts[..k].iter().map(|s| format!("{s}\\")).collect();
            let name = parts[k..].join("\\");
            if let Some(r) = self.index.get(&format!("{dir}{}", name_hash(&name).to_lowercase())) {
                return Some(*r);
            }
        }
        None
    }

    pub fn read(&self, r: FileRef) -> Result<Vec<u8>> {
        crate::unpack::unpack(&self.paks[r.pak], r.file)
    }

    pub fn read_path(&self, path: &str) -> Result<Vec<u8>> {
        let r = self.find(path).with_context(|| format!("{path}: not in any pack"))?;
        self.read(r)
    }

    pub fn files(&self) -> impl Iterator<Item = (FileRef, &crate::pak::PakFile)> {
        self.paks.iter().enumerate().flat_map(|(pi, p)| {
            p.files.iter().enumerate().map(move |(fi, f)| (FileRef { pak: pi, file: fi }, f))
        })
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn name_hash_matches_pack() {
        assert_eq!(super::name_hash("TuningsSport.VehicleTunings.Gbx"), "F1979CFDD90ED1A48C1487A4B3B395D9B3");
    }
}
