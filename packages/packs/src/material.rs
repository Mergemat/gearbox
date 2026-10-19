use anyhow::{Context, Result, bail};
use gearbox_gbx::reader::Reader;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MaterialIds {
    pub surface: u8,
    pub gameplay: u8,
}

pub fn material_ids(buf: &[u8]) -> Result<MaterialIds> {
    let w = 0x09079017u32.to_le_bytes();
    let p = buf.windows(4).position(|x| x == w).context("material without chunk 0x09079017")?;
    let v = u32::from_le_bytes(buf.get(p + 4..p + 8).context("truncated")?.try_into().unwrap());
    if v < 1 || v > 16 {
        bail!("material chunk 0x09079017 version {v}");
    }
    Ok(MaterialIds { surface: buf[p + 8], gameplay: buf[p + 9] })
}

pub fn user_material_ids(buf: &[u8], start: usize) -> Result<MaterialIds> {
    let mut r = Reader::new(buf);
    r.skip(start)?;
    anyhow::ensure!(r.u32()? == 0x090fd000, "user material chunk");
    let version = r.u32()?;
    anyhow::ensure!((10..=11).contains(&version), "user material version {version}");
    if version >= 11 {
        r.bool8()?;
    }
    crate::crystal::id(&mut r)?;
    crate::crystal::id(&mut r)?;
    r.string()?;
    Ok(MaterialIds { surface: r.u8()?, gameplay: r.u8()? })
}

#[derive(Clone, Debug, PartialEq)]
pub struct Modifier {
    pub skin: Option<String>,
    pub inline_skinned: Vec<(String, String)>,
    pub folder: String,
}

pub fn modifier(buf: &[u8], path: &str) -> Result<Modifier> {
    let layout = crate::nodes::layout(buf)?;
    let head = &layout.head;
    let w = 0x0915d000u32.to_le_bytes();
    let p = buf[head.body_start..].windows(4).position(|x| x == w).context("no chunk 0x0915D000")?
        + head.body_start;
    let mut r = Reader::new(buf);
    r.skip(p + 4)?;
    let ctx = crate::prefab::Ctx { buf, layout: &layout, path };
    let mut inline_skinned = Vec::new();
    let skin = ctx.noderef(&mut r, |c, r, _| {
        let start = r.pos();
        c.skip_node(r)?;
        inline_skinned = material_paths(&buf[start..r.pos()]);
        Ok(())
    })?;
    let skin = match skin {
        crate::prefab::NodeLink::External(s) => Some(s),
        _ => None,
    };
    let folder = r.string()?;
    Ok(Modifier { skin, inline_skinned, folder })
}

fn material_paths(b: &[u8]) -> Vec<(String, String)> {
    let mut out: Vec<(String, String)> = Vec::new();
    let mut prev: Option<(usize, String)> = None;
    let mut p = 0;
    while p + 4 <= b.len() {
        let n = u32::from_le_bytes(b[p..p + 4].try_into().unwrap()) as usize;
        if (1..400).contains(&n) && p + 4 + n <= b.len() {
            let s = &b[p + 4..p + 4 + n];
            if s.iter().all(|&c| (0x20..0x7f).contains(&c)) {
                let t = String::from_utf8_lossy(s).into_owned();
                if t.to_ascii_lowercase().ends_with(".material.gbx") {
                    let name = match &prev {
                        Some((end, nm)) if *end == p => nm.clone(),
                        _ => file_stem(&t),
                    };
                    if !out.iter().any(|(a, b)| *a == name && *b == t) {
                        out.push((name, t));
                    }
                    prev = None;
                } else {
                    prev = Some((p + 4 + n, t));
                }
                p += 4 + n;
                continue;
            }
        }
        p += 1;
    }
    out
}

fn file_stem(path: &str) -> String {
    let f = path.rsplit(|c| c == '/' || c == '\\').next().unwrap_or(path);
    f.split('.').next().unwrap_or("").to_string()
}

pub fn skin_materials(buf: &[u8]) -> Result<Vec<(String, String)>> {
    let head_end = crate::unpack::parse_head(buf).context("truncated header")??.body_start;
    Ok(material_paths(&buf[..head_end]))
}

pub fn skin_files(buf: &[u8]) -> Result<Vec<(u32, String, String)>> {
    let c = gearbox_gbx::container::Container::parse(buf)?;
    anyhow::ensure!(c.class_id == 0x090f4000, "not a game skin");
    let mut r = Reader::new(c.header_chunk(0x090f4000).context("game skin header absent")?);
    let version = r.u8()?;
    anyhow::ensure!(version <= 8, "game skin header version {version}");
    r.string()?;
    if version >= 1 {
        r.string()?;
        r.string()?;
    }
    let n = r.u8()?;
    (0..n)
        .map(|_| {
            let entry = (r.u32()?, r.string()?, r.string()?);
            if version >= 2 {
                r.bool32()?;
            }
            Ok(entry)
        })
        .collect()
}

pub fn replacement(
    material: &str,
    folder: &str,
    skin_folder: Option<&str>,
    skinned: &[(String, String)],
) -> Option<String> {
    let norm = |s: &str| s.replace('/', "\\").to_lowercase();
    let (name, _) = skinned.iter().find(|(_, p)| norm(p) == norm(material))?;
    let (folder, name) = match name.strip_prefix('*') {
        Some(n) => (skin_folder.unwrap_or(folder), n),
        None => (folder, name.as_str()),
    };
    if folder.is_empty() {
        return None;
    }
    Some(format!("{folder}{name}.Material.Gbx"))
}
