use std::io::{Cursor, Read};

use anyhow::{Context, Result, ensure};
use serde::Serialize;

use crate::{Container, reader::Reader};

#[derive(Debug, Default, Serialize)]
pub struct EmbeddedData {
    pub models: Vec<[Option<String>; 3]>,
    pub files: Vec<EmbeddedFile>,
    pub textures: Vec<String>,
}

#[derive(Debug, Serialize)]
pub struct EmbeddedFile {
    pub path: String,
    #[serde(skip)]
    pub data: Vec<u8>,
    pub item_model: Option<EmbeddedItemModel>,
}

#[derive(Debug, Serialize)]
pub struct EmbeddedItemModel {
    pub ident: [Option<String>; 3],
    pub item_type: i32,
}

impl EmbeddedFile {
    pub fn model_path(&self) -> Option<String> {
        self.item_model.as_ref()?;
        let mut path = self.path.replace('/', "\\");
        let namespaces = ["Items", "Blocks", "ClubItems"];
        let parts: Vec<_> = path.split('\\').collect();
        if !namespaces.iter().any(|n| parts[0].eq_ignore_ascii_case(n))
            && let Some(root) = parts.windows(2).position(|p| {
                ["Trackmania", "Trackmania2020"].iter().any(|n| p[0].eq_ignore_ascii_case(n))
                    && namespaces.iter().any(|n| p[1].eq_ignore_ascii_case(n))
            })
        {
            path = parts[root + 1..].join("\\");
        }
        for prefix in ["Items\\", "Blocks\\"] {
            if path.get(..prefix.len()).is_some_and(|p| p.eq_ignore_ascii_case(prefix)) {
                return Some(path[prefix.len()..].to_owned());
            }
        }
        if let Some(p) = path.get(..10).filter(|p| p.eq_ignore_ascii_case("ClubItems\\")) {
            return Some(format!("club:{}", &path[p.len()..]));
        }
        Some(path)
    }
}

impl EmbeddedData {
    pub fn item_file(&self, model: &str) -> Result<&EmbeddedFile> {
        let name = model.replace('/', "\\");
        let named = |same: &dyn Fn(&str) -> bool| -> Vec<&EmbeddedFile> {
            self.files
                .iter()
                .filter(|f| {
                    let ident = f.item_model.as_ref().and_then(|m| m.ident[0].clone());
                    [f.model_path(), ident].into_iter().flatten().any(|key| same(&key))
                })
                .collect()
        };
        let mut candidates = named(&|key| key == name);
        if candidates.is_empty() {
            candidates = named(&|key| key.eq_ignore_ascii_case(&name));
        }
        ensure!(candidates.len() <= 1, "ambiguous embedded item {model}");
        candidates.first().copied().with_context(|| format!("missing embedded item {model}"))
    }
}

pub(super) fn read(r: &mut Reader) -> Result<EmbeddedData> {
    let version = r.u32()?;
    ensure!(version <= 1, "embedded objects version {version}");
    let mut e = r.encapsulated()?;
    let mut embedded = EmbeddedData::default();
    for _ in 0..e.len()? {
        embedded.models.push(e.ident()?);
    }
    let zip = e.data()?;
    if !zip.is_empty() {
        let mut archive = zip::ZipArchive::new(Cursor::new(zip)).context("embedded ZIP")?;
        for i in 0..archive.len() {
            let mut file = archive.by_index(i)?;
            if file.is_dir() {
                continue;
            }
            let path = file.name()?.replace('/', "\\");
            let mut data = Vec::new();
            file.read_to_end(&mut data).with_context(|| format!("embedded file {path}"))?;
            let item_model = if path.to_ascii_lowercase().ends_with(".gbx") {
                let c = Container::parse(&data).with_context(|| format!("embedded GBX {path}"))?;
                if c.class_id == 0x2E00_2000 {
                    let desc = c.header_chunk(0x2E00_1003).context("item model without collector header")?;
                    let ident = Reader::new(desc).ident()?;
                    let desc = c.header_chunk(0x2E00_2000).context("item model without type header")?;
                    let item_type = Reader::new(desc).i32()?;
                    Some(EmbeddedItemModel { ident, item_type })
                } else {
                    None
                }
            } else {
                None
            };
            embedded.files.push(EmbeddedFile { path, data, item_model });
        }
    }
    if version >= 1 {
        for _ in 0..e.len()? {
            embedded.textures.push(e.string()?);
        }
    }
    ensure!(e.remaining() == 0, "unread embedded objects data");
    Ok(embedded)
}
