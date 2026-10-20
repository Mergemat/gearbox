use anyhow::{Context, Result, ensure};
use gearbox_gbx::reader::Reader;

use crate::{packs::PackSet, unpack};

#[derive(Clone, Debug)]
pub struct TerrainWater {
    pub blocks: Vec<String>,
    pub surface: f32,
    pub bottom: f32,
}

fn chunk(buf: &[u8], start: usize, id: u32) -> Result<Reader<'_>> {
    let at = buf[start..]
        .windows(4)
        .position(|bytes| bytes == id.to_le_bytes())
        .with_context(|| format!("missing water chunk {id:08x}"))?;
    let mut reader = Reader::new(buf);
    reader.skip(start + at + 4)?;
    Ok(reader)
}

fn identifier(reader: &mut Reader<'_>) -> Result<Option<String>> {
    let id = reader.u32()?;
    if id == u32::MAX {
        return Ok(None);
    }
    ensure!(matches!(id, 0x4000_0000 | 0x8000_0000), "unsupported water identifier {id:08x}");
    Ok(Some(reader.string()?))
}

pub fn terrain(packs: &PackSet, environment: &str) -> Result<TerrainWater> {
    let path = format!("Collections\\{environment}.Collection.Gbx");
    let bytes = packs.read_path(&path)?;
    let head = unpack::parse_head(&bytes).context("collection header")??;
    let mut reader = chunk(&bytes, head.body_start, 0x03033038)?;
    ensure!(reader.u32()? == 8, "unsupported collection water version");
    ensure!(reader.u32()? == u32::MAX, "collection water array not decoded");
    reader.u32()?;
    let name = identifier(&mut reader)?;
    let mut water = TerrainWater { blocks: Vec::new(), surface: reader.f32()?, bottom: reader.f32()? };
    for &index in head.ref_files.keys() {
        let Some(path) = head.ref_path(&path, index) else { continue };
        if !path.to_ascii_lowercase().ends_with(".zoneflat.gbx") {
            continue;
        }
        let bytes = packs.read_path(&path)?;
        let zone = unpack::parse_head(&bytes).context("terrain zone header")??;
        let mut reader = chunk(&bytes, zone.body_start, 0x0305c003)?;
        ensure!(reader.u32()? == 0, "unsupported terrain zone version");
        let block = reader.id()?.context("terrain zone name")?;
        reader.id()?;
        let water_at = chunk(&bytes, reader.pos(), 0x0305c00b)?.pos();
        reader.skip(water_at - reader.pos())?;
        if reader.id()? == name && name.is_some() {
            water.blocks.push(block);
        }
    }
    Ok(water)
}
