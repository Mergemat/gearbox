use anyhow::{Context, Result, bail, ensure};

use crate::reader::Reader;

pub struct Container<'a> {
    pub class_id: u32,
    pub header_chunks: Vec<(u32, &'a [u8])>,
    pub body: Vec<u8>,
}

impl<'a> Container<'a> {
    pub fn parse(buf: &'a [u8]) -> Result<Self> {
        let mut r = Reader::new(buf);
        ensure!(r.bytes(3)? == b"GBX", "not a GBX file");
        let version = r.u16()?;
        ensure!(version >= 6, "GBX version {version} not supported (TM2020 files are version 6)");
        let format = r.u8()?;
        ensure!(format == b'B', "text GBX format is not supported");
        let _ref_table_compression = r.u8()?;
        let body_compression = r.u8()?;
        let _unknown = r.u8()?;
        let class_id = r.u32()?;

        let user_data_len = r.len()?;
        let user_data = r.bytes(user_data_len)?;
        let header_chunks = parse_header_chunks(user_data).context("header chunks")?;

        r.u32()?;
        skip_ref_table(&mut r).context("reference table")?;

        let body = match body_compression {
            b'C' => {
                let uncompressed = r.len()?;
                let compressed = r.len()?;
                let data = r.bytes(compressed)?;
                lzokay_native::decompress_all(data, Some(uncompressed))
                    .map_err(|e| anyhow::anyhow!("LZO body decompression failed: {e:?}"))?
            }
            b'U' => r.bytes(r.remaining())?.to_vec(),
            c => bail!("unknown body compression byte {c:#x}"),
        };

        Ok(Self { class_id, header_chunks, body })
    }

    pub fn header(buf: &'a [u8]) -> Result<(u32, Vec<(u32, &'a [u8])>)> {
        let mut r = Reader::new(buf);
        ensure!(r.bytes(3)? == b"GBX", "not a GBX file");
        let version = r.u16()?;
        ensure!(version >= 6, "GBX version {version} not supported");
        ensure!(r.u8()? == b'B', "text GBX format is not supported");
        r.u8()?;
        r.u8()?;
        r.u8()?;
        let class_id = r.u32()?;
        let user_data_len = r.len()?;
        let user_data = r.bytes(user_data_len)?;
        Ok((class_id, parse_header_chunks(user_data).context("header chunks")?))
    }

    pub fn header_chunk(&self, id: u32) -> Option<&'a [u8]> {
        self.header_chunks.iter().find(|(cid, _)| *cid == id).map(|(_, d)| *d)
    }
}

fn parse_header_chunks(user_data: &[u8]) -> Result<Vec<(u32, &[u8])>> {
    if user_data.is_empty() {
        return Ok(Vec::new());
    }
    let mut r = Reader::new(user_data);
    let count = r.len()?;
    let mut descs = Vec::with_capacity(count);
    for _ in 0..count {
        let id = r.u32()?;
        let size = (r.u32()? & 0x7FFF_FFFF) as usize;
        descs.push((id, size));
    }
    descs.into_iter().map(|(id, size)| Ok((id, r.bytes(size)?))).collect()
}

fn skip_ref_table(r: &mut Reader) -> Result<()> {
    let count = r.len()?;
    if count == 0 {
        return Ok(());
    }
    let _ancestor_level = r.i32()?;
    skip_dirs(r)?;
    for _ in 0..count {
        let flags = r.i32()?;
        let is_resource = flags & 4 != 0;
        if is_resource {
            r.i32()?;
        } else {
            r.string()?;
        }
        let _node_index = r.i32()?;
        let _use_file = r.bool32()?;
        if !is_resource {
            let _dir_index = r.i32()?;
        }
    }
    Ok(())
}

fn skip_dirs(r: &mut Reader) -> Result<()> {
    let children = r.len()?;
    for _ in 0..children {
        r.string()?;
        skip_dirs(r)?;
    }
    Ok(())
}
