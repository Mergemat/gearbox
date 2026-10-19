
use anyhow::{Context, Result, ensure};

pub fn plain_gbx(buf: &[u8]) -> Result<Vec<u8>> {
    ensure!(buf.starts_with(b"GBX"), "not a GBX file");
    if buf.get(7) == Some(&b'U') {
        return Ok(buf.to_vec());
    }
    let mut tmp = buf.to_vec();
    tmp[7] = b'U';
    let head = crate::unpack::parse_head(&tmp).context("truncated header")??;
    let p = head.body_start;
    let ulen = u32::from_le_bytes(buf.get(p..p + 4).context("truncated")?.try_into().unwrap()) as usize;
    let clen = u32::from_le_bytes(buf.get(p + 4..p + 8).context("truncated")?.try_into().unwrap()) as usize;
    let data = buf.get(p + 8..p + 8 + clen).context("truncated body")?;
    let body = lzokay_native::decompress_all(data, Some(ulen)).map_err(|e| anyhow::anyhow!("LZO: {e:?}"))?;
    let mut out = tmp[..p].to_vec();
    out.extend_from_slice(&body);
    Ok(out)
}

pub struct Files<'a> {
    by_path: std::collections::HashMap<String, Vec<u8>>,
    packs: &'a dyn crate::catalog::Source,
}

fn norm(p: &str) -> String {
    p.replace('/', "\\").to_lowercase()
}

impl<'a> Files<'a> {
    pub fn new<'b>(files: impl IntoIterator<Item = (&'b str, &'b [u8])>, packs: &'a dyn crate::catalog::Source) -> Self {
        Self { by_path: files.into_iter().map(|(p, b)| (norm(p), b.to_vec())).collect(), packs }
    }
}

impl crate::catalog::Source for Files<'_> {
    fn read_path(&self, path: &str) -> Result<Vec<u8>> {
        match self.by_path.get(&norm(path)) {
            Some(b) => plain_gbx(b),
            None => self.packs.read_path(path),
        }
    }
    fn class_of(&self, path: &str) -> Option<u32> {
        let Some(mut b) = self.by_path.get(&norm(path)).cloned() else { return self.packs.class_of(path) };
        if b.len() > 7 {
            b[7] = b'U';
        }
        crate::unpack::parse_head(&b)?.ok().map(|h| h.class_id)
    }
}
