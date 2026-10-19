use anyhow::{Context, Result, bail, ensure};

pub const FACADE: u32 = 0xFACA_DE01;
pub const SKIP: u32 = 0x534B_4950;

const MAX_LEN: usize = 0x1000_0000;

pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
    id_version: Option<u32>,
    ids: Vec<String>,
    nodes: std::collections::HashSet<i32>,
}

pub enum NodeRef {
    Null,
    Shared(i32),
    New(u32),
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            id_version: None,
            ids: Vec::new(),
            nodes: Default::default(),
        }
    }

    pub fn buf(&self) -> &'a [u8] {
        self.buf
    }

    pub fn pos(&self) -> usize {
        self.pos
    }

    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8]> {
        ensure!(
            n <= self.remaining(),
            "read of {n} bytes at offset {} overruns buffer of {} bytes",
            self.pos,
            self.buf.len()
        );
        let out = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }

    pub fn skip(&mut self, n: usize) -> Result<()> {
        self.bytes(n).map(|_| ())
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N]> {
        Ok(self.bytes(N)?.try_into().unwrap())
    }

    pub fn u8(&mut self) -> Result<u8> {
        Ok(self.array::<1>()?[0])
    }

    pub fn u16(&mut self) -> Result<u16> {
        Ok(u16::from_le_bytes(self.array()?))
    }

    pub fn i16(&mut self) -> Result<i16> {
        Ok(i16::from_le_bytes(self.array()?))
    }

    pub fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.array()?))
    }

    pub fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(self.array()?))
    }

    pub fn f32(&mut self) -> Result<f32> {
        Ok(f32::from_le_bytes(self.array()?))
    }

    pub fn vec3(&mut self) -> Result<[f32; 3]> {
        Ok([self.f32()?, self.f32()?, self.f32()?])
    }

    pub fn bool32(&mut self) -> Result<bool> {
        match self.u32()? {
            0 => Ok(false),
            1 => Ok(true),
            v => bail!("invalid boolean {v} at offset {}", self.pos - 4),
        }
    }

    pub fn bool8(&mut self) -> Result<bool> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            v => bail!("invalid byte boolean {v} at offset {}", self.pos - 1),
        }
    }

    pub fn opt_i32(&mut self) -> Result<Option<i32>> {
        let v = self.i32()?;
        Ok((v != -1).then_some(v))
    }

    pub fn len(&mut self) -> Result<usize> {
        let n = self.i32()?;
        ensure!(
            n >= 0 && (n as usize) < MAX_LEN,
            "invalid length {n} at offset {}",
            self.pos - 4
        );
        Ok(n as usize)
    }

    pub fn string(&mut self) -> Result<String> {
        let n = self.len()?;
        Ok(String::from_utf8_lossy(self.bytes(n)?).into_owned())
    }

    pub fn data(&mut self) -> Result<&'a [u8]> {
        let n = self.len()?;
        self.bytes(n)
    }

    pub fn encapsulated(&mut self) -> Result<Reader<'a>> {
        ensure!(self.u32()? == 0, "unsupported encapsulation version");
        Ok(Reader::new(self.data()?))
    }

    pub fn id(&mut self) -> Result<Option<String>> {
        if self.id_version.is_none() {
            let v = self.u32()?;
            ensure!(v >= 3, "unsupported id version {v}");
            self.id_version = Some(v);
        }
        let index = self.u32()?;
        if index == u32::MAX {
            return Ok(None);
        }
        let flags = index & 0xC000_0000;
        if flags != 0x4000_0000 && flags != 0x8000_0000 {
            return Ok(Some(index.to_string()));
        }
        let slot = (index & 0x3FFF_FFFF) as usize;
        if slot == 0 {
            let s = self.string()?;
            self.ids.push(s.clone());
            return Ok(Some(s));
        }
        self.ids.get(slot - 1).cloned().map(Some).with_context(|| {
            format!(
                "id lookback index {slot} out of range ({} known)",
                self.ids.len()
            )
        })
    }

    pub fn ident(&mut self) -> Result<[Option<String>; 3]> {
        Ok([self.id()?, self.id()?, self.id()?])
    }

    pub fn pack_desc(&mut self) -> Result<String> {
        let version = self.u8()?;
        if version >= 3 {
            self.skip(32)?;
        }
        let path = self.string()?;
        if (!path.is_empty() && version >= 1) || version >= 3 {
            self.string()?;
        }
        Ok(path)
    }

    pub fn chunks(
        &mut self,
        class: &str,
        mut handle: impl FnMut(&mut Self, u32, bool) -> Result<bool>,
    ) -> Result<()> {
        loop {
            let id = self.u32()?;
            if id == FACADE {
                return Ok(());
            }
            let skippable = self.peek_u32() == Some(SKIP);
            if skippable {
                self.skip(4)?;
                let size = self.len()?;
                let start = self.pos;
                let end = start + size;
                ensure!(
                    end <= self.buf.len(),
                    "skippable chunk 0x{id:08X} overruns buffer"
                );
                let handled =
                    handle(self, id, true).with_context(|| format!("{class} chunk 0x{id:08X}"))?;
                if handled {
                    ensure!(
                        self.pos <= end,
                        "{class} chunk 0x{id:08X} read {} bytes past its declared size",
                        self.pos - end
                    );
                }
                self.pos = end;
            } else {
                let start = self.pos;
                let handled =
                    handle(self, id, false).with_context(|| format!("{class} chunk 0x{id:08X}"))?;
                if !handled {
                    bail!(
                        "unknown non-skippable {class} chunk 0x{id:08X} at offset {}",
                        start - 4
                    );
                }
            }
        }
    }

    pub fn node_ref(&mut self) -> Result<NodeRef> {
        let index = self.i32()?;
        if index == -1 {
            return Ok(NodeRef::Null);
        }
        if !self.nodes.insert(index) {
            return Ok(NodeRef::Shared(index));
        }
        Ok(NodeRef::New(self.u32()?))
    }

    pub fn peek_u32(&self) -> Option<u32> {
        self.buf
            .get(self.pos..self.pos + 4)
            .map(|b| u32::from_le_bytes(b.try_into().unwrap()))
    }
}
