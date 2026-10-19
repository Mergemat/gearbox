use std::fs::File;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail, ensure};

use crate::crypto::{Blowfish, Cipher};

pub const HEADER_KEY: [u8; 16] = [
    0x56, 0xee, 0xcb, 0xbb, 0xde, 0xb6, 0xbc, 0x90, 0xa1, 0x7d, 0xfc, 0xeb, 0x76, 0x1d, 0x59, 0xce,
];

pub fn key_from_base(base: &[u8; 16]) -> [u8; 16] {
    let mut s: Vec<u8> = base.iter().flat_map(|b| format!("{b:02X}").into_bytes()).collect();
    s.extend_from_slice(b"NadeoPak");
    crate::md5::md5(&s)
}

pub fn key_from_services(ms_key: &[u8; 16], checksum: &[u8; 32]) -> [u8; 16] {
    let mut k = [0u8; 16];
    for i in 0..8 {
        k[i] = ms_key[15 - i] ^ checksum[i];
        k[i + 8] = ms_key[7 - i] ^ checksum[i + 16];
    }
    k
}

#[derive(Clone, Debug)]
pub struct Folder {
    pub parent: Option<usize>,
    pub name: String,
}

#[derive(Clone, Debug)]
pub struct PakFile {
    pub folder: Option<usize>,
    pub name: String,
    pub unknown: u32,
    pub uncompressed_size: u32,
    pub compressed_size: u32,
    pub offset: u32,
    pub class_id: u32,
    pub flags: u64,
}

impl PakFile {
    pub fn is_compressed(&self) -> bool {
        self.flags & 0x3c != 0
    }
    pub fn dont_use_dummy_write(&self) -> bool {
        self.flags & 0x1_0000_0000 != 0
    }
    pub fn is_public(&self) -> bool {
        self.flags & 0x2_0000_0000_0000 != 0
    }
    pub fn force_no_crypt(&self) -> bool {
        self.flags & 0x4_0000_0000_0000 != 0
    }
    pub fn is_encrypted(&self) -> bool {
        !self.force_no_crypt() && !self.is_public()
    }
}

struct Plain<'a> {
    b: &'a [u8],
    pos: usize,
}

impl Plain<'_> {
    fn bytes(&mut self, n: usize) -> Result<&[u8]> {
        ensure!(self.pos + n <= self.b.len(), "pak header truncated");
        let s = &self.b[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }
    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.bytes(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.bytes(8)?.try_into().unwrap()))
    }
    fn string(&mut self) -> Result<String> {
        let n = self.u32()? as usize;
        ensure!(n < 1 << 20, "bad string length {n}");
        Ok(String::from_utf8_lossy(self.bytes(n)?).into_owned())
    }
}

pub struct DecReader<'a, 'k> {
    src: &'a [u8],
    src_pos: usize,
    cipher: Cipher<'k>,
    block: [u8; 8],
    block_pos: usize,
}

impl<'a, 'k> DecReader<'a, 'k> {
    pub fn new(src: &'a [u8], cipher: Cipher<'k>) -> Self {
        DecReader { src, src_pos: 0, cipher, block: [0; 8], block_pos: 8 }
    }
    pub fn cipher(&mut self) -> &mut Cipher<'k> {
        &mut self.cipher
    }
    pub fn byte(&mut self) -> Result<u8> {
        if self.block_pos == 8 {
            ensure!(self.src_pos + 8 <= self.src.len(), "encrypted data truncated");
            let c = u64::from_le_bytes(self.src[self.src_pos..self.src_pos + 8].try_into().unwrap());
            self.src_pos += 8;
            self.block = self.cipher.next_block(c).to_le_bytes();
            self.block_pos = 0;
        }
        let b = self.block[self.block_pos];
        self.block_pos += 1;
        Ok(b)
    }
    pub fn read(&mut self, out: &mut [u8]) -> Result<()> {
        for o in out {
            *o = self.byte()?;
        }
        Ok(())
    }
    fn arr<const N: usize>(&mut self) -> Result<[u8; N]> {
        let mut a = [0; N];
        self.read(&mut a)?;
        Ok(a)
    }
    pub fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.arr()?))
    }
    pub fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.arr()?))
    }
    pub fn string(&mut self) -> Result<String> {
        let n = self.u32()? as usize;
        ensure!(n < 1 << 16, "bad string length {n}");
        let mut v = vec![0; n];
        self.read(&mut v)?;
        Ok(String::from_utf8_lossy(&v).into_owned())
    }
}

pub struct Pak {
    pub path: PathBuf,
    file: File,
    pub len: u64,
    pub version: u32,
    pub checksum: [u8; 32],
    pub header_flags: u32,
    pub header_max_size: u32,
    pub title_id: String,
    pub usage_sub_dir: String,
    pub creation_build_info: String,
    header_iv_offset: usize,
    header: Vec<u8>,
    key: Option<[u8; 16]>,
    bf: Option<Blowfish>,
    pub gbx_headers_start: u32,
    pub file_size: u32,
    pub flags: u32,
    pub folders: Vec<Folder>,
    pub files: Vec<PakFile>,
}

impl Pak {
    pub fn open(path: &Path, key: Option<[u8; 16]>) -> Result<Pak> {
        let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
        let mut head = vec![0u8; 0x4000];
        let n = file.read_at(&mut head, 0)?;
        head.truncate(n);
        ensure!(head.starts_with(b"NadeoPak"), "{}: not a NadeoPak", path.display());
        let mut r = Plain { b: &head, pos: 8 };
        let version = r.u32()?;
        ensure!(version == 18, "{}: pack version {version}, only 18 is supported", path.display());
        let checksum: [u8; 32] = r.bytes(32)?.try_into().unwrap();
        let header_flags = r.u32()?;
        let header_max_size = r.u32()?;
        r.u32()?;
        for _ in 0..4 {
            r.string()?;
        }
        let _manialink = r.string()?;
        let _download = r.string()?;
        let _date = r.u64()?;
        let _comment = r.string()?;
        let _xml = r.string()?;
        let title_id = r.string()?;
        let usage_sub_dir = r.string()?;
        let creation_build_info = r.string()?;
        r.bytes(16)?;
        let included = r.u32()?;
        for _ in 0..included {
            r.bytes(32)?;
            r.string()?;
            r.u32()?;
            for _ in 0..5 {
                r.string()?;
            }
            r.u64()?;
            r.string()?;
            r.u32()?;
        }
        let header_iv_offset = r.pos;
        let mut header = vec![0u8; header_max_size as usize];
        let n = file.read_at(&mut header, 0)?;
        header.truncate(n);
        let mut pak = Pak {
            path: path.to_owned(),
            len: file.metadata()?.len(),
            file,
            version,
            checksum,
            header_flags,
            header_max_size,
            title_id,
            usage_sub_dir,
            creation_build_info,
            header_iv_offset,
            header,
            key: None,
            bf: None,
            gbx_headers_start: 0,
            file_size: 0,
            flags: 0,
            folders: Vec::new(),
            files: Vec::new(),
        };
        if let Some(k) = key {
            pak.set_key(k)?;
        }
        Ok(pak)
    }

    pub fn key(&self) -> Option<[u8; 16]> {
        self.key
    }

    fn header_key(&self, key: &[u8; 16]) -> [u8; 16] {
        let mut hk = if self.header_flags & 1 != 0 { *key } else { [0; 16] };
        if self.header_flags & 2 != 0 {
            for i in 0..16 {
                hk[i] ^= HEADER_KEY[i];
            }
        }
        hk
    }

    fn header_cipher_src(&self) -> Result<(u64, &[u8])> {
        let o = self.header_iv_offset;
        ensure!(o + 8 <= self.header.len(), "header truncated");
        let iv = u64::from_le_bytes(self.header[o..o + 8].try_into().unwrap());
        let end = (o + 8 + (self.header.len() - o - 8) / 8 * 8).min(self.header.len());
        Ok((iv, &self.header[o + 8..end]))
    }

    pub fn check_key(&self, key: &[u8; 16]) -> bool {
        self.check_header_cipher(&Blowfish::new(&self.header_key(key)))
    }

    pub fn check_header_cipher(&self, bf: &Blowfish) -> bool {
        let Ok((iv, src)) = self.header_cipher_src() else { return false };
        let mut d = DecReader::new(src, Cipher::new(bf, iv));
        let mut b = [0u8; 48];
        if d.read(&mut b).is_err() {
            return false;
        }
        let size = u32::from_le_bytes(b[44..48].try_into().unwrap());
        size as u64 == self.len
    }

    pub fn set_key(&mut self, key: [u8; 16]) -> Result<()> {
        ensure!(self.header_flags & 3 != 0, "unencrypted pack headers are not supported");
        let (iv, src) = self.header_cipher_src()?;
        let hbf = Blowfish::new(&self.header_key(&key));
        let mut d = DecReader::new(src, Cipher::new(&hbf, iv));
        let mut md5 = [0u8; 16];
        d.read(&mut md5)?;
        let gbx_headers_start = d.u32()?;
        let _gbx_headers_size = d.u32()?;
        let _gbx_headers_compr = d.u32()?;
        d.read(&mut [0u8; 16])?;
        let file_size = d.u32()?;
        d.read(&mut [0u8; 16])?;
        let real = self.len;
        if file_size as u64 != real {
            bail!("{}: wrong key (header says {file_size} bytes, file has {real})", self.path.display());
        }
        let flags = d.u32()?;
        let nfolders = d.u32()? as usize;
        ensure!(nfolders < 1 << 20, "bad folder count");
        let mut folders = Vec::with_capacity(nfolders);
        for _ in 0..nfolders {
            let parent = d.u32()? as i32;
            let mut name = d.string()?;
            if !name.ends_with('\\') && !name.ends_with('/') {
                name.push('\\');
            }
            folders.push(Folder { parent: (parent >= 0).then_some(parent as usize), name });
        }
        if folders.len() > 2 && folders[2].name.chars().count() > 4 {
            let utf16: Vec<u8> = folders[2].name.encode_utf16().flat_map(|c| c.to_le_bytes()).collect();
            d.cipher().dummy_write(&utf16[4..8]);
        }
        let nfiles = d.u32()? as usize;
        ensure!(nfiles < 1 << 22, "bad file count");
        let mut files = Vec::with_capacity(nfiles);
        for _ in 0..nfiles {
            let folder = d.u32()? as i32;
            let name = d.string()?;
            let unknown = d.u32()?;
            let uncompressed_size = d.u32()?;
            let compressed_size = d.u32()?;
            let offset = d.u32()?;
            let class_id = d.u32()?;
            d.u32()?;
            d.read(&mut [0u8; 16])?;
            let flags = d.u64()?;
            files.push(PakFile {
                folder: (folder >= 0).then_some(folder as usize),
                name,
                unknown,
                uncompressed_size,
                compressed_size,
                offset,
                class_id,
                flags,
            });
        }
        self.gbx_headers_start = gbx_headers_start;
        self.file_size = file_size;
        self.flags = flags;
        self.folders = folders;
        self.files = files;
        self.key = Some(key);
        self.bf = Some(Blowfish::new(&key));
        Ok(())
    }

    pub fn folder_path(&self, folder: Option<usize>) -> String {
        let mut parts = Vec::new();
        let mut f = folder;
        while let Some(i) = f {
            parts.push(self.folders[i].name.as_str());
            f = self.folders[i].parent;
            if parts.len() > 64 {
                break;
            }
        }
        parts.reverse();
        parts.concat()
    }

    pub fn file_path(&self, i: usize) -> String {
        let f = &self.files[i];
        self.folder_path(f.folder) + &f.name
    }

}

fn read_up_to(f: &File, buf: &mut [u8], at: u64) -> Result<usize> {
    let mut done = 0;
    while done < buf.len() {
        let n = f.read_at(&mut buf[done..], at + done as u64)?;
        if n == 0 {
            break;
        }
        done += n;
    }
    Ok(done)
}

#[derive(Clone)]
pub struct FileDecoder<'k> {
    src: std::rc::Rc<Vec<u8>>,
    src_pos: usize,
    cipher: Option<Cipher<'k>>,
    block: [u8; 8],
    block_pos: usize,
    pub lz: crate::lz4::Decoder,
    plain: Vec<u8>,
    compressed: bool,
    pub size: usize,
}

impl<'k> FileDecoder<'k> {
    pub fn new(pak: &'k Pak, i: usize) -> Result<Self> {
        let f = &pak.files[i];
        let at = pak.header_max_size as u64 + f.offset as u64;
        let (cipher, data_at) = if f.is_encrypted() {
            let bf = pak.bf.as_ref().context("pack key not set")?;
            let mut iv = [0u8; 8];
            pak.file.read_exact_at(&mut iv, at)?;
            (Some(Cipher::new(bf, u64::from_le_bytes(iv))), at + 8)
        } else {
            (None, at)
        };
        let len = (f.compressed_size as usize).div_ceil(8) * 8 + 8;
        let mut buf = vec![0u8; len];
        let n = read_up_to(&pak.file, &mut buf, data_at)?;
        buf.truncate(n);
        Ok(FileDecoder {
            src: std::rc::Rc::new(buf),
            src_pos: 0,
            cipher,
            block: [0; 8],
            block_pos: 8,
            lz: crate::lz4::Decoder::new(),
            plain: Vec::new(),
            compressed: f.is_compressed(),
            size: f.uncompressed_size as usize,
        })
    }

    fn byte(&mut self) -> Result<u8> {
        match &mut self.cipher {
            None => {
                ensure!(self.src_pos < self.src.len(), "file data truncated");
                self.src_pos += 1;
                Ok(self.src[self.src_pos - 1])
            }
            Some(c) => {
                if self.block_pos == 8 {
                    ensure!(self.src_pos + 8 <= self.src.len(), "file data truncated");
                    let w = u64::from_le_bytes(self.src[self.src_pos..self.src_pos + 8].try_into().unwrap());
                    self.src_pos += 8;
                    self.block = c.next_block(w).to_le_bytes();
                    self.block_pos = 0;
                }
                self.block_pos += 1;
                Ok(self.block[self.block_pos - 1])
            }
        }
    }

    pub fn delivered(&self) -> usize {
        match &self.cipher {
            None => self.src_pos,
            Some(_) => self.src_pos - (8 - self.block_pos),
        }
    }

    pub fn output(&self) -> &[u8] {
        if self.compressed { self.lz.output() } else { &self.plain }
    }

    pub fn done(&self) -> bool {
        self.output().len() >= self.size
    }

    pub fn pull(&mut self, n: usize) -> Result<usize> {
        ensure!(!self.done(), "pull past end of file");
        if self.compressed {
            let len = self.byte()? as usize | (self.byte()? as usize) << 8;
            ensure!(len <= crate::lz4::MAX_BLOCK, "bad lz4 block length {len}");
            let mut b = vec![0u8; len];
            for x in b.iter_mut() {
                *x = self.byte()?;
            }
            let got = self.lz.block(&b)?;
            ensure!(got == crate::lz4::BLOCK_OUT || self.lz.out_len() == self.size, "short lz4 block ({got} bytes)");
            ensure!(self.lz.out_len() <= self.size, "lz4 output past file size");
        } else {
            let n = n.min(self.size - self.plain.len());
            for _ in 0..n {
                let b = self.byte()?;
                self.plain.push(b);
            }
        }
        Ok(self.output().len())
    }

    pub fn dummy_write(&mut self, bytes: &[u8]) {
        if let Some(c) = &mut self.cipher {
            c.dummy_write(bytes);
        }
    }

    pub fn is_encrypted(&self) -> bool {
        self.cipher.is_some()
    }
}
