use std::collections::{HashMap, HashSet};
use std::sync::OnceLock;

use anyhow::{Context, Result};

use crate::classes::CLASSES;
use gearbox_gbx::classid;
use crate::pak::{FileDecoder, Pak};

struct Registry {
    ids: HashSet<u32>,
    parent: std::collections::HashMap<u32, u32>,
}

fn registry() -> &'static Registry {
    static R: OnceLock<Registry> = OnceLock::new();
    R.get_or_init(|| {
        let mut ids: HashSet<u32> = CLASSES.iter().map(|c| c.0).collect();
        let parent = CLASSES.iter().copied().collect();
        for id in legacy_ids() {
            ids.insert(id);
        }
        Registry { ids, parent }
    })
}

fn legacy_ids() -> Vec<u32> {
    let mut v = Vec::new();
    for engine in 0u32..0x40 {
        for class in 0u32..0x400 {
            let id = engine << 24 | class << 12;
            if classid::remap_class_id(id) != id {
                v.push(id);
            }
        }
    }
    v
}

pub fn is_class(id: u32) -> bool {
    id & 0xfff == 0 && registry().ids.contains(&id)
}

fn dummy_id(id: u32) -> u32 {
    let id = classid::remap_class_id(id);
    match id {
        0x0a003000 => 0x0a001000,
        0x090bf000 => 0x0804b000,
        0x0917e000 | 0x09184000 => 0x05010000,
        0x09185000 => 0x05002000,
        _ => match classid::archive_class_id(registry().parent[&id]) {
            0x07031000 => 0x07001000,
            parent => parent,
        },
    }
}

#[derive(Clone, Debug)]
pub struct GbxHead {
    pub class_id: u32,
    pub num_nodes: u32,
    pub body_start: usize,
    pub refs: HashSet<u32>,
    pub ref_files: HashMap<u32, String>,
    pub ancestor_level: u32,
    pub body_compressed: bool,
}

impl GbxHead {
    pub fn ref_path(&self, own: &str, index: u32) -> Option<String> {
        let rel = self.ref_files.get(&index)?;
        let own = own.replace('/', "\\");
        let mut parts: Vec<&str> = own.split('\\').collect();
        parts.pop();
        for _ in 0..self.ancestor_level {
            parts.pop()?;
        }
        let mut p: String = parts.iter().map(|s| format!("{s}\\")).collect();
        p.push_str(rel);
        Some(p)
    }
}

pub fn parse_head(buf: &[u8]) -> Option<Result<GbxHead>> {
    let mut r = gearbox_gbx::reader::Reader::new(buf);
    macro_rules! t {
        ($e:expr) => {
            match $e {
                Ok(v) => v,
                Err(_) => return None,
            }
        };
    }
    let magic = t!(r.bytes(3));
    if magic != b"GBX" {
        return Some(Err(anyhow::anyhow!("not a GBX file")));
    }
    let version = t!(r.u16());
    if version != 6 {
        return Some(Err(anyhow::anyhow!("GBX version {version}")));
    }
    let fmt = t!(r.bytes(4)).to_vec();
    let class_id = t!(r.u32());
    let user = t!(r.u32()) as usize;
    t!(r.skip(user));
    let num_nodes = t!(r.u32());
    let nrefs = t!(r.u32());
    let mut refs = HashSet::new();
    let mut ref_files = HashMap::new();
    let mut ancestor_level = 0;
    if nrefs > 0 {
        if nrefs > 100_000 {
            return Some(Err(anyhow::anyhow!("bad ref count")));
        }
        ancestor_level = t!(r.u32());
        fn dirs(r: &mut gearbox_gbx::reader::Reader, depth: usize, prefix: &str, out: &mut Vec<String>) -> Option<()> {
            let n = r.u32().ok()?;
            if n > 10_000 || depth > 32 {
                return None;
            }
            for _ in 0..n {
                let name = r.string().ok()?;
                let path = format!("{prefix}{name}\\");
                out.push(path.clone());
                dirs(r, depth + 1, &path, out)?;
            }
            Some(())
        }
        let mut folders = Vec::new();
        dirs(&mut r, 0, "", &mut folders)?;
        for _ in 0..nrefs {
            let flags = t!(r.u32());
            let name = if flags & 4 != 0 {
                t!(r.u32());
                None
            } else {
                Some(t!(r.string()))
            };
            let index = t!(r.u32());
            refs.insert(index);
            t!(r.u32());
            if flags & 4 == 0 {
                let folder = t!(r.u32()) as usize;
                let dir = folder.checked_sub(1).and_then(|i| folders.get(i)).cloned().unwrap_or_default();
                if let Some(n) = name {
                    ref_files.insert(index, format!("{dir}{n}"));
                }
            }
        }
    }
    Some(Ok(GbxHead { class_id, num_nodes, body_start: r.pos(), refs, ref_files, ancestor_level, body_compressed: fmt[2] == b'C' }))
}

#[derive(Clone)]
pub struct NodeScanner {
    pub head: Option<GbxHead>,
    seen: HashSet<u32>,
    pos: usize,
    ids: Vec<String>,
    pending: Vec<NodeEvent>,
    not_gbx: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NodeEvent {
    pub end: usize,
    pub class_id: u32,
    pub main: bool,
    pub raw: Option<[u8; 4]>,
}

impl NodeEvent {
    pub fn bytes(&self) -> [u8; 4] {
        self.raw.unwrap_or_else(|| dummy_id(self.class_id).to_le_bytes())
    }
}

impl Default for NodeScanner {
    fn default() -> Self {
        Self::new()
    }
}

fn u32_at(b: &[u8], p: usize) -> Option<u32> {
    b.get(p..p + 4).map(|s| u32::from_le_bytes(s.try_into().unwrap()))
}

fn new_id_at(b: &[u8], p: usize) -> Option<(String, usize)> {
    let flag = u32_at(b, p)?;
    if flag != 0x4000_0000 && flag != 0x8000_0000 {
        return None;
    }
    let len = u32_at(b, p + 4)? as usize;
    if len == 0 || len > 256 {
        return None;
    }
    let t = b.get(p + 8..p + 8 + len)?;
    if !t.iter().all(|&c| c >= 0x20 || c == b'\n' || c == b'\r' || c == b'\t') {
        return None;
    }
    Some((String::from_utf8_lossy(t).into_owned(), p + 8 + len))
}

enum Parsed<T> {
    No,
    NeedMore,
    Yes(T),
}

impl NodeScanner {
    pub fn new() -> Self {
        NodeScanner { head: None, seen: HashSet::new(), pos: 0, ids: Vec::new(), pending: Vec::new(), not_gbx: false }
    }

    fn tuning_chunk(&self, b: &[u8], p: usize) -> Parsed<([u8; 4], usize)> {
        let mut q = p + 4;
        let Some(mut v) = u32_at(b, q) else { return Parsed::NeedMore };
        if v == 3 {
            q += 4;
            match u32_at(b, q) {
                Some(x) => v = x,
                None => return Parsed::NeedMore,
            }
        }
        let name = if v & 0xc000_0000 != 0 && v & 0x3fff_ffff == 0 {
            match new_id_at(b, q) {
                Some((t, e)) => {
                    q = e;
                    t
                }
                None => return if b.len() < q + 264 { Parsed::NeedMore } else { Parsed::No },
            }
        } else if v & 0xc000_0000 != 0 && v != 0xffff_ffff {
            let i = (v & 0x3fff_ffff) as usize;
            q += 4;
            match self.ids.get(i - 1) {
                Some(t) => t.clone(),
                None => return Parsed::No,
            }
        } else {
            return Parsed::No;
        };
        if name.len() < 4 {
            return Parsed::No;
        }
        if u32_at(b, p) == Some(0x090c_9000) {
            return Parsed::Yes((name.as_bytes()[..4].try_into().unwrap(), q));
        }
        let Some(idx) = u32_at(b, q) else { return Parsed::NeedMore };
        q += 4;
        if idx != 0xffff_ffff && !self.seen.contains(&idx) && !self.head.as_ref().unwrap().refs.contains(&idx) {
            let Some(end) = b[q..].windows(4).position(|w| w == 0xfacade01u32.to_le_bytes()) else {
                return Parsed::NeedMore;
            };
            q += end + 4;
        }
        q += 4;
        if q > b.len() {
            return Parsed::NeedMore;
        }
        Parsed::Yes((name.as_bytes()[..4].try_into().unwrap(), q))
    }

    pub fn scan(&mut self, out: &[u8]) -> Result<Vec<NodeEvent>> {
        let mut ev = Vec::new();
        if self.not_gbx {
            return Ok(ev);
        }
        if self.head.is_none() {
            match parse_head(out) {
                None => return Ok(ev),
                Some(Err(_)) if !out.starts_with(b"GBX") => {
                    self.not_gbx = true;
                    return Ok(ev);
                }
                Some(Err(e)) => return Err(e),
                Some(Ok(h)) => {
                    if !h.body_compressed {
                        let Some(first) = u32_at(out, h.body_start) else { return Ok(ev) };
                        if is_class(first & !0xfff) {
                            ev.push(NodeEvent { end: h.body_start, class_id: h.class_id, main: true, raw: None });
                        }
                    }
                    self.pos = h.body_start;
                    self.head = Some(h);
                }
            }
        }
        if self.head.as_ref().unwrap().body_compressed {
            return Ok(ev);
        }
        let num_nodes = self.head.as_ref().unwrap().num_nodes;
        while self.pos + 8 <= out.len() {
            let p = self.pos;
            if let Some(i) = self.pending.iter().position(|e| e.end <= p) {
                ev.push(self.pending.remove(i));
                continue;
            }
            let w = u32_at(out, p).unwrap();
            if w == 0x0906_a001 && u32_at(out, p + 4) == Some(1) {
                ev.push(NodeEvent { end: p + 8, class_id: 0x0905_7000, main: false, raw: None });
            }
            if w == 0x0a01_1005 {
                let end = match u32_at(out, p + 4) {
                    Some(1) => Some(p + 8),
                    Some(c) if c & !0xfff == 0x0600_3000 => Some(p + 4),
                    _ => None,
                };
                if let Some(end) = end {
                    ev.push(NodeEvent { end, class_id: 0x0600_3000, main: false, raw: None });
                }
            }
            if w == 0x090e_b000 || w == 0x090c_9000 {
                match self.tuning_chunk(out, p) {
                    Parsed::NeedMore => break,
                    Parsed::Yes((name, end)) => {
                        self.pending.push(NodeEvent { end, class_id: 0, main: false, raw: Some(name) })
                    }
                    Parsed::No => {}
                }
            }
            if (w == 0x4000_0000 || w == 0x8000_0000) && let Some((t, _)) = new_id_at(out, p) {
                self.ids.push(t);
            }
            if w != 0 && w < num_nodes && !self.seen.contains(&w) && !self.head.as_ref().unwrap().refs.contains(&w) {
                let cid = u32_at(out, p + 4).unwrap();
                if is_class(cid) {
                    let chunked = u32_at(out, p + 8).is_none_or(|c| is_class(c & !0xfff));
                    if chunked {
                        ev.push(NodeEvent { end: p + 8, class_id: cid, main: false, raw: None });
                        self.seen.insert(w);
                        self.pos = p + 8;
                        continue;
                    }
                }
            }
            self.pos += 1;
        }
        while let Some(i) = self.pending.iter().position(|e| e.end <= out.len() && e.end <= self.pos + 8) {
            ev.push(self.pending.remove(i));
        }
        Ok(ev)
    }
}

pub fn unpack(pak: &Pak, i: usize) -> Result<Vec<u8>> {
    let f = &pak.files[i];
    let mut d = FileDecoder::new(pak, i)?;
    if !d.is_encrypted() || f.dont_use_dummy_write() {
        while !d.done() {
            d.pull(1 << 20)?;
        }
        return Ok(d.output().to_vec());
    }
    let compressed = f.is_compressed();
    let mut s = NodeScanner::new();
    while !d.done() {
        let n = if compressed { 0 } else { 256 - d.delivered() % 256 };
        d.pull(n).with_context(|| format!("file {i} ({})", pak.file_path(i)))?;
        for e in s.scan(d.output())? {
            d.dummy_write(&e.bytes());
        }
    }
    Ok(d.output().to_vec())
}
