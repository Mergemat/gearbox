use anyhow::{Result, bail, ensure};
use serde::Serialize;

use crate::container::Container;
use crate::reader::{NodeRef, Reader};

mod embedded;
pub use embedded::{EmbeddedData, EmbeddedFile, EmbeddedItemModel};
mod items;
mod media;

pub const CLASS_MAP: u32 = 0x0304_3000;
const CLASS_COLLECTOR_LIST: u32 = 0x0301_B000;
const CLASS_PARAMETERS: u32 = 0x0305_B000;
const CLASS_WAYPOINT: u32 = 0x2E00_9000;
const CLASS_BLOCK_SKIN: u32 = 0x0305_9000;

pub const BLOCK_SIZE: [f32; 3] = [32.0, 8.0, 32.0];

#[derive(Debug, Default, Serialize)]
pub struct Map {
    pub uid: Option<String>,
    pub name: Option<String>,
    pub author: Option<String>,
    pub decoration: Option<String>,
    pub collection: Option<String>,
    pub decoration_collection: Option<String>,
    pub player_model: Option<String>,
    pub build_version: Option<String>,
    pub flags: Option<u32>,
    pub size: [i32; 3],
    pub author_time_ms: Option<i32>,
    pub lap_race: bool,
    pub laps: u32,
    pub blocks: Vec<Block>,
    pub baked_blocks: Vec<Block>,
    pub items: Vec<Item>,
    pub embedded: EmbeddedData,
}

#[derive(Debug, Clone, Serialize)]
pub struct Block {
    pub name: String,
    pub dir: u8,
    pub coord: [i32; 3],
    pub flags: u32,
    pub variant: u8,
    pub sub_variant: u8,
    pub waypoint: Option<Waypoint>,
    pub free: Option<([f32; 3], [f32; 3])>,
    pub skin: Option<Vec<String>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Item {
    pub model: Option<String>,
    pub collection: Option<String>,
    pub author: Option<String>,
    pub yaw_pitch_roll: [f32; 3],
    pub block_coord: [u8; 3],
    pub position: [f32; 3],
    pub pivot: [f32; 3],
    pub scale: f32,
    pub flags: u16,
    pub anim_phase: u8,
    pub waypoint: Option<Waypoint>,
    pub placed_on_item: Option<usize>,
    pub snapped_on_block: Option<usize>,
    pub snapped_on_item: Option<usize>,
    pub snapped_on_group: Option<i32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Waypoint {
    pub tag: Option<String>,
    pub order: i32,
}

impl Block {
    pub fn is_ground(&self) -> bool {
        self.flags & (1 << 12) != 0
    }
    pub fn is_ghost(&self) -> bool {
        self.flags & (1 << 28) != 0
    }
    pub fn is_free(&self) -> bool {
        self.flags & (1 << 29) != 0
    }
}

impl Map {
    pub fn parse(file: &[u8]) -> Result<Self> {
        let c = Container::parse(file)?;
        ensure!(c.class_id == CLASS_MAP, "not a map (class 0x{:08X})", c.class_id);
        let mut map = Map::default();
        if let Some(h) = c.header_chunk(0x0304_3003) {
            let mut r = Reader::new(h);
            r.u8()?;
            let [uid, _, author] = r.ident()?;
            map.uid = uid;
            map.author = author;
            map.name = Some(r.string()?);
        }
        let mut r = Reader::new(&c.body);
        r.chunks("CGameCtnChallenge", |r, id, _| {
            match id {
                0x0304_300D => map.player_model = r.ident()?[0].clone(),
                0x0304_3011 => {
                    node(r, CLASS_COLLECTOR_LIST, |r| {
                        r.chunks("CGameCtnCollectorList", |r, id, _| {
                            ensure!(id == 0x0301_B000, "unexpected chunk");
                            for _ in 0..r.len()? {
                                r.ident()?;
                                r.i32()?;
                            }
                            Ok(true)
                        })
                    })?;
                    node(r, CLASS_PARAMETERS, |r| parameters(r, &mut map.author_time_ms))?;
                    r.i32()?;
                }
                0x0304_3018 => {
                    map.lap_race = r.bool32()?;
                    map.laps = r.u32()?;
                }
                0x0304_301F => {
                    let [uid, collection, author] = r.ident()?;
                    let name = r.string()?;
                    map.uid = map.uid.take().or(uid);
                    map.collection = collection;
                    map.author = map.author.take().or(author);
                    map.name = map.name.take().or(Some(name));
                    let [decoration, decoration_collection, _] = r.ident()?;
                    map.decoration = decoration;
                    map.decoration_collection = decoration_collection;
                    map.size = [r.i32()?, r.i32()?, r.i32()?];
                    r.bool32()?;
                    let version = r.u32()?;
                    ensure!(version == 6, "block data version {version}");
                    map.blocks = blocks(r)?;
                }
                0x0304_3022 => {
                    map.flags = Some(r.u32()?);
                }
                0x0304_3024 => {
                    r.pack_desc()?;
                }
                0x0304_3025 => r.skip(16)?,
                0x0304_3026 => media::node(r)?,
                0x0304_3028 => {
                    if r.bool32()? {
                        r.skip(1 + 36 + 12 + 12)?;
                    }
                    r.string()?;
                }
                0x0304_302A => {
                    r.bool32()?;
                }
                0x0304_3040 => map.items = items::read(r, map.blocks.len())?,
                0x0304_3048 => {
                    ensure!(r.u32()? == 0, "baked blocks version");
                    ensure!(r.u32()? == 6, "baked blocks block version");
                    map.baked_blocks = blocks(r)?;
                }
                0x0304_3049 => {
                    let version = r.u32()?;
                    for _ in 0..4 {
                        media::node(r)?;
                    }
                    if version >= 2 {
                        media::node(r)?;
                    }
                    if version >= 1 {
                        r.skip(12)?;
                    }
                }
                0x0304_3051 => {
                    ensure!(r.u32()? == 0, "map build-version chunk version");
                    r.id()?;
                    map.build_version = Some(r.string()?);
                }
                0x0304_3054 => map.embedded = embedded::read(r)?,
                0x0304_305F => {
                    ensure!(r.u32()? == 0, "free blocks version");
                    for b in map.blocks.iter_mut().chain(map.baked_blocks.iter_mut()).filter(|b| b.is_free())
                    {
                        b.free = Some((r.vec3()?, r.vec3()?));
                    }
                }
                0x0304_3063 => {
                    ensure!(r.u32()? == 0, "item animation phase version");
                    for item in &mut map.items {
                        item.anim_phase = r.u8()?;
                    }
                }
                _ => return Ok(false),
            }
            Ok(true)
        })?;
        Ok(map)
    }
}

fn node(r: &mut Reader, class: u32, body: impl FnOnce(&mut Reader) -> Result<()>) -> Result<()> {
    match r.node_ref()? {
        NodeRef::Null | NodeRef::Shared(_) => Ok(()),
        NodeRef::New(got) => {
            ensure!(got == class, "expected node 0x{class:08X}, got 0x{got:08X}");
            body(r)
        }
    }
}

fn parameters(r: &mut Reader, author_time: &mut Option<i32>) -> Result<()> {
    r.chunks("CGameCtnChallengeParameters", |r, id, _| {
        match id {
            0x0305_B000 => r.skip(32)?,
            0x0305_B001 => {
                for _ in 0..4 {
                    r.string()?;
                }
            }
            0x0305_B002 => r.skip(64)?,
            0x0305_B003 => r.skip(24)?,
            0x0305_B004 => {
                r.skip(12)?;
                *author_time = r.opt_i32()?;
                r.u32()?;
            }
            0x0305_B005 => r.skip(12)?,
            0x0305_B006 => {
                let n = r.len()?;
                r.skip(n * 4)?;
            }
            0x0305_B007 => r.skip(4)?,
            0x0305_B008 => r.skip(8)?,
            0x0305_B00A => {
                r.string()?;
                r.skip(12)?;
                *author_time = r.opt_i32()?;
                r.skip(8)?;
            }
            0x0305_B00D => media::node(r)?,
            _ => return Ok(false),
        }
        Ok(true)
    })
}

fn blocks(r: &mut Reader) -> Result<Vec<Block>> {
    let n = r.len()?;
    let mut out = Vec::with_capacity(n);
    let mut skins: std::collections::HashMap<i32, Vec<String>> = std::collections::HashMap::new();
    while out.len() < n {
        let name = r.id()?.unwrap_or_default();
        let dir = r.u8()?;
        let c = [r.u8()?, r.u8()?, r.u8()?];
        let flags = r.u32()?;
        if flags == u32::MAX {
            continue;
        }
        let mut skin = None;
        if flags & (1 << 15) != 0 {
            r.id()?;
            let index = r.peek_u32().unwrap_or(u32::MAX) as i32;
            skin = match r.node_ref()? {
                NodeRef::Null => None,
                NodeRef::Shared(i) => skins.get(&i).cloned(),
                NodeRef::New(class) => {
                    ensure!(
                        class == CLASS_BLOCK_SKIN,
                        "expected node 0x{CLASS_BLOCK_SKIN:08X}, got 0x{class:08X}"
                    );
                    let mut strings = Vec::new();
                    block_skin(r, &mut strings)?;
                    skins.insert(index, strings.clone());
                    Some(strings)
                }
            };
        }
        let mut waypoint = None;
        ensure!(flags & (1 << 19) == 0, "block char-physics property not supported");
        if flags & (1 << 20) != 0 {
            node(r, CLASS_WAYPOINT, |r| {
                waypoint = Some(waypoint_property(r)?);
                Ok(())
            })?;
        }
        if flags & (1 << 18) != 0 {
            for _ in 0..r.len()? {
                r.i32()?;
                r.i32()?;
                for _ in 0..r.len()? {
                    r.ident()?;
                }
            }
        }
        if flags & (1 << 17) != 0 {
            r.id()?;
            r.i32()?;
            r.i32()?;
        }
        out.push(Block {
            name,
            dir,
            coord: [c[0] as i32 - 1, c[1] as i32, c[2] as i32 - 1],
            flags,
            variant: (flags & 0x3F) as u8,
            sub_variant: ((flags >> 6) & 0x3F) as u8,
            waypoint,
            free: None,
            skin,
        });
    }
    Ok(out)
}

fn block_skin(r: &mut Reader, out: &mut Vec<String>) -> Result<()> {
    r.chunks("CGameCtnBlockSkin", |r, id, _| {
        match id {
            0x0305_9000 => {
                out.push(r.string()?);
                out.push(r.string()?);
            }
            0x0305_9001 => {
                out.push(r.string()?);
                out.push(r.pack_desc()?);
            }
            0x0305_9002 => {
                out.push(r.string()?);
                out.push(r.pack_desc()?);
                out.push(r.pack_desc()?);
            }
            0x0305_9003 => {
                r.u32()?;
                out.push(r.pack_desc()?);
            }
            _ => return Ok(false),
        }
        Ok(true)
    })
}

fn waypoint_property(r: &mut Reader) -> Result<Waypoint> {
    let mut w = Waypoint { tag: None, order: 0 };
    r.chunks("CGameWaypointSpecialProperty", |r, id, _| {
        match id {
            0x2E00_9000 => match r.u32()? {
                1 => {
                    let spawn = r.i32()?;
                    w.tag = Some(if spawn != 0 { "Spawn" } else { "Checkpoint" }.into());
                    w.order = r.i32()?;
                }
                2 => {
                    w.tag = Some(r.string()?);
                    w.order = r.i32()?;
                }
                v => bail!("waypoint version {v}"),
            },
            _ => return Ok(false),
        }
        Ok(true)
    })?;
    Ok(w)
}

#[derive(Clone, Debug, Default)]
pub struct MapHeader {
    pub uid: Option<String>,
    pub name: Option<String>,
    pub environment: Option<String>,
    pub author_login: Option<String>,
    pub author_name: Option<String>,
    pub author_time_ms: Option<u32>,
    pub thumbnail: Option<Vec<u8>>,
}

impl MapHeader {
    pub fn read(file: &[u8]) -> Result<Self> {
        let (class, chunks) = Container::header(file)?;
        ensure!(class == CLASS_MAP, "not a map (class 0x{class:08X})");
        let chunk = |id: u32| chunks.iter().find(|c| c.0 == id).map(|c| c.1);
        let mut h = MapHeader::default();
        if let Some(c) = chunk(0x0304_3002) {
            let mut r = Reader::new(c);
            if r.u8()? >= 1 {
                r.u32()?;
                r.skip(12)?;
                h.author_time_ms = Some(r.u32()?).filter(|t| *t != u32::MAX);
            }
        }
        if let Some(c) = chunk(0x0304_3003) {
            let mut r = Reader::new(c);
            r.u8()?;
            let [uid, env, login] = r.ident()?;
            h.uid = uid;
            h.environment = env;
            h.author_login = login;
            h.name = Some(r.string()?);
        }
        if let Some(c) = chunk(0x0304_3008) {
            let mut r = Reader::new(c);
            r.u32()?;
            r.u32()?;
            r.string()?;
            h.author_name = Some(r.string()?).filter(|s| !s.is_empty());
        }
        if let Some(c) = chunk(0x0304_3007) {
            let mut r = Reader::new(c);
            if r.u32()? != 0 {
                let n = r.u32()? as usize;
                r.skip("<Thumbnail.jpg>".len())?;
                if n > 0 {
                    h.thumbnail = Some(r.bytes(n)?.to_vec());
                }
            }
        }
        Ok(h)
    }
}
