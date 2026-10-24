use anyhow::{Context, Result, bail, ensure};
use serde::Serialize;

use crate::container::Container;
use crate::inputs::PlayerInputs;
use crate::reader::{NodeRef, Reader};
use crate::record::{self, RecordData};

pub const CLASS_REPLAY: u32 = 0x0309_3000;
pub const CLASS_GHOST: u32 = 0x0309_2000;
pub const CLASS_ENT_RECORD_DATA: u32 = 0x0911_F000;

#[derive(Debug, Serialize)]
pub struct Replay {
    pub map_uid: Option<String>,
    pub author_login: Option<String>,
    pub author_nickname: Option<String>,
    #[serde(skip)]
    pub map_gbx: Option<Vec<u8>>,
    pub ghosts: Vec<Ghost>,
}

#[derive(Debug, Default, Serialize)]
pub struct Ghost {
    pub nickname: Option<String>,
    pub login: Option<String>,
    pub player_model: Option<String>,
    pub race_time_ms: Option<i32>,
    pub respawns: Option<i32>,
    pub checkpoints_ms: Vec<Option<i32>>,
    pub validate_map_uid: Option<String>,
    pub exe_version: Option<String>,
    pub exe_checksum: Option<u32>,
    pub validation_seed: Option<i32>,
    pub race_settings: Option<String>,
    pub inputs: Vec<PlayerInputs>,
    #[serde(skip)]
    pub record: Option<RecordData>,
    pub record_entity_indices: Vec<u32>,
}

impl Ghost {
    pub fn record_entities(&self) -> impl Iterator<Item = &crate::record::Entity> {
        self.record.iter().flat_map(move |record| {
            let count = if self.record_entity_indices.is_empty() {
                record.entities.len()
            } else {
                self.record_entity_indices.len()
            };
            (0..count).filter_map(move |index| {
                let index = self.record_entity_indices.get(index).map_or(index, |&index| index as usize);
                record.entities.get(index)
            })
        })
    }
}

impl Replay {
    pub fn parse(file: &[u8]) -> Result<Self> {
        let c = Container::parse(file)?;
        let mut replay = Replay {
            map_uid: None,
            author_login: None,
            author_nickname: None,
            map_gbx: None,
            ghosts: vec![],
        };
        if c.class_id == CLASS_GHOST {
            let g = ghost(&mut Reader::new(&c.body)).context("ghost")?;
            replay.map_uid = g.validate_map_uid.clone();
            replay.ghosts.push(g);
            return Ok(replay);
        }
        ensure!(c.class_id == CLASS_REPLAY, "not a replay or ghost (class 0x{:08X})", c.class_id);
        if let Some(h) = c.header_chunk(0x0309_3000) {
            replay.map_uid = parse_header_basic(h).ok().flatten();
        }

        let mut r = Reader::new(&c.body);
        r.chunks("CGameCtnReplayRecord", |r, id, _| {
            match id {
                0x0309_3002 => replay.map_gbx = Some(r.data()?.to_vec()),
                0x0309_3014 => {
                    ensure!(r.u32()? == 10, "unexpected list version");
                    let n = r.len()?;
                    for _ in 0..n {
                        if let Some(Node::Ghost(g)) = read_node(r)? {
                            replay.ghosts.push(*g);
                        }
                    }
                    r.i32()?;
                    let extra = r.len()?;
                    r.skip(extra * 8)?;
                }
                0x0309_3015 => crate::map::media::node(r)?,
                0x0309_3018 => {
                    r.id()?;
                    r.i32()?;
                    replay.author_login = Some(r.string()?);
                    replay.author_nickname = Some(r.string()?);
                }
                0x0309_3024 => {
                    r.u32()?;
                    ensure!(read_node(r)?.is_none(), "unexpected data tape node");
                    read_node(r)?;
                }
                _ => return Ok(false),
            }
            Ok(true)
        })?;
        Ok(replay)
    }
}

fn parse_header_basic(h: &[u8]) -> Result<Option<String>> {
    let mut r = Reader::new(h);
    let version = r.u32()?;
    ensure!(version >= 2);
    Ok(r.ident()?[0].clone())
}

enum Node {
    Ghost(Box<Ghost>),
    Record(RecordData),
}

fn read_node(r: &mut Reader) -> Result<Option<Node>> {
    let class = match r.node_ref()? {
        NodeRef::Null => return Ok(None),
        NodeRef::Shared(index) => bail!("shared node reference #{index} not supported"),
        NodeRef::New(class) => class,
    };
    Ok(Some(match class {
        CLASS_GHOST => Node::Ghost(Box::new(ghost(r).context("ghost")?)),
        CLASS_ENT_RECORD_DATA => Node::Record(record::parse_node_chunks(r)?),
        _ => bail!("unsupported node class 0x{class:08X}"),
    }))
}

pub(crate) fn ghost(r: &mut Reader) -> Result<Ghost> {
    let mut g = Ghost::default();
    r.chunks("CGameCtnGhost", |r, id, _| {
        match id {
            0x0309_2000 => {
                let version = r.u32()?;
                let appearance = if version >= 9 { r.i32()? } else { 0 };
                g.player_model = r.ident()?[0].clone();
                r.vec3()?;
                let skins = r.len()?;
                for _ in 0..skins {
                    r.pack_desc()?;
                }
                if r.bool32()? {
                    skip_badge(r)?;
                }
                if appearance >= 1 {
                    r.string()?;
                }
                g.nickname = Some(r.string()?);
                r.string()?;
                if version >= 2 {
                    r.string()?;
                }
                if version >= 4 {
                    r.i32()?;
                }
                if version >= 5 {
                    if let Some(Node::Record(rec)) = read_node(r)? {
                        g.record = Some(rec);
                    }
                    let n = r.len()?;
                    g.record_entity_indices = (0..n).map(|_| r.u32()).collect::<Result<_>>()?;
                }
                return Ok(true);
            }
            0x0309_2005 => g.race_time_ms = r.opt_i32()?,
            0x0309_2008 => g.respawns = r.opt_i32()?,
            0x0309_200A => {
                r.i32()?;
            }
            0x0309_200B => {
                let n = r.len()?;
                for _ in 0..n {
                    g.checkpoints_ms.push(r.opt_i32()?);
                    r.i32()?;
                }
            }
            0x0309_200C => {
                r.i32()?;
            }
            0x0309_200E => {
                r.id()?;
            }
            0x0309_200F => g.login = Some(r.string()?),
            0x0309_2010 => g.validate_map_uid = r.id()?,
            0x0309_201C => r.skip(32)?,
            0x0309_201D => {
                let version = r.u32()?;
                let n = r.len()?;
                for _ in 0..n {
                    let input_version = r.i32()?;
                    r.i32()?;
                    let start_offset_ms = if version >= 4 { r.opt_i32()? } else { None };
                    let ticks = r.len()?;
                    let data = r.data()?.to_vec();
                    g.inputs.push(PlayerInputs { version: input_version, start_offset_ms, ticks, data });
                }
            }
            0x0309_2025 => {
                if r.u32()? == 0 {
                    return Ok(false);
                }
                let _events_duration = r.i32()?;
                r.i32()?;
                let names = r.len()?;
                for _ in 0..names {
                    r.id()?;
                }
                let events = r.len()?;
                r.i32()?;
                r.skip(events * 9)?;
                g.exe_version = Some(r.string()?);
                g.exe_checksum = Some(r.u32()?);
                r.i32()?;
                r.i32()?;
                g.race_settings = Some(r.string()?);
                g.validation_seed = Some(r.i32()?);
                r.bool32()?;
            }
            0x0303_F006 => {
                r.bool32()?;
                r.len()?;
                r.data()?;
            }
            _ => return Ok(false),
        }
        Ok(true)
    })?;
    Ok(g)
}

pub(crate) fn skip_badge(r: &mut Reader) -> Result<()> {
    let version = r.i32()?;
    r.vec3()?;
    if version == 0 {
        r.i32()?;
        r.string()?;
    }
    let stickers = r.len()?;
    for _ in 0..stickers {
        r.string()?;
        r.string()?;
    }
    let layers = r.len()?;
    for _ in 0..layers {
        r.string()?;
    }
    Ok(())
}
