use std::collections::HashMap;

use anyhow::{Result, bail, ensure};
use gearbox_gbx::reader::Reader;

use crate::packs::PackSet;

pub const CLASS_FLAT: u32 = 0x0305_d000;
pub const CLASS_FRONTIER: u32 = 0x0305_e000;
pub const CLASS_TRANSITION: u32 = 0x0314_d000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Zone {
    Flat,
    Frontier { from: Option<String>, to: Option<String>, height: i32, top: i32 },
    Transition,
}

impl Zone {
    pub fn height(&self) -> i32 {
        match self {
            Zone::Frontier { height, .. } => *height,
            _ => 0,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Zones(HashMap<(String, String), Zone>);

impl Zones {
    pub fn load(set: &PackSet) -> Result<Self> {
        let mut zones = HashMap::new();
        for (r, f) in set.files() {
            if !matches!(f.class_id, CLASS_FLAT | CLASS_FRONTIER | CLASS_TRANSITION) {
                continue;
            }
            let path = set.paks[r.pak].file_path(r.file);
            let Some((env, _)) = path.split_once('\\') else { continue };
            let (name, zone) = read(&set.read(r)?).map_err(|e| e.context(path.clone()))?;
            zones.insert((env.to_ascii_lowercase(), name.to_ascii_lowercase()), zone);
        }
        Ok(Self(zones))
    }

    pub fn get(&self, env: &str, name: &str) -> Option<Zone> {
        if let Some(zone) = self.0.get(&(env.to_ascii_lowercase(), name.to_ascii_lowercase())) {
            return Some(zone.clone());
        }
        let to = name.strip_prefix("VoidTo")?;
        matches!(self.get(env, to)?, Zone::Flat).then(|| Zone::Frontier {
            from: None,
            to: Some(to.to_string()),
            height: 0,
            top: 0,
        })
    }
}

struct Ids {
    versioned: bool,
    names: Vec<String>,
}

impl Ids {
    fn read(&mut self, r: &mut Reader) -> Result<Option<String>> {
        if !self.versioned {
            ensure!(r.u32()? == 3, "unsupported id version");
            self.versioned = true;
        }
        let id = r.u32()?;
        if id == u32::MAX {
            return Ok(None);
        }
        ensure!(id & 0xc000_0000 != 0, "numeric zone id {id}");
        Ok(Some(match (id & 0x3fff_ffff) as usize {
            0 => {
                let name = r.string()?;
                self.names.push(name.clone());
                name
            }
            n => self.names.get(n - 1).cloned().ok_or_else(|| anyhow::anyhow!("zone id {n} not defined"))?,
        }))
    }
}

fn chunk(r: &mut Reader, id: u32) -> Result<()> {
    let at = r.u32()?;
    ensure!(at == id, "expected zone chunk {id:08x}, found {at:08x}");
    Ok(())
}

fn skippable(r: &mut Reader, id: u32) -> Result<()> {
    chunk(r, id)?;
    ensure!(r.u32()? == u32::from_le_bytes(*b"PIKS"), "zone chunk {id:08x} is not skippable");
    let size = r.u32()? as usize;
    r.skip(size)
}

fn read(buf: &[u8]) -> Result<(String, Zone)> {
    let layout = crate::nodes::layout(buf)?;
    let class = layout.head.class_id;
    let mut r = Reader::new(buf);
    r.skip(layout.head.body_start)?;
    let mut ids = Ids { versioned: false, names: Vec::new() };
    chunk(&mut r, 0x0305_c003)?;
    r.u32()?;
    let Some(name) = ids.read(&mut r)? else { bail!("zone without a name") };
    ids.read(&mut r)?;
    r.u32()?;
    skippable(&mut r, 0x0305_c004)?;
    chunk(&mut r, 0x0305_c006)?;
    r.u32()?;
    chunk(&mut r, 0x0305_c007)?;
    r.u32()?;
    chunk(&mut r, 0x0305_c00b)?;
    ids.read(&mut r)?;
    chunk(&mut r, 0x0305_c00c)?;
    ids.read(&mut r)?;
    let zone = match class {
        CLASS_FLAT => Zone::Flat,
        CLASS_FRONTIER => {
            chunk(&mut r, 0x0305_e001)?;
            r.u32()?;
            let from = ids.read(&mut r)?;
            let to = ids.read(&mut r)?;
            chunk(&mut r, 0x0305_e002)?;
            Zone::Frontier { from, to, height: r.i32()?, top: r.i32()? }
        }
        CLASS_TRANSITION => Zone::Transition,
        _ => bail!("class {class:08x} is not a zone"),
    };
    Ok((name, zone))
}

const PATTERNS: [[i32; 9]; 15] = [
    [1, 1, 1, 1, 1, 1, 1, 1, 0],
    [1, 1, 0, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 1, 1, 0, 1, 0],
    [1, 1, 0, 1, 1, 0, 1, 1, 0],
    [0, 1, 0, 1, 1, 0, 1, 1, 0],
    [1, 1, 0, 1, 1, 0, 0, 1, 0],
    [0, 1, 0, 1, 1, 0, 0, 1, 0],
    [0, 0, 0, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 1, 1, 0, 0, 1, 0],
    [0, 1, 0, 0, 1, 0, 0, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 1, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
];

const EDGES: [[i32; 3]; 5] = [[0, 0, 0], [0, 1, 1], [1, 1, 0], [0, 1, 0], [1, 1, 1]];

fn pattern(index: u32, dir: u32) -> [i32; 9] {
    let grid = PATTERNS.get(index as usize).copied().unwrap_or([-1; 9]);
    std::array::from_fn(|i| {
        let (row, col) = (i / 3, i % 3);
        match dir {
            1 => grid[col * 3 + 2 - row],
            2 => grid[8 - i],
            3 => grid[(2 - col) * 3 + row],
            _ => grid[i],
        }
    })
}

fn edge(grid: &[i32; 9], face: usize) -> usize {
    let cells = match face {
        0 => [grid[8], grid[5], grid[2]],
        1 => [grid[2], grid[1], grid[0]],
        2 => [grid[0], grid[3], grid[6]],
        _ => [grid[6], grid[7], grid[8]],
    };
    EDGES.iter().position(|e| *e == cells).unwrap_or(5)
}

#[derive(Clone, Debug, PartialEq)]
pub enum Ground {
    None,
    Level(i32),
    Frontier { grid: [i32; 9], low: Option<i32>, high: i32, side: i32 },
    Unsupported(String),
}

impl Ground {
    pub fn of(zones: &Zones, env: &str, column: &crate::blockinfo::AutoTerrain, offset: i32) -> Ground {
        if column.steps.is_empty() {
            return Ground::None;
        }
        let zone = |name: &Option<String>| {
            name.as_deref().and_then(|name| zones.get(env, name)).ok_or_else(|| format!("unknown zone {name:?}"))
        };
        let steps = match column.steps.iter().map(zone).collect::<Result<Vec<_>, _>>() {
            Ok(steps) => steps,
            Err(e) => return Ground::Unsupported(e),
        };
        let heights: Vec<i32> = steps
            .iter()
            .scan(0, |height, zone| {
                *height += zone.height();
                Some(*height)
            })
            .collect();
        let high = heights[heights.len() - 1] + offset;
        let low = heights.len().checked_sub(2).map(|i| heights[i] + offset);
        let top = match steps.last() {
            Some(Zone::Frontier { top, .. }) => *top,
            _ => 0,
        };
        match zone(&column.target) {
            Ok(Zone::Flat) => Ground::Level(high),
            Ok(Zone::Frontier { .. }) => Ground::Frontier {
                grid: pattern(column.index, column.dir),
                low,
                high,
                side: low.unwrap_or(offset) + top,
            },
            Ok(Zone::Transition) => Ground::Unsupported(format!("transition zone {:?}", column.target)),
            Err(e) => Ground::Unsupported(e),
        }
    }

    pub fn contains(&self, y: i32, face: usize) -> Result<bool> {
        if face >= 4 {
            return Ok(false);
        }
        Ok(match self {
            Ground::None => false,
            Ground::Level(height) => y == *height,
            Ground::Frontier { grid, low, high, side } => {
                y == match edge(grid, face) {
                    4 => *high,
                    0 => low.ok_or_else(|| anyhow::anyhow!("single-step frontier ground"))?,
                    _ => *side,
                }
            }
            Ground::Unsupported(reason) => bail!("ground zone not supported: {reason}"),
        })
    }
}
