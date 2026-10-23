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
    Transition(Transition),
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Border {
    kind: u32,
    steps: Vec<Option<String>>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Transition {
    mode: u32,
    sub_zones: [Option<String>; 3],
    borders: [Border; 4],
}

impl Transition {
    fn borders(&self, zones: &Zones, env: &str, index: u32) -> Result<[Border; 4]> {
        if self.mode == 0 {
            return Ok(self.borders.clone());
        }
        let shapes = match self.mode {
            1 => &[
                [(3, 4), (0, 1), (2, 1), (1, 2)],
                [(3, 4), (1, 1), (2, 2), (0, 2)],
                [(0, 4), (0, 2), (1, 1), (2, 2)],
                [(0, 4), (2, 1), (1, 2), (0, 1)],
                [(1, 4), (1, 2), (0, 1), (2, 1)],
                [(1, 4), (2, 2), (0, 2), (1, 1)],
                [(0, 1), (0, 2), (1, 1), (1, 2)],
                [(2, 1), (2, 2), (0, 2), (0, 1)],
                [(1, 2), (1, 1), (2, 2), (2, 1)],
            ][..],
            2 => &[
                [(2, 3), (0, 1), (3, 3), (0, 2)],
                [(2, 1), (1, 1), (3, 2), (0, 2)],
                [(2, 2), (0, 1), (3, 1), (1, 2)],
            ][..],
            mode => bail!("unknown transition mode {mode}"),
        };
        let shape = if self.mode == 2 { index.checked_sub(9) } else { Some(index) }
            .and_then(|i| shapes.get(i as usize))
            .ok_or_else(|| anyhow::anyhow!("invalid transition index {index}"))?;
        let replacement = &self.sub_zones[usize::from(self.mode == 1)];
        let Zone::Frontier { from: Some(from), .. } = zones.resolve(env, replacement)? else {
            bail!("transition replacement is not a frontier");
        };
        Ok(std::array::from_fn(|i| {
            let (path, kind) = shape[i];
            let mut steps = vec![Some(format!("VoidTo{from}"))];
            let ids: &[usize] = match (self.mode, path) {
                (_, 0) => &[0],
                (1, 1) => &[1],
                (1, 2) | (2, 3) => &[0, 2],
                (2, 1) => &[2, 1],
                (2, 2) => &[2],
                _ => &[],
            };
            steps.extend(ids.iter().map(|&i| self.sub_zones[i].clone()));
            Border { kind, steps }
        }))
    }
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
pub struct Zones {
    zones: HashMap<(String, String), Zone>,
    blocks: HashMap<(String, String), String>,
    replacements: HashMap<String, Vec<(String, String)>>,
}

impl Zones {
    pub fn load(set: &PackSet) -> Result<Self> {
        let mut out = Self::default();
        for (r, f) in set.files() {
            if !matches!(f.class_id, CLASS_FLAT | CLASS_FRONTIER | CLASS_TRANSITION | 0x03033000) {
                continue;
            }
            let path = set.paks[r.pak].file_path(r.file);
            let Some((env, _)) = path.split_once('\\') else { continue };
            let buf = set.read(r)?;
            if f.class_id == 0x03033000 {
                out.replacements
                    .insert(crate::assets::block_name(&path).to_ascii_lowercase(), replacements(&buf)?);
                continue;
            }
            let (name, zone) = read(&buf).map_err(|e| e.context(path.clone()))?;
            let head = crate::unpack::parse_head(&buf).ok_or_else(|| anyhow::anyhow!("truncated zone {path}"))??;
            let block_class = match f.class_id {
                CLASS_FLAT => "gamectnblockinfoflat",
                CLASS_FRONTIER => "gamectnblockinfofrontier",
                _ => "gamectnblockinfotransition",
            };
            let block = head
                .ref_files
                .keys()
                .filter_map(|&id| head.ref_path(&path, id))
                .find(|path| path.to_ascii_lowercase().contains(block_class))
                .ok_or_else(|| anyhow::anyhow!("zone without block info: {path}"))?;
            out.blocks.insert(
                (env.to_ascii_lowercase(), crate::assets::block_name(&block).to_ascii_lowercase()),
                name.clone(),
            );
            out.zones.insert((env.to_ascii_lowercase(), name.to_ascii_lowercase()), zone);
        }
        Ok(out)
    }

    pub fn get(&self, env: &str, name: &str) -> Option<Zone> {
        if let Some(zone) = self.zones.get(&(env.to_ascii_lowercase(), name.to_ascii_lowercase())) {
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

    fn resolve(&self, env: &str, name: &Option<String>) -> Result<Zone> {
        name.as_deref()
            .and_then(|name| self.get(env, name))
            .ok_or_else(|| anyhow::anyhow!("unknown zone {name:?}"))
    }

    fn block_zone(&self, env: &str, block: &str) -> Result<(&str, Zone)> {
        let name = self
            .blocks
            .get(&(env.to_ascii_lowercase(), block.to_ascii_lowercase()))
            .ok_or_else(|| anyhow::anyhow!("terrain block without zone: {env}/{block}"))?;
        let zone = self.get(env, name).ok_or_else(|| anyhow::anyhow!("unknown terrain zone {name}"))?;
        Ok((name, zone))
    }

    pub fn terrain_matches(
        &self,
        env: &str,
        column: &crate::blockinfo::AutoTerrain,
        rotation: u32,
        block: &str,
        index: u32,
        direction: u32,
    ) -> Result<bool> {
        let (name, zone) = self.block_zone(env, block)?;
        let target =
            column.target.as_deref().ok_or_else(|| anyhow::anyhow!("terrain column without target"))?;
        if name.eq_ignore_ascii_case(target) {
            if matches!(zone, Zone::Flat) {
                return Ok(true);
            }
            if index == column.index {
                let rotated = (column.dir + rotation) & 3;
                if direction == rotated
                    || matches!(zone, Zone::Frontier { .. })
                        && (matches!(index, 4 | 13)
                            || matches!(index, 2 | 11) && direction == (rotated + 2) & 3)
                {
                    return Ok(true);
                }
            }
        }
        Ok(self.replacements.get(&env.to_ascii_lowercase()).is_some_and(|pairs| {
            pairs.iter().any(|(base, replacement)| {
                base.eq_ignore_ascii_case(target) && replacement.eq_ignore_ascii_case(name)
            })
        }))
    }

    fn transition_heights(
        &self,
        env: &str,
        transition: &Transition,
        index: u32,
        depth: usize,
    ) -> Result<[i32; 4]> {
        ensure!(depth <= self.zones.len(), "cyclic transition genealogy");
        let borders = transition.borders(self, env, index)?;
        let mut heights = [0; 4];
        for (i, border) in borders.iter().enumerate() {
            let steps =
                border.steps.iter().map(|name| self.resolve(env, name)).collect::<Result<Vec<_>>>()?;
            let last = steps.last().ok_or_else(|| anyhow::anyhow!("empty transition border"))?;
            let target = if border.kind == 4 {
                let Zone::Frontier { to, .. } = last else { bail!("transition child is not a frontier") };
                self.resolve(env, to)?
            } else {
                last.clone()
            };
            let high: i32 = steps.iter().map(Zone::height).sum();
            let low = high - last.height();
            heights[i] = match target {
                Zone::Flat => high,
                Zone::Frontier { .. } => {
                    low + match last {
                        Zone::Frontier { top, .. } => *top,
                        _ => 0,
                    }
                }
                Zone::Transition(ref transition) => {
                    low + self.transition_heights(env, transition, 0, depth + 1)?.into_iter().min().unwrap()
                }
            };
        }
        Ok(heights)
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

fn replacements(buf: &[u8]) -> Result<Vec<(String, String)>> {
    let head = crate::unpack::parse_head(buf).ok_or_else(|| anyhow::anyhow!("truncated collection"))??;
    let Some(at) = buf[head.body_start..].windows(4).position(|w| w == 0x0303301du32.to_le_bytes()) else {
        return Ok(Vec::new());
    };
    let ids = crate::blockinfo::DefinedIds::scan(buf, head.body_start);
    let mut r = Reader::new(buf);
    r.skip(head.body_start + at + 4)?;
    let count = r.u32()?;
    ensure!(count < 65536, "zone replacement count {count}");
    (0..count)
        .map(|_| {
            let base = ids.read(&mut r)?.ok_or_else(|| anyhow::anyhow!("zone replacement without base"))?;
            let replacement =
                ids.read(&mut r)?.ok_or_else(|| anyhow::anyhow!("zone replacement without target"))?;
            Ok((base, replacement))
        })
        .collect()
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
        CLASS_TRANSITION => Zone::Transition(read_transition(&mut r, &mut ids)?),
        _ => bail!("class {class:08x} is not a zone"),
    };
    Ok((name, zone))
}

fn read_transition(r: &mut Reader, ids: &mut Ids) -> Result<Transition> {
    let mut transition = Transition::default();
    loop {
        match r.u32()? {
            0xfaca_de01 => return Ok(transition),
            0x0314_d000 => {
                r.u32()?;
            }
            id @ (0x0314_d004 | 0x0314_d008) => {
                if id == 0x0314_d008 {
                    r.u32()?;
                }
                for border in &mut transition.borders {
                    border.kind = r.u32()?;
                    if id == 0x0314_d004 {
                        ids.read(r)?;
                    }
                    chunk(r, 0x0311_d002)?;
                    let count = r.u32()?;
                    ensure!(count < 256, "transition genealogy length {count}");
                    border.steps = (0..count).map(|_| ids.read(r)).collect::<Result<_>>()?;
                    r.u32()?;
                    r.u32()?;
                    ids.read(r)?;
                    chunk(r, 0xfaca_de01)?;
                }
            }
            0x0314_d005 => {
                r.u32()?;
                ids.read(r)?;
            }
            0x0314_d006 => {
                r.u32()?;
                transition.mode = r.u32()?;
            }
            0x0314_d007 => {
                r.u32()?;
                for name in &mut transition.sub_zones {
                    *name = ids.read(r)?;
                }
            }
            id => bail!("unsupported transition chunk {id:08x}"),
        }
    }
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
    Transition([i32; 4]),
    Unsupported(String),
}

impl Ground {
    pub fn of(zones: &Zones, env: &str, column: &crate::blockinfo::AutoTerrain, offset: i32) -> Ground {
        if column.steps.is_empty() {
            return Ground::None;
        }
        let steps = match column.steps.iter().map(|name| zones.resolve(env, name)).collect::<Result<Vec<_>>>()
        {
            Ok(steps) => steps,
            Err(e) => return Ground::Unsupported(format!("{e:#}")),
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
        match zones.resolve(env, &column.target) {
            Ok(Zone::Flat) => Ground::Level(high),
            Ok(Zone::Frontier { .. }) => Ground::Frontier {
                grid: pattern(column.index, column.dir),
                low,
                high,
                side: low.unwrap_or(offset) + top,
            },
            Ok(Zone::Transition(transition)) => {
                match zones.transition_heights(env, &transition, column.index, 0) {
                    Ok(heights) => Ground::Transition(std::array::from_fn(|face| {
                        low.unwrap_or(offset) + heights[(face as u32).wrapping_sub(column.dir) as usize & 3]
                    })),
                    Err(e) => Ground::Unsupported(format!("{e:#}")),
                }
            }
            Err(e) => Ground::Unsupported(format!("{e:#}")),
        }
    }

    pub fn contains(&self, y: i32, face: usize) -> Result<bool> {
        if face >= 4 {
            return Ok(false);
        }
        Ok(match self {
            Ground::None => false,
            Ground::Level(height) => y == *height,
            Ground::Transition(heights) => y == heights[face],
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
