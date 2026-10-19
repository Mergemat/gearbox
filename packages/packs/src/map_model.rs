use crate::{nodes, prefab::Ctx, unpack};
use anyhow::{Context, Result, ensure};
use gearbox_gbx::reader::Reader;

#[derive(Clone, Debug)]
pub struct WaterVolume {
    pub boxes: Vec<[i32; 6]>,
    pub position: [f32; 3],
    pub yaw: f32,
    pub cell: [f32; 3],
}

pub fn water_volumes(buf: &[u8], path: &str, ground: bool, index: usize) -> Result<Vec<WaterVolume>> {
    let l = nodes::layout(buf)?;
    let info = crate::blockinfo::read_block_info(buf, &l, path)?;
    let selected = info
        .variants
        .iter()
        .position(|v| (v.kind == crate::blockinfo::VariantKind::Ground) == ground && v.index == index)
        .context("water variant absent")?;
    let nested = crate::blockinfo::nested_blocks(buf, &l);
    let own = |p: &usize| !nested.iter().any(|&(s, e)| s <= *p && *p < e);
    let starts: Vec<_> = buf[l.head.body_start..]
        .windows(4)
        .enumerate()
        .filter(|(_, w)| *w == 0x0315b002u32.to_le_bytes())
        .map(|(i, _)| l.head.body_start + i)
        .filter(own)
        .collect();
    ensure!(
        starts.len() == info.variants.len(),
        "water variant boundaries {} != {}",
        starts.len(),
        info.variants.len()
    );
    let begin = starts[selected];
    let end = starts.get(selected + 1).copied().unwrap_or(buf.len());
    let chunks: Vec<_> = buf[begin..end]
        .windows(4)
        .enumerate()
        .filter(|(_, w)| *w == 0x0315b00bu32.to_le_bytes())
        .map(|(i, _)| begin + i)
        .filter(own)
        .collect();
    ensure!(chunks.len() <= 1, "multiple water chunks in a variant");
    let Some(&at) = chunks.first() else {
        return Ok(Vec::new());
    };
    let mut r = Reader::new(buf);
    r.skip(at + 4)?;
    let version = r.u32()?;
    ensure!(version == 1, "water volume archive version {version}");
    let n = r.u32()?;
    ensure!(n <= 4096, "water volume count {n}");
    let mut out = Vec::new();
    for _ in 0..n {
        let count = r.u32()?;
        ensure!(count <= 1 << 20, "water box count {count}");
        let mut boxes = Vec::new();
        for _ in 0..count {
            boxes.push([r.i32()?, r.i32()?, r.i32()?, r.i32()?, r.i32()?, r.i32()?]);
        }
        let position = r.vec3()?;
        let yaw = r.f32()?;
        let cell = r.vec3()?;
        let mut id = r.u32()?;
        if id == 3 {
            id = r.u32()?;
        }
        if id != u32::MAX && id & 0xc000_0000 != 0 && id & 0x3fff_ffff == 0 {
            r.string()?;
        }
        out.push(WaterVolume { boxes, position, yaw, cell });
    }
    Ok(out)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Animation {
    pub translation: Vec<(u8, u8, u32)>,
    pub rotation: Vec<(u8, u8, u32)>,
    pub trans_axis: u8,
    pub trans_range: [f32; 2],
    pub rot_axis: u8,
    pub rot_range: [f32; 2],
}

pub fn animation(buf: &[u8]) -> Result<Animation> {
    let h = unpack::parse_head(buf).context("animation header")??;
    ensure!(h.class_id == 0x2f0ca000, "animation class {:x}", h.class_id);
    let mut r = Reader::new(buf);
    r.skip(h.body_start)?;
    animation_node(&mut r)
}

pub(crate) fn animation_node(r: &mut Reader) -> Result<Animation> {
    ensure!(r.u32()? == 0, "unsupported legacy kinematic archive");
    ensure!(r.u32()? == 3, "unsupported kinematic archive subversion");
    fn curve(r: &mut Reader) -> Result<Vec<(u8, u8, u32)>> {
        ensure!(r.u32()? == 1, "unsupported normalised animation curve");
        let n = r.u32()?;
        ensure!(n <= 4, "animation curve has {n} segments");
        (0..n).map(|_| Ok((r.u8()?, r.u8()?, r.u32()?))).collect()
    }
    let translation = curve(r)?;
    let rotation = curve(r)?;
    let shader = r.u32()?;
    ensure!(shader <= 1, "shader animation kind {shader}");
    ensure!(r.u32()? == 0, "unsupported shader curve archive version");
    let n = r.u32()?;
    ensure!(n <= 12, "shader curve has {n} segments");
    r.skip(n as usize * 8)?;
    if shader == 1 {
        r.skip(16)?;
    }
    let trans_axis = r.u8()?;
    let trans_range = [r.f32()?, r.f32()?];
    let rot_axis = r.u8()?;
    let rot_range = [r.f32()?, r.f32()?];
    ensure!(trans_axis < 3 && rot_axis < 3, "invalid animation axis");
    Ok(Animation { translation, rotation, trans_axis, trans_range, rot_axis, rot_range })
}

#[derive(Clone, Debug, PartialEq)]
pub struct Dyna {
    pub dynamize_on_spawn: bool,
    pub mass: f32,
    pub animation: Option<String>,
    pub physical_animation: bool,
}
pub fn dyna(buf: &[u8], path: &str) -> Result<crate::prefab::DynaObject> {
    let l = nodes::layout(buf)?;
    let ctx = Ctx { buf, layout: &l, path };
    let mut r = Reader::new(buf);
    r.skip(l.head.body_start)?;
    ctx.dyna_object(&mut r)
}
