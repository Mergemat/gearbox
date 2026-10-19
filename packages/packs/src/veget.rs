use anyhow::{Result, ensure};
use gearbox_gbx::reader::Reader;

use crate::prefab::{Ctx, NodeLink};
use crate::surface::Surface;

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Params {
    pub scale_variation: f32,
    pub tilt_degrees: f32,
    pub random_y: bool,
}

fn count(r: &mut Reader, max: u32) -> Result<u32> {
    let n = r.u32()?;
    ensure!(n <= max, "vegetation count {n} exceeds {max}");
    Ok(n)
}

pub fn collision(ctx: &Ctx<'_>) -> Result<(Option<Surface>, Params)> {
    ensure!(
        ctx.layout.head.class_id == 0x2f086000,
        "not a vegetation tree model"
    );
    let mut r = Reader::new(ctx.buf);
    r.skip(ctx.layout.head.body_start)?;
    let version = r.u32()?;
    ensure!(version == 21, "vegetation tree version {version}");
    r.u32()?;
    let lods = count(&mut r, 3)?;
    let leaves = count(&mut r, 2)?;
    let materials = count(&mut r, 8)?;
    let mut visual_only = Vec::new();
    let node = |r: &mut Reader| ctx.noderef(r, |c, r, _| c.skip_node(r)).map(|_| ());
    for _ in 0..materials {
        for _ in 0..9 {
            node(&mut r)?;
        }
        visual_only.push(r.u8()? != 0);
    }
    let ids = count(&mut r, 8)?;
    for _ in 0..ids {
        r.id()?;
    }
    let mut collision_visual = false;
    for _ in 0..lods {
        let visuals = count(&mut r, 8)?;
        for _ in 0..visuals {
            let material = r.u16()? as usize;
            ensure!(
                material < visual_only.len(),
                "vegetation material slot {material}"
            );
            let visual = ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
            collision_visual |= !matches!(visual, NodeLink::Null) && !visual_only[material];
            r.u8()?;
        }
    }
    r.skip(leaves as usize * 4)?;
    r.skip(1 + 4 + 1 + 1)?;
    r.skip(8)?;
    let tilt_degrees = r.f32()?;
    let scale_variation = r.f32()?;
    let random_y = r.u32()? != 0;
    let params = Params {
        scale_variation,
        tilt_degrees,
        random_y,
    };
    let surf_version = r.u32()?;
    let surf = crate::surface::read_surf(&mut r, surf_version)?;
    let no_collision = r.u32()? != 0;
    if no_collision {
        return Ok((None, params));
    }
    if surf.is_none() {
        if !collision_visual {
            return Ok((None, params));
        }
        todo!("tree collision from lowest visual LOD");
    }
    Ok((
        Some(Surface {
            version: 0,
            surf_version,
            surf,
            materials: Vec::new(),
            material_ids: Vec::new(),
        }),
        params,
    ))
}
