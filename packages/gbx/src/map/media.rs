use crate::reader::{NodeRef, Reader};
use crate::replay::{self, CLASS_GHOST};
use anyhow::{Context, Result, ensure};

pub(super) fn node(r: &mut Reader) -> Result<()> {
    let class = match r.node_ref()? {
        NodeRef::Null | NodeRef::Shared(_) => return Ok(()),
        NodeRef::New(class) => class,
    };
    if class == replay::CLASS_ENT_RECORD_DATA {
        crate::record::parse_node_chunks(r)?;
        return Ok(());
    }
    if class == CLASS_GHOST {
        replay::ghost(r)?;
        return Ok(());
    }
    r.chunks("MediaTracker", |r, id, _| {
        match (class, id) {
            (0x0307_9000, 0x0307_900D) => {
                let version = r.u32()?;
                ensure!(version <= 1, "media clip version {version}");
                list(r)?;
                r.string()?;
                r.bool32()?;
                r.bool32()?;
                r.bool32()?;
                r.string()?;
                r.f32()?;
                r.i32()?;
            }
            (0x0307_8000, 0x0307_8001) => {
                r.string()?;
                list(r)?;
                r.i32()?;
            }
            (0x0307_8000, 0x0307_8005) => {
                let version = r.u32()?;
                ensure!(version <= 1, "media track version {version}");
                r.bool32()?;
                r.bool32()?;
                r.bool32()?;
                if version >= 1 {
                    r.f32()?;
                    r.f32()?;
                }
            }
            (0x0307_A000, 0x0307_A001..=0x0307_A003) => {
                let version = id & 0xFFF;
                list(r)?;
                for _ in 0..r.len()? {
                    if version >= 2 {
                        for _ in 0..4 {
                            r.i32()?;
                        }
                    }
                    if version >= 3 {
                        r.i32()?;
                        r.f32()?;
                    }
                    if version != 2 {
                        for _ in 0..r.len()? {
                            for _ in 0..3 {
                                r.i32()?;
                            }
                        }
                    }
                }
            }
            (0x030A_2000, 0x030A_2006) => {
                let version = 6 + r.u32()?;
                ensure!((6..=10).contains(&version), "custom camera key version {version}");
                for _ in 0..r.len()? {
                    r.f32()?;
                    r.i32()?;
                    r.bool32()?;
                    r.i32()?;
                    r.bool32()?;
                    r.i32()?;
                    r.vec3()?;
                    r.vec3()?;
                    r.f32()?;
                    r.vec3()?;
                    if version >= 7 {
                        r.f32()?;
                    }
                    for _ in 0..2 {
                        r.vec3()?;
                        r.vec3()?;
                        if version >= 10 {
                            r.i32()?;
                        }
                        r.f32()?;
                        r.vec3()?;
                        if version >= 7 {
                            r.f32()?;
                        }
                    }
                    if version == 8 {
                        r.i32()?;
                        r.i32()?;
                    }
                    if version >= 10 {
                        r.i32()?;
                    }
                }
            }
            (0x030A_1000, 0x030A_1003) => {
                let version = r.u32()?;
                ensure!(version <= 5, "path camera key version {version}");
                for _ in 0..r.len()? {
                    r.f32()?;
                    r.vec3()?;
                    r.vec3()?;
                    r.f32()?;
                    if version >= 3 {
                        r.f32()?;
                    }
                    r.bool32()?;
                    r.i32()?;
                    r.bool32()?;
                    r.i32()?;
                    r.vec3()?;
                    r.f32()?;
                    floats(r, 4)?;
                    if version >= 4 {
                        r.i32()?;
                        r.i32()?;
                    }
                }
            }
            (0x0308_4000, 0x0308_4007) => {
                let version = r.u32()?;
                ensure!(version <= 4, "game camera version {version}");
                floats(r, 2)?;
                if version < 2 {
                    r.id()?;
                } else {
                    r.i32()?;
                }
                r.i32()?;
                r.vec3()?;
                r.vec3()?;
                floats(r, 5)?;
                r.bool32()?;
                r.bool32()?;
                r.bool32()?;
                if version >= 1 {
                    r.f32()?;
                }
                if version >= 3 {
                    r.i32()?;
                }
            }
            (0x030A_0000, 0x030A_0001) => {
                let version = r.u32()?;
                ensure!(version <= 1, "orbital camera version {version}");
                for _ in 0..r.len()? {
                    r.f32()?;
                    r.u8()?;
                    floats(r, 3)?;
                    r.vec3()?;
                    floats(r, 3)?;
                    r.i32()?;
                    if version >= 1 {
                        for _ in 0..4 {
                            r.i32()?;
                        }
                    }
                }
            }
            (0x0319_9000, 0x0319_9000) => {
                let version = r.u32()?;
                ensure!(version <= 2, "fog key version {version}");
                for _ in 0..r.len()? {
                    floats(r, 4)?;
                    if version >= 1 {
                        r.f32()?;
                        r.vec3()?;
                    }
                    if version >= 2 {
                        floats(r, 2)?;
                    }
                }
            }
            (0x0308_5000, 0x0308_5000) => {
                for _ in 0..r.len()? {
                    floats(r, 3)?;
                }
            }
            (0x0312_8000, 0x0312_8000..=0x0312_8002) => {
                for _ in 0..r.len()? {
                    floats(r, 4)?;
                }
            }
            (0x030A_8000, 0x030A_8001) => {
                r.string()?;
                node(r)?;
            }
            (0x030A_8000, 0x030A_8002) => {
                r.vec3()?;
            }
            (0x030A_8000, 0x030A_8003) => {
                r.f32()?;
            }
            (0x030A_5000, 0x030A_5000) => {
                node(r)?;
                r.pack_desc()?;
            }
            (0x030A_5000, 0x030A_5001) => {
                r.f32()?;
            }
            (0x0701_0000, 0x0701_0001 | 0x0701_0002 | 0x0701_0004 | 0x0701_0005) => {
                let version = id & 0xFFF;
                for _ in 0..r.len()? {
                    floats(r, 7)?;
                    if version >= 2 {
                        r.f32()?;
                    }
                    if version >= 3 {
                        floats(r, 4)?;
                    }
                }
                r.bool32()?;
                if version >= 4 {
                    r.i32()?;
                    r.bool32()?;
                }
                if version >= 5 {
                    r.bool32()?;
                }
            }
            (0x030A_4000, 0x030A_4000) => {
                for _ in 0..r.len()? {
                    floats(r, 3)?;
                }
            }
            (0x030A_9000, 0x030A_9000) => {
                floats(r, 2)?;
            }
            (0x030A_B000, 0x030A_B000) => {
                for _ in 0..r.len()? {
                    floats(r, 2)?;
                }
                r.vec3()?;
                r.f32()?;
            }
            (0x0308_0000, 0x0308_0000..=0x0308_0003) => {
                for _ in 0..r.len()? {
                    floats(r, 29)?;
                }
            }
            (0x0304_B000 | 0x0304_C000, 0x0302_9001) => {
                let keys = r.len()?;
                floats(r, keys)?;
                ensure!(r.len()? == keys, "triangle key count differs");
                let vertices = r.len()?;
                for _ in 0..keys {
                    for _ in 0..vertices {
                        r.vec3()?;
                    }
                }
                for _ in 0..r.len()? {
                    floats(r, 4)?;
                }
                for _ in 0..r.len()? {
                    for _ in 0..3 {
                        r.i32()?;
                    }
                }
                for _ in 0..3 {
                    r.i32()?;
                }
                r.f32()?;
                for _ in 0..3 {
                    r.i32()?;
                }
            }
            (0x0329_F000, 0x0329_F000) => {
                let version = r.u32()?;
                ensure!((4..=11).contains(&version), "entity media version {version}");
                node(r)?;
                r.f32()?;
                for _ in 0..r.len()? {
                    r.i32()?;
                }
                for _ in 0..4 {
                    r.bool32()?;
                }
                if version < 5 {
                    r.vec3()?;
                }
                if version >= 11 {
                    r.i32()?;
                }
                r.ident()?;
                r.vec3()?;
                for _ in 0..r.len()? {
                    r.pack_desc()?;
                }
                if r.bool32()? {
                    replay::skip_badge(r)?;
                }
                if version >= 11 {
                    r.string()?;
                }
                for _ in 0..r.len()? {
                    r.f32()?;
                    r.i32()?;
                    if version >= 6 {
                        r.f32()?;
                        r.i32()?;
                        r.i32()?;
                        r.f32()?;
                        if version >= 9 {
                            r.f32()?;
                        }
                    }
                }
                if version == 5 {
                    r.f32()?;
                }
                if version >= 7 {
                    r.string()?;
                }
                if version >= 8 {
                    r.i32()?;
                }
                if version >= 11 {
                    r.i32()?;
                    r.i32()?;
                }
            }
            (0x0329_F000, 0x0329_F002) => {
                r.string()?;
            }
            (0x0312_9000, 0x0312_9000 | 0x0308_5000) => {
                for _ in 0..r.len()? {
                    floats(r, 2)?;
                }
            }
            (0x0318_6000, 0x0318_6000) => {
                r.pack_desc()?;
            }
            (0x0318_6000, 0x0318_6001) => {
                for _ in 0..r.len()? {
                    floats(r, 2)?;
                }
            }
            (0x0312_6000, 0x0312_6000..=0x0312_6002) => {
                let version = id & 0xFFF;
                for _ in 0..r.len()? {
                    floats(r, 3)?;
                    if version >= 1 {
                        r.i32()?;
                    }
                    if version >= 2 {
                        r.vec3()?;
                    }
                }
            }
            _ => return Ok(false),
        }
        Ok(true)
    })
    .with_context(|| format!("embedded node class 0x{class:08X}"))
}

fn list(r: &mut Reader) -> Result<()> {
    let version = r.u32()?;
    ensure!(version == 10, "media node list version {version}");
    for _ in 0..r.len()? {
        node(r)?;
    }
    Ok(())
}

fn floats(r: &mut Reader, count: usize) -> Result<()> {
    for _ in 0..count {
        r.f32()?;
    }
    Ok(())
}
