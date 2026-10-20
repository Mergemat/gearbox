use crate::reader::{NodeRef, Reader};
use crate::replay::{self, CLASS_GHOST};
use anyhow::{Context, Result, ensure};

pub(crate) fn node(r: &mut Reader) -> Result<()> {
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
            (0x0307_9000, 0x0307_9002) => {
                list(r)?;
                r.string()?;
                r.bool32()?;
            }
            (0x0307_9000, 0x0307_9003 | 0x0307_9005) => {
                list(r)?;
                r.string()?;
            }
            (0x0307_9000, 0x0307_9007 | 0x0307_900A..=0x0307_900C) => {
                r.u32()?;
            }
            (0x0307_9000, 0x0307_9008) => {
                r.f32()?;
            }
            (0x0307_9000, 0x0307_9009) => {
                r.string()?;
            }
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
            (0x0307_8000, 0x0307_8002 | 0x0307_8003) => {
                r.bool32()?;
            }
            (0x0307_8000, 0x0307_8004) => {
                r.bool32()?;
                r.bool32()?;
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
            (0x030A_1000, 0x030A_1000..=0x030A_1003) => {
                let version = if id == 0x030A_1003 { r.u32()? } else { id & 0xFFF };
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
            (0x030A_0000, 0x030A_0000) => {
                for _ in 0..r.len()? {
                    r.f32()?;
                    r.u8()?;
                    floats(r, 9)?;
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
                r.u32()?;
                r.bool32()?;
                r.u32()?;
                r.u32()?;
                if version < 6 {
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
                if version >= 10 {
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
            (0x0308_4000, 0x0308_4000) => floats(r, 3)?,
            (0x0308_4000, 0x0308_4001) => floats(r, 4)?,
            (0x0308_4000, 0x0308_4003..=0x0308_4006) => {
                r.f32()?;
                r.f32()?;
                r.id()?;
                r.i32()?;
                if id >= 0x0308_4004 {
                    floats(r, 11)?;
                }
                if id >= 0x0308_4005 {
                    r.bool32()?;
                }
                if id >= 0x0308_4006 {
                    r.bool32()?;
                }
            }
            (0x0309_F000, 0x030A_1000) => simple_camera_keys(r)?,
            (0x0309_F000, 0x030A_1001) => {
                r.i32()?;
            }
            (0x0309_F000, 0x030A_1002) => {
                let version = r.u32()?;
                ensure!(version <= 2, "simple camera version {version}");
                simple_camera_keys(r)?;
                r.i32()?;
                r.i32()?;
                if version >= 2 {
                    r.bool32()?;
                }
            }
            (0x0302_4000, 0x0302_4000) => keys(r, 3)?,
            (0x0308_1000, 0x0308_1001) => keys(r, 4)?,
            (0x0308_2000, 0x0308_2000)
            | (0x0313_3000, 0x0313_3000)
            | (0x0314_5000, 0x0314_5000)
            | (0x0338_B000, 0x0338_B000) => floats(r, 2)?,
            (0x0308_3000, 0x0308_3001) => keys(r, 3)?,
            (0x030A_6000, 0x030A_6000) => keys(r, 2)?,
            (0x030A_6000, 0x030A_6001) => keys(r, 3)?,
            (0x030A_7000, 0x030A_7001) => {
                r.pack_desc()?;
                keys(r, 3)?;
            }
            (0x030A_7000, 0x030A_7002) => {
                r.i32()?;
                r.bool32()?;
            }
            (0x030A_7000, 0x030A_7003) => {
                let version = r.u32()?;
                ensure!(version <= 2, "sound version {version}");
                r.i32()?;
                r.bool32()?;
                r.bool32()?;
                if version >= 1 {
                    r.bool32()?;
                }
                if version >= 2 {
                    r.bool32()?;
                    r.i32()?;
                }
            }
            (0x030A_7000, 0x030A_7004) => {
                r.pack_desc()?;
                let version = r.u32()?;
                ensure!(version <= 1, "sound key version {version}");
                keys(r, if version >= 1 { 6 } else { 3 })?;
            }
            (0x030C_0000, 0x030C_0000) => {
                let version = r.u32()?;
                floats(r, 2)?;
                if version >= 1 {
                    r.f32()?;
                }
            }
            (0x030E_5000, 0x030E_5001) => {
                floats(r, 2)?;
                node(r)?;
                r.f32()?;
            }
            (0x030E_5000, 0x030E_5002) => {
                let version = r.u32()?;
                if version >= 3 {
                    keys(r, 2)?;
                } else {
                    floats(r, 2)?;
                }
                node(r)?;
                r.f32()?;
                r.bool32()?;
                r.bool32()?;
                r.bool32()?;
            }
            (0x030E_B000, 0x030E_B000) | (0x0316_5000, 0x0316_5000) | (0x0316_D000, 0x0316_D000) => {
                r.u32()?;
                keys(r, 2)?;
            }
            (0x0312_7000, 0x0312_7000..=0x0312_7004) => keys(r, 5)?,
            (0x0312_A000, 0x0312_A001) => {
                r.u32()?;
                floats(r, 2)?;
                r.string()?;
            }
            (0x0313_3000, 0x0313_3001) | (0x0338_B000, 0x0338_B001) => {
                r.i32()?;
            }
            (0x0313_9000, 0x0313_9000) => floats(r, 2)?,
            (0x0313_9000, 0x0313_9001) => {
                floats(r, 12)?;
                r.u8()?;
                floats(r, 3)?;
                r.pack_desc()?;
            }
            (0x0316_1000, 0x0316_1000) => {
                let version = r.u32()?;
                r.string()?;
                if version == 0 {
                    floats(r, 2)?;
                }
                keys(r, 4)?;
            }
            (0x0316_6000, 0x0316_6000) => {
                r.u32()?;
                floats(r, 5)?;
            }
            (0x0316_C000, 0x0316_C000) => {
                let version = r.u32()?;
                r.i32()?;
                keys(r, 4)?;
                if version >= 1 {
                    r.i32()?;
                }
            }
            (0x0317_2000, 0x0317_2000) => {
                let version = r.u32()?;
                ensure!(version <= 2, "coloring version {version}");
                if version >= 2 {
                    r.i32()?;
                }
                for _ in 0..r.len()? {
                    floats(r, if version >= 1 { 3 } else { 2 })?;
                    if version >= 2 {
                        r.skip(2)?;
                    }
                }
                if version >= 1 {
                    r.i32()?;
                }
            }
            (0x0318_8000, 0x0318_8000) => {
                r.u32()?;
                keys(r, 4)?;
            }
            (0x0319_5000, 0x0319_5000) => {
                r.u32()?;
                floats(r, 3)?;
                r.string()?;
            }
            (0x031A_A000, 0x031A_A000) => {
                floats(r, 2)?;
                for _ in 0..r.len()? {
                    r.pack_desc()?;
                }
                keys(r, 18)?;
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

fn simple_camera_keys(r: &mut Reader) -> Result<()> {
    for _ in 0..r.len()? {
        r.f32()?;
        r.u8()?;
        floats(r, 15)?;
    }
    Ok(())
}

fn keys(r: &mut Reader, words: usize) -> Result<()> {
    for _ in 0..r.len()? {
        floats(r, words)?;
    }
    Ok(())
}

fn floats(r: &mut Reader, count: usize) -> Result<()> {
    for _ in 0..count {
        r.f32()?;
    }
    Ok(())
}
