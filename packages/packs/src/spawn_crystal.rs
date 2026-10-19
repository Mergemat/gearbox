use crate::prefab::Ctx;
use anyhow::{Result, bail, ensure};
use gearbox_gbx::reader::Reader;

fn count(r: &mut Reader) -> Result<usize> {
    let n = r.u32()? as usize;
    ensure!(n < 1 << 24, "crystal array count {n}");
    Ok(n)
}
fn array(r: &mut Reader, stride: usize) -> Result<()> {
    let n = count(r)?;
    r.skip(n * stride)
}
fn id(r: &mut Reader) -> Result<()> {
    let i = r.u32()?;
    if matches!(i, 0x4000_0000 | 0x8000_0000) {
        r.string()?;
    }
    Ok(())
}
fn width(bound: usize) -> usize {
    if bound <= 0xff {
        1
    } else if bound <= 0xffff {
        2
    } else {
        4
    }
}
fn optimized(r: &mut Reader, stride: usize) -> Result<()> {
    let n = count(r)?;
    let bytes = if n < 0xff {
        1
    } else if n < 0xffff {
        2
    } else {
        4
    };
    r.skip(n * stride * bytes)
}
fn geometry(r: &mut Reader, materials: usize) -> Result<()> {
    let v = r.u32()?;
    ensure!((29..=37).contains(&v), "spawn crystal geometry version {v}");
    r.u32()?;
    array(r, 8)?;
    let n = count(r)?;
    for _ in 0..n {
        r.skip(8 + 48)?;
        r.string()?;
        r.u32()?;
    }
    let groups = count(r)?;
    for _ in 0..groups {
        if v >= 31 {
            r.u32()?;
        }
        if v >= 36 {
            r.u8()?;
        } else {
            r.u32()?;
        }
        r.u32()?;
        r.string()?;
        r.u32()?;
        array(r, 4)?;
    }
    let embedded = if v >= 34 { r.u8()? != 0 } else { r.u32()? != 0 };
    ensure!(embedded, "external spawn crystal geometry");
    if v >= 33 {
        r.skip(8)?;
    }
    let positions = count(r)?;
    r.skip(12 * positions)?;
    let edges = count(r)?;
    if v < 35 {
        r.skip(edges * 8)?;
    } else {
        optimized(r, 2)?;
    }
    let faces = count(r)?;
    if v >= 37 {
        array(r, 8)?;
        optimized(r, 1)?;
    }
    for _ in 0..faces {
        let n = if v >= 35 {
            r.u8()? as usize + 3
        } else {
            count(r)?
        };
        r.skip(
            n * if v >= 34 {
                if positions < 0xff {
                    1
                } else if positions < 0xffff {
                    2
                } else {
                    4
                }
            } else {
                4
            },
        )?;
        if v < 37 {
            r.skip(n * 8)?;
        }
        r.skip(if v >= 33 && materials != 0 {
            width(materials)
        } else {
            4
        })?;
        r.skip(if v >= 33 { width(groups) } else { 4 })?;
    }
    if v < 30 {
        r.skip(faces * 4)?;
    }
    r.u32()?;
    if v < 32 {
        ensure!(r.u32()? == 0, "spawn crystal links");
        r.u32()?;
        r.string()?;
        if v < 30 {
            array(r, 4)?;
        }
    }
    if v < 36 {
        let a = count(r)?;
        let b = count(r)?;
        let c = count(r)?;
        r.skip((a + b + c) * 4)?;
        r.u32()?;
    }
    Ok(())
}

pub(super) fn spawn(ctx: &Ctx, start: usize) -> Result<Option<[u32; 12]>> {
    let mut r = Reader::new(ctx.buf);
    r.skip(start)?;
    let mut materials = 0;
    loop {
        let chunk = r.u32()?;
        if r.peek_u32() == Some(0x534b_4950) {
            r.u32()?;
            let n = count(&mut r)?;
            r.skip(n)?;
            continue;
        }
        match chunk {
            0x0905_1000 => {
                r.u32()?;
            }
            0x0900_3003 => {
                r.u32()?;
                materials = count(&mut r)?;
                for _ in 0..materials {
                    if r.string()?.is_empty() {
                        ctx.noderef(&mut r, |c, r, _| c.skip_node(r))?;
                    }
                }
            }
            0x0900_3005 => {
                ensure!(r.u32()? == 0, "spawn crystal layer-list version");
                let n = count(&mut r)?;
                for _ in 0..n {
                    let kind = r.u32()?;
                    let v = r.u32()?;
                    ensure!(v <= 2, "spawn crystal layer version {v}");
                    r.u32()?;
                    id(&mut r)?;
                    r.string()?;
                    if v >= 1 {
                        r.u32()?;
                    }
                    if matches!(kind, 1..=12 | 15 | 18) {
                        r.u32()?;
                        let n = count(&mut r)?;
                        for _ in 0..n {
                            r.u32()?;
                            id(&mut r)?;
                        }
                    }
                    let version = r.u32()?;
                    match kind {
                        0 => {
                            geometry(&mut r, materials)?;
                            array(&mut r, 4)?;
                            if version >= 1 {
                                r.skip(8)?;
                            }
                        }
                        1 | 8 => {
                            r.u32()?;
                        }
                        2 | 7 => {
                            r.skip(12)?;
                        }
                        3 | 5 => {
                            r.skip(12)?;
                        }
                        4 => {
                            r.skip(16)?;
                        }
                        6 => {
                            r.u32()?;
                        }
                        9 => {
                            r.skip(if version >= 1 { 12 } else { 8 })?;
                        }
                        10 => {
                            r.skip(8)?;
                        }
                        12 => {
                            ensure!(version == 0, "spawn crystal deformation version {version}");
                            r.skip(24 + 48)?;
                            array(&mut r, 48)?;
                        }
                        14 => {
                            geometry(&mut r, materials)?;
                            if version >= 1 {
                                array(&mut r, 4)?;
                            }
                        }
                        15 => {
                            ensure!(version <= 1, "spawn layer version {version}");
                            let p = r.vec3()?;
                            let a = [
                                r.f32()?,
                                r.f32()?,
                                if version >= 1 { r.f32()? } else { 0.0 },
                            ];
                            let a = a.map(|a| a * std::f32::consts::PI / 180.0);
                            let rot = crate::reader_math::quat_matrix(crate::reader_math::angles_quat(
                                a[0], a[1], a[2],
                            ));
                            return Ok(Some(std::array::from_fn(|i| {
                                if i < 9 { rot[i] } else { p[i - 9] }.to_bits()
                            })));
                        }
                        _ => bail!("unsupported spawn crystal layer type {kind}"),
                    }
                }
                return Ok(None);
            }
            0xfaca_de01 => return Ok(None),
            _ => bail!("unsupported spawn crystal chunk {chunk:#x}"),
        }
    }
}
