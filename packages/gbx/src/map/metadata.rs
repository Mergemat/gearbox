use super::AuthorClone;
use crate::reader::Reader;
use anyhow::{Result, bail, ensure};

enum Type {
    Scalar(u32),
    Array(Box<Type>, Box<Type>),
    Struct(Vec<Type>),
}

fn len(r: &mut Reader, version: u32) -> Result<usize> {
    if version == 0 {
        return r.len();
    }
    let first = r.u8()?;
    Ok(usize::from(first & 127) | if first > 127 { usize::from(r.u16()?) << 7 } else { 0 })
}

fn string(r: &mut Reader, version: u32) -> Result<String> {
    let n = len(r, version)?;
    Ok(String::from_utf8_lossy(r.bytes(n)?).into_owned())
}

fn ty(r: &mut Reader, version: u32) -> Result<Type> {
    let kind = if version == 0 { r.u32()? } else { u32::from(r.u8()?) };
    Ok(match kind {
        7 => Type::Array(Box::new(ty(r, version)?), Box::new(ty(r, version)?)),
        15 => {
            ensure!(version >= 2, "unsupported metadata struct version");
            let n = r.u8()?;
            r.string()?;
            let mut members = Vec::with_capacity(n as usize);
            for _ in 0..n {
                r.string()?;
                let member = ty(r, version)?;
                if version < 4 {
                    contents(r, &member, version)?;
                }
                members.push(member);
            }
            Type::Struct(members)
        }
        kind => Type::Scalar(kind),
    })
}

fn contents(r: &mut Reader, ty: &Type, version: u32) -> Result<Vec<AuthorClone>> {
    let mut clones = Vec::new();
    match ty {
        Type::Scalar(1) => r.skip(if version == 0 { 4 } else { 1 })?,
        Type::Scalar(2 | 3) => r.skip(4)?,
        Type::Scalar(5) => {
            string(r, version)?;
        }
        Type::Scalar(9 | 14) => r.skip(8)?,
        Type::Scalar(10 | 11) => r.skip(12)?,
        Type::Scalar(kind) => bail!("unsupported script metadata type {kind}"),
        Type::Array(key, value) => {
            let n = len(r, version)?;
            let regular = matches!(**key, Type::Scalar(0));
            let int_pairs = regular && matches!(**value, Type::Scalar(14));
            for _ in 0..n {
                if !regular {
                    contents(r, key, version)?;
                }
                if int_pairs {
                    clones.push(AuthorClone { offset_ms: r.i32()?, speed: 1_000_000.0 / r.i32()? as f32 });
                } else {
                    contents(r, value, version)?;
                }
            }
        }
        Type::Struct(members) => {
            for member in members {
                contents(r, member, version)?;
            }
        }
    }
    Ok(clones)
}

fn trait_value(
    r: &mut Reader,
    name: &str,
    ty: &Type,
    version: u32,
    enabled: &mut bool,
    clones: &mut Vec<AuthorClone>,
) -> Result<()> {
    if name == "Nadeo_IsCloneEnabled" && matches!(ty, Type::Scalar(1)) {
        *enabled = if version == 0 { r.bool32()? } else { r.bool8()? };
    } else {
        let value = contents(r, ty, version)?;
        if name == "Race_AuthorClones" {
            *clones = value;
        }
    }
    Ok(())
}

pub(super) fn read(r: &mut Reader) -> Result<(bool, Vec<AuthorClone>)> {
    let mut r = r.encapsulated()?;
    let mut clones = Vec::new();
    let mut enabled = false;
    r.chunks("ScriptMetadata", |r, id, _| {
        if id != 0x1100_2000 {
            return Ok(false);
        }
        let version = r.u32()?;
        ensure!((2..=6).contains(&version), "unsupported metadata version {version}");
        let version = version - 2;
        let n = len(r, version)?;
        if version < 3 {
            for _ in 0..n {
                let name = string(r, version)?;
                let ty = ty(r, version)?;
                trait_value(r, &name, &ty, version, &mut enabled, &mut clones)?;
            }
        } else {
            let types = (0..n).map(|_| ty(r, version)).collect::<Result<Vec<_>>>()?;
            for _ in 0..len(r, version)? {
                let name = string(r, version)?;
                let index = len(r, version)?;
                ensure!(index < types.len(), "invalid script metadata type index");
                trait_value(r, &name, &types[index], version, &mut enabled, &mut clones)?;
            }
        }
        Ok(true)
    })?;
    Ok((enabled, clones))
}
