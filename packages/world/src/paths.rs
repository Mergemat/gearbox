use std::path::Path;

pub fn map_stem(path: &Path) -> Option<String> {
    let name = path.file_name()?.to_string_lossy().into_owned();
    let n = name.len().checked_sub(".map.gbx".len())?;
    (name.is_char_boundary(n) && name[n..].eq_ignore_ascii_case(".map.gbx")).then(|| name[..n].to_string())
}
