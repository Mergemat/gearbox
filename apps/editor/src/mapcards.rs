use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, channel};
use std::time::SystemTime;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Mine,
    Downloads,
    Test,
}

impl Source {
    pub fn of(path: &Path) -> Self {
        if crate::catalog::test_maps().is_some_and(|t| path.starts_with(t)) {
            return Self::Test;
        }
        if dirs::download_dir().is_some_and(|d| path.starts_with(d)) { Self::Downloads } else { Self::Mine }
    }
}

#[derive(Clone)]
pub struct MapCard {
    pub path: PathBuf,
    pub name: String,
    pub author: Option<String>,
    pub author_time_ms: Option<u32>,
    pub environment: Option<String>,
    pub modified: SystemTime,
    pub source: Source,
    pub thumb: Option<Arc<gpui::RenderImage>>,
}

pub fn clean_name(s: &str) -> String {
    let mut out = String::new();
    let mut it = s.chars().peekable();
    while let Some(c) = it.next() {
        if c != '$' {
            out.push(c);
            continue;
        }
        match it.next() {
            Some('$') => out.push('$'),
            Some(h) if h.is_ascii_hexdigit() => {
                for _ in 0..2 {
                    if it.peek().is_some_and(|c| c.is_ascii_hexdigit()) {
                        it.next();
                    }
                }
            }
            Some('l' | 'L' | 'h' | 'H' | 'p' | 'P') if it.peek() == Some(&'[') => {
                for c in it.by_ref() {
                    if c == ']' {
                        break;
                    }
                }
            }
            _ => {}
        }
    }
    let t = out.trim();
    if t.is_empty() { s.to_string() } else { t.to_string() }
}

fn header_bytes(path: &Path) -> std::io::Result<Vec<u8>> {
    let mut buf = Vec::new();
    std::fs::File::open(path)?.take(512 * 1024).read_to_end(&mut buf)?;
    Ok(buf)
}

fn thumbnail(jpeg: &[u8], side: u32) -> Option<Arc<gpui::RenderImage>> {
    let img = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg).ok()?;
    let img = img.flipv().thumbnail(side, side).to_rgba8();
    let (w, h) = img.dimensions();
    let mut px = img.into_raw();
    for p in px.as_chunks_mut::<4>().0 {
        p.swap(0, 2);
    }
    let img = image::RgbaImage::from_raw(w, h, px)?;
    Some(Arc::new(gpui::RenderImage::new(smallvec::smallvec![image::Frame::new(img)])))
}

pub fn card(path: &Path) -> MapCard {
    let modified = std::fs::metadata(path).and_then(|m| m.modified()).unwrap_or(SystemTime::UNIX_EPOCH);
    let stem = crate::catalog::map_stem(path).unwrap_or_default();
    let h =
        header_bytes(path).ok().and_then(|b| gearbox_gbx::map::MapHeader::read(&b).ok()).unwrap_or_default();
    MapCard {
        path: path.to_path_buf(),
        name: h.name.as_deref().map(clean_name).unwrap_or(stem),
        author: h.author_name.as_deref().map(clean_name).or(h.author_login),
        author_time_ms: h.author_time_ms,
        environment: h.environment,
        modified,
        source: Source::of(path),
        thumb: h.thumbnail.as_deref().and_then(|j| thumbnail(j, 256)),
    }
}

pub fn load(paths: Vec<PathBuf>, wake: Arc<dyn Fn() + Send + Sync>) -> Receiver<MapCard> {
    let (tx, rx) = channel();
    std::thread::Builder::new()
        .name("map cards".into())
        .spawn(move || {
            for (i, p) in paths.iter().enumerate() {
                if tx.send(card(p)).is_err() {
                    return;
                }
                if i % 8 == 7 {
                    wake();
                }
            }
            wake();
        })
        .expect("spawning the map card reader");
    rx
}

#[cfg(test)]
mod tests {
    use super::clean_name;

    #[test]
    fn formatting_codes_are_dropped() {
        assert_eq!(clean_name("$o$f00Ginger$fffbread $l[https://x]Gorge$l"), "Gingerbread Gorge");
        assert_eq!(clean_name("$$5 off"), "$5 off");
        assert_eq!(clean_name("$i$sPlain"), "Plain");
    }
}
