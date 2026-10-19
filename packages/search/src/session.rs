use std::path::Path;
use anyhow::{Context, Result};
use gearbox_engine::Session;
use crate::input::Timeline;

pub fn open(packs: &Path, map: &Path, inputs: Option<&Timeline>) -> Result<Session> {
    gearbox_world::open_file(map, packs, gearbox_world::OpenOptions {
        t0: inputs.map_or(-1500, |t| t.t0), seed: inputs.and_then(|t| t.seed), car: None,
    }).with_context(|| format!("opening {}", map.display()))
}
