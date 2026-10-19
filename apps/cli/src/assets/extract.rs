use std::path::Path;

use anyhow::Result;
use gearbox_packs::packs::PackSet;

pub fn extract(packs: &Path, out: &Path, paks: &[String]) -> Result<()> {
    let set = PackSet::open(packs)?;
    let filters: Vec<String> = if paks.is_empty() {
        vec!["stadium.pak".into()]
    } else {
        paks.iter().map(|s| s.to_lowercase()).collect()
    };
    let f: Vec<&str> = filters.iter().map(|s| s.as_str()).collect();
    let cat = gearbox_packs::catalog::crawl(&set, &f)?;
    gearbox_packs::assets::save(&cat, out)?;
    let i = &cat.index;
    println!(
        "{} blocks, {} prefabs, {} surfaces, {} materials, {} modifiers, {} errors",
        i.blocks.len(),
        i.prefabs.len(),
        cat.surfaces.len(),
        i.materials.len(),
        i.modifiers.len(),
        i.errors.len()
    );
    for e in i.errors.iter().take(20) {
        println!("  {e}");
    }
    Ok(())
}
