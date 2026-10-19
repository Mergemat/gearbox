use anyhow::{Context, Result, ensure};
use gearbox_packs::assets::{Assets, Car};
use gearbox_packs::visual::{self, Scene, VisualCache};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

pub fn run(args: &[String]) -> Result<()> {
    let set = crate::game()?.open()?;
    let assets = Assets::from_catalog(gearbox_packs::catalog::cached(&set)?);
    let mut cache = VisualCache::new(set);
    match args[0].as_str() {
        "gltf" => {
            ensure!(args.len() == 3, "gltf <map> <out.glb>");
            let map = gearbox_gbx::Map::parse(&std::fs::read(&args[1])?)?;
            let mut scene = visual::visual_scene(&map, &assets, &mut cache)?;
            match cache.car(Car::Sport) {
                Ok(car) => {
                    let origin = scene
                        .instances
                        .iter()
                        .find(|i| i.name.contains("Start"))
                        .or_else(|| scene.instances.first())
                        .map(|i| [i.transform[9], i.transform[10] + 3., i.transform[11]])
                        .unwrap_or([0., 3., 0.]);
                    for mut i in car.instances {
                        for k in 0..3 {
                            i.transform[9 + k] += origin[k];
                        }
                        scene.instances.push(i);
                    }
                    scene.diagnostics.extend(car.diagnostics);
                }
                Err(e) => eprintln!("car: {e:#}"),
            };
            visual::gltf::export(Path::new(&args[2]), &scene, &cache)?;
            println!("{}", serde_json::to_string_pretty(&stats(&args[1], &scene, &cache))?);
        }
        "gltf-car" => {
            ensure!(args.len() == 3, "gltf-car <CarSport|CarSnow|CarRally|CarDesert> <out.glb>");
            let car = Car::from_name(&args[1]).context("unknown car")?;
            let car = cache.car(car)?;
            visual::gltf::export(
                Path::new(&args[2]),
                &Scene { instances: car.instances, diagnostics: car.diagnostics, ..Default::default() },
                &cache,
            )?;
        }
        _ => anyhow::bail!("unknown visual command"),
    }
    Ok(())
}
fn stats(name: &str, scene: &Scene, cache: &VisualCache) -> serde_json::Value {
    let meshes: BTreeSet<_> = scene.instances.iter().map(|i| i.mesh_id).collect();
    let materials: BTreeSet<_> = scene
        .instances
        .iter()
        .flat_map(|i| {
            cache.meshes[i.mesh_id]
                .submeshes
                .iter()
                .map(|s| i.material_overrides.get(&s.material).copied().unwrap_or(s.material))
        })
        .collect();
    let textures: BTreeSet<_> =
        materials.iter().flat_map(|&m| cache.materials[m].textures.iter().map(|t| t.texture)).collect();
    let mut failures = BTreeMap::<String, BTreeMap<String, usize>>::new();
    for d in &scene.diagnostics {
        *failures.entry(d.feature.clone()).or_default().entry(d.message.clone()).or_default() += 1;
    }
    serde_json::json!({"map":name,"placements":scene.placements,"decoded_placements":scene.decoded_placements,"instances":scene.instances.len(),"meshes":meshes.len(),"unique_triangles":meshes.iter().map(|&i|cache.meshes[i].indices.len()/3).sum::<usize>(),"instanced_triangles":scene.instances.iter().map(|i|cache.meshes[i.mesh_id].indices.len()/3).sum::<usize>(),"materials":materials.len(),"textures":textures.len(),"bounds_checked":scene.bounds_checks.len(),"bounds_overlap":scene.bounds_checks.iter().filter(|b|b.overlap).count(),"bounds_failures":scene.bounds_checks.iter().filter(|b|!b.overlap).collect::<Vec<_>>(),"bounds":scene.bounds_checks,"failures":failures,"diagnostics":scene.diagnostics})
}
