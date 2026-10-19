use gearbox_packs::{
    assets::{Assets, Car},
    visual::{VisualCache, bounds, point},
};

fn assets() -> Option<(Assets, VisualCache)> {
    let set = gearbox_packs::install::Game::locate(None).ok()?.open().unwrap();
    let assets = Assets::from_catalog(gearbox_packs::catalog::cached(&set).unwrap());
    Some((assets, VisualCache::new(set)))
}
#[test]
fn road_visual_and_collision_occupy_the_same_block() {
    let Some((assets, mut cache)) = assets() else { return };
    let parts = cache.model("Stadium/Media/Prefab/RoadTech/Straight_Air.Prefab.Gbx", 0).unwrap();
    assert!(!parts.is_empty());
    let visual = bounds(
        parts.iter().flat_map(|p| cache.meshes[p.mesh_id].positions.iter().map(|&v| point(p.transform, v))),
    )
    .unwrap();
    let collision = assets.block_model("RoadTechStraight", false, 0, 0, 0).unwrap();
    let mut positions = Vec::new();
    for part in collision {
        if let Some(node) = &part.surface.surf {
            if let gearbox_packs::surface::Surf::Mesh { vertices, .. } = &node.surf {
                let loc = part.chain.iter().fold(gearbox_packs::assets::Loc::IDENTITY, |loc, c| loc.then(c));
                let m = loc.matrix();
                for &p in vertices {
                    positions.push(std::array::from_fn(|i| {
                        m[i * 3] * p[0] + m[i * 3 + 1] * p[1] + m[i * 3 + 2] * p[2] + loc.position[i]
                    }));
                }
            }
        }
    }
    let collision = bounds(positions).unwrap();
    assert!(
        (0..3).all(|k| visual[0][k] <= collision[1][k] + 0.05 && collision[0][k] <= visual[1][k] + 0.05),
        "visual {visual:?}, collision {collision:?}"
    );
    for part in parts {
        let m = &cache.meshes[part.mesh_id];
        assert_eq!(m.normals.len(), m.positions.len());
        assert!(m.uvs.first().is_some_and(|uv| uv.len() == m.positions.len()));
        assert!(m.indices.iter().all(|&i| (i as usize) < m.positions.len()));
        assert!(m.submeshes.iter().all(|s| s.lod_mask & 1 != 0));
    }
    assert!(!cache.textures.is_empty());
}
#[test]
fn four_cars_have_textures_and_independently_movable_wheels() {
    let Some((_, mut cache)) = assets() else { return };
    for kind in Car::ALL {
        let car = cache.car(kind).unwrap();
        let mesh_count = cache.meshes.len();
        let repeated = cache.car(kind).unwrap();
        assert_eq!(cache.meshes.len(), mesh_count, "reloading a car duplicated cached meshes");
        assert_eq!(
            car.instances.iter().map(|i| i.mesh_id).collect::<Vec<_>>(),
            repeated.instances.iter().map(|i| i.mesh_id).collect::<Vec<_>>()
        );
        assert_eq!(car.wheels.len(), 4);
        assert!(car.diagnostics.is_empty(), "{:?}", car.diagnostics);
        for w in &car.wheels {
            assert!(!w.instances.is_empty(), "{} {}", car.car, w.name);
            for axis in [9, 11] {
                assert_eq!(
                    w.transform[axis].is_sign_positive(),
                    w.physics_transform[axis].is_sign_positive()
                );
            }
            for &i in &w.instances {
                let mesh = &cache.meshes[car.instances[i].mesh_id];
                assert!(
                    mesh.positions.iter().flatten().all(|v| v.abs() < 2.0),
                    "wheel geometry is not local: {} {}",
                    car.car,
                    w.name
                );
            }
        }
        assert!(
            car.instances.iter().any(|i| cache.meshes[i.mesh_id]
                .submeshes
                .iter()
                .any(|s| !cache.materials[*i.material_overrides.get(&s.material).unwrap_or(&s.material)]
                    .textures
                    .is_empty())),
            "{} has no textures",
            car.car
        );
    }
}
