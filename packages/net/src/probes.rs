use crate::observe::surface_class;
use gearbox_engine::collision::{CollisionWorld, Lists, QueryFilter, Ray};
use gearbox_engine::shape::Geometry;
use gearbox_engine::{Iso, Session, Vec3};

pub const RAYS: usize = 51;
pub const FEATURES: usize = 22;
pub const LEN: usize = RAYS * FEATURES;

pub fn read(session: &Session) -> Vec<f32> {
    let body = session.body();
    measure(
        Geometry::new(&session.world, &session.state),
        &session.state.collision_world[session.config.manager],
        Iso { pos: body.pos, rot: body.rot },
    )
}

fn measure(geometry: Geometry<'_>, manager: &CollisionWorld, pose: Iso) -> Vec<f32> {
    let filter = QueryFilter { lists: Lists::SURFACES, layers: 0x12_8002, ..QueryFilter::default() };
    let mut out = Vec::with_capacity(LEN);
    let mut cast = |origin: Vec3, direction: Vec3| {
        let hit = manager.raycast(
            geometry,
            Ray { origin: pose.transform_point(origin), direction: pose.rot * direction },
            filter,
        );
        let mut row = [0.0; FEATURES];
        row[1] = 1.0;
        if let Some(hit) = hit {
            row[0] = 1.0;
            row[1] = hit.fraction;
            row[2..5].copy_from_slice(&pose.rot.transpose_mul_vec(hit.normal).to_array());
            row[5 + surface_class(hit.material as u8)] = 1.0;
        }
        out.extend(row);
    };
    for pitch in [-0.5_f32, 0.0, 0.5] {
        let horizontal = (1.0 - pitch * pitch).sqrt();
        for azimuth in 0..12 {
            let angle = azimuth as f32 * std::f32::consts::TAU / 12.0;
            let direction = Vec3::new(angle.sin() * horizontal, pitch, angle.cos() * horizontal);
            cast(Vec3::ZERO, direction * 64.0);
        }
    }
    for forward in [0.0, 4.0, 12.0, 24.0, 48.0] {
        for across in [-4.0, 0.0, 4.0] {
            cast(Vec3::new(across, 4.0, forward), Vec3::new(0.0, -16.0, 0.0));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use gearbox_engine::static_world::{
        Convex, Entity, Face, Geometry as StaticGeometry, Item, Shape, Surface,
    };
    use gearbox_engine::surface;
    use gearbox_engine::tree::{AabbTree, Leaf};
    use gearbox_engine::{Aabb, State, World};

    fn floor(height: f32) -> World {
        let mut world = World::default();
        let bounds = Aabb::new(Vec3::new(0.0, height, 30.0), Vec3::new(20.0, 0.0, 50.0));
        world.statics.shapes.push(Shape {
            main_dir: Vec3::new(0.0, 1.0, 0.0),
            geometry: StaticGeometry::Convex(Convex {
                material: surface::ASPHALT as u16,
                vertices: vec![
                    Vec3::new(-20.0, height, -20.0),
                    Vec3::new(-20.0, height, 80.0),
                    Vec3::new(20.0, height, 80.0),
                    Vec3::new(20.0, height, -20.0),
                ],
                indices: vec![0, 1, 2, 3],
                faces: vec![Face {
                    normal: Vec3::new(0.0, 1.0, 0.0),
                    offset: -height,
                    axis: 1,
                    positive: true,
                    start: 0,
                    count: 4,
                }],
                edges: vec![],
                normals: vec![],
                directions: vec![],
                bounds,
            }),
        });
        world.statics.items.push(Item {
            entity: Entity::None,
            shape: 0,
            iso: Iso::IDENTITY,
            layer: 2,
            flags: Default::default(),
            query_flags: Default::default(),
            id: 0,
            index: 0,
        });
        world.statics.surfaces.push(Surface { shape: 0, iso: Iso::IDENTITY, item: 0, tag: 0 });
        world.statics.tree.nodes = AabbTree::build(&mut [Leaf { bounds, index: 0 }], 1).nodes;
        world
    }

    #[test]
    fn clearance_resolves_small_changes_within_one_voxel() {
        let state = State::default();
        let manager = CollisionWorld::default();
        let pose = Iso { pos: Vec3::new(0.0, 1.0, 0.0), ..Iso::IDENTITY };
        let low = measure(Geometry::new(&floor(0.1), &state), &manager, pose);
        let high = measure(Geometry::new(&floor(0.2), &state), &manager, pose);
        let at = 37 * FEATURES;
        assert_eq!(low[at], 1.0);
        assert_eq!(&low[at + 2..at + 5], &[0.0, 1.0, 0.0]);
        assert_eq!(low[at + 5], 1.0);
        assert!((low[at + 1] * 16.0 - 4.9).abs() < 1e-5);
        assert!((high[at + 1] * 16.0 - 4.8).abs() < 1e-5);
    }
}
