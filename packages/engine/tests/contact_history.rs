use gearbox_engine::{
    Aabb, DynamicItemId, EllipsoidId, Iso, Mat3, ShapeId, SphereId, State, Vec3, World,
    collision::{CollisionWorld, DynamicItem, EllipsoidShape, ShapeEntry, SphereShape},
    dyna::{Body, Dyna, DynaModel},
    static_world::{Convex, Entity, Face, Geometry, Item, Shape, Surface},
    tree::{AabbTree, Leaf},
    vehicle::{
        Tuning, Vehicle,
        impacts::ImpactModel,
        model::ForceGeometry,
        step::{Assets, StepCtx, collision_step},
    },
    water::drag::FluidTuning,
};

#[test]
fn contact_normals_retain_history_across_collision_steps() {
    let bounds = Aabb { center: Vec3::ZERO, half: Vec3::new(2.0, 2.0, 2.0) };
    let mut world = World::default();
    world.statics.shapes.push(Shape {
        main_dir: Vec3::ZERO,
        geometry: Geometry::Convex(Convex {
            material: 0,
            vertices: vec![
                Vec3::new(-1.0, 0.0, -1.0),
                Vec3::new(-1.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, -1.0),
            ],
            indices: vec![0, 1, 2, 3],
            faces: vec![Face {
                normal: Vec3::new(0.0, 1.0, 0.0),
                offset: 0.0,
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
    let tilted =
        Iso::new(Mat3::from_array([0.995, -0.1, 0.0, 0.1, 0.995, 0.0, 0.0, 0.0, 1.0]), Vec3::ZERO);
    for i in 0..2 {
        world.statics.items.push(Item {
            entity: Entity::None,
            shape: 0,
            iso: tilted,
            layer: 2,
            flags: Default::default(),
            query_flags: Default::default(),
            id: i,
            index: i,
        });
        world.statics.surfaces.push(Surface {
            shape: 0,
            iso: tilted,
            item: i,
            tag: u32::MAX as u64,
        });
    }
    world.statics.tree.nodes =
        AabbTree::build(&mut [Leaf { bounds, index: 0 }, Leaf { bounds, index: 1 }], 1).nodes;
    let mut state = State::default();
    state.ellipsoid_shape.push(EllipsoidShape { material: 0, radii: Vec3::new(0.4, 0.4, 0.4) });
    state.sphere_shape.push(SphereShape { material: 0, radius: 0.4 });
    state.dynamic_item.push(DynamicItem {
        shape: ShapeId::Ellipsoid(EllipsoidId(0)),
        layer: 4,
        entry_count: 1,
        ..Default::default()
    });
    state.shape_entry.push(ShapeEntry {
        shape: ShapeId::Ellipsoid(EllipsoidId(0)),
        item: Some(DynamicItemId(0).into()),
        tag: u32::MAX as u64,
        ..Default::default()
    });
    state.collision_world.push(CollisionWorld { shape_entries: 0..1, ..Default::default() });
    let mut dyna = Dyna::default();
    let body = dyna.handles.take();
    dyna.bodies.push(Body::default());
    dyna.bodies[0].state.set_pose(Iso::new(Mat3::IDENTITY, Vec3::new(0.0, 0.3, 0.0)));
    let mut vehicle = Vehicle { body: body.0, item: Some(DynamicItemId(0)), ..Default::default() };
    let tuning = Tuning { collision_scale: 1.0, ..Default::default() };
    let geometry = ForceGeometry::default();
    let impacts = ImpactModel::default();
    let fluid = FluidTuning::default();
    let model = DynaModel::default();
    let mut ctx = StepCtx {
        world: &mut world,
        state: &mut state,
        manager: 0,
        dyna: &mut dyna,
        vehicle: &mut vehicle,
        assets: Assets {
            tuning: &tuning,
            geometry: &geometry,
            impacts: &impacts,
            fluid: &fluid,
            body: &model,
        },
        tick: 0,
        dt: 0.01,
        ghost_contacts: &[],
        events: None,
    };
    let first = collision_step(&mut ctx, 0, 0.01, 0.01);
    assert_eq!(first.contacts.len(), 1);
    assert!(first.contacts.as_slice()[0].normal.x < -0.05);
    for surface in &mut ctx.world.statics.surfaces {
        surface.iso = Iso::IDENTITY;
    }
    for item in &mut ctx.world.statics.items {
        item.iso = Iso::IDENTITY;
    }
    ctx.state.dynamic_item[0].shape = ShapeId::Sphere(SphereId(0));
    ctx.state.shape_entry[0].shape = ShapeId::Sphere(SphereId(0));
    let second = collision_step(&mut ctx, 10_000_000, 0.01, 0.01);
    assert_eq!(second.contacts.len(), 1);
    assert!(
        second.contacts.as_slice()[0].normal.x < -0.05,
        "{:?}",
        second.contacts.as_slice()[0].normal
    );
}
