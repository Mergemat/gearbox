use anyhow::{Context, Result, ensure};
use gearbox_engine::{
    Aabb, CompoundInstanceId, EllipsoidId, Iso, ShapeId, SharedShapeId, SphereId, State, Vec3, World,
    collision::{CompoundInstance, EllipsoidShape, SphereShape},
    static_world as sw,
};
use gearbox_packs::{
    car_model::CarModel,
    surface::{Surf, SurfNode},
};

const GAMEPLAY_ID_MASK: u16 = 65280;

fn primitive(source: &SurfNode, state: &mut State) -> Result<ShapeId> {
    let material = 0;
    let direction = source.gameplay_main_dir.unwrap_or([0.0, 0.0, 1.0]);
    ensure!(
        direction.map(f32::to_bits) == [0.0f32.to_bits(), 0.0f32.to_bits(), 1.0f32.to_bits()],
        "unsupported car primitive gameplay direction"
    );
    match &source.surf {
        Surf::Sphere { radius, material: archived } => {
            let result = ShapeId::Sphere(SphereId(state.sphere_shape.len() as u32));
            state
                .sphere_shape
                .push(SphereShape { radius: *radius, material: archived.map_or(material, |id| id as u16) });
            Ok(result)
        }
        Surf::Ellipsoid { size, material: archived } => {
            let result = ShapeId::Ellipsoid(EllipsoidId(state.ellipsoid_shape.len() as u32));
            state.ellipsoid_shape.push(EllipsoidShape {
                radii: Vec3::from_array(*size),
                material: archived.map_or(material, |id| id as u16),
            });
            Ok(result)
        }
        _ => {
            anyhow::bail!("unsupported car compound child kind {}", source.kind)
        }
    }
}

pub(super) fn compound(source: &SurfNode, world: &mut World, state: &mut State) -> Result<ShapeId> {
    let Surf::Compound { surfs, locs, joints } = &source.surf else {
        anyhow::bail!("unsupported non-compound car collision");
    };
    ensure!(
        surfs.len() == locs.len() && surfs.len() == joints.len(),
        "unsupported car compound child/transform/joint counts"
    );
    let children = surfs.iter().map(|child| primitive(child, state)).collect::<Result<_>>()?;
    let result = ShapeId::Shared(SharedShapeId(world.statics.shapes.len() as u32));
    world.statics.shapes.push(sw::Shape {
        main_dir: source.gameplay_main_dir.map(Vec3::from_array).unwrap_or(Vec3::new(0.0, 0.0, 1.0)),
        geometry: sw::Geometry::Compound(sw::Compound {
            children,
            locations: locs.iter().copied().map(Iso::from_array).collect(),
            joints: joints.clone(),
            tree: sw::Tree::default(),
        }),
    });
    Ok(result)
}

pub(super) fn prepare(
    source: &CarModel,
    tuning: &gearbox_engine::vehicle::Tuning,
    wheels: &mut [gearbox_engine::vehicle::model::WheelDefinition],
    world: &mut World,
    state: &mut State,
) -> Result<(ShapeId, ShapeId)> {
    let surface = source.collision.surface.surf.as_ref().context("empty car collision surface")?;
    let original = compound(surface, world, state)?;
    let prepared = compound(surface, world, state)?;
    let skeleton = source.collision.skeleton.as_ref().context("car collision without skeleton")?;
    ensure!(
        source.wheels.iter().all(|wheel| skeleton.names.contains(&wheel.name)),
        "wheel absent from car skeleton"
    );
    let shape_index = prepared.shared().context("car compound reference")?.index();
    world.statics.shapes[shape_index].main_dir = Vec3::new(0.0, 0.0, 1.0);
    let sw::Geometry::Compound(base) = &world.statics.shapes[shape_index].geometry else {
        anyhow::bail!("unsupported car compound base");
    };
    ensure!(!base.joints.is_empty(), "unsupported car collision without skeleton joints");
    let mut transforms = Vec::new();
    let mut tags = Vec::new();
    for (index, (&joint, &child)) in base.joints.iter().zip(&base.children).enumerate() {
        let joint = usize::try_from(joint).context("unsupported negative car skeleton joint")?;
        let name = skeleton.names.get(joint).context("car joint absent from skeleton names")?;
        let transform =
            skeleton.transforms.get(joint).context("car joint absent from skeleton transforms")?;
        transforms.push(Iso::from_array(transform.map(f32::from_bits)));
        let wheel = source.wheels.iter().position(|wheel| wheel.name == *name);
        tags.push(wheel.map_or(sw::NO_CHILD_TAG, |wheel| wheel as u64));
        if let Some(wheel) = wheel {
            wheels[wheel].transform_index = Some(index);
            set_wheel(child, &wheels[wheel], tuning.wheel_surface, state)?;
        }
    }
    let template = instance(prepared, transforms, tags, state);
    Ok((original, template))
}

fn set_wheel(
    shape: ShapeId,
    wheel: &gearbox_engine::vehicle::model::WheelDefinition,
    material: u8,
    state: &mut State,
) -> Result<()> {
    match shape {
        ShapeId::Sphere(id) => {
            let sphere = &mut state.sphere_shape[id.index()];
            sphere.radius = wheel.radius;
            sphere.material = (sphere.material & GAMEPLAY_ID_MASK) | material as u16;
        }
        ShapeId::Ellipsoid(id) => {
            let ellipsoid = &mut state.ellipsoid_shape[id.index()];
            ellipsoid.radii = Vec3::new(wheel.contact_radius, wheel.radius, wheel.radius);
            ellipsoid.material = (ellipsoid.material & GAMEPLAY_ID_MASK) | material as u16;
        }
        _ => anyhow::bail!("unsupported wheel collision primitive"),
    }
    Ok(())
}

pub(super) fn instance(base: ShapeId, transforms: Vec<Iso>, tags: Vec<u64>, state: &mut State) -> ShapeId {
    let result = ShapeId::Compound(CompoundInstanceId(state.compound_instance.len() as u32));
    state.compound_instance.push(CompoundInstance { base, transforms, tags });
    result
}

pub(super) fn bounds(shape: ShapeId, world: &World, state: &State) -> Aabb {
    gearbox_engine::shape::Geometry::new(world, state).local_box(shape)
}
