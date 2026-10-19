use anyhow::{Result, ensure};
use gearbox_engine::{Aabb, ShapeId, SharedShapeId, Vec3,
    dyna::{DynaModel, KinematicModel, kinematic::{Easing, MotionCurve, Segment}}};
use gearbox_engine::static_world::{Geometry, Shape};

const DEFAULT_SUBSTEP_DISTANCE: f32 = 0.3;
const UNIT_INERTIA: f32 = 0.23873241;

pub(super) fn animation(archive: &gearbox_packs::map_model::Animation) -> Result<KinematicModel> {
    ensure!(archive.trans_axis < 3 && archive.rot_axis < 3,
        "invalid kinematic axis");
    let curve = |segments: &[(u8, u8, u32)]| MotionCurve {
        segments: segments.iter().map(|&(kind, reversed, duration)| Segment {
            easing: Easing::from_code(kind), reversed, duration,
        }).collect(), cached_duration: 0,
    };
    Ok(KinematicModel {
        translation: curve(&archive.translation), translation_axis: archive.trans_axis,
        translation_start: archive.trans_range[0], translation_end: archive.trans_range[1],
        rotation: curve(&archive.rotation), rotation_axis: archive.rot_axis,
        rotation_start: archive.rot_range[0], rotation_end: archive.rot_range[1],
    })
}

pub(super) fn body(shapes: &mut Vec<Shape>, shape: u32, mass: f32, cloned: bool) -> Result<DynaModel> {
    let mut model = DynaModel {
        mass: 1.0, inverse_inertia_matrix: gearbox_engine::Mat3::diagonal(
            Vec3::new(UNIT_INERTIA, UNIT_INERTIA, UNIT_INERTIA)),
        max_angular_velocity: 10000.0,
        externally_simulated: 1, substep_distance: DEFAULT_SUBSTEP_DISTANCE,
        kind: 1, ..Default::default()
    };
    validate_bounds(shapes, shape)?;
    if cloned { return Ok(model); }
    let bounds = archive_bounds(shapes, shape)?;
    let doubled = bounds.half + bounds.half;
    let square = Vec3::new(doubled.x * doubled.x, doubled.y * doubled.y, doubled.z * doubled.z);
    let scale = (1.0 / mass) * 12.0;
    model.mass = mass;
    model.center_x = bounds.center.x; model.center_y = bounds.center.y; model.center_z = bounds.center.z;
    model.inverse_inertia_matrix.rows[0].x = scale / (square.y + square.z);
    model.inverse_inertia_matrix.rows[1].y = scale / (square.x + square.z);
    model.inverse_inertia_matrix.rows[2].z = scale / (square.x + square.y);
    model.externally_simulated = 0;
    Ok(model)
}

fn archive_bounds(shapes: &mut Vec<Shape>, shape: u32) -> Result<Aabb> {
    match &shapes[shape as usize].geometry {
        Geometry::Mesh(mesh) => {
            let Some(&first) = mesh.vertices.first() else {
                return Ok(Aabb::new(Vec3::ZERO, Vec3::new(-1.0, -1.0, -1.0)));
            };
            let (mut low, mut high) = (first, first);
            for vertex in &mesh.vertices {
                for axis in 0..3 {
                    if low[axis] > vertex[axis] { low[axis] = vertex[axis]; }
                    if vertex[axis] > high[axis] { high[axis] = vertex[axis]; }
                }
            }
            Ok(Aabb::new((high + low) * 0.5, (high - low) * 0.5))
        }
        Geometry::Convex(hull) => Ok(hull.bounds),
        Geometry::Compound(_) => {
            let mut world = gearbox_engine::World::default();
            world.statics.shapes = std::mem::take(shapes);
            let state = gearbox_engine::State::default();
            let bounds = gearbox_engine::collision::Geometry::new(&world, &state)
                .local_box(ShapeId::Shared(SharedShapeId(shape)));
            *shapes = world.statics.shapes;
            Ok(bounds)
        }
    }
}

pub(super) fn validate_bounds(shapes: &[Shape], shape: u32) -> Result<()> {
    let shape = shapes.get(shape as usize)
        .ok_or_else(|| anyhow::anyhow!("missing map body shape"))?;
    if let Geometry::Compound(compound) = &shape.geometry {
        for child in &compound.children {
            let id = child.shared()
                .ok_or_else(|| anyhow::anyhow!("non-map compound child"))?;
            let child = shapes.get(id.index())
                .ok_or_else(|| anyhow::anyhow!("missing compound child"))?;
            ensure!(!matches!(child.geometry, Geometry::Compound(_)),
                "nested compound body bounds");
        }
    }
    Ok(())
}
