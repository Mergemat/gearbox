mod clip;
mod pressure;
pub use clip::{BOX_TOPOLOGY, BoxFace, submerged_triangles};
pub use pressure::integrate;

use super::{WaterManager, drag::FluidTuning, surface::SurfaceGrid};
use crate::{
    Aabb, Vec3,
    dyna::{BodyHandle, Dyna, ForceAcc, SolverFlags},
    math::fast_log2,
};

const LOG10_2: f32 = 0.30103;
const FRICTION_NUMERATOR: f32 = 0.075;
const MIN_TRIANGLE_CROSS_SQUARED: f32 = 1.0000001e-6;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Triangle {
    pub vertices: [Vec3; 3],
    pub normal: Vec3,
}

pub trait Geometry {
    fn height(&mut self, grid: &SurfaceGrid, point: Vec3) -> f32;
    fn split_horizontal(&mut self, triangle: Triangle, remaining: usize) -> Vec<Triangle>;
}

pub fn water_grid(water: WaterManager<'_>, bounds: Aabb) -> Option<SurfaceGrid> {
    let half_height = bounds.half.y.abs();
    let point = Vec3::new(bounds.center.x, bounds.center.y - half_height, bounds.center.z);
    let mut level = 0.0;
    if !water.query(point, half_height + half_height, Some(&mut level), None) {
        return None;
    }
    Some(SurfaceGrid::flat([bounds.center.x, bounds.center.z], [bounds.half.x, bounds.half.z], [2, 2], level))
}

pub fn body_forces(
    state: &crate::dyna::BodyState,
    center: Vec3,
    bounds: Aabb,
    tuning: &FluidTuning,
    grid: &SurfaceGrid,
    geometry: &mut impl Geometry,
) -> ForceAcc {
    let span = bounds.half.z * 2.0;
    let speed = state.vel.dot(state.vel).sqrt();
    let reynolds = speed * span / tuning.reference_length;
    let log = fast_log2(reynolds) * LOG10_2 - 2.0;
    let friction = FRICTION_NUMERATOR / (log * log);
    let mut triangles = submerged_triangles(bounds, state.pose(), grid, geometry);
    for index in (0..triangles.len()).rev() {
        let [a, b, c] = triangles[index].vertices;
        let first = b - a;
        let second = c - a;
        let cross = Vec3::new(
            first.y * second.z - second.y * first.z,
            second.x * first.z - first.x * second.z,
            first.x * second.y - first.y * second.x,
        );
        if MIN_TRIANGLE_CROSS_SQUARED > cross.dot(cross) {
            triangles.swap_remove(index);
        }
    }
    integrate(state, center, tuning, friction, grid, &triangles, geometry).total()
}

pub fn apply(
    water: Option<WaterManager<'_>>,
    dyna: &mut Dyna,
    handle: BodyHandle,
    bounds: Aabb,
    world_bounds: Aabb,
    tuning: &FluidTuning,
    geometry: &mut impl Geometry,
) -> Option<ForceAcc> {
    let water = water?;
    let grid = water_grid(water, world_bounds);
    dyna.bodies[0].record.flags.set(SolverFlags::IN_WATER, grid.is_some());
    let body = dyna.body(handle);
    let force = grid.map_or_else(ForceAcc::default, |grid| {
        body_forces(&body.state, body.model.center(), bounds, tuning, &grid, geometry)
    });
    let inverse_mass = 1.0 / body.model.mass;
    Some(ForceAcc {
        force: inverse_mass * force.force,
        torque: Vec3::new(
            body.inertia.diagonal.x * force.torque.x,
            body.inertia.diagonal.y * force.torque.y,
            body.inertia.diagonal.z * force.torque.z,
        ),
    })
}

pub struct CollisionGeometry;

impl Geometry for CollisionGeometry {
    fn height(&mut self, grid: &SurfaceGrid, point: Vec3) -> f32 {
        let planes: Vec<_> =
            grid.planes.iter().map(|p| [p.normal.x, p.normal.y, p.normal.z, p.offset]).collect();
        crate::collision::profile::HeightField {
            planes: &planes,
            origin: grid.origin,
            cells: grid.size,
            inverse_cell: grid.inverse_cell,
            cell_size: grid.cell,
        }
        .height(point)
    }
    fn split_horizontal(&mut self, triangle: Triangle, remaining: usize) -> Vec<Triangle> {
        let mut out = Vec::new();
        crate::shape::triangle::split_horizontal(triangle.vertices, triangle.normal, &mut out, remaining);
        out.into_iter().map(|t| Triangle { vertices: t.vertices, normal: t.normal }).collect()
    }
}
