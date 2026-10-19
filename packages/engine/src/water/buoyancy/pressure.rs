use super::super::{
    drag::{FluidForces, FluidTuning, SurfaceElement, surface_drag},
    query::LAYER_MARGIN,
    surface::SurfaceGrid,
};
use super::{Geometry, Triangle};
use crate::{Quat, Vec3, dyna::BodyState};

const THIRD: f32 = 0.33333334;
const TWO_THIRDS: f32 = 0.6666667;
const GRAVITY: f32 = 9.8;
const DENSITY: f32 = 997.0;

fn checked_sqrt(value: f32) -> f32 {
    assert!(!(0.0 > value), "negative fluid square root");
    value.sqrt()
}

fn centroid([first, second, third]: [Vec3; 3]) -> Vec3 {
    Vec3::new(
        ((first.x + second.x) + third.x) * THIRD,
        ((second.y + first.y) + third.y) * THIRD,
        ((first.z + second.z) + third.z) * THIRD,
    )
}

fn triangle_area([first, second, third]: [Vec3; 3]) -> f32 {
    let edge_c = third - first;
    let edge_b = second - first;
    let cross = Vec3::new(
        edge_c.z * edge_b.y - edge_c.y * edge_b.z,
        edge_c.y * edge_b.x - edge_c.x * edge_b.y,
        edge_c.x * edge_b.z - edge_c.z * edge_b.x,
    );
    checked_sqrt((cross.z * cross.z + cross.x * cross.x) + cross.y * cross.y) * 0.5
}

fn pressure(triangle: Triangle, depth: f32, coefficient: f32, area: f32, centroid: Vec3) -> (f32, Vec3) {
    let [first, second, third] = triangle.vertices;
    let normal = triangle.normal;
    let vertical = (normal.x * 0.0 - normal.y * 1.0) + normal.z * 0.0;
    let sign = if vertical.to_bits() >> 31 != 0 { -1.0 } else { 1.0 };
    let height = first.y - third.y;
    if LAYER_MARGIN > height || LAYER_MARGIN > 1.0 - vertical.abs() {
        return (((coefficient * depth) * area) * vertical, centroid);
    }
    let edge_c = third - first;
    let edge_b = second - first;
    if LAYER_MARGIN > (first.y - second.y).abs() {
        let midpoint =
            Vec3::new((first.x + second.x) * 0.5, (second.y + first.y) * 0.5, (first.z + second.z) * 0.5);
        let projected = edge_c.z * edge_b.x - edge_c.x * edge_b.z;
        let cross_x = edge_b.z * 0.0 - edge_c.z * edge_b.y;
        let cross_z = edge_c.x * edge_b.y - edge_b.x * 0.0;
        let projected_area =
            checked_sqrt((projected * projected + cross_x * cross_x) + cross_z * cross_z) * 0.5;
        let edge_length = checked_sqrt(edge_b.dot_yxz(edge_b));
        let gradient = ((((edge_length * coefficient) * height) * 0.5) * (height / 3.0 + 0.5)) * vertical;
        let force = ((coefficient * depth) * projected_area) * sign + gradient;
        return (force, (third - midpoint) * THIRD + midpoint);
    }
    let midpoint =
        Vec3::new((third.x + second.x) * 0.5, (second.y + third.y) * 0.5, (third.z + second.z) * 0.5);
    let edge_bc = second - third;
    let projected = edge_bc.x * edge_c.z - edge_bc.z * edge_c.x;
    let cross_x = edge_bc.z * 0.0 - edge_bc.y * edge_c.z;
    let cross_z = edge_bc.y * edge_c.x - edge_bc.x * 0.0;
    let projected_area = checked_sqrt((projected * projected + cross_x * cross_x) + cross_z * cross_z) * 0.5;
    let edge_length = checked_sqrt(edge_bc.dot_yxz(edge_bc));
    let gradient = ((((edge_length * coefficient) * height) * 0.5) * (height / 3.0 + 0.5)) * vertical;
    let force = ((coefficient * depth) * projected_area) * sign + gradient;
    (force, (midpoint - first) * TWO_THIRDS + first)
}

pub fn integrate(
    state: &BodyState,
    local_center: Vec3,
    tuning: &FluidTuning,
    friction: f32,
    grid: &SurfaceGrid,
    triangles: &[Triangle],
    geometry: &mut impl Geometry,
) -> FluidForces {
    let center = Quat::from_mat3(&state.rot).rotate(local_center) + state.pos;
    let gravity_scale = tuning.scale * GRAVITY;
    let mut forces = FluidForces::default();
    for &triangle in triangles {
        let centroid = centroid(triangle.vertices);
        let relative = centroid - center;
        let angular = state.ang_vel;
        let velocity = Vec3::new(
            (angular.y * relative.z - angular.z * relative.y) + state.vel.x,
            (angular.z * relative.x - angular.x * relative.z) + state.vel.y,
            (angular.x * relative.y - angular.y * relative.x) + state.vel.z,
        );
        let area = triangle_area(triangle.vertices);
        if 0.0 > geometry.height(grid, centroid) - centroid.y {
            continue;
        }
        let first = triangle.vertices[0];
        let depth = geometry.height(grid, first) - first.y;
        let (force, point) = pressure(triangle, depth, gravity_scale * DENSITY, area, centroid);
        let relative = point - center;
        let acc = &mut forces.hydrostatic;
        acc.force.x = acc.force.x + 0.0;
        acc.force.y = force + acc.force.y;
        acc.force.z = acc.force.z + 0.0;
        let zero_y = relative.y * 0.0;
        acc.torque.x = (zero_y - relative.z * force) + acc.torque.x;
        acc.torque.y = (relative.z * 0.0 - relative.x * 0.0) + acc.torque.y;
        acc.torque.z = (relative.x * force - zero_y) + acc.torque.z;
        surface_drag(
            SurfaceElement { velocity, normal: triangle.normal, point: centroid, center },
            area,
            DENSITY,
            friction,
            tuning,
            &mut forces,
        );
    }
    forces
}
