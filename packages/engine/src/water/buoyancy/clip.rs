use super::super::surface::SurfaceGrid;
use super::{Geometry, Triangle};
use crate::{Aabb, Iso, Vec3};

const NO_WATER: f32 = f32::MIN;
const TRIANGLE_LIMIT: usize = 80;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxFace {
    pub vertices: [usize; 4],
    pub normal: Vec3,
}

pub const BOX_TOPOLOGY: [BoxFace; 6] = [
    BoxFace { vertices: [0, 2, 3, 1], normal: Vec3::new(0.0, 0.0, -1.0) },
    BoxFace { vertices: [0, 4, 6, 2], normal: Vec3::new(-1.0, 0.0, 0.0) },
    BoxFace { vertices: [0, 1, 5, 4], normal: Vec3::new(0.0, -1.0, 0.0) },
    BoxFace { vertices: [1, 3, 7, 5], normal: Vec3::new(1.0, 0.0, 0.0) },
    BoxFace { vertices: [2, 6, 7, 3], normal: Vec3::new(0.0, 1.0, 0.0) },
    BoxFace { vertices: [4, 5, 7, 6], normal: Vec3::new(0.0, 0.0, 1.0) },
];

struct FaceClip {
    count: usize,
    start: usize,
    first: Vec3,
    apex: Vec3,
    skip_first: bool,
}

fn clip_face(vertices: [Vec3; 4], grid: &SurfaceGrid, geometry: &mut impl Geometry) -> Option<FaceClip> {
    let mut down = None;
    let mut up = None;
    let mut touches_missing = false;
    for edge in 0..4 {
        let first = vertices[edge];
        let second = vertices[(edge + 1) % 4];
        let first_height = geometry.height(grid, first);
        touches_missing |= first_height == NO_WATER;
        let second_height = geometry.height(grid, second);
        if (first.y > first_height) != (second.y > second_height) {
            if first.y > first_height {
                down = Some(edge);
            } else {
                up = Some((edge + 1) % 4);
            }
        }
    }
    let first = vertices[0];
    let height = geometry.height(grid, first);
    let crossings = down.zip(up);
    if (crossings.is_none() && first.y >= height) || touches_missing {
        return None;
    }
    let Some((down, up)) = crossings else {
        return Some(FaceClip { count: 2, start: 1, first: vertices[1], apex: first, skip_first: false });
    };
    let down_from = vertices[down];
    let up_from = vertices[up];
    let down_to = vertices[(down + 1) % 4];
    let up_to = vertices[(3 + up) % 4];
    let down_depth = down_from.y - geometry.height(grid, down_from);
    let up_depth = up_from.y - geometry.height(grid, up_from);
    let down_end_depth = down_to.y - geometry.height(grid, down_to);
    let down_ratio = down_depth / (down_depth - down_end_depth);
    let up_end_depth = up_to.y - geometry.height(grid, up_to);
    let up_ratio = up_depth / (up_depth - up_end_depth);
    let first = down_from + (down_to - down_from) * down_ratio;
    let apex = up_from + (up_to - up_from) * up_ratio;
    if geometry.height(grid, apex) == NO_WATER {
        return None;
    }
    let skip_first = geometry.height(grid, first) == NO_WATER;
    let count = if up < down {
        up + 3 - down
    } else if up == down {
        3
    } else {
        up - down - 1
    };
    Some(FaceClip { count, start: down, first, apex, skip_first })
}

pub fn submerged_triangles(
    bounds: Aabb,
    pose: Iso,
    grid: &SurfaceGrid,
    geometry: &mut impl Geometry,
) -> Vec<Triangle> {
    let vertices = bounds.corners().map(|point| {
        let rows = pose.rot.rows;
        Vec3::new(
            point.y * rows[0].y + rows[0].x * point.x + point.z * rows[0].z,
            point.y * rows[1].y + rows[1].x * point.x + point.z * rows[1].z,
            point.y * rows[2].y + rows[2].x * point.x + point.z * rows[2].z,
        ) + pose.pos
    });
    let mut triangles = Vec::new();
    for face in BOX_TOPOLOGY {
        let vertices = face.vertices.map(|index| vertices[index]);
        let Some(clip) = clip_face(vertices, grid, geometry) else {
            continue;
        };
        let normal = pose.rot.transform_y_first(face.normal);
        for index in usize::from(clip.skip_first)..clip.count {
            if triangles.len() >= TRIANGLE_LIMIT {
                break;
            }
            let first = if index == 0 { clip.first } else { vertices[(index + clip.start) % 4] };
            let second = vertices[(clip.start + 1 + index) % 4];
            let triangle = Triangle { vertices: [first, second, clip.apex], normal };
            let added = geometry.split_horizontal(triangle, TRIANGLE_LIMIT - triangles.len());
            assert!(added.len() <= TRIANGLE_LIMIT - triangles.len(), "U2 triangle limit");
            if added.len() > 1 && geometry.height(grid, added[added.len() - 1].vertices[0]) == NO_WATER {
                continue;
            }
            triangles.extend(added);
        }
    }
    triangles
}
