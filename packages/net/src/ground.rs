use gearbox_engine::consumer::PhysicalTriangle;
use gearbox_engine::{Session, gameplay};

use crate::observe::surface_class;

pub const CELL: f32 = 2.0;
pub const LAYERS: usize = 4;
pub const BEARINGS: [f32; 9] = [-70.0, -45.0, -25.0, -10.0, 0.0, 10.0, 25.0, 45.0, 70.0];
pub const DISTANCES: [f32; 10] = [3.0, 6.0, 10.0, 16.0, 24.0, 34.0, 48.0, 66.0, 90.0, 120.0];
pub const FEATURES: usize = 5;
pub const LEN: usize = BEARINGS.len() * DISTANCES.len() * FEATURES;

const BEARING_SIN: [f32; 9] = [
    -0.939_692_6,
    -0.707_106_8,
    -0.422_618_26,
    -0.173_648_18,
    0.0,
    0.173_648_18,
    0.422_618_26,
    0.707_106_8,
    0.939_692_6,
];
const BEARING_COS: [f32; 9] = [
    0.342_020_14,
    0.707_106_8,
    0.906_307_8,
    0.984_807_7,
    1.0,
    0.984_807_7,
    0.906_307_8,
    0.707_106_8,
    0.342_020_14,
];
const MIN_NORMAL_Y: f32 = 0.3;
const MERGE: f32 = 1.0;
const MAX_COLUMNS: usize = 4096;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Layer {
    pub height: f32,
    pub normal_y: f32,
    pub surface: u8,
    pub special: u8,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ground {
    origin: [f32; 2],
    size: [usize; 2],
    starts: Vec<u32>,
    layers: Vec<Layer>,
}

impl Ground {
    pub fn build(session: &Session) -> Self {
        let start = session.config.initial_pose.pos;
        Self::from_triangles(&session.static_triangles(), [start.x, start.z])
    }

    pub fn from_triangles(triangles: &[PhysicalTriangle], center: [f32; 2]) -> Self {
        let faces: Vec<_> = triangles.iter().filter_map(face).collect();
        if faces.is_empty() {
            return Self::default();
        }
        let reach = MAX_COLUMNS as f32 * CELL * 0.5;
        let mut low = [f32::INFINITY; 2];
        let mut high = [f32::NEG_INFINITY; 2];
        for (points, ..) in &faces {
            for p in points {
                for (k, v) in [p[0], p[2]].into_iter().enumerate() {
                    low[k] = low[k].min(v.max(center[k] - reach));
                    high[k] = high[k].max(v.min(center[k] + reach));
                }
            }
        }
        let origin = low.map(|v| (v / CELL).floor() * CELL);
        let size: [usize; 2] =
            std::array::from_fn(|k| (((high[k] - origin[k]) / CELL).floor() as usize + 1).min(MAX_COLUMNS));
        let mut hits: Vec<(u32, Layer)> = Vec::new();
        for (points, normal, layer) in &faces {
            rasterize(points, normal, *layer, origin, size, &mut hits);
        }
        let columns = size[0] * size[1];
        let mut starts = vec![0u32; columns + 1];
        for (column, _) in &hits {
            starts[*column as usize + 1] += 1;
        }
        for i in 0..columns {
            starts[i + 1] += starts[i];
        }
        let mut fill = starts.clone();
        let mut sorted = vec![Layer { height: 0.0, normal_y: 0.0, surface: 0, special: 0 }; hits.len()];
        for (column, layer) in hits {
            let at = &mut fill[column as usize];
            sorted[*at as usize] = layer;
            *at += 1;
        }
        let mut layers = Vec::with_capacity(sorted.len().min(columns * LAYERS));
        for column in 0..columns {
            let cell = &mut sorted[starts[column] as usize..starts[column + 1] as usize];
            cell.sort_unstable_by(|a, b| b.height.total_cmp(&a.height));
            let first = layers.len();
            for &layer in cell.iter() {
                if layers.len() - first == LAYERS {
                    break;
                }
                if layers.len() == first
                    || layers.last().is_some_and(|l: &Layer| l.height - layer.height > MERGE)
                {
                    layers.push(layer);
                }
            }
            starts[column] = first as u32;
        }
        starts[columns] = layers.len() as u32;
        Self { origin, size, starts, layers }
    }

    pub fn column(&self, x: f32, z: f32) -> &[Layer] {
        let i = ((x - self.origin[0]) / CELL).floor();
        let k = ((z - self.origin[1]) / CELL).floor();
        if !(i >= 0.0 && k >= 0.0 && (i as usize) < self.size[0] && (k as usize) < self.size[1]) {
            return &[];
        }
        let column = k as usize * self.size[0] + i as usize;
        &self.layers[self.starts[column] as usize..self.starts[column + 1] as usize]
    }

    pub fn under(&self, x: f32, z: f32, limit: f32) -> Option<&Layer> {
        self.column(x, z).iter().find(|layer| layer.height <= limit)
    }

    pub fn ahead(&self, position: [f32; 3], forward: [f32; 2], out: &mut [f32]) {
        assert_eq!(out.len(), LEN);
        let length = forward[0].hypot(forward[1]);
        let [fx, fz] = if length > 1e-6 { forward.map(|v| v / length) } else { [0.0, 1.0] };
        let samples = out.as_chunks_mut::<FEATURES>().0;
        for b in 0..BEARINGS.len() {
            let dx = BEARING_COS[b] * fx + BEARING_SIN[b] * fz;
            let dz = BEARING_COS[b] * fz - BEARING_SIN[b] * fx;
            for (d, &distance) in DISTANCES.iter().enumerate() {
                let x = position[0] + dx * distance;
                let z = position[2] + dz * distance;
                let limit = position[1] + 2.0 + 0.25 * distance;
                samples[b * DISTANCES.len() + d] = match self.under(x, z, limit) {
                    Some(layer) => [
                        1.0,
                        ((layer.height - position[1]) / 16.0).clamp(-4.0, 4.0),
                        layer.normal_y,
                        f32::from(layer.surface) / 11.0,
                        f32::from(layer.special) / 4.0,
                    ],
                    None => [0.0; FEATURES],
                };
            }
        }
    }
}

fn face(triangle: &PhysicalTriangle) -> Option<([[f32; 3]; 3], [f32; 3], Layer)> {
    let [a, b, c] = triangle.v;
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if !(length > 1e-9)
        || normal[1] < MIN_NORMAL_Y * length
        || triangle.v.iter().flatten().any(|v| !v.is_finite())
    {
        return None;
    }
    let class = surface_class(triangle.surface);
    let special = match triangle.gameplay {
        gameplay::TURBO | gameplay::TURBO_2 | gameplay::TURBO_ROULETTE => 1,
        gameplay::NO_GRIP => 2,
        gameplay::NO_STEERING => 3,
        gameplay::FREE_WHEELING => 4,
        _ => match class {
            12 => 1,
            14 => 2,
            15 => 3,
            13 => 4,
            _ => 0,
        },
    };
    let layer = Layer {
        height: 0.0,
        normal_y: normal[1] / length,
        surface: if class < 12 { class as u8 } else { 0 },
        special,
    };
    Some((triangle.v, normal, layer))
}

fn rasterize(
    points: &[[f32; 3]; 3],
    normal: &[f32; 3],
    layer: Layer,
    origin: [f32; 2],
    size: [usize; 2],
    hits: &mut Vec<(u32, Layer)>,
) {
    let [a, b, c] = points.map(|p| [p[0], p[2]]);
    let height = |x: f32, z: f32| {
        points[0][1] - (normal[0] * (x - points[0][0]) + normal[2] * (z - points[0][2])) / normal[1]
    };
    let mut push = |i: isize, k: isize, height: f32| {
        if i >= 0 && k >= 0 && (i as usize) < size[0] && (k as usize) < size[1] {
            hits.push(((k as usize * size[0] + i as usize) as u32, Layer { height, ..layer }));
        }
    };
    let span = |k: usize| {
        let low = a[k].min(b[k]).min(c[k]);
        let high = a[k].max(b[k]).max(c[k]);
        let first = ((low - origin[k]) / CELL - 0.5).ceil().max(0.0) as isize;
        let last = ((high - origin[k]) / CELL - 0.5).floor().min(size[k] as f32 - 1.0) as isize;
        first..=last
    };
    let edge =
        |p: [f32; 2], q: [f32; 2], x: f32, z: f32| (q[0] - p[0]) * (z - p[1]) - (q[1] - p[1]) * (x - p[0]);
    let mut found = false;
    for k in span(1) {
        let z = origin[1] + (k as f32 + 0.5) * CELL;
        for i in span(0) {
            let x = origin[0] + (i as f32 + 0.5) * CELL;
            let w = [edge(a, b, x, z), edge(b, c, x, z), edge(c, a, x, z)];
            if w.iter().all(|&w| w >= 0.0) || w.iter().all(|&w| w <= 0.0) {
                push(i, k, height(x, z));
                found = true;
            }
        }
    }
    if !found {
        let x = (a[0] + b[0] + c[0]) / 3.0;
        let z = (a[1] + b[1] + c[1]) / 3.0;
        let i = ((x - origin[0]) / CELL).floor() as isize;
        let k = ((z - origin[1]) / CELL).floor() as isize;
        push(i, k, height(x, z));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(height: f32, x: [f32; 2], z: [f32; 2], gameplay: u8) -> [PhysicalTriangle; 2] {
        let corner = |i: usize, k: usize| [x[i], height, z[k]];
        let triangle = |v| PhysicalTriangle { v, surface: gearbox_engine::surface::ASPHALT, gameplay };
        [
            triangle([corner(0, 0), corner(0, 1), corner(1, 1)]),
            triangle([corner(0, 0), corner(1, 1), corner(1, 0)]),
        ]
    }

    fn sample(out: &[f32], bearing: usize, distance: usize) -> &[f32] {
        let at = (bearing * DISTANCES.len() + distance) * FEATURES;
        &out[at..at + FEATURES]
    }

    #[test]
    fn a_car_under_a_bridge_sees_its_road_and_a_car_on_it_sees_the_deck() {
        let mut triangles = quad(0.0, [-200.0, 200.0], [-200.0, 200.0], gameplay::NONE).to_vec();
        triangles.extend(quad(8.0, [-6.0, 6.0], [-200.0, 200.0], gameplay::TURBO));
        let ground = Ground::from_triangles(&triangles, [0.0, 0.0]);
        let mut out = vec![0.0; LEN];
        ground.ahead([0.0, 0.5, 0.0], [0.0, 1.0], &mut out);
        assert_eq!(sample(&out, 4, 0), &[1.0, -0.5 / 16.0, 1.0, 0.0, 0.0]);
        ground.ahead([0.0, 8.5, 0.0], [0.0, 1.0], &mut out);
        assert_eq!(sample(&out, 4, 0), &[1.0, -0.5 / 16.0, 1.0, 0.0, 0.25]);
        assert_eq!(sample(&out, 8, 2)[..2], [1.0, -8.5 / 16.0]);
    }

    #[test]
    fn positive_bearings_turn_toward_the_car_x_axis() {
        let triangles = quad(1.0, [5.0, 200.0], [-200.0, 200.0], gameplay::NONE);
        let ground = Ground::from_triangles(&triangles, [0.0, 0.0]);
        let seen = |forward: [f32; 2], bearing: usize, distance: usize| {
            let mut out = vec![0.0; LEN];
            ground.ahead([0.0; 3], forward, &mut out);
            sample(&out, bearing, distance)[0]
        };
        assert_eq!([seen([0.0, 1.0], 0, 3), seen([0.0, 1.0], 8, 3)], [0.0, 1.0]);
        assert_eq!([seen([0.0, -2.0], 0, 3), seen([0.0, -2.0], 8, 3)], [1.0, 0.0]);
        assert_eq!([seen([1.0, 0.0], 4, 0), seen([1.0, 0.0], 4, 4)], [0.0, 1.0]);
        assert_eq!(seen([-1.0, 0.0], 4, 4), 0.0);
    }

    #[test]
    fn empty_ground_walls_and_samples_past_the_map_read_as_nothing() {
        let mut out = vec![1.0; LEN];
        Ground::default().ahead([0.0; 3], [0.0, 1.0], &mut out);
        assert!(out.iter().all(|&v| v == 0.0));
        let wall = PhysicalTriangle {
            v: [[0.0, 0.0, 5.0], [0.0, 10.0, 5.0], [10.0, 0.0, 5.0]],
            surface: gearbox_engine::surface::ASPHALT,
            gameplay: gameplay::NONE,
        };
        assert_eq!(Ground::from_triangles(&[wall], [0.0, 0.0]), Ground::default());
        let small = Ground::from_triangles(&quad(0.0, [-4.0, 4.0], [-4.0, 4.0], gameplay::NONE), [0.0, 0.0]);
        out.fill(1.0);
        small.ahead([0.0; 3], [0.0, 1.0], &mut out);
        assert_eq!(sample(&out, 4, 0)[0], 1.0);
        assert!(DISTANCES.iter().enumerate().skip(1).all(|(d, _)| sample(&out, 4, d) == [0.0; FEATURES]));
    }

    #[test]
    fn close_surfaces_merge_and_a_column_keeps_its_highest_layers() {
        let triangles: Vec<_> = [0.0, 0.5, 3.0, 6.0, 9.0, 12.0]
            .into_iter()
            .flat_map(|h| quad(h, [-10.0, 10.0], [-10.0, 10.0], gameplay::NONE))
            .collect();
        let ground = Ground::from_triangles(&triangles, [0.0, 0.0]);
        let heights: Vec<f32> = ground.column(1.0, 1.0).iter().map(|l| l.height).collect();
        assert_eq!(heights, [12.0, 9.0, 6.0, 3.0]);
        let low = Ground::from_triangles(&triangles[..4], [0.0, 0.0]);
        assert_eq!(low.column(1.0, 1.0).iter().map(|l| l.height).collect::<Vec<_>>(), [0.5]);
        assert_eq!(low.under(1.0, 1.0, 0.4), None);
    }
}
