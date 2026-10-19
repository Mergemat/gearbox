use crate::Vec3;

const HORIZONTAL_NORMAL_LIMIT: f32 = 0.99999;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HorizontalTriangle { pub vertices: [Vec3; 3], pub normal: Vec3 }

pub fn split_horizontal(
    vertices: [Vec3; 3], normal: Vec3, out: &mut Vec<HorizontalTriangle>, capacity: usize,
) {
    let y = |i: usize| vertices[i].y;
    let mut push = |a, b, c| {
        if out.len() < capacity { out.push(HorizontalTriangle { vertices: [a, b, c], normal }); }
    };
    if normal.y.abs() > HORIZONTAL_NORMAL_LIMIT {
        let [a, b, c] = order_y(y(0), y(1), y(2));
        push(vertices[a], vertices[b], vertices[c]);
        return;
    }
    let pair = if y(1) - y(0) == 0.0 { Some((0, 2, 1)) }
        else if y(2) - y(0) == 0.0 { Some((0, 1, 2)) }
        else if y(2) - y(1) == 0.0 { Some((1, 0, 2)) } else { None };
    if let Some((p, q, r)) = pair {
        if y(q) > y(p) { push(vertices[q], vertices[p], vertices[r]); }
        else { push(vertices[p], vertices[r], vertices[q]); }
        return;
    }
    let [a, b, c] = order_y(y(0), y(1), y(2));
    let t = (y(b) - y(c)) / (y(a) - y(c));
    let split = Vec3::new(
        vertices[c].x + (vertices[a].x - vertices[c].x) * t, y(b),
        vertices[c].z + (vertices[a].z - vertices[c].z) * t,
    );
    push(vertices[a], split, vertices[b]);
    push(split, vertices[b], vertices[c]);
}

fn order_y(a: f32, b: f32, c: f32) -> [usize; 3] {
    if a > b {
        if a > c { if b > c { [0, 1, 2] } else { [0, 2, 1] } } else { [2, 0, 1] }
    } else if c > a {
        if c > b { [2, 1, 0] } else { [1, 2, 0] }
    } else { [1, 0, 2] }
}
