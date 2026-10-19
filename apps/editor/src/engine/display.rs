pub use gearbox_engine::consumer::PhysicalTriangle as Tri;

#[derive(Clone, Debug, Default)]
pub struct World {
    pub tris: Vec<Tri>,
    pub min: [f32; 3],
    pub max: [f32; 3],
}

pub fn extract(session: &gearbox_engine::Session) -> World {
    let tris = session.static_triangles();
    let (mut min, mut max) = ([f32::MAX; 3], [f32::MIN; 3]);
    for tri in &tris {
        for vertex in tri.v {
            for axis in 0..3 {
                min[axis] = min[axis].min(vertex[axis]);
                max[axis] = max[axis].max(vertex[axis]);
            }
        }
    }
    if tris.is_empty() {
        (min, max) = ([0.0; 3], [0.0; 3]);
    }
    World { tris, min, max }
}
