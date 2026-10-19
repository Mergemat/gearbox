use gearbox_engine::Session;

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize)]
pub struct Hit {
    pub point: [f32; 3],
    pub normal: [f32; 3],
    pub surface: u8,
    pub material: u16,
    pub gameplay: u8,
    pub item: u64,
    pub triangle: u32,
    pub fraction: f32,
}
pub fn pick(session: &mut Session, origin: [f32; 3], direction: [f32; 3]) -> Option<Hit> {
    let hit = session.pick_static(origin.into(), direction.into())?;
    Some(Hit {
        point: hit.point.to_array(),
        normal: hit.normal.to_array(),
        surface: hit.material as u8,
        material: hit.material,
        gameplay: (hit.material >> 8) as u8,
        item: u64::from(hit.item),
        triangle: hit.triangle,
        fraction: hit.fraction,
    })
}
