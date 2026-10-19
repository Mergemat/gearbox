#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body(pub [f32; 22]);

impl From<&gearbox_engine::dyna::BodyState> for Body {
    fn from(body: &gearbox_engine::dyna::BodyState) -> Self {
        let mut values = [0.0; 22];
        values[..9].copy_from_slice(&body.rot.to_array());
        values[9..12].copy_from_slice(&body.pos.to_array());
        values[12..16].copy_from_slice(&body.quat.to_array());
        values[16..19].copy_from_slice(&body.vel.to_array());
        values[19..22].copy_from_slice(&body.ang_vel.to_array());
        Self(values)
    }
}
impl Body {
    pub fn position(&self) -> [f32; 3] {
        self.0[9..12].try_into().unwrap()
    }
    pub fn rotation(&self) -> [f32; 9] {
        self.0[..9].try_into().unwrap()
    }
    pub fn local_velocity(&self) -> [f32; 3] {
        self.0[16..19].try_into().unwrap()
    }
    pub fn local_angular_velocity(&self) -> [f32; 3] {
        self.0[19..22].try_into().unwrap()
    }
}
