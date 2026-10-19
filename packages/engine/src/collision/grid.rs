use crate::Vec3;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ByteGrid {
    pub scale: Vec3,
    pub offset: Vec3,
    pub size: [u32; 3],
    pub default: u32,
    pub data: Vec<u8>,
}
impl ByteGrid {
    pub fn sample(&self, position: Vec3) -> u32 {
        let x = crate::math::cvtt(position.x * self.scale.x + self.offset.x) as u32;
        let y = crate::math::cvtt(position.y * self.scale.y + self.offset.y) as u32;
        let z = crate::math::cvtt(position.z * self.scale.z + self.offset.z) as u32;
        let [nx, ny, nz] = self.size;
        if x < nx && y < ny && z < nz {
            let index = x.wrapping_mul(ny).wrapping_add(y).wrapping_mul(nz).wrapping_add(z);
            self.data[index as usize] as u32
        } else {
            self.default
        }
    }
}
