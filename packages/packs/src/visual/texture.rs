use super::*;
use anyhow::ensure;
use std::io::Cursor;

#[derive(Clone, Debug)]
pub enum TextureData {
    Dds { format: image_dds::ImageFormat, mipmaps: u32, layers: u32, depth: u32, blocks: Vec<u8> },
    Encoded { format: image::ImageFormat, file: Vec<u8> },
}
#[derive(Clone, Debug)]
pub struct Texture {
    pub name: String,
    pub width: u32,
    pub height: u32,
    pub data: TextureData,
}
impl Texture {
    pub fn decode(name: &str, bytes: Vec<u8>) -> Result<Self> {
        if bytes.starts_with(b"DDS ") {
            let dds = image_dds::ddsfile::Dds::read(Cursor::new(&bytes))?;
            let surface = image_dds::Surface::from_dds(&dds)?;
            let (format, mipmaps, layers, depth) =
                (surface.image_format, surface.mipmaps, surface.layers, surface.depth);
            Ok(Self {
                name: name.into(),
                width: dds.get_width(),
                height: dds.get_height(),
                data: TextureData::Dds { format, mipmaps, layers, depth, blocks: dds.data },
            })
        } else {
            let format = image::guess_format(&bytes)?;
            ensure!(
                matches!(format, image::ImageFormat::Png | image::ImageFormat::WebP),
                "texture image format {format:?}"
            );
            let reader = image::ImageReader::with_format(Cursor::new(&bytes), format);
            let (width, height) = reader.into_dimensions()?;
            Ok(Self { name: name.into(), width, height, data: TextureData::Encoded { format, file: bytes } })
        }
    }
    pub fn rgba8(&self) -> Result<image::RgbaImage> {
        match &self.data {
            TextureData::Dds { format, mipmaps, layers, depth, blocks } => Ok(image_dds::Surface {
                width: self.width,
                height: self.height,
                depth: *depth,
                layers: *layers,
                mipmaps: *mipmaps,
                image_format: *format,
                data: blocks.as_slice(),
            }
            .decode_layers_mipmaps_rgba8(0..*layers, 0..1)?
            .to_image(0)?),
            TextureData::Encoded { format, file } => {
                Ok(image::load_from_memory_with_format(file, *format)?.to_rgba8())
            }
        }
    }
}
impl VisualCache {
    pub fn texture(&mut self, path: &str) -> Result<TextureId> {
        let key = norm(path);
        if let Some(&i) = self.texture_ids.get(&key) {
            return Ok(i);
        }
        let bytes = self.read(path)?;
        self.files.remove(&key);
        let texture = Texture::decode(path, bytes)?;
        let i = self.textures.len();
        self.textures.push(texture);
        self.texture_ids.insert(key, i);
        Ok(i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bc1_dds_upload_payload_and_rgba_agree() {
        let mut b = vec![0; 136];
        b[..4].copy_from_slice(b"DDS ");
        for (offset, value) in
            [(4, 124u32), (8, 0x81007), (12, 4), (16, 4), (20, 8), (76, 32), (80, 4), (108, 0x1000)]
        {
            b[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        b[84..88].copy_from_slice(b"DXT1");
        b[128..130].copy_from_slice(&0xf800u16.to_le_bytes());
        let t = Texture::decode("fixture.dds", b).unwrap();
        assert_eq!((t.width, t.height), (4, 4));
        let TextureData::Dds { blocks, .. } = &t.data else { panic!("DDS payload lost") };
        assert_eq!(&blocks[..2], &[0, 248]);
        let image = t.rgba8().unwrap();
        assert!(image.pixels().all(|p| p.0 == [255, 0, 0, 255]));
    }
}
