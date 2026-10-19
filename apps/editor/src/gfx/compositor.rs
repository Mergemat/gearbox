use std::ptr::NonNull;
use std::sync::Arc;

use anyhow::{Context, Result, ensure};
use objc2_core_foundation::{CFBoolean, CFDictionary, CFRetained, CFString, CFType};
use objc2_core_video::{
    CVPixelBuffer, CVPixelBufferCreate, CVPixelBufferGetIOSurface, kCVPixelBufferIOSurfacePropertiesKey,
    kCVPixelBufferMetalCompatibilityKey, kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, kCVReturnSuccess,
};
use objc2_metal::{MTLDevice, MTLPixelFormat, MTLTextureDescriptor, MTLTextureType, MTLTextureUsage};

pub struct Slot {
    pub buffer: CFRetained<CVPixelBuffer>,
    pub y: wgpu::TextureView,
    pub cbcr: wgpu::TextureView,
}

pub struct SurfaceRing {
    slots: Vec<Slot>,
    next: usize,
}

const RING: usize = 4;

impl SurfaceRing {
    pub fn new(device: &wgpu::Device, size: [u32; 2]) -> Result<Self> {
        let slots = (0..RING).map(|_| slot(device, size)).collect::<Result<_>>()?;
        Ok(Self { slots, next: 0 })
    }

    pub fn advance(&mut self) -> (usize, &Slot) {
        let i = self.next;
        self.next = (self.next + 1) % self.slots.len();
        (i, &self.slots[i])
    }

    pub fn slot(&self, i: usize) -> &Slot {
        &self.slots[i]
    }

    pub fn len(&self) -> usize {
        self.slots.len()
    }
}

fn slot(device: &wgpu::Device, [w, h]: [u32; 2]) -> Result<Slot> {
    let io_props = CFDictionary::<CFString, CFType>::empty();
    let keys: [&CFString; 2] =
        unsafe { [kCVPixelBufferIOSurfacePropertiesKey, kCVPixelBufferMetalCompatibilityKey] };
    let values: [&CFType; 2] = [&io_props, CFBoolean::new(true)];
    let attrs = CFDictionary::<CFString, CFType>::from_slices(&keys, &values);
    let mut out: *mut CVPixelBuffer = std::ptr::null_mut();
    let r = unsafe {
        CVPixelBufferCreate(
            None,
            w as usize,
            h as usize,
            kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
            Some(attrs.as_opaque()),
            NonNull::from(&mut out),
        )
    };
    ensure!(r == kCVReturnSuccess, "CVPixelBufferCreate {w}x{h}: {r}");
    let buffer = unsafe { CFRetained::from_raw(NonNull::new(out).context("null pixel buffer")?) };
    let surface = CVPixelBufferGetIOSurface(Some(&buffer)).context("pixel buffer has no IOSurface")?;
    let hal = unsafe { device.as_hal::<wgpu::hal::api::Metal>() }.context("not a Metal device")?;
    let raw = hal.raw_device().clone();
    drop(hal);
    let plane = |index: usize,
                 mtl: MTLPixelFormat,
                 format: wgpu::TextureFormat,
                 pw: u32,
                 ph: u32|
     -> Result<wgpu::TextureView> {
        let desc = unsafe {
            MTLTextureDescriptor::texture2DDescriptorWithPixelFormat_width_height_mipmapped(
                mtl,
                pw as usize,
                ph as usize,
                false,
            )
        };
        desc.setUsage(MTLTextureUsage::RenderTarget | MTLTextureUsage::ShaderRead);
        let tex = raw
            .newTextureWithDescriptor_iosurface_plane(&desc, &surface, index)
            .context("IOSurface plane texture")?;
        let hal_tex = unsafe {
            wgpu::hal::metal::Device::texture_from_raw(
                tex,
                format,
                MTLTextureType::Type2D,
                1,
                1,
                wgpu::hal::CopyExtent { width: pw, height: ph, depth: 1 },
                None,
            )
        };
        let t = unsafe {
            device.create_texture_from_hal::<wgpu::hal::api::Metal>(
                hal_tex,
                &wgpu::TextureDescriptor {
                    label: Some("nv12 plane"),
                    size: wgpu::Extent3d { width: pw, height: ph, depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                    view_formats: &[],
                },
                wgpu::TextureUses::UNINITIALIZED,
            )
        };
        Ok(t.create_view(&Default::default()))
    };
    let y = plane(0, MTLPixelFormat::R8Unorm, wgpu::TextureFormat::R8Unorm, w, h)?;
    let cbcr =
        plane(1, MTLPixelFormat::RG8Unorm, wgpu::TextureFormat::Rg8Unorm, w.div_ceil(2), h.div_ceil(2))?;
    Ok(Slot { buffer, y, cbcr })
}

pub struct Nv12Pass {
    layout: wgpu::BindGroupLayout,
    y: wgpu::RenderPipeline,
    cbcr: wgpu::RenderPipeline,
}

impl Nv12Pass {
    pub fn new(d: &wgpu::Device) -> Self {
        let shader = d.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("nv12"),
            source: wgpu::ShaderSource::Wgsl(include_str!("nv12.wgsl").into()),
        });
        let layout = d.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("nv12"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            }],
        });
        let pl = d.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("nv12"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipe = |entry: &str, format: wgpu::TextureFormat| {
            d.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(entry),
                layout: Some(&pl),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs"),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(entry),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: None,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        Self {
            y: pipe("fs_y", wgpu::TextureFormat::R8Unorm),
            cbcr: pipe("fs_cbcr", wgpu::TextureFormat::Rg8Unorm),
            layout,
        }
    }

    pub fn encode(
        &self,
        d: &wgpu::Device,
        enc: &mut wgpu::CommandEncoder,
        src: &wgpu::TextureView,
        slot: &Slot,
        timestamps: Option<(&wgpu::QuerySet, u32)>,
    ) {
        let bind = d.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("nv12"),
            layout: &self.layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(src),
            }],
        });
        for (k, (pipe, view)) in [(&self.y, &slot.y), (&self.cbcr, &slot.cbcr)].into_iter().enumerate() {
            let ts = timestamps.map(|(q, base)| wgpu::RenderPassTimestampWrites {
                query_set: q,
                beginning_of_pass_write_index: (k == 0).then_some(base),
                end_of_pass_write_index: (k == 1).then_some(base + 1),
            });
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("nv12"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: ts,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(pipe);
            pass.set_bind_group(0, &bind, &[]);
            pass.draw(0..3, 0..1);
        }
    }
}

pub struct ImageReadback {
    buffer: wgpu::Buffer,
    size: [u32; 2],
    stride: u32,
}

impl ImageReadback {
    pub fn new(d: &wgpu::Device, size: [u32; 2]) -> Self {
        let stride = (size[0] * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
        let buffer = d.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: (stride * size[1]) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        Self { buffer, size, stride }
    }

    pub fn encode(&self, enc: &mut wgpu::CommandEncoder, src: &wgpu::Texture) {
        enc.copy_texture_to_buffer(
            src.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &self.buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(self.stride),
                    rows_per_image: None,
                },
            },
            wgpu::Extent3d { width: self.size[0], height: self.size[1], depth_or_array_layers: 1 },
        );
    }

    pub fn read(&self, d: &wgpu::Device) -> Result<Arc<gpui::RenderImage>> {
        let slice = self.buffer.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        d.poll(wgpu::PollType::Wait { submission_index: None, timeout: None })
            .map_err(|e| anyhow::anyhow!("{e}"))?;
        let [w, h] = self.size;
        let mut px = Vec::with_capacity((w * h * 4) as usize);
        {
            let data = slice.get_mapped_range().map_err(|e| anyhow::anyhow!("{e:?}"))?;
            for row in 0..h as usize {
                px.extend_from_slice(&data[row * self.stride as usize..][..w as usize * 4]);
            }
        }
        self.buffer.unmap();
        let img = image::RgbaImage::from_raw(w, h, px).context("image size")?;
        Ok(Arc::new(gpui::RenderImage::new(smallvec::smallvec![image::Frame::new(img)])))
    }
}
