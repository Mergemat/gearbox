use std::sync::Arc;
use std::time::Instant;

use anyhow::{Context, Result};
use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3};
use wgpu::util::DeviceExt;

use super::compositor::{ImageReadback, Nv12Pass, SurfaceRing};
use super::mesh::{DrawList, Instance, LineSetId, LineVertex, Mesh, MeshId, Pass, TextureId, Vertex};

const COLOR: wgpu::TextureFormat = wgpu::TextureFormat::Bgra8UnormSrgb;
const COLOR_RAW: wgpu::TextureFormat = wgpu::TextureFormat::Bgra8Unorm;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
const SAMPLES: u32 = 4;

pub struct Gpu {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub adapter: String,
    pub timestamps: bool,
    pub bc: bool,
}

impl Gpu {
    pub fn new() -> Result<Self> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::METAL,
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        });
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            ..Default::default()
        }))
        .context("no Metal adapter")?;
        let timestamps = adapter.features().contains(wgpu::Features::TIMESTAMP_QUERY);
        let bc = adapter.features().contains(wgpu::Features::TEXTURE_COMPRESSION_BC);
        let mut required_features = wgpu::Features::empty();
        if timestamps {
            required_features |= wgpu::Features::TIMESTAMP_QUERY;
        }
        if bc {
            required_features |= wgpu::Features::TEXTURE_COMPRESSION_BC;
        }
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("editor"),
            required_features,
            required_limits: wgpu::Limits { max_buffer_size: 1 << 30, ..wgpu::Limits::default() },
            ..Default::default()
        }))?;
        Ok(Self { device, queue, adapter: adapter.get_info().name, timestamps, bc })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Composite {
    Surface,
    Image,
}

pub enum Output {
    Surface(objc2_core_foundation::CFRetained<objc2_core_video::CVPixelBuffer>),
    Image(Arc<gpui::RenderImage>),
}

pub struct View {
    pub view_proj: Mat4,
    pub eye: glam::DVec3,
    pub fog: [f32; 2],
    pub exposure: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FrameTimes {
    pub encode_us: f32,
    pub wait_us: f32,
    pub gpu_scene_us: Option<f32>,
    pub gpu_composite_us: Option<f32>,
    pub readback_us: f32,
    pub triangles: u64,
    pub skipped: bool,
    pub pending: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    light: [f32; 4],
    fog: [f32; 4],
    sky: [f32; 4],
    origin: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct RawInstance {
    model: [[f32; 4]; 4],
    tint: [f32; 4],
}

const INSTANCE: u64 = std::mem::size_of::<RawInstance>() as u64;

struct GpuMesh {
    vertices: Arc<wgpu::Buffer>,
    indices: Arc<wgpu::Buffer>,
    first: u32,
    count: u32,
}

struct DynBuf {
    buf: Option<wgpu::Buffer>,
    cap: u64,
    usage: wgpu::BufferUsages,
    label: &'static str,
}

impl DynBuf {
    fn new(label: &'static str, usage: wgpu::BufferUsages) -> Self {
        Self { buf: None, cap: 0, usage: usage | wgpu::BufferUsages::COPY_DST, label }
    }
    fn reserve(&mut self, d: &wgpu::Device, len: u64) -> bool {
        if self.cap >= len && self.buf.is_some() {
            return false;
        }
        self.cap = len.next_power_of_two().max(4096);
        self.buf = Some(d.create_buffer(&wgpu::BufferDescriptor {
            label: Some(self.label),
            size: self.cap,
            usage: self.usage,
            mapped_at_creation: false,
        }));
        true
    }
    fn upload(&mut self, d: &wgpu::Device, q: &wgpu::Queue, bytes: &[u8]) -> bool {
        if bytes.is_empty() {
            return false;
        }
        let grew = self.reserve(d, bytes.len() as u64);
        q.write_buffer(self.buf.as_ref().unwrap(), 0, bytes);
        grew
    }
}

struct Targets {
    size: [u32; 2],
    msaa: wgpu::TextureView,
    resolve: wgpu::Texture,
    resolve_view: wgpu::TextureView,
    resolve_raw: wgpu::TextureView,
    depth: wgpu::TextureView,
}

struct Timestamps {
    set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    read: wgpu::Buffer,
    period_ns: f32,
}

pub struct Renderer {
    pub gpu: Gpu,
    mesh: wgpu::RenderPipeline,
    cutout: wgpu::RenderPipeline,
    decal: wgpu::RenderPipeline,
    glass: wgpu::RenderPipeline,
    holo: wgpu::RenderPipeline,
    glow: wgpu::RenderPipeline,
    overlay: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    overlay_lines: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    globals_layout: wgpu::BindGroupLayout,
    globals_bind: wgpu::BindGroup,
    store: DynBuf,
    ids: DynBuf,
    statics: Vec<RawInstance>,
    static_keys: Vec<(Pass, MeshId, TextureId)>,
    tex_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    textures: Vec<wgpu::BindGroup>,
    meshes: Vec<Option<GpuMesh>>,
    line_sets: Vec<Option<(wgpu::Buffer, u32)>>,
    dyn_tris: DynBuf,
    dyn_glass: DynBuf,
    dyn_overlay: DynBuf,
    dyn_lines: DynBuf,
    dyn_overlay_lines: DynBuf,
    targets: Option<Targets>,
    pub composite: Composite,
    ring: Option<SurfaceRing>,
    readback: Option<ImageReadback>,
    nv12: Nv12Pass,
    timestamps: Option<Timestamps>,
    pub sky: [u8; 3],
    started: Instant,
    pub pipelined: bool,
    seq: u64,
    done: Arc<std::sync::atomic::AtomicU64>,
    slot_seq: Vec<u64>,
    worker: Option<Worker>,
}

impl Renderer {
    pub fn new(gpu: Gpu, composite: Composite, sky: [u8; 3]) -> Self {
        let d = &gpu.device;
        let shader = d.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene"),
            source: wgpu::ShaderSource::Wgsl(include_str!("scene.wgsl").into()),
        });
        let globals = d.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let storage = |binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let globals_layout = d.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                storage(1),
                storage(2),
            ],
        });
        let mut store = DynBuf::new("instances", wgpu::BufferUsages::STORAGE);
        let mut ids = DynBuf::new("instance ids", wgpu::BufferUsages::STORAGE);
        store.reserve(d, 0);
        ids.reserve(d, 0);
        let globals_bind = bind_globals(d, &globals_layout, &globals, &store, &ids);
        let tex_layout = d.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("texture"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let sampler = d.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("texture"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: 8,
            ..Default::default()
        });
        let mesh_layout = d.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh"),
            bind_group_layouts: &[Some(&globals_layout), Some(&tex_layout)],
            immediate_size: 0,
        });
        let line_layout = d.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("lines"),
            bind_group_layouts: &[Some(&globals_layout)],
            immediate_size: 0,
        });
        let vattrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Snorm8x4, 2 => Unorm8x4, 3 => Float32x2];
        let lattrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Unorm8x4];
        let mesh_buffers = [Some(wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &vattrs,
        })];
        let line_buffers = [Some(wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<LineVertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &lattrs,
        })];
        let pipeline_ex = |label: &str,
                           line: bool,
                           depth_write: bool,
                           depth_test: bool,
                           blend: wgpu::BlendState,
                           fs_entry: &str,
                           bias: wgpu::DepthBiasState| {
            let (layout, vs, fs, buffers, topology): (_, _, _, &[Option<wgpu::VertexBufferLayout>], _) =
                if line {
                    (&line_layout, "vs_line", "fs_line", &line_buffers, wgpu::PrimitiveTopology::LineList)
                } else {
                    (&mesh_layout, "vs_mesh", fs_entry, &mesh_buffers, wgpu::PrimitiveTopology::TriangleList)
                };
            d.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some(vs),
                    compilation_options: Default::default(),
                    buffers,
                },
                primitive: wgpu::PrimitiveState { topology, cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH,
                    depth_write_enabled: Some(depth_write),
                    depth_compare: Some(if depth_test {
                        wgpu::CompareFunction::GreaterEqual
                    } else {
                        wgpu::CompareFunction::Always
                    }),
                    stencil: Default::default(),
                    bias,
                }),
                multisample: wgpu::MultisampleState {
                    count: SAMPLES,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: COLOR,
                        blend: Some(blend),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let pipeline = |label: &str, line: bool, depth_write: bool, depth_test: bool, blend: bool| {
            pipeline_ex(
                label,
                line,
                depth_write,
                depth_test,
                if blend { wgpu::BlendState::ALPHA_BLENDING } else { wgpu::BlendState::REPLACE },
                "fs_mesh",
                Default::default(),
            )
        };
        let additive = wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::SrcAlpha,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::Zero,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
        };
        let timestamps = gpu.timestamps.then(|| Timestamps {
            set: d.create_query_set(&wgpu::QuerySetDescriptor {
                label: Some("timestamps"),
                ty: wgpu::QueryType::Timestamp,
                count: 4,
            }),
            resolve: d.create_buffer(&wgpu::BufferDescriptor {
                label: Some("ts resolve"),
                size: 32,
                usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }),
            read: d.create_buffer(&wgpu::BufferDescriptor {
                label: Some("ts read"),
                size: 32,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            period_ns: gpu.queue.get_timestamp_period(),
        });
        let decal_bias = wgpu::DepthBiasState { constant: 8, slope_scale: 2.0, clamp: 0.0 };
        let mut r = Self {
            cutout: pipeline_ex(
                "cutout",
                false,
                true,
                true,
                wgpu::BlendState::REPLACE,
                "fs_cutout",
                Default::default(),
            ),
            decal: pipeline_ex(
                "decal",
                false,
                false,
                true,
                wgpu::BlendState::ALPHA_BLENDING,
                "fs_mesh",
                decal_bias,
            ),
            holo: pipeline_ex("holo", false, false, true, additive, "fs_holo", Default::default()),
            glow: pipeline_ex("glow", false, false, true, additive, "fs_glow", Default::default()),
            mesh: pipeline("mesh", false, true, true, false),
            glass: pipeline("glass", false, false, true, true),
            overlay: pipeline("overlay", false, false, false, true),
            lines: pipeline("lines", true, true, true, true),
            overlay_lines: pipeline("overlay lines", true, false, false, true),
            nv12: Nv12Pass::new(d),
            globals,
            globals_layout,
            globals_bind,
            store,
            ids,
            statics: Vec::new(),
            static_keys: Vec::new(),
            tex_layout,
            sampler,
            textures: Vec::new(),
            meshes: Vec::new(),
            line_sets: Vec::new(),
            dyn_tris: DynBuf::new("tris", wgpu::BufferUsages::VERTEX),
            dyn_glass: DynBuf::new("glass", wgpu::BufferUsages::VERTEX),
            dyn_overlay: DynBuf::new("overlay", wgpu::BufferUsages::VERTEX),
            dyn_lines: DynBuf::new("lines", wgpu::BufferUsages::VERTEX),
            dyn_overlay_lines: DynBuf::new("overlay lines", wgpu::BufferUsages::VERTEX),
            targets: None,
            composite,
            ring: None,
            readback: None,
            timestamps,
            sky,
            started: Instant::now(),
            pipelined: false,
            seq: 0,
            done: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            slot_seq: Vec::new(),
            worker: None,
            gpu,
        };
        r.add_texture(&[255; 4], 1, 1);
        r
    }

    pub fn add_mesh(&mut self, m: &Mesh) -> MeshId {
        self.add_mesh_parts(&m.vertices, &m.indices, &[(0, m.indices.len() as u32)])[0]
    }

    pub fn add_mesh_parts(
        &mut self,
        vertices: &[Vertex],
        indices: &[u32],
        parts: &[(u32, u32)],
    ) -> Vec<MeshId> {
        let d = &self.gpu.device;
        let vb = Arc::new(d.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh vertices"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX,
        }));
        let ib = Arc::new(d.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mesh indices"),
            contents: bytemuck::cast_slice(indices),
            usage: wgpu::BufferUsages::INDEX,
        }));
        parts
            .iter()
            .map(|&(first, count)| {
                self.insert_mesh(GpuMesh { vertices: vb.clone(), indices: ib.clone(), first, count })
            })
            .collect()
    }

    fn insert_mesh(&mut self, gm: GpuMesh) -> MeshId {
        match self.meshes.iter().position(Option::is_none) {
            Some(i) => {
                self.meshes[i] = Some(gm);
                MeshId(i as u32)
            }
            None => {
                self.meshes.push(Some(gm));
                MeshId(self.meshes.len() as u32 - 1)
            }
        }
    }

    pub fn set_statics(&mut self, items: &[Instance]) {
        self.static_keys = items.iter().map(|i| (i.pass(), i.mesh, i.texture)).collect();
        self.statics = items
            .iter()
            .map(|i| RawInstance { model: i.transform.to_cols_array_2d(), tint: i.tint })
            .collect();
        self.store.reserve(&self.gpu.device, (self.statics.len() + 64) as u64 * INSTANCE);
        self.write_statics();
    }

    fn reserve_store(&mut self, extra: usize) {
        if self.store.reserve(&self.gpu.device, (self.statics.len() + extra) as u64 * INSTANCE) {
            self.write_statics();
        }
    }

    fn write_statics(&mut self) {
        if !self.statics.is_empty() {
            self.gpu.queue.write_buffer(
                self.store.buf.as_ref().unwrap(),
                0,
                bytemuck::cast_slice(&self.statics),
            );
        }
        self.rebind();
    }

    fn rebind(&mut self) {
        self.globals_bind =
            bind_globals(&self.gpu.device, &self.globals_layout, &self.globals, &self.store, &self.ids);
    }

    pub fn add_lines(&mut self, v: &[LineVertex]) -> Option<LineSetId> {
        if v.is_empty() {
            return None;
        }
        let buf = self.gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("line set"),
            contents: bytemuck::cast_slice(v),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let set = Some((buf, v.len() as u32));
        Some(match self.line_sets.iter().position(Option::is_none) {
            Some(i) => {
                self.line_sets[i] = set;
                LineSetId(i as u32)
            }
            None => {
                self.line_sets.push(set);
                LineSetId(self.line_sets.len() as u32 - 1)
            }
        })
    }

    pub fn remove_lines(&mut self, id: LineSetId) {
        if let Some(s) = self.line_sets.get_mut(id.0 as usize) {
            *s = None;
        }
    }

    pub fn remove_mesh(&mut self, id: MeshId) {
        if let Some(m) = self.meshes.get_mut(id.0 as usize) {
            *m = None;
        }
    }

    pub fn add_texture(&mut self, rgba: &[u8], w: u32, h: u32) -> TextureId {
        self.add_texture_mips(wgpu::TextureFormat::Rgba8UnormSrgb, w, h, &[rgba])
    }

    pub fn add_texture_mips(
        &mut self,
        format: wgpu::TextureFormat,
        w: u32,
        h: u32,
        mips: &[&[u8]],
    ) -> TextureId {
        let d = &self.gpu.device;
        let data: Vec<u8> = mips.concat();
        let tex = d.create_texture_with_data(
            &self.gpu.queue,
            &wgpu::TextureDescriptor {
                label: Some("texture"),
                size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
                mip_level_count: mips.len() as u32,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            },
            wgpu::util::TextureDataOrder::LayerMajor,
            &data,
        );
        let view = tex.create_view(&Default::default());
        self.textures.push(d.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("texture"),
            layout: &self.tex_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&self.sampler) },
            ],
        }));
        TextureId(self.textures.len() as u32 - 1)
    }

    pub fn latest(&mut self) -> (Option<Output>, bool) {
        self.progress();
        let pending = self.done.load(std::sync::atomic::Ordering::Acquire) < self.seq;
        (self.newest_done(), pending)
    }

    fn progress(&self) {
        let _ = self.gpu.device.poll(wgpu::PollType::Poll);
    }

    fn newest_done(&self) -> Option<Output> {
        let done = self.done.load(std::sync::atomic::Ordering::Acquire);
        let ring = self.ring.as_ref()?;
        let (i, _) = self
            .slot_seq
            .iter()
            .enumerate()
            .filter(|(_, s)| **s != 0 && **s <= done)
            .max_by_key(|(_, s)| **s)?;
        Some(Output::Surface(ring.slot(i).buffer.clone()))
    }

    fn ensure_targets(&mut self, size: [u32; 2]) -> Result<()> {
        if self.targets.as_ref().is_some_and(|t| t.size == size) {
            return Ok(());
        }
        let d = &self.gpu.device;
        let tex = |label, format, samples, usage, view_formats: &[wgpu::TextureFormat]| {
            d.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d { width: size[0], height: size[1], depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: samples,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats,
            })
        };
        let resolve = tex(
            "resolve",
            COLOR,
            1,
            wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            &[COLOR_RAW],
        );
        self.targets = Some(Targets {
            size,
            msaa: tex("msaa", COLOR, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT, &[])
                .create_view(&Default::default()),
            resolve_view: resolve.create_view(&Default::default()),
            resolve_raw: resolve
                .create_view(&wgpu::TextureViewDescriptor { format: Some(COLOR_RAW), ..Default::default() }),
            resolve,
            depth: tex("depth", DEPTH, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT, &[])
                .create_view(&Default::default()),
        });
        match self.composite {
            Composite::Surface => {
                d.poll(wgpu::PollType::Wait { submission_index: None, timeout: None })
                    .map_err(|e| anyhow::anyhow!("{e}"))?;
                let ring = SurfaceRing::new(d, size)?;
                self.slot_seq = vec![0; ring.len()];
                self.ring = Some(ring);
            }
            Composite::Image => self.readback = Some(ImageReadback::new(d, size)),
        }
        Ok(())
    }

    pub fn render(&mut self, size: [u32; 2], view: &View, draw: &DrawList) -> Result<(Output, FrameTimes)> {
        let t0 = Instant::now();
        let size = [size[0].clamp(2, 8192) & !1, size[1].clamp(2, 8192) & !1];
        self.sync_worker();
        self.ensure_targets(size)?;
        let mut times = FrameTimes::default();
        let pipelined = self.pipelined && self.composite == Composite::Surface;
        if pipelined {
            use std::sync::atomic::Ordering;
            self.progress();
            let done = self.done.load(Ordering::Acquire);
            if self.seq.saturating_sub(done) >= 2
                && let Some(out) = self.newest_done()
            {
                times.encode_us = t0.elapsed().as_secs_f32() * 1e6;
                times.skipped = true;
                times.pending = true;
                return Ok((out, times));
            }
        }
        let q = &self.gpu.queue;
        let lin = |c: u8| (c as f32 / 255.0).powf(2.2);
        let g = Globals {
            view_proj: view.view_proj.to_cols_array_2d(),
            eye: [0.0, 0.0, 0.0, 1.0],
            light: Vec3::new(0.38, 0.82, 0.30).normalize().extend(view.exposure).into(),
            fog: [view.fog[0], view.fog[1], 0.85, self.started.elapsed().as_secs_f32() % 3600.0],
            sky: [lin(self.sky[0]), lin(self.sky[1]), lin(self.sky[2]), 1.0],
            origin: [view.eye.x as f32, view.eye.y as f32, view.eye.z as f32, 0.0],
        };
        q.write_buffer(&self.globals, 0, bytemuck::bytes_of(&g));

        let mut order: Vec<&Instance> = draw
            .instances
            .iter()
            .filter(|i| self.meshes.get(i.mesh.0 as usize).is_some_and(Option::is_some))
            .collect();
        order.sort_by_key(|i| (i.pass(), i.mesh, i.texture));
        let base = self.statics.len() as u32;
        let mut raw: Vec<RawInstance> = order
            .iter()
            .map(|i| RawInstance { model: i.transform.to_cols_array_2d(), tint: i.tint })
            .collect();
        raw.push(RawInstance { model: Mat4::IDENTITY.to_cols_array_2d(), tint: [1.0; 4] });
        let mut ids: Vec<u32> = Vec::with_capacity(draw.statics.len() + raw.len());
        let mut groups: Vec<((Pass, MeshId, TextureId), u32, u32)> = Vec::new();
        let (mut a, mut b) = (0, 0);
        loop {
            let ka = draw.statics.get(a).and_then(|&s| self.static_keys.get(s as usize));
            let kb = order.get(b).map(|i| (i.pass(), i.mesh, i.texture));
            let (key, id) = match (ka, kb) {
                (Some(x), Some(y)) if *x <= y => {
                    a += 1;
                    (*x, draw.statics[a - 1])
                }
                (Some(x), None) => {
                    a += 1;
                    (*x, draw.statics[a - 1])
                }
                (_, Some(y)) => {
                    b += 1;
                    (y, base + b as u32 - 1)
                }
                (None, None) => break,
            };
            let at = ids.len() as u32;
            ids.push(id);
            match groups.last_mut() {
                Some(g) if g.0 == key => g.2 = at + 1,
                _ => groups.push((key, at, at + 1)),
            }
        }
        ids.push(base + raw.len() as u32 - 1);
        let identity = ids.len() as u32 - 1;
        self.reserve_store(raw.len());
        let (d, q) = (&self.gpu.device, &self.gpu.queue);
        q.write_buffer(self.store.buf.as_ref().unwrap(), base as u64 * INSTANCE, bytemuck::cast_slice(&raw));
        if self.ids.upload(d, q, bytemuck::cast_slice(&ids)) {
            self.rebind();
        }
        let (d, q) = (&self.gpu.device, &self.gpu.queue);
        self.dyn_tris.upload(d, q, bytemuck::cast_slice(&draw.tris));
        self.dyn_glass.upload(d, q, bytemuck::cast_slice(&draw.glass));
        self.dyn_overlay.upload(d, q, bytemuck::cast_slice(&draw.overlay));
        self.dyn_lines.upload(d, q, bytemuck::cast_slice(&draw.lines));
        self.dyn_overlay_lines.upload(d, q, bytemuck::cast_slice(&draw.overlay_lines));

        let mut draws: Vec<Draw> = Vec::with_capacity(groups.len() + 8);
        let mut tris = 0u64;
        for &((kind, m, tex), k, e) in &groups {
            let Some(Some(gm)) = self.meshes.get(m.0 as usize) else { continue };
            let pipe = match kind {
                Pass::Cutout => &self.cutout,
                Pass::Decal => &self.decal,
                Pass::Glass => &self.glass,
                Pass::Holo => &self.holo,
                Pass::Glow => &self.glow,
                _ => &self.mesh,
            };
            draws.push(Draw::Indexed {
                pipe: pipe.clone(),
                tex: self.textures.get(tex.0 as usize).unwrap_or(&self.textures[0]).clone(),
                vb: gm.vertices.clone(),
                ib: gm.indices.clone(),
                idx: gm.first..gm.first + gm.count,
                inst: k..e,
            });
            tris += (gm.count / 3) as u64 * (e - k) as u64;
        }
        let blank = &self.textures[0];
        let imm = |draws: &mut Vec<Draw>, p: &wgpu::RenderPipeline, b: &DynBuf, n: usize, inst: bool| {
            if n > 0
                && let Some(buf) = &b.buf
            {
                draws.push(Draw::Plain {
                    pipe: p.clone(),
                    tex: inst.then(|| blank.clone()),
                    vb: buf.clone(),
                    n: n as u32,
                    inst: if inst { identity..identity + 1 } else { 0..1 },
                });
            }
        };
        imm(&mut draws, &self.mesh, &self.dyn_tris, draw.tris.len(), true);
        imm(&mut draws, &self.lines, &self.dyn_lines, draw.lines.len(), false);
        for id in &draw.line_sets {
            if let Some(Some((buf, n))) = self.line_sets.get(id.0 as usize) {
                draws.push(Draw::Plain {
                    pipe: self.lines.clone(),
                    tex: None,
                    vb: buf.clone(),
                    n: *n,
                    inst: 0..1,
                });
            }
        }
        imm(&mut draws, &self.glass, &self.dyn_glass, draw.glass.len(), true);
        imm(&mut draws, &self.overlay, &self.dyn_overlay, draw.overlay.len(), true);
        imm(&mut draws, &self.overlay_lines, &self.dyn_overlay_lines, draw.overlay_lines.len(), false);
        times.triangles = tris + (draw.tris.len() + draw.glass.len() + draw.overlay.len()) as u64 / 3;
        let t = self.targets.as_ref().unwrap();
        let job = Job {
            msaa: t.msaa.clone(),
            resolve: t.resolve_view.clone(),
            depth: t.depth.clone(),
            globals: self.globals_bind.clone(),
            sky: wgpu::Color {
                r: lin(self.sky[0]) as f64,
                g: lin(self.sky[1]) as f64,
                b: lin(self.sky[2]) as f64,
                a: 1.0,
            },
            draws,
        };
        if pipelined {
            let raw_view = t.resolve_raw.clone();
            let (i, slot) = self.ring.as_mut().unwrap().advance();
            let planes = (slot.y.clone(), slot.cbcr.clone());
            self.seq += 1;
            self.slot_seq[i] = self.seq;
            let seq = self.seq;
            self.worker().send(Submit { job, raw: raw_view, planes, seq });
            times.encode_us = t0.elapsed().as_secs_f32() * 1e6;
            let out = match self.newest_done() {
                Some(o) => o,
                None => {
                    let tw = Instant::now();
                    self.sync_worker();
                    self.gpu
                        .device
                        .poll(wgpu::PollType::Wait { submission_index: None, timeout: None })
                        .map_err(|e| anyhow::anyhow!("{e}"))?;
                    self.done.fetch_max(self.seq, std::sync::atomic::Ordering::AcqRel);
                    times.wait_us = tw.elapsed().as_secs_f32() * 1e6;
                    self.newest_done().context("no completed frame")?
                }
            };
            times.pending = self.done.load(std::sync::atomic::Ordering::Acquire) < self.seq;
            return Ok((out, times));
        }
        let (d, q) = (&self.gpu.device, &self.gpu.queue);
        let mut enc = d.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("frame") });
        record(&mut enc, &job, self.timestamps.as_ref().map(|ts| &ts.set));
        let output_slot = match self.composite {
            Composite::Surface => {
                let (i, slot) = self.ring.as_mut().unwrap().advance();
                self.nv12.encode(
                    d,
                    &mut enc,
                    &t.resolve_raw,
                    (&slot.y, &slot.cbcr),
                    self.timestamps.as_ref().map(|ts| (&ts.set, 2)),
                );
                self.seq += 1;
                self.slot_seq[i] = self.seq;
                Some(slot.buffer.clone())
            }
            Composite::Image => {
                self.readback.as_ref().unwrap().encode(&mut enc, &t.resolve);
                None
            }
        };
        if let Some(ts) = self.timestamps.as_ref() {
            let n = if output_slot.is_some() { 4 } else { 2 };
            enc.resolve_query_set(&ts.set, 0..n, &ts.resolve, 0);
            enc.copy_buffer_to_buffer(&ts.resolve, 0, &ts.read, 0, 32);
        }
        q.submit(Some(enc.finish()));
        if let Some(ts) = self.timestamps.as_ref() {
            ts.read.slice(..).map_async(wgpu::MapMode::Read, |_| {});
        }
        times.encode_us = t0.elapsed().as_secs_f32() * 1e6;
        let tw = Instant::now();
        d.poll(wgpu::PollType::Wait { submission_index: None, timeout: None })
            .map_err(|e| anyhow::anyhow!("{e}"))?;
        times.wait_us = tw.elapsed().as_secs_f32() * 1e6;
        if let Some(ts) = self.timestamps.as_ref() {
            {
                let v = ts.read.slice(..).get_mapped_range().map_err(|e| anyhow::anyhow!("{e:?}"))?;
                let t: &[u64] = bytemuck::cast_slice(&v);
                let us = |a: u64, b: u64| b.saturating_sub(a) as f32 * ts.period_ns / 1000.0;
                times.gpu_scene_us = Some(us(t[0], t[1]));
                if output_slot.is_some() {
                    times.gpu_composite_us = Some(us(t[2], t[3]));
                }
            }
            ts.read.unmap();
        }
        let out = match output_slot {
            Some(b) => Output::Surface(b),
            None => {
                let tr = Instant::now();
                let img = self.readback.as_ref().unwrap().read(d)?;
                times.readback_us = tr.elapsed().as_secs_f32() * 1e6;
                Output::Image(img)
            }
        };
        Ok((out, times))
    }

    fn worker(&mut self) -> &Worker {
        if self.worker.is_none() {
            self.worker = Some(Worker::spawn(
                self.gpu.device.clone(),
                self.gpu.queue.clone(),
                self.nv12.clone(),
                self.done.clone(),
            ));
        }
        self.worker.as_ref().unwrap()
    }

    fn sync_worker(&mut self) {
        if let Some(w) = &self.worker {
            w.wait();
        }
    }
}

enum Draw {
    Indexed {
        pipe: wgpu::RenderPipeline,
        tex: wgpu::BindGroup,
        vb: Arc<wgpu::Buffer>,
        ib: Arc<wgpu::Buffer>,
        idx: std::ops::Range<u32>,
        inst: std::ops::Range<u32>,
    },
    Plain {
        pipe: wgpu::RenderPipeline,
        tex: Option<wgpu::BindGroup>,
        vb: wgpu::Buffer,
        n: u32,
        inst: std::ops::Range<u32>,
    },
}

struct Job {
    msaa: wgpu::TextureView,
    resolve: wgpu::TextureView,
    depth: wgpu::TextureView,
    globals: wgpu::BindGroup,
    sky: wgpu::Color,
    draws: Vec<Draw>,
}

fn record(enc: &mut wgpu::CommandEncoder, job: &Job, timestamps: Option<&wgpu::QuerySet>) {
    let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some("scene"),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view: &job.msaa,
            depth_slice: None,
            resolve_target: Some(&job.resolve),
            ops: wgpu::Operations { load: wgpu::LoadOp::Clear(job.sky), store: wgpu::StoreOp::Discard },
        })],
        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
            view: &job.depth,
            depth_ops: Some(wgpu::Operations {
                load: wgpu::LoadOp::Clear(0.0),
                store: wgpu::StoreOp::Discard,
            }),
            stencil_ops: None,
        }),
        timestamp_writes: timestamps.map(|set| wgpu::RenderPassTimestampWrites {
            query_set: set,
            beginning_of_pass_write_index: Some(0),
            end_of_pass_write_index: Some(1),
        }),
        occlusion_query_set: None,
        multiview_mask: None,
    });
    pass.set_bind_group(0, &job.globals, &[]);
    for d in &job.draws {
        match d {
            Draw::Indexed { pipe, tex, vb, ib, idx, inst } => {
                pass.set_pipeline(pipe);
                pass.set_bind_group(1, tex, &[]);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(idx.clone(), 0, inst.clone());
            }
            Draw::Plain { pipe, tex, vb, n, inst } => {
                pass.set_pipeline(pipe);
                if let Some(t) = tex {
                    pass.set_bind_group(1, t, &[]);
                }
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.draw(0..*n, inst.clone());
            }
        }
    }
}

struct Submit {
    job: Job,
    raw: wgpu::TextureView,
    planes: (wgpu::TextureView, wgpu::TextureView),
    seq: u64,
}

struct Worker {
    tx: std::sync::mpsc::Sender<Submit>,
    busy: Arc<(std::sync::Mutex<bool>, std::sync::Condvar)>,
}

impl Worker {
    fn spawn(
        d: wgpu::Device,
        q: wgpu::Queue,
        nv12: Nv12Pass,
        done: Arc<std::sync::atomic::AtomicU64>,
    ) -> Self {
        let (tx, rx) = std::sync::mpsc::channel::<Submit>();
        let busy = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
        let state = busy.clone();
        std::thread::Builder::new()
            .name("render".into())
            .spawn(move || {
                while let Ok(s) = rx.recv() {
                    let mut enc =
                        d.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("frame") });
                    record(&mut enc, &s.job, None);
                    nv12.encode(&d, &mut enc, &s.raw, (&s.planes.0, &s.planes.1), None);
                    q.submit(Some(enc.finish()));
                    let (done, seq) = (done.clone(), s.seq);
                    q.on_submitted_work_done(move || {
                        done.fetch_max(seq, std::sync::atomic::Ordering::AcqRel);
                    });
                    drop(s.job);
                    let (m, cv) = &*state;
                    *m.lock().unwrap() = false;
                    cv.notify_all();
                }
            })
            .expect("spawning the render thread");
        Self { tx, busy }
    }

    fn send(&self, s: Submit) {
        *self.busy.0.lock().unwrap() = true;
        let _ = self.tx.send(s);
    }

    fn wait(&self) {
        let (m, cv) = &*self.busy;
        drop(cv.wait_while(m.lock().unwrap(), |b| *b).unwrap());
    }
}

fn bind_globals(
    d: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    globals: &wgpu::Buffer,
    store: &DynBuf,
    ids: &DynBuf,
) -> wgpu::BindGroup {
    d.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("globals"),
        layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: store.buf.as_ref().unwrap().as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: ids.buf.as_ref().unwrap().as_entire_binding() },
        ],
    })
}
