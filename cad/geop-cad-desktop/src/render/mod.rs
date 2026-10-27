//! The 3-D view on the GPU: a [`Drawing`] drawn by wgpu into a texture of
//! its own — multisampled, with a depth and a stencil buffer, which egui's
//! pass has none of — and that texture drawn into egui's pass where the
//! viewport is ([`ViewCallback`]). Each face's mesh is uploaded once, by
//! its key, and drawn with every place and colour it is drawn in.

use std::{collections::HashMap, sync::Arc};

use eframe::egui_wgpu::{self, CallbackResources, CallbackTrait, ScreenDescriptor};
use eframe::wgpu::{self, util::DeviceExt};

use crate::draw::{
    Drawing, FaceInstance, FlatVertex, Layer, LineInstance, MeshVertex, PointInstance,
};

/// How many samples a pixel of the view takes.
const SAMPLES: u32 = 4;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth24PlusStencil8;
const COLOR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
/// How many triangles of meshes no face drawn uses are kept on the GPU.
const KEPT_TRIANGLES: usize = 2_000_000;

/// What the view's shaders are told.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Uniforms {
    pub view_proj: [[f32; 4]; 4],
    pub eye: [f32; 4],
    pub back: [f32; 4],
    pub viewport: [f32; 4],
    pub cut: [f32; 4],
    pub cut_on: [f32; 4],
}

/// What one frame of the view draws.
pub struct Frame {
    /// The viewport's size, in physical pixels.
    pub size: [u32; 2],
    pub uniforms: Uniforms,
    /// The backdrop, sRGB.
    pub background: [f32; 4],
    pub drawing: Arc<Drawing>,
    /// The cap of a section, as two triangles, if one is on.
    pub cap: Vec<FlatVertex>,
}

/// The view drawn into egui's pass: see the module docs.
pub struct ViewCallback(pub Arc<Frame>);

impl CallbackTrait for ViewCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen: &ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if let Some(renderer) = resources.get_mut::<Renderer>() {
            renderer.draw(device, queue, encoder, &self.0);
        }
        Vec::new()
    }

    fn paint(
        &self,
        info: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &CallbackResources,
    ) {
        let Some(renderer) = resources.get::<Renderer>() else {
            return;
        };
        let Some(target) = &renderer.target else {
            return;
        };
        let viewport = info.viewport_in_pixels();
        if viewport.width_px <= 0 || viewport.height_px <= 0 {
            return;
        }
        pass.set_viewport(
            viewport.left_px as f32,
            viewport.top_px as f32,
            viewport.width_px as f32,
            viewport.height_px as f32,
            0.0,
            1.0,
        );
        pass.set_pipeline(&renderer.blit);
        pass.set_bind_group(0, &target.blit, &[]);
        pass.draw(0..3, 0..1);
    }
}

use eframe::egui;

/// The textures drawn into, of one size.
struct Target {
    size: [u32; 2],
    color: wgpu::TextureView,
    depth: wgpu::TextureView,
    resolved: wgpu::TextureView,
    blit: wgpu::BindGroup,
}

/// A buffer that grows to what is written to it.
struct Growing {
    buffer: Option<wgpu::Buffer>,
    usage: wgpu::BufferUsages,
    label: &'static str,
}

impl Growing {
    fn new(label: &'static str, usage: wgpu::BufferUsages) -> Self {
        Self {
            buffer: None,
            usage: usage | wgpu::BufferUsages::COPY_DST,
            label,
        }
    }

    fn write(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        let fits = self
            .buffer
            .as_ref()
            .is_some_and(|b| b.size() >= data.len() as u64);
        if !fits {
            self.buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(self.label),
                size: (data.len() as u64).next_power_of_two().max(256),
                usage: self.usage,
                mapped_at_creation: false,
            }));
        }
        if let Some(buffer) = &self.buffer {
            queue.write_buffer(buffer, 0, data);
        }
    }

    fn slice(&self, bytes: u64) -> Option<wgpu::BufferSlice<'_>> {
        (bytes > 0)
            .then(|| self.buffer.as_ref().map(|b| b.slice(..bytes)))
            .flatten()
    }
}

/// A face's mesh on the GPU.
struct Mesh {
    buffer: wgpu::Buffer,
    vertices: u32,
    /// The statics it was last drawn in: what decides which to drop.
    used: u64,
}

/// Where each layer of a frame's drawing lies in the buffers written for it.
#[derive(Clone, Copy, Default)]
struct Ranges {
    triangles: (u32, u32),
    lines: (u32, u32),
    points: (u32, u32),
}

/// Pipelines, buffers and the meshes uploaded: kept in egui's
/// [`CallbackResources`] for the life of the window.
pub struct Renderer {
    uniforms: wgpu::Buffer,
    view_group: wgpu::BindGroup,
    blit_layout: wgpu::BindGroupLayout,
    blit_uniforms: wgpu::Buffer,
    sampler: wgpu::Sampler,
    blit: wgpu::RenderPipeline,
    face: wgpu::RenderPipeline,
    count_back: wgpu::RenderPipeline,
    count_front: wgpu::RenderPipeline,
    cap: wgpu::RenderPipeline,
    /// Lines and points of the part, depth tested and cut by a section.
    base_lines: wgpu::RenderPipeline,
    base_points: wgpu::RenderPipeline,
    /// Seen through, behind what is in front: datum planes and axes, the grid.
    seen_flat: wgpu::RenderPipeline,
    seen_lines: wgpu::RenderPipeline,
    /// Over everything.
    top_flat: wgpu::RenderPipeline,
    top_lines: wgpu::RenderPipeline,
    top_points: wgpu::RenderPipeline,
    target: Option<Target>,
    meshes: HashMap<String, Mesh>,
    statics_key: u64,
    statics_count: u64,
    /// The faces' instances, and per mesh drawn its range of them.
    instances: Growing,
    face_draws: Vec<(String, u32, u32)>,
    base_key: u64,
    base_lines_buffer: Growing,
    base_points_buffer: Growing,
    base_counts: (u32, u32),
    lit_lines_buffer: Growing,
    lit_points_buffer: Growing,
    lit_counts: (u32, u32),
    /// What is drawn anew every frame.
    flat: Growing,
    lines: Growing,
    points: Growing,
}

#[allow(clippy::too_many_arguments)]
fn pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    label: &str,
    (vs, fs): (&str, &str),
    buffers: &[Option<wgpu::VertexBufferLayout>],
    cull: Option<wgpu::Face>,
    depth: wgpu::DepthStencilState,
    blend: Option<wgpu::BlendState>,
    writes: wgpu::ColorWrites,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(vs),
            compilation_options: Default::default(),
            buffers,
        },
        primitive: wgpu::PrimitiveState {
            cull_mode: cull,
            ..Default::default()
        },
        depth_stencil: Some(depth),
        multisample: wgpu::MultisampleState {
            count: SAMPLES,
            ..Default::default()
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(fs),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: COLOR,
                blend,
                write_mask: writes,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

fn depth(test: wgpu::CompareFunction, write: bool) -> wgpu::DepthStencilState {
    wgpu::DepthStencilState {
        format: DEPTH,
        depth_write_enabled: Some(write),
        depth_compare: Some(test),
        stencil: Default::default(),
        bias: Default::default(),
    }
}

fn stencil(compare: wgpu::CompareFunction, op: wgpu::StencilOperation) -> wgpu::StencilState {
    let face = wgpu::StencilFaceState {
        compare,
        fail_op: op,
        depth_fail_op: op,
        pass_op: op,
    };
    wgpu::StencilState {
        front: face,
        back: face,
        read_mask: 0xff,
        write_mask: 0xff,
    }
}

const MESH_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: std::mem::size_of::<MeshVertex>() as u64,
    step_mode: wgpu::VertexStepMode::Vertex,
    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
};
const FACE_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: std::mem::size_of::<FaceInstance>() as u64,
    step_mode: wgpu::VertexStepMode::Instance,
    attributes: &wgpu::vertex_attr_array![2 => Float32x4, 3 => Float32x4, 4 => Float32x4, 5 => Float32x4],
};
const FLAT_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: std::mem::size_of::<FlatVertex>() as u64,
    step_mode: wgpu::VertexStepMode::Vertex,
    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4],
};
const LINE_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: std::mem::size_of::<LineInstance>() as u64,
    step_mode: wgpu::VertexStepMode::Instance,
    attributes: &wgpu::vertex_attr_array![
        0 => Float32x3, 1 => Float32, 2 => Float32x3, 3 => Float32,
        4 => Float32x4, 5 => Float32, 6 => Float32, 7 => Float32
    ],
};
const POINT_LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
    array_stride: std::mem::size_of::<PointInstance>() as u64,
    step_mode: wgpu::VertexStepMode::Instance,
    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32, 2 => Float32x4, 3 => Float32],
};

impl Renderer {
    /// A renderer drawing into egui's passes of the format `target`.
    pub fn new(device: &wgpu::Device, target: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("geop view"),
            source: wgpu::ShaderSource::Wgsl(include_str!("view.wgsl").into()),
        });
        let uniforms = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("geop view uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let view_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("geop view"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let view_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("geop view"),
            layout: &view_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniforms.as_entire_binding(),
            }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("geop view"),
            bind_group_layouts: &[Some(&view_layout)],
            immediate_size: 0,
        });
        let blend = Some(wgpu::BlendState::ALPHA_BLENDING);
        let all = wgpu::ColorWrites::ALL;
        let less = wgpu::CompareFunction::LessEqual;
        let always = wgpu::CompareFunction::Always;
        let face = pipeline(
            device,
            &layout,
            &shader,
            "faces",
            ("vs_face", "fs_face"),
            &[Some(MESH_LAYOUT), Some(FACE_LAYOUT)],
            None,
            depth(wgpu::CompareFunction::Less, true),
            None,
            all,
        );
        let counter = |label, cull, op| {
            let mut state = depth(always, false);
            state.stencil = stencil(always, op);
            pipeline(
                device,
                &layout,
                &shader,
                label,
                ("vs_face", "fs_count"),
                &[Some(MESH_LAYOUT), Some(FACE_LAYOUT)],
                Some(cull),
                state,
                None,
                wgpu::ColorWrites::empty(),
            )
        };
        let count_back = counter(
            "count back",
            wgpu::Face::Front,
            wgpu::StencilOperation::IncrementWrap,
        );
        let count_front = counter(
            "count front",
            wgpu::Face::Back,
            wgpu::StencilOperation::DecrementWrap,
        );
        let mut cap_state = depth(less, true);
        cap_state.stencil = stencil(
            wgpu::CompareFunction::NotEqual,
            wgpu::StencilOperation::Replace,
        );
        let cap = pipeline(
            device,
            &layout,
            &shader,
            "cap",
            ("vs_flat", "fs_flat"),
            &[Some(FLAT_LAYOUT)],
            None,
            cap_state,
            None,
            all,
        );
        let lines = |label, fs, state| {
            pipeline(
                device,
                &layout,
                &shader,
                label,
                ("vs_line", fs),
                &[Some(LINE_LAYOUT)],
                None,
                state,
                blend,
                all,
            )
        };
        let points = |label, fs, state| {
            pipeline(
                device,
                &layout,
                &shader,
                label,
                ("vs_point", fs),
                &[Some(POINT_LAYOUT)],
                None,
                state,
                blend,
                all,
            )
        };
        let flat = |label, state| {
            pipeline(
                device,
                &layout,
                &shader,
                label,
                ("vs_flat", "fs_flat"),
                &[Some(FLAT_LAYOUT)],
                None,
                state,
                blend,
                all,
            )
        };

        let blit_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("geop blit"),
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
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let blit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("geop blit"),
            bind_group_layouts: &[Some(&blit_layout)],
            immediate_size: 0,
        });
        let blit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("geop blit"),
            layout: Some(&blit_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_blit"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_blit"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target,
                    blend: None,
                    write_mask: all,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let linear: [f32; 4] = [if target.is_srgb() { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0];
        let blit_uniforms = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("geop blit"),
            contents: bytemuck::cast_slice(&linear),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("geop blit"),
            ..Default::default()
        });
        Self {
            face,
            count_back,
            count_front,
            cap,
            base_lines: lines("base lines", "fs_line_cut", depth(less, false)),
            base_points: points("base points", "fs_point_cut", depth(less, false)),
            seen_flat: flat("seen flat", depth(less, false)),
            seen_lines: lines("seen lines", "fs_line", depth(less, false)),
            top_flat: flat("top flat", depth(always, false)),
            top_lines: lines("top lines", "fs_line", depth(always, false)),
            top_points: points("top points", "fs_point", depth(always, false)),
            uniforms,
            view_group,
            blit_layout,
            blit_uniforms,
            sampler,
            blit,
            target: None,
            meshes: HashMap::new(),
            statics_key: 0,
            statics_count: 0,
            instances: Growing::new("faces", wgpu::BufferUsages::VERTEX),
            face_draws: Vec::new(),
            base_key: 0,
            base_lines_buffer: Growing::new("base lines", wgpu::BufferUsages::VERTEX),
            base_points_buffer: Growing::new("base points", wgpu::BufferUsages::VERTEX),
            base_counts: (0, 0),
            lit_lines_buffer: Growing::new("lit lines", wgpu::BufferUsages::VERTEX),
            lit_points_buffer: Growing::new("lit points", wgpu::BufferUsages::VERTEX),
            lit_counts: (0, 0),
            flat: Growing::new("flat", wgpu::BufferUsages::VERTEX),
            lines: Growing::new("lines", wgpu::BufferUsages::VERTEX),
            points: Growing::new("points", wgpu::BufferUsages::VERTEX),
        }
    }

    fn target(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if self.target.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let texture = |label, format, samples, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: wgpu::Extent3d {
                        width: size[0],
                        height: size[1],
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: samples,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let attachment = wgpu::TextureUsages::RENDER_ATTACHMENT;
        let color = texture("geop view", COLOR, SAMPLES, attachment);
        let depth = texture("geop depth", DEPTH, SAMPLES, attachment);
        let resolved = texture(
            "geop resolved",
            COLOR,
            1,
            attachment | wgpu::TextureUsages::TEXTURE_BINDING,
        );
        let blit = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("geop blit"),
            layout: &self.blit_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&resolved),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.blit_uniforms.as_entire_binding(),
                },
            ],
        });
        self.target = Some(Target {
            size,
            color,
            depth,
            resolved,
            blit,
        });
    }

    /// Upload what changed of the part: its meshes not on the GPU yet,
    /// where and how each face is drawn, its edges and sketches.
    fn statics(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, drawing: &Drawing) {
        let statics = &drawing.statics;
        if statics.key == self.statics_key {
            return;
        }
        self.statics_key = statics.key;
        self.statics_count += 1;
        let now = self.statics_count;
        let mut instances: Vec<FaceInstance> = Vec::new();
        self.face_draws.clear();
        for (key, mesh, placed) in &statics.faces {
            let entry = self.meshes.entry(key.clone()).or_insert_with(|| {
                let vertices: Vec<MeshVertex> = (mesh.positions.iter().zip(&mesh.normals))
                    .map(|(&position, &normal)| MeshVertex { position, normal })
                    .collect();
                Mesh {
                    buffer: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("face"),
                        contents: bytemuck::cast_slice(&vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    }),
                    vertices: vertices.len() as u32,
                    used: now,
                }
            });
            entry.used = now;
            let from = instances.len() as u32;
            instances.extend_from_slice(placed);
            self.face_draws
                .push((key.clone(), from, instances.len() as u32));
        }
        // The meshes no face drawn uses, the least recently used first,
        // dropped past a budget.
        let mut unused: Vec<(u64, String, u32)> = (self.meshes.iter())
            .filter(|(_, m)| m.used != now)
            .map(|(k, m)| (m.used, k.clone(), m.vertices))
            .collect();
        unused.sort();
        let mut kept: usize = unused.iter().map(|u| u.2 as usize / 3).sum();
        for (_, key, vertices) in unused {
            if kept <= KEPT_TRIANGLES {
                break;
            }
            kept -= vertices as usize / 3;
            self.meshes.remove(&key);
        }
        self.instances
            .write(device, queue, bytemuck::cast_slice(&instances));
        if statics.base_key != self.base_key {
            self.base_key = statics.base_key;
            let base = &statics.base;
            self.base_counts = (base.lines.len() as u32, base.points.len() as u32);
            self.base_lines_buffer
                .write(device, queue, bytemuck::cast_slice(&base.lines));
            self.base_points_buffer
                .write(device, queue, bytemuck::cast_slice(&base.points));
        }
        let lit = &statics.lit;
        self.lit_counts = (lit.lines.len() as u32, lit.points.len() as u32);
        self.lit_lines_buffer
            .write(device, queue, bytemuck::cast_slice(&lit.lines));
        self.lit_points_buffer
            .write(device, queue, bytemuck::cast_slice(&lit.points));
    }

    fn draw(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        frame: &Frame,
    ) {
        if frame.size[0] == 0 || frame.size[1] == 0 {
            return;
        }
        self.target(device, frame.size);
        queue.write_buffer(&self.uniforms, 0, bytemuck::bytes_of(&frame.uniforms));
        let drawing = &frame.drawing;
        self.statics(device, queue, drawing);

        // What is drawn anew: the cap, the grid, what is seen through, and
        // the layers over everything, one after another in shared buffers.
        let mut flat: Vec<FlatVertex> = frame.cap.clone();
        let mut lines: Vec<LineInstance> = Vec::new();
        let mut points: Vec<PointInstance> = Vec::new();
        let mut add = |layer: &Layer| {
            let ranges = Ranges {
                triangles: (
                    flat.len() as u32,
                    (flat.len() + layer.triangles.len()) as u32,
                ),
                lines: (lines.len() as u32, (lines.len() + layer.lines.len()) as u32),
                points: (
                    points.len() as u32,
                    (points.len() + layer.points.len()) as u32,
                ),
            };
            flat.extend_from_slice(&layer.triangles);
            lines.extend_from_slice(&layer.lines);
            points.extend_from_slice(&layer.points);
            ranges
        };
        let grid = add(&drawing.grid);
        let seen = add(&drawing.translucent);
        let tops: Vec<Ranges> = drawing.overlays.iter().map(&mut add).collect();
        self.flat.write(device, queue, bytemuck::cast_slice(&flat));
        self.lines
            .write(device, queue, bytemuck::cast_slice(&lines));
        self.points
            .write(device, queue, bytemuck::cast_slice(&points));
        let sizes = (
            std::mem::size_of_val(flat.as_slice()) as u64,
            std::mem::size_of_val(lines.as_slice()) as u64,
            std::mem::size_of_val(points.as_slice()) as u64,
        );

        let Some(target) = &self.target else {
            return;
        };
        let [r, g, b, a] = frame.background.map(f64::from);
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("geop view"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &target.color,
                depth_slice: None,
                resolve_target: Some(&target.resolved),
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color { r, g, b, a }),
                    store: wgpu::StoreOp::Discard,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &target.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(0),
                    store: wgpu::StoreOp::Discard,
                }),
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_bind_group(0, &self.view_group, &[]);
        let flat_buffer = self.flat.slice(sizes.0);
        let line_buffer = self.lines.slice(sizes.1);
        let point_buffer = self.points.slice(sizes.2);
        let draw_layer = |pass: &mut wgpu::RenderPass,
                          ranges: Ranges,
                          pipelines: (
            &wgpu::RenderPipeline,
            &wgpu::RenderPipeline,
            &wgpu::RenderPipeline,
        )| {
            if ranges.triangles.1 > ranges.triangles.0
                && let Some(buffer) = flat_buffer
            {
                pass.set_pipeline(pipelines.0);
                pass.set_vertex_buffer(0, buffer);
                pass.draw(ranges.triangles.0..ranges.triangles.1, 0..1);
            }
            if ranges.lines.1 > ranges.lines.0
                && let Some(buffer) = line_buffer
            {
                pass.set_pipeline(pipelines.1);
                pass.set_vertex_buffer(0, buffer);
                pass.draw(0..6, ranges.lines.0..ranges.lines.1);
            }
            if ranges.points.1 > ranges.points.0
                && let Some(buffer) = point_buffer
            {
                pass.set_pipeline(pipelines.2);
                pass.set_vertex_buffer(0, buffer);
                pass.draw(0..6, ranges.points.0..ranges.points.1);
            }
        };
        // The grid under everything, never hiding what is drawn after it.
        draw_layer(
            &mut pass,
            grid,
            (&self.seen_flat, &self.seen_lines, &self.top_points),
        );
        // The faces.
        let faces = self.instances.buffer.as_ref();
        let draw_faces = |pass: &mut wgpu::RenderPass, pipeline: &wgpu::RenderPipeline| {
            let Some(faces) = faces else { return };
            pass.set_pipeline(pipeline);
            pass.set_vertex_buffer(1, faces.slice(..));
            for (key, from, to) in &self.face_draws {
                let Some(mesh) = self.meshes.get(key) else {
                    continue;
                };
                pass.set_vertex_buffer(0, mesh.buffer.slice(..));
                pass.draw(0..mesh.vertices, *from..*to);
            }
        };
        draw_faces(&mut pass, &self.face);
        // A section: the solids cut open counted in the stencil, and capped.
        if frame.uniforms.cut_on[3] > 0.5 && !frame.cap.is_empty() {
            draw_faces(&mut pass, &self.count_back);
            draw_faces(&mut pass, &self.count_front);
            if let Some(buffer) = flat_buffer {
                pass.set_pipeline(&self.cap);
                pass.set_stencil_reference(0);
                pass.set_vertex_buffer(0, buffer);
                pass.draw(0..frame.cap.len() as u32, 0..1);
            }
        }
        // The edges, sketches and vertices.
        let (base_lines, base_points) = self.base_counts;
        if base_lines > 0
            && let Some(buffer) = self.base_lines_buffer.buffer.as_ref()
        {
            pass.set_pipeline(&self.base_lines);
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..6, 0..base_lines);
        }
        if base_points > 0
            && let Some(buffer) = self.base_points_buffer.buffer.as_ref()
        {
            pass.set_pipeline(&self.base_points);
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..6, 0..base_points);
        }
        // Datum planes and axes, seen through.
        draw_layer(
            &mut pass,
            seen,
            (&self.seen_flat, &self.seen_lines, &self.top_points),
        );
        // What is lit of the part, over it.
        let (lit_lines, lit_points) = self.lit_counts;
        if lit_lines > 0
            && let Some(buffer) = self.lit_lines_buffer.buffer.as_ref()
        {
            pass.set_pipeline(&self.top_lines);
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..6, 0..lit_lines);
        }
        if lit_points > 0
            && let Some(buffer) = self.lit_points_buffer.buffer.as_ref()
        {
            pass.set_pipeline(&self.top_points);
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..6, 0..lit_points);
        }
        // Datums' points and frames, the step's visuals, the gizmo.
        for ranges in tops {
            draw_layer(
                &mut pass,
                ranges,
                (&self.top_flat, &self.top_lines, &self.top_points),
            );
        }
    }
}

/// Make the renderer for egui's wgpu `state`, once, at start.
pub fn install(state: &egui_wgpu::RenderState) {
    let renderer = Renderer::new(&state.device, state.target_format);
    state.renderer.write().callback_resources.insert(renderer);
}
