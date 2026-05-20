//! Canonical WGSL shaders and pipeline factories matching the vertex layouts
//! defined in `gpu_session`. Callers can either:
//!
//! 1. Use these directly: `let pipeline = build_mesh_pipeline(&device, ...)`.
//! 2. Read the source as a starting point and customize.
//!
//! All three pipelines share a common binding scheme:
//!   group(0) binding(0): Camera uniform (view-projection matrix)
//!   group(0) binding(1): Instance storage buffer (read-only)
//!
//! Vertex layouts are pulled from `MeshVertex/LineVertex/PointVertex::layout()`
//! so they automatically track any future field additions.

use crate::gpu_session::{LineVertex, MeshVertex, PointVertex};

/// Camera uniform — single matrix. 64 bytes.
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    pub view_proj: [[f32; 4]; 4],
}

impl Default for CameraUniform {
    fn default() -> Self {
        Self {
            view_proj: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }
}

// ============================================================================
// WGSL source (raw &str — embed in pipeline at creation time)
// ============================================================================

/// Lit mesh shader. Uses smooth per-vertex normals + per-vertex color +
/// per-instance model matrix + tint. Outputs world-space lit color.
pub const MESH_WGSL: &str = r#"
struct Camera { view_proj: mat4x4<f32>, };
struct Instance {
    model: mat4x4<f32>,
    tint: vec4<f32>,
    object_id: u32,
    flags: u32,
    _pad0: u32,
    _pad1: u32,
};

@group(0) @binding(0) var<uniform> camera: Camera;
@group(0) @binding(1) var<storage, read> instances: array<Instance>;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct VsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_main(in: VsIn, @builtin(instance_index) iid: u32) -> VsOut {
    let inst = instances[iid];
    let world = inst.model * vec4<f32>(in.position, 1.0);
    var out: VsOut;
    out.clip_pos = camera.view_proj * world;
    out.world_pos = world.xyz;
    // Transform normal (assumes uniform scale; for non-uniform, use inverse-transpose)
    out.normal = normalize((inst.model * vec4<f32>(in.normal, 0.0)).xyz);
    out.color = in.color * inst.tint;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    // Simple lambert + ambient
    let light_dir = normalize(vec3<f32>(0.4, 0.8, 0.5));
    let n_dot_l = max(dot(in.normal, light_dir), 0.0);
    let ambient = 0.25;
    let lit = ambient + (1.0 - ambient) * n_dot_l;
    return vec4<f32>(in.color.rgb * lit, in.color.a);
}
"#;

/// Unlit line shader. Per-vertex color × per-instance tint.
pub const LINE_WGSL: &str = r#"
struct Camera { view_proj: mat4x4<f32>, };
struct Instance {
    model: mat4x4<f32>,
    tint: vec4<f32>,
    object_id: u32,
    flags: u32,
    _pad0: u32,
    _pad1: u32,
};

@group(0) @binding(0) var<uniform> camera: Camera;
@group(0) @binding(1) var<storage, read> instances: array<Instance>;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct VsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VsIn, @builtin(instance_index) iid: u32) -> VsOut {
    let inst = instances[iid];
    let world = inst.model * vec4<f32>(in.position, 1.0);
    var out: VsOut;
    out.clip_pos = camera.view_proj * world;
    out.color = in.color * inst.tint;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

/// Point shader — same binding layout as the line shader; topology differs at
/// the pipeline level.
pub const POINT_WGSL: &str = LINE_WGSL;

// ============================================================================
// Bind group layout — shared across all three pipelines.
// ============================================================================

/// Build the canonical bind group layout (Camera @ binding 0,
/// instance storage @ binding 1). Use this when creating the pipeline AND
/// when building the bind group at draw time so they match.
pub fn build_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("gpu_session.bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    })
}

// ============================================================================
// Pipeline factories
// ============================================================================

fn build_pipeline(
    device: &wgpu::Device,
    label: &str,
    wgsl: &str,
    vertex_layout: wgpu::VertexBufferLayout<'static>,
    topology: wgpu::PrimitiveTopology,
    color_format: wgpu::TextureFormat,
    depth_format: Option<wgpu::TextureFormat>,
    bgl: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(&format!("{label}.shader")),
        source: wgpu::ShaderSource::Wgsl(wgsl.into()),
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(&format!("{label}.layout")),
        bind_group_layouts: &[Some(bgl)],
        immediate_size: 0,
    });
    let depth_stencil = depth_format.map(|fmt| wgpu::DepthStencilState {
        format: fmt,
        depth_write_enabled: Some(true),
        depth_compare: Some(wgpu::CompareFunction::Less),
        stencil: wgpu::StencilState::default(),
        bias: wgpu::DepthBiasState::default(),
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            buffers: &[vertex_layout],
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            targets: &[Some(wgpu::ColorTargetState {
                format: color_format,
                blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: None,
            polygon_mode: wgpu::PolygonMode::Fill,
            unclipped_depth: false,
            conservative: false,
        },
        depth_stencil,
        multisample: wgpu::MultisampleState::default(),
        multiview_mask: None,
        cache: None,
    })
}

/// Build the lit mesh render pipeline (TriangleList, MeshVertex layout).
pub fn build_mesh_pipeline(
    device: &wgpu::Device,
    color_format: wgpu::TextureFormat,
    depth_format: Option<wgpu::TextureFormat>,
    bgl: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    build_pipeline(
        device,
        "gpu_session.mesh_pipeline",
        MESH_WGSL,
        MeshVertex::layout(),
        wgpu::PrimitiveTopology::TriangleList,
        color_format,
        depth_format,
        bgl,
    )
}

/// Build the line render pipeline (LineList, LineVertex layout).
pub fn build_line_pipeline(
    device: &wgpu::Device,
    color_format: wgpu::TextureFormat,
    depth_format: Option<wgpu::TextureFormat>,
    bgl: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    build_pipeline(
        device,
        "gpu_session.line_pipeline",
        LINE_WGSL,
        LineVertex::layout(),
        wgpu::PrimitiveTopology::LineList,
        color_format,
        depth_format,
        bgl,
    )
}

/// Build the point render pipeline (PointList, PointVertex layout).
pub fn build_point_pipeline(
    device: &wgpu::Device,
    color_format: wgpu::TextureFormat,
    depth_format: Option<wgpu::TextureFormat>,
    bgl: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    build_pipeline(
        device,
        "gpu_session.point_pipeline",
        POINT_WGSL,
        PointVertex::layout(),
        wgpu::PrimitiveTopology::PointList,
        color_format,
        depth_format,
        bgl,
    )
}

#[cfg(test)]
#[path = "gpu_shaders_test.rs"]
mod gpu_shaders_test;
