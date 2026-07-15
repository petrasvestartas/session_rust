//! f32 render-side mesh extraction (Bevy-style split).
//!
//! The kernel [`Mesh`] stays f64 — that preserves C++/Python parity and `Point`
//! interop, and keeps double precision where it matters (NURBS, intersections,
//! tolerances). But the GPU only ever wants f32, and converting every frame is
//! waste. So, exactly like Bevy's render `Mesh` (which stores `Vec<[f32; 3]>`
//! attributes, separate from any math type), `Mesh::to_render()` flattens the
//! half-edge mesh once into GPU-ready f32 buffers.
//!
//! [`RenderVertex`] is `#[repr(C)] + Pod`, so the slice goes straight to a wgpu
//! vertex buffer with `bytemuck::cast_slice(&rm.vertices)` — no per-frame copy,
//! no `as f32` loop. [`Mesh::gpu_mesh`](crate::mesh::Mesh::gpu_mesh) wraps this:
//! it flattens + uploads ONCE and caches the resulting [`GpuMesh`], so the viewer
//! does zero conversion and only re-uploads when the mesh actually changes.

use bytemuck::{Pod, Zeroable};
use crate::mesh::Mesh;
use crate::point::Point;
use crate::color::Color;

/// One interleaved GPU vertex: position + normal + linear RGBA color, all f32,
/// tightly packed. Mirror this layout in the viewer's `wgpu::VertexBufferLayout`
/// (Float32x3, Float32x3, Float32x4 at offsets 0, 12, 24; stride 40).
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct RenderVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}

/// A mesh flattened to f32, ready for the GPU. `indices` are u32 into `vertices`.
#[derive(Clone, Debug, Default)]
pub struct RenderMesh {
    pub vertices: Vec<RenderVertex>,
    pub indices: Vec<u32>,
}

impl Mesh {
    /// Flatten this f64 mesh into an f32 [`RenderMesh`] for direct GPU upload.
    ///
    /// Per-vertex normals come from the `nx`/`ny`/`nz` vertex attributes when the
    /// mesh carries them (smooth-shaded surfaces); otherwise they are left zero
    /// and the shader derives flat normals from the triangle. Per-vertex color is
    /// the stored point color when the mesh's `color_mode` is `POINTCOLORS` (and they
    /// cover every vertex), else the object color. Faces use the cached `triangulation` if present, else a
    /// triangle fan — matching the viewer's original `mesh_to_vertices`.
    pub fn to_render(&self) -> RenderMesh {
        let mut keys: Vec<usize> = self.vertex.keys().copied().collect();
        keys.sort_unstable();
        let mut key_to_idx: std::collections::HashMap<usize, u32> = std::collections::HashMap::with_capacity(keys.len());
        for (i, &k) in keys.iter().enumerate() {
            key_to_idx.insert(k, i as u32);
        }

        let oc = self.objectcolor();
        let object_color = [oc.r, oc.g, oc.b, 1.0];
        let point_colors = self.get_pointcolors();
        // Honor the mesh's color_mode: per-vertex colors only when POINTCOLORS is the active mode
        // (and they cover every vertex). Otherwise the object color wins — so `set_objectcolor`
        // takes effect even though `add_vertex` seeds a white point-color per vertex.
        let has_point_colors =
            self.color_mode == crate::mesh::ColorMode::POINTCOLORS && point_colors.len() == keys.len();

        let mut vertices: Vec<RenderVertex> = Vec::with_capacity(keys.len());
        for (idx, k) in keys.iter().enumerate() {
            let v = &self.vertex[k];
            let nx = v.attributes.get("nx").copied().unwrap_or(0.0);
            let ny = v.attributes.get("ny").copied().unwrap_or(0.0);
            let nz = v.attributes.get("nz").copied().unwrap_or(0.0);
            let color = if has_point_colors {
                let c = &point_colors[idx];
                [c.r, c.g, c.b, 1.0]
            } else {
                object_color
            };
            vertices.push(RenderVertex {
                position: [v.x as f32, v.y as f32, v.z as f32],
                normal: [nx as f32, ny as f32, nz as f32],
                color,
            });
        }

        let mut indices: Vec<u32> = Vec::new();
        let mut face_keys: Vec<usize> = self.face.keys().copied().collect();
        face_keys.sort_unstable();
        for fk in face_keys {
            if let Some(tris) = self.triangulation.get(&fk) {
                if !tris.is_empty() {
                    for tri in tris {
                        if let (Some(&a), Some(&b), Some(&c)) =
                            (key_to_idx.get(&tri[0]), key_to_idx.get(&tri[1]), key_to_idx.get(&tri[2]))
                        {
                            indices.push(a);
                            indices.push(b);
                            indices.push(c);
                        }
                    }
                    continue;
                }
            }
            let verts_of_face = &self.face[&fk];
            if verts_of_face.len() < 3 {
                continue;
            }
            let v0 = match key_to_idx.get(&verts_of_face[0]) {
                Some(&i) => i,
                None => continue,
            };
            for i in 1..(verts_of_face.len() - 1) {
                let a = match key_to_idx.get(&verts_of_face[i]) {
                    Some(&i) => i,
                    None => continue,
                };
                let b = match key_to_idx.get(&verts_of_face[i + 1]) {
                    Some(&i) => i,
                    None => continue,
                };
                indices.push(v0);
                indices.push(a);
                indices.push(b);
            }
        }

        RenderMesh { vertices, indices }
    }

    /// The mesh's representative edge color for rendering: the first stored line
    /// color, or [`Color::black`] if the mesh has no edges. Every edge shares the
    /// same default today (`add_face` seeds `Color::black()`), so one sample is
    /// exact; genuine per-edge colors wait for the first-class edge path, which
    /// needs edge↔`linecolors` index alignment that `edges()` doesn't yet promise.
    pub fn edge_color(&self) -> Color {
        self.get_linecolors().first().cloned().unwrap_or_else(Color::black)
    }

    /// Drop the cached GPU buffers so the next `gpu_mesh()` rebuilds them. Called
    /// automatically on geometry edits (via `invalidate_triangle_bvh`) and color
    /// changes; call manually if you mutate render-affecting state another way.
    pub fn invalidate_gpu(&mut self) {
        self.gpu_cache.0 = None;
    }

    /// Build (once) and return cached GPU buffers ready to draw. The f64→f32 flatten
    /// (`to_render`) + upload runs only on the first call or after an edit; repeated
    /// calls are free. The viewer binds `gm.vbo`/`gm.ibo` and `draw_indexed(0..gm.index_count)`
    /// directly — no conversion on the viewer side. Pipelines use `RenderVertex::layout()`.
    pub fn gpu_mesh(&mut self, device: &wgpu::Device) -> &GpuMesh {
        if self.gpu_cache.0.is_none() {
            use wgpu::util::DeviceExt;
            let rm = self.to_render();
            let vbo = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("mesh.vbo"),
                contents: bytemuck::cast_slice(&rm.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let ibo = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("mesh.ibo"),
                contents: bytemuck::cast_slice(&rm.indices),
                usage: wgpu::BufferUsages::INDEX,
            });
            self.gpu_cache.0 = Some(GpuMesh {
                vbo,
                ibo,
                index_count: rm.indices.len() as u32,
            });
        }
        self.gpu_cache.0.as_ref().unwrap()
    }
}

impl RenderVertex {
    const ATTRIBS: [wgpu::VertexAttribute; 3] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4];

    /// Build a vertex from a kernel [`Point`] and [`Color`], with a zero normal. The
    /// f64→f32 position cast and the `Color`→`[f32; 4]` unpack both happen here — the
    /// correct precision/format boundary — so callers pass kernel types straight in
    /// and never sprinkle `as f32` or `[c.r, c.g, c.b, c.a]` at every use site. Handy
    /// for unlit line/edge geometry (the shader ignores the normal); for lit surfaces
    /// set the normal too.
    pub fn point(p: Point, color: &Color) -> Self {
        Self {
            position: p.to_f32(),
            normal: [0.0; 3],
            color: color.to_f32(),
        }
    }

    /// The `wgpu::VertexBufferLayout` a mesh pipeline must declare — mirrors this
    /// struct (pos f32x3 @0, normal f32x3 @12, color f32x4 @24, stride 40). The
    /// viewer passes `RenderVertex::layout()` instead of re-declaring offsets.
    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress, // 40
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// GPU-resident buffers for a mesh: bind `vbo`/`ibo` and `draw_indexed(0..index_count)`.
/// Built and cached by [`Mesh::gpu_mesh`](crate::mesh::Mesh::gpu_mesh).
pub struct GpuMesh {
    pub vbo: wgpu::Buffer,
    pub ibo: wgpu::Buffer,
    pub index_count: u32,
}

/// Lazily-built cache of a [`GpuMesh`] held by [`Mesh`]. Cloning a mesh resets the
/// cache to empty — a clone is logically new geometry and must NOT alias GPU
/// handles; it rebuilds on the next `gpu_mesh()` call. `#[serde(skip)]` on the
/// field means this never serializes (no proto/JSON parity impact).
#[derive(Default)]
pub struct GpuCache(pub Option<GpuMesh>);

impl Clone for GpuCache {
    fn clone(&self) -> Self {
        GpuCache(None)
    }
}
