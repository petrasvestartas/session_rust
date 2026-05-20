//! GPU mirror of Session. Holds three topology arenas (triangles, lines, points)
//! plus an instance buffer of per-object transforms/colors/flags.
//!
//! Vertices are flat `#[repr(C)] Pod` structs; positions are `[f32; 3]` taken
//! from `Point::position()`. Per-object state (model matrix, color, selection
//! flags) lives in `InstanceData` indexed by `instance_id`. A `PickTable`
//! maps `instance_id` ⇄ session `guid` so GPU-side IDs round-trip back to the
//! original `Geometry`.
//!
//! See plan: /Users/petras/.claude/plans/read-this-and-explain-functional-nest.md

use bytemuck::{Pod, Zeroable};
use std::collections::HashMap;

/// Which arena/topology a Session object lands in. Drives draw-pipeline choice.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GeometryKind {
    Mesh,
    Polyline,
    Line,
    Point,
    PointCloud,
    Plane,
    Obb,
    NurbsCurve,
    NurbsSurface,
    BRep,
    Element,
    Component,
}

impl GeometryKind {
    pub fn topology(self) -> wgpu::PrimitiveTopology {
        match self {
            GeometryKind::Mesh
            | GeometryKind::Plane
            | GeometryKind::NurbsSurface
            | GeometryKind::BRep
            | GeometryKind::Element
            | GeometryKind::Component => wgpu::PrimitiveTopology::TriangleList,
            GeometryKind::Polyline
            | GeometryKind::Line
            | GeometryKind::Obb
            | GeometryKind::NurbsCurve => wgpu::PrimitiveTopology::LineList,
            GeometryKind::Point | GeometryKind::PointCloud => wgpu::PrimitiveTopology::PointList,
        }
    }
}

/// Triangle-arena vertex. 28 bytes: position + normal + RGBA8.
/// Color uses `Unorm8x4` so the shader sees `vec4<f32>` in 0..=1.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [u8; 4],
}

impl MeshVertex {
    pub const ATTRIBS: [wgpu::VertexAttribute; 3] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Unorm8x4];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// Line-arena vertex. 16 bytes: position + RGBA8.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct LineVertex {
    pub position: [f32; 3],
    pub color: [u8; 4],
}

impl LineVertex {
    pub const ATTRIBS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Unorm8x4];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// Point-arena vertex. 16 bytes: position + RGBA8.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct PointVertex {
    pub position: [f32; 3],
    pub color: [u8; 4],
}

impl PointVertex {
    pub const ATTRIBS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Unorm8x4];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// Per-object data — one entry per Session object, indexed by `instance_id`.
/// Lives in a storage buffer; updated on transform/color/selection change.
/// 96 bytes, aligned to 16 (storage-buffer min alignment).
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct InstanceData {
    pub model: [[f32; 4]; 4], // 64 B — Xform as f32
    pub color: [f32; 4],      // 16 B — [r, g, b, a] in 0..=1
    pub object_id: u32,       //  4 B — instance_id mirrored for shader picking
    pub flags: u32,           //  4 B — bit0 selected, bit1 hovered, bit2 hidden
    pub _pad: [u32; 2],       //  8 B — align to 16
}

impl InstanceData {
    pub const FLAG_SELECTED: u32 = 1 << 0;
    pub const FLAG_HOVERED: u32 = 1 << 1;
    pub const FLAG_HIDDEN: u32 = 1 << 2;

    pub fn new(instance_id: u32) -> Self {
        Self {
            model: identity_matrix(),
            color: [1.0, 1.0, 1.0, 1.0],
            object_id: instance_id,
            flags: 0,
            _pad: [0, 0],
        }
    }
}

fn identity_matrix() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Maps `instance_id` (dense u32, used in GPU storage buffer) ⇄ session `guid`
/// (String UUID). Reused: the GPU upload path needs `guid → instance_id` to know
/// which storage-buffer slot to write; picking needs `instance_id → guid` to
/// resolve back to the original Geometry.
#[derive(Default, Debug)]
pub struct PickTable {
    instance_to_guid: Vec<Option<String>>,
    guid_to_instance: HashMap<String, u32>,
    free_instance_ids: Vec<u32>,
}

impl PickTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// Allocate an instance_id for a new guid. Reuses freed ids when available.
    pub fn allocate(&mut self, guid: &str) -> u32 {
        if let Some(&existing) = self.guid_to_instance.get(guid) {
            return existing;
        }
        let id = if let Some(reused) = self.free_instance_ids.pop() {
            self.instance_to_guid[reused as usize] = Some(guid.to_string());
            reused
        } else {
            let id = self.instance_to_guid.len() as u32;
            self.instance_to_guid.push(Some(guid.to_string()));
            id
        };
        self.guid_to_instance.insert(guid.to_string(), id);
        id
    }

    /// Free the instance_id for a guid. The slot is reusable.
    pub fn release(&mut self, guid: &str) {
        if let Some(id) = self.guid_to_instance.remove(guid) {
            self.instance_to_guid[id as usize] = None;
            self.free_instance_ids.push(id);
        }
    }

    pub fn resolve(&self, instance_id: u32) -> Option<&str> {
        self.instance_to_guid
            .get(instance_id as usize)
            .and_then(|s| s.as_deref())
    }

    pub fn instance_id(&self, guid: &str) -> Option<u32> {
        self.guid_to_instance.get(guid).copied()
    }

    pub fn capacity(&self) -> u32 {
        self.instance_to_guid.len() as u32
    }

    pub fn clear(&mut self) {
        self.instance_to_guid.clear();
        self.guid_to_instance.clear();
        self.free_instance_ids.clear();
    }
}

// ============================================================================
// GpuSession — composes three topology arenas + instance buffer + pick table.
// One per wgpu::Device. Owned by Session via `Session::gpu: Option<GpuSession>`.
// ============================================================================

use crate::gpu_adapters::{color_to_rgba_f32, kind_for_geometry};
use crate::gpu_arena::GpuArena;

/// Initial capacities (vertices/indices) for the three topology arenas.
/// Sized to comfortably hold a few hundred small geometries before growth.
pub const DEFAULT_TRI_VERTS: u32 = 4096;
pub const DEFAULT_TRI_INDS: u32 = 8192;
pub const DEFAULT_LINE_VERTS: u32 = 4096;
pub const DEFAULT_LINE_INDS: u32 = 8192;
pub const DEFAULT_POINT_VERTS: u32 = 4096;
pub const DEFAULT_INSTANCE_CAP: u32 = 1024;

/// GPU mirror of a `Session`. Three buffers per topology, one instance buffer,
/// one pick table.
pub struct GpuSession {
    pub tri: GpuArena<MeshVertex>,
    pub line: GpuArena<LineVertex>,
    pub point: GpuArena<PointVertex>,

    pub instance_buffer: wgpu::Buffer,
    pub instance_capacity: u32,
    pub instances_cpu: Vec<InstanceData>, // mirror; uploaded on change

    pub pick: PickTable,
}

impl GpuSession {
    pub fn new(device: &wgpu::Device) -> Self {
        let tri = GpuArena::<MeshVertex>::new(
            device,
            "gpu_session.tri",
            DEFAULT_TRI_VERTS,
            DEFAULT_TRI_INDS,
        );
        let line = GpuArena::<LineVertex>::new(
            device,
            "gpu_session.line",
            DEFAULT_LINE_VERTS,
            DEFAULT_LINE_INDS,
        );
        let point = GpuArena::<PointVertex>::new(
            device,
            "gpu_session.point",
            DEFAULT_POINT_VERTS,
            0, // points are unindexed
        );

        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("gpu_session.instances"),
            size: (DEFAULT_INSTANCE_CAP as u64) * (std::mem::size_of::<InstanceData>() as u64),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        Self {
            tri,
            line,
            point,
            instance_buffer,
            instance_capacity: DEFAULT_INSTANCE_CAP,
            instances_cpu: Vec::new(),
            pick: PickTable::new(),
        }
    }

    /// Cold start: clear everything, then re-add every object from `session`.
    /// O(n) in object count; only call after a major change or to recover from
    /// desync.
    pub fn rebuild_from(
        &mut self,
        session: &crate::session::Session,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        self.clear();
        for (guid, geom) in &session.lookup {
            self.add_geometry(guid, geom, device, queue);
        }
    }

    /// Add a single `Geometry` variant to the arena it belongs to and register
    /// its instance row. Idempotent on guid.
    pub fn add_geometry(
        &mut self,
        guid: &str,
        geom: &crate::session::Geometry,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        use crate::session::Geometry;
        let instance_id = self.pick.allocate(guid);
        let kind = kind_for_geometry(geom);

        match geom {
            Geometry::Point(p) => {
                let v = p.to_point_vertex();
                self.point.allocate(
                    guid, &[v], None, instance_id, kind, device, queue,
                );
                self.write_instance(instance_id, color_to_rgba_f32(&p.pointcolor), device, queue);
            }
            Geometry::Line(l) => {
                let vs = l.to_line_vertices();
                self.line.allocate(
                    guid, &vs, None, instance_id, kind, device, queue,
                );
                self.write_instance(instance_id, color_to_rgba_f32(&l.linecolor), device, queue);
            }
            Geometry::Polyline(pl) => {
                let (vs, is) = pl.to_line_vertices();
                self.line.allocate(
                    guid, &vs, Some(&is), instance_id, kind, device, queue,
                );
                self.write_instance(instance_id, color_to_rgba_f32(&pl.linecolor), device, queue);
            }
            Geometry::PointCloud(pc) => {
                let vs = pc.to_point_vertices();
                self.point.allocate(
                    guid, &vs, None, instance_id, kind, device, queue,
                );
                // Pointcloud has per-vertex colors; here we just store a default
                // white for the instance; per-vertex colors would require
                // extending PointVertex.
                self.write_instance(instance_id, [1.0, 1.0, 1.0, 1.0], device, queue);
            }
            Geometry::Mesh(m) => {
                let (vs, is) = m.to_mesh_vertices();
                self.tri.allocate(
                    guid, &vs, Some(&is), instance_id, kind, device, queue,
                );
                self.write_instance(instance_id, color_to_rgba_f32(m.objectcolor()), device, queue);
            }
            Geometry::Plane(pl) => {
                let (vs, is) = pl.to_mesh_vertices(1.0);
                self.tri.allocate(
                    guid, &vs, Some(&is), instance_id, kind, device, queue,
                );
                self.write_instance(instance_id, color_to_rgba_f32(&pl.linecolor), device, queue);
            }
            Geometry::OBB(bb) => {
                let (vs, is) = bb.to_line_vertices();
                self.line.allocate(
                    guid, &vs, Some(&is), instance_id, kind, device, queue,
                );
                self.write_instance(instance_id, [1.0, 1.0, 1.0, 1.0], device, queue);
            }
            Geometry::BRep(b) => {
                // Tessellate to Mesh and dispatch to the tri arena.
                let m = b.mesh();
                let (vs, is) = m.to_mesh_vertices();
                if !vs.is_empty() {
                    self.tri.allocate(
                        guid, &vs, Some(&is), instance_id, kind, device, queue,
                    );
                    self.write_instance(instance_id, color_to_rgba_f32(&b.surfacecolor), device, queue);
                } else {
                    self.pick.release(guid);
                }
            }
            Geometry::Element(e) => {
                // Element wraps either a Mesh or a BRep; dispatch accordingly.
                use crate::element::ElementGeometry;
                match e.geometry() {
                    ElementGeometry::Mesh(m) => {
                        let (vs, is) = m.to_mesh_vertices();
                        if !vs.is_empty() {
                            self.tri.allocate(
                                guid, &vs, Some(&is), instance_id, kind, device, queue,
                            );
                            self.write_instance(
                                instance_id,
                                color_to_rgba_f32(m.objectcolor()),
                                device,
                                queue,
                            );
                        } else {
                            self.pick.release(guid);
                        }
                    }
                    ElementGeometry::BRep(b) => {
                        let m = b.mesh();
                        let (vs, is) = m.to_mesh_vertices();
                        if !vs.is_empty() {
                            self.tri.allocate(
                                guid, &vs, Some(&is), instance_id, kind, device, queue,
                            );
                            self.write_instance(
                                instance_id,
                                color_to_rgba_f32(&b.surfacecolor),
                                device,
                                queue,
                            );
                        } else {
                            self.pick.release(guid);
                        }
                    }
                    ElementGeometry::None => {
                        self.pick.release(guid);
                    }
                }
            }
        }
    }

    /// Mark a guid's arena range free and release its instance slot.
    pub fn remove(&mut self, guid: &str) {
        // Free from whichever arena holds it (only one of the three).
        self.tri.free(guid);
        self.line.free(guid);
        self.point.free(guid);
        self.pick.release(guid);
    }

    /// Clear all arenas + pick table + instance mirror.
    pub fn clear(&mut self) {
        self.tri.clear();
        self.line.clear();
        self.point.clear();
        self.pick.clear();
        self.instances_cpu.clear();
    }

    /// Write the CPU mirror entry for this `instance_id` and upload to the GPU
    /// storage buffer. Grows the buffer (amortized 2×) if `instance_id` is
    /// beyond current capacity, preserving existing contents via
    /// copy_buffer_to_buffer.
    fn write_instance(
        &mut self,
        instance_id: u32,
        color: [f32; 4],
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        let id = instance_id as usize;
        if id >= self.instances_cpu.len() {
            self.instances_cpu.resize(id + 1, InstanceData::new(0));
        }
        let mut data = InstanceData::new(instance_id);
        data.color = color;
        self.instances_cpu[id] = data;

        if instance_id >= self.instance_capacity {
            self.grow_instance_buffer(instance_id + 1, device, queue);
        }
        let offset = (id as u64) * (std::mem::size_of::<InstanceData>() as u64);
        queue.write_buffer(&self.instance_buffer, offset, bytemuck::bytes_of(&data));
    }

    /// Resize the GPU instance buffer to fit at least `needed` slots,
    /// preserving existing data. Doubles capacity each grow.
    fn grow_instance_buffer(
        &mut self,
        needed: u32,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
    ) {
        let mut new_cap = self.instance_capacity.max(1) * 2;
        while new_cap < needed {
            new_cap *= 2;
        }
        let new_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("gpu_session.instances"),
            size: (new_cap as u64) * (std::mem::size_of::<InstanceData>() as u64),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let bytes_to_copy = (self.instance_capacity as u64)
            * (std::mem::size_of::<InstanceData>() as u64);
        if bytes_to_copy > 0 {
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("gpu_session.instances.grow"),
            });
            encoder.copy_buffer_to_buffer(
                &self.instance_buffer,
                0,
                &new_buffer,
                0,
                bytes_to_copy,
            );
            queue.submit(std::iter::once(encoder.finish()));
        }
        self.instance_buffer = new_buffer;
        self.instance_capacity = new_cap;
    }

    /// Update an instance row's transform (e.g. when the user moves the object).
    pub fn update_transform(
        &mut self,
        guid: &str,
        model: [[f32; 4]; 4],
        queue: &wgpu::Queue,
    ) -> bool {
        let id = match self.pick.instance_id(guid) {
            Some(i) => i,
            None => return false,
        };
        let idx = id as usize;
        if idx >= self.instances_cpu.len() {
            return false;
        }
        self.instances_cpu[idx].model = model;
        let offset = (idx as u64) * (std::mem::size_of::<InstanceData>() as u64);
        queue.write_buffer(
            &self.instance_buffer,
            offset,
            bytemuck::bytes_of(&self.instances_cpu[idx]),
        );
        true
    }

    /// Update an instance row's color.
    pub fn update_color(&mut self, guid: &str, color: [f32; 4], queue: &wgpu::Queue) -> bool {
        let id = match self.pick.instance_id(guid) {
            Some(i) => i,
            None => return false,
        };
        let idx = id as usize;
        if idx >= self.instances_cpu.len() {
            return false;
        }
        self.instances_cpu[idx].color = color;
        let offset = (idx as u64) * (std::mem::size_of::<InstanceData>() as u64);
        queue.write_buffer(
            &self.instance_buffer,
            offset,
            bytemuck::bytes_of(&self.instances_cpu[idx]),
        );
        true
    }

    /// Set / clear a flag bit on an instance row.
    pub fn set_flag(&mut self, guid: &str, flag: u32, on: bool, queue: &wgpu::Queue) -> bool {
        let id = match self.pick.instance_id(guid) {
            Some(i) => i,
            None => return false,
        };
        let idx = id as usize;
        if idx >= self.instances_cpu.len() {
            return false;
        }
        if on {
            self.instances_cpu[idx].flags |= flag;
        } else {
            self.instances_cpu[idx].flags &= !flag;
        }
        let offset = (idx as u64) * (std::mem::size_of::<InstanceData>() as u64);
        queue.write_buffer(
            &self.instance_buffer,
            offset,
            bytemuck::bytes_of(&self.instances_cpu[idx]),
        );
        true
    }

    pub fn guid_for_instance(&self, instance_id: u32) -> Option<&str> {
        self.pick.resolve(instance_id)
    }
}

#[cfg(test)]
#[path = "gpu_session_test.rs"]
mod gpu_session_test;
