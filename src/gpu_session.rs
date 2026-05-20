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

/// Triangle-arena vertex. 24 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

impl MeshVertex {
    pub const ATTRIBS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// Line-arena vertex. 12 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct LineVertex {
    pub position: [f32; 3],
}

impl LineVertex {
    pub const ATTRIBS: [wgpu::VertexAttribute; 1] =
        wgpu::vertex_attr_array![0 => Float32x3];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

/// Point-arena vertex. 12 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct PointVertex {
    pub position: [f32; 3],
}

impl PointVertex {
    pub const ATTRIBS: [wgpu::VertexAttribute; 1] =
        wgpu::vertex_attr_array![0 => Float32x3];

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

#[cfg(test)]
#[path = "gpu_session_test.rs"]
mod gpu_session_test;
