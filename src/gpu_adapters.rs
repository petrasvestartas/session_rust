//! Per-geometry-type adapters that produce `MeshVertex`/`LineVertex`/`PointVertex`
//! slices ready to upload via `GpuArena::allocate`. Tessellation for parametric
//! types (NurbsCurve → polyline, NurbsSurface → mesh) reuses the existing
//! `to_polyline_adaptive` / cached `m_mesh` paths.

use crate::gpu_session::{GeometryKind, LineVertex, MeshVertex, PointVertex};
use crate::{Color, Line, Mesh, OBB, Plane, Point, PointCloud, Polyline};

// ---------- Point ----------

impl Point {
    /// Position-only GPU vertex, ready for the point arena.
    pub fn to_point_vertex(&self) -> PointVertex {
        PointVertex { position: [self[0], self[1], self[2]] }
    }
}

// ---------- Line ----------

impl Line {
    /// Two endpoints for LineList topology.
    pub fn to_line_vertices(&self) -> [LineVertex; 2] {
        [
            LineVertex { position: [self.start()[0], self.start()[1], self.start()[2]] },
            LineVertex { position: [self.end()[0], self.end()[1], self.end()[2]] },
        ]
    }
}

// ---------- Polyline ----------

impl Polyline {
    /// LineList vertices + index pairs `[0,1, 1,2, 2,3, ...]`. Caller chooses
    /// closed/open by appending the wrap-around index pair externally.
    pub fn to_line_vertices(&self) -> (Vec<LineVertex>, Vec<u32>) {
        let pts = self.get_points();
        let verts: Vec<LineVertex> = pts
            .iter()
            .map(|p| LineVertex { position: [p[0], p[1], p[2]] })
            .collect();
        let n = verts.len();
        let mut inds = Vec::with_capacity(n.saturating_sub(1) * 2);
        for i in 0..n.saturating_sub(1) {
            inds.push(i as u32);
            inds.push((i + 1) as u32);
        }
        (verts, inds)
    }
}

// ---------- PointCloud ----------

impl PointCloud {
    /// Position-only vertices for PointList topology.
    pub fn to_point_vertices(&self) -> Vec<PointVertex> {
        self.get_points()
            .iter()
            .map(|p| PointVertex { position: [p[0], p[1], p[2]] })
            .collect()
    }
}

// ---------- Mesh ----------

impl Mesh {
    /// Walks vertex+face maps and produces a flat (vertex, index) pair for
    /// TriangleList topology. Per-vertex normals are zero by default; callers
    /// that need lit shading should compute and assign normals (e.g. via
    /// `compute_vertex_normals()` if available, else flat-per-face fallback).
    pub fn to_mesh_vertices(&self) -> (Vec<MeshVertex>, Vec<u32>) {
        // Deterministic vertex ordering by key
        let mut keys: Vec<usize> = self.vertex.keys().copied().collect();
        keys.sort_unstable();
        let key_to_idx: std::collections::HashMap<usize, u32> = keys
            .iter()
            .enumerate()
            .map(|(i, &k)| (k, i as u32))
            .collect();

        let mut verts: Vec<MeshVertex> = Vec::with_capacity(keys.len());
        for k in &keys {
            let v = &self.vertex[k];
            // Normal lookup from vertex attribute map if present (nx/ny/nz),
            // otherwise zero — caller's pipeline can compute screen-space.
            let nx = v.attributes.get("nx").copied().unwrap_or(0.0);
            let ny = v.attributes.get("ny").copied().unwrap_or(0.0);
            let nz = v.attributes.get("nz").copied().unwrap_or(0.0);
            verts.push(MeshVertex {
                position: [v.x, v.y, v.z],
                normal: [nx, ny, nz],
            });
        }

        // Indices: fan-triangulate each face's vertex list. For triangles
        // and quads this is fine; for N-gons it's not robust to non-convex
        // shapes — those should use the cached `triangulation` map if set.
        let mut inds: Vec<u32> = Vec::new();
        let mut face_keys: Vec<usize> = self.face.keys().copied().collect();
        face_keys.sort_unstable();
        for fk in face_keys {
            // Prefer cached triangulation if present (handles non-convex N-gons)
            if let Some(tris) = self.triangulation.get(&fk) {
                for tri in tris {
                    if let (Some(&a), Some(&b), Some(&c)) = (
                        key_to_idx.get(&tri[0]),
                        key_to_idx.get(&tri[1]),
                        key_to_idx.get(&tri[2]),
                    ) {
                        inds.push(a);
                        inds.push(b);
                        inds.push(c);
                    }
                }
                continue;
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
                inds.push(v0);
                inds.push(a);
                inds.push(b);
            }
        }
        (verts, inds)
    }
}

// ---------- Plane ----------

impl Plane {
    /// Two-triangle quad of `size`×`size` centered on the plane origin, oriented
    /// by the plane's x/y axes. For display only; mathematical planes are
    /// infinite.
    pub fn to_mesh_vertices(&self, size: f32) -> (Vec<MeshVertex>, Vec<u32>) {
        let o = self.origin();
        let x = self.x_axis();
        let y = self.y_axis();
        let z = self.z_axis();
        let h = size * 0.5;
        let mk = |sx: f32, sy: f32| MeshVertex {
            position: [
                o[0] + sx * x[0] + sy * y[0],
                o[1] + sx * x[1] + sy * y[1],
                o[2] + sx * x[2] + sy * y[2],
            ],
            normal: [z[0], z[1], z[2]],
        };
        let verts = vec![mk(-h, -h), mk(h, -h), mk(h, h), mk(-h, h)];
        let inds = vec![0, 1, 2, 0, 2, 3];
        (verts, inds)
    }
}

// ---------- OBB ----------

impl OBB {
    /// 8 corners + 12 edge index pairs for wireframe LineList rendering.
    pub fn to_line_vertices(&self) -> (Vec<LineVertex>, Vec<u32>) {
        let corners = self.corners();
        let verts: Vec<LineVertex> = corners
            .iter()
            .map(|p| LineVertex { position: [p[0], p[1], p[2]] })
            .collect();
        // Edges of a box: 4 bottom, 4 top, 4 vertical.
        // Assumes corner order matches typical convention; if `corners()` uses
        // a different order, adjust indices accordingly.
        let inds = vec![
            0, 1, 1, 2, 2, 3, 3, 0, // bottom face
            4, 5, 5, 6, 6, 7, 7, 4, // top face
            0, 4, 1, 5, 2, 6, 3, 7, // vertical edges
        ];
        (verts, inds)
    }
}

// ---------- Conversion helpers ----------

/// Convert a `Color` to `[f32; 4]` normalized to 0..=1 for the instance buffer.
pub fn color_to_rgba_f32(c: &Color) -> [f32; 4] {
    [
        c.r as f32 / 255.0,
        c.g as f32 / 255.0,
        c.b as f32 / 255.0,
        c.a as f32 / 255.0,
    ]
}

/// Return the `GeometryKind` that an `objects` member maps to.
pub fn kind_for_geometry(geom: &crate::session::Geometry) -> GeometryKind {
    use crate::session::Geometry;
    match geom {
        Geometry::Mesh(_) => GeometryKind::Mesh,
        Geometry::Polyline(_) => GeometryKind::Polyline,
        Geometry::Line(_) => GeometryKind::Line,
        Geometry::Point(_) => GeometryKind::Point,
        Geometry::PointCloud(_) => GeometryKind::PointCloud,
        Geometry::Plane(_) => GeometryKind::Plane,
        Geometry::OBB(_) => GeometryKind::Obb,
        Geometry::BRep(_) => GeometryKind::BRep,
        Geometry::Element(_) => GeometryKind::Element,
    }
}

#[cfg(test)]
#[path = "gpu_adapters_test.rs"]
mod gpu_adapters_test;
