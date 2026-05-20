//! Per-geometry-type adapters that produce `MeshVertex`/`LineVertex`/`PointVertex`
//! slices ready to upload via `GpuArena::allocate`. Tessellation for parametric
//! types (NurbsCurve → polyline, NurbsSurface → mesh) reuses the existing
//! `to_polyline_adaptive` / cached `m_mesh` paths.

use crate::gpu_session::{GeometryKind, LineVertex, MeshVertex, PointVertex};
use crate::{Color, Line, Mesh, OBB, Plane, Point, PointCloud, Polyline};

// ---------- Point ----------

impl Point {
    /// Position + the Point's `pointcolor` packed as RGBA8.
    pub fn to_point_vertex(&self) -> PointVertex {
        PointVertex {
            position: [self[0], self[1], self[2]],
            color: color_to_rgba_u8(&self.pointcolor),
        }
    }
}

// ---------- Line ----------

impl Line {
    /// Two endpoints. Both inherit the Line's `linecolor`.
    pub fn to_line_vertices(&self) -> [LineVertex; 2] {
        let color = color_to_rgba_u8(&self.linecolor);
        [
            LineVertex { position: [self.start()[0], self.start()[1], self.start()[2]], color },
            LineVertex { position: [self.end()[0], self.end()[1], self.end()[2]], color },
        ]
    }
}

// ---------- Polyline ----------

impl Polyline {
    /// LineList vertices + index pairs `[0,1, 1,2, 2,3, ...]`. All vertices
    /// inherit the polyline's `linecolor`.
    pub fn to_line_vertices(&self) -> (Vec<LineVertex>, Vec<u32>) {
        let color = color_to_rgba_u8(&self.linecolor);
        let pts = self.get_points();
        let verts: Vec<LineVertex> = pts
            .iter()
            .map(|p| LineVertex { position: [p[0], p[1], p[2]], color })
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
    /// Position + per-point color (falls back to white when no colors stored).
    pub fn to_point_vertices(&self) -> Vec<PointVertex> {
        let pts = self.get_points();
        let has_colors = self.color_count() == pts.len();
        pts.iter()
            .enumerate()
            .map(|(i, p)| {
                let color = if has_colors {
                    color_to_rgba_u8(&self.get_color(i))
                } else {
                    [255, 255, 255, 255]
                };
                PointVertex {
                    position: [p[0], p[1], p[2]],
                    color,
                }
            })
            .collect()
    }
}

// ---------- Mesh ----------

impl Mesh {
    /// Walks vertex+face maps and produces a flat (vertex, index) pair for
    /// TriangleList topology.
    /// - Per-vertex color from `pointcolors[i]` when populated, else
    ///   `objectcolor`, else white.
    /// - Per-vertex normals from `nx/ny/nz` attributes; if absent, computes
    ///   smooth-averaged flat normals from face geometry (so lit shading
    ///   "just works" without a separate pass).
    pub fn to_mesh_vertices(&self) -> (Vec<MeshVertex>, Vec<u32>) {
        // Deterministic vertex ordering by key
        let mut keys: Vec<usize> = self.vertex.keys().copied().collect();
        keys.sort_unstable();
        let key_to_idx: std::collections::HashMap<usize, u32> = keys
            .iter()
            .enumerate()
            .map(|(i, &k)| (k, i as u32))
            .collect();

        // ---- positions + per-vertex color ----
        let object_color_rgba = color_to_rgba_u8(self.objectcolor());
        let point_colors = self.get_pointcolors();
        let has_point_colors = point_colors.len() == keys.len();

        let mut verts: Vec<MeshVertex> = Vec::with_capacity(keys.len());
        let mut any_attr_normal = false;
        for (idx, k) in keys.iter().enumerate() {
            let v = &self.vertex[k];
            let nx = v.attributes.get("nx").copied();
            let ny = v.attributes.get("ny").copied();
            let nz = v.attributes.get("nz").copied();
            if nx.is_some() || ny.is_some() || nz.is_some() {
                any_attr_normal = true;
            }
            let normal = [nx.unwrap_or(0.0), ny.unwrap_or(0.0), nz.unwrap_or(0.0)];
            let color = if has_point_colors {
                color_to_rgba_u8(&point_colors[idx])
            } else {
                object_color_rgba
            };
            verts.push(MeshVertex {
                position: [v.x, v.y, v.z],
                normal,
                color,
            });
        }

        // ---- triangulated indices ----
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

        // ---- compute smooth-averaged flat normals if none were stored ----
        if !any_attr_normal {
            compute_vertex_normals_in_place(&mut verts, &inds);
        }

        (verts, inds)
    }
}

/// Compute smooth-averaged flat normals from triangle geometry. Each triangle
/// contributes its face normal (cross product, unnormalized so larger triangles
/// weight more) to all three of its vertices; then we normalize.
fn compute_vertex_normals_in_place(verts: &mut [MeshVertex], inds: &[u32]) {
    // Zero existing normals
    for v in verts.iter_mut() {
        v.normal = [0.0, 0.0, 0.0];
    }
    // Accumulate per-triangle face normals
    let mut i = 0;
    while i + 2 < inds.len() {
        let ia = inds[i] as usize;
        let ib = inds[i + 1] as usize;
        let ic = inds[i + 2] as usize;
        i += 3;
        if ia >= verts.len() || ib >= verts.len() || ic >= verts.len() {
            continue;
        }
        let a = verts[ia].position;
        let b = verts[ib].position;
        let c = verts[ic].position;
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let nx = ab[1] * ac[2] - ab[2] * ac[1];
        let ny = ab[2] * ac[0] - ab[0] * ac[2];
        let nz = ab[0] * ac[1] - ab[1] * ac[0];
        for &idx in &[ia, ib, ic] {
            verts[idx].normal[0] += nx;
            verts[idx].normal[1] += ny;
            verts[idx].normal[2] += nz;
        }
    }
    // Normalize
    for v in verts.iter_mut() {
        let n = v.normal;
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 1e-12 {
            v.normal = [n[0] / len, n[1] / len, n[2] / len];
        } else {
            v.normal = [0.0, 0.0, 1.0]; // arbitrary fallback for degenerate
        }
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
        let color = color_to_rgba_u8(&self.linecolor);
        let mk = |sx: f32, sy: f32| MeshVertex {
            position: [
                o[0] + sx * x[0] + sy * y[0],
                o[1] + sx * x[1] + sy * y[1],
                o[2] + sx * x[2] + sy * y[2],
            ],
            normal: [z[0], z[1], z[2]],
            color,
        };
        let verts = vec![mk(-h, -h), mk(h, -h), mk(h, h), mk(-h, h)];
        let inds = vec![0, 1, 2, 0, 2, 3];
        (verts, inds)
    }
}

// ---------- OBB ----------

impl OBB {
    /// 8 corners + 12 edge index pairs for wireframe LineList rendering.
    /// White by default — OBBs don't carry their own color attribute.
    pub fn to_line_vertices(&self) -> (Vec<LineVertex>, Vec<u32>) {
        let corners = self.corners();
        let color = [255, 255, 255, 255];
        let verts: Vec<LineVertex> = corners
            .iter()
            .map(|p| LineVertex { position: [p[0], p[1], p[2]], color })
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

/// Convert a `Color` to `[u8; 4]` for vertex attributes (Unorm8x4).
pub fn color_to_rgba_u8(c: &Color) -> [u8; 4] {
    [c.r, c.g, c.b, c.a]
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
