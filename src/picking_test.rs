/// Ray-casting and picking integration tests.
/// Run with: cargo test picking
#[cfg(test)]
mod picking_tests {
    use crate::{Color, Mesh, Point, Vector};
    use crate::session::Session;
    use crate::xform::Xform;
    use crate::tolerance::Tolerance;

    const MM_TO_UNIT: f32 = 0.001;

    // -----------------------------------------------------------------------
    // Viewer matrix helpers (replicate session_viewer/src/pick.rs logic)
    // -----------------------------------------------------------------------

    fn mat4_mul_cm(a: &[[f32; 4]; 4], b: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
        let mut out = [[0.0f32; 4]; 4];
        for c in 0..4 {
            for r in 0..4 {
                let mut s = 0.0;
                for k in 0..4 {
                    s += a[k][r] * b[c][k];
                }
                out[c][r] = s;
            }
        }
        out
    }

    fn mat4_unproject(inv_vp: &[[f32; 4]; 4], ndc: [f32; 3]) -> [f32; 3] {
        let (x, y, z, w) = (ndc[0], ndc[1], ndc[2], 1.0f32);
        let ox = inv_vp[0][0]*x + inv_vp[1][0]*y + inv_vp[2][0]*z + inv_vp[3][0]*w;
        let oy = inv_vp[0][1]*x + inv_vp[1][1]*y + inv_vp[2][1]*z + inv_vp[3][1]*w;
        let oz = inv_vp[0][2]*x + inv_vp[1][2]*y + inv_vp[2][2]*z + inv_vp[3][2]*w;
        let ow = inv_vp[0][3]*x + inv_vp[1][3]*y + inv_vp[2][3]*z + inv_vp[3][3]*w;
        let inv_w = if ow.abs() > 1e-30 { 1.0 / ow } else { 1.0 };
        [ox*inv_w, oy*inv_w, oz*inv_w]
    }

    fn mat4_inverse(m: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
        let f = |c: usize, r: usize| m[c][r];
        let (m00,m01,m02,m03) = (f(0,0),f(1,0),f(2,0),f(3,0));
        let (m10,m11,m12,m13) = (f(0,1),f(1,1),f(2,1),f(3,1));
        let (m20,m21,m22,m23) = (f(0,2),f(1,2),f(2,2),f(3,2));
        let (m30,m31,m32,m33) = (f(0,3),f(1,3),f(2,3),f(3,3));
        let a2323 = m22*m33 - m23*m32; let a1323 = m21*m33 - m23*m31;
        let a1223 = m21*m32 - m22*m31; let a0323 = m20*m33 - m23*m30;
        let a0223 = m20*m32 - m22*m30; let a0123 = m20*m31 - m21*m30;
        let a2313 = m12*m33 - m13*m32; let a1313 = m11*m33 - m13*m31;
        let a1213 = m11*m32 - m12*m31; let a2312 = m12*m23 - m13*m22;
        let a1312 = m11*m23 - m13*m21; let a1212 = m11*m22 - m12*m21;
        let a0313 = m10*m33 - m13*m30; let a0213 = m10*m32 - m12*m30;
        let a0312 = m10*m23 - m13*m20; let a0212 = m10*m22 - m12*m20;
        let a0113 = m10*m31 - m11*m30; let a0112 = m10*m21 - m11*m20;
        let det = m00*(m11*a2323 - m12*a1323 + m13*a1223)
                - m01*(m10*a2323 - m12*a0323 + m13*a0223)
                + m02*(m10*a1323 - m11*a0323 + m13*a0123)
                - m03*(m10*a1223 - m11*a0223 + m12*a0123);
        let inv_det = if det.abs() > 1e-30 { 1.0 / det } else { 0.0 };
        let r = |a: f32, b: f32, c: f32, d: f32| (a - b + c - d) * inv_det;
        let mut out = [[0.0f32; 4]; 4];
        out[0][0] = r(m11*a2323, m12*a1323, m13*a1223, 0.0);
        out[0][1] = r(0.0, m01*a2323, m02*a1323, m03*a1223);
        out[0][2] = r(m01*a2313, m02*a1313, m03*a1213, 0.0);
        out[0][3] = r(0.0, m01*a2312, m02*a1312, m03*a1212);
        out[1][0] = r(0.0, m10*a2323, m12*a0323, m13*a0223);
        out[1][1] = r(m00*a2323, m02*a0323, m03*a0223, 0.0);
        out[1][2] = r(0.0, m00*a2313, m02*a0313, m03*a0213);
        out[1][3] = r(m00*a2312, m02*a0312, m03*a0212, 0.0);
        out[2][0] = r(m10*a1323, m11*a0323, m13*a0123, 0.0);
        out[2][1] = r(0.0, m00*a1323, m01*a0323, m03*a0123);
        out[2][2] = r(m00*a1313, m01*a0313, m03*a0113, 0.0);
        out[2][3] = r(0.0, m00*a1312, m01*a0312, m03*a0112);
        out[3][0] = r(0.0, m10*a1223, m11*a0223, m12*a0123);
        out[3][1] = r(m00*a1223, m01*a0223, m02*a0123, 0.0);
        out[3][2] = r(0.0, m00*a1213, m01*a0213, m02*a0113);
        out[3][3] = r(m00*a1212, m01*a0212, m02*a0112, 0.0);
        let mut t = [[0.0f32; 4]; 4];
        for c in 0..4 { for rr in 0..4 { t[c][rr] = out[rr][c]; } }
        t
    }

    fn screen_to_world_ray(
        view: &[[f32; 4]; 4],
        proj: &[[f32; 4]; 4],
        viewport: (f32, f32),
        cursor: (f32, f32),
    ) -> (Point, Vector) {
        let (w, h) = viewport;
        let ndc_x = (cursor.0 / w) * 2.0 - 1.0;
        let ndc_y = 1.0 - (cursor.1 / h) * 2.0;
        let inv = mat4_inverse(&mat4_mul_cm(proj, view));
        let p_near = mat4_unproject(&inv, [ndc_x, ndc_y, 0.0]);
        let p_far  = mat4_unproject(&inv, [ndc_x, ndc_y, 0.5]);
        let dx = p_far[0] - p_near[0];
        let dy = p_far[1] - p_near[1];
        let dz = p_far[2] - p_near[2];
        let len = (dx*dx + dy*dy + dz*dz).sqrt().max(1e-30);
        let origin = Point::new(p_near[0], p_near[1], p_near[2]);
        let dir    = Vector::new(dx/len, dy/len, dz/len);
        (origin, dir)
    }

    fn build_view_matrix(eye_m: [f32; 3], tgt_m: [f32; 3], up: [f32; 3]) -> [[f32; 4]; 4] {
        let eye   = Point::new(eye_m[0], eye_m[1], eye_m[2]);
        let tgt   = Point::new(tgt_m[0], tgt_m[1], tgt_m[2]);
        let up_v  = Vector::new(up[0], up[1], up[2]);
        let view  = Xform::look_at_right_handed(&eye, &tgt, &up_v);
        let scale = Xform::scale_xyz(MM_TO_UNIT, MM_TO_UNIT, MM_TO_UNIT);
        (&view * &scale).to_cols()
    }

    fn build_proj_matrix(fov_y: f32, aspect: f32, near: f32, far: f32) -> [[f32; 4]; 4] {
        Xform::perspective(fov_y, aspect, near, far).to_cols()
    }

    /// Project world_mm through view_proj → NDC.
    fn project_to_ndc(
        view: &[[f32; 4]; 4],
        proj: &[[f32; 4]; 4],
        world_mm: [f32; 3],
    ) -> [f32; 3] {
        let p4 = [world_mm[0], world_mm[1], world_mm[2], 1.0f32];
        let mut vs = [0.0f32; 4];
        for r in 0..4 {
            vs[r] = view[0][r]*p4[0] + view[1][r]*p4[1] + view[2][r]*p4[2] + view[3][r]*p4[3];
        }
        let mut cs = [0.0f32; 4];
        for r in 0..4 {
            cs[r] = proj[0][r]*vs[0] + proj[1][r]*vs[1] + proj[2][r]*vs[2] + proj[3][r]*vs[3];
        }
        let inv_w = if cs[3].abs() > 1e-10 { 1.0 / cs[3] } else { 1.0 };
        [cs[0]*inv_w, cs[1]*inv_w, cs[2]*inv_w]
    }

    fn make_box(cx: f32, cy: f32, cz: f32, r: f32) -> Mesh {
        let mut m = Mesh::new();
        let pts = [
            Point::new(cx-r, cy-r, cz-r), Point::new(cx+r, cy-r, cz-r),
            Point::new(cx+r, cy+r, cz-r), Point::new(cx-r, cy+r, cz-r),
            Point::new(cx-r, cy-r, cz+r), Point::new(cx+r, cy-r, cz+r),
            Point::new(cx+r, cy+r, cz+r), Point::new(cx-r, cy+r, cz+r),
        ];
        let v: Vec<_> = pts.iter().map(|p| m.add_vertex(p.clone(), None)).collect();
        for f in &[[0,3,2,1],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,4,7,3],[1,2,6,5]] {
            m.add_face(vec![v[f[0]], v[f[1]], v[f[2]], v[f[3]]], None);
        }
        m.set_objectcolor(Color::new(1.0, 0.0, 0.0, 1.0));
        m
    }

    // -----------------------------------------------------------------------
    // Session-level ray_cast tests — these exercise the BVH + mesh BVH
    // -----------------------------------------------------------------------

    /// Ray shoots straight toward a box at origin from -x.
    #[test]
    fn test_ray_cast_origin_box_hit() {
        let mut s = Session::new("t");
        s.add_mesh(make_box(0.0, 0.0, 0.0, 100.0), None);
        let origin = Point::new(-500.0, 0.0, 0.0);
        let dir    = Vector::new(1.0, 0.0, 0.0);
        let hits = s.ray_cast(&origin, &dir, 1.0);
        assert!(!hits.is_empty(), "ray along +x should hit box at origin");
    }

    /// Ray shoots in the opposite direction — no hit expected.
    #[test]
    fn test_ray_cast_origin_box_miss_behind() {
        let mut s = Session::new("t");
        s.add_mesh(make_box(0.0, 0.0, 0.0, 100.0), None);
        let origin = Point::new(-500.0, 0.0, 0.0);
        let dir    = Vector::new(-1.0, 0.0, 0.0);
        let hits = s.ray_cast(&origin, &dir, 1.0);
        assert!(hits.is_empty(), "ray pointing away should miss");
    }

    /// Box at (2000, 2000, 1000) mm — same coordinates as box_blue in demo.
    #[test]
    fn test_ray_cast_far_box_hit() {
        let mut s = Session::new("t");
        s.add_mesh(make_box(2000.0, 2000.0, 1000.0, 400.0), None);
        // Shoot from the approximate default-camera world position (mm) toward box center.
        let cam_mm = Point::new(1840.0, -1840.0, 1500.0);
        let dx = 2000.0 - 1840.0;
        let dy = 2000.0 - (-1840.0_f32);
        let dz = 1000.0 - 1500.0_f32;
        let dir = Vector::new(dx, dy, dz);
        let hits = s.ray_cast(&cam_mm, &dir, 1.0);
        assert!(!hits.is_empty(), "ray from ~camera toward (2000,2000,1000) should hit box_blue-equivalent");
    }

    /// Box at (−2500, 1000, 2000) mm — same as box_green in demo.
    #[test]
    fn test_ray_cast_far_box_green() {
        let mut s = Session::new("t");
        s.add_mesh(make_box(-2500.0, 1000.0, 2000.0, 350.0), None);
        let cam_mm = Point::new(1840.0, -1840.0, 1500.0);
        let dx = -2500.0 - 1840.0_f32;
        let dy = 1000.0 - (-1840.0_f32);
        let dz = 2000.0 - 1500.0_f32;
        let dir = Vector::new(dx, dy, dz);
        let hits = s.ray_cast(&cam_mm, &dir, 1.0);
        assert!(!hits.is_empty(), "ray toward box_green-equivalent should hit");
    }

    /// Multiple meshes at different distances — all should be individually selectable.
    #[test]
    fn test_ray_cast_per_object_selectability() {
        let cases: &[(&str, (f32, f32, f32), f32)] = &[
            ("triangle_origin",  (  0.0,    0.0,    0.0), 400.0),
            ("box_small",        (600.0,  600.0,  200.0), 120.0),
            ("box_blue",        (2000.0, 2000.0, 1000.0), 400.0),
            ("box_green",      (-2500.0, 1000.0, 2000.0), 350.0),
            ("box_red",         (1000.0,-2000.0, 1500.0), 500.0),
            ("tetra_cyan",      (3000.0,    0.0,  500.0), 600.0),
        ];

        let cam_mm = Point::new(1840.0, -1840.0, 1500.0);

        for &(name, (cx, cy, cz), r) in cases {
            let mut s = Session::new("t");
            s.add_mesh(make_box(cx, cy, cz, r), None);

            let dx = cx - cam_mm[0];
            let dy = cy - cam_mm[1];
            let dz = cz - cam_mm[2];
            let dir = Vector::new(dx, dy, dz);
            let hits = s.ray_cast(&cam_mm, &dir, 1.0);
            assert!(!hits.is_empty(),
                "FAIL: {} at ({},{},{}) r={} — ray_cast returned no hits", name, cx, cy, cz, r);
        }
    }

    // -----------------------------------------------------------------------
    // Mesh-level BVH ray cast
    // -----------------------------------------------------------------------

    /// The mesh BVH alone should also hit a far box.
    #[test]
    fn test_mesh_bvh_ray_cast_far() {
        use crate::Line;
        let mut m = make_box(2000.0, 2000.0, 1000.0, 400.0);
        let origin = Point::new(1840.0, -1840.0, 1500.0);
        let dx = 2000.0 - 1840.0;
        let dy = 2000.0 - (-1840.0_f32);
        let dz = 1000.0 - 1500.0_f32;
        let len = (dx*dx + dy*dy + dz*dz).sqrt();
        let end = Point::new(
            origin[0] + dx/len * 1e6,
            origin[1] + dy/len * 1e6,
            origin[2] + dz/len * 1e6,
        );
        let ray = Line::from_points(&origin, &end);
        let hit = m.ray_cast_bvh(&ray, 1e-6);
        assert!(hit.is_some(), "mesh BVH ray_cast should hit far box");
    }

    // -----------------------------------------------------------------------
    // Session BVH world-size coverage
    // -----------------------------------------------------------------------

    /// BVH world_size must cover ALL objects including far ones;
    /// otherwise Morton codes get clamped and BVH is wrong.
    #[test]
    fn test_bvh_world_size_covers_far_objects() {
        use crate::spatial_bvh::SpatialBVH;
        use crate::obb::OBB;
        use crate::aabb::AABB;

        let positions: &[(f32, f32, f32, f32)] = &[
            (   0.0,    0.0,    0.0, 400.0),
            (2000.0, 2000.0, 1000.0, 400.0),
            (-2500.0, 1000.0, 2000.0, 350.0),
            (3000.0,    0.0,  500.0, 600.0),
        ];

        let obbs: Vec<OBB> = positions.iter().map(|&(cx, cy, cz, r)| {
            let pts = vec![
                Point::new(cx-r, cy-r, cz-r),
                Point::new(cx+r, cy+r, cz+r),
            ];
            OBB::from_points(&pts, 1.0)
        }).collect();

        let world_size = SpatialBVH::compute_world_size(&obbs);

        // All object centers must be within [-world_size/2, +world_size/2] cube.
        for (i, obb) in obbs.iter().enumerate() {
            let half = world_size / 2.0;
            assert!(obb.center[0].abs() <= half,
                "OBB {} center.x {} outside world_size {}", i, obb.center[0], world_size);
            assert!(obb.center[1].abs() <= half,
                "OBB {} center.y {} outside world_size {}", i, obb.center[1], world_size);
            assert!(obb.center[2].abs() <= half,
                "OBB {} center.z {} outside world_size {}", i, obb.center[2], world_size);
        }
    }

    // -----------------------------------------------------------------------
    // Viewer ray-generation round-trip tests
    // These replicate session_viewer/src/pick.rs screen_to_world_ray logic.
    // -----------------------------------------------------------------------

    /// Round-trip: project a world point → NDC → pixel → ray, check direction aligns.
    fn check_ray_roundtrip(
        view: &[[f32; 4]; 4],
        proj: &[[f32; 4]; 4],
        vp: (f32, f32),
        world_mm: [f32; 3],
        label: &str,
    ) {
        let ndc = project_to_ndc(view, proj, world_mm);
        assert!(
            ndc[0].abs() <= 1.0 && ndc[1].abs() <= 1.0,
            "{label}: NDC ({:.3},{:.3}) outside frustum — adjust camera so object is visible",
            ndc[0], ndc[1]
        );
        // NDC → pixel
        let cx = (ndc[0] + 1.0) / 2.0 * vp.0;
        let cy = (1.0 - ndc[1]) / 2.0 * vp.1;
        let (origin, dir) = screen_to_world_ray(view, proj, vp, (cx, cy));
        // Expected direction: from ray origin toward world point
        let dx = world_mm[0] - origin[0];
        let dy = world_mm[1] - origin[1];
        let dz = world_mm[2] - origin[2];
        let len = (dx*dx + dy*dy + dz*dz).sqrt().max(1e-6);
        let dot = dir[0]*(dx/len) + dir[1]*(dy/len) + dir[2]*(dz/len);
        assert!(
            dot > 0.999,
            "{label}: ray direction mismatch — dot={:.6}, dir=({:.4},{:.4},{:.4}), expected=({:.4},{:.4},{:.4}), origin=({:.1},{:.1},{:.1})",
            dot, dir[0], dir[1], dir[2], dx/len, dy/len, dz/len, origin[0], origin[1], origin[2]
        );
    }

    /// Simple sanity: camera on +Z axis, object at origin.
    #[test]
    fn test_viewer_ray_simple_z_camera() {
        let eye_m = [0.0f32, 0.0, 10.0];
        let tgt_m = [0.0f32, 0.0, 0.0];
        let up    = [0.0f32, 1.0, 0.0];
        let view  = build_view_matrix(eye_m, tgt_m, up);
        let proj  = build_proj_matrix(Tolerance::PI / 3.0, 16.0/9.0, 0.01, 100.0);
        let vp    = (1920.0f32, 1080.0);
        check_ray_roundtrip(&view, &proj, vp, [0.0, 0.0, 0.0], "origin_z_cam");
        check_ray_roundtrip(&view, &proj, vp, [2000.0, 1000.0, 0.0], "offset_z_cam");
    }

    /// Camera targeting box_blue so it is in-frustum; verify round-trip.
    #[test]
    fn test_viewer_ray_box_blue_in_frustum() {
        let box_blue = [2000.0f32, 2000.0, 1000.0];
        // Camera at 3 m above/behind box_blue
        let eye_m = [2.0f32, -1.0, 1.0];
        let tgt_m = [2.0f32, 2.0, 1.0];
        let up    = [0.0f32, 0.0, 1.0];
        let near  = 3.0_f32 * 0.001;
        let view  = build_view_matrix(eye_m, tgt_m, up);
        let proj  = build_proj_matrix(Tolerance::PI / 3.0, 16.0/9.0, near, 100_000.0);
        let vp    = (1920.0f32, 1080.0);
        check_ray_roundtrip(&view, &proj, vp, box_blue, "box_blue_targeted");
    }

    /// Default camera (approximate); origin and small box in frustum.
    #[test]
    fn test_viewer_ray_default_camera() {
        let eye_m = [1.836f32, -1.836, 1.5];
        let tgt_m = [0.0f32, 0.0, 0.0];
        let up    = [-0.354f32, 0.354, 0.866];
        let near  = 3.0_f32 * 0.001;
        let view  = build_view_matrix(eye_m, tgt_m, up);
        let proj  = build_proj_matrix(Tolerance::PI / 3.0, 16.0/9.0, near, 100_000.0);
        let vp    = (1920.0f32, 1080.0);
        check_ray_roundtrip(&view, &proj, vp, [0.0, 0.0, 0.0], "origin_default");
        check_ray_roundtrip(&view, &proj, vp, [600.0, 600.0, 200.0], "small_box_default");
    }

    /// Ray origin must be in mm (near-plane, ~distance*1mm from camera eye position in mm).
    #[test]
    fn test_viewer_ray_origin_is_mm() {
        let eye_m = [1.836f32, -1.836, 1.5];
        let tgt_m = [0.0f32, 0.0, 0.0];
        let up    = [-0.354f32, 0.354, 0.866];
        let near  = 3.0_f32 * 0.001;
        let view  = build_view_matrix(eye_m, tgt_m, up);
        let proj  = build_proj_matrix(Tolerance::PI / 3.0, 16.0/9.0, near, 100_000.0);
        let vp    = (1920.0f32, 1080.0);
        let (origin, _) = screen_to_world_ray(&view, &proj, vp, (960.0, 540.0));
        let eye_mm = [eye_m[0]*1000.0, eye_m[1]*1000.0, eye_m[2]*1000.0];
        let dx = origin[0] - eye_mm[0];
        let dy = origin[1] - eye_mm[1];
        let dz = origin[2] - eye_mm[2];
        let dist = (dx*dx + dy*dy + dz*dz).sqrt();
        // near plane is 3.0*0.001 m = 3.0 mm from eye
        assert!(dist < 10.0, "ray origin should be near-plane (~3mm from eye), got {:.2}mm", dist);
    }

    /// Full end-to-end: viewer ray generation + session ray_cast for far objects.
    #[test]
    fn test_viewer_to_session_end_to_end() {
        let cases: &[(&str, [f32; 3], f32)] = &[
            ("box_blue",  [2000.0, 2000.0, 1000.0], 400.0),
            ("box_green", [-2500.0, 1000.0, 2000.0], 350.0),
            ("box_red",   [1000.0, -2000.0, 1500.0], 500.0),
        ];

        for &(name, center, radius) in cases {
            let mut s = Session::new("t");
            s.add_mesh(make_box(center[0], center[1], center[2], radius), None);

            // Camera targeting this box from in front
            let eye_m = [center[0]*MM_TO_UNIT - 3.0, center[1]*MM_TO_UNIT, center[2]*MM_TO_UNIT];
            let tgt_m = [center[0]*MM_TO_UNIT, center[1]*MM_TO_UNIT, center[2]*MM_TO_UNIT];
            let up    = [0.0f32, 0.0, 1.0];
            let near  = 3.0_f32 * 0.001;
            let view  = build_view_matrix(eye_m, tgt_m, up);
            let proj  = build_proj_matrix(Tolerance::PI / 3.0, 16.0/9.0, near, 100_000.0);
            let vp    = (1920.0f32, 1080.0);

            // Click at screen center (should be pointing directly at box center)
            let (origin, dir) = screen_to_world_ray(&view, &proj, vp, (960.0, 540.0));
            let hits = s.ray_cast(&origin, &dir, 1.0);
            assert!(
                !hits.is_empty(),
                "end-to-end FAIL: {name} — viewer ray_cast returned no hits. origin=({:.1},{:.1},{:.1}) dir=({:.4},{:.4},{:.4})",
                origin[0], origin[1], origin[2], dir[0], dir[1], dir[2]
            );
        }
    }
}
