#[cfg(test)]
mod tests {
    use crate::gpu_adapters::*;
    use crate::gpu_session::*;
    use crate::{Color, Line, Point, Polyline};

    #[test]
    fn point_to_vertex_position_matches() {
        let p = Point::new(1.0, 2.0, 3.0);
        let v = p.to_point_vertex();
        assert_eq!(v.position, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn line_to_two_vertices() {
        let l = Line::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let v = l.to_line_vertices();
        assert_eq!(v[0].position, [0.0, 0.0, 0.0]);
        assert_eq!(v[1].position, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn polyline_emits_linelist_indices() {
        let p = Polyline::from_coords(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0]);
        let (verts, inds) = p.to_line_vertices();
        assert_eq!(verts.len(), 3);
        // 2 segments → 4 indices
        assert_eq!(inds, vec![0, 1, 1, 2]);
        // All vertices share the polyline's linecolor
        assert_eq!(verts[0].color, verts[1].color);
    }

    #[test]
    fn point_vertex_carries_pointcolor() {
        let mut p = Point::new(0.0, 0.0, 0.0);
        p.pointcolor = Color::new(10, 20, 30, 40);
        let v = p.to_point_vertex();
        assert_eq!(v.color, [10, 20, 30, 40]);
    }

    #[test]
    fn mesh_no_normals_computes_flat() {
        use crate::Mesh;
        // Single triangle in the XY plane → normal should be +Z
        let mut m = Mesh::new();
        let a = m.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let b = m.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let c = m.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        m.add_face(vec![a, b, c], None);
        let (verts, _inds) = m.to_mesh_vertices();
        assert_eq!(verts.len(), 3);
        for v in &verts {
            assert!(
                (v.normal[2] - 1.0).abs() < 1e-4,
                "expected +Z normal, got {:?}",
                v.normal
            );
        }
    }

    #[test]
    fn color_to_rgba_f32_normalizes() {
        let c = Color::new(255, 128, 0, 200);
        let rgba = color_to_rgba_f32(&c);
        assert!((rgba[0] - 1.0).abs() < 1e-6);
        assert!((rgba[1] - 128.0 / 255.0).abs() < 1e-6);
        assert!((rgba[2] - 0.0).abs() < 1e-6);
        assert!((rgba[3] - 200.0 / 255.0).abs() < 1e-6);
    }
}
