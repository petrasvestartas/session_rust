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
