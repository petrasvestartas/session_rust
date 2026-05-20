#[cfg(test)]
mod tests {
    use crate::session::Session;
    use crate::session_pick::{screen_to_world_ray, Ray};
    use crate::{Line, Point};

    fn identity() -> [[f32; 4]; 4] {
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    #[test]
    fn screen_center_with_identity_yields_ray_through_origin() {
        let ray = screen_to_world_ray(&identity(), &identity(), (800.0, 600.0), (400.0, 300.0));
        assert!(ray.origin[0].abs() < 1e-4);
        assert!(ray.origin[1].abs() < 1e-4);
        // Direction normalized along z
        let len = (ray.direction[0].powi(2)
            + ray.direction[1].powi(2)
            + ray.direction[2].powi(2))
        .sqrt();
        assert!((len - 1.0).abs() < 1e-4);
    }

    #[test]
    fn pick_by_ray_hits_a_point() {
        let mut s = Session::new("test");
        // Point at world origin.
        let _ = s.add_point(Point::new(0.0, 0.0, 0.0), None);
        // Ray from -Z looking toward +Z passes through it.
        let ray = Ray::new([0.0, 0.0, -5.0], [0.0, 0.0, 1.0]);
        let hits = s.pick_by_ray(ray, 0.1);
        assert!(!hits.is_empty(), "expected at least one hit");
    }

    #[test]
    fn pick_by_ray_hits_a_line() {
        let mut s = Session::new("test");
        // Line on the x-axis from (-1,0,0) to (1,0,0).
        let _ = s.add_line(Line::new(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0), None);
        // Ray from above looking down should hit the line at x=0.
        let ray = Ray::new([0.0, 5.0, 0.0], [0.0, -1.0, 0.0]);
        let hits = s.pick_by_ray(ray, 0.5);
        assert!(!hits.is_empty(), "expected at least one hit");
    }

    #[test]
    fn pick_by_ray_returns_empty_when_no_geometry() {
        let mut s = Session::new("test");
        let ray = Ray::new([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]);
        let hits = s.pick_by_ray(ray, 0.1);
        assert!(hits.is_empty());
    }
}
