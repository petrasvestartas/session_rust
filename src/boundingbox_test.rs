#[cfg(test)]
mod tests {
    use crate::boundingbox::BoundingBox;
    use crate::encoders::{json_dump, json_load};
    use crate::plane::Plane;
    use crate::point::Point;
    use crate::vector::Vector;

    #[test]
    fn test_box_default() {
        let b = BoundingBox::default();
        assert_eq!(b.center[0], 0.0);
        assert_eq!(b.center[1], 0.0);
        assert_eq!(b.center[2], 0.0);
        assert_eq!(b.x_axis[0], 1.0);
        assert_eq!(b.y_axis[1], 1.0);
        assert_eq!(b.z_axis[2], 1.0);
        assert_eq!(b.half_size[0], 0.5);
        assert_eq!(b.half_size[1], 0.5);
        assert_eq!(b.half_size[2], 0.5);
        assert!(!b.guid.is_empty());
    }

    #[test]
    fn test_box_new() {
        let center = Point::new(1.0, 2.0, 3.0);
        let x_axis = Vector::new(1.0, 0.0, 0.0);
        let y_axis = Vector::new(0.0, 1.0, 0.0);
        let z_axis = Vector::new(0.0, 0.0, 1.0);
        let half_size = Vector::new(2.0, 3.0, 4.0);

        let b = BoundingBox::new(center, x_axis, y_axis, z_axis, half_size);

        assert_eq!(b.center[0], 1.0);
        assert_eq!(b.center[1], 2.0);
        assert_eq!(b.center[2], 3.0);
        assert_eq!(b.half_size[0], 2.0);
        assert_eq!(b.half_size[1], 3.0);
        assert_eq!(b.half_size[2], 4.0);
    }

    #[test]
    fn test_box_from_plane() {
        let plane = Plane::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
        );
        let b = BoundingBox::from_plane(&plane, 4.0, 6.0, 8.0);

        assert_eq!(b.center[0], 0.0);
        assert_eq!(b.half_size[0], 2.0);
        assert_eq!(b.half_size[1], 3.0);
        assert_eq!(b.half_size[2], 4.0);
    }

    #[test]
    fn test_box_corners() {
        let center = Point::new(0.0, 0.0, 0.0);
        let x_axis = Vector::new(1.0, 0.0, 0.0);
        let y_axis = Vector::new(0.0, 1.0, 0.0);
        let z_axis = Vector::new(0.0, 0.0, 1.0);
        let half_size = Vector::new(1.0, 1.0, 1.0);

        let b = BoundingBox::new(center, x_axis, y_axis, z_axis, half_size);
        let corners = b.corners();

        assert_eq!(corners.len(), 8);
        assert_eq!(corners[0][0], 1.0);
        assert_eq!(corners[0][1], 1.0);
        assert_eq!(corners[0][2], -1.0);
    }

    #[test]
    fn test_box_two_rectangles() {
        let center = Point::new(0.0, 0.0, 0.0);
        let x_axis = Vector::new(1.0, 0.0, 0.0);
        let y_axis = Vector::new(0.0, 1.0, 0.0);
        let z_axis = Vector::new(0.0, 0.0, 1.0);
        let half_size = Vector::new(1.0, 1.0, 1.0);

        let b = BoundingBox::new(center, x_axis, y_axis, z_axis, half_size);
        let rects = b.two_rectangles();

        assert_eq!(rects.len(), 10);
        assert_eq!(rects[0][0], rects[4][0]);
        assert_eq!(rects[0][1], rects[4][1]);
        assert_eq!(rects[0][2], rects[4][2]);
    }

    #[test]
    fn test_box_collision_overlapping() {
        let box1 = BoundingBox::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 1.0, 1.0),
        );
        let box2 = BoundingBox::new(
            Point::new(0.5, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 1.0, 1.0),
        );

        assert!(box1.collides_with(&box2));
    }

    #[test]
    fn test_box_collision_separated() {
        let box1 = BoundingBox::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 1.0, 1.0),
        );
        let box2 = BoundingBox::new(
            Point::new(5.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 1.0, 1.0),
        );

        assert!(!box1.collides_with(&box2));
    }

    #[test]
    fn test_box_json_serialization() {
        let b = BoundingBox::new(
            Point::new(1.0, 2.0, 3.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(2.0, 3.0, 4.0),
        );

        let data = b.jsondump().unwrap();
        assert!(data.contains("\"type\""));
        assert!(data.contains("\"BoundingBox\""));
        assert!(data.contains("\"center\""));
        assert!(data.contains("\"x_axis\""));
        assert!(data.contains("\"y_axis\""));
        assert!(data.contains("\"z_axis\""));
        assert!(data.contains("\"half_size\""));
        assert!(data.contains("\"guid\""));
        assert!(data.contains("\"name\""));
    }

    #[test]
    fn test_box_json_round_trip() {
        let mut original = BoundingBox::new(
            Point::new(1.0, 2.0, 3.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(2.0, 3.0, 4.0),
        );
        original.name = "test_box".to_string();

        let data = original.jsondump().unwrap();
        let loaded = BoundingBox::jsonload(&data).unwrap();

        assert_eq!(loaded.center[0], original.center[0]);
        assert_eq!(loaded.center[1], original.center[1]);
        assert_eq!(loaded.center[2], original.center[2]);
        assert_eq!(loaded.half_size[0], original.half_size[0]);
        assert_eq!(loaded.half_size[1], original.half_size[1]);
        assert_eq!(loaded.half_size[2], original.half_size[2]);
        assert_eq!(loaded.name, original.name);
    }

    #[test]
    fn test_box_inflate() {
        let mut b = BoundingBox::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 2.0, 3.0),
        );

        b.inflate(0.5);

        assert_eq!(b.half_size[0], 1.5);
        assert_eq!(b.half_size[1], 2.5);
        assert_eq!(b.half_size[2], 3.5);
    }

    #[test]
    fn test_box_from_point() {
        let pt = Point::new(1.0, 2.0, 3.0);
        let b = BoundingBox::from_point(pt, 0.0);

        assert_eq!(b.center[0], 1.0);
        assert_eq!(b.center[1], 2.0);
        assert_eq!(b.center[2], 3.0);
        assert_eq!(b.half_size[0], 0.0);
        assert_eq!(b.half_size[1], 0.0);
        assert_eq!(b.half_size[2], 0.0);
    }

    #[test]
    fn test_box_from_point_with_inflate() {
        let pt = Point::new(1.0, 2.0, 3.0);
        let b = BoundingBox::from_point(pt, 0.5);

        assert_eq!(b.center[0], 1.0);
        assert_eq!(b.center[1], 2.0);
        assert_eq!(b.center[2], 3.0);
        assert_eq!(b.half_size[0], 0.5);
        assert_eq!(b.half_size[1], 0.5);
        assert_eq!(b.half_size[2], 0.5);
    }

    #[test]
    fn test_box_from_points() {
        let points = vec![Point::new(0.0, 0.0, 0.0), Point::new(2.0, 4.0, 6.0)];
        let b = BoundingBox::from_points(&points, 0.0);

        assert_eq!(b.center[0], 1.0);
        assert_eq!(b.center[1], 2.0);
        assert_eq!(b.center[2], 3.0);
        assert_eq!(b.half_size[0], 1.0);
        assert_eq!(b.half_size[1], 2.0);
        assert_eq!(b.half_size[2], 3.0);
    }

    #[test]
    fn test_box_from_points_with_inflate() {
        let points = vec![Point::new(0.0, 0.0, 0.0), Point::new(2.0, 4.0, 6.0)];
        let b = BoundingBox::from_points(&points, 0.5);

        assert_eq!(b.center[0], 1.0);
        assert_eq!(b.center[1], 2.0);
        assert_eq!(b.center[2], 3.0);
        assert_eq!(b.half_size[0], 1.5);
        assert_eq!(b.half_size[1], 2.5);
        assert_eq!(b.half_size[2], 3.5);
    }

    #[test]
    fn test_box_from_line() {
        let line = crate::line::Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let b = BoundingBox::from_line(&line, 0.0);

        assert_eq!(b.center[0], 5.0);
        assert_eq!(b.center[1], 0.0);
        assert_eq!(b.center[2], 0.0);
        assert_eq!(b.half_size[0], 5.0);
    }

    #[test]
    fn test_box_from_line_with_inflate() {
        let line = crate::line::Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let b = BoundingBox::from_line(&line, 1.0);

        assert_eq!(b.center[0], 5.0);
        assert_eq!(b.center[1], 0.0);
        assert_eq!(b.center[2], 0.0);
        assert_eq!(b.half_size[0], 6.0);
        assert_eq!(b.half_size[1], 1.0);
        assert_eq!(b.half_size[2], 1.0);
    }

    #[test]
    fn test_box_from_polyline() {
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ];
        let polyline = crate::polyline::Polyline::new(points);
        let b = BoundingBox::from_polyline(&polyline, 0.0);

        assert_eq!(b.center[0], 0.5);
        assert_eq!(b.center[1], 0.5);
        assert_eq!(b.center[2], 0.0);
        assert_eq!(b.half_size[0], 0.5);
        assert_eq!(b.half_size[1], 0.5);
        assert_eq!(b.half_size[2], 0.0);
    }

    #[test]
    fn test_box_from_polyline_with_inflate() {
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ];
        let polyline = crate::polyline::Polyline::new(points);
        let b = BoundingBox::from_polyline(&polyline, 0.5);

        assert_eq!(b.center[0], 0.5);
        assert_eq!(b.center[1], 0.5);
        assert_eq!(b.center[2], 0.0);
        assert_eq!(b.half_size[0], 1.0);
        assert_eq!(b.half_size[1], 1.0);
        assert_eq!(b.half_size[2], 0.5);
    }

    #[test]
    fn test_box_to_json_from_json() {
        let original = BoundingBox::new(
            Point::new(1.0, 2.0, 3.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(2.0, 3.0, 4.0),
        );
        let filename = "test_boundingbox.json";

        json_dump(&original, filename, true).unwrap();
        let loaded = json_load::<BoundingBox>(filename).unwrap();

        assert_eq!(loaded.center[0], original.center[0]);
        assert_eq!(loaded.center[1], original.center[1]);
        assert_eq!(loaded.center[2], original.center[2]);
        assert_eq!(loaded.half_size[0], original.half_size[0]);
        assert_eq!(loaded.half_size[1], original.half_size[1]);
        assert_eq!(loaded.half_size[2], original.half_size[2]);
        assert_eq!(loaded.name, original.name);
        assert_eq!(loaded.guid, original.guid);
    }
}
