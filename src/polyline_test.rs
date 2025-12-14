#[cfg(test)]
use crate::encoders::{json_dump, json_load};
#[cfg(test)]
use crate::{Plane, Point, Polyline, Vector};
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

#[test]
fn test_polyline_new() {
    let points = vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(0.0, 1.0, 0.0),
    ];
    let polyline = Polyline::new(points);
    assert_eq!(polyline.len(), 3);
    assert_eq!(polyline.segment_count(), 2);
}

#[test]
fn test_polyline_default() {
    let polyline = Polyline::default();
    assert_eq!(polyline.len(), 0);
    assert!(polyline.is_empty());
    assert_eq!(polyline.segment_count(), 0);
}

#[test]
fn test_polyline_length() {
    let points = vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
    ];
    let polyline = Polyline::new(points);
    let length = polyline.length();
    assert!((length - 2.0).abs() < 1e-5);
}

#[test]
fn test_polyline_add_point() {
    let mut polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
    assert_eq!(polyline.len(), 2);

    polyline.add_point(Point::new(1.0, 1.0, 0.0));
    assert_eq!(polyline.len(), 3);
    assert_eq!(polyline.segment_count(), 2);
}

#[test]
fn test_polyline_insert_point() {
    let mut polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0)]);

    polyline.insert_point(1, Point::new(1.0, 0.0, 0.0));
    assert_eq!(polyline.len(), 3);
    assert_eq!(polyline.get_points()[1][0], 1.0);
}

#[test]
fn test_polyline_remove_point() {
    let mut polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
    ]);

    let removed = polyline.remove_point(1);
    assert!(removed.is_some());
    assert_eq!(removed.unwrap()[0], 1.0);
    assert_eq!(polyline.len(), 2);
}

#[test]
fn test_polyline_reverse() {
    let mut polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
    ]);

    polyline.reverse();
    assert_eq!(polyline.get_points()[0][0], 2.0);
    assert_eq!(polyline.get_points()[1][0], 1.0);
    assert_eq!(polyline.get_points()[2][0], 0.0);
}

#[test]
fn test_polyline_reversed() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
    ]);

    let reversed = polyline.reversed();
    assert_eq!(reversed.get_points()[0][0], 2.0);
    assert_eq!(reversed.get_points()[1][0], 1.0);
    assert_eq!(reversed.get_points()[2][0], 0.0);

    // Original should be unchanged
    assert_eq!(polyline.get_points()[0][0], 0.0);
}

#[test]
fn test_polyline_add_assign_vector() {
    let mut polyline = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]);
    let v = Vector::new(4.0, 5.0, 6.0);
    polyline += &v;

    assert_eq!(polyline.get_points()[0][0], 5.0);
    assert_eq!(polyline.get_points()[0][1], 7.0);
    assert_eq!(polyline.get_points()[0][2], 9.0);
    assert_eq!(polyline.get_points()[1][0], 8.0);
    assert_eq!(polyline.get_points()[1][1], 10.0);
    assert_eq!(polyline.get_points()[1][2], 12.0);
}

#[test]
fn test_polyline_add_vector() {
    let polyline = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]);
    let v = Vector::new(4.0, 5.0, 6.0);
    let polyline2 = polyline + &v;

    assert_eq!(polyline2.get_points()[0][0], 5.0);
    assert_eq!(polyline2.get_points()[0][1], 7.0);
    assert_eq!(polyline2.get_points()[0][2], 9.0);
}

#[test]
fn test_polyline_sub_assign_vector() {
    let mut polyline = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]);
    let v = Vector::new(4.0, 5.0, 6.0);
    polyline -= &v;

    assert_eq!(polyline.get_points()[0][0], -3.0);
    assert_eq!(polyline.get_points()[0][1], -3.0);
    assert_eq!(polyline.get_points()[0][2], -3.0);
    assert_eq!(polyline.get_points()[1][0], 0.0);
    assert_eq!(polyline.get_points()[1][1], 0.0);
    assert_eq!(polyline.get_points()[1][2], 0.0);
}

#[test]
fn test_polyline_sub_vector() {
    let polyline = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]);
    let v = Vector::new(4.0, 5.0, 6.0);
    let polyline2 = polyline - &v;

    assert_eq!(polyline2.get_points()[0][0], -3.0);
    assert_eq!(polyline2.get_points()[0][1], -3.0);
    assert_eq!(polyline2.get_points()[0][2], -3.0);
    assert_eq!(polyline2.get_points()[1][0], 0.0);
    assert_eq!(polyline2.get_points()[1][1], 0.0);
    assert_eq!(polyline2.get_points()[1][2], 0.0);
}

#[test]
fn test_polyline_display() {
    let polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
    let display_str = format!("{polyline}");
    assert!(display_str.contains("Polyline"));
    assert!(display_str.contains("points=2"));
}

#[test]
fn test_polyline_json_serialization() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
    ]);

    let json = serde_json::to_string(&polyline).unwrap();
    let deserialized: Polyline = serde_json::from_str(&json).unwrap();

    assert_eq!(deserialized.len(), 3);
    assert_eq!(deserialized.get_points()[0][0], 0.0);
    assert_eq!(deserialized.get_points()[1][0], 1.0);
    assert_eq!(deserialized.get_points()[2][1], 1.0);
}

#[test]
fn test_polyline_to_json_data() {
    let polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);

    let json_string = polyline.jsondump().unwrap();
    assert!(json_string.contains("Polyline"));
    assert!(json_string.contains("coords"));
}

#[test]
fn test_polyline_from_json_data() {
    let polyline = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)]);

    let json_string = polyline.jsondump().unwrap();
    let deserialized = Polyline::jsonload(&json_string).unwrap();

    assert_eq!(deserialized.len(), 2);
    assert_eq!(deserialized.get_points()[0][0], 1.0);
    assert_eq!(deserialized.get_points()[1][0], 4.0);
}

#[test]
fn test_polyline_to_json_from_json() {
    let polyline = Polyline::new(vec![
        Point::new(1.0, 2.0, 3.0),
        Point::new(4.0, 5.0, 6.0),
        Point::new(7.0, 8.0, 9.0),
    ]);

    let filepath = "test_polyline.json";
    json_dump(&polyline, filepath, true).unwrap();
    let loaded = json_load::<Polyline>(filepath).unwrap();

    assert_eq!(loaded.len(), 3);
    assert_eq!(loaded.get_points()[0][0], 1.0);
    assert_eq!(loaded.get_points()[1][1], 5.0);
    assert_eq!(loaded.get_points()[2][2], 9.0);
}

#[test]
fn test_polyline_get_point() {
    let polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 2.0, 3.0)]);

    let point = polyline.get_point(1);
    assert!(point.is_some());
    assert_eq!(point.unwrap()[0], 1.0);

    let invalid = polyline.get_point(10);
    assert!(invalid.is_none());
}

#[test]
fn test_polyline_set_point() {
    let mut polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 2.0, 3.0)]);

    polyline.set_point(1, &Point::new(5.0, 6.0, 7.0));

    assert_eq!(polyline.get_points()[1][0], 5.0);
    assert_eq!(polyline.get_points()[1][1], 6.0);
    assert_eq!(polyline.get_points()[1][2], 7.0);
}

#[test]
fn test_polyline_shift() {
    let mut polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
    ]);

    polyline.shift(1);

    assert_eq!(polyline.get_points()[0][0], 1.0);
    assert_eq!(polyline.get_points()[1][0], 2.0);
    assert_eq!(polyline.get_points()[2][0], 0.0);
}

#[test]
fn test_polyline_length_squared() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
    ]);

    let length = polyline.length();
    assert!((length - 2.0).abs() < 1e-5);
}

#[test]
fn test_polyline_point_at_parameter() {
    let start = Point::new(0.0, 0.0, 0.0);
    let end = Point::new(2.0, 0.0, 0.0);

    let mid = Polyline::point_at_parameter(&start, &end, 0.5);
    assert_eq!(mid[0], 1.0);
    assert_eq!(mid[1], 0.0);
    assert_eq!(mid[2], 0.0);
}

#[test]
fn test_polyline_closest_point_to_line() {
    let line_start = Point::new(0.0, 0.0, 0.0);
    let line_end = Point::new(2.0, 0.0, 0.0);
    let test_point = Point::new(1.0, 1.0, 0.0);

    let t = Polyline::closest_point_to_line(&test_point, &line_start, &line_end);
    assert!((t - 0.5).abs() < 1e-5);
}

#[test]
fn test_polyline_line_line_overlap() {
    let line0_start = Point::new(0.0, 0.0, 0.0);
    let line0_end = Point::new(2.0, 0.0, 0.0);
    let line1_start = Point::new(1.0, 0.0, 0.0);
    let line1_end = Point::new(3.0, 0.0, 0.0);

    let overlap = Polyline::line_line_overlap(&line0_start, &line0_end, &line1_start, &line1_end);

    assert!(overlap.is_some());
    let (overlap_start, overlap_end) = overlap.unwrap();
    assert!((overlap_start[0] - 1.0).abs() < 1e-5);
    assert!((overlap_end[0] - 2.0).abs() < 1e-5);
}

#[test]
fn test_polyline_line_line_average() {
    let line0_start = Point::new(0.0, 0.0, 0.0);
    let line0_end = Point::new(2.0, 0.0, 0.0);
    let line1_start = Point::new(0.0, 2.0, 0.0);
    let line1_end = Point::new(2.0, 2.0, 0.0);

    let (avg_start, avg_end) =
        Polyline::line_line_average(&line0_start, &line0_end, &line1_start, &line1_end);

    assert!((avg_start[1] - 1.0).abs() < 1e-5);
    assert!((avg_end[1] - 1.0).abs() < 1e-5);
}

#[test]
fn test_polyline_line_line_overlap_average() {
    let line0_start = Point::new(0.0, 0.0, 0.0);
    let line0_end = Point::new(3.0, 0.0, 0.0);
    let line1_start = Point::new(1.0, 0.0, 0.0);
    let line1_end = Point::new(4.0, 0.0, 0.0);

    let (output_start, output_end) =
        Polyline::line_line_overlap_average(&line0_start, &line0_end, &line1_start, &line1_end);

    assert!(output_start[0] >= 0.0);
    assert!(output_end[0] <= 4.0);
}

#[test]
fn test_polyline_line_from_projected_points() {
    let line_start = Point::new(0.0, 0.0, 0.0);
    let line_end = Point::new(2.0, 0.0, 0.0);
    let points = vec![Point::new(0.5, 1.0, 0.0), Point::new(1.5, -1.0, 0.0)];

    let result = Polyline::line_from_projected_points(&line_start, &line_end, &points);

    assert!(result.is_some());
    let (output_start, output_end) = result.unwrap();
    assert!((output_start[0] - 0.5).abs() < 1e-5);
    assert!((output_end[0] - 1.5).abs() < 1e-5);
}

#[test]
fn test_polyline_closest_distance_and_point() {
    let polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0)]);
    let test_point = Point::new(1.0, 1.0, 0.0);

    let (distance, edge_id, closest_point) = polyline.closest_distance_and_point(&test_point);

    assert_eq!(edge_id, 0);
    assert!((closest_point[0] - 1.0).abs() < 1e-5);
    assert!((distance - 1.0).abs() < 1e-5);
}

#[test]
fn test_polyline_is_closed() {
    let open_polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
    ]);
    assert!(!open_polyline.is_closed());

    let closed_polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
        Point::new(0.0, 0.0, 0.0),
    ]);
    assert!(closed_polyline.is_closed());
}

#[test]
fn test_polyline_center() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
        Point::new(2.0, 2.0, 0.0),
        Point::new(0.0, 2.0, 0.0),
    ]);

    let c = polyline.center();
    assert!((c[0] - 1.0).abs() < 1e-5);
    assert!((c[1] - 1.0).abs() < 1e-5);
    assert!((c[2] - 0.0).abs() < 1e-5);
}

#[test]
fn test_polyline_center_vec() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
        Point::new(2.0, 2.0, 0.0),
    ]);

    let c = polyline.center_vec();
    assert!((c[0] - 4.0 / 3.0).abs() < 1e-5);
    assert!((c[1] - 2.0 / 3.0).abs() < 1e-5);
}

#[test]
fn test_polyline_get_average_plane() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(0.0, 1.0, 0.0),
    ]);

    let (_origin, _x_axis, _y_axis, z_axis) = polyline.get_average_plane();

    assert!((z_axis[2] - 1.0).abs() < 1e-5);
}

#[test]
fn test_polyline_get_fast_plane() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(0.0, 1.0, 0.0),
    ]);

    let (origin, _plane) = polyline.get_fast_plane();

    assert_eq!(origin[0], 0.0);
    assert_eq!(origin[1], 0.0);
    assert_eq!(origin[2], 0.0);
}

#[test]
fn test_polyline_get_middle_line() {
    let line0_start = Point::new(0.0, 0.0, 0.0);
    let line0_end = Point::new(2.0, 0.0, 0.0);
    let line1_start = Point::new(0.0, 2.0, 0.0);
    let line1_end = Point::new(2.0, 2.0, 0.0);

    let (output_start, output_end) =
        Polyline::get_middle_line(&line0_start, &line0_end, &line1_start, &line1_end);

    assert!((output_start[1] - 1.0).abs() < 1e-5);
    assert!((output_end[1] - 1.0).abs() < 1e-5);
}

#[test]
fn test_polyline_extend_line() {
    let mut start = Point::new(0.0, 0.0, 0.0);
    let mut end = Point::new(1.0, 0.0, 0.0);

    Polyline::extend_line(&mut start, &mut end, 0.5, 0.5);

    assert!((start[0] - (-0.5)).abs() < 1e-5);
    assert!((end[0] - 1.5).abs() < 1e-5);
}

#[test]
fn test_polyline_scale_line() {
    let mut start = Point::new(0.0, 0.0, 0.0);
    let mut end = Point::new(2.0, 0.0, 0.0);

    Polyline::scale_line(&mut start, &mut end, 0.25);

    assert!((start[0] - 0.5).abs() < 1e-5);
    assert!((end[0] - 1.5).abs() < 1e-5);
}

#[test]
fn test_polyline_extend_segment() {
    let mut polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);

    polyline.extend_segment(0, 0.5, 0.5, 0.0, 0.0);

    assert!((polyline.get_points()[0][0] - (-0.5)).abs() < 1e-5);
    assert!((polyline.get_points()[1][0] - 1.5).abs() < 1e-5);
}

#[test]
fn test_polyline_extend_segment_equally_static() {
    let mut start = Point::new(0.0, 0.0, 0.0);
    let mut end = Point::new(1.0, 0.0, 0.0);

    Polyline::extend_segment_equally_static(&mut start, &mut end, 0.5, 0.0);

    assert!((start[0] - (-0.5)).abs() < 1e-5);
    assert!((end[0] - 1.5).abs() < 1e-5);
}

#[test]
fn test_polyline_extend_segment_equally() {
    let mut polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);

    polyline.extend_segment_equally(0, 0.5, 0.0);

    assert!((polyline.get_points()[0][0] - (-0.5)).abs() < 1e-5);
    assert!((polyline.get_points()[1][0] - 1.5).abs() < 1e-5);
}

#[test]
fn test_polyline_move_by() {
    let mut polyline = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
    let translation = Vector::new(1.0, 1.0, 1.0);

    polyline.move_by(&translation);

    assert_eq!(polyline.get_points()[0][0], 1.0);
    assert_eq!(polyline.get_points()[0][1], 1.0);
    assert_eq!(polyline.get_points()[0][2], 1.0);
    assert_eq!(polyline.get_points()[1][0], 2.0);
    assert_eq!(polyline.get_points()[1][1], 1.0);
    assert_eq!(polyline.get_points()[1][2], 1.0);
}

#[test]
fn test_polyline_is_clockwise() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
    ]);
    let plane = Plane::default();

    let _clockwise = polyline.is_clockwise(&plane);
    // Just test it doesn't crash - the function returns a boolean value
}

#[test]
fn test_polyline_flip() {
    let mut polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
    ]);

    polyline.flip();

    assert_eq!(polyline.get_points()[0][0], 2.0);
    assert_eq!(polyline.get_points()[1][0], 1.0);
    assert_eq!(polyline.get_points()[2][0], 0.0);
}

#[test]
fn test_polyline_get_convex_corners() {
    let polyline = Polyline::new(vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 0.0, 0.0),
        Point::new(1.0, 1.0, 0.0),
        Point::new(0.0, 1.0, 0.0),
    ]);

    let convex_corners = polyline.get_convex_corners();

    assert_eq!(convex_corners.len(), 4);
}

#[test]
fn test_polyline_tween_two_polylines() {
    let polyline0 = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
    let polyline1 = Polyline::new(vec![Point::new(0.0, 2.0, 0.0), Point::new(1.0, 2.0, 0.0)]);

    let result = Polyline::tween_two_polylines(&polyline0, &polyline1, 0.5);

    assert!((result.get_points()[0][1] - 1.0).abs() < 1e-5);
    assert!((result.get_points()[1][1] - 1.0).abs() < 1e-5);
}

#[cfg(feature = "protobuf")]
#[test]
fn test_polyline_protobuf_roundtrip() {
    let mut polyline = Polyline::new(vec![
        Point::new(1.0, 2.0, 3.0),
        Point::new(4.0, 5.0, 6.0),
        Point::new(7.0, 8.0, 9.0),
    ]);
    polyline.guid = "test-guid-12345".to_string();
    polyline.name = "test_polyline".to_string();
    polyline.width = 2.5;
    polyline.linecolor.r = 255;
    polyline.linecolor.g = 128;
    polyline.linecolor.b = 64;

    // Serialize to protobuf
    let data = polyline.to_protobuf();
    assert!(!data.is_empty());

    // Deserialize from protobuf
    let loaded = Polyline::from_protobuf(&data).unwrap();

    // Verify all fields
    assert_eq!(loaded.guid, "test-guid-12345");
    assert_eq!(loaded.name, "test_polyline");
    assert_eq!(loaded.point_count(), 3);
    assert!((loaded.width - 2.5).abs() < 1e-10);
    assert_eq!(loaded.linecolor.r, 255);
    assert_eq!(loaded.linecolor.g, 128);
    assert_eq!(loaded.linecolor.b, 64);

    // Verify points
    let points = loaded.get_points();
    assert!((points[0][0] - 1.0).abs() < 1e-10);
    assert!((points[0][1] - 2.0).abs() < 1e-10);
    assert!((points[0][2] - 3.0).abs() < 1e-10);
    assert!((points[1][0] - 4.0).abs() < 1e-10);
    assert!((points[2][2] - 9.0).abs() < 1e-10);
}


pub fn run_polyline_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::Polyline;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Constructor with points
        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(1.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 1.0, 0.0);
        let p3 = Point::new(0.0, 1.0, 0.0);
        let pl = Polyline::new(vec![p0, p1, p2, p3]);

        // Basic properties
        let point_count = pl.len();
        let segment_count = pl.segment_count();
        let is_empty = pl.is_empty();

        // Get point
        let pt = pl.get_point(1).unwrap().clone();

        // Minimal and Full String Representation
        let plstr = pl.to_string();
        let plrepr = pl.repr();

        // Copy (duplicates everything except guid)
        let plcopy = pl.duplicate();
        let plother = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)]);

        // Translation operators
        let pl2 = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)]);
        let v = Vector::new(1.0, 1.0, 1.0);
        let pl_add = pl2.clone() + &v;
        let pl_sub = pl2 - &v;

        // Polyline with custom color and width
        let mut plc = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)]);
        plc.linecolor = Color::with_name(255, 0, 0, 255, "red");
        plc.width = 2.5;

        MINI_CHECK!(pl.name == "my_polyline" && point_count == 4 && !pl.guid.is_empty());
        MINI_CHECK!(segment_count == 3 && is_empty == false);
        MINI_CHECK!(pt[0] == 1.0 && pt[1] == 0.0 && pt[2] == 0.0);
        MINI_CHECK!(plstr.contains("Polyline") && plstr.contains("points=4"));
        MINI_CHECK!(plrepr.contains("Polyline(my_polyline") && plrepr.contains("4 points"));
        MINI_CHECK!(plcopy.coords == plother.coords);
        MINI_CHECK!(plcopy.guid != pl.guid);
        MINI_CHECK!(pl_add.get_points()[0][0] == 1.0 && pl_add.get_points()[0][1] == 1.0);
        MINI_CHECK!(pl_sub.get_points()[0][0] == -1.0 && pl_sub.get_points()[0][1] == -1.0);
        MINI_CHECK!(plc.linecolor.r == 255 && plc.linecolor.g == 0 && plc.width == 2.5);

    })
}

pub fn run_polyline_transformation() -> TestResult {
    MINI_TEST!("transformation", {
        use crate::Polyline;
        use crate::Point;
        use crate::Xform;

        let mut pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)]);
        pl.xform = Xform::translation(10.0, 0.0, 0.0);
        let pl_transformed = pl.transformed();
        pl.transform();

        MINI_CHECK!(pl_transformed.get_points()[0][0] == 10.0 && pl_transformed.get_points()[1][0] == 11.0);
        MINI_CHECK!(pl.get_points()[0][0] == 10.0 && pl.get_points()[1][0] == 11.0);
        MINI_CHECK!(pl.xform == Xform::identity());

    })
}

pub fn run_polyline_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::Polyline;
        use crate::Point;
        use crate::encoders::{json_dump, json_load};

        let mut pl = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0), Point::new(7.0, 8.0, 9.0), Point::new(10.0, 11.0, 12.0)]);
        pl.name = "test_polyline".to_string();

        // json_dump(fname) / json_load(fname) - file-based serialization
        let fname = "test_polyline.json";
        json_dump(&pl, fname, true).unwrap();
        let loaded: Polyline = json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_polyline");
        MINI_CHECK!(loaded.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[1][1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[2][2], 9.0));

    })
}

#[cfg(feature = "protobuf")]
pub fn run_polyline_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0), Point::new(7.0, 8.0, 9.0), Point::new(10.0, 11.0, 12.0)]);
        pl.name = "test_polyline".to_string();

        // protobuf_dump(fname) / protobuf_load(fname) - file-based serialization
        let fname = "test_polyline.bin";
        pl.protobuf_dump(fname);
        let loaded = Polyline::protobuf_load(fname);

        MINI_CHECK!(loaded.name == "test_polyline");
        MINI_CHECK!(loaded.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[1][1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[2][2], 9.0));

    })
}

pub fn run_polyline_length() -> TestResult {
    MINI_TEST!("length", {
        use crate::Polyline;
        use crate::Point;

        // L-shaped polyline: 1 unit right, 1 unit up, 1 unit left = 3 units total
        let pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)]);
        let ln = pl.length();
        let mag_sq = pl.magnitude_squared();

        MINI_CHECK!(TOLERANCE.is_close(ln, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(mag_sq, 3.0));

    })
}

pub fn run_polyline_center() -> TestResult {
    MINI_TEST!("center", {
        use crate::Polyline;
        use crate::Point;

        // Square polyline
        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0)
        ]);
        let c = pl.center();
        let cv = pl.center_vec();

        MINI_CHECK!(TOLERANCE.is_close(c[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(c[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(c[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cv[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(cv[1], 1.0));

    })
}

pub fn run_polyline_is_closed() -> TestResult {
    MINI_TEST!("is_closed", {
        use crate::Polyline;
        use crate::Point;

        // Open polyline
        let open_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0)
        ]);
        let is_open = open_pl.is_closed();

        // Closed polyline (first and last point same)
        let closed_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0)
        ]);
        let is_closed = closed_pl.is_closed();

        MINI_CHECK!(is_open == false);
        MINI_CHECK!(is_closed == true);

    })
}

pub fn run_polyline_reverse() -> TestResult {
    MINI_TEST!("reverse", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0)]);

        // Test reversed() returns new polyline
        let rev = pl.reversed();
        let orig_first = pl.get_points()[0][0];
        let rev_first = rev.get_points()[0][0];

        // Test reverse() in place
        pl.reverse();
        let in_place_first = pl.get_points()[0][0];

        MINI_CHECK!(orig_first == 0.0);
        MINI_CHECK!(rev_first == 3.0);
        MINI_CHECK!(in_place_first == 3.0);

    })
}

pub fn run_polyline_closest_point() -> TestResult {
    MINI_TEST!("closest_point", {
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0), Point::new(2.0, 2.0, 0.0), Point::new(0.0, 2.0, 0.0)]);
        let test_pt = Point::new(1.0, 1.0, 0.0);
        let (distance, edge_id, closest) = pl.closest_distance_and_point(&test_pt);

        MINI_CHECK!(edge_id == 0);
        MINI_CHECK!(TOLERANCE.is_close(closest[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(closest[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(distance, 1.0));



    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Polyline", "constructor", crate::polyline_test::run_polyline_constructor);
REGISTER_MINI_TEST!("Polyline", "transformation", crate::polyline_test::run_polyline_transformation);
REGISTER_MINI_TEST!("Polyline", "json_roundtrip", crate::polyline_test::run_polyline_json_roundtrip);
#[cfg(feature = "protobuf")]
REGISTER_MINI_TEST!("Polyline", "protobuf_roundtrip", crate::polyline_test::run_polyline_protobuf_roundtrip);
REGISTER_MINI_TEST!("Polyline", "length", crate::polyline_test::run_polyline_length);
REGISTER_MINI_TEST!("Polyline", "center", crate::polyline_test::run_polyline_center);
REGISTER_MINI_TEST!("Polyline", "is_closed", crate::polyline_test::run_polyline_is_closed);
REGISTER_MINI_TEST!("Polyline", "reverse", crate::polyline_test::run_polyline_reverse);
REGISTER_MINI_TEST!("Polyline", "closest_point", crate::polyline_test::run_polyline_closest_point);
