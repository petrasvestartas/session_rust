use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_polyline_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
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

        // Index operator
        let pt_idx = &pl[1];
        let mut pl_copy = pl.duplicate();
        pl_copy[0].copy_from_slice(&[5.0, 6.0, 7.0]);

        // Minimal and Full String Representation
        let plstr = pl.to_string();
        let plrepr = pl.repr();

        // Copy (duplicates everything except guid)
        let plcopy = pl.duplicate();
        let plother = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);

        // No-copy operators
        let mut plmult = pl.duplicate();
        plmult *= 2.0;
        let mut pldiv = pl.duplicate();
        pldiv /= 2.0;
        let mut pladd = pl.duplicate();
        pladd += &Vector::new(1.0, 1.0, 1.0);
        let mut plsub = pl.duplicate();
        plsub -= &Vector::new(1.0, 1.0, 1.0);

        // Copy operators
        let rmul = pl.clone() * 2.0;
        let rdiv = pl.clone() / 2.0;
        let radd = pl.clone() + &Vector::new(1.0, 1.0, 1.0);
        let rdif = pl.clone() - &Vector::new(1.0, 1.0, 1.0);

        // Negation (reverse point order)
        let plneg = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
        ]);
        let neg = -plneg;

        // Polyline with custom color and width
        let mut plc = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        plc.linecolor = Color::with_name(255, 0, 0, 255, "red");
        plc.width = 2.5;

        MINI_CHECK!(pl.name == "my_polyline" && point_count == 4 && !pl.guid().is_empty());
        MINI_CHECK!(segment_count == 3 && is_empty == false);
        MINI_CHECK!(pt[0] == 1.0 && pt[1] == 0.0 && pt[2] == 0.0);
        MINI_CHECK!(pt_idx[0] == 1.0 && pl_copy[0][0] == 5.0 && pl_copy[0][1] == 6.0);
        MINI_CHECK!(plstr.contains("Polyline") && plstr.contains("points=4"));
        MINI_CHECK!(plrepr.contains("Polyline(my_polyline") && plrepr.contains("4 points"));
        MINI_CHECK!(plcopy.coords == plother.coords);
        MINI_CHECK!(plcopy.guid() != pl.guid());
        MINI_CHECK!(plmult.get_points()[1][0] == 2.0);
        MINI_CHECK!(pldiv.get_points()[1][0] == 0.5);
        MINI_CHECK!(pladd.get_points()[0][0] == 1.0 && pladd.get_points()[0][1] == 1.0);
        MINI_CHECK!(plsub.get_points()[0][0] == -1.0 && plsub.get_points()[0][1] == -1.0);
        MINI_CHECK!(rmul.get_points()[1][0] == 2.0);
        MINI_CHECK!(rdiv.get_points()[1][0] == 0.5);
        MINI_CHECK!(radd.get_points()[0][0] == 1.0 && radd.get_points()[0][1] == 1.0);
        MINI_CHECK!(rdif.get_points()[0][0] == -1.0 && rdif.get_points()[0][1] == -1.0);
        MINI_CHECK!(neg.get_points()[0][0] == 3.0 && neg.get_points()[3][0] == 0.0);
        MINI_CHECK!(plc.linecolor.r == 255 && plc.linecolor.g == 0 && plc.width == 2.5);

    })
}

pub fn run_polyline_from_coords() -> TestResult {
    MINI_TEST!("From Coords", {
        use crate::Polyline;

        let coords = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0];
        let pl = Polyline::from_coords(coords);

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[0][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[1][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[2][1], 1.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "From Coords", crate::polyline_test::run_polyline_from_coords);

pub fn run_polyline_from_sides() -> TestResult {
    MINI_TEST!("From Sides", {
        use crate::Polyline;

        let sq = Polyline::from_sides(4, 1.0, false);
        let sq_closed = Polyline::from_sides(4, 1.0, true);

        MINI_CHECK!(sq.point_count() == 4);
        MINI_CHECK!(sq_closed.point_count() == 5);
        MINI_CHECK!(sq_closed.is_closed());
    })
}
REGISTER_MINI_TEST!("Polyline", "From Sides", crate::polyline_test::run_polyline_from_sides);

pub fn run_polyline_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Polyline;
        use crate::Point;
        use crate::Xform;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        pl.xform = Xform::translation(10.0, 0.0, 0.0);
        let pl_transformed = pl.transformed();
        pl.transform();

        MINI_CHECK!(pl_transformed.get_points()[0][0] == 10.0);
        MINI_CHECK!(pl_transformed.get_points()[1][0] == 11.0);
        MINI_CHECK!(pl.get_points()[0][0] == 10.0 && pl.get_points()[1][0] == 11.0);
        MINI_CHECK!(pl.xform == Xform::identity());

    })
}

pub fn run_polyline_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Polyline;
        use crate::Point;
        use crate::file_encoders::{file_json_dump, file_json_load};

        let mut pl = Polyline::new(vec![
            Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0),
            Point::new(7.0, 8.0, 9.0), Point::new(10.0, 11.0, 12.0),
        ]);
        pl.name = "test_polyline".to_string();

        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        let fname = "serialization/test_polyline.json";
        file_json_dump(&pl, fname, true).unwrap();
        let loaded: Polyline = file_json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_polyline");
        MINI_CHECK!(loaded.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[1][1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[2][2], 9.0));

    })
}

pub fn run_polyline_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0),
            Point::new(7.0, 8.0, 9.0), Point::new(10.0, 11.0, 12.0),
        ]);
        pl.name = "test_polyline".to_string();

        // pb_dump(fname) / pb_load(fname) - file-based serialization
        let fname = "serialization/test_polyline.bin";
        pl.pb_dump(fname);
        let loaded = Polyline::pb_load(fname);

        MINI_CHECK!(loaded.name == "test_polyline");
        MINI_CHECK!(loaded.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[1][1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[2][2], 9.0));

    })
}

pub fn run_polyline_length() -> TestResult {
    MINI_TEST!("Length", {
        use crate::Polyline;
        use crate::Point;

        // L-shaped polyline: 1 unit right, 1 unit up, 1 unit left = 3 units total
        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        let ln = pl.length();
        let mag_sq = pl.magnitude_squared();

        MINI_CHECK!(TOLERANCE.is_close(ln, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(mag_sq, 3.0));

    })
}

pub fn run_polyline_center() -> TestResult {
    MINI_TEST!("Center", {
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

        MINI_CHECK!(TOLERANCE.is_close(c[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(c[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(c[2], 0.0));

    })
}

pub fn run_polyline_is_closed() -> TestResult {
    MINI_TEST!("Is Closed", {
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

pub fn run_polyline_closed() -> TestResult {
    MINI_TEST!("Closed", {
        use crate::Polyline;
        use crate::Point;

        let open_pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0)]);
        let closed_from_open = open_pl.closed();

        let closed_pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0)]);
        let closed_from_closed = closed_pl.closed();

        MINI_CHECK!(closed_from_open.point_count() == 5);
        MINI_CHECK!(closed_from_open.is_closed() == true);
        MINI_CHECK!(closed_from_closed.point_count() == 5);
        MINI_CHECK!(closed_from_closed.is_closed() == true);

    })
}

pub fn run_polyline_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
        ]);

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
    MINI_TEST!("Closest Point", {
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0), Point::new(0.0, 2.0, 0.0),
        ]);
        let test_pt = Point::new(1.0, 1.0, 0.0);
        let (distance, edge_id, closest) = pl.closest_distance_and_point(&test_pt);

        MINI_CHECK!(edge_id == 0);
        MINI_CHECK!(TOLERANCE.is_close(closest[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(closest[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(distance, 1.0));



    })
}

pub fn run_polyline_closest_point_to_line() -> TestResult {
    MINI_TEST!("Closest Point To Line", {
        use crate::Polyline;
        use crate::Point;

        let line_start = Point::new(0.0, 0.0, 0.0);
        let line_end = Point::new(2.0, 0.0, 0.0);
        let pt = Point::new(1.0, 1.0, 0.0);
        let t = Polyline::closest_point_to_line(&pt, &line_start, &line_end);

        MINI_CHECK!(TOLERANCE.is_close(t, 0.5));
    })
}
REGISTER_MINI_TEST!("Polyline", "Closest Point To Line", crate::polyline_test::run_polyline_closest_point_to_line);

pub fn run_polyline_line_line_overlap() -> TestResult {
    MINI_TEST!("Line Line Overlap", {
        use crate::Polyline;
        use crate::Point;

        let s0 = Point::new(0.0, 0.0, 0.0);
        let e0 = Point::new(2.0, 0.0, 0.0);
        let s1 = Point::new(1.0, 0.0, 0.0);
        let e1 = Point::new(3.0, 0.0, 0.0);
        let result = Polyline::line_line_overlap(&s0, &e0, &s1, &e1);

        MINI_CHECK!(result.is_some());
        let (os, oe) = result.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(os[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(oe[0], 2.0));

        let s2 = Point::new(5.0, 0.0, 0.0);
        let e2 = Point::new(6.0, 0.0, 0.0);
        let no_overlap = Polyline::line_line_overlap(&s0, &e0, &s2, &e2);
        MINI_CHECK!(no_overlap.is_none());
    })
}
REGISTER_MINI_TEST!("Polyline", "Line Line Overlap", crate::polyline_test::run_polyline_line_line_overlap);

pub fn run_polyline_line_line_average() -> TestResult {
    MINI_TEST!("Line Line Average", {
        use crate::Polyline;
        use crate::Point;

        let s0 = Point::new(0.0, 0.0, 0.0);
        let e0 = Point::new(2.0, 0.0, 0.0);
        let s1 = Point::new(0.0, 2.0, 0.0);
        let e1 = Point::new(2.0, 2.0, 0.0);
        let (os, oe) = Polyline::line_line_average(&s0, &e0, &s1, &e1);

        MINI_CHECK!(TOLERANCE.is_close(os[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(os[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(oe[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(oe[1], 1.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Line Line Average", crate::polyline_test::run_polyline_line_line_average);

pub fn run_polyline_line_line_overlap_average() -> TestResult {
    MINI_TEST!("Line Line Overlap Average", {
        use crate::Polyline;
        use crate::Point;

        let s0 = Point::new(0.0, 0.0, 0.0);
        let e0 = Point::new(2.0, 0.0, 0.0);
        let s1 = Point::new(1.0, 2.0, 0.0);
        let e1 = Point::new(3.0, 2.0, 0.0);
        let (os, oe) = Polyline::line_line_overlap_average(&s0, &e0, &s1, &e1);

        MINI_CHECK!(TOLERANCE.is_close(os[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(oe[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(os[1], 1.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Line Line Overlap Average", crate::polyline_test::run_polyline_line_line_overlap_average);

pub fn run_polyline_line_from_projected_points() -> TestResult {
    MINI_TEST!("Line From Projected Points", {
        use crate::Polyline;
        use crate::Point;

        let s = Point::new(0.0, 0.0, 0.0);
        let e = Point::new(4.0, 0.0, 0.0);
        let pts = vec![Point::new(1.0, 1.0, 0.0), Point::new(3.0, -1.0, 0.0)];
        let result = Polyline::line_from_projected_points(&s, &e, &pts);

        MINI_CHECK!(result.is_some());
        let (os, oe) = result.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(os[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(oe[0], 3.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Line From Projected Points", crate::polyline_test::run_polyline_line_from_projected_points);

pub fn run_polyline_point_in_polygon_2d() -> TestResult {
    MINI_TEST!("Point In Polygon 2d", {
        use crate::Polyline;
        use crate::Point;

        let sq = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);

        MINI_CHECK!(sq.point_in_polygon_2d(&Point::new(0.5, 0.5, 0.0)));
        MINI_CHECK!(!sq.point_in_polygon_2d(&Point::new(2.0, 2.0, 0.0)));
    })
}
REGISTER_MINI_TEST!("Polyline", "Point In Polygon 2d", crate::polyline_test::run_polyline_point_in_polygon_2d);

pub fn run_polyline_extend_segment() -> TestResult {
    MINI_TEST!("Extend Segment", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
        ]);
        pl.extend_segment(0, 0.5, 0.5, 0.0, 0.0);
        let first = pl.get_points()[0][0];
        let second = pl.get_points()[1][0];

        MINI_CHECK!(TOLERANCE.is_close(first, -0.5));
        MINI_CHECK!(TOLERANCE.is_close(second, 1.5));
    })
}

pub fn run_polyline_extend_segment_equally() -> TestResult {
    MINI_TEST!("Extend Segment Equally", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
        ]);
        pl.extend_segment_equally(0, 0.5, 0.0);
        let first = pl.get_points()[0][0];
        let second = pl.get_points()[1][0];

        MINI_CHECK!(TOLERANCE.is_close(first, -0.5));
        MINI_CHECK!(TOLERANCE.is_close(second, 1.5));
    })
}

pub fn run_polyline_extend_line_segment() -> TestResult {
    MINI_TEST!("Extend Line Segment", {
        use crate::Polyline;
        use crate::Point;

        let mut start = Point::new(0.0, 0.0, 0.0);
        let mut end = Point::new(1.0, 0.0, 0.0);
        Polyline::extend_line_segment(&mut start, &mut end, 0.5, 0.5);

        MINI_CHECK!(TOLERANCE.is_close(start[0], -0.5));
        MINI_CHECK!(TOLERANCE.is_close(end[0], 1.5));
    })
}
REGISTER_MINI_TEST!("Polyline", "Extend Line Segment", crate::polyline_test::run_polyline_extend_line_segment);

pub fn run_polyline_shrink_line_segment() -> TestResult {
    MINI_TEST!("Shrink Line Segment", {
        use crate::Polyline;
        use crate::Point;

        let mut start = Point::new(0.0, 0.0, 0.0);
        let mut end = Point::new(1.0, 0.0, 0.0);
        Polyline::shrink_line_segment(&mut start, &mut end, 0.1);

        MINI_CHECK!(TOLERANCE.is_close(start[0], 0.1));
        MINI_CHECK!(TOLERANCE.is_close(end[0], 0.9));
    })
}
REGISTER_MINI_TEST!("Polyline", "Shrink Line Segment", crate::polyline_test::run_polyline_shrink_line_segment);

pub fn run_polyline_get_points() -> TestResult {
    MINI_TEST!("Get Points", {
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        let points = pl.get_points();

        MINI_CHECK!(points.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(points[0][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(points[0][1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(points[1][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(points[1][1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(points[2][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(points[2][1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(points[3][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(points[3][1], 1.0));
    })
}

pub fn run_polyline_get_lines() -> TestResult {
    MINI_TEST!("Get Lines", {
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        let lines = pl.get_lines();

        MINI_CHECK!(lines.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(lines[0][0], 0.0) && TOLERANCE.is_close(lines[0][3], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(lines[1][0], 1.0) && TOLERANCE.is_close(lines[1][4], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(lines[2][0], 1.0) && TOLERANCE.is_close(lines[2][3], 0.0));
    })
}

pub fn run_polyline_add_point() -> TestResult {
    MINI_TEST!("Add Point", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
        ]);
        pl.add_point(Point::new(2.0, 0.0, 0.0));

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[2][0], 2.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Add Point", crate::polyline_test::run_polyline_add_point);

pub fn run_polyline_insert_point() -> TestResult {
    MINI_TEST!("Insert Point", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ]);
        pl.insert_point(1, Point::new(1.0, 0.0, 0.0));

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[1][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[2][0], 2.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Insert Point", crate::polyline_test::run_polyline_insert_point);

pub fn run_polyline_remove_point() -> TestResult {
    MINI_TEST!("Remove Point", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ]);
        let removed = pl.remove_point(1);

        MINI_CHECK!(removed.is_some());
        MINI_CHECK!(pl.point_count() == 2);
        MINI_CHECK!(TOLERANCE.is_close(removed.unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[1][0], 2.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Remove Point", crate::polyline_test::run_polyline_remove_point);

pub fn run_polyline_shift() -> TestResult {
    MINI_TEST!("Shift", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
        ]);
        pl.shift(1);
        let first_after_shift = pl.get_points()[0][0];
        pl.shift(-1);
        let first_after_unshift = pl.get_points()[0][0];

        MINI_CHECK!(TOLERANCE.is_close(first_after_shift, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(first_after_unshift, 0.0));
    })
}

pub fn run_polyline_point_at() -> TestResult {
    MINI_TEST!("Point At", {
        use crate::Polyline;
        use crate::Point;

        let start = Point::new(0.0, 0.0, 0.0);
        let end = Point::new(2.0, 0.0, 0.0);
        let mid = Polyline::point_at(&start, &end, 0.5);
        let quarter = Polyline::point_at(&start, &end, 0.25);

        MINI_CHECK!(TOLERANCE.is_close(mid[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(quarter[0], 0.5));
    })
}

pub fn run_polyline_is_clockwise() -> TestResult {
    MINI_TEST!("Is Clockwise", {
        use crate::Polyline;
        use crate::Point;
        use crate::Plane;

        // Clockwise square (when viewed from +Z)
        let cw_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(1.0, 0.0, 0.0),
        ]);
        // Counter-clockwise square
        let ccw_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        let plane = Plane::default();

        MINI_CHECK!(cw_pl.is_clockwise(&plane) == true);
        MINI_CHECK!(ccw_pl.is_clockwise(&plane) == false);
    })
}

pub fn run_polyline_convex_corners() -> TestResult {
    MINI_TEST!("Convex Corners", {
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        let corners = pl.get_convex_corners();

        MINI_CHECK!(corners.len() == 4);
    })
}

pub fn run_polyline_tween() -> TestResult {
    MINI_TEST!("Tween", {
        use crate::Polyline;
        use crate::Point;

        let pl0 = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);
        let pl1 = Polyline::new(vec![
            Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0), Point::new(2.0, 1.0, 0.0),
        ]);
        let tweened = Polyline::tween_two_polylines(&pl0, &pl1, 0.5);

        MINI_CHECK!(TOLERANCE.is_close(tweened.get_points()[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(tweened.get_points()[1][0], 2.0));
    })
}

pub fn run_polyline_average_plane() -> TestResult {
    MINI_TEST!("Average Plane", {
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0), Point::new(0.0, 2.0, 0.0),
        ]);
        let (origin, _x_axis, _y_axis, z_axis) = pl.get_average_plane();
        let (fast_origin, _fast_plane) = pl.get_fast_plane();

        MINI_CHECK!(TOLERANCE.is_close(origin[0], 1.0) && TOLERANCE.is_close(origin[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(z_axis[2].abs(), 1.0));
        MINI_CHECK!(fast_origin[0] >= 0.0);
    })
}

pub fn run_polyline_interpolate_points() -> TestResult {
    MINI_TEST!("Interpolate Points", {
        use crate::Polyline;
        use crate::Point;

        let from = Point::new(0.0, 0.0, 0.0);
        let to   = Point::new(4.0, 0.0, 0.0);

        // type 0: no endpoints → 3 interior points
        let pts0 = Polyline::interpolate_points(&from, &to, 3, 0);
        MINI_CHECK!(pts0.len() == 3);
        MINI_CHECK!((pts0[0][0] - 1.0).abs() < 1e-9);
        MINI_CHECK!((pts0[1][0] - 2.0).abs() < 1e-9);
        MINI_CHECK!((pts0[2][0] - 3.0).abs() < 1e-9);

        // type 1: both endpoints → 5 points
        let pts1 = Polyline::interpolate_points(&from, &to, 3, 1);
        MINI_CHECK!(pts1.len() == 5);
        MINI_CHECK!((pts1[0][0] - 0.0).abs() < 1e-9);
        MINI_CHECK!((pts1[4][0] - 4.0).abs() < 1e-9);
    })
}

pub fn run_polyline_quick_hull() -> TestResult {
    MINI_TEST!("Quick Hull", {
        use crate::Polyline;
        use crate::Point;

        // Square with interior point
        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(1.0, 1.0, 0.0), // interior, should be excluded
        ]);
        let hull = Polyline::quick_hull(&poly);
        MINI_CHECK!(hull.point_count() == 4);
    })
}

pub fn run_polyline_bounding_rectangle() -> TestResult {
    MINI_TEST!("Bounding Rectangle", {
        use crate::Polyline;
        use crate::Point;

        // Axis-aligned rectangle
        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 3.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let rect = Polyline::bounding_rectangle(&poly);
        MINI_CHECK!(rect.is_some());
        let r = rect.unwrap();
        MINI_CHECK!(r.point_count() == 5);
        // First == last (closed)
        let pts = r.get_points();
        MINI_CHECK!((pts[0][0] - pts[4][0]).abs() < 1e-6);
    })
}

pub fn run_polyline_grid_of_points() -> TestResult {
    MINI_TEST!("Grid Of Points In Polygon", {
        use crate::Polyline;
        use crate::Point;

        // Square 0..4 x 0..4 with div_dist=1
        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let pts = Polyline::grid_of_points_in_polygon(&poly, 0.0, 1.0, 200);
        MINI_CHECK!(pts.len() > 0);
        for p in &pts {
            MINI_CHECK!(p[0] >= -1e-9 && p[0] <= 4.0 + 1e-9);
            MINI_CHECK!(p[1] >= -1e-9 && p[1] <= 4.0 + 1e-9);
        }
    })
}

pub fn run_polyline_polylabel() -> TestResult {
    MINI_TEST!("Polylabel", {
        use crate::Polyline;
        use crate::Point;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let polys = vec![poly];
        let (c, _plane, r) = Polyline::polylabel(&polys, 0.5);
        MINI_CHECK!((c[0] - 5.0).abs() < 0.6);
        MINI_CHECK!((c[1] - 5.0).abs() < 0.6);
        MINI_CHECK!((r - 5.0).abs() < 0.6);
    })
}

pub fn run_polyline_polylabel_circle_division_points() -> TestResult {
    MINI_TEST!("Polylabel Circle Division Points", {
        use crate::Polyline;
        use crate::Point;
        use crate::Vector;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let polys = vec![poly];
        let dir = Vector::new(0.0, 0.0, 0.0);
        let pts = Polyline::polylabel_circle_division_points(&dir, &polys, 4, 0.5, 1.0, true);
        MINI_CHECK!(pts.len() == 4);
        for p in &pts {
            MINI_CHECK!(p[2].abs() < 1e-6);
        }
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Polyline", "Constructor", crate::polyline_test::run_polyline_constructor);
REGISTER_MINI_TEST!("Polyline", "Transformation", crate::polyline_test::run_polyline_transformation);
REGISTER_MINI_TEST!("Polyline", "Json Roundtrip", crate::polyline_test::run_polyline_json_roundtrip);
REGISTER_MINI_TEST!("Polyline", "Protobuf Roundtrip", crate::polyline_test::run_polyline_protobuf_roundtrip);
REGISTER_MINI_TEST!("Polyline", "Length", crate::polyline_test::run_polyline_length);
REGISTER_MINI_TEST!("Polyline", "Center", crate::polyline_test::run_polyline_center);
REGISTER_MINI_TEST!("Polyline", "Is Closed", crate::polyline_test::run_polyline_is_closed);
REGISTER_MINI_TEST!("Polyline", "Closed", crate::polyline_test::run_polyline_closed);
REGISTER_MINI_TEST!("Polyline", "Reverse", crate::polyline_test::run_polyline_reverse);
REGISTER_MINI_TEST!("Polyline", "Closest Point", crate::polyline_test::run_polyline_closest_point);
REGISTER_MINI_TEST!("Polyline", "Extend Segment", crate::polyline_test::run_polyline_extend_segment);
REGISTER_MINI_TEST!("Polyline", "Extend Segment Equally", crate::polyline_test::run_polyline_extend_segment_equally);
REGISTER_MINI_TEST!("Polyline", "Get Points", crate::polyline_test::run_polyline_get_points);
REGISTER_MINI_TEST!("Polyline", "Get Lines", crate::polyline_test::run_polyline_get_lines);
REGISTER_MINI_TEST!("Polyline", "Shift", crate::polyline_test::run_polyline_shift);
REGISTER_MINI_TEST!("Polyline", "Point At", crate::polyline_test::run_polyline_point_at);
REGISTER_MINI_TEST!("Polyline", "Is Clockwise", crate::polyline_test::run_polyline_is_clockwise);
REGISTER_MINI_TEST!("Polyline", "Convex Corners", crate::polyline_test::run_polyline_convex_corners);
REGISTER_MINI_TEST!("Polyline", "Tween", crate::polyline_test::run_polyline_tween);
REGISTER_MINI_TEST!("Polyline", "Average Plane", crate::polyline_test::run_polyline_average_plane);
REGISTER_MINI_TEST!("Polyline", "Interpolate Points", crate::polyline_test::run_polyline_interpolate_points);
REGISTER_MINI_TEST!("Polyline", "Quick Hull", crate::polyline_test::run_polyline_quick_hull);
REGISTER_MINI_TEST!("Polyline", "Bounding Rectangle", crate::polyline_test::run_polyline_bounding_rectangle);
REGISTER_MINI_TEST!("Polyline", "Grid Of Points In Polygon", crate::polyline_test::run_polyline_grid_of_points);
REGISTER_MINI_TEST!("Polyline", "Polylabel", crate::polyline_test::run_polyline_polylabel);
REGISTER_MINI_TEST!("Polyline", "Polylabel Circle Division Points", crate::polyline_test::run_polyline_polylabel_circle_division_points);

pub fn run_polyline_boolean_op() -> TestResult {
    MINI_TEST!("Boolean Op", {
        use crate::{Point, Polyline};
        let sq_a = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new( 1.0, -1.0, 0.0),
            Point::new( 1.0,  1.0, 0.0),
            Point::new(-1.0,  1.0, 0.0),
        ]);
        let sq_b = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ]);
        let sq_inside = Polyline::new(vec![
            Point::new(-0.5, -0.5, 0.0),
            Point::new( 0.5, -0.5, 0.0),
            Point::new( 0.5,  0.5, 0.0),
            Point::new(-0.5,  0.5, 0.0),
        ]);
        let sq_disjoint = Polyline::new(vec![
            Point::new(5.0, 5.0, 0.0),
            Point::new(6.0, 5.0, 0.0),
            Point::new(6.0, 6.0, 0.0),
            Point::new(5.0, 6.0, 0.0),
        ]);

        let isect = Polyline::boolean_op(&sq_a, &sq_b, 0);
        let uni = Polyline::boolean_op(&sq_a, &sq_b, 1);
        let diff = Polyline::boolean_op(&sq_a, &sq_b, 2);
        MINI_CHECK!(isect.len() == 1);
        MINI_CHECK!(isect[0].point_count() == 4);
        MINI_CHECK!(uni.len() == 1);
        MINI_CHECK!(uni[0].point_count() == 8);
        MINI_CHECK!(diff.len() == 1);
        MINI_CHECK!(diff[0].point_count() == 6);

        let isect_in = Polyline::boolean_op(&sq_a, &sq_inside, 0);
        let uni_in = Polyline::boolean_op(&sq_a, &sq_inside, 1);
        let diff_in = Polyline::boolean_op(&sq_a, &sq_inside, 2);
        MINI_CHECK!(isect_in.len() == 1);
        MINI_CHECK!(isect_in[0].point_count() == 4);
        MINI_CHECK!(uni_in.len() == 1);
        MINI_CHECK!(uni_in[0].point_count() == 4);
        MINI_CHECK!(diff_in.len() == 1);
        MINI_CHECK!(diff_in[0].point_count() == 4);

        let isect_dis = Polyline::boolean_op(&sq_a, &sq_disjoint, 0);
        let uni_dis = Polyline::boolean_op(&sq_a, &sq_disjoint, 1);
        let diff_dis = Polyline::boolean_op(&sq_a, &sq_disjoint, 2);
        MINI_CHECK!(isect_dis.len() == 0);
        MINI_CHECK!(uni_dis.len() == 2);
        MINI_CHECK!(diff_dis.len() == 1);
    })
}
REGISTER_MINI_TEST!("Polyline", "Boolean Op", crate::polyline_test::run_polyline_boolean_op);

pub fn run_polyline_boolean_op_plane() -> TestResult {
    MINI_TEST!("Boolean Op Plane", {
        use crate::{Plane, Point, Polyline, Vector};
        // Two overlapping squares lifted to z=5 and clipped against the z=5 plane
        let plane = Plane::from_point_normal(Point::new(0.0,0.0,5.0), Vector::new(0.0,0.0,1.0));
        let sq_a = Polyline::new(vec![Point::new(-1.0,-1.0,5.0), Point::new(1.0,-1.0,5.0), Point::new(1.0,1.0,5.0), Point::new(-1.0,1.0,5.0), Point::new(-1.0,-1.0,5.0)]);
        let sq_b = Polyline::new(vec![Point::new(0.0,0.0,5.0), Point::new(2.0,0.0,5.0), Point::new(2.0,2.0,5.0), Point::new(0.0,2.0,5.0), Point::new(0.0,0.0,5.0)]);
        let isect = Polyline::boolean_op_plane(&sq_a, &sq_b, &plane, 0);
        let uni = Polyline::boolean_op_plane(&sq_a, &sq_b, &plane, 1);
        let diff = Polyline::boolean_op_plane(&sq_a, &sq_b, &plane, 2);
        MINI_CHECK!(isect.len() == 1);
        MINI_CHECK!(uni.len() == 1);
        MINI_CHECK!(diff.len() == 1);
        for i in 0..isect[0].point_count() { MINI_CHECK!(TOLERANCE.is_close(isect[0][i][2], 5.0)); }
        for i in 0..uni[0].point_count() { MINI_CHECK!(TOLERANCE.is_close(uni[0][i][2], 5.0)); }
        for i in 0..diff[0].point_count() { MINI_CHECK!(TOLERANCE.is_close(diff[0][i][2], 5.0)); }
    })
}
REGISTER_MINI_TEST!("Polyline", "Boolean Op Plane", crate::polyline_test::run_polyline_boolean_op_plane);

pub fn run_polyline_merge_collinear() -> TestResult {
    MINI_TEST!("Merge Collinear", {
        use crate::Polyline;
        use crate::Point;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ]);
        pl.merge_collinear(1e-3);

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[1][0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_points()[2][1], 1.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Merge Collinear", crate::polyline_test::run_polyline_merge_collinear);

pub fn run_polyline_simplify_points() -> TestResult {
    MINI_TEST!("Simplify Points", {
        use crate::Point;
        use crate::Polyline;
        let mut pts = vec![];
        for i in 0..100_usize {
            let x = i as f32;
            let y = (i as f32 * 0.1).sin() * 0.001;
            let z = 0.0;
            pts.push(Point::new(x, y, z));
        }
        let result_tight = Polyline::simplify_points(&pts, 0.0001);
        let result_loose = Polyline::simplify_points(&pts, 0.01);
        let result_very_loose = Polyline::simplify_points(&pts, 1.0);

        MINI_CHECK!(result_tight.len() <= pts.len());
        MINI_CHECK!(result_loose.len() <= result_tight.len());
        MINI_CHECK!(result_very_loose.len() <= result_loose.len());
        MINI_CHECK!(result_tight[0][0] == pts[0][0]);
        MINI_CHECK!(result_tight[result_tight.len() - 1][0] == pts[pts.len() - 1][0]);
    })
}
REGISTER_MINI_TEST!("Polyline", "Simplify Points", crate::polyline_test::run_polyline_simplify_points);

pub fn run_polyline_simplify() -> TestResult {
    MINI_TEST!("Simplify", {
        use crate::Point;
        use crate::Polyline;
        let mut pts = vec![];
        for i in 0..20_usize {
            let x = i as f32;
            let y = 0.0;
            let z = 0.0;
            pts.push(Point::new(x, y, z));
        }
        let pl = Polyline::new(pts);
        let result = pl.simplify(0.001);

        MINI_CHECK!(result.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(result.get_point(0).unwrap()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.get_point(1).unwrap()[0], 19.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Simplify", crate::polyline_test::run_polyline_simplify);

pub fn run_polyline_simplify_collinear() -> TestResult {
    MINI_TEST!("Simplify Collinear", {
        use crate::Point;
        use crate::Polyline;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];
        let result = Polyline::simplify_points(&pts, 0.001);

        MINI_CHECK!(result.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(result[0][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result[result.len() - 1][0], 4.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Simplify Collinear", crate::polyline_test::run_polyline_simplify_collinear);

pub fn run_polyline_simplify_zigzag() -> TestResult {
    MINI_TEST!("Simplify Zigzag", {
        use crate::Point;
        use crate::Polyline;
        let mut pts = vec![];
        for i in 0..10_usize {
            let x = i as f32;
            let y = if i % 2 == 1 { 1.0 } else { 0.0 };
            let z = 0.0;
            pts.push(Point::new(x, y, z));
        }
        let result_tight = Polyline::simplify_points(&pts, 0.1);
        let result_loose = Polyline::simplify_points(&pts, 2.0);

        MINI_CHECK!(result_tight.len() == 10);
        MINI_CHECK!(result_loose.len() < result_tight.len());
    })
}
REGISTER_MINI_TEST!("Polyline", "Simplify Zigzag", crate::polyline_test::run_polyline_simplify_zigzag);

pub fn run_polyline_simplify_two_points() -> TestResult {
    MINI_TEST!("Simplify Two Points", {
        use crate::Point;
        use crate::Polyline;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 1.0),
        ];
        let result = Polyline::simplify_points(&pts, 0.001);

        MINI_CHECK!(result.len() == 2);
    })
}
REGISTER_MINI_TEST!("Polyline", "Simplify Two Points", crate::polyline_test::run_polyline_simplify_two_points);


pub fn run_polyline_transformed_xform() -> TestResult {
    MINI_TEST!("Transformed Xform", {
        use crate::{Point, Polyline, Xform};
        let pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
        let xf = Xform::translation(10.0, 0.0, 0.0);
        let pl_x = pl.transformed_xform(&xf);
        MINI_CHECK!(TOLERANCE.is_close(pl_x.get_point(0).unwrap()[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_x.get_point(1).unwrap()[0], 11.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Transformed Xform", crate::polyline_test::run_polyline_transformed_xform);

pub fn run_polyline_translate() -> TestResult {
    MINI_TEST!("Translate", {
        use crate::{Point, Polyline, Vector};
        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        pl.translate(&Vector::new(5.0, 0.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(0).unwrap()[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(2).unwrap()[0], 6.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Translate", crate::polyline_test::run_polyline_translate);

pub fn run_polyline_extend_edge_equally() -> TestResult {
    MINI_TEST!("Extend Edge Equally", {
        use crate::{Point, Polyline};
        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        pl.extend_edge_equally(0, 1.0);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(0).unwrap()[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(1).unwrap()[0], 11.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(4).unwrap()[0], -1.0));
    })
}
REGISTER_MINI_TEST!("Polyline", "Extend Edge Equally", crate::polyline_test::run_polyline_extend_edge_equally);
