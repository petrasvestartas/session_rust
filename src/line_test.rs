use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_line_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Line;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Constructor
        let mut l = Line::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        // Setters
        l[0] = 10.0;
        l[1] = 20.0;
        l[2] = 30.0;
        l[3] = 40.0;
        l[4] = 50.0;
        l[5] = 60.0;

        // Getters
        let x0 = l[0];
        let y0 = l[1];
        let z0 = l[2];
        let x1 = l[3];
        let y1 = l[4];
        let z1 = l[5];

        // Minimal and Full String Representation
        let lstr = l.str();
        let lrepr = l.repr();

        // Copy (duplicate everything but guid)
        let lcopy = l.duplicate();
        let _lother = Line::new(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

        // No-copy operators
        let mut lmult = l.duplicate();
        lmult *= 2.0;
        let mut ldiv = l.duplicate();
        ldiv /= 2.0;
        let mut ladd = l.duplicate();
        ladd += &Vector::new(1.0, 1.0, 1.0);
        let mut lsub = l.duplicate();
        lsub -= &Vector::new(1.0, 1.0, 1.0);

        // Copy operators
        let rmul = &l * 2.0;
        let rdiv = &l / 2.0;
        let radd = &l + &Vector::new(1.0, 1.0, 1.0);
        let rdif = &l - &Vector::new(1.0, 1.0, 1.0);

        // Negation (flip start and end)
        let lneg = Line::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let neg = -lneg;

        // From points constructor
        let p0 = Point::new(1.0, 2.0, 3.0);
        let p1 = Point::new(4.0, 5.0, 6.0);
        let l2p = Line::from_points(&p0, &p1);

        // from_point_and_vector constructor
        let pv = Point::new(1.0, 2.0, 3.0);
        let vv = Vector::new(3.0, 4.0, 5.0);
        let l_pv = Line::from_point_and_vector(&pv, &vv);

        // from_point_direction_length constructor
        let pd = Point::new(0.0, 0.0, 0.0);
        let dd = Vector::new(1.0, 0.0, 0.0);
        let l_pdl = Line::from_point_direction_length(&pd, &dd, 5.0);

        // Line with custom color and width
        let mut lc = Line::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        lc.linecolor = Color::with_name(1.0, 0.0, 0.0, 1.0, "red");
        lc.width = 2.5;

        // with_name constructor
        let lwn = Line::with_name("custom", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

        // get_middle_line
        let (ms, me) = Line::get_middle_line(
            &Point::new(0.0, 0.0, 0.0), &Point::new(2.0, 0.0, 0.0),
            &Point::new(0.0, 2.0, 0.0), &Point::new(2.0, 2.0, 0.0));

        MINI_CHECK!(l.name == "my_line" && !l.guid().is_empty());
        MINI_CHECK!(l[0] == 10.0 && l[1] == 20.0 && l[2] == 30.0);
        MINI_CHECK!(x0 == 10.0 && y0 == 20.0 && z0 == 30.0);
        MINI_CHECK!(x1 == 40.0 && y1 == 50.0 && z1 == 60.0);
        MINI_CHECK!(lstr.contains("10") && lstr.contains("20") && lstr.contains("60"));
        MINI_CHECK!(lrepr.contains("my_line") && lrepr.contains("10") && lrepr.contains("Color"));
        MINI_CHECK!(lcopy.guid() != l.guid());
        MINI_CHECK!(lmult[0] == 20.0 && lmult[3] == 80.0);
        MINI_CHECK!(ldiv[0] == 5.0 && ldiv[3] == 20.0);
        MINI_CHECK!(ladd[0] == 11.0 && ladd[3] == 41.0);
        MINI_CHECK!(lsub[0] == 9.0 && lsub[3] == 39.0);
        MINI_CHECK!(rmul[0] == 20.0 && rmul[3] == 80.0);
        MINI_CHECK!(rdiv[0] == 5.0 && rdiv[3] == 20.0);
        MINI_CHECK!(radd[0] == 11.0 && radd[3] == 41.0);
        MINI_CHECK!(rdif[0] == 9.0 && rdif[3] == 39.0);
        MINI_CHECK!(neg[0] == 4.0 && neg[1] == 5.0 && neg[2] == 6.0);
        MINI_CHECK!(neg[3] == 1.0 && neg[4] == 2.0 && neg[5] == 3.0);
        MINI_CHECK!(l2p[0] == 1.0 && l2p[3] == 4.0);
        MINI_CHECK!(l_pv[0] == 1.0 && l_pv[1] == 2.0 && l_pv[2] == 3.0);
        MINI_CHECK!(l_pv[3] == 4.0 && l_pv[4] == 6.0 && l_pv[5] == 8.0);
        MINI_CHECK!(l_pdl[0] == 0.0 && l_pdl[3] == 5.0);
        MINI_CHECK!(lc.linecolor.r == 1.0 && lc.linecolor.g == 0.0 && lc.width == 2.5);
        MINI_CHECK!(lwn.name == "custom" && lwn[3] == 1.0);
        MINI_CHECK!(TOLERANCE.is_close(ms[1], 1.0) && TOLERANCE.is_close(me[1], 1.0));
    })
}

pub fn run_line_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Line;
        use crate::Xform;

        let mut l = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        l.xform = Xform::translation(10.0, 0.0, 0.0);
        let l_transformed = l.transformed(); // Make a copy
        l.transform(); // After the call, "xform" is reset

        MINI_CHECK!(l_transformed[0] == 10.0 && l_transformed[3] == 11.0);
        MINI_CHECK!(l[0] == 10.0 && l[3] == 11.0);
        MINI_CHECK!(l.xform == Xform::identity());
    })
}

pub fn run_line_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Line;

        let mut l = Line::new(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
        l.name = "test_line".to_string();

        //   jsondump()      │ String       │ to JSON string (object)
        //   jsonload(s)     │ String       │ from JSON string (object)
        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        // JSON object (string)
        let js = l.jsondump().unwrap();
        let loaded_j = Line::jsonload(&js).unwrap();

        MINI_CHECK!(loaded_j.name == "test_line");

        // String
        let s = l.file_json_dumps();
        let loaded_s = Line::file_json_loads(&s);
        MINI_CHECK!(loaded_s.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s[0], 42.1));

        // File
        let fname = "serialization/test_line.json";
        l.file_json_dump(fname).unwrap();
        let loaded = Line::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK!(TOLERANCE.is_close(loaded[3], 168.4));
        MINI_CHECK!(TOLERANCE.is_close(loaded[4], 210.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded[5], 252.6));
    })
}

pub fn run_line_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Line;

        let mut l = Line::new(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
        l.name = "test_line".to_string();

        //   pb_dumps()      | bytes        | to protobuf bytes
        //   pb_loads(b)     | bytes        | from protobuf bytes
        //   pb_dump(path)   | file         | write to file
        //   pb_load(path)   | file         | read from file

        // Bytes
        let b = l.pb_dumps();
        let loaded_s = Line::pb_loads(&b).unwrap();

        MINI_CHECK!(loaded_s.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s[0], 42.1));
        MINI_CHECK!(loaded_s.guid() == l.guid());

        // File
        let fname = "serialization/test_line.bin";
        l.pb_dump(fname);
        let loaded = Line::pb_load(fname);

        MINI_CHECK!(loaded.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK!(TOLERANCE.is_close(loaded[3], 168.4));
        MINI_CHECK!(TOLERANCE.is_close(loaded[4], 210.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded[5], 252.6));
        MINI_CHECK!(loaded.guid() == l.guid());
    })
}

pub fn run_line_length() -> TestResult {
    MINI_TEST!("Length", {
        use crate::Line;

        let l = Line::new(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        let ln = l.length();
        let lsq = l.squared_length();

        MINI_CHECK!(TOLERANCE.is_close(ln, 5.0));
        MINI_CHECK!(TOLERANCE.is_close(lsq, 25.0));
    })
}

pub fn run_line_to_vector() -> TestResult {
    MINI_TEST!("To Vector", {
        use crate::Line;

        let l = Line::new(1.0, 2.0, 3.0, 4.0, 6.0, 9.0);
        let v = l.to_vector();

        MINI_CHECK!(v[0] == 3.0 && v[1] == 4.0 && v[2] == 6.0);
    })
}

pub fn run_line_to_direction() -> TestResult {
    MINI_TEST!("To Direction", {
        use crate::Line;

        let l = Line::new(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        let d = l.to_direction();

        MINI_CHECK!(TOLERANCE.is_close(d[0], 0.6));
        MINI_CHECK!(TOLERANCE.is_close(d[1], 0.8));
        MINI_CHECK!(TOLERANCE.is_close(d[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d.magnitude(), 1.0));
    })
}

pub fn run_line_point_at() -> TestResult {
    MINI_TEST!("Point At", {
        use crate::Line;

        let l = Line::new(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let ps = l.point_at(0.0);
        let pm = l.point_at(0.5);
        let pe = l.point_at(1.0);

        MINI_CHECK!(ps[0] == 0.0 && ps[1] == 0.0 && ps[2] == 0.0);
        MINI_CHECK!(pm[0] == 5.0 && pm[1] == 5.0 && pm[2] == 5.0);
        MINI_CHECK!(pe[0] == 10.0 && pe[1] == 10.0 && pe[2] == 10.0);
    })
}

pub fn run_line_closest_point() -> TestResult {
    MINI_TEST!("Closest Point", {
        use crate::Line;
        use crate::Point;

        let l = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let p1 = Point::new(5.0, 5.0, 0.0);
        let p2 = Point::new(-5.0, 0.0, 0.0);
        let p3 = Point::new(15.0, 0.0, 0.0);
        let (t1, cp1) = l.closest_point(&p1, true);
        let (t2, cp2) = l.closest_point(&p2, true);
        let (t3, cp3) = l.closest_point(&p3, true);

        MINI_CHECK!(cp1[0] == 5.0 && cp1[1] == 0.0 && cp1[2] == 0.0);
        MINI_CHECK!(cp2[0] == 0.0 && cp2[1] == 0.0 && cp2[2] == 0.0);
        MINI_CHECK!(cp3[0] == 10.0 && cp3[1] == 0.0 && cp3[2] == 0.0);
        MINI_CHECK!(TOLERANCE.is_close(t1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(t2, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(t3, 1.0));
    })
}

pub fn run_line_start_end_center() -> TestResult {
    MINI_TEST!("Start End Center", {
        use crate::Line;

        let l = Line::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let start = l.start();
        let end = l.end();
        let center = l.center();

        MINI_CHECK!(start[0] == 1.0 && start[1] == 2.0 && start[2] == 3.0);
        MINI_CHECK!(end[0] == 4.0 && end[1] == 5.0 && end[2] == 6.0);
        MINI_CHECK!(center[0] == 2.5 && center[1] == 3.5 && center[2] == 4.5);
    })
}

pub fn run_line_fit_points() -> TestResult {
    MINI_TEST!("Fit Points", {
        use crate::Line;
        use crate::Point;

        let fit_pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.5),
            Point::new(2.0, 2.0, 1.0),
            Point::new(3.0, 3.0, 1.5),
        ];
        let l_fit = Line::fit_points(&fit_pts, None);

        MINI_CHECK!(l_fit.length() > 0.0);
    })
}

pub fn run_line_subdivide() -> TestResult {
    MINI_TEST!("Subdivide", {
        use crate::Line;

        let l = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);

        // subdivide by count
        let pts = l.subdivide(3);

        // subdivide_by_distance
        let pts_dist = l.subdivide_by_distance(2.5);

        MINI_CHECK!(pts.len() == 3);
        MINI_CHECK!(pts[0][0] == 0.0);
        MINI_CHECK!(pts[1][0] == 5.0);
        MINI_CHECK!(pts[2][0] == 10.0);
        MINI_CHECK!(pts_dist.len() == 5);
        MINI_CHECK!(TOLERANCE.is_close(pts_dist[0][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pts_dist[1][0], 2.5));
        MINI_CHECK!(TOLERANCE.is_close(pts_dist[4][0], 10.0));
    })
}

pub fn run_line_overlap() -> TestResult {
    MINI_TEST!("Overlap", {
        use crate::{Line, Point};
        let l0 = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        let l1 = Line::from_points(&Point::new(5.0, 0.0, 0.0), &Point::new(15.0, 0.0, 0.0));
        let out = l0.overlap(&l1).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(out.start()[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(out.end()[0], 10.0));
    })
}

pub fn run_line_overlap_average() -> TestResult {
    MINI_TEST!("Overlap Average", {
        use crate::{Line, Point};
        let l0 = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        let l1 = Line::from_points(&Point::new(5.0, 0.0, 0.0), &Point::new(15.0, 0.0, 0.0));
        let out = l0.overlap_average(&l1).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(out.start()[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(out.end()[0], 10.0));
    })
}

pub fn run_line_extend() -> TestResult {
    MINI_TEST!("Extend", {
        use crate::{Line, Point};
        let mut l = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        l.extend(1.0, 2.0);
        MINI_CHECK!(TOLERANCE.is_close(l.start()[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(l.end()[0], 12.0));
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Line", "Constructor", crate::line_test::run_line_constructor);
REGISTER_MINI_TEST!("Line", "Transformation", crate::line_test::run_line_transformation);
REGISTER_MINI_TEST!("Line", "Json Roundtrip", crate::line_test::run_line_json_roundtrip);
REGISTER_MINI_TEST!("Line", "Protobuf Roundtrip", crate::line_test::run_line_protobuf_roundtrip);
REGISTER_MINI_TEST!("Line", "Length", crate::line_test::run_line_length);
REGISTER_MINI_TEST!("Line", "To Vector", crate::line_test::run_line_to_vector);
REGISTER_MINI_TEST!("Line", "To Direction", crate::line_test::run_line_to_direction);
REGISTER_MINI_TEST!("Line", "Point At", crate::line_test::run_line_point_at);
REGISTER_MINI_TEST!("Line", "Closest Point", crate::line_test::run_line_closest_point);
REGISTER_MINI_TEST!("Line", "Start End Center", crate::line_test::run_line_start_end_center);
REGISTER_MINI_TEST!("Line", "Fit Points", crate::line_test::run_line_fit_points);
REGISTER_MINI_TEST!("Line", "Subdivide", crate::line_test::run_line_subdivide);
REGISTER_MINI_TEST!("Line", "Overlap", crate::line_test::run_line_overlap);
REGISTER_MINI_TEST!("Line", "Overlap Average", crate::line_test::run_line_overlap_average);
REGISTER_MINI_TEST!("Line", "Extend", crate::line_test::run_line_extend);
