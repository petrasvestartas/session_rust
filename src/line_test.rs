use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_line_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;
        use crate::Line;
        use crate::Point;
        use crate::Vector;

        let mut line = Line::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        line[0] = 10.0;
        line[1] = 20.0;
        line[2] = 30.0;
        line[3] = 40.0;
        line[4] = 50.0;
        line[5] = 60.0;

        let x0 = line[0];
        let y0 = line[1];
        let z0 = line[2];
        let x1 = line[3];
        let y1 = line[4];
        let z1 = line[5];

        let lstr = line.str();
        let lrepr = line.repr();

        let lcopy = line.duplicate();
        let lother = Line::new(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);

        let mut lmult = line.duplicate();
        lmult *= 2.0;
        let mut ldiv = line.duplicate();
        ldiv /= 2.0;
        let mut ladd = line.duplicate();
        ladd += &Vector::new(1.0, 1.0, 1.0);
        let mut lsub = line.duplicate();
        lsub -= &Vector::new(1.0, 1.0, 1.0);

        let rmul = &line * 2.0;
        let rdiv = &line / 2.0;
        let radd = &line + &Vector::new(1.0, 1.0, 1.0);
        let rdif = &line - &Vector::new(1.0, 1.0, 1.0);

        let lneg = Line::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let neg = -&lneg;

        let p0 = Point::new(1.0, 2.0, 3.0);
        let p1 = Point::new(4.0, 5.0, 6.0);
        let l2p = Line::from_points(&p0, &p1);

        let pv = Point::new(1.0, 2.0, 3.0);
        let vv = Vector::new(3.0, 4.0, 5.0);
        let l_pv = Line::from_point_and_vector(&pv, &vv);

        let pd = Point::new(0.0, 0.0, 0.0);
        let dd = Vector::new(1.0, 0.0, 0.0);
        let l_pdl = Line::from_point_direction_length(&pd, &dd, 5.0);

        let mut lc = Line::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        lc.linecolor = Color::with_name(1.0, 0.0, 0.0, 1.0, "red");
        lc.width = 2.5;

        let lwn = Line::with_name("custom", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

        let (ms, me) = Line::get_middle_line(
            &Point::new(0.0, 0.0, 0.0),
            &Point::new(2.0, 0.0, 0.0),
            &Point::new(0.0, 2.0, 0.0),
            &Point::new(2.0, 2.0, 0.0),
        );

        MINI_CHECK!(line.name == "my_line");
        MINI_CHECK!(line[0] == 10.0 && line[1] == 20.0 && line[2] == 30.0);
        MINI_CHECK!(line.width == 1.0);
        MINI_CHECK!(line.linecolor == Color::black());
        MINI_CHECK!(line.guid() != "");
        MINI_CHECK!(
            x0 == 10.0 && y0 == 20.0 && z0 == 30.0 && x1 == 40.0 && y1 == 50.0 && z1 == 60.0
        );
        MINI_CHECK!(lstr == "10.000000, 20.000000, 30.000000, 40.000000, 50.000000, 60.000000");
        MINI_CHECK!(lrepr == "Line(my_line, 10.000000, 20.000000, 30.000000, 40.000000, 50.000000, 60.000000, Color(black, 0.0, 0.0, 0.0, 1.0), 1.000000)");
        MINI_CHECK!(lcopy == line && lcopy.guid() != line.guid());
        MINI_CHECK!(lother == line && lneg != line);
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
        MINI_CHECK!(lc.linecolor[0] == 1.0 && lc.linecolor[1] == 0.0 && lc.width == 2.5);
        MINI_CHECK!(lwn.name == "custom" && lwn[3] == 1.0);
        MINI_CHECK!(TOLERANCE.is_close(ms[1], 1.0) && TOLERANCE.is_close(me[1], 1.0));
    })
}

pub fn run_line_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Line;
        use crate::Xform;

        let mut line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let xform = Xform::translation(10.0, 0.0, 0.0);
        let moved = line.transformed(&xform);
        line.transform(&xform);

        MINI_CHECK!(moved[0] == 10.0 && moved[3] == 11.0);
        MINI_CHECK!(line[0] == 10.0 && line[3] == 11.0);
    })
}

pub fn run_line_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Line;

        let mut line = Line::new(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
        line.name = "test_line".to_string();
        line.dash = vec![3.0, 2.0];

        let j = line.jsondump().unwrap();
        let loaded_j = Line::jsonload(&j).unwrap();

        let s = line.file_json_dumps();
        let loaded_s = Line::file_json_loads(&s);

        let fname = "serialization/test_line.json";
        line.file_json_dump(fname).unwrap();
        let loaded = Line::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded_j.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded_j[0], 42.1));
        MINI_CHECK!(loaded_s.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s[0], 42.1));
        MINI_CHECK!(loaded.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK!(TOLERANCE.is_close(loaded[3], 168.4));
        MINI_CHECK!(TOLERANCE.is_close(loaded[4], 210.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded[5], 252.6));
        MINI_CHECK!(loaded.dash == vec![3.0, 2.0]);
    })
}

pub fn run_line_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Line;

        let mut line = Line::new(42.1, 84.2, 126.3, 168.4, 210.5, 252.6);
        line.name = "test_line".to_string();
        line.dash = vec![3.0, 2.0];

        let guid = line.guid().to_string();
        let s = line.pb_dumps();
        let loaded_s = Line::pb_loads(&s).unwrap();

        let fname = "serialization/test_line.bin";
        line.pb_dump(fname).unwrap();
        let loaded = Line::pb_load(fname).unwrap();
        let converted = Line::from_proto(line.to_proto());

        MINI_CHECK!(loaded_s.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s[0], 42.1));
        MINI_CHECK!(loaded_s.guid() == guid);
        MINI_CHECK!(loaded.name == "test_line");
        MINI_CHECK!(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], 126.3));
        MINI_CHECK!(TOLERANCE.is_close(loaded[3], 168.4));
        MINI_CHECK!(TOLERANCE.is_close(loaded[4], 210.5));
        MINI_CHECK!(TOLERANCE.is_close(loaded[5], 252.6));
        MINI_CHECK!(loaded.dash == vec![3.0, 2.0]);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(converted == line);
        MINI_CHECK!(converted.guid() == guid);
    })
}

pub fn run_line_length() -> TestResult {
    MINI_TEST!("Length", {
        use crate::Line;

        let line = Line::new(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        let ln = line.length();
        let lsq = line.squared_length();

        MINI_CHECK!(TOLERANCE.is_close(ln, 5.0));
        MINI_CHECK!(TOLERANCE.is_close(lsq, 25.0));
    })
}

pub fn run_line_to_vector() -> TestResult {
    MINI_TEST!("To Vector", {
        use crate::Line;

        let line = Line::new(1.0, 2.0, 3.0, 4.0, 6.0, 9.0);
        let v = line.to_vector();

        MINI_CHECK!(v[0] == 3.0 && v[1] == 4.0 && v[2] == 6.0);
    })
}

pub fn run_line_to_direction() -> TestResult {
    MINI_TEST!("To Direction", {
        use crate::Line;

        let line = Line::new(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        let d = line.to_direction();

        MINI_CHECK!(TOLERANCE.is_close(d[0], 0.6));
        MINI_CHECK!(TOLERANCE.is_close(d[1], 0.8));
        MINI_CHECK!(TOLERANCE.is_close(d[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d.magnitude(), 1.0));
    })
}

pub fn run_line_point_at() -> TestResult {
    MINI_TEST!("Point At", {
        use crate::Line;

        let line = Line::new(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let ps = line.point_at(0.0);
        let pm = line.point_at(0.5);
        let pe = line.point_at(1.0);

        MINI_CHECK!(ps[0] == 0.0 && ps[1] == 0.0 && ps[2] == 0.0);
        MINI_CHECK!(pm[0] == 5.0 && pm[1] == 5.0 && pm[2] == 5.0);
        MINI_CHECK!(pe[0] == 10.0 && pe[1] == 10.0 && pe[2] == 10.0);
    })
}

pub fn run_line_closest_point() -> TestResult {
    MINI_TEST!("Closest Point", {
        use crate::Line;
        use crate::Point;

        let line = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let p1 = Point::new(5.0, 5.0, 0.0);
        let p2 = Point::new(-5.0, 0.0, 0.0);
        let p3 = Point::new(15.0, 0.0, 0.0);
        let (t1, cp1) = line.closest_point(&p1, true);
        let (t2, cp2) = line.closest_point(&p2, true);
        let (t3, cp3) = line.closest_point(&p3, true);

        MINI_CHECK!(cp1[0] == 5.0 && cp1[1] == 0.0 && cp1[2] == 0.0);
        MINI_CHECK!(cp2[0] == 0.0 && cp2[1] == 0.0 && cp2[2] == 0.0);
        MINI_CHECK!(cp3[0] == 10.0 && cp3[1] == 0.0 && cp3[2] == 0.0);
        MINI_CHECK!(TOLERANCE.is_close(t1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(t2, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(t3, 1.0));
    })
}

pub fn run_line_closest_point_unlimited() -> TestResult {
    MINI_TEST!("Closest Point Unlimited", {
        use crate::Line;
        use crate::Point;

        let line = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let before = line.closest_point(&Point::new(-5.0, 2.0, 0.0), false);
        let after = line.closest_point(&Point::new(15.0, 3.0, 0.0), false);

        MINI_CHECK!(TOLERANCE.is_close(before.0, -0.5));
        MINI_CHECK!(TOLERANCE.is_point_close(&before.1, &Point::new(-5.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_close(after.0, 1.5));
        MINI_CHECK!(TOLERANCE.is_point_close(&after.1, &Point::new(15.0, 0.0, 0.0)));
    })
}

pub fn run_line_start_end_center() -> TestResult {
    MINI_TEST!("Start End Center", {
        use crate::Line;

        let line = Line::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let start = line.start();
        let end = line.end();
        let center = line.center();

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

        let l_vertical = Line::fit_points(
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 2.0, 0.0),
                Point::new(0.0, 3.0, 0.0),
            ],
            None,
        );

        MINI_CHECK!(l_vertical.to_direction()[1].abs() > 0.99);

        let l_skew = Line::fit_points(
            &[
                Point::new(3.0, 0.0, 0.0),
                Point::new(-3.0, 0.0, 0.0),
                Point::new(0.0, 2.4, 2.4),
                Point::new(0.0, -2.4, -2.4),
            ],
            None,
        );
        let skew = l_skew.to_direction();

        MINI_CHECK!(TOLERANCE.is_close(skew[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(skew[1].abs(), 0.5_f64.sqrt()));
        MINI_CHECK!(TOLERANCE.is_close(skew[1], skew[2]));
    })
}

pub fn run_line_fit_points_uneven() -> TestResult {
    MINI_TEST!("Fit Points Uneven", {
        use crate::Line;
        use crate::Point;

        let line = Line::fit_points(
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 9.0, 0.0),
            ],
            None,
        );

        MINI_CHECK!(TOLERANCE.is_close(line.length(), 9.0));
        MINI_CHECK!(TOLERANCE.is_point_close(&line.start(), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&line.end(), &Point::new(0.0, 9.0, 0.0)));
    })
}

pub fn run_line_subdivide() -> TestResult {
    MINI_TEST!("Subdivide", {
        use crate::Line;

        let line = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let pts = line.subdivide(3);
        let pts_dist = line.subdivide_by_distance(2.5);

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
        use crate::Line;
        use crate::Point;

        let l0 = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        let l1 = Line::from_points(&Point::new(5.0, 0.0, 0.0), &Point::new(15.0, 0.0, 0.0));
        let out = l0.overlap(&l1);

        MINI_CHECK!(out.is_some());
        MINI_CHECK!(TOLERANCE.is_close(out.as_ref().unwrap().start()[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(out.as_ref().unwrap().end()[0], 10.0));
    })
}

pub fn run_line_overlap_average() -> TestResult {
    MINI_TEST!("Overlap Average", {
        use crate::Line;
        use crate::Point;

        let l0 = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        let l1 = Line::from_points(&Point::new(5.0, 0.0, 0.0), &Point::new(15.0, 0.0, 0.0));
        let out = l0.overlap_average(&l1);

        MINI_CHECK!(out.is_some());
        MINI_CHECK!(TOLERANCE.is_close(out.as_ref().unwrap().start()[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(out.as_ref().unwrap().end()[0], 10.0));
    })
}

pub fn run_line_extend() -> TestResult {
    MINI_TEST!("Extend", {
        use crate::Line;
        use crate::Point;

        let mut line = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        line.extend(1.0, 2.0);

        MINI_CHECK!(TOLERANCE.is_close(line.start()[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(line.end()[0], 12.0));
    })
}

pub fn run_line_extend_keeps_properties() -> TestResult {
    MINI_TEST!("Extend Keeps Properties", {
        use crate::Color;
        use crate::Line;
        use crate::Point;

        let mut line = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(10.0, 0.0, 0.0));
        line.name = "beam".to_string();
        line.width = 3.0;
        line.dash = vec![2.0, 1.0];
        line.linecolor = Color::red();
        let guid = line.guid().to_string();
        line.extend(1.0, 2.0);

        MINI_CHECK!(line.name == "beam");
        MINI_CHECK!(line.width == 3.0);
        MINI_CHECK!(line.dash == vec![2.0, 1.0]);
        MINI_CHECK!(line.linecolor == Color::red());
        MINI_CHECK!(line.guid() == guid);
    })
}

REGISTER_MINI_TEST!(
    "Line",
    "Constructor",
    crate::line_test::run_line_constructor
);
REGISTER_MINI_TEST!(
    "Line",
    "Transformation",
    crate::line_test::run_line_transformation
);
REGISTER_MINI_TEST!(
    "Line",
    "Json Roundtrip",
    crate::line_test::run_line_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Line",
    "Protobuf Roundtrip",
    crate::line_test::run_line_protobuf_roundtrip
);
REGISTER_MINI_TEST!("Line", "Length", crate::line_test::run_line_length);
REGISTER_MINI_TEST!("Line", "To Vector", crate::line_test::run_line_to_vector);
REGISTER_MINI_TEST!(
    "Line",
    "To Direction",
    crate::line_test::run_line_to_direction
);
REGISTER_MINI_TEST!("Line", "Point At", crate::line_test::run_line_point_at);
REGISTER_MINI_TEST!(
    "Line",
    "Closest Point",
    crate::line_test::run_line_closest_point
);
REGISTER_MINI_TEST!(
    "Line",
    "Closest Point Unlimited",
    crate::line_test::run_line_closest_point_unlimited
);
REGISTER_MINI_TEST!(
    "Line",
    "Start End Center",
    crate::line_test::run_line_start_end_center
);
REGISTER_MINI_TEST!("Line", "Fit Points", crate::line_test::run_line_fit_points);
REGISTER_MINI_TEST!(
    "Line",
    "Fit Points Uneven",
    crate::line_test::run_line_fit_points_uneven
);
REGISTER_MINI_TEST!("Line", "Subdivide", crate::line_test::run_line_subdivide);
REGISTER_MINI_TEST!("Line", "Overlap", crate::line_test::run_line_overlap);
REGISTER_MINI_TEST!(
    "Line",
    "Overlap Average",
    crate::line_test::run_line_overlap_average
);
REGISTER_MINI_TEST!("Line", "Extend", crate::line_test::run_line_extend);
REGISTER_MINI_TEST!(
    "Line",
    "Extend Keeps Properties",
    crate::line_test::run_line_extend_keeps_properties
);
