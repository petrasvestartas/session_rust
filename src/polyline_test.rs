use crate::mini_test::TestResult;
use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_polyline_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(1.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 1.0, 0.0);
        let p3 = Point::new(0.0, 1.0, 0.0);
        let pl = Polyline::new(vec![p0, p1, p2, p3]);
        let point_count = pl.len();
        let segment_count = pl.segment_count();
        let is_empty = pl.is_empty();
        let pt = pl.get_point(1).unwrap();
        let pt_idx = &pl[1];
        let mut pl_copy = pl.duplicate();
        pl_copy.set_point(0, &Point::new(5.0, 6.0, 7.0));
        let plstr = pl.str();
        let plrepr = pl.repr();
        let plcopy = pl.duplicate();
        let plother = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);

        let mut plmult = pl.duplicate();
        plmult *= 2.0;
        let mut pldiv = pl.duplicate();
        pldiv /= 2.0;
        let mut pladd = pl.duplicate();
        pladd += &Vector::new(1.0, 1.0, 1.0);
        let mut plsub = pl.duplicate();
        plsub -= &Vector::new(1.0, 1.0, 1.0);

        let rmul = &pl * 2.0;
        let rdiv = &pl / 2.0;
        let radd = &pl + &Vector::new(1.0, 1.0, 1.0);
        let rdif = &pl - &Vector::new(1.0, 1.0, 1.0);

        let plneg = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);
        let neg = -plneg;

        let mut plc = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        plc.linecolor = Color::with_name(1.0, 0.0, 0.0, 1.0, "red");
        plc.width = 2.5;

        MINI_CHECK!(pl.name == "my_polyline" && !pl.guid().is_empty() && point_count == 4);
        MINI_CHECK!(segment_count == 3 && !is_empty);
        MINI_CHECK!(pt[0] == 1.0 && pt[1] == 0.0 && pt[2] == 0.0);
        MINI_CHECK!(pt_idx[0] == 1.0 && pl_copy[0][0] == 5.0 && pl_copy[0][1] == 6.0);
        MINI_CHECK!(plstr.contains("(0, 0, 0)"));
        MINI_CHECK!(plrepr.contains("Polyline(my_polyline"));
        MINI_CHECK!(plrepr.contains("4 points"));
        MINI_CHECK!(plcopy == plother);
        MINI_CHECK!(plcopy.guid() != pl.guid());
        MINI_CHECK!(plmult.get_point(1).unwrap()[0] == 2.0);
        MINI_CHECK!(pldiv.get_point(1).unwrap()[0] == 0.5);
        MINI_CHECK!(pladd.get_point(0).unwrap()[0] == 1.0 && pladd.get_point(0).unwrap()[1] == 1.0);
        MINI_CHECK!(
            plsub.get_point(0).unwrap()[0] == -1.0 && plsub.get_point(0).unwrap()[1] == -1.0
        );
        MINI_CHECK!(rmul.get_point(1).unwrap()[0] == 2.0);
        MINI_CHECK!(rdiv.get_point(1).unwrap()[0] == 0.5);
        MINI_CHECK!(radd.get_point(0).unwrap()[0] == 1.0 && radd.get_point(0).unwrap()[1] == 1.0);
        MINI_CHECK!(rdif.get_point(0).unwrap()[0] == -1.0 && rdif.get_point(0).unwrap()[1] == -1.0);
        MINI_CHECK!(neg.get_point(0).unwrap()[0] == 3.0 && neg.get_point(3).unwrap()[0] == 0.0);
        MINI_CHECK!(plc.linecolor[0] == 1.0 && plc.linecolor[1] == 0.0 && plc.width == 2.5);
    })
}

pub fn run_polyline_from_coords() -> TestResult {
    MINI_TEST!("From Coords", {
        use crate::Polyline;

        let coords = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0];
        let pl = Polyline::from_coords(coords);

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(0).unwrap()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(1).unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(2).unwrap()[1], 1.0));
    })
}

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

pub fn run_polyline_rectangle() -> TestResult {
    MINI_TEST!("Rectangle", {
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let o = Point::new(0.0, 0.0, 0.0);
        let r = Polyline::rectangle(&o, &Vector::x_axis(), &Vector::y_axis(), 2.0, 1.0, true);

        MINI_CHECK!(r.point_count() == 5);
        MINI_CHECK!(r.is_closed());
        MINI_CHECK!(TOLERANCE.is_close(r.get_point(2).unwrap()[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(r.get_point(2).unwrap()[1], 1.0));
    })
}

pub fn run_polyline_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Point;
        use crate::Polyline;
        use crate::Xform;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let pl_xf = Xform::translation(10.0, 0.0, 0.0);
        let pl_transformed = pl.transformed(&pl_xf);
        pl.transform(&pl_xf);

        MINI_CHECK!(
            pl_transformed.get_point(0).unwrap()[0] == 10.0
                && pl_transformed.get_point(1).unwrap()[0] == 11.0
        );
        MINI_CHECK!(pl.get_point(0).unwrap()[0] == 10.0 && pl.get_point(1).unwrap()[0] == 11.0);
    })
}

pub fn run_polyline_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(1.0, 2.0, 3.0),
            Point::new(4.0, 5.0, 6.0),
            Point::new(7.0, 8.0, 9.0),
            Point::new(10.0, 11.0, 12.0),
        ]);
        pl.name = "test_polyline".to_string();
        pl.dash = vec![3.0, 2.0];

        let j = pl.jsondump().unwrap();
        let loaded_j = Polyline::jsonload(&j).unwrap();

        let s = pl.file_json_dumps();
        let loaded_s = Polyline::file_json_loads(&s);

        let fname = "serialization/test_polyline.json";
        pl.file_json_dump(fname).unwrap();
        let loaded = Polyline::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded_j.name == "test_polyline");
        MINI_CHECK!(TOLERANCE.is_close(loaded_j.get_point(0).unwrap()[0], 1.0));
        MINI_CHECK!(loaded_s.name == "test_polyline");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s.get_point(0).unwrap()[0], 1.0));
        MINI_CHECK!(loaded.name == "test_polyline");
        MINI_CHECK!(loaded.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(0).unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(1).unwrap()[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(2).unwrap()[2], 9.0));
        MINI_CHECK!(loaded.dash == vec![3.0, 2.0]);
        MINI_CHECK!(loaded.guid() == pl.guid());
    })
}

pub fn run_polyline_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(1.0, 2.0, 3.0),
            Point::new(4.0, 5.0, 6.0),
            Point::new(7.0, 8.0, 9.0),
            Point::new(10.0, 11.0, 12.0),
        ]);
        pl.name = "test_polyline".to_string();
        pl.dash = vec![3.0, 2.0];

        let guid = pl.guid().to_string();
        let s = pl.pb_dumps();
        let loaded_s = Polyline::pb_loads(&s).unwrap();

        let fname = "serialization/test_polyline.bin";
        pl.pb_dump(fname).unwrap();
        let loaded = Polyline::pb_load(fname).unwrap();
        let converted = Polyline::from_proto(pl.to_proto());

        MINI_CHECK!(loaded_s.name == "test_polyline");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s.get_point(0).unwrap()[0], 1.0));
        MINI_CHECK!(loaded_s.guid() == guid);
        MINI_CHECK!(loaded.name == "test_polyline");
        MINI_CHECK!(loaded.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(0).unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(1).unwrap()[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(2).unwrap()[2], 9.0));
        MINI_CHECK!(loaded.dash == vec![3.0, 2.0]);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(converted == pl);
        MINI_CHECK!(converted.guid() == guid);
    })
}

pub fn run_polyline_length() -> TestResult {
    MINI_TEST!("Length", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let ln = pl.length();
        let mag_sq = pl.length_squared();

        MINI_CHECK!(TOLERANCE.is_close(ln, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(mag_sq, 3.0));
    })
}

pub fn run_polyline_center() -> TestResult {
    MINI_TEST!("Center", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ]);
        let c = pl.center();

        MINI_CHECK!(TOLERANCE.is_close(c[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(c[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(c[2], 0.0));
    })
}

pub fn run_polyline_is_closed() -> TestResult {
    MINI_TEST!("Is Closed", {
        use crate::Point;
        use crate::Polyline;

        let open_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let is_open = open_pl.is_closed();

        let closed_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let is_closed = closed_pl.is_closed();

        MINI_CHECK!(!is_open);
        MINI_CHECK!(is_closed);
    })
}

pub fn run_polyline_closed() -> TestResult {
    MINI_TEST!("Closed", {
        use crate::Point;
        use crate::Polyline;

        let open_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let closed_from_open = open_pl.closed();

        let closed_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let closed_from_closed = closed_pl.closed();

        MINI_CHECK!(closed_from_open.point_count() == 5);
        MINI_CHECK!(closed_from_open.is_closed());
        MINI_CHECK!(closed_from_closed.point_count() == 5);
        MINI_CHECK!(closed_from_closed.is_closed());
    })
}

pub fn run_polyline_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);

        let rev = pl.reversed();
        let orig_first = pl.get_point(0).unwrap()[0];
        let rev_first = rev.get_point(0).unwrap()[0];

        pl.reverse();
        let in_place_first = pl.get_point(0).unwrap()[0];

        MINI_CHECK!(orig_first == 0.0);
        MINI_CHECK!(rev_first == 3.0);
        MINI_CHECK!(in_place_first == 3.0);
    })
}

pub fn run_polyline_closest_point() -> TestResult {
    MINI_TEST!("Closest Point", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
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
        use crate::Point;
        use crate::Polyline;

        let line_start = Point::new(0.0, 0.0, 0.0);
        let line_end = Point::new(2.0, 0.0, 0.0);
        let pt = Point::new(1.0, 1.0, 0.0);
        let t = Polyline::closest_point_to_line(&pt, &line_start, &line_end);

        MINI_CHECK!(TOLERANCE.is_close(t, 0.5));
    })
}

pub fn run_polyline_line_line_overlap() -> TestResult {
    MINI_TEST!("Line Line Overlap", {
        use crate::Point;
        use crate::Polyline;

        let s0 = Point::new(0.0, 0.0, 0.0);
        let e0 = Point::new(2.0, 0.0, 0.0);
        let s1 = Point::new(1.0, 0.0, 0.0);
        let e1 = Point::new(3.0, 0.0, 0.0);
        let result = Polyline::line_line_overlap(&s0, &e0, &s1, &e1);

        MINI_CHECK!(result.is_some());
        MINI_CHECK!(TOLERANCE.is_close(result.as_ref().unwrap().0[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(result.as_ref().unwrap().1[0], 2.0));

        let s2 = Point::new(5.0, 0.0, 0.0);
        let e2 = Point::new(6.0, 0.0, 0.0);
        let no_overlap = Polyline::line_line_overlap(&s0, &e0, &s2, &e2);

        MINI_CHECK!(no_overlap.is_none());
    })
}

pub fn run_polyline_line_line_average() -> TestResult {
    MINI_TEST!("Line Line Average", {
        use crate::Point;
        use crate::Polyline;

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

pub fn run_polyline_line_line_overlap_average() -> TestResult {
    MINI_TEST!("Line Line Overlap Average", {
        use crate::Point;
        use crate::Polyline;

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

pub fn run_polyline_line_from_projected_points() -> TestResult {
    MINI_TEST!("Line From Projected Points", {
        use crate::Point;
        use crate::Polyline;

        let s = Point::new(0.0, 0.0, 0.0);
        let e = Point::new(4.0, 0.0, 0.0);
        let pts = vec![Point::new(1.0, 1.0, 0.0), Point::new(3.0, -1.0, 0.0)];
        let result = Polyline::line_from_projected_points(&s, &e, &pts);

        MINI_CHECK!(result.is_some());
        MINI_CHECK!(TOLERANCE.is_close(result.as_ref().unwrap().0[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(result.as_ref().unwrap().1[0], 3.0));
    })
}

pub fn run_polyline_point_in_polygon_2d() -> TestResult {
    MINI_TEST!("Point In Polygon 2d", {
        use crate::Point;
        use crate::Polyline;

        let sq = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);

        MINI_CHECK!(sq.point_in_polygon_2d(&Point::new(0.5, 0.5, 0.0)));
        MINI_CHECK!(!sq.point_in_polygon_2d(&Point::new(2.0, 2.0, 0.0)));
    })
}

pub fn run_polyline_trim_rectangles_by_plane() -> TestResult {
    MINI_TEST!("Trim Rectangles By Plane", {
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let mut first = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let mut second = Polyline::new(vec![
            Point::new(0.0, 0.0, 1.0),
            Point::new(4.0, 0.0, 1.0),
            Point::new(4.0, 1.0, 1.0),
            Point::new(0.0, 1.0, 1.0),
            Point::new(0.0, 0.0, 1.0),
        ]);
        let plane =
            Plane::from_point_normal(Point::new(3.0, 0.0, 0.0), Vector::new(-1.0, 0.0, 0.0), None);
        let ok = Polyline::trim_rectangles_by_plane(&mut first, &mut second, &plane);

        MINI_CHECK!(ok);
        MINI_CHECK!(TOLERANCE.is_close(first[1][0], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(first[2][0], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(second[1][0], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(first[0][0], 0.0));
    })
}

pub fn run_polyline_extend_segment() -> TestResult {
    MINI_TEST!("Extend Segment", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);
        pl.extend_segment(0, 0.5, 0.5, 0.0, 0.0);
        let first = pl.get_point(0).unwrap()[0];
        let second = pl.get_point(1).unwrap()[0];

        MINI_CHECK!(TOLERANCE.is_close(first, -0.5));
        MINI_CHECK!(TOLERANCE.is_close(second, 1.5));
    })
}

pub fn run_polyline_extend_segment_equally() -> TestResult {
    MINI_TEST!("Extend Segment Equally", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);
        pl.extend_segment_equally(0, 0.5, 0.0);
        let first = pl.get_point(0).unwrap()[0];
        let second = pl.get_point(1).unwrap()[0];

        MINI_CHECK!(TOLERANCE.is_close(first, -0.5));
        MINI_CHECK!(TOLERANCE.is_close(second, 1.5));
    })
}

pub fn run_polyline_extend_line_segment() -> TestResult {
    MINI_TEST!("Extend Line Segment", {
        use crate::Point;
        use crate::Polyline;

        let mut start = Point::new(1.0, 0.0, 0.0);
        let mut end = Point::new(3.0, 0.0, 0.0);
        Polyline::extend_line_segment(&mut start, &mut end, 0.5, 0.5);

        MINI_CHECK!(TOLERANCE.is_close(start[0], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(end[0], 3.5));
    })
}

pub fn run_polyline_shrink_line_segment() -> TestResult {
    MINI_TEST!("Shrink Line Segment", {
        use crate::Point;
        use crate::Polyline;

        let mut start = Point::new(0.0, 0.0, 0.0);
        let mut end = Point::new(10.0, 0.0, 0.0);
        Polyline::shrink_line_segment(&mut start, &mut end, 0.1);

        MINI_CHECK!(TOLERANCE.is_close(start[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(end[0], 9.0));
    })
}

pub fn run_polyline_get_points() -> TestResult {
    MINI_TEST!("Get Points", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let points = pl.get_points();

        MINI_CHECK!(points.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(points[0][0], 0.0) && TOLERANCE.is_close(points[0][1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(points[1][0], 1.0) && TOLERANCE.is_close(points[1][1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(points[2][0], 1.0) && TOLERANCE.is_close(points[2][1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(points[3][0], 0.0) && TOLERANCE.is_close(points[3][1], 1.0));
    })
}

pub fn run_polyline_get_lines() -> TestResult {
    MINI_TEST!("Get Lines", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
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
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
        pl.add_point(Point::new(2.0, 0.0, 0.0));

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(2).unwrap()[0], 2.0));
    })
}

pub fn run_polyline_insert_point() -> TestResult {
    MINI_TEST!("Insert Point", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0)]);
        pl.insert_point(1, Point::new(1.0, 0.0, 0.0));

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(1).unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(2).unwrap()[0], 2.0));
    })
}

pub fn run_polyline_remove_point() -> TestResult {
    MINI_TEST!("Remove Point", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ]);
        let out = pl.remove_point(1);

        MINI_CHECK!(out.is_some());
        MINI_CHECK!(pl.point_count() == 2);
        MINI_CHECK!(TOLERANCE.is_close(out.unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(1).unwrap()[0], 2.0));
    })
}

pub fn run_polyline_shift() -> TestResult {
    MINI_TEST!("Shift", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);
        pl.shift(1);
        let first_after_shift = pl.get_point(0).unwrap()[0];
        pl.shift(-1);
        let first_after_unshift = pl.get_point(0).unwrap()[0];

        MINI_CHECK!(TOLERANCE.is_close(first_after_shift, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(first_after_unshift, 0.0));
    })
}

pub fn run_polyline_point_at() -> TestResult {
    MINI_TEST!("Point At", {
        use crate::Point;
        use crate::Polyline;

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
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;

        let cw_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
        ]);
        let ccw_pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let plane = Plane::default();

        MINI_CHECK!(cw_pl.is_clockwise(&plane));
        MINI_CHECK!(!ccw_pl.is_clockwise(&plane));
    })
}

pub fn run_polyline_convex_corners() -> TestResult {
    MINI_TEST!("Convex Corners", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let corners = pl.get_convex_corners();

        MINI_CHECK!(corners.len() == 4);
    })
}

pub fn run_polyline_tween() -> TestResult {
    MINI_TEST!("Tween", {
        use crate::Point;
        use crate::Polyline;

        let pl0 = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let pl1 = Polyline::new(vec![
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ]);
        let tweened = Polyline::tween_two_polylines(&pl0, &pl1, 0.5);

        MINI_CHECK!(TOLERANCE.is_close(tweened.get_point(0).unwrap()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(tweened.get_point(1).unwrap()[0], 2.0));
    })
}

pub fn run_polyline_average_plane() -> TestResult {
    MINI_TEST!("Average Plane", {
        use crate::Point;
        use crate::Polyline;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ]);
        let (origin, _x_axis, _y_axis, z_axis) = pl.get_average_plane();
        let (fast_origin, _fast_plane) = pl.get_fast_plane();

        MINI_CHECK!(TOLERANCE.is_close(origin[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(origin[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(z_axis[2].abs(), 1.0));
        MINI_CHECK!(fast_origin[0] >= 0.0);
    })
}

pub fn run_polyline_interpolate_points() -> TestResult {
    MINI_TEST!("Interpolate Points", {
        use crate::Point;
        use crate::Polyline;

        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(4.0, 0.0, 0.0);

        let pts0 = Polyline::interpolate_points(&a, &b, 3, 0);
        let pts1 = Polyline::interpolate_points(&a, &b, 3, 1);
        let pts2 = Polyline::interpolate_points(&a, &b, 3, 2);

        MINI_CHECK!(pts0.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pts0[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pts0[1][0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pts0[2][0], 3.0));
        MINI_CHECK!(pts1.len() == 5);
        MINI_CHECK!(TOLERANCE.is_close(pts1[0][0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pts1[4][0], 4.0));
        MINI_CHECK!(pts2.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(pts2[0][0], 0.0));
    })
}

pub fn run_polyline_quick_hull() -> TestResult {
    MINI_TEST!("Quick Hull", {
        use crate::Point;
        use crate::Polyline;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ]);
        let hull = Polyline::quick_hull(&poly);

        MINI_CHECK!(hull.point_count() == 4);
    })
}

pub fn run_polyline_bounding_rectangle() -> TestResult {
    MINI_TEST!("Bounding Rectangle", {
        use crate::Point;
        use crate::Polyline;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 3.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
        ]);
        let rect = Polyline::bounding_rectangle(&poly);

        MINI_CHECK!(rect.is_some() && rect.as_ref().unwrap().point_count() == 5);
        MINI_CHECK!(TOLERANCE.is_close(rect.as_ref().unwrap().get_point(0).unwrap()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(
            rect.as_ref().unwrap().get_point(0).unwrap()[0],
            rect.as_ref().unwrap().get_point(4).unwrap()[0]
        ));
    })
}

pub fn run_polyline_grid_of_points() -> TestResult {
    MINI_TEST!("Grid Of Points In Polygon", {
        use crate::Point;
        use crate::Polyline;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
        ]);
        let pts = Polyline::grid_of_points_in_polygon(&poly, 0.0, 1.0, 100);

        MINI_CHECK!(!pts.is_empty());

        for p in &pts {
            MINI_CHECK!(p[0] >= 0.0 && p[0] <= 4.0);
            MINI_CHECK!(p[1] >= 0.0 && p[1] <= 4.0);
        }
    })
}

pub fn run_polyline_polylabel() -> TestResult {
    MINI_TEST!("Polylabel", {
        use crate::Point;
        use crate::Polyline;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
        ]);
        let polys = vec![poly];
        let (c, _, r) = Polyline::polylabel(&polys, 0.5);

        MINI_CHECK!((c[0] - 5.0).abs() < 0.6);
        MINI_CHECK!((c[1] - 5.0).abs() < 0.6);
        MINI_CHECK!((r - 5.0).abs() < 0.6);
    })
}

pub fn run_polyline_polylabel_circle_division_points() -> TestResult {
    MINI_TEST!("Polylabel Circle Division Points", {
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
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

pub fn run_polyline_boolean_op() -> TestResult {
    MINI_TEST!("Boolean Op", {
        use crate::Point;
        use crate::Polyline;

        let sq_a = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
        ]);
        let sq_b = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
        ]);
        let sq_inside = Polyline::new(vec![
            Point::new(-0.5, -0.5, 0.0),
            Point::new(0.5, -0.5, 0.0),
            Point::new(0.5, 0.5, 0.0),
            Point::new(-0.5, 0.5, 0.0),
        ]);
        let sq_disjoint = Polyline::new(vec![
            Point::new(5.0, 5.0, 0.0),
            Point::new(6.0, 5.0, 0.0),
            Point::new(6.0, 6.0, 0.0),
            Point::new(5.0, 6.0, 0.0),
        ]);

        let isect = Polyline::boolean_op(&sq_a, &sq_b, 0, None);
        let uni = Polyline::boolean_op(&sq_a, &sq_b, 1, None);
        let diff = Polyline::boolean_op(&sq_a, &sq_b, 2, None);

        MINI_CHECK!(isect.len() == 1);
        MINI_CHECK!(isect[0].point_count() == 4);
        MINI_CHECK!(uni.len() == 1);
        MINI_CHECK!(uni[0].point_count() == 8);
        MINI_CHECK!(diff.len() == 1);
        MINI_CHECK!(diff[0].point_count() == 6);

        let isect_in = Polyline::boolean_op(&sq_a, &sq_inside, 0, None);
        let uni_in = Polyline::boolean_op(&sq_a, &sq_inside, 1, None);
        let diff_in = Polyline::boolean_op(&sq_a, &sq_inside, 2, None);

        MINI_CHECK!(isect_in.len() == 1);
        MINI_CHECK!(isect_in[0].point_count() == 4);
        MINI_CHECK!(uni_in.len() == 1);
        MINI_CHECK!(uni_in[0].point_count() == 4);
        MINI_CHECK!(diff_in.len() == 1);
        MINI_CHECK!(diff_in[0].point_count() == 4);

        let isect_dis = Polyline::boolean_op(&sq_a, &sq_disjoint, 0, None);
        let uni_dis = Polyline::boolean_op(&sq_a, &sq_disjoint, 1, None);
        let diff_dis = Polyline::boolean_op(&sq_a, &sq_disjoint, 2, None);

        MINI_CHECK!(isect_dis.is_empty());
        MINI_CHECK!(uni_dis.len() == 2);
        MINI_CHECK!(diff_dis.len() == 1);
    })
}

pub fn run_polyline_boolean_op_plane() -> TestResult {
    MINI_TEST!("Boolean Op Plane", {
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let origin = Point::new(0.0, 0.0, 5.0);
        let normal = Vector::new(0.0, 0.0, 1.0);
        let plane = Plane::from_point_normal(origin, normal, None);
        let sq_a = Polyline::new(vec![
            Point::new(-1.0, -1.0, 5.0),
            Point::new(1.0, -1.0, 5.0),
            Point::new(1.0, 1.0, 5.0),
            Point::new(-1.0, 1.0, 5.0),
            Point::new(-1.0, -1.0, 5.0),
        ]);
        let sq_b = Polyline::new(vec![
            Point::new(0.0, 0.0, 5.0),
            Point::new(2.0, 0.0, 5.0),
            Point::new(2.0, 2.0, 5.0),
            Point::new(0.0, 2.0, 5.0),
            Point::new(0.0, 0.0, 5.0),
        ]);
        let isect = Polyline::boolean_op(&sq_a, &sq_b, 0, Some(&plane));
        let uni = Polyline::boolean_op(&sq_a, &sq_b, 1, Some(&plane));
        let diff = Polyline::boolean_op(&sq_a, &sq_b, 2, Some(&plane));

        MINI_CHECK!(isect.len() == 1);
        MINI_CHECK!(uni.len() == 1);
        MINI_CHECK!(diff.len() == 1);

        for p in isect[0].get_points() {
            MINI_CHECK!(TOLERANCE.is_close(p[2], 5.0));
        }

        for p in uni[0].get_points() {
            MINI_CHECK!(TOLERANCE.is_close(p[2], 5.0));
        }

        for p in diff[0].get_points() {
            MINI_CHECK!(TOLERANCE.is_close(p[2], 5.0));
        }
    })
}

pub fn run_polyline_merge_collinear() -> TestResult {
    MINI_TEST!("Merge Collinear", {
        use crate::Point;
        use crate::Polyline;

        let mut pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ]);
        pl.merge_collinear(Tolerance::APPROXIMATION);

        MINI_CHECK!(pl.point_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(1).unwrap()[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.get_point(2).unwrap()[1], 1.0));
    })
}

pub fn run_polyline_simplify_points() -> TestResult {
    MINI_TEST!("Simplify Points", {
        use crate::Point;
        use crate::Polyline;

        let mut pts = Vec::new();

        for i in 0..100 {
            let x = i as f64;
            let y = (i as f64 * 0.1).sin() * 0.001;
            let z = 0.0;
            pts.push(Point::new(x, y, z));
        }

        let result_tight = Polyline::simplify_points(&pts, 0.0001);
        let result_loose = Polyline::simplify_points(&pts, 0.01);
        let result_very_loose = Polyline::simplify_points(&pts, 1.0);

        MINI_CHECK!(result_tight.len() <= pts.len());
        MINI_CHECK!(result_loose.len() <= result_tight.len());
        MINI_CHECK!(result_very_loose.len() <= result_loose.len());
        MINI_CHECK!(result_tight.first().unwrap()[0] == pts.first().unwrap()[0]);
        MINI_CHECK!(result_tight.last().unwrap()[0] == pts.last().unwrap()[0]);
    })
}

pub fn run_polyline_simplify() -> TestResult {
    MINI_TEST!("Simplify", {
        use crate::Point;
        use crate::Polyline;

        let mut pts = Vec::new();

        for i in 0..20 {
            let x = i as f64;
            let y = 0.0;
            let z = 0.0;
            pts.push(Point::new(x, y, z));
        }

        let pl = Polyline::new(pts);
        let result = pl.simplify(0.001);

        MINI_CHECK!(result.point_count() == 2);
        MINI_CHECK!(TOLERANCE.is_close(result.get_point(0).unwrap()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.get_point(1).unwrap()[0], 19.0));
    })
}

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
        MINI_CHECK!(TOLERANCE.is_close(result.first().unwrap()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.last().unwrap()[0], 4.0));
    })
}

pub fn run_polyline_simplify_zigzag() -> TestResult {
    MINI_TEST!("Simplify Zigzag", {
        use crate::Point;
        use crate::Polyline;

        let mut pts = Vec::new();

        for i in 0..10 {
            let x = i as f64;
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

pub fn run_polyline_simplify_two_points() -> TestResult {
    MINI_TEST!("Simplify Two Points", {
        use crate::Point;
        use crate::Polyline;

        let pts = vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)];
        let result = Polyline::simplify_points(&pts, 0.001);

        MINI_CHECK!(result.len() == 2);
    })
}

pub fn run_polyline_translate() -> TestResult {
    MINI_TEST!("Translate", {
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

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

pub fn run_polyline_extend_edge_equally() -> TestResult {
    MINI_TEST!("Extend Edge Equally", {
        use crate::Point;
        use crate::Polyline;

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

pub fn run_polyline_offset_sides() -> TestResult {
    MINI_TEST!("Offset Sides", {
        use crate::Point;
        use crate::Polyline;

        let square = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let split = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let moved = square.offset_sides(&[1.0, 0.0, 0.0, 0.0]);
        let stepped = split.offset_sides(&[1.0, 2.0, 0.0, 0.0, 0.0]);

        MINI_CHECK!(moved.point_count() == 5);
        MINI_CHECK!(moved.is_closed());
        MINI_CHECK!(TOLERANCE.is_close(moved.get_point(0).unwrap()[1], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(moved.get_point(1).unwrap()[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(moved.get_point(1).unwrap()[1], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(moved.get_point(2).unwrap()[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(stepped.get_point(0).unwrap()[1], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(stepped.get_point(1).unwrap()[1], -2.0));
        MINI_CHECK!(TOLERANCE.is_close(stepped.get_point(2).unwrap()[1], -2.0));
    })
}

pub fn run_polyline_offset_sides_degenerate() -> TestResult {
    MINI_TEST!("Offset Sides Degenerate", {
        use crate::Point;
        use crate::Polyline;

        let square = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let empty = Polyline::new(Vec::new()).offset_sides(&[1.0]);
        let short_distances = square.offset_sides(&[1.0, 1.0]);

        MINI_CHECK!(empty.point_count() == 0);
        MINI_CHECK!(short_distances.point_count() == 0);
    })
}

REGISTER_MINI_TEST!(
    "Polyline",
    "Constructor",
    crate::polyline_test::run_polyline_constructor
);
REGISTER_MINI_TEST!(
    "Polyline",
    "From Coords",
    crate::polyline_test::run_polyline_from_coords
);
REGISTER_MINI_TEST!(
    "Polyline",
    "From Sides",
    crate::polyline_test::run_polyline_from_sides
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Rectangle",
    crate::polyline_test::run_polyline_rectangle
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Transformation",
    crate::polyline_test::run_polyline_transformation
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Json Roundtrip",
    crate::polyline_test::run_polyline_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Protobuf Roundtrip",
    crate::polyline_test::run_polyline_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Length",
    crate::polyline_test::run_polyline_length
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Center",
    crate::polyline_test::run_polyline_center
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Is Closed",
    crate::polyline_test::run_polyline_is_closed
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Closed",
    crate::polyline_test::run_polyline_closed
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Reverse",
    crate::polyline_test::run_polyline_reverse
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Closest Point",
    crate::polyline_test::run_polyline_closest_point
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Closest Point To Line",
    crate::polyline_test::run_polyline_closest_point_to_line
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Line Line Overlap",
    crate::polyline_test::run_polyline_line_line_overlap
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Line Line Average",
    crate::polyline_test::run_polyline_line_line_average
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Line Line Overlap Average",
    crate::polyline_test::run_polyline_line_line_overlap_average
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Line From Projected Points",
    crate::polyline_test::run_polyline_line_from_projected_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Point In Polygon 2d",
    crate::polyline_test::run_polyline_point_in_polygon_2d
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Trim Rectangles By Plane",
    crate::polyline_test::run_polyline_trim_rectangles_by_plane
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Extend Segment",
    crate::polyline_test::run_polyline_extend_segment
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Extend Segment Equally",
    crate::polyline_test::run_polyline_extend_segment_equally
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Extend Line Segment",
    crate::polyline_test::run_polyline_extend_line_segment
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Shrink Line Segment",
    crate::polyline_test::run_polyline_shrink_line_segment
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Get Points",
    crate::polyline_test::run_polyline_get_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Get Lines",
    crate::polyline_test::run_polyline_get_lines
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Add Point",
    crate::polyline_test::run_polyline_add_point
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Insert Point",
    crate::polyline_test::run_polyline_insert_point
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Remove Point",
    crate::polyline_test::run_polyline_remove_point
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Shift",
    crate::polyline_test::run_polyline_shift
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Point At",
    crate::polyline_test::run_polyline_point_at
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Is Clockwise",
    crate::polyline_test::run_polyline_is_clockwise
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Convex Corners",
    crate::polyline_test::run_polyline_convex_corners
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Tween",
    crate::polyline_test::run_polyline_tween
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Average Plane",
    crate::polyline_test::run_polyline_average_plane
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Interpolate Points",
    crate::polyline_test::run_polyline_interpolate_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Quick Hull",
    crate::polyline_test::run_polyline_quick_hull
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Bounding Rectangle",
    crate::polyline_test::run_polyline_bounding_rectangle
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Grid Of Points In Polygon",
    crate::polyline_test::run_polyline_grid_of_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Polylabel",
    crate::polyline_test::run_polyline_polylabel
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Polylabel Circle Division Points",
    crate::polyline_test::run_polyline_polylabel_circle_division_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Boolean Op",
    crate::polyline_test::run_polyline_boolean_op
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Boolean Op Plane",
    crate::polyline_test::run_polyline_boolean_op_plane
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Merge Collinear",
    crate::polyline_test::run_polyline_merge_collinear
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Simplify Points",
    crate::polyline_test::run_polyline_simplify_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Simplify",
    crate::polyline_test::run_polyline_simplify
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Simplify Collinear",
    crate::polyline_test::run_polyline_simplify_collinear
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Simplify Zigzag",
    crate::polyline_test::run_polyline_simplify_zigzag
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Simplify Two Points",
    crate::polyline_test::run_polyline_simplify_two_points
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Translate",
    crate::polyline_test::run_polyline_translate
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Extend Edge Equally",
    crate::polyline_test::run_polyline_extend_edge_equally
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Offset Sides",
    crate::polyline_test::run_polyline_offset_sides
);
REGISTER_MINI_TEST!(
    "Polyline",
    "Offset Sides Degenerate",
    crate::polyline_test::run_polyline_offset_sides_degenerate
);
