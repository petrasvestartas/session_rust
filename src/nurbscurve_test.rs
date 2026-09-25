#![allow(clippy::excessive_precision, clippy::needless_range_loop)]
use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_nurbscurve_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_domain(0.0, 1.0);

        let cstr = curve.str();
        let crepr = curve.repr();

        let ccopy = curve.duplicate();
        let cother = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.is_valid());
        MINI_CHECK!(curve.cv_count() == 4);
        MINI_CHECK!(curve.degree() == 2);
        MINI_CHECK!(curve.order() == 3);
        MINI_CHECK!(curve.name == "my_nurbscurve");
        MINI_CHECK!(!curve.guid().is_empty());
        MINI_CHECK!(cstr == "NurbsCurve(name=my_nurbscurve, degree=2, cvs=4)");
        MINI_CHECK!(crepr.contains("name=my_nurbscurve"));
        MINI_CHECK!(ccopy.cv_count() == curve.cv_count());
        MINI_CHECK!(ccopy.guid() != curve.guid());
        MINI_CHECK!(ccopy == curve);
        MINI_CHECK!(cother != curve);
    })
}

pub fn run_nurbscurve_create_interpolated() -> TestResult {
    MINI_TEST!("Create Interpolated", {
        use crate::nurbsknot::CurveInterpStyle;
        use crate::nurbsknot::CurveNurbsKnotStyle;
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(14.0, 9.0, 0.0),
            Point::new(21.0, 22.0, 0.0),
            Point::new(26.0, 10.0, 0.0),
            Point::new(35.0, 19.0, 0.0),
            Point::new(41.0, 13.0, 0.0),
        ];

        let c = NurbsCurve::create_interpolated(
            &points,
            CurveNurbsKnotStyle::Chord,
            CurveInterpStyle::Rhino,
        );

        MINI_CHECK!(c.is_valid());
        MINI_CHECK!(c.degree() == 3);
        MINI_CHECK!(c.order() == 4);
        MINI_CHECK!(c.cv_count() == 7);
        MINI_CHECK!(!c.is_rational());
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(c.domain_start()), &points[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(c.domain_end()), &points[4]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.get_cv(0).unwrap(), &points[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.get_cv(6).unwrap(), &points[4]));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &c.get_cv(1).unwrap(),
            &Point::new(15.342776949, 13.734888836, 0.0)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &c.get_cv(3).unwrap(),
            &Point::new(24.678472471, 0.354555126, 0.0)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &c.get_cv(5).unwrap(),
            &Point::new(39.626394361, 15.472490151, 0.0)
        ));

        let co = NurbsCurve::create_interpolated(
            &points,
            CurveNurbsKnotStyle::Chord,
            CurveInterpStyle::Occt,
        );

        MINI_CHECK!(co.cv_count() == 7);
        MINI_CHECK!(TOLERANCE.is_point_close(&co.get_cv(0).unwrap(), &points[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&co.get_cv(6).unwrap(), &points[4]));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &co.get_cv(1).unwrap(),
            &Point::new(17.3526678158, 24.4472657919, 0.0)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &co.get_cv(3).unwrap(),
            &Point::new(24.7854378511, 2.1457823679, 0.0)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &co.get_cv(5).unwrap(),
            &Point::new(39.1865250566, 18.5349257754, 0.0)
        ));

        let closed_pts = vec![
            Point::new(4.0, 20.0, 0.0),
            Point::new(-2.0, 20.0, 0.0),
            Point::new(-2.0, 25.0, 0.0),
            Point::new(-3.0, 28.0, 0.0),
            Point::new(-10.0, 28.0, 0.0),
            Point::new(-10.0, 21.0, 0.0),
            Point::new(-13.0, 16.0, 0.0),
            Point::new(-8.0, 14.0, 0.0),
            Point::new(-6.0, 11.0, 0.0),
            Point::new(0.0, 15.0, 0.0),
        ];

        let cp = NurbsCurve::create_interpolated(
            &closed_pts,
            CurveNurbsKnotStyle::ChordPeriodic,
            CurveInterpStyle::Rhino,
        );

        MINI_CHECK!(cp.is_valid());
        MINI_CHECK!(cp.degree() == 3);
        MINI_CHECK!(cp.cv_count() == 13);
        MINI_CHECK!(cp.is_closed());
    })
}

pub fn run_nurbscurve_create_from_parameters() -> TestResult {
    MINI_TEST!("Create From Parameters", {
        use crate::NurbsCurve;
        use crate::Point;

        let p4 = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 6.0, 0.0),
            Point::new(6.0, -3.0, 3.0),
            Point::new(10.0, 0.0, 0.0),
        ];

        let c = NurbsCurve::create_from_parameters(
            &p4,
            &[1.0, 1.0, 1.0, 1.0],
            &[0.0, 1.0],
            &[4, 4],
            3,
            false,
        );

        MINI_CHECK!(c.is_valid());
        MINI_CHECK!(c.degree() == 3);
        MINI_CHECK!(c.cv_count() == 4);
        MINI_CHECK!(!c.is_rational());
        MINI_CHECK!((c.domain_start() - 0.0).abs() < 1e-12 && (c.domain_end() - 1.0).abs() < 1e-12);
        MINI_CHECK!(TOLERANCE.is_point_close(&c.get_cv(0).unwrap(), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.get_cv(3).unwrap(), &Point::new(10.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(0.5), &Point::new(4.625, 1.125, 1.125)));

        let w = 0.5 * (2.0_f64).sqrt();
        let cpts = vec![
            Point::new(0.0, -1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0),
            Point::new(-1.0, 0.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(0.0, -1.0, 0.0),
        ];

        let circle = NurbsCurve::create_from_parameters(
            &cpts,
            &[1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0],
            &[0.0, 0.25, 0.5, 0.75, 1.0],
            &[3, 2, 2, 2, 3],
            2,
            false,
        );

        MINI_CHECK!(circle.is_valid());
        MINI_CHECK!(circle.degree() == 2);
        MINI_CHECK!(circle.cv_count() == 9);
        MINI_CHECK!(circle.is_rational());
        MINI_CHECK!(TOLERANCE.is_point_close(&circle.point_at(0.5), &Point::new(0.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&circle.point_at(0.125), &Point::new(-w, -w, 0.0)));

        for k in 0..=16 {
            let pp = circle.point_at(k as f64 / 16.0);

            MINI_CHECK!(((pp[0] * pp[0] + pp[1] * pp[1]).sqrt() - 1.0).abs() < 1e-9);
        }
    })
}

pub fn run_nurbscurve_create_fitted() -> TestResult {
    MINI_TEST!("Create Fitted", {
        use crate::NurbsCurve;
        use crate::Point;

        let mut pts: Vec<Point> = Vec::new();

        for i in 0..=20 {
            let t = i as f64 * 2.0 * PI / 20.0;
            pts.push(Point::new(t, 3.0 * t.sin(), 0.0));
        }

        let c = NurbsCurve::create_fitted(&pts, 8, 3, false);

        MINI_CHECK!(c.is_valid());
        MINI_CHECK!(c.degree() == 3);
        MINI_CHECK!(c.cv_count() == 8);
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(c.domain_start()), &pts[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(c.domain_end()), &pts[20]));

        let mut cpts: Vec<Point> = Vec::new();

        for i in 0..24 {
            let a = i as f64 * 2.0 * PI / 24.0;
            cpts.push(Point::new(a.cos(), a.sin(), 0.0));
        }

        let cp = NurbsCurve::create_fitted(&cpts, 10, 3, true);

        MINI_CHECK!(cp.is_valid());
        MINI_CHECK!(cp.is_closed());
        MINI_CHECK!(cp.cv_count() == 13);
    })
}

pub fn run_nurbscurve_join() -> TestResult {
    MINI_TEST!("Join", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Primitives;

        let arc1 = Primitives::arc(
            &Point::new(-1.0, 0.0, 0.0),
            &Point::new(0.0, 1.0, 0.0),
            &Point::new(1.0, 0.0, 0.0),
        );
        let mut arc2 = Primitives::arc(
            &Point::new(1.0, 0.0, 0.0),
            &Point::new(1.5, -1.0, 0.0),
            &Point::new(1.0, -2.0, 0.0),
        );
        let pts = vec![Point::new(1.0, -2.0, 0.0), Point::new(-1.0, 0.0, 0.0)];
        let line = NurbsCurve::create(false, 1, &pts);
        arc2.reverse();

        let joined = NurbsCurve::join(
            &[line.duplicate(), arc1.duplicate(), arc2.duplicate()],
            None,
        );

        MINI_CHECK!(joined.len() == 1);
        MINI_CHECK!(joined[0].is_valid());
        MINI_CHECK!(joined[0].is_closed());
        MINI_CHECK!(joined[0].degree() == 2);
        MINI_CHECK!(joined[0].cv_count() == 7);

        let l1 = NurbsCurve::create(
            false,
            1,
            &[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)],
        );
        let l2 = NurbsCurve::create(
            false,
            1,
            &[Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0)],
        );
        let l3 = NurbsCurve::create(
            false,
            1,
            &[Point::new(9.0, 9.0, 0.0), Point::new(8.0, 8.0, 0.0)],
        );

        let separate = NurbsCurve::join(&[l1, l3, l2], None);

        MINI_CHECK!(separate.len() == 2);
        MINI_CHECK!(separate[0].cv_count() == 3);
        MINI_CHECK!((separate[0].length(None) - 2.0).abs() < 1e-9);
    })
}

pub fn run_nurbscurve_attributes() -> TestResult {
    MINI_TEST!("Attributes", {
        use crate::NurbsCurve;
        use crate::Plane;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        let is_valid = curve.is_valid();
        let is_valid_nurbsknot_vector = curve.is_valid_nurbsknot_vector();
        let is_clamped_start = curve.is_clamped(0);
        let is_clamped_end = curve.is_clamped(1);
        let is_clamped_both = curve.is_clamped(2);

        MINI_CHECK!(is_valid);
        MINI_CHECK!(is_valid_nurbsknot_vector);
        MINI_CHECK!(is_clamped_start && is_clamped_end && is_clamped_both);

        let is_rational = curve.is_rational();
        let closed = curve.is_closed();
        let periodic = curve.is_periodic();
        let linear = curve.is_linear(None);
        let planar = curve.is_planar(None, None);
        let arc = curve.is_arc(None, None);
        let plane = Plane::xy_plane();
        let on_plane = curve.is_in_plane(&plane, None);
        let is_open = curve.is_natural(None);
        let is_polyline = curve.is_polyline().0;
        let is_singular = curve.is_singular();
        let is_duplicate = curve.is_duplicate(&curve, false, None);
        let is_continuous =
            curve.is_continuous(1, curve.domain_middle(), None, None, None, None, None);

        MINI_CHECK!(!is_rational);
        MINI_CHECK!(!closed);
        MINI_CHECK!(!periodic);
        MINI_CHECK!(!linear);
        MINI_CHECK!(planar);
        MINI_CHECK!(!arc);
        MINI_CHECK!(on_plane);
        MINI_CHECK!(!is_open);
        MINI_CHECK!(is_polyline == 0);
        MINI_CHECK!(!is_singular);
        MINI_CHECK!(is_duplicate);
        MINI_CHECK!(is_continuous);

        let mut copy_curve = curve.duplicate();
        let before_pt = copy_curve.point_at(1.5);
        copy_curve.insert_nurbsknot(1.5, 1);

        MINI_CHECK!(TOLERANCE.is_point_close(&before_pt, &copy_curve.point_at(1.5)));

        let greville0 = curve.greville_abcissa(0);
        let greville = curve.get_greville_abcissae();

        MINI_CHECK!(TOLERANCE.is_close(greville0, 0.0));
        MINI_CHECK!(greville.len() == 4);
        MINI_CHECK!(TOLERANCE.is_close(greville[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(greville[1], 0.879872167739067));
        MINI_CHECK!(TOLERANCE.is_close(greville[2], 2.639616503217201));
        MINI_CHECK!(TOLERANCE.is_close(greville[3], 3.519488670956267));

        let dimension = curve.dimension();
        let degree = curve.degree();
        let order = curve.order();
        let cv_count = curve.cv_count();
        let cv_size = curve.cv_size();
        let nurbsknot_count = curve.nurbsknot_count();
        let span_count = curve.span_count();

        MINI_CHECK!(dimension == 3);
        MINI_CHECK!(degree == 2);
        MINI_CHECK!(order == 3);
        MINI_CHECK!(cv_count == 4);
        MINI_CHECK!(cv_size == 3);
        MINI_CHECK!(nurbsknot_count == 5);
        MINI_CHECK!(span_count == 2);

        let p = curve.cv(1).unwrap();
        let cv_point = curve.get_cv(1).unwrap();
        let cv4 = curve.get_cv_4d(1).unwrap();

        MINI_CHECK!(p[0] == 1.0 && p[1] == 1.0 && p[2] == 0.0);
        MINI_CHECK!(cv_point == Point::new(1.0, 1.0, 0.0));
        MINI_CHECK!(cv4.0 == 1.0 && cv4.1 == 1.0 && cv4.2 == 0.0 && cv4.3 == 1.0);

        curve.set_cv(2, &Point::new(2.0, 0.0, 0.5));

        MINI_CHECK!(curve.get_cv(2).unwrap()[0] == 2.0);
        MINI_CHECK!(curve.get_cv(2).unwrap()[1] == 0.0);
        MINI_CHECK!(curve.get_cv(2).unwrap()[2] == 0.5);

        curve.set_cv_4d(2, 2.0, 0.0, 0.5, 0.707);

        let cv4_weighted = curve.get_cv_4d(2).unwrap();
        let weight = curve.weight(2);

        MINI_CHECK!(
            cv4_weighted.0 == 2.0
                && cv4_weighted.1 == 0.0
                && cv4_weighted.2 == 0.5
                && cv4_weighted.3 == 0.707
        );
        MINI_CHECK!(weight == 0.707);

        curve.set_weight(2, 0.5);

        MINI_CHECK!(curve.weight(2) == 0.5);

        let nurbsknot3 = curve.nurbsknot(3).unwrap();
        let end_nurbsknot = curve.nurbsknot(4).unwrap();
        curve.set_nurbsknot(4, end_nurbsknot);

        MINI_CHECK!(TOLERANCE.is_close(nurbsknot3, 3.519488670956267));
        MINI_CHECK!(TOLERANCE.is_close(curve.nurbsknot(4).unwrap(), end_nurbsknot));

        let m0 = curve.nurbsknot_multiplicity(0);
        let m1 = curve.nurbsknot_multiplicity(1);
        let m2 = curve.nurbsknot_multiplicity(2);
        let m3 = curve.nurbsknot_multiplicity(3);
        let m4 = curve.nurbsknot_multiplicity(4);
        let superfluous_nurbsknot = curve.superfluous_nurbsknot(1);

        MINI_CHECK!(m0 == 2);
        MINI_CHECK!(m1 == 2);
        MINI_CHECK!(m2 == 1);
        MINI_CHECK!(m3 == 2);
        MINI_CHECK!(m4 == 2);
        MINI_CHECK!(TOLERANCE.is_close(superfluous_nurbsknot, 7.038977341912535));

        let nurbsknots = curve.nurbsknot_array();
        let k0 = nurbsknots[0];
        let nurbsknot_vector = curve.get_nurbsknots();
        let cvs = curve.cv_array();
        let cx0 = cvs[0];

        MINI_CHECK!(k0 == 0.0);
        MINI_CHECK!(TOLERANCE.is_close(nurbsknot_vector[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(nurbsknot_vector[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(nurbsknot_vector[2], 1.759744335478134));
        MINI_CHECK!(TOLERANCE.is_close(nurbsknot_vector[3], 3.519488670956267));
        MINI_CHECK!(TOLERANCE.is_close(nurbsknot_vector[4], 3.519488670956267));
        MINI_CHECK!(cx0 == 0.0);

        let interval = curve.domain();
        let start = curve.domain_start();
        let middle = curve.domain_middle();
        let end = curve.domain_end();

        MINI_CHECK!(
            TOLERANCE.is_close(interval.0, 0.0)
                && TOLERANCE.is_close(interval.1, 3.519488670956267)
        );
        MINI_CHECK!(TOLERANCE.is_close(start, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(middle, 1.759744335478134));
        MINI_CHECK!(TOLERANCE.is_close(end, 3.519488670956267));

        curve.set_domain(0.0, 1.0);

        let intervals = curve.get_span_vector();
        let discontinuity =
            curve.get_next_discontinuity(2, curve.domain_start(), curve.domain_end());

        MINI_CHECK!(curve.domain_start() == 0.0);
        MINI_CHECK!(curve.domain_middle() == 0.5);
        MINI_CHECK!(curve.domain_end() == 1.0);
        MINI_CHECK!(
            TOLERANCE.is_close(intervals[0], 0.0)
                && TOLERANCE.is_close(intervals[1], 0.5)
                && TOLERANCE.is_close(intervals[2], 1.0)
        );
        MINI_CHECK!(discontinuity.0 && TOLERANCE.is_close(discontinuity.1, 0.5));
    })
}

pub fn run_nurbscurve_conversions() -> TestResult {
    MINI_TEST!("Conversions", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let adaptive_pts = curve.to_polyline_adaptive(0.1, 0.0, 0.0).0;
        let div_pts = curve.divide_by_count(10, true).0;
        let len_pts = curve.divide_by_length(0.5).0;

        MINI_CHECK!(adaptive_pts.len() == 27);
        MINI_CHECK!(TOLERANCE.is_point_close(&adaptive_pts[0], &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&adaptive_pts[13], &Point::new(2.0, 0.5, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&adaptive_pts[26], &Point::new(4.0, 0.0, 0.0)));
        MINI_CHECK!(div_pts.len() == 10);
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[0],
            &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[1],
            &Point::new(0.328571016773017, 0.598213507757063, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[2],
            &Point::new(0.740744944144815, 1.140321237310326, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[3],
            &Point::new(1.338524001477341, 1.232716038191446, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[4],
            &Point::new(1.712929668000343, 0.664818751028787, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[5],
            &Point::new(2.287070333148604, 0.664818752348101, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[6],
            &Point::new(2.661475999779531, 1.232716039392177, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[7],
            &Point::new(3.259255057037078, 1.140321236176910, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[8],
            &Point::new(3.671428983538974, 0.598213507250245, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &div_pts[9],
            &Point::new(4.000000000000000, 0.000000000000000, 0.000000000000000)
        ));
        MINI_CHECK!(len_pts.len() == 13);
        MINI_CHECK!(TOLERANCE.is_point_close(&len_pts[0], &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &len_pts[6],
            &Point::new(1.928691288503169, 0.510169864670676, 0.0)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &len_pts[12],
            &Point::new(3.934494396222682, 0.128829843907475, 0.0)
        ));
    })
}

pub fn run_nurbscurve_evaluation() -> TestResult {
    MINI_TEST!("Evaluation", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Vector;

        let points = vec![
            Point::new(1.957614, 1.140253, -0.191281),
            Point::new(0.912252, 1.886721, 0.0),
            Point::new(3.089381, 2.701879, -0.696251),
            Point::new(5.015145, 1.189141, 0.35799),
            Point::new(1.854155, 0.514663, 0.347694),
            Point::new(3.309532, 1.328666, 0.0),
            Point::new(3.544072, 2.194233, 0.696217),
            Point::new(2.903513, 2.091287, 0.696217),
            Point::new(2.752484, 1.45432, 0.0),
            Point::new(2.406227, 1.288248, 0.0),
            Point::new(2.15032, 1.868606, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        let length = curve.length(None);
        let point_at = curve.point_at(0.5);
        let derivatives = curve.evaluate(0.5, 2);
        let tangent = curve.tangent_at(0.5);

        MINI_CHECK!(TOLERANCE.is_close(length, 11.3010276326));
        MINI_CHECK!(TOLERANCE.is_close(point_at[0], 1.463452399002842));
        MINI_CHECK!(TOLERANCE.is_close(point_at[1], 1.680997287875395));
        MINI_CHECK!(TOLERANCE.is_close(point_at[2], -0.124474565996108));
        MINI_CHECK!(derivatives.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(derivatives[0][0], 1.463452399002842));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[0][1], 1.680997287875395));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[0][2], -0.124474565996108));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[1][0], -0.311619416021204));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[1][1], 0.974021205471335));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[1][2], -0.037441955449586));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[2][0], 2.706815143892446));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[2][1], -0.429869481117820));
        MINI_CHECK!(TOLERANCE.is_close(derivatives[2][2], -0.684219293829483));
        MINI_CHECK!(TOLERANCE.is_close(tangent[0], -0.304511941745027));
        MINI_CHECK!(TOLERANCE.is_close(tangent[1], 0.951805546117607));
        MINI_CHECK!(TOLERANCE.is_close(tangent[2], -0.036587972264639));

        let f = curve.plane_at(0.5, true);

        MINI_CHECK!(TOLERANCE.is_close(f.origin()[0], 3.156927375000000));
        MINI_CHECK!(TOLERANCE.is_close(f.origin()[1], 1.335111500000000));
        MINI_CHECK!(TOLERANCE.is_close(f.origin()[2], 0.130488875000000));
        MINI_CHECK!(TOLERANCE.is_close(f.x_axis()[0], 0.701806140304030));
        MINI_CHECK!(TOLERANCE.is_close(f.x_axis()[1], 0.697509131556264));
        MINI_CHECK!(TOLERANCE.is_close(f.x_axis()[2], 0.144738221721788));
        MINI_CHECK!(TOLERANCE.is_close(f.y_axis()[0], -0.513930504714161));
        MINI_CHECK!(TOLERANCE.is_close(f.y_axis()[1], 0.355053088776962));
        MINI_CHECK!(TOLERANCE.is_close(f.y_axis()[2], 0.780905077761815));
        MINI_CHECK!(TOLERANCE.is_close(f.z_axis()[0], 0.493298669931115));
        MINI_CHECK!(TOLERANCE.is_close(f.z_axis()[1], -0.622429365908747));
        MINI_CHECK!(TOLERANCE.is_close(f.z_axis()[2], 0.607649657861031));

        MINI_CHECK!(!curve.plane_at(-0.1, true).is_valid());
        MINI_CHECK!(!curve.plane_at(1.1, true).is_valid());
        MINI_CHECK!(curve.plane_at(curve.domain_start(), false).is_valid());
        MINI_CHECK!(curve.plane_at(curve.domain_end(), false).is_valid());
        MINI_CHECK!(!curve.plane_at(curve.domain_start() - 0.1, false).is_valid());

        let pf = curve.perpendicular_plane_at(0.5, true);

        MINI_CHECK!(TOLERANCE.is_point_close(
            &pf.origin(),
            &Point::new(3.156927375000000, 1.335111500000000, 0.130488875000000)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &pf.x_axis(),
            &Vector::new(0.632703652329189, -0.703685357647999, 0.323284713157168)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &pf.y_axis(),
            &Vector::new(0.327344206830723, -0.135306795251661, -0.935167279909370)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &pf.z_axis(),
            &Vector::new(0.701806140314880, 0.697509131546342, 0.144738221716994)
        ));
        MINI_CHECK!(!curve.perpendicular_plane_at(-0.1, true).is_valid());
        MINI_CHECK!(!curve.perpendicular_plane_at(1.1, true).is_valid());
        MINI_CHECK!(curve
            .perpendicular_plane_at(curve.domain_start(), false)
            .is_valid());
        MINI_CHECK!(curve
            .perpendicular_plane_at(curve.domain_end(), false)
            .is_valid());
        MINI_CHECK!(!curve
            .perpendicular_plane_at(curve.domain_start() - 0.1, false)
            .is_valid());

        let frames = curve.get_perpendicular_planes(4);

        MINI_CHECK!(frames.len() == 5);
        MINI_CHECK!(TOLERANCE.is_point_close(
            &frames[0].origin(),
            &Point::new(1.957614, 1.140253, -0.191281)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[0].x_axis(),
            &Vector::new(0.532767753269467, 0.809398954921174, -0.247046256496055)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[0].y_axis(),
            &Vector::new(-0.261213903019039, -0.120386647366337, -0.957744408496052)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[0].z_axis(),
            &Vector::new(-0.804938393882267, 0.574787253606414, 0.147288136473484)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &frames[2].origin(),
            &Point::new(3.676077075808618, 0.909845354074582, 0.350126131660904)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[2].x_axis(),
            &Vector::new(-0.188216728828592, 0.616420980974357, -0.764591156896073)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[2].y_axis(),
            &Vector::new(0.183061410483993, -0.742842969436200, -0.643950963001702)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[2].z_axis(),
            &Vector::new(-0.964916049706230, -0.261169479407185, 0.026972579511507)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &frames[4].origin(),
            &Point::new(2.150320000000000, 1.868606000000000, 0.000000000000000)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[4].x_axis(),
            &Vector::new(0.183261707646767, 0.080808692310795, 0.979737261594868)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[4].y_axis(),
            &Vector::new(0.896455027441244, 0.395289116385372, -0.200287039627106)
        ));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &frames[4].z_axis(),
            &Vector::new(-0.403464410184726, 0.914995338629816, 0.000000000000000)
        ));

        let p0 = curve.point_at_start();
        let p1 = curve.point_at_middle();
        let p2 = curve.point_at_end();

        MINI_CHECK!(TOLERANCE.is_close(p0[0], 1.957614));
        MINI_CHECK!(TOLERANCE.is_close(p0[1], 1.140253));
        MINI_CHECK!(TOLERANCE.is_close(p0[2], -0.191281));
        MINI_CHECK!(TOLERANCE.is_close(p1[0], 3.156927375));
        MINI_CHECK!(TOLERANCE.is_close(p1[1], 1.3351115));
        MINI_CHECK!(TOLERANCE.is_close(p1[2], 0.130488875));
        MINI_CHECK!(TOLERANCE.is_close(p2[0], 2.15032));
        MINI_CHECK!(TOLERANCE.is_close(p2[1], 1.868606));
        MINI_CHECK!(TOLERANCE.is_close(p2[2], 0.0));

        curve.set_start_point(&Point::new(1.957614, 1.140253, 2.0));
        curve.set_end_point(&Point::new(2.15032, 1.868606, 2.0));

        MINI_CHECK!(TOLERANCE.is_close(curve.point_at_start()[2], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(curve.point_at_end()[2], 2.0));
    })
}

pub fn run_nurbscurve_modifications() -> TestResult {
    MINI_TEST!("Modifications", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        let mut curve_reversed = curve.duplicate();
        curve_reversed.reverse();

        MINI_CHECK!(
            TOLERANCE.is_point_close(&curve_reversed.point_at_start(), &curve.point_at_end())
        );

        curve.swap_coordinates(0, 1);

        MINI_CHECK!(TOLERANCE.is_point_close(&curve.get_cv(0).unwrap(), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&curve.get_cv(1).unwrap(), &Point::new(2.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&curve.get_cv(2).unwrap(), &Point::new(0.0, 2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&curve.get_cv(3).unwrap(), &Point::new(2.0, 3.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&curve.get_cv(4).unwrap(), &Point::new(0.0, 4.0, 0.0)));

        let mut ct = curve.duplicate();
        let a = ct.domain_start() + (ct.domain_end() - ct.domain_start()) / 3.0;
        let b = ct.domain_start() + 2.0 * (ct.domain_end() - ct.domain_start()) / 3.0;
        ct.trim(a, b);

        MINI_CHECK!(ct.length(None) < curve.length(None));

        let split_t = curve.domain_middle();
        let halves = curve.split(split_t);

        MINI_CHECK!(TOLERANCE.is_point_close(&curve.point_at(split_t), &halves.0.point_at_end()));
        MINI_CHECK!(TOLERANCE.is_point_close(&curve.point_at(split_t), &halves.1.point_at_start()));

        let mut curve_extended = curve.duplicate();
        curve_extended.extend(curve.domain_start() - 0.5, curve.domain_end() + 0.5);

        MINI_CHECK!(curve_extended.length(None) > curve.length(None));

        let mut curve_rational = curve.duplicate();
        let original_length = curve.length(None);
        curve_rational.make_rational();
        curve_rational.set_weight(2, 10.0);

        MINI_CHECK!(curve_rational.length(None) != original_length);

        curve_rational.make_non_rational(true);

        MINI_CHECK!(curve_rational.length(None) == original_length);

        let points_open = points.clone();
        let mut curve_open = NurbsCurve::new(3, false, 3, 5);

        for i in 0..5 {
            curve_open.set_cv(i, &points_open[i]);
        }

        for i in 0..curve_open.nurbsknot_count() {
            curve_open.set_nurbsknot(i, i as f64 * 1.0);
        }

        curve_open.clamp_end(2);

        let nurbsknots = curve_open.get_nurbsknots();

        MINI_CHECK!(TOLERANCE.is_close(nurbsknots[0], nurbsknots[1]));
        MINI_CHECK!(TOLERANCE.is_close(
            nurbsknots[nurbsknots.len() - 2],
            nurbsknots[nurbsknots.len() - 1]
        ));

        let mut raised = curve.duplicate();
        raised.increase_degree(3);

        MINI_CHECK!(curve.degree() != raised.degree());
        MINI_CHECK!(TOLERANCE.is_point_close(&curve.point_at_middle(), &raised.point_at_middle()));

        let closed_pts = vec![
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(-1.0, 0.0, 0.0),
            Point::new(0.0, -1.0, 0.0),
        ];

        let mut c = NurbsCurve::create(true, 2, &closed_pts);
        let expected_start = c.point_at(c.domain_middle());
        c.change_closed_curve_seam(c.domain_middle());

        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at_start(), &expected_start));
    })
}

pub fn run_nurbscurve_transformations() -> TestResult {
    MINI_TEST!("Transformations", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Xform;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];

        let mut curve1 = NurbsCurve::create(false, 2, &points);
        let curve1_xf = Xform::translation(0.0, 0.0, 1.0);
        curve1.transform(&curve1_xf);

        let mut curve2 = NurbsCurve::create(false, 2, &points);
        let x = Xform::translation(0.0, 0.0, 1.0);
        curve2.transform(&x);

        let curve3 = NurbsCurve::create(false, 2, &points);
        let curve3_xf = Xform::translation(0.0, 0.0, 10.0);
        let curve3_transformed = curve3.transformed(&curve3_xf);

        let curve4 = NurbsCurve::create(false, 2, &points);
        let x = Xform::translation(0.0, 0.0, 10.0);
        let curve4_transformed = curve4.transformed(&x);

        MINI_CHECK!(curve1.cv(0).unwrap()[2] == 1.0);
        MINI_CHECK!(curve2.cv(0).unwrap()[2] == 1.0);
        MINI_CHECK!(curve3_transformed.cv(0).unwrap()[2] == 10.0);
        MINI_CHECK!(curve4_transformed.cv(0).unwrap()[2] == 10.0);
    })
}

pub fn run_nurbscurve_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::NurbsCurve;
        use crate::Point;
        use std::path::PathBuf;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let guid = curve.guid().to_string();
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_nurbscurve.json");
        curve.file_json_dump(filename.to_str().unwrap()).unwrap();

        let json = curve.jsondump().unwrap();
        let loaded_json = NurbsCurve::jsonload(&json).unwrap();
        let loaded_json_string = NurbsCurve::file_json_loads(&curve.file_json_dumps());
        let loaded_from_file = NurbsCurve::file_json_load(filename.to_str().unwrap()).unwrap();

        MINI_CHECK!(loaded_json == curve);
        MINI_CHECK!(loaded_json_string == curve);
        MINI_CHECK!(loaded_from_file == curve);
        MINI_CHECK!(loaded_from_file.guid() == guid);
    })
}

pub fn run_nurbscurve_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::NurbsCurve;
        use crate::Point;
        use std::path::PathBuf;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let guid = curve.guid().to_string();
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_nurbscurve.bin");
        curve.pb_dump(filename.to_str().unwrap()).unwrap();

        let loaded_proto_string = NurbsCurve::pb_loads(&curve.pb_dumps()).unwrap();
        let loaded = NurbsCurve::pb_load(filename.to_str().unwrap()).unwrap();
        let converted = NurbsCurve::from_proto(curve.to_proto());

        MINI_CHECK!(loaded_proto_string == curve);
        MINI_CHECK!(loaded == curve);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(converted == curve);
        MINI_CHECK!(converted.guid() == guid);
    })
}

pub fn run_nurbscurve_curvature() -> TestResult {
    MINI_TEST!("Curvature", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Primitives;

        let r = 2.0;
        let circle = Primitives::circle(0.0, 0.0, 0.0, r);
        let t0 = circle.domain_start();
        let t1 = circle.domain_end();

        for i in 0..=8 {
            let t = t0 + (t1 - t0) * (i as f64) / 8.0;

            MINI_CHECK!((circle.curvature_at(t) - 1.0 / r).abs() < 1e-6);
        }

        let line_pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ];

        let line = NurbsCurve::create(false, 1, &line_pts);

        MINI_CHECK!(line.curvature_at(line.domain_middle()) < 1e-9);
    })
}

pub fn run_nurbscurve_closest_point() -> TestResult {
    MINI_TEST!("Closest Point", {
        use crate::nurbsknot::CurveInterpStyle;
        use crate::nurbsknot::CurveNurbsKnotStyle;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Primitives;

        let circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let cp = circle.closest_point(&Point::new(5.0, 0.0, 0.0));
        let cp2 = circle.closest_point(&Point::new(0.0, 5.0, 0.0));

        MINI_CHECK!((cp[0] - 2.0).abs() < 1e-5 && cp[1].abs() < 1e-5 && cp[2].abs() < 1e-5);
        MINI_CHECK!(cp2[0].abs() < 1e-5 && (cp2[1] - 2.0).abs() < 1e-5);

        let ipts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 2.0),
            Point::new(6.0, 0.0, -3.0),
            Point::new(8.0, 0.0, 0.0),
        ];

        let ic = NurbsCurve::create_interpolated(
            &ipts,
            CurveNurbsKnotStyle::Chord,
            CurveInterpStyle::Occt,
        );
        let pc = ic.closest_point(&Point::new(2.0, -1.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_point_close(&pc, &Point::new(0.5808155659, 0.0, 0.9672315271)));

        let p0 = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 6.0, 0.0),
            Point::new(6.0, -3.0, 3.0),
            Point::new(10.0, 0.0, 0.0),
        ];

        let p1 = vec![
            Point::new(6.0, -3.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(6.0, 6.0, 3.0),
            Point::new(3.0, 12.0, 0.0),
        ];

        let c0 = NurbsCurve::create_from_parameters(
            &p0,
            &[1.0, 1.0, 1.0, 1.0],
            &[0.0, 1.0],
            &[4, 4],
            3,
            false,
        );
        let c1 = NurbsCurve::create_from_parameters(
            &p1,
            &[1.0, 1.0, 1.0, 1.0],
            &[0.0, 1.0],
            &[4, 4],
            3,
            false,
        );
        let params = c0.closest_parameters_curve(&c1);
        let closest = c0.closest_points_curve(&c1);

        MINI_CHECK!(
            (params.0 - 0.4757682937).abs() < 1e-6 && (params.1 - 0.3366914716).abs() < 1e-6
        );
        MINI_CHECK!(TOLERANCE.is_point_close(
            &closest.0,
            &Point::new(4.389607399, 1.285537564, 1.067964425)
        ));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &closest.1,
            &Point::new(4.552264625, 1.380381100, 0.676740741)
        ));
    })
}

REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Constructor",
    crate::nurbscurve_test::run_nurbscurve_constructor
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Create Interpolated",
    crate::nurbscurve_test::run_nurbscurve_create_interpolated
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Create From Parameters",
    crate::nurbscurve_test::run_nurbscurve_create_from_parameters
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Create Fitted",
    crate::nurbscurve_test::run_nurbscurve_create_fitted
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Join",
    crate::nurbscurve_test::run_nurbscurve_join
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Attributes",
    crate::nurbscurve_test::run_nurbscurve_attributes
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Conversions",
    crate::nurbscurve_test::run_nurbscurve_conversions
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Evaluation",
    crate::nurbscurve_test::run_nurbscurve_evaluation
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Modifications",
    crate::nurbscurve_test::run_nurbscurve_modifications
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Transformations",
    crate::nurbscurve_test::run_nurbscurve_transformations
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Json Roundtrip",
    crate::nurbscurve_test::run_nurbscurve_json_roundtrip
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Protobuf Roundtrip",
    crate::nurbscurve_test::run_nurbscurve_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Curvature",
    crate::nurbscurve_test::run_nurbscurve_curvature
);
REGISTER_MINI_TEST!(
    "NurbsCurve",
    "Closest Point",
    crate::nurbscurve_test::run_nurbscurve_closest_point
);
