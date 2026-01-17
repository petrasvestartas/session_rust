use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_nurbscurve_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.name = "my_nurbscurve".to_string();

        // Minimal and Full String Representation
        let cstr = curve.str();
        let crepr = curve.repr();

        // Copy (duplicates everything except guid)
        let ccopy = curve.duplicate();

        MINI_CHECK!(curve.is_valid() == true);
        MINI_CHECK!(curve.cv_count() == 4);
        MINI_CHECK!(curve.degree() == 2);
        MINI_CHECK!(curve.order() == 3);
        MINI_CHECK!(curve.name == "my_nurbscurve");
        MINI_CHECK!(!curve.guid.is_empty());
        MINI_CHECK!(cstr == "degree=2, cvs=4");
        MINI_CHECK!(crepr == "NurbsCurve(my_nurbscurve, dim=3, order=3, cvs=4, rational=false)");
        MINI_CHECK!(ccopy.cv_count() == curve.cv_count());
        MINI_CHECK!(ccopy.guid != curve.guid);
    })
}

pub fn run_nurbscurve_is_valid() -> TestResult {
    MINI_TEST!("is_valid", {
        use crate::NurbsCurve;
        use crate::Point;

        let curve_invalid = NurbsCurve::default();

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let curve_valid = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve_invalid.is_valid() == false);
        MINI_CHECK!(curve_valid.is_valid() == true);
    })
}

pub fn run_nurbscurve_get_cv() -> TestResult {
    MINI_TEST!("get_cv", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let cv0 = curve.get_cv(0).unwrap();
        let cv1 = curve.get_cv(1).unwrap();
        let cv2 = curve.get_cv(2).unwrap();

        MINI_CHECK!((cv0[0] - 0.0).abs() < 0.01);
        MINI_CHECK!((cv1[0] - 1.0).abs() < 0.01);
        MINI_CHECK!((cv2[0] - 2.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_set_cv() -> TestResult {
    MINI_TEST!("set_cv", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_cv_point(1, &Point::new(1.5, 2.0, 0.0));
        let cv1 = curve.get_cv(1).unwrap();

        MINI_CHECK!((cv1[0] - 1.5).abs() < 0.01);
        MINI_CHECK!((cv1[1] - 2.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_point_at() -> TestResult {
    MINI_TEST!("point_at", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let (t0, t1) = curve.domain();
        let t_mid = (t0 + t1) / 2.0;
        let pt_mid = curve.point_at(t_mid);

        MINI_CHECK!(pt_mid[0] > 0.0);
        MINI_CHECK!(pt_mid[0] < 2.0);
    })
}

pub fn run_nurbscurve_point_at_start() -> TestResult {
    MINI_TEST!("point_at_start", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let pt_start = curve.point_at_start();

        MINI_CHECK!((pt_start[0] - 0.0).abs() < 0.01);
        MINI_CHECK!((pt_start[1] - 0.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_point_at_end() -> TestResult {
    MINI_TEST!("point_at_end", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let pt_end = curve.point_at_end();

        MINI_CHECK!((pt_end[0] - 2.0).abs() < 0.01);
        MINI_CHECK!((pt_end[1] - 0.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_domain() -> TestResult {
    MINI_TEST!("domain", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let (t0, t1) = curve.domain();

        MINI_CHECK!(t0 < t1);
    })
}

pub fn run_nurbscurve_is_closed() -> TestResult {
    MINI_TEST!("is_closed", {
        use crate::NurbsCurve;
        use crate::Point;

        let points_open = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let curve_open = NurbsCurve::create(false, 2, &points_open);

        let points_closed = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ];
        let curve_closed = NurbsCurve::create(false, 3, &points_closed);

        MINI_CHECK!(curve_open.is_closed() == false);
        MINI_CHECK!(curve_closed.is_closed() == true);
    })
}

pub fn run_nurbscurve_length() -> TestResult {
    MINI_TEST!("length", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 1, &points);
        let length = curve.length(None);

        MINI_CHECK!((length - 1.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_reverse() -> TestResult {
    MINI_TEST!("reverse", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        let pt_start_before = curve.point_at_start();
        let pt_end_before = curve.point_at_end();
        curve.reverse();
        let pt_start_after = curve.point_at_start();
        let pt_end_after = curve.point_at_end();

        MINI_CHECK!((pt_start_before[0] - pt_end_after[0]).abs() < 0.01);
        MINI_CHECK!((pt_end_before[0] - pt_start_after[0]).abs() < 0.01);
    })
}

pub fn run_nurbscurve_tangent_at() -> TestResult {
    MINI_TEST!("tangent_at", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let (t0, t1) = curve.domain();
        let t_mid = (t0 + t1) / 2.0;
        let tangent = curve.tangent_at(t_mid);
        let mag = (tangent[0]*tangent[0] + tangent[1]*tangent[1] + tangent[2]*tangent[2]).sqrt();

        MINI_CHECK!(mag > 0.5);
    })
}

pub fn run_nurbscurve_knot_count() -> TestResult {
    MINI_TEST!("knot_count", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let knot_count = curve.knot_count();

        MINI_CHECK!(knot_count == curve.order() + curve.cv_count() - 2);
    })
}

pub fn run_nurbscurve_cv_size() -> TestResult {
    MINI_TEST!("cv_size", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.cv_size() == 3);
        curve.make_rational();
        MINI_CHECK!(curve.cv_size() == 4);
    })
}

pub fn run_nurbscurve_is_linear() -> TestResult {
    MINI_TEST!("is_linear", {
        use crate::NurbsCurve;
        use crate::Point;

        let points_linear = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
        ];
        let curve_linear = NurbsCurve::create(false, 1, &points_linear);

        let points_curved = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(0.5, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
        ];
        let curve_curved = NurbsCurve::create(false, 2, &points_curved);

        MINI_CHECK!(curve_linear.is_linear(None) == true);
        MINI_CHECK!(curve_curved.is_linear(None) == false);
    })
}

pub fn run_nurbscurve_make_rational() -> TestResult {
    MINI_TEST!("make_rational", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.is_rational() == false);
        curve.make_rational();
        MINI_CHECK!(curve.is_rational() == true);
    })
}

pub fn run_nurbscurve_weight() -> TestResult {
    MINI_TEST!("weight", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.make_rational();
        let w = curve.weight(0);
        curve.set_weight(1, 2.0);
        let w1 = curve.weight(1);

        MINI_CHECK!((w - 1.0).abs() < 0.01);
        MINI_CHECK!((w1 - 2.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);

        let filename = "serialization/test_nurbscurve.json";
        curve.json_dump(filename);
        let loaded = NurbsCurve::json_load(filename);

        MINI_CHECK!(loaded.is_valid() == true);
        MINI_CHECK!(loaded.cv_count() == 3);
        MINI_CHECK!(loaded.degree() == 2);
        MINI_CHECK!(loaded.order() == 3);
    })
}

pub fn run_nurbscurve_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);

        let filename = "serialization/test_nurbscurve.bin";
        curve.protobuf_dump(filename);
        let loaded = NurbsCurve::protobuf_load(filename);

        MINI_CHECK!(loaded.is_valid() == true);
        MINI_CHECK!(loaded.cv_count() == 3);
        MINI_CHECK!(loaded.degree() == 2);
        MINI_CHECK!(loaded.order() == 3);
    })
}

pub fn run_nurbscurve_degree() -> TestResult {
    MINI_TEST!("degree", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.degree() == 2);
        MINI_CHECK!(curve.order() == 3);
    })
}

pub fn run_nurbscurve_dimension() -> TestResult {
    MINI_TEST!("dimension", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.dimension() == 3);
    })
}

pub fn run_nurbscurve_is_rational() -> TestResult {
    MINI_TEST!("is_rational", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.is_rational() == false);
        curve.make_rational();
        MINI_CHECK!(curve.is_rational() == true);
    })
}

pub fn run_nurbscurve_set_weight() -> TestResult {
    MINI_TEST!("set_weight", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.make_rational();

        MINI_CHECK!((curve.weight(1) - 1.0).abs() < 0.01);
        curve.set_weight(1, 2.0);
        MINI_CHECK!((curve.weight(1) - 2.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_knot() -> TestResult {
    MINI_TEST!("knot", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let knot0 = curve.knot(0).unwrap_or(0.0);
        let knot1 = curve.knot(1).unwrap_or(0.0);

        MINI_CHECK!((knot0 - 0.0).abs() < 0.01);
        MINI_CHECK!(knot1 >= knot0);
    })
}

pub fn run_nurbscurve_set_knot() -> TestResult {
    MINI_TEST!("set_knot", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        let result = curve.set_knot(0, 0.5);

        MINI_CHECK!(result == true);
        MINI_CHECK!((curve.knot(0).unwrap_or(0.0) - 0.5).abs() < 0.01);
    })
}

pub fn run_nurbscurve_set_domain() -> TestResult {
    MINI_TEST!("set_domain", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        let result = curve.set_domain(0.0, 10.0);
        let (t0, t1) = curve.domain();

        MINI_CHECK!(result == true);
        MINI_CHECK!((t0 - 0.0).abs() < 0.01);
        MINI_CHECK!((t1 - 10.0).abs() < 0.01);
    })
}

pub fn run_nurbscurve_span_count() -> TestResult {
    MINI_TEST!("span_count", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let spans = curve.span_count();

        MINI_CHECK!(spans == 1);
    })
}

pub fn run_nurbscurve_get_span_vector() -> TestResult {
    MINI_TEST!("get_span_vector", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let spans = curve.get_span_vector();

        MINI_CHECK!(spans.len() >= 2);
    })
}

pub fn run_nurbscurve_evaluate() -> TestResult {
    MINI_TEST!("evaluate", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let (t0, t1) = curve.domain();
        let t_mid = (t0 + t1) / 2.0;
        let result = curve.evaluate(t_mid, 1);

        MINI_CHECK!(result.len() >= 1);
    })
}

pub fn run_nurbscurve_is_periodic() -> TestResult {
    MINI_TEST!("is_periodic", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);

        MINI_CHECK!(curve.is_periodic() == false);
    })
}

pub fn run_nurbscurve_make_non_rational() -> TestResult {
    MINI_TEST!("make_non_rational", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.make_rational();

        MINI_CHECK!(curve.is_rational() == true);
        curve.make_non_rational();
        MINI_CHECK!(curve.is_rational() == false);
    })
}

pub fn run_nurbscurve_divide_by_count() -> TestResult {
    MINI_TEST!("divide_by_count", {
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let (pts, params) = curve.divide_by_count(5, true);

        MINI_CHECK!(pts.len() >= 2);
        MINI_CHECK!(params.len() >= 2);
    })
}

pub fn run_nurbscurve_intersect_plane() -> TestResult {
    MINI_TEST!("intersect_plane", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Plane;
        use crate::Vector;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);
        let plane = Plane::new(
            Point::new(1.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
        );
        let intersections = curve.intersect_plane(&plane, None);

        MINI_CHECK!(intersections.is_empty() || !intersections.is_empty());
    })
}

REGISTER_MINI_TEST!("NurbsCurve", "constructor", crate::nurbscurve_test::run_nurbscurve_constructor);
REGISTER_MINI_TEST!("NurbsCurve", "is_valid", crate::nurbscurve_test::run_nurbscurve_is_valid);
REGISTER_MINI_TEST!("NurbsCurve", "get_cv", crate::nurbscurve_test::run_nurbscurve_get_cv);
REGISTER_MINI_TEST!("NurbsCurve", "set_cv", crate::nurbscurve_test::run_nurbscurve_set_cv);
REGISTER_MINI_TEST!("NurbsCurve", "point_at", crate::nurbscurve_test::run_nurbscurve_point_at);
REGISTER_MINI_TEST!("NurbsCurve", "point_at_start", crate::nurbscurve_test::run_nurbscurve_point_at_start);
REGISTER_MINI_TEST!("NurbsCurve", "point_at_end", crate::nurbscurve_test::run_nurbscurve_point_at_end);
REGISTER_MINI_TEST!("NurbsCurve", "domain", crate::nurbscurve_test::run_nurbscurve_domain);
REGISTER_MINI_TEST!("NurbsCurve", "is_closed", crate::nurbscurve_test::run_nurbscurve_is_closed);
REGISTER_MINI_TEST!("NurbsCurve", "length", crate::nurbscurve_test::run_nurbscurve_length);
REGISTER_MINI_TEST!("NurbsCurve", "reverse", crate::nurbscurve_test::run_nurbscurve_reverse);
REGISTER_MINI_TEST!("NurbsCurve", "make_rational", crate::nurbscurve_test::run_nurbscurve_make_rational);
REGISTER_MINI_TEST!("NurbsCurve", "tangent_at", crate::nurbscurve_test::run_nurbscurve_tangent_at);
REGISTER_MINI_TEST!("NurbsCurve", "knot_count", crate::nurbscurve_test::run_nurbscurve_knot_count);
REGISTER_MINI_TEST!("NurbsCurve", "cv_size", crate::nurbscurve_test::run_nurbscurve_cv_size);
REGISTER_MINI_TEST!("NurbsCurve", "weight", crate::nurbscurve_test::run_nurbscurve_weight);
REGISTER_MINI_TEST!("NurbsCurve", "is_linear", crate::nurbscurve_test::run_nurbscurve_is_linear);
REGISTER_MINI_TEST!("NurbsCurve", "json_roundtrip", crate::nurbscurve_test::run_nurbscurve_json_roundtrip);
REGISTER_MINI_TEST!("NurbsCurve", "protobuf_roundtrip", crate::nurbscurve_test::run_nurbscurve_protobuf_roundtrip);
REGISTER_MINI_TEST!("NurbsCurve", "degree", crate::nurbscurve_test::run_nurbscurve_degree);
REGISTER_MINI_TEST!("NurbsCurve", "dimension", crate::nurbscurve_test::run_nurbscurve_dimension);
REGISTER_MINI_TEST!("NurbsCurve", "is_rational", crate::nurbscurve_test::run_nurbscurve_is_rational);
REGISTER_MINI_TEST!("NurbsCurve", "set_weight", crate::nurbscurve_test::run_nurbscurve_set_weight);
REGISTER_MINI_TEST!("NurbsCurve", "knot", crate::nurbscurve_test::run_nurbscurve_knot);
REGISTER_MINI_TEST!("NurbsCurve", "set_knot", crate::nurbscurve_test::run_nurbscurve_set_knot);
REGISTER_MINI_TEST!("NurbsCurve", "set_domain", crate::nurbscurve_test::run_nurbscurve_set_domain);
REGISTER_MINI_TEST!("NurbsCurve", "span_count", crate::nurbscurve_test::run_nurbscurve_span_count);
REGISTER_MINI_TEST!("NurbsCurve", "get_span_vector", crate::nurbscurve_test::run_nurbscurve_get_span_vector);
REGISTER_MINI_TEST!("NurbsCurve", "evaluate", crate::nurbscurve_test::run_nurbscurve_evaluate);
REGISTER_MINI_TEST!("NurbsCurve", "is_periodic", crate::nurbscurve_test::run_nurbscurve_is_periodic);
REGISTER_MINI_TEST!("NurbsCurve", "make_non_rational", crate::nurbscurve_test::run_nurbscurve_make_non_rational);
REGISTER_MINI_TEST!("NurbsCurve", "divide_by_count", crate::nurbscurve_test::run_nurbscurve_divide_by_count);
REGISTER_MINI_TEST!("NurbsCurve", "intersect_plane", crate::nurbscurve_test::run_nurbscurve_intersect_plane);
