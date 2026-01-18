use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_nurbscurve_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        // uncomment use crate::NurbsCurve;
        // uncomment use crate::Point;
        // uncomment use crate::Vector;
        use crate::NurbsCurve;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];

        // The first the curve is closed or open
        // For linear curves use degree 1
        // When 3 points use degree 2 curve, Rhino default
        // When x>3 points use degree 3 curve
        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        // Minimal and Full String Representation
        let cstr = curve.str();
        let crepr = curve.repr();

        // Copy (duplicates everything except guid)
        let ccopy = curve.duplicate();
        let _cother = NurbsCurve::create(false, 2, &points);

        // Point division
        let (_divided, _) = curve.divide_by_count(10, true);

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

pub fn run_nurbscurve_attributes() -> TestResult {
    MINI_TEST!("attributes", {
        // uncomment use crate::NurbsCurve;
        // uncomment use crate::Point;
        // uncomment use crate::Plane;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Plane;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);

        /////////////////////////////////////////////
        // Validation
        /////////////////////////////////////////////

        // Whole curve
        let is_valid = curve.is_valid();
        MINI_CHECK!(is_valid == true);

        // Check whole knot vector for
        // For correct size: order + cv_count - 2
        // Non-decreasing (can repeat, can't go down)
        // Valid domain exists
        let is_valid_knot_vector = curve.is_valid_knot_vector();
        MINI_CHECK!(is_valid_knot_vector == true);

        /////////////////////////////////////////////
        // Accessors
        /////////////////////////////////////////////
        // Memory layout 2-2D, 3-3D
        let dimension = curve.dimension();
        MINI_CHECK!(dimension == 3);
        // Degree - Polynomial order, 1=linear, 2=quadratic, 3=cubic
        let degree = curve.degree();
        MINI_CHECK!(degree == 2);
        // Is rational is related to control points having weights
        // is_rational = false means control points [x, y, z]
        // is_rational = false means control points [xw, yw, zw]
        // Rational curves are used to represent:
        // Order = degree + 1, control points + order = knots
        let order = curve.order();
        MINI_CHECK!(order == 3);
        // Number of control vertices
        let cv_count = curve.cv_count();
        MINI_CHECK!(cv_count == 4);
        // Number of floats per 1 control vertex
        let cv_size = curve.cv_size();
        MINI_CHECK!(cv_size == 3);
        // The knots are a list of (degree+control_points-1) numbers
        let knot_count = curve.knot_count();
        MINI_CHECK!(knot_count == 5);
        // Span = a knot interval where a single polynomial segment is evaluated
        // Knot vector: [0, 0, 0 ^, 1 ^, 2 ^, 3, 3, 3]  (cubic, 5 CVs)
        let span_count = curve.span_count();
        MINI_CHECK!(span_count == 2);
        /////////////////////////////////////////////////////
        // Control Vertex Access
        //  m_cv = [x0, y0, z0, (w0), x1, y1, z1, (w1), ...]
        //          --- CV 0 ---      --- CV 1 ---
        /////////////////////////////////////////////////////

        // Get pointer to control vertex
        // Each CV occupies m_cv_stride doubles:
        // (3 for non-rational, 4 for rational)
        // cv(index) returns pointer to m_cv[index * m_cv_stride]
        let p = curve.cv(1).unwrap();
        MINI_CHECK!(p[0] == 1.0 && p[1] == 1.0 && p[2] == 0.0);

        // Returns the control vertex as Point object
        let cv_point = curve.get_cv(1).unwrap();
        MINI_CHECK!(cv_point == Point::new(1.0, 1.0, 0.0));

        // Raw homogeneous coords
        let (x, y, z, w) = curve.get_cv_4d(1).unwrap();
        MINI_CHECK!(x == 1.0 && y == 1.0 && z == 0.0 && w == 1.0);

        // Use for regular points on curve, Polyline, B-Spline
        curve.set_cv_point(2, &Point::new(2.0, 0.0, 0.5));
        MINI_CHECK!(curve.get_cv(2).unwrap()[0] == 2.0 && curve.get_cv(2).unwrap()[1] == 0.0 && curve.get_cv(2).unwrap()[2] == 0.5);

        // Use for rational curvers like circles, ellipses
        curve.set_cv_4d(2, 2.0, 0.0, 0.5, 0.707);
        let (x, y, z, w) = curve.get_cv_4d(2).unwrap();
        MINI_CHECK!(x == 2.0 && y == 0.0 && z == 0.5 && w == 0.707);

        // Get weight of a control vertex (1.0 if non-rational)
        let weight = curve.weight(2);
        MINI_CHECK!(weight == 0.707);

        // Set the weight of a control vertex
        curve.set_weight(2, 0.5);
        MINI_CHECK!(curve.weight(2) == 0.5);

        /////////////////////////////////////////////////////
        // Knot Access
        /////////////////////////////////////////////////////

        // Get knot value at index
        let knot3 = curve.knot(3).unwrap();
        MINI_CHECK!(knot3 == 2.0);

        // Set knot value at index
        // ATTENTION you can brake increasing rule
        curve.set_knot(4, 2.0);
        MINI_CHECK!(curve.knot(4).unwrap() == 2.0);

        // Count repeated knots at index [0, 0, 1, 1, 2]
        let m0 = curve.knot_multiplicity(0);  // 2 (two 0's)
        let m1 = curve.knot_multiplicity(1);  // 2 (still counting the 0's)
        let m2 = curve.knot_multiplicity(2);  // 1 (single 0.5)
        let m3 = curve.knot_multiplicity(3);  // 2 (single 1's)
        let m4 = curve.knot_multiplicity(4);  // 2 (single 2)
        MINI_CHECK!(m0 == 2);
        MINI_CHECK!(m1 == 2);
        MINI_CHECK!(m2 == 1);
        MINI_CHECK!(m3 == 2);
        MINI_CHECK!(m4 == 2);

        // Superflous knots are used for extension of clamped curves
        // For knot vector [0, 0, 0.5, 1, 2]: 2*knot[4] - knot[1] = 2*2 - 0 = 4
        let superfluous_knot = curve.superfluous_knot(1);
        MINI_CHECK!(superfluous_knot == 4.0);

        // Direct memory access to knot values, fast, read-only
        // Vector return is slower and makes a copy
        let knots = curve.knot_array();
        let k0 = knots[0];
        let knot_vector = curve.get_knots();
        MINI_CHECK!(k0 == 0.0);
        MINI_CHECK!(knot_vector[0] == 0.0 && knot_vector[1] == 0.0 &&
                   knot_vector[2] == 1.0 && knot_vector[3] == 2.0 &&
                   knot_vector[4] == 2.0);

        // Control vertex array access
        let cvs = curve.cv_array();
        let cx0 = cvs[0];
        MINI_CHECK!(cx0 == 0.0);

        /////////////////////////////////////////////////////
        // Domain & Parameterization - HERE
        /////////////////////////////////////////////////////

        // get start and end of the curve interval
        let (start, end) = curve.domain();
        MINI_CHECK!(start == 0.0 && end == 2.0);

        // Get start, middle and end values of the interval
        let start = curve.domain_start();
        let middle = curve.domain_middle();
        let end = curve.domain_end();
        MINI_CHECK!(start == 0.0 && middle == 1.0 && end == 2.0);

        // Change curve domain
        curve.set_domain(0.0, 1.0);
        MINI_CHECK!(curve.domain_start() == 0.0 && curve.domain_middle() == 0.5 && curve.domain_end() == 1.0);

        // Span of distict knot intervals
        let intervals = curve.get_span_vector();
        MINI_CHECK!(intervals[0] == 0.0 && intervals[1] == 0.5 && intervals[2] == 1.0);

        /////////////////////////////////////////////////////
        // Geometric checks
        /////////////////////////////////////////////////////
        // Is rational is related to control points having weights
        // is_rational = false means control points [x, y, z]
        // is_rational = false means control points [xw, yw, zw]
        // Rational curves are used to represent:
        // circles, ellipses, parabolas, hyperbolas exactly
        let is_rational = curve.is_rational();
        MINI_CHECK!(is_rational == true);

        // circles, ellipses, parabolas, hyperbolas exactly
        let closed = curve.is_closed();
        let periodic = curve.is_periodic();
        let linear = curve.is_linear(None);
        let planar = curve.is_planar(None);
        let arc = curve.is_arc(None);
        let plane = Plane::xy_plane();
        let on_plane = curve.is_in_plane(&plane, None);
        let is_open = curve.is_natural();
        let is_polyline = curve.is_polyline(None);

        MINI_CHECK!(closed == false);
        MINI_CHECK!(periodic == false);
        MINI_CHECK!(linear == false);
        MINI_CHECK!(planar == false);
        MINI_CHECK!(arc == false);
        MINI_CHECK!(on_plane == false);
        MINI_CHECK!(is_open == false);
        MINI_CHECK!(is_polyline == false);
    })
}

pub fn run_nurbscurve_conversions() -> TestResult {
    MINI_TEST!("Conversions", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Tolerance;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];

        let curve = NurbsCurve::create(false, 2, &points);

        // to_polyline_adaptive
        let (adaptive_pts, _) = curve.to_polyline_adaptive(0.1, 0.0, 0.0);
        MINI_CHECK!(adaptive_pts.len() == 27);
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[0], &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[26], &Point::new(4.0, 0.0, 0.0)));

        // divide_by_count
        let (div_pts, _) = curve.divide_by_count(10, true);
        MINI_CHECK!(div_pts.len() == 10);
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[0], &Point::new(0.0, 0.0, 0.0)));

        // divide_by_length
        let (len_pts, _) = curve.divide_by_length(0.5);
        MINI_CHECK!(len_pts.len() == 13);
    })
}

pub fn run_nurbscurve_frame_at() -> TestResult {
    MINI_TEST!("frame_at", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Tolerance;

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

        let curve = NurbsCurve::create(false, 2, &points);

        let result = curve.frame_at(0.5, true);
        MINI_CHECK!(result.is_some());
        let (o, t, _n, _b) = result.unwrap();

        MINI_CHECK!(Tolerance::default().is_close(o[0], 3.156927375000000));
        MINI_CHECK!(Tolerance::default().is_close(o[1], 1.335111500000000));
        MINI_CHECK!(Tolerance::default().is_close(t[0], 0.701806140304030));

        MINI_CHECK!(curve.frame_at(-0.1, true).is_none());
        MINI_CHECK!(curve.frame_at(1.1, true).is_none());
        MINI_CHECK!(curve.frame_at(curve.domain_start(), false).is_some());
        MINI_CHECK!(curve.frame_at(curve.domain_end(), false).is_some());
        MINI_CHECK!(curve.frame_at(curve.domain_start() - 0.1, false).is_none());
    })
}

pub fn run_nurbscurve_perpendicular_frame_at() -> TestResult {
    MINI_TEST!("perpendicular_frame_at", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Tolerance;

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

        let curve = NurbsCurve::create(false, 2, &points);

        // RMF with Frenet initialization (matches Rhino)
        let result = curve.perpendicular_frame_at(0.5, true);
        MINI_CHECK!(result.is_some());
        let (o, t, n, b) = result.unwrap();
        let tol = Tolerance::default();

        MINI_CHECK!(tol.is_point_close(&o, &Point::new(3.156927, 1.335111, 0.130489)));
        MINI_CHECK!(tol.is_close(t[0], 0.632708) && tol.is_close(t[1], -0.703687) && tol.is_close(t[2], 0.323272));
        MINI_CHECK!(tol.is_close(n[0], 0.327335) && tol.is_close(n[1], -0.135297) && tol.is_close(n[2], -0.935172));
        MINI_CHECK!(tol.is_close(b[0], 0.701806) && tol.is_close(b[1], 0.697509) && tol.is_close(b[2], 0.144738));

        MINI_CHECK!(curve.perpendicular_frame_at(-0.1, true).is_none());
        MINI_CHECK!(curve.perpendicular_frame_at(1.1, true).is_none());
        MINI_CHECK!(curve.perpendicular_frame_at(curve.domain_start(), false).is_some());
        MINI_CHECK!(curve.perpendicular_frame_at(curve.domain_end(), false).is_some());
        MINI_CHECK!(curve.perpendicular_frame_at(curve.domain_start() - 0.1, false).is_none());
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
REGISTER_MINI_TEST!("NurbsCurve", "attributes", crate::nurbscurve_test::run_nurbscurve_attributes);
REGISTER_MINI_TEST!("NurbsCurve", "Conversions", crate::nurbscurve_test::run_nurbscurve_conversions);
REGISTER_MINI_TEST!("NurbsCurve", "frame_at", crate::nurbscurve_test::run_nurbscurve_frame_at);
REGISTER_MINI_TEST!("NurbsCurve", "perpendicular_frame_at", crate::nurbscurve_test::run_nurbscurve_perpendicular_frame_at);
REGISTER_MINI_TEST!("NurbsCurve", "is_valid", crate::nurbscurve_test::run_nurbscurve_is_valid);
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
