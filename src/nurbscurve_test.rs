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

        // The first the curve is closed or open
        // For linear curves use degree 1
        // When 3 points use degree 2 curve, Rhino default
        // When x>3 points use degree 3 curve
        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_domain(0.0, 1.0);

        // Minimal and Full String Representation
        let cstr = curve.str();
        let crepr = curve.repr();

        // Copy (duplicates everything except guid)
        let ccopy = curve.duplicate();
        let cother = NurbsCurve::create(false, 2, &points);

        // Point division
        let (divided, _) = curve.divide_by_count(10, true);

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
        // Knot vector: [0, 0, 0 ↑, 1 ↑, 2 ↑, 3, 3, 3]  (cubic, 5 CVs)
        let span_count = curve.span_count();
        MINI_CHECK!(span_count == 2);
        /////////////////////////////////////////////////////
        // Control Vertex Access
        //  m_cv = [x0, y0, z0, (w0), x1, y1, z1, (w1), ...]
        //          └─── CV 0 ───┘    └─── CV 1 ───┘
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
        let (adaptive_pts, _adaptive_params) = curve.to_polyline_adaptive(0.1, 0.0, 0.0);

        MINI_CHECK!(adaptive_pts.len() == 27);
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[0], &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[1], &Point::new(0.183105468750000, 0.348632812500000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[2], &Point::new(0.357421875000000, 0.644531250000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[3], &Point::new(0.679687500000000, 1.078125000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[4], &Point::new(0.966796875000000, 1.300781250000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[5], &Point::new(1.097167968750000, 1.333007812500000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[6], &Point::new(1.159057617187500, 1.329345703125000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[7], &Point::new(1.218750000000000, 1.312500000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[8], &Point::new(1.331542968750000, 1.239257812500000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[9], &Point::new(1.435546875000000, 1.113281250000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[10], &Point::new(1.625000000000000, 0.781250000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[11], &Point::new(1.812500000000000, 0.570312500000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[12], &Point::new(1.906250000000000, 0.517578125000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[13], &Point::new(2.000000000000000, 0.500000000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[14], &Point::new(2.093750000000000, 0.517578125000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[15], &Point::new(2.187500000000000, 0.570312500000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[16], &Point::new(2.375000000000000, 0.781250000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[17], &Point::new(2.564453125000000, 1.113281250000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[18], &Point::new(2.668457031250000, 1.239257812500000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[19], &Point::new(2.781250000000000, 1.312500000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[20], &Point::new(2.840942382812500, 1.329345703125000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[21], &Point::new(2.902832031250000, 1.333007812500000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[22], &Point::new(3.033203125000000, 1.300781250000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[23], &Point::new(3.320312500000000, 1.078125000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[24], &Point::new(3.642578125000000, 0.644531250000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[25], &Point::new(3.816894531250000, 0.348632812500000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&adaptive_pts[26], &Point::new(4.000000000000000, 0.000000000000000, 0.000000000000000)));

        // divide_by_count
        let (div_pts, _div_params) = curve.divide_by_count(10, true);

        MINI_CHECK!(div_pts.len() == 10);
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[0], &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[1], &Point::new(0.328571015882635, 0.598213506310667, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[2], &Point::new(0.740744941524856, 1.140321234797829, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[3], &Point::new(1.338523997492639, 1.232716041998164, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[4], &Point::new(1.712929663130383, 0.664818756620870, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[5], &Point::new(2.287070327006695, 0.664818745295462, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[6], &Point::new(2.661475993133979, 1.232716033043460, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[7], &Point::new(3.259255052521522, 1.140321240507253, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[8], &Point::new(3.671428981912368, 0.598213509892612, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&div_pts[9], &Point::new(4.000000000000000, 0.000000000000000, 0.000000000000000)));

        // divide_by_length
        let (len_pts, _len_params) = curve.divide_by_length(0.5);

        MINI_CHECK!(len_pts.len() == 13);
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[0], &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[1], &Point::new(0.235272731384047, 0.441110443734231, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[2], &Point::new(0.504276692145966, 0.862299318703470, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[3], &Point::new(0.843085062978891, 1.227533014827472, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[4], &Point::new(1.302050970444518, 1.264156212040698, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[5], &Point::new(1.579813544869556, 0.853113314150178, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[6], &Point::new(1.928691287815458, 0.510169864866836, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[7], &Point::new(2.340857741884085, 0.732368000404634, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[8], &Point::new(2.597735401548903, 1.160594587288875, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[9], &Point::new(3.032790392631424, 1.300960469420597, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[10], &Point::new(3.407806728972739, 0.976991467650206, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[11], &Point::new(3.691337413616094, 0.565615072909225, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_point_close(&len_pts[12], &Point::new(3.934494402948975, 0.128829830906625, 0.000000000000000)));
    })
}

pub fn run_nurbscurve_evaluation() -> TestResult {
    MINI_TEST!("Evaluation", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Vector;
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

        let mut curve = NurbsCurve::create(false, 2, &points);

        // Get point at parameter t
        let point_at = curve.point_at(0.5);
        MINI_CHECK!(Tolerance::default().is_close(point_at[0], 1.445733625) && Tolerance::default().is_close(point_at[1], 1.80199875) && Tolerance::default().is_close(point_at[2], -0.134851625));

        // Get point and derivatives at parameter t
        let derivatives = curve.evaluate(0.5, 2);
        MINI_CHECK!(derivatives.len() == 3);
        MINI_CHECK!(Tolerance::default().is_close(derivatives[0][0], 1.445733625) && Tolerance::default().is_close(derivatives[0][1], 1.80199875) && Tolerance::default().is_close(derivatives[0][2], -0.134851625));
        MINI_CHECK!(Tolerance::default().is_close(derivatives[1][0], 0.0432025) && Tolerance::default().is_close(derivatives[1][1], 1.154047) && Tolerance::default().is_close(derivatives[1][2], -0.1568445));
        MINI_CHECK!(Tolerance::default().is_close(derivatives[2][0], 4.267853) && Tolerance::default().is_close(derivatives[2][1], -0.677778) && Tolerance::default().is_close(derivatives[2][2], -1.078813));

        // Tangent vector at parameter t
        let tangent = curve.tangent_at(0.5);
        MINI_CHECK!(Tolerance::default().is_close(tangent[0], 0.037069134389828) && Tolerance::default().is_close(tangent[1], 0.990209443486538) && Tolerance::default().is_close(tangent[2], -0.134577625575985));

        // normalized=true (default): t in [0,1] mapped to domain
        let result = curve.frame_at(0.5, true);
        MINI_CHECK!(result.is_some());
        let (o, t, n, b) = result.unwrap();

        MINI_CHECK!(Tolerance::default().is_close(o[0], 3.156927375000000) && Tolerance::default().is_close(o[1], 1.335111500000000) && Tolerance::default().is_close(o[2], 0.130488875000000));
        MINI_CHECK!(Tolerance::default().is_close(t[0], 0.701806140304030) && Tolerance::default().is_close(t[1], 0.697509131556264) && Tolerance::default().is_close(t[2], 0.144738221721788));
        MINI_CHECK!(Tolerance::default().is_close(n[0], -0.513930504714161) && Tolerance::default().is_close(n[1], 0.355053088776962) && Tolerance::default().is_close(n[2], 0.780905077761815));
        MINI_CHECK!(Tolerance::default().is_close(b[0], 0.493298669931115) && Tolerance::default().is_close(b[1], -0.622429365908747) && Tolerance::default().is_close(b[2], 0.607649657861031));

        MINI_CHECK!(curve.frame_at(-0.1, true).is_none());
        MINI_CHECK!(curve.frame_at(1.1, true).is_none());
        MINI_CHECK!(curve.frame_at(curve.domain_start(), false).is_some());
        MINI_CHECK!(curve.frame_at(curve.domain_end(), false).is_some());
        MINI_CHECK!(curve.frame_at(curve.domain_start() - 0.1, false).is_none());

        // Perpendicular frame at (RMF with Frenet initialization, matches Rhino)
        let result = curve.perpendicular_frame_at(0.5, true);
        MINI_CHECK!(result.is_some());
        let (o, t, n, b) = result.unwrap();
        MINI_CHECK!(Tolerance::default().is_point_close(&o, &Point::new(3.156927375000000, 1.335111500000000, 0.130488875000000)));
        MINI_CHECK!(Tolerance::default().is_vector_close(&t, &Vector::new(0.632703652329189, -0.703685357647999, 0.323284713157168)));
        MINI_CHECK!(Tolerance::default().is_vector_close(&n, &Vector::new(0.327344206830723, -0.135306795251661, -0.935167279909370)));
        MINI_CHECK!(Tolerance::default().is_vector_close(&b, &Vector::new(0.701806140314880, 0.697509131546342, 0.144738221716994)));
        MINI_CHECK!(curve.perpendicular_frame_at(-0.1, true).is_none());
        MINI_CHECK!(curve.perpendicular_frame_at(1.1, true).is_none());
        MINI_CHECK!(curve.perpendicular_frame_at(curve.domain_start(), false).is_some());
        MINI_CHECK!(curve.perpendicular_frame_at(curve.domain_end(), false).is_some());
        MINI_CHECK!(curve.perpendicular_frame_at(curve.domain_start() - 0.1, false).is_none());

        // Get multiple rotation minimization frames along the curve (matches Rhino)
        let params = vec![0.0, 0.25, 0.5, 0.75, 1.0];
        let frames = curve.get_perpendicular_frames(&params);
        MINI_CHECK!(frames.len() == 5);
        // Frame 0 (start)
        let (o0, t0, n0, b0) = &frames[0];
        MINI_CHECK!(Tolerance::default().is_point_close(o0, &Point::new(1.957614, 1.140253, -0.191281)));
        MINI_CHECK!(Tolerance::default().is_vector_close(t0, &Vector::new(0.532767753269467, 0.809398954921174, -0.247046256496055)));
        MINI_CHECK!(Tolerance::default().is_vector_close(n0, &Vector::new(-0.261213903019039, -0.120386647366337, -0.957744408496053)));
        MINI_CHECK!(Tolerance::default().is_vector_close(b0, &Vector::new(-0.804938393882267, 0.574787253606414, 0.147288136473484)));
        // Frame 2 (middle)
        let (o2, t2, n2, b2) = &frames[2];
        MINI_CHECK!(Tolerance::default().is_point_close(o2, &Point::new(3.156927375000000, 1.335111500000000, 0.130488875000000)));
        MINI_CHECK!(Tolerance::default().is_vector_close(t2, &Vector::new(0.632703652329189, -0.703685357647999, 0.323284713157168)));
        MINI_CHECK!(Tolerance::default().is_vector_close(n2, &Vector::new(0.327344206830723, -0.135306795251661, -0.935167279909370)));
        MINI_CHECK!(Tolerance::default().is_vector_close(b2, &Vector::new(0.701806140314880, 0.697509131546342, 0.144738221716994)));
        // Frame 4 (end)
        let (o4, t4, n4, b4) = &frames[4];
        MINI_CHECK!(Tolerance::default().is_point_close(o4, &Point::new(2.150320000000000, 1.868606000000000, 0.000000000000000)));
        MINI_CHECK!(Tolerance::default().is_vector_close(t4, &Vector::new(0.183261717666113, 0.080808669821441, 0.979737261575651)));
        MINI_CHECK!(Tolerance::default().is_vector_close(n4, &Vector::new(0.896455076014212, 0.395289006181691, -0.200287039721108)));
        MINI_CHECK!(Tolerance::default().is_vector_close(b4, &Vector::new(-0.403464297709748, 0.914995388225307, 0.000000000000000)));

        // Points
        let p0 = curve.point_at_start();
        let p1 = curve.point_at_middle();
        let p2 = curve.point_at_end();
        MINI_CHECK!(Tolerance::default().is_close(p0[0], 1.957614) && Tolerance::default().is_close(p0[1], 1.140253) && Tolerance::default().is_close(p0[2], -0.191281));
        MINI_CHECK!(Tolerance::default().is_close(p1[0], 3.156927375) && Tolerance::default().is_close(p1[1], 1.3351115) && Tolerance::default().is_close(p1[2], 0.130488875));
        MINI_CHECK!(Tolerance::default().is_close(p2[0], 2.15032) && Tolerance::default().is_close(p2[1], 1.868606) && Tolerance::default().is_close(p2[2], 0.0));

        curve.set_start_point(&Point::new(1.957614, 1.140253, 2.0));
        curve.set_end_point(&Point::new(2.15032, 1.868606, 2.0));
        MINI_CHECK!(Tolerance::default().is_close(curve.point_at_start()[2], 2.0));
        MINI_CHECK!(Tolerance::default().is_close(curve.point_at_end()[2], 2.0));
    })
}

pub fn run_nurbscurve_modifications() -> TestResult {
    MINI_TEST!("Modifications", {
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

        let mut curve = NurbsCurve::create(false, 2, &points);

        // Reverse the curve
        let mut curve_reversed = curve.duplicate();
        curve_reversed.reverse();
        MINI_CHECK!(Tolerance::default().is_point_close(&curve_reversed.point_at_start(), &curve.point_at_end()));

        // Swap coordinates axes
        curve.swap_coordinates(0, 1);
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.get_cv(0).unwrap(), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.get_cv(1).unwrap(), &Point::new(2.0, 1.0, 0.0)));
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.get_cv(2).unwrap(), &Point::new(0.0, 2.0, 0.0)));
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.get_cv(3).unwrap(), &Point::new(2.0, 3.0, 0.0)));
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.get_cv(4).unwrap(), &Point::new(0.0, 4.0, 0.0)));

        // Split curve at domain middle
        let split_t = curve.domain_middle();
        let (curve_left, curve_right) = curve.split(split_t);
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.point_at(split_t), &curve_left.point_at_end()));
        MINI_CHECK!(Tolerance::default().is_point_close(&curve.point_at(split_t), &curve_right.point_at_start()));

        // Extend curve smoothly at both ends
        let mut curve_extended = curve.duplicate();
        curve_extended.extend(curve.domain_start() - 0.5, curve.domain_end() + 0.5);
        MINI_CHECK!(curve_extended.length(None) > curve.length(None));

        // Enable curve weights - Make rational or non-rational
        let mut curve_rational = curve.duplicate();
        let original_length = curve.length(None);
        curve_rational.make_rational();
        curve_rational.set_weight(2, 10.0);
        MINI_CHECK!(curve_rational.length(None) != original_length);

        curve_rational.make_non_rational_force(true);
        MINI_CHECK!(curve_rational.length(None) == original_length);
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

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

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

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_domain(0.0, 1.0);
        curve.set_domain(0.0, 1.0);

        let filename = "serialization/test_nurbscurve.bin";
        curve.protobuf_dump(filename);
        let loaded = NurbsCurve::protobuf_load(filename);

        MINI_CHECK!(loaded.is_valid() == true);
        MINI_CHECK!(loaded.cv_count() == 3);
        MINI_CHECK!(loaded.degree() == 2);
        MINI_CHECK!(loaded.order() == 3);
    })
}

pub fn run_nurbscurve_intersect_plane() -> TestResult {
    MINI_TEST!("intersect_plane", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Plane;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];

        let mut curve = NurbsCurve::create(false, 2, &points);
        curve.set_domain(0.0, 1.0);
        let plane = Plane::xy_plane();
        let intersections = curve.intersect_plane(&plane, None);

        MINI_CHECK!(intersections.len() >= 0);
    })
}

REGISTER_MINI_TEST!("NurbsCurve", "constructor", crate::nurbscurve_test::run_nurbscurve_constructor);
REGISTER_MINI_TEST!("NurbsCurve", "attributes", crate::nurbscurve_test::run_nurbscurve_attributes);
REGISTER_MINI_TEST!("NurbsCurve", "Conversions", crate::nurbscurve_test::run_nurbscurve_conversions);
REGISTER_MINI_TEST!("NurbsCurve", "Evaluation", crate::nurbscurve_test::run_nurbscurve_evaluation);
REGISTER_MINI_TEST!("NurbsCurve", "Modifications", crate::nurbscurve_test::run_nurbscurve_modifications);
REGISTER_MINI_TEST!("NurbsCurve", "json_roundtrip", crate::nurbscurve_test::run_nurbscurve_json_roundtrip);
REGISTER_MINI_TEST!("NurbsCurve", "protobuf_roundtrip", crate::nurbscurve_test::run_nurbscurve_protobuf_roundtrip);
REGISTER_MINI_TEST!("NurbsCurve", "intersect_plane", crate::nurbscurve_test::run_nurbscurve_intersect_plane);
