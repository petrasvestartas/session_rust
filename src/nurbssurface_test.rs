use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_nurbssurface_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Get mesh
        let _m = s.mesh();

        // Point division matching Rhino's 4x6 grid
        let (p, _v, _uv) = s.divide_by_count_points(4, 6);

        // Minimal and Full String Representation
        let sstr = s.to_string();
        let srepr = s.repr();

        // Copy (duplicates everything except guid)
        let scopy = s.duplicate();
        let _sother = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        MINI_CHECK!(s.is_valid() == true);
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 4);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(s.cv_count_dir(None) == 16);
        MINI_CHECK!(s.degree(0) == 3);
        MINI_CHECK!(s.degree(1) == 3);
        MINI_CHECK!(s.order(0) == 4);
        MINI_CHECK!(s.order(1) == 4);
        MINI_CHECK!(s.dimension() == 3);
        MINI_CHECK!(!s.is_rational());
        MINI_CHECK!(s.knot_count(0) == 6);
        MINI_CHECK!(s.knot_count(1) == 6);
        MINI_CHECK!(s.name == "my_nurbssurface");
        MINI_CHECK!(!s.guid.is_empty());
        MINI_CHECK!(sstr == "NurbsSurface(name=my_nurbssurface, degree=(3,3), cvs=(4,4))");
        MINI_CHECK!(srepr == "NurbsSurface(\n  name=my_nurbssurface,\n  degree=(3,3),\n  cvs=(4,4),\n  rational=false,\n  control_points=[\n    0, 0, 0\n    -1, 0.75, 2\n    -1, 4.25, 2\n    0, 5, 0\n    0.75, -1, 2\n    1.25, 1.25, 4\n    1.25, 3.75, 4\n    0.75, 6, 2\n    4.25, -1, 2\n    3.75, 1.25, 4\n    3.75, 3.75, 4\n    4.25, 6, 2\n    5, 0, 0\n    6, 0.75, 2\n    6, 4.25, 2\n    5, 5, 0\n  ]\n)");
        MINI_CHECK!(scopy.cv_count_dir(None) == s.cv_count_dir(None));
        MINI_CHECK!(scopy.guid != s.guid);
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][0], &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][1], &Point::new(-0.416666666666667, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][2], &Point::new(-0.666666666666667, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][3], &Point::new(-0.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][4], &Point::new(-0.666666666666667, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][5], &Point::new(-0.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[0][6], &Point::new(0.000000000000000, 5.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][0], &Point::new(0.992187500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][1], &Point::new(0.881510416666667, 0.333912037037037, 1.958333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][2], &Point::new(0.815104166666667, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][3], &Point::new(0.792968750000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][4], &Point::new(0.815104166666667, 3.620370370370370, 2.458333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][5], &Point::new(0.881510416666667, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[1][6], &Point::new(0.992187500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][0], &Point::new(2.500000000000000, -0.750000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][1], &Point::new(2.500000000000000, 0.252314814814815, 2.333333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][2], &Point::new(2.500000000000000, 1.351851851851852, 2.833333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][3], &Point::new(2.500000000000000, 2.500000000000000, 3.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][4], &Point::new(2.500000000000000, 3.648148148148148, 2.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][5], &Point::new(2.500000000000000, 4.747685185185186, 2.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[2][6], &Point::new(2.500000000000000, 5.750000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][0], &Point::new(4.007812500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][1], &Point::new(4.118489583333334, 0.333912037037037, 1.958333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][2], &Point::new(4.184895833333334, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][3], &Point::new(4.207031250000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][4], &Point::new(4.184895833333333, 3.620370370370370, 2.458333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][5], &Point::new(4.118489583333333, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[3][6], &Point::new(4.007812500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][0], &Point::new(5.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][1], &Point::new(5.416666666666668, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][2], &Point::new(5.666666666666668, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][3], &Point::new(5.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][4], &Point::new(5.666666666666666, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][5], &Point::new(5.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&p[4][6], &Point::new(5.000000000000000, 5.000000000000000, 0.000000000000000)));
    })
}

pub fn run_nurbssurface_booleans_queries() -> TestResult {
    MINI_TEST!("Booleans Queries", {
        use crate::primitives::Primitives;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 5.0);

        // Validity surface and knots
        let is_valid = s.is_valid();
        let are_knots_valid = s.is_valid_knot_vector(0) && s.is_valid_knot_vector(1);

        // Are control points weights enabled?
        let is_rational = s.is_rational();

        // Sphere has one seam that is closed, but two poles
        let is_closed = s.is_closed(0) == true && s.is_closed(1) == false;

        // sphere cannot be truly periodic because it has poles
        let is_periodic = s.is_periodic(0) && s.is_periodic(1);

        // Planarity
        let is_planar = s.is_planar(1e-6);

        // Surface is collapsed to a point
        let is_point = s.is_singular(0) && s.is_singular(1) && s.is_singular(2) && s.is_singular(3);

        // Most surfaces are clamped except periodic surfaces
        let is_clamped = s.is_clamped(0, 2) && s.is_clamped(1, 2);

        MINI_CHECK!(is_valid);
        MINI_CHECK!(are_knots_valid);
        MINI_CHECK!(is_rational);
        MINI_CHECK!(is_closed);
        MINI_CHECK!(!is_periodic);
        MINI_CHECK!(!is_planar);
        MINI_CHECK!(!is_point);
        MINI_CHECK!(is_clamped);
    })
}

pub fn run_nurbssurface_attributes() -> TestResult {
    MINI_TEST!("Attributes", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Check the dimentions of a surface
        // Mostly 3d
        // But 2d can be used for: scalar field over parameter space e.g. czrvatzre map, distance field
        // Planar geometry: texture coordinates
        let dimensions = s.dimension();

        // Degree types 1 - linear, 2 - quadratic, 3 - cubic
        let order_u = s.order(0);
        let order_v = s.order(1);

        // Control vertex count
        let cv_count_u = s.cv_count_dir(Some(0));
        let cv_count_v = s.cv_count_dir(Some(1));
        let cv_count = s.cv_count_dir(None);
        let cv_size = s.cv_size();

        // Number of knots
        let k_count_0 = s.knot_count(0);
        let k_count_1 = s.knot_count(1);

        // Span count
        let s_count_0 = s.span_count(0);
        let s_count_1 = s.span_count(1);

        MINI_CHECK!(dimensions == 3);
        MINI_CHECK!(order_u == 4);
        MINI_CHECK!(order_v == 4);
        MINI_CHECK!(cv_count_u > 0);
        MINI_CHECK!(cv_count_v > 0);
        MINI_CHECK!(cv_count > 0);
        MINI_CHECK!(cv_size > 0);
        MINI_CHECK!(k_count_0 > 0);
        MINI_CHECK!(k_count_1 > 0);
        MINI_CHECK!(s_count_0 > 0);
        MINI_CHECK!(s_count_1 > 0);
    })
}

pub fn run_nurbssurface_control_vertices_access() -> TestResult {
    MINI_TEST!("Control Vertices Access", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        s.make_rational();

        // Raw CV access
        let cv_slice = s.cv(0, 0).unwrap();
        MINI_CHECK!(cv_slice[2] == 0.0);
        let cv_mut_slice = s.cv_mut(0, 0).unwrap();
        cv_mut_slice[2] = 10.0;
        MINI_CHECK!(s.cv(0, 0).unwrap()[2] == 10.0);

        // Point and Weight
        // NOTE
        // point is (Xw, Yw, Zw, w)
        // cv pointer is (X, Y, Z)
        let cv = s.get_cv(0, 0).unwrap();
        MINI_CHECK!(cv == Point::new(0.0, 0.0, 10.0));
        let (x, y, z, w) = s.get_cv_4d(0, 0).unwrap();
        MINI_CHECK!(x == 0.0 && y == 0.0 && z == 10.0 && w == 1.0);

        s.set_cv(0, 0, &Point::new(0.0, 0.0, 5.0));
        MINI_CHECK!(s.get_cv(0, 0).unwrap() == Point::new(0.0, 0.0, 5.0));
        s.set_cv_4d(0, 0, 0.0, 0.0, 4.0, 0.5);
        MINI_CHECK!(s.get_cv(0, 0).unwrap() == Point::new(0.0, 0.0, 8.0) && s.cv(0, 0).unwrap()[2] == 4.0 && s.weight(0, 0) == 0.5);

        let _w = s.weight(0, 0);
        s.set_weight(0, 0, 1.0);
        MINI_CHECK!(s.weight(0, 0) == 1.0);
    })
}

pub fn run_nurbssurface_knot_access() -> TestResult {
    MINI_TEST!("Knot Access", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Get knot vectors and individual knot
        let knots_u = s.get_knots(0);
        for i in 0..s.knot_count(0) as usize {
            let knot = s.knot(0, i).unwrap();
            MINI_CHECK!(knot == knots_u[i]);
        }

        let knots_v = s.get_knots(1);
        for i in 0..s.knot_count(1) as usize {
            let knot = s.knot(1, i).unwrap();
            MINI_CHECK!(knot == knots_v[i]);
        }

        // Set knots
        let _is_set = s.set_knot(0, 2, 0.5);
        MINI_CHECK!(s.knot(0, 2).unwrap() == 0.5);
        let _is_set = s.set_knot(0, 2, 0.0);

        // Verify start multiplicity
        let mult_u_start = s.knot_multiplicity(0, 0);
        let mult_v_start = s.knot_multiplicity(1, 0);
        MINI_CHECK!(mult_u_start == 3);
        MINI_CHECK!(mult_v_start == 3);

        s.insert_knot(0, 0.1, 2);
        MINI_CHECK!(s.knot_count(0) == 8);
        MINI_CHECK!(s.knot(0, 3).unwrap() == 0.1);
        MINI_CHECK!(s.knot_multiplicity(0, 3) == 2);
    })
}

pub fn run_nurbssurface_domain() -> TestResult {
    MINI_TEST!("Domain", {
        use crate::NurbsSurface;
        use crate::Point;


        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Get domain 0 - 1
        let domain_u = s.domain(0).unwrap();
        let _domain_v = s.domain(1).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(domain_u.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(domain_u.1, 1.0));

        // Set Domain
        let is_set_u = s.set_domain(0, -1.1, 2.3);
        let is_set_v = s.set_domain(1, -5.1, 1.3);
        MINI_CHECK!(is_set_u && TOLERANCE.is_close(s.domain(1).unwrap().0, -5.1));
        MINI_CHECK!(is_set_v && TOLERANCE.is_close(s.domain(1).unwrap().1, 1.3));

        // Get sorted list of distinct knot values
        let span_vector = s.get_span_vector(0);
        let first_item = span_vector[0];
        let last_item = span_vector[span_vector.len() - 1];
        MINI_CHECK!(TOLERANCE.is_close(first_item, -1.1));
        MINI_CHECK!(TOLERANCE.is_close(last_item, 2.3));
    })
}

pub fn run_nurbssurface_division() -> TestResult {
    MINI_TEST!("Division", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Vector;


        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // points, normals, uv
        let (division_points, vectors, uvs0) = s.divide_by_count_points(3, 3);

        // planes, uv
        let (planes, _uvs1) = s.divide_by_count_planes(3, 3);

        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[0][0], &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[0][1], &Point::new(-0.666666666666667, 1.46296296296296, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[0][2], &Point::new(-0.666666666666667, 3.53703703703704, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[0][3], &Point::new(0.0, 5.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[1][0], &Point::new(1.46296296296296, -0.666666666666667, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[1][1], &Point::new(1.3641975308642, 1.3641975308642, 2.66666666666667)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[1][2], &Point::new(1.3641975308642, 3.6358024691358, 2.66666666666667)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[1][3], &Point::new(1.46296296296296, 5.66666666666667, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[2][0], &Point::new(3.53703703703704, -0.666666666666667, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[2][1], &Point::new(3.6358024691358, 1.3641975308642, 2.66666666666667)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[2][2], &Point::new(3.6358024691358, 3.6358024691358, 2.66666666666667)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[2][3], &Point::new(3.53703703703704, 5.66666666666667, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[3][0], &Point::new(5.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[3][1], &Point::new(5.66666666666667, 1.46296296296296, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[3][2], &Point::new(5.66666666666667, 3.53703703703704, 1.33333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[3][3], &Point::new(5.0, 5.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[0][0], &Vector::new(-0.704360725060499, -0.704360725060499, -0.0880450906325624)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[0][1], &Vector::new(-0.722897836195991, -0.327787263130091, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[0][2], &Vector::new(-0.722897836195991, 0.327787263130091, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[0][3], &Vector::new(-0.704360725060499, 0.704360725060499, -0.0880450906325624)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[1][0], &Vector::new(-0.327787263130091, -0.722897836195991, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[1][1], &Vector::new(-0.280457757277237, -0.280457757277237, 0.917979788865771)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[1][2], &Vector::new(-0.280457757277237, 0.280457757277237, 0.917979788865771)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[1][3], &Vector::new(-0.327787263130091, 0.722897836195991, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[2][0], &Vector::new(0.327787263130091, -0.722897836195991, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[2][1], &Vector::new(0.280457757277237, -0.280457757277237, 0.917979788865771)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[2][2], &Vector::new(0.280457757277237, 0.280457757277237, 0.917979788865771)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[2][3], &Vector::new(0.327787263130091, 0.722897836195991, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[3][0], &Vector::new(0.704360725060499, -0.704360725060499, -0.0880450906325624)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[3][1], &Vector::new(0.722897836195991, -0.327787263130091, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[3][2], &Vector::new(0.722897836195991, 0.327787263130091, 0.608255068661856)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&vectors[3][3], &Vector::new(0.704360725060499, 0.704360725060499, -0.0880450906325624)));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][0].0, 0.0) && TOLERANCE.is_close(uvs0[0][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][1].0, 0.0) && TOLERANCE.is_close(uvs0[0][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][2].0, 0.0) && TOLERANCE.is_close(uvs0[0][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][3].0, 0.0) && TOLERANCE.is_close(uvs0[0][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][0].0, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][1].0, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][2].0, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][3].0, 0.333333333333333) && TOLERANCE.is_close(uvs0[1][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][0].0, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][1].0, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][2].0, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][3].0, 0.666666666666667) && TOLERANCE.is_close(uvs0[2][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][0].0, 1.0) && TOLERANCE.is_close(uvs0[3][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][1].0, 1.0) && TOLERANCE.is_close(uvs0[3][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][2].0, 1.0) && TOLERANCE.is_close(uvs0[3][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][3].0, 1.0) && TOLERANCE.is_close(uvs0[3][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][0].x_axis(), &Vector::new(0.317999364001908, -0.423999152002544, 0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][1].x_axis(), &Vector::new(0.657483781160109, -0.0556600026378928, 0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][2].x_axis(), &Vector::new(0.657483781160109, 0.055660002637893, 0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][3].x_axis(), &Vector::new(0.317999364001908, 0.423999152002544, 0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][0].x_axis(), &Vector::new(0.93542594448836, -0.158100159631836, 0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][1].x_axis(), &Vector::new(0.957938608304167, -0.0211991946512679, 0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][2].x_axis(), &Vector::new(0.957938608304167, 0.0211991946512677, 0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][3].x_axis(), &Vector::new(0.93542594448836, 0.158100159631835, 0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][0].x_axis(), &Vector::new(0.93542594448836, 0.158100159631835, -0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][1].x_axis(), &Vector::new(0.957938608304167, 0.0211991946512679, -0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][2].x_axis(), &Vector::new(0.957938608304167, -0.021199194651268, -0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][3].x_axis(), &Vector::new(0.93542594448836, -0.158100159631836, -0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][0].x_axis(), &Vector::new(0.317999364001908, 0.423999152002544, -0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][1].x_axis(), &Vector::new(0.657483781160109, 0.0556600026378928, -0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][2].x_axis(), &Vector::new(0.657483781160109, -0.0556600026378928, -0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][3].x_axis(), &Vector::new(0.317999364001908, -0.423999152002544, -0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][0].y_axis(), &Vector::new(-0.423999152002544, 0.317999364001908, 0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][1].y_axis(), &Vector::new(-0.158100159631836, 0.93542594448836, 0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][2].y_axis(), &Vector::new(0.158100159631835, 0.93542594448836, -0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[0][3].y_axis(), &Vector::new(0.423999152002544, 0.317999364001908, -0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][0].y_axis(), &Vector::new(-0.0556600026378928, 0.657483781160109, 0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][1].y_axis(), &Vector::new(-0.0211991946512679, 0.957938608304167, 0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][2].y_axis(), &Vector::new(0.0211991946512679, 0.957938608304167, -0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[1][3].y_axis(), &Vector::new(0.0556600026378928, 0.657483781160109, -0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][0].y_axis(), &Vector::new(0.0556600026378928, 0.657483781160109, 0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][1].y_axis(), &Vector::new(0.0211991946512678, 0.957938608304167, 0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][2].y_axis(), &Vector::new(-0.0211991946512678, 0.957938608304167, -0.286189127792116)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[2][3].y_axis(), &Vector::new(-0.0556600026378928, 0.657483781160109, -0.751410035611553)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][0].y_axis(), &Vector::new(0.423999152002544, 0.317999364001908, 0.847998304005088)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][1].y_axis(), &Vector::new(0.158100159631835, 0.93542594448836, 0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][2].y_axis(), &Vector::new(-0.158100159631836, 0.93542594448836, -0.316200319263671)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&planes[3][3].y_axis(), &Vector::new(-0.423999152002544, 0.317999364001908, -0.847998304005088)));
    })
}

pub fn run_nurbssurface_evaluation() -> TestResult {
    MINI_TEST!("Evaluation", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::vector::Vector;

        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let u = 0.5;
        let v = 0.5;

        // point_at(u, v) - returns Point
        let p1 = s.point_at(u, v).unwrap();
        MINI_CHECK!(TOLERANCE.is_point_close(&p1, &Point::new(2.5, 2.5, 3.0)));

        // normal_at(u, v) - returns Vector
        let n1 = s.normal_at(u, v);
        MINI_CHECK!(TOLERANCE.is_vector_close(&n1, &Vector::new(0.0, 0.0, 1.0)));

        // evaluate(u, v, num_derivs) - returns vector of derivatives
        let derivs = s.evaluate(u, v, 1);
        MINI_CHECK!(TOLERANCE.is_vector_close(&derivs[0], &Vector::new(2.5, 2.5, 3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&derivs[1], &Vector::new(0.0, 6.9375, 0.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&derivs[2], &Vector::new(6.9375, 0.0, 0.0)));

        // point_at_corner(u_end, v_end) - corner point
        let p_corner = s.point_at_corner(1, 1).unwrap();
        MINI_CHECK!(TOLERANCE.is_point_close(&p_corner, &Point::new(5.0, 5.0, 0.0)));

        // get isocurve - returns NurbsCurve
        let iso_u = s.iso_curve(0, v).unwrap();
        let iso_v = s.iso_curve(1, u).unwrap();
        MINI_CHECK!(TOLERANCE.is_point_close(&iso_u.point_at(0.5), &Point::new(2.5, 2.5, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&iso_v.point_at(0.5), &Point::new(2.5, 2.5, 3.0)));
    })
}

pub fn run_nurbssurface_modification() -> TestResult {
    MINI_TEST!("Modification", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let points = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();


        // Reverse one direction
        let mut s_rev = s.clone();
        s_rev.reverse(0);
        MINI_CHECK!(s_rev.point_at_corner(0, 0).unwrap() == s.point_at_corner(1, 0).unwrap());
        MINI_CHECK!(s_rev.normal_at(0.5, 0.5) == s.normal_at(0.5, 0.5) * -1.0);

        // Swap u and v direction
        let mut s_tr = s.clone();
        s_tr.transpose();
        MINI_CHECK!(s.point_at(0.0, 0.5).unwrap() == s_tr.point_at(0.5, 0.0).unwrap());

        // Swap coordinates - swap x and z
        let mut s_swap = s.clone();
        s_swap.swap_coordinates(0, 2);
        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap()[0] == s_swap.point_at(0.5, 0.5).unwrap()[2]);
        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap()[2] == s_swap.point_at(0.5, 0.5).unwrap()[0]);

        // Trim surface, domain changed but parametrization preserved
        let mut s_trim = s.clone();
        s_trim.trim(0, (0.25, 0.75));
        MINI_CHECK!(TOLERANCE.is_close(s_trim.domain(0).unwrap().0, 0.25) && TOLERANCE.is_close(s_trim.domain(0).unwrap().1, 0.75));
        MINI_CHECK!(TOLERANCE.is_point_close(&s.point_at(0.25, 0.5).unwrap(), &s_trim.point_at(0.25, 0.5).unwrap()));

        // Split surface into 4 quadrants, check shared corner point is the same
        let (west, east) = s.split(0, 0.5);
        let west = west.unwrap();
        let east = east.unwrap();
        let (ww, we) = west.split(1, (west.domain(1).unwrap().0 + west.domain(1).unwrap().1) / 2.0);
        let (ew, ee) = east.split(1, (east.domain(1).unwrap().0 + east.domain(1).unwrap().1) / 2.0);
        let ww = ww.unwrap();
        let we = we.unwrap();
        let ew = ew.unwrap();
        let ee = ee.unwrap();
        let center = s.point_at(0.5, 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_point_close(&ww.point_at_corner(1, 1).unwrap(), &center));
        MINI_CHECK!(TOLERANCE.is_point_close(&we.point_at_corner(1, 0).unwrap(), &center));
        MINI_CHECK!(TOLERANCE.is_point_close(&ew.point_at_corner(0, 1).unwrap(), &center));
        MINI_CHECK!(TOLERANCE.is_point_close(&ee.point_at_corner(0, 0).unwrap(), &center));

        // Make rational and change weight
        let mut s_rat = s.clone();
        s_rat.make_rational();
        s_rat.set_weight(2, 2, 3.0);
        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap() != s_rat.point_at(0.5, 0.5).unwrap());
        s_rat.make_non_rational();
        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap() == s_rat.point_at(0.5, 0.5).unwrap());

        // Increase degree
        let mut s_deg = s.clone();
        s_deg.increase_degree(0, 6);
        s_deg.increase_degree(1, 6);
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 4 && s.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(s_deg.cv_count_dir(Some(0)) == 7 && s_deg.cv_count_dir(Some(1)) == 7);
    })
}

pub fn run_nurbssurface_isocurve() -> TestResult {
    MINI_TEST!("Isocurve", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create surface
        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();

        // Extract iso-u curve (v varies)
        let (u0, u1) = surf.domain(0).unwrap();
        let u_mid = (u0 + u1) / 2.0;
        let iso_u = surf.iso_curve(0, u_mid);

        // Extract iso-v curve (u varies)
        let (v0, v1) = surf.domain(1).unwrap();
        let v_mid = (v0 + v1) / 2.0;
        let iso_v = surf.iso_curve(1, v_mid);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(iso_u.is_some());
        MINI_CHECK!(iso_u.unwrap().is_valid());
        MINI_CHECK!(iso_v.is_some());
        MINI_CHECK!(iso_v.unwrap().is_valid());
    })
}

pub fn run_nurbssurface_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::xform::Xform;


        // Create simple surface
        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let mut surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Apply translation
        let xf = Xform::translation(1.0, 2.0, 3.0);
        surf.transform(&xf);

        // Check transformed CV
        let pt = surf.get_cv(0, 0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 3.0));
    })
}

pub fn run_nurbssurface_json_roundtrip() -> TestResult {
    MINI_TEST!("Json_roundtrip", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::color::Color;
        use std::path::PathBuf;

        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.name = "test_nurbssurface".to_string();
        surf.width = 2.0;
        surf.facecolors = vec![Color::new(255, 128, 64, 255)];
        surf.pointcolors = vec![Color::new(0, 255, 0, 255)];
        surf.linecolors = vec![Color::new(0, 0, 255, 255)];

        // JSON object
        let json = surf.jsondump().unwrap();
        let loaded_json = NurbsSurface::jsonload(&json).unwrap();

        // String
        let json_string = surf.json_dumps();
        let loaded_json_string = NurbsSurface::json_loads(&json_string);

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_nurbssurface.json");
        surf.json_dump(filename.to_str().unwrap());
        let loaded_from_file = NurbsSurface::json_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_json == surf);
        MINI_CHECK!(loaded_json_string == surf);
        MINI_CHECK!(loaded_from_file == surf);
    })
}

pub fn run_nurbssurface_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf_roundtrip", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::color::Color;
        use std::path::PathBuf;

        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.name = "test_nurbssurface".to_string();
        surf.width = 2.0;
        surf.facecolors = vec![Color::new(255, 128, 64, 255)];
        surf.pointcolors = vec![Color::new(0, 255, 0, 255)];
        surf.linecolors = vec![Color::new(0, 0, 255, 255)];

        // String
        let proto_string = surf.pb_dumps();
        let loaded_proto_string = NurbsSurface::pb_loads(&proto_string).unwrap();

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_nurbssurface.bin");
        surf.pb_dump(filename.to_str().unwrap());
        let loaded = NurbsSurface::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_proto_string == surf);
        MINI_CHECK!(loaded == surf);
    })
}

pub fn run_nurbssurface_advanced_accessors() -> TestResult {
    MINI_TEST!("Advanced_accessors", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;


        // Create rational surface for testing get_cv_4d/set_cv_4d
        let points = vec![Point::new(0.0, 0.0, 0.0); 9];
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.make_rational();

        // Test set_cv_4d with homogeneous coordinates
        let x = 2.0;
        let y = 3.0;
        let z = 4.0;
        let w = 2.0;

        // Set CV using set_cv_4d
        surf.set_cv_4d(1, 1, x, y, z, w);

        // Get CV and verify using get_cv_4d
        let (rx, ry, rz, rw) = surf.get_cv_4d(1, 1).unwrap();

        // Also test get_cv
        let pt = surf.get_cv(1, 1).unwrap();
        let retrieved_w = surf.weight(1, 1);

        // Test knot_multiplicity
        let mult = surf.knot_count(0);
        let first_knot_mult = if mult > 0 {
            let first_val = surf.knot(0, 0).unwrap();
            let mut count = 1;
            for i in 1..mult as usize {
                if let Some(val) = surf.knot(0, i) {
                    if (val - first_val).abs() < 1e-10 {
                        count += 1;
                    } else {
                        break;
                    }
                }
            }
            count
        } else {
            0
        };

        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(TOLERANCE.is_close(rx, x));
        MINI_CHECK!(TOLERANCE.is_close(ry, y));
        MINI_CHECK!(TOLERANCE.is_close(rz, z));
        MINI_CHECK!(TOLERANCE.is_close(rw, w));
        // get_cv returns Euclidean coordinates, so it divides homogeneous coords by w
        MINI_CHECK!(TOLERANCE.is_close(pt[0], x/w));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], y/w));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], z/w));
        MINI_CHECK!(TOLERANCE.is_close(retrieved_w, w));
        MINI_CHECK!(first_knot_mult > 0);
    })
}

pub fn run_nurbssurface_singularity() -> TestResult {
    MINI_TEST!("Singularity", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create a simple surface with all CVs at different points (non-singular)
        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Test is_singular for each side
        let is_singular_south = surf.is_singular(0);
        let is_singular_east = surf.is_singular(1);
        let is_singular_north = surf.is_singular(2);
        let is_singular_west = surf.is_singular(3);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(!is_singular_south);
        MINI_CHECK!(!is_singular_east);
        MINI_CHECK!(!is_singular_north);
        MINI_CHECK!(!is_singular_west);
    })
}

pub fn run_nurbssurface_domain_operations() -> TestResult {
    MINI_TEST!("Domain_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;


        let points = vec![Point::new(0.0, 0.0, 0.0); 9];
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();

        // Get initial domain
        let dom_u = surf.domain(0).unwrap();
        let dom_v = surf.domain(1).unwrap();

        // Set new domain
        surf.set_domain(0, 0.0, 10.0);
        surf.set_domain(1, 5.0, 15.0);

        let new_dom_u = surf.domain(0).unwrap();
        let new_dom_v = surf.domain(1).unwrap();

        // Get span vectors
        let span_u = surf.get_span_vector(0);
        let span_v = surf.get_span_vector(1);

        MINI_CHECK!(dom_u.0 == 0.0 && dom_u.1 > 0.0);
        MINI_CHECK!(dom_v.0 == 0.0 && dom_v.1 > 0.0);
        MINI_CHECK!(TOLERANCE.is_close(new_dom_u.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(new_dom_u.1, 10.0));
        MINI_CHECK!(TOLERANCE.is_close(new_dom_v.0, 5.0));
        MINI_CHECK!(TOLERANCE.is_close(new_dom_v.1, 15.0));
        MINI_CHECK!(span_u.len() > 0);
        MINI_CHECK!(span_v.len() > 0);
    })
}

pub fn run_nurbssurface_corner_points() -> TestResult {
    MINI_TEST!("Corner_points", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;


        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 10.0, 0.0),
            Point::new(10.0, 0.0, 0.0), Point::new(10.0, 10.0, 0.0),
        ];
        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Get corner points
        let p00 = surf.point_at_corner(0, 0).unwrap();
        let p10 = surf.point_at_corner(1, 0).unwrap();
        let p01 = surf.point_at_corner(0, 1).unwrap();
        let p11 = surf.point_at_corner(1, 1).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(p00[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(p10[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(p01[1], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(p11[0], 10.0) && TOLERANCE.is_close(p11[1], 10.0));
    })
}

pub fn run_nurbssurface_swap_coordinates() -> TestResult {
    MINI_TEST!("Swap_coordinates", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;


        let points = vec![
            Point::new(1.0, 2.0, 3.0), Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let mut surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Swap X and Y
        surf.swap_coordinates(0, 1);

        let pt = surf.get_cv(0, 0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 3.0));
    })
}

pub fn run_nurbssurface_get_knots() -> TestResult {
    MINI_TEST!("Get_knots", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 3, 2, 4, 3, &points).unwrap();

        let knots_u = surf.get_knots(0);
        let knots_v = surf.get_knots(1);

        MINI_CHECK!(knots_u.len() == surf.knot_count(0) as usize);
        MINI_CHECK!(knots_v.len() == surf.knot_count(1) as usize);
        MINI_CHECK!(knots_u.len() > 0);
        MINI_CHECK!(knots_v.len() > 0);
    })
}

pub fn run_nurbssurface_make_non_rational() -> TestResult {
    MINI_TEST!("Make_non_rational", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create surface, then make rational with all weights = 1
        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.make_rational();

        // Set all weights to 1.0
        for i in 0..3 {
            for j in 0..3 {
                surf.set_weight(i, j, 1.0);
            }
        }

        let was_rational = surf.is_rational();
        surf.make_non_rational();
        let is_rational_after = surf.is_rational();

        MINI_CHECK!(was_rational);
        MINI_CHECK!(!is_rational_after);
    })
}

pub fn run_nurbssurface_create_clamped_uniform() -> TestResult {
    MINI_TEST!("Create_clamped_uniform", {
        use crate::nurbssurface::NurbsSurface;

        let mut surf = NurbsSurface::new();
        surf.create_clamped_uniform(3, 4, 3, 4, 4, 1.0, 2.0);

        let _dom_u = surf.domain(0);
        let _dom_v = surf.domain(1);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.dimension() == 3);
        MINI_CHECK!(surf.order(0) == 4);
        MINI_CHECK!(surf.order(1) == 3);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 4);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(surf.is_clamped(0, 0) && surf.is_clamped(0, 1));
        MINI_CHECK!(surf.is_clamped(1, 0) && surf.is_clamped(1, 1));
    })
}

pub fn run_nurbssurface_knot_multiplicity() -> TestResult {
    MINI_TEST!("Knot_multiplicity", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Check first knot multiplicity (should be equal to degree for clamped)
        let mult_u_start = surf.knot_multiplicity(0, 0);
        let mult_v_start = surf.knot_multiplicity(1, 0);

        // Check last knot multiplicity
        let last_u = surf.knot_count(0) - 1;
        let last_v = surf.knot_count(1) - 1;
        let mult_u_end = surf.knot_multiplicity(0, last_u as usize);
        let mult_v_end = surf.knot_multiplicity(1, last_v as usize);

        MINI_CHECK!(mult_u_start >= surf.degree(0) as usize);
        MINI_CHECK!(mult_v_start >= surf.degree(1) as usize);
        MINI_CHECK!(mult_u_end >= surf.degree(0) as usize);
        MINI_CHECK!(mult_v_end >= surf.degree(1) as usize);
    })
}

// Register tests with the shared registry
REGISTER_MINI_TEST!("NurbsSurface", "Constructor", crate::nurbssurface_test::run_nurbssurface_constructor);
REGISTER_MINI_TEST!("NurbsSurface", "Booleans Queries", crate::nurbssurface_test::run_nurbssurface_booleans_queries);
REGISTER_MINI_TEST!("NurbsSurface", "Attributes", crate::nurbssurface_test::run_nurbssurface_attributes);
REGISTER_MINI_TEST!("NurbsSurface", "Control Vertices Access", crate::nurbssurface_test::run_nurbssurface_control_vertices_access);
REGISTER_MINI_TEST!("NurbsSurface", "Knot Access", crate::nurbssurface_test::run_nurbssurface_knot_access);
REGISTER_MINI_TEST!("NurbsSurface", "Domain", crate::nurbssurface_test::run_nurbssurface_domain);
REGISTER_MINI_TEST!("NurbsSurface", "Division", crate::nurbssurface_test::run_nurbssurface_division);
REGISTER_MINI_TEST!("NurbsSurface", "Evaluation", crate::nurbssurface_test::run_nurbssurface_evaluation);
REGISTER_MINI_TEST!("NurbsSurface", "Modification", crate::nurbssurface_test::run_nurbssurface_modification);
REGISTER_MINI_TEST!("NurbsSurface", "Isocurve", crate::nurbssurface_test::run_nurbssurface_isocurve);
REGISTER_MINI_TEST!("NurbsSurface", "Transformation", crate::nurbssurface_test::run_nurbssurface_transformation);
REGISTER_MINI_TEST!("NurbsSurface", "Json_roundtrip", crate::nurbssurface_test::run_nurbssurface_json_roundtrip);
REGISTER_MINI_TEST!("NurbsSurface", "Protobuf_roundtrip", crate::nurbssurface_test::run_nurbssurface_protobuf_roundtrip);
REGISTER_MINI_TEST!("NurbsSurface", "Advanced_accessors", crate::nurbssurface_test::run_nurbssurface_advanced_accessors);
REGISTER_MINI_TEST!("NurbsSurface", "Singularity", crate::nurbssurface_test::run_nurbssurface_singularity);
REGISTER_MINI_TEST!("NurbsSurface", "Domain_operations", crate::nurbssurface_test::run_nurbssurface_domain_operations);
REGISTER_MINI_TEST!("NurbsSurface", "Corner_points", crate::nurbssurface_test::run_nurbssurface_corner_points);
REGISTER_MINI_TEST!("NurbsSurface", "Swap_coordinates", crate::nurbssurface_test::run_nurbssurface_swap_coordinates);
REGISTER_MINI_TEST!("NurbsSurface", "Get_knots", crate::nurbssurface_test::run_nurbssurface_get_knots);
REGISTER_MINI_TEST!("NurbsSurface", "Make_non_rational", crate::nurbssurface_test::run_nurbssurface_make_non_rational);
REGISTER_MINI_TEST!("NurbsSurface", "Create_clamped_uniform", crate::nurbssurface_test::run_nurbssurface_create_clamped_uniform);
REGISTER_MINI_TEST!("NurbsSurface", "Knot_multiplicity", crate::nurbssurface_test::run_nurbssurface_knot_multiplicity);
