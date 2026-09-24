#![allow(
    clippy::bool_comparison,
    clippy::excessive_precision,
    clippy::needless_range_loop
)]
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_nurbssurface_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let (p, _v, _uv) = s.divide_by_count_points(4, 6);

        let sstr = s.to_string();
        let srepr = s.repr();

        let scopy = s.duplicate();
        let sother = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        MINI_CHECK!(s.is_valid());
        MINI_CHECK!(s.cv_count(0) == 4);
        MINI_CHECK!(s.cv_count(1) == 4);
        MINI_CHECK!(s.cv_count_total() == 16);
        MINI_CHECK!(s.degree(0) == 3);
        MINI_CHECK!(s.degree(1) == 3);
        MINI_CHECK!(s.order(0) == 4);
        MINI_CHECK!(s.order(1) == 4);
        MINI_CHECK!(s.dimension() == 3);
        MINI_CHECK!(!s.is_rational());
        MINI_CHECK!(s.nurbsknot_count(0) == 6);
        MINI_CHECK!(s.nurbsknot_count(1) == 6);
        MINI_CHECK!(s.name == "my_nurbssurface");
        MINI_CHECK!(!s.guid().is_empty());
        MINI_CHECK!(sstr == "NurbsSurface(name=my_nurbssurface, degree=(3,3), cvs=(4,4))");
        MINI_CHECK!(srepr == "NurbsSurface(\n  name=my_nurbssurface,\n  degree=(3,3),\n  cvs=(4,4),\n  rational=false,\n  control_points=[\n    0, 0, 0\n    -1, 0.75, 2\n    -1, 4.25, 2\n    0, 5, 0\n    0.75, -1, 2\n    1.25, 1.25, 4\n    1.25, 3.75, 4\n    0.75, 6, 2\n    4.25, -1, 2\n    3.75, 1.25, 4\n    3.75, 3.75, 4\n    4.25, 6, 2\n    5, 0, 0\n    6, 0.75, 2\n    6, 4.25, 2\n    5, 5, 0\n  ]\n)");
        MINI_CHECK!(scopy.cv_count_total() == s.cv_count_total());
        MINI_CHECK!(scopy.guid() != s.guid());
        MINI_CHECK!(s == sother);
        MINI_CHECK!(!(s != sother));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][0],
            &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][1],
            &Point::new(-0.416666666666667, 0.578703703703704, 0.833333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][2],
            &Point::new(-0.666666666666667, 1.462962962962963, 1.333333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][3],
            &Point::new(-0.750000000000000, 2.500000000000000, 1.500000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][4],
            &Point::new(-0.666666666666667, 3.537037037037037, 1.333333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][5],
            &Point::new(-0.416666666666667, 4.421296296296297, 0.833333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[0][6],
            &Point::new(0.000000000000000, 5.000000000000000, 0.000000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][0],
            &Point::new(0.992187500000000, -0.562500000000000, 1.125000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][1],
            &Point::new(0.881510416666667, 0.333912037037037, 1.958333333333334)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][2],
            &Point::new(0.815104166666667, 1.379629629629630, 2.458333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][3],
            &Point::new(0.792968750000000, 2.500000000000000, 2.625000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][4],
            &Point::new(0.815104166666667, 3.620370370370370, 2.458333333333334)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][5],
            &Point::new(0.881510416666667, 4.666087962962964, 1.958333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[1][6],
            &Point::new(0.992187500000000, 5.562500000000000, 1.125000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][0],
            &Point::new(2.500000000000000, -0.750000000000000, 1.500000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][1],
            &Point::new(2.500000000000000, 0.252314814814815, 2.333333333333334)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][2],
            &Point::new(2.500000000000000, 1.351851851851852, 2.833333333333334)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][3],
            &Point::new(2.500000000000000, 2.500000000000000, 3.000000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][4],
            &Point::new(2.500000000000000, 3.648148148148148, 2.833333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][5],
            &Point::new(2.500000000000000, 4.747685185185186, 2.333333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[2][6],
            &Point::new(2.500000000000000, 5.750000000000000, 1.500000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][0],
            &Point::new(4.007812500000000, -0.562500000000000, 1.125000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][1],
            &Point::new(4.118489583333334, 0.333912037037037, 1.958333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][2],
            &Point::new(4.184895833333334, 1.379629629629630, 2.458333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][3],
            &Point::new(4.207031250000000, 2.500000000000000, 2.625000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][4],
            &Point::new(4.184895833333333, 3.620370370370370, 2.458333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][5],
            &Point::new(4.118489583333333, 4.666087962962964, 1.958333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[3][6],
            &Point::new(4.007812500000000, 5.562500000000000, 1.125000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][0],
            &Point::new(5.000000000000000, 0.000000000000000, 0.000000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][1],
            &Point::new(5.416666666666668, 0.578703703703704, 0.833333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][2],
            &Point::new(5.666666666666668, 1.462962962962963, 1.333333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][3],
            &Point::new(5.750000000000000, 2.500000000000000, 1.500000000000000)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][4],
            &Point::new(5.666666666666666, 3.537037037037037, 1.333333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][5],
            &Point::new(5.416666666666667, 4.421296296296297, 0.833333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &p[4][6],
            &Point::new(5.000000000000000, 5.000000000000000, 0.000000000000000)
        ));
    })
}

pub fn run_nurbssurface_create_from_parameters() -> TestResult {
    MINI_TEST!("Create From Parameters", {
        use crate::NurbsSurface;
        use crate::Point;

        let grid = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(3.0, 0.0, 0.0),
            ],
            vec![
                Point::new(0.0, 1.0, 0.0),
                Point::new(1.0, 1.0, 2.0),
                Point::new(2.0, 1.0, 2.0),
                Point::new(3.0, 1.0, 0.0),
            ],
            vec![
                Point::new(0.0, 2.0, 0.0),
                Point::new(1.0, 2.0, 2.0),
                Point::new(2.0, 2.0, 2.0),
                Point::new(3.0, 2.0, 0.0),
            ],
            vec![
                Point::new(0.0, 3.0, 0.0),
                Point::new(1.0, 3.0, 0.0),
                Point::new(2.0, 3.0, 0.0),
                Point::new(3.0, 3.0, 0.0),
            ],
        ];
        let w: Vec<Vec<f64>> = vec![vec![1.0; 4]; 4];
        let s = NurbsSurface::create_from_parameters(
            &grid,
            &w,
            &[0.0, 1.0],
            &[0.0, 1.0],
            &[4, 4],
            &[4, 4],
            3,
            3,
            false,
            false,
        );

        MINI_CHECK!(s.is_valid());
        MINI_CHECK!(s.degree(0) == 3 && s.degree(1) == 3);
        MINI_CHECK!(s.cv_count(0) == 4 && s.cv_count(1) == 4);
        MINI_CHECK!(!s.is_rational());

        let (u0, u1) = s.domain(0).unwrap();
        let (v0, v1) = s.domain(1).unwrap();

        MINI_CHECK!(u0.abs() < 1e-12 && (u1 - 1.0).abs() < 1e-12);
        MINI_CHECK!(v0.abs() < 1e-12 && (v1 - 1.0).abs() < 1e-12);
        MINI_CHECK!(
            TOLERANCE.is_point_close(&s.point_at(0.0, 0.0).unwrap(), &Point::new(0.0, 0.0, 0.0))
        );
        MINI_CHECK!(
            TOLERANCE.is_point_close(&s.point_at(1.0, 1.0).unwrap(), &Point::new(3.0, 3.0, 0.0))
        );
        MINI_CHECK!(
            TOLERANCE.is_point_close(&s.point_at(0.5, 0.5).unwrap(), &Point::new(1.5, 1.5, 1.125))
        );
        MINI_CHECK!(TOLERANCE.is_point_close(
            &s.point_at(0.37, 0.41).unwrap(),
            &Point::new(1.11, 1.23, 1.01496402)
        ));

        let fr = s.frame_at(0.3, 0.4);

        MINI_CHECK!(TOLERANCE.is_point_close(&fr.origin(), &s.point_at(0.3, 0.4).unwrap()));

        let n = s.normal_at(0.3, 0.4);
        let za = fr.z_axis();

        MINI_CHECK!(
            (za[0] - n[0]).abs() < 1e-9
                && (za[1] - n[1]).abs() < 1e-9
                && (za[2] - n[2]).abs() < 1e-9
        );

        use crate::Line;
        let hits = s.intersections_with_line(&Line::new(1.5, 1.5, -5.0, 1.5, 1.5, 5.0));

        MINI_CHECK!(hits.len() == 1);
        MINI_CHECK!(TOLERANCE.is_point_close(&hits[0], &Point::new(1.5, 1.5, 1.125)));
    })
}

pub fn run_nurbssurface_booleans_queries() -> TestResult {
    MINI_TEST!("Booleans Queries", {
        use crate::tolerance::Tolerance;
        use crate::Plane;
        use crate::Primitives;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 5.0);

        let is_valid = s.is_valid();
        let are_nurbsknots_valid = s.is_valid_nurbsknot_vector(0) && s.is_valid_nurbsknot_vector(1);

        let is_rational = s.is_rational();

        let is_closed = s.is_closed(0) && !s.is_closed(1);

        let is_periodic = s.is_periodic(0) && s.is_periodic(1);

        let mut plane = Plane::xy_plane();
        let is_planar = s.is_planar(Some(&mut plane), Tolerance::ZERO_TOLERANCE);

        let is_point = s.is_singular(0) && s.is_singular(1) && s.is_singular(2) && s.is_singular(3);

        let is_clamped = s.is_clamped(0, 2) && s.is_clamped(1, 2);

        MINI_CHECK!(is_valid);
        MINI_CHECK!(are_nurbsknots_valid);
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
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let dimensions = s.dimension();

        let order_u = s.order(0);
        let order_v = s.order(1);

        let cv_count_u = s.cv_count(0);
        let cv_count_v = s.cv_count(1);
        let cv_count = s.cv_count_total();
        let cv_size = s.cv_size();

        let k_count_0 = s.nurbsknot_count(0);
        let k_count_1 = s.nurbsknot_count(1);

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
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        s.make_rational();

        let cv_slice = s.cv(0, 0).unwrap();

        MINI_CHECK!(cv_slice[2] == 0.0);

        let cv_mut_slice = s.cv_mut(0, 0).unwrap();
        cv_mut_slice[2] = 10.0;

        MINI_CHECK!(s.cv(0, 0).unwrap()[2] == 10.0);

        let cv = s.get_cv(0, 0).unwrap();

        MINI_CHECK!(cv == Point::new(0.0, 0.0, 10.0));

        let (x, y, z, w) = s.get_cv_4d(0, 0).unwrap();

        MINI_CHECK!(x == 0.0 && y == 0.0 && z == 10.0 && w == 1.0);

        s.set_cv(0, 0, &Point::new(0.0, 0.0, 5.0));

        MINI_CHECK!(s.get_cv(0, 0).unwrap() == Point::new(0.0, 0.0, 5.0));

        s.set_cv_4d(0, 0, 0.0, 0.0, 4.0, 0.5);

        MINI_CHECK!(s.get_cv(0, 0).unwrap() == Point::new(0.0, 0.0, 8.0));
        MINI_CHECK!(s.cv(0, 0).unwrap()[2] == 4.0);
        MINI_CHECK!(s.weight(0, 0) == 0.5);

        let _w = s.weight(0, 0);
        s.set_weight(0, 0, 1.0);

        MINI_CHECK!(s.weight(0, 0) == 1.0);
    })
}

pub fn run_nurbssurface_nurbsknot_access() -> TestResult {
    MINI_TEST!("NurbsKnot Access", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let nurbsknots_u = s.get_nurbsknots(0);

        for i in 0..s.nurbsknot_count(0) {
            let nurbsknot = s.nurbsknot(0, i).unwrap();

            MINI_CHECK!(nurbsknot == nurbsknots_u[i]);
        }

        let nurbsknots_v = s.get_nurbsknots(1);

        for i in 0..s.nurbsknot_count(1) {
            let nurbsknot = s.nurbsknot(1, i).unwrap();

            MINI_CHECK!(nurbsknot == nurbsknots_v[i]);
        }

        let is_set = s.set_nurbsknot(0, 2, 0.5);

        MINI_CHECK!(is_set);
        MINI_CHECK!(s.nurbsknot(0, 2).unwrap() == 0.5);

        let is_set = s.set_nurbsknot(0, 2, 0.0);

        MINI_CHECK!(is_set);

        let mult_u_start = s.nurbsknot_multiplicity(0, 0);
        let mult_v_start = s.nurbsknot_multiplicity(1, 0);

        MINI_CHECK!(mult_u_start == 3);
        MINI_CHECK!(mult_v_start == 3);

        s.insert_nurbsknot(0, 0.1, 2);

        MINI_CHECK!(s.nurbsknot_count(0) == 8);
        MINI_CHECK!(s.nurbsknot(0, 3).unwrap() == 0.1);
        MINI_CHECK!(s.nurbsknot_multiplicity(0, 3) == 2);
    })
}

pub fn run_nurbssurface_domain() -> TestResult {
    MINI_TEST!("Domain", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let domain_u = s.domain(0).unwrap();
        let domain_v = s.domain(1).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(domain_u.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(domain_u.1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(domain_v.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(domain_v.1, 1.0));

        let is_set_u = s.set_domain(0, -1.1, 2.3);
        let is_set_v = s.set_domain(1, -5.1, 1.3);

        MINI_CHECK!(is_set_u && TOLERANCE.is_close(s.domain(1).unwrap().0, -5.1));
        MINI_CHECK!(is_set_v && TOLERANCE.is_close(s.domain(1).unwrap().1, 1.3));

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
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let (division_points, vectors, uvs0) = s.divide_by_count_points(3, 3);

        let (planes, _uvs1) = s.divide_by_count_planes(3, 3);

        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[0][0], &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[0][1],
            &Point::new(-0.666666666666667, 1.46296296296296, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[0][2],
            &Point::new(-0.666666666666667, 3.53703703703704, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[0][3], &Point::new(0.0, 5.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[1][0],
            &Point::new(1.46296296296296, -0.666666666666667, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[1][1],
            &Point::new(1.3641975308642, 1.3641975308642, 2.66666666666667)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[1][2],
            &Point::new(1.3641975308642, 3.6358024691358, 2.66666666666667)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[1][3],
            &Point::new(1.46296296296296, 5.66666666666667, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[2][0],
            &Point::new(3.53703703703704, -0.666666666666667, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[2][1],
            &Point::new(3.6358024691358, 1.3641975308642, 2.66666666666667)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[2][2],
            &Point::new(3.6358024691358, 3.6358024691358, 2.66666666666667)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[2][3],
            &Point::new(3.53703703703704, 5.66666666666667, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[3][0], &Point::new(5.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[3][1],
            &Point::new(5.66666666666667, 1.46296296296296, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(
            &division_points[3][2],
            &Point::new(5.66666666666667, 3.53703703703704, 1.33333333333333)
        ));

        MINI_CHECK!(TOLERANCE.is_point_close(&division_points[3][3], &Point::new(5.0, 5.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[0][0],
            &Vector::new(-0.704360725060499, -0.704360725060499, -0.0880450906325624)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[0][1],
            &Vector::new(-0.722897836195991, -0.327787263130091, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[0][2],
            &Vector::new(-0.722897836195991, 0.327787263130091, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[0][3],
            &Vector::new(-0.704360725060499, 0.704360725060499, -0.0880450906325624)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[1][0],
            &Vector::new(-0.327787263130091, -0.722897836195991, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[1][1],
            &Vector::new(-0.280457757277237, -0.280457757277237, 0.917979788865771)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[1][2],
            &Vector::new(-0.280457757277237, 0.280457757277237, 0.917979788865771)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[1][3],
            &Vector::new(-0.327787263130091, 0.722897836195991, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[2][0],
            &Vector::new(0.327787263130091, -0.722897836195991, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[2][1],
            &Vector::new(0.280457757277237, -0.280457757277237, 0.917979788865771)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[2][2],
            &Vector::new(0.280457757277237, 0.280457757277237, 0.917979788865771)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[2][3],
            &Vector::new(0.327787263130091, 0.722897836195991, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[3][0],
            &Vector::new(0.704360725060499, -0.704360725060499, -0.0880450906325624)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[3][1],
            &Vector::new(0.722897836195991, -0.327787263130091, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[3][2],
            &Vector::new(0.722897836195991, 0.327787263130091, 0.608255068661856)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &vectors[3][3],
            &Vector::new(0.704360725060499, 0.704360725060499, -0.0880450906325624)
        ));

        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][0].0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][1].0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][2].0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][3].0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[0][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][0].0, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][1].0, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][2].0, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][3].0, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[1][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][0].0, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][1].0, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][2].0, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][3].0, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[2][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][0].0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][0].1, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][1].0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][1].1, 0.333333333333333));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][2].0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][2].1, 0.666666666666667));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][3].0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(uvs0[3][3].1, 1.0));
        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][0].x_axis(),
            &Vector::new(0.317999364001908, -0.423999152002544, 0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][1].x_axis(),
            &Vector::new(0.657483781160109, -0.0556600026378928, 0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][2].x_axis(),
            &Vector::new(0.657483781160109, 0.055660002637893, 0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][3].x_axis(),
            &Vector::new(0.317999364001908, 0.423999152002544, 0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][0].x_axis(),
            &Vector::new(0.93542594448836, -0.158100159631836, 0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][1].x_axis(),
            &Vector::new(0.957938608304167, -0.0211991946512679, 0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][2].x_axis(),
            &Vector::new(0.957938608304167, 0.0211991946512677, 0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][3].x_axis(),
            &Vector::new(0.93542594448836, 0.158100159631835, 0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][0].x_axis(),
            &Vector::new(0.93542594448836, 0.158100159631835, -0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][1].x_axis(),
            &Vector::new(0.957938608304167, 0.0211991946512679, -0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][2].x_axis(),
            &Vector::new(0.957938608304167, -0.021199194651268, -0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][3].x_axis(),
            &Vector::new(0.93542594448836, -0.158100159631836, -0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][0].x_axis(),
            &Vector::new(0.317999364001908, 0.423999152002544, -0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][1].x_axis(),
            &Vector::new(0.657483781160109, 0.0556600026378928, -0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][2].x_axis(),
            &Vector::new(0.657483781160109, -0.0556600026378928, -0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][3].x_axis(),
            &Vector::new(0.317999364001908, -0.423999152002544, -0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][0].y_axis(),
            &Vector::new(-0.423999152002544, 0.317999364001908, 0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][1].y_axis(),
            &Vector::new(-0.158100159631836, 0.93542594448836, 0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][2].y_axis(),
            &Vector::new(0.158100159631835, 0.93542594448836, -0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[0][3].y_axis(),
            &Vector::new(0.423999152002544, 0.317999364001908, -0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][0].y_axis(),
            &Vector::new(-0.0556600026378928, 0.657483781160109, 0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][1].y_axis(),
            &Vector::new(-0.0211991946512679, 0.957938608304167, 0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][2].y_axis(),
            &Vector::new(0.0211991946512679, 0.957938608304167, -0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[1][3].y_axis(),
            &Vector::new(0.0556600026378928, 0.657483781160109, -0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][0].y_axis(),
            &Vector::new(0.0556600026378928, 0.657483781160109, 0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][1].y_axis(),
            &Vector::new(0.0211991946512678, 0.957938608304167, 0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][2].y_axis(),
            &Vector::new(-0.0211991946512678, 0.957938608304167, -0.286189127792116)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[2][3].y_axis(),
            &Vector::new(-0.0556600026378928, 0.657483781160109, -0.751410035611553)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][0].y_axis(),
            &Vector::new(0.423999152002544, 0.317999364001908, 0.847998304005088)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][1].y_axis(),
            &Vector::new(0.158100159631835, 0.93542594448836, 0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][2].y_axis(),
            &Vector::new(-0.158100159631836, 0.93542594448836, -0.316200319263671)
        ));

        MINI_CHECK!(TOLERANCE.is_vector_close(
            &planes[3][3].y_axis(),
            &Vector::new(-0.423999152002544, 0.317999364001908, -0.847998304005088)
        ));
    })
}

pub fn run_nurbssurface_evaluation() -> TestResult {
    MINI_TEST!("Evaluation", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Vector;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let u = 0.5;
        let v = 0.5;

        let p1 = s.point_at(u, v).unwrap();

        MINI_CHECK!(TOLERANCE.is_point_close(&p1, &Point::new(2.5, 2.5, 3.0)));

        let n1 = s.normal_at(u, v);

        MINI_CHECK!(TOLERANCE.is_vector_close(&n1, &Vector::new(0.0, 0.0, 1.0)));

        let derivs = s.evaluate(u, v, 1);

        MINI_CHECK!(TOLERANCE.is_vector_close(&derivs[0], &Vector::new(2.5, 2.5, 3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&derivs[1], &Vector::new(0.0, 6.9375, 0.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&derivs[2], &Vector::new(6.9375, 0.0, 0.0)));

        let p_corner = s.point_at_corner(1, 1).unwrap();

        MINI_CHECK!(TOLERANCE.is_point_close(&p_corner, &Point::new(5.0, 5.0, 0.0)));

        let iso_u = s.iso_curve(0, v).unwrap();
        let iso_v = s.iso_curve(1, u).unwrap();

        MINI_CHECK!(TOLERANCE.is_point_close(&iso_u.point_at(0.5), &Point::new(2.5, 2.5, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&iso_v.point_at(0.5), &Point::new(2.5, 2.5, 3.0)));
    })
}

pub fn run_nurbssurface_modification() -> TestResult {
    MINI_TEST!("Modification", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let mut s_rev = s.duplicate();
        s_rev.reverse(0);

        MINI_CHECK!(s_rev.point_at_corner(0, 0).unwrap() == s.point_at_corner(1, 0).unwrap());
        MINI_CHECK!(s_rev.normal_at(0.5, 0.5) == s.normal_at(0.5, 0.5) * -1.0);

        let mut s_tr = s.duplicate();
        s_tr.transpose();

        MINI_CHECK!(s.point_at(0.0, 0.5).unwrap() == s_tr.point_at(0.5, 0.0).unwrap());

        let mut s_swap = s.duplicate();
        s_swap.swap_coordinates(0, 2);

        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap()[0] == s_swap.point_at(0.5, 0.5).unwrap()[2]);
        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap()[2] == s_swap.point_at(0.5, 0.5).unwrap()[0]);

        let mut s_trim = s.duplicate();
        s_trim.trim(0, (0.25, 0.75));

        MINI_CHECK!(TOLERANCE.is_close(s_trim.domain(0).unwrap().0, 0.25));
        MINI_CHECK!(TOLERANCE.is_close(s_trim.domain(0).unwrap().1, 0.75));
        MINI_CHECK!(TOLERANCE.is_point_close(
            &s.point_at(0.25, 0.5).unwrap(),
            &s_trim.point_at(0.25, 0.5).unwrap()
        ));

        let (west, east) = s.split(0, 0.5);
        let west = west.unwrap();
        let east = east.unwrap();
        let (ww, we) = west.split(
            1,
            (west.domain(1).unwrap().0 + west.domain(1).unwrap().1) / 2.0,
        );
        let (ew, ee) = east.split(
            1,
            (east.domain(1).unwrap().0 + east.domain(1).unwrap().1) / 2.0,
        );
        let ww = ww.unwrap();
        let we = we.unwrap();
        let ew = ew.unwrap();
        let ee = ee.unwrap();
        let center = s.point_at(0.5, 0.5).unwrap();

        MINI_CHECK!(TOLERANCE.is_point_close(&ww.point_at_corner(1, 1).unwrap(), &center));
        MINI_CHECK!(TOLERANCE.is_point_close(&we.point_at_corner(1, 0).unwrap(), &center));
        MINI_CHECK!(TOLERANCE.is_point_close(&ew.point_at_corner(0, 1).unwrap(), &center));
        MINI_CHECK!(TOLERANCE.is_point_close(&ee.point_at_corner(0, 0).unwrap(), &center));

        let mut s_rat = s.duplicate();
        s_rat.make_rational();
        s_rat.set_weight(2, 2, 3.0);

        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap() != s_rat.point_at(0.5, 0.5).unwrap());

        s_rat.make_non_rational();

        MINI_CHECK!(s.point_at(0.5, 0.5).unwrap() == s_rat.point_at(0.5, 0.5).unwrap());

        let mut s_deg = s.duplicate();
        s_deg.increase_degree(0, 6);
        s_deg.increase_degree(1, 6);

        MINI_CHECK!(s.cv_count(0) == 4 && s.cv_count(1) == 4);
        MINI_CHECK!(s_deg.cv_count(0) == 7 && s_deg.cv_count(1) == 7);
    })
}

pub fn run_nurbssurface_transformations() -> TestResult {
    MINI_TEST!("Transformations", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Xform;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        let mut surface1 = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        let surface1_xf = Xform::translation(0.0, 0.0, 1.0);
        surface1.transform(&surface1_xf);

        MINI_CHECK!(surface1.cv(0, 0).unwrap()[2] == 1.0);

        let mut surface2 = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        let x = Xform::translation(0.0, 0.0, 1.0);
        surface2.transform(&x);

        MINI_CHECK!(surface2.cv(0, 0).unwrap()[2] == 1.0);

        let surface3 = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        let surface3_xf = Xform::translation(0.0, 0.0, 10.0);
        let surface3_transformed = surface3.transformed(&surface3_xf);

        MINI_CHECK!(surface3_transformed.cv(0, 0).unwrap()[2] == 10.0);

        let surface4 = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        let x = Xform::translation(0.0, 0.0, 10.0);
        let surface4_transformed = surface4.transformed(&x);

        MINI_CHECK!(surface4_transformed.cv(0, 0).unwrap()[2] == 10.0);
    })
}

pub fn run_nurbssurface_meshing() -> TestResult {
    MINI_TEST!("Meshing", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 3.0);
        let mesh_sphere = sphere.mesh();
        let mesh_sphere_adaptive = sphere.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_sphere.is_valid());
        MINI_CHECK!(mesh_sphere_adaptive.is_valid());

        let cone = Primitives::cone_surface(0.0, 12.0, 0.0, 2.0, 6.0);
        let mesh_cone = cone.mesh();
        let mesh_cone_adaptive = cone.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_cone.is_valid());
        MINI_CHECK!(mesh_cone_adaptive.is_valid());

        let torus = Primitives::torus_surface(0.0, 24.0, 0.0, 4.0, 1.5);
        let mesh_torus = torus.mesh();
        let mesh_torus_adaptive = torus.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_torus.is_valid());
        MINI_CHECK!(mesh_torus_adaptive.is_valid());

        let loft = Primitives::create_loft(
            &[
                Primitives::circle(0.0, 38.0, 0.0, 2.0),
                Primitives::circle(0.0, 38.0, 2.0, 1.0),
                Primitives::circle(0.0, 38.0, 4.0, 1.5),
                Primitives::circle(0.0, 38.0, 6.0, 0.8),
            ],
            3,
        );
        let mesh_loft = loft.mesh();
        let mesh_loft_adaptive = loft.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_loft.is_valid());
        MINI_CHECK!(mesh_loft_adaptive.is_valid());

        let ext_dir = Vector::new(0.0, 0.0, 5.0);
        let cylinder =
            Primitives::create_extrusion(&Primitives::circle(0.0, 52.0, 0.0, 3.0), &ext_dir);
        let mesh_cylinder = cylinder.mesh();
        let mesh_cylinder_adaptive = cylinder.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_cylinder.is_valid());
        MINI_CHECK!(mesh_cylinder_adaptive.is_valid());

        let ra = NurbsCurve::create(
            false,
            1,
            &[Point::new(0.0, 64.0, 0.0), Point::new(5.0, 64.0, 5.0)],
        );
        let rb = NurbsCurve::create(
            false,
            1,
            &[Point::new(0.0, 69.0, 5.0), Point::new(5.0, 69.0, 0.0)],
        );
        let hypar = Primitives::create_ruled(&ra, &rb);
        let mesh_hypar = hypar.mesh();
        let mesh_hypar_adaptive = hypar.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_hypar.is_valid());
        MINI_CHECK!(mesh_hypar_adaptive.is_valid());

        let profile = Primitives::circle(0.0, 0.0, 0.0, 1.0);
        let rail = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(0.0, 76.0, 0.0),
                Point::new(0.0, 81.0, 0.0),
                Point::new(2.0, 85.0, 0.0),
            ],
        );
        let sweep1 = Primitives::create_sweep1(&rail, &profile);
        let mesh_sweep1 = sweep1.mesh();
        let mesh_sweep1_adaptive = sweep1.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_sweep1.is_valid());
        MINI_CHECK!(mesh_sweep1_adaptive.is_valid());

        let r1 = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(0.0, 89.0, 0.0),
                Point::new(1.0, 93.0, 0.0),
                Point::new(2.0, 94.0, 0.0),
            ],
        );
        let r2 = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(4.0, 89.0, 0.0),
                Point::new(4.0, 93.0, 0.0),
                Point::new(3.0, 94.0, 0.0),
            ],
        );
        let sh1 = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(0.0, 89.0, 0.0),
                Point::new(2.0, 89.0, 2.0),
                Point::new(4.0, 89.0, 0.0),
            ],
        );
        let sh2 = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(2.0, 94.0, 0.0),
                Point::new(2.5, 94.0, 1.5),
                Point::new(3.0, 94.0, 0.0),
            ],
        );
        let sweep2 = Primitives::create_sweep2(&r1, &r2, &[sh1, sh2]);
        let mesh_sweep2 = sweep2.mesh();
        let mesh_sweep2_adaptive = sweep2.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_sweep2.is_valid());
        MINI_CHECK!(mesh_sweep2_adaptive.is_valid());

        let south = NurbsCurve::create(
            false,
            3,
            &[
                Point::new(1.0, 104.0, 0.0),
                Point::new(1.0, 106.0, 3.0),
                Point::new(1.0, 109.0, 3.0),
                Point::new(1.0, 111.0, 0.0),
            ],
        );
        let west = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(10.0, 104.0, 0.0),
                Point::new(5.5, 104.0, 3.5),
                Point::new(1.0, 104.0, 0.0),
            ],
        );
        let north = NurbsCurve::create(
            false,
            3,
            &[
                Point::new(10.0, 104.0, 0.0),
                Point::new(10.0, 106.0, 3.0),
                Point::new(10.0, 109.0, 3.0),
                Point::new(10.0, 111.0, 0.0),
            ],
        );
        let east = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(10.0, 111.0, 0.0),
                Point::new(5.5, 111.0, 3.5),
                Point::new(1.0, 111.0, 0.0),
            ],
        );
        let arched = Primitives::create_edge(&south, &west, &north, &east);
        let mesh_arched = arched.mesh();
        let mesh_arched_adaptive = arched.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_arched.is_valid());
        MINI_CHECK!(mesh_arched_adaptive.is_valid());

        let wave = Primitives::wave_surface(5.0, 1.5);
        let mesh_wave = wave.mesh();
        let mesh_wave_adaptive = wave.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_wave.is_valid());
        MINI_CHECK!(mesh_wave_adaptive.is_valid());

        let planar = NurbsCurve::create(
            false,
            1,
            &[
                Point::new(0.0, 132.0, 0.0),
                Point::new(6.0, 132.0, 0.0),
                Point::new(6.0, 136.0, 0.0),
                Point::new(0.0, 136.0, 0.0),
                Point::new(0.0, 132.0, 0.0),
            ],
        );
        let pln = Primitives::create_planar(&planar);
        let mesh_planar = pln.mesh();
        let mesh_planar_adaptive = pln.mesh_adaptive(45.0, 0.0, 0.0, 0.0);

        MINI_CHECK!(mesh_planar.is_valid());
        MINI_CHECK!(mesh_planar_adaptive.is_valid());
    })
}

pub fn run_nurbssurface_split_by_plane() -> TestResult {
    MINI_TEST!("Split By Plane", {
        use crate::Plane;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 4.0);
        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 2.0), Vector::new(0.3, 0.0, 1.0), None);

        let parts = cyl.split_by_plane(&plane, 0.0);

        MINI_CHECK!(parts.len() == 2);

        for ts in &parts {
            let m = ts.mesh_q(20.0, 0.005);

            MINI_CHECK!(ts.is_trimmed());
            MINI_CHECK!(m.number_of_faces() > 0);
        }

        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let plane2 =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.3), Vector::new(0.0, 0.0, 1.0), None);

        let caps = sphere.split_by_plane(&plane2, 0.0);

        MINI_CHECK!(caps.len() == 2);
    })
}

pub fn run_nurbssurface_split_by_curves() -> TestResult {
    MINI_TEST!("Split By Curves", {
        use crate::nurbsknot::CurveInterpStyle;
        use crate::nurbsknot::CurveNurbsKnotStyle;
        use crate::Closest;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Primitives;

        let wave = Primitives::wave_surface(10.0, 1.0);
        let mut lift_pts = Vec::new();

        for i in 0..21 {
            let x = 10.0 * i as f64 / 20.0;
            let y = 5.0 + 2.0 * x.sin();
            let (u, v, _d) =
                Closest::surface_point(&wave, &Point::new(x, y, 0.0), 0.0, 0.0, 0.0, 0.0);
            lift_pts.push(wave.point_at(u, v).unwrap());
        }

        let crv = NurbsCurve::create_interpolated(
            &lift_pts,
            CurveNurbsKnotStyle::Chord,
            CurveInterpStyle::Rhino,
        );

        let parts = wave.split_by_curves(&[crv], 0.0);

        MINI_CHECK!(parts.len() == 2);
        MINI_CHECK!(parts[0].is_trimmed());
        MINI_CHECK!(parts[1].is_trimmed());

        let off = NurbsCurve::create(
            false,
            1,
            &[Point::new(50.0, 50.0, 50.0), Point::new(60.0, 60.0, 60.0)],
        );

        let whole = wave.split_by_curves(&[off], 0.0);

        MINI_CHECK!(whole.len() == 1);
    })
}

pub fn run_nurbssurface_split_by_line() -> TestResult {
    MINI_TEST!("Split By Line", {
        use crate::Line;
        use crate::Point;
        use crate::Primitives;

        let wave = Primitives::wave_surface(10.0, 1.0);
        let line = Line::from_points(&Point::new(-1.0, 5.0, 0.0), &Point::new(11.0, 5.0, 0.0));

        let parts = wave.split_by_line(&line, 0.0);

        MINI_CHECK!(parts.len() == 2);
        MINI_CHECK!(parts[0].is_trimmed());
        MINI_CHECK!(parts[1].is_trimmed());
    })
}

pub fn run_nurbssurface_split_by_surface() -> TestResult {
    MINI_TEST!("Split By Surface", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Primitives;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
        let flat = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(-3.0, -3.0, 0.0),
                Point::new(-3.0, 3.0, 0.0),
                Point::new(3.0, -3.0, 0.0),
                Point::new(3.0, 3.0, 0.0),
            ],
        )
        .unwrap();

        let parts = cyl.split_by_surface(&flat, 0.0);

        MINI_CHECK!(parts.len() == 2);

        for ts in &parts {
            let m = ts.mesh_q(20.0, 0.005);

            MINI_CHECK!(ts.is_trimmed());
            MINI_CHECK!(m.number_of_faces() > 0);
        }
    })
}

pub fn run_nurbssurface_split_by_brep() -> TestResult {
    MINI_TEST!("Split By Brep", {
        use crate::BRep;
        use crate::NurbsSurface;
        use crate::Point;

        let flat = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(-3.0, -3.0, 0.0),
                Point::new(-3.0, 3.0, 0.0),
                Point::new(3.0, -3.0, 0.0),
                Point::new(3.0, 3.0, 0.0),
            ],
        )
        .unwrap();
        let cutter = BRep::create_box(2.0, 2.0, 2.0);

        let parts = flat.split_by_brep(&cutter, 0.0);

        MINI_CHECK!(parts.len() == 2);

        for ts in &parts {
            let m = ts.mesh_q(20.0, 0.005);

            MINI_CHECK!(ts.is_trimmed());
            MINI_CHECK!(m.number_of_faces() > 0);
        }
    })
}

pub fn run_nurbssurface_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::NurbsSurface;
        use crate::Point;
        use std::path::PathBuf;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];
        let surface = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        let json = surface.jsondump().unwrap();
        let loaded_json = NurbsSurface::jsonload(&json).unwrap();

        let json_string = surface.file_json_dumps();
        let loaded_json_string = NurbsSurface::file_json_loads(&json_string);

        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_nurbssurface.json");
        surface.file_json_dump(filename.to_str().unwrap()).unwrap();
        let loaded_from_file = NurbsSurface::file_json_load(filename.to_str().unwrap()).unwrap();

        MINI_CHECK!(loaded_json == surface);
        MINI_CHECK!(loaded_json_string == surface);
        MINI_CHECK!(loaded_from_file == surface);
    })
}

pub fn run_nurbssurface_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::NurbsSurface;
        use crate::Point;
        use std::path::PathBuf;

        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];
        let surface = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();
        let guid = surface.guid().to_string();
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_nurbssurface.bin");
        surface.pb_dump(filename.to_str().unwrap());

        let loaded_proto_string = NurbsSurface::pb_loads(&surface.pb_dumps()).unwrap();
        let loaded = NurbsSurface::pb_load(filename.to_str().unwrap());
        let converted = NurbsSurface::from_proto(surface.to_proto()).unwrap();

        MINI_CHECK!(loaded_proto_string == surface);
        MINI_CHECK!(loaded == surface);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(converted == surface);
        MINI_CHECK!(converted.guid() == guid);
    })
}

pub fn run_nurbssurface_closest_point() -> TestResult {
    MINI_TEST!("Closest Point", {
        use crate::Point;
        use crate::Primitives;

        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let cp = sphere.closest_point(&Point::new(5.0, 0.0, 0.0));

        MINI_CHECK!((cp[0] - 2.0).abs() < 1e-4 && cp[1].abs() < 1e-4 && cp[2].abs() < 1e-4);
    })
}

pub fn run_nurbssurface_curvature() -> TestResult {
    MINI_TEST!("Curvature", {
        use crate::Primitives;

        let r = 2.0;
        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, r);
        let (u0, u1) = sphere.domain(0).unwrap();
        let (v0, v1) = sphere.domain(1).unwrap();
        let um = u0 + 0.37 * (u1 - u0);
        let vm = v0 + 0.41 * (v1 - v0);

        MINI_CHECK!((sphere.gaussian_curvature(um, vm) - 1.0 / (r * r)).abs() < 1e-3);
        MINI_CHECK!((sphere.mean_curvature(um, vm).abs() - 1.0 / r).abs() < 1e-3);
    })
}

REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Constructor",
    crate::nurbssurface_test::run_nurbssurface_constructor
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Create From Parameters",
    crate::nurbssurface_test::run_nurbssurface_create_from_parameters
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Booleans Queries",
    crate::nurbssurface_test::run_nurbssurface_booleans_queries
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Attributes",
    crate::nurbssurface_test::run_nurbssurface_attributes
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Control Vertices Access",
    crate::nurbssurface_test::run_nurbssurface_control_vertices_access
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "NurbsKnot Access",
    crate::nurbssurface_test::run_nurbssurface_nurbsknot_access
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Domain",
    crate::nurbssurface_test::run_nurbssurface_domain
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Division",
    crate::nurbssurface_test::run_nurbssurface_division
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Evaluation",
    crate::nurbssurface_test::run_nurbssurface_evaluation
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Modification",
    crate::nurbssurface_test::run_nurbssurface_modification
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Transformations",
    crate::nurbssurface_test::run_nurbssurface_transformations
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Meshing",
    crate::nurbssurface_test::run_nurbssurface_meshing
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Split By Plane",
    crate::nurbssurface_test::run_nurbssurface_split_by_plane
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Split By Curves",
    crate::nurbssurface_test::run_nurbssurface_split_by_curves
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Split By Line",
    crate::nurbssurface_test::run_nurbssurface_split_by_line
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Split By Surface",
    crate::nurbssurface_test::run_nurbssurface_split_by_surface
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Split By Brep",
    crate::nurbssurface_test::run_nurbssurface_split_by_brep
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Json Roundtrip",
    crate::nurbssurface_test::run_nurbssurface_json_roundtrip
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Protobuf Roundtrip",
    crate::nurbssurface_test::run_nurbssurface_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Closest Point",
    crate::nurbssurface_test::run_nurbssurface_closest_point
);
REGISTER_MINI_TEST!(
    "NurbsSurface",
    "Curvature",
    crate::nurbssurface_test::run_nurbssurface_curvature
);
