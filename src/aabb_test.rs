use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_aabb_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Point;
        use crate::AABB;

        let a = AABB::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
        let empty = AABB::default();
        let inside = AABB::new(0.5, 0.0, 0.0, 0.5, 0.5, 0.5);
        let outside = AABB::new(10.0, 0.0, 0.0, 0.5, 0.5, 0.5);

        let astr = a.str();
        let arepr = a.repr();
        let corners = a.get_corners();
        let edges = a.get_edges();

        let b = AABB::new(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let mut grown = AABB::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
        grown.union_with(&b);

        let c = AABB::merge(
            &AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0),
            &AABB::new(4.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        );

        MINI_CHECK!(empty == AABB::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        MINI_CHECK!(a == AABB::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0));
        MINI_CHECK!(a != empty);
        MINI_CHECK!(astr == "0.000000, 0.000000, 0.000000, 1.000000, 2.000000, 3.000000");
        MINI_CHECK!(arepr == "AABB(0.000000, 0.000000, 0.000000, 1.000000, 2.000000, 3.000000)");
        MINI_CHECK!(TOLERANCE.is_close(a.area(), 88.0));
        MINI_CHECK!(a.center() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(a.diagonal(), 2.0 * 14.0_f64.sqrt()));
        MINI_CHECK!(a.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(a.volume(), 48.0));
        MINI_CHECK!(a.closest_point(&Point::new(0.0, 0.0, 0.0)) == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a.closest_point(&Point::new(10.0, 0.0, 0.0)) == Point::new(1.0, 0.0, 0.0));
        MINI_CHECK!(a.contains(&Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(!a.contains(&Point::new(10.0, 0.0, 0.0)));
        MINI_CHECK!(a.corner(false, false, false) == Point::new(-1.0, -2.0, -3.0));
        MINI_CHECK!(a.corner(true, true, true) == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(corners.len() == 8);
        MINI_CHECK!(edges.len() == 12);
        MINI_CHECK!(a.point_at(1.0, 0.0, 0.0) == Point::new(1.0, 0.0, 0.0));
        MINI_CHECK!(a.point_at(0.0, 0.0, 0.0) == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a.intersects(&inside));
        MINI_CHECK!(!a.intersects(&outside));
        MINI_CHECK!(grown.min_point() == Point::new(-1.0, -2.0, -3.0));
        MINI_CHECK!(grown.max_point() == Point::new(6.0, 2.0, 3.0));
        MINI_CHECK!(c.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(c.max_point() == Point::new(5.0, 1.0, 1.0));
    })
}

pub fn run_aabb_empty() -> TestResult {
    MINI_TEST!("Empty", {
        use crate::Point;
        use crate::AABB;

        let mut a = AABB::empty();
        let empty_valid = a.is_valid();
        let empty_diagonal = a.diagonal();

        a.union_with(&AABB::empty());
        let merged_valid = a.is_valid();

        a.union_with_point(1.0, 2.0, 3.0);
        let point_valid = a.is_valid();
        let point_min = a.min_point();
        let point_max = a.max_point();

        a.union_with_point(-1.0, 0.0, 5.0);
        let grown_min = a.min_point();
        let grown_max = a.max_point();

        a.union_with(&AABB::empty());
        let b = AABB::merge(&AABB::empty(), &AABB::new(4.0, 0.0, 0.0, 1.0, 1.0, 1.0));

        MINI_CHECK!(!empty_valid);
        MINI_CHECK!(TOLERANCE.is_close(empty_diagonal, 0.0));
        MINI_CHECK!(!merged_valid);
        MINI_CHECK!(point_valid);
        MINI_CHECK!(point_min == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(point_max == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(grown_min == Point::new(-1.0, 0.0, 3.0));
        MINI_CHECK!(grown_max == Point::new(1.0, 2.0, 5.0));
        MINI_CHECK!(a.max_point() == Point::new(1.0, 2.0, 5.0));
        MINI_CHECK!(b.min_point() == Point::new(3.0, -1.0, -1.0));
        MINI_CHECK!(b.max_point() == Point::new(5.0, 1.0, 1.0));
    })
}

pub fn run_aabb_transform() -> TestResult {
    MINI_TEST!("Transform", {
        use crate::Point;
        use crate::Xform;
        use crate::AABB;

        let mut a = AABB::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);
        let moved = a.transformed(&Xform::translation(1.0, 2.0, 3.0));
        let turned = a.transformed(&Xform::rotation_z(90.0, true));
        let empty = AABB::empty().transformed(&Xform::translation(1.0, 0.0, 0.0));
        a.transform(&Xform::scale_xyz(2.0, 2.0, 2.0));

        MINI_CHECK!(moved.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(moved.max_point() == Point::new(2.0, 4.0, 6.0));
        MINI_CHECK!(turned.min_point() == Point::new(-2.0, -1.0, -3.0));
        MINI_CHECK!(turned.max_point() == Point::new(2.0, 1.0, 3.0));
        MINI_CHECK!(a.max_point() == Point::new(2.0, 4.0, 6.0));
        MINI_CHECK!(!empty.is_valid());
    })
}

pub fn run_aabb_from_geometry() -> TestResult {
    MINI_TEST!("From Geometry", {
        use crate::Color;
        use crate::Line;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::PointCloud;
        use crate::Polyline;
        use crate::Primitives;
        use crate::Vector;
        use crate::AABB;

        let a_pt = AABB::from_point(&Point::new(1.0, 2.0, 3.0), 0.5);

        let a_pts = AABB::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(3.0, 4.0, 5.0)], 0.0);

        let a_negative = AABB::from_points(
            &[Point::new(-5.0, -4.0, -3.0), Point::new(-1.0, -2.0, -1.0)],
            0.0,
        );

        let ln = Line::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
        let a_line = AABB::from_line(&ln, 1.0);

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
        ]);
        let a_pl = AABB::from_polyline(&pl, 0.0);

        let cube = Primitives::cube(2.0);
        let a_mesh = AABB::from_mesh(&cube, 0.0);

        let pc = PointCloud::new(
            vec![Point::new(0.0, 0.0, 0.0), Point::new(4.0, 2.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
            ],
        );
        let a_pc = AABB::from_pointcloud(&pc, 0.0);

        let curve = NurbsCurve::create(
            false,
            2,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(3.0, 0.0, 0.0),
            ],
        );
        let a_nc = AABB::from_nurbscurve(&curve, 0.5, false);

        let surf = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(0.0, 2.0, 0.0),
                Point::new(2.0, 2.0, 2.0),
            ],
        )
        .unwrap();
        let a_ns = AABB::from_nurbssurface(&surf, 0.0);

        MINI_CHECK!(a_pt.center() == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(a_pt.hx, 0.5));
        MINI_CHECK!(a_pts.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pts.max_point() == Point::new(3.0, 4.0, 5.0));
        MINI_CHECK!(a_negative.min_point() == Point::new(-5.0, -4.0, -3.0));
        MINI_CHECK!(a_negative.max_point() == Point::new(-1.0, -2.0, -1.0));
        MINI_CHECK!(a_line.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(a_line.max_point() == Point::new(5.0, 1.0, 1.0));
        MINI_CHECK!(a_pl.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pl.max_point() == Point::new(2.0, 2.0, 0.0));
        MINI_CHECK!(a_mesh.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(a_mesh.max_point() == Point::new(1.0, 1.0, 1.0));
        MINI_CHECK!(a_pc.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pc.max_point() == Point::new(4.0, 2.0, 6.0));
        MINI_CHECK!(a_nc.is_valid());
        MINI_CHECK!(a_nc.contains(&Point::new(1.5, 0.0, 0.0)));
        MINI_CHECK!(a_ns.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(a_ns.volume(), 8.0));
    })
}

REGISTER_MINI_TEST!(
    "AABB",
    "Constructor",
    crate::aabb_test::run_aabb_constructor
);
REGISTER_MINI_TEST!("AABB", "Empty", crate::aabb_test::run_aabb_empty);
REGISTER_MINI_TEST!("AABB", "Transform", crate::aabb_test::run_aabb_transform);
REGISTER_MINI_TEST!(
    "AABB",
    "From Geometry",
    crate::aabb_test::run_aabb_from_geometry
);
