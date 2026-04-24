use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_aabb_constructor() -> crate::mini_test::TestResult {
    use crate::tolerance::TOLERANCE;
    use crate::{AABB, Point};
    MINI_TEST!("Constructor", {
        // AABB(0,0,0, 1,2,3) — dims 2×4×6
        let a = AABB::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);

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
        MINI_CHECK!(a.get_corners().len() == 8);
        MINI_CHECK!(a.get_edges().len() == 12);
        MINI_CHECK!(a.point_at(1.0, 0.0, 0.0) == Point::new(1.0, 0.0, 0.0));
        MINI_CHECK!(a.point_at(0.0, 0.0, 0.0) == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a.intersects(&AABB::new(0.5, 0.0, 0.0, 0.5, 0.5, 0.5)));
        MINI_CHECK!(!a.intersects(&AABB::new(10.0, 0.0, 0.0, 0.5, 0.5, 0.5)));
        let mut a = a;
        let b = AABB::new(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        a.union_with(&b);
        MINI_CHECK!(a.min_point() == Point::new(-1.0, -2.0, -3.0));
        MINI_CHECK!(a.max_point() == Point::new(6.0, 2.0, 3.0));
        let c = AABB::merge(AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0), AABB::new(4.0, 0.0, 0.0, 1.0, 1.0, 1.0));
        MINI_CHECK!(c.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(c.max_point() == Point::new(5.0, 1.0, 1.0));
    })
}

pub fn run_aabb_from_geometry() -> TestResult {
    MINI_TEST!("From Geometry", {
        use crate::AABB;
        use crate::Color;
        use crate::Line;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::PointCloud;
        use crate::Polyline;
        use crate::Primitives;
        use crate::Vector;

        let a_pt = AABB::from_point(&Point::new(1.0, 2.0, 3.0), 0.5);

        MINI_CHECK!(a_pt.center() == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(a_pt.hx, 0.5));

        let a_pts = AABB::from_points(&[
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 4.0, 5.0),
        ], 0.0);

        MINI_CHECK!(a_pts.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pts.max_point() == Point::new(3.0, 4.0, 5.0));

        let ln = Line::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
        let a_line = AABB::from_line(&ln, 1.0);

        MINI_CHECK!(a_line.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(a_line.max_point() == Point::new(5.0, 1.0, 1.0));

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
        ]);
        let a_pl = AABB::from_polyline(&pl, 0.0);

        MINI_CHECK!(a_pl.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pl.max_point() == Point::new(2.0, 2.0, 0.0));

        let cube = Primitives::cube(2.0);
        let a_mesh = AABB::from_mesh(&cube, 0.0);

        MINI_CHECK!(a_mesh.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(a_mesh.max_point() == Point::new(1.0, 1.0, 1.0));

        let pc = PointCloud::new(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(4.0, 2.0, 6.0),
            ],
            vec![
                Vector::new(0.0, 0.0, 1.0),
                Vector::new(0.0, 0.0, 1.0),
            ],
            vec![
                Color::new(255, 0, 0, 255),
                Color::new(0, 255, 0, 255),
            ],
        );
        let a_pc = AABB::from_pointcloud(&pc, 0.0);

        MINI_CHECK!(a_pc.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pc.max_point() == Point::new(4.0, 2.0, 6.0));

        let curve = NurbsCurve::create(false, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);
        let a_nc = AABB::from_nurbscurve(&curve, 0.5, false);

        MINI_CHECK!(a_nc.is_valid());
        MINI_CHECK!(a_nc.contains(&Point::new(1.5, 0.0, 0.0)));

        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(2.0, 2.0, 2.0),
        ]).unwrap();
        let a_ns = AABB::from_nurbssurface(&surf, 0.0);

        MINI_CHECK!(a_ns.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(a_ns.volume(), 8.0));
    })
}

REGISTER_MINI_TEST!("AABB", "Constructor", crate::aabb_test::run_aabb_constructor);
REGISTER_MINI_TEST!("AABB", "From Geometry", crate::aabb_test::run_aabb_from_geometry);
