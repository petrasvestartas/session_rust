use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_closest_line_point() -> TestResult {
    MINI_TEST!("Line_point", {
        use crate::Closest;
        use crate::Line;
        use crate::Point;

        let l = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);

        let (cp1, t1, d1) = Closest::line_point(&l, &Point::new(5.0, 5.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cp1[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(cp1[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(t1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(d1, 5.0));

        let (cp2, t2, d2) = Closest::line_point(&l, &Point::new(-5.0, 0.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cp2[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(t2, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 5.0));

        let (cp3, t3, d3) = Closest::line_point(&l, &Point::new(15.0, 0.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cp3[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(t3, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d3, 5.0));
    })
}

pub fn run_closest_polyline_point() -> TestResult {
    MINI_TEST!("Polyline_point", {
        use crate::Closest;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0), Point::new(10.0, 10.0, 0.0)]);

        let (_cp1, _t1, d1) = Closest::polyline_point(&pl, &Point::new(5.0, 5.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d1, 5.0));

        let (cp2, _t2, d2) = Closest::polyline_point(&pl, &Point::new(10.0, 5.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cp2[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(cp2[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
    })
}

pub fn run_closest_curve_point() -> TestResult {
    MINI_TEST!("Curve_point", {
        use crate::Closest;
        use crate::NurbsCurve;
        use crate::Point;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 2.0, 0.0),
            Point::new(3.0, 2.0, 0.0), Point::new(4.0, 0.0, 0.0),
        ];
        let crv = NurbsCurve::create(false, 3, &pts);

        let (t, dist) = Closest::curve_point(&crv, &Point::new(2.0, 3.0, 0.0), 0.0, 0.0);
        MINI_CHECK!(dist < 1.6);
        let cp = crv.point_at(t);
        MINI_CHECK!(TOLERANCE.is_close(cp.distance(&Point::new(2.0, 3.0, 0.0), None), dist));

        let (_t2, dist2) = Closest::curve_point(&crv, &Point::new(0.0, 0.0, 0.0), 0.0, 0.0);
        MINI_CHECK!(dist2 < 0.01);
    })
}

pub fn run_closest_surface_point() -> TestResult {
    MINI_TEST!("Surface_point", {
        use crate::Closest;
        use crate::NurbsSurface;
        use crate::Point;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0), Point::new(3.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0), Point::new(1.0, 1.0, 1.0), Point::new(2.0, 1.0, 1.0), Point::new(3.0, 1.0, 0.0),
            Point::new(0.0, 2.0, 0.0), Point::new(1.0, 2.0, 1.0), Point::new(2.0, 2.0, 1.0), Point::new(3.0, 2.0, 0.0),
            Point::new(0.0, 3.0, 0.0), Point::new(1.0, 3.0, 0.0), Point::new(2.0, 3.0, 0.0), Point::new(3.0, 3.0, 0.0),
        ];
        let srf = NurbsSurface::create(false, false, 3, 3, 4, 4, &pts).unwrap();

        let (u, v, dist) = Closest::surface_point(&srf, &Point::new(1.5, 1.5, 2.0), 0.0, 0.0, 0.0, 0.0);
        MINI_CHECK!(dist < 1.5);
        let cp = srf.point_at(u, v).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(cp.distance(&Point::new(1.5, 1.5, 2.0), None), dist));

        let (_u2, _v2, dist2) = Closest::surface_point(&srf, &Point::new(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0);
        MINI_CHECK!(dist2 < 0.01);
    })
}

pub fn run_closest_mesh_point() -> TestResult {
    MINI_TEST!("Mesh_point", {
        use crate::Closest;
        use crate::Primitives;
        use crate::Point;

        let m = Primitives::cube(2.0);

        let (cp1, _fk1, d1) = Closest::mesh_point(&m, &Point::new(0.0, 0.0, 2.0));
        MINI_CHECK!(TOLERANCE.is_close(cp1[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));

        let (_cp2, _fk2, d2) = Closest::mesh_point(&m, &Point::new(1.0, 1.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
    })
}

pub fn run_closest_pointcloud_point() -> TestResult {
    MINI_TEST!("Pointcloud_point", {
        use crate::Closest;
        use crate::PointCloud;
        use crate::Point;

        let pc = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(5.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0), Point::new(10.0, 10.0, 0.0)], vec![], vec![]);

        let (cp1, i1, d1) = Closest::pointcloud_point(&pc, &Point::new(4.0, 0.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cp1[0], 5.0));
        MINI_CHECK!(i1 == 1);
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));

        let (_cp2, i2, d2) = Closest::pointcloud_point(&pc, &Point::new(10.0, 10.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
        MINI_CHECK!(i2 == 3);
    })
}

REGISTER_MINI_TEST!("Closest", "Line_point", crate::closest_test::run_closest_line_point);
REGISTER_MINI_TEST!("Closest", "Polyline_point", crate::closest_test::run_closest_polyline_point);
REGISTER_MINI_TEST!("Closest", "Curve_point", crate::closest_test::run_closest_curve_point);
REGISTER_MINI_TEST!("Closest", "Surface_point", crate::closest_test::run_closest_surface_point);
REGISTER_MINI_TEST!("Closest", "Mesh_point", crate::closest_test::run_closest_mesh_point);
REGISTER_MINI_TEST!("Closest", "Pointcloud_point", crate::closest_test::run_closest_pointcloud_point);
