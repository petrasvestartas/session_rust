use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_closest_line_point() -> TestResult {
    MINI_TEST!("Line Point", {
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
    MINI_TEST!("Polyline Point", {
        use crate::Closest;
        use crate::Polyline;
        use crate::Point;

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
        ]);

        let (_cp1, _t1, d1) = Closest::polyline_point(&pl, &Point::new(5.0, 5.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(d1, 5.0));

        let (cp2, _t2, d2) = Closest::polyline_point(&pl, &Point::new(10.0, 5.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cp2[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(cp2[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
    })
}

pub fn run_closest_curve_point() -> TestResult {
    MINI_TEST!("Curve Point", {
        use crate::Closest;
        use crate::NurbsCurve;
        use crate::Point;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
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
    MINI_TEST!("Surface Point", {
        use crate::Closest;
        use crate::NurbsSurface;
        use crate::Point;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 1.0),
            Point::new(2.0, 1.0, 1.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(1.0, 2.0, 1.0),
            Point::new(2.0, 2.0, 1.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
            Point::new(1.0, 3.0, 0.0),
            Point::new(2.0, 3.0, 0.0),
            Point::new(3.0, 3.0, 0.0),
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

pub fn run_closest_surface_curve() -> TestResult {
    MINI_TEST!("Surface Curve", {
        use crate::Closest;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Primitives;
        use crate::nurbsknot::CurveNurbsKnotStyle;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 4.0);
        let (u0, u1) = cyl.domain(0).unwrap();
        let (v0, v1) = cyl.domain(1).unwrap();
        let ps = cyl.point_at(u0, 0.5).unwrap();
        let seam_ang = ps[1].atan2(ps[0]);
        let mut crv_pts = Vec::new();
        for i in 0..21 {
            let a = seam_ang - 0.8 + 1.6 * i as f64 / 20.0;
            let z = 1.0 + 2.0 * i as f64 / 20.0;
            crv_pts.push(Point::new(a.cos(), a.sin(), z));
        }
        let crv = NurbsCurve::create_interpolated(&crv_pts, CurveNurbsKnotStyle::Chord);

        let pcurves = Closest::surface_curve(&cyl, &crv, 0.0, 0.0, None);

        MINI_CHECK!(pcurves.len() == 2);
        let mut on_border = 0;
        let mut inside = true;
        for pcurve in &pcurves {
            MINI_CHECK!(pcurve.is_valid());
            for e in [0.0, 1.0] {
                let p2 = pcurve.point_at(e);
                if (p2[0] - u0).abs() < 1e-9 || (p2[0] - u1).abs() < 1e-9 {
                    on_border += 1;
                }
            }
            for i in 0..17 {
                let p2 = pcurve.point_at(i as f64 / 16.0);
                if p2[0] < u0 - 1e-6 || p2[0] > u1 + 1e-6 || p2[1] < v0 - 1e-6 || p2[1] > v1 + 1e-6 {
                    inside = false;
                }
            }
        }
        MINI_CHECK!(on_border == 2);
        MINI_CHECK!(inside);

        let off = NurbsCurve::create(false, 1, &[Point::new(20.0, 20.0, 20.0), Point::new(30.0, 30.0, 30.0)]);

        MINI_CHECK!(Closest::surface_curve(&cyl, &off, 0.0, 0.0, None).len() == 0);
    })
}

pub fn run_closest_mesh_point() -> TestResult {
    MINI_TEST!("Mesh Point", {
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

pub fn run_closest_mesh_point_aabb() -> TestResult {
    MINI_TEST!("Mesh Point AABB", {
        use crate::Closest;
        use crate::Primitives;
        use crate::Point;

        let m = Primitives::cube(2.0);

        let (cp1, _fk1, d1) = Closest::mesh_point_aabb(&m, &Point::new(0.0, 0.0, 2.0));

        MINI_CHECK!(TOLERANCE.is_close(cp1[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));

        let (_cp2, _fk2, d2) = Closest::mesh_point_aabb(&m, &Point::new(1.0, 1.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
    })
}

pub fn run_closest_pointcloud_point() -> TestResult {
    MINI_TEST!("Pointcloud Point", {
        use crate::Closest;
        use crate::PointCloud;
        use crate::Point;

        let pc = PointCloud::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
        ], vec![], vec![]);

        let (cp1, i1, d1) = Closest::pointcloud_point(&pc, &Point::new(4.0, 0.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(cp1[0], 5.0));
        MINI_CHECK!(i1 == 1);
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));

        let (_cp2, i2, d2) = Closest::pointcloud_point(&pc, &Point::new(10.0, 10.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
        MINI_CHECK!(i2 == 3);
    })
}

pub fn run_closest_pointcloud_point_kdtree() -> TestResult {
    MINI_TEST!("Pointcloud Point SpatialKDTree", {
        use crate::Closest;
        use crate::PointCloud;
        use crate::Point;

        // SpatialKDTree variant: same result as linear scan, O(log n) query
        let pc = PointCloud::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(5.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
        ], vec![], vec![]);

        let (cp1, i1, d1) = Closest::pointcloud_point_kdtree(&pc, &Point::new(4.0, 0.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(cp1[0], 5.0));
        MINI_CHECK!(i1 == 1);
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));

        let (_cp2, i2, d2) = Closest::pointcloud_point_kdtree(&pc, &Point::new(10.0, 10.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
        MINI_CHECK!(i2 == 3);
    })
}

pub fn run_closest_lines_closest() -> TestResult {
    MINI_TEST!("Lines Closest", {
        use crate::Closest;
        use crate::Line;

        // 3 lines: first two sharing an endpoint, third far away
        let lines = vec![
            Line::new(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
            Line::new(5.0, 0.0, 0.0, 10.0, 0.0, 0.0),
            Line::new(100.0, 0.0, 0.0, 110.0, 0.0, 0.0),
        ];

        let pairs = Closest::lines_closest(&lines, 0.01);

        MINI_CHECK!(pairs.len() == 1);
        MINI_CHECK!(pairs[0].0 == 0);
        MINI_CHECK!(pairs[0].1 == 1);
    })
}

pub fn run_closest_polylines_closest() -> TestResult {
    MINI_TEST!("Polylines Closest", {
        use crate::Closest;
        use crate::Polyline;
        use crate::Point;

        let pls = vec![
            Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(5.0, 0.0, 0.0)]),
            Polyline::new(vec![Point::new(5.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0)]),
            Polyline::new(vec![Point::new(100.0, 0.0, 0.0), Point::new(110.0, 0.0, 0.0)]),
        ];

        let pairs = Closest::polylines_closest(&pls, 0.01);

        MINI_CHECK!(pairs.len() == 1);
        MINI_CHECK!(pairs[0].0 == 0);
        MINI_CHECK!(pairs[0].1 == 1);
    })
}

pub fn run_closest_nurbscurves_closest() -> TestResult {
    MINI_TEST!("Nurbscurves Closest", {
        use crate::Closest;
        use crate::NurbsCurve;
        use crate::Point;

        let curves = vec![
            NurbsCurve::create(false, 1, &[Point::new(0.0, 0.0, 0.0), Point::new(5.0, 0.0, 0.0)]),
            NurbsCurve::create(false, 1, &[Point::new(5.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0)]),
            NurbsCurve::create(false, 1, &[Point::new(100.0, 0.0, 0.0), Point::new(110.0, 0.0, 0.0)]),
        ];

        let pairs = Closest::nurbscurves_closest(&curves, 0.01);

        MINI_CHECK!(pairs.len() == 1);
        MINI_CHECK!(pairs[0].0 == 0);
        MINI_CHECK!(pairs[0].1 == 1);
    })
}

pub fn run_closest_boxes_closest() -> TestResult {
    MINI_TEST!("Boxes Closest", {
        use crate::aabb::AABB;
        use crate::Closest;

        // 3 boxes: first two touching faces (shared at x=1), third far away
        let boxes = vec![
            AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0),
            AABB::new(2.0, 0.0, 0.0, 1.0, 1.0, 1.0),
            AABB::new(20.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        ];

        let pairs = Closest::boxes_closest(&boxes, 0.01);

        MINI_CHECK!(pairs.len() == 1);
        MINI_CHECK!(pairs[0].0 == 0);
        MINI_CHECK!(pairs[0].1 == 1);
    })
}

REGISTER_MINI_TEST!("Closest", "Line Point", crate::closest_test::run_closest_line_point);
REGISTER_MINI_TEST!("Closest", "Polyline Point", crate::closest_test::run_closest_polyline_point);
REGISTER_MINI_TEST!("Closest", "Curve Point", crate::closest_test::run_closest_curve_point);
REGISTER_MINI_TEST!("Closest", "Surface Point", crate::closest_test::run_closest_surface_point);
REGISTER_MINI_TEST!("Closest", "Surface Curve", crate::closest_test::run_closest_surface_curve);
REGISTER_MINI_TEST!("Closest", "Mesh Point", crate::closest_test::run_closest_mesh_point);
REGISTER_MINI_TEST!("Closest", "Mesh Point AABB", crate::closest_test::run_closest_mesh_point_aabb);
REGISTER_MINI_TEST!("Closest", "Pointcloud Point", crate::closest_test::run_closest_pointcloud_point);
REGISTER_MINI_TEST!("Closest", "Pointcloud Point SpatialKDTree", crate::closest_test::run_closest_pointcloud_point_kdtree);
REGISTER_MINI_TEST!("Closest", "Lines Closest", crate::closest_test::run_closest_lines_closest);
REGISTER_MINI_TEST!("Closest", "Polylines Closest", crate::closest_test::run_closest_polylines_closest);
REGISTER_MINI_TEST!("Closest", "Nurbscurves Closest", crate::closest_test::run_closest_nurbscurves_closest);
REGISTER_MINI_TEST!("Closest", "Boxes Closest", crate::closest_test::run_closest_boxes_closest);
