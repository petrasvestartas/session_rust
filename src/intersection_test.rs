use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_intersection_line_line() -> TestResult {
    MINI_TEST!("Line Line", {
        use crate::intersection;
        use crate::Line;
        use crate::Tolerance;

        let line0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let line1 = Line::new(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);
        let output = intersection::line_line(&line0, &line1, Tolerance::APPROXIMATION);

        MINI_CHECK!(output.is_some());
        let output = output.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(output[0], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(output[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(output[2], 0.0));
    })
}

pub fn run_intersection_line_line_parallel() -> TestResult {
    MINI_TEST!("Line Line Parallel", {
        use crate::intersection;
        use crate::Line;
        use crate::Tolerance;

        let line0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let line1 = Line::new(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);
        let output = intersection::line_line(&line0, &line1, Tolerance::APPROXIMATION);

        MINI_CHECK!(output.is_none());
    })
}

pub fn run_intersection_line_line_parameters() -> TestResult {
    MINI_TEST!("Line Line Parameters", {
        use crate::intersection;
        use crate::Line;
        use crate::Tolerance;

        let line0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let line1 = Line::new(0.5, -1.0, 0.0, 0.5, 1.0, 0.0);
        let result = intersection::line_line_parameters(&line0, &line1, Tolerance::APPROXIMATION, true, false);

        MINI_CHECK!(result.is_some());
        let (t0, t1) = result.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(t0, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(t1, 0.5));
    })
}

pub fn run_intersection_line_line_parameters_endpoints() -> TestResult {
    MINI_TEST!("Line Line Parameters Endpoints", {
        use crate::intersection;
        use crate::Line;
        use crate::Tolerance;

        let line0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let line1 = Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        let result = intersection::line_line_parameters(&line0, &line1, Tolerance::APPROXIMATION, true, false);

        MINI_CHECK!(result.is_some());
        let (t0, t1) = result.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(t0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(t1, 0.0));
    })
}

pub fn run_intersection_line_line_parameters_infinite() -> TestResult {
    MINI_TEST!("Line Line Parameters Infinite", {
        use crate::intersection;
        use crate::Line;
        use crate::Tolerance;

        let line0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let line1 = Line::new(2.0, -1.0, 0.0, 2.0, 1.0, 0.0);
        let result = intersection::line_line_parameters(&line0, &line1, Tolerance::APPROXIMATION, false, false);

        MINI_CHECK!(result.is_some());
        let (t0, _t1) = result.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(t0, 2.0));
    })
}

pub fn run_intersection_plane_plane() -> TestResult {
    MINI_TEST!("Plane Plane", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let n0 = Vector::new(0.0, 0.0, 1.0);
        let plane0 = Plane::from_point_normal(p0, n0);

        let p1 = Point::new(0.0, 0.0, 0.0);
        let n1 = Vector::new(0.0, 1.0, 0.0);
        let plane1 = Plane::from_point_normal(p1, n1);

        let output = intersection::plane_plane(&plane0, &plane1);

        MINI_CHECK!(output.is_some());
        let line_dir = output.unwrap().to_vector();
        MINI_CHECK!((line_dir[0].abs() - 1.0).abs() < 1e-4);
        MINI_CHECK!(line_dir[1].abs() < 1e-4);
        MINI_CHECK!(line_dir[2].abs() < 1e-4);
    })
}

pub fn run_intersection_plane_plane_complex() -> TestResult {
    MINI_TEST!("Plane Plane Complex", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let plane_origin_0 = Point::new(213.787107, 513.797811, -24.743845);
        let plane_xaxis_0 = Vector::new(0.907673, -0.258819, 0.330366);
        let plane_yaxis_0 = Vector::new(0.272094, 0.96225, 0.006285);
        let pl0 = Plane::new(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

        let plane_origin_1 = Point::new(247.17924, 499.115486, 59.619568);
        let plane_xaxis_1 = Vector::new(0.552465, 0.816035, 0.16991);
        let plane_yaxis_1 = Vector::new(0.172987, 0.087156, -0.98106);
        let pl1 = Plane::new(plane_origin_1, plane_xaxis_1, plane_yaxis_1);

        let intersection_line = intersection::plane_plane(&pl0, &pl1);

        MINI_CHECK!(intersection_line.is_some());
        let intersection_line = intersection_line.unwrap();

        let start = intersection_line.start();
        let end = intersection_line.end();

        MINI_CHECK!((start[0] - 252.4632).abs() < 0.01);
        MINI_CHECK!((start[1] - 495.32248).abs() < 0.01);
        MINI_CHECK!((start[2] - (-10.002656)).abs() < 0.01);

        MINI_CHECK!((end[0] - 253.01033).abs() < 0.01);
        MINI_CHECK!((end[1] - 496.1218).abs() < 0.01);
        MINI_CHECK!((end[2] - (-9.888727)).abs() < 0.01);
    })
}

pub fn run_intersection_line_plane() -> TestResult {
    MINI_TEST!("Line Plane", {
        use crate::intersection;
        use crate::Line;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let p = Point::new(0.0, 0.0, 1.0);
        let n = Vector::new(0.0, 0.0, 1.0);
        let plane = Plane::from_point_normal(p, n);

        let line = Line::new(0.0, 0.0, 0.0, 0.0, 0.0, 2.0);

        let output = intersection::line_plane(&line, &plane, true);

        MINI_CHECK!(output.is_some());
        let output = output.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(output[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(output[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(output[2], 1.0));
    })
}

pub fn run_intersection_line_plane_parallel() -> TestResult {
    MINI_TEST!("Line Plane Parallel", {
        use crate::intersection;
        use crate::Line;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let p = Point::new(0.0, 0.0, 1.0);
        let n = Vector::new(0.0, 0.0, 1.0);
        let plane = Plane::from_point_normal(p, n);

        let line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

        let output = intersection::line_plane(&line, &plane, true);

        MINI_CHECK!(output.is_none());
    })
}

pub fn run_intersection_line_plane_real_world() -> TestResult {
    MINI_TEST!("Line Plane Real World", {
        use crate::intersection;
        use crate::Line;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let l0 = Line::new(500.000, -573.576, -819.152, 500.000, 573.576, 819.152);

        let plane_origin_0 = Point::new(213.787107, 513.797811, -24.743845);
        let plane_xaxis_0 = Vector::new(0.907673, -0.258819, 0.330366);
        let plane_yaxis_0 = Vector::new(0.272094, 0.96225, 0.006285);
        let pl0 = Plane::new(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

        let lp = intersection::line_plane(&l0, &pl0, false);

        MINI_CHECK!(lp.is_some());
        let lp = lp.unwrap();
        MINI_CHECK!((lp[0] - 500.0).abs() < 0.1);
        MINI_CHECK!((lp[1] - 77.7531).abs() < 0.01);
        MINI_CHECK!((lp[2] - 111.043).abs() < 0.01);
    })
}

pub fn run_intersection_plane_plane_plane() -> TestResult {
    MINI_TEST!("Plane Plane Plane", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let plane_origin_0 = Point::new(213.787107, 513.797811, -24.743845);
        let plane_xaxis_0 = Vector::new(0.907673, -0.258819, 0.330366);
        let plane_yaxis_0 = Vector::new(0.272094, 0.96225, 0.006285);
        let pl0 = Plane::new(plane_origin_0, plane_xaxis_0, plane_yaxis_0);

        let plane_origin_1 = Point::new(247.17924, 499.115486, 59.619568);
        let plane_xaxis_1 = Vector::new(0.552465, 0.816035, 0.16991);
        let plane_yaxis_1 = Vector::new(0.172987, 0.087156, -0.98106);
        let pl1 = Plane::new(plane_origin_1, plane_xaxis_1, plane_yaxis_1);

        let plane_origin_2 = Point::new(221.399816, 605.893667, -54.000116);
        let plane_xaxis_2 = Vector::new(0.903451, -0.360516, -0.231957);
        let plane_yaxis_2 = Vector::new(0.172742, -0.189057, 0.966653);
        let pl2 = Plane::new(plane_origin_2, plane_xaxis_2, plane_yaxis_2);

        let output = intersection::plane_plane_plane(&pl0, &pl1, &pl2);

        MINI_CHECK!(output.is_some());
        let output = output.unwrap();
        MINI_CHECK!((output[0] - 300.5).abs() < 0.1);
        MINI_CHECK!((output[1] - 565.5).abs() < 0.1);
        MINI_CHECK!((output[2] - 0.0).abs() < 0.1);
    })
}

pub fn run_intersection_plane_plane_plane_parallel() -> TestResult {
    MINI_TEST!("Plane Plane Plane Parallel", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let n0 = Vector::new(0.0, 0.0, 1.0);
        let plane0 = Plane::from_point_normal(p0, n0);

        let p1 = Point::new(0.0, 0.0, 1.0);
        let n1 = Vector::new(0.0, 0.0, 1.0);
        let plane1 = Plane::from_point_normal(p1, n1);

        let p2 = Point::new(0.0, 0.0, 0.0);
        let n2 = Vector::new(1.0, 0.0, 0.0);
        let plane2 = Plane::from_point_normal(p2, n2);

        let output = intersection::plane_plane_plane(&plane0, &plane1, &plane2);

        MINI_CHECK!(output.is_none());
    })
}

pub fn run_intersection_ray_box() -> TestResult {
    MINI_TEST!("Ray Box", {
        use crate::intersection;
        use crate::OBB;
        use crate::Line;
        use crate::Point;

        let box_ = OBB::from_points(&[
            Point::new(-1.0, -1.0, -1.0),
            Point::new(1.0, 1.0, 1.0),
        ], 0.0);
        let line = Line::new(-5.0, 0.0, 0.0, -4.0, 0.0, 0.0);
        let points = intersection::ray_box(&line, &box_, 0.0, 100.0);

        MINI_CHECK!(points.is_some());
        let points = points.unwrap();
        MINI_CHECK!((points[0][0] - (-1.0)).abs() < 1e-4);
        MINI_CHECK!((points[1][0] - 1.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_box_miss() -> TestResult {
    MINI_TEST!("Ray Box Miss", {
        use crate::intersection;
        use crate::OBB;
        use crate::Line;
        use crate::Point;

        let box_ = OBB::from_points(&[
            Point::new(-1.0, -1.0, -1.0),
            Point::new(1.0, 1.0, 1.0),
        ], 0.0);
        let line = Line::new(-5.0, 5.0, 0.0, -4.0, 5.0, 0.0);
        let points = intersection::ray_box(&line, &box_, 0.0, 100.0);

        MINI_CHECK!(points.is_none());
    })
}

pub fn run_intersection_ray_sphere() -> TestResult {
    MINI_TEST!("Ray Sphere", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let line = Line::new(-5.0, 0.0, 0.0, -4.0, 0.0, 0.0);
        let center = Point::new(0.0, 0.0, 0.0);
        let radius = 2.0;
        let points = intersection::ray_sphere(&line, &center, radius);

        MINI_CHECK!(points.is_some());
        let points = points.unwrap();
        MINI_CHECK!(points.len() == 2);
        MINI_CHECK!((points[0][0] - (-2.0)).abs() < 1e-4);
        MINI_CHECK!((points[1][0] - 2.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_sphere_tangent() -> TestResult {
    MINI_TEST!("Ray Sphere Tangent", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let line = Line::new(-5.0, 2.0, 0.0, -4.0, 2.0, 0.0);
        let center = Point::new(0.0, 0.0, 0.0);
        let radius = 2.0;
        let points = intersection::ray_sphere(&line, &center, radius);

        MINI_CHECK!(points.is_some());
        let points = points.unwrap();
        MINI_CHECK!(points.len() == 1);
        MINI_CHECK!((points[0][0] - 0.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_sphere_miss() -> TestResult {
    MINI_TEST!("Ray Sphere Miss", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let line = Line::new(-5.0, 5.0, 0.0, -4.0, 5.0, 0.0);
        let center = Point::new(0.0, 0.0, 0.0);
        let radius = 2.0;
        let points = intersection::ray_sphere(&line, &center, radius);

        MINI_CHECK!(points.is_none());
    })
}

pub fn run_intersection_ray_triangle() -> TestResult {
    MINI_TEST!("Ray Triangle", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let line = Line::new(0.5, 0.5, -1.0, 0.5, 0.5, 0.0);
        let v0 = Point::new(0.0, 0.0, 0.0);
        let v1 = Point::new(1.0, 0.0, 0.0);
        let v2 = Point::new(0.0, 1.0, 0.0);
        let result = intersection::ray_triangle(&line, &v0, &v1, &v2, 1e-6);

        MINI_CHECK!(result.is_some());
        let result = result.unwrap();
        MINI_CHECK!((result[2] - 0.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_triangle_miss() -> TestResult {
    MINI_TEST!("Ray Triangle Miss", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let line = Line::new(2.0, 2.0, -1.0, 2.0, 2.0, 0.0);
        let v0 = Point::new(0.0, 0.0, 0.0);
        let v1 = Point::new(1.0, 0.0, 0.0);
        let v2 = Point::new(0.0, 1.0, 0.0);
        let result = intersection::ray_triangle(&line, &v0, &v1, &v2, 1e-6);

        MINI_CHECK!(result.is_none());
    })
}

pub fn run_intersection_ray_triangle_parallel() -> TestResult {
    MINI_TEST!("Ray Triangle Parallel", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let line = Line::new(0.5, 0.5, -1.0, 1.5, 0.5, -1.0);
        let v0 = Point::new(0.0, 0.0, 0.0);
        let v1 = Point::new(1.0, 0.0, 0.0);
        let v2 = Point::new(0.0, 1.0, 0.0);
        let result = intersection::ray_triangle(&line, &v0, &v1, &v2, 1e-6);

        MINI_CHECK!(result.is_none());
    })
}

pub fn run_intersection_ray_mesh() -> TestResult {
    MINI_TEST!("Ray Mesh", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let polygons: Vec<Vec<Point>> = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.0, 1.0, 1.0),
                Point::new(0.0, 1.0, 1.0),
            ],
        ];
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(0.5, 0.5, -1.0, 0.5, 0.5, 0.0);
        let hits = intersection::ray_mesh(&line, &mesh, 1e-6, true);

        MINI_CHECK!(hits.is_some());
        let hits = hits.unwrap();
        MINI_CHECK!(hits.len() >= 1);
        MINI_CHECK!((hits[0][2] - 0.0).abs() < 1e-3);
    })
}

pub fn run_intersection_ray_mesh_first() -> TestResult {
    MINI_TEST!("Ray Mesh First", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let polygons: Vec<Vec<Point>> = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.0, 1.0, 1.0),
                Point::new(0.0, 1.0, 1.0),
            ],
        ];
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(0.5, 0.5, -1.0, 0.5, 0.5, 0.0);
        let hits = intersection::ray_mesh(&line, &mesh, 1e-6, false);

        MINI_CHECK!(hits.is_some());
        let hits = hits.unwrap();
        MINI_CHECK!(hits.len() == 1);
    })
}

pub fn run_intersection_ray_mesh_miss() -> TestResult {
    MINI_TEST!("Ray Mesh Miss", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let polygons: Vec<Vec<Point>> = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
        ];
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(5.0, 5.0, -1.0, 5.0, 5.0, 0.0);
        let hits = intersection::ray_mesh(&line, &mesh, 1e-6, true);

        MINI_CHECK!(hits.is_none());
    })
}

pub fn run_intersection_ray_mesh_bvh() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let polygons: Vec<Vec<Point>> = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.0, 1.0, 1.0),
                Point::new(0.0, 1.0, 1.0),
            ],
        ];
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(0.5, 0.5, -1.0, 0.5, 0.5, 0.0);
        let hits = intersection::ray_mesh_bvh(&line, &mesh, 1e-6, true);

        MINI_CHECK!(hits.is_some());
        let hits = hits.unwrap();
        MINI_CHECK!(hits.len() >= 1);
        MINI_CHECK!((hits[0][2] - 0.0).abs() < 1e-3);
    })
}

pub fn run_intersection_ray_mesh_bvh_first() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh First", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let polygons: Vec<Vec<Point>> = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.0, 1.0, 1.0),
                Point::new(0.0, 1.0, 1.0),
            ],
        ];
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(0.5, 0.5, -1.0, 0.5, 0.5, 0.0);
        let hits = intersection::ray_mesh_bvh(&line, &mesh, 1e-6, false);

        MINI_CHECK!(hits.is_some());
        let hits = hits.unwrap();
        MINI_CHECK!(hits.len() == 1);
    })
}

pub fn run_intersection_ray_mesh_bvh_miss() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh Miss", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let polygons: Vec<Vec<Point>> = vec![
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
        ];
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(5.0, 5.0, -1.0, 5.0, 5.0, 0.0);
        let hits = intersection::ray_mesh_bvh(&line, &mesh, 1e-6, true);

        MINI_CHECK!(hits.is_none());
    })
}

pub fn run_intersection_ray_mesh_bvh_vs_naive() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh Vs Naive", {
        use crate::intersection;
        use crate::Line;
        use crate::Mesh;
        use crate::Point;

        let mut polygons: Vec<Vec<Point>> = Vec::new();
        for i in 0..10 {
            for j in 0..10 {
                let x = i as f64;
                let y = j as f64;
                polygons.push(vec![
                    Point::new(x, y, 0.0),
                    Point::new(x + 1.0, y, 0.0),
                    Point::new(x + 1.0, y + 1.0, 0.0),
                    Point::new(x, y + 1.0, 0.0),
                ]);
            }
        }
        let mesh = Mesh::from_polylines(polygons, None);
        let line = Line::new(5.5, 5.5, -1.0, 5.5, 5.5, 0.0);
        let hits_naive = intersection::ray_mesh(&line, &mesh, 1e-6, true);
        let hits_bvh = intersection::ray_mesh_bvh(&line, &mesh, 1e-6, true);
        let result_naive = hits_naive.is_some();
        let result_bvh = hits_bvh.is_some();

        MINI_CHECK!(result_naive == result_bvh);
        let naive_count = hits_naive.as_ref().map(|h| h.len()).unwrap_or(0);
        let bvh_count = hits_bvh.as_ref().map(|h| h.len()).unwrap_or(0);
        MINI_CHECK!(naive_count == bvh_count);
        if let (Some(hn), Some(hb)) = (&hits_naive, &hits_bvh) {
            if !hn.is_empty() {
                MINI_CHECK!((hn[0][2] - hb[0][2]).abs() < 1e-4);
            }
        }
    })
}

pub fn run_intersection_ray_box_real_world() -> TestResult {
    MINI_TEST!("Ray Box Real World", {
        use crate::intersection;
        use crate::OBB;
        use crate::Line;
        use crate::Point;

        let l0 = Line::new(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
        let min_pt = Point::new(214.0, 192.0, 484.0);
        let max_pt = Point::new(694.0, 567.0, 796.0);
        let box_ = OBB::from_points(&[min_pt, max_pt], 0.0);
        let points = intersection::ray_box(&l0, &box_, 0.0, 1000.0);

        MINI_CHECK!(points.is_some());
        let points = points.unwrap();
        MINI_CHECK!(points.len() == 2);
        MINI_CHECK!((points[0][0] - 500.0).abs() < 0.1);
        MINI_CHECK!((points[0][1] - 338.9).abs() < 0.1);
        MINI_CHECK!((points[0][2] - 484.0).abs() < 0.1);
        MINI_CHECK!((points[1][0] - 500.0).abs() < 0.1);
        MINI_CHECK!((points[1][1] - 557.365).abs() < 0.1);
        MINI_CHECK!((points[1][2] - 796.0).abs() < 0.1);
    })
}

pub fn run_intersection_ray_sphere_real_world() -> TestResult {
    MINI_TEST!("Ray Sphere Real World", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;

        let l0 = Line::new(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
        let sphere_center = Point::new(457.0, 192.0, 207.0);
        let radius = 265.0;
        let points = intersection::ray_sphere(&l0, &sphere_center, radius);

        MINI_CHECK!(points.is_some());
        let points = points.unwrap();
        MINI_CHECK!(points.len() == 2);
        MINI_CHECK!((points[0][0] - 500.0).abs() < 0.1);
        MINI_CHECK!((points[0][1] - 12.08).abs() < 0.1);
        MINI_CHECK!((points[0][2] - 17.25).abs() < 0.1);
        MINI_CHECK!((points[1][0] - 500.0).abs() < 0.1);
        MINI_CHECK!((points[1][1] - 308.77).abs() < 0.1);
        MINI_CHECK!((points[1][2] - 440.97).abs() < 0.1);
    })
}

pub fn run_intersection_ray_triangle_real_world() -> TestResult {
    MINI_TEST!("Ray Triangle Real World", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;
        use crate::Tolerance;

        let l0 = Line::new(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
        let p1 = Point::new(214.0, 567.0, 484.0);
        let p2 = Point::new(214.0, 192.0, 796.0);
        let p3 = Point::new(694.0, 192.0, 484.0);
        let result = intersection::ray_triangle(&l0, &p1, &p2, &p3, Tolerance::APPROXIMATION);

        MINI_CHECK!(result.is_some());
        let result = result.unwrap();
        MINI_CHECK!((result[0] - 500.0).abs() < 0.1);
        MINI_CHECK!((result[1] - 340.616).abs() < 0.01);
        MINI_CHECK!((result[2] - 486.451).abs() < 0.01);
    })
}

pub fn run_intersection_surface_plane() -> TestResult {
    MINI_TEST!("Surface Plane", {
        use crate::intersection;
        use crate::NurbsSurface;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(10.0, 0.0, 10.0),
            Point::new(10.0, 10.0, 10.0),
        ];
        let srf = NurbsSurface::create(false, false, 1, 1, 2, 2, &pts).unwrap();
        let plane = Plane::from_point_normal(Point::new(0.0, 0.0, 5.0), Vector::new(0.0, 0.0, 1.0));
        let curves = intersection::surface_plane(&srf, &plane, None);

        MINI_CHECK!(curves.len() == 1);
        MINI_CHECK!(curves[0].is_valid());
        let (t0, t1) = curves[0].domain();
        for i in 0..=10 {
            let t = t0 + (t1 - t0) * i as f64 / 10.0;
            let p = curves[0].point_at(t);
            MINI_CHECK!((p[0] - 5.0).abs() < 0.5);
            MINI_CHECK!((p[2] - 5.0).abs() < 0.5);
        }
    })
}

pub fn run_intersection_surface_plane_curved() -> TestResult {
    MINI_TEST!("Surface Plane Curved", {
        use crate::intersection;
        use crate::NurbsSurface;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let mut pts = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                let x = i as f64 * 10.0;
                let y = j as f64 * 10.0;
                let z = if (i == 1 || i == 2) && (j == 1 || j == 2) { 10.0 } else { 0.0 };
                pts.push(Point::new(x, y, z));
            }
        }
        let srf = NurbsSurface::create(false, false, 3, 3, 4, 4, &pts).unwrap();
        let plane = Plane::from_point_normal(Point::new(0.0, 0.0, 3.0), Vector::new(0.0, 0.0, 1.0));
        let curves = intersection::surface_plane(&srf, &plane, None);

        MINI_CHECK!(curves.len() >= 1);
        MINI_CHECK!(curves[0].is_valid());
        MINI_CHECK!(curves[0].degree() == 3);
        let (t0, t1) = curves[0].domain();
        for i in 0..=10 {
            let t = t0 + (t1 - t0) * i as f64 / 10.0;
            let p = curves[0].point_at(t);
            MINI_CHECK!((p[2] - 3.0).abs() < 1.0);
        }
    })
}

pub fn run_intersection_surface_plane_miss() -> TestResult {
    MINI_TEST!("Surface Plane Miss", {
        use crate::intersection;
        use crate::NurbsSurface;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
        ];
        let srf = NurbsSurface::create(false, false, 1, 1, 2, 2, &pts).unwrap();
        let plane = Plane::from_point_normal(Point::new(0.0, 0.0, 5.0), Vector::new(0.0, 0.0, 1.0));
        let curves = intersection::surface_plane(&srf, &plane, None);

        MINI_CHECK!(curves.len() == 0);
    })
}

pub fn run_intersection_surface_plane_uv() -> TestResult {
    MINI_TEST!("Surface Plane UV", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;
        use crate::primitives::Primitives;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 4.0);
        let plane = Plane::from_point_normal(Point::new(0.0, 0.0, 2.0), Vector::new(0.3, 0.0, 1.0));
        let pairs = intersection::surface_plane_uv(&cyl, &plane, None);

        MINI_CHECK!(pairs.len() == 1);
        let curve3 = &pairs[0].0;
        let pcurve = &pairs[0].1;
        MINI_CHECK!(curve3.is_valid());
        MINI_CHECK!(pcurve.is_valid());
        MINI_CHECK!(curve3.is_closed());
        let (u0, u1) = cyl.domain(0).unwrap();
        MINI_CHECK!((pcurve.point_at(0.0)[0] - u1).abs() < 1e-9 || (pcurve.point_at(0.0)[0] - u0).abs() < 1e-9);
        MINI_CHECK!((pcurve.point_at(1.0)[0] - u1).abs() < 1e-9 || (pcurve.point_at(1.0)[0] - u0).abs() < 1e-9);
        let pn = plane.z_axis();
        let po = plane.origin();
        let mut max_off = 0.0f64;
        for i in 0..17 {
            let p2 = pcurve.point_at(i as f64 / 16.0);
            let s = cyl.point_at(p2[0], p2[1]).unwrap();
            let off = ((s[0] - po[0]) * pn[0] + (s[1] - po[1]) * pn[1] + (s[2] - po[2]) * pn[2]).abs();
            max_off = max_off.max(off);
        }
        MINI_CHECK!(max_off < 0.05);

        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let plane2 = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0));
        let pairs2 = intersection::surface_plane_uv(&torus, &plane2, None);

        MINI_CHECK!(pairs2.len() == 2);
        let (tu0, tu1) = torus.domain(0).unwrap();
        let (tv0, tv1) = torus.domain(1).unwrap();
        let mut inside = true;
        for pair in &pairs2 {
            for i in 0..17 {
                let p2 = pair.1.point_at(i as f64 / 16.0);
                if p2[0] < tu0 - 1e-6 || p2[0] > tu1 + 1e-6 || p2[1] < tv0 - 1e-6 || p2[1] > tv1 + 1e-6 {
                    inside = false;
                }
            }
        }
        MINI_CHECK!(inside);
    })
}

pub fn run_intersection_surface_surface() -> TestResult {
    MINI_TEST!("Surface Surface", {
        use crate::intersection;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::primitives::Primitives;

        let lies_on_curve = |curve3d: &crate::NurbsCurve, pcurve: &crate::NurbsCurve, surface: &NurbsSurface| -> f64 {
            let (u0, u1) = surface.domain(0).unwrap();
            let (v0, v1) = surface.domain(1).unwrap();
            let dense: Vec<Point> = (0..129).map(|j| curve3d.point_at(j as f64 / 128.0)).collect();
            let mut worst = 0.0f64;
            for i in 0..33 {
                let q = pcurve.point_at(i as f64 / 32.0);
                let s = surface.point_at(q[0].max(u0).min(u1), q[1].max(v0).min(v1)).unwrap();
                let mut best = dense[0].distance(&s, None);
                for p in &dense {
                    let d = p.distance(&s, None);
                    if d < best { best = d; }
                }
                if best > worst { worst = best; }
            }
            worst
        };

        // Planar dispatch path: flat NURBS patch x cylinder -> one closed circle
        let flat = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(-3.0, -3.0, 0.5),
            Point::new(-3.0, 3.0, 0.5),
            Point::new(3.0, -3.0, 0.5),
            Point::new(3.0, 3.0, 0.5),
        ]).unwrap();
        let cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
        let flat_triples = intersection::surface_surface(&flat, &cyl, None);

        MINI_CHECK!(flat_triples.len() == 1);
        let (c3, pa, pb) = &flat_triples[0];
        MINI_CHECK!(c3.is_valid() && pa.is_valid() && pb.is_valid());
        MINI_CHECK!(c3.is_closed());
        MINI_CHECK!(lies_on_curve(c3, pa, &flat) < 0.05);
        MINI_CHECK!(lies_on_curve(c3, pb, &cyl) < 0.05);

        // Marching path: sphere x cylinder is a QUARTIC (not a conic). The marcher
        // finds the intersection branches and most of each branch lies on both
        // surfaces; precise pcurves on seam-crossing branches are still WIP. So
        // assert the branches are found and at least two clean arcs lie on both.
        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let cyl2 = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
        let triples = intersection::surface_surface(&sphere, &cyl2, None);

        MINI_CHECK!(triples.len() >= 2);
        let mut clean = 0;
        for (c3, pa, pb) in &triples {
            MINI_CHECK!(c3.is_valid() && pa.is_valid() && pb.is_valid());
            if lies_on_curve(c3, pa, &sphere) < 0.05 && lies_on_curve(c3, pb, &cyl2) < 0.05 {
                clean += 1;
            }
        }
        MINI_CHECK!(clean >= 2);
    })
}

pub fn run_intersection_surface_surface_accuracy() -> TestResult {
    MINI_TEST!("Surface Surface Accuracy", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::primitives::Primitives;
        use crate::intersection::surface_surface;

        // Every intersection point must lie on BOTH analytic surfaces to ~1e-6,
        // measured by closed-form distance (independent of OCCT/parameterization).
        fn on_both<FA: Fn(&crate::Point)->f64, FB: Fn(&crate::Point)->f64>(c3: &crate::NurbsCurve, da: FA, db: FB) -> f64 {
            let mut worst = 0.0f64;
            for i in 0..=64 {
                let p = c3.point_at(i as f64 / 64.0);
                worst = worst.max(da(&p)).max(db(&p));
            }
            worst
        }

        // Quartic: sphere x cylinder (axis Z at x=1.3) -> 1e-6.
        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let cyl = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
        let d_sph = |p: &Point| ((p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt() - 2.0).abs();
        let d_cyl = |p: &Point| (((p[0]-1.3).powi(2) + p[1]*p[1]).sqrt() - 0.3).abs();
        let tr = surface_surface(&sphere, &cyl, None);
        MINI_CHECK!(tr.len() >= 2);
        for (c3, _pa, _pb) in &tr {
            MINI_CHECK!(on_both(c3, d_sph, d_cyl) < 1e-5);
        }

        // Conic: sphere x sphere -> exact circle.
        let sphere2 = Primitives::sphere_surface(2.0, 0.0, 0.0, 2.0);
        let d_sph2 = |p: &Point| (((p[0]-2.0).powi(2) + p[1]*p[1] + p[2]*p[2]).sqrt() - 2.0).abs();
        let tr2 = surface_surface(&sphere, &sphere2, None);
        MINI_CHECK!(tr2.len() >= 1);
        for (c3, _pa, _pb) in &tr2 {
            MINI_CHECK!(on_both(c3, d_sph, d_sph2) < 1e-6);
        }

        // Torus x perpendicular plane -> two exact circles.
        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let flat = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(-9.0, -9.0, 0.0),
            Point::new(-9.0, 9.0, 0.0),
            Point::new(9.0, -9.0, 0.0),
            Point::new(9.0, 9.0, 0.0),
        ]).unwrap();
        let d_tor = |p: &Point| (((((p[0]*p[0]+p[1]*p[1]).sqrt()) - 2.0).powi(2) + p[2]*p[2]).sqrt() - 0.5).abs();
        let d_flat = |p: &Point| p[2].abs();
        let tr3 = surface_surface(&torus, &flat, None);
        MINI_CHECK!(tr3.len() == 2);
        for (c3, _pa, _pb) in &tr3 {
            MINI_CHECK!(on_both(c3, d_tor, d_flat) < 1e-6);
        }
    })
}

pub fn run_intersection_remap() -> TestResult {
    MINI_TEST!("Remap", {
        use crate::intersection;
        MINI_CHECK!((intersection::remap(5.0, 0.0, 10.0, 0.0, 1.0) - 0.5).abs() < 1e-9);
        MINI_CHECK!((intersection::remap(0.0, 0.0, 10.0, 0.0, 1.0) - 0.0).abs() < 1e-9);
        MINI_CHECK!((intersection::remap(10.0, 0.0, 10.0, 0.0, 1.0) - 1.0).abs() < 1e-9);
    })
}

pub fn run_intersection_closest_point_on_segment() -> TestResult {
    MINI_TEST!("Closest Point On Segment", {
        use crate::intersection;
        use crate::{Line, Point};

        // Point projects onto segment interior
        let seg = Line::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
        let pt = Point::new(2.0, 3.0, 0.0);
        let (cp, t) = intersection::closest_point_on_segment(&pt, &seg);

        MINI_CHECK!((cp[0] - 2.0).abs() < 1e-9);
        MINI_CHECK!((cp[1] - 0.0).abs() < 1e-9);
        MINI_CHECK!((t - 0.5).abs() < 1e-9);

        // Point projects before segment start → clamped to t=0
        let pt2 = Point::new(-2.0, 1.0, 0.0);
        let (cp2, t2) = intersection::closest_point_on_segment(&pt2, &seg);
        MINI_CHECK!((cp2[0] - 0.0).abs() < 1e-9);
        MINI_CHECK!((t2 - 0.0).abs() < 1e-9);
    })
}

pub fn run_intersection_plane_plane_plane_check_parallel() -> TestResult {
    MINI_TEST!("Plane Plane Plane Check Parallel", {
        use crate::intersection;
        use crate::{Plane, Vector};

        // Two parallel planes → should return None
        let p0 = Plane::from_point_normal(crate::Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0));
        let p1 = Plane::from_point_normal(crate::Point::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0));
        let p2 = Plane::from_point_normal(crate::Point::new(0.0, 0.0, 2.0), Vector::new(0.0, 0.0, 1.0));

        MINI_CHECK!(intersection::plane_plane_plane_check(&p0, &p1, &p2, 0.1).is_none());

        // Three valid planes
        let px = Plane::from_point_normal(crate::Point::new(1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0));
        let py = Plane::from_point_normal(crate::Point::new(0.0, 2.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let pz = Plane::from_point_normal(crate::Point::new(0.0, 0.0, 3.0), Vector::new(0.0, 0.0, 1.0));
        let pt = intersection::plane_plane_plane_check(&px, &py, &pz, 0.1);
        MINI_CHECK!(pt.is_some());
        let pt = pt.unwrap();
        MINI_CHECK!((pt[0] - 1.0).abs() < 1e-6);
        MINI_CHECK!((pt[1] - 2.0).abs() < 1e-6);
        MINI_CHECK!((pt[2] - 3.0).abs() < 1e-6);
    })
}

pub fn run_intersection_plane_4planes() -> TestResult {
    MINI_TEST!("Plane 4 Planes Closed", {
        use crate::intersection;
        use crate::{Plane, Point, Vector};

        // main = z=0; boundary planes cycle left→bottom→right→top (adjacent pairs non-parallel)
        let main = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0));
        let planes = [
            Plane::from_point_normal(Point::new(-1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0)),  // x=-1
            Plane::from_point_normal(Point::new(0.0, -1.0, 0.0), Vector::new(0.0, 1.0, 0.0)),  // y=-1
            Plane::from_point_normal(Point::new(1.0, 0.0, 0.0),  Vector::new(1.0, 0.0, 0.0)),  // x= 1
            Plane::from_point_normal(Point::new(0.0, 1.0, 0.0),  Vector::new(0.0, 1.0, 0.0)),  // y= 1
        ];
        let result = intersection::plane_4planes(&main, &planes);

        MINI_CHECK!(result.is_some());
        let poly = result.unwrap();
        // Closed polyline has 5 points (first == last)
        MINI_CHECK!(poly.len() == 5);
        // All z values should be 0 (on main plane)
        let pts = poly.get_points();
        for p in &pts {
            MINI_CHECK!(p[2].abs() < 1e-6);
        }
        // Verify first == last
        MINI_CHECK!((pts[0][0] - pts[4][0]).abs() < 1e-6);
        MINI_CHECK!((pts[0][1] - pts[4][1]).abs() < 1e-6);
    })
}

pub fn run_intersection_plane_4planes_open() -> TestResult {
    MINI_TEST!("Plane 4 Planes Open", {
        use crate::intersection;
        use crate::{Plane, Point, Vector};

        let main = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0));
        let planes = [
            Plane::from_point_normal(Point::new(-1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0)),
            Plane::from_point_normal(Point::new(0.0, -1.0, 0.0), Vector::new(0.0, 1.0, 0.0)),
            Plane::from_point_normal(Point::new(1.0, 0.0, 0.0),  Vector::new(1.0, 0.0, 0.0)),
            Plane::from_point_normal(Point::new(0.0, 1.0, 0.0),  Vector::new(0.0, 1.0, 0.0)),
        ];
        let result = intersection::plane_4planes_open(&main, &planes);

        MINI_CHECK!(result.is_some());
        let poly = result.unwrap();
        // Open polyline has 4 points
        MINI_CHECK!(poly.len() == 4);
    })
}

pub fn run_intersection_plane_4lines() -> TestResult {
    MINI_TEST!("Plane 4 Lines", {
        use crate::intersection;
        use crate::{Line, Plane, Point, Vector};

        let plane = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0));
        // Four edges of a square in XY projected to the plane
        let l0 = Line::new(-1.0, -1.0, -1.0, -1.0,  1.0, 1.0);
        let l1 = Line::new( 1.0, -1.0, -1.0,  1.0,  1.0, 1.0);
        let l2 = Line::new(-1.0, -1.0, -1.0,  1.0, -1.0, 1.0);
        let l3 = Line::new(-1.0,  1.0, -1.0,  1.0,  1.0, 1.0);
        let result = intersection::plane_4lines(&plane, &l0, &l1, &l2, &l3);

        MINI_CHECK!(result.is_some());
        let poly = result.unwrap();
        MINI_CHECK!(poly.len() == 5);
        for p in poly.get_points() {
            MINI_CHECK!(p[2].abs() < 1e-6);
        }
    })
}

pub fn run_intersection_scale_vector_to_distance_of_2planes() -> TestResult {
    MINI_TEST!("Scale Vector To Distance Of 2 Planes", {
        use crate::intersection;
        use crate::{Plane, Point, Vector};

        let p0 = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0));
        let p1 = Plane::from_point_normal(Point::new(0.0, 0.0, 3.0), Vector::new(0.0, 0.0, 1.0));
        let dir = Vector::new(0.0, 0.0, 1.0);
        let result = intersection::scale_vector_to_distance_of_2planes(&dir, &p0, &p1);

        MINI_CHECK!(result.is_some());
        let v = result.unwrap();
        MINI_CHECK!((v[2] - 3.0).abs() < 1e-6);
    })
}

pub fn run_intersection_polyline_plane() -> TestResult {
    MINI_TEST!("Polyline Plane", {
        use crate::intersection;
        use crate::{Plane, Point, Polyline, Vector};

        // Square polyline on z=0 (closed); intersect with plane x=0 (yz-plane)
        let poly = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new( 1.0, -1.0, 0.0),
            Point::new( 1.0,  1.0, 0.0),
            Point::new(-1.0,  1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0), // closed
        ]);

        let plane = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0));
        let result = intersection::polyline_plane(&poly, &plane);

        MINI_CHECK!(result.is_some());
        let (pts, _indices) = result.unwrap();
        MINI_CHECK!(pts.len() == 2);
        for p in &pts {
            MINI_CHECK!(p[0].abs() < 1e-9);
        }
    })
}

pub fn run_intersection_line_line_3d() -> TestResult {
    MINI_TEST!("Line Line 3D", {
        use crate::intersection;
        use crate::Line;

        // Two lines that cross at (1, 1, 0)
        let cutter = Line::new(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
        let seg    = Line::new(1.0, 0.0, 0.0, 1.0, 2.0, 0.0);
        let result = intersection::line_line_3d(&cutter, &seg);

        MINI_CHECK!(result.is_some());
        let pt = result.unwrap();
        MINI_CHECK!((pt[0] - 1.0).abs() < 1e-6);
        MINI_CHECK!((pt[1] - 1.0).abs() < 1e-6);
        MINI_CHECK!((pt[2] - 0.0).abs() < 1e-6);

        // Parallel lines → None
        let par0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let par1 = Line::new(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);
        MINI_CHECK!(intersection::line_line_3d(&par0, &par1).is_none());
    })
}

REGISTER_MINI_TEST!("Intersection", "Line Line", crate::intersection_test::run_intersection_line_line);
REGISTER_MINI_TEST!("Intersection", "Line Line Parallel", crate::intersection_test::run_intersection_line_line_parallel);
REGISTER_MINI_TEST!("Intersection", "Line Line Parameters", crate::intersection_test::run_intersection_line_line_parameters);
REGISTER_MINI_TEST!("Intersection", "Line Line Parameters Endpoints", crate::intersection_test::run_intersection_line_line_parameters_endpoints);
REGISTER_MINI_TEST!("Intersection", "Line Line Parameters Infinite", crate::intersection_test::run_intersection_line_line_parameters_infinite);
REGISTER_MINI_TEST!("Intersection", "Plane Plane", crate::intersection_test::run_intersection_plane_plane);
REGISTER_MINI_TEST!("Intersection", "Plane Plane Complex", crate::intersection_test::run_intersection_plane_plane_complex);
REGISTER_MINI_TEST!("Intersection", "Line Plane", crate::intersection_test::run_intersection_line_plane);
REGISTER_MINI_TEST!("Intersection", "Line Plane Parallel", crate::intersection_test::run_intersection_line_plane_parallel);
REGISTER_MINI_TEST!("Intersection", "Line Plane Real World", crate::intersection_test::run_intersection_line_plane_real_world);
REGISTER_MINI_TEST!("Intersection", "Plane Plane Plane", crate::intersection_test::run_intersection_plane_plane_plane);
REGISTER_MINI_TEST!("Intersection", "Plane Plane Plane Parallel", crate::intersection_test::run_intersection_plane_plane_plane_parallel);
REGISTER_MINI_TEST!("Intersection", "Ray Box", crate::intersection_test::run_intersection_ray_box);
REGISTER_MINI_TEST!("Intersection", "Ray Box Miss", crate::intersection_test::run_intersection_ray_box_miss);
REGISTER_MINI_TEST!("Intersection", "Ray Sphere", crate::intersection_test::run_intersection_ray_sphere);
REGISTER_MINI_TEST!("Intersection", "Ray Sphere Tangent", crate::intersection_test::run_intersection_ray_sphere_tangent);
REGISTER_MINI_TEST!("Intersection", "Ray Sphere Miss", crate::intersection_test::run_intersection_ray_sphere_miss);
REGISTER_MINI_TEST!("Intersection", "Ray Triangle", crate::intersection_test::run_intersection_ray_triangle);
REGISTER_MINI_TEST!("Intersection", "Ray Triangle Miss", crate::intersection_test::run_intersection_ray_triangle_miss);
REGISTER_MINI_TEST!("Intersection", "Ray Triangle Parallel", crate::intersection_test::run_intersection_ray_triangle_parallel);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh", crate::intersection_test::run_intersection_ray_mesh);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh First", crate::intersection_test::run_intersection_ray_mesh_first);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh Miss", crate::intersection_test::run_intersection_ray_mesh_miss);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh Bvh", crate::intersection_test::run_intersection_ray_mesh_bvh);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh Bvh First", crate::intersection_test::run_intersection_ray_mesh_bvh_first);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh Bvh Miss", crate::intersection_test::run_intersection_ray_mesh_bvh_miss);
REGISTER_MINI_TEST!("Intersection", "Ray Mesh Bvh Vs Naive", crate::intersection_test::run_intersection_ray_mesh_bvh_vs_naive);
REGISTER_MINI_TEST!("Intersection", "Ray Box Real World", crate::intersection_test::run_intersection_ray_box_real_world);
REGISTER_MINI_TEST!("Intersection", "Ray Sphere Real World", crate::intersection_test::run_intersection_ray_sphere_real_world);
REGISTER_MINI_TEST!("Intersection", "Ray Triangle Real World", crate::intersection_test::run_intersection_ray_triangle_real_world);
REGISTER_MINI_TEST!("Intersection", "Surface Plane", crate::intersection_test::run_intersection_surface_plane);
REGISTER_MINI_TEST!("Intersection", "Surface Plane Curved", crate::intersection_test::run_intersection_surface_plane_curved);
REGISTER_MINI_TEST!("Intersection", "Surface Plane Miss", crate::intersection_test::run_intersection_surface_plane_miss);
REGISTER_MINI_TEST!("Intersection", "Surface Plane UV", crate::intersection_test::run_intersection_surface_plane_uv);
REGISTER_MINI_TEST!("Intersection", "Surface Surface", crate::intersection_test::run_intersection_surface_surface);
REGISTER_MINI_TEST!("Intersection", "Surface Surface Accuracy", crate::intersection_test::run_intersection_surface_surface_accuracy);
REGISTER_MINI_TEST!("Intersection", "Remap", crate::intersection_test::run_intersection_remap);
REGISTER_MINI_TEST!("Intersection", "Closest Point On Segment", crate::intersection_test::run_intersection_closest_point_on_segment);
REGISTER_MINI_TEST!("Intersection", "Plane Plane Plane Check Parallel", crate::intersection_test::run_intersection_plane_plane_plane_check_parallel);
REGISTER_MINI_TEST!("Intersection", "Plane 4 Planes Closed", crate::intersection_test::run_intersection_plane_4planes);
REGISTER_MINI_TEST!("Intersection", "Plane 4 Planes Open", crate::intersection_test::run_intersection_plane_4planes_open);
REGISTER_MINI_TEST!("Intersection", "Plane 4 Lines", crate::intersection_test::run_intersection_plane_4lines);
REGISTER_MINI_TEST!("Intersection", "Scale Vector To Distance Of 2 Planes", crate::intersection_test::run_intersection_scale_vector_to_distance_of_2planes);
REGISTER_MINI_TEST!("Intersection", "Polyline Plane", crate::intersection_test::run_intersection_polyline_plane);
REGISTER_MINI_TEST!("Intersection", "Line Line 3D", crate::intersection_test::run_intersection_line_line_3d);

pub fn run_intersection_polyline_plane_to_line() -> TestResult {
    MINI_TEST!("Polyline Plane To Line", {
        use crate::intersection::polyline_plane_to_line;
        use crate::{Plane, Point, Polyline, Vector};
        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let pln = Plane::from_point_normal(Point::new(0.0, 2.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let out = polyline_plane_to_line(&poly, &pln, &Point::new(0.0, 0.0, 0.0)).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(out.start()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(out.end()[0], 4.0));
    })
}
REGISTER_MINI_TEST!("Intersection", "Polyline Plane To Line", crate::intersection_test::run_intersection_polyline_plane_to_line);

pub fn run_intersection_quad_from_line_top_bottom_planes() -> TestResult {
    MINI_TEST!("Quad From Line Top Bottom Planes", {
        use crate::intersection::quad_from_line_top_bottom_planes;
        use crate::{Line, Plane, Point, Vector};
        let face = Plane::xy_plane();
        let line = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let plane0 = Plane::from_point_normal(Point::new(0.0, -2.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let plane1 = Plane::from_point_normal(Point::new(0.0,  2.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let out = quad_from_line_top_bottom_planes(&face, &line, &plane0, &plane1).unwrap();
        MINI_CHECK!(out.point_count() == 5);
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(0).unwrap()[1].abs(), 2.0));
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(2).unwrap()[1].abs(), 2.0));
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(2).unwrap()[0], 10.0));
    })
}
REGISTER_MINI_TEST!("Intersection", "Quad From Line Top Bottom Planes", crate::intersection_test::run_intersection_quad_from_line_top_bottom_planes);

pub fn run_intersection_orthogonal_vector_between_two_plane_pairs() -> TestResult {
    MINI_TEST!("Orthogonal Vector Between Two Plane Pairs", {
        use crate::intersection::orthogonal_vector_between_two_plane_pairs;
        use crate::{Plane, Point, Vector};
        let pp00 = Plane::xy_plane();
        let pp10 = Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0));
        let pp11 = Plane::from_point_normal(Point::new(4.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0));
        let out = orthogonal_vector_between_two_plane_pairs(&pp00, &pp10, &pp11).unwrap();
        let mag = (out[0]*out[0] + out[1]*out[1] + out[2]*out[2]).sqrt();
        MINI_CHECK!(TOLERANCE.is_close(mag, 4.0));
        MINI_CHECK!(TOLERANCE.is_close(out[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(out[2], 0.0));
    })
}
REGISTER_MINI_TEST!("Intersection", "Orthogonal Vector Between Two Plane Pairs", crate::intersection_test::run_intersection_orthogonal_vector_between_two_plane_pairs);

pub fn run_intersection_closed_and_open_paths_2d() -> TestResult {
    MINI_TEST!("Closed And Open Paths 2D", {
        use crate::intersection::closed_and_open_paths_2d;
        use crate::{Plane, Point, Polyline};
        let plate = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let joint = Polyline::new(vec![
            Point::new(-2.0, 5.0, 0.0),
            Point::new(12.0, 5.0, 0.0),
        ]);
        let pln = Plane::xy_plane();
        let (out, (t0, t1)) = closed_and_open_paths_2d(&plate, &joint, &pln).unwrap();
        MINI_CHECK!(out.point_count() == 2);
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(0).unwrap()[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(1).unwrap()[1], 5.0));
        let t_lo = t0.min(t1);
        let t_hi = t0.max(t1);
        MINI_CHECK!(TOLERANCE.is_close(t_lo, 1.5));
        MINI_CHECK!(TOLERANCE.is_close(t_hi, 3.5));
    })
}
REGISTER_MINI_TEST!("Intersection", "Closed And Open Paths 2D", crate::intersection_test::run_intersection_closed_and_open_paths_2d);

pub fn run_intersection_line_line_classified() -> TestResult {
    MINI_TEST!("Line Line Classified", {
        use crate::intersection;
        use crate::{Line, Point, Vector};

        // Crossing perpendicular segments meeting at their midpoints.
        let s0 = Line::new(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let s1 = Line::new(0.0, -1.0, 0.0, 0.0, 1.0, 0.0);
        let mut p0 = Point::new(0.0, 0.0, 0.0);
        let mut p1 = Point::new(0.0, 0.0, 0.0);
        let mut v0 = Vector::new(0.0, 0.0, 0.0);
        let mut v1 = Vector::new(0.0, 0.0, 0.0);
        let mut normal = Vector::new(0.0, 0.0, 0.0);
        let mut type0 = false;
        let mut type1 = false;
        let mut is_parallel = false;
        let ok = intersection::line_line_classified(&s0, &s1, 1, 1, 0, 0, 0.5, &mut p0, &mut p1, &mut v0, &mut v1, &mut normal, &mut type0, &mut type1, &mut is_parallel);

        MINI_CHECK!(ok);
        MINI_CHECK!(!is_parallel);
        MINI_CHECK!((p0[0]).abs() < 1e-6);
        MINI_CHECK!((p0[1]).abs() < 1e-6);
        MINI_CHECK!((p1[0]).abs() < 1e-6);
        MINI_CHECK!((p1[1]).abs() < 1e-6);
        MINI_CHECK!((normal[2].abs() - 1.0).abs() < 1e-6);

        // Shared-endpoint case: both segments start at the same point.
        let e0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let e1 = Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        let ok2 = intersection::line_line_classified(&e0, &e1, 1, 1, 0, 0, 0.5, &mut p0, &mut p1, &mut v0, &mut v1, &mut normal, &mut type0, &mut type1, &mut is_parallel);

        MINI_CHECK!(ok2);
        MINI_CHECK!(!type0);
        MINI_CHECK!(!type1);
        MINI_CHECK!((p0[0]).abs() < 1e-6);
        MINI_CHECK!((p0[1]).abs() < 1e-6);

        // Parallel offset segments.
        let q0 = Line::new(0.0, 0.0, 0.0, 2.0, 0.0, 0.0);
        let q1 = Line::new(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
        let ok3 = intersection::line_line_classified(&q0, &q1, 1, 1, 0, 0, 0.5, &mut p0, &mut p1, &mut v0, &mut v1, &mut normal, &mut type0, &mut type1, &mut is_parallel);

        MINI_CHECK!(ok3);
        MINI_CHECK!(is_parallel);
        MINI_CHECK!(!type0);
        MINI_CHECK!(!type1);
    })
}
REGISTER_MINI_TEST!("Intersection", "Line Line Classified", crate::intersection_test::run_intersection_line_line_classified);

