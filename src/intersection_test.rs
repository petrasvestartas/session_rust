use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

/// Worst 3D distance from the pcurve lifted onto the surface to the section curve.
fn lies_on_curve(
    curve3d: &crate::NurbsCurve,
    pcurve: &crate::NurbsCurve,
    surface: &crate::NurbsSurface,
) -> f64 {
    let (u0, u1) = surface.domain(0).unwrap();
    let (v0, v1) = surface.domain(1).unwrap();
    let mut dense: Vec<crate::Point> = Vec::with_capacity(129);

    for j in 0..129 {
        dense.push(curve3d.point_at(j as f64 / 128.0));
    }

    let mut worst = 0.0f64;

    for i in 0..33 {
        let q = pcurve.point_at(i as f64 / 32.0);
        let s = surface
            .point_at(q[0].max(u0).min(u1), q[1].max(v0).min(v1))
            .unwrap();

        let mut best = dense[0].distance(&s, None);

        for p in &dense {
            best = best.min(p.distance(&s, None));
        }

        worst = worst.max(best);
    }

    worst
}

/// Worst distance of the section curve from either analytic surface.
fn on_both(
    c3: &crate::NurbsCurve,
    da: fn(&crate::Point) -> f64,
    db: fn(&crate::Point) -> f64,
) -> f64 {
    let mut worst = 0.0f64;

    for i in 0..=64 {
        let p = c3.point_at(i as f64 / 64.0);
        worst = worst.max(da(&p).max(db(&p)));
    }

    worst
}

/// Distance from the radius-2 sphere at the origin.
fn distance_sphere(p: &crate::Point) -> f64 {
    ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - 2.0).abs()
}

/// Distance from the radius-0.3 cylinder on the z axis at x = 1.3.
fn distance_cylinder(p: &crate::Point) -> f64 {
    (((p[0] - 1.3) * (p[0] - 1.3) + p[1] * p[1]).sqrt() - 0.3).abs()
}

/// Distance from the radius-2 sphere at x = 2.
fn distance_sphere2(p: &crate::Point) -> f64 {
    (((p[0] - 2.0) * (p[0] - 2.0) + p[1] * p[1] + p[2] * p[2]).sqrt() - 2.0).abs()
}

/// Distance from the torus of radii 2 and 0.5 at the origin.
fn distance_torus(p: &crate::Point) -> f64 {
    let ring = (p[0] * p[0] + p[1] * p[1]).sqrt() - 2.0;

    ((ring * ring + p[2] * p[2]).sqrt() - 0.5).abs()
}

/// Distance from the xy plane.
fn distance_flat(p: &crate::Point) -> f64 {
    p[2].abs()
}

/// Planar degree-1 surface through four corner points.
fn bilinear(
    p00: crate::Point,
    p01: crate::Point,
    p10: crate::Point,
    p11: crate::Point,
) -> crate::NurbsSurface {
    crate::NurbsSurface::create(false, false, 1, 1, 2, 2, &[p00, p01, p10, p11]).unwrap()
}

/// Worst distance of the pcurve lifted onto the surface from a reference shape.
fn lifted_distance(
    pcurve: &crate::NurbsCurve,
    surface: &crate::NurbsSurface,
    d: fn(&crate::Point) -> f64,
) -> f64 {
    let (t0, t1) = pcurve.domain();
    let mut worst = 0.0f64;

    for i in 0..=32 {
        let uv = pcurve.point_at(t0 + (t1 - t0) * i as f64 / 32.0);
        worst = worst.max(d(&surface.point_at(uv[0], uv[1]).unwrap()));
    }

    worst
}

/// Distance from the cone of base radius 1.5 at z = 0 and apex at z = 3.
fn distance_cone(p: &crate::Point) -> f64 {
    ((p[0] * p[0] + p[1] * p[1]).sqrt() - (3.0 - p[2]) * 0.5).abs()
}

/// Distance from the plane z = 0.5.
fn distance_flat_half(p: &crate::Point) -> f64 {
    (p[2] - 0.5).abs()
}

/// Distance from the plane x = 0.2.
fn distance_wall(p: &crate::Point) -> f64 {
    (p[0] - 0.2).abs()
}

/// Distance from the radius-1 cylinder on the z axis.
fn distance_unit_cylinder(p: &crate::Point) -> f64 {
    ((p[0] * p[0] + p[1] * p[1]).sqrt() - 1.0).abs()
}

/// Distance from the radius-1 cylinder on the x axis.
fn distance_x_cylinder(p: &crate::Point) -> f64 {
    ((p[1] * p[1] + p[2] * p[2]).sqrt() - 1.0).abs()
}

/// Distance from the radius-2.2 cylinder on the z axis.
fn distance_wide_cylinder(p: &crate::Point) -> f64 {
    ((p[0] * p[0] + p[1] * p[1]).sqrt() - 2.2).abs()
}

/// Distance from the torus of radii 1 and 0.3 at z = 1.
fn distance_high_torus(p: &crate::Point) -> f64 {
    let ring = (p[0] * p[0] + p[1] * p[1]).sqrt() - 1.0;

    ((ring * ring + (p[2] - 1.0) * (p[2] - 1.0)).sqrt() - 0.3).abs()
}

/// Distance from the torus of radii 2.3 and 0.5 at z = 0.3.
fn distance_wide_torus(p: &crate::Point) -> f64 {
    let ring = (p[0] * p[0] + p[1] * p[1]).sqrt() - 2.3;

    ((ring * ring + (p[2] - 0.3) * (p[2] - 0.3)).sqrt() - 0.5).abs()
}

/// Distance from the square of half size 1.6 at z = 0.5.
fn distance_square(p: &crate::Point) -> f64 {
    (p[2] - 0.5).abs().max(
        0.0f64
            .max(p[0].abs() - 1.6)
            .max(0.0f64.max(p[1].abs() - 1.6)),
    )
}

/// Distance from the plane through the slanted cutter.
fn distance_slanted(p: &crate::Point) -> f64 {
    (-2.0 * p[0] + p[1] + 10.0 * p[2] - 3.0).abs() / 105.0f64.sqrt()
}

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
        let result = intersection::line_line_parameters(
            &line0,
            &line1,
            Tolerance::APPROXIMATION,
            true,
            false,
        );

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
        let result = intersection::line_line_parameters(
            &line0,
            &line1,
            Tolerance::APPROXIMATION,
            true,
            false,
        );

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
        let result = intersection::line_line_parameters(
            &line0,
            &line1,
            Tolerance::APPROXIMATION,
            false,
            false,
        );

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
        let plane0 = Plane::from_point_normal(p0, n0, None);

        let p1 = Point::new(0.0, 0.0, 0.0);
        let n1 = Vector::new(0.0, 1.0, 0.0);
        let plane1 = Plane::from_point_normal(p1, n1, None);

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

pub fn run_intersection_plane_plane_to_line_canonical() -> TestResult {
    MINI_TEST!("Plane Plane To Line Canonical", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let p0 = Point::new(0.0, 0.0, 2.0);
        let n0 = Vector::new(0.0, 0.0, 1.0);
        let plane0 = Plane::from_point_normal(p0, n0.clone(), None);

        let p1 = Point::new(3.0, 0.0, 0.0);
        let n1 = Vector::new(1.0, 0.0, 0.0);
        let plane1 = Plane::from_point_normal(p1, n1, None);

        let output = intersection::plane_plane_to_line_canonical(&plane0, &plane1);

        MINI_CHECK!(output.is_some());

        let output = output.unwrap();

        MINI_CHECK!(TOLERANCE.is_close(output.start()[0], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(output.start()[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(output.start()[2], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(output.end()[1], -1.0));

        let p2 = Point::new(0.0, 0.0, 5.0);
        let plane2 = Plane::from_point_normal(p2, n0, None);

        MINI_CHECK!(intersection::plane_plane_to_line_canonical(&plane0, &plane2).is_none());
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
        let plane = Plane::from_point_normal(p, n, None);

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
        let plane = Plane::from_point_normal(p, n, None);

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
        let plane0 = Plane::from_point_normal(p0, n0, None);

        let p1 = Point::new(0.0, 0.0, 1.0);
        let n1 = Vector::new(0.0, 0.0, 1.0);
        let plane1 = Plane::from_point_normal(p1, n1, None);

        let p2 = Point::new(0.0, 0.0, 0.0);
        let n2 = Vector::new(1.0, 0.0, 0.0);
        let plane2 = Plane::from_point_normal(p2, n2, None);

        let output = intersection::plane_plane_plane(&plane0, &plane1, &plane2);

        MINI_CHECK!(output.is_none());
    })
}

pub fn run_intersection_ray_box() -> TestResult {
    MINI_TEST!("Ray Box", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;
        use crate::OBB;

        let center = Point::new(0.0, 0.0, 0.0);
        let x_axis = Vector::new(1.0, 0.0, 0.0);
        let y_axis = Vector::new(0.0, 1.0, 0.0);
        let z_axis = Vector::new(0.0, 0.0, 1.0);
        let half_size = Vector::new(1.0, 1.0, 1.0);
        let box_ = OBB::new(center, x_axis, y_axis, z_axis, half_size);

        let origin = Point::new(-5.0, 0.0, 0.0);
        let direction = Vector::new(1.0, 0.0, 0.0);

        let (result, tmin, tmax) =
            intersection::ray_box_parameters(&origin, &direction, &box_, 0.0, 100.0);

        MINI_CHECK!(result);
        MINI_CHECK!((tmin - 4.0).abs() < 1e-4);
        MINI_CHECK!((tmax - 6.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_box_miss() -> TestResult {
    MINI_TEST!("Ray Box Miss", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;
        use crate::OBB;

        let center = Point::new(0.0, 0.0, 0.0);
        let x_axis = Vector::new(1.0, 0.0, 0.0);
        let y_axis = Vector::new(0.0, 1.0, 0.0);
        let z_axis = Vector::new(0.0, 0.0, 1.0);
        let half_size = Vector::new(1.0, 1.0, 1.0);
        let box_ = OBB::new(center, x_axis, y_axis, z_axis, half_size);

        let origin = Point::new(-5.0, 5.0, 0.0);
        let direction = Vector::new(1.0, 0.0, 0.0);

        let (result, _tmin, _tmax) =
            intersection::ray_box_parameters(&origin, &direction, &box_, 0.0, 100.0);

        MINI_CHECK!(!result);
    })
}

pub fn run_intersection_ray_sphere() -> TestResult {
    MINI_TEST!("Ray Sphere", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;

        let origin = Point::new(-5.0, 0.0, 0.0);
        let direction = Vector::new(1.0, 0.0, 0.0);
        let center = Point::new(0.0, 0.0, 0.0);
        let radius = 2.0;

        let (hits, t0, t1) =
            intersection::ray_sphere_parameters(&origin, &direction, &center, radius);

        MINI_CHECK!(hits == 2);
        MINI_CHECK!((t0 - 3.0).abs() < 1e-4);
        MINI_CHECK!((t1 - 7.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_sphere_tangent() -> TestResult {
    MINI_TEST!("Ray Sphere Tangent", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;

        let origin = Point::new(-5.0, 2.0, 0.0);
        let direction = Vector::new(1.0, 0.0, 0.0);
        let center = Point::new(0.0, 0.0, 0.0);
        let radius = 2.0;

        let (hits, t0, _t1) =
            intersection::ray_sphere_parameters(&origin, &direction, &center, radius);

        MINI_CHECK!(hits == 1);
        MINI_CHECK!((t0 - 5.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_sphere_miss() -> TestResult {
    MINI_TEST!("Ray Sphere Miss", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;

        let origin = Point::new(-5.0, 5.0, 0.0);
        let direction = Vector::new(1.0, 0.0, 0.0);
        let center = Point::new(0.0, 0.0, 0.0);
        let radius = 2.0;

        let (hits, _t0, _t1) =
            intersection::ray_sphere_parameters(&origin, &direction, &center, radius);

        MINI_CHECK!(hits == 0);
    })
}

pub fn run_intersection_ray_triangle() -> TestResult {
    MINI_TEST!("Ray Triangle", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;

        let origin = Point::new(0.5, 0.5, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let v0 = Point::new(0.0, 0.0, 0.0);
        let v1 = Point::new(1.0, 0.0, 0.0);
        let v2 = Point::new(0.0, 1.0, 0.0);

        let (result, t, _u, _v, parallel) =
            intersection::ray_triangle_parameters(&origin, &direction, &v0, &v1, &v2, 1e-6);

        MINI_CHECK!(result);
        MINI_CHECK!(!parallel);
        MINI_CHECK!((t - 1.0).abs() < 1e-4);
    })
}

pub fn run_intersection_ray_triangle_miss() -> TestResult {
    MINI_TEST!("Ray Triangle Miss", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;

        let origin = Point::new(2.0, 2.0, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let v0 = Point::new(0.0, 0.0, 0.0);
        let v1 = Point::new(1.0, 0.0, 0.0);
        let v2 = Point::new(0.0, 1.0, 0.0);

        let (result, _t, _u, _v, _parallel) =
            intersection::ray_triangle_parameters(&origin, &direction, &v0, &v1, &v2, 1e-6);

        MINI_CHECK!(!result);
    })
}

pub fn run_intersection_ray_triangle_parallel() -> TestResult {
    MINI_TEST!("Ray Triangle Parallel", {
        use crate::intersection;
        use crate::Point;
        use crate::Vector;

        let origin = Point::new(0.5, 0.5, -1.0);
        let direction = Vector::new(1.0, 0.0, 0.0);

        let v0 = Point::new(0.0, 0.0, 0.0);
        let v1 = Point::new(1.0, 0.0, 0.0);
        let v2 = Point::new(0.0, 1.0, 0.0);

        let (result, _t, _u, _v, parallel) =
            intersection::ray_triangle_parameters(&origin, &direction, &v0, &v1, &v2, 1e-6);

        MINI_CHECK!(!result);
        MINI_CHECK!(parallel);
    })
}

pub fn run_intersection_ray_mesh() -> TestResult {
    MINI_TEST!("Ray Mesh", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

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

        let origin = Point::new(0.5, 0.5, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result, hits) = intersection::ray_mesh_hits(
            &origin,
            &direction,
            &mesh,
            true,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(result);
        MINI_CHECK!(!hits.is_empty());
        MINI_CHECK!((hits[0].t - 1.0).abs() < 1e-3);
    })
}

pub fn run_intersection_ray_mesh_first() -> TestResult {
    MINI_TEST!("Ray Mesh First", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

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

        let origin = Point::new(0.5, 0.5, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result, hits) = intersection::ray_mesh_hits(
            &origin,
            &direction,
            &mesh,
            false,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(result);
        MINI_CHECK!(hits.len() == 1);
    })
}

pub fn run_intersection_ray_mesh_miss() -> TestResult {
    MINI_TEST!("Ray Mesh Miss", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

        let polygons: Vec<Vec<Point>> = vec![vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]];

        let mesh = Mesh::from_polylines(polygons, None);

        let origin = Point::new(5.0, 5.0, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result, hits) = intersection::ray_mesh_hits(
            &origin,
            &direction,
            &mesh,
            true,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(!result);
        MINI_CHECK!(hits.is_empty());
    })
}

pub fn run_intersection_ray_mesh_bvh() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

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

        let mut mesh = Mesh::from_polylines(polygons, None);

        let origin = Point::new(0.5, 0.5, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result, hits) = intersection::ray_mesh_bvh_hits(
            &origin,
            &direction,
            &mut mesh,
            true,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(result);
        MINI_CHECK!(!hits.is_empty());
        MINI_CHECK!((hits[0].t - 1.0).abs() < 1e-3);
    })
}

pub fn run_intersection_ray_mesh_bvh_first() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh First", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

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

        let mut mesh = Mesh::from_polylines(polygons, None);

        let origin = Point::new(0.5, 0.5, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result, hits) = intersection::ray_mesh_bvh_hits(
            &origin,
            &direction,
            &mut mesh,
            false,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(result);
        MINI_CHECK!(hits.len() == 1);
    })
}

pub fn run_intersection_ray_mesh_bvh_miss() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh Miss", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

        let polygons: Vec<Vec<Point>> = vec![vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]];

        let mut mesh = Mesh::from_polylines(polygons, None);

        let origin = Point::new(5.0, 5.0, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result, hits) = intersection::ray_mesh_bvh_hits(
            &origin,
            &direction,
            &mut mesh,
            true,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(!result);
        MINI_CHECK!(hits.is_empty());
    })
}

pub fn run_intersection_ray_mesh_bvh_vs_naive() -> TestResult {
    MINI_TEST!("Ray Mesh Bvh Vs Naive", {
        use crate::intersection;
        use crate::Mesh;
        use crate::Point;
        use crate::Tolerance;
        use crate::Vector;

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

        let mut mesh = Mesh::from_polylines(polygons, None);

        let origin = Point::new(5.5, 5.5, -1.0);
        let direction = Vector::new(0.0, 0.0, 1.0);

        let (result_naive, hits_naive) = intersection::ray_mesh_hits(
            &origin,
            &direction,
            &mesh,
            true,
            Tolerance::ZERO_TOLERANCE,
        );

        let (result_bvh, hits_bvh) = intersection::ray_mesh_bvh_hits(
            &origin,
            &direction,
            &mut mesh,
            true,
            Tolerance::ZERO_TOLERANCE,
        );

        MINI_CHECK!(result_naive == result_bvh);
        MINI_CHECK!(hits_naive.len() == hits_bvh.len());

        if !hits_naive.is_empty() {
            MINI_CHECK!((hits_naive[0].t - hits_bvh[0].t).abs() < 1e-4);
            MINI_CHECK!(hits_naive[0].face_index == hits_bvh[0].face_index);
        }
    })
}

pub fn run_intersection_ray_box_real_world() -> TestResult {
    MINI_TEST!("Ray Box Real World", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;
        use crate::OBB;

        let l0 = Line::new(500.0, -573.576, -819.152, 500.0, 573.576, 819.152);
        let min_pt = Point::new(214.0, 192.0, 484.0);
        let max_pt = Point::new(694.0, 567.0, 796.0);
        let box_ = OBB::from_points(&[min_pt, max_pt], 0.0, None);
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

pub fn run_intersection_curve_plane() -> TestResult {
    MINI_TEST!("Curve Plane", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let origin = Point::new(1.0, 0.0, 0.0);
        let normal = Vector::new(1.0, 0.0, 0.0);
        let plane = Plane::from_point_normal(origin, normal, None);
        let params = intersection::curve_plane(&circle, &plane, None);
        let points = intersection::curve_plane_points(&circle, &plane, None);

        MINI_CHECK!(params.len() == 2);
        MINI_CHECK!(points.len() == 2);

        for p in &points {
            MINI_CHECK!((p[0] - 1.0).abs() < 1e-9);
            MINI_CHECK!((p[1].abs() - 3.0_f64.sqrt()).abs() < 1e-9);
        }

        let offset = 1.98 / 2.0_f64.sqrt();
        let diagonal = Plane::from_point_normal(
            Point::new(offset, offset, 0.0),
            Vector::new(1.0, 1.0, 0.0),
            None,
        );
        let hidden = intersection::curve_plane(&circle, &diagonal, None);

        MINI_CHECK!(hidden.len() == 2);
    })
}

pub fn run_intersection_curve_plane_bezier_clipping() -> TestResult {
    MINI_TEST!("Curve Plane Bezier Clipping", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let origin = Point::new(1.0, 0.0, 0.0);
        let normal = Vector::new(1.0, 0.0, 0.0);
        let plane = Plane::from_point_normal(origin, normal, None);
        let params = intersection::curve_plane_bezier_clipping(&circle, &plane, None);

        MINI_CHECK!(params.len() == 2);

        for t in &params {
            MINI_CHECK!((circle.point_at(*t)[0] - 1.0).abs() < 1e-9);
        }

        let unit = Primitives::circle(0.0, 0.0, 0.0, 1.0);
        let tilted =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.2), Vector::new(0.3, 0.1, 1.0), None);
        let roots = intersection::curve_plane_bezier_clipping(&unit, &tilted, None);

        MINI_CHECK!(roots.len() == 2);
    })
}

pub fn run_intersection_curve_plane_algebraic() -> TestResult {
    MINI_TEST!("Curve Plane Algebraic", {
        use crate::intersection;
        use crate::NurbsCurve;
        use crate::Plane;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let curve = NurbsCurve::create(
            false,
            3,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 2.0, 0.0),
                Point::new(2.0, 2.0, 0.0),
                Point::new(3.0, 0.0, 0.0),
            ],
        );

        let origin = Point::new(1.0, 0.0, 0.0);
        let normal = Vector::new(1.0, 0.0, 0.0);
        let plane = Plane::from_point_normal(origin, normal, None);
        let params = intersection::curve_plane_algebraic(&curve, &plane, None);

        MINI_CHECK!(params.len() == 1);
        MINI_CHECK!((curve.point_at(params[0])[0] - 1.0).abs() < 1e-9);
        MINI_CHECK!((curve.point_at(params[0])[1] - 4.0 / 3.0).abs() < 1e-9);

        let circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let circle_params = intersection::curve_plane_algebraic(&circle, &plane, None);

        MINI_CHECK!(circle_params.len() == 2);
    })
}

pub fn run_intersection_curve_plane_production() -> TestResult {
    MINI_TEST!("Curve Plane Production", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let origin = Point::new(1.0, 0.0, 0.0);
        let normal = Vector::new(1.0, 0.0, 0.0);
        let plane = Plane::from_point_normal(origin, normal, None);
        let params = intersection::curve_plane_production(&circle, &plane, None);

        MINI_CHECK!(params.len() == 2);

        for t in &params {
            MINI_CHECK!((circle.point_at(*t)[0] - 1.0).abs() < 1e-9);
        }
    })
}

pub fn run_intersection_curve_closest_point() -> TestResult {
    MINI_TEST!("Curve Closest Point", {
        use crate::intersection;
        use crate::Point;
        use crate::Primitives;

        let circle = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let test_point = Point::new(3.0, 0.0, 0.0);
        let result = intersection::curve_closest_point(&circle, &test_point, 0.0, 0.0);
        let closest = circle.point_at(result.0);

        MINI_CHECK!((result.1 - 1.0).abs() < 1e-6);
        MINI_CHECK!((closest[0] - 2.0).abs() < 1e-6);
        MINI_CHECK!(closest[1].abs() < 1e-6);
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
        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 5.0), Vector::new(0.0, 0.0, 1.0), None);
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
                let z = if (i == 1 || i == 2) && (j == 1 || j == 2) {
                    10.0
                } else {
                    0.0
                };
                pts.push(Point::new(x, y, z));
            }
        }

        let srf = NurbsSurface::create(false, false, 3, 3, 4, 4, &pts).unwrap();
        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 3.0), Vector::new(0.0, 0.0, 1.0), None);
        let curves = intersection::surface_plane(&srf, &plane, None);

        MINI_CHECK!(!curves.is_empty());
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
        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 5.0), Vector::new(0.0, 0.0, 1.0), None);
        let curves = intersection::surface_plane(&srf, &plane, None);

        MINI_CHECK!(curves.is_empty());
    })
}

pub fn run_intersection_surface_plane_uv() -> TestResult {
    MINI_TEST!("Surface Plane UV", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Primitives;
        use crate::Vector;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 4.0);
        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 2.0), Vector::new(0.3, 0.0, 1.0), None);
        let pairs = intersection::surface_plane_uv(&cyl, &plane, None);

        MINI_CHECK!(pairs.len() == 1);

        let curve3 = &pairs[0].0;
        let pcurve = &pairs[0].1;

        MINI_CHECK!(curve3.is_valid());
        MINI_CHECK!(pcurve.is_valid());
        MINI_CHECK!(curve3.is_closed());

        let (u0, u1) = cyl.domain(0).unwrap();

        MINI_CHECK!(
            (pcurve.point_at(0.0)[0] - u1).abs() < 1e-9
                || (pcurve.point_at(0.0)[0] - u0).abs() < 1e-9
        );
        MINI_CHECK!(
            (pcurve.point_at(1.0)[0] - u1).abs() < 1e-9
                || (pcurve.point_at(1.0)[0] - u0).abs() < 1e-9
        );

        let pn = plane.z_axis();
        let po = plane.origin();
        let mut max_off = 0.0f64;

        for i in 0..17 {
            let p2 = pcurve.point_at(i as f64 / 16.0);
            let s = cyl.point_at(p2[0], p2[1]).unwrap();
            let off =
                ((s[0] - po[0]) * pn[0] + (s[1] - po[1]) * pn[1] + (s[2] - po[2]) * pn[2]).abs();
            max_off = max_off.max(off);
        }

        MINI_CHECK!(max_off < 0.05);

        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let plane2 =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0), None);
        let pairs2 = intersection::surface_plane_uv(&torus, &plane2, None);

        MINI_CHECK!(pairs2.len() == 2);

        let (tu0, tu1) = torus.domain(0).unwrap();
        let (tv0, tv1) = torus.domain(1).unwrap();
        let mut inside = true;

        for pair in &pairs2 {
            for i in 0..17 {
                let p2 = pair.1.point_at(i as f64 / 16.0);

                if p2[0] < tu0 - 1e-6
                    || p2[0] > tu1 + 1e-6
                    || p2[1] < tv0 - 1e-6
                    || p2[1] > tv1 + 1e-6
                {
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
        use crate::Primitives;

        let flat = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(-3.0, -3.0, 0.5),
                Point::new(-3.0, 3.0, 0.5),
                Point::new(3.0, -3.0, 0.5),
                Point::new(3.0, 3.0, 0.5),
            ],
        )
        .unwrap();
        let cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
        let flat_triples = intersection::surface_surface(&flat, &cyl, None);

        MINI_CHECK!(flat_triples.len() == 1);

        let (c3, pa, pb) = &flat_triples[0];

        MINI_CHECK!(c3.is_valid() && pa.is_valid() && pb.is_valid());
        MINI_CHECK!(c3.is_closed());
        MINI_CHECK!(lies_on_curve(c3, pa, &flat) < 0.05);
        MINI_CHECK!(lies_on_curve(c3, pb, &cyl) < 0.05);

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

        let sphere2 = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);

        let flat04 = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(-3.0, -3.0, 0.4),
                Point::new(-3.0, 3.0, 0.4),
                Point::new(3.0, -3.0, 0.4),
                Point::new(3.0, 3.0, 0.4),
            ],
        )
        .unwrap();

        let ex_triples = intersection::surface_surface(&sphere2, &flat04, None);

        MINI_CHECK!(ex_triples.len() == 1);

        let ex_c3 = &ex_triples[0].0;
        let expected_r = 3.84f64.sqrt();
        let mut max_dev = 0.0f64;

        for j in 0..=256 {
            let p = ex_c3.point_at(j as f64 / 256.0);
            let rr = (p[0] * p[0] + p[1] * p[1]).sqrt();
            max_dev = max_dev.max((rr - expected_r).abs());
            max_dev = max_dev.max((p[2] - 0.4).abs());
        }

        MINI_CHECK!(max_dev < 1e-9);
    })
}

pub fn run_intersection_surface_surface_accuracy() -> TestResult {
    MINI_TEST!("Surface Surface Accuracy", {
        use crate::intersection::surface_surface;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Primitives;

        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let cyl = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
        let tr = surface_surface(&sphere, &cyl, None);

        MINI_CHECK!(tr.len() >= 2);

        for (c3, _pa, _pb) in &tr {
            MINI_CHECK!(on_both(c3, distance_sphere, distance_cylinder) < 1e-5);
        }

        let sphere2 = Primitives::sphere_surface(2.0, 0.0, 0.0, 2.0);
        let tr2 = surface_surface(&sphere, &sphere2, None);

        MINI_CHECK!(!tr2.is_empty());

        for (c3, _pa, _pb) in &tr2 {
            MINI_CHECK!(on_both(c3, distance_sphere, distance_sphere2) < 1e-6);
        }

        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let flat = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(-9.0, -9.0, 0.0),
                Point::new(-9.0, 9.0, 0.0),
                Point::new(9.0, -9.0, 0.0),
                Point::new(9.0, 9.0, 0.0),
            ],
        )
        .unwrap();
        let tr3 = surface_surface(&torus, &flat, None);

        MINI_CHECK!(tr3.len() == 2);

        for (c3, _pa, _pb) in &tr3 {
            MINI_CHECK!(on_both(c3, distance_torus, distance_flat) < 1e-6);
        }
    })
}

pub fn run_intersection_surface_surface_planes() -> TestResult {
    MINI_TEST!("Surface Surface Planes", {
        use crate::intersection::surface_surface;
        use crate::Point;

        let flat = bilinear(
            Point::new(-3.0, -3.0, 0.5),
            Point::new(-3.0, 3.0, 0.5),
            Point::new(3.0, -3.0, 0.5),
            Point::new(3.0, 3.0, 0.5),
        );
        let wall = bilinear(
            Point::new(0.2, -3.0, -3.0),
            Point::new(0.2, -3.0, 3.0),
            Point::new(0.2, 3.0, -3.0),
            Point::new(0.2, 3.0, 3.0),
        );
        let far = bilinear(
            Point::new(5.0, -3.0, -3.0),
            Point::new(5.0, -3.0, 3.0),
            Point::new(5.0, 3.0, -3.0),
            Point::new(5.0, 3.0, 3.0),
        );
        let tr = surface_surface(&flat, &wall, None);

        MINI_CHECK!(tr.len() == 1);

        let c3 = &tr[0].0;
        let start = c3.point_at_start();
        let end = c3.point_at_end();

        MINI_CHECK!(
            TOLERANCE.is_close(start[0], 0.2)
                && TOLERANCE.is_close(start[1], -3.0)
                && TOLERANCE.is_close(start[2], 0.5)
        );
        MINI_CHECK!(
            TOLERANCE.is_close(end[0], 0.2)
                && TOLERANCE.is_close(end[1], 3.0)
                && TOLERANCE.is_close(end[2], 0.5)
        );
        MINI_CHECK!(lies_on_curve(c3, &tr[0].1, &flat) < 1e-9);
        MINI_CHECK!(lies_on_curve(c3, &tr[0].2, &wall) < 1e-9);
        MINI_CHECK!(surface_surface(&flat, &far, None).is_empty());
    })
}

pub fn run_intersection_surface_surface_plane_cone() -> TestResult {
    MINI_TEST!("Surface Surface Plane Cone", {
        use crate::intersection::surface_surface;
        use crate::Point;
        use crate::Primitives;

        let cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
        let flat = bilinear(
            Point::new(-3.0, -3.0, 0.5),
            Point::new(-3.0, 3.0, 0.5),
            Point::new(3.0, -3.0, 0.5),
            Point::new(3.0, 3.0, 0.5),
        );
        let steep = bilinear(
            Point::new(1.1, -3.0, -3.0),
            Point::new(1.1, 3.0, -3.0),
            Point::new(-0.3, -3.0, 4.0),
            Point::new(-0.3, 3.0, 4.0),
        );
        let slant = bilinear(
            Point::new(-3.0, -3.0, 8.5),
            Point::new(-3.0, 3.0, 8.5),
            Point::new(3.0, -3.0, -3.5),
            Point::new(3.0, 3.0, -3.5),
        );
        let axial = bilinear(
            Point::new(0.0, -3.0, -3.0),
            Point::new(0.0, -3.0, 4.0),
            Point::new(0.0, 3.0, -3.0),
            Point::new(0.0, 3.0, 4.0),
        );
        let circle = surface_surface(&cone, &flat, None);

        MINI_CHECK!(circle.len() == 1);
        MINI_CHECK!(on_both(&circle[0].0, distance_cone, distance_flat_half) < 1e-9);
        MINI_CHECK!(lies_on_curve(&circle[0].0, &circle[0].1, &cone) < 1e-9);

        let hyperbola = surface_surface(&cone, &steep, None);

        MINI_CHECK!(hyperbola.len() == 1);
        MINI_CHECK!(hyperbola[0].0.degree() == 2);
        MINI_CHECK!(on_both(&hyperbola[0].0, distance_cone, distance_cone) < 1e-9);

        let parabola = surface_surface(&cone, &slant, None);

        MINI_CHECK!(parabola.len() == 1);
        MINI_CHECK!(on_both(&parabola[0].0, distance_cone, distance_cone) < 1e-6);

        let lines = surface_surface(&cone, &axial, None);

        MINI_CHECK!(lines.len() == 2);

        for line in &lines {
            let apex = line.0.point_at_start();

            MINI_CHECK!(
                TOLERANCE.is_close(apex[0], 0.0)
                    && TOLERANCE.is_close(apex[1], 0.0)
                    && TOLERANCE.is_close(apex[2], 3.0)
            );
            MINI_CHECK!(on_both(&line.0, distance_cone, distance_cone) < 1e-9);
        }
    })
}

pub fn run_intersection_surface_surface_plane_torus() -> TestResult {
    MINI_TEST!("Surface Surface Plane Torus", {
        use crate::intersection::surface_surface;
        use crate::Point;
        use crate::Primitives;

        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let wall = bilinear(
            Point::new(0.2, -3.0, -3.0),
            Point::new(0.2, -3.0, 3.0),
            Point::new(0.2, 3.0, -3.0),
            Point::new(0.2, 3.0, 3.0),
        );
        let tr = surface_surface(&torus, &wall, None);

        MINI_CHECK!(tr.len() == 2);

        for t in &tr {
            MINI_CHECK!(t.0.is_closed());
            MINI_CHECK!(on_both(&t.0, distance_torus, distance_wall) < 1e-4);
            MINI_CHECK!(lies_on_curve(&t.0, &t.2, &wall) < 1e-4);
        }
    })
}

pub fn run_intersection_surface_surface_cylinders() -> TestResult {
    MINI_TEST!("Surface Surface Cylinders", {
        use crate::intersection::surface_surface;
        use crate::Primitives;
        use crate::Tolerance;
        use crate::Vector;
        use crate::Xform;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
        let beside = Primitives::cylinder_surface(1.5, 0.0, -2.0, 1.0, 4.0);
        let across = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0).transformed(
            &Xform::rotation(&Vector::new(0.0, 1.0, 0.0), Tolerance::HALF_PI, false),
        );
        let lines = surface_surface(&cyl, &beside, None);

        MINI_CHECK!(lines.len() == 2);

        for line in &lines {
            let start = line.0.point_at_start();
            let end = line.0.point_at_end();

            MINI_CHECK!(TOLERANCE.is_close(start[0], 0.75) && TOLERANCE.is_close(end[0], 0.75));
            MINI_CHECK!(
                TOLERANCE.is_close(start[1].abs(), 0.4375f64.sqrt())
                    && TOLERANCE.is_close(start[1], end[1])
            );
            MINI_CHECK!(lies_on_curve(&line.0, &line.1, &cyl) < 1e-9);
        }

        let ellipses = surface_surface(&cyl, &across, None);

        MINI_CHECK!(ellipses.len() == 2);

        for ellipse in &ellipses {
            MINI_CHECK!(ellipse.0.is_closed());
            MINI_CHECK!(on_both(&ellipse.0, distance_unit_cylinder, distance_x_cylinder) < 1e-9);
        }
    })
}

pub fn run_intersection_surface_surface_coaxial_quadrics() -> TestResult {
    MINI_TEST!("Surface Surface Coaxial Quadrics", {
        use crate::intersection::surface_surface;
        use crate::Primitives;

        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
        let cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
        let sphere_cyl = surface_surface(&sphere, &cyl, None);

        MINI_CHECK!(sphere_cyl.len() == 2);

        for t in &sphere_cyl {
            MINI_CHECK!(TOLERANCE.is_close(t.0.point_at_start()[2].abs(), 3.0f64.sqrt()));
            MINI_CHECK!(on_both(&t.0, distance_sphere, distance_unit_cylinder) < 1e-9);
            MINI_CHECK!(lies_on_curve(&t.0, &t.1, &sphere) < 1e-9);
            MINI_CHECK!(lies_on_curve(&t.0, &t.2, &cyl) < 1e-9);
        }

        let cyl_cone = surface_surface(&cyl, &cone, None);

        MINI_CHECK!(cyl_cone.len() == 1);
        MINI_CHECK!(TOLERANCE.is_close(cyl_cone[0].0.point_at_start()[2], 1.0));
        MINI_CHECK!(on_both(&cyl_cone[0].0, distance_unit_cylinder, distance_cone) < 1e-9);
        MINI_CHECK!(lies_on_curve(&cyl_cone[0].0, &cyl_cone[0].2, &cone) < 1e-9);

        let cone_sphere = surface_surface(&cone, &sphere, None);

        MINI_CHECK!(cone_sphere.len() == 2);

        for t in &cone_sphere {
            MINI_CHECK!(on_both(&t.0, distance_cone, distance_sphere) < 1e-9);
        }
    })
}

pub fn run_intersection_surface_surface_coaxial_tori() -> TestResult {
    MINI_TEST!("Surface Surface Coaxial Tori", {
        use crate::intersection::surface_surface;
        use crate::Primitives;

        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let wide_cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 2.2, 4.0);
        let cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
        let high_torus = Primitives::torus_surface(0.0, 0.0, 1.0, 1.0, 0.3);
        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let wide_torus = Primitives::torus_surface(0.0, 0.0, 0.3, 2.3, 0.5);
        let cyl_torus = surface_surface(&wide_cyl, &torus, None);

        MINI_CHECK!(cyl_torus.len() == 2);

        for t in &cyl_torus {
            MINI_CHECK!(TOLERANCE.is_close(t.0.point_at_start()[2].abs(), 0.21f64.sqrt()));
            MINI_CHECK!(on_both(&t.0, distance_wide_cylinder, distance_torus) < 1e-9);
            MINI_CHECK!(lies_on_curve(&t.0, &t.2, &torus) < 1e-9);
        }

        let cone_torus = surface_surface(&cone, &high_torus, None);

        MINI_CHECK!(cone_torus.len() == 2);

        for t in &cone_torus {
            MINI_CHECK!(on_both(&t.0, distance_cone, distance_high_torus) < 1e-9);
            MINI_CHECK!(lies_on_curve(&t.0, &t.2, &high_torus) < 1e-9);
        }

        let sphere_torus = surface_surface(&sphere, &torus, None);

        MINI_CHECK!(sphere_torus.len() == 2);

        for t in &sphere_torus {
            MINI_CHECK!(on_both(&t.0, distance_sphere, distance_torus) < 1e-9);
        }

        let torus_torus = surface_surface(&torus, &wide_torus, None);

        MINI_CHECK!(torus_torus.len() == 2);

        for t in &torus_torus {
            MINI_CHECK!(on_both(&t.0, distance_torus, distance_wide_torus) < 1e-9);
            MINI_CHECK!(lies_on_curve(&t.0, &t.1, &torus) < 1e-9);
        }
    })
}

pub fn run_intersection_cut_curves_on_surface() -> TestResult {
    MINI_TEST!("Cut Curves On Surface", {
        use crate::intersection;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Primitives;

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
        let cyl = Primitives::cylinder_surface(0.0, 0.0, -2.0, 1.0, 4.0);
        let pcurves = intersection::cut_curves_on_surface(&flat, &cyl, None);

        MINI_CHECK!(pcurves.len() == 1);
        MINI_CHECK!(pcurves[0].is_valid());

        let mut max_off = 0.0_f64;

        for i in 0..=16 {
            let uv = pcurves[0].point_at(i as f64 / 16.0);
            let p = flat.point_at(uv[0], uv[1]).unwrap();
            max_off = max_off.max(((p[0] * p[0] + p[1] * p[1]).sqrt() - 1.0).abs());
        }

        MINI_CHECK!(max_off < 1e-3);
    })
}

pub fn run_intersection_cut_curves_on_surface_pullbacks() -> TestResult {
    MINI_TEST!("Cut Curves On Surface Pullbacks", {
        use crate::intersection::cut_curves_on_surface;
        use crate::Point;
        use crate::Primitives;

        let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
        let cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
        let wall = bilinear(
            Point::new(0.2, -3.0, -3.0),
            Point::new(0.2, -3.0, 3.0),
            Point::new(0.2, 3.0, -3.0),
            Point::new(0.2, 3.0, 3.0),
        );
        let square = bilinear(
            Point::new(-1.6, -1.6, 0.5),
            Point::new(-1.6, 1.6, 0.5),
            Point::new(1.6, -1.6, 0.5),
            Point::new(1.6, 1.6, 0.5),
        );
        let sphere_cuts = cut_curves_on_surface(&sphere, &wall, None);

        MINI_CHECK!(sphere_cuts.len() == 3);

        for pc in &sphere_cuts {
            MINI_CHECK!(lifted_distance(pc, &sphere, distance_wall) < 5e-3);
        }

        let cone_cuts = cut_curves_on_surface(&cone, &wall, None);

        MINI_CHECK!(cone_cuts.len() == 2);

        for pc in &cone_cuts {
            MINI_CHECK!(lifted_distance(pc, &cone, distance_wall) < 1e-3);
        }

        let square_cuts = cut_curves_on_surface(&sphere, &square, None);

        MINI_CHECK!(square_cuts.len() == 4);

        for pc in &square_cuts {
            MINI_CHECK!(lifted_distance(pc, &sphere, distance_square) < 2e-3);
        }
    })
}

pub fn run_intersection_cut_curves_on_surface_torus() -> TestResult {
    MINI_TEST!("Cut Curves On Surface Torus", {
        use crate::intersection::cut_curves_on_surface;
        use crate::Point;
        use crate::Primitives;

        let torus = Primitives::torus_surface(0.0, 0.0, 0.0, 2.0, 0.5);
        let wall = bilinear(
            Point::new(0.2, -3.0, -3.0),
            Point::new(0.2, -3.0, 3.0),
            Point::new(0.2, 3.0, -3.0),
            Point::new(0.2, 3.0, 3.0),
        );
        let cuts = cut_curves_on_surface(&torus, &wall, None);

        MINI_CHECK!(cuts.len() == 4);

        for pc in &cuts {
            MINI_CHECK!(lifted_distance(pc, &torus, distance_wall) < 1e-5);
        }
    })
}

pub fn run_intersection_cut_curves_slanted_cutter() -> TestResult {
    MINI_TEST!("Cut Curves Slanted Cutter", {
        use crate::intersection::cut_curves_on_surface;
        use crate::Point;
        use crate::Primitives;

        let cone = Primitives::cone_surface(0.0, 0.0, 0.0, 1.5, 3.0);
        let slanted = bilinear(
            Point::new(-3.0, -3.0, 0.0),
            Point::new(-3.0, 3.0, -0.6),
            Point::new(3.0, -3.0, 1.2),
            Point::new(3.0, 3.0, 0.6),
        );
        let cuts = cut_curves_on_surface(&cone, &slanted, None);

        MINI_CHECK!(cuts.len() == 2);

        let domain = cuts[0].domain();
        let uv = cuts[0].point_at((domain.0 + domain.1) * 0.5);
        let p = cone.point_at(uv[0], uv[1]).unwrap_or_default();

        MINI_CHECK!(distance_cone(&p) < 1e-3);
        MINI_CHECK!(distance_slanted(&p) < 1e-3);
    })
}

pub fn run_intersection_cut_curves_plane_trapezoid() -> TestResult {
    MINI_TEST!("Cut Curves Plane Trapezoid", {
        use crate::intersection::cut_curves_on_surface;
        use crate::Point;

        let trapezoid = bilinear(
            Point::new(-3.0, -3.0, 0.0),
            Point::new(-1.0, 3.0, 0.0),
            Point::new(3.0, -3.0, 0.0),
            Point::new(7.0, 3.0, 0.0),
        );
        let wall = bilinear(
            Point::new(6.0, -5.0, -1.0),
            Point::new(6.0, 5.0, -1.0),
            Point::new(6.0, -5.0, 1.0),
            Point::new(6.0, 5.0, 1.0),
        );
        let target_cuts = cut_curves_on_surface(&trapezoid, &wall, None);
        let cutter_cuts = cut_curves_on_surface(&wall, &trapezoid, None);

        MINI_CHECK!(target_cuts.len() == 1);
        MINI_CHECK!(cutter_cuts.len() == 1);

        let target_domain = target_cuts[0].domain();
        let target_uv0 = target_cuts[0].point_at(target_domain.0);
        let target_uv1 = target_cuts[0].point_at(target_domain.1);
        let target_p0 = trapezoid
            .point_at(target_uv0[0], target_uv0[1])
            .unwrap_or_default();
        let target_p1 = trapezoid
            .point_at(target_uv1[0], target_uv1[1])
            .unwrap_or_default();

        MINI_CHECK!((target_p0[0] - 6.0).abs() < 1e-3);
        MINI_CHECK!((target_p1[0] - 6.0).abs() < 1e-3);
        MINI_CHECK!((f64::min(target_p0[1], target_p1[1]) - 1.5).abs() < 1e-3);
        MINI_CHECK!((f64::max(target_p0[1], target_p1[1]) - 3.0).abs() < 1e-3);

        let cutter_domain = cutter_cuts[0].domain();
        let cutter_uv0 = cutter_cuts[0].point_at(cutter_domain.0);
        let cutter_uv1 = cutter_cuts[0].point_at(cutter_domain.1);
        let cutter_p0 = wall
            .point_at(cutter_uv0[0], cutter_uv0[1])
            .unwrap_or_default();
        let cutter_p1 = wall
            .point_at(cutter_uv1[0], cutter_uv1[1])
            .unwrap_or_default();

        MINI_CHECK!((f64::min(cutter_p0[1], cutter_p1[1]) - 1.5).abs() < 1e-3);
        MINI_CHECK!((f64::max(cutter_p0[1], cutter_p1[1]) - 3.0).abs() < 1e-3);
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
        use crate::Line;
        use crate::Point;

        let seg = Line::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
        let pt = Point::new(2.0, 3.0, 0.0);
        let (cp, t) = intersection::closest_point_on_segment(&pt, &seg);

        MINI_CHECK!((cp[0] - 2.0).abs() < 1e-9);
        MINI_CHECK!((cp[1] - 0.0).abs() < 1e-9);
        MINI_CHECK!((t - 0.5).abs() < 1e-9);

        let pt2 = Point::new(-2.0, 1.0, 0.0);
        let (cp2, t2) = intersection::closest_point_on_segment(&pt2, &seg);

        MINI_CHECK!((cp2[0] - 0.0).abs() < 1e-9);
        MINI_CHECK!((t2 - 0.0).abs() < 1e-9);
    })
}

pub fn run_intersection_plane_plane_plane_check_parallel() -> TestResult {
    MINI_TEST!("Plane Plane Plane Check Parallel", {
        use crate::intersection;
        use crate::Plane;
        use crate::Vector;

        let p0 = Plane::from_point_normal(
            crate::Point::new(0.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        );
        let p1 = Plane::from_point_normal(
            crate::Point::new(0.0, 0.0, 1.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        );
        let p2 = Plane::from_point_normal(
            crate::Point::new(0.0, 0.0, 2.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        );

        MINI_CHECK!(intersection::plane_plane_plane_check(&p0, &p1, &p2, 0.1).is_none());

        let px = Plane::from_point_normal(
            crate::Point::new(1.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            None,
        );
        let py = Plane::from_point_normal(
            crate::Point::new(0.0, 2.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            None,
        );
        let pz = Plane::from_point_normal(
            crate::Point::new(0.0, 0.0, 3.0),
            Vector::new(0.0, 0.0, 1.0),
            None,
        );
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
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let main =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0), None);
        let planes = [
            Plane::from_point_normal(Point::new(-1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None),
            Plane::from_point_normal(Point::new(0.0, -1.0, 0.0), Vector::new(0.0, 1.0, 0.0), None),
            Plane::from_point_normal(Point::new(1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None),
            Plane::from_point_normal(Point::new(0.0, 1.0, 0.0), Vector::new(0.0, 1.0, 0.0), None),
        ];
        let result = intersection::plane_4planes(&main, &planes);

        MINI_CHECK!(result.is_some());

        let poly = result.unwrap();

        MINI_CHECK!(poly.len() == 5);

        let pts = poly.get_points();

        for p in &pts {
            MINI_CHECK!(p[2].abs() < 1e-6);
        }

        MINI_CHECK!((pts[0][0] - pts[4][0]).abs() < 1e-6);
        MINI_CHECK!((pts[0][1] - pts[4][1]).abs() < 1e-6);
    })
}

pub fn run_intersection_plane_4planes_open() -> TestResult {
    MINI_TEST!("Plane 4 Planes Open", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let main =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0), None);
        let planes = [
            Plane::from_point_normal(Point::new(-1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None),
            Plane::from_point_normal(Point::new(0.0, -1.0, 0.0), Vector::new(0.0, 1.0, 0.0), None),
            Plane::from_point_normal(Point::new(1.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None),
            Plane::from_point_normal(Point::new(0.0, 1.0, 0.0), Vector::new(0.0, 1.0, 0.0), None),
        ];
        let result = intersection::plane_4planes_open(&main, &planes);

        MINI_CHECK!(result.is_some());

        let poly = result.unwrap();

        MINI_CHECK!(poly.len() == 4);
    })
}

pub fn run_intersection_plane_4lines() -> TestResult {
    MINI_TEST!("Plane 4 Lines", {
        use crate::intersection;
        use crate::Line;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0), None);
        let l0 = Line::new(-1.0, -1.0, -1.0, -1.0, 1.0, 1.0);
        let l1 = Line::new(1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
        let l2 = Line::new(-1.0, -1.0, -1.0, 1.0, -1.0, 1.0);
        let l3 = Line::new(-1.0, 1.0, -1.0, 1.0, 1.0, 1.0);
        let result = intersection::plane_4lines(&plane, &l0, &l1, &l2, &l3);

        MINI_CHECK!(result.is_some());

        let poly = result.unwrap();

        MINI_CHECK!(poly.len() == 5);

        for p in poly.get_points() {
            MINI_CHECK!(p[2].abs() < 1e-6);
        }
    })
}

pub fn run_intersection_line_two_planes() -> TestResult {
    MINI_TEST!("Line Two Planes", {
        use crate::intersection;
        use crate::Line;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let line = Line::new(0.0, 0.0, -5.0, 0.0, 0.0, 5.0);
        let o0 = Point::new(0.0, 0.0, -1.0);
        let o1 = Point::new(0.0, 0.0, 2.0);
        let n = Vector::new(0.0, 0.0, 1.0);
        let plane0 = Plane::from_point_normal(o0, n.clone(), None);
        let plane1 = Plane::from_point_normal(o1, n, None);
        let output = intersection::line_two_planes(&line, &plane0, &plane1);

        MINI_CHECK!(output.is_some());

        let output = output.unwrap();

        MINI_CHECK!(TOLERANCE.is_close(output.start()[2], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(output.end()[2], 2.0));
    })
}

pub fn run_intersection_scale_vector_to_distance_of_2planes() -> TestResult {
    MINI_TEST!("Scale Vector To Distance Of 2 Planes", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let p0 =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0), None);
        let p1 =
            Plane::from_point_normal(Point::new(0.0, 0.0, 3.0), Vector::new(0.0, 0.0, 1.0), None);
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
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let poly = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0),
        ]);

        let plane =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None);
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

        let cutter = Line::new(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
        let seg = Line::new(1.0, 0.0, 0.0, 1.0, 2.0, 0.0);
        let result = intersection::line_line_3d(&cutter, &seg);

        MINI_CHECK!(result.is_some());

        let pt = result.unwrap();

        MINI_CHECK!((pt[0] - 1.0).abs() < 1e-6);
        MINI_CHECK!((pt[1] - 1.0).abs() < 1e-6);
        MINI_CHECK!((pt[2] - 0.0).abs() < 1e-6);

        let par0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let par1 = Line::new(0.0, 1.0, 0.0, 1.0, 1.0, 0.0);

        MINI_CHECK!(intersection::line_line_3d(&par0, &par1).is_none());
    })
}

pub fn run_intersection_polyline_boolean() -> TestResult {
    MINI_TEST!("Polyline Boolean", {
        use crate::intersection;
        use crate::Point;
        use crate::Polyline;

        let a = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let b = Polyline::new(vec![
            Point::new(1.0, 1.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(3.0, 3.0, 0.0),
            Point::new(1.0, 3.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ]);
        let intersection = intersection::polyline_boolean(&a, &b, 0);
        let united = intersection::polyline_boolean(&a, &b, 1);
        let difference = intersection::polyline_boolean(&a, &b, 2);

        MINI_CHECK!(intersection.len() == 1);
        MINI_CHECK!(united.len() == 1);
        MINI_CHECK!(difference.len() == 1);

        for i in 0..intersection[0].point_count() {
            let p = intersection[0].get_point(i).unwrap();

            MINI_CHECK!(p[0] > 1.0 - 1e-9 && p[0] < 2.0 + 1e-9);
            MINI_CHECK!(p[1] > 1.0 - 1e-9 && p[1] < 2.0 + 1e-9);
        }
    })
}

pub fn run_intersection_offset_in_3d() -> TestResult {
    MINI_TEST!("Offset In 3D", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;

        let mut square = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let plane = Plane::xy_plane();
        let ok = intersection::offset_in_3d(&mut square, &plane, 0.5);

        MINI_CHECK!(ok);
        MINI_CHECK!(TOLERANCE.is_close(square.get_point(0).unwrap()[0], -0.5));
        MINI_CHECK!(TOLERANCE.is_close(square.get_point(0).unwrap()[1], -0.5));

        for i in 0..square.point_count() {
            let p = square.get_point(i).unwrap();

            MINI_CHECK!(TOLERANCE.is_close((p[0] - 1.0).abs(), 1.5));
            MINI_CHECK!(TOLERANCE.is_close((p[1] - 1.0).abs(), 1.5));
        }
    })
}

pub fn run_intersection_polyline_boolean_2d_in_plane() -> TestResult {
    MINI_TEST!("Polyline Boolean 2D In Plane", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;

        let a = Polyline::new(vec![
            Point::new(0.0, 0.0, 1.0),
            Point::new(2.0, 0.0, 1.0),
            Point::new(2.0, 2.0, 1.0),
            Point::new(0.0, 2.0, 1.0),
            Point::new(0.0, 0.0, 1.0),
        ]);
        let b = Polyline::new(vec![
            Point::new(1.0, 1.0, 1.0),
            Point::new(3.0, 1.0, 1.0),
            Point::new(3.0, 3.0, 1.0),
            Point::new(1.0, 3.0, 1.0),
            Point::new(1.0, 1.0, 1.0),
        ]);
        let plane = Plane::xy_plane();
        let result =
            intersection::polyline_boolean_2d_in_plane(&a, &b, &plane, 0, false, 0.01, 0.0);

        MINI_CHECK!(result.is_some());

        let result = result.unwrap();

        MINI_CHECK!(result.point_count() >= 4);

        for i in 0..result.point_count() {
            let p = result.get_point(i).unwrap();

            MINI_CHECK!(p[0] > 1.0 - 1e-9 && p[0] < 2.0 + 1e-9);
            MINI_CHECK!(p[1] > 1.0 - 1e-9 && p[1] < 2.0 + 1e-9);
            MINI_CHECK!(TOLERANCE.is_close(p[2], 1.0));
        }

        let tiny = intersection::polyline_boolean_2d_in_plane(&a, &b, &plane, 0, false, 2.0, 0.0);

        MINI_CHECK!(tiny.is_none());
    })
}

pub fn run_intersection_polyline_plane_to_line() -> TestResult {
    MINI_TEST!("Polyline Plane To Line", {
        use crate::intersection::polyline_plane_to_line;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;
        let poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let pln =
            Plane::from_point_normal(Point::new(0.0, 2.0, 0.0), Vector::new(0.0, 1.0, 0.0), None);
        let out = polyline_plane_to_line(&poly, &pln, &Point::new(0.0, 0.0, 0.0));

        MINI_CHECK!(out.is_some());

        let out = out.unwrap();

        MINI_CHECK!(TOLERANCE.is_close(out.start()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(out.end()[0], 4.0));
    })
}

pub fn run_intersection_quad_from_line_top_bottom_planes() -> TestResult {
    MINI_TEST!("Quad From Line Top Bottom Planes", {
        use crate::intersection::quad_from_line_top_bottom_planes;
        use crate::Line;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;
        let face = Plane::xy_plane();
        let line = Line::new(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let plane0 =
            Plane::from_point_normal(Point::new(0.0, -2.0, 0.0), Vector::new(0.0, 1.0, 0.0), None);
        let plane1 =
            Plane::from_point_normal(Point::new(0.0, 2.0, 0.0), Vector::new(0.0, 1.0, 0.0), None);
        let out = quad_from_line_top_bottom_planes(&face, &line, &plane0, &plane1);

        MINI_CHECK!(out.is_some());

        let out = out.unwrap();

        MINI_CHECK!(out.point_count() == 5);
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(0).unwrap()[1].abs(), 2.0));
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(2).unwrap()[1].abs(), 2.0));
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(2).unwrap()[0], 10.0));
    })
}

pub fn run_intersection_orthogonal_vector_between_two_plane_pairs() -> TestResult {
    MINI_TEST!("Orthogonal Vector Between Two Plane Pairs", {
        use crate::intersection::orthogonal_vector_between_two_plane_pairs;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;
        let pp00 = Plane::xy_plane();
        let pp10 =
            Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None);
        let pp11 =
            Plane::from_point_normal(Point::new(4.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), None);
        let out = orthogonal_vector_between_two_plane_pairs(&pp00, &pp10, &pp11);

        MINI_CHECK!(out.is_some());

        let out = out.unwrap();
        let mag = (out[0] * out[0] + out[1] * out[1] + out[2] * out[2]).sqrt();

        MINI_CHECK!(TOLERANCE.is_close(mag, 4.0));
        MINI_CHECK!(TOLERANCE.is_close(out[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(out[2], 0.0));
    })
}

pub fn run_intersection_closed_and_open_paths_2d() -> TestResult {
    MINI_TEST!("Closed And Open Paths 2D", {
        use crate::intersection::closed_and_open_paths_2d;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        let plate = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 10.0, 0.0),
            Point::new(0.0, 10.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let joint = Polyline::new(vec![Point::new(-2.0, 5.0, 0.0), Point::new(12.0, 5.0, 0.0)]);
        let pln = Plane::xy_plane();
        let result = closed_and_open_paths_2d(&plate, &joint, &pln);

        MINI_CHECK!(result.is_some());

        let (out, (t0, t1)) = result.unwrap();

        MINI_CHECK!(out.point_count() == 2);
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(0).unwrap()[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(out.get_point(1).unwrap()[1], 5.0));

        let t_lo = t0.min(t1);
        let t_hi = t0.max(t1);

        MINI_CHECK!(TOLERANCE.is_close(t_lo, 1.5));
        MINI_CHECK!(TOLERANCE.is_close(t_hi, 3.5));
    })
}

pub fn run_intersection_face_to_face() -> TestResult {
    MINI_TEST!("Face To Face", {
        use crate::intersection;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let polylines = vec![
            vec![
                Polyline::new(vec![
                    Point::new(0.0, 0.0, 0.0),
                    Point::new(2.0, 0.0, 0.0),
                    Point::new(2.0, 1.0, 0.0),
                    Point::new(0.0, 1.0, 0.0),
                    Point::new(0.0, 0.0, 0.0),
                ]),
                Polyline::new(vec![
                    Point::new(0.0, 0.0, 1.0),
                    Point::new(2.0, 0.0, 1.0),
                    Point::new(2.0, 1.0, 1.0),
                    Point::new(0.0, 1.0, 1.0),
                    Point::new(0.0, 0.0, 1.0),
                ]),
            ],
            vec![
                Polyline::new(vec![
                    Point::new(1.0, 0.5, 1.0),
                    Point::new(3.0, 0.5, 1.0),
                    Point::new(3.0, 1.5, 1.0),
                    Point::new(1.0, 1.5, 1.0),
                    Point::new(1.0, 0.5, 1.0),
                ]),
                Polyline::new(vec![
                    Point::new(1.0, 0.5, 2.0),
                    Point::new(3.0, 0.5, 2.0),
                    Point::new(3.0, 1.5, 2.0),
                    Point::new(1.0, 1.5, 2.0),
                    Point::new(1.0, 0.5, 2.0),
                ]),
            ],
        ];
        let o00 = Point::new(1.0, 0.5, 0.0);
        let o01 = Point::new(1.0, 0.5, 1.0);
        let o10 = Point::new(2.0, 1.0, 1.0);
        let o11 = Point::new(2.0, 1.0, 2.0);
        let down = Vector::new(0.0, 0.0, -1.0);
        let up = Vector::new(0.0, 0.0, 1.0);

        let planes = vec![
            vec![
                Plane::from_point_normal(o00, down.clone(), None),
                Plane::from_point_normal(o01, up.clone(), None),
            ],
            vec![
                Plane::from_point_normal(o10, down, None),
                Plane::from_point_normal(o11, up, None),
            ],
        ];

        let adjacency = vec![0, 1, -1, -1];
        let contacts = intersection::face_to_face(&adjacency, &polylines, &planes, 0.01);

        MINI_CHECK!(contacts.len() == 1);
        MINI_CHECK!(contacts[0].0 == 0);
        MINI_CHECK!(contacts[0].1 == 1);
        MINI_CHECK!(contacts[0].2 == 1);
        MINI_CHECK!(contacts[0].3 == 0);
        MINI_CHECK!(contacts[0].4 == 2);
        MINI_CHECK!(contacts[0].5.is_closed());

        for i in 0..contacts[0].5.point_count() {
            let p = contacts[0].5.get_point(i).unwrap();

            MINI_CHECK!(p[0] > 1.0 - 1e-9 && p[0] < 2.0 + 1e-9);
            MINI_CHECK!(p[1] > 0.5 - 1e-9 && p[1] < 1.0 + 1e-9);
            MINI_CHECK!(TOLERANCE.is_close(p[2], 1.0));
        }
    })
}

pub fn run_intersection_adjacency_search() -> TestResult {
    MINI_TEST!("Adjacency Search", {
        use crate::intersection;
        use crate::Element;
        use crate::Mesh;
        use crate::Point;

        let a = Element::from_mesh(
            Mesh::from_polylines(
                vec![vec![
                    Point::new(0.0, 0.0, 0.0),
                    Point::new(1.0, 0.0, 0.0),
                    Point::new(1.0, 1.0, 0.0),
                    Point::new(0.0, 1.0, 0.0),
                ]],
                None,
            ),
            "my_element",
        );
        let b = Element::from_mesh(
            Mesh::from_polylines(
                vec![vec![
                    Point::new(1.0, 0.0, 0.0),
                    Point::new(2.0, 0.0, 0.0),
                    Point::new(2.0, 1.0, 0.0),
                    Point::new(1.0, 1.0, 0.0),
                ]],
                None,
            ),
            "my_element",
        );
        let c = Element::from_mesh(
            Mesh::from_polylines(
                vec![vec![
                    Point::new(5.0, 0.0, 0.0),
                    Point::new(6.0, 0.0, 0.0),
                    Point::new(6.0, 1.0, 0.0),
                    Point::new(5.0, 1.0, 0.0),
                ]],
                None,
            ),
            "my_element",
        );
        let mut elements = vec![a, b, c];
        let adjacency = intersection::adjacency_search(&mut elements, 0.01);

        MINI_CHECK!(adjacency.len() == 4);
        MINI_CHECK!(adjacency[0] == 0);
        MINI_CHECK!(adjacency[1] == 1);
        MINI_CHECK!(adjacency[2] == -1);
        MINI_CHECK!(adjacency[3] == -1);
    })
}

pub fn run_intersection_line_line_classified() -> TestResult {
    MINI_TEST!("Line Line Classified", {
        use crate::intersection;
        use crate::Line;
        use crate::Point;
        use crate::Vector;

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
        let ok = intersection::line_line_classified(
            &s0,
            &s1,
            1,
            1,
            0,
            0,
            0.5,
            &mut p0,
            &mut p1,
            &mut v0,
            &mut v1,
            &mut normal,
            &mut type0,
            &mut type1,
            &mut is_parallel,
        );

        MINI_CHECK!(ok);
        MINI_CHECK!(!is_parallel);
        MINI_CHECK!((p0[0]).abs() < 1e-6);
        MINI_CHECK!((p0[1]).abs() < 1e-6);
        MINI_CHECK!((p1[0]).abs() < 1e-6);
        MINI_CHECK!((p1[1]).abs() < 1e-6);
        MINI_CHECK!((normal[2].abs() - 1.0).abs() < 1e-6);

        let e0 = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let e1 = Line::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        let ok2 = intersection::line_line_classified(
            &e0,
            &e1,
            1,
            1,
            0,
            0,
            0.5,
            &mut p0,
            &mut p1,
            &mut v0,
            &mut v1,
            &mut normal,
            &mut type0,
            &mut type1,
            &mut is_parallel,
        );

        MINI_CHECK!(ok2);
        MINI_CHECK!(!type0);
        MINI_CHECK!(!type1);
        MINI_CHECK!((p0[0]).abs() < 1e-6);
        MINI_CHECK!((p0[1]).abs() < 1e-6);

        let q0 = Line::new(0.0, 0.0, 0.0, 2.0, 0.0, 0.0);
        let q1 = Line::new(0.0, 1.0, 0.0, 2.0, 1.0, 0.0);
        let ok3 = intersection::line_line_classified(
            &q0,
            &q1,
            1,
            1,
            0,
            0,
            0.5,
            &mut p0,
            &mut p1,
            &mut v0,
            &mut v1,
            &mut normal,
            &mut type0,
            &mut type1,
            &mut is_parallel,
        );

        MINI_CHECK!(ok3);
        MINI_CHECK!(is_parallel);
        MINI_CHECK!(!type0);
        MINI_CHECK!(!type1);
    })
}

REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line",
    crate::intersection_test::run_intersection_line_line
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line Parallel",
    crate::intersection_test::run_intersection_line_line_parallel
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line Parameters",
    crate::intersection_test::run_intersection_line_line_parameters
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line Parameters Endpoints",
    crate::intersection_test::run_intersection_line_line_parameters_endpoints
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line Parameters Infinite",
    crate::intersection_test::run_intersection_line_line_parameters_infinite
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane Plane",
    crate::intersection_test::run_intersection_plane_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane Plane Complex",
    crate::intersection_test::run_intersection_plane_plane_complex
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane Plane To Line Canonical",
    crate::intersection_test::run_intersection_plane_plane_to_line_canonical
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Plane",
    crate::intersection_test::run_intersection_line_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Plane Parallel",
    crate::intersection_test::run_intersection_line_plane_parallel
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Plane Real World",
    crate::intersection_test::run_intersection_line_plane_real_world
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane Plane Plane",
    crate::intersection_test::run_intersection_plane_plane_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane Plane Plane Parallel",
    crate::intersection_test::run_intersection_plane_plane_plane_parallel
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Box",
    crate::intersection_test::run_intersection_ray_box
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Box Miss",
    crate::intersection_test::run_intersection_ray_box_miss
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Sphere",
    crate::intersection_test::run_intersection_ray_sphere
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Sphere Tangent",
    crate::intersection_test::run_intersection_ray_sphere_tangent
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Sphere Miss",
    crate::intersection_test::run_intersection_ray_sphere_miss
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Triangle",
    crate::intersection_test::run_intersection_ray_triangle
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Triangle Miss",
    crate::intersection_test::run_intersection_ray_triangle_miss
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Triangle Parallel",
    crate::intersection_test::run_intersection_ray_triangle_parallel
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh",
    crate::intersection_test::run_intersection_ray_mesh
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh First",
    crate::intersection_test::run_intersection_ray_mesh_first
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh Miss",
    crate::intersection_test::run_intersection_ray_mesh_miss
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh Bvh",
    crate::intersection_test::run_intersection_ray_mesh_bvh
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh Bvh First",
    crate::intersection_test::run_intersection_ray_mesh_bvh_first
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh Bvh Miss",
    crate::intersection_test::run_intersection_ray_mesh_bvh_miss
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Mesh Bvh Vs Naive",
    crate::intersection_test::run_intersection_ray_mesh_bvh_vs_naive
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Box Real World",
    crate::intersection_test::run_intersection_ray_box_real_world
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Sphere Real World",
    crate::intersection_test::run_intersection_ray_sphere_real_world
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Ray Triangle Real World",
    crate::intersection_test::run_intersection_ray_triangle_real_world
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Curve Plane",
    crate::intersection_test::run_intersection_curve_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Curve Plane Bezier Clipping",
    crate::intersection_test::run_intersection_curve_plane_bezier_clipping
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Curve Plane Algebraic",
    crate::intersection_test::run_intersection_curve_plane_algebraic
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Curve Plane Production",
    crate::intersection_test::run_intersection_curve_plane_production
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Curve Closest Point",
    crate::intersection_test::run_intersection_curve_closest_point
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Plane",
    crate::intersection_test::run_intersection_surface_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Plane Curved",
    crate::intersection_test::run_intersection_surface_plane_curved
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Plane Miss",
    crate::intersection_test::run_intersection_surface_plane_miss
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Plane UV",
    crate::intersection_test::run_intersection_surface_plane_uv
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface",
    crate::intersection_test::run_intersection_surface_surface
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Accuracy",
    crate::intersection_test::run_intersection_surface_surface_accuracy
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Planes",
    crate::intersection_test::run_intersection_surface_surface_planes
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Plane Cone",
    crate::intersection_test::run_intersection_surface_surface_plane_cone
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Plane Torus",
    crate::intersection_test::run_intersection_surface_surface_plane_torus
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Cylinders",
    crate::intersection_test::run_intersection_surface_surface_cylinders
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Coaxial Quadrics",
    crate::intersection_test::run_intersection_surface_surface_coaxial_quadrics
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Surface Surface Coaxial Tori",
    crate::intersection_test::run_intersection_surface_surface_coaxial_tori
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Cut Curves On Surface",
    crate::intersection_test::run_intersection_cut_curves_on_surface
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Cut Curves On Surface Pullbacks",
    crate::intersection_test::run_intersection_cut_curves_on_surface_pullbacks
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Cut Curves On Surface Torus",
    crate::intersection_test::run_intersection_cut_curves_on_surface_torus
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Cut Curves Slanted Cutter",
    crate::intersection_test::run_intersection_cut_curves_slanted_cutter
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Cut Curves Plane Trapezoid",
    crate::intersection_test::run_intersection_cut_curves_plane_trapezoid
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Remap",
    crate::intersection_test::run_intersection_remap
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Closest Point On Segment",
    crate::intersection_test::run_intersection_closest_point_on_segment
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane Plane Plane Check Parallel",
    crate::intersection_test::run_intersection_plane_plane_plane_check_parallel
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane 4 Planes Closed",
    crate::intersection_test::run_intersection_plane_4planes
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane 4 Planes Open",
    crate::intersection_test::run_intersection_plane_4planes_open
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Plane 4 Lines",
    crate::intersection_test::run_intersection_plane_4lines
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Two Planes",
    crate::intersection_test::run_intersection_line_two_planes
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Scale Vector To Distance Of 2 Planes",
    crate::intersection_test::run_intersection_scale_vector_to_distance_of_2planes
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Polyline Plane",
    crate::intersection_test::run_intersection_polyline_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line 3D",
    crate::intersection_test::run_intersection_line_line_3d
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Polyline Boolean",
    crate::intersection_test::run_intersection_polyline_boolean
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Offset In 3D",
    crate::intersection_test::run_intersection_offset_in_3d
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Polyline Boolean 2D In Plane",
    crate::intersection_test::run_intersection_polyline_boolean_2d_in_plane
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Polyline Plane To Line",
    crate::intersection_test::run_intersection_polyline_plane_to_line
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Quad From Line Top Bottom Planes",
    crate::intersection_test::run_intersection_quad_from_line_top_bottom_planes
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Orthogonal Vector Between Two Plane Pairs",
    crate::intersection_test::run_intersection_orthogonal_vector_between_two_plane_pairs
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Closed And Open Paths 2D",
    crate::intersection_test::run_intersection_closed_and_open_paths_2d
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Face To Face",
    crate::intersection_test::run_intersection_face_to_face
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Adjacency Search",
    crate::intersection_test::run_intersection_adjacency_search
);
REGISTER_MINI_TEST!(
    "Intersection",
    "Line Line Classified",
    crate::intersection_test::run_intersection_line_line_classified
);
