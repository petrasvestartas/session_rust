use crate::{Line, Point};
use crate::closest::Closest;

pub fn line_line_parameters(
    line0: &Line,
    line1: &Line,
    tolerance: f64,
    intersect_segments: bool,
    near_parallel_as_closest: bool,
) -> Option<(f64, f64)> {
    let p0_start = line0.start();
    let p0_end = line0.end();
    let p1_start = line1.start();
    let p1_end = line1.end();

    if p0_start == p1_start {
        return Some((0.0, 0.0));
    }
    if p0_start == p1_end {
        return Some((0.0, 1.0));
    }
    if p0_end == p1_start {
        return Some((1.0, 0.0));
    }
    if p0_end == p1_end {
        return Some((1.0, 1.0));
    }

    let a = line0.to_vector();
    let b = line1.to_vector();
    let c = p1_start - p0_start;

    let aa = a.dot(&a);
    let bb = b.dot(&b);
    let ab = a.dot(&b);
    let ac = a.dot(&c);
    let bc = b.dot(&c);

    let det = aa * bb - ab * ab;

    let zero_tol = aa.max(bb) * f64::EPSILON;
    if det.abs() < zero_tol {
        if !near_parallel_as_closest {
            return None;
        }
        let mut t0 = if aa > 0.0 { ac / aa } else { 0.0 };
        let mut t1 = if bb > 0.0 { (bc + t0 * ab) / bb } else { 0.0 };

        if intersect_segments {
            t0 = t0.clamp(0.0, 1.0);
            t1 = t1.clamp(0.0, 1.0);
        }

        if tolerance > 0.0 {
            let pt0 = line0.point_at(t0);
            let pt1 = line1.point_at(t1);
            if pt0.distance(&pt1, None) > tolerance {
                return None;
            }
        }
        return Some((t0, t1));
    }

    let inv_det = 1.0 / det;
    let mut t0 = (bb * ac - ab * bc) * inv_det;
    let mut t1 = (ab * ac - aa * bc) * inv_det;

    if intersect_segments {
        t0 = t0.clamp(0.0, 1.0);
        t1 = t1.clamp(0.0, 1.0);
    }

    if tolerance > 0.0 {
        let pt0 = line0.point_at(t0);
        let pt1 = line1.point_at(t1);
        if pt0.distance(&pt1, None) > tolerance {
            return None;
        }
    }

    Some((t0, t1))
}

/// Find intersection point between two 3D lines.
///
/// # Arguments
/// * `line0` - First line
/// * `line1` - Second line
/// * `tolerance` - Maximum distance between lines to consider them intersecting
///
/// # Returns
/// * `Some(Point)` - Intersection point (midpoint of closest approach for skew lines)
/// * `None` - If lines don't intersect within tolerance
pub fn line_line(line0: &Line, line1: &Line, tolerance: f64) -> Option<Point> {
    let result = line_line_parameters(line0, line1, tolerance, true, false)?;

    let (t0, t1) = result;
    let p0 = line0.point_at(t0);
    let p1 = line1.point_at(t1);

    Some(Point::new(
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    ))
}

/// Find intersection line between two planes.
///
/// # Arguments
/// * `plane0` - First plane
/// * `plane1` - Second plane
///
/// # Returns
/// * `Some(Line)` - Intersection line (infinite) if planes intersect
/// * `None` - If planes are parallel
pub fn plane_plane(plane0: &crate::Plane, plane1: &crate::Plane) -> Option<Line> {
    let d = plane1.z_axis().cross(&plane0.z_axis());

    let origin0 = plane0.origin();
    let origin1 = plane1.origin();
    let p = Point::new(
        (origin0[0] + origin1[0]) * 0.5,
        (origin0[1] + origin1[1]) * 0.5,
        (origin0[2] + origin1[2]) * 0.5,
    );

    let plane2 = crate::Plane::from_point_normal(p, d.clone());

    let output_p = plane_plane_plane(plane0, plane1, &plane2)?;

    Some(Line::new(
        output_p[0],
        output_p[1],
        output_p[2],
        output_p[0] + d[0],
        output_p[1] + d[1],
        output_p[2] + d[2],
    ))
}

fn plane_value_at(plane: &crate::Plane, point: &Point) -> f64 {
    plane.a() * point[0] + plane.b() * point[1] + plane.c() * point[2] + plane.d()
}

/// Find intersection point between a line and a plane.
///
/// # Arguments
/// * `line` - Line to intersect
/// * `plane` - Plane to intersect
/// * `is_finite` - If true, treat line as finite segment; if false, treat as infinite
///
/// # Returns
/// * `Some(Point)` - Intersection point if exists
/// * `None` - If line is parallel to plane or intersection is outside segment bounds
pub fn line_plane(line: &Line, plane: &crate::Plane, is_finite: bool) -> Option<Point> {
    let pt0 = line.start();
    let pt1 = line.end();

    let a = plane_value_at(plane, &pt0);
    let b = plane_value_at(plane, &pt1);
    let d = a - b;

    let (t, rc) = if d == 0.0 {
        let t = if a.abs() < b.abs() {
            0.0
        } else if b.abs() < a.abs() {
            1.0
        } else {
            0.5
        };
        (t, false)
    } else {
        let d_inv = 1.0 / d;
        let fd = d_inv.abs();
        if fd > 1.0 && (a.abs() >= f64::MAX / fd || b.abs() >= f64::MAX / fd) {
            (0.5, false)
        } else {
            (a / (a - b), true)
        }
    };

    let s = 1.0 - t;

    let output = Point::new(
        if line[0] == line[3] { line[0] } else { s * line[0] + t * line[3] },
        if line[1] == line[4] { line[1] } else { s * line[1] + t * line[4] },
        if line[2] == line[5] { line[2] } else { s * line[2] + t * line[5] },
    );

    if is_finite && !(0.0..=1.0).contains(&t) {
        return None;
    }

    if rc {
        Some(output)
    } else {
        None
    }
}

pub fn plane_plane_plane(
    plane0: &crate::Plane,
    plane1: &crate::Plane,
    plane2: &crate::Plane,
) -> Option<Point> {
    let n0 = plane0.z_axis();
    let n1 = plane1.z_axis();
    let n2 = plane2.z_axis();

    let det = n0.dot(&n1.cross(&n2));

    if det.abs() < 1e-10 {
        return None;
    }

    let d0 = plane0.d();
    let d1 = plane1.d();
    let d2 = plane2.d();

    let inv_det = 1.0 / det;
    let p = (n1.cross(&n2) * (-d0) + n2.cross(&n0) * (-d1) + n0.cross(&n1) * (-d2)) * inv_det;

    Some(Point::new(p[0], p[1], p[2]))
}

/// Find intersection points between a line and an axis-aligned bounding box.
///
/// # Arguments
/// * `line` - Line to intersect
/// * `box_` - Axis-aligned bounding box
/// * `t0` - Minimum parameter value to consider (e.g., 0.0 for ray origin)
/// * `t1` - Maximum parameter value to consider (e.g., 1000.0 for max distance)
///
/// # Returns
/// * `Some(Vec<Point>)` - Vector of 2 points [entry, exit] if intersection exists
/// * `None` - If no intersection within [t0, t1] range
///
/// # Note
/// Points are sorted from line start (entry first, exit second)
pub fn ray_box(line: &Line, box_: &crate::BoundingBox, t0: f64, t1: f64) -> Option<Vec<Point>> {
    let origin = line.start();
    let direction = line.to_vector();

    let box_min = box_.min_point();
    let box_max = box_.max_point();

    // Calculate inverse direction (avoid division by zero)
    let inv_dir_x = if direction[0] != 0.0 {
        1.0 / direction[0]
    } else {
        f64::INFINITY
    };
    let inv_dir_y = if direction[1] != 0.0 {
        1.0 / direction[1]
    } else {
        f64::INFINITY
    };
    let inv_dir_z = if direction[2] != 0.0 {
        1.0 / direction[2]
    } else {
        f64::INFINITY
    };

    // Calculate intersections with X slabs
    let tx1 = (box_min[0] - origin[0]) * inv_dir_x;
    let tx2 = (box_max[0] - origin[0]) * inv_dir_x;

    let mut tmin = tx1.min(tx2);
    let mut tmax = tx1.max(tx2);

    // Calculate intersections with Y slabs
    let ty1 = (box_min[1] - origin[1]) * inv_dir_y;
    let ty2 = (box_max[1] - origin[1]) * inv_dir_y;

    tmin = tmin.max(ty1.min(ty2));
    tmax = tmax.min(ty1.max(ty2));

    // Calculate intersections with Z slabs
    let tz1 = (box_min[2] - origin[2]) * inv_dir_z;
    let tz2 = (box_max[2] - origin[2]) * inv_dir_z;

    tmin = tmin.max(tz1.min(tz2));
    tmax = tmax.min(tz1.max(tz2));

    // Clip to valid range
    tmin = tmin.max(t0);
    tmax = tmax.min(t1);

    // Check if intersection exists
    if tmax < tmin {
        return None;
    }

    // Calculate actual intersection points
    let entry = Point::new(
        origin[0] + direction[0] * tmin,
        origin[1] + direction[1] * tmin,
        origin[2] + direction[2] * tmin,
    );

    let exit = Point::new(
        origin[0] + direction[0] * tmax,
        origin[1] + direction[1] * tmax,
        origin[2] + direction[2] * tmax,
    );

    Some(vec![entry, exit])
}

/// Find intersection points between a line and a sphere.
///
/// # Arguments
/// * `line` - Line to intersect
/// * `center` - Sphere center point
/// * `radius` - Sphere radius
///
/// # Returns
/// * `Some(Vec<Point>)` - Vector of 1 point (tangent) or 2 points (entry/exit)
/// * `None` - If no intersection
///
/// # Note
/// Points are sorted from line start
pub fn ray_sphere(line: &Line, center: &Point, radius: f64) -> Option<Vec<Point>> {
    let origin = line.start();
    let direction = line.to_vector();

    // Vector from origin to center
    let o_x = origin[0] - center[0];
    let o_y = origin[1] - center[1];
    let o_z = origin[2] - center[2];

    // Quadratic equation coefficients
    let a = direction[0] * direction[0]
        + direction[1] * direction[1]
        + direction[2] * direction[2];
    let b = 2.0 * (direction[0] * o_x + direction[1] * o_y + direction[2] * o_z);
    let c = o_x * o_x + o_y * o_y + o_z * o_z - radius * radius;

    // Discriminant
    let disc = b * b - 4.0 * a * c;

    if disc < 0.0 {
        return None;
    }

    // Calculate intersection parameters
    let dist_sqrt = disc.sqrt();
    let q = if b < 0.0 {
        (-b - dist_sqrt) / 2.0
    } else {
        (-b + dist_sqrt) / 2.0
    };

    let mut t0 = q / a;
    let mut t1 = c / q;

    // Sort parameters
    if t0 > t1 {
        std::mem::swap(&mut t0, &mut t1);
    }

    // Calculate intersection points
    let mut points = Vec::new();

    // First intersection
    let p0 = Point::new(
        origin[0] + direction[0] * t0,
        origin[1] + direction[1] * t0,
        origin[2] + direction[2] * t0,
    );
    points.push(p0);

    // Second intersection (if different from first)
    if (t1 - t0).abs() > 1e-10 {
        let p1 = Point::new(
            origin[0] + direction[0] * t1,
            origin[1] + direction[1] * t1,
            origin[2] + direction[2] * t1,
        );
        points.push(p1);
    }

    Some(points)
}

/// Find intersection point between a line and a triangle.
///
/// # Arguments
/// * `line` - Line to intersect (start point used as origin, direction computed internally)
/// * `v0` - First vertex of triangle
/// * `v1` - Second vertex of triangle
/// * `v2` - Third vertex of triangle
/// * `epsilon` - Tolerance for parallel detection
///
/// # Returns
/// * `Some(Point)` - Intersection point if exists
/// * `None` - If no intersection (parallel or outside triangle)
pub fn ray_triangle(
    line: &Line,
    v0: &Point,
    v1: &Point,
    v2: &Point,
    epsilon: f64,
) -> Option<Point> {
    let origin = line.start();
    let direction = line.to_vector();

    // Möller-Trumbore algorithm
    let edge1_x = v1[0] - v0[0];
    let edge1_y = v1[1] - v0[1];
    let edge1_z = v1[2] - v0[2];

    let edge2_x = v2[0] - v0[0];
    let edge2_y = v2[1] - v0[1];
    let edge2_z = v2[2] - v0[2];

    // pvec = direction.cross(edge2)
    let pvec_x = direction[1] * edge2_z - direction[2] * edge2_y;
    let pvec_y = direction[2] * edge2_x - direction[0] * edge2_z;
    let pvec_z = direction[0] * edge2_y - direction[1] * edge2_x;

    // det = edge1.dot(pvec)
    let det = edge1_x * pvec_x + edge1_y * pvec_y + edge1_z * pvec_z;

    if det > -epsilon && det < epsilon {
        return None; // Parallel
    }

    let inv_det = 1.0 / det;

    // tvec = origin - v0
    let tvec_x = origin[0] - v0[0];
    let tvec_y = origin[1] - v0[1];
    let tvec_z = origin[2] - v0[2];

    // u = tvec.dot(pvec) * inv_det
    let u = (tvec_x * pvec_x + tvec_y * pvec_y + tvec_z * pvec_z) * inv_det;

    if u < -epsilon || u > 1.0 + epsilon {
        return None;
    }

    // qvec = tvec.cross(edge1)
    let qvec_x = tvec_y * edge1_z - tvec_z * edge1_y;
    let qvec_y = tvec_z * edge1_x - tvec_x * edge1_z;
    let qvec_z = tvec_x * edge1_y - tvec_y * edge1_x;

    // v = direction.dot(qvec) * inv_det
    let v = (direction[0] * qvec_x + direction[1] * qvec_y + direction[2] * qvec_z) * inv_det;

    if v < -epsilon || u + v > 1.0 + epsilon {
        return None;
    }

    // t = edge2.dot(qvec) * inv_det
    let t = (edge2_x * qvec_x + edge2_y * qvec_y + edge2_z * qvec_z) * inv_det;

    // Calculate intersection point: origin + t * direction
    Some(Point::new(
        origin[0] + t * direction[0],
        origin[1] + t * direction[1],
        origin[2] + t * direction[2],
    ))
}

//==========================================================================================
// NURBS Curve Intersection Functions
//==========================================================================================

use crate::{NurbsCurve, Plane, Tolerance, Vector};

fn curve_signed_distance_to_plane(pt: &Point, plane: &Plane) -> f64 {
    let v = Vector::new(
        pt[0] - plane.origin()[0],
        pt[1] - plane.origin()[1],
        pt[2] - plane.origin()[2],
    );
    v.dot(&plane.z_axis())
}

/// Find all intersections between NURBS curve and plane
pub fn curve_plane(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<f64> {
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
    let mut results = Vec::new();

    if !curve.is_valid() {
        return results;
    }

    let (_t_start, t_end) = curve.domain();
    let span_params = curve.get_span_vector();

    for i in 0..(span_params.len() - 1) {
        let t0 = span_params[i];
        let t1 = span_params[i + 1];

        if (t1 - t0).abs() < tol {
            continue;
        }

        let d0 = curve_signed_distance_to_plane(&curve.point_at(t0), plane);
        let d1 = curve_signed_distance_to_plane(&curve.point_at(t1), plane);

        if d0 * d1 < 0.0 {
            let mut ta = t0;
            let mut tb = t1;
            let mut tm = (ta + tb) * 0.5;

            for _ in 0..50 {
                tm = (ta + tb) * 0.5;
                let dm = curve_signed_distance_to_plane(&curve.point_at(tm), plane);
                if dm.abs() < tol {
                    break;
                }
                if dm * d0 < 0.0 {
                    tb = tm;
                } else {
                    ta = tm;
                }
            }
            results.push(tm);
        } else if d0.abs() < tol {
            if results.is_empty() || (results.last().unwrap() - t0).abs() >= tol {
                results.push(t0);
            }
        }
    }

    let d_end = curve_signed_distance_to_plane(&curve.point_at(t_end), plane);
    if d_end.abs() < tol {
        if results.is_empty() || (results.last().unwrap() - t_end).abs() >= tol {
            results.push(t_end);
        }
    }

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());
    results.dedup_by(|a, b| (*a - *b).abs() < tol * 2.0);

    results
}

/// Find all intersection points between NURBS curve and plane
pub fn curve_plane_points(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<Point> {
    curve_plane(curve, plane, tolerance)
        .iter()
        .map(|&t| curve.point_at(t))
        .collect()
}

/// Curve-plane intersection using Bézier clipping (advanced method)
pub fn curve_plane_bezier_clipping(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<f64> {
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

    if !curve.is_valid() {
        return Vec::new();
    }

    let mut results = Vec::new();
    let (t0, t1) = curve.domain();

    fn clip_recursive(
        curve: &NurbsCurve,
        plane: &Plane,
        ta: f64,
        tb: f64,
        depth: i32,
        tol: f64,
        results: &mut Vec<f64>,
    ) {
        if depth > 50 {
            let tm = (ta + tb) * 0.5;
            let pm = curve.point_at(tm);
            let dist = curve_signed_distance_to_plane(&pm, plane);
            if dist.abs() < tol {
                results.push(tm);
            }
            return;
        }

        if (tb - ta).abs() < tol * 0.01 {
            let tm = (ta + tb) * 0.5;
            let pm = curve.point_at(tm);
            let dist = curve_signed_distance_to_plane(&pm, plane);

            if dist.abs() < tol {
                let mut t = tm;
                for _ in 0..10 {
                    let pt = curve.point_at(t);
                    let tan = curve.tangent_at(t);
                    let f = curve_signed_distance_to_plane(&pt, plane);
                    let df = tan.dot(&plane.z_axis());
                    if df.abs() < 1e-12 {
                        break;
                    }
                    let dt = -f / df;
                    t += dt;
                    if dt.abs() < tol * 0.01 {
                        break;
                    }
                    if t < ta || t > tb {
                        t = tm;
                        break;
                    }
                }

                let pt_final = curve.point_at(t);
                if curve_signed_distance_to_plane(&pt_final, plane).abs() < tol && ta <= t && t <= tb {
                    results.push(t);
                }
            }
            return;
        }

        let num_samples = (curve.order() + 1).min(10);
        let mut distances = Vec::new();
        let mut params = Vec::new();

        let dt = (tb - ta) / (num_samples - 1) as f64;
        for i in 0..num_samples {
            let t = ta + i as f64 * dt;
            let p = curve.point_at(t);
            distances.push(curve_signed_distance_to_plane(&p, plane));
            params.push(t);
        }

        let d_min = distances.iter().cloned().fold(f64::INFINITY, f64::min);
        let d_max = distances.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        if d_min > tol || d_max < -tol {
            return;
        }

        let mut t_min = ta;
        let mut t_max = tb;

        for i in 0..(distances.len() - 1) {
            if distances[i] * distances[i + 1] < 0.0 {
                let d0 = distances[i];
                let d1 = distances[i + 1];
                let t_clip = params[i] - d0 * (params[i + 1] - params[i]) / (d1 - d0);
                if d0 > 0.0 {
                    t_max = t_max.min(t_clip + (tb - ta) * 0.1);
                } else {
                    t_min = t_min.max(t_clip - (tb - ta) * 0.1);
                }
            }
        }

        if t_min >= t_max {
            t_min = ta;
            t_max = tb;
        }

        t_min = t_min.max(ta);
        t_max = t_max.min(tb);

        let reduction = (t_max - t_min) / (tb - ta);

        if reduction > 0.8 || (t_max - t_min) < tol * 0.1 {
            let tm = (ta + tb) * 0.5;
            clip_recursive(curve, plane, ta, tm, depth + 1, tol, results);
            clip_recursive(curve, plane, tm, tb, depth + 1, tol, results);
        } else {
            clip_recursive(curve, plane, t_min, t_max, depth + 1, tol, results);
        }
    }

    clip_recursive(curve, plane, t0, t1, 0, tol, &mut results);

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());
    results.dedup_by(|a, b| (*a - *b).abs() < tol * 2.0);

    results
}

/// Curve-plane intersection using algebraic/hodograph method
pub fn curve_plane_algebraic(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<f64> {
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

    if !curve.is_valid() {
        return Vec::new();
    }

    let mut results = Vec::new();
    let spans = curve.get_span_vector();

    for span_idx in 0..(spans.len() - 1) {
        let span_t0 = spans[span_idx];
        let span_t1 = spans[span_idx + 1];

        if (span_t1 - span_t0).abs() < tol {
            continue;
        }

        let d0 = curve_signed_distance_to_plane(&curve.point_at(span_t0), plane);
        let d1 = curve_signed_distance_to_plane(&curve.point_at(span_t1), plane);

        if d0 * d1 > tol * tol {
            continue;
        }

        let mut ta = span_t0;
        let mut tb = span_t1;
        let mut da = d0;

        for _ in 0..20 {
            if (tb - ta).abs() < tol * 0.1 {
                break;
            }
            let tm = (ta + tb) * 0.5;
            let dm = curve_signed_distance_to_plane(&curve.point_at(tm), plane);
            if dm.abs() < tol {
                ta = tm;
                tb = tm;
                break;
            }
            if da * dm < 0.0 {
                tb = tm;
            } else {
                ta = tm;
                da = dm;
            }
        }

        let mut t = (ta + tb) * 0.5;

        for _ in 0..15 {
            let pt = curve.point_at(t);
            let f = curve_signed_distance_to_plane(&pt, plane);
            if f.abs() < tol {
                break;
            }
            let tan = curve.tangent_at(t);
            let df = plane.z_axis().dot(&tan);
            if df.abs() < 1e-10 {
                if f * da < 0.0 {
                    t = (ta + t) * 0.5;
                } else {
                    t = (t + tb) * 0.5;
                }
                continue;
            }
            let dt = -f / df;
            let t_new = t + dt;
            let t_new = t_new.max(span_t0).min(span_t1);
            if dt.abs() < tol * 0.01 {
                t = t_new;
                break;
            }
            t = t_new;
        }

        let pt_final = curve.point_at(t);
        if curve_signed_distance_to_plane(&pt_final, plane).abs() < tol {
            let is_duplicate = results.iter().any(|&existing_t: &f64| (t - existing_t).abs() < tol * 2.0);
            if !is_duplicate {
                results.push(t);
            }
        }
    }

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());
    results
}

/// Curve-plane intersection using production CAD kernel method
pub fn curve_plane_production(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<f64> {
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

    if !curve.is_valid() {
        return Vec::new();
    }

    let mut results = Vec::new();
    let spans = curve.get_span_vector();

    for span_idx in 0..(spans.len() - 1) {
        let span_t0 = spans[span_idx];
        let span_t1 = spans[span_idx + 1];

        if (span_t1 - span_t0).abs() < tol {
            continue;
        }

        fn subdivide_and_solve(
            curve: &NurbsCurve,
            plane: &Plane,
            ta: f64,
            tb: f64,
            depth: i32,
            tol: f64,
            results: &mut Vec<f64>,
        ) {
            if depth > 30 {
                return;
            }

            let pa = curve.point_at(ta);
            let pb = curve.point_at(tb);
            let da = curve_signed_distance_to_plane(&pa, plane);
            let db = curve_signed_distance_to_plane(&pb, plane);

            if da * db > tol * tol {
                return;
            }

            let segment_length = pa.distance(&pb, None);
            if segment_length < tol * 10.0 || (tb - ta).abs() < tol * 0.001 {
                let t_init = if (db - da).abs() > tol {
                    (ta - da * (tb - ta) / (db - da)).max(ta).min(tb)
                } else {
                    (ta + tb) * 0.5
                };

                let mut t = t_init;
                for _ in 0..5 {
                    let pt = curve.point_at(t);
                    let f = curve_signed_distance_to_plane(&pt, plane);
                    if f.abs() < tol {
                        if ta <= t && t <= tb {
                            let is_duplicate = results.iter().any(|&e| (t - e).abs() < tol * 2.0);
                            if !is_duplicate {
                                results.push(t);
                            }
                        }
                        return;
                    }
                    let df = curve.tangent_at(t).dot(&plane.z_axis());
                    if df.abs() < 1e-10 {
                        t = (ta + tb) * 0.5;
                        break;
                    }
                    let dt = -f / df;
                    t = (t + dt).max(ta).min(tb);
                    if dt.abs() < tol * 0.001 {
                        break;
                    }
                }

                let pt_final = curve.point_at(t);
                if curve_signed_distance_to_plane(&pt_final, plane).abs() < tol && ta <= t && t <= tb {
                    let is_duplicate = results.iter().any(|&e| (t - e).abs() < tol * 2.0);
                    if !is_duplicate {
                        results.push(t);
                    }
                }
                return;
            }

            let tm = (ta + tb) * 0.5;
            let pm = curve.point_at(tm);

            let v = Vector::new(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
            let w = Vector::new(pm[0] - pa[0], pm[1] - pa[1], pm[2] - pa[2]);

            if v.magnitude() > Tolerance::ZERO_TOLERANCE {
                let t_proj = w.dot(&v) / v.dot(&v);
                let p_proj = Point::new(
                    pa[0] + t_proj * v[0],
                    pa[1] + t_proj * v[1],
                    pa[2] + t_proj * v[2],
                );
                let deviation = pm.distance(&p_proj, None);

                if deviation < tol * 10.0 {
                    if (db - da).abs() > tol {
                        let mut t_root = (ta - da * (tb - ta) / (db - da)).max(ta).min(tb);
                        for _ in 0..3 {
                            let pt = curve.point_at(t_root);
                            let f = curve_signed_distance_to_plane(&pt, plane);
                            if f.abs() < tol {
                                break;
                            }
                            let df = curve.tangent_at(t_root).dot(&plane.z_axis());
                            if df.abs() > 1e-10 {
                                t_root = (t_root - f / df).max(ta).min(tb);
                            }
                        }

                        if curve_signed_distance_to_plane(&curve.point_at(t_root), plane).abs() < tol {
                            let is_duplicate = results.iter().any(|&e| (t_root - e).abs() < tol * 2.0);
                            if !is_duplicate {
                                results.push(t_root);
                            }
                        }
                    }
                    return;
                }
            }

            subdivide_and_solve(curve, plane, ta, tm, depth + 1, tol, results);
            subdivide_and_solve(curve, plane, tm, tb, depth + 1, tol, results);
        }

        subdivide_and_solve(curve, plane, span_t0, span_t1, 0, tol, &mut results);
    }

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());
    results
}

/// Find closest point on NURBS curve to test point
pub fn curve_closest_point(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64) -> (f64, f64) {
    Closest::curve_point(curve, test_point, t0, t1)
}
