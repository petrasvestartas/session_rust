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

use crate::{NurbsCurve, NurbsSurface, Plane, Tolerance, Vector};
use crate::knot::CurveKnotStyle;

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

/// Find intersection curves between a NURBS surface and a plane
/// Find intersection points between a ray (Line) and a mesh using brute-force triangle testing.
pub fn ray_mesh(line: &Line, mesh: &crate::Mesh, epsilon: f64, find_all: bool) -> Option<Vec<Point>> {
    let (vertices, faces) = mesh.to_vertices_and_faces();
    let mut tris: Vec<(Point, Point, Point)> = Vec::new();
    for face in &faces {
        if face.len() < 3 { continue; }
        let v0 = &vertices[face[0]];
        for j in 1..face.len() - 1 {
            tris.push((v0.clone(), vertices[face[j]].clone(), vertices[face[j + 1]].clone()));
        }
    }
    if tris.is_empty() { return None; }

    let origin = line.start();
    let direction = line.to_vector().normalized();
    let mut hits: Vec<(f64, Point)> = Vec::new();

    for (v0, v1, v2) in &tris {
        if let Some(p) = ray_triangle(line, v0, v1, v2, epsilon) {
            let t = (p[0] - origin[0]) * direction[0]
                  + (p[1] - origin[1]) * direction[1]
                  + (p[2] - origin[2]) * direction[2];
            if t >= 0.0 {
                hits.push((t, p));
            }
        }
    }

    if hits.is_empty() { return None; }
    hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    if find_all {
        Some(hits.into_iter().map(|(_, p)| p).collect())
    } else {
        Some(vec![hits[0].1.clone()])
    }
}

/// Find intersection points between a ray (Line) and a mesh using BVH acceleration.
pub fn ray_mesh_bvh(line: &Line, mesh: &crate::Mesh, epsilon: f64, find_all: bool) -> Option<Vec<Point>> {
    let (vertices, faces) = mesh.to_vertices_and_faces();
    let mut tris: Vec<(Point, Point, Point)> = Vec::new();
    for face in &faces {
        if face.len() < 3 { continue; }
        let v0 = &vertices[face[0]];
        for j in 1..face.len() - 1 {
            tris.push((v0.clone(), vertices[face[j]].clone(), vertices[face[j + 1]].clone()));
        }
    }
    if tris.is_empty() { return None; }

    let tri_boxes: Vec<crate::BoundingBox> = tris.iter()
        .map(|(v0, v1, v2)| crate::BoundingBox::from_points(&[v0.clone(), v1.clone(), v2.clone()], 0.0))
        .collect();

    let world_size = crate::BVH::compute_world_size(&tri_boxes);
    let bvh = crate::BVH::from_boxes(&tri_boxes, world_size);

    let origin = line.start();
    let direction = line.to_vector().normalized();
    let mut candidate_ids: Vec<usize> = Vec::new();
    let found = bvh.ray_cast(&origin, &direction, &mut candidate_ids, true);
    if !found { return None; }

    let mut hits: Vec<(f64, Point)> = Vec::new();
    for idx in candidate_ids {
        if idx >= tris.len() { continue; }
        let (ref v0, ref v1, ref v2) = tris[idx];
        if let Some(p) = ray_triangle(line, v0, v1, v2, epsilon) {
            let t = (p[0] - origin[0]) * direction[0]
                  + (p[1] - origin[1]) * direction[1]
                  + (p[2] - origin[2]) * direction[2];
            if t >= 0.0 {
                hits.push((t, p));
            }
        }
    }

    if hits.is_empty() { return None; }
    hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    if find_all {
        Some(hits.into_iter().map(|(_, p)| p).collect())
    } else {
        Some(vec![hits[0].1.clone()])
    }
}

/// Find intersection curves between a NURBS surface and a plane
pub fn surface_plane(surface: &NurbsSurface, plane: &Plane, tolerance: Option<f64>) -> Vec<NurbsCurve> {
    if !surface.is_valid() { return vec![]; }
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE).max(Tolerance::ZERO_TOLERANCE);

    let (u0, u1) = match surface.domain(0) { Some(d) => d, None => return vec![] };
    let (v0, v1) = match surface.domain(1) { Some(d) => d, None => return vec![] };
    let range_u = u1 - u0;
    let range_v = v1 - v0;
    let closed_u = surface.is_closed(0);
    let closed_v = surface.is_closed(1);

    let wrap_u = |u: f64| -> f64 {
        if closed_u {
            let mut t = (u - u0) % range_u;
            if t < 0.0 { t += range_u; }
            return u0 + t;
        }
        u.max(u0).min(u1)
    };
    let wrap_v = |v: f64| -> f64 {
        if closed_v {
            let mut t = (v - v0) % range_v;
            if t < 0.0 { t += range_v; }
            return v0 + t;
        }
        v.max(v0).min(v1)
    };

    let pn = plane.z_axis();
    let p0 = plane.origin();

    let g = |u: f64, v: f64| -> f64 {
        let p = surface.point_at(wrap_u(u), wrap_v(v)).unwrap_or(Point::new(0.0, 0.0, 0.0));
        (p[0] - p0[0]) * pn[0] + (p[1] - p0[1]) * pn[1] + (p[2] - p0[2]) * pn[2]
    };

    let g_and_grad = |u: f64, v: f64| -> (f64, f64, f64) {
        let derivs = surface.evaluate(wrap_u(u), wrap_v(v), 1);
        if derivs.len() < 3 { return (g(u, v), 0.0, 0.0); }
        let s = &derivs[0];
        let su = &derivs[2];
        let sv = &derivs[1];
        let val = (s[0] - p0[0]) * pn[0] + (s[1] - p0[1]) * pn[1] + (s[2] - p0[2]) * pn[2];
        let gu = su[0] * pn[0] + su[1] * pn[1] + su[2] * pn[2];
        let gv = sv[0] * pn[0] + sv[1] * pn[1] + sv[2] * pn[2];
        (val, gu, gv)
    };

    let newton_correct = |u: &mut f64, v: &mut f64| -> bool {
        for _ in 0..10 {
            let (val, gu, gv) = g_and_grad(*u, *v);
            if val.abs() < tol { return true; }
            let mag2 = gu * gu + gv * gv;
            if mag2 < 1e-28 { return false; }
            *u -= val * gu / mag2;
            *v -= val * gv / mag2;
            *u = wrap_u(*u);
            *v = wrap_v(*v);
        }
        g(*u, *v).abs() < tol * 10.0
    };

    // 1. Find seeds
    let spans_u = surface.get_span_vector(0);
    let spans_v = surface.get_span_vector(1);
    let nu = (spans_u.len() as i32 - 1).max(1) * 4;
    let nv = (spans_v.len() as i32 - 1).max(1) * 4;
    let du = range_u / nu as f64;
    let dv = range_v / nv as f64;

    let mu = (u0 + u1) * 0.5;
    let mv = (v0 + v1) * 0.5;
    let pmid = surface.point_at(mu, mv).unwrap_or(Point::new(0.0, 0.0, 0.0));
    let pmid_u = surface.point_at(wrap_u(mu + du), mv).unwrap_or(Point::new(0.0, 0.0, 0.0));
    let pmid_v = surface.point_at(mu, wrap_v(mv + dv)).unwrap_or(Point::new(0.0, 0.0, 0.0));
    let uv_to_3d_u = pmid.distance(&pmid_u, None) / du;
    let uv_to_3d_v = pmid.distance(&pmid_v, None) / dv;
    let mut uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);
    let mut uv_to_3d_min = uv_to_3d_u.min(uv_to_3d_v);
    if uv_to_3d < 1e-10 { uv_to_3d = 1.0; }
    if uv_to_3d_min < 1e-10 { uv_to_3d_min = 1.0; }

    let cols = nv + 1;
    let mut dist = vec![0.0f64; ((nu + 1) * cols) as usize];
    for i in 0..=nu {
        let u = u0 + du * i as f64;
        for j in 0..=nv {
            let v = v0 + dv * j as f64;
            let mut d = g(u, v);
            if d == 0.0 { d = -1e-14; }
            dist[(i * cols + j) as usize] = d;
        }
    }

    struct Seed { u: f64, v: f64, used: bool }
    let mut seeds: Vec<Seed> = Vec::new();

    // Horizontal edges
    let h_jmax = if closed_v { nv - 1 } else { nv };
    for i in 0..nu {
        for j in 0..=h_jmax {
            let d0 = dist[(i * cols + j) as usize];
            let d1 = dist[((i + 1) * cols + j) as usize];
            if d0 * d1 < 0.0 {
                let t = d0 / (d0 - d1);
                let mut su = u0 + du * (i as f64 + t);
                let mut sv = v0 + dv * j as f64;
                if newton_correct(&mut su, &mut sv) {
                    seeds.push(Seed { u: su, v: sv, used: false });
                }
            }
        }
    }
    // Vertical edges
    let v_imax = if closed_u { nu - 1 } else { nu };
    for i in 0..=v_imax {
        for j in 0..nv {
            let d0 = dist[(i * cols + j) as usize];
            let d1 = dist[(i * cols + j + 1) as usize];
            if d0 * d1 < 0.0 {
                let t = d0 / (d0 - d1);
                let mut su = u0 + du * i as f64;
                let mut sv = v0 + dv * (j as f64 + t);
                if newton_correct(&mut su, &mut sv) {
                    seeds.push(Seed { u: su, v: sv, used: false });
                }
            }
        }
    }

    // Deduplicate seeds (3D distance)
    let seed_tol_3d = (du.max(dv)) * uv_to_3d;
    for i in 0..seeds.len() {
        if seeds[i].used { continue; }
        let pi = surface.point_at(seeds[i].u, seeds[i].v).unwrap_or(Point::new(0.0, 0.0, 0.0));
        for j in (i + 1)..seeds.len() {
            if seeds[j].used { continue; }
            let pj = surface.point_at(seeds[j].u, seeds[j].v).unwrap_or(Point::new(0.0, 0.0, 0.0));
            if pi.distance(&pj, None) < seed_tol_3d {
                seeds[j].used = true;
            }
        }
    }

    // 2. Trace intersection curves
    let step = du.min(dv) * 0.25;
    let max_steps = (nu * nv * 32) as usize;
    let close_tol_3d = step * 4.0 * uv_to_3d_min;
    let consume_tol_3d = step * uv_to_3d * 2.0;

    let mut result: Vec<NurbsCurve> = Vec::new();

    for seed_idx in 0..seeds.len() {
        if seeds[seed_idx].used { continue; }
        seeds[seed_idx].used = true;
        let seed_u = seeds[seed_idx].u;
        let seed_v = seeds[seed_idx].v;

        // Tangent at UV from analytical gradient
        let tangent_at_uv = |u: f64, v: f64, dir: f64| -> Option<(f64, f64)> {
            let (_, gu, gv) = g_and_grad(u, v);
            let mag = f64::hypot(gu, gv);
            if mag < 1e-14 { return None; }
            Some((-gv / mag * dir, gu / mag * dir))
        };

        // Trace one direction; returns (points, closed)
        let trace_dir = |su: f64, sv: f64, dir: f64, seeds: &mut Vec<Seed>| -> (Vec<(f64, f64)>, bool) {
            let mut out: Vec<(f64, f64)> = Vec::new();
            let mut u = su;
            let mut v = sv;
            let mut prev_tu = 0.0f64;
            let mut prev_tv = 0.0f64;
            let p_start = surface.point_at(su, sv).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let mut p_prev = p_start.clone();
            let mut dist_traveled = 0.0f64;

            for _ in 0..max_steps {
                let (mut tu, mut tv) = match tangent_at_uv(u, v, dir) {
                    Some(t) => t,
                    None => {
                        if f64::hypot(prev_tu, prev_tv) < 1e-14 { break; }
                        (prev_tu, prev_tv)
                    }
                };

                // Adaptive step
                let mut local_step = step;
                if f64::hypot(prev_tu, prev_tv) > 1e-14 {
                    let dot = (tu * prev_tu + tv * prev_tv).max(-1.0).min(1.0);
                    if dot < 0.95 { local_step = step * 0.25; }
                    else if dot < 0.985 { local_step = step * 0.5; }
                }

                // RK2: midpoint tangent
                let u_mid = u + local_step * 0.5 * tu;
                let v_mid = v + local_step * 0.5 * tv;
                if let Some((tu2, tv2)) = tangent_at_uv(u_mid, v_mid, dir) {
                    tu = tu2;
                    tv = tv2;
                }
                prev_tu = tu;
                prev_tv = tv;

                let mut un = u + local_step * tu;
                let mut vn = v + local_step * tv;

                let mut hit_boundary = false;
                if (!closed_u && (un < u0 || un > u1)) || (!closed_v && (vn < v0 || vn > v1)) {
                    let mut tc = 1.0f64;
                    if !closed_u && tu > 0.0 && un > u1 { tc = tc.min((u1 - u) / (local_step * tu)); }
                    if !closed_u && tu < 0.0 && un < u0 { tc = tc.min((u0 - u) / (local_step * tu)); }
                    if !closed_v && tv > 0.0 && vn > v1 { tc = tc.min((v1 - v) / (local_step * tv)); }
                    if !closed_v && tv < 0.0 && vn < v0 { tc = tc.min((v0 - v) / (local_step * tv)); }
                    un = u + tc * local_step * tu;
                    vn = v + tc * local_step * tv;
                    hit_boundary = true;
                }
                un = wrap_u(un);
                vn = wrap_v(vn);

                if !newton_correct(&mut un, &mut vn) { break; }

                let p_cur = surface.point_at(un, vn).unwrap_or(Point::new(0.0, 0.0, 0.0));
                dist_traveled += p_prev.distance(&p_cur, None);

                // Loop closure
                if dist_traveled > close_tol_3d * 3.0 && p_start.distance(&p_cur, None) < close_tol_3d {
                    out.push((un, vn));
                    return (out, true);
                }

                out.push((un, vn));
                u = un;
                v = vn;
                p_prev = p_cur.clone();

                if hit_boundary { break; }

                for other in seeds.iter_mut() {
                    if !other.used {
                        let po = surface.point_at(other.u, other.v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                        if p_cur.distance(&po, None) < consume_tol_3d {
                            other.used = true;
                        }
                    }
                }
            }
            (out, false)
        };

        let (fwd, fwd_closed) = trace_dir(seed_u, seed_v, 1.0, &mut seeds);
        let bwd = if !fwd_closed {
            trace_dir(seed_u, seed_v, -1.0, &mut seeds).0
        } else {
            Vec::new()
        };

        // Assemble UV trace: reverse(bwd) + seed + fwd
        let mut uv_trace: Vec<(f64, f64)> = Vec::with_capacity(bwd.len() + 1 + fwd.len());
        for i in (0..bwd.len()).rev() {
            uv_trace.push(bwd[i]);
        }
        uv_trace.push((seed_u, seed_v));
        for p in &fwd {
            uv_trace.push(*p);
        }

        if uv_trace.len() < 4 { continue; }

        // Detect closed loops
        let p_first = surface.point_at(uv_trace[0].0, uv_trace[0].1).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let p_last = surface.point_at(uv_trace.last().unwrap().0, uv_trace.last().unwrap().1).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let is_loop = fwd_closed || (uv_trace.len() >= 6 && p_first.distance(&p_last, None) < close_tol_3d);
        if is_loop { uv_trace.pop(); }
        if uv_trace.len() < 4 { continue; }

        // Unwrap UV trace for smooth interpolation across seams
        let mut uv_unwrapped = uv_trace.clone();
        for i in 1..uv_unwrapped.len() {
            let du_jump = uv_unwrapped[i].0 - uv_unwrapped[i - 1].0;
            let dv_jump = uv_unwrapped[i].1 - uv_unwrapped[i - 1].1;
            if closed_u {
                if du_jump > range_u * 0.5 { uv_unwrapped[i].0 -= range_u; }
                else if du_jump < -range_u * 0.5 { uv_unwrapped[i].0 += range_u; }
            }
            if closed_v {
                if dv_jump > range_v * 0.5 { uv_unwrapped[i].1 -= range_v; }
                else if dv_jump < -range_v * 0.5 { uv_unwrapped[i].1 += range_v; }
            }
        }

        // 3. Evaluate all trace points to 3D
        let all_pts: Vec<Point> = uv_trace.iter()
            .map(|&(u, v)| surface.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0)))
            .collect();

        // 4. Circle detection
        let mut crv = NurbsCurve::new(3, false, 4, 0);
        if is_loop && all_pts.len() >= 6 {
            let ax = plane.x_axis();
            let ay = plane.y_axis();
            let po = plane.origin();
            let to2d = |p: &Point| -> (f64, f64) {
                let dx = p[0] - po[0];
                let dy = p[1] - po[1];
                let dz = p[2] - po[2];
                (dx * ax[0] + dy * ax[1] + dz * ax[2], dx * ay[0] + dy * ay[1] + dz * ay[2])
            };

            let n = all_pts.len();
            let (x1, y1) = to2d(&all_pts[0]);
            let (x2, y2) = to2d(&all_pts[n / 3]);
            let (x3, y3) = to2d(&all_pts[2 * n / 3]);

            let ax_ = x2 - x1;
            let ay_ = y2 - y1;
            let bx_ = x3 - x1;
            let by_ = y3 - y1;
            let d_val = 2.0 * (ax_ * by_ - ay_ * bx_);

            if d_val.abs() > 1e-10 {
                let a2 = ax_ * ax_ + ay_ * ay_;
                let b2 = bx_ * bx_ + by_ * by_;
                let ccx = x1 + (by_ * a2 - ay_ * b2) / d_val;
                let ccy = y1 + (ax_ * b2 - bx_ * a2) / d_val;
                let radius = f64::hypot(x1 - ccx, y1 - ccy);

                let mut max_dev = 0.0f64;
                for p in &all_pts {
                    let (px, py) = to2d(p);
                    max_dev = max_dev.max((f64::hypot(px - ccx, py - ccy) - radius).abs());
                }

                let circle_tol = (radius * 1e-4).max(1e-6);
                if radius > 1e-10 && max_dev < circle_tol {
                    let cx3d = po[0] + ccx * ax[0] + ccy * ay[0];
                    let cy3d = po[1] + ccx * ax[1] + ccy * ay[1];
                    let cz3d = po[2] + ccx * ax[2] + ccy * ay[2];

                    let w = std::f64::consts::FRAC_1_SQRT_2;
                    let cx_: [f64; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
                    let cy_: [f64; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
                    let wts: [f64; 9] = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
                    crv = NurbsCurve::new(3, true, 3, 9);
                    let knots: [f64; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
                    for i in 0..10 { crv.set_knot(i, knots[i]); }
                    for i in 0..9 {
                        let px = cx3d + radius * (cx_[i] * ax[0] + cy_[i] * ay[0]);
                        let py = cy3d + radius * (cx_[i] * ax[1] + cy_[i] * ay[1]);
                        let pz = cz3d + radius * (cx_[i] * ax[2] + cy_[i] * ay[2]);
                        crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                    }
                }
            }
        }

        // 4b. Ellipse (conic) detection for non-circular closed curves
        if !crv.is_valid() && is_loop && all_pts.len() >= 8 {
            let ax = plane.x_axis();
            let ay = plane.y_axis();
            let po = plane.origin();
            let to2d = |p: &Point| -> (f64, f64) {
                let dx = p[0] - po[0];
                let dy = p[1] - po[1];
                let dz = p[2] - po[2];
                (dx * ax[0] + dy * ax[1] + dz * ax[2], dx * ay[0] + dy * ay[1] + dz * ay[2])
            };

            let n = all_pts.len();
            // Build normal equations (5x5 symmetric system)
            let mut ata = [[0.0f64; 5]; 5];
            let mut atb = [0.0f64; 5];
            for i in 0..n {
                let (x, y) = to2d(&all_pts[i]);
                let row = [x * x, x * y, y * y, x, y];
                for r in 0..5 {
                    atb[r] += row[r];
                    for c in 0..5 {
                        ata[r][c] += row[r] * row[c];
                    }
                }
            }
            // Solve 5x5 system by Gaussian elimination
            let mut m_mat = [[0.0f64; 6]; 5];
            for r in 0..5 {
                for c in 0..5 { m_mat[r][c] = ata[r][c]; }
                m_mat[r][5] = atb[r];
            }
            let mut ok = true;
            for col in 0..5 {
                if !ok { break; }
                let mut pivot = col;
                for r in (col + 1)..5 {
                    if m_mat[r][col].abs() > m_mat[pivot][col].abs() { pivot = r; }
                }
                if m_mat[pivot][col].abs() < 1e-20 { ok = false; break; }
                if pivot != col {
                    for j in col..=5 {
                        let tmp = m_mat[col][j];
                        m_mat[col][j] = m_mat[pivot][j];
                        m_mat[pivot][j] = tmp;
                    }
                }
                for r in (col + 1)..5 {
                    let f = m_mat[r][col] / m_mat[col][col];
                    for j in col..=5 { m_mat[r][j] -= f * m_mat[col][j]; }
                }
            }
            let mut coef = [0.0f64; 5];
            if ok {
                for i in (0..5).rev() {
                    let mut s = m_mat[i][5];
                    for j in (i + 1)..5 { s -= m_mat[i][j] * coef[j]; }
                    coef[i] = s / m_mat[i][i];
                }
            }
            let ca = coef[0];
            let cb = coef[1];
            let cc = coef[2];
            let cd = coef[3];
            let ce = coef[4];
            let disc = cb * cb - 4.0 * ca * cc;

            if ok && disc < -1e-10 && ca.abs() > 1e-14 {
                let mut max_conic_dev = 0.0f64;
                for p in &all_pts {
                    let (x, y) = to2d(p);
                    let val = ca * x * x + cb * x * y + cc * y * y + cd * x + ce * y - 1.0;
                    max_conic_dev = max_conic_dev.max(val.abs());
                }
                let scale = ca.abs().max(cc.abs());
                let norm_dev = max_conic_dev / scale.max(1e-10);

                if norm_dev < 0.01 {
                    let det = 4.0 * ca * cc - cb * cb;
                    let cx = (cb * ce - 2.0 * cc * cd) / det;
                    let cy = (cb * cd - 2.0 * ca * ce) / det;
                    let theta = 0.5 * f64::atan2(cb, ca - cc);
                    let cos_t = theta.cos();
                    let sin_t = theta.sin();
                    let a2 = ca * cos_t * cos_t + cb * cos_t * sin_t + cc * sin_t * sin_t;
                    let c2 = ca * sin_t * sin_t - cb * cos_t * sin_t + cc * cos_t * cos_t;
                    let f_val = ca * cx * cx + cb * cx * cy + cc * cy * cy + cd * cx + ce * cy - 1.0;
                    let rhs = -f_val;

                    if rhs > 1e-14 && a2 > 1e-14 && c2 > 1e-14 {
                        let semi_a = (rhs / a2).sqrt();
                        let semi_b = (rhs / c2).sqrt();

                        let cx3d = po[0] + cx * ax[0] + cy * ay[0];
                        let cy3d = po[1] + cx * ax[1] + cy * ay[1];
                        let cz3d = po[2] + cx * ax[2] + cy * ay[2];

                        let ea = Vector::new(
                            cos_t * ax[0] + sin_t * ay[0],
                            cos_t * ax[1] + sin_t * ay[1],
                            cos_t * ax[2] + sin_t * ay[2],
                        );
                        let eb = Vector::new(
                            -sin_t * ax[0] + cos_t * ay[0],
                            -sin_t * ax[1] + cos_t * ay[1],
                            -sin_t * ax[2] + cos_t * ay[2],
                        );

                        let w = std::f64::consts::FRAC_1_SQRT_2;
                        let cx_: [f64; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
                        let cy_: [f64; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
                        let wts: [f64; 9] = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
                        crv = NurbsCurve::new(3, true, 3, 9);
                        let knots: [f64; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
                        for i in 0..10 { crv.set_knot(i, knots[i]); }
                        for i in 0..9 {
                            let px = cx3d + semi_a * cx_[i] * ea[0] + semi_b * cy_[i] * eb[0];
                            let py = cy3d + semi_a * cx_[i] * ea[1] + semi_b * cy_[i] * eb[1];
                            let pz = cz3d + semi_a * cx_[i] * ea[2] + semi_b * cy_[i] * eb[2];
                            crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                        }

                        // Verify ellipse fit
                        let mut max_ell_dev = 0.0f64;
                        for p in &all_pts {
                            let (px2, py2) = to2d(p);
                            let lx = cos_t * (px2 - cx) + sin_t * (py2 - cy);
                            let ly = -sin_t * (px2 - cx) + cos_t * (py2 - cy);
                            let ang = f64::atan2(ly / semi_b, lx / semi_a);
                            let ex = cx + semi_a * ang.cos() * cos_t - semi_b * ang.sin() * sin_t;
                            let ey = cy + semi_a * ang.cos() * sin_t + semi_b * ang.sin() * cos_t;
                            let dev = f64::hypot(px2 - ex, py2 - ey);
                            max_ell_dev = max_ell_dev.max(dev);
                        }
                        let ell_tol = semi_a.max(semi_b) * 5e-3;
                        if max_ell_dev > ell_tol {
                            crv = NurbsCurve::new(3, false, 4, 0); // reject
                        }
                    }
                }
            }
        }

        // 5. 2D plane-constrained fitting for non-circular/elliptical curves
        if !crv.is_valid() {
            let m = all_pts.len();
            if m < 4 { continue; }

            let ax = plane.x_axis();
            let ay = plane.y_axis();
            let po = plane.origin();
            let mut pts_2d: Vec<Point> = Vec::with_capacity(m);
            for i in 0..m {
                let dx = all_pts[i][0] - po[0];
                let dy = all_pts[i][1] - po[1];
                let dz = all_pts[i][2] - po[2];
                let px = dx * ax[0] + dy * ax[1] + dz * ax[2];
                let py = dx * ay[0] + dy * ay[1] + dz * ay[2];
                pts_2d.push(Point::new(px, py, 0.0));
            }

            // Chord-length params
            let mut chords = vec![0.0f64; m];
            let mut total_len = 0.0f64;
            for i in 1..m {
                total_len += pts_2d[i].distance(&pts_2d[i - 1], None);
                chords[i] = total_len;
            }
            if is_loop && m > 1 {
                total_len += pts_2d[0].distance(&pts_2d[m - 1], None);
            }
            if total_len > 1e-14 {
                for i in 1..m { chords[i] /= total_len; }
            }

            let fit_tol = step * (uv_to_3d + uv_to_3d_min) * 0.5;
            let mut total_turning = 0.0f64;
            for i in 1..(m - 1) {
                let dx1 = pts_2d[i][0] - pts_2d[i - 1][0];
                let dy1 = pts_2d[i][1] - pts_2d[i - 1][1];
                let dx2 = pts_2d[i + 1][0] - pts_2d[i][0];
                let dy2 = pts_2d[i + 1][1] - pts_2d[i][1];
                let l1 = f64::hypot(dx1, dy1);
                let l2 = f64::hypot(dx2, dy2);
                if l1 > 1e-14 && l2 > 1e-14 {
                    let c = ((dx1 * dx2 + dy1 * dy2) / (l1 * l2)).max(-1.0).min(1.0);
                    total_turning += c.acos();
                }
            }
            let mut target_cvs = 8_i32.max((total_turning / 0.5) as i32 + 6);
            let max_cvs = (m as i32) - 1;
            let mut crv_2d = NurbsCurve::new(3, false, 4, 0);
            for _ in 0..5 {
                if target_cvs > max_cvs { break; }
                crv_2d = NurbsCurve::create_fitted(&pts_2d, target_cvs as usize, 3, is_loop);
                if !crv_2d.is_valid() { break; }
                let (ft0, ft1) = crv_2d.domain();
                let mut max_dev = 0.0f64;
                for i in 0..m {
                    let t = ft0 + (ft1 - ft0) * chords[i];
                    max_dev = max_dev.max(crv_2d.point_at(t).distance(&pts_2d[i], None));
                }
                if max_dev < fit_tol { break; }
                target_cvs = (target_cvs * 2).min(max_cvs);
            }
            if !crv_2d.is_valid() {
                crv_2d = if is_loop {
                    NurbsCurve::create_interpolated(&pts_2d, CurveKnotStyle::ChordPeriodic)
                } else {
                    NurbsCurve::create_interpolated(&pts_2d, CurveKnotStyle::Chord)
                };
            }

            // Lift 2D CVs back to 3D
            if crv_2d.is_valid() {
                crv = crv_2d;
                for i in 0..crv.cv_count() {
                    if let Some(cv2) = crv.get_cv(i) {
                        let cx = cv2[0];
                        let cy = cv2[1];
                        crv.set_cv(i, &Point::new(
                            po[0] + cx * ax[0] + cy * ay[0],
                            po[1] + cx * ax[1] + cy * ay[1],
                            po[2] + cx * ax[2] + cy * ay[2],
                        ));
                    }
                }
            }
        }
        if !crv.is_valid() { continue; }

        // Deduplicate: skip if ALL sample points are close to an existing curve
        let (ct0, ct1) = crv.domain();
        let dup_tol = step * uv_to_3d * 3.0;
        let mut dup = false;
        for existing in &result {
            let (et0, et1) = existing.domain();
            let mut all_close = true;
            for &f in &[0.25, 0.5, 0.75] {
                let cp = crv.point_at(ct0 + (ct1 - ct0) * f);
                let ep = existing.point_at(et0 + (et1 - et0) * f);
                let em = existing.point_at((et0 + et1) * 0.5);
                let d = cp.distance(&ep, None).min(cp.distance(&em, None));
                if d > dup_tol { all_close = false; break; }
            }
            if all_close { dup = true; break; }
        }
        if !dup { result.push(crv); }
    }

    result
}
