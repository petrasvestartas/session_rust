use crate::{Line, Point};
use crate::closest::Closest;

pub fn line_line_parameters(
    line0: &Line,
    line1: &Line,
    tolerance: f32,
    intersect_segments: bool,
    near_parallel_as_closest: bool,
) -> Option<(f32, f32)> {
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

    let zero_tol = aa.max(bb) * f32::EPSILON;
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
pub fn line_line(line0: &Line, line1: &Line, tolerance: f32) -> Option<Point> {
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

/// Plane-plane intersection with CGAL-canonical anchor (foot-of-perpendicular
/// from world origin onto the intersection line). Independent of input-plane
/// origin choice — matches wood's `cgal::intersection_util::plane_plane`
/// (cgal_intersection_util.cpp:493-511) bit-for-bit, giving identical results
/// to wood for parallel input planes.
pub fn plane_plane_to_line_canonical(plane0: &crate::Plane, plane1: &crate::Plane) -> Option<Line> {
    let n0 = plane0.z_axis();
    let n1 = plane1.z_axis();
    let d = crate::Vector::new(
        n1[1] * n0[2] - n1[2] * n0[1],
        n1[2] * n0[0] - n1[0] * n0[2],
        n1[0] * n0[1] - n1[1] * n0[0],
    );
    let d_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if d_sq < 1e-20 {
        return None;
    }

    let o0 = plane0.origin();
    let o1 = plane1.origin();
    let k0 = n0[0] * o0[0] + n0[1] * o0[1] + n0[2] * o0[2];
    let k1 = n1[0] * o1[0] + n1[1] * o1[1] + n1[2] * o1[2];
    let n0n0 = n0[0] * n0[0] + n0[1] * n0[1] + n0[2] * n0[2];
    let n1n1 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];
    let n0n1 = n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2];
    let det = n0n0 * n1n1 - n0n1 * n0n1;
    if det.abs() < 1e-20 {
        return None;
    }
    let c0 = (k0 * n1n1 - k1 * n0n1) / det;
    let c1 = (k1 * n0n0 - k0 * n0n1) / det;
    let anchor = Point::new(
        c0 * n0[0] + c1 * n1[0],
        c0 * n0[1] + c1 * n1[1],
        c0 * n0[2] + c1 * n1[2],
    );
    Some(Line::from_points(
        &anchor,
        &Point::new(anchor[0] + d[0], anchor[1] + d[1], anchor[2] + d[2]),
    ))
}

fn plane_value_at(plane: &crate::Plane, point: &Point) -> f32 {
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
        if fd > 1.0 && (a.abs() >= f32::MAX / fd || b.abs() >= f32::MAX / fd) {
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
pub fn ray_box(line: &Line, box_: &crate::OBB, t0: f32, t1: f32) -> Option<Vec<Point>> {
    let origin = line.start();
    let direction = line.to_vector();

    let box_min = box_.min_point();
    let box_max = box_.max_point();

    // Calculate inverse direction (avoid division by zero)
    let inv_dir_x = if direction[0] != 0.0 {
        1.0 / direction[0]
    } else {
        f32::INFINITY
    };
    let inv_dir_y = if direction[1] != 0.0 {
        1.0 / direction[1]
    } else {
        f32::INFINITY
    };
    let inv_dir_z = if direction[2] != 0.0 {
        1.0 / direction[2]
    } else {
        f32::INFINITY
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
pub fn ray_sphere(line: &Line, center: &Point, radius: f32) -> Option<Vec<Point>> {
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
    epsilon: f32,
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
use crate::nurbsknot::CurveNurbsKnotStyle;

fn curve_signed_distance_to_plane(pt: &Point, plane: &Plane) -> f32 {
    let v = Vector::new(
        pt[0] - plane.origin()[0],
        pt[1] - plane.origin()[1],
        pt[2] - plane.origin()[2],
    );
    v.dot(&plane.z_axis())
}

/// Find all intersections between NURBS curve and plane
pub fn curve_plane(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f32>) -> Vec<f32> {
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
pub fn curve_plane_points(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f32>) -> Vec<Point> {
    curve_plane(curve, plane, tolerance)
        .iter()
        .map(|&t| curve.point_at(t))
        .collect()
}

/// Curve-plane intersection using Bézier clipping (advanced method)
pub fn curve_plane_bezier_clipping(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f32>) -> Vec<f32> {
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

    if !curve.is_valid() {
        return Vec::new();
    }

    let mut results = Vec::new();
    let (t0, t1) = curve.domain();

    fn clip_recursive(
        curve: &NurbsCurve,
        plane: &Plane,
        ta: f32,
        tb: f32,
        depth: i32,
        tol: f32,
        results: &mut Vec<f32>,
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

        let dt = (tb - ta) / (num_samples - 1) as f32;
        for i in 0..num_samples {
            let t = ta + i as f32 * dt;
            let p = curve.point_at(t);
            distances.push(curve_signed_distance_to_plane(&p, plane));
            params.push(t);
        }

        let d_min = distances.iter().cloned().fold(f32::INFINITY, f32::min);
        let d_max = distances.iter().cloned().fold(f32::NEG_INFINITY, f32::max);

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
pub fn curve_plane_algebraic(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f32>) -> Vec<f32> {
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
            let is_duplicate = results.iter().any(|&existing_t: &f32| (t - existing_t).abs() < tol * 2.0);
            if !is_duplicate {
                results.push(t);
            }
        }
    }

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());
    results
}

/// Curve-plane intersection using production CAD kernel method
pub fn curve_plane_production(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f32>) -> Vec<f32> {
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
            ta: f32,
            tb: f32,
            depth: i32,
            tol: f32,
            results: &mut Vec<f32>,
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
pub fn curve_closest_point(curve: &NurbsCurve, test_point: &Point, t0: f32, t1: f32) -> (f32, f32) {
    Closest::curve_point(curve, test_point, t0, t1)
}

/// Find intersection curves between a NURBS surface and a plane
/// Find intersection points between a ray (Line) and a mesh using brute-force triangle testing.
pub fn ray_mesh(line: &Line, mesh: &crate::Mesh, epsilon: f32, find_all: bool) -> Option<Vec<Point>> {
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
    let mut hits: Vec<(f32, Point)> = Vec::new();

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

/// Find intersection points between a ray (Line) and a mesh using SpatialBVH acceleration.
pub fn ray_mesh_bvh(line: &Line, mesh: &crate::Mesh, epsilon: f32, find_all: bool) -> Option<Vec<Point>> {
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

    let tri_boxes: Vec<crate::OBB> = tris.iter()
        .map(|(v0, v1, v2)| crate::OBB::from_points(&[v0.clone(), v1.clone(), v2.clone()], 0.0))
        .collect();

    let world_size = crate::SpatialBVH::compute_world_size(&tri_boxes);
    let bvh = crate::SpatialBVH::from_boxes(&tri_boxes, world_size);

    let origin = line.start();
    let direction = line.to_vector().normalized();
    let mut candidate_ids: Vec<usize> = Vec::new();
    let found = bvh.ray_cast(&origin, &direction, &mut candidate_ids, true);
    if !found { return None; }

    let mut hits: Vec<(f32, Point)> = Vec::new();
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
pub fn surface_plane(surface: &NurbsSurface, plane: &Plane, tolerance: Option<f32>) -> Vec<NurbsCurve> {
    if !surface.is_valid() { return vec![]; }
    let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE).max(Tolerance::ZERO_TOLERANCE);

    let (u0, u1) = match surface.domain(0) { Some(d) => d, None => return vec![] };
    let (v0, v1) = match surface.domain(1) { Some(d) => d, None => return vec![] };
    let range_u = u1 - u0;
    let range_v = v1 - v0;
    let closed_u = surface.is_closed(0);
    let closed_v = surface.is_closed(1);

    let wrap_u = |u: f32| -> f32 {
        if closed_u {
            let mut t = (u - u0) % range_u;
            if t < 0.0 { t += range_u; }
            return u0 + t;
        }
        u.max(u0).min(u1)
    };
    let wrap_v = |v: f32| -> f32 {
        if closed_v {
            let mut t = (v - v0) % range_v;
            if t < 0.0 { t += range_v; }
            return v0 + t;
        }
        v.max(v0).min(v1)
    };

    let pn = plane.z_axis();
    let p0 = plane.origin();

    let g = |u: f32, v: f32| -> f32 {
        let p = surface.point_at(wrap_u(u), wrap_v(v)).unwrap_or(Point::new(0.0, 0.0, 0.0));
        (p[0] - p0[0]) * pn[0] + (p[1] - p0[1]) * pn[1] + (p[2] - p0[2]) * pn[2]
    };

    let g_and_grad = |u: f32, v: f32| -> (f32, f32, f32) {
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

    let newton_correct = |u: &mut f32, v: &mut f32| -> bool {
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
    let du = range_u / nu as f32;
    let dv = range_v / nv as f32;

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
    let mut dist = vec![0.0f32; ((nu + 1) * cols) as usize];
    for i in 0..=nu {
        let u = u0 + du * i as f32;
        for j in 0..=nv {
            let v = v0 + dv * j as f32;
            let mut d = g(u, v);
            if d == 0.0 { d = -1e-14; }
            dist[(i * cols + j) as usize] = d;
        }
    }

    struct Seed { u: f32, v: f32, used: bool }
    let mut seeds: Vec<Seed> = Vec::new();

    // Horizontal edges
    let h_jmax = if closed_v { nv - 1 } else { nv };
    for i in 0..nu {
        for j in 0..=h_jmax {
            let d0 = dist[(i * cols + j) as usize];
            let d1 = dist[((i + 1) * cols + j) as usize];
            if d0 * d1 < 0.0 {
                let t = d0 / (d0 - d1);
                let mut su = u0 + du * (i as f32 + t);
                let mut sv = v0 + dv * j as f32;
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
                let mut su = u0 + du * i as f32;
                let mut sv = v0 + dv * (j as f32 + t);
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
        let tangent_at_uv = |u: f32, v: f32, dir: f32| -> Option<(f32, f32)> {
            let (_, gu, gv) = g_and_grad(u, v);
            let mag = f32::hypot(gu, gv);
            if mag < 1e-14 { return None; }
            Some((-gv / mag * dir, gu / mag * dir))
        };

        // Trace one direction; returns (points, closed)
        let trace_dir = |su: f32, sv: f32, dir: f32, seeds: &mut Vec<Seed>| -> (Vec<(f32, f32)>, bool) {
            let mut out: Vec<(f32, f32)> = Vec::new();
            let mut u = su;
            let mut v = sv;
            let mut prev_tu = 0.0f32;
            let mut prev_tv = 0.0f32;
            let p_start = surface.point_at(su, sv).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let mut p_prev = p_start.clone();
            let mut dist_traveled = 0.0f32;

            for _ in 0..max_steps {
                let (mut tu, mut tv) = match tangent_at_uv(u, v, dir) {
                    Some(t) => t,
                    None => {
                        if f32::hypot(prev_tu, prev_tv) < 1e-14 { break; }
                        (prev_tu, prev_tv)
                    }
                };

                // Adaptive step
                let mut local_step = step;
                if f32::hypot(prev_tu, prev_tv) > 1e-14 {
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
                    let mut tc = 1.0f32;
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
        let mut uv_trace: Vec<(f32, f32)> = Vec::with_capacity(bwd.len() + 1 + fwd.len());
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
            let to2d = |p: &Point| -> (f32, f32) {
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
                let radius = f32::hypot(x1 - ccx, y1 - ccy);

                let mut max_dev = 0.0f32;
                for p in &all_pts {
                    let (px, py) = to2d(p);
                    max_dev = max_dev.max((f32::hypot(px - ccx, py - ccy) - radius).abs());
                }

                let circle_tol = (radius * 1e-4).max(1e-6);
                if radius > 1e-10 && max_dev < circle_tol {
                    let cx3d = po[0] + ccx * ax[0] + ccy * ay[0];
                    let cy3d = po[1] + ccx * ax[1] + ccy * ay[1];
                    let cz3d = po[2] + ccx * ax[2] + ccy * ay[2];

                    let w = std::f32::consts::FRAC_1_SQRT_2;
                    let cx_: [f32; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
                    let cy_: [f32; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
                    let wts: [f32; 9] = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
                    crv = NurbsCurve::new(3, true, 3, 9);
                    let nurbsknots: [f32; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
                    for i in 0..10 { crv.set_nurbsknot(i, nurbsknots[i]); }
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
            let to2d = |p: &Point| -> (f32, f32) {
                let dx = p[0] - po[0];
                let dy = p[1] - po[1];
                let dz = p[2] - po[2];
                (dx * ax[0] + dy * ax[1] + dz * ax[2], dx * ay[0] + dy * ay[1] + dz * ay[2])
            };

            let n = all_pts.len();
            // Build normal equations (5x5 symmetric system)
            let mut ata = [[0.0f32; 5]; 5];
            let mut atb = [0.0f32; 5];
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
            let mut m_mat = [[0.0f32; 6]; 5];
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
            let mut coef = [0.0f32; 5];
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
                let mut max_conic_dev = 0.0f32;
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
                    let theta = 0.5 * f32::atan2(cb, ca - cc);
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

                        let w = std::f32::consts::FRAC_1_SQRT_2;
                        let cx_: [f32; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
                        let cy_: [f32; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
                        let wts: [f32; 9] = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
                        crv = NurbsCurve::new(3, true, 3, 9);
                        let nurbsknots: [f32; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
                        for i in 0..10 { crv.set_nurbsknot(i, nurbsknots[i]); }
                        for i in 0..9 {
                            let px = cx3d + semi_a * cx_[i] * ea[0] + semi_b * cy_[i] * eb[0];
                            let py = cy3d + semi_a * cx_[i] * ea[1] + semi_b * cy_[i] * eb[1];
                            let pz = cz3d + semi_a * cx_[i] * ea[2] + semi_b * cy_[i] * eb[2];
                            crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                        }

                        // Verify ellipse fit
                        let mut max_ell_dev = 0.0f32;
                        for p in &all_pts {
                            let (px2, py2) = to2d(p);
                            let lx = cos_t * (px2 - cx) + sin_t * (py2 - cy);
                            let ly = -sin_t * (px2 - cx) + cos_t * (py2 - cy);
                            let ang = f32::atan2(ly / semi_b, lx / semi_a);
                            let ex = cx + semi_a * ang.cos() * cos_t - semi_b * ang.sin() * sin_t;
                            let ey = cy + semi_a * ang.cos() * sin_t + semi_b * ang.sin() * cos_t;
                            let dev = f32::hypot(px2 - ex, py2 - ey);
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
            let mut chords = vec![0.0f32; m];
            let mut total_len = 0.0f32;
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
            let mut total_turning = 0.0f32;
            for i in 1..(m - 1) {
                let dx1 = pts_2d[i][0] - pts_2d[i - 1][0];
                let dy1 = pts_2d[i][1] - pts_2d[i - 1][1];
                let dx2 = pts_2d[i + 1][0] - pts_2d[i][0];
                let dy2 = pts_2d[i + 1][1] - pts_2d[i][1];
                let l1 = f32::hypot(dx1, dy1);
                let l2 = f32::hypot(dx2, dy2);
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
                let mut max_dev = 0.0f32;
                for i in 0..m {
                    let t = ft0 + (ft1 - ft0) * chords[i];
                    max_dev = max_dev.max(crv_2d.point_at(t).distance(&pts_2d[i], None));
                }
                if max_dev < fit_tol { break; }
                target_cvs = (target_cvs * 2).min(max_cvs);
            }
            if !crv_2d.is_valid() {
                crv_2d = if is_loop {
                    NurbsCurve::create_interpolated(&pts_2d, CurveNurbsKnotStyle::ChordPeriodic)
                } else {
                    NurbsCurve::create_interpolated(&pts_2d, CurveNurbsKnotStyle::Chord)
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

// ── Joint geometry utilities (ported from cgal_intersection_util) ──────────

use crate::Polyline;

/// Check if two vectors are nearly parallel (|cos angle| >= cos(angle_tol)).
fn vectors_nearly_parallel(v0: &Vector, v1: &Vector, angle_tol: f32) -> bool {
    let m0 = v0.magnitude();
    let m1 = v1.magnitude();
    if m0 < Tolerance::ZERO_TOLERANCE || m1 < Tolerance::ZERO_TOLERANCE { return false; }
    let cos_angle = v0.dot(v1) / (m0 * m1);
    cos_angle.abs() >= angle_tol.cos()
}

/// 3-plane intersection with parallelism guard (0.1 rad default).
pub fn plane_plane_plane_check(
    p0: &Plane, p1: &Plane, p2: &Plane,
    angle_tol: f32,
) -> Option<Point> {
    if vectors_nearly_parallel(&p0.z_axis(), &p1.z_axis(), angle_tol) { return None; }
    if vectors_nearly_parallel(&p0.z_axis(), &p2.z_axis(), angle_tol) { return None; }
    if vectors_nearly_parallel(&p1.z_axis(), &p2.z_axis(), angle_tol) { return None; }
    plane_plane_plane(p0, p1, p2)
}

/// Intersect main plane with 4 ordered boundary planes → closed quad (5 pts).
pub fn plane_4planes(main: &Plane, planes: &[Plane; 4]) -> Option<Polyline> {
    let p0 = plane_plane_plane_check(&planes[0], &planes[1], main, 0.1)?;
    let p1 = plane_plane_plane_check(&planes[1], &planes[2], main, 0.1)?;
    let p2 = plane_plane_plane_check(&planes[2], &planes[3], main, 0.1)?;
    let p3 = plane_plane_plane_check(&planes[3], &planes[0], main, 0.1)?;
    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Same as plane_4planes but open (4 pts, last ≠ first).
pub fn plane_4planes_open(main: &Plane, planes: &[Plane; 4]) -> Option<Polyline> {
    let p0 = plane_plane_plane_check(&planes[0], &planes[1], main, 0.1)?;
    let p1 = plane_plane_plane_check(&planes[1], &planes[2], main, 0.1)?;
    let p2 = plane_plane_plane_check(&planes[2], &planes[3], main, 0.1)?;
    let p3 = plane_plane_plane_check(&planes[3], &planes[0], main, 0.1)?;
    Some(Polyline::new(vec![p0, p1, p2, p3]))
}

/// Intersect plane with 4 line segments → closed quad (5 pts).
pub fn plane_4lines(plane: &Plane, l0: &Line, l1: &Line, l2: &Line, l3: &Line) -> Option<Polyline> {
    let p0 = line_plane(l0, plane, false)?;
    let p1 = line_plane(l1, plane, false)?;
    let p2 = line_plane(l2, plane, false)?;
    let p3 = line_plane(l3, plane, false)?;
    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Build joint quad from collision face and two bounding planes.
pub fn get_quad_from_line_topbottomplanes(
    face_plane: &Plane, line: &Line, plane0: &Plane, plane1: &Plane,
) -> Option<Polyline> {
    let dir = line.to_vector();
    let s = line.start();
    let e = line.end();
    let lp0 = Plane::from_point_normal(s, dir.clone());
    let lp1 = Plane::from_point_normal(e, dir);
    let p0 = plane_plane_plane_check(&lp0, plane0, face_plane, 0.1)?;
    let p1 = plane_plane_plane_check(&lp0, plane1, face_plane, 0.1)?;
    let p2 = plane_plane_plane_check(&lp1, plane1, face_plane, 0.1)?;
    let p3 = plane_plane_plane_check(&lp1, plane0, face_plane, 0.1)?;
    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Scale direction vector to span the distance between two planes.
pub fn scale_vector_to_distance_of_2planes(
    dir: &Vector, p0: &Plane, p1: &Plane,
) -> Option<Vector> {
    if dir.magnitude() < Tolerance::ZERO_TOLERANCE { return None; }
    let origin = Point::new(0.0, 0.0, 0.0);
    let tip = Point::new(dir[0], dir[1], dir[2]);
    let ray = Line::new(origin[0], origin[1], origin[2], tip[0], tip[1], tip[2]);
    let q0 = line_plane(&ray, p0, false)?;
    let q1 = line_plane(&ray, p1, false)?;
    let output = Vector::new(q1[0] - q0[0], q1[1] - q0[1], q1[2] - q0[2]);
    // Validity: squared-distance ratio < 10 (mirrors CGAL)
    let n1 = p1.z_axis();
    let n1_mag = n1.magnitude();
    if n1_mag < Tolerance::ZERO_TOLERANCE { return None; }
    let o0 = p0.origin();
    let d = (o0[0] - p1.origin()[0]) * n1[0] / n1_mag
          + (o0[1] - p1.origin()[1]) * n1[1] / n1_mag
          + (o0[2] - p1.origin()[2]) * n1[2] / n1_mag;
    let dist_ortho_sq = d * d;
    if dist_ortho_sq < Tolerance::ZERO_TOLERANCE { return None; }
    let dist_sq = output.dot(&output);
    if dist_sq / dist_ortho_sq >= 10.0 { return None; }
    Some(output)
}

/// Orthogonal vector between two line-pairs (each defined by plane×plane).
pub fn get_orthogonal_vector_between_two_plane_pairs(
    pp0_0: &Plane, pp1_0: &Plane, pp1_1: &Plane,
) -> Option<Vector> {
    let l0 = plane_plane(pp0_0, pp1_0)?;
    let l1 = plane_plane(pp0_0, pp1_1)?;
    let (t0, t1) = line_line_parameters(&l0, &l1, 0.0, false, true)?;
    let pt0 = l0.point_at(t0);
    let pt1 = l1.point_at(t1);
    Some(Vector::new(pt1[0] - pt0[0], pt1[1] - pt0[1], pt1[2] - pt0[2]))
}

/// Clip line segment to region between two planes (in-place-style, returns new Line).
pub fn line_two_planes(line: &Line, p0: &Plane, p1: &Plane) -> Option<Line> {
    let q0 = line_plane(line, p0, true)?;
    let q1 = line_plane(line, p1, true)?;
    Some(Line::new(q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]))
}

/// Intersect all polyline perimeter edges with a plane.
/// Returns (points, edge_ids) if exactly 2 intersections are found.
pub fn polyline_plane(poly: &Polyline, plane: &Plane) -> Option<(Vec<Point>, Vec<usize>)> {
    let n = poly.point_count();
    if n < 2 { return None; }
    let mut points = Vec::new();
    let mut ids = Vec::new();
    for i in 0..n - 1 {
        if let (Some(a), Some(b)) = (poly.get_point(i), poly.get_point(i + 1)) {
            let va = plane_value_at(plane, &a);
            let vb = plane_value_at(plane, &b);
            if va.abs() < Tolerance::ZERO_TOLERANCE || vb.abs() < Tolerance::ZERO_TOLERANCE {
                continue;
            }
            let seg = Line::new(a[0], a[1], a[2], b[0], b[1], b[2]);
            if let Some(hit) = line_plane(&seg, plane, true) {
                points.push(hit);
                ids.push(i);
            }
        }
    }
    if points.len() == 2 { Some((points, ids)) } else { None }
}

/// Intersect polyline perimeter with plane → single segment, aligned to reference start.
pub fn polyline_plane_to_line(poly: &Polyline, plane: &Plane, align_start: &Point) -> Option<Line> {
    let (pts, _) = polyline_plane(poly, plane)?;
    let d0sq = (pts[0][0]-align_start[0]).powi(2)
             + (pts[0][1]-align_start[1]).powi(2)
             + (pts[0][2]-align_start[2]).powi(2);
    let d1sq = (pts[1][0]-align_start[0]).powi(2)
             + (pts[1][1]-align_start[1]).powi(2)
             + (pts[1][2]-align_start[2]).powi(2);
    if d0sq <= d1sq {
        Some(Line::new(pts[0][0], pts[0][1], pts[0][2], pts[1][0], pts[1][1], pts[1][2]))
    } else {
        Some(Line::new(pts[1][0], pts[1][1], pts[1][2], pts[0][0], pts[0][1], pts[0][2]))
    }
}

/// Build a closed quad polyline from a joint line plus two side planes.
///
/// End-cap planes are perpendicular to the joint line at each endpoint;
/// the four corners are 3-plane intersections.
pub fn quad_from_line_top_bottom_planes(
    face_plane: &Plane,
    line: &Line,
    plane0: &Plane,
    plane1: &Plane,
) -> Option<Polyline> {
    let direction = line.to_vector();
    let s = line.start();
    let lp0 = Plane::from_point_normal(s, direction.clone());
    let e = line.end();
    let lp1 = Plane::from_point_normal(e, direction);
    let p0 = plane_plane_plane(&lp0, plane0, face_plane)?;
    let p1 = plane_plane_plane(&lp0, plane1, face_plane)?;
    let p2 = plane_plane_plane(&lp1, plane1, face_plane)?;
    let p3 = plane_plane_plane(&lp1, plane0, face_plane)?;
    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Vector orthogonal to the (pp00, pp10) intersection line, anchored on (pp00, pp11).
///
/// Verbatim port of wood `cgal_intersection_util.cpp:619-628`:
///
/// ```text
///   plane_plane(pp00, pp10, l0);
///   plane_plane(pp00, pp11, l1);
///   output = l1.point() - l0.projection(l1.point());
/// ```
pub fn orthogonal_vector_between_two_plane_pairs(
    pp00: &Plane,
    pp10: &Plane,
    pp11: &Plane,
) -> Option<Vector> {
    let l0 = plane_plane(pp00, pp10)?;
    let l1 = plane_plane(pp00, pp11)?;
    let p1 = l1.start();
    let ldir = l0.to_vector();
    let len_sq = ldir[0]*ldir[0] + ldir[1]*ldir[1] + ldir[2]*ldir[2];
    if len_sq < 1e-20 {
        return None;
    }
    let l0s = l0.start();
    let vx = p1[0] - l0s[0];
    let vy = p1[1] - l0s[1];
    let vz = p1[2] - l0s[2];
    let t = (vx*ldir[0] + vy*ldir[1] + vz*ldir[2]) / len_sq;
    let px = l0s[0] + ldir[0]*t;
    let py = l0s[1] + ldir[1]*t;
    let pz = l0s[2] + ldir[2]*t;
    Some(Vector::new(p1[0]-px, p1[1]-py, p1[2]-pz))
}

/// Clip an open joint outline against a closed plate polygon in 2D.
///
/// Port of the wood `wood_element.cpp:438-651` helper. Returns the clipped
/// 3D polyline plus parametric positions `(t0, t1)` on the plate edges, or
/// `None` if the joint outline does not intersect the plate polygon.
pub fn closed_and_open_paths_2d(
    plate: &Polyline,
    joint: &Polyline,
    plane: &Plane,
) -> Option<(Polyline, (f32, f32))> {
    let origin = plate.get_point(0)?;
    let mut xax = plane.x_axis();
    let mut yax = plane.y_axis();
    xax.normalize_self();
    yax.normalize_self();

    let to_2d = |pp: &Point| -> (f32, f32) {
        let dx = pp[0]-origin[0];
        let dy = pp[1]-origin[1];
        let dz = pp[2]-origin[2];
        (dx*xax[0]+dy*xax[1]+dz*xax[2],
         dx*yax[0]+dy*yax[1]+dz*yax[2])
    };
    let to_3d = |u: f32, v: f32| -> Point {
        Point::new(origin[0] + u*xax[0] + v*yax[0],
                   origin[1] + u*xax[1] + v*yax[1],
                   origin[2] + u*xax[2] + v*yax[2])
    };

    // Plate outline (2D), strip closing duplicate.
    let mut plate_n = plate.point_count();
    if plate_n > 1 {
        let f = plate.get_point(0).unwrap();
        let l = plate.get_point(plate_n-1).unwrap();
        if (f[0]-l[0]).abs() < 1e-6 && (f[1]-l[1]).abs() < 1e-6 && (f[2]-l[2]).abs() < 1e-6 {
            plate_n -= 1;
        }
    }
    let plate2d: Vec<(f32, f32)> = (0..plate_n)
        .map(|i| to_2d(&plate.get_point(i).unwrap()))
        .collect();
    if plate2d.len() < 3 {
        return None;
    }

    let joint2d: Vec<(f32, f32)> = (0..joint.point_count())
        .map(|i| to_2d(&joint.get_point(i).unwrap()))
        .collect();
    if joint2d.len() < 2 {
        return None;
    }

    let pip = |px: f32, py: f32| -> bool {
        let mut wn = 0i32;
        let n = plate2d.len();
        for i in 0..n {
            let a = plate2d[i];
            let b = plate2d[(i+1) % n];
            if a.1 <= py {
                if b.1 > py {
                    let e = (b.0-a.0)*(py-a.1) - (px-a.0)*(b.1-a.1);
                    if e > 0.0 { wn += 1; }
                }
            } else if b.1 <= py {
                let e = (b.0-a.0)*(py-a.1) - (px-a.0)*(b.1-a.1);
                if e < 0.0 { wn -= 1; }
            }
        }
        wn != 0
    };

    let seg_seg_2d = |s0: (f32, f32), s1: (f32, f32), e0: (f32, f32), e1: (f32, f32)| -> Option<(f32, f32)> {
        let sx = s1.0-s0.0; let sy = s1.1-s0.1;
        let ex = e1.0-e0.0; let ey = e1.1-e0.1;
        let denom = sx*ey - sy*ex;
        if denom.abs() < 1e-20 {
            return None;
        }
        let dx = e0.0-s0.0; let dy = e0.1-s0.1;
        let t_s = (dx*ey - dy*ex) / denom;
        let t_e = (dx*sy - dy*sx) / denom;
        Some((t_s, t_e))
    };

    const EPS: f32 = 1e-9;
    let mut pieces: Vec<Vec<(f32, f32)>> = Vec::new();
    for s in 0..joint2d.len()-1 {
        let p0 = joint2d[s];
        let p1 = joint2d[s+1];
        let mut ts: Vec<f32> = vec![0.0];
        for i in 0..plate2d.len() {
            let a = plate2d[i];
            let b = plate2d[(i+1) % plate2d.len()];
            if let Some((t_s, t_e)) = seg_seg_2d(p0, p1, a, b) {
                if t_s > EPS && t_s < 1.0 - EPS && t_e >= -EPS && t_e <= 1.0 + EPS {
                    ts.push(t_s);
                }
            }
        }
        ts.push(1.0);
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ts.dedup_by(|a, b| (*a - *b).abs() < EPS);

        let mut current: Vec<(f32, f32)> = Vec::new();
        for i in 0..ts.len()-1 {
            let t_mid = 0.5 * (ts[i] + ts[i+1]);
            let mx = p0.0 + (p1.0-p0.0)*t_mid;
            let my = p0.1 + (p1.1-p0.1)*t_mid;
            if pip(mx, my) {
                let sub_a = (p0.0 + (p1.0-p0.0)*ts[i],   p0.1 + (p1.1-p0.1)*ts[i]);
                let sub_b = (p0.0 + (p1.0-p0.0)*ts[i+1], p0.1 + (p1.1-p0.1)*ts[i+1]);
                if current.is_empty() {
                    current.push(sub_a);
                    current.push(sub_b);
                } else {
                    let last = *current.last().unwrap();
                    let dx = last.0 - sub_a.0;
                    let dy = last.1 - sub_a.1;
                    if dx*dx + dy*dy < 1e-18 {
                        current.push(sub_b);
                    } else {
                        pieces.push(std::mem::take(&mut current));
                        current.push(sub_a);
                        current.push(sub_b);
                    }
                }
            } else if !current.is_empty() {
                pieces.push(std::mem::take(&mut current));
            }
        }
        if !current.is_empty() {
            pieces.push(current);
        }
    }

    let sq2 = |a: (f32, f32), b: (f32, f32)| -> f32 {
        let dx = a.0-b.0; let dy = a.1-b.1;
        dx*dx + dy*dy
    };
    const DISTANCE_SQ: f32 = 0.01;
    let mut c2d: Vec<(f32, f32)> = Vec::new();
    let mut count = 0i32;
    for piece in &pieces {
        if piece.len() <= 1 { continue; }
        if count == 0 {
            c2d = piece.clone();
        } else {
            let mut pts = piece.clone();
            if sq2(*c2d.last().unwrap(), pts[0]) > DISTANCE_SQ
                && sq2(*c2d.last().unwrap(), *pts.last().unwrap()) > DISTANCE_SQ {
                c2d.reverse();
            }
            if sq2(*c2d.last().unwrap(), pts[0]) > sq2(*c2d.last().unwrap(), *pts.last().unwrap()) {
                pts.reverse();
            }
            for j in 1..pts.len() {
                c2d.push(pts[j]);
            }
        }
        count += 1;
    }

    if c2d.len() < 2 {
        return None;
    }

    let closest_param = |p: (f32, f32), a: (f32, f32), b: (f32, f32)| -> f32 {
        let abx = b.0-a.0; let aby = b.1-a.1;
        let l2 = abx*abx + aby*aby;
        if l2 < 1e-20 { return 0.0; }
        let apx = p.0-a.0; let apy = p.1-a.1;
        let mut t = (apx*abx + apy*aby) / l2;
        if t < 0.0 { t = 0.0; }
        if t > 1.0 { t = 1.0; }
        t
    };
    let sq_dist_seg = |p: (f32, f32), a: (f32, f32), b: (f32, f32)| -> f32 {
        let abx = b.0-a.0; let aby = b.1-a.1;
        let l2 = abx*abx + aby*aby;
        if l2 < 1e-20 {
            let dx = p.0-a.0; let dy = p.1-a.1;
            return dx*dx + dy*dy;
        }
        let apx = p.0-a.0; let apy = p.1-a.1;
        let mut t = (apx*abx + apy*aby) / l2;
        if t < 0.0 { t = 0.0; }
        if t > 1.0 { t = 1.0; }
        let px = a.0 + t*abx;
        let py = a.1 + t*aby;
        let dx = p.0-px;
        let dy = p.1-py;
        dx*dx + dy*dy
    };

    let mut t0 = -1.0_f32;
    let mut t1 = -1.0_f32;
    for i in 0..plate2d.len() {
        let a = plate2d[i];
        let b = plate2d[(i+1) % plate2d.len()];
        for jj in 0..2 {
            let idx = if jj == 0 { 0 } else { c2d.len() - 1 };
            let d = sq_dist_seg(c2d[idx], a, b);
            if jj == 0 && d < 1.0 {
                t0 = i as f32 + closest_param(c2d[0], a, b);
            } else if jj == 1 && d < 1.0 {
                t1 = i as f32 + closest_param(*c2d.last().unwrap(), a, b);
            }
        }
        if t0 >= 0.0 && t1 >= 0.0 { break; }
    }

    let mut reverse_flag = t0 > t1;
    if (t0.floor() as usize) == 0 && (t1.floor() as usize) == c2d.len() - 1 {
        reverse_flag = !reverse_flag;
    }
    if reverse_flag {
        std::mem::swap(&mut t0, &mut t1);
        c2d.reverse();
    }

    if t0 < 0.0 || t1 < 0.0 {
        return None;
    }

    let out_pts: Vec<Point> = c2d.iter().map(|p| to_3d(p.0, p.1)).collect();
    Some((Polyline::new(out_pts), (t0, t1)))
}

/// 3D skew line intersection via closest-approach on cutter.
pub fn line_line_3d(cutter: &Line, seg: &Line) -> Option<Point> {
    let (t0, _) = line_line_parameters(cutter, seg, 0.0, false, false)?;
    Some(cutter.point_at(t0))
}

/// Project point onto finite segment; returns (closest_point, t ∈ [0,1]).
pub fn closest_point_on_segment(pt: &Point, seg: &Line) -> (Point, f32) {
    let mut t = Polyline::closest_point_to_line(pt, &seg.start(), &seg.end());
    t = t.clamp(0.0, 1.0);
    (seg.point_at(t), t)
}

/// Linear remap: map val from [from1,to1] to [from2,to2].
pub fn remap(val: f32, from1: f32, to1: f32, from2: f32, to2: f32) -> f32 {
    let span = to1 - from1;
    if span.abs() < Tolerance::ZERO_TOLERANCE { return from2; }
    let t = (val - from1) / span;
    from2 + t * (to2 - from2)
}

pub fn face_to_face(
    adjacency: &[i32],
    polylines: &[Vec<crate::polyline::Polyline>],
    planes: &[Vec<crate::plane::Plane>],
    coplanar_tolerance: f32,
) -> Vec<(i32, i32, i32, i32, i32, crate::polyline::Polyline)> {
    use crate::plane::Plane;
    use crate::polyline::Polyline;
    use crate::vector::Vector;

    let mut results = Vec::new();
    let mut idx = 0;
    while idx + 1 < adjacency.len() {
        let a = adjacency[idx] as usize;
        let b = adjacency[idx + 1] as usize;
        idx += 4;

        let mut found = false;
        for i in 0..planes[a].len() {
            if found { break; }
            for j in 0..planes[b].len() {
                let oa = planes[a][i].origin();
                let za = planes[a][i].z_axis();
                let ob = planes[b][j].origin();
                let zb = planes[b][j].z_axis();
                if !Plane::is_coplanar_from_normals(&oa, &za, &ob, &zb, false, coplanar_tolerance) {
                    continue;
                }

                let pts_i = polylines[a][i].get_points();
                if pts_i.len() < 2 { continue; }
                let mut edge = Vector::new(
                    pts_i[1][0] - pts_i[0][0],
                    pts_i[1][1] - pts_i[0][1],
                    pts_i[1][2] - pts_i[0][2],
                );
                edge.normalize_self();
                let zax = planes[a][i].z_axis();
                let mut yax = zax.cross(&edge);
                yax.normalize_self();
                let pln = Plane::from_axes(pts_i[0].clone(), edge, yax, zax);

                let bools = Polyline::boolean_op_plane(&polylines[a][i], &polylines[b][j], &pln, 0);
                if bools.is_empty() || bools[0].point_count() < 3 { continue; }

                let typ = (if i > 1 { 0 } else { 1 }) + (if j > 1 { 0 } else { 1 });
                let jpl = if bools[0].is_closed() { bools[0].clone() } else { bools[0].closed() };
                results.push((a as i32, b as i32, i as i32, j as i32, typ as i32, jpl));
                found = true;
                break;
            }
        }
    }

    results
}

// ─────────────────────────────────────────────────────────────────────────────
// WOOD: face_to_face_wood — detailed timber-joint topology detection
// ─────────────────────────────────────────────────────────────────────────────
//
// Direct port of the C++ wood-library function `wood::main::face_to_face` from
// `cmake/src/wood/include/wood_main.cpp`. This is the heavy-weight cousin of
// the lightweight `face_to_face` above: it not only detects that two element
// faces are coplanar and overlap, but also constructs the full alignment
// lines and the volumetric joint regions used by the joint-library
// (mortise/tenon, half-lap, butt, cross-lap, etc.).
//
// The algorithm is intricate. Key conventions, kept identical to the C++:
//
//   * Input polylines/planes are indexed [0]=top face, [1]=bottom face,
//     [2..]=side faces (one per side). All faces are convex polygons.
//   * The "type" of a joint is determined by the face-class of each side:
//       type0 = (i > 1 ? 0 : 1) — 0 if side, 1 if top
//       type1 = (j > 1 ? 0 : 1)
//       type  = type0 + type1
//     so 0 = side-side, 1 = top-side, 2 = top-top.
//   * Inside each branch the type is further refined into the codes the
//     joint library uses:  11 = side-side parallel out-of-plane,
//     12 = side-side parallel in-plane,  13 = side-side rotated/perpendicular,
//     20 = top-side,  40 = top-top.
//   * `joint_volumes_pair_a_pair_b` holds up to 4 closed-quad polylines.
//     Pair A (indices 0,1) is the male side, pair B (indices 2,3) the female.
//     Type 13/20/40 only fill pair A (indices 0,1); type 12 fills all four.
//
// The function fails (returns `None`) if no face pair is coplanar+overlapping,
// or if any geometric helper degenerates (parallel-plane intersections,
// collapsed alignment lines, dihedral angles below the validity threshold).

/// Tunable parameters for `face_to_face_wood`. The original C++ code reads
/// these from a global state (`wood::GLOBALS::*`); the Rust port passes them
/// explicitly so the function is pure and re-entrant.
#[derive(Clone, Debug)]
pub struct WoodConfig {
    /// Per-joint extension parameters, packed in triples
    /// `(width_extension, height_extension, line_extension)`. The function
    /// picks one triple based on `joint_id`, clamping to the last available
    /// triple if `joint_id` exceeds what's defined.
    pub joint_volume_extension: Vec<f32>,
    /// Minimum joint length (linear, not squared); the function rejects
    /// joints whose alignment line is shorter than this minus the line
    /// extension parameter.
    pub limit_min_joint_length: f32,
    /// Squared distance below which an alignment line is treated as
    /// degenerate. Mirrors `wood::GLOBALS::DISTANCE_SQUARED`.
    pub distance_squared: f32,
    /// Dihedral angle (degrees) cutoff between out-of-plane (≤ this value)
    /// and in-plane (> this value) parallel side-to-side joints. Wood
    /// default is 150°.
    pub face_to_face_side_to_side_joints_dihedral_angle: f32,
    /// If true, every side-to-side joint is forced through the rotated
    /// branch even when the alignment lines are parallel.
    pub face_to_face_side_to_side_joints_all_treated_as_rotated: bool,
    /// If true, the rotated branch uses the average of both alignment lines
    /// as its joint axis; if false it uses `joint_line0` directly.
    pub face_to_face_side_to_side_joints_rotated_joint_as_average: bool,
}

impl Default for WoodConfig {
    fn default() -> Self {
        Self {
            joint_volume_extension: vec![0.0, 0.0, 0.0],
            limit_min_joint_length: 0.0,
            distance_squared: 1e-6,
            face_to_face_side_to_side_joints_dihedral_angle: 150.0,
            face_to_face_side_to_side_joints_all_treated_as_rotated: false,
            face_to_face_side_to_side_joints_rotated_joint_as_average: true,
        }
    }
}

/// Output of a successful `face_to_face_wood` call. Mirrors the seven
/// out-parameters of the original C++ function (`el_ids`, `face_ids`, `type`,
/// `joint_area`, `joint_lines`, and `joint_volumes_pairA_pairB`).
#[derive(Clone, Debug)]
pub struct WoodJoint {
    /// Element-id pair. May be SWAPPED relative to the caller's input when a
    /// male/female flip is required (out-of-plane side-to-side, top-side).
    pub el_ids: (i32, i32),
    /// `(face_indices_for_el0, face_indices_for_el1)`. Both slots in each
    /// array currently hold the same matched face index — the C++ kept the
    /// shape pair-of-arrays for forward compatibility with multi-face joints.
    pub face_ids: ([i32; 2], [i32; 2]),
    /// Refined joint type code consumed by the joint library. See the module
    /// header above for the meaning of 11/12/13/20/40.
    pub joint_type: i32,
    /// 2D Boolean intersection polygon (the overlap area between the two
    /// coplanar faces) carried as a polyline in the original 3D coordinates.
    pub joint_area: crate::Polyline,
    /// Joint alignment lines. For top-side and rotated/parallel side-side
    /// both entries hold the same line (`joint_lines[1]` is a duplicate);
    /// for out-of-plane parallel side-side they hold the male line and the
    /// female line, possibly reversed if a male/female flip occurred.
    pub joint_lines: [Line; 2],
    /// Up to 4 closed-quad polylines describing the volumetric joint
    /// regions. Indices follow the C++ `joint_volumes_pairA_pairB`:
    ///   * type 13 (rotated side-side): slots 0 and 1 are filled.
    ///   * type 11 (out-of-plane parallel): slots 0 and 1 are filled.
    ///   * type 12 (in-plane parallel): all four slots are filled.
    ///   * type 20 (top-side): slots 0 and 1 are filled.
    ///   * type 40 (top-top): slots 0 and 1 are filled.
    pub joint_volumes_pair_a_pair_b: [Option<crate::Polyline>; 4],
}

// ── wood-private helpers ────────────────────────────────────────────────────

/// Approximate dihedral angle (degrees, unsigned [0, 180]) of edge `pq` in the
/// tetrahedron `pqrs`. Mirrors `CGAL::approximate_dihedral_angle(p, q, r, s)`
/// followed by `std::abs(...)` as the wood caller uses it.
///
/// This is the angle between half-plane (pqr) and half-plane (pqs), measured
/// in the plane perpendicular to edge pq.
fn approximate_dihedral_angle(p: &Point, q: &Point, r: &Point, s: &Point) -> f32 {
    use crate::Vector;
    let pq = Vector::new(q[0] - p[0], q[1] - p[1], q[2] - p[2]);
    let pr = Vector::new(r[0] - p[0], r[1] - p[1], r[2] - p[2]);
    let ps = Vector::new(s[0] - p[0], s[1] - p[1], s[2] - p[2]);
    let n1 = pq.cross(&pr);
    let n2 = pq.cross(&ps);
    let m1 = n1.magnitude();
    let m2 = n2.magnitude();
    if m1 < crate::tolerance::Tolerance::ZERO_TOLERANCE
        || m2 < crate::tolerance::Tolerance::ZERO_TOLERANCE
    {
        return 0.0;
    }
    let cos_theta = (n1.dot(&n2) / (m1 * m2)).clamp(-1.0, 1.0);
    cos_theta.acos().to_degrees()
}

/// Average overlap segment of two near-parallel 3D line segments.
///
/// Algorithm:
///   1. Project all four endpoints onto `l0`'s axis (parameter `t`).
///   2. The overlap region is `[max(min, min) .. min(max, max)]` of the t's.
///   3. For the two t-values bounding the overlap, sample line0 at that t
///      and find the closest point on line1; their midpoint is one endpoint
///      of the average segment.
///
/// Returns `None` if `l0` is degenerate or if there is no overlap.
fn line_line_overlap_average(l0: &Line, l1: &Line) -> Option<Line> {
    use crate::Vector;
    let s0 = l0.start();
    let e0 = l0.end();
    let s1 = l1.start();
    let e1 = l1.end();

    let d0 = Vector::new(e0[0] - s0[0], e0[1] - s0[1], e0[2] - s0[2]);
    let len0_sq = d0.dot(&d0);
    if len0_sq < crate::tolerance::Tolerance::ZERO_TOLERANCE {
        return None;
    }

    // Project a point onto line0's axis as a parameter t (0 = s0, 1 = e0).
    let proj = |p: &Point| -> f32 {
        let dx = p[0] - s0[0];
        let dy = p[1] - s0[1];
        let dz = p[2] - s0[2];
        (dx * d0[0] + dy * d0[1] + dz * d0[2]) / len0_sq
    };
    let t_a = 0.0_f32;
    let t_b = 1.0_f32;
    let t_c = proj(&s1);
    let t_d = proj(&e1);

    // Overlap interval on line0's parameterization.
    let (lo0, hi0) = (t_a.min(t_b), t_a.max(t_b));
    let (lo1, hi1) = (t_c.min(t_d), t_c.max(t_d));
    let lo = lo0.max(lo1);
    let hi = hi0.min(hi1);
    if hi <= lo {
        return None;
    }

    // Sample line0 at the overlap endpoints.
    let pt0_lo = Point::new(s0[0] + lo * d0[0], s0[1] + lo * d0[1], s0[2] + lo * d0[2]);
    let pt0_hi = Point::new(s0[0] + hi * d0[0], s0[1] + hi * d0[1], s0[2] + hi * d0[2]);

    // Find the corresponding closest points on line1, clamped to [s1, e1].
    let closest_on_l1 = |pt: &Point| -> Point {
        let d1 = Vector::new(e1[0] - s1[0], e1[1] - s1[1], e1[2] - s1[2]);
        let len1_sq = d1.dot(&d1);
        if len1_sq < crate::tolerance::Tolerance::ZERO_TOLERANCE {
            return s1.clone();
        }
        let dx = pt[0] - s1[0];
        let dy = pt[1] - s1[1];
        let dz = pt[2] - s1[2];
        let t = ((dx * d1[0] + dy * d1[1] + dz * d1[2]) / len1_sq).clamp(0.0, 1.0);
        Point::new(s1[0] + t * d1[0], s1[1] + t * d1[1], s1[2] + t * d1[2])
    };
    let pt1_lo = closest_on_l1(&pt0_lo);
    let pt1_hi = closest_on_l1(&pt0_hi);

    // Average the two pairs to get the final overlap segment.
    let avg_lo = Point::new(
        (pt0_lo[0] + pt1_lo[0]) * 0.5,
        (pt0_lo[1] + pt1_lo[1]) * 0.5,
        (pt0_lo[2] + pt1_lo[2]) * 0.5,
    );
    let avg_hi = Point::new(
        (pt0_hi[0] + pt1_hi[0]) * 0.5,
        (pt0_hi[1] + pt1_hi[1]) * 0.5,
        (pt0_hi[2] + pt1_hi[2]) * 0.5,
    );
    Some(Line::from_points(&avg_lo, &avg_hi))
}

/// Slide the two endpoints of polyline edge `edge_idx` outward (or inward,
/// for negative `distance`) along the edge's tangent direction. The
/// neighbouring edges deform accordingly. For closed polylines (where
/// `pts[n-1] == pts[0]`) the closing duplicate is kept in sync.
///
/// Used by `face_to_face_wood` to scale joint volume rectangles by the
/// `JOINT_VOLUME_EXTENSION` config: extending opposite edges by the same
/// amount preserves the rectangle and grows it uniformly along that axis.
fn extend_polyline_edge_equally(
    poly: &mut crate::Polyline,
    edge_idx: usize,
    distance: f32,
) {
    let n = poly.point_count();
    if n < 2 || edge_idx + 1 >= n {
        return;
    }
    let i = edge_idx;
    let j = edge_idx + 1;
    let pi = match poly.get_point(i) { Some(p) => p, None => return };
    let pj = match poly.get_point(j) { Some(p) => p, None => return };
    let dx = pj[0] - pi[0];
    let dy = pj[1] - pi[1];
    let dz = pj[2] - pi[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-12 {
        return;
    }
    let inv_len = 1.0 / len;
    let ux = dx * inv_len * distance;
    let uy = dy * inv_len * distance;
    let uz = dz * inv_len * distance;
    let new_pi = Point::new(pi[0] - ux, pi[1] - uy, pi[2] - uz);
    let new_pj = Point::new(pj[0] + ux, pj[1] + uy, pj[2] + uz);
    poly.set_point(i, &new_pi);
    poly.set_point(j, &new_pj);
    // Closed polylines: index 0 and n-1 are the same point. If we just moved
    // either end of the joining seam, keep both copies in sync.
    if i == 0 {
        poly.set_point(n - 1, &new_pi);
    }
    if j == n - 1 {
        poly.set_point(0, &new_pj);
    }
}

/// Apply an `Xform` to a single `Point` without going through the `Point.xform`
/// field setup dance. Used by the rotated-joint branch to project the joint
/// area into the local 2D frame for AABB extraction.
fn xform_apply_point(xform: &crate::Xform, p: &Point) -> Point {
    let m = &xform.m;
    let (x, y, z) = (p[0], p[1], p[2]);
    let w = m[3] * x + m[7] * y + m[11] * z + m[15];
    let w_inv = if w.abs() > 1e-10 { 1.0 / w } else { 1.0 };
    Point::new(
        (m[0] * x + m[4] * y + m[8] * z + m[12]) * w_inv,
        (m[1] * x + m[5] * y + m[9] * z + m[13]) * w_inv,
        (m[2] * x + m[6] * y + m[10] * z + m[14]) * w_inv,
    )
}

/// Detailed face-to-face joint detection between two timber elements.
///
/// Direct Rust port of `wood::main::face_to_face` from
/// `cmake/src/wood/include/wood_main.cpp`. See the module header above for
/// the conventions on input polyline/plane ordering and joint type codes.
///
/// The function tries every face pair `(i, j)` between `polylines_0` and
/// `polylines_1`. For the first pair that is coplanar AND has a non-empty
/// 2D Boolean intersection (the "joint area"), it builds the alignment lines
/// and volumetric joint regions, then returns. Subsequent pairs are not
/// tried because the joint library only emits one joint per element pair.
///
/// Returns `None` if no face pair matches, or if any geometric helper
/// degenerates partway through (parallel-plane intersection failure,
/// collapsed alignment line, dihedral angle below validity threshold).
pub fn face_to_face_wood(
    joint_id: usize,
    polylines_0: &[crate::Polyline],
    polylines_1: &[crate::Polyline],
    planes_0: &[crate::Plane],
    planes_1: &[crate::Plane],
    insertion_vectors_0: &[crate::Vector],
    insertion_vectors_1: &[crate::Vector],
    el_ids_in: (i32, i32),
    config: &WoodConfig,
) -> Option<WoodJoint> {
    use crate::{Plane, Polyline, Vector, Xform};

    // Pick which extension triple to use from the config.
    // The C++ original: `extension_id = min(joint_id, count-1) * 3` where
    // `count = floor(JOINT_VOLUME_EXTENSION.size() / 3.0) - 1`.
    let extension_variables_count = if config.joint_volume_extension.len() / 3 == 0 {
        0
    } else {
        (config.joint_volume_extension.len() / 3) - 1
    };
    let extension_id = if extension_variables_count == 0 {
        0
    } else {
        joint_id.min(extension_variables_count) * 3
    };
    // Convenience accessors with bounds protection (same defaults as C++ when
    // the array is too short — falls back to 0 in production builds).
    let ext = |k: usize| -> f32 {
        config.joint_volume_extension.get(k + extension_id).copied().unwrap_or(0.0)
    };
    let ext_w = ext(0); // edges 0,2 → joint width  scaling
    let ext_h = ext(1); // edges 1,3 → joint height scaling
    let ext_l = ext(2); // joint-line axial extension

    // Mutable copies that may be reordered by the male/female flip branch.
    let mut el_ids = el_ids_in;
    let mut face_ids: ([i32; 2], [i32; 2]) = ([0; 2], [0; 2]);

    // Outer loop over every face pair (face_a, face_b).
    for i in 0..planes_0.len() {
        for j in 0..planes_1.len() {
            // ── 1. Coplanarity test (cheap; ~10 ms across the workload). ──
            // The C++ uses `cgal::plane_util::is_coplanar(P0, P1, false)`
            // which is the antiparallel-only test (faces touching back-to-back).
            let coplanar = Plane::is_coplanar_from_normals(
                &planes_0[i].origin(),
                &planes_0[i].z_axis(),
                &planes_1[j].origin(),
                &planes_1[j].z_axis(),
                false,
                crate::tolerance::Tolerance::APPROXIMATION,
            );
            if !coplanar {
                continue;
            }

            // ── 2. 2D Boolean intersection between the two coplanar faces. ──
            // Returns the overlap polygon as a single Polyline, or empty if
            // the polygons don't actually touch in their shared plane.
            let isect_results =
                Polyline::boolean_op_plane(&polylines_0[i], &polylines_1[j], &planes_0[i], 0);
            if isect_results.is_empty() {
                continue;
            }
            let joint_area_open = isect_results.into_iter().next().unwrap();
            if joint_area_open.point_count() < 3 {
                continue;
            }
            // Promote to a closed polyline (caller convention: closing dup).
            let joint_area = if joint_area_open.is_closed() {
                joint_area_open
            } else {
                joint_area_open.closed()
            };

            // ── 3. Record matched face indices for the output. ──
            // The C++ keeps both slots equal because it only stores ONE
            // matched face per element today. We mirror that.
            face_ids.0[0] = i as i32;
            face_ids.0[1] = i as i32;
            face_ids.1[0] = j as i32;
            face_ids.1[1] = j as i32;

            // ── 4. Joint type from the geometric class of each side. ──
            //   type0 = 0 if face is a side, 1 if face is top/bottom
            //   type  = 0 (side-side), 1 (top-side), or 2 (top-top)
            let type0: i32 = if i > 1 { 0 } else { 1 };
            let type1: i32 = if j > 1 { 0 } else { 1 };
            let mut joint_type: i32 = type0 + type1;

            // ── 5. Build the side-A alignment line (`joint_line0`) when ──
            //      face A is a side face (i > 1). For top faces this stays
            //      a degenerate sentinel — its length is later required to
            //      pass the LIMIT_MIN_JOINT_LENGTH check, so top-top joints
            //      naturally bypass it via the `type == 2` branch below.
            //
            // The alignment segment goes from the midpoint of edge `i-2` of
            // the top/bottom polylines to the midpoint of edge `i-1`. This
            // is the "natural axis" of the side face (in the wood library's
            // top/bottom + side convention, side `k` connects vertices `k`
            // of the top and bottom rings).
            let mut joint_line0 = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(0.0, 0.0, 0.0));
            // Average plane of the two reference (top + bottom) faces of
            // element 0 — i.e. the mid-thickness plane of element 0.
            let avg_plane_0 = Plane::from_point_normal(
                Point::mid_point(&polylines_0[0].get_point(0)?, &polylines_0[1].get_point(0)?),
                planes_0[0].z_axis(),
            );
            let mut joint_quads0: Option<Polyline> = None;

            if i > 1 {
                // Alignment segment from midpoint(top[i-2], bottom[i-2])
                // to midpoint(top[i-1], bottom[i-1]).
                let a0 = polylines_0[0].get_point(i - 2)?;
                let a1 = polylines_0[1].get_point(i - 2)?;
                let b0 = polylines_0[0].get_point(i - 1)?;
                let b1 = polylines_0[1].get_point(i - 1)?;
                let alignment_segment =
                    Line::from_points(&Point::mid_point(&a0, &a1), &Point::mid_point(&b0, &b1));

                // Intersect joint area with the average plane → 1D segment.
                // The Rust helper aligns the result so its start is closest
                // to the alignment_segment's start point.
                let line_opt = polyline_plane_to_line(
                    &joint_area,
                    &avg_plane_0,
                    &alignment_segment.start(),
                );
                let line = line_opt?;
                if line.squared_length() <= config.distance_squared {
                    return None;
                }
                joint_line0 = line;
                // Build the side-face joint quad from joint_line0 +
                // top + bottom planes of element 0.
                joint_quads0 = get_quad_from_line_topbottomplanes(
                    &planes_0[i],
                    &joint_line0,
                    &planes_0[0],
                    &planes_0[1],
                );
                if joint_quads0.is_none() {
                    return None;
                }
            }

            // ── 6. Same for side-B alignment line (`joint_line1`). ──
            let mut joint_line1 = Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(0.0, 0.0, 0.0));
            let avg_plane_1 = Plane::from_point_normal(
                Point::mid_point(&polylines_1[0].get_point(0)?, &polylines_1[1].get_point(0)?),
                planes_1[0].z_axis(),
            );
            let mut joint_quads1: Option<Polyline> = None;

            if j > 1 {
                let a0 = polylines_1[0].get_point(j - 2)?;
                let a1 = polylines_1[1].get_point(j - 2)?;
                let b0 = polylines_1[0].get_point(j - 1)?;
                let b1 = polylines_1[1].get_point(j - 1)?;
                let alignment_segment =
                    Line::from_points(&Point::mid_point(&a0, &a1), &Point::mid_point(&b0, &b1));
                let line_opt = polyline_plane_to_line(
                    &joint_area,
                    &avg_plane_1,
                    &alignment_segment.start(),
                );
                let line = line_opt?;
                if line.squared_length() <= config.distance_squared {
                    return None;
                }
                joint_line1 = line;
                joint_quads1 = get_quad_from_line_topbottomplanes(
                    &planes_1[j],
                    &joint_line1,
                    &planes_1[0],
                    &planes_1[1],
                );
                if joint_quads1.is_none() {
                    return None;
                }
            }

            // ── 7. Validate joint line length and apply axial extension. ──
            // The wood C++ derives:
            //     joint_line_extension_limit = (ext_l * 2)^2
            //     limit_min_squared          = limit_min_joint_length^2
            // and rejects the joint when
            //     joint_line_extension_limit > line.squared_length() - limit_min_squared
            // i.e. extending the line by ext_l on each end would shrink it
            // below the configured minimum length. This guards both
            // joint_line0 and joint_line1 — for `type == 2` (top-top) both
            // are still default zero-length segments and we'd fail here, so
            // the top-top branch below skips this check entirely.
            if joint_type < 2 {
                let joint_line_extension_limit = (ext_l * 2.0).powi(2);
                let limit_min_squared = config.limit_min_joint_length.powi(2);
                if i > 1
                    && joint_line_extension_limit
                        > joint_line0.squared_length() - limit_min_squared
                {
                    return None;
                }
                if j > 1
                    && joint_line_extension_limit
                        > joint_line1.squared_length() - limit_min_squared
                {
                    return None;
                }
                joint_line0.extend_equally(ext_l);
                joint_line1.extend_equally(ext_l);
            }

            // ── 8. Insertion direction (optional). ──
            // If either element has insertion vectors assigned, the male
            // (whichever has the higher face index) takes priority.
            let mut dir = Vector::new(0.0, 0.0, 0.0);
            let mut dir_set = false;
            if !insertion_vectors_0.is_empty() && !insertion_vectors_1.is_empty() {
                dir = if i > j {
                    insertion_vectors_0[i].clone()
                } else {
                    insertion_vectors_1[j].clone()
                };
                dir_set = (dir[0].abs() + dir[1].abs() + dir[2].abs()) > 0.01;
            }

            // ── 9. Branch on joint type (0 / 1 / 2). ──
            //      Each branch fills `joint_lines`, `joint_volumes_pair_a_pair_b`
            //      and the refined `joint_type` (11/12/13/20/40), then returns.
            let mut joint_lines = [
                Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(0.0, 0.0, 0.0)),
                Line::from_points(&Point::new(0.0, 0.0, 0.0), &Point::new(0.0, 0.0, 0.0)),
            ];
            let mut joint_volumes: [Option<Polyline>; 4] = [None, None, None, None];

            if joint_type == 0 {
                // ────────────────────────────────────────────────────────
                // SIDE-SIDE
                // ────────────────────────────────────────────────────────
                joint_lines[0] = joint_line0.clone();
                joint_lines[1] = joint_line1.clone();

                // Are the two side faces' alignment lines parallel? The
                // wood library distinguishes "rotated" (perpendicular or
                // skew) from "parallel" elements; the rotated branch builds
                // a single averaged rectangle, the parallel branch
                // distinguishes in-plane vs out-of-plane via dihedral angle.
                let v0 = Vector::new(
                    joint_line0.start()[0] - joint_line0.end()[0],
                    joint_line0.start()[1] - joint_line0.end()[1],
                    joint_line0.start()[2] - joint_line0.end()[2],
                );
                let v1 = Vector::new(
                    joint_line1.start()[0] - joint_line1.end()[0],
                    joint_line1.start()[1] - joint_line1.end()[1],
                    joint_line1.start()[2] - joint_line1.end()[2],
                );
                let parallel = v0.is_parallel_to(&v1);

                if parallel == 0
                    || config.face_to_face_side_to_side_joints_all_treated_as_rotated
                {
                    // ──────────────────────────────────────────────
                    // Rotated / perpendicular elements (type 13)
                    // ──────────────────────────────────────────────
                    //
                    // Build an averaged segment between the two alignment
                    // lines (matching endpoints by closest distance), then
                    // construct a local 2D frame around it, project the
                    // joint area into 2D, take its AABB, and extrude it to
                    // a thickness rectangle in 3D.

                    let average_segment = if Point::distance(
                        &joint_line0.start(),
                        &joint_line1.start(),
                        None,
                    ) < Point::distance(
                        &joint_line0.start(),
                        &joint_line1.end(),
                        None,
                    ) {
                        Line::from_points(
                            &Point::mid_point(&joint_line0.start(), &joint_line1.start()),
                            &Point::mid_point(&joint_line0.end(), &joint_line1.end()),
                        )
                    } else {
                        Line::from_points(
                            &Point::mid_point(&joint_line0.start(), &joint_line1.end()),
                            &Point::mid_point(&joint_line0.end(), &joint_line1.start()),
                        )
                    };
                    let axis_segment =
                        if config.face_to_face_side_to_side_joints_rotated_joint_as_average {
                            average_segment
                        } else {
                            joint_line0.clone()
                        };

                    // Local frame: x = along axis, z = face normal,
                    // y = z × x (then reorthogonalised against actual
                    // element thicknesses, see below).
                    let o = axis_segment.start();
                    let mut x = axis_segment.to_vector();
                    let z = planes_0[i].z_axis();
                    let mut y = z.cross(&x);
                    y.normalize_self();

                    // The C++ has an alternative branch when
                    // `rotated_joint_as_average == false`: y becomes the
                    // first plate's bottom-face normal, z becomes x×y.
                    let mut z = z;
                    if !config.face_to_face_side_to_side_joints_rotated_joint_as_average {
                        y = planes_0[0].z_axis();
                        z = x.cross(&y);
                    }

                    // Re-orient y by intersecting a thick test segment
                    // through the joint center with the two outer plates'
                    // top/bottom planes — this picks up the actual signed
                    // direction across the assembly.
                    let center_pt = polylines_0[i].center();
                    let thickness_a = (planes_0[0]
                        .origin()
                        .distance(&planes_0[1].projection(&planes_0[0].origin()), None))
                    .max(
                        planes_1[0]
                            .origin()
                            .distance(&planes_1[1].projection(&planes_1[0].origin()), None),
                    );
                    let mut y_scaled = y.clone();
                    y_scaled = Vector::new(
                        y_scaled[0] * (thickness_a * 2.0),
                        y_scaled[1] * (thickness_a * 2.0),
                        y_scaled[2] * (thickness_a * 2.0),
                    );
                    let y_line = Line::from_points(
                        &Point::new(
                            center_pt[0] + y_scaled[0],
                            center_pt[1] + y_scaled[1],
                            center_pt[2] + y_scaled[2],
                        ),
                        &Point::new(
                            center_pt[0] - y_scaled[0],
                            center_pt[1] - y_scaled[1],
                            center_pt[2] - y_scaled[2],
                        ),
                    );
                    if let Some(clipped) = line_two_planes(&y_line, &planes_0[0], &planes_1[1]) {
                        y = Vector::new(
                            clipped.end()[0] - clipped.start()[0],
                            clipped.end()[1] - clipped.start()[1],
                            clipped.end()[2] - clipped.start()[2],
                        );
                    }
                    x = y.cross(&z);

                    let xform = Xform::plane_to_xy(&o, &x, &y, &z);

                    // Project joint area into the local 2D frame and grab
                    // its axis-aligned bounding box.
                    let pts3d = joint_area.get_points();
                    let proj_pts: Vec<Point> = pts3d
                        .iter()
                        .map(|p| xform_apply_point(&xform, p))
                        .collect();
                    if proj_pts.is_empty() {
                        return None;
                    }
                    let mut xmin = proj_pts[0][0];
                    let mut xmax = xmin;
                    let mut ymin = proj_pts[0][1];
                    let mut ymax = ymin;
                    for p in &proj_pts[1..] {
                        if p[0] < xmin {
                            xmin = p[0];
                        } else if p[0] > xmax {
                            xmax = p[0];
                        }
                        if p[1] < ymin {
                            ymin = p[1];
                        } else if p[1] > ymax {
                            ymax = p[1];
                        }
                    }
                    // Average rectangle in local 2D, vertices ordered to
                    // match the C++ shape: { p0+x+y, p3, p1, p2 }.
                    let zmin = proj_pts[0][2];
                    let r0 = Point::new(xmax, ymax, zmin); // p0+x+y
                    let r1 = Point::new(xmin, ymax, zmin); // p3
                    let r2 = Point::new(xmin, ymin, zmin); // p1
                    let r3 = Point::new(xmax, ymin, zmin); // p2
                    let xform_inv = xform.inverse()?;
                    let r0_3d = xform_apply_point(&xform_inv, &r0);
                    let r1_3d = xform_apply_point(&xform_inv, &r1);
                    let r2_3d = xform_apply_point(&xform_inv, &r2);
                    let r3_3d = xform_apply_point(&xform_inv, &r3);
                    let average_rectangle = [r0_3d, r1_3d, r2_3d, r3_3d];

                    // Offset by the element thickness along the chosen
                    // axis (insertion direction if available, otherwise z).
                    let mut offset_vector = if dir_set { dir.clone() } else { z.clone() };
                    offset_vector.normalize_self();
                    let d0 = 0.5
                        * planes_0[0].origin().distance(
                            &planes_0[1].projection(&planes_0[0].origin()),
                            None,
                        );
                    offset_vector = Vector::new(
                        offset_vector[0] * d0,
                        offset_vector[1] * d0,
                        offset_vector[2] * d0,
                    );

                    // Build pair A (rectangle 0) and pair B (rectangle 1)
                    // by extruding average_rectangle along ±offset_vector.
                    let mk = |a: &Point, ov: &Vector| -> Polyline {
                        Polyline::new(vec![
                            Point::new(a[0] + ov[0], a[1] + ov[1], a[2] + ov[2]),
                            Point::new(a[0] - ov[0], a[1] - ov[1], a[2] - ov[2]),
                            Point::new(a[0] - ov[0], a[1] - ov[1], a[2] - ov[2]),
                            Point::new(a[0] + ov[0], a[1] + ov[1], a[2] + ov[2]),
                            Point::new(a[0] + ov[0], a[1] + ov[1], a[2] + ov[2]),
                        ])
                    };
                    // The C++ form is more specific: each rectangle uses
                    // average_rectangle vertices [3] and [0] (or [2] and
                    // [1]), each offset ±. We replicate it directly.
                    let mut vol0 = Polyline::new(vec![
                        Point::new(
                            average_rectangle[3][0] + offset_vector[0],
                            average_rectangle[3][1] + offset_vector[1],
                            average_rectangle[3][2] + offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[3][0] - offset_vector[0],
                            average_rectangle[3][1] - offset_vector[1],
                            average_rectangle[3][2] - offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[0][0] - offset_vector[0],
                            average_rectangle[0][1] - offset_vector[1],
                            average_rectangle[0][2] - offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[0][0] + offset_vector[0],
                            average_rectangle[0][1] + offset_vector[1],
                            average_rectangle[0][2] + offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[3][0] + offset_vector[0],
                            average_rectangle[3][1] + offset_vector[1],
                            average_rectangle[3][2] + offset_vector[2],
                        ),
                    ]);
                    let mut vol1 = Polyline::new(vec![
                        Point::new(
                            average_rectangle[2][0] + offset_vector[0],
                            average_rectangle[2][1] + offset_vector[1],
                            average_rectangle[2][2] + offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[2][0] - offset_vector[0],
                            average_rectangle[2][1] - offset_vector[1],
                            average_rectangle[2][2] - offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[1][0] - offset_vector[0],
                            average_rectangle[1][1] - offset_vector[1],
                            average_rectangle[1][2] - offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[1][0] + offset_vector[0],
                            average_rectangle[1][1] + offset_vector[1],
                            average_rectangle[1][2] + offset_vector[2],
                        ),
                        Point::new(
                            average_rectangle[2][0] + offset_vector[0],
                            average_rectangle[2][1] + offset_vector[1],
                            average_rectangle[2][2] + offset_vector[2],
                        ),
                    ]);

                    // Apply joint width/height extensions to all 4 edges of
                    // each rectangle (opposite-edge pairs preserve shape).
                    for &k in &[0_usize, 2] {
                        extend_polyline_edge_equally(&mut vol0, k, ext_w);
                        extend_polyline_edge_equally(&mut vol1, k, ext_w);
                    }
                    for &k in &[1_usize, 3] {
                        extend_polyline_edge_equally(&mut vol0, k, ext_h);
                        extend_polyline_edge_equally(&mut vol1, k, ext_h);
                    }

                    joint_volumes[0] = Some(vol0);
                    joint_volumes[1] = Some(vol1);
                    joint_type = 13;
                    let _ = mk; // helper kept for parity with C++ comments

                    return Some(WoodJoint {
                        el_ids,
                        face_ids,
                        joint_type,
                        joint_area,
                        joint_lines,
                        joint_volumes_pair_a_pair_b: joint_volumes,
                    });
                } else {
                    // ──────────────────────────────────────────────
                    // Parallel elements
                    // ──────────────────────────────────────────────
                    //
                    // Take the averaged overlap between joint_line0 and
                    // joint_line1, then split on dihedral angle:
                    //   <  20°            → invalid
                    //   ≤  configured cut → out-of-plane (type 11)
                    //   >  configured cut → in-plane     (type 12)

                    let lj = line_line_overlap_average(&joint_line0, &joint_line1)?;
                    joint_lines[0] = lj.clone();
                    joint_lines[1] = lj.clone();

                    // End planes that bound the joint along its axis.
                    let mut pl_end0 = Plane::from_point_normal(lj.start(), lj.to_vector());
                    if dir_set {
                        pl_end0 = Plane::from_point_normal(lj.start(), dir.clone());
                    }
                    let pl_end1 = Plane::from_point_normal(lj.end(), pl_end0.z_axis());

                    // Dihedral angle of the joint edge in the tetrahedron
                    // (lj.start, lj.end, center0, center1).
                    let center0 =
                        avg_plane_0.projection(&polylines_0[0].center());
                    let center1 =
                        avg_plane_1.projection(&polylines_1[0].center());
                    let dihedral = approximate_dihedral_angle(
                        &lj.start(),
                        &lj.end(),
                        &center0,
                        &center1,
                    );

                    if dihedral < 20.0 {
                        return None;
                    } else if dihedral
                        <= config.face_to_face_side_to_side_joints_dihedral_angle
                    {
                        // ────── Out-of-plane (type 11) ──────
                        //
                        // Probe the joint axis 90° (in the face plane) to
                        // figure out which adjacent element planes are
                        // closer, then build an "open" plane×4-plane
                        // intersection to get a quad.

                        let connection_normal = planes_0[i].z_axis();
                        let lj_normal = lj.to_vector();
                        let lj_v_90_unscaled = lj_normal.cross(&connection_normal);
                        let lj_v_90 = Vector::new(
                            lj_v_90_unscaled[0] * 0.5,
                            lj_v_90_unscaled[1] * 0.5,
                            lj_v_90_unscaled[2] * 0.5,
                        );
                        let lj_l_90 = Line::new(
                            lj.start()[0],
                            lj.start()[1],
                            lj.start()[2],
                            lj.start()[0] + lj_v_90[0],
                            lj.start()[1] + lj_v_90[1],
                            lj.start()[2] + lj_v_90[2],
                        );
                        let pl0_0_p = line_plane(&lj_l_90, &planes_0[0], false)?;
                        let pl1_0_p = line_plane(&lj_l_90, &planes_1[0], false)?;
                        let pl1_1_p = line_plane(&lj_l_90, &planes_1[1], false)?;

                        // Choose the adjacent element planes by which is
                        // farther from the probe point on plane0[0].
                        let d_to_pl1_0 = Point::distance(&pl0_0_p, &pl1_0_p, None);
                        let d_to_pl1_1 = Point::distance(&pl0_0_p, &pl1_1_p, None);
                        let larger_to_pl1_0 = d_to_pl1_0 > d_to_pl1_1;
                        let planes4: [Plane; 4] = if larger_to_pl1_0 {
                            [
                                planes_1[1].clone(),
                                planes_0[0].clone(),
                                planes_1[0].clone(),
                                planes_0[1].clone(),
                            ]
                        } else {
                            [
                                planes_1[0].clone(),
                                planes_0[0].clone(),
                                planes_1[1].clone(),
                                planes_0[1].clone(),
                            ]
                        };

                        let mut vol0 = plane_4planes_open(&pl_end0, &planes4)?;
                        let mut vol1 = plane_4planes_open(&pl_end1, &planes4)?;

                        // Consistent volume orientation: rotate the
                        // 4-vertex Polyline by 2 if the second vertex is
                        // not on the negative side of plane_0[i].
                        let need_rotate = {
                            let p1 = vol0.get_point(1).unwrap();
                            !planes_0[i].has_on_negative_side(&p1)
                        };
                        if need_rotate {
                            let pts0: Vec<Point> = (0..vol0.point_count())
                                .map(|k| vol0.get_point(k).unwrap())
                                .collect();
                            let pts1: Vec<Point> = (0..vol1.point_count())
                                .map(|k| vol1.get_point(k).unwrap())
                                .collect();
                            let n0 = pts0.len();
                            let n1 = pts1.len();
                            let mut rot0 = Vec::with_capacity(n0);
                            for k in 0..n0 {
                                rot0.push(pts0[(k + 2) % n0].clone());
                            }
                            let mut rot1 = Vec::with_capacity(n1);
                            for k in 0..n1 {
                                rot1.push(pts1[(k + 2) % n1].clone());
                            }
                            vol0 = Polyline::new(rot0);
                            vol1 = Polyline::new(rot1);
                        }

                        // The C++ then reverses the volumes AND swaps the
                        // element ids — the male/female flip. We do the
                        // same so the joint library always sees the male
                        // element first.
                        let pts0: Vec<Point> = (0..vol0.point_count())
                            .map(|k| vol0.get_point(k).unwrap())
                            .rev()
                            .collect();
                        let pts1: Vec<Point> = (0..vol1.point_count())
                            .map(|k| vol1.get_point(k).unwrap())
                            .rev()
                            .collect();
                        let n0 = pts0.len();
                        let n1 = pts1.len();
                        let mut rot0 = Vec::with_capacity(n0);
                        for k in 0..n0 {
                            rot0.push(pts0[(k + 3) % n0].clone());
                        }
                        let mut rot1 = Vec::with_capacity(n1);
                        for k in 0..n1 {
                            rot1.push(pts1[(k + 3) % n1].clone());
                        }
                        vol0 = Polyline::new(rot0);
                        vol1 = Polyline::new(rot1);
                        el_ids = (el_ids.1, el_ids.0);
                        face_ids = (face_ids.1, face_ids.0);
                        joint_lines.reverse();

                        // Close the rectangles (append the first vertex).
                        let p0_0 = vol0.get_point(0).unwrap();
                        vol0.add_point(p0_0);
                        let p0_1 = vol1.get_point(0).unwrap();
                        vol1.add_point(p0_1);

                        // Apply width/height extensions on opposite edges.
                        for &k in &[0_usize, 2] {
                            extend_polyline_edge_equally(&mut vol0, k, ext_w);
                            extend_polyline_edge_equally(&mut vol1, k, ext_w);
                        }
                        for &k in &[1_usize, 3] {
                            extend_polyline_edge_equally(&mut vol0, k, ext_h);
                            extend_polyline_edge_equally(&mut vol1, k, ext_h);
                        }

                        joint_volumes[0] = Some(vol0);
                        joint_volumes[1] = Some(vol1);
                        joint_type = 11;

                        return Some(WoodJoint {
                            el_ids,
                            face_ids,
                            joint_type,
                            joint_area,
                            joint_lines,
                            joint_volumes_pair_a_pair_b: joint_volumes,
                        });
                    } else {
                        // ────── In-plane (type 12) ──────
                        //
                        // Compute two planes offset from the matched face
                        // plane by ±half the element thickness, then form
                        // two 4-plane loops (one per element) and intersect
                        // each with the two end planes → 4 joint volumes.

                        let d0 = 0.5
                            * planes_0[0].origin().distance(
                                &planes_0[1].projection(&planes_0[0].origin()),
                                None,
                            );
                        let offset_plane_0 = planes_0[i].translate_by_normal(-d0);
                        let offset_plane_1 = planes_0[i].translate_by_normal(d0);

                        // Winding fix: if plane1[0] is farther from
                        // plane0[0] than plane1[1] is, swap so the loop
                        // goes around the joint consistently.
                        let pt00 = planes_0[0].origin();
                        let proj00 = planes_1[0].projection(&pt00);
                        let proj01 = planes_1[1].projection(&pt00);
                        let w0 = Point::distance(&pt00, &proj00, None);
                        let w1 = Point::distance(&pt00, &proj01, None);
                        let (p1_0, p1_1) = if w0 > w1 {
                            (planes_1[1].clone(), planes_1[0].clone())
                        } else {
                            (planes_1[0].clone(), planes_1[1].clone())
                        };

                        let loop_planes_0: [Plane; 4] = [
                            offset_plane_0.clone(),
                            planes_0[0].clone(),
                            offset_plane_1.clone(),
                            planes_0[1].clone(),
                        ];
                        let loop_planes_1: [Plane; 4] = [
                            offset_plane_0.clone(),
                            p1_0,
                            offset_plane_1.clone(),
                            p1_1,
                        ];

                        let mut vol0 = plane_4planes(&pl_end0, &loop_planes_0)?;
                        let mut vol1 = plane_4planes(&pl_end1, &loop_planes_0)?;
                        let mut vol2 = plane_4planes(&pl_end0, &loop_planes_1)?;
                        let mut vol3 = plane_4planes(&pl_end1, &loop_planes_1)?;

                        for vol in [&mut vol0, &mut vol1, &mut vol2, &mut vol3].iter_mut() {
                            for &k in &[0_usize, 2] {
                                extend_polyline_edge_equally(vol, k, ext_w);
                            }
                            for &k in &[1_usize, 3] {
                                extend_polyline_edge_equally(vol, k, ext_h);
                            }
                        }

                        joint_volumes[0] = Some(vol0);
                        joint_volumes[1] = Some(vol1);
                        joint_volumes[2] = Some(vol2);
                        joint_volumes[3] = Some(vol3);
                        joint_type = 12;

                        return Some(WoodJoint {
                            el_ids,
                            face_ids,
                            joint_type,
                            joint_area,
                            joint_lines,
                            joint_volumes_pair_a_pair_b: joint_volumes,
                        });
                    }
                }
            } else if joint_type == 1 {
                // ────────────────────────────────────────────────────────
                // TOP-SIDE (type 20)
                // ────────────────────────────────────────────────────────
                //
                // The element with the higher face index is the male (its
                // side face is what defines the joint axis). The female is
                // the other element's top/bottom face. The joint volume is
                // built by extruding the male's side-face quad (`joint_quads`)
                // by an offset vector that spans the female's thickness.

                let male_or_female = i > j; // true: male = element 0
                let joint_line_for_volumes = if male_or_female {
                    joint_line0.clone()
                } else {
                    joint_line1.clone()
                };
                joint_lines[0] = joint_line_for_volumes.clone();
                joint_lines[1] = joint_line_for_volumes;

                let plane0_0 = if male_or_female {
                    planes_0[0].clone()
                } else {
                    planes_1[0].clone()
                };
                // Female collision plane (top of the female element).
                let plane1_0 = if !male_or_female {
                    planes_0[i].clone()
                } else {
                    planes_1[j].clone()
                };
                let other_idx = if !male_or_female {
                    (i as i32 - 1).unsigned_abs() as usize
                } else {
                    (j as i32 - 1).unsigned_abs() as usize
                };
                let plane1_1 = if !male_or_female {
                    planes_0[other_idx].clone()
                } else {
                    planes_1[other_idx].clone()
                };
                let quad_0_owned = if male_or_female {
                    joint_quads0.clone()
                } else {
                    joint_quads1.clone()
                };
                let quad_0 = quad_0_owned?;

                let mut offset_vector = get_orthogonal_vector_between_two_plane_pairs(
                    &plane0_0,
                    &plane1_0,
                    &plane1_1,
                )?;
                if dir_set {
                    if let Some(scaled) = scale_vector_to_distance_of_2planes(
                        &dir,
                        &plane1_0,
                        &plane1_1,
                    ) {
                        offset_vector = scaled;
                    }
                }

                // If the female (= top) element is element 0, swap so the
                // joint library always sees the male first.
                if !male_or_female {
                    el_ids = (el_ids.1, el_ids.0);
                    face_ids = (face_ids.1, face_ids.0);
                }

                let m_id = if male_or_female { 0 } else { 1 };
                let f_id = if male_or_female { 1 } else { 0 };
                let q0 = quad_0.get_point(0)?;
                let q1 = quad_0.get_point(1)?;
                let q2 = quad_0.get_point(2)?;
                let q3 = quad_0.get_point(3)?;

                let mk_quad = |a: &Point, b: &Point, ov: &Vector| -> Polyline {
                    Polyline::new(vec![
                        a.clone(),
                        b.clone(),
                        Point::new(b[0] + ov[0], b[1] + ov[1], b[2] + ov[2]),
                        Point::new(a[0] + ov[0], a[1] + ov[1], a[2] + ov[2]),
                        a.clone(),
                    ])
                };
                let mut male_vol = mk_quad(&q0, &q1, &offset_vector);
                let mut female_vol = mk_quad(&q3, &q2, &offset_vector);

                for &k in &[0_usize, 2] {
                    extend_polyline_edge_equally(&mut male_vol, k, ext_w);
                    extend_polyline_edge_equally(&mut female_vol, k, ext_w);
                }
                for &k in &[1_usize, 3] {
                    extend_polyline_edge_equally(&mut male_vol, k, ext_h);
                    extend_polyline_edge_equally(&mut female_vol, k, ext_h);
                }
                joint_volumes[m_id] = Some(male_vol);
                joint_volumes[f_id] = Some(female_vol);
                joint_type = 20;

                return Some(WoodJoint {
                    el_ids,
                    face_ids,
                    joint_type,
                    joint_area,
                    joint_lines,
                    joint_volumes_pair_a_pair_b: joint_volumes,
                });
            } else {
                // ────────────────────────────────────────────────────────
                // TOP-TOP (type 40)
                // ────────────────────────────────────────────────────────
                //
                // Build the bounding rectangle of the joint area in the
                // shared plane, then translate it ±thickness along each
                // element's normal to form two extruded slabs. The four
                // corners of those slabs are reorganised into two
                // rectangles matching the wood::joint_lib convention.

                let rect = Polyline::bounding_rectangle(&joint_area)?;
                let mut vol_a = rect.clone();
                let mut vol_b = rect;

                // Movement direction (insertion vector if available, else
                // the face normal). The C++ flips the sign twice, leaving
                // dir0 in the original direction and dir1 = -dir0.
                let mut dir0 = if dir_set {
                    if i < insertion_vectors_0.len() {
                        insertion_vectors_0[i].clone()
                    } else {
                        planes_0[i].z_axis()
                    }
                } else {
                    planes_0[i].z_axis()
                };
                dir0.normalize_self();
                let dir1_pre = Vector::new(-dir0[0], -dir0[1], -dir0[2]);
                // After both negations the C++ ends up with `dir0 *= -1`
                // and `dir1 *= -1`, i.e. dir0 flipped and dir1 = +dir0.
                let dir0 = Vector::new(-dir0[0], -dir0[1], -dir0[2]);
                let dir1 = Vector::new(-dir1_pre[0], -dir1_pre[1], -dir1_pre[2]);

                // Element thicknesses across the matched face.
                let next_plane_0 = if i == 0 { 1 } else { 0 };
                let next_plane_1 = if j == 0 { 1 } else { 0 };
                let dist_0 = planes_0[i]
                    .origin()
                    .distance(&planes_0[next_plane_0].projection(&planes_0[i].origin()), None);
                let dist_1 = planes_1[j]
                    .origin()
                    .distance(&planes_1[next_plane_1].projection(&planes_1[j].origin()), None);
                let dir0 =
                    Vector::new(dir0[0] * dist_0, dir0[1] * dist_0, dir0[2] * dist_0);
                let dir1 =
                    Vector::new(dir1[0] * dist_1, dir1[1] * dist_1, dir1[2] * dist_1);

                // Translate the rectangles.
                for k in 0..vol_a.point_count() {
                    let p = vol_a.get_point(k).unwrap();
                    vol_a.set_point(
                        k,
                        &Point::new(p[0] + dir0[0], p[1] + dir0[1], p[2] + dir0[2]),
                    );
                }
                for k in 0..vol_b.point_count() {
                    let p = vol_b.get_point(k).unwrap();
                    vol_b.set_point(
                        k,
                        &Point::new(p[0] + dir1[0], p[1] + dir1[1], p[2] + dir1[2]),
                    );
                }

                // Reformat into the two-rectangle convention used by the
                // joint library: temp0 = (a[0], a[1], b[1], b[0], a[0]),
                // temp1 = (a[3], a[2], b[2], b[3], a[3]).
                let a0 = vol_a.get_point(0)?;
                let a1 = vol_a.get_point(1)?;
                let a2 = vol_a.get_point(2)?;
                let a3 = vol_a.get_point(3)?;
                let b0 = vol_b.get_point(0)?;
                let b1 = vol_b.get_point(1)?;
                let b2 = vol_b.get_point(2)?;
                let b3 = vol_b.get_point(3)?;

                let mut temp0 = Polyline::new(vec![
                    a0.clone(),
                    a1.clone(),
                    b1.clone(),
                    b0.clone(),
                    a0.clone(),
                ]);
                let mut temp1 = Polyline::new(vec![
                    a3.clone(),
                    a2.clone(),
                    b2.clone(),
                    b3.clone(),
                    a3.clone(),
                ]);

                for &k in &[0_usize, 2] {
                    extend_polyline_edge_equally(&mut temp0, k, ext_w);
                    extend_polyline_edge_equally(&mut temp1, k, ext_w);
                }
                for &k in &[1_usize, 3] {
                    extend_polyline_edge_equally(&mut temp0, k, ext_h);
                    extend_polyline_edge_equally(&mut temp1, k, ext_h);
                }

                joint_volumes[0] = Some(temp0);
                joint_volumes[1] = Some(temp1);
                joint_type = 40;

                return Some(WoodJoint {
                    el_ids,
                    face_ids,
                    joint_type,
                    joint_area,
                    joint_lines,
                    joint_volumes_pair_a_pair_b: joint_volumes,
                });
            }
        }
    }
    None
}

/// Thin wrapper over `Polyline::boolean_op` mirroring C++ `Intersection::polyline_boolean`.
pub fn polyline_boolean(a: &Polyline, b: &Polyline, clip_type: i32) -> Vec<Polyline> {
    Polyline::boolean_op(a, b, clip_type)
}

/// 2D boolean between two closed planar polylines, projected into the plane's
/// canonical 2D frame (`base1`/`base2`). `intersection_type`: 0=Intersect,
/// 1=Union, 2=Difference, 3=Xor. Returns the result polyline (closed, 3D) on
/// success, or `None` on empty/degenerate/triangle-reject/sub-`min_area`.
/// Verbatim port of C++ `Intersection::polyline_boolean_2d_in_plane`.
pub fn polyline_boolean_2d_in_plane(
    polyline0: &Polyline,
    polyline1: &Polyline,
    plane: &crate::Plane,
    intersection_type: i32,
    include_triangles: bool,
    min_area: f32,
    collapse_eps: f32,
) -> Option<Polyline> {
    let n0 = polyline0.point_count();
    let n1 = polyline1.point_count();
    if n0 < 3 || n1 < 3 {
        return None;
    }
    let origin = polyline0.get_point(0)?;
    let xax = plane.base1();
    let yax = plane.base2();

    let to_2d = |pl: &Polyline| -> Polyline {
        let n = pl.point_count();
        let mut pts2d: Vec<Point> = Vec::with_capacity(n + 1);
        for i in 0..n {
            let p = pl.get_point(i).unwrap();
            let dx = p[0]-origin[0]; let dy = p[1]-origin[1]; let dz = p[2]-origin[2];
            let u = dx*xax[0] + dy*xax[1] + dz*xax[2];
            let v = dx*yax[0] + dy*yax[1] + dz*yax[2];
            pts2d.push(Point::new(u, v, 0.0));
        }
        if pts2d.len() > 1 {
            let f = &pts2d[0];
            let l = pts2d.last().unwrap();
            let dx = f[0]-l[0]; let dy = f[1]-l[1];
            if dx*dx + dy*dy > 1e-12 {
                let p0 = pts2d[0].clone();
                pts2d.push(p0);
            }
        }
        Polyline::new(pts2d)
    };
    let a2d = to_2d(polyline0);
    let b2d = to_2d(polyline1);

    let result_2d: Vec<Polyline> = if (0..=2).contains(&intersection_type) {
        Polyline::boolean_op(&a2d, &b2d, intersection_type)
    } else if intersection_type == 3 {
        let u = Polyline::boolean_op(&a2d, &b2d, 1);
        let inter = Polyline::boolean_op(&a2d, &b2d, 0);
        if u.is_empty() {
            return None;
        }
        if inter.is_empty() { u } else { Polyline::boolean_op(&u[0], &inter[0], 2) }
    } else {
        return None;
    };
    if result_2d.is_empty() {
        return None;
    }

    let c = &result_2d[0];
    let mut nc = c.point_count();
    if nc > 1 {
        let f = c.get_point(0).unwrap();
        let l = c.get_point(nc-1).unwrap();
        let dx = f[0]-l[0]; let dy = f[1]-l[1];
        if dx*dx + dy*dy < 1e-12 {
            nc -= 1;
        }
    }
    if nc < 3 {
        return None;
    }

    let mut src2d: Vec<Point> = (0..nc).map(|i| c.get_point(i).unwrap()).collect();
    if collapse_eps > 0.0 {
        let eps_sq = collapse_eps * collapse_eps;
        let mut collapsed: Vec<Point> = Vec::with_capacity(src2d.len());
        for p in &src2d {
            if let Some(last) = collapsed.last() {
                let dx = p[0] - last[0];
                let dy = p[1] - last[1];
                if dx*dx + dy*dy < eps_sq {
                    continue;
                }
            }
            collapsed.push(p.clone());
        }
        if collapsed.len() >= 2 {
            let dx = collapsed.last().unwrap()[0] - collapsed[0][0];
            let dy = collapsed.last().unwrap()[1] - collapsed[0][1];
            if dx*dx + dy*dy < eps_sq {
                collapsed.pop();
            }
        }
        src2d = collapsed;
        nc = src2d.len();
        if nc < 3 {
            return None;
        }
    }
    if nc == 3 && !include_triangles {
        return None;
    }

    let mut area = 0.0;
    for i in 0..nc {
        let p0 = &src2d[i];
        let p1 = &src2d[(i+1) % nc];
        area += p0[0]*p1[1] - p1[0]*p0[1];
    }
    if area.abs() * 0.5 <= min_area {
        return None;
    }

    let mut pts: Vec<Point> = Vec::with_capacity(nc + 1);
    for p in &src2d {
        let u = p[0]; let v = p[1];
        pts.push(Point::new(
            origin[0] + u*xax[0] + v*yax[0],
            origin[1] + u*xax[1] + v*yax[1],
            origin[2] + u*xax[2] + v*yax[2],
        ));
    }
    pts.push(pts[0].clone());
    Some(Polyline::new(pts))
}

/// Native miter-join polygon offset in plane-space 2D. Verbatim port of
/// C++ `Intersection::offset_in_3d`. Mutates `polyline` in place and returns
/// true on success. Uses `plane.base1()/base2()` canonical axes so the output
/// is deterministic across plane constructions.
pub fn offset_in_3d(polyline: &mut Polyline, plane: &crate::Plane, offset: f32) -> bool {
    let n_raw = polyline.point_count();
    if n_raw < 3 {
        return false;
    }
    let origin = polyline.get_point(0).unwrap();
    let xax = plane.base1();
    let yax = plane.base2();

    let mut path: Vec<(f32, f32)> = Vec::with_capacity(n_raw);
    for i in 0..n_raw {
        let p = polyline.get_point(i).unwrap();
        let dx = p[0] - origin[0]; let dy = p[1] - origin[1]; let dz = p[2] - origin[2];
        let u = dx*xax[0] + dy*xax[1] + dz*xax[2];
        let v = dx*yax[0] + dy*yax[1] + dz*yax[2];
        path.push((u, v));
    }
    if path.len() >= 2 {
        let dx = path.last().unwrap().0 - path[0].0;
        let dy = path.last().unwrap().1 - path[0].1;
        if dx*dx + dy*dy < 1e-12 {
            path.pop();
        }
    }
    let n = path.len();
    if n < 3 {
        return false;
    }

    let mut signed_area = 0.0;
    for i in 0..n {
        let (ax, ay) = path[i];
        let (bx, by) = path[(i+1) % n];
        signed_area += ax * by - bx * ay;
    }
    let delta = if signed_area < 0.0 { -offset } else { offset };

    let mut normals: Vec<(f32, f32)> = Vec::with_capacity(n);
    for i in 0..n {
        let (ax, ay) = path[i];
        let (bx, by) = path[(i+1) % n];
        let ex = bx - ax; let ey = by - ay;
        let len = (ex*ex + ey*ey).sqrt();
        if len < 1e-12 {
            normals.push((0.0, 0.0));
        } else {
            normals.push((ey/len, -ex/len));
        }
    }

    let mut out: Vec<(f32, f32)> = Vec::with_capacity(n * 3);
    for i in 0..n {
        let (npx, npy) = normals[(i + n - 1) % n];
        let (nnx, nny) = normals[i];
        let cos_a = npx*nnx + npy*nny;
        let sin_a = npx*nny - npy*nnx;
        let denom = 1.0 + cos_a;
        let concave = (cos_a > -0.999) && (sin_a * delta < 0.0) && (offset > 0.0);
        let (px, py) = path[i];
        if concave {
            out.push((px + npx * delta, py + npy * delta));
            out.push((px, py));
            out.push((px + nnx * delta, py + nny * delta));
        } else if denom.abs() < 1e-9 {
            let bx = npx + nnx; let by = npy + nny;
            let bl = (bx*bx + by*by).sqrt();
            if bl < 1e-12 {
                out.push((px + nnx * delta, py + nny * delta));
            } else {
                out.push((px + (bx/bl) * delta, py + (by/bl) * delta));
            }
        } else {
            let k = delta / denom;
            out.push((px + (npx + nnx) * k, py + (npy + nny) * k));
        }
    }
    let nout = out.len();
    if nout < 3 {
        return false;
    }

    let mut out_area = 0.0;
    for i in 0..nout {
        let (ax, ay) = out[i];
        let (bx, by) = out[(i+1) % nout];
        out_area += ax * by - bx * ay;
    }
    if out_area.abs() * 0.5 < 0.0001 {
        return false;
    }

    let mut cp = 0usize;
    let mut cd = (out[0].0 - path[0].0).powi(2) + (out[0].1 - path[0].1).powi(2);
    for i in 1..nout {
        let d = (out[i].0 - path[0].0).powi(2) + (out[i].1 - path[0].1).powi(2);
        if d < cd { cd = d; cp = i; }
    }
    if cp != 0 {
        out.rotate_left(cp);
    }

    let mut pts: Vec<Point> = Vec::with_capacity(nout + 1);
    for &(u, v) in &out {
        pts.push(Point::new(
            origin[0] + u*xax[0] + v*yax[0],
            origin[1] + u*xax[1] + v*yax[1],
            origin[2] + u*xax[2] + v*yax[2],
        ));
    }
    pts.push(pts[0].clone());
    *polyline = Polyline::new(pts);
    true
}

pub fn adjacency_search(elements: &mut [crate::element::Element], inflate: f32) -> Vec<i32> {
    use crate::obb::OBB;
    use crate::spatial_bvh::SpatialBVH;

    let n = elements.len();
    let mut obbs: Vec<OBB> = Vec::with_capacity(n);
    for elem in elements.iter_mut() {
        let mut pts: Vec<Point> = Vec::new();
        for pl in elem.polylines() {
            for p in pl.get_points() { pts.push(p); }
        }
        obbs.push(OBB::from_points(&pts, inflate));
    }

    let mut bvh = SpatialBVH::new();
    bvh.build(&obbs);
    let mut adjacency: Vec<i32> = Vec::new();
    for i in 0..n {
        let hits = bvh.query_aabb(&obbs[i]);
        for j in hits {
            if (i as i32) < (j as i32) && obbs[i].collides_with(&obbs[j]) {
                adjacency.push(i as i32);
                adjacency.push(j as i32);
                adjacency.push(-1);
                adjacency.push(-1);
            }
        }
    }
    adjacency
}
