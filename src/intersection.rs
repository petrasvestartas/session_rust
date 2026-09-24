use crate::aabb::AABB;
use crate::closest::Closest;
use crate::element::Element;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbsknot::CurveInterpStyle;
use crate::nurbsknot::CurveNurbsKnotStyle;
use crate::nurbssurface::NurbsSurface;
use crate::obb::OBB;
use crate::plane::Plane;
use crate::point::Point;
use crate::polyline::Polyline;
use crate::spatial_bvh::SpatialBVH;
use crate::tolerance::Tolerance;
use crate::vector::Vector;

// ═══════════════════════════════════════════════════════════════════════════
// Lines and planes
// ═══════════════════════════════════════════════════════════════════════════

/// Signed plane equation value at a point.
fn plane_value_at(plane: &Plane, point: &Point) -> f64 {
    plane.a() * point[0] + plane.b() * point[1] + plane.c() * point[2] + plane.d()
}

/// Intersection point of two segments, the midpoint of closest approach within tolerance.
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

/// Parameters (0 or 1) of an exactly shared endpoint of two segments, or None.
fn shared_endpoint_parameters(line0: &Line, line1: &Line) -> Option<(f64, f64)> {
    let ends0 = [line0.start(), line0.end()];
    let ends1 = [line1.start(), line1.end()];

    for (i, e0) in ends0.iter().enumerate() {
        for (j, e1) in ends1.iter().enumerate() {
            if e0[0] == e1[0] && e0[1] == e1[1] && e0[2] == e1[2] {
                return Some((i as f64, j as f64));
            }
        }
    }

    None
}

/// Clamp a parameter to [0, 1].
fn clamp_unit(t: f64) -> f64 {
    if t < 0.0 {
        return 0.0;
    }

    if t > 1.0 {
        return 1.0;
    }

    t
}

/// Parameters of closest approach of two lines, clamped to the segments when requested.
pub fn line_line_parameters(
    line0: &Line,
    line1: &Line,
    tolerance: f64,
    intersect_segments: bool,
    near_parallel_as_closest: bool,
) -> Option<(f64, f64)> {
    if let Some(shared) = shared_endpoint_parameters(line0, line1) {
        return Some(shared);
    }

    let a = line0.to_vector();
    let b = line1.to_vector();
    let c = line1.start() - line0.start();

    let aa = a.dot(&a);
    let bb = b.dot(&b);
    let ab = a.dot(&b);
    let ac = a.dot(&c);
    let bc = b.dot(&c);

    let det = aa * bb - ab * ab;
    let zero_tol = aa.max(bb) * f64::EPSILON;
    let parallel = det.abs() < zero_tol;

    if parallel && !near_parallel_as_closest {
        return None;
    }

    let (mut t0, mut t1) = if parallel {
        let t0 = if aa > 0.0 { ac / aa } else { 0.0 };
        let t1 = if bb > 0.0 { (bc + t0 * ab) / bb } else { 0.0 };

        (t0, t1)
    } else {
        let inv_det = 1.0 / det;

        ((bb * ac - ab * bc) * inv_det, (ab * ac - aa * bc) * inv_det)
    };

    if intersect_segments {
        t0 = clamp_unit(t0);
        t1 = clamp_unit(t1);
    }

    if tolerance > 0.0 && line0.point_at(t0).distance(&line1.point_at(t1), None) > tolerance {
        return None;
    }

    Some((t0, t1))
}

/// Intersection line of two planes, anchored on plane0's origin.
pub fn plane_plane(plane0: &Plane, plane1: &Plane) -> Option<Line> {
    let d = plane1.z_axis().cross(&plane0.z_axis());

    let origin0 = plane0.origin();
    let origin1 = plane1.origin();
    let p = Point::new(
        (origin0[0] + origin1[0]) * 0.5,
        (origin0[1] + origin1[1]) * 0.5,
        (origin0[2] + origin1[2]) * 0.5,
    );

    let plane2 = Plane::from_point_normal(p, d.clone(), None);

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

/// Intersection line of two planes, anchored at the foot of the world origin.
pub fn plane_plane_to_line_canonical(plane0: &Plane, plane1: &Plane) -> Option<Line> {
    let n0 = plane0.z_axis();
    let n1 = plane1.z_axis();
    let d = Vector::new(
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

/// Intersection point of a line and a plane.
pub fn line_plane(line: &Line, plane: &Plane, is_finite: bool) -> Option<Point> {
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
        if line[0] == line[3] {
            line[0]
        } else {
            s * line[0] + t * line[3]
        },
        if line[1] == line[4] {
            line[1]
        } else {
            s * line[1] + t * line[4]
        },
        if line[2] == line[5] {
            line[2]
        } else {
            s * line[2] + t * line[5]
        },
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

/// Intersection point of three planes.
pub fn plane_plane_plane(plane0: &Plane, plane1: &Plane, plane2: &Plane) -> Option<Point> {
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

// ═══════════════════════════════════════════════════════════════════════════
// Rays
// ═══════════════════════════════════════════════════════════════════════════

/// Ray-mesh hit.
#[derive(Debug, Clone)]
pub struct RayHit {
    pub t: f64,          // Parameter along the ray.
    pub point: Point,    // Hit point.
    pub u: f64,          // Barycentric u.
    pub v: f64,          // Barycentric v.
    pub face_index: i32, // Hit face.
}

impl Default for RayHit {
    /// Construct an empty miss.
    fn default() -> Self {
        RayHit::new(0.0, Point::default(), 0.0, 0.0, -1)
    }
}

impl RayHit {
    /// Construct from ray parameter, point, barycentrics and face.
    pub fn new(t: f64, point: Point, u: f64, v: f64, face_index: i32) -> Self {
        RayHit {
            t,
            point,
            u,
            v,
            face_index,
        }
    }
}

/// Ray-box slab test returning the entry and exit parameters.
pub fn ray_box_parameters(
    origin: &Point,
    direction: &Vector,
    box_: &OBB,
    t0: f64,
    t1: f64,
) -> (bool, f64, f64) {
    let box_min = box_.min_point();
    let box_max = box_.max_point();

    let inv_dir = Vector::new(
        if direction[0] != 0.0 {
            1.0 / direction[0]
        } else {
            f64::MAX
        },
        if direction[1] != 0.0 {
            1.0 / direction[1]
        } else {
            f64::MAX
        },
        if direction[2] != 0.0 {
            1.0 / direction[2]
        } else {
            f64::MAX
        },
    );

    let tx1 = (box_min[0] - origin[0]) * inv_dir[0];
    let tx2 = (box_max[0] - origin[0]) * inv_dir[0];

    let mut tmin = tx1.min(tx2);
    let mut tmax = tx1.max(tx2);

    let ty1 = (box_min[1] - origin[1]) * inv_dir[1];
    let ty2 = (box_max[1] - origin[1]) * inv_dir[1];

    tmin = tmin.max(ty1.min(ty2));
    tmax = tmax.min(ty1.max(ty2));

    let tz1 = (box_min[2] - origin[2]) * inv_dir[2];
    let tz2 = (box_max[2] - origin[2]) * inv_dir[2];

    tmin = tmin.max(tz1.min(tz2));
    tmax = tmax.min(tz1.max(tz2));

    tmin = tmin.max(t0);
    tmax = tmax.min(t1);

    (tmax >= tmin, tmin, tmax)
}

/// Line-box entry and exit points.
pub fn ray_box(line: &Line, box_: &OBB, t0: f64, t1: f64) -> Option<Vec<Point>> {
    let origin = line.start();
    let direction = line.to_vector();

    let (hit, tmin, tmax) = ray_box_parameters(&origin, &direction, box_, t0, t1);

    if !hit {
        return None;
    }

    let entry = &origin + &direction * tmin;
    let exit = &origin + &direction * tmax;

    Some(vec![entry, exit])
}

/// Ray-sphere parameters, returning the hit count.
pub fn ray_sphere_parameters(
    origin: &Point,
    direction: &Vector,
    center: &Point,
    radius: f64,
) -> (i32, f64, f64) {
    let offset = origin - center;
    let a = direction.dot(direction);
    let b = 2.0 * direction.dot(&offset);
    let c = offset.dot(&offset) - (radius * radius);
    let disc = b * b - 4.0 * a * c;

    if disc < 0.0 {
        return (0, 0.0, 0.0);
    }

    let root = disc.sqrt();
    let q = if b < 0.0 {
        (-b - root) / 2.0
    } else {
        (-b + root) / 2.0
    };

    let mut t0 = q / a;
    let mut t1 = c / q;

    if t1 == t0 {
        return (1, t0, t1);
    }

    if t0 > t1 {
        std::mem::swap(&mut t0, &mut t1);
    }

    (2, t0, t1)
}

/// Line-sphere hit points.
pub fn ray_sphere(line: &Line, center: &Point, radius: f64) -> Option<Vec<Point>> {
    let origin = line.start();
    let direction = line.to_vector();

    let (hits, t0, t1) = ray_sphere_parameters(&origin, &direction, center, radius);

    if hits == 0 {
        return None;
    }

    let mut points = vec![&origin + &direction * t0];

    if hits == 2 {
        points.push(&origin + &direction * t1);
    }

    Some(points)
}

/// Moller-Trumbore ray-triangle test returning (hit, t, u, v, parallel).
pub fn ray_triangle_parameters(
    origin: &Point,
    direction: &Vector,
    v0: &Point,
    v1: &Point,
    v2: &Point,
    epsilon: f64,
) -> (bool, f64, f64, f64, bool) {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let pvec = direction.cross(&edge2);

    let det = edge1.dot(&pvec);

    if det > -epsilon && det < epsilon {
        return (false, 0.0, 0.0, 0.0, true);
    }

    let inv_det = 1.0 / det;

    let tvec = origin - v0;
    let u = tvec.dot(&pvec) * inv_det;

    if u < 0.0 - epsilon || u > 1.0 + epsilon {
        return (false, 0.0, u, 0.0, false);
    }

    let qvec = tvec.cross(&edge1);
    let v = direction.dot(&qvec) * inv_det;

    if v < 0.0 - epsilon || u + v > 1.0 + epsilon {
        return (false, 0.0, u, v, false);
    }

    let t = edge2.dot(&qvec) * inv_det;

    (true, t, u, v, false)
}

/// Line-triangle hit point.
pub fn ray_triangle(
    line: &Line,
    v0: &Point,
    v1: &Point,
    v2: &Point,
    epsilon: f64,
) -> Option<Point> {
    let origin = line.start();
    let direction = line.to_vector();

    let (hit, t, _, _, _) = ray_triangle_parameters(&origin, &direction, v0, v1, v2, epsilon);

    if !hit {
        return None;
    }

    Some(&origin + &direction * t)
}

/// Whether hit a sorts before hit b: smaller t, ties within 1e-6 broken by the lower face index.
fn ray_hit_before(a: &RayHit, b: &RayHit) -> bool {
    let eps = 1e-6;
    let dt = a.t - b.t;

    if dt.abs() <= eps {
        return a.face_index < b.face_index;
    }

    a.t < b.t
}

/// Three-way comparison built on ray_hit_before.
fn ray_hit_order(a: &RayHit, b: &RayHit) -> std::cmp::Ordering {
    if ray_hit_before(a, b) {
        return std::cmp::Ordering::Less;
    }

    if ray_hit_before(b, a) {
        return std::cmp::Ordering::Greater;
    }

    std::cmp::Ordering::Equal
}

/// Sorts hits by t and keeps only the nearest unless find_all; false when there is none.
fn sort_ray_hits(hits: &mut Vec<RayHit>, find_all: bool) -> bool {
    if hits.is_empty() {
        return false;
    }

    hits.sort_by(ray_hit_order);

    if !find_all {
        hits.truncate(1);
    }

    true
}

/// Ray-mesh hits by brute force sorted by t, only the nearest unless find_all.
pub fn ray_mesh_hits(
    origin: &Point,
    direction: &Vector,
    mesh: &Mesh,
    find_all: bool,
    epsilon: f64,
) -> (bool, Vec<RayHit>) {
    let mut hits: Vec<RayHit> = Vec::new();

    let (vertices, faces) = mesh.to_vertices_and_faces();

    for (i, face) in faces.iter().enumerate() {
        if face.len() < 3 {
            continue;
        }

        for j in 1..face.len() - 1 {
            let v0 = &vertices[face[0]];
            let v1 = &vertices[face[j]];
            let v2 = &vertices[face[j + 1]];

            let (hit, t, u, v, _) = ray_triangle_parameters(origin, direction, v0, v1, v2, epsilon);

            if !hit || t < 0.0 {
                continue;
            }

            hits.push(RayHit::new(t, origin + &(direction * t), u, v, i as i32));
        }
    }

    (sort_ray_hits(&mut hits, find_all), hits)
}

/// Ray-mesh hits through the mesh's triangle BVH sorted by t, only the nearest unless find_all.
pub fn ray_mesh_bvh_hits(
    origin: &Point,
    direction: &Vector,
    mesh: &mut Mesh,
    find_all: bool,
    epsilon: f64,
) -> (bool, Vec<RayHit>) {
    let mut hits: Vec<RayHit> = Vec::new();

    let mut candidates: Vec<usize> = Vec::new();

    if !mesh.triangle_bvh_ray_cast(origin, direction, &mut candidates, find_all) {
        return (false, hits);
    }

    for tri_id in candidates {
        let Some((face_idx, _, v0, v1, v2)) = mesh.get_triangle_by_id(tri_id) else {
            continue;
        };

        let (hit, t, u, v, _) = ray_triangle_parameters(origin, direction, &v0, &v1, &v2, epsilon);

        if !hit || t < 0.0 {
            continue;
        }

        hits.push(RayHit::new(
            t,
            origin + &(direction * t),
            u,
            v,
            face_idx as i32,
        ));
    }

    (sort_ray_hits(&mut hits, find_all), hits)
}

/// Line-mesh hit points by brute force sorted by t, only the nearest unless find_all.
pub fn ray_mesh(line: &Line, mesh: &Mesh, epsilon: f64, find_all: bool) -> Option<Vec<Point>> {
    let (found, hits) = ray_mesh_hits(&line.start(), &line.to_vector(), mesh, find_all, epsilon);

    if !found {
        return None;
    }

    let mut result = Vec::new();

    for hit in hits {
        result.push(hit.point);
    }

    Some(result)
}

/// Line-mesh hit points through the mesh's triangle BVH sorted by t, only the nearest unless find_all.
pub fn ray_mesh_bvh(line: &Line, mesh: &Mesh, epsilon: f64, find_all: bool) -> Option<Vec<Point>> {
    let mut cached = mesh.clone();
    let (found, hits) = ray_mesh_bvh_hits(
        &line.start(),
        &line.to_vector(),
        &mut cached,
        find_all,
        epsilon,
    );

    if !found {
        return None;
    }

    let mut result = Vec::new();

    for hit in hits {
        result.push(hit.point);
    }

    Some(result)
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS curve helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Sorted values without neighbours closer than tolerance to the last kept one.
fn unique_sorted(values: &[f64], tolerance: f64) -> Vec<f64> {
    let mut unique: Vec<f64> = Vec::new();

    for &value in values {
        if unique.is_empty() || (unique[unique.len() - 1] - value).abs() >= tolerance {
            unique.push(value);
        }
    }

    unique
}

/// Signed distance of a point to the plane.
fn curve_signed_distance_to_plane(pt: &Point, plane: &Plane) -> f64 {
    let v = pt - &plane.origin();

    v.dot(&plane.z_axis())
}

/// Rate of change of the signed plane distance with the curve parameter.
fn curve_plane_slope(curve: &NurbsCurve, plane: &Plane, t: f64) -> f64 {
    let derivs = curve.evaluate(t, 1);

    if derivs.len() < 2 {
        return 0.0;
    }

    derivs[1].dot(&plane.z_axis())
}

/// Appends t unless a value within tolerance is already present.
fn append_unique(values: &mut Vec<f64>, t: f64, tolerance: f64) {
    for existing in values.iter() {
        if (existing - t).abs() < tolerance {
            return;
        }
    }

    values.push(t);
}

/// Newton for the plane crossing in [a, b] from the midpoint, bisecting whenever a step is flat or leaves the bracket.
fn curve_plane_newton_bracket(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    mut a: f64,
    mut b: f64,
) -> Option<f64> {
    let mut f_a = curve_signed_distance_to_plane(&curve.point_at(a), plane);
    let mut t = (a + b) * 0.5;

    for _ in 0..10 {
        let f = curve_signed_distance_to_plane(&curve.point_at(t), plane);

        if f.abs() < tolerance {
            return Some(t);
        }

        let df = curve_plane_slope(curve, plane, t);
        let flat = df.abs() < 1e-14;
        let t_new = if flat { t } else { t - f / df };

        if flat || t_new < a || t_new > b {
            if f * f_a < 0.0 {
                b = t;
            } else {
                a = t;
                f_a = f;
            }

            t = (a + b) * 0.5;
            continue;
        }

        if (t_new - t).abs() < tolerance {
            return Some(t_new);
        }

        t = t_new;
    }

    None
}

/// Bisect the plane crossing between t0 and t1 down to tolerance.
fn curve_find_root_bisection(
    curve: &NurbsCurve,
    plane: &Plane,
    mut t0: f64,
    mut t1: f64,
    tolerance: f64,
) -> Option<f64> {
    let max_iterations = 50;
    let mut d0 = curve_signed_distance_to_plane(&curve.point_at(t0), plane);
    let mut _d1 = curve_signed_distance_to_plane(&curve.point_at(t1), plane);

    if d0 * _d1 > 0.0 {
        return None;
    }

    for _ in 0..max_iterations {
        let t_mid = (t0 + t1) * 0.5;
        let d_mid = curve_signed_distance_to_plane(&curve.point_at(t_mid), plane);

        if d_mid.abs() < tolerance || (t1 - t0) < tolerance {
            return Some(t_mid);
        }

        if d0 * d_mid < 0.0 {
            t1 = t_mid;
            _d1 = d_mid;
        } else {
            t0 = t_mid;
            d0 = d_mid;
        }
    }

    let t_result = (t0 + t1) * 0.5;

    if curve_signed_distance_to_plane(&curve.point_at(t_result), plane).abs() < tolerance * 10.0 {
        Some(t_result)
    } else {
        None
    }
}

/// Polish a plane crossing parameter with Newton steps.
fn curve_refine_intersection_newton(
    curve: &NurbsCurve,
    plane: &Plane,
    t: &mut f64,
    tolerance: f64,
) -> bool {
    let max_iterations = 10;
    let step_tolerance = tolerance * 0.01;

    for _ in 0..max_iterations {
        let pt = curve.point_at(*t);

        let f = curve_signed_distance_to_plane(&pt, plane);
        let df = curve_plane_slope(curve, plane, *t);

        if f.abs() < tolerance {
            return true;
        }

        if df.abs() < 1e-12 {
            return false;
        }

        let dt = -f / df;

        if dt.abs() < step_tolerance {
            return true;
        }

        *t += dt;

        let (t0, t1) = curve.domain();

        if *t < t0 {
            *t = t0;
        }

        if *t > t1 {
            *t = t1;
        }
    }

    curve_signed_distance_to_plane(&curve.point_at(*t), plane).abs() < tolerance * 2.0
}

/// Bezier-clipping recursion of the curve-plane distance on [ta, tb].
fn curve_plane_clip(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    ta: f64,
    tb: f64,
    depth: i32,
    results: &mut Vec<f64>,
) {
    if depth > 50 {
        let tm = (ta + tb) * 0.5;
        let pm = curve.point_at(tm);
        let dist = curve_signed_distance_to_plane(&pm, plane);

        if dist.abs() < tolerance {
            results.push(tm);
        }

        return;
    }

    if (tb - ta).abs() < tolerance * 0.01 {
        let tm = (ta + tb) * 0.5;
        let pm = curve.point_at(tm);
        let dist = curve_signed_distance_to_plane(&pm, plane);

        if dist.abs() < tolerance {
            let mut t = tm;

            for _ in 0..10 {
                let pt = curve.point_at(t);
                let f = curve_signed_distance_to_plane(&pt, plane);
                let df = curve_plane_slope(curve, plane, t);

                if df.abs() < 1e-12 {
                    break;
                }

                let dt = -f / df;
                t += dt;

                if dt.abs() < tolerance * 0.01 {
                    break;
                }

                if t < ta || t > tb {
                    t = tm;
                    break;
                }
            }

            let pt_final = curve.point_at(t);

            if curve_signed_distance_to_plane(&pt_final, plane).abs() < tolerance
                && t >= ta
                && t <= tb
            {
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

    let mut d_min = distances[0];
    let mut d_max = distances[0];

    for d in &distances {
        if *d < d_min {
            d_min = *d;
        }

        if *d > d_max {
            d_max = *d;
        }
    }

    if d_min > tolerance || d_max < -tolerance {
        return;
    }

    let mut t_min = ta;
    let mut t_max = tb;

    for i in 0..distances.len() - 1 {
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

    t_min = ta.max(t_min);
    t_max = tb.min(t_max);
    let reduction = (t_max - t_min) / (tb - ta);

    if reduction > 0.8 || (t_max - t_min) < tolerance * 0.1 {
        let tm = (ta + tb) * 0.5;
        curve_plane_clip(curve, plane, tolerance, ta, tm, depth + 1, results);
        curve_plane_clip(curve, plane, tolerance, tm, tb, depth + 1, results);
    } else {
        curve_plane_clip(curve, plane, tolerance, t_min, t_max, depth + 1, results);
    }
}

/// Hodograph subdivision of one span with Newton polishing of the crossings.
fn curve_plane_subdivide_algebraic(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    a: f64,
    b: f64,
    depth: i32,
    results: &mut Vec<f64>,
) {
    if depth > 30 {
        return;
    }

    let p_a = curve.point_at(a);
    let p_b = curve.point_at(b);
    let normal = plane.z_axis();
    let f_a = normal.dot(&(&p_a - &plane.origin()));
    let f_b = normal.dot(&(&p_b - &plane.origin()));

    if f_a * f_b > 0.0 {
        return;
    }

    let mid_t = (a + b) * 0.5;
    let p_mid = curve.point_at(mid_t);
    let mut line_dir = &p_b - &p_a;
    let line_len = line_dir.magnitude();

    if line_len > 1e-14 {
        line_dir = &line_dir / line_len;
    }

    let deviation = (&p_mid - &p_a).cross(&line_dir).magnitude().abs();

    if deviation < tolerance * 10.0 || (b - a) < tolerance * 10.0 {
        if let Some(t) = curve_plane_newton_bracket(curve, plane, tolerance, a, b) {
            if t >= a && t <= b {
                append_unique(results, t, tolerance * 10.0);
            }
        }
    } else {
        curve_plane_subdivide_algebraic(curve, plane, tolerance, a, mid_t, depth + 1, results);
        curve_plane_subdivide_algebraic(curve, plane, tolerance, mid_t, b, depth + 1, results);
    }
}

/// True when the chord of [a, b] deviates less than ten tolerances from the curve.
fn curve_nearly_linear(curve: &NurbsCurve, tolerance: f64, a: f64, b: f64) -> bool {
    let p_a = curve.point_at(a);
    let p_b = curve.point_at(b);
    let p_mid = curve.point_at((a + b) * 0.5);
    let ab = &p_b - &p_a;
    let line_length = ab.magnitude();

    if line_length < 1e-14 {
        return true;
    }

    let am = &p_mid - &p_a;
    let cross_mag = ab.cross(&am).magnitude();
    let deviation = cross_mag / line_length;

    deviation < tolerance * 10.0
}

/// Span subdivision to nearly linear pieces with Newton polishing of the crossings.
fn curve_plane_subdivide_production(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    a: f64,
    b: f64,
    depth: i32,
    results: &mut Vec<f64>,
) {
    if depth > 30 {
        return;
    }

    let p_a = curve.point_at(a);
    let p_b = curve.point_at(b);
    let normal = plane.z_axis();
    let f_a = normal.dot(&(&p_a - &plane.origin()));
    let f_b = normal.dot(&(&p_b - &plane.origin()));

    if f_a * f_b > 0.0 {
        return;
    }

    if curve_nearly_linear(curve, tolerance, a, b) || (b - a) < tolerance * 10.0 {
        if let Some(t) = curve_plane_newton_bracket(curve, plane, tolerance, a, b) {
            if t >= a && t <= b {
                append_unique(results, t, tolerance * 10.0);
            }
        }
    } else {
        let mid = (a + b) * 0.5;
        curve_plane_subdivide_production(curve, plane, tolerance, a, mid, depth + 1, results);
        curve_plane_subdivide_production(curve, plane, tolerance, mid, b, depth + 1, results);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS curves
// ═══════════════════════════════════════════════════════════════════════════

/// Appends t unless it lies within tolerance of the last parameter.
fn append_parameter(params: &mut Vec<f64>, t: f64, tolerance: f64) {
    if params.is_empty() || (params[params.len() - 1] - t).abs() >= tolerance {
        params.push(t);
    }
}

/// Crossing pairs hidden inside a span whose ends lie on one side, found on degree * 2 sub-intervals.
fn curve_plane_hidden_pairs(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    t0: f64,
    t1: f64,
    intersections: &mut Vec<f64>,
) {
    let count = (curve.degree() * 2) as i32;
    let dt = (t1 - t0) / count as f64;

    for i in 0..count {
        let s0 = t0 + i as f64 * dt;
        let s1 = t0 + (i + 1) as f64 * dt;
        let d0 = curve_signed_distance_to_plane(&curve.point_at(s0), plane);
        let d1 = curve_signed_distance_to_plane(&curve.point_at(s1), plane);

        if d0 * d1 < 0.0 {
            if let Some(mut t_intersection) =
                curve_find_root_bisection(curve, plane, s0, s1, tolerance)
            {
                curve_refine_intersection_newton(curve, plane, &mut t_intersection, tolerance);
                intersections.push(t_intersection);
            }
        }
    }
}

/// Crossings inside each knot span, plus span starts and the curve end lying on the plane.
fn curve_plane_spans(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    intersections: &mut Vec<f64>,
) {
    let span_params = curve.get_span_vector();

    for i in 0..(span_params.len() - 1) {
        let t0 = span_params[i];
        let t1 = span_params[i + 1];

        if (t1 - t0).abs() < tolerance {
            continue;
        }

        let d0 = curve_signed_distance_to_plane(&curve.point_at(t0), plane);
        let d1 = curve_signed_distance_to_plane(&curve.point_at(t1), plane);

        if d0 * d1 < 0.0 {
            if let Some(mut t_intersection) =
                curve_find_root_bisection(curve, plane, t0, t1, tolerance)
            {
                curve_refine_intersection_newton(curve, plane, &mut t_intersection, tolerance);
                intersections.push(t_intersection);
            }
        } else if d0.abs() < tolerance {
            append_parameter(intersections, t0, tolerance);
        } else if curve.degree() > 1 {
            curve_plane_hidden_pairs(curve, plane, tolerance, t0, t1, intersections);
        }
    }

    let t_end = curve.domain().1;

    if curve_signed_distance_to_plane(&curve.point_at(t_end), plane).abs() < tolerance {
        append_parameter(intersections, t_end, tolerance);
    }
}

/// Extra crossings of a high-degree curve found on degree * 4 uniform samples.
fn curve_plane_samples(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    intersections: &mut Vec<f64>,
) {
    let (t_start, t_end) = curve.domain();
    let num_samples = (curve.degree() * 4) as i32;
    let dt = (t_end - t_start) / num_samples as f64;

    for i in 0..num_samples {
        let t0 = t_start + i as f64 * dt;
        let t1 = t_start + (i + 1) as f64 * dt;
        let d0 = curve_signed_distance_to_plane(&curve.point_at(t0), plane);
        let d1 = curve_signed_distance_to_plane(&curve.point_at(t1), plane);
        let mut crossing = None;

        if d0 * d1 < 0.0 {
            crossing = curve_find_root_bisection(curve, plane, t0, t1, tolerance);
        }

        let Some(mut t_intersection) = crossing else {
            continue;
        };

        let mut is_new = true;

        for &existing in intersections.iter() {
            if (existing - t_intersection).abs() < tolerance * 2.0 {
                is_new = false;
                break;
            }
        }

        if is_new {
            curve_refine_intersection_newton(curve, plane, &mut t_intersection, tolerance);
            intersections.push(t_intersection);
        }
    }
}

/// Curve-plane intersection parameters by sampling, bisection and Newton refinement.
pub fn curve_plane(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<f64> {
    let mut intersections = Vec::new();

    if !curve.is_valid() {
        return intersections;
    }

    let tol = if tolerance.unwrap_or(0.0) <= 0.0 {
        Tolerance::ZERO_TOLERANCE
    } else {
        tolerance.unwrap()
    };

    curve_plane_spans(curve, plane, tol, &mut intersections);

    if curve.degree() > 3 && intersections.len() < curve.degree() {
        curve_plane_samples(curve, plane, tol, &mut intersections);
    }

    intersections.sort_by(|a, b| a.partial_cmp(b).unwrap());

    unique_sorted(&intersections, tol * 2.0)
}

/// Curve-plane intersection points.
pub fn curve_plane_points(curve: &NurbsCurve, plane: &Plane, tolerance: Option<f64>) -> Vec<Point> {
    curve_plane(curve, plane, tolerance)
        .iter()
        .map(|&t| curve.point_at(t))
        .collect()
}

/// Curve-plane intersection parameters by Bezier clipping.
pub fn curve_plane_bezier_clipping(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: Option<f64>,
) -> Vec<f64> {
    let mut results = Vec::new();

    if !curve.is_valid() {
        return results;
    }

    let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
    let (t0, t1) = curve.domain();
    curve_plane_clip(curve, plane, tolerance, t0, t1, 0, &mut results);

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());

    unique_sorted(&results, tolerance * 10.0)
}

/// Curve-plane intersection parameters by hodograph subdivision.
pub fn curve_plane_algebraic(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: Option<f64>,
) -> Vec<f64> {
    if !curve.is_valid() {
        return Vec::new();
    }

    let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
    let mut results = Vec::new();
    let spans = curve.get_span_vector();

    if spans.len() < 2 {
        return Vec::new();
    }

    for i in 0..spans.len() - 1 {
        let span_t0 = spans[i];
        let span_t1 = spans[i + 1];

        if (span_t1 - span_t0).abs() < tolerance {
            continue;
        }

        curve_plane_subdivide_algebraic(curve, plane, tolerance, span_t0, span_t1, 0, &mut results);
    }

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());

    unique_sorted(&results, tolerance * 10.0)
}

/// Curve-plane intersection parameters by span subdivision and Newton polishing.
pub fn curve_plane_production(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: Option<f64>,
) -> Vec<f64> {
    if !curve.is_valid() {
        return Vec::new();
    }

    let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
    let mut results = Vec::new();
    let spans = curve.get_span_vector();

    if spans.len() < 2 {
        return Vec::new();
    }

    for i in 0..spans.len() - 1 {
        let span_t0 = spans[i];
        let span_t1 = spans[i + 1];

        if (span_t1 - span_t0).abs() < tolerance {
            continue;
        }

        curve_plane_subdivide_production(
            curve,
            plane,
            tolerance,
            span_t0,
            span_t1,
            0,
            &mut results,
        );
    }

    results.sort_by(|a, b| a.partial_cmp(b).unwrap());

    unique_sorted(&results, tolerance * 10.0)
}

/// Closest curve parameter and distance to a point, optionally within [t0, t1].
pub fn curve_closest_point(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64) -> (f64, f64) {
    Closest::curve_point(curve, test_point, t0, t1)
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS surface helpers
// ═══════════════════════════════════════════════════════════════════════════

/// One traced surface-plane curve in parameter space.
struct SurfacePlaneTrace {
    uv_trace: Vec<(f64, f64)>,     // Traced (u, v) samples.
    uv_unwrapped: Vec<(f64, f64)>, // Samples with seam wraps undone.
    is_loop: bool,                 // Whether the trace closes on itself.
}

/// All traces of one surface-plane section with the scales used.
struct SurfacePlaneTraceResult {
    traces: Vec<SurfacePlaneTrace>, // Traced curves.
    step: f64,                      // uv step used.
    uv_to_3d: f64,                  // Largest uv-to-3D scale seen.
    uv_to_3d_min: f64,              // Smallest uv-to-3D scale seen.
}

/// Grid crossing of the surface-plane distance, the start of one trace.
struct SurfacePlaneSeed {
    u: f64,     // Seed u.
    v: f64,     // Seed v.
    used: bool, // Whether a trace already passed the seed.
}

/// Signed surface-plane distance over the surface's UV domain with the tracing scales.
struct SurfacePlaneField<'a> {
    surface: &'a NurbsSurface, // Traced surface.
    pn: Vector,                // Plane normal.
    p0: Point,                 // Plane origin.
    tolerance: f64,            // Newton tolerance.
    u0: f64,                   // Domain start in u.
    u1: f64,                   // Domain end in u.
    v0: f64,                   // Domain start in v.
    v1: f64,                   // Domain end in v.
    range_u: f64,              // Domain length in u.
    range_v: f64,              // Domain length in v.
    closed_u: bool,            // Whether u wraps around a seam.
    closed_v: bool,            // Whether v wraps around a seam.
    nu: i32,                   // Grid cells in u.
    nv: i32,                   // Grid cells in v.
    du: f64,                   // Grid cell size in u.
    dv: f64,                   // Grid cell size in v.
    uv_to_3d: f64,             // Largest uv-to-3D scale.
    uv_to_3d_min: f64,         // Smallest uv-to-3D scale.
    step: f64,                 // Marching step in uv.
    max_steps: i32,            // Marching step cap per direction.
    close_tol_3d: f64,         // 3D distance that closes a loop.
    consume_tol_3d: f64,       // 3D distance that consumes a seed.
    join_tol: f64,             // 3D distance that joins two traces.
}

impl<'a> SurfacePlaneField<'a> {
    /// Sample the domain and derive the tracing scales; None without a domain.
    fn new(surface: &'a NurbsSurface, plane: &Plane, tolerance: f64) -> Option<Self> {
        let (u0, u1) = surface.domain(0)?;
        let (v0, v1) = surface.domain(1)?;
        let spans_u = surface.get_span_vector(0);
        let spans_v = surface.get_span_vector(1);
        let nu = (spans_u.len() as i32 - 1).max(1) * 4;
        let nv = (spans_v.len() as i32 - 1).max(1) * 4;
        let range_u = u1 - u0;
        let range_v = v1 - v0;

        let mut field = SurfacePlaneField {
            surface,
            pn: plane.z_axis(),
            p0: plane.origin(),
            tolerance,
            u0,
            u1,
            v0,
            v1,
            range_u,
            range_v,
            closed_u: surface.is_closed(0),
            closed_v: surface.is_closed(1),
            nu,
            nv,
            du: range_u / nu as f64,
            dv: range_v / nv as f64,
            uv_to_3d: 0.0,
            uv_to_3d_min: 0.0,
            step: 0.0,
            max_steps: 0,
            close_tol_3d: 0.0,
            consume_tol_3d: 0.0,
            join_tol: 0.0,
        };

        let du = field.du;
        let dv = field.dv;
        let mu = (u0 + u1) * 0.5;
        let mv = (v0 + v1) * 0.5;
        let pmid = field.point((mu, mv));
        let uv_to_3d_u = pmid.distance(&field.point((field.wrap_u(mu + du), mv)), None) / du;
        let uv_to_3d_v = pmid.distance(&field.point((mu, field.wrap_v(mv + dv))), None) / dv;
        field.uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);
        field.uv_to_3d_min = uv_to_3d_u.min(uv_to_3d_v);

        if field.uv_to_3d < 1e-10 {
            field.uv_to_3d = 1.0;
        }

        if field.uv_to_3d_min < 1e-10 {
            field.uv_to_3d_min = 1.0;
        }

        field.step = du.min(dv) * 0.25;
        field.max_steps = nu * nv * 32;
        field.close_tol_3d = field.step * 4.0 * field.uv_to_3d_min;
        field.consume_tol_3d = field.step * field.uv_to_3d * 2.0;
        field.join_tol = du.max(dv) * field.uv_to_3d * 1.5;

        Some(field)
    }

    /// Wrap u across a closed seam or clamp it to the domain.
    fn wrap_u(&self, u: f64) -> f64 {
        if self.closed_u {
            let mut t = (u - self.u0) % self.range_u;

            if t < 0.0 {
                t += self.range_u;
            }

            return self.u0 + t;
        }

        self.u0.max(u.min(self.u1))
    }

    /// Wrap v across a closed seam or clamp it to the domain.
    fn wrap_v(&self, v: f64) -> f64 {
        if self.closed_v {
            let mut t = (v - self.v0) % self.range_v;

            if t < 0.0 {
                t += self.range_v;
            }

            return self.v0 + t;
        }

        self.v0.max(v.min(self.v1))
    }

    /// Signed plane distance at (u, v).
    fn value(&self, u: f64, v: f64) -> f64 {
        let p = self.point((self.wrap_u(u), self.wrap_v(v)));
        let p0 = &self.p0;
        let pn = &self.pn;

        (p[0] - p0[0]) * pn[0] + (p[1] - p0[1]) * pn[1] + (p[2] - p0[2]) * pn[2]
    }

    /// Signed plane distance and its uv gradient at (u, v).
    fn value_and_gradient(&self, u: f64, v: f64) -> (f64, f64, f64) {
        let derivs = self.surface.evaluate(self.wrap_u(u), self.wrap_v(v), 1);

        if derivs.len() < 3 {
            return (self.value(u, v), 0.0, 0.0);
        }

        let s = &derivs[0];
        let su = &derivs[2];
        let sv = &derivs[1];
        let p0 = &self.p0;
        let pn = &self.pn;
        let val = (s[0] - p0[0]) * pn[0] + (s[1] - p0[1]) * pn[1] + (s[2] - p0[2]) * pn[2];
        let gu = su[0] * pn[0] + su[1] * pn[1] + su[2] * pn[2];
        let gv = sv[0] * pn[0] + sv[1] * pn[1] + sv[2] * pn[2];

        (val, gu, gv)
    }

    /// Newton-project (u, v) onto the zero set; false when it does not converge.
    fn newton_correct(&self, u: &mut f64, v: &mut f64) -> bool {
        for _ in 0..10 {
            let (val, gu, gv) = self.value_and_gradient(*u, *v);

            if val.abs() < self.tolerance {
                return true;
            }

            let mag2 = gu * gu + gv * gv;

            if mag2 < 1e-28 {
                return false;
            }

            *u -= val * gu / mag2;
            *v -= val * gv / mag2;
            *u = self.wrap_u(*u);
            *v = self.wrap_v(*v);
        }

        self.value(*u, *v).abs() < self.tolerance * 10.0
    }

    /// Unit uv tangent of the zero set at (u, v) in direction dir.
    fn tangent(&self, u: f64, v: f64, dir: i32) -> Option<(f64, f64)> {
        let (_, gu, gv) = self.value_and_gradient(u, v);
        let mag = f64::hypot(gu, gv);

        if mag < 1e-14 {
            return None;
        }

        Some((-gv / mag * dir as f64, gu / mag * dir as f64))
    }

    /// Surface point at a uv sample.
    fn point(&self, q: (f64, f64)) -> Point {
        self.surface
            .point_at(q.0, q.1)
            .unwrap_or(Point::new(0.0, 0.0, 0.0))
    }

    /// Newton-slide (cu, cv) along one seam line, axis 0 moving v and axis 1 moving u.
    fn seam_newton(&self, mut cu: f64, mut cv: f64, axis: i32) -> (f64, f64) {
        for _ in 0..10 {
            let (val, gu, gv) = self.value_and_gradient(cu, cv);

            if val.abs() < self.tolerance {
                break;
            }

            if axis == 0 {
                if gv.abs() < 1e-14 {
                    break;
                }

                cv -= val / gv;
            } else {
                if gu.abs() < 1e-14 {
                    break;
                }

                cu -= val / gu;
            }
        }

        (cu, cv)
    }

    /// Newton-project (u, v) onto the zero set to 1e-12; false on a flat gradient.
    fn polish(&self, u: &mut f64, v: &mut f64) -> bool {
        for _ in 0..8 {
            let (val, gu, gv) = self.value_and_gradient(*u, *v);

            if val.abs() < 1e-12 {
                return true;
            }

            let mag2 = gu * gu + gv * gv;

            if mag2 < 1e-28 {
                return false;
            }

            *u -= val * gu / mag2;
            *v -= val * gv / mag2;
        }

        true
    }
}

/// Signed plane distance on the (nu + 1) x (nv + 1) grid, exact zeros nudged negative.
fn surface_plane_grid(field: &SurfacePlaneField) -> Vec<f64> {
    let cols = field.nv + 1;
    let mut dist = vec![0.0f64; ((field.nu + 1) * cols) as usize];

    for i in 0..=field.nu {
        let u = field.u0 + field.du * i as f64;

        for j in 0..=field.nv {
            let v = field.v0 + field.dv * j as f64;
            let mut d = field.value(u, v);

            if d == 0.0 {
                d = -1e-14;
            }

            dist[(i * cols + j) as usize] = d;
        }
    }

    dist
}

/// Newton-corrected sign changes along the grid edges, near duplicates marked used.
fn surface_plane_seeds(field: &SurfacePlaneField, dist: &[f64]) -> Vec<SurfacePlaneSeed> {
    let mut seeds: Vec<SurfacePlaneSeed> = Vec::new();
    let cols = field.nv + 1;
    let h_jmax = if field.closed_v {
        field.nv - 1
    } else {
        field.nv
    };

    for i in 0..field.nu {
        for j in 0..=h_jmax {
            let d0 = dist[(i * cols + j) as usize];
            let d1 = dist[((i + 1) * cols + j) as usize];

            if d0 * d1 < 0.0 {
                let t = d0 / (d0 - d1);
                let mut su = field.u0 + field.du * (i as f64 + t);
                let mut sv = field.v0 + field.dv * j as f64;

                if field.newton_correct(&mut su, &mut sv) {
                    seeds.push(SurfacePlaneSeed {
                        u: su,
                        v: sv,
                        used: false,
                    });
                }
            }
        }
    }

    let v_imax = if field.closed_u {
        field.nu - 1
    } else {
        field.nu
    };

    for i in 0..=v_imax {
        for j in 0..field.nv {
            let d0 = dist[(i * cols + j) as usize];
            let d1 = dist[(i * cols + j + 1) as usize];

            if d0 * d1 < 0.0 {
                let t = d0 / (d0 - d1);
                let mut su = field.u0 + field.du * i as f64;
                let mut sv = field.v0 + field.dv * (j as f64 + t);

                if field.newton_correct(&mut su, &mut sv) {
                    seeds.push(SurfacePlaneSeed {
                        u: su,
                        v: sv,
                        used: false,
                    });
                }
            }
        }
    }

    let seed_tol_3d = field.du.max(field.dv) * field.uv_to_3d;

    for i in 0..seeds.len() {
        if seeds[i].used {
            continue;
        }

        let pi = field.point((seeds[i].u, seeds[i].v));

        for other in seeds.iter_mut().skip(i + 1) {
            if other.used {
                continue;
            }

            if pi.distance(&field.point((other.u, other.v)), None) < seed_tol_3d {
                other.used = true;
            }
        }
    }

    seeds
}

/// Step (u, v) by local_step along (tu, tv), pulled back onto an open domain boundary; true when clamped.
fn domain_step(
    field: &SurfacePlaneField,
    u: f64,
    v: f64,
    local_step: f64,
    tu: f64,
    tv: f64,
) -> (f64, f64, bool) {
    let un = u + local_step * tu;
    let vn = v + local_step * tv;

    let out_u = !field.closed_u && (un < field.u0 || un > field.u1);
    let out_v = !field.closed_v && (vn < field.v0 || vn > field.v1);

    if !out_u && !out_v {
        return (un, vn, false);
    }

    let mut tc = 1.0f64;

    if !field.closed_u && tu > 0.0 && un > field.u1 {
        tc = tc.min((field.u1 - u) / (local_step * tu));
    }

    if !field.closed_u && tu < 0.0 && un < field.u0 {
        tc = tc.min((field.u0 - u) / (local_step * tu));
    }

    if !field.closed_v && tv > 0.0 && vn > field.v1 {
        tc = tc.min((field.v1 - v) / (local_step * tv));
    }

    if !field.closed_v && tv < 0.0 && vn < field.v0 {
        tc = tc.min((field.v0 - v) / (local_step * tv));
    }

    (u + tc * local_step * tu, v + tc * local_step * tv, true)
}

/// Retry a failed Newton projection with the step halved up to four times.
fn newton_retry(
    field: &SurfacePlaneField,
    u: f64,
    v: f64,
    local_step: f64,
    tu: f64,
    tv: f64,
) -> Option<(f64, f64)> {
    let mut ls = local_step;

    for _ in 0..4 {
        ls *= 0.5;
        let mut un = field.wrap_u(u + ls * tu);
        let mut vn = field.wrap_v(v + ls * tv);

        if field.newton_correct(&mut un, &mut vn) {
            return Some((un, vn));
        }
    }

    None
}

/// Mark every unused seed within the consume distance of p as used.
fn consume_seeds(field: &SurfacePlaneField, p: &Point, seeds: &mut [SurfacePlaneSeed]) {
    for other in seeds.iter_mut() {
        if !other.used && p.distance(&field.point((other.u, other.v)), None) < field.consume_tol_3d
        {
            other.used = true;
        }
    }
}

/// Step length for the turn between two unit tangents: a quarter or half step on sharp turns.
fn turn_step(field: &SurfacePlaneField, tu: f64, tv: f64, prev_tu: f64, prev_tv: f64) -> f64 {
    if f64::hypot(prev_tu, prev_tv) <= 1e-14 {
        return field.step;
    }

    let dot = (-1.0f64).max(1.0f64.min(tu * prev_tu + tv * prev_tv));

    if dot < 0.95 {
        return field.step * 0.25;
    }

    if dot < 0.985 {
        return field.step * 0.5;
    }

    field.step
}

/// March the zero set from (su, sv) in direction dir; true when it closes on its start.
fn surface_plane_march(
    field: &SurfacePlaneField,
    su: f64,
    sv: f64,
    dir: i32,
    seeds: &mut [SurfacePlaneSeed],
    out: &mut Vec<(f64, f64)>,
) -> bool {
    let mut u = su;
    let mut v = sv;
    let mut prev_tu = 0.0f64;
    let mut prev_tv = 0.0f64;
    let p_start = field.point((su, sv));
    let mut p_prev = p_start.clone();
    let mut dist_traveled = 0.0f64;

    for _ in 0..field.max_steps {
        let (mut tu, mut tv) = match field.tangent(u, v, dir) {
            Some(t) => t,
            None => {
                if f64::hypot(prev_tu, prev_tv) < 1e-14 {
                    break;
                }

                (prev_tu, prev_tv)
            }
        };

        let local_step = turn_step(field, tu, tv, prev_tu, prev_tv);

        if let Some((tu2, tv2)) =
            field.tangent(u + local_step * 0.5 * tu, v + local_step * 0.5 * tv, dir)
        {
            tu = tu2;
            tv = tv2;
        }

        prev_tu = tu;
        prev_tv = tv;

        let (un_raw, vn_raw, hit_boundary) = domain_step(field, u, v, local_step, tu, tv);
        let mut un = field.wrap_u(un_raw);
        let mut vn = field.wrap_v(vn_raw);

        if !field.newton_correct(&mut un, &mut vn) {
            let Some((ur, vr)) = newton_retry(field, u, v, local_step, tu, tv) else {
                break;
            };

            un = ur;
            vn = vr;
        }

        let p_cur = field.point((un, vn));
        dist_traveled += p_prev.distance(&p_cur, None);
        out.push((un, vn));

        if dist_traveled > field.close_tol_3d * 3.0
            && p_start.distance(&p_cur, None) < field.close_tol_3d
        {
            return true;
        }

        u = un;
        v = vn;
        p_prev = p_cur.clone();

        if hit_boundary {
            break;
        }

        consume_seeds(field, &p_cur, seeds);
    }

    false
}

/// Undo the seam jumps of a closed domain in the unwrapped copy of a trace.
fn unwrap_trace(field: &SurfacePlaneField, uv: &mut [(f64, f64)]) {
    for i in 1..uv.len() {
        let du_jump = uv[i].0 - uv[i - 1].0;
        let dv_jump = uv[i].1 - uv[i - 1].1;

        if field.closed_u {
            if du_jump > field.range_u * 0.5 {
                uv[i].0 -= field.range_u;
            } else if du_jump < -field.range_u * 0.5 {
                uv[i].0 += field.range_u;
            }
        }

        if field.closed_v {
            if dv_jump > field.range_v * 0.5 {
                uv[i].1 -= field.range_v;
            } else if dv_jump < -field.range_v * 0.5 {
                uv[i].1 += field.range_v;
            }
        }
    }
}

/// Trace one seed both ways into a trace; None when it is too short to keep.
fn surface_plane_trace_seed(
    field: &SurfacePlaneField,
    seeds: &mut [SurfacePlaneSeed],
    index: usize,
) -> Option<SurfacePlaneTrace> {
    let seed_u = seeds[index].u;
    let seed_v = seeds[index].v;
    let mut fwd: Vec<(f64, f64)> = Vec::new();
    let mut bwd: Vec<(f64, f64)> = Vec::new();
    let fwd_closed = surface_plane_march(field, seed_u, seed_v, 1, seeds, &mut fwd);

    if !fwd_closed {
        surface_plane_march(field, seed_u, seed_v, -1, seeds, &mut bwd);
    }

    let mut uv_trace: Vec<(f64, f64)> = Vec::with_capacity(bwd.len() + 1 + fwd.len());

    for p in bwd.iter().rev() {
        uv_trace.push(*p);
    }

    uv_trace.push((seed_u, seed_v));

    for p in &fwd {
        uv_trace.push(*p);
    }

    if uv_trace.len() < 4 {
        return None;
    }

    let p_first = field.point(uv_trace[0]);
    let p_last = field.point(uv_trace[uv_trace.len() - 1]);
    let is_loop =
        fwd_closed || (uv_trace.len() >= 6 && p_first.distance(&p_last, None) < field.close_tol_3d);

    if is_loop {
        uv_trace.pop();
    }

    if uv_trace.len() < 4 {
        return None;
    }

    let mut uv_unwrapped = uv_trace.clone();
    unwrap_trace(field, &mut uv_unwrapped);

    Some(SurfacePlaneTrace {
        uv_trace,
        uv_unwrapped,
        is_loop,
    })
}

/// Whether every eighth sample of trace a lies within the join distance of trace b.
fn trace_covered_by(
    field: &SurfacePlaneField,
    a: &SurfacePlaneTrace,
    b: &SurfacePlaneTrace,
) -> bool {
    let stride = 1usize.max(a.uv_trace.len() / 8);
    let mut k = 0;

    while k < a.uv_trace.len() {
        let q = field.point(a.uv_trace[k]);
        let mut best = 1e300f64;

        for r in &b.uv_trace {
            best = best.min(q.distance(&field.point(*r), None));
        }

        if best > field.join_tol {
            return false;
        }

        k += stride;
    }

    true
}

/// Empty every open trace that a trace at least as long already covers.
fn drop_covered_traces(field: &SurfacePlaneField, traces: &mut [SurfacePlaneTrace]) {
    for i in 0..traces.len() {
        if traces[i].uv_trace.is_empty() || traces[i].is_loop {
            continue;
        }

        for j in 0..traces.len() {
            if i == j || traces[j].uv_trace.is_empty() {
                continue;
            }

            if traces[j].uv_trace.len() < traces[i].uv_trace.len() {
                continue;
            }

            if trace_covered_by(field, &traces[i], &traces[j]) {
                traces[i].uv_trace.clear();
                break;
            }
        }
    }
}

/// Append trace j to the end of trace i, reversed when requested, and close i when it meets itself.
fn append_trace(
    field: &SurfacePlaneField,
    traces: &mut [SurfacePlaneTrace],
    i: usize,
    j: usize,
    reversed: bool,
) {
    let mut add = traces[j].uv_trace.clone();

    if reversed {
        add.reverse();
    }

    traces[i].uv_trace.extend_from_slice(&add);
    traces[j].uv_trace.clear();

    let a = &mut traces[i];
    let first = field.point(a.uv_trace[0]);
    let last = field.point(a.uv_trace[a.uv_trace.len() - 1]);

    if first.distance(&last, None) < field.join_tol {
        a.is_loop = true;
        a.uv_trace.pop();
    }

    a.uv_unwrapped = a.uv_trace.clone();
    unwrap_trace(field, &mut a.uv_unwrapped);
}

/// Join the first open trace pair whose end meets a start or end; false when none does.
fn join_one_trace_pair(field: &SurfacePlaneField, traces: &mut [SurfacePlaneTrace]) -> bool {
    for i in 0..traces.len() {
        if traces[i].uv_trace.len() < 2 || traces[i].is_loop {
            continue;
        }

        let ie = field.point(traces[i].uv_trace[traces[i].uv_trace.len() - 1]);

        for j in 0..traces.len() {
            if i == j || traces[j].uv_trace.len() < 2 || traces[j].is_loop {
                continue;
            }

            let ja = field.point(traces[j].uv_trace[0]);
            let jb = field.point(traces[j].uv_trace[traces[j].uv_trace.len() - 1]);
            let fwd2 = ie.distance(&ja, None) < field.join_tol;
            let rev2 = ie.distance(&jb, None) < field.join_tol;

            if !fwd2 && !rev2 {
                continue;
            }

            append_trace(field, traces, i, j, rev2);

            return true;
        }
    }

    false
}

/// Drop short traces and close the open ones whose ends meet.
fn close_traces(field: &SurfacePlaneField, traces: &mut Vec<SurfacePlaneTrace>) {
    let mut kept: Vec<SurfacePlaneTrace> = Vec::new();

    for t in traces.drain(..) {
        if t.uv_trace.len() >= 4 {
            kept.push(t);
        }
    }

    *traces = kept;

    for t in traces.iter_mut() {
        if t.is_loop || t.uv_trace.len() < 6 {
            continue;
        }

        let first = field.point(t.uv_trace[0]);
        let last = field.point(t.uv_trace[t.uv_trace.len() - 1]);

        if first.distance(&last, None) < field.join_tol {
            t.is_loop = true;
            t.uv_trace.pop();
            t.uv_unwrapped.pop();
        }
    }
}

/// Snap one open trace end within a grid cell of the domain boundary onto it.
fn snap_trace_end(field: &SurfacePlaneField, q: &mut (f64, f64), qu: &mut (f64, f64)) {
    if !field.closed_u {
        if (q.0 - field.u0).abs() < field.du {
            q.0 = field.u0;
            qu.0 = field.u0;
        }

        if (q.0 - field.u1).abs() < field.du {
            q.0 = field.u1;
            qu.0 = field.u1;
        }
    } else if q.0 - field.u0 < field.du {
        q.0 = field.u0;
    } else if field.u1 - q.0 < field.du {
        q.0 = field.u1;
    }

    if !field.closed_v {
        if (q.1 - field.v0).abs() < field.dv {
            q.1 = field.v0;
            qu.1 = field.v0;
        }

        if (q.1 - field.v1).abs() < field.dv {
            q.1 = field.v1;
            qu.1 = field.v1;
        }
    } else if q.1 - field.v0 < field.dv {
        q.1 = field.v0;
    } else if field.v1 - q.1 < field.dv {
        q.1 = field.v1;
    }
}

/// Seed and trace surface/plane intersection curves in UV space.
fn surface_plane_traces(
    surface: &NurbsSurface,
    plane: &Plane,
    tolerance: f64,
) -> SurfacePlaneTraceResult {
    let Some(field) = SurfacePlaneField::new(surface, plane, tolerance) else {
        return SurfacePlaneTraceResult {
            traces: Vec::new(),
            step: 0.0,
            uv_to_3d: 1.0,
            uv_to_3d_min: 1.0,
        };
    };

    let dist = surface_plane_grid(&field);
    let mut gmax = 0.0f64;

    for d in &dist {
        gmax = gmax.max(d.abs());
    }

    if gmax < tolerance.max(1e-9) * 10.0 {
        return SurfacePlaneTraceResult {
            traces: Vec::new(),
            step: field.step,
            uv_to_3d: field.uv_to_3d,
            uv_to_3d_min: field.uv_to_3d_min,
        };
    }

    let mut seeds = surface_plane_seeds(&field, &dist);
    let mut traces: Vec<SurfacePlaneTrace> = Vec::new();

    for i in 0..seeds.len() {
        if seeds[i].used {
            continue;
        }

        seeds[i].used = true;

        if let Some(trace) = surface_plane_trace_seed(&field, &mut seeds, i) {
            traces.push(trace);
        }
    }

    drop_covered_traces(&field, &mut traces);

    for _ in 0..traces.len() {
        if !join_one_trace_pair(&field, &mut traces) {
            break;
        }
    }

    close_traces(&field, &mut traces);

    for t in traces.iter_mut() {
        if t.is_loop || t.uv_trace.is_empty() {
            continue;
        }

        let last = t.uv_trace.len() - 1;
        let last_unwrapped = t.uv_unwrapped.len() - 1;
        snap_trace_end(&field, &mut t.uv_trace[0], &mut t.uv_unwrapped[0]);
        snap_trace_end(
            &field,
            &mut t.uv_trace[last],
            &mut t.uv_unwrapped[last_unwrapped],
        );
    }

    SurfacePlaneTraceResult {
        traces,
        step: field.step,
        uv_to_3d: field.uv_to_3d,
        uv_to_3d_min: field.uv_to_3d_min,
    }
}

/// Points projected into the plane's 2D frame, z = 0.
fn plane_points_2d(pts: &[Point], plane: &Plane) -> Vec<Point> {
    let ax = plane.x_axis();
    let ay = plane.y_axis();
    let po = plane.origin();
    let mut pts_2d: Vec<Point> = Vec::with_capacity(pts.len());

    for p in pts {
        let dx = p[0] - po[0];
        let dy = p[1] - po[1];
        let dz = p[2] - po[2];
        let px = dx * ax[0] + dy * ax[1] + dz * ax[2];
        let py = dx * ay[0] + dy * ay[1] + dz * ay[2];
        pts_2d.push(Point::new(px, py, 0.0));
    }

    pts_2d
}

/// Normalized cumulative chord length of each point, the closing chord included for loops.
fn chord_parameters(pts: &[Point], is_loop: bool) -> Vec<f64> {
    let m = pts.len();
    let mut chords = vec![0.0f64; m];
    let mut total_len = 0.0f64;

    for i in 1..m {
        total_len += pts[i].distance(&pts[i - 1], None);
        chords[i] = total_len;
    }

    if is_loop && m > 1 {
        total_len += pts[0].distance(&pts[m - 1], None);
    }

    if total_len > 1e-14 {
        for chord in chords.iter_mut().skip(1) {
            *chord /= total_len;
        }
    }

    chords
}

/// Sum of the turning angles along a planar polyline.
fn total_turning(pts: &[Point]) -> f64 {
    let mut turning = 0.0f64;

    for i in 1..pts.len().saturating_sub(1) {
        let dx1 = pts[i][0] - pts[i - 1][0];
        let dy1 = pts[i][1] - pts[i - 1][1];
        let dx2 = pts[i + 1][0] - pts[i][0];
        let dy2 = pts[i + 1][1] - pts[i][1];
        let l1 = f64::hypot(dx1, dy1);
        let l2 = f64::hypot(dx2, dy2);

        if l1 > 1e-14 && l2 > 1e-14 {
            let c = (-1.0f64).max(1.0f64.min((dx1 * dx2 + dy1 * dy2) / (l1 * l2)));
            turning += c.acos();
        }
    }

    turning
}

/// Largest distance from each point to the curve, found by ternary search around its chord parameter.
fn fitted_max_deviation(cand: &NurbsCurve, pts: &[Point], chords: &[f64]) -> f64 {
    let m = pts.len();
    let (ft0, ft1) = cand.domain();
    let mut max_dev = 0.0f64;

    for (p, chord) in pts.iter().zip(chords) {
        let t = ft0 + (ft1 - ft0) * chord;
        let w2 = (ft1 - ft0) * 2.0 / (m as i32 - 1).max(1) as f64;
        let mut lo = ft0.max(t - w2);
        let mut hi = ft1.min(t + w2);

        for _ in 0..20 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;

            if cand.point_at(m1).distance(p, None) < cand.point_at(m2).distance(p, None) {
                hi = m2;
            } else {
                lo = m1;
            }
        }

        max_dev = max_dev.max(cand.point_at(0.5 * (lo + hi)).distance(p, None));
    }

    max_dev
}

/// Cubic fitted to the points in the plane's frame, CVs doubled until within fit_tol, lifted back to 3D.
fn fit_planar_freeform(
    all_pts: &[Point],
    is_loop: bool,
    plane: &Plane,
    fit_tol: f64,
) -> NurbsCurve {
    let m = all_pts.len() as i32;

    if m < 4 {
        return NurbsCurve::new(3, false, 4, 0);
    }

    let pts_2d = plane_points_2d(all_pts, plane);
    let chords = chord_parameters(&pts_2d, is_loop);
    let mut target_cvs = 8_i32.max((total_turning(&pts_2d) / 0.5) as i32 + 6);
    let max_cvs = (m - 1).min(128);
    let mut crv_2d = NurbsCurve::new(3, false, 4, 0);
    let mut best_dev = 1e300f64;

    for _ in 0..6 {
        if target_cvs > max_cvs {
            break;
        }

        let cand = NurbsCurve::create_fitted(&pts_2d, target_cvs as usize, 3, is_loop);

        if !cand.is_valid() {
            break;
        }

        let max_dev = fitted_max_deviation(&cand, &pts_2d, &chords);

        if max_dev < best_dev {
            best_dev = max_dev;
            crv_2d = cand;
        }

        if max_dev < fit_tol {
            break;
        }

        target_cvs = (target_cvs * 2).min(max_cvs + 1);
    }

    if !crv_2d.is_valid() {
        crv_2d = if is_loop {
            NurbsCurve::create_interpolated(
                &pts_2d,
                CurveNurbsKnotStyle::ChordPeriodic,
                CurveInterpStyle::Rhino,
            )
        } else {
            NurbsCurve::create_interpolated(
                &pts_2d,
                CurveNurbsKnotStyle::Chord,
                CurveInterpStyle::Rhino,
            )
        };
    }

    if !crv_2d.is_valid() {
        return NurbsCurve::new(3, false, 4, 0);
    }

    let ax = plane.x_axis();
    let ay = plane.y_axis();
    let po = plane.origin();

    for i in 0..crv_2d.cv_count() {
        if let Some(cv2) = crv_2d.get_cv(i) {
            let cx = cv2[0];
            let cy = cv2[1];
            crv_2d.set_cv(
                i,
                &Point::new(
                    po[0] + cx * ax[0] + cy * ay[0],
                    po[1] + cx * ax[1] + cy * ay[1],
                    po[2] + cx * ax[2] + cy * ay[2],
                ),
            );
        }
    }

    crv_2d
}

/// Fits a 3D plane-constrained NurbsCurve to traced intersection points.
fn surface_plane_fit_3d(
    all_pts: &[Point],
    is_loop: bool,
    plane: &Plane,
    step: f64,
    uv_to_3d: f64,
    uv_to_3d_min: f64,
    allow_conics: bool,
) -> NurbsCurve {
    let mut crv = NurbsCurve::new(3, false, 4, 0);

    if allow_conics && is_loop && all_pts.len() >= 6 {
        let ax = plane.x_axis();
        let ay = plane.y_axis();
        let po = plane.origin();
        let to2d = |p: &Point| -> (f64, f64) {
            let dx = p[0] - po[0];
            let dy = p[1] - po[1];
            let dz = p[2] - po[2];
            (
                dx * ax[0] + dy * ax[1] + dz * ax[2],
                dx * ay[0] + dy * ay[1] + dz * ay[2],
            )
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

            for p in all_pts {
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
                let nurbsknots: [f64; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

                for (i, knot) in nurbsknots.iter().enumerate() {
                    crv.set_nurbsknot(i, *knot);
                }

                for i in 0..9 {
                    let px = cx3d + radius * (cx_[i] * ax[0] + cy_[i] * ay[0]);
                    let py = cy3d + radius * (cx_[i] * ax[1] + cy_[i] * ay[1]);
                    let pz = cz3d + radius * (cx_[i] * ax[2] + cy_[i] * ay[2]);
                    crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                }
            }
        }
    }

    if !crv.is_valid() && allow_conics && is_loop && all_pts.len() >= 8 {
        let ax = plane.x_axis();
        let ay = plane.y_axis();
        let po = plane.origin();
        let to2d = |p: &Point| -> (f64, f64) {
            let dx = p[0] - po[0];
            let dy = p[1] - po[1];
            let dz = p[2] - po[2];
            (
                dx * ax[0] + dy * ax[1] + dz * ax[2],
                dx * ay[0] + dy * ay[1] + dz * ay[2],
            )
        };

        let n = all_pts.len();
        let mut ata = [[0.0f64; 5]; 5];
        let mut atb = [0.0f64; 5];

        for p in all_pts.iter().take(n) {
            let (x, y) = to2d(p);
            let row = [x * x, x * y, y * y, x, y];

            for r in 0..5 {
                atb[r] += row[r];

                for c in 0..5 {
                    ata[r][c] += row[r] * row[c];
                }
            }
        }

        let mut m_mat = [[0.0f64; 6]; 5];

        for r in 0..5 {
            for c in 0..5 {
                m_mat[r][c] = ata[r][c];
            }

            m_mat[r][5] = atb[r];
        }

        let mut ok = true;

        for col in 0..5 {
            if !ok {
                break;
            }

            let mut pivot = col;

            for r in (col + 1)..5 {
                if m_mat[r][col].abs() > m_mat[pivot][col].abs() {
                    pivot = r;
                }
            }

            if m_mat[pivot][col].abs() < 1e-20 {
                ok = false;
                break;
            }

            if pivot != col {
                m_mat.swap(col, pivot);
            }

            for r in (col + 1)..5 {
                let f = m_mat[r][col] / m_mat[col][col];
                let pivot_row = m_mat[col];

                for j in col..=5 {
                    m_mat[r][j] -= f * pivot_row[j];
                }
            }
        }

        let mut coef = [0.0f64; 5];

        if ok {
            for i in (0..5).rev() {
                let mut s = m_mat[i][5];

                for j in (i + 1)..5 {
                    s -= m_mat[i][j] * coef[j];
                }

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

            for p in all_pts {
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
                    let nurbsknots: [f64; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

                    for (i, knot) in nurbsknots.iter().enumerate() {
                        crv.set_nurbsknot(i, *knot);
                    }

                    for i in 0..9 {
                        let px = cx3d + semi_a * cx_[i] * ea[0] + semi_b * cy_[i] * eb[0];
                        let py = cy3d + semi_a * cx_[i] * ea[1] + semi_b * cy_[i] * eb[1];
                        let pz = cz3d + semi_a * cx_[i] * ea[2] + semi_b * cy_[i] * eb[2];
                        crv.set_cv_4d(i, px * wts[i], py * wts[i], pz * wts[i], wts[i]);
                    }

                    let mut max_ell_dev = 0.0f64;

                    for p in all_pts {
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
                        crv = NurbsCurve::new(3, false, 4, 0);
                    }
                }
            }
        }
    }

    if !crv.is_valid() {
        crv = fit_planar_freeform(
            all_pts,
            is_loop,
            plane,
            step * (uv_to_3d + uv_to_3d_min) * 0.5 * 5e-4,
        );
    }

    crv
}

/// Seam-free run of uv samples cut from one trace.
struct SurfacePlanePiece {
    uv: Vec<(f64, f64)>, // Samples in parameter space.
    is_loop: bool,       // Whether the piece still closes on itself.
}

/// Whether the quarter, half and three-quarter samples of a trace all lie within dup_tol of one kept trace.
fn is_duplicate_trace(trace_pts3: &[Point], kept_pts3: &[Vec<Point>], dup_tol: f64) -> bool {
    let m = trace_pts3.len();

    for other in kept_pts3 {
        let mut all_close = true;

        for f in [0.25, 0.5, 0.75] {
            let cp = &trace_pts3[((m - 1) as f64 * f) as usize];
            let mut dmin = dup_tol + 1.0;

            for q in other.iter().step_by(5) {
                dmin = dmin.min(cp.distance(q, None));
            }

            if dmin > dup_tol {
                all_close = false;
                break;
            }
        }

        if all_close {
            return true;
        }
    }

    false
}

/// Append the loop start shifted by whole periods after the end; returns the shift (closure_du, closure_dv).
fn close_unwrapped_loop(field: &SurfacePlaneField, pts: &mut Vec<(f64, f64)>) -> (f64, f64) {
    let last = pts[pts.len() - 1];
    let mut du_j = pts[0].0 - last.0;
    let mut dv_j = pts[0].1 - last.1;

    if field.closed_u {
        while du_j > field.range_u * 0.5 {
            du_j -= field.range_u;
        }

        while du_j < -field.range_u * 0.5 {
            du_j += field.range_u;
        }
    }

    if field.closed_v {
        while dv_j > field.range_v * 0.5 {
            dv_j -= field.range_v;
        }

        while dv_j < -field.range_v * 0.5 {
            dv_j += field.range_v;
        }
    }

    let closure_du = (last.0 + du_j) - pts[0].0;
    let closure_dv = (last.1 + dv_j) - pts[0].1;
    pts.push((pts[0].0 + closure_du, pts[0].1 + closure_dv));

    (closure_du, closure_dv)
}

/// Seam crossings (t, axis, seam value) of the segment pa-pb, sorted by t.
fn seam_crossings(
    field: &SurfacePlaneField,
    pa: (f64, f64),
    pb: (f64, f64),
) -> Vec<(f64, i32, f64)> {
    let mut crossings: Vec<(f64, i32, f64)> = Vec::new();

    if field.closed_u && (pb.0 - pa.0).abs() > 1e-15 {
        let k0 = ((pa.0 - field.u0) / field.range_u).floor() as i32;
        let k1 = ((pb.0 - field.u0) / field.range_u).floor() as i32;

        for k in (k0.min(k1) + 1)..=k0.max(k1) {
            let l = field.u0 + k as f64 * field.range_u;
            let t = (l - pa.0) / (pb.0 - pa.0);

            if 0.0 < t && t < 1.0 {
                crossings.push((t, 0, l));
            }
        }
    }

    if field.closed_v && (pb.1 - pa.1).abs() > 1e-15 {
        let k0 = ((pa.1 - field.v0) / field.range_v).floor() as i32;
        let k1 = ((pb.1 - field.v0) / field.range_v).floor() as i32;

        for k in (k0.min(k1) + 1)..=k0.max(k1) {
            let l = field.v0 + k as f64 * field.range_v;
            let t = (l - pa.1) / (pb.1 - pa.1);

            if 0.0 < t && t < 1.0 {
                crossings.push((t, 1, l));
            }
        }
    }

    crossings.sort_by(|a, b| a.partial_cmp(b).unwrap());

    crossings
}

/// Snap q onto a seam it lies on within 1e-9 of the period after a real move from pa; true when snapped.
fn snap_to_seam(field: &SurfacePlaneField, pa: (f64, f64), q: &mut (f64, f64)) -> bool {
    let mut on_seam = false;

    if field.closed_u {
        let k = ((q.0 - field.u0) / field.range_u).round();
        let l = field.u0 + k * field.range_u;

        if (q.0 - l).abs() < field.range_u * 1e-9 && (q.0 - pa.0).abs() > field.range_u * 1e-9 {
            q.0 = l;
            on_seam = true;
        }
    }

    if field.closed_v {
        let k = ((q.1 - field.v0) / field.range_v).round();
        let l = field.v0 + k * field.range_v;

        if (q.1 - l).abs() < field.range_v * 1e-9 && (q.1 - pa.1).abs() > field.range_v * 1e-9 {
            q.1 = l;
            on_seam = true;
        }
    }

    on_seam
}

/// Samples with the seam crossings inserted, and the indices of the samples on a seam.
fn split_at_seams(field: &SurfacePlaneField, pts: &[(f64, f64)]) -> (Vec<(f64, f64)>, Vec<usize>) {
    let mut cross_idx: Vec<usize> = Vec::new();
    let mut out_pts: Vec<(f64, f64)> = vec![pts[0]];

    for i in 1..pts.len() {
        let pa = pts[i - 1];
        let pb = pts[i];

        for (t, axis, l) in seam_crossings(field, pa, pb) {
            let mut cu = pa.0 + (pb.0 - pa.0) * t;
            let mut cv_ = pa.1 + (pb.1 - pa.1) * t;

            if axis == 0 {
                cv_ = field.seam_newton(l, cv_, 0).1;
                cu = l;
            } else {
                cu = field.seam_newton(cu, l, 1).0;
                cv_ = l;
            }

            out_pts.push((cu, cv_));
            cross_idx.push(out_pts.len() - 1);
        }

        let mut q = (pb.0, pb.1);
        let on_seam = i < pts.len() - 1 && snap_to_seam(field, pa, &mut q);
        out_pts.push(q);

        if on_seam {
            cross_idx.push(out_pts.len() - 1);
        }
    }

    (out_pts, cross_idx)
}

/// Cut the samples at the seam indices; a loop's last piece wraps around to its first seam.
fn seam_pieces(
    out_pts: &[(f64, f64)],
    cross_idx: &[usize],
    is_loop: bool,
    wrap_drift: bool,
    closure_du: f64,
    closure_dv: f64,
) -> Vec<SurfacePlanePiece> {
    let mut pieces: Vec<SurfacePlanePiece> = Vec::new();

    if cross_idx.is_empty() {
        pieces.push(SurfacePlanePiece {
            uv: out_pts.to_vec(),
            is_loop: is_loop && !wrap_drift,
        });

        return pieces;
    }

    if is_loop {
        for ci in 0..cross_idx.len() - 1 {
            pieces.push(SurfacePlanePiece {
                uv: out_pts[cross_idx[ci]..=cross_idx[ci + 1]].to_vec(),
                is_loop: false,
            });
        }

        let mut wrap_piece: Vec<(f64, f64)> = out_pts[cross_idx[cross_idx.len() - 1]..].to_vec();

        for p in &out_pts[1..=cross_idx[0]] {
            wrap_piece.push((p.0 + closure_du, p.1 + closure_dv));
        }

        pieces.push(SurfacePlanePiece {
            uv: wrap_piece,
            is_loop: false,
        });

        return pieces;
    }

    let mut bounds: Vec<usize> = vec![0];

    for &ci in cross_idx {
        bounds.push(ci);
    }

    bounds.push(out_pts.len() - 1);

    for bi in 0..bounds.len() - 1 {
        if bounds[bi + 1] > bounds[bi] {
            pieces.push(SurfacePlanePiece {
                uv: out_pts[bounds[bi]..=bounds[bi + 1]].to_vec(),
                is_loop: false,
            });
        }
    }

    pieces
}

/// Seam-free uv pieces of one trace.
fn trace_pieces(field: &SurfacePlaneField, trace: &SurfacePlaneTrace) -> Vec<SurfacePlanePiece> {
    let mut pts = trace.uv_unwrapped.clone();
    let mut closure = (0.0, 0.0);

    if trace.is_loop && pts.len() >= 2 {
        closure = close_unwrapped_loop(field, &mut pts);
    }

    let (out_pts, cross_idx) = split_at_seams(field, &pts);
    let wrap_drift = closure.0.abs() > field.range_u * 0.5 || closure.1.abs() > field.range_v * 0.5;

    seam_pieces(
        &out_pts,
        &cross_idx,
        trace.is_loop,
        wrap_drift,
        closure.0,
        closure.1,
    )
}

/// Shift a piece by whole periods so its middle sample lies in the base domain.
fn shift_piece_to_domain(field: &SurfacePlaneField, piece_pts: &mut [(f64, f64)]) {
    let mid = piece_pts[piece_pts.len() / 2];

    if field.closed_u {
        let k_u = ((mid.0 - field.u0) / field.range_u).floor() as i32;

        if k_u != 0 {
            for p in piece_pts.iter_mut() {
                p.0 -= k_u as f64 * field.range_u;
            }
        }
    }

    if field.closed_v {
        let k_v = ((mid.1 - field.v0) / field.range_v).floor() as i32;

        if k_v != 0 {
            for p in piece_pts.iter_mut() {
                p.1 -= k_v as f64 * field.range_v;
            }
        }
    }
}

/// Insert zero-set samples between a and b while the chord midpoint sags more than step * 1e-4, four levels deep.
fn densify_segment(
    field: &SurfacePlaneField,
    au: f64,
    av: f64,
    bu: f64,
    bv: f64,
    depth: i32,
    pts_uv: &mut Vec<Point>,
) {
    let mu = 0.5 * (au + bu);
    let mv = 0.5 * (av + bv);
    let mut cu = mu;
    let mut cv2 = mv;

    if !field.polish(&mut cu, &mut cv2) {
        return;
    }

    let sag = f64::hypot(cu - mu, cv2 - mv);

    if sag > field.step * 1e-4 && depth < 4 {
        densify_segment(field, au, av, cu, cv2, depth + 1, pts_uv);
        pts_uv.push(Point::new(cu, cv2, 0.0));
        densify_segment(field, cu, cv2, bu, bv, depth + 1, pts_uv);
    } else {
        pts_uv.push(Point::new(cu, cv2, 0.0));
    }
}

/// Piece samples with zero-set samples inserted where a segment sags.
fn densify_piece(field: &SurfacePlaneField, piece_pts: &[(f64, f64)]) -> Vec<Point> {
    let mut pts_uv: Vec<Point> = Vec::with_capacity(piece_pts.len() * 4);

    for i in 1..piece_pts.len() {
        let a = piece_pts[i - 1];
        let b = piece_pts[i];
        pts_uv.push(Point::new(a.0, a.1, 0.0));
        densify_segment(field, a.0, a.1, b.0, b.1, 0, &mut pts_uv);
    }

    let last = piece_pts[piece_pts.len() - 1];
    pts_uv.push(Point::new(last.0, last.1, 0.0));

    pts_uv
}

/// Largest distance from each point to the curve at its chord parameter.
fn chord_max_deviation(cand: &NurbsCurve, pts: &[Point], chords: &[f64]) -> f64 {
    let (ft0, ft1) = cand.domain();
    let mut max_dev = 0.0f64;

    for (p, chord) in pts.iter().zip(chords) {
        let t = ft0 + (ft1 - ft0) * chord;
        max_dev = max_dev.max(cand.point_at(t).distance(p, None));
    }

    max_dev
}

/// Cubic pcurve through the uv samples, CVs doubled until within step * 2e-3, with the last CV count tried.
fn fit_pcurve(pts_uv: &[Point], piece_loop: bool, step: f64) -> (NurbsCurve, i32) {
    let mp = pts_uv.len() as i32;
    let chords = chord_parameters(pts_uv, piece_loop);
    let max_cvs = (mp - 1).min(96);
    let mut pcurve = NurbsCurve::new(3, false, 4, 0);
    let mut pcurve_dev = 1e300f64;
    let mut target_cvs = 8_i32.max((total_turning(pts_uv) / 0.5) as i32 + 6);

    for _ in 0..6 {
        if target_cvs > max_cvs {
            break;
        }

        let cand = NurbsCurve::create_fitted(pts_uv, target_cvs as usize, 3, piece_loop);

        if !cand.is_valid() {
            break;
        }

        let max_dev = chord_max_deviation(&cand, pts_uv, &chords);

        if max_dev < pcurve_dev {
            pcurve_dev = max_dev;
            pcurve = cand;
        }

        if max_dev < step * 2e-3 {
            break;
        }

        target_cvs = (target_cvs * 2).min(max_cvs + 1);
    }

    if !pcurve.is_valid() {
        pcurve = if piece_loop {
            NurbsCurve::create_interpolated(
                pts_uv,
                CurveNurbsKnotStyle::ChordPeriodic,
                CurveInterpStyle::Rhino,
            )
        } else {
            NurbsCurve::create_interpolated(
                pts_uv,
                CurveNurbsKnotStyle::Chord,
                CurveInterpStyle::Rhino,
            )
        };
    }

    (pcurve, target_cvs)
}

/// Refit the pcurve with twice the CVs when it strays from the zero set by more than vali_tol.
fn refit_pcurve(
    field: &SurfacePlaneField,
    pts_uv: &[Point],
    piece_loop: bool,
    target_cvs: i32,
    vali_tol: f64,
    pcurve: &mut NurbsCurve,
) {
    let max_cvs = (pts_uv.len() as i32 - 1).min(96);
    let mut max_off = 0.0f64;

    for i in 0..17 {
        let pc = pcurve.point_at(i as f64 / 16.0);
        let (val, _, _) = field.value_and_gradient(pc[0], pc[1]);
        max_off = max_off.max(val.abs());
    }

    if max_off > vali_tol && target_cvs * 2 <= max_cvs {
        let mut refit = NurbsCurve::create_fitted(pts_uv, (target_cvs * 2) as usize, 3, piece_loop);

        if refit.is_valid() {
            refit.set_domain(0.0, 1.0);
            *pcurve = refit;
        }
    }
}

/// 3D section curve and uv pcurve of one seam-free piece; None when a fit fails.
fn piece_curves(
    field: &SurfacePlaneField,
    plane: &Plane,
    piece: &mut SurfacePlanePiece,
) -> Option<(NurbsCurve, NurbsCurve)> {
    shift_piece_to_domain(field, &mut piece.uv);

    let pts_uv = densify_piece(field, &piece.uv);
    let mut pts3: Vec<Point> = Vec::with_capacity(pts_uv.len());

    for p in &pts_uv {
        pts3.push(field.point((field.wrap_u(p[0]), field.wrap_v(p[1]))));
    }

    let mut crv3 = surface_plane_fit_3d(
        &pts3,
        piece.is_loop,
        plane,
        field.step,
        field.uv_to_3d,
        field.uv_to_3d_min,
        false,
    );

    if !crv3.is_valid() {
        crv3 = if piece.is_loop {
            NurbsCurve::create_interpolated(
                &pts3,
                CurveNurbsKnotStyle::ChordPeriodic,
                CurveInterpStyle::Rhino,
            )
        } else {
            NurbsCurve::create_interpolated(
                &pts3,
                CurveNurbsKnotStyle::Chord,
                CurveInterpStyle::Rhino,
            )
        };
    }

    if !crv3.is_valid() {
        return None;
    }

    let (mut pcurve, target_cvs) = fit_pcurve(&pts_uv, piece.is_loop, field.step);

    if !pcurve.is_valid() {
        return None;
    }

    crv3.set_domain(0.0, 1.0);
    pcurve.set_domain(0.0, 1.0);

    let fit_tol = field.step * (field.uv_to_3d + field.uv_to_3d_min) * 0.5;
    let vali_tol = (10.0 * field.tolerance).max(fit_tol * 2.0);
    refit_pcurve(
        field,
        &pts_uv,
        piece.is_loop,
        target_cvs,
        vali_tol,
        &mut pcurve,
    );

    Some((crv3, pcurve))
}

/// Solve an n x n linear system by Gaussian elimination with partial pivoting.
fn solve_gauss(m: &[Vec<f64>], rhs: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut a: Vec<Vec<f64>> = (0..n)
        .map(|r| {
            let mut row = m[r].clone();
            row.push(rhs[r]);
            row
        })
        .collect();

    for col in 0..n {
        let mut pivot = col;

        for r in (col + 1)..n {
            if a[r][col].abs() > a[pivot][col].abs() {
                pivot = r;
            }
        }

        if a[pivot][col].abs() < 1e-20 {
            return None;
        }

        if pivot != col {
            a.swap(col, pivot);
        }

        for r in (col + 1)..n {
            let f = a[r][col] / a[col][col];

            let (upper, lower) = a.split_at_mut(r);
            let pivot_row = &upper[col];

            for j in col..=n {
                lower[0][j] -= f * pivot_row[j];
            }
        }
    }

    let mut x = vec![0.0f64; n];

    for i in (0..n).rev() {
        let mut s = a[i][n];

        for j in (i + 1)..n {
            s -= a[i][j] * x[j];
        }

        x[i] = s / a[i][i];
    }

    Some(x)
}

// ═══════════════════════════════════════════════════════════════════════════
// Analytic quadric surface intersection
// ═══════════════════════════════════════════════════════════════════════════

#[inline]
fn vdot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn vcross(u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
}

#[inline]
fn vunit(v: [f64; 3]) -> [f64; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

    if l > 1e-300 {
        [v[0] / l, v[1] / l, v[2] / l]
    } else {
        v
    }
}

/// Two unit vectors spanning the plane perpendicular to unit n.
fn ortho_basis(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let ax = if n[0].abs() <= n[1].abs() && n[0].abs() <= n[2].abs() {
        1.0
    } else {
        0.0
    };
    let ay = if ax == 0.0 && n[1].abs() <= n[2].abs() {
        1.0
    } else {
        0.0
    };
    let az = if ax == 0.0 && ay == 0.0 { 1.0 } else { 0.0 };
    let ux = ay * n[2] - az * n[1];
    let uy = az * n[0] - ax * n[2];
    let uz = ax * n[1] - ay * n[0];
    let ul = (ux * ux + uy * uy + uz * uz).sqrt();
    let (ux, uy, uz) = (ux / ul, uy / ul, uz / ul);
    let vx = n[1] * uz - n[2] * uy;
    let vy = n[2] * ux - n[0] * uz;
    let vz = n[0] * uy - n[1] * ux;

    ([ux, uy, uz], [vx, vy, vz])
}

/// Exact 9-CV rational NURBS circle.
fn exact_circle(cx: f64, cy: f64, cz: f64, xa: [f64; 3], ya: [f64; 3], radius: f64) -> NurbsCurve {
    let w = (2.0_f64).sqrt() / 2.0;
    let px = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
    let py = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
    let wts = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let mut crv = NurbsCurve::new(3, true, 3, 9);
    let knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

    for (i, knot) in knots.iter().enumerate() {
        crv.set_nurbsknot(i, *knot);
    }

    for i in 0..9 {
        let x = cx + radius * (px[i] * xa[0] + py[i] * ya[0]);
        let y = cy + radius * (px[i] * xa[1] + py[i] * ya[1]);
        let z = cz + radius * (px[i] * xa[2] + py[i] * ya[2]);
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    crv.set_domain(0.0, 1.0);

    crv
}

/// Exact 9-CV rational NURBS ellipse.
fn exact_ellipse(
    cx: f64,
    cy: f64,
    cz: f64,
    ea: [f64; 3],
    eb: [f64; 3],
    semi_a: f64,
    semi_b: f64,
) -> NurbsCurve {
    let w = (2.0_f64).sqrt() / 2.0;
    let px = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
    let py = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
    let wts = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let mut crv = NurbsCurve::new(3, true, 3, 9);
    let knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

    for (i, knot) in knots.iter().enumerate() {
        crv.set_nurbsknot(i, *knot);
    }

    for i in 0..9 {
        let x = cx + semi_a * px[i] * ea[0] + semi_b * py[i] * eb[0];
        let y = cy + semi_a * px[i] * ea[1] + semi_b * py[i] * eb[1];
        let z = cz + semi_a * px[i] * ea[2] + semi_b * py[i] * eb[2];
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    crv.set_domain(0.0, 1.0);

    crv
}

/// Eigenvalues/vectors of a symmetric 3x3 matrix (cyclic Jacobi).
fn jacobi_eig3(m: &[[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut a = *m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();

        if off < 1e-18 {
            break;
        }

        for &(p, q) in &[(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }

            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = (if theta >= 0.0 { 1.0 } else { -1.0 })
                / (theta.abs() + (theta * theta + 1.0).sqrt());

            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;

            for row in a.iter_mut() {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }

            let row_p = a[p];
            let row_q = a[q];

            for k in 0..3 {
                a[p][k] = c * row_p[k] - s * row_q[k];
                a[q][k] = s * row_p[k] + c * row_q[k];
            }

            for row in v.iter_mut() {
                let (vkp, vkq) = (row[p], row[q]);
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }

    let eigvals = [a[0][0], a[1][1], a[2][2]];
    let eigvecs = [
        [v[0][0], v[1][0], v[2][0]],
        [v[0][1], v[1][1], v[2][1]],
        [v[0][2], v[1][2], v[2][2]],
    ];

    (eigvals, eigvecs)
}

/// Recognized analytic surface type.
enum RecSurf {
    Plane([f64; 3], [f64; 3]),
    Sphere([f64; 3], f64),
    Cylinder([f64; 3], [f64; 3], f64),
    Cone([f64; 3], [f64; 3], f64),
    Torus([f64; 3], [f64; 3], f64, f64),
}

/// Recognize a cylinder from surface samples: axis point, axis direction and radius.
fn fit_cylinder(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], [f64; 3], f64)> {
    let (u0, u1) = surface.domain(0)?;
    let (v0, v1) = surface.domain(1)?;
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut nrm: Vec<[f64; 3]> = Vec::new();

    for i in 0..5 {
        for j in 0..5 {
            let uu = u0 + (u1 - u0) * i as f64 / 4.0;
            let vv = v0 + (v1 - v0) * j as f64 / 4.0;
            let p = surface.point_at(uu, vv)?;
            pts.push([p[0], p[1], p[2]]);
            let n = surface.normal_at(uu, vv);
            nrm.push([n[0], n[1], n[2]]);
        }
    }

    let mut m = [[0.0; 3]; 3];

    for n in &nrm {
        for r in 0..3 {
            for c in 0..3 {
                m[r][c] += n[r] * n[c];
            }
        }
    }

    let (evals, evecs) = jacobi_eig3(&m);
    let mut kmin = 0usize;

    for k in 1..3 {
        if evals[k] < evals[kmin] {
            kmin = k;
        }
    }

    let w = evecs[kmin];
    let wl = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();

    if wl < 1e-12 {
        return None;
    }

    let w = [w[0] / wl, w[1] / wl, w[2] / wl];
    let (ea, eb) = ortho_basis(w);
    let p0 = pts[0];
    let mut ata = vec![vec![0.0; 3]; 3];
    let mut atb = vec![0.0; 3];
    let mut proj: Vec<(f64, f64)> = Vec::new();

    for p in &pts {
        let dp = [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]];
        let x = dp[0] * ea[0] + dp[1] * ea[1] + dp[2] * ea[2];
        let y = dp[0] * eb[0] + dp[1] * eb[1] + dp[2] * eb[2];
        proj.push((x, y));
        let row = [x, y, 1.0];
        let rhs = -(x * x + y * y);

        for r in 0..3 {
            atb[r] += row[r] * rhs;

            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    let sol = solve_gauss(&ata, &atb, 3)?;
    let ccx = -sol[0] / 2.0;
    let ccy = -sol[1] / 2.0;
    let r2 = ccx * ccx + ccy * ccy - sol[2];

    if r2 <= 1e-18 {
        return None;
    }

    let r = r2.sqrt();

    for &(x, y) in &proj {
        if (((x - ccx).powi(2) + (y - ccy).powi(2)).sqrt() - r).abs() > tol {
            return None;
        }
    }

    let axis_pt = [
        p0[0] + ccx * ea[0] + ccy * eb[0],
        p0[1] + ccx * ea[1] + ccy * eb[1],
        p0[2] + ccx * ea[2] + ccy * eb[2],
    ];

    Some((axis_pt, w, r))
}

/// Recognize a cone from surface samples: apex, axis and half angle.
fn fit_cone(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], [f64; 3], f64)> {
    let (u0, u1) = surface.domain(0)?;
    let (v0, v1) = surface.domain(1)?;
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut nrm: Vec<([f64; 3], [f64; 3])> = Vec::new();
    let nu_s = 8;

    for i in 0..nu_s {
        let uu = u0 + (u1 - u0) * i as f64 / nu_s as f64;

        for j in 0..5 {
            let vv = v0 + (v1 - v0) * j as f64 / 4.0;
            let p = surface.point_at(uu, vv)?;
            pts.push([p[0], p[1], p[2]]);
            let n = surface.normal_at(uu, vv);
            let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();

            if nl < 1e-12 {
                continue;
            }

            let pp = surface.point_at(uu, vv)?;
            nrm.push(([n[0] / nl, n[1] / nl, n[2] / nl], [pp[0], pp[1], pp[2]]));
        }
    }

    if nrm.len() < 4 {
        return None;
    }

    let mut ata = vec![vec![0.0; 3]; 3];
    let mut atb = vec![0.0; 3];

    for (n, p) in &nrm {
        let npd = n[0] * p[0] + n[1] * p[1] + n[2] * p[2];

        for r in 0..3 {
            atb[r] += n[r] * npd;

            for c in 0..3 {
                ata[r][c] += n[r] * n[c];
            }
        }
    }

    let vv = solve_gauss(&ata, &atb, 3)?;
    let mut gs: Vec<[f64; 3]> = Vec::new();

    for p in &pts {
        let d = [p[0] - vv[0], p[1] - vv[1], p[2] - vv[2]];
        let dl = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();

        if dl < tol {
            continue;
        }

        gs.push([d[0] / dl, d[1] / dl, d[2] / dl]);
    }

    if gs.len() < 3 {
        return None;
    }

    let mut g = [[0.0; 3]; 3];

    for gg in &gs {
        for r in 0..3 {
            for c in 0..3 {
                g[r][c] += gg[r] * gg[c];
            }
        }
    }

    let (gevals, gevecs) = jacobi_eig3(&g);
    let mut kmax = 0usize;

    for k in 1..3 {
        if gevals[k] > gevals[kmax] {
            kmax = k;
        }
    }

    let mut w = gevecs[kmax];
    let sx = [
        gs.iter().map(|g| g[0]).sum::<f64>(),
        gs.iter().map(|g| g[1]).sum::<f64>(),
        gs.iter().map(|g| g[2]).sum::<f64>(),
    ];

    if w[0] * sx[0] + w[1] * sx[1] + w[2] * sx[2] < 0.0 {
        w = [-w[0], -w[1], -w[2]];
    }

    let wl = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();

    if wl < 1e-12 {
        return None;
    }

    let w = [w[0] / wl, w[1] / wl, w[2] / wl];
    let mut angs: Vec<f64> = Vec::new();

    for gg in &gs {
        angs.push(
            (gg[0] * w[0] + gg[1] * w[1] + gg[2] * w[2])
                .clamp(-1.0, 1.0)
                .acos(),
        );
    }

    let alpha = angs.iter().sum::<f64>() / angs.len() as f64;

    if !(1e-4..=std::f64::consts::PI / 2.0 - 1e-4).contains(&alpha) {
        return None;
    }

    let ca = alpha.cos();

    for p in &pts {
        let d = [p[0] - vv[0], p[1] - vv[1], p[2] - vv[2]];
        let axd = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
        let perp = (0.0_f64)
            .max((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) - axd * axd)
            .sqrt();

        if (perp - axd * alpha.tan()).abs() * ca > tol {
            return None;
        }
    }

    Some(([vv[0], vv[1], vv[2]], w, alpha))
}

/// Recognize a sphere from surface samples: center and radius.
fn fit_sphere(surface: &NurbsSurface, tol: f64) -> Option<(f64, f64, f64, f64)> {
    let (u0, u1) = surface.domain(0)?;
    let (v0, v1) = surface.domain(1)?;
    let mut pts: Vec<[f64; 3]> = Vec::new();

    for i in 0..5 {
        for j in 0..5 {
            let uu = u0 + (u1 - u0) * i as f64 / 4.0;
            let vv = v0 + (v1 - v0) * j as f64 / 4.0;
            let p = surface.point_at(uu, vv)?;
            pts.push([p[0], p[1], p[2]]);
        }
    }

    let mut ata = vec![vec![0.0; 4]; 4];
    let mut atb = vec![0.0; 4];

    for p in &pts {
        let row = [p[0], p[1], p[2], 1.0];
        let rhs = -(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);

        for r in 0..4 {
            atb[r] += row[r] * rhs;

            for c in 0..4 {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    let sol = solve_gauss(&ata, &atb, 4)?;
    let cx = -sol[0] / 2.0;
    let cy = -sol[1] / 2.0;
    let cz = -sol[2] / 2.0;
    let r2 = cx * cx + cy * cy + cz * cz - sol[3];

    if r2 <= 0.0 {
        return None;
    }

    let r = r2.sqrt();

    for p in &pts {
        let d = ((p[0] - cx).powi(2) + (p[1] - cy).powi(2) + (p[2] - cz).powi(2)).sqrt();

        if (d - r).abs() > tol {
            return None;
        }
    }

    Some((cx, cy, cz, r))
}

/// Recognize a torus from the smallest-variance axis and a tube cross-section circle fit.
fn fit_torus(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], [f64; 3], f64, f64)> {
    let (u0, u1) = surface.domain(0)?;
    let (v0, v1) = surface.domain(1)?;
    let mut pts: Vec<[f64; 3]> = Vec::new();

    for i in 0..8 {
        for j in 0..8 {
            let p = surface.point_at(
                u0 + (u1 - u0) * i as f64 / 8.0,
                v0 + (v1 - v0) * j as f64 / 8.0,
            )?;
            pts.push([p[0], p[1], p[2]]);
        }
    }

    let n = pts.len() as f64;
    let cen = [
        pts.iter().map(|p| p[0]).sum::<f64>() / n,
        pts.iter().map(|p| p[1]).sum::<f64>() / n,
        pts.iter().map(|p| p[2]).sum::<f64>() / n,
    ];
    let mut mm = [[0.0; 3]; 3];

    for p in &pts {
        let d = [p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]];

        for r in 0..3 {
            for c in 0..3 {
                mm[r][c] += d[r] * d[c];
            }
        }
    }

    let (evals, evecs) = jacobi_eig3(&mm);
    let mut kmin = 0usize;

    for k in 1..3 {
        if evals[k] < evals[kmin] {
            kmin = k;
        }
    }

    let w = evecs[kmin];
    let wl = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();

    if wl < 1e-12 {
        return None;
    }

    let w = [w[0] / wl, w[1] / wl, w[2] / wl];
    let mut ata = vec![vec![0.0; 3]; 3];
    let mut atb = vec![0.0; 3];
    let mut rhoa: Vec<(f64, f64)> = Vec::new();

    for p in &pts {
        let d = [p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]];
        let a = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
        let perp = [d[0] - a * w[0], d[1] - a * w[1], d[2] - a * w[2]];
        let rho = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
        rhoa.push((rho, a));
        let row = [rho, a, 1.0];
        let rhs = -(rho * rho + a * a);

        for r in 0..3 {
            atb[r] += row[r] * rhs;

            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    let sol = solve_gauss(&ata, &atb, 3)?;
    let rr = -sol[0] / 2.0;
    let a0 = -sol[1] / 2.0;
    let r2 = rr * rr + a0 * a0 - sol[2];

    if r2 <= 1e-18 || rr <= 0.0 {
        return None;
    }

    let r = r2.sqrt();

    if rr <= r * 0.5 {
        return None;
    }

    for &(rho, a) in &rhoa {
        if (((rho - rr).powi(2) + (a - a0).powi(2)).sqrt() - r).abs() > tol {
            return None;
        }
    }

    let center = [cen[0] + a0 * w[0], cen[1] + a0 * w[1], cen[2] + a0 * w[2]];

    Some((center, w, rr, r))
}

/// Classify a surface as plane, cylinder, cone, sphere or torus within tol.
fn recognize_surface(surface: &NurbsSurface, tol: f64) -> Option<RecSurf> {
    if surface.is_planar(None, tol) {
        let (u0, u1) = surface.domain(0)?;
        let (v0, v1) = surface.domain(1)?;
        let o = surface.point_at((u0 + u1) * 0.5, (v0 + v1) * 0.5)?;
        let n = surface.normal_at((u0 + u1) * 0.5, (v0 + v1) * 0.5);

        return Some(RecSurf::Plane([o[0], o[1], o[2]], [n[0], n[1], n[2]]));
    }

    if let Some(sph) = fit_sphere(surface, tol) {
        return Some(RecSurf::Sphere([sph.0, sph.1, sph.2], sph.3));
    }

    if let Some(cyl) = fit_cylinder(surface, tol) {
        return Some(RecSurf::Cylinder(cyl.0, cyl.1, cyl.2));
    }

    if let Some(cone) = fit_cone(surface, tol) {
        return Some(RecSurf::Cone(cone.0, cone.1, cone.2));
    }

    if let Some(tor) = fit_torus(surface, tol) {
        return Some(RecSurf::Torus(tor.0, tor.1, tor.2, tor.3));
    }

    None
}

/// Solve ((X-V).w)^2 - cos^2a |X-V|^2 = 0 along X = x0 + t d. Returns roots.
fn line_cone(x0: [f64; 3], d: [f64; 3], v: [f64; 3], w: [f64; 3], alpha: f64) -> Vec<f64> {
    let ca2 = alpha.cos().powi(2);
    let e = [x0[0] - v[0], x0[1] - v[1], x0[2] - v[2]];
    let aa = e[0] * w[0] + e[1] * w[1] + e[2] * w[2];
    let bb = d[0] * w[0] + d[1] * w[1] + d[2] * w[2];
    let cc = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    let dd = e[0] * d[0] + e[1] * d[1] + e[2] * d[2];
    let ee = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let qa = bb * bb - ca2 * ee;
    let qb = 2.0 * aa * bb - 2.0 * ca2 * dd;
    let qc = aa * aa - ca2 * cc;

    if qa.abs() < 1e-14 {
        return if qb.abs() < 1e-300 {
            vec![]
        } else {
            vec![-qc / qb]
        };
    }

    let disc = qb * qb - 4.0 * qa * qc;

    if disc < 0.0 {
        return vec![];
    }

    let sq = disc.sqrt();

    vec![(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)]
}

/// Exact plane-sphere section: a circle or nothing.
fn plane_sphere(o: [f64; 3], nraw: [f64; 3], c: [f64; 3], r: f64) -> Option<NurbsCurve> {
    let nu = vunit(nraw);
    let d = (c[0] - o[0]) * nu[0] + (c[1] - o[1]) * nu[1] + (c[2] - o[2]) * nu[2];

    if d.abs() >= r {
        return None;
    }

    let cc = [c[0] - d * nu[0], c[1] - d * nu[1], c[2] - d * nu[2]];
    let rr = (r * r - d * d).sqrt();
    let (xa, ya) = ortho_basis(nu);

    Some(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr))
}

/// Exact plane-cylinder section: an ellipse or nothing.
fn plane_cylinder(
    o: [f64; 3],
    nraw: [f64; 3],
    p_pt: [f64; 3],
    wraw: [f64; 3],
    r: f64,
) -> Option<NurbsCurve> {
    let nu = vunit(nraw);
    let w = vunit(wraw);
    let wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];

    if wn.abs() < 1e-7 {
        return None;
    }

    let t = ((o[0] - p_pt[0]) * nu[0] + (o[1] - p_pt[1]) * nu[1] + (o[2] - p_pt[2]) * nu[2]) / wn;
    let cc = [p_pt[0] + t * w[0], p_pt[1] + t * w[1], p_pt[2] + t * w[2]];
    let mraw = vcross(w, nu);

    if (mraw[0] * mraw[0] + mraw[1] * mraw[1] + mraw[2] * mraw[2]).sqrt() < 1e-9 {
        let (xa, ya) = ortho_basis(nu);

        return Some(exact_circle(cc[0], cc[1], cc[2], xa, ya, r));
    }

    let minor = vunit(mraw);
    let major = vunit([w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]]);
    Some(exact_ellipse(
        cc[0],
        cc[1],
        cc[2],
        major,
        minor,
        r / wn.abs(),
        r,
    ))
}

/// Plane-cone section: exact ellipse when closed, fitted arcs otherwise.
fn plane_cone(
    o: [f64; 3],
    nraw: [f64; 3],
    v: [f64; 3],
    wraw: [f64; 3],
    alpha: f64,
) -> Option<NurbsCurve> {
    let nu = vunit(nraw);
    let w = vunit(wraw);
    let wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];

    if (wn.abs() - 1.0).abs() < 1e-9 {
        let dax = (o[0] - v[0]) * w[0] + (o[1] - v[1]) * w[1] + (o[2] - v[2]) * w[2];
        let rr = dax.abs() * alpha.tan();
        let cc = [v[0] + dax * w[0], v[1] + dax * w[1], v[2] + dax * w[2]];

        if rr < 1e-12 {
            return None;
        }

        let (xa, ya) = ortho_basis(nu);

        return Some(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));
    }

    let mraw = vcross(w, nu);
    let ml = (mraw[0] * mraw[0] + mraw[1] * mraw[1] + mraw[2] * mraw[2]).sqrt();

    if ml < 1e-12 {
        return None;
    }

    let m = [mraw[0] / ml, mraw[1] / ml, mraw[2] / ml];
    let major0 = vunit([w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]]);
    let dv = (v[0] - o[0]) * nu[0] + (v[1] - o[1]) * nu[1] + (v[2] - o[2]) * nu[2];
    let vp = [v[0] - dv * nu[0], v[1] - dv * nu[1], v[2] - dv * nu[2]];
    let ts = line_cone(vp, major0, v, w, alpha);

    if ts.len() != 2 {
        return None;
    }

    let a_pt = [
        vp[0] + ts[0] * major0[0],
        vp[1] + ts[0] * major0[1],
        vp[2] + ts[0] * major0[2],
    ];
    let bp = [
        vp[0] + ts[1] * major0[0],
        vp[1] + ts[1] * major0[1],
        vp[2] + ts[1] * major0[2],
    ];
    let cc = [
        (a_pt[0] + bp[0]) * 0.5,
        (a_pt[1] + bp[1]) * 0.5,
        (a_pt[2] + bp[2]) * 0.5,
    ];
    let semi_major = 0.5
        * ((bp[0] - a_pt[0]).powi(2) + (bp[1] - a_pt[1]).powi(2) + (bp[2] - a_pt[2]).powi(2))
            .sqrt();

    let major = vunit([bp[0] - a_pt[0], bp[1] - a_pt[1], bp[2] - a_pt[2]]);
    let tm = line_cone(cc, m, v, w, alpha);

    if tm.len() != 2 {
        return None;
    }

    let semi_minor = 0.5 * (tm[1] - tm[0]).abs();

    if semi_major < 1e-12 || semi_minor < 1e-12 {
        return None;
    }

    Some(exact_ellipse(
        cc[0], cc[1], cc[2], major, m, semi_major, semi_minor,
    ))
}

/// Plane ∩ torus. Returns None if the plane is not perpendicular to the torus.
fn plane_torus(
    o: [f64; 3],
    nraw: [f64; 3],
    c: [f64; 3],
    wraw: [f64; 3],
    rr: f64,
    r: f64,
) -> Option<Vec<NurbsCurve>> {
    let nu = vunit(nraw);
    let w = vunit(wraw);
    let wn = w[0] * nu[0] + w[1] * nu[1] + w[2] * nu[2];

    if (wn.abs() - 1.0).abs() > 1e-7 {
        return None;
    }

    let d = (o[0] - c[0]) * w[0] + (o[1] - c[1]) * w[1] + (o[2] - c[2]) * w[2];

    if d.abs() > r {
        return Some(vec![]);
    }

    let h = (0.0_f64).max(r * r - d * d).sqrt();
    let cc = [c[0] + d * w[0], c[1] + d * w[1], c[2] + d * w[2]];
    let (xa, ya) = ortho_basis(w);
    let mut out: Vec<NurbsCurve> = Vec::new();

    for radius in [rr + h, rr - h] {
        if radius > 1e-12 {
            out.push(exact_circle(cc[0], cc[1], cc[2], xa, ya, radius));
        }
    }

    Some(out)
}

/// Tri-state result of an exact plane/plane SSI clipped to both (finite) faces.
enum PpResult {
    Line(NurbsCurve),
    Empty,
    Marcher,
}

/// Exact plane-plane line clipped to both finite faces.
fn ssi_plane_plane(
    sa: &NurbsSurface,
    oa: [f64; 3],
    na_raw: [f64; 3],
    sb: &NurbsSurface,
    ob: [f64; 3],
    nb_raw: [f64; 3],
) -> PpResult {
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let na = vunit(na_raw);
    let nb = vunit(nb_raw);
    let v = vcross(na, nb);
    let vl = dot(v, v).sqrt();

    if vl < 1e-9 {
        return PpResult::Marcher;
    }

    let da = dot(na, oa);
    let db = dot(nb, ob);
    let nb_x_v = vcross(nb, v);
    let v_x_na = vcross(v, na);
    let inv = 1.0 / (vl * vl);
    let anchor = [
        (da * nb_x_v[0] + db * v_x_na[0]) * inv,
        (da * nb_x_v[1] + db * v_x_na[1]) * inv,
        (da * nb_x_v[2] + db * v_x_na[2]) * inv,
    ];
    let dir = [v[0] / vl, v[1] / vl, v[2] / vl];

    let axis_clip = |c: f64, d: f64, t0: &mut f64, t1: &mut f64| -> bool {
        if d.abs() < 1e-15 {
            return (-1e-9..=1.0 + 1e-9).contains(&c);
        }

        let mut ta = (0.0 - c) / d;
        let mut tb = (1.0 - c) / d;

        if ta > tb {
            std::mem::swap(&mut ta, &mut tb);
        }

        *t0 = (*t0).max(ta);
        *t1 = (*t1).min(tb);
        true
    };

    let mut tmin: f64 = -1e300;
    let mut tmax: f64 = 1e300;

    for s in [sa, sb] {
        let (u0, u1) = s.domain(0).unwrap_or((0.0, 1.0));
        let (v0, v1) = s.domain(1).unwrap_or((0.0, 1.0));
        let o = s.point_at(u0, v0).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let pu = s.point_at(u1, v0).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let pv = s.point_at(u0, v1).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let o3 = [o[0], o[1], o[2]];
        let eu = [pu[0] - o[0], pu[1] - o[1], pu[2] - o[2]];
        let ev = [pv[0] - o[0], pv[1] - o[1], pv[2] - o[2]];
        let exx = dot(eu, eu);
        let eyy = dot(ev, ev);
        let exy = dot(eu, ev);
        let det = exx * eyy - exy * exy;

        if det.abs() < 1e-18 {
            return PpResult::Marcher;
        }

        let frac = |r: [f64; 3]| -> (f64, f64) {
            let rx = dot(r, eu);
            let ry = dot(r, ev);
            ((eyy * rx - exy * ry) / det, (exx * ry - exy * rx) / det)
        };
        let (a0, b0) = frac([anchor[0] - o3[0], anchor[1] - o3[1], anchor[2] - o3[2]]);
        let (dav, dbv) = frac(dir);
        let mut t0: f64 = -1e300;
        let mut t1: f64 = 1e300;

        if !axis_clip(a0, dav, &mut t0, &mut t1) || !axis_clip(b0, dbv, &mut t0, &mut t1) || t0 > t1
        {
            return PpResult::Empty;
        }

        tmin = tmin.max(t0);
        tmax = tmax.min(t1);
    }

    if tmax - tmin <= 1e-9 {
        return PpResult::Empty;
    }

    let a = Point::new(
        anchor[0] + tmin * dir[0],
        anchor[1] + tmin * dir[1],
        anchor[2] + tmin * dir[2],
    );
    let b = Point::new(
        anchor[0] + tmax * dir[0],
        anchor[1] + tmax * dir[1],
        anchor[2] + tmax * dir[2],
    );
    let mut c3 = NurbsCurve::create(false, 1, &[a, b]);
    c3.set_domain(0.0, 1.0);

    PpResult::Line(c3)
}

/// Analytic pcurve of an exact 3D intersection conic on a recognized quadric surface.
fn analytic_pcurve(srf: &NurbsSurface, recog: &RecSurf, c3d: &NurbsCurve) -> Option<NurbsCurve> {
    let (u0, u1) = srf.domain(0)?;
    let (v0, v1) = srf.domain(1)?;
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    match recog {
        RecSurf::Plane(_o, _n) => {
            let o = srf.point_at(u0, v0)?;
            let pu = srf.point_at(u1, v0)?;
            let pv = srf.point_at(u0, v1)?;
            let ex = [pu[0] - o[0], pu[1] - o[1], pu[2] - o[2]];
            let ey = [pv[0] - o[0], pv[1] - o[1], pv[2] - o[2]];
            let exx = dot(ex, ex);
            let eyy = dot(ey, ey);
            let exy = dot(ex, ey);
            let det = exx * eyy - exy * exy;

            if det.abs() < 1e-18 {
                return None;
            }

            let mut pc = c3d.clone();

            for i in 0..c3d.cv_count() {
                let p = c3d.get_cv(i)?;
                let r = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
                let rx = dot(r, ex);
                let ry = dot(r, ey);
                let a = (eyy * rx - exy * ry) / det;
                let b = (exx * ry - exy * rx) / det;
                let u = u0 + a * (u1 - u0);
                let v = v0 + b * (v1 - v0);

                if c3d.is_rational() {
                    let w = c3d.weight(i);
                    pc.set_cv_4d(i, u * w, v * w, 0.0, w);
                } else {
                    pc.set_cv(i, &Point::new(u, v, 0.0));
                }
            }

            Some(pc)
        }

        RecSurf::Cylinder(p1, p2, _r) => {
            let ap = *p1;
            let mut ax = *p2;
            let an = dot(ax, ax).sqrt();

            if an < 1e-12 {
                return None;
            }

            ax = [ax[0] / an, ax[1] / an, ax[2] / an];
            let height = |p: Point| {
                let r = [p[0] - ap[0], p[1] - ap[1], p[2] - ap[2]];
                dot(r, ax)
            };
            let um = 0.5 * (u0 + u1);
            let h0 = height(srf.point_at(um, v0)?);
            let h1 = height(srf.point_at(um, v1)?);

            if (h1 - h0).abs() < 1e-12 {
                return None;
            }

            let (t0, t1) = c3d.domain();
            let mut hmin: f64 = 1e300;
            let mut hmax: f64 = -1e300;
            let mut hsum = 0.0;
            let mut ns = 0;

            for i in 0..=32 {
                let h = height(c3d.point_at(t0 + (t1 - t0) * i as f64 / 32.0));
                hmin = hmin.min(h);
                hmax = hmax.max(h);
                hsum += h;
                ns += 1;
            }

            if hmax - hmin > 1e-5 * (h1 - h0).abs() {
                return None;
            }

            if c3d.point_at(t0).distance(&c3d.point_at(t1), None) > 1e-6 * ((h1 - h0).abs() + 1.0) {
                return None;
            }

            let hc = hsum / ns as f64;
            let vc = v0 + (hc - h0) / (h1 - h0) * (v1 - v0);

            if vc < v0.min(v1) - 1e-9 || vc > v0.max(v1) + 1e-9 {
                return None;
            }

            Some(NurbsCurve::create(
                false,
                1,
                &[Point::new(u0, vc, 0.0), Point::new(u1, vc, 0.0)],
            ))
        }

        RecSurf::Sphere(c, r) => {
            let um = 0.5 * (u0 + u1);
            let sp = srf.point_at(um, v0)?;
            let np = srf.point_at(um, v1)?;
            let mut ax = [np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]];
            let an = dot(ax, ax).sqrt();

            if an < 1e-12 {
                return None;
            }

            ax = [ax[0] / an, ax[1] / an, ax[2] / an];
            let cc = *c;
            let height = |p: Point| {
                let rr = [p[0] - cc[0], p[1] - cc[1], p[2] - cc[2]];
                dot(rr, ax)
            };
            let (t0, t1) = c3d.domain();
            let mut hmin: f64 = 1e300;
            let mut hmax: f64 = -1e300;
            let mut hsum = 0.0;
            let mut ns = 0;

            for i in 0..=32 {
                let h = height(c3d.point_at(t0 + (t1 - t0) * i as f64 / 32.0));
                hmin = hmin.min(h);
                hmax = hmax.max(h);
                hsum += h;
                ns += 1;
            }

            if hmax - hmin > *r * 1e-4 {
                return None;
            }

            if c3d.point_at(t0).distance(&c3d.point_at(t1), None) > *r * 1e-3 {
                return None;
            }

            let hc = hsum / ns as f64;
            let mut va = v0;
            let mut vb = v1;
            let mut ha = height(srf.point_at(um, va)?);
            let hb = height(srf.point_at(um, vb)?);

            if (hc - ha) * (hc - hb) > 0.0 {
                return None;
            }

            for _ in 0..60 {
                let vm = 0.5 * (va + vb);
                let hm = height(srf.point_at(um, vm).unwrap_or(Point::new(0.0, 0.0, 0.0)));

                if (hm - hc) * (ha - hc) <= 0.0 {
                    vb = vm;
                } else {
                    va = vm;
                    ha = hm;
                }
            }

            let vc = 0.5 * (va + vb);
            Some(NurbsCurve::create(
                false,
                1,
                &[Point::new(u0, vc, 0.0), Point::new(u1, vc, 0.0)],
            ))
        }

        RecSurf::Cone(p1, p2, _r) => {
            let mut ax = *p2;
            let an = dot(ax, ax).sqrt();

            if an < 1e-12 {
                return None;
            }

            ax = [ax[0] / an, ax[1] / an, ax[2] / an];
            let aa = *p1;
            let height = |p: Point| {
                let r = [p[0] - aa[0], p[1] - aa[1], p[2] - aa[2]];
                dot(r, ax)
            };
            let (t0, t1) = c3d.domain();
            let clen = c3d
                .point_at(t0)
                .distance(&c3d.point_at(0.5 * (t0 + t1)), None);

            let hscale = clen.max(1e-9);
            let mut hmin: f64 = 1e300;
            let mut hmax: f64 = -1e300;
            let mut hsum = 0.0;
            let mut ns = 0;

            for i in 0..=32 {
                let h = height(c3d.point_at(t0 + (t1 - t0) * i as f64 / 32.0));
                hmin = hmin.min(h);
                hmax = hmax.max(h);
                hsum += h;
                ns += 1;
            }

            if hmax - hmin > hscale * 1e-4 {
                return None;
            }

            if c3d.point_at(t0).distance(&c3d.point_at(t1), None) > hscale * 1e-3 {
                return None;
            }

            let hc = hsum / ns as f64;
            let um2 = 0.5 * (u0 + u1);
            let mut va = v0;
            let mut vb = v1;
            let mut ha = height(srf.point_at(um2, va)?);
            let hb = height(srf.point_at(um2, vb)?);

            if (hc - ha) * (hc - hb) > 0.0 {
                return None;
            }

            for _ in 0..60 {
                let vmid = 0.5 * (va + vb);
                let hm = height(srf.point_at(um2, vmid).unwrap_or(Point::new(0.0, 0.0, 0.0)));

                if (hm - hc) * (ha - hc) <= 0.0 {
                    vb = vmid;
                } else {
                    va = vmid;
                    ha = hm;
                }
            }

            let vc = 0.5 * (va + vb);
            Some(NurbsCurve::create(
                false,
                1,
                &[Point::new(u0, vc, 0.0), Point::new(u1, vc, 0.0)],
            ))
        }

        _ => None,
    }
}

/// Degree-1 UV polyline through pull-back nodes.
fn emit_pullback_curve(nodes: &[[f64; 2]]) -> NurbsCurve {
    let mut pts: Vec<Point> = Vec::with_capacity(nodes.len());

    for node in nodes {
        pts.push(Point::new(node[0], node[1], 0.0));
    }

    NurbsCurve::create(false, 1, &pts)
}

/// Pull a 3D curve back to sphere parameters through longitude and latitude.
fn analytic_sphere_pullback(
    srf: &NurbsSurface,
    recog: &RecSurf,
    c3d: &NurbsCurve,
) -> Vec<NurbsCurve> {
    let cc = match recog {
        RecSurf::Sphere(c, _r) => *c,
        _ => return vec![],
    };
    let (u0, u1) = match srf.domain(0) {
        Some(d) => d,
        None => return vec![],
    };
    let (v0, v1) = match srf.domain(1) {
        Some(d) => d,
        None => return vec![],
    };
    let range_u = u1 - u0;

    if range_u < 1e-9 {
        return vec![];
    }

    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let um = 0.5 * (u0 + u1);
    let vm = 0.5 * (v0 + v1);
    let sp = match srf.point_at(um, v0) {
        Some(p) => p,
        None => return vec![],
    };
    let np = match srf.point_at(um, v1) {
        Some(p) => p,
        None => return vec![],
    };
    let mut zs = [np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]];
    let zn = dot(zs, zs).sqrt();

    if zn < 1e-12 {
        return vec![];
    }

    zs = [zs[0] / zn, zs[1] / zn, zs[2] / zn];
    let p0 = match srf.point_at(u0, vm) {
        Some(p) => p,
        None => return vec![],
    };
    let x0 = [p0[0] - cc[0], p0[1] - cc[1], p0[2] - cc[2]];
    let h0 = dot(x0, zs);
    let mut xs = [x0[0] - h0 * zs[0], x0[1] - h0 * zs[1], x0[2] - h0 * zs[2]];
    let xn = dot(xs, xs).sqrt();

    if xn < 1e-12 {
        return vec![];
    }

    xs = [xs[0] / xn, xs[1] / xn, xs[2] / xn];
    let ys = [
        zs[1] * xs[2] - zs[2] * xs[1],
        zs[2] * xs[0] - zs[0] * xs[2],
        zs[0] * xs[1] - zs[1] * xs[0],
    ];
    const PI: f64 = std::f64::consts::PI;
    const TWO_PI: f64 = 2.0 * PI;
    const NT: usize = 128;
    let mut tu = [0.0_f64; NT + 1];
    let mut tlon = [0.0_f64; NT + 1];

    for k in 0..=NT {
        let u = u0 + range_u * k as f64 / NT as f64;
        let p = srf.point_at(u, vm).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let r = [p[0] - cc[0], p[1] - cc[1], p[2] - cc[2]];
        let mut lon = dot(r, ys).atan2(dot(r, xs));

        if k > 0 {
            while lon - tlon[k - 1] > PI {
                lon -= TWO_PI;
            }

            while lon - tlon[k - 1] < -PI {
                lon += TWO_PI;
            }
        }

        tu[k] = u;
        tlon[k] = lon;
    }

    let lon_incr = tlon[NT] >= tlon[0];
    let lon_lo = tlon[0].min(tlon[NT]);
    let lon_hi = tlon[0].max(tlon[NT]);
    let u_from_lon = |mut lon: f64| -> f64 {
        while lon < lon_lo - 1e-9 {
            lon += TWO_PI;
        }

        while lon > lon_hi + 1e-9 {
            lon -= TWO_PI;
        }

        let mut lo = 0usize;
        let mut hi = NT;

        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            let above = if lon_incr {
                tlon[mid] < lon
            } else {
                tlon[mid] > lon
            };

            if above {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let denom = tlon[hi] - tlon[lo];
        let f = if denom.abs() > 1e-15 {
            (lon - tlon[lo]) / denom
        } else {
            0.0
        };
        tu[lo] + (tu[hi] - tu[lo]) * f
    };
    let mut tv = [0.0_f64; NT + 1];
    let mut th = [0.0_f64; NT + 1];

    for k in 0..=NT {
        let v = v0 + (v1 - v0) * k as f64 / NT as f64;
        let p = srf.point_at(um, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let r = [p[0] - cc[0], p[1] - cc[1], p[2] - cc[2]];
        tv[k] = v;
        th[k] = dot(r, zs);
    }

    let incr = th[NT] >= th[0];

    if (th[NT] - th[0]).abs() < 1e-12 {
        return vec![];
    }

    let v_from_height = |h: f64| -> f64 {
        if incr {
            if h <= th[0] {
                return tv[0];
            }

            if h >= th[NT] {
                return tv[NT];
            }
        } else {
            if h >= th[0] {
                return tv[0];
            }

            if h <= th[NT] {
                return tv[NT];
            }
        }

        let mut lo = 0usize;
        let mut hi = NT;

        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            let above = if incr { th[mid] < h } else { th[mid] > h };

            if above {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let denom = th[hi] - th[lo];
        let f = if denom.abs() > 1e-15 {
            (h - th[lo]) / denom
        } else {
            0.0
        };
        tv[lo] + (tv[hi] - tv[lo]) * f
    };
    let (t0, t1) = c3d.domain();
    let project_t = |t: f64| -> (f64, f64) {
        let p = c3d.point_at(t);
        let r = [p[0] - cc[0], p[1] - cc[1], p[2] - cc[2]];
        let lon = dot(r, ys).atan2(dot(r, xs));
        let h = dot(r, zs);
        let mut u = u_from_lon(lon);

        for _ in 0..2 {
            let du_ = range_u * 1e-7;
            let uc = u.max(u0).min(u1);
            let pc0 = srf.point_at(uc, vm).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let rc0 = [pc0[0] - cc[0], pc0[1] - cc[1], pc0[2] - cc[2]];
            let mut g0 = dot(rc0, ys).atan2(dot(rc0, xs)) - lon;

            while g0 > PI {
                g0 -= TWO_PI;
            }

            while g0 < -PI {
                g0 += TWO_PI;
            }

            let pc1 = srf
                .point_at((uc + du_).min(u1), vm)
                .unwrap_or(Point::new(0.0, 0.0, 0.0));

            let rc1 = [pc1[0] - cc[0], pc1[1] - cc[1], pc1[2] - cc[2]];
            let mut g1 = dot(rc1, ys).atan2(dot(rc1, xs)) - lon;

            while g1 > PI {
                g1 -= TWO_PI;
            }

            while g1 < -PI {
                g1 += TWO_PI;
            }

            let dg = (g1 - g0) / du_;

            if dg.abs() < 1e-12 {
                break;
            }

            u = (uc - g0 / dg).max(u0).min(u1);
        }

        let mut v = v_from_height(h);

        for _ in 0..2 {
            let dv_ = (v1 - v0) * 1e-7;
            let vc2 = v.max(v0.min(v1)).min(v0.max(v1));
            let qc0 = srf.point_at(um, vc2).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let g0 =
                (qc0[0] - cc[0]) * zs[0] + (qc0[1] - cc[1]) * zs[1] + (qc0[2] - cc[2]) * zs[2] - h;

            let qc1 = srf
                .point_at(um, (vc2 + dv_).min(v0.max(v1)))
                .unwrap_or(Point::new(0.0, 0.0, 0.0));

            let g1 =
                (qc1[0] - cc[0]) * zs[0] + (qc1[1] - cc[1]) * zs[1] + (qc1[2] - cc[2]) * zs[2] - h;

            let dg = (g1 - g0) / dv_;

            if dg.abs() < 1e-12 {
                break;
            }

            v = (vc2 - g0 / dg).max(v0.min(v1)).min(v0.max(v1));
        }

        (u, v)
    };
    let n = (c3d.cv_count() * 8).max(120);
    let mut uv: Vec<[f64; 2]> = Vec::with_capacity(n + 1);
    let mut prev_u = 0.0_f64;

    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let (mut u, v) = project_t(t);

        if i > 0 {
            while u - prev_u > range_u * 0.5 {
                u -= range_u;
            }

            while u - prev_u < -range_u * 0.5 {
                u += range_u;
            }
        }

        prev_u = u;
        uv.push([u, v]);
    }

    if uv.len() < 2 {
        return vec![];
    }

    let mut out: Vec<NurbsCurve> = Vec::new();
    let mut seg: Vec<[f64; 2]> = Vec::new();
    let kof = |u: f64| -> i64 { ((u - u0) / range_u + 1e-9).floor() as i64 };
    let mut cur_k = kof(uv[0][0]);
    seg.push([uv[0][0] - cur_k as f64 * range_u, uv[0][1]]);

    for i in 1..uv.len() {
        let ki = kof(uv[i][0]);

        while ki != cur_k {
            let step: i64 = if ki > cur_k { 1 } else { -1 };
            let nk = cur_k + step;
            let seam_cont = u0 + (if step > 0 { nk } else { cur_k }) as f64 * range_u;
            let denom = uv[i][0] - uv[i - 1][0];
            let mut f = if denom.abs() > 1e-15 {
                (seam_cont - uv[i - 1][0]) / denom
            } else {
                0.0
            };
            f = f.clamp(0.0, 1.0);
            let vc = uv[i - 1][1] + (uv[i][1] - uv[i - 1][1]) * f;
            seg.push([seam_cont - cur_k as f64 * range_u, vc]);

            if seg.len() >= 2 {
                out.push(emit_pullback_curve(&seg));
            }

            seg.clear();
            seg.push([seam_cont - nk as f64 * range_u, vc]);
            cur_k = nk;
        }

        seg.push([uv[i][0] - cur_k as f64 * range_u, uv[i][1]]);
    }

    if seg.len() >= 2 {
        out.push(emit_pullback_curve(&seg));
    }

    out
}

/// Analytic pull-back of a 3D curve onto a recognized cone or cylinder.
fn analytic_cone_pullback(
    srf: &NurbsSurface,
    recog: &RecSurf,
    c3d: &NurbsCurve,
) -> Vec<NurbsCurve> {
    let (aa, mut zc) = match recog {
        RecSurf::Cone(p1, p2, _r) => (*p1, *p2),
        RecSurf::Cylinder(p1, p2, _r) => (*p1, *p2),
        _ => return vec![],
    };
    let (u0, u1) = match srf.domain(0) {
        Some(d) => d,
        None => return vec![],
    };
    let (v0, v1) = match srf.domain(1) {
        Some(d) => d,
        None => return vec![],
    };
    let range_u = u1 - u0;

    if range_u < 1e-9 {
        return vec![];
    }

    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let zn = dot(zc, zc).sqrt();

    if zn < 1e-12 {
        return vec![];
    }

    zc = [zc[0] / zn, zc[1] / zn, zc[2] / zn];
    let height = |p: &Point| -> f64 {
        let r = [p[0] - aa[0], p[1] - aa[1], p[2] - aa[2]];
        dot(r, zc)
    };
    let um = 0.5 * (u0 + u1);
    let h0 = height(&srf.point_at(um, v0).unwrap_or(Point::new(0.0, 0.0, 0.0)));
    let h1 = height(&srf.point_at(um, v1).unwrap_or(Point::new(0.0, 0.0, 0.0)));

    if (h1 - h0).abs() < 1e-12 {
        return vec![];
    }

    let v_from_height = |h: f64| -> f64 { v0 + (h - h0) / (h1 - h0) * (v1 - v0) };
    let v_ref = if h0.abs() >= h1.abs() { v0 } else { v1 };
    let p0 = match srf.point_at(u0, v_ref) {
        Some(p) => p,
        None => return vec![],
    };
    let x0 = [p0[0] - aa[0], p0[1] - aa[1], p0[2] - aa[2]];
    let hp = dot(x0, zc);
    let mut xc = [x0[0] - hp * zc[0], x0[1] - hp * zc[1], x0[2] - hp * zc[2]];
    let xn = dot(xc, xc).sqrt();

    if xn < 1e-12 {
        return vec![];
    }

    xc = [xc[0] / xn, xc[1] / xn, xc[2] / xn];
    let yc = [
        zc[1] * xc[2] - zc[2] * xc[1],
        zc[2] * xc[0] - zc[0] * xc[2],
        zc[0] * xc[1] - zc[1] * xc[0],
    ];
    const PI: f64 = std::f64::consts::PI;
    const TWO_PI: f64 = 2.0 * PI;
    const NT: usize = 128;
    let mut tu = [0.0_f64; NT + 1];
    let mut tlon = [0.0_f64; NT + 1];

    for k in 0..=NT {
        let u = u0 + range_u * k as f64 / NT as f64;
        let p = srf.point_at(u, v_ref).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let r = [p[0] - aa[0], p[1] - aa[1], p[2] - aa[2]];
        let mut lon = dot(r, yc).atan2(dot(r, xc));

        if k > 0 {
            while lon - tlon[k - 1] > PI {
                lon -= TWO_PI;
            }

            while lon - tlon[k - 1] < -PI {
                lon += TWO_PI;
            }
        }

        tu[k] = u;
        tlon[k] = lon;
    }

    let lon_incr = tlon[NT] >= tlon[0];
    let lon_lo = tlon[0].min(tlon[NT]);
    let lon_hi = tlon[0].max(tlon[NT]);
    let u_from_lon = |mut lon: f64| -> f64 {
        while lon < lon_lo - 1e-9 {
            lon += TWO_PI;
        }

        while lon > lon_hi + 1e-9 {
            lon -= TWO_PI;
        }

        let mut lo = 0usize;
        let mut hi = NT;

        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            let above = if lon_incr {
                tlon[mid] < lon
            } else {
                tlon[mid] > lon
            };

            if above {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let denom = tlon[hi] - tlon[lo];
        let f = if denom.abs() > 1e-15 {
            (lon - tlon[lo]) / denom
        } else {
            0.0
        };
        tu[lo] + (tu[hi] - tu[lo]) * f
    };
    let (t0, t1) = c3d.domain();
    let n = (c3d.cv_count() * 8).max(120);
    let mut prev_lon_s = 0.0_f64;
    let mut project_t = |tq: f64| -> (f64, f64) {
        let p = c3d.point_at(tq);
        let r = [p[0] - aa[0], p[1] - aa[1], p[2] - aa[2]];
        let rad = (dot(r, xc) * dot(r, xc) + dot(r, yc) * dot(r, yc))
            .max(0.0)
            .sqrt();

        let lon = if rad > 1e-12 {
            dot(r, yc).atan2(dot(r, xc))
        } else {
            prev_lon_s
        };
        prev_lon_s = lon;
        let mut u = u_from_lon(lon);

        if rad > 1e-12 {
            for _ in 0..2 {
                let du_ = range_u * 1e-7;
                let uc = u.max(u0).min(u1);
                let pc0 = srf.point_at(uc, v_ref).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let rc0 = [pc0[0] - aa[0], pc0[1] - aa[1], pc0[2] - aa[2]];
                let mut g0 = dot(rc0, yc).atan2(dot(rc0, xc)) - lon;

                while g0 > PI {
                    g0 -= TWO_PI;
                }

                while g0 < -PI {
                    g0 += TWO_PI;
                }

                let pc1 = srf
                    .point_at((uc + du_).min(u1), v_ref)
                    .unwrap_or(Point::new(0.0, 0.0, 0.0));

                let rc1 = [pc1[0] - aa[0], pc1[1] - aa[1], pc1[2] - aa[2]];
                let mut g1 = dot(rc1, yc).atan2(dot(rc1, xc)) - lon;

                while g1 > PI {
                    g1 -= TWO_PI;
                }

                while g1 < -PI {
                    g1 += TWO_PI;
                }

                let dg = (g1 - g0) / du_;

                if dg.abs() < 1e-12 {
                    break;
                }

                u = (uc - g0 / dg).max(u0).min(u1);
            }
        }

        (u, v_from_height(dot(r, zc)))
    };
    let mut uv: Vec<[f64; 2]> = Vec::with_capacity(n + 1);
    let mut prev_u = 0.0_f64;

    for i in 0..=n {
        let tq = t0 + (t1 - t0) * i as f64 / n as f64;
        let (mut u, v) = project_t(tq);

        if i > 0 {
            while u - prev_u > range_u * 0.5 {
                u -= range_u;
            }

            while u - prev_u < -range_u * 0.5 {
                u += range_u;
            }
        }

        prev_u = u;
        uv.push([u, v]);
    }

    if uv.len() < 2 {
        return vec![];
    }

    let mut out: Vec<NurbsCurve> = Vec::new();
    let mut seg: Vec<[f64; 2]> = Vec::new();
    let kof = |u: f64| -> i64 { ((u - u0) / range_u + 1e-9).floor() as i64 };
    let mut cur_k = kof(uv[0][0]);
    seg.push([uv[0][0] - cur_k as f64 * range_u, uv[0][1]]);

    for i in 1..uv.len() {
        let ki = kof(uv[i][0]);

        while ki != cur_k {
            let step: i64 = if ki > cur_k { 1 } else { -1 };
            let nk = cur_k + step;
            let seam_cont = u0 + (if step > 0 { nk } else { cur_k }) as f64 * range_u;
            let denom = uv[i][0] - uv[i - 1][0];
            let mut f = if denom.abs() > 1e-15 {
                (seam_cont - uv[i - 1][0]) / denom
            } else {
                0.0
            };
            f = f.clamp(0.0, 1.0);
            let vc = uv[i - 1][1] + (uv[i][1] - uv[i - 1][1]) * f;
            seg.push([seam_cont - cur_k as f64 * range_u, vc]);

            if seg.len() >= 2 {
                out.push(emit_pullback_curve(&seg));
            }

            seg.clear();
            seg.push([seam_cont - nk as f64 * range_u, vc]);
            cur_k = nk;
        }

        seg.push([uv[i][0] - cur_k as f64 * range_u, uv[i][1]]);
    }

    if seg.len() >= 2 {
        out.push(emit_pullback_curve(&seg));
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// Coaxial quadric pairs
// ═══════════════════════════════════════════════════════════════════════════

/// Distance of P from the axis through apt along adir.
#[allow(clippy::too_many_arguments)]
fn point_axis_dist(apt: [f64; 3], adir: [f64; 3], p: [f64; 3]) -> f64 {
    let u = vunit(adir);
    let dp = [p[0] - apt[0], p[1] - apt[1], p[2] - apt[2]];
    let t = vdot(dp, u);
    let perp = [dp[0] - t * u[0], dp[1] - t * u[1], dp[2] - t * u[2]];

    vdot(perp, perp).sqrt()
}

/// Coordinate of P along the axis through apt along adir.
fn axial_coord(apt: [f64; 3], adir: [f64; 3], p: [f64; 3]) -> f64 {
    let u = vunit(adir);

    (p[0] - apt[0]) * u[0] + (p[1] - apt[1]) * u[1] + (p[2] - apt[2]) * u[2]
}

/// Whether two axes coincide within tol.
fn axes_coaxial(p1: [f64; 3], d1: [f64; 3], p2: [f64; 3], d2: [f64; 3], tol: f64) -> bool {
    let u1 = vunit(d1);
    let u2 = vunit(d2);
    let cx = vcross(u1, u2);

    if vdot(cx, cx).sqrt() > tol {
        return false;
    }

    point_axis_dist(p1, u1, p2) <= tol
}

/// Axial extent of the surface along the cylinder axis.
fn cyl_span(srf: &NurbsSurface, apt: [f64; 3], adir: [f64; 3]) -> (f64, f64) {
    let u = vunit(adir);
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 1.0));
    let um = 0.5 * (u0 + u1);
    let mut smin: f64 = 1e300;
    let mut smax: f64 = -1e300;

    for vv in [v0, v1] {
        if let Some(p) = srf.point_at(um, vv) {
            let s = (p[0] - apt[0]) * u[0] + (p[1] - apt[1]) * u[1] + (p[2] - apt[2]) * u[2];
            smin = smin.min(s);
            smax = smax.max(s);
        }
    }

    (smin, smax)
}

/// Closest point of two lines, false when parallel.
fn lines_closest_point(
    p1: [f64; 3],
    d1: [f64; 3],
    p2: [f64; 3],
    d2: [f64; 3],
    tol: f64,
) -> Option<[f64; 3]> {
    let u = vunit(d1);
    let v = vunit(d2);
    let w0 = [p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]];
    let a = vdot(u, u);
    let b = vdot(u, v);
    let c = vdot(v, v);
    let d = vdot(u, w0);
    let e = vdot(v, w0);
    let den = a * c - b * b;

    if den.abs() < 1e-12 {
        return None;
    }

    let sc = (b * e - c * d) / den;
    let tc = (a * e - b * d) / den;
    let q1 = [p1[0] + sc * u[0], p1[1] + sc * u[1], p1[2] + sc * u[2]];
    let q2 = [p2[0] + tc * v[0], p2[1] + tc * v[1], p2[2] + tc * v[2]];
    let diff = [q1[0] - q2[0], q1[1] - q2[1], q1[2] - q2[2]];

    if vdot(diff, diff).sqrt() > tol {
        return None;
    }

    Some([
        0.5 * (q1[0] + q2[0]),
        0.5 * (q1[1] + q2[1]),
        0.5 * (q1[2] + q2[2]),
    ])
}

/// Coaxial cylinder-sphere section: circles.
fn ssi_cylinder_sphere(
    cyl_p: [f64; 3],
    cyl_w: [f64; 3],
    rc: f64,
    sph_c: [f64; 3],
    r_sph: f64,
) -> Option<Vec<NurbsCurve>> {
    let ktol = 1e-6;
    let w = vunit(cyl_w);

    if point_axis_dist(cyl_p, w, sph_c) > ktol {
        return None;
    }

    let mut out: Vec<NurbsCurve> = Vec::new();

    if r_sph < rc - ktol {
        return Some(out);
    }

    let dist = (r_sph * r_sph - rc * rc).max(0.0).sqrt();
    let (xa, ya) = ortho_basis(w);

    if dist <= ktol {
        out.push(exact_circle(sph_c[0], sph_c[1], sph_c[2], xa, ya, rc));

        return Some(out);
    }

    for s in [dist, -dist] {
        let cc = [
            sph_c[0] + s * w[0],
            sph_c[1] + s * w[1],
            sph_c[2] + s * w[2],
        ];
        out.push(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));
    }

    Some(out)
}

/// Coaxial cylinder-cone section: circles.
fn ssi_cylinder_cone(
    cyl_p: [f64; 3],
    cyl_w: [f64; 3],
    rc: f64,
    cone_apex: [f64; 3],
    cone_a: [f64; 3],
    alpha: f64,
) -> Option<Vec<NurbsCurve>> {
    let ktol = 1e-6;
    let w = vunit(cyl_w);
    let a = vunit(cone_a);

    if !axes_coaxial(cyl_p, w, cone_apex, a, ktol) {
        return None;
    }

    let ta = alpha.tan();

    if ta < 1e-9 {
        return None;
    }

    let s = rc / ta;
    let mut out: Vec<NurbsCurve> = Vec::new();

    if s < ktol {
        return Some(out);
    }

    let cc = [
        cone_apex[0] + s * a[0],
        cone_apex[1] + s * a[1],
        cone_apex[2] + s * a[2],
    ];
    let (xa, ya) = ortho_basis(a);
    out.push(exact_circle(cc[0], cc[1], cc[2], xa, ya, rc));

    Some(out)
}

/// Coaxial cone-sphere section: circles.
fn ssi_cone_sphere(
    cone_apex: [f64; 3],
    cone_a: [f64; 3],
    alpha: f64,
    sph_c: [f64; 3],
    r_sph: f64,
) -> Option<Vec<NurbsCurve>> {
    let ktol = 1e-6;
    let a = vunit(cone_a);

    if point_axis_dist(cone_apex, a, sph_c) > ktol {
        return None;
    }

    let dsign = axial_coord(cone_apex, a, sph_c);
    let d = dsign.abs();
    let dir = if d > ktol && dsign < 0.0 {
        [-a[0], -a[1], -a[2]]
    } else {
        a
    };
    let t = alpha.tan();
    let t2 = t * t;
    let aq = 1.0 + t2;
    let bq = 2.0 * t2 * d;
    let cq = t2 * d * d - r_sph * r_sph;
    let disc = bq * bq - 4.0 * aq * cq;
    let mut out: Vec<NurbsCurve> = Vec::new();

    if disc < -ktol {
        return Some(out);
    }

    let sq = disc.max(0.0).sqrt();
    let xs: Vec<f64> = if sq <= ktol {
        vec![-bq / (2.0 * aq)]
    } else {
        vec![(-bq - sq) / (2.0 * aq), (-bq + sq) / (2.0 * aq)]
    };
    let (xa, ya) = ortho_basis(a);

    for x in xs {
        let s_ax = d + x;

        if s_ax < ktol {
            continue;
        }

        let rr = t * s_ax;

        if rr < ktol {
            continue;
        }

        let cc = [
            cone_apex[0] + s_ax * dir[0],
            cone_apex[1] + s_ax * dir[1],
            cone_apex[2] + s_ax * dir[2],
        ];
        out.push(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr));
    }

    Some(out)
}

/// Cylinder-cylinder section: circles when coaxial, Steinmetz curves when the axes meet.
#[allow(clippy::too_many_arguments)]
fn ssi_cylinder_cylinder(
    sa: &NurbsSurface,
    p1: [f64; 3],
    w1raw: [f64; 3],
    r1: f64,
    sb: &NurbsSurface,
    p2: [f64; 3],
    w2raw: [f64; 3],
    r2: f64,
) -> Option<Vec<NurbsCurve>> {
    let ktol = 1e-6;
    let w1 = vunit(w1raw);
    let w2 = vunit(w2raw);
    let cx = vcross(w1, w2);
    let sinmag = vdot(cx, cx).sqrt();
    let mut out: Vec<NurbsCurve> = Vec::new();

    if sinmag <= ktol {
        let dline = point_axis_dist(p1, w1, p2);

        if dline <= ktol {
            if (r1 - r2).abs() <= ktol {
                return None;
            }

            return Some(out);
        }

        let off = vdot([p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]], w1);
        let p2p = [
            p2[0] - off * w1[0],
            p2[1] - off * w1[1],
            p2[2] - off * w1[2],
        ];
        let d = dline;

        if d > r1 + r2 + ktol {
            return Some(out);
        }

        if d < (r1 - r2).abs() - ktol {
            return Some(out);
        }

        let xdir = vunit([p2p[0] - p1[0], p2p[1] - p1[1], p2p[2] - p1[2]]);
        let ydir = vunit(vcross(w1, xdir));
        let aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
        let h = (r1 * r1 - aa * aa).max(0.0).sqrt();
        let foot = [
            p1[0] + aa * xdir[0],
            p1[1] + aa * xdir[1],
            p1[2] + aa * xdir[2],
        ];
        let (s0a, s1a) = cyl_span(sa, p1, w1);
        let (s0b, s1b) = cyl_span(sb, p1, w1);
        let slo = s0a.max(s0b);
        let shi = s1a.min(s1b);

        if shi - slo <= ktol {
            return Some(out);
        }

        let emit = |bp: [f64; 3], out: &mut Vec<NurbsCurve>| {
            let e0 = [
                bp[0] + slo * w1[0],
                bp[1] + slo * w1[1],
                bp[2] + slo * w1[2],
            ];
            let e1 = [
                bp[0] + shi * w1[0],
                bp[1] + shi * w1[1],
                bp[2] + shi * w1[2],
            ];
            let mut ln = NurbsCurve::create(
                false,
                1,
                &[
                    Point::new(e0[0], e0[1], e0[2]),
                    Point::new(e1[0], e1[1], e1[2]),
                ],
            );
            ln.set_domain(0.0, 1.0);
            out.push(ln);
        };

        if h <= ktol {
            emit(foot, &mut out);
        } else {
            emit(
                [
                    foot[0] + h * ydir[0],
                    foot[1] + h * ydir[1],
                    foot[2] + h * ydir[2],
                ],
                &mut out,
            );
            emit(
                [
                    foot[0] - h * ydir[0],
                    foot[1] - h * ydir[1],
                    foot[2] - h * ydir[2],
                ],
                &mut out,
            );
        }

        return Some(out);
    }

    let rmax = r1.max(r2);

    if rmax < 1e-12 || (r1 - r2).abs() / rmax > 1e-6 {
        return None;
    }

    let pint = lines_closest_point(p1, w1, p2, w2, ktol)?;
    let r = 0.5 * (r1 + r2);
    let ang = vdot(w1, w2).clamp(-1.0, 1.0).acos();
    let sh = (0.5 * ang).sin();
    let ch = (0.5 * ang).cos();

    if sh < 1e-9 || ch < 1e-9 {
        return None;
    }

    let minor = vunit(cx);
    let maj1 = vunit([w1[0] + w2[0], w1[1] + w2[1], w1[2] + w2[2]]);
    let maj2 = vunit([w1[0] - w2[0], w1[1] - w2[1], w1[2] - w2[2]]);
    out.push(exact_ellipse(
        pint[0],
        pint[1],
        pint[2],
        maj1,
        minor,
        r / sh,
        r,
    ));
    out.push(exact_ellipse(
        pint[0],
        pint[1],
        pint[2],
        maj2,
        minor,
        r / ch,
        r,
    ));

    Some(out)
}

/// Exact section of two recognized analytic surfaces, empty when no case applies.
fn analytic_ssi(
    a: &NurbsSurface,
    b: &NurbsSurface,
    tolerance: Option<f64>,
) -> Option<Vec<(NurbsCurve, NurbsCurve, NurbsCurve)>> {
    let tol = match tolerance {
        Some(t) if t > 0.0 => t,
        _ => 1e-12,
    };
    let rtol = tol.max(1e-7) * 1e4;
    let ra = recognize_surface(a, rtol)?;
    let rb = recognize_surface(b, rtol)?;

    let single = |c: Option<NurbsCurve>| -> Vec<NurbsCurve> {
        match c {
            Some(crv) => vec![crv],
            None => vec![],
        }
    };

    let c3_list: Vec<NurbsCurve> = match (&ra, &rb) {
        (RecSurf::Plane(oa, na), RecSurf::Plane(ob, nb)) => {
            match ssi_plane_plane(a, *oa, *na, b, *ob, *nb) {
                PpResult::Line(c) => vec![c],
                PpResult::Empty => vec![],
                PpResult::Marcher => return None,
            }
        }

        (RecSurf::Plane(o, n), RecSurf::Sphere(c, r)) => single(plane_sphere(*o, *n, *c, *r)),
        (RecSurf::Sphere(c, r), RecSurf::Plane(o, n)) => single(plane_sphere(*o, *n, *c, *r)),
        (RecSurf::Plane(o, n), RecSurf::Cylinder(p, w, r)) => {
            single(plane_cylinder(*o, *n, *p, *w, *r))
        }

        (RecSurf::Cylinder(p, w, r), RecSurf::Plane(o, n)) => {
            single(plane_cylinder(*o, *n, *p, *w, *r))
        }

        (RecSurf::Plane(o, n), RecSurf::Cone(v, w, alpha)) => {
            single(plane_cone(*o, *n, *v, *w, *alpha))
        }

        (RecSurf::Cone(v, w, alpha), RecSurf::Plane(o, n)) => {
            single(plane_cone(*o, *n, *v, *w, *alpha))
        }

        (RecSurf::Plane(o, n), RecSurf::Torus(c, w, rr, r)) => {
            plane_torus(*o, *n, *c, *w, *rr, *r)?
        }

        (RecSurf::Torus(c, w, rr, r), RecSurf::Plane(o, n)) => {
            plane_torus(*o, *n, *c, *w, *rr, *r)?
        }

        (RecSurf::Sphere(c1, r1), RecSurf::Sphere(c2, r2)) => {
            let dv = [c2[0] - c1[0], c2[1] - c1[1], c2[2] - c1[2]];
            let dist = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();
            let mut c3: Option<NurbsCurve> = None;

            if 1e-12 < dist && dist < r1 + r2 && dist > (r1 - r2).abs() {
                let nu = [dv[0] / dist, dv[1] / dist, dv[2] / dist];
                let aa = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
                let rr2 = r1 * r1 - aa * aa;

                if rr2 > 0.0 {
                    let cc = [c1[0] + aa * nu[0], c1[1] + aa * nu[1], c1[2] + aa * nu[2]];
                    let (xa, ya) = ortho_basis(nu);
                    c3 = Some(exact_circle(cc[0], cc[1], cc[2], xa, ya, rr2.sqrt()));
                }
            }

            single(c3)
        }

        (RecSurf::Cylinder(p, w, r), RecSurf::Sphere(c, sr)) => {
            ssi_cylinder_sphere(*p, *w, *r, *c, *sr)?
        }

        (RecSurf::Sphere(c, sr), RecSurf::Cylinder(p, w, r)) => {
            ssi_cylinder_sphere(*p, *w, *r, *c, *sr)?
        }

        (RecSurf::Cylinder(p, w, r), RecSurf::Cone(v2, w2, alpha)) => {
            ssi_cylinder_cone(*p, *w, *r, *v2, *w2, *alpha)?
        }

        (RecSurf::Cone(v2, w2, alpha), RecSurf::Cylinder(p, w, r)) => {
            ssi_cylinder_cone(*p, *w, *r, *v2, *w2, *alpha)?
        }

        (RecSurf::Cone(v2, w2, alpha), RecSurf::Sphere(c, sr)) => {
            ssi_cone_sphere(*v2, *w2, *alpha, *c, *sr)?
        }

        (RecSurf::Sphere(c, sr), RecSurf::Cone(v2, w2, alpha)) => {
            ssi_cone_sphere(*v2, *w2, *alpha, *c, *sr)?
        }

        (RecSurf::Cylinder(pa, wa, ra_r), RecSurf::Cylinder(pb, wb, rb_r)) => {
            ssi_cylinder_cylinder(a, *pa, *wa, *ra_r, b, *pb, *wb, *rb_r)?
        }

        _ => return None,
    };

    let mut triples: Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> = Vec::new();

    for cc3 in c3_list {
        let mut pa = analytic_pcurve(a, &ra, &cc3);
        let mut pb = analytic_pcurve(b, &rb, &cc3);

        if pa.is_none() && matches!(ra, RecSurf::Sphere(..)) {
            let v = analytic_sphere_pullback(a, &ra, &cc3);

            if !v.is_empty() {
                pa = Some(v[0].clone());
            }
        }

        if pb.is_none() && matches!(rb, RecSurf::Sphere(..)) {
            let v = analytic_sphere_pullback(b, &rb, &cc3);

            if !v.is_empty() {
                pb = Some(v[0].clone());
            }
        }

        if pa.is_none() && matches!(ra, RecSurf::Cone(..) | RecSurf::Cylinder(..)) {
            let v = analytic_cone_pullback(a, &ra, &cc3);

            if !v.is_empty() {
                pa = Some(v[0].clone());
            }
        }

        if pb.is_none() && matches!(rb, RecSurf::Cone(..) | RecSurf::Cylinder(..)) {
            let v = analytic_cone_pullback(b, &rb, &cc3);

            if !v.is_empty() {
                pb = Some(v[0].clone());
            }
        }

        if pa.is_none() {
            let v = Closest::surface_curve(a, &cc3, 0.0, 0.0, 0.0);

            if !v.is_empty() {
                pa = Some(v[0].clone());
            }
        }

        if pb.is_none() {
            let v = Closest::surface_curve(b, &cc3, 0.0, 0.0, 0.0);

            if !v.is_empty() {
                pb = Some(v[0].clone());
            }
        }

        if let (Some(pa), Some(pb)) = (pa, pb) {
            if pa.is_valid() && pb.is_valid() {
                triples.push((cc3, pa, pb));
            }
        }
    }

    Some(triples)
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS surfaces
// ═══════════════════════════════════════════════════════════════════════════

/// Surface-plane section curves.
pub fn surface_plane(
    surface: &NurbsSurface,
    plane: &Plane,
    tolerance: Option<f64>,
) -> Vec<NurbsCurve> {
    if !surface.is_valid() {
        return vec![];
    }

    let tolerance = tolerance
        .unwrap_or(Tolerance::ZERO_TOLERANCE)
        .max(Tolerance::ZERO_TOLERANCE);

    let traced = surface_plane_traces(surface, plane, tolerance);
    let step = traced.step;
    let uv_to_3d = traced.uv_to_3d;
    let uv_to_3d_min = traced.uv_to_3d_min;

    let mut result: Vec<NurbsCurve> = Vec::new();

    for trace in &traced.traces {
        let uv_trace = &trace.uv_trace;
        let is_loop = &trace.is_loop;
        let all_pts: Vec<Point> = uv_trace
            .iter()
            .map(|&(u, v)| surface.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0)))
            .collect();

        let crv = surface_plane_fit_3d(
            &all_pts,
            *is_loop,
            plane,
            step,
            uv_to_3d,
            uv_to_3d_min,
            true,
        );

        if !crv.is_valid() {
            continue;
        }

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

                if d > dup_tol {
                    all_close = false;
                    break;
                }
            }

            if all_close {
                dup = true;
                break;
            }
        }

        if !dup {
            result.push(crv);
        }
    }

    result
}

/// Surface-plane section curves paired with their UV pcurves.
pub fn surface_plane_uv(
    surface: &NurbsSurface,
    plane: &Plane,
    tolerance: Option<f64>,
) -> Vec<(NurbsCurve, NurbsCurve)> {
    if !surface.is_valid() {
        return vec![];
    }

    let tolerance = match tolerance {
        Some(t) if t > 0.0 => t,
        _ => Tolerance::ZERO_TOLERANCE,
    };

    let Some(field) = SurfacePlaneField::new(surface, plane, tolerance) else {
        return vec![];
    };

    let traced = surface_plane_traces(surface, plane, tolerance);
    let dup_tol = traced.step * traced.uv_to_3d * 3.0;

    let mut result: Vec<(NurbsCurve, NurbsCurve)> = Vec::new();
    let mut kept_pts3: Vec<Vec<Point>> = Vec::new();

    for trace in &traced.traces {
        let mut trace_pts3: Vec<Point> = Vec::with_capacity(trace.uv_trace.len());

        for q in &trace.uv_trace {
            trace_pts3.push(field.point(*q));
        }

        if is_duplicate_trace(&trace_pts3, &kept_pts3, dup_tol) {
            continue;
        }

        kept_pts3.push(trace_pts3);

        for mut piece in trace_pieces(&field, trace) {
            if piece.uv.len() < 2 {
                continue;
            }

            if let Some(curves) = piece_curves(&field, plane, &mut piece) {
                result.push(curves);
            }
        }
    }

    result
}

/// Surface-surface section curves with their UV pcurves on both surfaces.
pub fn surface_surface(
    a: &NurbsSurface,
    b: &NurbsSurface,
    tolerance: Option<f64>,
) -> Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    if a.is_valid() && b.is_valid() {
        if let Some(ana) = analytic_ssi(a, b, tolerance) {
            return ana;
        }
    }

    if !a.is_valid() || !b.is_valid() {
        return vec![];
    }

    let tolerance = match tolerance {
        Some(t) if t > 0.0 => t,
        _ => 1e-12,
    };

    let plane_from = |srf: &NurbsSurface| -> Plane {
        let (s0, s1) = srf.domain(0).unwrap();
        let (t0, t1) = srf.domain(1).unwrap();
        let po = srf
            .point_at((s0 + s1) * 0.5, (t0 + t1) * 0.5)
            .unwrap_or(Point::new(0.0, 0.0, 0.0));

        let nn = srf.normal_at((s0 + s1) * 0.5, (t0 + t1) * 0.5);
        Plane::from_point_normal(po, Vector::new(nn[0], nn[1], nn[2]), None)
    };

    if a.is_planar(None, 1e-9) {
        let plane = plane_from(a);
        let mut result = Vec::new();

        for (c3, pb) in surface_plane_uv(b, &plane, Some(tolerance)) {
            let pas = Closest::surface_curve(a, &c3, 0.0, 0.0, tolerance);

            if pas.len() == 1 {
                result.push((c3, pas[0].clone(), pb));
            }
        }

        return result;
    }

    if b.is_planar(None, 1e-9) {
        let plane = plane_from(b);
        let mut result = Vec::new();

        for (c3, pa) in surface_plane_uv(a, &plane, Some(tolerance)) {
            let pbs = Closest::surface_curve(b, &c3, 0.0, 0.0, tolerance);

            if pbs.len() == 1 {
                result.push((c3, pa, pbs[0].clone()));
            }
        }

        return result;
    }

    let (au0, au1) = a.domain(0).unwrap();
    let (av0, av1) = a.domain(1).unwrap();
    let (bu0, bu1) = b.domain(0).unwrap();
    let (bv0, bv1) = b.domain(1).unwrap();
    let a_range_u = au1 - au0;
    let a_range_v = av1 - av0;
    let b_range_u = bu1 - bu0;
    let b_range_v = bv1 - bv0;
    let a_closed_u = a.is_closed(0);
    let a_closed_v = a.is_closed(1);
    let b_closed_u = b.is_closed(0);
    let b_closed_v = b.is_closed(1);

    let make_wrap = |c0: f64, c1: f64, rng: f64, closed: bool| {
        move |t: f64| -> f64 {
            if closed {
                let mut f = (t - c0) % rng;

                if f < 0.0 {
                    f += rng;
                }

                return c0 + f;
            }

            t.max(c0).min(c1)
        }
    };

    let a_wrap_u = make_wrap(au0, au1, a_range_u, a_closed_u);
    let a_wrap_v = make_wrap(av0, av1, a_range_v, a_closed_v);
    let b_wrap_u = make_wrap(bu0, bu1, b_range_u, b_closed_u);
    let b_wrap_v = make_wrap(bv0, bv1, b_range_v, b_closed_v);

    let eval_a = |u: f64, v: f64| -> (Vector, Vector, Vector) {
        let d = a.evaluate(a_wrap_u(u), a_wrap_v(v), 1);
        (d[0].clone(), d[2].clone(), d[1].clone())
    };
    let eval_b = |u: f64, v: f64| -> (Vector, Vector, Vector) {
        let d = b.evaluate(b_wrap_u(u), b_wrap_v(v), 1);
        (d[0].clone(), d[2].clone(), d[1].clone())
    };

    let spans_au = a.get_span_vector(0);
    let spans_av = a.get_span_vector(1);
    let spans_bu = b.get_span_vector(0);
    let spans_bv = b.get_span_vector(1);
    let a_nu = (spans_au.len().saturating_sub(1)).max(1) * 4;
    let a_nv = (spans_av.len().saturating_sub(1)).max(1) * 4;
    let b_nu = (spans_bu.len().saturating_sub(1)).max(1) * 4;
    let b_nv = (spans_bv.len().saturating_sub(1)).max(1) * 4;
    let a_du = a_range_u / a_nu as f64;
    let a_dv = a_range_v / a_nv as f64;
    let b_du = b_range_u / b_nu as f64;
    let b_dv = b_range_v / b_nv as f64;

    let cell_boxes = |srf: &NurbsSurface,
                      c0u: f64,
                      dcu: f64,
                      ncu: usize,
                      c0v: f64,
                      dcv: f64,
                      ncv: usize|
     -> Vec<[f64; 8]> {
        let mut s_grid: Vec<Vec<Point>> = Vec::with_capacity(2 * ncu + 1);

        for i in 0..(2 * ncu + 1) {
            let mut row = Vec::with_capacity(2 * ncv + 1);

            for j in 0..(2 * ncv + 1) {
                row.push(
                    srf.point_at(c0u + dcu * 0.5 * i as f64, c0v + dcv * 0.5 * j as f64)
                        .unwrap_or(Point::new(0.0, 0.0, 0.0)),
                );
            }

            s_grid.push(row);
        }

        let mut boxes = Vec::new();

        for ci in 0..ncu {
            for cj in 0..ncv {
                let mut xs = f64::INFINITY;
                let mut ys = f64::INFINITY;
                let mut zs = f64::INFINITY;
                let mut xl = f64::NEG_INFINITY;
                let mut yl = f64::NEG_INFINITY;
                let mut zl = f64::NEG_INFINITY;

                for row in s_grid.iter().skip(2 * ci).take(3) {
                    for p in row.iter().skip(2 * cj).take(3) {
                        xs = xs.min(p[0]);
                        ys = ys.min(p[1]);
                        zs = zs.min(p[2]);
                        xl = xl.max(p[0]);
                        yl = yl.max(p[1]);
                        zl = zl.max(p[2]);
                    }
                }

                let ctr = &s_grid[2 * ci + 1][2 * cj + 1];
                let cx = (s_grid[2 * ci][2 * cj][0]
                    + s_grid[2 * ci + 2][2 * cj][0]
                    + s_grid[2 * ci][2 * cj + 2][0]
                    + s_grid[2 * ci + 2][2 * cj + 2][0])
                    * 0.25;
                let cy = (s_grid[2 * ci][2 * cj][1]
                    + s_grid[2 * ci + 2][2 * cj][1]
                    + s_grid[2 * ci][2 * cj + 2][1]
                    + s_grid[2 * ci + 2][2 * cj + 2][1])
                    * 0.25;
                let cz = (s_grid[2 * ci][2 * cj][2]
                    + s_grid[2 * ci + 2][2 * cj][2]
                    + s_grid[2 * ci][2 * cj + 2][2]
                    + s_grid[2 * ci + 2][2 * cj + 2][2])
                    * 0.25;
                let sag =
                    ((ctr[0] - cx).powi(2) + (ctr[1] - cy).powi(2) + (ctr[2] - cz).powi(2)).sqrt();

                let inf = 2.0 * sag + tolerance;
                boxes.push([
                    xs - inf,
                    ys - inf,
                    zs - inf,
                    xl + inf,
                    yl + inf,
                    zl + inf,
                    c0u + dcu * (ci as f64 + 0.5),
                    c0v + dcv * (cj as f64 + 0.5),
                ]);
            }
        }

        boxes
    };

    let boxes_a = cell_boxes(a, au0, a_du, a_nu, av0, a_dv, a_nv);
    let boxes_b = cell_boxes(b, bu0, b_du, b_nu, bv0, b_dv, b_nv);

    let cell_3d = |boxes: &[[f64; 8]]| -> f64 {
        let mut best = f64::INFINITY;

        for bx in boxes.iter().take(64) {
            let d = ((bx[3] - bx[0]).powi(2) + (bx[4] - bx[1]).powi(2) + (bx[5] - bx[2]).powi(2))
                .sqrt();

            if 1e-12 < d && d < best {
                best = d;
            }
        }

        if best < f64::INFINITY {
            best
        } else {
            1.0
        }
    };

    let h_init = cell_3d(&boxes_a).min(cell_3d(&boxes_b)) * 0.25;
    let conv_tol = tolerance.max(h_init * 1e-7);

    let clamp_open = |x: &mut [f64; 4]| {
        if !a_closed_u {
            x[0] = x[0].max(au0).min(au1);
        }

        if !a_closed_v {
            x[1] = x[1].max(av0).min(av1);
        }

        if !b_closed_u {
            x[2] = x[2].max(bu0).min(bu1);
        }

        if !b_closed_v {
            x[3] = x[3].max(bv0).min(bv1);
        }
    };

    let correct = |x: &mut [f64; 4], pin: Option<(&[f64; 3], &[f64; 3])>| -> bool {
        for _ in 0..8 {
            let (sa, sau, sav) = eval_a(x[0], x[1]);
            let (sb, sbu, sbv) = eval_b(x[2], x[3]);
            let f = [sa[0] - sb[0], sa[1] - sb[1], sa[2] - sb[2]];

            if (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt() < conv_tol {
                return true;
            }

            let j: [[f64; 4]; 3] = [
                [sau[0], sav[0], -sbu[0], -sbv[0]],
                [sau[1], sav[1], -sbu[1], -sbv[1]],
                [sau[2], sav[2], -sbu[2], -sbv[2]],
            ];

            match pin {
                None => {
                    let jjt: Vec<Vec<f64>> = (0..3)
                        .map(|r| {
                            (0..3)
                                .map(|q| (0..4).map(|c| j[r][c] * j[q][c]).sum())
                                .collect()
                        })
                        .collect();

                    let y = match solve_gauss(&jjt, &f, 3) {
                        Some(y) => y,
                        None => return false,
                    };

                    for c in 0..4 {
                        x[c] -= (0..3).map(|r| j[r][c] * y[r]).sum::<f64>();
                    }
                }

                Some((d, pp)) => {
                    let m: Vec<Vec<f64>> = vec![
                        j[0].to_vec(),
                        j[1].to_vec(),
                        j[2].to_vec(),
                        vec![
                            d[0] * sau[0] + d[1] * sau[1] + d[2] * sau[2],
                            d[0] * sav[0] + d[1] * sav[1] + d[2] * sav[2],
                            0.0,
                            0.0,
                        ],
                    ];
                    let rhs = [
                        f[0],
                        f[1],
                        f[2],
                        d[0] * (sa[0] - pp[0]) + d[1] * (sa[1] - pp[1]) + d[2] * (sa[2] - pp[2]),
                    ];
                    let dx = match solve_gauss(&m, &rhs, 4) {
                        Some(dx) => dx,
                        None => return false,
                    };

                    for c in 0..4 {
                        x[c] -= dx[c];
                    }
                }
            }

            clamp_open(x);
        }

        let (sa, _, _) = eval_a(x[0], x[1]);
        let (sb, _, _) = eval_b(x[2], x[3]);
        let g =
            ((sa[0] - sb[0]).powi(2) + (sa[1] - sb[1]).powi(2) + (sa[2] - sb[2]).powi(2)).sqrt();

        g < conv_tol * 10.0
    };

    let mut seeds: Vec<[f64; 5]> = Vec::new();
    let seed_tol_3d = cell_3d(&boxes_a).max(cell_3d(&boxes_b));
    let mut pair_budget: i64 = 20000;
    'outer: for ba in &boxes_a {
        if pair_budget < 0 {
            break;
        }

        for bb in &boxes_b {
            if bb[0] > ba[3]
                || bb[3] < ba[0]
                || bb[1] > ba[4]
                || bb[4] < ba[1]
                || bb[2] > ba[5]
                || bb[5] < ba[2]
            {
                continue;
            }

            pair_budget -= 1;

            if pair_budget < 0 {
                break 'outer;
            }

            let mut x = [ba[6], ba[7], bb[6], bb[7]];

            if !correct(&mut x, None) {
                continue;
            }

            let (sa, _, _) = eval_a(x[0], x[1]);
            let mut dup = false;

            for sd in &seeds {
                let (so, _, _) = eval_a(sd[0], sd[1]);

                if ((sa[0] - so[0]).powi(2) + (sa[1] - so[1]).powi(2) + (sa[2] - so[2]).powi(2))
                    .sqrt()
                    < seed_tol_3d
                {
                    dup = true;
                    break;
                }
            }

            if !dup {
                seeds.push([
                    a_wrap_u(x[0]),
                    a_wrap_v(x[1]),
                    b_wrap_u(x[2]),
                    b_wrap_v(x[3]),
                    0.0,
                ]);
            }
        }
    }

    let max_steps = (a_nu * a_nv + b_nu * b_nv) * 32;
    let close_tol = h_init * 3.0;
    let consume_tol = h_init * 2.0;

    let tangent_3d = |x: &[f64; 4],
                      dir_sign: f64|
     -> Option<([f64; 3], Vector, Vector, Vector, Vector, Vector)> {
        let (sa, sau, sav) = eval_a(x[0], x[1]);
        let (_sb, sbu, sbv) = eval_b(x[2], x[3]);
        let na = [
            sau[1] * sav[2] - sau[2] * sav[1],
            sau[2] * sav[0] - sau[0] * sav[2],
            sau[0] * sav[1] - sau[1] * sav[0],
        ];
        let nb = [
            sbu[1] * sbv[2] - sbu[2] * sbv[1],
            sbu[2] * sbv[0] - sbu[0] * sbv[2],
            sbu[0] * sbv[1] - sbu[1] * sbv[0],
        ];
        let d = [
            na[1] * nb[2] - na[2] * nb[1],
            na[2] * nb[0] - na[0] * nb[2],
            na[0] * nb[1] - na[1] * nb[0],
        ];
        let dl = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let nal = (na[0] * na[0] + na[1] * na[1] + na[2] * na[2]).sqrt();
        let nbl = (nb[0] * nb[0] + nb[1] * nb[1] + nb[2] * nb[2]).sqrt();

        if dl < 1e-4 * nal * nbl || dl < 1e-30 {
            return None;
        }

        Some((
            [
                d[0] / dl * dir_sign,
                d[1] / dl * dir_sign,
                d[2] / dl * dir_sign,
            ],
            sa,
            sau,
            sav,
            sbu,
            sbv,
        ))
    };

    let trace_dir =
        |x0: &[f64; 4], dir_sign: f64, seeds: &mut Vec<[f64; 5]>| -> (Vec<[f64; 4]>, bool) {
            let mut out: Vec<[f64; 4]> = Vec::new();
            let mut x = *x0;
            let mut prev_d: Option<[f64; 3]> = None;
            let (sa0, _, _) = eval_a(x[0], x[1]);
            let p_start = [sa0[0], sa0[1], sa0[2]];
            let mut p_prev = p_start;
            let mut dist_traveled = 0.0;
            let mut h = h_init;
            let mut smooth = 0;

            for _step in 0..max_steps {
                let tng = match tangent_3d(&x, dir_sign) {
                    Some(t) => t,
                    None => break,
                };
                let (d, sa, sau, sav, sbu, sbv) = tng;
                let mut accepted = false;
                let mut attempts = 0;
                let mut xn = [0.0f64; 4];
                let mut p_cur = [0.0f64; 3];
                let mut step_len = 0.0;
                let mut hit_boundary = false;

                while attempts < 7 && !accepted {
                    let duv_a = solve_gauss(
                        &[
                            vec![
                                sau[0].powi(2) + sau[1].powi(2) + sau[2].powi(2),
                                sau[0] * sav[0] + sau[1] * sav[1] + sau[2] * sav[2],
                            ],
                            vec![
                                sau[0] * sav[0] + sau[1] * sav[1] + sau[2] * sav[2],
                                sav[0].powi(2) + sav[1].powi(2) + sav[2].powi(2),
                            ],
                        ],
                        &[
                            h * (d[0] * sau[0] + d[1] * sau[1] + d[2] * sau[2]),
                            h * (d[0] * sav[0] + d[1] * sav[1] + d[2] * sav[2]),
                        ],
                        2,
                    );
                    let duv_b = solve_gauss(
                        &[
                            vec![
                                sbu[0].powi(2) + sbu[1].powi(2) + sbu[2].powi(2),
                                sbu[0] * sbv[0] + sbu[1] * sbv[1] + sbu[2] * sbv[2],
                            ],
                            vec![
                                sbu[0] * sbv[0] + sbu[1] * sbv[1] + sbu[2] * sbv[2],
                                sbv[0].powi(2) + sbv[1].powi(2) + sbv[2].powi(2),
                            ],
                        ],
                        &[
                            h * (d[0] * sbu[0] + d[1] * sbu[1] + d[2] * sbu[2]),
                            h * (d[0] * sbv[0] + d[1] * sbv[1] + d[2] * sbv[2]),
                        ],
                        2,
                    );
                    let (duv_a, duv_b) = match (duv_a, duv_b) {
                        (Some(da), Some(db)) => (da, db),
                        _ => return (out, false),
                    };
                    let delta = [duv_a[0], duv_a[1], duv_b[0], duv_b[1]];
                    let mut tc = 1.0f64;
                    hit_boundary = false;

                    for &(idx, lo, hi, closed) in &[
                        (0usize, au0, au1, a_closed_u),
                        (1, av0, av1, a_closed_v),
                        (2, bu0, bu1, b_closed_u),
                        (3, bv0, bv1, b_closed_v),
                    ] {
                        if closed || delta[idx].abs() < 1e-15 {
                            continue;
                        }

                        if x[idx] + delta[idx] > hi {
                            tc = tc.min((hi - x[idx]) / delta[idx]);
                            hit_boundary = true;
                        }

                        if x[idx] + delta[idx] < lo {
                            tc = tc.min((lo - x[idx]) / delta[idx]);
                            hit_boundary = true;
                        }
                    }

                    let mut xn_local = [0.0f64; 4];

                    for k in 0..4 {
                        xn_local[k] = x[k] + tc * delta[k];
                    }

                    let p_pred = [
                        sa[0] + d[0] * h * tc,
                        sa[1] + d[1] * h * tc,
                        sa[2] + d[2] * h * tc,
                    ];

                    if !correct(&mut xn_local, Some((&d, &p_pred))) {
                        return (out, false);
                    }

                    xn = xn_local;
                    let (san, _, _) = eval_a(xn[0], xn[1]);
                    p_cur = [san[0], san[1], san[2]];
                    step_len = ((p_cur[0] - p_prev[0]).powi(2)
                        + (p_cur[1] - p_prev[1]).powi(2)
                        + (p_cur[2] - p_prev[2]).powi(2))
                    .sqrt();

                    if let Some(pd) = prev_d {
                        if step_len > 1e-14 {
                            let sd_ = [
                                (p_cur[0] - p_prev[0]) / step_len,
                                (p_cur[1] - p_prev[1]) / step_len,
                                (p_cur[2] - p_prev[2]) / step_len,
                            ];
                            let ddot = sd_[0] * pd[0] + sd_[1] * pd[1] + sd_[2] * pd[2];

                            if ddot < 0.985 && attempts < 6 && !hit_boundary {
                                h *= 0.5;
                                attempts += 1;
                                smooth = 0;
                                continue;
                            }
                        }
                    }

                    accepted = true;
                }

                if !accepted {
                    break;
                }

                prev_d = Some(d);
                smooth += 1;

                if smooth >= 5 && h < h_init * 2.0 {
                    h *= 1.4;
                    smooth = 0;
                }

                x = xn;
                dist_traveled += step_len;

                if dist_traveled > close_tol * 3.0
                    && ((p_cur[0] - p_start[0]).powi(2)
                        + (p_cur[1] - p_start[1]).powi(2)
                        + (p_cur[2] - p_start[2]).powi(2))
                    .sqrt()
                        < close_tol
                {
                    out.push(x);

                    return (out, true);
                }

                out.push(x);
                p_prev = p_cur;

                if hit_boundary {
                    break;
                }

                for sd in seeds.iter_mut() {
                    if sd[4] == 0.0 {
                        let (so, _, _) = eval_a(sd[0], sd[1]);

                        if ((p_cur[0] - so[0]).powi(2)
                            + (p_cur[1] - so[1]).powi(2)
                            + (p_cur[2] - so[2]).powi(2))
                        .sqrt()
                            < consume_tol
                        {
                            sd[4] = 1.0;
                        }
                    }
                }
            }

            (out, false)
        };

    let axes: [(usize, f64, f64, bool); 4] = [
        (0, au0, a_range_u, a_closed_u),
        (1, av0, a_range_v, a_closed_v),
        (2, bu0, b_range_u, b_closed_u),
        (3, bv0, b_range_v, b_closed_v),
    ];

    let eval3_q = |q: &[f64]| -> [f64; 3] {
        let (sa, _, _) = eval_a(q[0], q[1]);
        [sa[0], sa[1], sa[2]]
    };

    let mut result: Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> = Vec::new();
    let mut kept_pts3: Vec<Vec<[f64; 3]>> = Vec::new();

    for si in 0..seeds.len() {
        if seeds[si][4] != 0.0 {
            continue;
        }

        seeds[si][4] = 1.0;
        let mut x0 = [seeds[si][0], seeds[si][1], seeds[si][2], seeds[si][3]];

        if !correct(&mut x0, None) {
            continue;
        }

        let (fwd, fwd_closed) = trace_dir(&x0, 1.0, &mut seeds);
        let bwd = if !fwd_closed {
            trace_dir(&x0, -1.0, &mut seeds).0
        } else {
            Vec::new()
        };

        let mut quad: Vec<[f64; 4]> = Vec::new();

        for i in (0..bwd.len()).rev() {
            quad.push(bwd[i]);
        }

        quad.push(x0);

        for p in &fwd {
            quad.push(*p);
        }

        if quad.len() < 4 {
            continue;
        }

        for i in 1..quad.len() {
            for &(idx, _c0, rng, closed) in &axes {
                if !closed {
                    continue;
                }

                let jump = quad[i][idx] - quad[i - 1][idx];

                if jump > rng * 0.5 {
                    quad[i][idx] -= rng;
                } else if jump < -rng * 0.5 {
                    quad[i][idx] += rng;
                }
            }
        }

        let p_first = eval3_q(&quad[0]);
        let p_last = eval3_q(&quad[quad.len() - 1]);
        let gap2 = ((p_first[0] - p_last[0]).powi(2)
            + (p_first[1] - p_last[1]).powi(2)
            + (p_first[2] - p_last[2]).powi(2))
        .sqrt();
        let is_loop = fwd_closed || (quad.len() >= 6 && gap2 < close_tol);

        if is_loop {
            quad.pop();
        }

        if quad.len() < 4 {
            continue;
        }

        let m = quad.len();
        let trace_pts3: Vec<[f64; 3]> = quad.iter().map(|q| eval3_q(q)).collect();
        let dup_tol = h_init * 2.0;
        let mut dup = false;

        for other in &kept_pts3 {
            let mut all_close = true;

            for &f in &[0.25, 0.5, 0.75] {
                let cp = trace_pts3[((m - 1) as f64 * f) as usize];
                let mut dmin = dup_tol + 1.0;

                for op in other {
                    dmin = dmin.min(
                        ((cp[0] - op[0]).powi(2)
                            + (cp[1] - op[1]).powi(2)
                            + (cp[2] - op[2]).powi(2))
                        .sqrt(),
                    );
                }

                if dmin > dup_tol {
                    all_close = false;
                    break;
                }
            }

            if all_close {
                dup = true;
                break;
            }
        }

        if dup {
            continue;
        }

        kept_pts3.push(trace_pts3);

        let gap3 = |qi: &[f64; 4], qj: &[f64; 4]| -> f64 {
            let pi = eval3_q(qi);
            let pj = eval3_q(qj);
            ((pi[0] - pj[0]).powi(2) + (pi[1] - pj[1]).powi(2) + (pi[2] - pj[2]).powi(2)).sqrt()
        };

        for _gp in 0..4 {
            let mut gg: Vec<f64> = Vec::with_capacity(quad.len().saturating_sub(1));

            for i in 0..quad.len() - 1 {
                gg.push(gap3(&quad[i], &quad[i + 1]));
            }

            if gg.is_empty() {
                break;
            }

            let mut sorted = gg.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = sorted[sorted.len() / 2];

            if med <= 0.0 {
                break;
            }

            let mut changed = false;
            let mut i = 0;

            while i < quad.len() - 1 && quad.len() < 4000 {
                if gap3(&quad[i], &quad[i + 1]) > 1.5 * med {
                    let mut mid = [0.0f64; 4];

                    for k in 0..4 {
                        mid[k] = (quad[i][k] + quad[i + 1][k]) * 0.5;
                    }

                    if correct(&mut mid, None) {
                        quad.insert(i + 1, mid);
                        changed = true;
                        i += 2;
                        continue;
                    }
                }

                i += 1;
            }

            if !changed {
                break;
            }
        }

        let mut closure = [0.0f64; 4];

        if is_loop && quad.len() >= 2 {
            let mut virt = quad[0];

            for &(idx, _c0, rng, closed) in &axes {
                let mut jump = quad[0][idx] - quad[quad.len() - 1][idx];

                if closed {
                    while jump > rng * 0.5 {
                        jump -= rng;
                    }

                    while jump < -rng * 0.5 {
                        jump += rng;
                    }
                }

                virt[idx] = quad[quad.len() - 1][idx] + jump;
                closure[idx] = virt[idx] - quad[0][idx];
            }

            quad.push(virt);
        }

        let mut out_pts: Vec<[f64; 4]> = vec![quad[0]];
        let mut cross_idx: Vec<usize> = Vec::new();

        for i in 1..quad.len() {
            let pa_ = quad[i - 1];
            let pb_ = quad[i];
            let mut crossings: Vec<(f64, usize, f64)> = Vec::new();

            for &(idx, c0, rng, closed) in &axes {
                if !closed || (pb_[idx] - pa_[idx]).abs() <= 1e-15 {
                    continue;
                }

                let k0 = ((pa_[idx] - c0) / rng).floor() as i64;
                let k1 = ((pb_[idx] - c0) / rng).floor() as i64;

                for k in (k0.min(k1) + 1)..=(k0.max(k1)) {
                    let l = c0 + k as f64 * rng;
                    let t = (l - pa_[idx]) / (pb_[idx] - pa_[idx]);

                    if 0.0 < t && t < 1.0 {
                        crossings.push((t, idx, l));
                    }
                }
            }

            crossings.sort_by(|a, b| a.partial_cmp(b).unwrap());

            for &(t, idx, l) in &crossings {
                let mut cp = [0.0f64; 4];

                for k in 0..4 {
                    cp[k] = pa_[k] + (pb_[k] - pa_[k]) * t;
                }

                cp[idx] = l;
                correct(&mut cp, None);
                out_pts.push(cp);
                cross_idx.push(out_pts.len() - 1);
            }

            out_pts.push(pb_);

            if i < quad.len() - 1 {
                let mut on_seam = false;

                for &(idx, c0, rng, closed) in &axes {
                    if !closed {
                        continue;
                    }

                    let k = ((pb_[idx] - c0) / rng).round();
                    let l = c0 + k * rng;

                    if (pb_[idx] - l).abs() < rng * 1e-9 && (pb_[idx] - pa_[idx]).abs() > rng * 1e-9
                    {
                        let last = out_pts.len() - 1;
                        out_pts[last][idx] = l;
                        on_seam = true;
                    }
                }

                if on_seam {
                    cross_idx.push(out_pts.len() - 1);
                }
            }
        }

        let mut wrap_drift = false;

        for &(idx, _c0, rng, _closed) in &axes {
            if closure[idx].abs() > rng * 0.5 {
                wrap_drift = true;
            }
        }

        let mut pieces: Vec<(Vec<[f64; 4]>, bool)> = Vec::new();

        if cross_idx.is_empty() {
            pieces.push((out_pts.clone(), is_loop && !wrap_drift));
        } else if is_loop {
            for w in cross_idx.windows(2) {
                let (ia, ib) = (w[0], w[1]);
                pieces.push((out_pts[ia..=ib].to_vec(), false));
            }

            let mut wrap_piece: Vec<[f64; 4]> = out_pts[cross_idx[cross_idx.len() - 1]..].to_vec();

            for p in &out_pts[1..=cross_idx[0]] {
                let mut wp = [0.0f64; 4];

                for k in 0..4 {
                    wp[k] = p[k] + closure[k];
                }

                wrap_piece.push(wp);
            }

            pieces.push((wrap_piece, false));
        } else {
            let mut bounds: Vec<usize> = vec![0];

            for &c in &cross_idx {
                bounds.push(c);
            }

            bounds.push(out_pts.len() - 1);

            for w in bounds.windows(2) {
                let (ia, ib) = (w[0], w[1]);

                if ib > ia {
                    pieces.push((out_pts[ia..=ib].to_vec(), false));
                }
            }
        }

        for (mut piece_pts, piece_loop) in pieces {
            if piece_pts.len() < 2 {
                continue;
            }

            let mid = piece_pts[piece_pts.len() / 2];

            for &(idx, c0, rng, closed) in &axes {
                if !closed {
                    continue;
                }

                let k_s = ((mid[idx] - c0) / rng).floor();

                if k_s != 0.0 {
                    for p in piece_pts.iter_mut() {
                        p[idx] -= k_s * rng;
                    }
                }
            }

            let mut pts3: Vec<[f64; 3]> = piece_pts.iter().map(|p| eval3_q(p)).collect();
            let mut chord3 = 0.0;

            for i in 1..pts3.len() {
                chord3 += ((pts3[i][0] - pts3[i - 1][0]).powi(2)
                    + (pts3[i][1] - pts3[i - 1][1]).powi(2)
                    + (pts3[i][2] - pts3[i - 1][2]).powi(2))
                .sqrt();
            }

            if chord3 < h_init * 0.5 {
                continue;
            }

            let refine_tol = (tolerance * 100.0).max(5e-6);

            for _dp in 0..8 {
                let mut refined = false;
                let mut new_pp: Vec<[f64; 4]> = vec![piece_pts[0]];
                let mut i = 0;

                while i < piece_pts.len() - 1 && piece_pts.len() < 3000 {
                    let pa2 = piece_pts[i];
                    let pb2 = piece_pts[i + 1];
                    let p3a = eval3_q(&pa2);
                    let p3b = eval3_q(&pb2);
                    let mut mid = [0.0f64; 4];

                    for k in 0..4 {
                        mid[k] = (pa2[k] + pb2[k]) * 0.5;
                    }

                    if correct(&mut mid, None) {
                        let p3m = eval3_q(&mid);
                        let ex = p3b[0] - p3a[0];
                        let ey = p3b[1] - p3a[1];
                        let ez = p3b[2] - p3a[2];
                        let l2 = ex * ex + ey * ey + ez * ez;
                        let dev = if l2 > 1e-30 {
                            let tt = ((p3m[0] - p3a[0]) * ex
                                + (p3m[1] - p3a[1]) * ey
                                + (p3m[2] - p3a[2]) * ez)
                                / l2;

                            let cx = p3a[0] + tt * ex;
                            let cy = p3a[1] + tt * ey;
                            let cz = p3a[2] + tt * ez;
                            ((p3m[0] - cx).powi(2) + (p3m[1] - cy).powi(2) + (p3m[2] - cz).powi(2))
                                .sqrt()
                        } else {
                            0.0
                        };

                        if dev > refine_tol {
                            new_pp.push(mid);
                            refined = true;
                        }
                    }

                    new_pp.push(pb2);
                    i += 1;
                }

                piece_pts = new_pp;

                if !refined {
                    break;
                }
            }

            pts3 = piece_pts.iter().map(|p| eval3_q(p)).collect();

            let fit_track = |pts2: &[Point], fit_tol_track: f64| -> NurbsCurve {
                let mp = pts2.len();
                let mut total_turning = 0.0f64;

                for i in 1..(mp.saturating_sub(1)) {
                    let dx1 = pts2[i][0] - pts2[i - 1][0];
                    let dy1 = pts2[i][1] - pts2[i - 1][1];
                    let dz1 = pts2[i][2] - pts2[i - 1][2];
                    let dx2 = pts2[i + 1][0] - pts2[i][0];
                    let dy2 = pts2[i + 1][1] - pts2[i][1];
                    let dz2 = pts2[i + 1][2] - pts2[i][2];
                    let l1 = (dx1 * dx1 + dy1 * dy1 + dz1 * dz1).sqrt();
                    let l2 = (dx2 * dx2 + dy2 * dy2 + dz2 * dz2).sqrt();

                    if l1 > 1e-14 && l2 > 1e-14 {
                        let c = ((dx1 * dx2 + dy1 * dy2 + dz1 * dz2) / (l1 * l2)).clamp(-1.0, 1.0);
                        total_turning += c.acos();
                    }
                }

                let mut chords = vec![0.0f64; mp];
                let mut total_len = 0.0f64;

                for i in 1..mp {
                    total_len += pts2[i].distance(&pts2[i - 1], None);
                    chords[i] = total_len;
                }

                if piece_loop && mp > 1 {
                    total_len += pts2[0].distance(&pts2[mp - 1], None);
                }

                if total_len > 1e-14 {
                    for chord in chords.iter_mut().skip(1) {
                        *chord /= total_len;
                    }
                }

                let mut target_cvs = 8_i32.max((total_turning / 0.5) as i32 + 6);
                let max_cvs = 8_i32.max(((mp as i32) - 1).min((mp / 3) as i32));
                let mut best = NurbsCurve::new(3, false, 4, 0);
                let mut best_dev = f64::INFINITY;

                while target_cvs <= max_cvs {
                    let crv = NurbsCurve::create_fitted(pts2, target_cvs as usize, 3, piece_loop);

                    if !crv.is_valid() {
                        break;
                    }

                    let (ft0, ft1) = crv.domain();
                    let mut dev = 0.0f64;

                    for i in 0..mp {
                        let t = ft0 + (ft1 - ft0) * chords[i];
                        dev = dev.max(crv.point_at(t).distance(&pts2[i], None));
                    }

                    if dev < best_dev {
                        best = crv;
                        best_dev = dev;
                    }

                    if dev < fit_tol_track {
                        break;
                    }

                    target_cvs *= 2;
                }

                if best_dev >= fit_tol_track {
                    let interp = if piece_loop {
                        NurbsCurve::create_interpolated(
                            pts2,
                            CurveNurbsKnotStyle::ChordPeriodic,
                            CurveInterpStyle::Rhino,
                        )
                    } else {
                        NurbsCurve::create_interpolated(
                            pts2,
                            CurveNurbsKnotStyle::Chord,
                            CurveInterpStyle::Rhino,
                        )
                    };

                    if interp.is_valid() {
                        best = interp;
                    }
                }

                if best.is_valid() {
                    best.set_domain(0.0, 1.0);
                }

                best
            };

            let pts3_p: Vec<Point> = pts3.iter().map(|p| Point::new(p[0], p[1], p[2])).collect();
            let pts_pa: Vec<Point> = piece_pts
                .iter()
                .map(|p| Point::new(p[0], p[1], 0.0))
                .collect();

            let pts_pb: Vec<Point> = piece_pts
                .iter()
                .map(|p| Point::new(p[2], p[3], 0.0))
                .collect();

            let crv3 = fit_track(&pts3_p, (tolerance * 10.0).max(1e-7));
            let pcurve_a = fit_track(&pts_pa, a_du.min(a_dv) * 1e-4);
            let pcurve_b = fit_track(&pts_pb, b_du.min(b_dv) * 1e-4);

            if !crv3.is_valid() || !pcurve_a.is_valid() || !pcurve_b.is_valid() {
                continue;
            }

            result.push((crv3, pcurve_a, pcurve_b));
        }
    }

    result
}

/// Keep the pcurve sub-segments whose lifted 3D point lies inside the cutter footprint.
fn clip_pcurve_to_cutter(
    target: &NurbsSurface,
    pc: &NurbsCurve,
    cutter: &NurbsSurface,
) -> Vec<NurbsCurve> {
    let n = (pc.cv_count() * 4).max(16);
    let (d0, d1) = pc.domain();
    let (cu0, cu1) = cutter.domain(0).unwrap_or((0.0, 1.0));
    let (cv0, cv1) = cutter.domain(1).unwrap_or((0.0, 1.0));
    let zero = Point::new(0.0, 0.0, 0.0);
    let c00 = cutter.point_at(cu0, cv0).unwrap_or(zero.clone());
    let c11 = cutter.point_at(cu1, cv1).unwrap_or(zero.clone());
    let on_tol = (1e-7f64).max(c00.distance(&c11, None) * 1e-4);

    let q00 = cutter.point_at(cu0, cv0).unwrap_or(zero.clone());
    let q10 = cutter.point_at(cu1, cv0).unwrap_or(zero.clone());
    let q01 = cutter.point_at(cu0, cv1).unwrap_or(zero.clone());
    let eu = Vector::new(q10[0] - q00[0], q10[1] - q00[1], q10[2] - q00[2]);
    let ev = Vector::new(q01[0] - q00[0], q01[1] - q00[1], q01[2] - q00[2]);
    let eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
    let ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
    let fast_planar = eu2 > 1e-28 && ev2 > 1e-28;
    let gap = |t: f64| -> f64 {
        let uv = pc.point_at(t);
        let p3 = target
            .point_at(uv[0], uv[1])
            .unwrap_or(Point::new(0.0, 0.0, 0.0));

        if fast_planar {
            let (dx, dy, dz) = (p3[0] - q00[0], p3[1] - q00[1], p3[2] - q00[2]);
            let a = ((dx * eu[0] + dy * eu[1] + dz * eu[2]) / eu2).clamp(0.0, 1.0);
            let b = ((dx * ev[0] + dy * ev[1] + dz * ev[2]) / ev2).clamp(0.0, 1.0);
            let cx = q00[0] + a * eu[0] + b * ev[0];
            let cy = q00[1] + a * eu[1] + b * ev[1];
            let cz = q00[2] + a * eu[2] + b * ev[2];

            return ((p3[0] - cx).powi(2) + (p3[1] - cy).powi(2) + (p3[2] - cz).powi(2)).sqrt();
        }

        Closest::surface_point(cutter, &p3, 0.0, 0.0, 0.0, 0.0).2
    };
    let refine = |t_in: f64, t_out: f64| -> f64 {
        let (mut a, mut b) = (t_in, t_out);

        for _ in 0..20 {
            let tm = (a + b) * 0.5;

            if gap(tm) < on_tol {
                a = tm;
            } else {
                b = tm;
            }
        }

        b
    };

    let mut flags: Vec<(f64, bool)> = Vec::with_capacity(n + 1);

    for i in 0..=n {
        let t = d0 + (d1 - d0) * i as f64 / n as f64;
        flags.push((t, gap(t) < on_tol));
    }

    let mut pieces = Vec::new();
    let mut i = 0;

    while i <= n {
        if flags[i].1 {
            let mut j = i;

            while j < n && flags[j + 1].1 {
                j += 1;
            }

            let ta = if i == 0 {
                flags[i].0
            } else {
                refine(flags[i].0, flags[i - 1].0)
            };
            let tb = if j == n {
                flags[j].0
            } else {
                refine(flags[j].0, flags[j + 1].0)
            };

            if tb - ta > (d1 - d0) * 1e-6 {
                let mut piece = pc.duplicate();

                if piece.trim(ta, tb) && piece.is_valid() {
                    pieces.push(piece);
                }
            }

            i = j + 1;
        } else {
            i += 1;
        }
    }

    pieces
}

/// UV pcurves of the cutter's section on the target, clipped to the cutter footprint.
pub fn cut_curves_on_surface(
    target: &NurbsSurface,
    cutter: &NurbsSurface,
    tolerance: Option<f64>,
) -> Vec<NurbsCurve> {
    let cutter_planar = cutter.is_planar(None, 1e-6);
    let rtol = tolerance.unwrap_or(1e-7).max(1e-7) * 1e4;
    let rt = recognize_surface(target, rtol);
    let mut out = Vec::new();

    for tr in surface_surface(target, cutter, tolerance) {
        let c3d = &tr.0;
        let pa_an = rt.as_ref().and_then(|r| analytic_pcurve(target, r, c3d));
        let pcs: Vec<NurbsCurve> = if let Some(pa) = pa_an {
            vec![pa]
        } else if matches!(rt.as_ref(), Some(RecSurf::Sphere(..))) {
            let mut v = analytic_sphere_pullback(target, rt.as_ref().unwrap(), c3d);

            if v.is_empty() {
                v = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance.unwrap_or(0.0));
            }

            if v.is_empty() {
                v.push(tr.1.clone());
            }

            v
        } else {
            let mut v = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance.unwrap_or(0.0));

            if v.is_empty() {
                v.push(tr.1.clone());
            }

            v
        };

        for pc in pcs {
            if cutter_planar {
                out.extend(clip_pcurve_to_cutter(target, &pc, cutter));
            } else {
                out.push(pc);
            }
        }
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// Polylines and plane sets
// ═══════════════════════════════════════════════════════════════════════════

/// Whether two vectors are parallel within angle_tol.
fn vectors_nearly_parallel(v0: &Vector, v1: &Vector, angle_tol: f64) -> bool {
    let m0 = v0.magnitude();
    let m1 = v1.magnitude();

    if m0 < Tolerance::ZERO_TOLERANCE || m1 < Tolerance::ZERO_TOLERANCE {
        return false;
    }

    let cos_angle = v0.dot(v1) / (m0 * m1);

    cos_angle.abs() >= angle_tol.cos()
}

/// Three-plane intersection that rejects near-parallel pairs.
pub fn plane_plane_plane_check(
    p0: &Plane,
    p1: &Plane,
    p2: &Plane,
    angle_tol: f64,
) -> Option<Point> {
    if vectors_nearly_parallel(&p0.z_axis(), &p1.z_axis(), angle_tol) {
        return None;
    }

    if vectors_nearly_parallel(&p0.z_axis(), &p2.z_axis(), angle_tol) {
        return None;
    }

    if vectors_nearly_parallel(&p1.z_axis(), &p2.z_axis(), angle_tol) {
        return None;
    }

    plane_plane_plane(p0, p1, p2)
}

/// Linear remap of val from [from1, to1] to [from2, to2].
pub fn remap(val: f64, from1: f64, to1: f64, from2: f64, to2: f64) -> f64 {
    let span = to1 - from1;

    if span.abs() < Tolerance::ZERO_TOLERANCE {
        return from2;
    }

    let t = (val - from1) / span;

    from2 + t * (to2 - from2)
}

/// Closest point on a finite segment and its parameter in [0, 1].
pub fn closest_point_on_segment(pt: &Point, seg: &Line) -> (Point, f64) {
    let start = seg.start();
    let end = seg.end();
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let dz = end[2] - start[2];
    let len_sq = dx * dx + dy * dy + dz * dz;

    if len_sq < 1e-20 {
        return (start, 0.0);
    }

    let vx = pt[0] - start[0];
    let vy = pt[1] - start[1];
    let vz = pt[2] - start[2];
    let t = clamp_unit((vx * dx + vy * dy + vz * dz) / len_sq);

    (
        Point::new(start[0] + t * dx, start[1] + t * dy, start[2] + t * dz),
        t,
    )
}

/// Closed quad of the main plane cut by four ordered boundary planes.
pub fn plane_4planes(main: &Plane, planes: &[Plane; 4]) -> Option<Polyline> {
    let p0 = plane_plane_plane_check(&planes[0], &planes[1], main, 0.1)?;
    let p1 = plane_plane_plane_check(&planes[1], &planes[2], main, 0.1)?;
    let p2 = plane_plane_plane_check(&planes[2], &planes[3], main, 0.1)?;
    let p3 = plane_plane_plane_check(&planes[3], &planes[0], main, 0.1)?;

    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Open four-point polyline of the main plane cut by four ordered boundary planes.
pub fn plane_4planes_open(main: &Plane, planes: &[Plane; 4]) -> Option<Polyline> {
    let mut corners: Vec<Point> = Vec::with_capacity(4);

    for i in 0..4 {
        let edge = plane_plane_to_line_canonical(&planes[i], &planes[(i + 1) % 4])?;
        corners.push(line_plane(&edge, main, false)?);
    }

    Some(Polyline::new(corners))
}

/// Closed quad of a plane cut by four infinite lines.
pub fn plane_4lines(plane: &Plane, l0: &Line, l1: &Line, l2: &Line, l3: &Line) -> Option<Polyline> {
    let p0 = line_plane(l0, plane, false)?;
    let p1 = line_plane(l1, plane, false)?;
    let p2 = line_plane(l2, plane, false)?;
    let p3 = line_plane(l3, plane, false)?;

    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Clips a segment to the two plane intersections.
pub fn line_two_planes(line: &Line, p0: &Plane, p1: &Plane) -> Option<Line> {
    let q0 = line_plane(line, p0, true)?;
    let q1 = line_plane(line, p1, true)?;

    Some(Line::new(q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]))
}

/// Polyline edge crossings with a plane and their edge indices.
pub fn polyline_plane(poly: &Polyline, plane: &Plane) -> Option<(Vec<Point>, Vec<usize>)> {
    let n = poly.point_count();

    if n < 2 {
        return None;
    }

    let mut points: Vec<Point> = Vec::new();
    let mut edge_ids: Vec<usize> = Vec::new();

    for i in 0..n - 1 {
        let a = poly.get_point(i)?;
        let b = poly.get_point(i + 1)?;
        let va = plane_value_at(plane, &a);
        let vb = plane_value_at(plane, &b);
        let a_on = va.abs() < Tolerance::ZERO_TOLERANCE;
        let b_on = vb.abs() < Tolerance::ZERO_TOLERANCE;

        if a_on && b_on {
            continue;
        }

        if a_on {
            points.push(a);
            edge_ids.push(i);
            continue;
        }

        if b_on {
            if i + 2 == n {
                let front = poly.get_point(0)?;
                let closes = (b[0] - front[0]).abs() < Tolerance::ZERO_TOLERANCE
                    && (b[1] - front[1]).abs() < Tolerance::ZERO_TOLERANCE
                    && (b[2] - front[2]).abs() < Tolerance::ZERO_TOLERANCE;

                if !closes {
                    points.push(b);
                    edge_ids.push(i);
                }
            }

            continue;
        }

        let seg = Line::new(a[0], a[1], a[2], b[0], b[1], b[2]);

        if let Some(hit) = line_plane(&seg, plane, true) {
            points.push(hit);
            edge_ids.push(i);
        }
    }

    if points.is_empty() {
        return None;
    }

    Some((points, edge_ids))
}

/// Closest approach point on the infinite cutter to the segment.
pub fn line_line_3d(cutter: &Line, seg: &Line) -> Option<Point> {
    let (t0, _) = line_line_parameters(cutter, seg, 0.0, false, false)?;

    Some(cutter.point_at(t0))
}

/// Direction scaled to span the distance between two planes.
pub fn scale_vector_to_distance_of_2planes(dir: &Vector, p0: &Plane, p1: &Plane) -> Option<Vector> {
    if dir.magnitude() < Tolerance::ZERO_TOLERANCE {
        return None;
    }

    let origin = Point::new(0.0, 0.0, 0.0);
    let tip = Point::new(dir[0], dir[1], dir[2]);
    let ray = Line::new(origin[0], origin[1], origin[2], tip[0], tip[1], tip[2]);
    let q0 = line_plane(&ray, p0, false)?;
    let q1 = line_plane(&ray, p1, false)?;
    let output = &q1 - &q0;
    let n1 = p1.z_axis();
    let n1_mag = n1.magnitude();

    if n1_mag < Tolerance::ZERO_TOLERANCE {
        return None;
    }

    let o0 = p0.origin();
    let d = (o0[0] - p1.origin()[0]) * n1[0] / n1_mag
        + (o0[1] - p1.origin()[1]) * n1[1] / n1_mag
        + (o0[2] - p1.origin()[2]) * n1[2] / n1_mag;

    let dist_ortho_sq = d * d;

    if dist_ortho_sq < Tolerance::ZERO_TOLERANCE {
        return None;
    }

    let dist_sq = output.dot(&output);

    if dist_sq / dist_ortho_sq >= 10.0 {
        return None;
    }

    Some(output)
}

// ═══════════════════════════════════════════════════════════════════════════
// Plane 2D helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Projects a point into plane coordinates.
fn plane_to_2d(p: &Point, origin: &Point, xax: &Vector, yax: &Vector) -> [f64; 2] {
    let d = p - origin;

    [d.dot(xax), d.dot(yax)]
}

/// Lift plane coordinates back to a point.
fn plane_to_3d(p: &[f64; 2], origin: &Point, xax: &Vector, yax: &Vector) -> Point {
    origin + xax * p[0] + yax * p[1]
}

/// Squared distance of two 2D points.
fn distance_sq_2d(a: &[f64; 2], b: &[f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];

    dx * dx + dy * dy
}

/// Signed area of a 2D ring, positive when counter-clockwise.
fn signed_area_2d(ring: &[[f64; 2]]) -> f64 {
    let mut area = 0.0;
    let n = ring.len();

    for i in 0..n {
        area += ring[i][0] * ring[(i + 1) % n][1] - ring[(i + 1) % n][0] * ring[i][1];
    }

    area
}

/// Projects a polyline into plane coordinates.
fn polyline_to_2d(
    polyline: &Polyline,
    origin: &Point,
    xax: &Vector,
    yax: &Vector,
) -> Vec<[f64; 2]> {
    let mut ring: Vec<[f64; 2]> = Vec::with_capacity(polyline.point_count());

    for i in 0..polyline.point_count() {
        ring.push(plane_to_2d(
            &polyline.get_point(i).unwrap(),
            origin,
            xax,
            yax,
        ));
    }

    if ring.len() > 1 && distance_sq_2d(&ring[0], ring.last().unwrap()) < 1e-12 {
        ring.pop();
    }

    ring
}

/// Lift a 2D ring back to a polyline.
fn polyline_to_3d(ring: &[[f64; 2]], origin: &Point, xax: &Vector, yax: &Vector) -> Polyline {
    let mut pts: Vec<Point> = Vec::with_capacity(ring.len() + 1);

    for p in ring {
        pts.push(plane_to_3d(p, origin, xax, yax));
    }

    pts.push(pts[0].clone());

    Polyline::new(pts)
}

/// Even-odd point in polygon test.
fn point_in_polygon_2d(ring: &[[f64; 2]], p: &[f64; 2]) -> bool {
    let mut wn = 0i32;
    let n = ring.len();

    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        let e = (b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1]);

        if a[1] <= p[1] && b[1] > p[1] && e > 0.0 {
            wn += 1;
        } else if a[1] > p[1] && b[1] <= p[1] && e < 0.0 {
            wn -= 1;
        }
    }

    wn != 0
}

/// Segment-segment crossing with parameters on both.
fn seg_seg_2d(s0: &[f64; 2], s1: &[f64; 2], e0: &[f64; 2], e1: &[f64; 2]) -> Option<(f64, f64)> {
    let sx = s1[0] - s0[0];
    let sy = s1[1] - s0[1];
    let ex = e1[0] - e0[0];
    let ey = e1[1] - e0[1];
    let denom = sx * ey - sy * ex;

    if denom.abs() < 1e-20 {
        return None;
    }

    let dx = e0[0] - s0[0];
    let dy = e0[1] - s0[1];

    Some(((dx * ey - dy * ex) / denom, (dx * sy - dy * sx) / denom))
}

/// Overlap range of two collinear segments on the first.
fn collinear_overlap_2d(
    s0: &[f64; 2],
    s1: &[f64; 2],
    e0: &[f64; 2],
    e1: &[f64; 2],
) -> Option<(f64, f64)> {
    let sx = s1[0] - s0[0];
    let sy = s1[1] - s0[1];
    let ex = e1[0] - e0[0];
    let ey = e1[1] - e0[1];
    let sl2 = sx * sx + sy * sy;
    let el2 = ex * ex + ey * ey;

    if sl2 < 1e-20 || el2 < 1e-20 {
        return None;
    }

    if ((sx * ey - sy * ex) / (sl2 * el2).sqrt()).abs() > 1e-4 {
        return None;
    }

    let apx = s0[0] - e0[0];
    let apy = s0[1] - e0[1];

    if ((apx * ey - apy * ex) / el2.sqrt()).abs() > 1e-3 {
        return None;
    }

    let ts0 = (apx * ex + apy * ey) / el2;
    let ts1 = ((s1[0] - e0[0]) * ex + (s1[1] - e0[1]) * ey) / el2;
    let ov_min = ts0.min(ts1).max(0.0);
    let ov_max = ts0.max(ts1).min(1.0);

    if ov_max - ov_min < 1e-9 {
        return None;
    }

    let tsr = ts1 - ts0;

    if tsr.abs() < 1e-20 {
        return None;
    }

    let t_enter = ((ov_min - ts0) / tsr).min((ov_max - ts0) / tsr).max(0.0);
    let t_exit = ((ov_min - ts0) / tsr).max((ov_max - ts0) / tsr).min(1.0);

    if t_exit - t_enter <= 1e-9 {
        return None;
    }

    Some((t_enter, t_exit))
}

/// Parameter of the closest point on segment ab to p.
fn closest_param_2d(p: &[f64; 2], a: &[f64; 2], b: &[f64; 2]) -> f64 {
    let abx = b[0] - a[0];
    let aby = b[1] - a[1];
    let l2 = abx * abx + aby * aby;

    if l2 < 1e-20 {
        return 0.0;
    }

    (((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2).clamp(0.0, 1.0)
}

/// Squared distance from p to segment ab.
fn distance_sq_seg_2d(p: &[f64; 2], a: &[f64; 2], b: &[f64; 2]) -> f64 {
    let t = closest_param_2d(p, a, b);

    distance_sq_2d(p, &[a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])])
}

/// Parameters along one joint segment where it crosses or overlaps the plate edges.
fn segment_plate_parameters_2d(
    plate: &[[f64; 2]],
    p0: &[f64; 2],
    p1: &[f64; 2],
    coll_ranges: &mut Vec<(f64, f64)>,
) -> Vec<f64> {
    const EPS: f64 = 1e-9;
    let mut ts: Vec<f64> = vec![0.0];

    for i in 0..plate.len() {
        let a = plate[i];
        let b = plate[(i + 1) % plate.len()];

        if let Some((t_s, t_e)) = seg_seg_2d(p0, p1, &a, &b) {
            if t_s > EPS && t_s < 1.0 - EPS && (-EPS..=1.0 + EPS).contains(&t_e) {
                ts.push(t_s);
            }
        }

        let overlap = collinear_overlap_2d(p0, p1, &a, &b);

        if overlap.is_none() {
            continue;
        }

        let (t_in, t_out) = overlap.unwrap();
        coll_ranges.push((t_in, t_out));

        if t_in > EPS && t_in < 1.0 - EPS {
            ts.push(t_in);
        }

        if t_out > EPS && t_out < 1.0 - EPS {
            ts.push(t_out);
        }
    }

    ts.push(1.0);
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < EPS);

    ts
}

/// Whether t falls in any of the ranges.
fn in_ranges_2d(ranges: &[(f64, f64)], t: f64) -> bool {
    for r in ranges {
        if t >= r.0 - 1e-9 && t <= r.1 + 1e-9 {
            return true;
        }
    }

    false
}

/// Sub-segments of the open joint path inside the plate, as separate pieces.
fn clip_open_path_2d(plate: &[[f64; 2]], joint: &[[f64; 2]]) -> Vec<Vec<[f64; 2]>> {
    let mut pieces: Vec<Vec<[f64; 2]>> = Vec::new();

    for s in 0..joint.len() - 1 {
        let p0 = joint[s];
        let p1 = joint[s + 1];
        let mut coll_ranges: Vec<(f64, f64)> = Vec::new();
        let ts = segment_plate_parameters_2d(plate, &p0, &p1, &mut coll_ranges);
        let mut current: Vec<[f64; 2]> = Vec::new();

        for i in 0..ts.len() - 1 {
            let t_mid = 0.5 * (ts[i] + ts[i + 1]);
            let mid = [
                p0[0] + (p1[0] - p0[0]) * t_mid,
                p0[1] + (p1[1] - p0[1]) * t_mid,
            ];
            let include = point_in_polygon_2d(plate, &mid) || in_ranges_2d(&coll_ranges, t_mid);

            if !include {
                if !current.is_empty() {
                    pieces.push(std::mem::take(&mut current));
                }

                current.clear();
                continue;
            }

            let sub_a = [
                p0[0] + (p1[0] - p0[0]) * ts[i],
                p0[1] + (p1[1] - p0[1]) * ts[i],
            ];
            let sub_b = [
                p0[0] + (p1[0] - p0[0]) * ts[i + 1],
                p0[1] + (p1[1] - p0[1]) * ts[i + 1],
            ];

            if !current.is_empty() && distance_sq_2d(current.last().unwrap(), &sub_a) >= 1e-18 {
                pieces.push(std::mem::take(&mut current));
                current.clear();
            }

            if current.is_empty() {
                current.push(sub_a);
            }

            current.push(sub_b);
        }

        if !current.is_empty() {
            pieces.push(current);
        }
    }

    pieces
}

/// Chains clipped pieces end to end into one path.
fn chain_pieces_2d(pieces: &[Vec<[f64; 2]>]) -> Vec<[f64; 2]> {
    const DISTANCE_SQ: f64 = 0.01;
    let mut chain: Vec<[f64; 2]> = Vec::new();

    for piece in pieces {
        if piece.len() <= 1 {
            continue;
        }

        if chain.is_empty() {
            chain = piece.clone();
            continue;
        }

        let mut pts = piece.clone();

        if distance_sq_2d(chain.last().unwrap(), &pts[0]) > DISTANCE_SQ
            && distance_sq_2d(chain.last().unwrap(), pts.last().unwrap()) > DISTANCE_SQ
        {
            chain.reverse();
        }

        if distance_sq_2d(chain.last().unwrap(), &pts[0])
            > distance_sq_2d(chain.last().unwrap(), pts.last().unwrap())
        {
            pts.reverse();
        }

        for p in pts.iter().skip(1) {
            chain.push(*p);
        }
    }

    chain
}

/// Plate edge parameters of the chain ends, or -1 when an end is off the plate.
fn chain_plate_parameters_2d(plate: &[[f64; 2]], chain: &[[f64; 2]]) -> (f64, f64) {
    let mut t0 = -1.0_f64;
    let mut t1 = -1.0_f64;

    for i in 0..plate.len() {
        let a = plate[i];
        let b = plate[(i + 1) % plate.len()];

        if distance_sq_seg_2d(&chain[0], &a, &b) < 1.0 {
            t0 = i as f64 + closest_param_2d(&chain[0], &a, &b);
        }

        if distance_sq_seg_2d(chain.last().unwrap(), &a, &b) < 1.0 {
            t1 = i as f64 + closest_param_2d(chain.last().unwrap(), &a, &b);
        }

        if t0 >= 0.0 && t1 >= 0.0 {
            return (t0, t1);
        }
    }

    (t0, t1)
}

/// Miter offset of a closed 2D ring by delta along the edge normals.
fn offset_ring_2d(ring: &[[f64; 2]], delta: f64, concave_notch: bool) -> Vec<[f64; 2]> {
    let n = ring.len();
    let mut normals: Vec<[f64; 2]> = Vec::with_capacity(n);

    for i in 0..n {
        let ex = ring[(i + 1) % n][0] - ring[i][0];
        let ey = ring[(i + 1) % n][1] - ring[i][1];
        let len = (ex * ex + ey * ey).sqrt();

        if len < 1e-12 {
            normals.push([0.0, 0.0]);
        } else {
            normals.push([ey / len, -ex / len]);
        }
    }

    let mut out: Vec<[f64; 2]> = Vec::with_capacity(n * 3);

    for i in 0..n {
        let np = normals[(i + n - 1) % n];
        let nn = normals[i];
        let p = ring[i];
        let cos_a = np[0] * nn[0] + np[1] * nn[1];
        let sin_a = np[0] * nn[1] - np[1] * nn[0];
        let denom = 1.0 + cos_a;

        if cos_a > -0.999 && sin_a * delta < 0.0 && concave_notch {
            out.push([p[0] + np[0] * delta, p[1] + np[1] * delta]);
            out.push(p);
            out.push([p[0] + nn[0] * delta, p[1] + nn[1] * delta]);
        } else if denom.abs() < 1e-9 {
            let bx = np[0] + nn[0];
            let by = np[1] + nn[1];
            let bl = (bx * bx + by * by).sqrt();

            if bl < 1e-12 {
                out.push([p[0] + nn[0] * delta, p[1] + nn[1] * delta]);
            } else {
                out.push([p[0] + (bx / bl) * delta, p[1] + (by / bl) * delta]);
            }
        } else {
            let k = delta / denom;
            out.push([p[0] + (np[0] + nn[0]) * k, p[1] + (np[1] + nn[1]) * k]);
        }
    }

    out
}

// ═══════════════════════════════════════════════════════════════════════════
// Polyline booleans
// ═══════════════════════════════════════════════════════════════════════════

/// Boolean of two closed planar polylines, clip_type 0 intersection, 1 union, 2 difference.
pub fn polyline_boolean(a: &Polyline, b: &Polyline, clip_type: i32) -> Vec<Polyline> {
    Polyline::boolean_op(a, b, clip_type, None)
}

/// Miter offset of a closed polyline in the plane's 2D frame, positive outward, in place.
pub fn offset_in_3d(polyline: &mut Polyline, plane: &Plane, offset: f64) -> bool {
    if polyline.point_count() < 3 {
        return false;
    }

    let origin = polyline.get_point(0).unwrap();
    let xax = plane.base1();
    let yax = plane.base2();
    let ring = polyline_to_2d(polyline, &origin, &xax, &yax);

    if ring.len() < 3 {
        return false;
    }

    let delta = if signed_area_2d(&ring) < 0.0 {
        -offset
    } else {
        offset
    };
    let mut out = offset_ring_2d(&ring, delta, offset > 0.0);

    if out.len() < 3 {
        return false;
    }

    if signed_area_2d(&out).abs() * 0.5 < 0.0001 {
        return false;
    }

    let mut cp = 0usize;

    for i in 1..out.len() {
        if distance_sq_2d(&out[i], &ring[0]) < distance_sq_2d(&out[cp], &ring[0]) {
            cp = i;
        }
    }

    out.rotate_left(cp);
    *polyline = polyline_to_3d(&out, &origin, &xax, &yax);

    true
}

/// Boolean in the plane's 2D frame, intersection_type 0 intersect, 1 union, 2 difference, 3 xor.
pub fn polyline_boolean_2d_in_plane(
    polyline0: &Polyline,
    polyline1: &Polyline,
    plane: &Plane,
    intersection_type: i32,
    include_triangles: bool,
    min_area: f64,
    collapse_eps: f64,
) -> Option<Polyline> {
    if polyline0.point_count() < 3 || polyline1.point_count() < 3 {
        return None;
    }

    let origin = polyline0.get_point(0)?;
    let xax = plane.base1();
    let yax = plane.base2();
    let flat_origin = Point::new(0.0, 0.0, 0.0);
    let flat_x = Vector::new(1.0, 0.0, 0.0);
    let flat_y = Vector::new(0.0, 1.0, 0.0);
    let a2d = polyline_to_3d(
        &polyline_to_2d(polyline0, &origin, &xax, &yax),
        &flat_origin,
        &flat_x,
        &flat_y,
    );
    let b2d = polyline_to_3d(
        &polyline_to_2d(polyline1, &origin, &xax, &yax),
        &flat_origin,
        &flat_x,
        &flat_y,
    );
    let result_2d: Vec<Polyline> = if (0..=2).contains(&intersection_type) {
        Polyline::boolean_op(&a2d, &b2d, intersection_type, None)
    } else if intersection_type == 3 {
        let u = Polyline::boolean_op(&a2d, &b2d, 1, None);
        let inter = Polyline::boolean_op(&a2d, &b2d, 0, None);

        if u.is_empty() {
            return None;
        }

        if inter.is_empty() {
            u
        } else {
            Polyline::boolean_op(&u[0], &inter[0], 2, None)
        }
    } else {
        return None;
    };

    if result_2d.is_empty() {
        return None;
    }

    let mut ring = polyline_to_2d(&result_2d[0], &flat_origin, &flat_x, &flat_y);

    if ring.len() < 3 {
        return None;
    }

    if collapse_eps > 0.0 {
        let eps_sq = collapse_eps * collapse_eps;
        let mut collapsed: Vec<[f64; 2]> = Vec::with_capacity(ring.len());

        for p in &ring {
            if collapsed.is_empty() || distance_sq_2d(p, collapsed.last().unwrap()) >= eps_sq {
                collapsed.push(*p);
            }
        }

        if collapsed.len() >= 2 && distance_sq_2d(collapsed.last().unwrap(), &collapsed[0]) < eps_sq
        {
            collapsed.pop();
        }

        ring = collapsed;

        if ring.len() < 3 {
            return None;
        }
    }

    if ring.len() == 3 && !include_triangles {
        return None;
    }

    if signed_area_2d(&ring).abs() * 0.5 <= min_area {
        return None;
    }

    Some(polyline_to_3d(&ring, &origin, &xax, &yax))
}

// ═══════════════════════════════════════════════════════════════════════════
// Joints
// ═══════════════════════════════════════════════════════════════════════════

/// Polyline-plane crossings as one line oriented from align_start.
pub fn polyline_plane_to_line(poly: &Polyline, plane: &Plane, align_start: &Point) -> Option<Line> {
    let (pts, _) = polyline_plane(poly, plane)?;

    if pts.len() < 2 {
        return None;
    }

    let mut ia = 0usize;
    let mut ib = 1usize;
    let mut best = -1.0_f64;

    for p1 in 0..pts.len() - 1 {
        for p2 in p1 + 1..pts.len() {
            let d = (&pts[p1] - &pts[p2]).magnitude_squared();

            if d > best {
                best = d;
                ia = p1;
                ib = p2;
            }
        }
    }

    let a = &pts[ia];
    let b = &pts[ib];

    if (a - align_start).magnitude_squared() <= (b - align_start).magnitude_squared() {
        Some(Line::from_points(a, b))
    } else {
        Some(Line::from_points(b, a))
    }
}

/// Closed quad from a joint line bounded by top and bottom planes on a face plane.
pub fn quad_from_line_top_bottom_planes(
    face_plane: &Plane,
    line: &Line,
    plane0: &Plane,
    plane1: &Plane,
) -> Option<Polyline> {
    let lp0 = Plane::from_point_normal(line.start(), line.to_vector(), None);
    let lp1 = Plane::from_point_normal(line.end(), line.to_vector(), None);
    let edge0 = plane_plane(plane0, face_plane)?;
    let edge1 = plane_plane(plane1, face_plane)?;
    let p0 = line_plane(&edge0, &lp0, false)?;
    let p1 = line_plane(&edge1, &lp0, false)?;
    let p2 = line_plane(&edge1, &lp1, false)?;
    let p3 = line_plane(&edge0, &lp1, false)?;

    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Vector orthogonal to the (pp00, pp10) line, anchored on the (pp00, pp11) line.
pub fn orthogonal_vector_between_two_plane_pairs(
    pp00: &Plane,
    pp10: &Plane,
    pp11: &Plane,
) -> Option<Vector> {
    let l0 = plane_plane_to_line_canonical(pp00, pp10)?;
    let l1 = plane_plane_to_line_canonical(pp00, pp11)?;

    if l0.to_vector().magnitude_squared() < 1e-20 {
        return None;
    }

    Some(&l1.start() - &l0.closest_point(&l1.start(), false).1)
}

/// Open joint outline clipped to a closed plate polygon with the plate edge parameters.
pub fn closed_and_open_paths_2d(
    plate: &Polyline,
    joint: &Polyline,
    plane: &Plane,
) -> Option<(Polyline, (f64, f64))> {
    let origin = plate.get_point(0)?;
    let xax = plane.base1();
    let yax = plane.base2();
    let plate2d = polyline_to_2d(plate, &origin, &xax, &yax);

    if plate2d.len() < 3 {
        return None;
    }

    let mut joint2d: Vec<[f64; 2]> = Vec::with_capacity(joint.point_count());

    for i in 0..joint.point_count() {
        joint2d.push(plane_to_2d(&joint.get_point(i)?, &origin, &xax, &yax));
    }

    if joint2d.len() < 2 {
        return None;
    }

    let mut c2d = chain_pieces_2d(&clip_open_path_2d(&plate2d, &joint2d));

    if c2d.len() < 2 {
        return None;
    }

    let (mut t0, mut t1) = chain_plate_parameters_2d(&plate2d, &c2d);
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

    let mut out_pts: Vec<Point> = Vec::with_capacity(c2d.len());

    for p in &c2d {
        out_pts.push(plane_to_3d(p, &origin, &xax, &yax));
    }

    Some((Polyline::new(out_pts), (t0, t1)))
}

// ═══════════════════════════════════════════════════════════════════════════
// Elements
// ═══════════════════════════════════════════════════════════════════════════

/// Face-to-face contacts (a, b, face_a, face_b, type, polyline) with type 0 side-side, 1 side-top, 2 top-top.
pub fn face_to_face(
    adjacency: &[i32],
    polylines: &[Vec<Polyline>],
    planes: &[Vec<Plane>],
    coplanar_tolerance: f64,
) -> Vec<(i32, i32, i32, i32, i32, Polyline)> {
    let mut results = Vec::new();
    let mut face_boxes: Vec<Vec<[f64; 6]>> = Vec::with_capacity(polylines.len());

    for faces in polylines {
        let mut boxes: Vec<[f64; 6]> = Vec::with_capacity(faces.len());

        for f in faces {
            let mut bx = [
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            let c = &f.coords;
            let mut k = 0;

            while k + 2 < c.len() {
                bx[0] = bx[0].min(c[k]);
                bx[3] = bx[3].max(c[k]);
                bx[1] = bx[1].min(c[k + 1]);
                bx[4] = bx[4].max(c[k + 1]);
                bx[2] = bx[2].min(c[k + 2]);
                bx[5] = bx[5].max(c[k + 2]);
                k += 3;
            }

            for k in 0..3 {
                bx[k] -= coplanar_tolerance;
                bx[k + 3] += coplanar_tolerance;
            }

            boxes.push(bx);
        }

        face_boxes.push(boxes);
    }

    let mut idx = 0;

    while idx + 1 < adjacency.len() {
        let a = adjacency[idx] as usize;
        let b = adjacency[idx + 1] as usize;
        let mut found = false;
        let mut i = 0;

        while i < planes[a].len() && !found {
            let oa = planes[a][i].origin();
            let za = planes[a][i].z_axis();
            let ba = &face_boxes[a][i];

            for j in 0..planes[b].len() {
                let bb = &face_boxes[b][j];

                if ba[0] > bb[3]
                    || bb[0] > ba[3]
                    || ba[1] > bb[4]
                    || bb[1] > ba[4]
                    || ba[2] > bb[5]
                    || bb[2] > ba[5]
                {
                    continue;
                }

                if !Plane::is_coplanar_from_normals(
                    &oa,
                    &za,
                    &planes[b][j].origin(),
                    &planes[b][j].z_axis(),
                    false,
                    coplanar_tolerance,
                ) {
                    continue;
                }

                let pts_i = polylines[a][i].get_points();
                let mut edge = Vector::new(
                    pts_i[1][0] - pts_i[0][0],
                    pts_i[1][1] - pts_i[0][1],
                    pts_i[1][2] - pts_i[0][2],
                );
                edge.normalize_self();
                let zax = za.clone();
                let mut yax = zax.cross(&edge);
                yax.normalize_self();
                let pln = Plane::from_frame(pts_i[0].clone(), edge, yax, zax);
                let bools = Polyline::boolean_op(&polylines[a][i], &polylines[b][j], 0, Some(&pln));

                if bools.is_empty() || bools[0].point_count() < 3 {
                    continue;
                }

                let typ = (if i > 1 { 0 } else { 1 }) + (if j > 1 { 0 } else { 1 });
                let jpl = if bools[0].is_closed() {
                    bools[0].clone()
                } else {
                    bools[0].closed()
                };
                results.push((a as i32, b as i32, i as i32, j as i32, typ, jpl));
                found = true;
                break;
            }

            i += 1;
        }

        idx += 4;
    }

    results
}

/// Adjacent element pairs by BVH broad phase and OBB narrow phase.
pub fn adjacency_search(elements: &mut [Element], inflate: f64) -> Vec<i32> {
    let n = elements.len();
    let mut obbs: Vec<OBB> = Vec::with_capacity(n);

    for element in elements.iter_mut() {
        let mut pts: Vec<Point> = Vec::new();

        for pl in element.polylines() {
            for p in pl.get_points() {
                pts.push(p);
            }
        }

        obbs.push(OBB::from_points(&pts, inflate, None));
    }

    let mut aabbs: Vec<AABB> = Vec::with_capacity(n);

    for obb in &obbs {
        aabbs.push(obb.aabb());
    }

    let mut ws = 0.0_f64;

    for a in &aabbs {
        ws = ws.max((a.cx + a.hx).abs());
        ws = ws.max((a.cy + a.hy).abs());
        ws = ws.max((a.cz + a.hz).abs());
        ws = ws.max((a.cx - a.hx).abs());
        ws = ws.max((a.cy - a.hy).abs());
        ws = ws.max((a.cz - a.hz).abs());
    }

    let mut bvh = SpatialBVH::new();
    bvh.build_from_aabbs(&aabbs, ws * 2.0);
    let mut adjacency: Vec<i32> = Vec::new();

    for i in 0..n {
        let hits = bvh.query_aabb(&aabbs[i]);

        for j in hits {
            if i < j && obbs[i].collides_with(&obbs[j]) {
                adjacency.push(i as i32);
                adjacency.push(j as i32);
                adjacency.push(-1);
                adjacency.push(-1);
            }
        }
    }

    adjacency
}

/// Classifies two segments as end-to-end, side-to-end or cross with closest points and directions.
#[allow(clippy::too_many_arguments)]
pub fn line_line_classified(
    s0: &Line,
    s1: &Line,
    n_segs_0: i32,
    n_segs_1: i32,
    cur_seg_0: i32,
    cur_seg_1: i32,
    above_closer_to_edge: f64,
    p0: &mut Point,
    p1: &mut Point,
    v0: &mut Vector,
    v1: &mut Vector,
    normal: &mut Vector,
    type0: &mut bool,
    type1: &mut bool,
    is_parallel: &mut bool,
) -> bool {
    const DIST_SQ: f64 = 1e-6;
    const EPS_PAR: f64 = 1.0;
    *v0 = s0.to_vector();
    *v1 = s1.to_vector();
    *normal = v0.cross(v1);
    let ang = v0.angle(v1, false, true, None);
    *is_parallel = normal.magnitude_squared() < 1e-24 || (90.0 - (ang - 90.0).abs()) < EPS_PAR;

    if *is_parallel {
        *normal = Plane::from_point_normal(s0.start(), v0.clone(), None).base1();
    }

    normal.normalize_self();
    let ends0 = [s0.start(), s0.end()];
    let ends1 = [s1.start(), s1.end()];

    for i in 0..2 {
        for j in 0..2 {
            if (&ends0[i] - &ends1[j]).magnitude_squared() >= DIST_SQ {
                continue;
            }

            *p0 = ends0[i].clone();
            *p1 = ends0[i].clone();
            *v0 = &ends0[1 - i] - &ends0[i];
            *v1 = &ends1[1 - j] - &ends1[j];
            v0.normalize_self();
            v1.normalize_self();
            *type0 = false;
            *type1 = false;

            return true;
        }
    }

    v0.normalize_self();
    v1.normalize_self();

    if *is_parallel {
        let mut pts: Vec<(f64, f64)> = Vec::with_capacity(4);

        for q in [s0.start(), s0.end(), s1.start(), s1.end()] {
            let q0 = s0.closest_point(&q, false).1;
            let q1 = s1.closest_point(&q, false).1;
            pts.push(((&q0 - &s0.start()).dot(v0), (&q1 - &s1.start()).dot(v1)));
        }

        pts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let m0 = &s0.start() + &(&*v0 * ((pts[1].0 + pts[2].0) * 0.5));
        let m1 = &s1.start() + &(&*v1 * ((pts[1].1 + pts[2].1) * 0.5));
        let avg = &m0 + &(&(&m1 - &m0) * 0.5);
        *p0 = s0.closest_point(&avg, false).1;
        *p1 = s1.closest_point(&avg, false).1;

        if s0.closest_point(p0, false).0 > 0.5 {
            *v0 = -v0.clone();
        }

        if s1.closest_point(p1, false).0 > 0.5 {
            *v1 = -v1.clone();
        }

        *type0 = false;
        *type1 = false;

        return true;
    }

    let (t0_v, t1_v) = match line_line_parameters(s0, s1, 0.0, false, true) {
        Some(t) => t,
        None => return false,
    };
    let t0c = t0_v.clamp(0.0, 1.0);
    let t1c = t1_v.clamp(0.0, 1.0);
    *p0 = s0.point_at(t0c);
    *p1 = s1.point_at(t1c);
    let tt0 = (t0c + cur_seg_0 as f64) / n_segs_0 as f64;
    let tt1 = (t1c + cur_seg_1 as f64) / n_segs_1 as f64;
    let close0 = 2.0 * (0.5 - tt0).abs();
    let close1 = 2.0 * (0.5 - tt1).abs();

    if above_closer_to_edge < 0.0 {
        *type0 = true;
        *type1 = true;
    } else if above_closer_to_edge > 1.0 {
        *type0 = tt0 >= tt1;
        *type1 = tt0 < tt1;
    } else {
        *type0 = close0 <= above_closer_to_edge;
        *type1 = close1 <= above_closer_to_edge;

        if close0 > close1 && !*type0 && !*type1 {
            *type1 = true;
        } else if close0 < close1 && !*type0 && !*type1 {
            *type0 = true;
        }
    }

    if tt0 > 0.5 && !*type0 {
        *v0 = -v0.clone();
    }

    if tt1 > 0.5 && !*type1 {
        *v1 = -v1.clone();
    }

    true
}
