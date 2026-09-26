#![allow(clippy::needless_range_loop)]

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
use crate::tolerance::PI;
use crate::tolerance::TWO_PI;
use crate::vector::Vector;

// ═══════════════════════════════════════════════════════════════════════════
// Lines and planes
// ═══════════════════════════════════════════════════════════════════════════
/// Largest absolute coefficient of a 3x3 system with its row and column, the first one on ties.
fn max_pivot_3x3(rows: &[[f64; 3]; 3]) -> (f64, usize, usize) {
    let mut temp = rows[0][0].abs();
    let mut i = 0;
    let mut j = 0;

    for r in 0..3 {
        for c in 0..3 {
            let val = rows[r][c].abs();

            if val > temp {
                temp = val;
                i = r;
                j = c;
            }
        }
    }

    (temp, i, j)
}

/// Rows of a 3x3 system in a 3x4 work array, row i swapped to the top.
fn to_work_array(rows: &[[f64; 3]; 3], ds: &[f64; 3], i: usize) -> [f64; 12] {
    let mut w = [0.0f64; 12];
    let mut src = [0usize, 1, 2];
    src.swap(0, i);

    for r in 0..3 {
        for c in 0..3 {
            w[4 * r + c] = rows[src[r]][c];
        }

        w[4 * r + 3] = ds[src[r]];
    }

    w
}

/// Swap two coefficient columns in all rows of the 3x4 work array, and the unknowns they solve for.
fn swap_columns(w: &mut [f64; 12], slot: &mut [usize; 3], c0: usize, c1: usize) {
    for r in 0..3 {
        w.swap(4 * r + c0, 4 * r + c1);
    }

    slot.swap(c0, c1);
}

/// Scale the top row to a unit pivot and clear the first column of the rows below.
fn eliminate_first_column(w: &mut [f64; 12]) {
    let mut temp = 1.0 / w[0];
    w[1] *= temp;
    w[2] *= temp;
    w[3] *= temp;

    for r in [4, 8] {
        temp = -w[r];

        if temp != 0.0 {
            for c in 1..4 {
                w[r + c] += temp * w[c];
            }
        }
    }
}

/// Largest absolute coefficient of the lower-right 2x2 block with its row and column, the first one on ties.
fn max_pivot_2x2(w: &[f64; 12]) -> (f64, usize, usize) {
    let mut temp = w[5].abs();
    let mut i = 0;
    let mut j = 0;

    for r in 0..2 {
        for c in 0..2 {
            let val = w[5 + 4 * r + c].abs();

            if val > temp {
                temp = val;
                i = r;
                j = c;
            }
        }
    }

    (temp, i, j)
}

/// Widen the [minpiv, maxpiv] pivot range by val.
fn update_pivot_range(val: f64, maxpiv: &mut f64, minpiv: &mut f64) {
    if val > *maxpiv {
        *maxpiv = val;
    } else if val < *minpiv {
        *minpiv = val;
    }
}

/// Eliminate the second and third columns using the rows at offsets pivot and other; false when the last pivot is zero.
fn eliminate_last_columns(
    w: &mut [f64; 12],
    pivot: usize,
    other: usize,
    maxpiv: &mut f64,
    minpiv: &mut f64,
) -> bool {
    let mut temp = 1.0 / w[pivot + 1];
    w[pivot + 2] *= temp;
    w[pivot + 3] *= temp;
    temp = -w[1];

    if temp != 0.0 {
        w[2] += temp * w[pivot + 2];
        w[3] += temp * w[pivot + 3];
    }

    temp = -w[other + 1];

    if temp != 0.0 {
        w[other + 2] += temp * w[pivot + 2];
        w[other + 3] += temp * w[pivot + 3];
    }

    temp = w[other + 2];

    if temp == 0.0 {
        return false;
    }

    update_pivot_range(temp.abs(), maxpiv, minpiv);
    w[other + 3] /= temp;
    temp = -w[pivot + 2];

    if temp != 0.0 {
        w[pivot + 3] += temp * w[other + 3];
    }

    temp = -w[2];

    if temp != 0.0 {
        w[3] += temp * w[other + 3];
    }

    true
}

/// Gaussian elimination of a 3x3 system with full pivoting: (rank, x, y, z, pivot_ratio).
fn solve_3x3(
    row0: &[f64; 3],
    row1: &[f64; 3],
    row2: &[f64; 3],
    d0: f64,
    d1: f64,
    d2: f64,
) -> (i32, f64, f64, f64, f64) {
    let rows = [*row0, *row1, *row2];
    let (temp, i, j) = max_pivot_3x3(&rows);

    if temp == 0.0 {
        return (0, 0.0, 0.0, 0.0, 0.0);
    }

    let mut maxpiv = temp.abs();
    let mut minpiv = maxpiv;
    let mut slot = [0usize, 1, 2];
    let mut w = to_work_array(&rows, &[d0, d1, d2], i);

    if j != 0 {
        swap_columns(&mut w, &mut slot, 0, j);
    }

    eliminate_first_column(&mut w);
    let (temp, i, j) = max_pivot_2x2(&w);

    if temp == 0.0 {
        return (1, 0.0, 0.0, 0.0, 0.0);
    }

    update_pivot_range(temp.abs(), &mut maxpiv, &mut minpiv);

    if j != 0 {
        swap_columns(&mut w, &mut slot, 1, 2);
    }

    let pivot = if i != 0 { 8 } else { 4 };
    let other = if i != 0 { 4 } else { 8 };

    if !eliminate_last_columns(&mut w, pivot, other, &mut maxpiv, &mut minpiv) {
        return (2, 0.0, 0.0, 0.0, 0.0);
    }

    let mut sol = [0.0f64; 3];
    sol[slot[0]] = w[3];
    sol[slot[1]] = w[pivot + 3];
    sol[slot[2]] = w[other + 3];

    (3, sol[0], sol[1], sol[2], minpiv / maxpiv)
}

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
    let (rank, x, y, z, pr) = solve_3x3(
        &[plane0.a(), plane0.b(), plane0.c()],
        &[plane1.a(), plane1.b(), plane1.c()],
        &[plane2.a(), plane2.b(), plane2.c()],
        -plane0.d(),
        -plane1.d(),
        -plane2.d(),
    );

    if rank == 3 && pr > 1e-12 {
        return Some(Point::new(x, y, z));
    }

    None
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

/// Newton refinement of the plane crossing in a tiny interval [ta, tb], kept when it stays inside and on the plane.
fn curve_plane_refine(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    ta: f64,
    tb: f64,
    results: &mut Vec<f64>,
) {
    let tm = (ta + tb) * 0.5;
    let pm = curve.point_at(tm);
    let dist = curve_signed_distance_to_plane(&pm, plane);

    if dist.abs() >= tolerance {
        return;
    }

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

    if curve_signed_distance_to_plane(&pt_final, plane).abs() < tolerance && t >= ta && t <= tb {
        results.push(t);
    }
}

/// Part of [ta, tb] where the sampled distance crosses the plane, the whole interval when unclear; None when it misses.
fn curve_plane_clip_range(
    curve: &NurbsCurve,
    plane: &Plane,
    tolerance: f64,
    ta: f64,
    tb: f64,
) -> Option<(f64, f64)> {
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
        return None;
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

    Some((t_min, t_max))
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
        curve_plane_refine(curve, plane, tolerance, ta, tb, results);

        return;
    }

    let Some((t_min, t_max)) = curve_plane_clip_range(curve, plane, tolerance, ta, tb) else {
        return;
    };
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

/// Midpoint tangent and step at (u, v) along dir, the previous tangent reused where the field has none; None when neither exists.
fn march_tangent(
    field: &SurfacePlaneField,
    u: f64,
    v: f64,
    dir: i32,
    prev_tu: f64,
    prev_tv: f64,
) -> Option<(f64, f64, f64)> {
    let (mut tu, mut tv) = match field.tangent(u, v, dir) {
        Some(t) => t,
        None => {
            if f64::hypot(prev_tu, prev_tv) < 1e-14 {
                return None;
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

    Some((tu, tv, local_step))
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
        let Some((tu, tv, local_step)) = march_tangent(field, u, v, dir, prev_tu, prev_tv) else {
            break;
        };

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
fn fitted_max_deviation(
    cand: &NurbsCurve,
    pts: &[Point],
    chords: &[f64],
    iterations: usize,
) -> f64 {
    let m = pts.len();
    let (ft0, ft1) = cand.domain();
    let mut max_dev = 0.0f64;

    for (p, chord) in pts.iter().zip(chords) {
        let t = ft0 + (ft1 - ft0) * chord;
        let w2 = (ft1 - ft0) * 2.0 / (m as i32 - 1).max(1) as f64;
        let mut lo = ft0.max(t - w2);
        let mut hi = ft1.min(t + w2);

        for _ in 0..iterations {
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

/// Best cubic fitted to 2D points, CVs doubled until within fit_tol; invalid when no fit succeeds.
fn fit_freeform_2d(pts_2d: &[Point], chords: &[f64], is_loop: bool, fit_tol: f64) -> NurbsCurve {
    let m = pts_2d.len() as i32;
    let mut target_cvs = 8_i32.max((total_turning(pts_2d) / 0.5) as i32 + 6);
    let max_cvs = (m - 1).min(128);
    let mut crv_2d = NurbsCurve::new(3, false, 4, 0);
    let mut best_dev = 1e300f64;

    for _ in 0..6 {
        if target_cvs > max_cvs {
            break;
        }

        let cand = NurbsCurve::create_fitted(pts_2d, target_cvs as usize, 3, is_loop);

        if !cand.is_valid() {
            break;
        }

        let max_dev = fitted_max_deviation(&cand, pts_2d, chords, 20);

        if max_dev < best_dev {
            best_dev = max_dev;
            crv_2d = cand;
        }

        if max_dev < fit_tol {
            break;
        }

        target_cvs = (target_cvs * 2).min(max_cvs + 1);
    }

    crv_2d
}

/// Move the CVs of a curve drawn in the plane's 2D frame to 3D, in place.
fn lift_to_plane(crv_2d: &mut NurbsCurve, plane: &Plane) {
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
    let mut crv_2d = fit_freeform_2d(&pts_2d, &chords, is_loop, fit_tol);

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

    lift_to_plane(&mut crv_2d, plane);

    crv_2d
}

/// Rational 9-CV circle on knots 0..4 around (cx, cy, cz) in the plane of the unit axes xa, ya.
fn circle_nurbs(
    cx: f64,
    cy: f64,
    cz: f64,
    xa: &[f64; 3],
    ya: &[f64; 3],
    radius: f64,
) -> NurbsCurve {
    let w = 2.0f64.sqrt() / 2.0;
    let px = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
    let py = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
    let wts = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
    let mut crv = NurbsCurve::new(3, true, 3, 9);

    for i in 0..10 {
        crv.set_nurbsknot(i, knots[i]);
    }

    for i in 0..9 {
        let x = cx + radius * (px[i] * xa[0] + py[i] * ya[0]);
        let y = cy + radius * (px[i] * xa[1] + py[i] * ya[1]);
        let z = cz + radius * (px[i] * xa[2] + py[i] * ya[2]);
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    crv
}

/// Rational 9-CV ellipse on knots 0..4 around (cx, cy, cz) with semi-axes along the unit axes ea, eb.
#[allow(clippy::too_many_arguments)]
fn ellipse_nurbs(
    cx: f64,
    cy: f64,
    cz: f64,
    ea: &[f64; 3],
    eb: &[f64; 3],
    semi_a: f64,
    semi_b: f64,
) -> NurbsCurve {
    let w = 2.0f64.sqrt() / 2.0;
    let px = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
    let py = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
    let wts = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
    let mut crv = NurbsCurve::new(3, true, 3, 9);

    for i in 0..10 {
        crv.set_nurbsknot(i, knots[i]);
    }

    for i in 0..9 {
        let x = cx + semi_a * px[i] * ea[0] + semi_b * py[i] * eb[0];
        let y = cy + semi_a * px[i] * ea[1] + semi_b * py[i] * eb[1];
        let z = cz + semi_a * px[i] * ea[2] + semi_b * py[i] * eb[2];
        crv.set_cv_4d(i, x * wts[i], y * wts[i], z * wts[i], wts[i]);
    }

    crv
}

/// Coordinates of p in the 2D frame (po, ax, ay).
fn plane_coords_2d(p: &Point, po: &Point, ax: &Vector, ay: &Vector) -> (f64, f64) {
    let dx = p[0] - po[0];
    let dy = p[1] - po[1];
    let dz = p[2] - po[2];

    (
        dx * ax[0] + dy * ax[1] + dz * ax[2],
        dx * ay[0] + dy * ay[1] + dz * ay[2],
    )
}

/// Exact circle of a closed planar trace when every point lies on the circle through three of them.
fn fit_plane_circle(all_pts: &[Point], plane: &Plane) -> NurbsCurve {
    let ax = plane.x_axis();
    let ay = plane.y_axis();
    let po = plane.origin();
    let n = all_pts.len();
    let (x1, y1) = plane_coords_2d(&all_pts[0], &po, &ax, &ay);
    let (x2, y2) = plane_coords_2d(&all_pts[n / 3], &po, &ax, &ay);
    let (x3, y3) = plane_coords_2d(&all_pts[2 * n / 3], &po, &ax, &ay);
    let ax_ = x2 - x1;
    let ay_ = y2 - y1;
    let bx_ = x3 - x1;
    let by_ = y3 - y1;
    let dd = 2.0 * (ax_ * by_ - ay_ * bx_);

    if dd.abs() <= 1e-10 {
        return NurbsCurve::default();
    }

    let a2 = ax_ * ax_ + ay_ * ay_;
    let b2 = bx_ * bx_ + by_ * by_;
    let ccx = x1 + (by_ * a2 - ay_ * b2) / dd;
    let ccy = y1 + (ax_ * b2 - bx_ * a2) / dd;
    let radius = (x1 - ccx).hypot(y1 - ccy);
    let mut max_dev = 0.0f64;

    for p in all_pts {
        let (px, py) = plane_coords_2d(p, &po, &ax, &ay);
        max_dev = f64::max(max_dev, ((px - ccx).hypot(py - ccy) - radius).abs());
    }

    if radius <= 1e-10 || max_dev >= f64::max(radius * 1e-5, 1e-6) {
        return NurbsCurve::default();
    }

    let cx3d = po[0] + ccx * ax[0] + ccy * ay[0];
    let cy3d = po[1] + ccx * ax[1] + ccy * ay[1];
    let cz3d = po[2] + ccx * ax[2] + ccy * ay[2];

    circle_nurbs(
        cx3d,
        cy3d,
        cz3d,
        &[ax[0], ax[1], ax[2]],
        &[ay[0], ay[1], ay[2]],
        radius,
    )
}

/// Augmented normal equations [AtA | Atb] of the conic fit through the points in the frame (po, ax, ay).
fn conic_normal_equations(
    all_pts: &[Point],
    po: &Point,
    ax: &Vector,
    ay: &Vector,
) -> [[f64; 6]; 5] {
    let mut ata = [[0.0f64; 5]; 5];
    let mut atb = [0.0f64; 5];

    for p in all_pts {
        let (x, y) = plane_coords_2d(p, po, ax, ay);
        let row = [x * x, x * y, y * y, x, y];

        for r in 0..5 {
            atb[r] += row[r];

            for c in 0..5 {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    let mut m = [[0.0f64; 6]; 5];

    for r in 0..5 {
        for c in 0..5 {
            m[r][c] = ata[r][c];
        }

        m[r][5] = atb[r];
    }

    m
}

/// Solve an augmented 5x6 system by Gaussian elimination with partial pivoting; None when singular.
fn solve_augmented_5x5(mut m: [[f64; 6]; 5]) -> Option<[f64; 5]> {
    for col in 0..5 {
        let mut pivot = col;

        for r in col + 1..5 {
            if m[r][col].abs() > m[pivot][col].abs() {
                pivot = r;
            }
        }

        if m[pivot][col].abs() < 1e-20 {
            return None;
        }

        if pivot != col {
            m.swap(col, pivot);
        }

        for r in col + 1..5 {
            let f = m[r][col] / m[col][col];

            for j in col..=5 {
                m[r][j] -= f * m[col][j];
            }
        }
    }

    let mut coef = [0.0f64; 5];

    for i in (0..5).rev() {
        let mut s = m[i][5];

        for j in i + 1..5 {
            s -= m[i][j] * coef[j];
        }

        coef[i] = s / m[i][i];
    }

    Some(coef)
}

/// Least-squares conic A x^2 + B xy + C y^2 + D x + E y = 1 through the points in the plane's frame, None when singular.
fn fit_plane_conic(all_pts: &[Point], po: &Point, ax: &Vector, ay: &Vector) -> Option<[f64; 5]> {
    solve_augmented_5x5(conic_normal_equations(all_pts, po, ax, ay))
}

/// Largest residual of the conic coef over the points in the frame (po, ax, ay).
fn conic_max_deviation(
    all_pts: &[Point],
    po: &Point,
    ax: &Vector,
    ay: &Vector,
    coef: &[f64; 5],
) -> f64 {
    let ca = coef[0];
    let cb = coef[1];
    let cc = coef[2];
    let cd = coef[3];
    let ce = coef[4];
    let mut max_conic_dev = 0.0f64;

    for p in all_pts {
        let (x, y) = plane_coords_2d(p, po, ax, ay);
        max_conic_dev = f64::max(
            max_conic_dev,
            (ca * x * x + cb * x * y + cc * y * y + cd * x + ce * y - 1.0).abs(),
        );
    }

    max_conic_dev
}

/// Largest distance from the points to the ellipse (cx, cy, semi_a, semi_b, theta) in the plane's frame.
#[allow(clippy::too_many_arguments)]
fn plane_ellipse_deviation(
    all_pts: &[Point],
    po: &Point,
    ax: &Vector,
    ay: &Vector,
    cx: f64,
    cy: f64,
    semi_a: f64,
    semi_b: f64,
    cos_t: f64,
    sin_t: f64,
) -> f64 {
    let mut max_ell_dev = 0.0f64;

    for p in all_pts {
        let (px2, py2) = plane_coords_2d(p, po, ax, ay);
        let lx = cos_t * (px2 - cx) + sin_t * (py2 - cy);
        let ly = -sin_t * (px2 - cx) + cos_t * (py2 - cy);
        let ang = (ly / semi_b).atan2(lx / semi_a);
        let ex = cx + semi_a * ang.cos() * cos_t - semi_b * ang.sin() * sin_t;
        let ey = cy + semi_a * ang.cos() * sin_t + semi_b * ang.sin() * cos_t;
        max_ell_dev = f64::max(max_ell_dev, (px2 - ex).hypot(py2 - ey));
    }

    max_ell_dev
}

/// Exact ellipse of a closed planar trace from a least-squares conic, invalid when it deviates.
fn fit_plane_ellipse(all_pts: &[Point], plane: &Plane) -> NurbsCurve {
    let ax = plane.x_axis();
    let ay = plane.y_axis();
    let po = plane.origin();
    let coef = match fit_plane_conic(all_pts, &po, &ax, &ay) {
        Some(coef) => coef,
        None => return NurbsCurve::default(),
    };
    let ca = coef[0];
    let cb = coef[1];
    let cc = coef[2];
    let cd = coef[3];
    let ce = coef[4];
    let disc = cb * cb - 4.0 * ca * cc;

    if disc >= -1e-10 || ca.abs() <= 1e-14 {
        return NurbsCurve::default();
    }

    let max_conic_dev = conic_max_deviation(all_pts, &po, &ax, &ay, &coef);

    if max_conic_dev / f64::max(f64::max(ca.abs(), cc.abs()), 1e-10) >= 0.01 {
        return NurbsCurve::default();
    }

    let det = 4.0 * ca * cc - cb * cb;
    let cx = (cb * ce - 2.0 * cc * cd) / det;
    let cy = (cb * cd - 2.0 * ca * ce) / det;
    let theta = 0.5 * cb.atan2(ca - cc);
    let cos_t = theta.cos();
    let sin_t = theta.sin();
    let a2 = ca * cos_t * cos_t + cb * cos_t * sin_t + cc * sin_t * sin_t;
    let c2 = ca * sin_t * sin_t - cb * cos_t * sin_t + cc * cos_t * cos_t;
    let rhs = -(ca * cx * cx + cb * cx * cy + cc * cy * cy + cd * cx + ce * cy - 1.0);

    if rhs <= 1e-14 || a2 <= 1e-14 || c2 <= 1e-14 {
        return NurbsCurve::default();
    }

    let semi_a = (rhs / a2).sqrt();
    let semi_b = (rhs / c2).sqrt();
    let cx3d = po[0] + cx * ax[0] + cy * ay[0];
    let cy3d = po[1] + cx * ax[1] + cy * ay[1];
    let cz3d = po[2] + cx * ax[2] + cy * ay[2];
    let mut ea = [0.0f64; 3];
    let mut eb = [0.0f64; 3];

    for d in 0..3 {
        ea[d] = cos_t * ax[d] + sin_t * ay[d];
        eb[d] = -sin_t * ax[d] + cos_t * ay[d];
    }

    let crv = ellipse_nurbs(cx3d, cy3d, cz3d, &ea, &eb, semi_a, semi_b);

    let ell_tol = f64::max(f64::max(semi_a, semi_b) * 1e-5, 2e-6);

    if plane_ellipse_deviation(all_pts, &po, &ax, &ay, cx, cy, semi_a, semi_b, cos_t, sin_t)
        > ell_tol
    {
        return NurbsCurve::default();
    }

    crv
}

/// Fit a 3D plane-constrained NurbsCurve to traced intersection points.
#[allow(clippy::too_many_arguments)]
fn surface_plane_fit_3d(
    all_pts: &[Point],
    is_loop: bool,
    plane: &Plane,
    step: f64,
    uv_to_3d: f64,
    uv_to_3d_min: f64,
    allow_conics: bool,
) -> NurbsCurve {
    let mut crv = NurbsCurve::default();

    if allow_conics && is_loop && all_pts.len() >= 6 {
        crv = fit_plane_circle(all_pts, plane);
    }

    if !crv.is_valid() && allow_conics && is_loop && all_pts.len() >= 8 {
        crv = fit_plane_ellipse(all_pts, plane);
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
/// Surface point at (u, v), the origin when the surface cannot evaluate.
fn srf_point(srf: &NurbsSurface, u: f64, v: f64) -> Point {
    srf.point_at(u, v).unwrap_or_default()
}

/// Surface domain in dir, (0, 1) when the surface has none.
fn srf_domain(srf: &NurbsSurface, dir: usize) -> (f64, f64) {
    srf.domain(dir).unwrap_or((0.0, 1.0))
}

/// Dot product of two triples.
fn ssi_dot(u: &[f64; 3], v: &[f64; 3]) -> f64 {
    u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
}

/// Cross product of two triples.
fn ssi_cross(u: &[f64; 3], v: &[f64; 3]) -> [f64; 3] {
    [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
}

/// Unit triple, or the input when degenerate.
fn ssi_unit(v: &[f64; 3]) -> [f64; 3] {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

    if length > 1e-300 {
        [v[0] / length, v[1] / length, v[2] / length]
    } else {
        *v
    }
}

/// Normalize a triple in place; false when shorter than 1e-12.
fn normalize_axis(v: &mut [f64; 3]) -> bool {
    let length = ssi_dot(v, v).sqrt();

    if length < 1e-12 {
        return false;
    }

    v[0] /= length;
    v[1] /= length;
    v[2] /= length;

    true
}

/// Two unit vectors spanning the plane perpendicular to unit n.
fn ortho_basis(n: &[f64; 3]) -> ([f64; 3], [f64; 3]) {
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
    let mut ux = ay * n[2] - az * n[1];
    let mut uy = az * n[0] - ax * n[2];
    let mut uz = ax * n[1] - ay * n[0];
    let ul = (ux * ux + uy * uy + uz * uz).sqrt();
    ux /= ul;
    uy /= ul;
    uz /= ul;
    let vx = n[1] * uz - n[2] * uy;
    let vy = n[2] * ux - n[0] * uz;
    let vz = n[0] * uy - n[1] * ux;

    ([ux, uy, uz], [vx, vy, vz])
}

/// Exact 9-CV rational NURBS circle on domain [0, 1].
fn exact_circle(
    cx: f64,
    cy: f64,
    cz: f64,
    xa: &[f64; 3],
    ya: &[f64; 3],
    radius: f64,
) -> NurbsCurve {
    let mut crv = circle_nurbs(cx, cy, cz, xa, ya, radius);
    crv.set_domain(0.0, 1.0);

    crv
}

/// Exact 9-CV rational NURBS ellipse on domain [0, 1].
#[allow(clippy::too_many_arguments)]
fn exact_ellipse(
    cx: f64,
    cy: f64,
    cz: f64,
    ea: &[f64; 3],
    eb: &[f64; 3],
    semi_a: f64,
    semi_b: f64,
) -> NurbsCurve {
    let mut crv = ellipse_nurbs(cx, cy, cz, ea, eb, semi_a, semi_b);
    crv.set_domain(0.0, 1.0);

    crv
}

/// Jacobi rotation of the symmetric a that zeroes a[p][q], accumulated into the eigenvector columns of v.
fn jacobi_rotate(a: &mut [[f64; 3]; 3], v: &mut [[f64; 3]; 3], p: usize, q: usize) {
    let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
    let t = (if theta >= 0.0 { 1.0 } else { -1.0 }) / (theta.abs() + (theta * theta + 1.0).sqrt());
    let c = 1.0 / (t * t + 1.0).sqrt();
    let s = t * c;

    for k in 0..3 {
        let akp = a[k][p];
        let akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
    }

    for k in 0..3 {
        let apk = a[p][k];
        let aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
    }

    for k in 0..3 {
        let vkp = v[k][p];
        let vkq = v[k][q];
        v[k][p] = c * vkp - s * vkq;
        v[k][q] = s * vkp + c * vkq;
    }
}

/// Eigenvalues/vectors of a symmetric 3x3 matrix (cyclic Jacobi).
fn jacobi_eig3(m: &[[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut a = [[0.0f64; 3]; 3];
    let mut v = [[0.0f64; 3]; 3];

    for r in 0..3 {
        for c in 0..3 {
            a[r][c] = m[r][c];
            v[r][c] = if r == c { 1.0 } else { 0.0 };
        }
    }

    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();

        if off < 1e-18 {
            break;
        }

        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }

            jacobi_rotate(&mut a, &mut v, p, q);
        }
    }

    let eigvals = [a[0][0], a[1][1], a[2][2]];
    let mut eigvecs = [[0.0f64; 3]; 3];

    for k in 0..3 {
        eigvecs[k] = [v[0][k], v[1][k], v[2][k]];
    }

    (eigvals, eigvecs)
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix.
fn smallest_eigenvector(m: &[[f64; 3]; 3]) -> [f64; 3] {
    let (evals, evecs) = jacobi_eig3(m);
    let mut kmin = 0;

    for k in 1..3 {
        if evals[k] < evals[kmin] {
            kmin = k;
        }
    }

    evecs[kmin]
}

/// Eigenvector of the largest eigenvalue of a symmetric 3x3 matrix.
fn largest_eigenvector(m: &[[f64; 3]; 3]) -> [f64; 3] {
    let (evals, evecs) = jacobi_eig3(m);
    let mut kmax = 0;

    for k in 1..3 {
        if evals[k] > evals[kmax] {
            kmax = k;
        }
    }

    evecs[kmax]
}

/// Kind of a recognized surface.
#[derive(Clone, Copy, PartialEq)]
enum RecogKind {
    None,     // Not recognized.
    Plane,    // Plane.
    Sphere,   // Sphere.
    Cylinder, // Cylinder.
    Cone,     // Cone.
    Torus,    // Torus.
}

/// Recognized-surface descriptor.
#[derive(Clone, Copy)]
struct RecogSurface {
    kind: RecogKind, // Recognized kind.
    p1: [f64; 3],    // Origin, center or apex.
    p2: [f64; 3],    // Normal or axis.
    r: f64,          // Radius or major radius.
    r2: f64,         // Half angle or minor radius.
}

/// Points of an n x n parameter grid stepping the domain by 1 / div.
fn sample_grid(surface: &NurbsSurface, n: usize, div: f64) -> Vec<[f64; 3]> {
    let (u0, u1) = srf_domain(surface, 0);
    let (v0, v1) = srf_domain(surface, 1);
    let mut pts = Vec::new();

    for i in 0..n {
        for j in 0..n {
            let p = srf_point(
                surface,
                u0 + (u1 - u0) * i as f64 / div,
                v0 + (v1 - v0) * j as f64 / div,
            );
            pts.push([p[0], p[1], p[2]]);
        }
    }

    pts
}

/// Normals of an n x n parameter grid stepping the domain by 1 / div.
fn sample_grid_normals(surface: &NurbsSurface, n: usize, div: f64) -> Vec<[f64; 3]> {
    let (u0, u1) = srf_domain(surface, 0);
    let (v0, v1) = srf_domain(surface, 1);
    let mut nrm = Vec::new();

    for i in 0..n {
        for j in 0..n {
            let v = surface.normal_at(
                u0 + (u1 - u0) * i as f64 / div,
                v0 + (v1 - v0) * j as f64 / div,
            );
            nrm.push([v[0], v[1], v[2]]);
        }
    }

    nrm
}

/// Least-squares circle through 2D samples: center and squared radius, None when singular.
fn fit_circle_2d(xy: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let mut ata = vec![vec![0.0f64; 3]; 3];
    let mut atb = vec![0.0f64; 3];

    for p in xy {
        let row = [p.0, p.1, 1.0];
        let rhs = -(p.0 * p.0 + p.1 * p.1);

        for r in 0..3 {
            atb[r] += row[r] * rhs;

            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    let sol = solve_gauss(&ata, &atb, 3)?;
    let cx = -sol[0] / 2.0;
    let cy = -sol[1] / 2.0;

    Some((cx, cy, cx * cx + cy * cy - sol[2]))
}

/// Recognize a cylinder from surface samples: axis point, axis direction and radius.
fn fit_cylinder(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], [f64; 3], f64)> {
    let pts = sample_grid(surface, 5, 4.0);
    let nrm = sample_grid_normals(surface, 5, 4.0);
    let mut m = [[0.0f64; 3]; 3];

    for n in &nrm {
        for r in 0..3 {
            for c in 0..3 {
                m[r][c] += n[r] * n[c];
            }
        }
    }

    let mut w = smallest_eigenvector(&m);

    if !normalize_axis(&mut w) {
        return None;
    }

    let (ea, eb) = ortho_basis(&w);
    let p0 = pts[0];
    let mut proj = Vec::new();

    for p in &pts {
        let dp = [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]];
        proj.push((ssi_dot(&dp, &ea), ssi_dot(&dp, &eb)));
    }

    let (ccx, ccy, r2) = fit_circle_2d(&proj)?;

    if r2 <= 1e-18 {
        return None;
    }

    let r = r2.sqrt();

    for pr in &proj {
        if (((pr.0 - ccx) * (pr.0 - ccx) + (pr.1 - ccy) * (pr.1 - ccy)).sqrt() - r).abs() > tol {
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

/// Cone samples on an 8 x 5 grid with the unit normals that are not degenerate.
#[allow(clippy::type_complexity)]
fn sample_cone(surface: &NurbsSurface) -> (Vec<[f64; 3]>, Vec<([f64; 3], [f64; 3])>) {
    let (u0, u1) = srf_domain(surface, 0);
    let (v0, v1) = srf_domain(surface, 1);
    let mut pts = Vec::new();
    let mut nrm = Vec::new();

    for i in 0..8 {
        let uu = u0 + (u1 - u0) * i as f64 / 8.0;

        for j in 0..5 {
            let vv = v0 + (v1 - v0) * j as f64 / 4.0;
            let p = srf_point(surface, uu, vv);
            pts.push([p[0], p[1], p[2]]);
            let n = surface.normal_at(uu, vv);
            let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();

            if nl < 1e-12 {
                continue;
            }

            nrm.push(([n[0] / nl, n[1] / nl, n[2] / nl], [p[0], p[1], p[2]]));
        }
    }

    (pts, nrm)
}

/// Least-squares meeting point of the tangent planes through (unit normal, point) samples.
fn cone_apex(nrm: &[([f64; 3], [f64; 3])]) -> Option<Vec<f64>> {
    let mut ata = vec![vec![0.0f64; 3]; 3];
    let mut atb = vec![0.0f64; 3];

    for (n, p) in nrm {
        let npd = ssi_dot(n, p);

        for r in 0..3 {
            atb[r] += n[r] * npd;

            for c in 0..3 {
                ata[r][c] += n[r] * n[c];
            }
        }
    }

    solve_gauss(&ata, &atb, 3)
}

/// Mean generator direction of unit apex-to-sample vectors, oriented away from the apex.
fn cone_axis(gs: &[[f64; 3]]) -> Option<[f64; 3]> {
    let mut gram = [[0.0f64; 3]; 3];

    for g in gs {
        for r in 0..3 {
            for c in 0..3 {
                gram[r][c] += g[r] * g[c];
            }
        }
    }

    let mut w = largest_eigenvector(&gram);
    let mut sx = [0.0f64; 3];

    for g in gs {
        sx[0] += g[0];
        sx[1] += g[1];
        sx[2] += g[2];
    }

    if ssi_dot(&w, &sx) < 0.0 {
        w = [-w[0], -w[1], -w[2]];
    }

    if !normalize_axis(&mut w) {
        return None;
    }

    Some(w)
}

/// Recognize a cone from surface samples: apex, axis and half angle.
fn fit_cone(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], [f64; 3], f64)> {
    let (pts, nrm) = sample_cone(surface);

    if nrm.len() < 4 {
        return None;
    }

    let vertex = cone_apex(&nrm)?;
    let mut gs = Vec::new();

    for p in &pts {
        let d = [p[0] - vertex[0], p[1] - vertex[1], p[2] - vertex[2]];
        let dl = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();

        if dl < tol {
            continue;
        }

        gs.push([d[0] / dl, d[1] / dl, d[2] / dl]);
    }

    if gs.len() < 3 {
        return None;
    }

    let w = cone_axis(&gs)?;
    let mut sumang = 0.0;

    for g in &gs {
        sumang += ssi_dot(g, &w).clamp(-1.0, 1.0).acos();
    }

    let alpha = sumang / gs.len() as f64;

    if !(1e-4..=PI / 2.0 - 1e-4).contains(&alpha) {
        return None;
    }

    let ca = alpha.cos();

    for p in &pts {
        let d = [p[0] - vertex[0], p[1] - vertex[1], p[2] - vertex[2]];
        let axd = ssi_dot(&d, &w);
        let perp = (ssi_dot(&d, &d) - axd * axd).max(0.0).sqrt();

        if (perp - axd * alpha.tan()).abs() * ca > tol {
            return None;
        }
    }

    Some(([vertex[0], vertex[1], vertex[2]], w, alpha))
}

/// Recognize a sphere from surface samples: center and radius.
fn fit_sphere(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], f64)> {
    let pts = sample_grid(surface, 5, 4.0);
    let mut ata = vec![vec![0.0f64; 4]; 4];
    let mut atb = vec![0.0f64; 4];

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
    let ccx = -sol[0] / 2.0;
    let ccy = -sol[1] / 2.0;
    let ccz = -sol[2] / 2.0;
    let r2 = ccx * ccx + ccy * ccy + ccz * ccz - sol[3];

    if r2 <= 0.0 {
        return None;
    }

    let r = r2.sqrt();

    for p in &pts {
        let d = ((p[0] - ccx) * (p[0] - ccx)
            + (p[1] - ccy) * (p[1] - ccy)
            + (p[2] - ccz) * (p[2] - ccz))
            .sqrt();

        if (d - r).abs() > tol {
            return None;
        }
    }

    Some(([ccx, ccy, ccz], r))
}

/// Centroid of the points and the unit direction of least spread about it; None when degenerate.
fn principal_axis(pts: &[[f64; 3]]) -> Option<([f64; 3], [f64; 3])> {
    let n = pts.len() as f64;
    let mut cen = [0.0f64; 3];

    for p in pts {
        cen[0] += p[0];
        cen[1] += p[1];
        cen[2] += p[2];
    }

    cen[0] /= n;
    cen[1] /= n;
    cen[2] /= n;
    let mut m = [[0.0f64; 3]; 3];

    for p in pts {
        let d = [p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]];

        for r in 0..3 {
            for c in 0..3 {
                m[r][c] += d[r] * d[c];
            }
        }
    }

    let mut w = smallest_eigenvector(&m);

    if !normalize_axis(&mut w) {
        return None;
    }

    Some((cen, w))
}

/// Recognize a torus from the smallest-variance axis and a tube cross-section circle fit.
fn fit_torus(surface: &NurbsSurface, tol: f64) -> Option<([f64; 3], [f64; 3], f64, f64)> {
    let pts = sample_grid(surface, 8, 8.0);
    let (cen, w) = principal_axis(&pts)?;
    let mut rhoa = Vec::new();

    for p in &pts {
        let d = [p[0] - cen[0], p[1] - cen[1], p[2] - cen[2]];
        let a = ssi_dot(&d, &w);
        let perp = [d[0] - a * w[0], d[1] - a * w[1], d[2] - a * w[2]];
        rhoa.push((ssi_dot(&perp, &perp).sqrt(), a));
    }

    let (rmaj, a0, r2) = fit_circle_2d(&rhoa)?;

    if r2 <= 1e-18 || rmaj <= 0.0 {
        return None;
    }

    let r = r2.sqrt();

    if rmaj <= r * 0.5 {
        return None;
    }

    for ra in &rhoa {
        if (((ra.0 - rmaj) * (ra.0 - rmaj) + (ra.1 - a0) * (ra.1 - a0)).sqrt() - r).abs() > tol {
            return None;
        }
    }

    let center = [cen[0] + a0 * w[0], cen[1] + a0 * w[1], cen[2] + a0 * w[2]];

    Some((center, w, rmaj, r))
}

/// Point and normal at the middle of the surface domain.
fn surface_mid_frame(srf: &NurbsSurface) -> (Point, Vector) {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);

    (
        srf_point(srf, (u0 + u1) * 0.5, (v0 + v1) * 0.5),
        srf.normal_at((u0 + u1) * 0.5, (v0 + v1) * 0.5),
    )
}

/// Classify a surface as plane, cylinder, cone, sphere or torus within tol.
fn recognize_surface(surface: &NurbsSurface, tol: f64) -> RecogSurface {
    let mut rs = RecogSurface {
        kind: RecogKind::None,
        p1: [0.0; 3],
        p2: [0.0; 3],
        r: 0.0,
        r2: 0.0,
    };

    if surface.is_planar(None, tol) {
        let (o, n) = surface_mid_frame(surface);
        rs.kind = RecogKind::Plane;
        rs.p1 = [o[0], o[1], o[2]];
        rs.p2 = [n[0], n[1], n[2]];

        return rs;
    }

    if let Some((c, r)) = fit_sphere(surface, tol) {
        rs.kind = RecogKind::Sphere;
        rs.p1 = c;
        rs.r = r;

        return rs;
    }

    if let Some((p1, p2, r)) = fit_cylinder(surface, tol) {
        rs.kind = RecogKind::Cylinder;
        rs.p1 = p1;
        rs.p2 = p2;
        rs.r = r;
    } else if let Some((p1, p2, r)) = fit_cone(surface, tol) {
        rs.kind = RecogKind::Cone;
        rs.p1 = p1;
        rs.p2 = p2;
        rs.r = r;
    } else if let Some((p1, p2, r, r2)) = fit_torus(surface, tol) {
        rs.kind = RecogKind::Torus;
        rs.p1 = p1;
        rs.p2 = p2;
        rs.r = r;
        rs.r2 = r2;
    }

    rs
}

/// Solve ((X-V).w)^2 - cos^2a |X-V|^2 = 0 along X = x0 + t d. Returns roots.
fn line_cone(x0: &[f64; 3], d: &[f64; 3], apex: &[f64; 3], w: &[f64; 3], alpha: f64) -> Vec<f64> {
    let ca2 = alpha.cos() * alpha.cos();
    let e = [x0[0] - apex[0], x0[1] - apex[1], x0[2] - apex[2]];
    let a = ssi_dot(&e, w);
    let b = ssi_dot(d, w);
    let c = ssi_dot(&e, &e);
    let dd = ssi_dot(&e, d);
    let ee = ssi_dot(d, d);
    let qa = b * b - ca2 * ee;
    let qb = 2.0 * a * b - 2.0 * ca2 * dd;
    let qc = a * a - ca2 * c;

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

/// Exact plane-sphere circle.
fn ssi_plane_sphere(plane: &RecogSurface, sph: &RecogSurface) -> Option<NurbsCurve> {
    let o = plane.p1;
    let nu = ssi_unit(&plane.p2);
    let c = sph.p1;
    let r = sph.r;
    let d = (c[0] - o[0]) * nu[0] + (c[1] - o[1]) * nu[1] + (c[2] - o[2]) * nu[2];

    if d.abs() >= r {
        return None;
    }

    let cc = [c[0] - d * nu[0], c[1] - d * nu[1], c[2] - d * nu[2]];
    let rr = (r * r - d * d).sqrt();
    let (xa, ya) = ortho_basis(&nu);

    Some(exact_circle(cc[0], cc[1], cc[2], &xa, &ya, rr))
}

/// Exact plane-cylinder section: an ellipse or nothing.
fn ssi_plane_cylinder(plane: &RecogSurface, cyl: &RecogSurface) -> Option<NurbsCurve> {
    let o = plane.p1;
    let nu = ssi_unit(&plane.p2);
    let p = cyl.p1;
    let w = ssi_unit(&cyl.p2);
    let r = cyl.r;
    let wn = ssi_dot(&w, &nu);

    if wn.abs() < 1e-7 {
        return None;
    }

    let t = ((o[0] - p[0]) * nu[0] + (o[1] - p[1]) * nu[1] + (o[2] - p[2]) * nu[2]) / wn;
    let cc = [p[0] + t * w[0], p[1] + t * w[1], p[2] + t * w[2]];
    let mraw = ssi_cross(&w, &nu);

    if ssi_dot(&mraw, &mraw).sqrt() < 1e-9 {
        let (xa, ya) = ortho_basis(&nu);

        return Some(exact_circle(cc[0], cc[1], cc[2], &xa, &ya, r));
    }

    let minor = ssi_unit(&mraw);
    let major = ssi_unit(&[w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]]);

    Some(exact_ellipse(
        cc[0],
        cc[1],
        cc[2],
        &major,
        &minor,
        r / wn.abs(),
        r,
    ))
}

/// Degree-1 segment of the line through q along w between axial offsets s0 and s1.
fn axis_segment(q: &[f64; 3], w: &[f64; 3], s0: f64, s1: f64) -> NurbsCurve {
    let e0 = Point::new(q[0] + s0 * w[0], q[1] + s0 * w[1], q[2] + s0 * w[2]);
    let e1 = Point::new(q[0] + s1 * w[0], q[1] + s1 * w[1], q[2] + s1 * w[2]);

    NurbsCurve::create(false, 1, &[e0, e1])
}

/// Axial range of a cylinder surface over three u and both v boundaries, padded by 5%.
fn cylinder_axial_range(srf: &NurbsSurface, p: &[f64; 3], w: &[f64; 3]) -> (f64, f64) {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let mut smin = 1e300;
    let mut smax = -1e300;

    for uu in [u0, 0.5 * (u0 + u1), u1] {
        for vv in [v0, v1] {
            let q = srf_point(srf, uu, vv);
            let s = (q[0] - p[0]) * w[0] + (q[1] - p[1]) * w[1] + (q[2] - p[2]) * w[2];
            smin = f64::min(smin, s);
            smax = f64::max(smax, s);
        }
    }

    let pad = 0.05 * f64::max(1e-9, smax - smin);

    (smin - pad, smax + pad)
}

/// Ruling lines of a plane parallel to the cylinder axis.
fn ssi_plane_cylinder_lines(
    plane: &RecogSurface,
    cyl: &RecogSurface,
    cyl_srf: &NurbsSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    let o = plane.p1;
    let nu = ssi_unit(&plane.p2);
    let p = cyl.p1;
    let w = ssi_unit(&cyl.p2);
    let r = cyl.r;
    let wn = ssi_dot(&w, &nu);

    if wn.abs() >= 1e-7 {
        return false;
    }

    let ds = (p[0] - o[0]) * nu[0] + (p[1] - o[1]) * nu[1] + (p[2] - o[2]) * nu[2];
    let d = ds.abs();
    let tt = r * 1e-9 + 1e-12;

    if d > r + tt {
        return true;
    }

    let (smin, smax) = cylinder_axial_range(cyl_srf, &p, &w);
    let foot = [p[0] - ds * nu[0], p[1] - ds * nu[1], p[2] - ds * nu[2]];
    let mut feet = Vec::new();

    if d >= r - tt {
        feet.push(foot);
    } else {
        let h = f64::max(0.0, r * r - d * d).sqrt();
        let s3 = ssi_unit(&ssi_cross(&w, &nu));
        feet.push([
            foot[0] + h * s3[0],
            foot[1] + h * s3[1],
            foot[2] + h * s3[2],
        ]);
        feet.push([
            foot[0] - h * s3[0],
            foot[1] - h * s3[1],
            foot[2] - h * s3[2],
        ]);
    }

    for q in &feet {
        let line = axis_segment(q, &w, smin, smax);

        if line.is_valid() {
            out.push(line);
        }
    }

    true
}

/// Height of the surface along the cone axis from the apex.
fn cone_axial_extent(srf: &NurbsSurface, apex: &[f64; 3], axis: &[f64; 3]) -> f64 {
    let w = ssi_unit(axis);
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let um = 0.5 * (u0 + u1);
    let mut h = 0.0;

    for vv in [v0, v1] {
        let p = srf_point(srf, um, vv);
        let s = (p[0] - apex[0]) * w[0] + (p[1] - apex[1]) * w[1] + (p[2] - apex[2]) * w[2];
        h = f64::max(h, s);
    }

    h
}

/// Whether a conic lies within the cone height H.
fn conic_within_cone(c: &NurbsCurve, apex: &[f64; 3], w: &[f64; 3], height: f64) -> bool {
    let (t0, t1) = c.domain();
    let pad = 1e-7 * f64::max(1.0, height);

    for i in 0..=64 {
        let p = c.point_at(t0 + (t1 - t0) * i as f64 / 64.0);
        let s = (p[0] - apex[0]) * w[0] + (p[1] - apex[1]) * w[1] + (p[2] - apex[2]) * w[2];

        if s < -pad || s > height + pad {
            return false;
        }
    }

    true
}

/// Fit a degree-2 rational arc through sampled points.
fn fit_conic_arc(pts: &[Point]) -> NurbsCurve {
    let m = pts.len();

    if m < 2 {
        return NurbsCurve::default();
    }

    if m == 2 {
        return NurbsCurve::create(false, 1, pts);
    }

    if m <= 4 {
        return NurbsCurve::create_interpolated(
            pts,
            CurveNurbsKnotStyle::Chord,
            CurveInterpStyle::Rhino,
        );
    }

    let mut num_cvs = (m / 6).clamp(8, 64);

    if num_cvs >= m {
        num_cvs = m - 1;
    }

    let c = NurbsCurve::create_fitted(pts, num_cvs, 3, false);

    if !c.is_valid() {
        return NurbsCurve::create_interpolated(
            pts,
            CurveNurbsKnotStyle::Chord,
            CurveInterpStyle::Rhino,
        );
    }

    c
}

/// Exact ellipse of a plane cutting a cone away from the apex.
fn build_exact_plane_cone_ellipse(
    o: &[f64; 3],
    nu: &[f64; 3],
    apex: &[f64; 3],
    w: &[f64; 3],
    alpha: f64,
) -> Option<NurbsCurve> {
    let wn = ssi_dot(w, nu);
    let mut m = ssi_cross(w, nu);
    let ml = ssi_dot(&m, &m).sqrt();

    if ml < 1e-12 {
        return None;
    }

    m = [m[0] / ml, m[1] / ml, m[2] / ml];
    let mut major = ssi_unit(&[w[0] - wn * nu[0], w[1] - wn * nu[1], w[2] - wn * nu[2]]);
    let dv = (apex[0] - o[0]) * nu[0] + (apex[1] - o[1]) * nu[1] + (apex[2] - o[2]) * nu[2];
    let vp = [
        apex[0] - dv * nu[0],
        apex[1] - dv * nu[1],
        apex[2] - dv * nu[2],
    ];
    let ts = line_cone(&vp, &major, apex, w, alpha);

    if ts.len() != 2 {
        return None;
    }

    let pa = [
        vp[0] + ts[0] * major[0],
        vp[1] + ts[0] * major[1],
        vp[2] + ts[0] * major[2],
    ];
    let pb = [
        vp[0] + ts[1] * major[0],
        vp[1] + ts[1] * major[1],
        vp[2] + ts[1] * major[2],
    ];
    let cc = [
        (pa[0] + pb[0]) * 0.5,
        (pa[1] + pb[1]) * 0.5,
        (pa[2] + pb[2]) * 0.5,
    ];
    let ab = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
    let semi_major = 0.5 * ssi_dot(&ab, &ab).sqrt();
    major = ssi_unit(&ab);
    let tm = line_cone(&cc, &m, apex, w, alpha);

    if tm.len() != 2 {
        return None;
    }

    let semi_minor = 0.5 * (tm[1] - tm[0]).abs();

    if semi_major < 1e-12 || semi_minor < 1e-12 {
        return None;
    }

    Some(exact_ellipse(
        cc[0], cc[1], cc[2], &major, &m, semi_major, semi_minor,
    ))
}

/// Single rational quadratic Bezier conic arc through A and B with mid control point T.
fn conic_bezier(pa: &[f64; 3], pt: &[f64; 3], pb: &[f64; 3], wmid: f64) -> NurbsCurve {
    let mut crv = NurbsCurve::new(3, true, 3, 3);
    let knots = [0.0, 0.0, 1.0, 1.0];

    for i in 0..4 {
        crv.set_nurbsknot(i, knots[i]);
    }

    crv.set_cv_4d(0, pa[0], pa[1], pa[2], 1.0);
    crv.set_cv_4d(1, pt[0] * wmid, pt[1] * wmid, pt[2] * wmid, wmid);
    crv.set_cv_4d(2, pb[0], pb[1], pb[2], 1.0);
    crv.set_domain(0.0, 1.0);

    crv
}

/// Frame of an open plane-cone conic: cone data and the conic's in-plane axes.
struct PlaneConeFrame {
    o: [f64; 3],    // Plane origin.
    nu: [f64; 3],   // Unit plane normal.
    apex: [f64; 3], // Cone apex.
    w: [f64; 3],    // Unit cone axis.
    height: f64,    // Cone height.
    cosa: f64,      // Cosine of the half angle.
    sina: f64,      // Sine of the half angle.
    ta: f64,        // Tangent of the half angle.
    na: f64,        // Plane normal along the axis.
    cost: f64,      // Absolute na.
    sint: f64,      // Length of nu x w.
    axex: [f64; 3], // In-plane axis towards the cone axis.
    axey: [f64; 3], // In-plane axis across the cone axis.
    axw: f64,       // axex along the cone axis.
    d0: f64,        // Signed apex distance to the plane.
    tol: f64,       // On-surface tolerance.
}

/// Conic frame of a plane cutting a cone; None when the plane is perpendicular to or contains the axis.
fn plane_cone_frame(
    o: &[f64; 3],
    nu: &[f64; 3],
    apex: &[f64; 3],
    w: &[f64; 3],
    alpha: f64,
    height: f64,
) -> Option<PlaneConeFrame> {
    let na = ssi_dot(nu, w);
    let axey = ssi_cross(nu, w);
    let sint = ssi_dot(&axey, &axey).sqrt();

    if sint < 1e-12 {
        return None;
    }

    let axey = [axey[0] / sint, axey[1] / sint, axey[2] / sint];
    let mut axex = ssi_cross(&axey, nu);
    let mut axw = ssi_dot(&axex, w);

    if axw < 0.0 {
        axex = [-axex[0], -axex[1], -axex[2]];
        axw = -axw;
    }

    if axw < 1e-12 {
        return None;
    }

    Some(PlaneConeFrame {
        o: *o,
        nu: *nu,
        apex: *apex,
        w: *w,
        height,
        cosa: alpha.cos(),
        sina: alpha.sin(),
        ta: alpha.tan(),
        na,
        cost: na.abs(),
        sint,
        axex,
        axey,
        axw,
        d0: (apex[0] - o[0]) * nu[0] + (apex[1] - o[1]) * nu[1] + (apex[2] - o[2]) * nu[2],
        tol: 1e-6 * f64::max(1.0, height),
    })
}

/// Whether 17 samples of the curve lie on both the plane and the cone within the frame tolerance.
fn conic_on_plane_cone(f: &PlaneConeFrame, c: &NurbsCurve) -> bool {
    for i in 0..=16 {
        let q = c.point_at(i as f64 / 16.0);
        let dp =
            ((q[0] - f.o[0]) * f.nu[0] + (q[1] - f.o[1]) * f.nu[1] + (q[2] - f.o[2]) * f.nu[2])
                .abs();
        let zz =
            (q[0] - f.apex[0]) * f.w[0] + (q[1] - f.apex[1]) * f.w[1] + (q[2] - f.apex[2]) * f.w[2];
        let wx = q[0] - f.apex[0] - zz * f.w[0];
        let wy = q[1] - f.apex[1] - zz * f.w[1];
        let wz = q[2] - f.apex[2] - zz * f.w[2];
        let rho = (wx * wx + wy * wy + wz * wz).sqrt();

        if dp > f.tol || (rho - f.ta * zz).abs() > f.tol * (1.0 + f.ta) {
            return false;
        }

        if zz < -f.tol || zz > f.height + f.tol {
            return false;
        }
    }

    true
}

/// Exact plane-cone parabola arc cut at the cone height.
fn plane_cone_parabola(f: &PlaneConeFrame) -> Option<NurbsCurve> {
    if f.cost < 1e-12 {
        return None;
    }

    let sax = -f.d0 / f.na;
    let cen = [
        f.apex[0] + sax * f.w[0],
        f.apex[1] + sax * f.w[1],
        f.apex[2] + sax * f.w[2],
    ];
    let distance = sax.abs();
    let dc = 0.5 * distance / f.cosa;
    let pf = dc * f.sina * f.sina;

    if pf < 1e-15 {
        return None;
    }

    for cs in [-1.0, 1.0] {
        let c2 = [
            cen[0] + cs * dc * f.axex[0],
            cen[1] + cs * dc * f.axex[1],
            cen[2] + cs * dc * f.axex[2],
        ];
        let zc = (c2[0] - f.apex[0]) * f.w[0]
            + (c2[1] - f.apex[1]) * f.w[1]
            + (c2[2] - f.apex[2]) * f.w[2];
        let t1s = 2.0 * pf * (f.height - zc) / f.axw;

        if t1s <= 0.0 {
            continue;
        }

        let t1 = t1s.sqrt();
        let xi = t1s / (2.0 * pf);
        let pa = [
            c2[0] + xi * f.axex[0] - t1 * f.axey[0],
            c2[1] + xi * f.axex[1] - t1 * f.axey[1],
            c2[2] + xi * f.axex[2] - t1 * f.axey[2],
        ];
        let pb = [
            c2[0] + xi * f.axex[0] + t1 * f.axey[0],
            c2[1] + xi * f.axex[1] + t1 * f.axey[1],
            c2[2] + xi * f.axex[2] + t1 * f.axey[2],
        ];
        let pt = [
            c2[0] - xi * f.axex[0],
            c2[1] - xi * f.axex[1],
            c2[2] - xi * f.axex[2],
        ];
        let arc = conic_bezier(&pa, &pt, &pb, 1.0);

        if arc.is_valid() && conic_on_plane_cone(f, &arc) {
            return Some(arc);
        }
    }

    None
}

/// Semi-axes and centers of the plane-cone hyperbola, one center per nappe; None when degenerate.
fn hyperbola_centers(f: &PlaneConeFrame) -> Option<(f64, f64, Vec<[f64; 3]>)> {
    let a;
    let b;
    let mut centers = Vec::new();

    if f.cost < 1e-6 {
        a = f.d0.abs() / f.ta;
        b = f.d0.abs();
        centers.push([
            f.apex[0] - f.d0 * f.nu[0],
            f.apex[1] - f.d0 * f.nu[1],
            f.apex[2] - f.d0 * f.nu[2],
        ]);
    } else {
        let dd = f.sina * f.sina - f.cost * f.cost;

        if dd < 1e-12 {
            return None;
        }

        let sax = -f.d0 / f.na;
        let cen = [
            f.apex[0] + sax * f.w[0],
            f.apex[1] + sax * f.w[1],
            f.apex[2] + sax * f.w[2],
        ];
        let distance = sax.abs();
        let dc = f.sint * f.sina * f.sina * distance / dd;
        a = f.cost * f.sina * f.cosa * distance / dd;
        b = f.cost * f.sina * distance / dd.sqrt();
        centers.push([
            cen[0] - dc * f.axex[0],
            cen[1] - dc * f.axex[1],
            cen[2] - dc * f.axex[2],
        ]);
        centers.push([
            cen[0] + dc * f.axex[0],
            cen[1] + dc * f.axex[1],
            cen[2] + dc * f.axex[2],
        ]);
    }

    if a < 1e-15 || b < 1e-15 {
        return None;
    }

    Some((a, b, centers))
}

/// Exact plane-cone hyperbola branch cut at the cone height.
fn plane_cone_hyperbola(f: &PlaneConeFrame) -> Option<NurbsCurve> {
    let (a, b, centers) = hyperbola_centers(f)?;

    for c2 in &centers {
        let zc = (c2[0] - f.apex[0]) * f.w[0]
            + (c2[1] - f.apex[1]) * f.w[1]
            + (c2[2] - f.apex[2]) * f.w[2];

        for sg in [1.0, -1.0] {
            let ch = (f.height - zc) / (sg * a * f.axw);

            if ch <= 1.0 + 1e-12 {
                continue;
            }

            let sh = (ch * ch - 1.0).sqrt();
            let xi = sg * a * ch;
            let xt = sg * a / ch;
            let pa = [
                c2[0] + xi * f.axex[0] - b * sh * f.axey[0],
                c2[1] + xi * f.axex[1] - b * sh * f.axey[1],
                c2[2] + xi * f.axex[2] - b * sh * f.axey[2],
            ];
            let pb = [
                c2[0] + xi * f.axex[0] + b * sh * f.axey[0],
                c2[1] + xi * f.axex[1] + b * sh * f.axey[1],
                c2[2] + xi * f.axex[2] + b * sh * f.axey[2],
            ];
            let pt = [
                c2[0] + xt * f.axex[0],
                c2[1] + xt * f.axex[1],
                c2[2] + xt * f.axex[2],
            ];
            let arc = conic_bezier(&pa, &pt, &pb, ch);

            if arc.is_valid() && conic_on_plane_cone(f, &arc) {
                return Some(arc);
            }
        }
    }

    None
}

/// Exact plane-cone hyperbola or parabola arc (IntAna_QuadQuadGeo.cxx:752-953 port).
#[allow(clippy::too_many_arguments)]
fn build_exact_plane_cone_open(
    o: &[f64; 3],
    nu: &[f64; 3],
    apex: &[f64; 3],
    w: &[f64; 3],
    alpha: f64,
    height: f64,
    parabola: bool,
) -> Option<NurbsCurve> {
    let f = plane_cone_frame(o, nu, apex, w, alpha, height)?;

    if parabola {
        return plane_cone_parabola(&f);
    }

    plane_cone_hyperbola(&f)
}

/// Plane-cone section: the plane, the cone and the cone's polar frame.
struct PlaneConeSection {
    o: [f64; 3],    // Plane origin.
    nu: [f64; 3],   // Unit plane normal.
    apex: [f64; 3], // Cone apex.
    w: [f64; 3],    // Unit cone axis.
    e1: [f64; 3],   // First unit axis normal.
    e2: [f64; 3],   // Second unit axis normal.
    alpha: f64,     // Half angle.
    height: f64,    // Cone height.
    ta: f64,        // Tangent of the half angle.
    cosa: f64,      // Cosine of the half angle.
    sina: f64,      // Sine of the half angle.
    na: f64,        // Plane normal along the axis.
    pp: f64,        // Plane normal along e1.
    qp: f64,        // Plane normal along e2.
    cost: f64,      // Absolute na.
    sint: f64,      // Plane normal across the axis.
    costa: f64,     // Cosine of the plane-to-generator angle sum.
    d0: f64,        // Signed apex distance to the plane.
}

impl PlaneConeSection {
    /// Plane normal along the generator at polar angle phi, scaled by cos alpha.
    fn denom(&self, phi: f64) -> f64 {
        self.na + self.ta * (self.pp * phi.cos() + self.qp * phi.sin())
    }

    /// Axial height of the section point at polar angle phi.
    fn height_at(&self, phi: f64) -> f64 {
        let d = self.denom(phi);

        if d.abs() < 1e-300 {
            1e308
        } else {
            -self.d0 / d
        }
    }

    /// Section point at polar angle phi.
    fn point(&self, phi: f64) -> Point {
        let s = self.height_at(phi);
        let rr = s * self.ta;
        let c = phi.cos();
        let sn = phi.sin();
        let apex = &self.apex;
        let w = &self.w;
        let e1 = &self.e1;
        let e2 = &self.e2;

        Point::new(
            apex[0] + s * w[0] + rr * (c * e1[0] + sn * e2[0]),
            apex[1] + s * w[1] + rr * (c * e1[1] + sn * e2[1]),
            apex[2] + s * w[2] + rr * (c * e1[2] + sn * e2[2]),
        )
    }

    /// Polar angle in [pa, pb] where the denominator reaches dtarget, by bisection.
    fn refine_base(&self, mut pa: f64, mut pb: f64, dtarget: f64) -> f64 {
        let mut fa = self.denom(pa) - dtarget;

        for _ in 0..60 {
            let pm = 0.5 * (pa + pb);
            let fm = self.denom(pm) - dtarget;

            if (fm < 0.0) == (fa < 0.0) {
                pa = pm;
                fa = fm;
            } else {
                pb = pm;
            }
        }

        0.5 * (pa + pb)
    }
}

/// Section of a recognized plane and cone; None when the cone is flat, a line or has no height.
fn plane_cone_section(
    plane: &RecogSurface,
    cone: &RecogSurface,
    cone_srf: &NurbsSurface,
) -> Option<PlaneConeSection> {
    let o = plane.p1;
    let nu = ssi_unit(&plane.p2);
    let apex = cone.p1;
    let w = ssi_unit(&cone.p2);
    let alpha = cone.r;

    if !(1e-7..=PI / 2.0 - 1e-7).contains(&alpha) {
        return None;
    }

    let height = cone_axial_extent(cone_srf, &apex, &w);

    if height < 1e-12 {
        return None;
    }

    let (e1, e2) = ortho_basis(&w);
    let na = ssi_dot(&nu, &w);
    let pp = ssi_dot(&nu, &e1);
    let qp = ssi_dot(&nu, &e2);
    let cost = na.abs();
    let sint = f64::max(0.0, pp * pp + qp * qp).sqrt();
    let cosa = alpha.cos();
    let sina = alpha.sin();

    Some(PlaneConeSection {
        o,
        nu,
        apex,
        w,
        e1,
        e2,
        alpha,
        height,
        ta: alpha.tan(),
        cosa,
        sina,
        na,
        pp,
        qp,
        cost,
        sint,
        costa: cost * cosa - sint * sina,
        d0: (apex[0] - o[0]) * nu[0] + (apex[1] - o[1]) * nu[1] + (apex[2] - o[2]) * nu[2],
    })
}

/// Runs of consecutive in-range polar samples, closed at both ends at the cone base.
fn collect_cone_runs(
    s: &PlaneConeSection,
    ok: &[bool],
    start: usize,
    dtarget: f64,
    runs: &mut Vec<Vec<Point>>,
) {
    let n = ok.len();
    let nf = n as f64;
    let mut cur = Vec::new();
    let mut inside = false;

    for i in 0..=n {
        let k = (start + i) % n;
        let uphi = TWO_PI * start as f64 / nf + TWO_PI * i as f64 / nf;
        let v = ok[k];

        if v && !inside {
            if i > 0 {
                cur.push(s.point(s.refine_base(uphi - TWO_PI / nf, uphi, dtarget)));
            }

            cur.push(s.point(uphi));
            inside = true;
        } else if v && inside {
            cur.push(s.point(uphi));
        } else if !v && inside {
            cur.push(s.point(s.refine_base(uphi - TWO_PI / nf, uphi, dtarget)));

            if cur.len() >= 2 {
                runs.push(cur.clone());
            }

            cur.clear();
            inside = false;
        }
    }

    if inside && cur.len() >= 2 {
        runs.push(cur);
    }
}

/// Sample the plane-cone section as point runs, one per branch, and whether it closes.
fn sample_plane_cone_arcs(s: &PlaneConeSection) -> (Vec<Vec<Point>>, bool) {
    let mut runs = Vec::new();
    let n = 720usize;
    let nf = n as f64;
    let eps = 1e-9 * f64::max(1.0, s.height);
    let mut ok = vec![false; n];
    let mut cnt = 0;

    for k in 0..n {
        let h = s.height_at(TWO_PI * k as f64 / nf);
        ok[k] = h > eps && h < s.height + eps;

        if ok[k] {
            cnt += 1;
        }
    }

    if cnt == 0 {
        return (runs, false);
    }

    if cnt == n {
        let mut closed_loop = Vec::new();

        for k in 0..=n {
            closed_loop.push(s.point(TWO_PI * (k % n) as f64 / nf));
        }

        runs.push(closed_loop);

        return (runs, true);
    }

    let mut start = 0;

    while start < n && ok[start] {
        start += 1;
    }

    let dtarget = if s.height > 1e-300 {
        -s.d0 / s.height
    } else {
        0.0
    };
    collect_cone_runs(s, &ok, start, dtarget, &mut runs);

    (runs, false)
}

/// Degree-1 segment from q to q + len d.
fn ray_segment(q: &[f64; 3], d: &[f64; 3], length: f64) -> NurbsCurve {
    let e0 = Point::new(q[0], q[1], q[2]);
    let e1 = Point::new(
        q[0] + length * d[0],
        q[1] + length * d[1],
        q[2] + length * d[2],
    );

    NurbsCurve::create(false, 1, &[e0, e1])
}

/// Plane through the cone apex: one tangent generator or two generator lines.
fn plane_cone_through_apex(s: &PlaneConeSection, out: &mut Vec<NurbsCurve>) {
    let nu = &s.nu;
    let w = &s.w;

    if s.costa.abs() < 1e-6 {
        let g = ssi_unit(&[
            w[0] - s.na * nu[0],
            w[1] - s.na * nu[1],
            w[2] - s.na * nu[2],
        ]);
        let gw = ssi_dot(&g, w);

        if gw > 1e-9 {
            out.push(ray_segment(&s.apex, &g, s.height / gw));
        }

        return;
    }

    if s.cost < s.sina {
        let axey = ssi_cross(nu, w);
        let axex = ssi_cross(&axey, nu);
        let dh = f64::max(0.0, s.sina * s.sina - s.cost * s.cost).sqrt() / s.cosa;

        for sgn in [1.0, -1.0] {
            let d = [
                axex[0] + sgn * dh * axey[0],
                axex[1] + sgn * dh * axey[1],
                axex[2] + sgn * dh * axey[2],
            ];
            let dw = ssi_dot(&d, w);

            if dw < 1e-12 {
                continue;
            }

            out.push(ray_segment(&s.apex, &d, s.height / dw));
        }
    }
}

/// Exact plane-cone conic: circle, ellipse, parabola or hyperbola; false when none fits the cone.
fn plane_cone_exact(s: &PlaneConeSection, out: &mut Vec<NurbsCurve>) -> bool {
    let ang = 1e-6;
    let mut is_circle = false;
    let mut is_parabola = false;
    let mut is_hyperbola = false;
    let mut is_ellipse = false;

    if s.cost < ang {
        is_hyperbola = true;
    } else if s.costa.abs() < ang {
        is_parabola = true;
    } else if s.sint < ang {
        is_circle = true;
    } else if s.cost < s.sina {
        is_hyperbola = true;
    } else {
        is_ellipse = true;
    }

    if is_circle {
        let v = &s.apex;
        let w = &s.w;
        let dax = (s.o[0] - v[0]) * w[0] + (s.o[1] - v[1]) * w[1] + (s.o[2] - v[2]) * w[2];
        let rr = dax.abs() * s.ta;

        if rr > 1e-12 {
            let cc = [v[0] + dax * w[0], v[1] + dax * w[1], v[2] + dax * w[2]];
            let circ = exact_circle(cc[0], cc[1], cc[2], &s.e1, &s.e2, rr);

            if conic_within_cone(&circ, v, w, s.height) {
                out.push(circ);
            }
        }

        return true;
    }

    if is_ellipse {
        if let Some(c3) = build_exact_plane_cone_ellipse(&s.o, &s.nu, &s.apex, &s.w, s.alpha) {
            if conic_within_cone(&c3, &s.apex, &s.w, s.height) {
                out.push(c3);

                return true;
            }
        }
    }

    if is_parabola || is_hyperbola {
        if let Some(c3) =
            build_exact_plane_cone_open(&s.o, &s.nu, &s.apex, &s.w, s.alpha, s.height, is_parabola)
        {
            out.push(c3);

            return true;
        }
    }

    false
}

/// Plane-cone section: exact lines or conic when possible, fitted arcs otherwise.
fn ssi_plane_cone(
    plane: &RecogSurface,
    cone: &RecogSurface,
    cone_srf: &NurbsSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    let s = match plane_cone_section(plane, cone, cone_srf) {
        Some(s) => s,
        None => return false,
    };

    if s.d0.abs() < 1e-6 * f64::max(1.0, s.height) {
        plane_cone_through_apex(&s, out);

        return true;
    }

    if plane_cone_exact(&s, out) {
        return true;
    }

    let (runs, _) = sample_plane_cone_arcs(&s);

    for r in &runs {
        let c = fit_conic_arc(r);

        if c.is_valid() {
            out.push(c);
        }
    }

    true
}

/// Exact plane-torus circles for a plane perpendicular to the axis.
fn ssi_plane_torus(plane: &RecogSurface, tor: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let o = plane.p1;
    let nu = ssi_unit(&plane.p2);
    let center = tor.p1;
    let w = ssi_unit(&tor.p2);
    let rmaj = tor.r;
    let r = tor.r2;
    let wn = ssi_dot(&w, &nu);

    if (wn.abs() - 1.0).abs() > 1e-7 {
        return false;
    }

    let d = (o[0] - center[0]) * w[0] + (o[1] - center[1]) * w[1] + (o[2] - center[2]) * w[2];

    if d.abs() > r {
        return true;
    }

    let h = f64::max(0.0, r * r - d * d).sqrt();
    let cc = [
        center[0] + d * w[0],
        center[1] + d * w[1],
        center[2] + d * w[2],
    ];
    let (xa, ya) = ortho_basis(&w);

    for rr in [rmaj + h, rmaj - h] {
        if rr > 1e-12 {
            out.push(exact_circle(cc[0], cc[1], cc[2], &xa, &ya, rr));
        }
    }

    true
}

/// Corner frame of a bilinear face: origin, edge vectors and their Gram matrix.
struct FaceFrame {
    o: [f64; 3],  // Corner at (u0, v0).
    eu: [f64; 3], // Edge to (u1, v0).
    ev: [f64; 3], // Edge to (u0, v1).
    exx: f64,     // eu . eu
    eyy: f64,     // ev . ev
    exy: f64,     // eu . ev
    det: f64,     // Gram determinant.
}

impl FaceFrame {
    /// Face fractions (al, be) of the offset r from the corner.
    fn fraction(&self, r: &[f64; 3]) -> (f64, f64) {
        let rx = ssi_dot(r, &self.eu);
        let ry = ssi_dot(r, &self.ev);

        (
            (self.eyy * rx - self.exy * ry) / self.det,
            (self.exx * ry - self.exy * rx) / self.det,
        )
    }
}

/// Corner frame of a surface from its corners (u0, v0), (u1, v0) and (u0, v1).
fn face_frame(s: &NurbsSurface) -> FaceFrame {
    let (u0, u1) = srf_domain(s, 0);
    let (v0, v1) = srf_domain(s, 1);
    let o = srf_point(s, u0, v0);
    let pu = srf_point(s, u1, v0);
    let pv = srf_point(s, u0, v1);
    let eu = [pu[0] - o[0], pu[1] - o[1], pu[2] - o[2]];
    let ev = [pv[0] - o[0], pv[1] - o[1], pv[2] - o[2]];
    let exx = ssi_dot(&eu, &eu);
    let eyy = ssi_dot(&ev, &ev);
    let exy = ssi_dot(&eu, &ev);

    FaceFrame {
        o: [o[0], o[1], o[2]],
        eu,
        ev,
        exx,
        eyy,
        exy,
        det: exx * eyy - exy * exy,
    }
}

/// Boundary samples per side in one direction: cv_count - 1 when linear, else 4 * cv_count.
fn boundary_steps(s: &NurbsSurface, dir: usize) -> usize {
    if s.degree(dir) == 1 {
        s.cv_count(dir) - 1
    } else {
        4 * s.cv_count(dir)
    }
}

/// Surface boundary in loop order, each side split into boundary_steps pieces.
fn cutter_boundary(cutter: &NurbsSurface) -> Vec<Point> {
    let (cu0, cu1) = srf_domain(cutter, 0);
    let (cv0, cv1) = srf_domain(cutter, 1);
    let nu = boundary_steps(cutter, 0);
    let nv = boundary_steps(cutter, 1);
    let mut points = Vec::new();

    for i in 0..nu {
        points.push(srf_point(
            cutter,
            cu0 + (cu1 - cu0) * i as f64 / nu as f64,
            cv0,
        ));
    }

    for i in 0..nv {
        points.push(srf_point(
            cutter,
            cu1,
            cv0 + (cv1 - cv0) * i as f64 / nv as f64,
        ));
    }

    for i in (1..=nu).rev() {
        points.push(srf_point(
            cutter,
            cu0 + (cu1 - cu0) * i as f64 / nu as f64,
            cv1,
        ));
    }

    for i in (1..=nv).rev() {
        points.push(srf_point(
            cutter,
            cu0,
            cv0 + (cv1 - cv0) * i as f64 / nv as f64,
        ));
    }

    points
}

/// Boundary polygon of a surface in the plane through it, empty without area: (outline, frame).
fn boundary_outline(s: &NurbsSurface) -> (Polyline, Plane) {
    let boundary = cutter_boundary(s);
    let mut normal = Vector::new(0.0, 0.0, 0.0);

    for i in 1..boundary.len().saturating_sub(1) {
        normal += (&boundary[i] - &boundary[0]).cross(&(&boundary[i + 1] - &boundary[0]));
    }

    if normal.magnitude() < 1e-14 {
        return (Polyline::default(), Plane::default());
    }

    let frame = Plane::from_point_normal(boundary[0].clone(), normal, None);
    let mut outline = Polyline::default();

    for p in &boundary {
        let d = p - &frame.origin();
        outline.add_point(Point::new(
            d.dot(&frame.x_axis()),
            d.dot(&frame.y_axis()),
            0.0,
        ));
    }

    (outline, frame)
}

/// Whether the surface is the parallelogram of its corner frame, mapped affinely, checked on the boundary grid.
fn is_parallelogram_face(s: &NurbsSurface, f: &FaceFrame) -> bool {
    if f.det.abs() < 1e-18 {
        return false;
    }

    let (cu0, cu1) = srf_domain(s, 0);
    let (cv0, cv1) = srf_domain(s, 1);
    let nu = boundary_steps(s, 0);
    let nv = boundary_steps(s, 1);
    let tol = 1e-9 * (f.exx + f.eyy).sqrt();

    for i in 0..=nu {
        let a = i as f64 / nu as f64;

        for j in 0..=nv {
            let b = j as f64 / nv as f64;
            let p = srf_point(s, cu0 + (cu1 - cu0) * a, cv0 + (cv1 - cv0) * b);
            let q = Point::new(
                f.o[0] + a * f.eu[0] + b * f.ev[0],
                f.o[1] + a * f.eu[1] + b * f.ev[1],
                f.o[2] + a * f.eu[2] + b * f.ev[2],
            );

            if p.distance(&q, None) > tol {
                return false;
            }
        }
    }

    true
}

/// Narrow [t0, t1] to where c + t d lies in [0, 1]; false when d is zero and c is outside.
fn clip_axis(c: f64, d: f64, t0: &mut f64, t1: &mut f64) -> bool {
    if d.abs() < 1e-15 {
        return (-1e-9..=1.0 + 1e-9).contains(&c);
    }

    let mut ta = (0.0 - c) / d;
    let mut tb = (1.0 - c) / d;

    if ta > tb {
        std::mem::swap(&mut ta, &mut tb);
    }

    *t0 = f64::max(*t0, ta);
    *t1 = f64::min(*t1, tb);

    true
}

/// Narrow [tmin, tmax] to the part of the line inside the parallelogram of the frame; empty when it misses.
fn clip_line_to_face(
    f: &FaceFrame,
    anchor: &[f64; 3],
    dir: &[f64; 3],
    tmin: &mut f64,
    tmax: &mut f64,
    empty: &mut bool,
) -> bool {
    let (a0, b0) = f.fraction(&[anchor[0] - f.o[0], anchor[1] - f.o[1], anchor[2] - f.o[2]]);
    let (da, db) = f.fraction(dir);
    let mut t0 = -1e300;
    let mut t1 = 1e300;

    if !clip_axis(a0, da, &mut t0, &mut t1) || !clip_axis(b0, db, &mut t0, &mut t1) || t0 > t1 {
        *empty = true;

        return false;
    }

    *tmin = f64::max(*tmin, t0);
    *tmax = f64::min(*tmax, t1);

    true
}

/// Parameter spans of the line inside the boundary polygon of the face; false when the polygon has no area.
fn clip_line_to_outline(
    s: &NurbsSurface,
    anchor: &[f64; 3],
    dir: &[f64; 3],
    spans: &mut Vec<(f64, f64)>,
) -> bool {
    let (outline, frame) = boundary_outline(s);

    if outline.point_count() == 0 {
        return false;
    }

    let offset = &Point::new(anchor[0], anchor[1], anchor[2]) - &frame.origin();
    let direction = Vector::new(dir[0], dir[1], dir[2]);
    let ax = offset.dot(&frame.x_axis());
    let ay = offset.dot(&frame.y_axis());
    let dx = direction.dot(&frame.x_axis());
    let dy = direction.dot(&frame.y_axis());
    let n = outline.point_count();
    let mut ts = Vec::new();

    for i in 0..n {
        let a = &outline[i];
        let b = &outline[(i + 1) % n];
        let ex = b[0] - a[0];
        let ey = b[1] - a[1];
        let denom = dx * ey - dy * ex;

        if denom.abs() < 1e-15 {
            continue;
        }

        let wx = a[0] - ax;
        let wy = a[1] - ay;
        let along = (wx * dy - wy * dx) / denom;

        if (-1e-12..=1.0 + 1e-12).contains(&along) {
            ts.push((wx * ey - wy * ex) / denom);
        }
    }

    ts.sort_by(f64::total_cmp);

    for i in 0..ts.len().saturating_sub(1) {
        let mid = 0.5 * (ts[i] + ts[i + 1]);

        if ts[i + 1] - ts[i] <= 1e-9
            || !outline.point_in_polygon_2d(&Point::new(ax + mid * dx, ay + mid * dy, 0.0))
        {
            continue;
        }

        match spans.last_mut() {
            Some(last) if ts[i] - last.1 <= 1e-9 => last.1 = ts[i + 1],
            _ => spans.push((ts[i], ts[i + 1])),
        }
    }

    true
}

/// Parameter spans of the line inside the face: its corner parallelogram, else its boundary polygon; empty when it misses.
fn clip_line_to_face_spans(
    s: &NurbsSurface,
    anchor: &[f64; 3],
    dir: &[f64; 3],
    spans: &mut Vec<(f64, f64)>,
    empty: &mut bool,
) -> bool {
    let f = face_frame(s);

    if !is_parallelogram_face(s, &f) {
        return clip_line_to_outline(s, anchor, dir, spans);
    }

    let mut tmin = -1e300;
    let mut tmax = 1e300;

    if !clip_line_to_face(&f, anchor, dir, &mut tmin, &mut tmax, empty) {
        return false;
    }

    spans.push((tmin, tmax));

    true
}

/// Exact plane-plane line clipped to both finite faces.
fn ssi_plane_plane(
    sa: &NurbsSurface,
    pa: &RecogSurface,
    sb: &NurbsSurface,
    pb: &RecogSurface,
    out: &mut Vec<NurbsCurve>,
    empty: &mut bool,
) -> bool {
    *empty = false;
    let na = ssi_unit(&pa.p2);
    let nb = ssi_unit(&pb.p2);
    let v = ssi_cross(&na, &nb);
    let vl = ssi_dot(&v, &v).sqrt();

    if vl < 1e-9 {
        return false;
    }

    let da = ssi_dot(&na, &pa.p1);
    let db = ssi_dot(&nb, &pb.p1);
    let nb_x_v = ssi_cross(&nb, &v);
    let v_x_na = ssi_cross(&v, &na);
    let inv = 1.0 / (vl * vl);
    let anchor = [
        (da * nb_x_v[0] + db * v_x_na[0]) * inv,
        (da * nb_x_v[1] + db * v_x_na[1]) * inv,
        (da * nb_x_v[2] + db * v_x_na[2]) * inv,
    ];
    let dir = [v[0] / vl, v[1] / vl, v[2] / vl];
    let mut spans_a = Vec::new();
    let mut spans_b = Vec::new();

    if !clip_line_to_face_spans(sa, &anchor, &dir, &mut spans_a, empty)
        || !clip_line_to_face_spans(sb, &anchor, &dir, &mut spans_b, empty)
    {
        return false;
    }

    for span_a in &spans_a {
        for span_b in &spans_b {
            let tmin = f64::max(span_a.0, span_b.0);
            let tmax = f64::min(span_a.1, span_b.1);

            if tmax - tmin <= 1e-9 {
                continue;
            }

            let start = Point::new(
                anchor[0] + tmin * dir[0],
                anchor[1] + tmin * dir[1],
                anchor[2] + tmin * dir[2],
            );
            let end = Point::new(
                anchor[0] + tmax * dir[0],
                anchor[1] + tmax * dir[1],
                anchor[2] + tmax * dir[2],
            );
            let mut c3 = NurbsCurve::create(false, 1, &[start, end]);
            c3.set_domain(0.0, 1.0);
            out.push(c3);
        }
    }

    *empty = out.is_empty();

    !*empty
}

/// Tri-state analytic result: not analytic, recognised empty, or curve triples.
#[derive(Clone, Copy, PartialEq)]
enum AnalyticStatus {
    NotAnalytic, // Not both surfaces recognized, or no exact case.
    Hit,         // Exact case evaluated, possibly without curves.
}

/// Analytic section result: status and the curve triples.
struct AnalyticResult {
    status: AnalyticStatus, // Whether both surfaces were recognized and whether they meet.
    triples: Vec<(NurbsCurve, NurbsCurve, NurbsCurve)>, // 3D curve with both pullbacks.
}

/// Angle shifted by whole turns to within half a turn of prev.
fn unwrap_angle(mut a: f64, prev: f64) -> f64 {
    while a - prev > PI {
        a -= TWO_PI;
    }

    while a - prev < -PI {
        a += TWO_PI;
    }

    a
}

/// Angle shifted by whole turns into [-pi, pi].
fn wrap_angle(mut a: f64) -> f64 {
    while a > PI {
        a -= TWO_PI;
    }

    while a < -PI {
        a += TWO_PI;
    }

    a
}

/// Angle shifted by whole turns into [lo - 1e-9, hi + 1e-9] when the range allows.
fn wrap_to_range(mut a: f64, lo: f64, hi: f64) -> f64 {
    while a < lo - 1e-9 {
        a += TWO_PI;
    }

    while a > hi + 1e-9 {
        a -= TWO_PI;
    }

    a
}

/// Value shifted by whole periods to within half a period of prev.
fn unwrap_period(mut x: f64, prev: f64, period: f64) -> f64 {
    while x - prev > period * 0.5 {
        x -= period;
    }

    while x - prev < -period * 0.5 {
        x += period;
    }

    x
}

/// Index of the period cell of x counted from x0.
fn period_index(x: f64, x0: f64, period: f64) -> i32 {
    ((x - x0) / period + 1e-9).floor() as i32
}

/// Height of p along the unit axis through origin.
fn axis_height(p: &Point, origin: &[f64; 3], axis: &[f64; 3]) -> f64 {
    let r = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];

    ssi_dot(&r, axis)
}

/// Squared distance of p from the unit axis through origin.
fn axis_radial_sq(p: &Point, origin: &[f64; 3], axis: &[f64; 3]) -> f64 {
    let r = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
    let h = ssi_dot(&r, axis);
    let px = r[0] - h * axis[0];
    let py = r[1] - h * axis[1];
    let pz = r[2] - h * axis[2];

    px * px + py * py + pz * pz
}

/// Distance between the curve's end points.
fn curve_gap(c: &NurbsCurve) -> f64 {
    let (t0, t1) = c.domain();

    c.point_at(t0).distance(&c.point_at(t1), None)
}

/// Constant-v UV line from u0 to u1.
fn iso_v_line(u0: f64, u1: f64, vc: f64) -> NurbsCurve {
    NurbsCurve::create(
        false,
        1,
        &[Point::new(u0, vc, 0.0), Point::new(u1, vc, 0.0)],
    )
}

/// Height range and mean of 33 curve samples along the unit axis through origin.
fn curve_height_stats(c3d: &NurbsCurve, origin: &[f64; 3], axis: &[f64; 3]) -> (f64, f64, f64) {
    let (t0, t1) = c3d.domain();
    let ns = 33;
    let mut hsum = 0.0;
    let mut hmin = 1e300;
    let mut hmax = -1e300;

    for i in 0..ns {
        let h = axis_height(
            &c3d.point_at(t0 + (t1 - t0) * i as f64 / 32.0),
            origin,
            axis,
        );
        hmin = f64::min(hmin, h);
        hmax = f64::max(hmax, h);
        hsum += h;
    }

    (hmin, hmax, hsum / ns as f64)
}

/// v on the line u = um where the axial height reaches hc, by bisection; None when hc is outside.
#[allow(clippy::too_many_arguments)]
fn bisect_height_v(
    srf: &NurbsSurface,
    um: f64,
    v0: f64,
    v1: f64,
    hc: f64,
    origin: &[f64; 3],
    axis: &[f64; 3],
) -> Option<f64> {
    let mut va = v0;
    let mut vb = v1;
    let mut ha = axis_height(&srf_point(srf, um, va), origin, axis);
    let hb = axis_height(&srf_point(srf, um, vb), origin, axis);

    if (hc - ha) * (hc - hb) > 0.0 {
        return None;
    }

    for _ in 0..60 {
        let vm = 0.5 * (va + vb);
        let hm = axis_height(&srf_point(srf, um, vm), origin, axis);

        if (hm - hc) * (ha - hc) <= 0.0 {
            vb = vm;
        } else {
            va = vm;
            ha = hm;
        }
    }

    Some(0.5 * (va + vb))
}

/// Pcurve on a planar face that is not its corner parallelogram: a polyline through 65 inverted points, invalid when the curve leaves the face.
fn inverted_plane_pcurve(srf: &NurbsSurface, f: &FaceFrame, c3d: &NurbsCurve) -> NurbsCurve {
    let (t0, t1) = c3d.domain();
    let tol = 1e-6 * (f.exx + f.eyy).sqrt();
    let mut uvs = Vec::new();

    for i in 0..=64 {
        let p = c3d.point_at(t0 + (t1 - t0) * i as f64 / 64.0);
        let closest = Closest::surface_point(srf, &p, 0.0, 0.0, 0.0, 0.0);

        if closest.2 > tol {
            return NurbsCurve::default();
        }

        uvs.push(Point::new(closest.0, closest.1, 0.0));
    }

    let mut pc = NurbsCurve::create(false, 1, &uvs);

    if !pc.set_domain(t0, t1) {
        return NurbsCurve::default();
    }

    pc
}

/// Plane pcurve: inverted points on a face that is not its corner parallelogram, else the control points mapped to its parameters.
fn plane_pcurve(srf: &NurbsSurface, c3d: &NurbsCurve) -> NurbsCurve {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let f = face_frame(srf);

    if !is_parallelogram_face(srf, &f) {
        let inverted = inverted_plane_pcurve(srf, &f, c3d);

        if inverted.is_valid() {
            return inverted;
        }
    }

    if f.det.abs() < 1e-18 {
        return NurbsCurve::default();
    }

    let mut pc = c3d.clone();

    for i in 0..c3d.cv_count() {
        let cv = c3d.get_cv(i).unwrap_or_default();
        let (a, b) = f.fraction(&[cv[0] - f.o[0], cv[1] - f.o[1], cv[2] - f.o[2]]);
        let u = u0 + a * (u1 - u0);
        let v = v0 + b * (v1 - v0);

        if c3d.is_rational() {
            let w = c3d.weight(i);
            pc.set_cv_4d(i, u * w, v * w, 0.0, w);
        } else {
            pc.set_cv(i, &Point::new(u, v, 0.0));
        }
    }

    pc
}

/// Cylinder pcurve of a circle perpendicular to the axis: a constant-v line.
fn cylinder_pcurve(srf: &NurbsSurface, recog: &RecogSurface, c3d: &NurbsCurve) -> NurbsCurve {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let mut ax = recog.p2;

    if !normalize_axis(&mut ax) {
        return NurbsCurve::default();
    }

    let um = 0.5 * (u0 + u1);
    let h0 = axis_height(&srf_point(srf, um, v0), &recog.p1, &ax);
    let h1 = axis_height(&srf_point(srf, um, v1), &recog.p1, &ax);

    if (h1 - h0).abs() < 1e-12 {
        return NurbsCurve::default();
    }

    let (hmin, hmax, hc) = curve_height_stats(c3d, &recog.p1, &ax);

    if hmax - hmin > 1e-5 * (h1 - h0).abs() {
        return NurbsCurve::default();
    }

    if curve_gap(c3d) > 1e-6 * ((h1 - h0).abs() + 1.0) {
        return NurbsCurve::default();
    }

    let vc = v0 + (hc - h0) / (h1 - h0) * (v1 - v0);

    if vc < f64::min(v0, v1) - 1e-9 || vc > f64::max(v0, v1) + 1e-9 {
        return NurbsCurve::default();
    }

    iso_v_line(u0, u1, vc)
}

/// Sphere pcurve of a latitude circle: a constant-v line.
fn sphere_pcurve(srf: &NurbsSurface, recog: &RecogSurface, c3d: &NurbsCurve) -> NurbsCurve {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let um = 0.5 * (u0 + u1);
    let sp = srf_point(srf, um, v0);
    let np = srf_point(srf, um, v1);
    let mut ax = [np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]];

    if !normalize_axis(&mut ax) {
        return NurbsCurve::default();
    }

    let (hmin, hmax, hc) = curve_height_stats(c3d, &recog.p1, &ax);

    if hmax - hmin > recog.r * 1e-4 {
        return NurbsCurve::default();
    }

    if curve_gap(c3d) > recog.r * 1e-3 {
        return NurbsCurve::default();
    }

    match bisect_height_v(srf, um, v0, v1, hc, &recog.p1, &ax) {
        Some(vc) => iso_v_line(u0, u1, vc),
        None => NurbsCurve::default(),
    }
}

/// Cone pcurve of a circle perpendicular to the axis: a constant-v line.
fn cone_pcurve(srf: &NurbsSurface, recog: &RecogSurface, c3d: &NurbsCurve) -> NurbsCurve {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let mut ax = recog.p2;

    if !normalize_axis(&mut ax) {
        return NurbsCurve::default();
    }

    let (t0, t1) = c3d.domain();
    let clen = c3d
        .point_at(t0)
        .distance(&c3d.point_at(0.5 * (t0 + t1)), None);
    let hscale = f64::max(clen, 1e-9);
    let (hmin, hmax, hc) = curve_height_stats(c3d, &recog.p1, &ax);

    if hmax - hmin > hscale * 1e-4 {
        return NurbsCurve::default();
    }

    if curve_gap(c3d) > hscale * 1e-3 {
        return NurbsCurve::default();
    }

    match bisect_height_v(srf, 0.5 * (u0 + u1), v0, v1, hc, &recog.p1, &ax) {
        Some(vc) => iso_v_line(u0, u1, vc),
        None => NurbsCurve::default(),
    }
}

/// Tube angle of p about the circle of radius rmaj around the unit axis w through center.
fn torus_minor_angle(p: &Point, center: &[f64; 3], w: &[f64; 3], rmaj: f64) -> f64 {
    let d = [p[0] - center[0], p[1] - center[1], p[2] - center[2]];
    let z = ssi_dot(&d, w);
    let hx = d[0] - z * w[0];
    let hy = d[1] - z * w[1];
    let hz = d[2] - z * w[2];
    let rho = (hx * hx + hy * hy + hz * hz).sqrt();

    z.atan2(rho - rmaj)
}

/// Range and mean of the unwrapped tube angle over 33 curve samples.
fn torus_angle_stats(
    c3d: &NurbsCurve,
    center: &[f64; 3],
    w: &[f64; 3],
    rmaj: f64,
) -> (f64, f64, f64) {
    let (t0, t1) = c3d.domain();
    let ns = 33;
    let mut aprev = 0.0;
    let mut asum = 0.0;
    let mut amin = 1e300;
    let mut amax = -1e300;

    for i in 0..ns {
        let mut a = torus_minor_angle(
            &c3d.point_at(t0 + (t1 - t0) * i as f64 / 32.0),
            center,
            w,
            rmaj,
        );

        if i > 0 {
            a = unwrap_angle(a, aprev);
        }

        aprev = a;
        amin = f64::min(amin, a);
        amax = f64::max(amax, a);
        asum += a;
    }

    (amin, amax, asum / ns as f64)
}

/// Unwrapped tube angle at 257 samples of the line u = um.
fn torus_angle_table(
    srf: &NurbsSurface,
    um: f64,
    v0: f64,
    v1: f64,
    center: &[f64; 3],
    w: &[f64; 3],
    rmaj: f64,
) -> (Vec<f64>, Vec<f64>) {
    let nv = 256;
    let mut tv = vec![0.0f64; nv + 1];
    let mut ta = vec![0.0f64; nv + 1];
    let mut ap = 0.0;

    for k in 0..=nv {
        let v = v0 + (v1 - v0) * k as f64 / nv as f64;
        let mut a = torus_minor_angle(&srf_point(srf, um, v), center, w, rmaj);

        if k > 0 {
            a = unwrap_angle(a, ap);
        }

        ap = a;
        tv[k] = v;
        ta[k] = a;
    }

    (tv, ta)
}

/// Parameter and value arrays read backwards: the x where the tabulated y reaches y.
fn inverse_table(xs: &[f64], ys: &[f64], y: f64) -> f64 {
    let nt = ys.len() - 1;
    let incr = ys[nt] >= ys[0];
    let y = wrap_to_range(y, f64::min(ys[0], ys[nt]), f64::max(ys[0], ys[nt]));
    let mut lo = 0;
    let mut hi = nt;

    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        let above = if incr { ys[mid] < y } else { ys[mid] > y };

        if above {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let denom = ys[hi] - ys[lo];
    let f = if denom.abs() > 1e-15 {
        (y - ys[lo]) / denom
    } else {
        0.0
    };

    xs[lo] + (xs[hi] - xs[lo]) * f
}

/// Newton-refine v on the line u = um so the tube angle reaches a_target.
#[allow(clippy::too_many_arguments)]
fn torus_refine_v(
    srf: &NurbsSurface,
    um: f64,
    mut vc: f64,
    a_target: f64,
    v0: f64,
    v1: f64,
    center: &[f64; 3],
    w: &[f64; 3],
    rmaj: f64,
) -> f64 {
    let dv = (v1 - v0) * 1e-7;
    let vlo = f64::min(v0, v1);
    let vhi = f64::max(v0, v1);

    for _ in 0..3 {
        let g0 = wrap_angle(
            torus_minor_angle(&srf_point(srf, um, vc.max(vlo).min(vhi)), center, w, rmaj)
                - a_target,
        );
        let vd = f64::min(vc + dv, vhi);
        let g1 = wrap_angle(
            torus_minor_angle(&srf_point(srf, um, vd.max(vlo).min(vhi)), center, w, rmaj)
                - a_target,
        );
        let dg = (g1 - g0) / dv;

        if dg.abs() < 1e-12 {
            break;
        }

        let vn = (vc - g0 / dg).max(vlo).min(vhi);

        if (vn - vc).abs() <= 1e-15 * f64::max(1.0, vc.abs()) {
            vc = vn;
            break;
        }

        vc = vn;
    }

    vc
}

/// Torus pcurve of a circle of constant tube angle: a constant-v line.
fn torus_pcurve(srf: &NurbsSurface, recog: &RecogSurface, c3d: &NurbsCurve) -> NurbsCurve {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let mut w = recog.p2;

    if !normalize_axis(&mut w) {
        return NurbsCurve::default();
    }

    let rmaj = recog.r;
    let rmin = recog.r2;

    if rmin < 1e-12 || rmaj <= rmin {
        return NurbsCurve::default();
    }

    let (amin, amax, a_target) = torus_angle_stats(c3d, &recog.p1, &w, rmaj);

    if amax - amin > 1e-4 {
        return NurbsCurve::default();
    }

    if curve_gap(c3d) > rmin * 1e-3 {
        return NurbsCurve::default();
    }

    let um = 0.5 * (u0 + u1);
    let (tv, ta) = torus_angle_table(srf, um, v0, v1, &recog.p1, &w, rmaj);
    let alo = f64::min(ta[0], ta[ta.len() - 1]);
    let ahi = f64::max(ta[0], ta[ta.len() - 1]);
    let a_target = wrap_to_range(a_target, alo, ahi);

    if a_target < alo - 1e-9 || a_target > ahi + 1e-9 {
        return NurbsCurve::default();
    }

    let vc = torus_refine_v(
        srf,
        um,
        inverse_table(&tv, &ta, a_target),
        a_target,
        v0,
        v1,
        &recog.p1,
        &w,
        rmaj,
    );

    iso_v_line(u0, u1, vc)
}

/// Analytic pcurve of an exact 3D intersection conic on a recognized quadric surface.
fn analytic_pcurve(srf: &NurbsSurface, recog: &RecogSurface, c3d: &NurbsCurve) -> NurbsCurve {
    match recog.kind {
        RecogKind::Plane => plane_pcurve(srf, c3d),
        RecogKind::Cylinder => cylinder_pcurve(srf, recog, c3d),
        RecogKind::Sphere => sphere_pcurve(srf, recog, c3d),
        RecogKind::Cone => cone_pcurve(srf, recog, c3d),
        RecogKind::Torus => torus_pcurve(srf, recog, c3d),
        RecogKind::None => NurbsCurve::default(),
    }
}

/// Orthonormal frame about a surface axis.
#[derive(Clone, Copy)]
struct AxisFrame {
    o: [f64; 3], // Origin on the axis.
    x: [f64; 3], // First radial direction.
    y: [f64; 3], // Second radial direction.
    z: [f64; 3], // Unit axis.
}

/// Frame about the unit axis z through origin with x towards p; None when p lies on the axis.
fn axis_frame(origin: &[f64; 3], z: &[f64; 3], p: &Point) -> Option<AxisFrame> {
    let r = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
    let h = ssi_dot(&r, z);
    let mut x = [r[0] - h * z[0], r[1] - h * z[1], r[2] - h * z[2]];

    if !normalize_axis(&mut x) {
        return None;
    }

    Some(AxisFrame {
        o: *origin,
        x,
        y: ssi_cross(z, &x),
        z: *z,
    })
}

/// Longitude of q about the frame axis.
fn frame_longitude(f: &AxisFrame, q: &Point) -> f64 {
    let r = [q[0] - f.o[0], q[1] - f.o[1], q[2] - f.o[2]];

    ssi_dot(&r, &f.y).atan2(ssi_dot(&r, &f.x))
}

/// Torus tube angle of q for major radius rmaj and minor radius rmin.
fn frame_tube_angle(f: &AxisFrame, rmaj: f64, rmin: f64, q: &Point) -> f64 {
    let rho = axis_radial_sq(q, &f.o, &f.z).sqrt();

    (axis_height(q, &f.o, &f.z) / rmin).atan2((rho - rmaj) / rmin)
}

/// Angle of surface points along one parameter line: longitude, or the torus tube angle.
struct AngleProbe<'a> {
    srf: &'a NurbsSurface, // Sampled surface.
    frame: AxisFrame,      // Frame about the surface axis.
    fixed: f64,            // The parameter held fixed.
    x_is_u: bool,          // Whether the free parameter is u.
    tube: bool,            // Tube angle instead of longitude.
    rmaj: f64,             // Torus major radius.
    rmin: f64,             // Torus minor radius.
}

impl AngleProbe<'_> {
    /// Surface point at free parameter x.
    fn point(&self, x: f64) -> Point {
        if self.x_is_u {
            srf_point(self.srf, x, self.fixed)
        } else {
            srf_point(self.srf, self.fixed, x)
        }
    }

    /// Angle at free parameter x.
    fn angle(&self, x: f64) -> f64 {
        let q = self.point(x);

        if self.tube {
            frame_tube_angle(&self.frame, self.rmaj, self.rmin, &q)
        } else {
            frame_longitude(&self.frame, &q)
        }
    }
}

/// Tabulated angle along one parameter line.
struct AngleMap<'a> {
    probe: AngleProbe<'a>, // Angle along the parameter line.
    lo: f64,               // Parameter start.
    hi: f64,               // Parameter end.
    xs: Vec<f64>,          // Tabulated parameters.
    ys: Vec<f64>,          // Unwrapped angles at xs.
}

/// Tabulate 129 unwrapped angles of the probe over [lo, hi].
fn angle_map(probe: AngleProbe<'_>, lo: f64, hi: f64) -> AngleMap<'_> {
    let nt = 128;
    let range = hi - lo;
    let mut xs = vec![0.0f64; nt + 1];
    let mut ys = vec![0.0f64; nt + 1];

    for k in 0..=nt {
        let x = lo + range * k as f64 / nt as f64;
        let mut y = probe.angle(x);

        if k > 0 {
            y = unwrap_angle(y, ys[k - 1]);
        }

        xs[k] = x;
        ys[k] = y;
    }

    AngleMap {
        probe,
        lo,
        hi,
        xs,
        ys,
    }
}

/// Two Newton steps moving x in [lo, hi] until the probe angle reaches y.
fn polish_angle(probe: &AngleProbe<'_>, mut x: f64, y: f64, lo: f64, hi: f64) -> f64 {
    let dx = (hi - lo) * 1e-7;

    for _ in 0..2 {
        let xc = x.max(lo).min(hi);
        let g0 = wrap_angle(probe.angle(xc) - y);
        let g1 = wrap_angle(probe.angle(f64::min(xc + dx, hi)) - y);
        let dg = (g1 - g0) / dx;

        if dg.abs() < 1e-12 {
            break;
        }

        x = (xc - g0 / dg).max(lo).min(hi);
    }

    x
}

/// Parameter where the tabulated angle reaches y, Newton-polished.
fn map_parameter(m: &AngleMap<'_>, y: f64) -> f64 {
    polish_angle(&m.probe, inverse_table(&m.xs, &m.ys, y), y, m.lo, m.hi)
}

/// Height along the frame axis at 129 samples of the line u = um.
fn height_table(
    srf: &NurbsSurface,
    f: &AxisFrame,
    um: f64,
    v0: f64,
    v1: f64,
) -> (Vec<f64>, Vec<f64>) {
    let nt = 128;
    let mut tv = vec![0.0f64; nt + 1];
    let mut th = vec![0.0f64; nt + 1];

    for k in 0..=nt {
        let v = v0 + (v1 - v0) * k as f64 / nt as f64;
        tv[k] = v;
        th[k] = axis_height(&srf_point(srf, um, v), &f.o, &f.z);
    }

    (tv, th)
}

/// The x where the tabulated y reaches y, clamped to the table ends.
fn clamped_table(xs: &[f64], ys: &[f64], y: f64) -> f64 {
    let nt = ys.len() - 1;
    let incr = ys[nt] >= ys[0];

    if incr && y <= ys[0] {
        return xs[0];
    }

    if incr && y >= ys[nt] {
        return xs[nt];
    }

    if !incr && y >= ys[0] {
        return xs[0];
    }

    if !incr && y <= ys[nt] {
        return xs[nt];
    }

    let mut lo = 0;
    let mut hi = nt;

    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        let above = if incr { ys[mid] < y } else { ys[mid] > y };

        if above {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let denom = ys[hi] - ys[lo];
    let f = if denom.abs() > 1e-15 {
        (y - ys[lo]) / denom
    } else {
        0.0
    };

    xs[lo] + (xs[hi] - xs[lo]) * f
}

/// Two Newton steps moving v on the line u = um until the axial height reaches h.
#[allow(clippy::too_many_arguments)]
fn sphere_refine_v(
    srf: &NurbsSurface,
    f: &AxisFrame,
    um: f64,
    mut v: f64,
    h: f64,
    v0: f64,
    v1: f64,
) -> f64 {
    let vlo = f64::min(v0, v1);
    let vhi = f64::max(v0, v1);

    for _ in 0..2 {
        let dv = (v1 - v0) * 1e-7;
        let vc = v.max(vlo).min(vhi);
        let g0 = axis_height(&srf_point(srf, um, vc), &f.o, &f.z) - h;
        let g1 = axis_height(&srf_point(srf, um, f64::min(vc + dv, vhi)), &f.o, &f.z) - h;
        let dg = (g1 - g0) / dv;

        if dg.abs() < 1e-12 {
            break;
        }

        v = (vc - g0 / dg).max(vlo).min(vhi);
    }

    v
}

/// Seam-free run of pull-back samples with the 3D curve parameter of each.
#[derive(Clone, Default)]
struct PullbackRun {
    uv: Vec<Point>, // Samples in surface parameters.
    ts: Vec<f64>,   // 3D curve parameter of each sample.
}

/// Append the sample (u, v) at 3D curve parameter t to the run.
fn push_run_sample(run: &mut PullbackRun, u: f64, v: f64, t: f64) {
    run.uv.push(Point::new(u, v, 0.0));
    run.ts.push(t);
}

/// Drop an end run shorter than a hundredth of the sample step: an end sample lying just across a seam.
fn drop_seam_slivers(runs: &mut Vec<PullbackRun>, step: f64) {
    if runs.len() > 1 {
        let last = &runs[runs.len() - 1];

        if last.ts[last.ts.len() - 1] - last.ts[0] < step * 0.01 {
            runs.pop();
        }
    }

    if runs.len() > 1 {
        let first = &runs[0];

        if first.ts[first.ts.len() - 1] - first.ts[0] < step * 0.01 {
            runs.remove(0);
        }
    }
}

/// Runs of (u, v, t) samples with u unwrapped, split where u crosses the seam.
fn split_pullback_u(uv: &[[f64; 3]], u0: f64, range_u: f64) -> Vec<PullbackRun> {
    let mut out = Vec::new();
    let mut seg = PullbackRun::default();
    let mut cur_k = period_index(uv[0][0], u0, range_u);
    push_run_sample(
        &mut seg,
        uv[0][0] - cur_k as f64 * range_u,
        uv[0][1],
        uv[0][2],
    );

    for i in 1..uv.len() {
        let ki = period_index(uv[i][0], u0, range_u);

        while ki != cur_k {
            let step = if ki > cur_k { 1 } else { -1 };
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
            let tc = uv[i - 1][2] + (uv[i][2] - uv[i - 1][2]) * f;
            push_run_sample(&mut seg, seam_cont - cur_k as f64 * range_u, vc, tc);

            if seg.uv.len() >= 2 {
                out.push(seg);
            }

            seg = PullbackRun::default();
            push_run_sample(&mut seg, seam_cont - nk as f64 * range_u, vc, tc);
            cur_k = nk;
        }

        push_run_sample(
            &mut seg,
            uv[i][0] - cur_k as f64 * range_u,
            uv[i][1],
            uv[i][2],
        );
    }

    if seg.uv.len() >= 2 {
        out.push(seg);
    }

    drop_seam_slivers(
        &mut out,
        (uv[uv.len() - 1][2] - uv[0][2]) / (uv.len() - 1) as f64,
    );

    out
}

/// Pull a 3D curve back to sphere parameters through longitude and latitude.
fn analytic_sphere_pullback(
    srf: &NurbsSurface,
    recog: &RecogSurface,
    c3d: &NurbsCurve,
) -> Vec<PullbackRun> {
    if recog.kind != RecogKind::Sphere {
        return vec![];
    }

    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let range_u = u1 - u0;

    if range_u < 1e-9 {
        return vec![];
    }

    let um = 0.5 * (u0 + u1);
    let vm = 0.5 * (v0 + v1);
    let sp = srf_point(srf, um, v0);
    let np = srf_point(srf, um, v1);
    let mut axis = [np[0] - sp[0], np[1] - sp[1], np[2] - sp[2]];

    if !normalize_axis(&mut axis) {
        return vec![];
    }

    let frame = match axis_frame(&recog.p1, &axis, &srf_point(srf, u0, vm)) {
        Some(frame) => frame,
        None => return vec![],
    };
    let probe = AngleProbe {
        srf,
        frame,
        fixed: vm,
        x_is_u: true,
        tube: false,
        rmaj: 0.0,
        rmin: 0.0,
    };
    let lon_map = angle_map(probe, u0, u1);
    let (tv, th) = height_table(srf, &frame, um, v0, v1);

    if (th[th.len() - 1] - th[0]).abs() < 1e-12 {
        return vec![];
    }

    let (t0, t1) = c3d.domain();
    let n = usize::max(c3d.cv_count() * 8, 120);
    let mut uv = Vec::new();
    let mut prev_u = 0.0;

    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let p = c3d.point_at(t);
        let h = axis_height(&p, &frame.o, &frame.z);
        let mut u = map_parameter(&lon_map, frame_longitude(&frame, &p));
        let v = sphere_refine_v(srf, &frame, um, clamped_table(&tv, &th, h), h, v0, v1);

        if i > 0 {
            u = unwrap_period(u, prev_u, range_u);
        }

        prev_u = u;
        uv.push([u, v, t]);
    }

    split_pullback_u(&uv, u0, range_u)
}

/// Samples (u, v, t) with u unwrapped; an apex sample (u NaN) takes the u of the generator it lies on.
fn fill_apex_samples(raw: &[[f64; 3]], u0: f64, range_u: f64) -> Vec<[f64; 3]> {
    let mut uv: Vec<[f64; 3]> = Vec::new();

    for i in 0..raw.len() {
        let s = raw[i];

        if !s[0].is_nan() {
            let u = if uv.is_empty() {
                s[0]
            } else {
                unwrap_period(s[0], uv[uv.len() - 1][0], range_u)
            };
            uv.push([u, s[1], s[2]]);
            continue;
        }

        let mut j = i + 1;

        while j < raw.len() && raw[j][0].is_nan() {
            j += 1;
        }

        if uv.is_empty() {
            uv.push([if j < raw.len() { raw[j][0] } else { u0 }, s[1], s[2]]);
            continue;
        }

        let u_prev = uv[uv.len() - 1][0];
        uv.push([u_prev, s[1], s[2]]);

        if j != i + 1 || j == raw.len() {
            continue;
        }

        let u_next = raw[j][0]
            + (period_index(u_prev, u0, range_u) - period_index(raw[j][0], u0, range_u)) as f64
                * range_u;

        if (u_next - u_prev).abs() > range_u * 1e-12 {
            uv.push([u_next, s[1], s[2]]);
        }
    }

    uv
}

/// Samples (u, v, t) of a curve on a cone or cylinder, u unwrapped and v linear in the axial height.
#[allow(clippy::too_many_arguments)]
fn cone_pullback_samples(
    c3d: &NurbsCurve,
    frame: &AxisFrame,
    lon_map: &AngleMap<'_>,
    h0: f64,
    h1: f64,
    v0: f64,
    v1: f64,
) -> Vec<[f64; 3]> {
    let (t0, t1) = c3d.domain();
    let apex_tol = (h1 - h0).abs() * 1e-9;
    let n = usize::max(c3d.cv_count() * 8, 120);
    let mut raw = Vec::new();

    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let p = c3d.point_at(t);
        let r = [p[0] - frame.o[0], p[1] - frame.o[1], p[2] - frame.o[2]];
        let rx = ssi_dot(&r, &frame.x);
        let ry = ssi_dot(&r, &frame.y);
        let rad = (rx * rx + ry * ry).sqrt();
        let u = if rad > apex_tol {
            map_parameter(lon_map, ry.atan2(rx))
        } else {
            f64::NAN
        };
        let v = v0 + (ssi_dot(&r, &frame.z) - h0) / (h1 - h0) * (v1 - v0);
        raw.push([u, v, t]);
    }

    fill_apex_samples(&raw, lon_map.lo, lon_map.hi - lon_map.lo)
}

/// Analytic pull-back of a 3D curve onto a recognized cone or cylinder.
fn analytic_cone_pullback(
    srf: &NurbsSurface,
    recog: &RecogSurface,
    c3d: &NurbsCurve,
) -> Vec<PullbackRun> {
    if recog.kind != RecogKind::Cone && recog.kind != RecogKind::Cylinder {
        return vec![];
    }

    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let range_u = u1 - u0;
    let mut axis = recog.p2;

    if range_u < 1e-9 || !normalize_axis(&mut axis) {
        return vec![];
    }

    let um = 0.5 * (u0 + u1);
    let h0 = axis_height(&srf_point(srf, um, v0), &recog.p1, &axis);
    let h1 = axis_height(&srf_point(srf, um, v1), &recog.p1, &axis);

    if (h1 - h0).abs() < 1e-12 {
        return vec![];
    }

    let v_ref = if h0.abs() >= h1.abs() { v0 } else { v1 };
    let frame = match axis_frame(&recog.p1, &axis, &srf_point(srf, u0, v_ref)) {
        Some(frame) => frame,
        None => return vec![],
    };
    let probe = AngleProbe {
        srf,
        frame,
        fixed: v_ref,
        x_is_u: true,
        tube: false,
        rmaj: 0.0,
        rmin: 0.0,
    };
    let lon_map = angle_map(probe, u0, u1);
    let uv = cone_pullback_samples(c3d, &frame, &lon_map, h0, h1, v0, v1);

    split_pullback_u(&uv, u0, range_u)
}

/// Period cells of a torus pull-back in (a, b), swapped when a is the surface v.
struct PeriodGrid {
    a0: f64,       // Start of a.
    range_a: f64,  // Period of a.
    b0: f64,       // Start of b.
    range_b: f64,  // Period of b.
    swapped: bool, // Whether a is the surface v.
}

/// Append the sample (a, b, t) shifted into cell (ka, kb) in surface (u, v) order.
fn push_pullback_point(g: &PeriodGrid, seg: &mut PullbackRun, p: &[f64; 3], ka: i32, kb: i32) {
    let uu = p[0] - ka as f64 * g.range_a;
    let vv = p[1] - kb as f64 * g.range_b;

    if g.swapped {
        push_run_sample(seg, vv, uu, p[2]);
    } else {
        push_run_sample(seg, uu, vv, p[2]);
    }
}

/// Split the step p -> q at its first cell boundary, advancing p; false when q is in the current cell.
fn cross_period(
    g: &PeriodGrid,
    p: &mut [f64; 3],
    q: &[f64; 3],
    ka: &mut i32,
    kb: &mut i32,
    seg: &mut PullbackRun,
    out: &mut Vec<PullbackRun>,
) -> bool {
    let kqa = period_index(q[0], g.a0, g.range_a);
    let kqb = period_index(q[1], g.b0, g.range_b);

    if kqa == *ka && kqb == *kb {
        return false;
    }

    let mut fa = 2.0;
    let mut fb = 2.0;
    let mut sa = 0;
    let mut sb = 0;

    if kqa != *ka {
        sa = if kqa > *ka { 1 } else { -1 };
        let bound = g.a0 + (if sa > 0 { *ka + 1 } else { *ka }) as f64 * g.range_a;
        let den = q[0] - p[0];
        fa = if den.abs() > 1e-15 {
            (bound - p[0]) / den
        } else {
            0.0
        };
    }

    if kqb != *kb {
        sb = if kqb > *kb { 1 } else { -1 };
        let bound = g.b0 + (if sb > 0 { *kb + 1 } else { *kb }) as f64 * g.range_b;
        let den = q[1] - p[1];
        fb = if den.abs() > 1e-15 {
            (bound - p[1]) / den
        } else {
            0.0
        };
    }

    let f = f64::min(fa, fb).clamp(0.0, 1.0);
    let mut c = [
        p[0] + (q[0] - p[0]) * f,
        p[1] + (q[1] - p[1]) * f,
        p[2] + (q[2] - p[2]) * f,
    ];

    if fa <= fb {
        c[0] = g.a0 + (if sa > 0 { *ka + 1 } else { *ka }) as f64 * g.range_a;
    } else {
        c[1] = g.b0 + (if sb > 0 { *kb + 1 } else { *kb }) as f64 * g.range_b;
    }

    push_pullback_point(g, seg, &c, *ka, *kb);

    if seg.uv.len() >= 2 {
        out.push(std::mem::take(seg));
    }

    *seg = PullbackRun::default();

    if fa <= fb {
        *ka += sa;
    } else {
        *kb += sb;
    }

    push_pullback_point(g, seg, &c, *ka, *kb);
    *p = c;

    true
}

/// Runs of (a, b, t) samples with a and b unwrapped, split at both seams.
fn split_pullback_ab(ab: &[[f64; 3]], g: &PeriodGrid) -> Vec<PullbackRun> {
    let mut out = Vec::new();
    let mut seg = PullbackRun::default();
    let mut ka = period_index(ab[0][0], g.a0, g.range_a);
    let mut kb = period_index(ab[0][1], g.b0, g.range_b);
    push_pullback_point(g, &mut seg, &ab[0], ka, kb);

    for i in 1..ab.len() {
        let mut p = ab[i - 1];

        for _ in 0..8 {
            if !cross_period(g, &mut p, &ab[i], &mut ka, &mut kb, &mut seg, &mut out) {
                break;
            }
        }

        push_pullback_point(g, &mut seg, &ab[i], ka, kb);
    }

    if seg.uv.len() >= 2 {
        out.push(seg);
    }

    drop_seam_slivers(
        &mut out,
        (ab[ab.len() - 1][2] - ab[0][2]) / (ab.len() - 1) as f64,
    );

    out
}

/// Sample of a 5 x 5 grid farthest from the unit axis through center.
fn farthest_from_axis(srf: &NurbsSurface, center: &[f64; 3], axis: &[f64; 3]) -> Point {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let mut pf = srf_point(srf, u0, v0);
    let mut best = -1.0;

    for i in 0..=4 {
        for j in 0..=4 {
            let q = srf_point(
                srf,
                u0 + (u1 - u0) * i as f64 / 4.0,
                v0 + (v1 - v0) * j as f64 / 4.0,
            );
            let d = axis_radial_sq(&q, center, axis);

            if d > best {
                best = d;
                pf = q;
            }
        }
    }

    pf
}

/// Whether the torus's longitude runs along v rather than u.
fn torus_swapped(srf: &NurbsSurface, frame: &AxisFrame) -> bool {
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let um = 0.5 * (u0 + u1);
    let vm = 0.5 * (v0 + v1);
    let lu1 = frame_longitude(frame, &srf_point(srf, u0 + 0.6 * (u1 - u0), vm));
    let lu0 = frame_longitude(frame, &srf_point(srf, u0 + 0.3 * (u1 - u0), vm));
    let lv1 = frame_longitude(frame, &srf_point(srf, um, v0 + 0.6 * (v1 - v0)));
    let lv0 = frame_longitude(frame, &srf_point(srf, um, v0 + 0.3 * (v1 - v0)));

    wrap_angle(lv1 - lv0).abs() > wrap_angle(lu1 - lu0).abs()
}

/// Free parameter of 17 samples along the probe line farthest from the axis.
fn farthest_on_line(probe: &AngleProbe<'_>, lo: f64, hi: f64) -> f64 {
    let range = hi - lo;
    let mut x_ref = lo;
    let mut best = -1.0;

    for j in 0..=16 {
        let x = lo + range * j as f64 / 16.0;
        let d = axis_radial_sq(&probe.point(x), &probe.frame.o, &probe.frame.z);

        if d > best {
            best = d;
            x_ref = x;
        }
    }

    x_ref
}

/// Analytic pull-back of a 3D curve onto a recognized torus.
fn analytic_torus_pullback(
    srf: &NurbsSurface,
    recog: &RecogSurface,
    c3d: &NurbsCurve,
) -> Vec<PullbackRun> {
    let domain_u = srf_domain(srf, 0);
    let domain_v = srf_domain(srf, 1);
    let mut axis = recog.p2;
    let rmaj = recog.r;
    let rmin = recog.r2;

    if recog.kind != RecogKind::Torus
        || domain_u.1 - domain_u.0 < 1e-9
        || domain_v.1 - domain_v.0 < 1e-9
    {
        return vec![];
    }

    if !normalize_axis(&mut axis) || rmaj < 1e-12 || rmin < 1e-12 {
        return vec![];
    }

    let frame = match axis_frame(&recog.p1, &axis, &farthest_from_axis(srf, &recog.p1, &axis)) {
        Some(frame) => frame,
        None => return vec![],
    };
    let swapped = torus_swapped(srf, &frame);
    let (a0, a1) = if swapped { domain_v } else { domain_u };
    let (b0, b1) = if swapped { domain_u } else { domain_v };
    let tube_probe = AngleProbe {
        srf,
        frame,
        fixed: 0.5 * (a0 + a1),
        x_is_u: swapped,
        tube: true,
        rmaj,
        rmin,
    };
    let b_ref = farthest_on_line(&tube_probe, b0, b1);
    let lon_probe = AngleProbe {
        srf,
        frame,
        fixed: b_ref,
        x_is_u: !swapped,
        tube: false,
        rmaj: 0.0,
        rmin: 0.0,
    };
    let lon_map = angle_map(lon_probe, a0, a1);
    let tube_map = angle_map(tube_probe, b0, b1);
    let (t0, t1) = c3d.domain();
    let n = usize::max(c3d.cv_count() * 8, 4000);
    let mut ab = Vec::new();
    let mut prev_a = 0.0;
    let mut prev_b = 0.0;

    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let q = c3d.point_at(t);
        let mut a = map_parameter(&lon_map, frame_longitude(&frame, &q));
        let mut b = map_parameter(&tube_map, frame_tube_angle(&frame, rmaj, rmin, &q));

        if i > 0 {
            a = unwrap_period(a, prev_a, a1 - a0);
            b = unwrap_period(b, prev_b, b1 - b0);
        }

        prev_a = a;
        prev_b = b;
        ab.push([a, b, t]);
    }

    let grid = PeriodGrid {
        a0,
        range_a: a1 - a0,
        b0,
        range_b: b1 - b0,
        swapped,
    };

    split_pullback_ab(&ab, &grid)
}

/// Analytic pull-back matching the recognized kind: sphere, cone or cylinder, torus.
fn analytic_pullback(
    srf: &NurbsSurface,
    recog: &RecogSurface,
    c3d: &NurbsCurve,
) -> Vec<PullbackRun> {
    match recog.kind {
        RecogKind::Torus => analytic_torus_pullback(srf, recog, c3d),
        RecogKind::Sphere => analytic_sphere_pullback(srf, recog, c3d),
        RecogKind::Cone | RecogKind::Cylinder => analytic_cone_pullback(srf, recog, c3d),
        _ => vec![],
    }
}

/// Degree-1 pcurves of the pull-back runs.
fn run_curves(runs: &[PullbackRun]) -> Vec<NurbsCurve> {
    let mut out = Vec::new();

    for run in runs {
        out.push(NurbsCurve::create(false, 1, &run.uv));
    }

    out
}

/// Run sample interpolated at 3D curve parameter t.
fn run_point(run: &PullbackRun, t: f64) -> Point {
    for i in 0..run.ts.len() - 1 {
        if t > run.ts[i + 1] {
            continue;
        }

        let dt = run.ts[i + 1] - run.ts[i];
        let f = if dt > 0.0 {
            ((t - run.ts[i]) / dt).clamp(0.0, 1.0)
        } else {
            0.0
        };

        return Point::new(
            run.uv[i][0] + (run.uv[i + 1][0] - run.uv[i][0]) * f,
            run.uv[i][1] + (run.uv[i + 1][1] - run.uv[i][1]) * f,
            0.0,
        );
    }

    run.uv[run.uv.len() - 1].clone()
}

/// Samples of the run covering the 3D curve span [lo, hi], empty when none does.
fn run_span_points(runs: &[PullbackRun], lo: f64, hi: f64) -> Vec<Point> {
    let mid = 0.5 * (lo + hi);

    for run in runs {
        if mid < run.ts[0] || mid > run.ts[run.ts.len() - 1] {
            continue;
        }

        let mut pts = vec![run_point(run, lo)];

        for i in 0..run.ts.len() {
            if run.ts[i] > lo && run.ts[i] < hi {
                pts.push(run.uv[i].clone());
            }
        }

        pts.push(run_point(run, hi));

        return pts;
    }

    vec![]
}

/// Degree-1 pcurve of the runs over [lo, hi]; a span past the domain end wraps onto the start of the closed curve.
fn run_span(runs: &[PullbackRun], domain: (f64, f64), lo: f64, hi: f64) -> NurbsCurve {
    let period = domain.1 - domain.0;
    let mut pts = run_span_points(runs, lo, f64::min(hi, domain.1));

    if hi > domain.1 {
        let head = run_span_points(runs, domain.0, hi - period);

        if !head.is_empty() {
            pts.extend_from_slice(&head[1..]);
        }
    }

    if pts.len() < 2 {
        return NurbsCurve::default();
    }

    NurbsCurve::create(false, 1, &pts)
}
// ═══════════════════════════════════════════════════════════════════════════
// Coaxial quadric pairs
// ═══════════════════════════════════════════════════════════════════════════
/// Distance of p from the axis through apt along adir.
fn point_axis_dist(apt: &[f64; 3], adir: &[f64; 3], p: &[f64; 3]) -> f64 {
    let u = ssi_unit(adir);
    let dp = [p[0] - apt[0], p[1] - apt[1], p[2] - apt[2]];
    let t = ssi_dot(&dp, &u);
    let perp = [dp[0] - t * u[0], dp[1] - t * u[1], dp[2] - t * u[2]];

    ssi_dot(&perp, &perp).sqrt()
}

/// Coordinate of p along the axis through apt along adir.
fn axial_coord(apt: &[f64; 3], adir: &[f64; 3], p: &[f64; 3]) -> f64 {
    let u = ssi_unit(adir);

    (p[0] - apt[0]) * u[0] + (p[1] - apt[1]) * u[1] + (p[2] - apt[2]) * u[2]
}

/// Whether two axes coincide within tol.
fn axes_coaxial(p1: &[f64; 3], d1: &[f64; 3], p2: &[f64; 3], d2: &[f64; 3], tol: f64) -> bool {
    let u1 = ssi_unit(d1);
    let u2 = ssi_unit(d2);
    let cx = ssi_cross(&u1, &u2);

    if ssi_dot(&cx, &cx).sqrt() > tol {
        return false;
    }

    point_axis_dist(p1, &u1, p2) <= tol
}

/// Axial extent of the surface along the cylinder axis.
fn cyl_span(srf: &NurbsSurface, apt: &[f64; 3], adir: &[f64; 3]) -> (f64, f64) {
    let u = ssi_unit(adir);
    let (u0, u1) = srf_domain(srf, 0);
    let (v0, v1) = srf_domain(srf, 1);
    let um = 0.5 * (u0 + u1);
    let mut smin = 1e300;
    let mut smax = -1e300;

    for vv in [v0, v1] {
        let p = srf_point(srf, um, vv);
        let s = (p[0] - apt[0]) * u[0] + (p[1] - apt[1]) * u[1] + (p[2] - apt[2]) * u[2];
        smin = f64::min(smin, s);
        smax = f64::max(smax, s);
    }

    (smin, smax)
}

/// Closest point of two lines, None when parallel or apart.
fn lines_closest_point(
    p1: &[f64; 3],
    d1: &[f64; 3],
    p2: &[f64; 3],
    d2: &[f64; 3],
    tol: f64,
) -> Option<[f64; 3]> {
    let u = ssi_unit(d1);
    let v = ssi_unit(d2);
    let w0 = [p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]];
    let a = ssi_dot(&u, &u);
    let b = ssi_dot(&u, &v);
    let c = ssi_dot(&v, &v);
    let d = ssi_dot(&u, &w0);
    let e = ssi_dot(&v, &w0);
    let den = a * c - b * b;

    if den.abs() < 1e-12 {
        return None;
    }

    let sc = (b * e - c * d) / den;
    let tc = (a * e - b * d) / den;
    let q1 = [p1[0] + sc * u[0], p1[1] + sc * u[1], p1[2] + sc * u[2]];
    let q2 = [p2[0] + tc * v[0], p2[1] + tc * v[1], p2[2] + tc * v[2]];
    let diff = [q1[0] - q2[0], q1[1] - q2[1], q1[2] - q2[2]];

    if ssi_dot(&diff, &diff).sqrt() > tol {
        return None;
    }

    Some([
        0.5 * (q1[0] + q2[0]),
        0.5 * (q1[1] + q2[1]),
        0.5 * (q1[2] + q2[2]),
    ])
}

/// Circles of radius rad around the unit axis w through center at axial offsets zs.
fn axis_circles(center: &[f64; 3], w: &[f64; 3], zs: &[f64], rad: f64, out: &mut Vec<NurbsCurve>) {
    let (xa, ya) = ortho_basis(w);

    for z in zs {
        let cc = [
            center[0] + z * w[0],
            center[1] + z * w[1],
            center[2] + z * w[2],
        ];
        out.push(exact_circle(cc[0], cc[1], cc[2], &xa, &ya, rad));
    }
}

/// Coaxial cylinder-sphere section: circles.
fn ssi_cylinder_sphere(cyl: &RecogSurface, sph: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let p = cyl.p1;
    let w = ssi_unit(&cyl.p2);
    let rc = cyl.r;
    let center = sph.p1;
    let rsph = sph.r;

    if point_axis_dist(&p, &w, &center) > ktol {
        return false;
    }

    if rsph < rc - ktol {
        return true;
    }

    let dist = f64::max(0.0, rsph * rsph - rc * rc).sqrt();

    if dist <= ktol {
        let (xa, ya) = ortho_basis(&w);
        out.push(exact_circle(center[0], center[1], center[2], &xa, &ya, rc));

        return true;
    }

    axis_circles(&center, &w, &[dist, -dist], rc, out);

    true
}

/// Coaxial cylinder-cone section: circles.
fn ssi_cylinder_cone(cyl: &RecogSurface, cone: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let pc = cyl.p1;
    let w = ssi_unit(&cyl.p2);
    let rc = cyl.r;
    let apex = cone.p1;
    let a = ssi_unit(&cone.p2);
    let alpha = cone.r;

    if !axes_coaxial(&pc, &w, &apex, &a, ktol) {
        return false;
    }

    let ta = alpha.tan();

    if ta < 1e-9 {
        return false;
    }

    let s = rc / ta;

    if s < ktol {
        return true;
    }

    axis_circles(&apex, &a, &[s], rc, out);

    true
}

/// Coaxial cone-sphere section: circles.
fn ssi_cone_sphere(cone: &RecogSurface, sph: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let apex = cone.p1;
    let a = ssi_unit(&cone.p2);
    let alpha = cone.r;
    let center = sph.p1;
    let rsph = sph.r;

    if point_axis_dist(&apex, &a, &center) > ktol {
        return false;
    }

    let dsign = axial_coord(&apex, &a, &center);
    let d = dsign.abs();
    let dir = if d > ktol && dsign < 0.0 {
        [-a[0], -a[1], -a[2]]
    } else {
        a
    };
    let t = alpha.tan();
    let t2 = t * t;
    let qa = 1.0 + t2;
    let qb = 2.0 * t2 * d;
    let qc = t2 * d * d - rsph * rsph;
    let disc = qb * qb - 4.0 * qa * qc;

    if disc < -ktol {
        return true;
    }

    let sq = f64::max(0.0, disc).sqrt();
    let xs = if sq <= ktol {
        vec![-qb / (2.0 * qa)]
    } else {
        vec![(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)]
    };
    let (xa, ya) = ortho_basis(&a);

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
            apex[0] + s_ax * dir[0],
            apex[1] + s_ax * dir[1],
            apex[2] + s_ax * dir[2],
        ];
        out.push(exact_circle(cc[0], cc[1], cc[2], &xa, &ya, rr));
    }

    true
}

/// Points where the cross-section circles of two parallel cylinders at axis distance d meet, one when they touch.
fn parallel_cylinder_feet(
    p1: &[f64; 3],
    w1: &[f64; 3],
    r1: f64,
    p2: &[f64; 3],
    r2: f64,
    d: f64,
    ktol: f64,
) -> Vec<[f64; 3]> {
    let off = ssi_dot(&[p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]], w1);
    let p2p = [
        p2[0] - off * w1[0],
        p2[1] - off * w1[1],
        p2[2] - off * w1[2],
    ];
    let xdir = ssi_unit(&[p2p[0] - p1[0], p2p[1] - p1[1], p2p[2] - p1[2]]);
    let ydir = ssi_unit(&ssi_cross(w1, &xdir));
    let aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h = f64::max(0.0, r1 * r1 - aa * aa).sqrt();
    let foot = [
        p1[0] + aa * xdir[0],
        p1[1] + aa * xdir[1],
        p1[2] + aa * xdir[2],
    ];
    let mut feet = Vec::new();

    if h <= ktol {
        feet.push(foot);
    } else {
        feet.push([
            foot[0] + h * ydir[0],
            foot[1] + h * ydir[1],
            foot[2] + h * ydir[2],
        ]);
        feet.push([
            foot[0] - h * ydir[0],
            foot[1] - h * ydir[1],
            foot[2] - h * ydir[2],
        ]);
    }

    feet
}

/// Parallel cylinders: shared ruling lines, false when coaxial with equal radii.
fn ssi_parallel_cylinders(
    sa: &NurbsSurface,
    ra: &RecogSurface,
    sb: &NurbsSurface,
    rb: &RecogSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    let ktol = 1e-6;
    let p1 = ra.p1;
    let w1 = ssi_unit(&ra.p2);
    let r1 = ra.r;
    let p2 = rb.p1;
    let r2 = rb.r;
    let d = point_axis_dist(&p1, &w1, &p2);

    if d <= ktol {
        if (r1 - r2).abs() <= ktol {
            return false;
        }

        return true;
    }

    if d > r1 + r2 + ktol || d < (r1 - r2).abs() - ktol {
        return true;
    }

    let (s0a, s1a) = cyl_span(sa, &p1, &w1);
    let (s0b, s1b) = cyl_span(sb, &p1, &w1);
    let slo = f64::max(s0a, s0b);
    let shi = f64::min(s1a, s1b);

    if shi - slo <= ktol {
        return true;
    }

    for bp in &parallel_cylinder_feet(&p1, &w1, r1, &p2, r2, d, ktol) {
        let mut line = axis_segment(bp, &w1, slo, shi);
        line.set_domain(0.0, 1.0);
        out.push(line);
    }

    true
}

/// Cylinder-cylinder section: lines when parallel, Steinmetz ellipses when equal axes meet.
fn ssi_cylinder_cylinder(
    sa: &NurbsSurface,
    ra: &RecogSurface,
    sb: &NurbsSurface,
    rb: &RecogSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    let ktol = 1e-6;
    let p1 = ra.p1;
    let w1 = ssi_unit(&ra.p2);
    let r1 = ra.r;
    let p2 = rb.p1;
    let w2 = ssi_unit(&rb.p2);
    let r2 = rb.r;
    let cx = ssi_cross(&w1, &w2);

    if ssi_dot(&cx, &cx).sqrt() <= ktol {
        return ssi_parallel_cylinders(sa, ra, sb, rb, out);
    }

    let rmax = f64::max(r1, r2);

    if rmax < 1e-12 || (r1 - r2).abs() / rmax > 1e-6 {
        return false;
    }

    let pint = match lines_closest_point(&p1, &w1, &p2, &w2, ktol) {
        Some(pint) => pint,
        None => return false,
    };
    let r = 0.5 * (r1 + r2);
    let ang = ssi_dot(&w1, &w2).clamp(-1.0, 1.0).acos();
    let sh = (0.5 * ang).sin();
    let ch = (0.5 * ang).cos();

    if sh < 1e-9 || ch < 1e-9 {
        return false;
    }

    let minor = ssi_unit(&cx);
    let maj1 = ssi_unit(&[w1[0] + w2[0], w1[1] + w2[1], w1[2] + w2[2]]);
    let maj2 = ssi_unit(&[w1[0] - w2[0], w1[1] - w2[1], w1[2] - w2[2]]);
    out.push(exact_ellipse(
        pint[0],
        pint[1],
        pint[2],
        &maj1,
        &minor,
        r / sh,
        r,
    ));
    out.push(exact_ellipse(
        pint[0],
        pint[1],
        pint[2],
        &maj2,
        &minor,
        r / ch,
        r,
    ));

    true
}

/// Exact circles of a coaxial cylinder-torus pair.
fn ssi_cylinder_torus(cyl: &RecogSurface, tor: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let p = cyl.p1;
    let wc = ssi_unit(&cyl.p2);
    let rc = cyl.r;
    let center = tor.p1;
    let w = ssi_unit(&tor.p2);
    let rmaj = tor.r;
    let r = tor.r2;

    if r >= rmaj - ktol {
        return false;
    }

    if !axes_coaxial(&p, &wc, &center, &w, ktol) {
        return false;
    }

    let dr = rc - rmaj;
    let h2 = r * r - dr * dr;

    if h2 < -ktol {
        return true;
    }

    let h = f64::max(0.0, h2).sqrt();

    if h <= ktol {
        axis_circles(&center, &w, &[0.0], rc, out);
    } else {
        axis_circles(&center, &w, &[h, -h], rc, out);
    }

    true
}

/// Circles of a coaxial cone and one side (rsign = +rmaj or -rmaj) of the torus tube.
fn cone_torus_circles(
    center: &[f64; 3],
    w: &[f64; 3],
    t: f64,
    za: f64,
    r: f64,
    rsign: f64,
    out: &mut Vec<NurbsCurve>,
) {
    let ktol = 1e-6;
    let qa = t * t + 1.0;
    let qb = -2.0 * t * (t * za + rsign);
    let qc = (t * za + rsign) * (t * za + rsign) - r * r;
    let disc = qb * qb - 4.0 * qa * qc;

    if disc < -ktol {
        return;
    }

    let sq = f64::max(0.0, disc).sqrt();
    let zs = if sq <= ktol {
        vec![-qb / (2.0 * qa)]
    } else {
        vec![(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)]
    };

    for z in zs {
        let rad = t * (z - za).abs();

        if rad < ktol {
            continue;
        }

        axis_circles(center, w, &[z], rad, out);
    }
}

/// Coaxial cone-torus section: circles.
fn ssi_cone_torus(cone: &RecogSurface, tor: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let apex = cone.p1;
    let a = ssi_unit(&cone.p2);
    let alpha = cone.r;
    let center = tor.p1;
    let w = ssi_unit(&tor.p2);
    let rmaj = tor.r;
    let r = tor.r2;

    if r >= rmaj - ktol {
        return false;
    }

    if !axes_coaxial(&apex, &a, &center, &w, ktol) {
        return false;
    }

    let t = alpha.tan();

    if t < 1e-9 {
        return false;
    }

    let za = axial_coord(&center, &w, &apex);
    cone_torus_circles(&center, &w, t, za, r, rmaj, out);
    cone_torus_circles(&center, &w, t, za, r, -rmaj, out);

    true
}

/// Circles where the tube circle (rmaj, 0) of radius r meets the circle of radius r2 at offset (dx, dz) from it.
#[allow(clippy::too_many_arguments)]
fn meridian_circles(
    center: &[f64; 3],
    w: &[f64; 3],
    rmaj: f64,
    r: f64,
    dx: f64,
    dz: f64,
    r2: f64,
    out: &mut Vec<NurbsCurve>,
) {
    let ktol = 1e-6;
    let d = (dx * dx + dz * dz).sqrt();
    let aa = 0.5 * (r * r - r2 * r2 + d * d) / d;
    let h = f64::max(0.0, r * r - aa * aa).sqrt();
    let dirx = dx / d;
    let dirz = dz / d;
    let phx = rmaj + aa * dirx;
    let phz = aa * dirz;
    let perpx = -dirz;
    let perpz = dirx;
    let signs: Vec<f64> = if h <= ktol {
        vec![0.0]
    } else {
        vec![1.0, -1.0]
    };

    for s in signs {
        let xi = phx + s * h * perpx;
        let z = phz + s * h * perpz;
        let rad = xi.abs();

        if rad < ktol {
            continue;
        }

        axis_circles(center, w, &[z], rad, out);
    }
}

/// Coaxial sphere-torus section: circles.
fn ssi_sphere_torus(sph: &RecogSurface, tor: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let sc = sph.p1;
    let rsph = sph.r;
    let center = tor.p1;
    let w = ssi_unit(&tor.p2);
    let rmaj = tor.r;
    let r = tor.r2;

    if r >= rmaj - ktol {
        return false;
    }

    if point_axis_dist(&center, &w, &sc) > ktol {
        return false;
    }

    let zs = axial_coord(&center, &w, &sc);
    let d = (rmaj * rmaj + zs * zs).sqrt();

    if d < ktol {
        return true;
    }

    if d - ktol > r + rsph || d + ktol < (r - rsph).abs() {
        return true;
    }

    meridian_circles(&center, &w, rmaj, r, 0.0 - rmaj, zs - 0.0, rsph, out);

    true
}

/// Spiric loop frame of two equal parallel-axis tori.
struct SpiricFrame {
    c1: [f64; 3], // First torus center.
    ex: [f64; 3], // Unit direction between the centers.
    ey: [f64; 3], // Unit axis cross ex.
    w: [f64; 3],  // Unit common axis.
    rmaj: f64,    // Major radius.
    r: f64,       // Minor radius.
    c: f64,       // Half the center distance.
    be: f64,      // Semi-axis of the inner loops.
}

/// In-plane point (x, y) of a spiric loop at tube offset t; None when the loop does not reach t.
fn spiric_xy(f: &SpiricFrame, inner: bool, t: f64) -> Option<(f64, f64)> {
    if inner {
        let g = t / f.c;

        if g.abs() >= 1.0 {
            return None;
        }

        return Some((f.c + f.rmaj * g, f.be * (1.0 - g * g).sqrt()));
    }

    let rho = f.rmaj + t;
    let y2 = rho * rho - f.c * f.c;

    if y2 <= 0.0 {
        return None;
    }

    Some((f.c, y2.sqrt()))
}

/// Both mirrored spiric loops as periodic interpolants, none when either misses a sample.
fn emit_spiric_loops(f: &SpiricFrame, inner: bool, out: &mut Vec<NurbsCurve>) {
    let n = 512;

    for sgn in [1.0, -1.0] {
        let mut pts = Vec::with_capacity(n);

        for k in 0..n {
            let phi = TWO_PI * k as f64 / n as f64;
            let t = f.r * phi.cos();
            let z = f.r * phi.sin();
            let (x, y) = match spiric_xy(f, inner, t) {
                Some(xy) => xy,
                None => return,
            };
            let yy = sgn * y;

            pts.push(Point::new(
                f.c1[0] + x * f.ex[0] + yy * f.ey[0] + z * f.w[0],
                f.c1[1] + x * f.ex[1] + yy * f.ey[1] + z * f.w[1],
                f.c1[2] + x * f.ex[2] + yy * f.ey[2] + z * f.w[2],
            ));
        }

        let mut closed_loop = NurbsCurve::create_interpolated(
            &pts,
            CurveNurbsKnotStyle::ChordPeriodic,
            CurveInterpStyle::Rhino,
        );

        if closed_loop.is_valid() {
            closed_loop.set_domain(0.0, 1.0);
            out.push(closed_loop);
        }
    }
}

/// Exact spiric loops of two equal parallel-axis tori.
fn ssi_torus_torus_spiric(ta: &RecogSurface, tb: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let c1 = ta.p1;
    let w = ssi_unit(&ta.p2);
    let c2 = tb.p1;
    let cxw = ssi_cross(&w, &ssi_unit(&tb.p2));

    if ssi_dot(&cxw, &cxw).sqrt() > ktol
        || (ta.r2 - tb.r2).abs() > ktol
        || (ta.r - tb.r).abs() > ktol
    {
        return false;
    }

    if axial_coord(&c1, &w, &c2).abs() > ktol {
        return false;
    }

    let dp = [c2[0] - c1[0], c2[1] - c1[1], c2[2] - c1[2]];
    let hax = ssi_dot(&dp, &w);
    let mut ex = [dp[0] - hax * w[0], dp[1] - hax * w[1], dp[2] - hax * w[2]];
    let d = ssi_dot(&ex, &ex).sqrt();

    if d <= ktol {
        return false;
    }

    ex = [ex[0] / d, ex[1] / d, ex[2] / d];
    let mut f = SpiricFrame {
        c1,
        ex,
        ey: ssi_cross(&w, &ex),
        w,
        rmaj: 0.5 * (ta.r + tb.r),
        r: 0.5 * (ta.r2 + tb.r2),
        c: 0.5 * d,
        be: 0.0,
    };

    if (f.rmaj - f.c).abs() <= ktol {
        return false;
    }

    let lo2 = (f.rmaj - f.r) * (f.rmaj - f.r) - f.c * f.c;
    let hi2 = (f.rmaj + f.r) * (f.rmaj + f.r) - f.c * f.c;

    if hi2 > ktol && lo2 <= ktol {
        return false;
    }

    if f.rmaj > f.c && f.r >= f.c - ktol {
        return false;
    }

    if lo2 > ktol {
        emit_spiric_loops(&f, false, out);
    }

    if f.rmaj > f.c + ktol {
        f.be = (f.rmaj * f.rmaj - f.c * f.c).sqrt();
        emit_spiric_loops(&f, true, out);
    }

    true
}

/// Coaxial torus-torus section: circles.
fn ssi_torus_torus(ta: &RecogSurface, tb: &RecogSurface, out: &mut Vec<NurbsCurve>) -> bool {
    let ktol = 1e-6;
    let c1 = ta.p1;
    let w = ssi_unit(&ta.p2);
    let rmaj1 = ta.r;
    let r1 = ta.r2;
    let c2 = tb.p1;
    let w2 = ssi_unit(&tb.p2);
    let rmaj2 = tb.r;
    let r2 = tb.r2;

    if r1 >= rmaj1 - ktol || r2 >= rmaj2 - ktol {
        return false;
    }

    if !axes_coaxial(&c1, &w, &c2, &w2, ktol) {
        return ssi_torus_torus_spiric(ta, tb, out);
    }

    let z2 = axial_coord(&c1, &w, &c2);
    let dx_r = rmaj2 - rmaj1;
    let d = (dx_r * dx_r + z2 * z2).sqrt();

    if d < ktol {
        return false;
    }

    if d - ktol > r1 + r2 || d + ktol < (r1 - r2).abs() {
        return true;
    }

    meridian_circles(&c1, &w, rmaj1, r1, dx_r, z2, r2, out);

    true
}

/// Exact sphere-sphere circle.
fn ssi_sphere_sphere(ra: &RecogSurface, rb: &RecogSurface, out: &mut Vec<NurbsCurve>) {
    let c1 = ra.p1;
    let r1 = ra.r;
    let c2 = rb.p1;
    let r2 = rb.r;
    let dv = [c2[0] - c1[0], c2[1] - c1[1], c2[2] - c1[2]];
    let dist = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();
    let tan_tol = (r1 + r2) * 1e-9;

    if dist <= 1e-12 || dist >= r1 + r2 - tan_tol || dist <= (r1 - r2).abs() + tan_tol {
        return;
    }

    let nu = [dv[0] / dist, dv[1] / dist, dv[2] / dist];
    let aa = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
    let rr2 = r1 * r1 - aa * aa;

    if rr2 > 0.0 {
        axis_circles(&c1, &nu, &[aa], rr2.sqrt(), out);
    }
}

/// Exact sections of a plane with a recognized surface; false when the case is not analytic.
fn plane_section_curves(
    plane: &RecogSurface,
    srf: &NurbsSurface,
    rs: &RecogSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    if rs.kind == RecogKind::Sphere {
        if let Some(c3) = ssi_plane_sphere(plane, rs) {
            out.push(c3);
        }

        return true;
    }

    if rs.kind == RecogKind::Cylinder {
        if !ssi_plane_cylinder_lines(plane, rs, srf, out) {
            if let Some(c3) = ssi_plane_cylinder(plane, rs) {
                out.push(c3);
            }
        }

        return true;
    }

    if rs.kind == RecogKind::Cone {
        return ssi_plane_cone(plane, rs, srf, out);
    }

    ssi_plane_torus(plane, rs, out)
}

/// Exact sections of two recognized curved surfaces; false when the case is not analytic.
fn quadric_section_curves(
    a: &NurbsSurface,
    ra: &RecogSurface,
    b: &NurbsSurface,
    rb: &RecogSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    let ka = ra.kind;
    let kb = rb.kind;

    if ka == RecogKind::Sphere && kb == RecogKind::Sphere {
        ssi_sphere_sphere(ra, rb, out);

        return true;
    }

    if ka == RecogKind::Cylinder && kb == RecogKind::Sphere {
        return ssi_cylinder_sphere(ra, rb, out);
    }

    if ka == RecogKind::Sphere && kb == RecogKind::Cylinder {
        return ssi_cylinder_sphere(rb, ra, out);
    }

    if ka == RecogKind::Cylinder && kb == RecogKind::Cone {
        return ssi_cylinder_cone(ra, rb, out);
    }

    if ka == RecogKind::Cone && kb == RecogKind::Cylinder {
        return ssi_cylinder_cone(rb, ra, out);
    }

    if ka == RecogKind::Cone && kb == RecogKind::Sphere {
        return ssi_cone_sphere(ra, rb, out);
    }

    if ka == RecogKind::Sphere && kb == RecogKind::Cone {
        return ssi_cone_sphere(rb, ra, out);
    }

    if ka == RecogKind::Cylinder && kb == RecogKind::Cylinder {
        return ssi_cylinder_cylinder(a, ra, b, rb, out);
    }

    if ka == RecogKind::Cylinder && kb == RecogKind::Torus {
        return ssi_cylinder_torus(ra, rb, out);
    }

    if ka == RecogKind::Torus && kb == RecogKind::Cylinder {
        return ssi_cylinder_torus(rb, ra, out);
    }

    if ka == RecogKind::Cone && kb == RecogKind::Torus {
        return ssi_cone_torus(ra, rb, out);
    }

    if ka == RecogKind::Torus && kb == RecogKind::Cone {
        return ssi_cone_torus(rb, ra, out);
    }

    if ka == RecogKind::Sphere && kb == RecogKind::Torus {
        return ssi_sphere_torus(ra, rb, out);
    }

    if ka == RecogKind::Torus && kb == RecogKind::Sphere {
        return ssi_sphere_torus(rb, ra, out);
    }

    if ka == RecogKind::Torus && kb == RecogKind::Torus {
        return ssi_torus_torus(ra, rb, out);
    }

    false
}

/// Exact 3D sections of two recognized surfaces; false when the pair is not analytic.
fn analytic_curves(
    a: &NurbsSurface,
    ra: &RecogSurface,
    b: &NurbsSurface,
    rb: &RecogSurface,
    out: &mut Vec<NurbsCurve>,
) -> bool {
    if ra.kind == RecogKind::Plane && rb.kind == RecogKind::Plane {
        let mut empty = false;

        if ssi_plane_plane(a, ra, b, rb, out, &mut empty) {
            return true;
        }

        return empty;
    }

    if ra.kind == RecogKind::Plane {
        return plane_section_curves(ra, b, rb, out);
    }

    if rb.kind == RecogKind::Plane {
        return plane_section_curves(rb, a, ra, out);
    }

    quadric_section_curves(a, ra, b, rb, out)
}

/// Pull-back runs of an exact section on one recognized surface, empty when the analytic pcurve applies.
fn analytic_side_runs(
    srf: &NurbsSurface,
    recog: &RecogSurface,
    c3: &NurbsCurve,
) -> Vec<PullbackRun> {
    if analytic_pcurve(srf, recog, c3).is_valid() {
        return vec![];
    }

    analytic_pullback(srf, recog, c3)
}

/// One recognized surface of an exact section with the pull-back runs of the section curve.
struct SectionSide<'a> {
    srf: &'a NurbsSurface,   // Recognized surface.
    recog: &'a RecogSurface, // Its recognized kind and parameters.
    runs: Vec<PullbackRun>,  // Pull-back runs, empty when the analytic pcurve applies.
}

/// Pcurve of the piece over [lo, hi] of a section curve with this domain: its pull-back runs, else analytic, then projected.
fn analytic_side_pcurve(
    side: &SectionSide<'_>,
    piece: &NurbsCurve,
    domain: (f64, f64),
    lo: f64,
    hi: f64,
) -> NurbsCurve {
    if !side.runs.is_empty() {
        return run_span(&side.runs, domain, lo, hi);
    }

    let mut pc = analytic_pcurve(side.srf, side.recog, piece);

    if !pc.is_valid() {
        let v = Closest::surface_curve(side.srf, piece, 0.0, 0.0, 0.0);

        if !v.is_empty() {
            pc = v[0].clone();
        }
    }

    pc
}

/// Parameters of c3 that cut it at every seam crossing of both sides' runs, the domain ends included.
fn seam_cuts(c3: &NurbsCurve, runs_a: &[PullbackRun], runs_b: &[PullbackRun]) -> Vec<f64> {
    let domain = c3.domain();
    let eps = (domain.1 - domain.0) * 1e-9;
    let mut ts = vec![domain.0, domain.1];

    for runs in [runs_a, runs_b] {
        for run in runs.iter().skip(1) {
            ts.push(run.ts[0]);
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut cuts = vec![ts[0]];

    for &t in ts.iter().skip(1) {
        if t - cuts[cuts.len() - 1] > eps {
            cuts.push(t);
        }
    }

    let last = cuts.len() - 1;
    cuts[last] = domain.1;

    cuts
}

/// Whether the runs end where they start, so the end piece of a closed curve continues onto its first.
fn runs_close(runs: &[PullbackRun]) -> bool {
    if runs.is_empty() {
        return true;
    }

    let last = &runs[runs.len() - 1];

    runs[0].uv[0].distance(&last.uv[last.uv.len() - 1], None) < 1e-9
}

/// Push the piece of c3 over [lo, hi] with both pcurves; hi past the domain end wraps the closed curve onto its start.
fn push_section_piece(
    sa: &SectionSide<'_>,
    sb: &SectionSide<'_>,
    c3: &NurbsCurve,
    lo: f64,
    hi: f64,
    triples: &mut Vec<(NurbsCurve, NurbsCurve, NurbsCurve)>,
) {
    let domain = c3.domain();
    let mut piece = c3.clone();

    if hi > domain.1 && !piece.change_closed_curve_seam(lo) {
        return;
    }

    if !piece.trim(lo, hi) {
        return;
    }

    let pa = analytic_side_pcurve(sa, &piece, domain, lo, hi);
    let pb = analytic_side_pcurve(sb, &piece, domain, lo, hi);

    if pa.is_valid() && pb.is_valid() {
        triples.push((piece, pa, pb));
    }
}

/// Exact section of two recognized analytic surfaces, empty when no case applies.
fn analytic_ssi(a: &NurbsSurface, b: &NurbsSurface, tolerance: f64) -> AnalyticResult {
    let mut res = AnalyticResult {
        status: AnalyticStatus::NotAnalytic,
        triples: Vec::new(),
    };
    let rtol = f64::max(tolerance, 1e-7) * 1e4;
    let ra = recognize_surface(a, rtol);
    let rb = recognize_surface(b, rtol);

    if ra.kind == RecogKind::None || rb.kind == RecogKind::None {
        return res;
    }

    let mut c3_list = Vec::new();

    if !analytic_curves(a, &ra, b, &rb, &mut c3_list) {
        return res;
    }

    for cc3 in c3_list {
        let sa = SectionSide {
            srf: a,
            recog: &ra,
            runs: analytic_side_runs(a, &ra, &cc3),
        };
        let sb = SectionSide {
            srf: b,
            recog: &rb,
            runs: analytic_side_runs(b, &rb, &cc3),
        };
        let cuts = seam_cuts(&cc3, &sa.runs, &sb.runs);
        let wrap =
            cuts.len() > 2 && cc3.is_closed() && runs_close(&sa.runs) && runs_close(&sb.runs);
        let first = if wrap { 1 } else { 0 };
        let last = cuts.len() - if wrap { 2 } else { 1 };

        for k in first..last {
            push_section_piece(&sa, &sb, &cc3, cuts[k], cuts[k + 1], &mut res.triples);
        }

        if wrap {
            push_section_piece(
                &sa,
                &sb,
                &cc3,
                cuts[last],
                cuts[first] + cuts[cuts.len() - 1] - cuts[0],
                &mut res.triples,
            );
        }
    }

    res.status = AnalyticStatus::Hit;

    res
}

// ═══════════════════════════════════════════════════════════════════════════
// NURBS surfaces
// ═══════════════════════════════════════════════════════════════════════════
/// Whether crv runs through a curve of curves at a quarter, half and three quarters of its domain within tolerance.
fn is_duplicate_curve(crv: &NurbsCurve, curves: &[NurbsCurve], tolerance: f64) -> bool {
    let (ct0, ct1) = crv.domain();

    for existing in curves {
        let (et0, et1) = existing.domain();
        let mut all_close = true;

        for &f in &[0.25, 0.5, 0.75] {
            let cp = crv.point_at(ct0 + (ct1 - ct0) * f);
            let ep = existing.point_at(et0 + (et1 - et0) * f);
            let em = existing.point_at((et0 + et1) * 0.5);
            let d = cp.distance(&ep, None).min(cp.distance(&em, None));

            if d > tolerance {
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
        let mut all_pts: Vec<Point> = Vec::with_capacity(uv_trace.len());

        for &(u, v) in uv_trace {
            all_pts.push(surface.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0)));
        }

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

        let dup_tol = step * uv_to_3d * 3.0;

        if !is_duplicate_curve(&crv, &result, dup_tol) {
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

/// Drop near-zero-length section curves.
fn drop_point_sections(
    trs: Vec<(NurbsCurve, NurbsCurve, NurbsCurve)>,
    tolerance: f64,
) -> Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    let min_len = f64::max(tolerance * 10.0, 1e-9);
    let mut kept = Vec::new();

    for t in trs {
        if t.0.length(None) >= min_len {
            kept.push(t);
        }
    }

    kept
}

/// Plane through the middle of the surface domain.
fn surface_mid_plane(srf: &NurbsSurface) -> Plane {
    let (po, nn) = surface_mid_frame(srf);

    Plane::from_point_normal(po, Vector::new(nn[0], nn[1], nn[2]), None)
}

/// Plane-surface section triples, the plane's pcurve found by projection.
fn planar_section_triples(
    planar: &NurbsSurface,
    other: &NurbsSurface,
    planar_first: bool,
    tolerance: f64,
) -> Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    let mut result = Vec::new();

    for (c3, pc) in surface_plane_uv(other, &surface_mid_plane(planar), Some(tolerance)) {
        let pps = Closest::surface_curve(planar, &c3, 0.0, 0.0, 0.0);

        if pps.len() != 1 {
            continue;
        }

        if planar_first {
            result.push((c3, pps[0].clone(), pc));
        } else {
            result.push((c3, pc, pps[0].clone()));
        }
    }

    drop_point_sections(result, tolerance)
}

/// Bounding box of one grid cell from its 3 x 3 samples, inflated by twice its sag, and the cell center.
#[allow(clippy::too_many_arguments)]
fn cell_box(
    samples: &[Vec<Point>],
    ci: usize,
    cj: usize,
    c0u: f64,
    dcu: f64,
    c0v: f64,
    dcv: f64,
    tolerance: f64,
) -> [f64; 8] {
    let mut minx = f64::INFINITY;
    let mut miny = minx;
    let mut minz = minx;
    let mut maxx = -minx;
    let mut maxy = -minx;
    let mut maxz = -minx;

    for i in 2 * ci..2 * ci + 3 {
        for j in 2 * cj..2 * cj + 3 {
            let p = &samples[i][j];
            minx = f64::min(minx, p[0]);
            maxx = f64::max(maxx, p[0]);
            miny = f64::min(miny, p[1]);
            maxy = f64::max(maxy, p[1]);
            minz = f64::min(minz, p[2]);
            maxz = f64::max(maxz, p[2]);
        }
    }

    let ctr = &samples[2 * ci + 1][2 * cj + 1];
    let p00 = &samples[2 * ci][2 * cj];
    let p10 = &samples[2 * ci + 2][2 * cj];
    let p01 = &samples[2 * ci][2 * cj + 2];
    let p11 = &samples[2 * ci + 2][2 * cj + 2];
    let cx = (p00[0] + p10[0] + p01[0] + p11[0]) * 0.25;
    let cy = (p00[1] + p10[1] + p01[1] + p11[1]) * 0.25;
    let cz = (p00[2] + p10[2] + p01[2] + p11[2]) * 0.25;
    let sag = ((ctr[0] - cx) * (ctr[0] - cx)
        + (ctr[1] - cy) * (ctr[1] - cy)
        + (ctr[2] - cz) * (ctr[2] - cz))
        .sqrt();
    let inf = 2.0 * sag + tolerance;

    [
        minx - inf,
        miny - inf,
        minz - inf,
        maxx + inf,
        maxy + inf,
        maxz + inf,
        c0u + dcu * (ci as f64 + 0.5),
        c0v + dcv * (cj as f64 + 0.5),
    ]
}

/// Inflated bounding boxes and centers of an ncu x ncv grid of surface cells.
#[allow(clippy::too_many_arguments)]
fn surface_cell_boxes(
    srf: &NurbsSurface,
    c0u: f64,
    dcu: f64,
    ncu: usize,
    c0v: f64,
    dcv: f64,
    ncv: usize,
    tolerance: f64,
) -> Vec<[f64; 8]> {
    let mut samples = Vec::new();

    for i in 0..2 * ncu + 1 {
        let mut row = Vec::new();

        for j in 0..2 * ncv + 1 {
            row.push(srf_point(
                srf,
                c0u + dcu * 0.5 * i as f64,
                c0v + dcv * 0.5 * j as f64,
            ));
        }

        samples.push(row);
    }

    let mut boxes = Vec::new();

    for ci in 0..ncu {
        for cj in 0..ncv {
            boxes.push(cell_box(&samples, ci, cj, c0u, dcu, c0v, dcv, tolerance));
        }
    }

    boxes
}

/// Smallest non-degenerate diagonal among the first 64 boxes, 1 when none.
fn cell_diagonal(boxes: &[[f64; 8]]) -> f64 {
    let mut best = f64::INFINITY;

    for bx in boxes.iter().take(64) {
        let d = ((bx[3] - bx[0]) * (bx[3] - bx[0])
            + (bx[4] - bx[1]) * (bx[4] - bx[1])
            + (bx[5] - bx[2]) * (bx[5] - bx[2]))
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
}

/// Grid seed of a surface-surface trace in joint parameters.
struct SurfaceSurfaceSeed {
    u: f64,     // Seed u on a.
    v: f64,     // Seed v on a.
    s: f64,     // Seed u on b.
    t: f64,     // Seed v on b.
    used: bool, // Whether a trace already passed the seed.
}

/// Joint parameter space (au, av, bu, bv) of two surfaces with the marching scales.
struct SurfaceSurfaceField<'a> {
    a: &'a NurbsSurface,    // First surface.
    b: &'a NurbsSurface,    // Second surface.
    tolerance: f64,         // Section tolerance.
    lo: [f64; 4],           // Domain starts.
    hi: [f64; 4],           // Domain ends.
    range: [f64; 4],        // Domain lengths.
    closed: [bool; 4],      // Whether each parameter wraps around a seam.
    step: [f64; 4],         // Grid cell size per parameter.
    boxes_a: Vec<[f64; 8]>, // Cell boxes of a.
    boxes_b: Vec<[f64; 8]>, // Cell boxes of b.
    h_init: f64,            // Initial 3D marching step.
    conv_tol: f64,          // Corrector convergence tolerance.
    seed_tol: f64,          // 3D distance that merges two seeds.
    max_steps: usize,       // Marching step cap per direction.
    close_tol: f64,         // 3D distance that closes a loop.
    consume_tol: f64,       // 3D distance that consumes a seed.
}

impl<'a> SurfaceSurfaceField<'a> {
    /// Sample both domains into cell boxes and derive the marching scales.
    fn new(a: &'a NurbsSurface, b: &'a NurbsSurface, tolerance: f64) -> Self {
        let mut lo = [0.0f64; 4];
        let mut hi = [0.0f64; 4];
        let mut range = [0.0f64; 4];
        let mut closed = [false; 4];
        let mut cells = [0usize; 4];
        let mut step = [0.0f64; 4];
        let srfs = [a, b];

        for k in 0..4 {
            let srf = srfs[k / 2];
            let (d0, d1) = srf_domain(srf, k % 2);
            lo[k] = d0;
            hi[k] = d1;
            range[k] = hi[k] - lo[k];
            closed[k] = srf.is_closed(k % 2);
            cells[k] = usize::max(srf.get_span_vector(k % 2).len().saturating_sub(1), 1) * 4;
            step[k] = range[k] / cells[k] as f64;
        }

        let boxes_a = surface_cell_boxes(
            a, lo[0], step[0], cells[0], lo[1], step[1], cells[1], tolerance,
        );
        let boxes_b = surface_cell_boxes(
            b, lo[2], step[2], cells[2], lo[3], step[3], cells[3], tolerance,
        );
        let h_init = f64::min(cell_diagonal(&boxes_a), cell_diagonal(&boxes_b)) * 0.25;
        let seed_tol = f64::max(cell_diagonal(&boxes_a), cell_diagonal(&boxes_b));

        SurfaceSurfaceField {
            a,
            b,
            tolerance,
            lo,
            hi,
            range,
            closed,
            step,
            boxes_a,
            boxes_b,
            h_init,
            conv_tol: f64::max(tolerance, h_init * 1e-7),
            seed_tol,
            max_steps: (cells[0] * cells[1] + cells[2] * cells[3]) * 32,
            close_tol: h_init * 3.0,
            consume_tol: h_init * 2.0,
        }
    }

    /// Wrap parameter k across a closed seam or clamp it to the domain.
    fn wrap(&self, k: usize, t: f64) -> f64 {
        if self.closed[k] {
            let mut f = (t - self.lo[k]) % self.range[k];

            if f < 0.0 {
                f += self.range[k];
            }

            return self.lo[k] + f;
        }

        self.lo[k].max(t.min(self.hi[k]))
    }

    /// Point and first derivatives (s, su, sv) of a at (u, v).
    fn eval_a(&self, u: f64, v: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let d = self.a.evaluate(self.wrap(0, u), self.wrap(1, v), 1);

        (
            [d[0][0], d[0][1], d[0][2]],
            [d[2][0], d[2][1], d[2][2]],
            [d[1][0], d[1][1], d[1][2]],
        )
    }

    /// Point and first derivatives (s, su, sv) of b at (u, v).
    fn eval_b(&self, u: f64, v: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let d = self.b.evaluate(self.wrap(2, u), self.wrap(3, v), 1);

        (
            [d[0][0], d[0][1], d[0][2]],
            [d[2][0], d[2][1], d[2][2]],
            [d[1][0], d[1][1], d[1][2]],
        )
    }

    /// Point of a at the joint parameters q.
    fn point(&self, q: &[f64; 4]) -> [f64; 3] {
        self.eval_a(q[0], q[1]).0
    }

    /// Clamp the open parameters of x to their domains.
    fn clamp_open(&self, x: &mut [f64; 4]) {
        for k in 0..4 {
            if !self.closed[k] {
                x[k] = self.lo[k].max(x[k].min(self.hi[k]));
            }
        }
    }

    /// Newton-project x onto the section, optionally pinned to the plane through pp normal to pd.
    fn correct(&self, x: &mut [f64; 4], has_pin: bool, pd: &[f64; 3], pp: &[f64; 3]) -> bool {
        for _ in 0..8 {
            let (sa, sau, sav) = self.eval_a(x[0], x[1]);
            let (sb, sbu, sbv) = self.eval_b(x[2], x[3]);
            let res = [sa[0] - sb[0], sa[1] - sb[1], sa[2] - sb[2]];

            if (res[0] * res[0] + res[1] * res[1] + res[2] * res[2]).sqrt() < self.conv_tol {
                return true;
            }

            let mut jac = [[0.0f64; 4]; 3];

            for k in 0..3 {
                jac[k][0] = sau[k];
                jac[k][1] = sav[k];
                jac[k][2] = -sbu[k];
                jac[k][3] = -sbv[k];
            }

            let ok = if has_pin {
                newton_step_pinned(&jac, &res, &sa, &sau, &sav, pd, pp, x)
            } else {
                newton_step_free(&jac, &res, x)
            };

            if !ok {
                return false;
            }

            self.clamp_open(x);
        }

        let sa = self.eval_a(x[0], x[1]).0;
        let sb = self.eval_b(x[2], x[3]).0;
        let g = ((sa[0] - sb[0]) * (sa[0] - sb[0])
            + (sa[1] - sb[1]) * (sa[1] - sb[1])
            + (sa[2] - sb[2]) * (sa[2] - sb[2]))
            .sqrt();

        g < self.conv_tol * 10.0
    }

    /// Newton-project x onto the section with parameter k held fixed; x is kept when it fails.
    fn correct_on_seam(&self, x: &mut [f64; 4], k: usize) -> bool {
        let mut y = *x;
        let mut free = [0usize; 3];
        let mut j = 0;

        for c in 0..4 {
            if c != k {
                free[j] = c;
                j += 1;
            }
        }

        for _ in 0..8 {
            let (sa, sau, sav) = self.eval_a(y[0], y[1]);
            let (sb, sbu, sbv) = self.eval_b(y[2], y[3]);
            let res = [sa[0] - sb[0], sa[1] - sb[1], sa[2] - sb[2]];

            if (res[0] * res[0] + res[1] * res[1] + res[2] * res[2]).sqrt() < self.conv_tol {
                *x = y;

                return true;
            }

            let cols = [
                sau,
                sav,
                [-sbu[0], -sbu[1], -sbu[2]],
                [-sbv[0], -sbv[1], -sbv[2]],
            ];
            let mut jac = vec![vec![0.0; 3]; 3];

            for r in 0..3 {
                for c in 0..3 {
                    jac[r][c] = cols[free[c]][r];
                }
            }

            let dx = match solve_gauss(&jac, &res, 3) {
                Some(dx) => dx,
                None => return false,
            };

            for c in 0..3 {
                y[free[c]] -= dx[c];
            }

            self.clamp_open(&mut y);
        }

        let sa = self.eval_a(y[0], y[1]).0;
        let sb = self.eval_b(y[2], y[3]).0;
        let g = ((sa[0] - sb[0]) * (sa[0] - sb[0])
            + (sa[1] - sb[1]) * (sa[1] - sb[1])
            + (sa[2] - sb[2]) * (sa[2] - sb[2]))
            .sqrt();

        if g >= self.conv_tol * 10.0 {
            return false;
        }

        *x = y;

        true
    }

    /// Unit 3D section tangent at x in direction dir_sign, None at a tangency, and both surfaces' derivatives.
    #[allow(clippy::type_complexity)]
    fn tangent(
        &self,
        x: &[f64; 4],
        dir_sign: f64,
    ) -> (
        Option<[f64; 3]>,
        [f64; 3],
        [f64; 3],
        [f64; 3],
        [f64; 3],
        [f64; 3],
    ) {
        let (sa, sau, sav) = self.eval_a(x[0], x[1]);
        let (_, sbu, sbv) = self.eval_b(x[2], x[3]);
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
            return (None, sa, sau, sav, sbu, sbv);
        }

        let dir = [
            d[0] / dl * dir_sign,
            d[1] / dl * dir_sign,
            d[2] / dl * dir_sign,
        ];

        (Some(dir), sa, sau, sav, sbu, sbv)
    }
}

/// Minimum-norm Newton step x -= jac^T (jac jac^T)^-1 res.
fn newton_step_free(jac: &[[f64; 4]; 3], res: &[f64; 3], x: &mut [f64; 4]) -> bool {
    let mut jjt = vec![vec![0.0f64; 3]; 3];

    for r in 0..3 {
        for q in 0..3 {
            let mut s = 0.0;

            for c in 0..4 {
                s += jac[r][c] * jac[q][c];
            }

            jjt[r][q] = s;
        }
    }

    let y = match solve_gauss(&jjt, &[res[0], res[1], res[2]], 3) {
        Some(y) => y,
        None => return false,
    };

    for c in 0..4 {
        let mut s = 0.0;

        for r in 0..3 {
            s += jac[r][c] * y[r];
        }

        x[c] -= s;
    }

    true
}

/// Newton step with a fourth row pinning a's point to the plane through pp normal to pd.
#[allow(clippy::too_many_arguments)]
fn newton_step_pinned(
    jac: &[[f64; 4]; 3],
    res: &[f64; 3],
    sa: &[f64; 3],
    sau: &[f64; 3],
    sav: &[f64; 3],
    pd: &[f64; 3],
    pp: &[f64; 3],
    x: &mut [f64; 4],
) -> bool {
    let m = vec![
        vec![jac[0][0], jac[0][1], jac[0][2], jac[0][3]],
        vec![jac[1][0], jac[1][1], jac[1][2], jac[1][3]],
        vec![jac[2][0], jac[2][1], jac[2][2], jac[2][3]],
        vec![
            pd[0] * sau[0] + pd[1] * sau[1] + pd[2] * sau[2],
            pd[0] * sav[0] + pd[1] * sav[1] + pd[2] * sav[2],
            0.0,
            0.0,
        ],
    ];
    let rhs = [
        res[0],
        res[1],
        res[2],
        pd[0] * (sa[0] - pp[0]) + pd[1] * (sa[1] - pp[1]) + pd[2] * (sa[2] - pp[2]),
    ];
    let dx = match solve_gauss(&m, &rhs, 4) {
        Some(dx) => dx,
        None => return false,
    };

    for c in 0..4 {
        x[c] -= dx[c];
    }

    true
}

/// Distance between two 3D triples.
fn triple_distance(p: &[f64; 3], q: &[f64; 3]) -> f64 {
    ((p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]) + (p[2] - q[2]) * (p[2] - q[2]))
        .sqrt()
}

/// Whether a's point at x lies within the seed tolerance of an existing seed.
fn seed_is_duplicate(
    field: &SurfaceSurfaceField<'_>,
    x: &[f64; 4],
    seeds: &[SurfaceSurfaceSeed],
) -> bool {
    let p = field.point(x);

    for sd in seeds {
        if triple_distance(&p, &field.point(&[sd.u, sd.v, 0.0, 0.0])) < field.seed_tol {
            return true;
        }
    }

    false
}

/// Corrected centers of overlapping cell-box pairs, one per distinct 3D point, at most 20000 pairs.
fn surface_surface_seeds(field: &SurfaceSurfaceField<'_>) -> Vec<SurfaceSurfaceSeed> {
    let mut seeds = Vec::new();
    let mut pair_budget: i32 = 20000;
    let dummy3 = [0.0f64; 3];

    for ba in &field.boxes_a {
        if pair_budget < 0 {
            break;
        }

        for bb in &field.boxes_b {
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
                break;
            }

            let mut x = [ba[6], ba[7], bb[6], bb[7]];

            if !field.correct(&mut x, false, &dummy3, &dummy3)
                || seed_is_duplicate(field, &x, &seeds)
            {
                continue;
            }

            seeds.push(SurfaceSurfaceSeed {
                u: field.wrap(0, x[0]),
                v: field.wrap(1, x[1]),
                s: field.wrap(2, x[2]),
                t: field.wrap(3, x[3]),
                used: false,
            });
        }
    }

    seeds
}

/// Marching state of one trace direction.
struct SurfaceSurfaceMarch {
    x: [f64; 4],        // Current joint parameters.
    d: [f64; 3],        // Current 3D direction.
    sa: [f64; 3],       // Point of a at x.
    sau: [f64; 3],      // u-derivative of a at x.
    sav: [f64; 3],      // v-derivative of a at x.
    sbu: [f64; 3],      // u-derivative of b at x.
    sbv: [f64; 3],      // v-derivative of b at x.
    have_prev_d: bool,  // Whether a previous step exists.
    prev_d: [f64; 3],   // Previous 3D direction.
    p_prev: [f64; 3],   // Previous 3D point.
    h: f64,             // Current 3D step.
    smooth: i32,        // Accepted steps since the last change of h.
    tang_reuse: i32,    // Steps that reused the previous direction.
    why: &'static str,  // Reason the march stopped.
    xn: [f64; 4],       // Accepted next parameters.
    p_cur: [f64; 3],    // Accepted next 3D point.
    step_len: f64,      // Accepted 3D step length.
    hit_boundary: bool, // Whether the accepted step reached an open boundary.
}

/// Direction of the next step, reusing the previous one up to three times at tangencies.
fn march_direction(
    field: &SurfaceSurfaceField<'_>,
    m: &mut SurfaceSurfaceMarch,
    dir_sign: f64,
) -> bool {
    let (d, sa, sau, sav, sbu, sbv) = field.tangent(&m.x, dir_sign);
    m.sa = sa;
    m.sau = sau;
    m.sav = sav;
    m.sbu = sbu;
    m.sbv = sbv;

    if let Some(d) = d {
        m.d = d;
        m.tang_reuse = 0;

        return true;
    }

    if !m.have_prev_d || m.tang_reuse >= 3 {
        m.why = "tangency";

        return false;
    }

    m.d = m.prev_d;
    m.tang_reuse += 1;

    true
}

/// Parameters after a step h along d, cut back at open boundaries, and the predicted point; None when a surface is singular.
fn march_predict(field: &SurfaceSurfaceField<'_>, m: &mut SurfaceSurfaceMarch) -> Option<[f64; 3]> {
    let sau = &m.sau;
    let sav = &m.sav;
    let sbu = &m.sbu;
    let sbv = &m.sbv;
    let d = &m.d;
    let ma = vec![
        vec![
            sau[0] * sau[0] + sau[1] * sau[1] + sau[2] * sau[2],
            sau[0] * sav[0] + sau[1] * sav[1] + sau[2] * sav[2],
        ],
        vec![
            sau[0] * sav[0] + sau[1] * sav[1] + sau[2] * sav[2],
            sav[0] * sav[0] + sav[1] * sav[1] + sav[2] * sav[2],
        ],
    ];
    let ra = [
        m.h * (d[0] * sau[0] + d[1] * sau[1] + d[2] * sau[2]),
        m.h * (d[0] * sav[0] + d[1] * sav[1] + d[2] * sav[2]),
    ];
    let mb = vec![
        vec![
            sbu[0] * sbu[0] + sbu[1] * sbu[1] + sbu[2] * sbu[2],
            sbu[0] * sbv[0] + sbu[1] * sbv[1] + sbu[2] * sbv[2],
        ],
        vec![
            sbu[0] * sbv[0] + sbu[1] * sbv[1] + sbu[2] * sbv[2],
            sbv[0] * sbv[0] + sbv[1] * sbv[1] + sbv[2] * sbv[2],
        ],
    ];
    let rb = [
        m.h * (d[0] * sbu[0] + d[1] * sbu[1] + d[2] * sbu[2]),
        m.h * (d[0] * sbv[0] + d[1] * sbv[1] + d[2] * sbv[2]),
    ];
    let duv_a = solve_gauss(&ma, &ra, 2)?;
    let duv_b = solve_gauss(&mb, &rb, 2)?;
    let delta = [duv_a[0], duv_a[1], duv_b[0], duv_b[1]];
    let mut tc = 1.0f64;
    m.hit_boundary = false;

    for k in 0..4 {
        if field.closed[k] || delta[k].abs() < 1e-15 {
            continue;
        }

        if m.x[k] + delta[k] > field.hi[k] {
            tc = f64::min(tc, (field.hi[k] - m.x[k]) / delta[k]);
            m.hit_boundary = true;
        }

        if m.x[k] + delta[k] < field.lo[k] {
            tc = f64::min(tc, (field.lo[k] - m.x[k]) / delta[k]);
            m.hit_boundary = true;
        }
    }

    for k in 0..4 {
        m.xn[k] = m.x[k] + tc * delta[k];
    }

    Some([
        m.sa[0] + m.d[0] * m.h * tc,
        m.sa[1] + m.d[1] * m.h * tc,
        m.sa[2] + m.d[2] * m.h * tc,
    ])
}

/// Whether the step from p_prev to p_cur turns more than acos(0.985) from the previous direction.
fn march_turns_sharply(m: &SurfaceSurfaceMarch) -> bool {
    let sd0 = (m.p_cur[0] - m.p_prev[0]) / m.step_len;
    let sd1 = (m.p_cur[1] - m.p_prev[1]) / m.step_len;
    let sd2 = (m.p_cur[2] - m.p_prev[2]) / m.step_len;

    sd0 * m.prev_d[0] + sd1 * m.prev_d[1] + sd2 * m.prev_d[2] < 0.985
}

/// Up to seven attempts at one step, halving h after a failed corrector or a sharp turn.
fn march_step(field: &SurfaceSurfaceField<'_>, m: &mut SurfaceSurfaceMarch) -> bool {
    let mut attempts = 0;

    while attempts < 7 {
        let p_pred = match march_predict(field, m) {
            Some(p_pred) => p_pred,
            None => {
                m.why = "singular";

                return false;
            }
        };
        let d = m.d;

        if !field.correct(&mut m.xn, true, &d, &p_pred) {
            m.why = "corrector";
            m.h *= 0.5;
            attempts += 1;
            m.smooth = 0;
            continue;
        }

        m.p_cur = field.point(&m.xn);
        m.step_len = triple_distance(&m.p_cur, &m.p_prev);

        if m.have_prev_d
            && m.step_len > 1e-14
            && march_turns_sharply(m)
            && attempts < 6
            && !m.hit_boundary
        {
            m.why = "angle";
            m.h *= 0.5;
            attempts += 1;
            m.smooth = 0;
            continue;
        }

        return true;
    }

    false
}

/// Mark the unused seeds within the consume tolerance of p as used.
fn consume_seeds_near(
    field: &SurfaceSurfaceField<'_>,
    p: &[f64; 3],
    seeds: &mut [SurfaceSurfaceSeed],
) {
    for sd in seeds.iter_mut() {
        if !sd.used && triple_distance(p, &field.point(&[sd.u, sd.v, 0.0, 0.0])) < field.consume_tol
        {
            sd.used = true;
        }
    }
}

/// Marching state at x0 with the initial step and no previous direction.
fn start_march(
    field: &SurfaceSurfaceField<'_>,
    x0: &[f64; 4],
    p_start: &[f64; 3],
) -> SurfaceSurfaceMarch {
    SurfaceSurfaceMarch {
        x: *x0,
        d: [0.0; 3],
        sa: [0.0; 3],
        sau: [0.0; 3],
        sav: [0.0; 3],
        sbu: [0.0; 3],
        sbv: [0.0; 3],
        have_prev_d: false,
        prev_d: [0.0; 3],
        p_prev: *p_start,
        h: field.h_init,
        smooth: 0,
        tang_reuse: 0,
        why: "maxsteps",
        xn: [0.0; 4],
        p_cur: [0.0; 3],
        step_len: 0.0,
        hit_boundary: false,
    }
}

/// March from x0 in direction dir_sign until it closes, leaves the domain, stalls or hits the step cap: (closed, samples, why).
fn trace_dir(
    field: &SurfaceSurfaceField<'_>,
    x0: &[f64; 4],
    dir_sign: f64,
    seeds: &mut [SurfaceSurfaceSeed],
) -> (bool, Vec<[f64; 4]>, &'static str) {
    let mut out = Vec::new();
    let p_start = field.point(x0);
    let mut m = start_march(field, x0, &p_start);
    let mut dist_traveled = 0.0;

    for _ in 0..field.max_steps {
        if !march_direction(field, &mut m, dir_sign) || !march_step(field, &mut m) {
            break;
        }

        m.why = "maxsteps";
        m.prev_d = m.d;
        m.have_prev_d = true;
        m.smooth += 1;

        if m.smooth >= 5 && m.h < field.h_init * 2.0 {
            m.h *= 1.4;
            m.smooth = 0;
        }

        m.x = m.xn;
        dist_traveled += m.step_len;
        out.push(m.x);

        if dist_traveled > field.close_tol * 3.0
            && triple_distance(&m.p_cur, &p_start) < field.close_tol
        {
            return (true, out, "closed");
        }

        m.p_prev = m.p_cur;

        if m.hit_boundary {
            m.why = "boundary";
            break;
        }

        consume_seeds_near(field, &m.p_cur, seeds);
    }

    (false, out, m.why)
}

/// Shift closed parameters by whole periods so consecutive samples never jump more than half a period.
fn unwrap_quad(field: &SurfaceSurfaceField<'_>, quad: &mut [[f64; 4]]) {
    for i in 1..quad.len() {
        for k in 0..4 {
            if !field.closed[k] {
                continue;
            }

            let jump = quad[i][k] - quad[i - 1][k];

            if jump > field.range[k] * 0.5 {
                quad[i][k] -= field.range[k];
            } else if jump < -field.range[k] * 0.5 {
                quad[i][k] += field.range[k];
            }
        }
    }
}

/// Trace both directions from a seed into one unwrapped run: (quad, is_loop), None when it is too short.
fn trace_seed(
    field: &SurfaceSurfaceField<'_>,
    x0: &[f64; 4],
    seeds: &mut [SurfaceSurfaceSeed],
) -> Option<(Vec<[f64; 4]>, bool)> {
    let (fwd_closed, fwd, fwd_why) = trace_dir(field, x0, 1.0, seeds);
    let mut bwd = Vec::new();
    let mut bwd_why = "?";

    if !fwd_closed {
        let traced = trace_dir(field, x0, -1.0, seeds);
        bwd = traced.1;
        bwd_why = traced.2;
    }

    let mut quad = Vec::new();

    for i in (0..bwd.len()).rev() {
        quad.push(bwd[i]);
    }

    quad.push(*x0);

    for p in &fwd {
        quad.push(*p);
    }

    let min_pts = if !fwd_closed && fwd_why == "boundary" && bwd_why == "boundary" {
        2
    } else {
        4
    };

    if quad.len() < min_pts {
        return None;
    }

    unwrap_quad(field, &mut quad);
    let gap = triple_distance(&field.point(&quad[0]), &field.point(&quad[quad.len() - 1]));
    let is_loop = fwd_closed || (quad.len() >= 6 && gap < field.close_tol);

    if is_loop {
        quad.pop();
    }

    if quad.len() < min_pts {
        return None;
    }

    Some((quad, is_loop))
}

/// Whether the quarter, half and three-quarter samples all lie within dup_tol of one kept run.
fn is_duplicate_quad(trace_pts3: &[[f64; 3]], kept_pts3: &[Vec<[f64; 3]>], dup_tol: f64) -> bool {
    let m = trace_pts3.len();

    for other in kept_pts3 {
        let mut all_close = true;

        for f in [0.25, 0.5, 0.75] {
            let cp = &trace_pts3[((m - 1) as f64 * f) as usize];
            let mut dmin = dup_tol + 1.0;

            for op in other {
                dmin = f64::min(dmin, triple_distance(cp, op));
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

/// Insert corrected midpoints into gaps longer than 1.5 median gaps, at most four passes.
fn densify_quad(field: &SurfaceSurfaceField<'_>, quad: &mut Vec<[f64; 4]>) {
    let dummy3 = [0.0f64; 3];

    for _ in 0..4 {
        let mut gg = Vec::new();

        for i in 0..quad.len().saturating_sub(1) {
            gg.push(triple_distance(
                &field.point(&quad[i]),
                &field.point(&quad[i + 1]),
            ));
        }

        if gg.is_empty() {
            break;
        }

        gg.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = gg[gg.len() / 2];

        if med <= 0.0 {
            break;
        }

        let mut changed = false;
        let mut i = 0;

        while i + 1 < quad.len() && quad.len() < 4000 {
            if triple_distance(&field.point(&quad[i]), &field.point(&quad[i + 1])) > 1.5 * med {
                let mut midq = [0.0f64; 4];

                for k in 0..4 {
                    midq[k] = (quad[i][k] + quad[i + 1][k]) * 0.5;
                }

                if field.correct(&mut midq, false, &dummy3, &dummy3) {
                    quad.insert(i + 1, midq);
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
}

/// Append the loop start shifted by whole periods after the end; returns the shift.
fn close_quad(
    field: &SurfaceSurfaceField<'_>,
    quad: &mut Vec<[f64; 4]>,
    is_loop: bool,
) -> [f64; 4] {
    let mut closure = [0.0f64; 4];

    if !is_loop || quad.len() < 2 {
        return closure;
    }

    let mut virt = quad[0];
    let last = quad[quad.len() - 1];

    for k in 0..4 {
        let mut jump = quad[0][k] - last[k];

        if field.closed[k] {
            jump = unwrap_period(jump, 0.0, field.range[k]);
        }

        virt[k] = last[k] + jump;
        closure[k] = virt[k] - quad[0][k];
    }

    quad.push(virt);

    closure
}

/// Seam crossings (t, parameter, seam value) of the step pa -> pb, sorted by t.
fn quad_seam_crossings(
    field: &SurfaceSurfaceField<'_>,
    pa: &[f64; 4],
    pb: &[f64; 4],
) -> Vec<(f64, usize, f64)> {
    let mut crossings = Vec::new();

    for k in 0..4 {
        if !field.closed[k] || (pb[k] - pa[k]).abs() <= 1e-15 {
            continue;
        }

        let k0 = ((pa[k] - field.lo[k]) / field.range[k]).floor() as i32;
        let k1 = ((pb[k] - field.lo[k]) / field.range[k]).floor() as i32;

        for j in i32::min(k0, k1) + 1..=i32::max(k0, k1) {
            let seam = field.lo[k] + j as f64 * field.range[k];
            let t = (seam - pa[k]) / (pb[k] - pa[k]);

            if 0.0 < t && t < 1.0 {
                crossings.push((t, k, seam));
            }
        }
    }

    crossings.sort_by(|a, b| a.partial_cmp(b).unwrap());

    crossings
}

/// Snap closed parameters of p that sit on a seam onto it; false when none moved.
fn snap_quad_to_seam(field: &SurfaceSurfaceField<'_>, prev: &[f64; 4], p: &mut [f64; 4]) -> bool {
    let mut on_seam = false;

    for k in 0..4 {
        if !field.closed[k] {
            continue;
        }

        let j = ((p[k] - field.lo[k]) / field.range[k]).round();
        let seam = field.lo[k] + j * field.range[k];

        if (p[k] - seam).abs() < field.range[k] * 1e-9
            && (p[k] - prev[k]).abs() > field.range[k] * 1e-9
        {
            p[k] = seam;
            on_seam = true;
        }
    }

    on_seam
}

/// Insert corrected seam crossings into the run: (samples, indices of every seam sample).
fn split_quad_at_seams(
    field: &SurfaceSurfaceField<'_>,
    quad: &[[f64; 4]],
) -> (Vec<[f64; 4]>, Vec<usize>) {
    let mut out_pts = vec![quad[0]];
    let mut cross_idx = Vec::new();

    for i in 1..quad.len() {
        let pa = &quad[i - 1];
        let pb = &quad[i];

        for (t, idx, seam) in quad_seam_crossings(field, pa, pb) {
            let mut cp = [0.0f64; 4];

            for k in 0..4 {
                cp[k] = pa[k] + (pb[k] - pa[k]) * t;
            }

            cp[idx] = seam;
            field.correct_on_seam(&mut cp, idx);
            out_pts.push(cp);
            cross_idx.push(out_pts.len() - 1);
        }

        out_pts.push(*pb);
        let last = out_pts.len() - 1;

        if i < quad.len() - 1 && snap_quad_to_seam(field, pa, &mut out_pts[last]) {
            cross_idx.push(last);
        }
    }

    (out_pts, cross_idx)
}

/// Seam-free pieces of a split run, the last loop piece wrapped past the start by the closure shift.
fn quad_pieces(
    field: &SurfaceSurfaceField<'_>,
    out_pts: &[[f64; 4]],
    cross_idx: &[usize],
    is_loop: bool,
    closure: &[f64; 4],
) -> Vec<(Vec<[f64; 4]>, bool)> {
    let mut pieces = Vec::new();
    let mut wrap_drift = false;

    for k in 0..4 {
        if closure[k].abs() > field.range[k] * 0.5 {
            wrap_drift = true;
        }
    }

    if cross_idx.is_empty() {
        pieces.push((out_pts.to_vec(), is_loop && !wrap_drift));

        return pieces;
    }

    if !is_loop {
        let mut bounds = vec![0];
        bounds.extend_from_slice(cross_idx);
        bounds.push(out_pts.len() - 1);

        for bi in 0..bounds.len() - 1 {
            if bounds[bi + 1] > bounds[bi] {
                pieces.push((out_pts[bounds[bi]..=bounds[bi + 1]].to_vec(), false));
            }
        }

        return pieces;
    }

    for ci in 0..cross_idx.len() - 1 {
        pieces.push((out_pts[cross_idx[ci]..=cross_idx[ci + 1]].to_vec(), false));
    }

    let mut wrap_piece = out_pts[cross_idx[cross_idx.len() - 1]..].to_vec();

    for pi in 1..=cross_idx[0] {
        let mut p = [0.0f64; 4];

        for k in 0..4 {
            p[k] = out_pts[pi][k] + closure[k];
        }

        wrap_piece.push(p);
    }

    pieces.push((wrap_piece, false));

    pieces
}

/// Shift closed parameters of a piece by whole periods so its middle sample lies in the domain.
fn shift_quad_piece(field: &SurfaceSurfaceField<'_>, piece_pts: &mut [[f64; 4]]) {
    let mid = piece_pts[piece_pts.len() / 2];

    for k in 0..4 {
        if !field.closed[k] {
            continue;
        }

        let k_s = ((mid[k] - field.lo[k]) / field.range[k]).floor() as i32;

        if k_s != 0 {
            for p in piece_pts.iter_mut() {
                p[k] -= k_s as f64 * field.range[k];
            }
        }
    }
}

/// Distance of pm from the line through pa and pb, 0 for a degenerate chord.
fn chord_deviation(pa: &[f64; 3], pm: &[f64; 3], pb: &[f64; 3]) -> f64 {
    let ex = pb[0] - pa[0];
    let ey = pb[1] - pa[1];
    let ez = pb[2] - pa[2];
    let l2 = ex * ex + ey * ey + ez * ez;

    if l2 <= 1e-30 {
        return 0.0;
    }

    let tt = ((pm[0] - pa[0]) * ex + (pm[1] - pa[1]) * ey + (pm[2] - pa[2]) * ez) / l2;
    let c = [pa[0] + tt * ex, pa[1] + tt * ey, pa[2] + tt * ez];

    triple_distance(pm, &c)
}

/// Insert corrected midpoints that deviate from their chord, at most eight passes and 3000 samples.
fn refine_quad_piece(field: &SurfaceSurfaceField<'_>, piece_pts: &mut Vec<[f64; 4]>) {
    let dummy3 = [0.0f64; 3];
    let refine_tol = f64::max(field.tolerance * 100.0, 5e-6);

    for _ in 0..8 {
        let mut refined = false;
        let mut new_pp = vec![piece_pts[0]];
        let n = if piece_pts.len() < 3000 {
            piece_pts.len() - 1
        } else {
            0
        };

        for i in 0..n {
            let mut midq = [0.0f64; 4];

            for k in 0..4 {
                midq[k] = (piece_pts[i][k] + piece_pts[i + 1][k]) * 0.5;
            }

            if field.correct(&mut midq, false, &dummy3, &dummy3) {
                let dev = chord_deviation(
                    &field.point(&piece_pts[i]),
                    &field.point(&midq),
                    &field.point(&piece_pts[i + 1]),
                );

                if dev > refine_tol {
                    new_pp.push(midq);
                    refined = true;
                }
            }

            new_pp.push(piece_pts[i + 1]);
        }

        *piece_pts = new_pp;

        if !refined {
            break;
        }
    }
}

/// Sum of the turning angles along a 3D polyline.
fn total_turning_3d(pts: &[Point]) -> f64 {
    let mut turning = 0.0;

    for i in 1..pts.len().saturating_sub(1) {
        let dx1 = pts[i][0] - pts[i - 1][0];
        let dy1 = pts[i][1] - pts[i - 1][1];
        let dz1 = pts[i][2] - pts[i - 1][2];
        let dx2 = pts[i + 1][0] - pts[i][0];
        let dy2 = pts[i + 1][1] - pts[i][1];
        let dz2 = pts[i + 1][2] - pts[i][2];
        let l1 = (dx1 * dx1 + dy1 * dy1 + dz1 * dz1).sqrt();
        let l2 = (dx2 * dx2 + dy2 * dy2 + dz2 * dz2).sqrt();

        if l1 > 1e-14 && l2 > 1e-14 {
            let c = ((dx1 * dx2 + dy1 * dy2 + dz1 * dz2) / (l1 * l2)).clamp(-1.0, 1.0);
            turning += c.acos();
        }
    }

    turning
}

/// Cubic fitted to a traced run, CVs doubled until within fit_tol, interpolated when fitting fails.
fn fit_track(pts: &[Point], fit_tol: f64, is_loop: bool) -> NurbsCurve {
    let mp = pts.len();
    let chords = chord_parameters(pts, is_loop);
    let mut target_cvs = usize::max(8, (total_turning_3d(pts) / 0.5) as usize + 6);
    let max_cvs = usize::max(8, usize::min(mp.saturating_sub(1), mp / 3));
    let mut best = NurbsCurve::default();
    let mut best_dev = f64::INFINITY;

    while target_cvs <= max_cvs {
        let crv = NurbsCurve::create_fitted(pts, target_cvs, 3, is_loop);

        if !crv.is_valid() {
            break;
        }

        let dev = fitted_max_deviation(&crv, pts, &chords, 24);

        if dev < best_dev {
            best = crv;
            best_dev = dev;
        }

        if dev < fit_tol {
            break;
        }

        target_cvs *= 2;
    }

    if best_dev >= fit_tol {
        let interp = if is_loop {
            NurbsCurve::create_interpolated(
                pts,
                CurveNurbsKnotStyle::ChordPeriodic,
                CurveInterpStyle::Rhino,
            )
        } else {
            NurbsCurve::create_interpolated(
                pts,
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
}

/// Section triple of one seam-free piece: refined, then its 3D curve and both pcurves fitted; None when degenerate.
fn piece_triple(
    field: &SurfaceSurfaceField<'_>,
    piece_pts: &mut Vec<[f64; 4]>,
    piece_loop: bool,
) -> Option<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    shift_quad_piece(field, piece_pts);
    let mut chord3 = 0.0;

    for i in 1..piece_pts.len() {
        chord3 += triple_distance(&field.point(&piece_pts[i]), &field.point(&piece_pts[i - 1]));
    }

    if chord3 < field.h_init * 0.05 {
        return None;
    }

    refine_quad_piece(field, piece_pts);
    let mut pts3 = Vec::new();
    let mut pts_pa = Vec::new();
    let mut pts_pb = Vec::new();

    for q in piece_pts.iter() {
        let p = field.point(q);
        pts3.push(Point::new(p[0], p[1], p[2]));
        pts_pa.push(Point::new(q[0], q[1], 0.0));
        pts_pb.push(Point::new(q[2], q[3], 0.0));
    }

    let crv3 = fit_track(&pts3, f64::max(field.tolerance * 10.0, 1e-7), piece_loop);
    let pcurve_a = fit_track(
        &pts_pa,
        f64::min(field.step[0], field.step[1]) * 1e-4,
        piece_loop,
    );
    let pcurve_b = fit_track(
        &pts_pb,
        f64::min(field.step[2], field.step[3]) * 1e-4,
        piece_loop,
    );

    if !crv3.is_valid() || !pcurve_a.is_valid() || !pcurve_b.is_valid() {
        return None;
    }

    Some((crv3, pcurve_a, pcurve_b))
}

/// Section triples of two freeform surfaces by seeding, marching and fitting every trace.
fn marched_section_triples(
    a: &NurbsSurface,
    b: &NurbsSurface,
    tolerance: f64,
) -> Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    let field = SurfaceSurfaceField::new(a, b, tolerance);
    let dummy3 = [0.0f64; 3];
    let mut seeds = surface_surface_seeds(&field);
    let mut result = Vec::new();
    let mut kept_pts3: Vec<Vec<[f64; 3]>> = Vec::new();

    for si in 0..seeds.len() {
        if seeds[si].used {
            continue;
        }

        seeds[si].used = true;
        let mut x0 = [seeds[si].u, seeds[si].v, seeds[si].s, seeds[si].t];

        if !field.correct(&mut x0, false, &dummy3, &dummy3) {
            continue;
        }

        let (mut quad, is_loop) = match trace_seed(&field, &x0, &mut seeds) {
            Some(traced) => traced,
            None => continue,
        };
        let mut trace_pts3 = Vec::new();

        for q in &quad {
            trace_pts3.push(field.point(q));
        }

        if is_duplicate_quad(&trace_pts3, &kept_pts3, field.h_init * 2.0) {
            continue;
        }

        kept_pts3.push(trace_pts3);
        densify_quad(&field, &mut quad);
        let closure = close_quad(&field, &mut quad, is_loop);
        let (out_pts, cross_idx) = split_quad_at_seams(&field, &quad);

        for (mut piece_pts, piece_loop) in
            quad_pieces(&field, &out_pts, &cross_idx, is_loop, &closure)
        {
            if piece_pts.len() < 2 {
                continue;
            }

            if let Some(triple) = piece_triple(&field, &mut piece_pts, piece_loop) {
                result.push(triple);
            }
        }
    }

    drop_point_sections(result, tolerance)
}

/// Surface-surface section curves with their UV pcurves on both surfaces.
pub fn surface_surface(
    a: &NurbsSurface,
    b: &NurbsSurface,
    tolerance: Option<f64>,
) -> Vec<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    if !a.is_valid() || !b.is_valid() {
        return vec![];
    }

    let tolerance = match tolerance {
        Some(t) if t > 0.0 => t,
        _ => Tolerance::ZERO_TOLERANCE,
    };
    let analytic = analytic_ssi(a, b, tolerance);

    if analytic.status != AnalyticStatus::NotAnalytic {
        return drop_point_sections(analytic.triples, tolerance);
    }

    if a.is_planar(None, 1e-9) {
        return planar_section_triples(a, b, true, tolerance);
    }

    if b.is_planar(None, 1e-9) {
        return planar_section_triples(b, a, false, tolerance);
    }

    marched_section_triples(a, b, tolerance)
}

/// Distance from a pcurve's lifted point to the cutter: clamped in the corner frame of a rectangle, else to the boundary polygon.
struct CutterGap<'a> {
    target: &'a NurbsSurface, // Surface the pcurve lives on.
    pc: &'a NurbsCurve,       // Pcurve on the target.
    cutter: &'a NurbsSurface, // Cutting surface.
    q00: Point,               // Cutter corner at (u0, v0).
    eu: [f64; 3],             // Cutter edge to (u1, v0).
    ev: [f64; 3],             // Cutter edge to (u0, v1).
    eu2: f64,                 // Squared length of eu.
    ev2: f64,                 // Squared length of ev.
    rectangle: bool,          // Whether the cutter is a 2 x 2 rectangle.
    frame: Plane,             // Plane of the boundary polygon.
    outline: Polyline,        // Boundary polygon in the frame, empty without area.
}

impl<'a> CutterGap<'a> {
    /// Corner frame and boundary polygon of the cutter.
    fn new(target: &'a NurbsSurface, pc: &'a NurbsCurve, cutter: &'a NurbsSurface) -> Self {
        let (cu0, cu1) = srf_domain(cutter, 0);
        let (cv0, cv1) = srf_domain(cutter, 1);
        let q00 = srf_point(cutter, cu0, cv0);
        let q10 = srf_point(cutter, cu1, cv0);
        let q01 = srf_point(cutter, cu0, cv1);
        let q11 = srf_point(cutter, cu1, cv1);
        let eu = [q10[0] - q00[0], q10[1] - q00[1], q10[2] - q00[2]];
        let ev = [q01[0] - q00[0], q01[1] - q00[1], q01[2] - q00[2]];
        let eu2 = eu[0] * eu[0] + eu[1] * eu[1] + eu[2] * eu[2];
        let ev2 = ev[0] * ev[0] + ev[1] * ev[1] + ev[2] * ev[2];
        let square =
            (eu[0] * ev[0] + eu[1] * ev[1] + eu[2] * ev[2]).abs() <= 1e-9 * (eu2 * ev2).sqrt();
        let corner = Point::new(
            q00[0] + eu[0] + ev[0],
            q00[1] + eu[1] + ev[1],
            q00[2] + eu[2] + ev[2],
        );
        let parallelogram = q11.distance(&corner, None) <= 1e-9 * (eu2 + ev2).sqrt();
        let bilinear = cutter.cv_count(0) == 2 && cutter.cv_count(1) == 2;
        let rectangle = eu2 > 1e-28 && ev2 > 1e-28 && bilinear && square && parallelogram;
        let (outline, frame) = if rectangle {
            (Polyline::default(), Plane::default())
        } else {
            boundary_outline(cutter)
        };

        CutterGap {
            target,
            pc,
            cutter,
            q00,
            eu,
            ev,
            eu2,
            ev2,
            rectangle,
            frame,
            outline,
        }
    }

    /// Distance to the cutter at pcurve parameter t.
    fn gap(&self, t: f64) -> f64 {
        let uv = self.pc.point_at(t);
        let p3 = srf_point(self.target, uv[0], uv[1]);

        if self.rectangle {
            return self.rectangle_gap(&p3);
        }

        if self.outline.point_count() == 0 {
            return Closest::surface_point(self.cutter, &p3, 0.0, 0.0, 0.0, 0.0).2;
        }

        self.outline_gap(&p3)
    }

    /// Distance from p3 to the rectangle spanned by eu and ev.
    fn rectangle_gap(&self, p3: &Point) -> f64 {
        let q00 = &self.q00;
        let eu = &self.eu;
        let ev = &self.ev;
        let dx = p3[0] - q00[0];
        let dy = p3[1] - q00[1];
        let dz = p3[2] - q00[2];
        let a = ((dx * eu[0] + dy * eu[1] + dz * eu[2]) / self.eu2).clamp(0.0, 1.0);
        let b = ((dx * ev[0] + dy * ev[1] + dz * ev[2]) / self.ev2).clamp(0.0, 1.0);
        let cx = q00[0] + a * eu[0] + b * ev[0];
        let cy = q00[1] + a * eu[1] + b * ev[1];
        let cz = q00[2] + a * eu[2] + b * ev[2];

        ((p3[0] - cx) * (p3[0] - cx) + (p3[1] - cy) * (p3[1] - cy) + (p3[2] - cz) * (p3[2] - cz))
            .sqrt()
    }

    /// Distance from p3 to the region inside the boundary polygon.
    fn outline_gap(&self, p3: &Point) -> f64 {
        let d = p3 - &self.frame.origin();
        let p = Point::new(
            d.dot(&self.frame.x_axis()),
            d.dot(&self.frame.y_axis()),
            d.dot(&self.frame.z_axis()),
        );

        if self.outline.point_in_polygon_2d(&p) {
            return p[2].abs();
        }

        let n = self.outline.point_count();
        let mut d2 = f64::MAX;

        for i in 0..n {
            let a = &self.outline[i];
            let b = &self.outline[(i + 1) % n];
            let ex = b[0] - a[0];
            let ey = b[1] - a[1];
            let len2 = ex * ex + ey * ey;
            let mut s = 0.0;

            if len2 > 0.0 {
                s = (((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / len2).clamp(0.0, 1.0);
            }

            let dx = p[0] - a[0] - s * ex;
            let dy = p[1] - a[1] - s * ey;
            d2 = f64::min(d2, dx * dx + dy * dy);
        }

        (d2 + p[2] * p[2]).sqrt()
    }
}

/// Footprint edge between an inside and an outside parameter, by 24 bisections.
fn refine_footprint_edge(g: &CutterGap<'_>, t_in: f64, t_out: f64, edge_tol: f64) -> f64 {
    let mut a = t_in;
    let mut b = t_out;

    for _ in 0..24 {
        let tm = (a + b) * 0.5;

        if g.gap(tm) < edge_tol {
            a = tm;
        } else {
            b = tm;
        }
    }

    b
}

/// Parameter spans of the pcurve inside the footprint from n + 1 samples, ends bisected to the edge.
fn footprint_spans(g: &CutterGap<'_>, n: usize, on_tol: f64, edge_tol: f64) -> Vec<(f64, f64)> {
    let (d0, d1) = g.pc.domain();
    let mut flags = Vec::with_capacity(n + 1);

    for i in 0..=n {
        let t = d0 + (d1 - d0) * i as f64 / n as f64;
        flags.push((t, g.gap(t) < on_tol));
    }

    let mut spans = Vec::new();
    let mut i = 0;

    while i <= n {
        if !flags[i].1 {
            i += 1;
            continue;
        }

        let mut j = i;

        while j < n && flags[j + 1].1 {
            j += 1;
        }

        let ta = if i == 0 {
            flags[i].0
        } else {
            refine_footprint_edge(g, flags[i].0, flags[i - 1].0, edge_tol)
        };
        let tb = if j == n {
            flags[j].0
        } else {
            refine_footprint_edge(g, flags[j].0, flags[j + 1].0, edge_tol)
        };

        if tb - ta > (d1 - d0) * 1e-6 {
            spans.push((ta, tb));
        }

        i = j + 1;
    }

    spans
}

/// Join the first and last spans of a closed pcurve across its start into one polyline piece.
fn join_wrapped_spans(
    pc: &NurbsCurve,
    n: usize,
    spans: &mut Vec<(f64, f64)>,
    pieces: &mut Vec<NurbsCurve>,
) {
    let (d0, d1) = pc.domain();
    let pc_closed = pc.point_at(d0).distance(&pc.point_at(d1), None) < 1e-9;
    let wraps = pc_closed
        && spans.len() >= 2
        && spans[0].0 <= d0 + (d1 - d0) * 1e-9
        && spans[spans.len() - 1].1 >= d1 - (d1 - d0) * 1e-9;

    if !wraps {
        return;
    }

    let ta = spans[spans.len() - 1].0;
    let tb = spans[0].1;
    spans.pop();
    spans.remove(0);
    let m2 = usize::max(32, n / 2);
    let mut pts = Vec::new();
    let len1 = d1 - ta;
    let len2 = tb - d0;
    let tot = len1 + len2;

    for k2 in 0..=m2 {
        let f = tot * k2 as f64 / m2 as f64;
        let t = if f < len1 { ta + f } else { d0 + (f - len1) };
        pts.push(pc.point_at(f64::min(t, d1)));
    }

    let joined = NurbsCurve::create(false, 1, &pts);

    if joined.is_valid() {
        pieces.push(joined);
    }
}

/// Keep the pcurve sub-segments whose lifted 3D point lies inside the cutter footprint.
fn clip_pcurve_to_cutter(
    target: &NurbsSurface,
    pc: &NurbsCurve,
    cutter: &NurbsSurface,
) -> Vec<NurbsCurve> {
    let g = CutterGap::new(target, pc, cutter);
    let (_, cu1) = srf_domain(cutter, 0);
    let (_, cv1) = srf_domain(cutter, 1);
    let corner_diag = g.q00.distance(&srf_point(cutter, cu1, cv1), None);
    let n = usize::max(pc.cv_count() * 4, 16);
    let mut spans = footprint_spans(
        &g,
        n,
        f64::max(1e-6, corner_diag * 2e-3),
        f64::max(1e-6, corner_diag * 2e-4),
    );
    let mut pieces = Vec::new();
    join_wrapped_spans(pc, n, &mut spans, &mut pieces);

    for sp in &spans {
        let mut piece = pc.clone();

        if piece.trim(sp.0, sp.1) && piece.is_valid() {
            pieces.push(piece);
        }
    }

    pieces
}

/// Pcurves of one section on the target: analytic, pulled back, projected, then the traced pcurve.
fn target_pcurves(
    target: &NurbsSurface,
    rt: &RecogSurface,
    tr: &(NurbsCurve, NurbsCurve, NurbsCurve),
    tolerance: f64,
) -> Vec<NurbsCurve> {
    let c3d = &tr.0;
    let pa_tr = &tr.1;
    let pa_an = analytic_pcurve(target, rt, c3d);

    if pa_an.is_valid() {
        return vec![pa_an];
    }

    if rt.kind == RecogKind::None || rt.kind == RecogKind::Plane {
        if pa_tr.is_valid() {
            return vec![pa_tr.clone()];
        }

        return Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
    }

    let mut pcs = run_curves(&analytic_pullback(target, rt, c3d));

    if pcs.is_empty() {
        pcs = Closest::surface_curve(target, c3d, 0.0, 0.0, tolerance);
    }

    if pcs.is_empty() {
        pcs.push(pa_tr.clone());
    }

    pcs
}

/// UV pcurves of the cutter's section on the target, clipped to the cutter footprint.
pub fn cut_curves_on_surface(
    target: &NurbsSurface,
    cutter: &NurbsSurface,
    tolerance: Option<f64>,
) -> Vec<NurbsCurve> {
    let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
    let mut out = Vec::new();
    let cutter_planar = cutter.is_planar(None, 1e-6);
    let rt = recognize_surface(target, f64::max(tolerance, 1e-7) * 1e4);

    for tr in surface_surface(target, cutter, Some(tolerance)) {
        for pc in target_pcurves(target, &rt, &tr, tolerance) {
            if !cutter_planar {
                out.push(pc);
                continue;
            }

            out.extend(clip_pcurve_to_cutter(target, &pc, cutter));
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
pub fn plane_4planes(main_plane: &Plane, planes: &[Plane; 4]) -> Option<Polyline> {
    let p0 = plane_plane_plane_check(&planes[0], &planes[1], main_plane, 0.1)?;
    let p1 = plane_plane_plane_check(&planes[1], &planes[2], main_plane, 0.1)?;
    let p2 = plane_plane_plane_check(&planes[2], &planes[3], main_plane, 0.1)?;
    let p3 = plane_plane_plane_check(&planes[3], &planes[0], main_plane, 0.1)?;

    Some(Polyline::new(vec![p0.clone(), p1, p2, p3, p0]))
}

/// Open four-point polyline of the main plane cut by four ordered boundary planes.
pub fn plane_4planes_open(main_plane: &Plane, planes: &[Plane; 4]) -> Option<Polyline> {
    let mut corners: Vec<Point> = Vec::with_capacity(4);

    for i in 0..4 {
        let edge = plane_plane_to_line_canonical(&planes[i], &planes[(i + 1) % 4])?;
        corners.push(line_plane(&edge, main_plane, false)?);
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
pub fn line_two_planes(line: &Line, plane0: &Plane, plane1: &Plane) -> Option<Line> {
    let q0 = line_plane(line, plane0, true)?;
    let q1 = line_plane(line, plane1, true)?;

    Some(Line::new(q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]))
}

/// Polyline edge crossings with a plane and their edge indices.
pub fn polyline_plane(polyline: &Polyline, plane: &Plane) -> Option<(Vec<Point>, Vec<usize>)> {
    let n = polyline.point_count();

    if n < 2 {
        return None;
    }

    let mut points: Vec<Point> = Vec::new();
    let mut edge_ids: Vec<usize> = Vec::new();

    for i in 0..n - 1 {
        let a = polyline.get_point(i)?;
        let b = polyline.get_point(i + 1)?;
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
                let front = polyline.get_point(0)?;
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
pub fn scale_vector_to_distance_of_2planes(
    direction: &Vector,
    plane0: &Plane,
    plane1: &Plane,
) -> Option<Vector> {
    if direction.magnitude() < Tolerance::ZERO_TOLERANCE {
        return None;
    }

    let origin = Point::new(0.0, 0.0, 0.0);
    let tip = Point::new(direction[0], direction[1], direction[2]);
    let ray = Line::new(origin[0], origin[1], origin[2], tip[0], tip[1], tip[2]);
    let q0 = line_plane(&ray, plane0, false)?;
    let q1 = line_plane(&ray, plane1, false)?;
    let output = &q1 - &q0;
    let n1 = plane1.z_axis();
    let n1_mag = n1.magnitude();

    if n1_mag < Tolerance::ZERO_TOLERANCE {
        return None;
    }

    let o0 = plane0.origin();
    let d = (o0[0] - plane1.origin()[0]) * n1[0] / n1_mag
        + (o0[1] - plane1.origin()[1]) * n1[1] / n1_mag
        + (o0[2] - plane1.origin()[2]) * n1[2] / n1_mag;

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

/// Boolean of two flat polylines, intersection_type 0 intersect, 1 union, 2 difference, 3 xor; empty on failure.
fn polyline_boolean_2d(a2d: &Polyline, b2d: &Polyline, intersection_type: i32) -> Vec<Polyline> {
    if (0..=2).contains(&intersection_type) {
        return Polyline::boolean_op(a2d, b2d, intersection_type, None);
    }

    if intersection_type != 3 {
        return Vec::new();
    }

    let u = Polyline::boolean_op(a2d, b2d, 1, None);
    let inter = Polyline::boolean_op(a2d, b2d, 0, None);

    if u.is_empty() {
        return Vec::new();
    }

    if inter.is_empty() {
        return u;
    }

    Polyline::boolean_op(&u[0], &inter[0], 2, None)
}

/// Ring without consecutive points closer than eps, the closing point included.
fn collapse_close_points(ring: &[[f64; 2]], eps: f64) -> Vec<[f64; 2]> {
    let eps_sq = eps * eps;
    let mut collapsed: Vec<[f64; 2]> = Vec::with_capacity(ring.len());

    for p in ring {
        if collapsed.is_empty() || distance_sq_2d(p, collapsed.last().unwrap()) >= eps_sq {
            collapsed.push(*p);
        }
    }

    if collapsed.len() >= 2 && distance_sq_2d(collapsed.last().unwrap(), &collapsed[0]) < eps_sq {
        collapsed.pop();
    }

    collapsed
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
    let result_2d = polyline_boolean_2d(&a2d, &b2d, intersection_type);

    if result_2d.is_empty() {
        return None;
    }

    let mut ring = polyline_to_2d(&result_2d[0], &flat_origin, &flat_x, &flat_y);

    if ring.len() < 3 {
        return None;
    }

    if collapse_eps > 0.0 {
        ring = collapse_close_points(&ring, collapse_eps);

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
/// Bounding box (min xyz, max xyz) of every face, padded by tolerance.
fn padded_face_boxes(polylines: &[Vec<Polyline>], tolerance: f64) -> Vec<Vec<[f64; 6]>> {
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
                bx[k] -= tolerance;
                bx[k + 3] += tolerance;
            }

            boxes.push(bx);
        }

        face_boxes.push(boxes);
    }

    face_boxes
}

/// Closed overlap of two coplanar faces in the frame of face_a's first edge and normal za, None when they only touch.
fn coplanar_face_overlap(face_a: &Polyline, za: &Vector, face_b: &Polyline) -> Option<Polyline> {
    let pts_i = face_a.get_points();
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
    let bools = Polyline::boolean_op(face_a, face_b, 0, Some(&pln));

    if bools.is_empty() || bools[0].point_count() < 3 {
        return None;
    }

    if bools[0].is_closed() {
        Some(bools[0].clone())
    } else {
        Some(bools[0].closed())
    }
}

/// Face-to-face contacts (a, b, face_a, face_b, type, polyline) with type 0 side-side, 1 side-top, 2 top-top.
pub fn face_to_face(
    adjacency: &[i32],
    polylines: &[Vec<Polyline>],
    planes: &[Vec<Plane>],
    coplanar_tolerance: f64,
) -> Vec<(i32, i32, i32, i32, i32, Polyline)> {
    let mut results = Vec::new();
    let face_boxes = padded_face_boxes(polylines, coplanar_tolerance);
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

                let Some(jpl) = coplanar_face_overlap(&polylines[a][i], &za, &polylines[b][j])
                else {
                    continue;
                };

                let typ = (if i > 1 { 0 } else { 1 }) + (if j > 1 { 0 } else { 1 });
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

/// Directions of two segments, their unit normal, and whether they are parallel within one degree.
fn line_line_frame(
    s0: &Line,
    s1: &Line,
    v0: &mut Vector,
    v1: &mut Vector,
    normal: &mut Vector,
) -> bool {
    const EPS_PAR: f64 = 1.0;
    *v0 = s0.to_vector();
    *v1 = s1.to_vector();
    *normal = v0.cross(v1);
    let ang = v0.angle(v1, false, true, None);
    let is_parallel = normal.magnitude_squared() < 1e-24 || (90.0 - (ang - 90.0).abs()) < EPS_PAR;

    if is_parallel {
        *normal = Plane::from_point_normal(s0.start(), v0.clone(), None).base1();
    }

    normal.normalize_self();

    is_parallel
}

/// End shared by two segments as p0 and p1, with unit directions leaving it along each segment.
fn line_line_shared_end(
    s0: &Line,
    s1: &Line,
    p0: &mut Point,
    p1: &mut Point,
    v0: &mut Vector,
    v1: &mut Vector,
) -> bool {
    const DIST_SQ: f64 = 1e-6;
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

            return true;
        }
    }

    false
}

/// Closest points of two parallel segments at the middle of their overlap, each unit direction flipped to leave its nearer end.
fn line_line_parallel(
    s0: &Line,
    s1: &Line,
    p0: &mut Point,
    p1: &mut Point,
    v0: &mut Vector,
    v1: &mut Vector,
) {
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
}

/// Types (0 end, 1 side) from the positions tt0 and tt1 along the polylines, the direction of an end past the middle flipped.
fn line_line_types(
    tt0: f64,
    tt1: f64,
    above_closer_to_edge: f64,
    type0: &mut bool,
    type1: &mut bool,
    v0: &mut Vector,
    v1: &mut Vector,
) {
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
    *is_parallel = line_line_frame(s0, s1, v0, v1, normal);

    if line_line_shared_end(s0, s1, p0, p1, v0, v1) {
        *type0 = false;
        *type1 = false;

        return true;
    }

    v0.normalize_self();
    v1.normalize_self();

    if *is_parallel {
        line_line_parallel(s0, s1, p0, p1, v0, v1);
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
    line_line_types(tt0, tt1, above_closer_to_edge, type0, type1, v0, v1);

    true
}
