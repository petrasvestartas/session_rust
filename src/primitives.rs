#![allow(clippy::needless_range_loop)]
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbsknot;
use crate::nurbsknot::CurveInterpStyle;
use crate::nurbsknot::CurveNurbsKnotStyle;
use crate::nurbssurface::NurbsSurface;
use crate::plane::Plane;
use crate::point::Point;
use crate::tolerance::Tolerance;
use crate::tolerance::PI;
use crate::vector::Vector;
use crate::xform::Xform;

// ═══════════════════════════════════════════════════════════════════════════
// Rational quadratic circle pattern
// ═══════════════════════════════════════════════════════════════════════════

const CIRCLE_W: f64 = std::f64::consts::FRAC_1_SQRT_2;
const CIRCLE_X: [f64; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
const CIRCLE_Y: [f64; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
const CIRCLE_WEIGHTS: [f64; 9] = [
    1.0, CIRCLE_W, 1.0, CIRCLE_W, 1.0, CIRCLE_W, 1.0, CIRCLE_W, 1.0,
];
const CIRCLE_NURBSKNOTS: [f64; 10] = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

/// Row j of a surface set to a circle in the plane z = cz, weights scaled by weight.
fn set_circle_row(
    srf: &mut NurbsSurface,
    j: usize,
    cx: f64,
    cy: f64,
    cz: f64,
    radius: f64,
    weight: f64,
) {
    for i in 0..9 {
        let w = CIRCLE_WEIGHTS[i] * weight;
        let px = cx + radius * CIRCLE_X[i];
        let py = cy + radius * CIRCLE_Y[i];
        srf.set_cv_4d(i, j, px * w, py * w, cz * w, w);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Appends n points of a circle of the given radius in the plane z.
fn add_ring(vertices: &mut Vec<Point>, n: usize, radius: f64, z: f64) {
    for i in 0..n {
        let angle = 2.0 * PI * i as f64 / n as f64;
        vertices.push(Point::new(radius * angle.cos(), radius * angle.sin(), z));
    }
}

/// Face without consecutive duplicate vertices.
fn dedup_face(face: &[usize]) -> Vec<usize> {
    let mut unique = Vec::new();

    for k in 0..face.len() {
        if face[k] != face[(k + 1) % face.len()] {
            unique.push(face[k]);
        }
    }

    unique
}

/// Vertex keys of the surface sampled on a (u_count + 1) x (v_count + 1) grid; seam and poles share keys.
fn surface_grid(
    surface: &NurbsSurface,
    u_count: usize,
    v_count: usize,
    mesh: &mut Mesh,
) -> Vec<Vec<usize>> {
    let (u0, u1) = surface.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = surface.domain(1).unwrap_or((0.0, 1.0));
    let closed_u = surface.is_closed(0);
    let singular_south = surface.is_singular(0);
    let singular_north = surface.is_singular(2);
    let mut grid = vec![vec![0usize; v_count + 1]; u_count + 1];

    for i in 0..=u_count {
        let u = u0 + (u1 - u0) * i as f64 / u_count as f64;

        for j in 0..=v_count {
            let v = v0 + (v1 - v0) * j as f64 / v_count as f64;

            if closed_u && i == u_count {
                grid[i][j] = grid[0][j];
            } else if singular_south && j == 0 && i > 0 {
                grid[i][j] = grid[0][0];
            } else if singular_north && j == v_count && i > 0 {
                grid[i][j] = grid[0][v_count];
            } else {
                grid[i][j] = mesh.add_vertex(surface.point_at(u, v).unwrap_or_default(), None);
            }
        }
    }

    grid
}

/// Vertex keys of the surface sampled at v offset by t cells on a (u_count + 1) x v_count grid; seam shares keys.
fn surface_mid_grid(
    surface: &NurbsSurface,
    u_count: usize,
    v_count: usize,
    t: f64,
    mesh: &mut Mesh,
) -> Vec<Vec<usize>> {
    let (u0, u1) = surface.domain(0).unwrap_or((0.0, 1.0));
    let (v0, v1) = surface.domain(1).unwrap_or((0.0, 1.0));
    let closed_u = surface.is_closed(0);
    let mut grid = vec![vec![0usize; v_count]; u_count + 1];

    for i in 0..=u_count {
        let u = u0 + (u1 - u0) * i as f64 / u_count as f64;

        for j in 0..v_count {
            let v = v0 + (v1 - v0) * (j as f64 + t) / v_count as f64;

            if closed_u && i == u_count {
                grid[i][j] = grid[0][j];
            } else {
                grid[i][j] = mesh.add_vertex(surface.point_at(u, v).unwrap_or_default(), None);
            }
        }
    }

    grid
}

// ═══════════════════════════════════════════════════════════════════════════
// Curve compatibility
// ═══════════════════════════════════════════════════════════════════════════

/// Sorted union of two nurbsknot vectors, equal values kept once.
fn merge_nurbsknot_vectors(a: &[f64], b: &[f64]) -> Vec<f64> {
    let tol = 1e-10;
    let mut merged = Vec::new();
    let mut i = 0;
    let mut j = 0;

    while i < a.len() && j < b.len() {
        if (a[i] - b[j]).abs() < tol {
            merged.push(a[i]);
            i += 1;
            j += 1;
        } else if a[i] < b[j] {
            merged.push(a[i]);
            i += 1;
        } else {
            merged.push(b[j]);
            j += 1;
        }
    }

    while i < a.len() {
        merged.push(a[i]);
        i += 1;
    }

    while j < b.len() {
        merged.push(b[j]);
        j += 1;
    }

    merged
}

/// True when both nurbsknot vectors match within 1e-10.
fn nurbsknot_vectors_equal(a: &[f64], b: &[f64]) -> bool {
    let tol = 1e-10;

    if a.len() != b.len() {
        return false;
    }

    for i in 0..a.len() {
        if (a[i] - b[i]).abs() > tol {
            return false;
        }
    }

    true
}

/// Same degree, rationality, domain [0, 1] and nurbsknot vector for every curve.
fn make_curves_compatible(curves: &mut [NurbsCurve]) {
    if curves.len() < 2 {
        return;
    }

    let mut max_degree = 0;
    let mut any_rational = false;

    for c in curves.iter() {
        max_degree = max_degree.max(c.degree());
        any_rational = any_rational || c.is_rational();
    }

    for c in curves.iter_mut() {
        if c.degree() < max_degree {
            c.increase_degree(max_degree);
        }

        if any_rational {
            c.make_rational();
        }
    }

    let mut compatible = true;

    for i in 1..curves.len() {
        if curves[i].cv_count() != curves[0].cv_count()
            || !nurbsknot_vectors_equal(&curves[i].get_nurbsknots(), &curves[0].get_nurbsknots())
        {
            compatible = false;
        }
    }

    if compatible {
        return;
    }

    for c in curves.iter_mut() {
        c.set_domain(0.0, 1.0);
    }

    let mut unified = curves[0].get_nurbsknots();

    for i in 1..curves.len() {
        unified = merge_nurbsknot_vectors(&unified, &curves[i].get_nurbsknots());
    }

    let tol = 1e-10;

    for c in curves.iter_mut() {
        let nurbsknots = c.get_nurbsknots();
        let mut ci = 0;

        for ui in 0..unified.len() {
            if ci < nurbsknots.len() && (nurbsknots[ci] - unified[ui]).abs() < tol {
                ci += 1;
            } else {
                c.insert_nurbsknot(unified[ui], 1);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Planar helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Bilinear patch: u runs p00 to p10, v runs p00 to p01.
fn bilinear_patch(p00: &Point, p10: &Point, p01: &Point, p11: &Point) -> NurbsSurface {
    let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
    srf.set_cv(0, 0, p00);
    srf.set_cv(1, 0, p10);
    srf.set_cv(0, 1, p01);
    srf.set_cv(1, 1, p11);

    srf
}

/// Unit direction of the longest edge of a closed polygon.
fn longest_edge_dir(pts: &[Point]) -> Vector {
    let mut best = Vector::new(0.0, 0.0, 0.0);

    for i in 0..pts.len() {
        let edge = &pts[(i + 1) % pts.len()] - &pts[i];

        if edge.magnitude() > best.magnitude() {
            best = edge;
        }
    }

    best.normalized()
}

/// Bilinear patch in the frame covering the points with a 5% margin.
fn bounded_patch(pts: &[Point], origin: &Point, x_axis: &Vector, y_axis: &Vector) -> NurbsSurface {
    let mut min_u = 1e30;
    let mut max_u = -1e30;
    let mut min_v = 1e30;
    let mut max_v = -1e30;

    for pt in pts {
        let d = pt - origin;
        min_u = f64::min(min_u, d.dot(x_axis));
        max_u = f64::max(max_u, d.dot(x_axis));
        min_v = f64::min(min_v, d.dot(y_axis));
        max_v = f64::max(max_v, d.dot(y_axis));
    }

    let mut pad = f64::max(max_u - min_u, max_v - min_v) * 0.05;

    if pad < 1e-6 {
        pad = 1.0;
    }

    min_u -= pad;
    max_u += pad;
    min_v -= pad;
    max_v += pad;

    bilinear_patch(
        &(origin + x_axis * min_u + y_axis * min_v),
        &(origin + x_axis * max_u + y_axis * min_v),
        &(origin + x_axis * min_u + y_axis * max_v),
        &(origin + x_axis * max_u + y_axis * max_v),
    )
}

// ═══════════════════════════════════════════════════════════════════════════
// Loft helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Section parameters in [0, 1] from the mean CV distance between consecutive sections.
fn loft_section_params(curves: &[NurbsCurve]) -> Vec<f64> {
    let n = curves.len();
    let cv_count = curves[0].cv_count();
    let mut v_params = vec![0.0; n];

    for k in 1..n {
        let mut sum = 0.0;

        for i in 0..cv_count {
            sum += curves[k - 1]
                .get_cv(i)
                .unwrap_or_default()
                .distance(&curves[k].get_cv(i).unwrap_or_default(), None);
        }

        v_params[k] = v_params[k - 1] + sum / cv_count as f64;
    }

    let total = v_params[n - 1];

    for k in 0..n {
        v_params[k] = if total > 1e-14 {
            v_params[k] / total
        } else {
            k as f64 / (n - 1) as f64
        };
    }

    v_params
}

/// Clamped nurbsknot vector averaging the section parameters.
fn loft_nurbsknots(v_params: &[f64], order_v: usize) -> Vec<f64> {
    let n = v_params.len();
    let degree_v = order_v - 1;
    let mut nurbsknots = vec![v_params[0]; order_v + n - 2];

    for j in 1..=(n - order_v) {
        let mut sum = 0.0;

        for i in j..(j + degree_v) {
            sum += v_params[i];
        }

        nurbsknots[degree_v - 1 + j] = sum / degree_v as f64;
    }

    for i in (n - 1)..(order_v + n - 2) {
        nurbsknots[i] = v_params[n - 1];
    }

    nurbsknots
}

/// Row of the collocation matrix: the cv_count basis values at t.
fn loft_basis_row(nurbsknots: &[f64], order: usize, cv_count: usize, t: f64) -> Vec<f64> {
    let mut row = vec![0.0; cv_count];
    let span = nurbsknot::find_span(order, cv_count, nurbsknots, t, 0, 0);
    let base = span + order - 1;

    if nurbsknots[base - 1] == nurbsknots[base] {
        row[if t <= nurbsknots[base] {
            span
        } else {
            span + order - 1
        }] = 1.0;

        return row;
    }

    let basis = nurbsknot::eval_basis(order, nurbsknots, span, t);

    for j in 0..order {
        if span + j < cv_count {
            row[span + j] = basis[j];
        }
    }

    row
}

/// Solves a x = b by Gaussian elimination with partial pivoting, one right-hand side per column of b.
fn solve_linear(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    let n = a.len();
    let dim = b[0].len();

    for col in 0..n {
        let mut max_row = col;

        for row in (col + 1)..n {
            if a[row][col].abs() > a[max_row][col].abs() {
                max_row = row;
            }
        }

        if a[max_row][col].abs() < 1e-14 {
            continue;
        }

        a.swap(col, max_row);
        b.swap(col, max_row);

        for row in (col + 1)..n {
            let factor = a[row][col] / a[col][col];

            for c in col..n {
                a[row][c] -= factor * a[col][c];
            }

            for d in 0..dim {
                b[row][d] -= factor * b[col][d];
            }
        }
    }

    let mut x = vec![vec![0.0; dim]; n];

    for row in (0..n).rev() {
        for d in 0..dim {
            x[row][d] = b[row][d];

            for c in (row + 1)..n {
                x[row][d] -= a[row][c] * x[c][d];
            }

            if a[row][row].abs() > 1e-14 {
                x[row][d] /= a[row][row];
            }
        }
    }

    x
}

// ═══════════════════════════════════════════════════════════════════════════
// Sweep helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Point at fraction s from a to b.
fn lerp_point(a: &Point, b: &Point, s: f64) -> Point {
    a + (b - a) * s
}

/// Vector at fraction s from a to b.
fn lerp_vector(a: &Vector, b: &Vector, s: f64) -> Vector {
    a + (b - a) * s
}

/// World to the profile frame: centroid origin, x toward the start point, z the profile normal.
fn profile_to_xy(profile: &NurbsCurve) -> Xform {
    let mut centroid = Vector::new(0.0, 0.0, 0.0);

    for i in 0..profile.cv_count() {
        centroid += profile.get_cv(i).unwrap_or_default() - Point::new(0.0, 0.0, 0.0);
    }

    let origin = Point::new(0.0, 0.0, 0.0) + centroid / profile.cv_count() as f64;
    let (t0, t1) = profile.domain();
    let pa = profile.point_at(t0);
    let pb = profile.point_at(t0 + (t1 - t0) / 3.0);
    let pc = profile.point_at(t0 + 2.0 * (t1 - t0) / 3.0);
    let mut normal = (&pb - &pa).cross(&(&pc - &pa));

    if !normal.normalize_self() {
        normal = Vector::new(1.0, 0.0, 0.0);
    }

    let mut x_axis = &pa - &origin;

    if !x_axis.normalize_self() {
        x_axis = Vector::new(0.0, 1.0, 0.0);
    }

    x_axis -= &normal * x_axis.dot(&normal);

    if !x_axis.normalize_self() {
        x_axis = Vector::new(0.0, 1.0, 0.0);
    }

    Xform::world_to_frame(&origin, &x_axis, &normal.cross(&x_axis), &normal)
}

/// Frame of a sweep shape: start point origin, x along the chord, z across it.
fn shape_plane(shape: &NurbsCurve) -> Plane {
    let start = shape.point_at_start();
    let mut dir = &shape.point_at_end() - &start;

    if !dir.normalize_self() {
        dir = Vector::new(1.0, 0.0, 0.0);
    }

    let mut side = dir.cross(&Vector::new(0.0, 0.0, 1.0));

    if side.magnitude() < 1e-10 {
        side = dir.cross(&Vector::new(0.0, 1.0, 0.0));
    }

    let up = side.cross(&dir);

    Plane::new(start, dir, up)
}

/// Chord length of a shape, 1 when degenerate.
fn shape_width(shape: &NurbsCurve) -> f64 {
    let width = shape.point_at_start().distance(&shape.point_at_end(), None);

    if width < 1e-14 {
        1.0
    } else {
        width
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Curves ordered head to tail, reversed where needed; empty when they do not close a loop.
fn chain_curves(input: &[NurbsCurve]) -> Vec<NurbsCurve> {
    let tol = 1e-6;
    let mut chain = vec![input[0].duplicate()];
    let mut used = vec![false; input.len()];
    used[0] = true;

    for _step in 1..input.len() {
        let tail = chain[chain.len() - 1].point_at_end();
        let mut found = false;

        for i in 0..input.len() {
            if found || used[i] {
                continue;
            }

            let mut next = input[i].duplicate();

            if next.point_at_start().distance(&tail, None) >= tol
                && next.point_at_end().distance(&tail, None) < tol
            {
                next.reverse();
            }

            if next.point_at_start().distance(&tail, None) >= tol {
                continue;
            }

            chain.push(next);
            used[i] = true;
            found = true;
        }

        if !found {
            return Vec::new();
        }
    }

    if chain[chain.len() - 1]
        .point_at_end()
        .distance(&chain[0].point_at_start(), None)
        > tol
    {
        return Vec::new();
    }

    chain
}

/// Greville abcissae mapped to [0, 1].
fn normalized_greville(curve: &NurbsCurve) -> Vec<f64> {
    let mut grev = curve.get_greville_abcissae();
    let (t0, t1) = curve.domain();

    for g in grev.iter_mut() {
        *g = if t1 > t0 { (*g - t0) / (t1 - t0) } else { 0.0 };
    }

    grev
}

/// Factory for primitive meshes, NURBS curves and NURBS surfaces.
pub struct Primitives;

impl Primitives {
    // ═══════════════════════════════════════════════════════════════════════════
    // Mesh primitives
    // ═══════════════════════════════════════════════════════════════════════════

    /// Arrow mesh along a line: cylinder body over 80% of the length, cone head of 1.5x radius over 20%.
    pub fn arrow_mesh(line: &Line, radius: f64) -> Mesh {
        let start = line.start();
        let axis = line.to_vector();
        let length = line.length();
        let body = &Self::line_frame(line, &(&start + &axis * 0.4))
            * &Xform::scale_xyz(radius * 2.0, radius * 2.0, length * 0.8);
        let head = &Self::line_frame(line, &(&start + &axis * 0.9))
            * &Xform::scale_xyz(radius * 3.0, radius * 3.0, length * 0.2);
        let mut mesh = Mesh::new();
        Self::add_geometry(&mut mesh, &Self::unit_cylinder_geometry(), &body);
        Self::add_geometry(&mut mesh, &Self::unit_cone_geometry(), &head);

        mesh
    }

    /// Ten-sided cylinder mesh along a line.
    pub fn cylinder_mesh(line: &Line, radius: f64) -> Mesh {
        let start = line.start();
        let axis = line.to_vector();
        let xform = &Self::line_frame(line, &(&start + &axis * 0.5))
            * &Xform::scale_xyz(radius * 2.0, radius * 2.0, line.length());
        let mut mesh = Mesh::new();
        Self::add_geometry(&mut mesh, &Self::unit_cylinder_geometry(), &xform);

        mesh
    }

    /// Ten-sided cylinder mesh with hemispherical caps along a line.
    pub fn capsule_mesh(line: &Line, radius: f64) -> Mesh {
        let mut mesh = Mesh::new();
        Self::add_geometry(
            &mut mesh,
            &Self::capsule_geometry(line.length(), radius),
            &Self::line_frame(line, &line.start()),
        );

        mesh
    }

    /// One capsule mesh per edge, colored by mesh.linecolors[i].
    pub fn edge_pipes(mesh: &Mesh, radius: f64) -> Vec<Mesh> {
        let edges = mesh.edges();
        let colors = mesh.get_linecolors();
        let count = edges.len().min(colors.len());
        let mut pipes = Vec::new();

        for i in 0..count {
            let (u, v) = edges[i];
            let mut pipe = Self::capsule_mesh(
                &Line::from_points(&mesh.vertex[&u].position(), &mesh.vertex[&v].position()),
                radius,
            );

            pipe.set_facecolors(vec![colors[i].clone(); pipe.number_of_faces()]);
            pipes.push(pipe);
        }

        pipes
    }

    /// Tetrahedron mesh (4 triangles) with the given edge length.
    pub fn tetrahedron(edge: f64) -> Mesh {
        let a = edge / 2.0;
        let h = edge * (2.0_f64 / 3.0).sqrt();
        let r = edge / 3.0_f64.sqrt();
        let z0 = -h / 4.0;
        let z1 = 3.0 * h / 4.0;
        let faces = vec![
            vec![
                Point::new(a, -r / 2.0, z0),
                Point::new(-a, -r / 2.0, z0),
                Point::new(0.0, r, z0),
            ],
            vec![
                Point::new(0.0, 0.0, z1),
                Point::new(-a, -r / 2.0, z0),
                Point::new(a, -r / 2.0, z0),
            ],
            vec![
                Point::new(0.0, 0.0, z1),
                Point::new(0.0, r, z0),
                Point::new(-a, -r / 2.0, z0),
            ],
            vec![
                Point::new(0.0, 0.0, z1),
                Point::new(a, -r / 2.0, z0),
                Point::new(0.0, r, z0),
            ],
        ];

        Mesh::from_polylines(faces, Some(1e-10))
    }

    /// Cube mesh (6 quads) with the given edge length.
    pub fn cube(edge: f64) -> Mesh {
        let a = edge / 2.0;
        let v0 = Point::new(-a, -a, -a);
        let v1 = Point::new(a, -a, -a);
        let v2 = Point::new(a, a, -a);
        let v3 = Point::new(-a, a, -a);
        let v4 = Point::new(-a, -a, a);
        let v5 = Point::new(a, -a, a);
        let v6 = Point::new(a, a, a);
        let v7 = Point::new(-a, a, a);
        let faces = vec![
            vec![v3.clone(), v2.clone(), v1.clone(), v0.clone()],
            vec![v4.clone(), v5.clone(), v6.clone(), v7.clone()],
            vec![v0.clone(), v1.clone(), v5.clone(), v4.clone()],
            vec![v2.clone(), v3.clone(), v7.clone(), v6.clone()],
            vec![v0.clone(), v4.clone(), v7.clone(), v3.clone()],
            vec![v1.clone(), v2.clone(), v6.clone(), v5.clone()],
        ];

        Mesh::from_polylines(faces, Some(1e-10))
    }

    /// Octahedron mesh (8 triangles) with the given edge length.
    pub fn octahedron(edge: f64) -> Mesh {
        let a = edge / 2.0_f64.sqrt();
        let px = Point::new(a, 0.0, 0.0);
        let nx = Point::new(-a, 0.0, 0.0);
        let py = Point::new(0.0, a, 0.0);
        let ny = Point::new(0.0, -a, 0.0);
        let pz = Point::new(0.0, 0.0, a);
        let nz = Point::new(0.0, 0.0, -a);
        let faces = vec![
            vec![pz.clone(), px.clone(), py.clone()],
            vec![pz.clone(), py.clone(), nx.clone()],
            vec![pz.clone(), nx.clone(), ny.clone()],
            vec![pz.clone(), ny.clone(), px.clone()],
            vec![nz.clone(), py.clone(), px.clone()],
            vec![nz.clone(), nx.clone(), py.clone()],
            vec![nz.clone(), ny.clone(), nx.clone()],
            vec![nz.clone(), px.clone(), ny.clone()],
        ];

        Mesh::from_polylines(faces, Some(1e-10))
    }

    /// Icosahedron mesh (20 triangles) with the given edge length.
    pub fn icosahedron(edge: f64) -> Mesh {
        let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let s = edge / 2.0;
        let sp = s * phi;
        let verts = [
            Point::new(-s, sp, 0.0),
            Point::new(s, sp, 0.0),
            Point::new(-s, -sp, 0.0),
            Point::new(s, -sp, 0.0),
            Point::new(0.0, -s, sp),
            Point::new(0.0, s, sp),
            Point::new(0.0, -s, -sp),
            Point::new(0.0, s, -sp),
            Point::new(sp, 0.0, -s),
            Point::new(sp, 0.0, s),
            Point::new(-sp, 0.0, -s),
            Point::new(-sp, 0.0, s),
        ];
        let idx: [[usize; 3]; 20] = [
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        let mut faces = Vec::new();

        for f in &idx {
            faces.push(vec![
                verts[f[0]].clone(),
                verts[f[1]].clone(),
                verts[f[2]].clone(),
            ]);
        }

        Mesh::from_polylines(faces, Some(1e-10))
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Curve primitives
    // ═══════════════════════════════════════════════════════════════════════════

    /// Full circle as a rational quadratic NURBS (9 CVs).
    pub fn circle(cx: f64, cy: f64, cz: f64, radius: f64) -> NurbsCurve {
        Self::ellipse(cx, cy, cz, radius, radius)
    }

    /// Full ellipse as a rational quadratic NURBS (9 CVs).
    pub fn ellipse(cx: f64, cy: f64, cz: f64, major_radius: f64, minor_radius: f64) -> NurbsCurve {
        let mut curve = NurbsCurve::new(3, true, 3, 9);

        for i in 0..10 {
            curve.set_nurbsknot(i, CIRCLE_NURBSKNOTS[i]);
        }

        for i in 0..9 {
            let w = CIRCLE_WEIGHTS[i];
            let px = cx + major_radius * CIRCLE_X[i];
            let py = cy + minor_radius * CIRCLE_Y[i];
            curve.set_cv_4d(i, px * w, py * w, cz * w, w);
        }

        curve
    }

    /// Circular arc from start through the arc midpoint to end as a rational quadratic NURBS; a line when collinear.
    pub fn arc(start: &Point, mid: &Point, end: &Point) -> NurbsCurve {
        let chord = end - start;
        let chord_mid = start + &chord * 0.5;
        let sagitta = mid - &chord_mid;

        if chord.cross(&sagitta).magnitude() < Tolerance::ZERO_TOLERANCE {
            return NurbsCurve::create(false, 1, &[start.clone(), end.clone()]);
        }

        let h = chord.magnitude() * 0.5;
        let s = sagitta.magnitude();
        let radius = (h * h + s * s) / (2.0 * s);
        let mut w = (radius - s) / radius;

        if w.abs() < Tolerance::ZERO_TOLERANCE {
            w = Tolerance::ZERO_TOLERANCE;
        }

        let mut curve = NurbsCurve::new(3, true, 3, 3);
        curve.m_nurbsknot = vec![0.0, 0.0, 1.0, 1.0];
        curve.set_cv_4d(0, start[0], start[1], start[2], 1.0);
        let weighted = &chord_mid * w + sagitta;
        curve.set_cv_4d(1, weighted[0], weighted[1], weighted[2], w);
        curve.set_cv_4d(2, end[0], end[1], end[2], 1.0);

        curve
    }

    /// Parabola through three points with p1 as the apex, as a quadratic NURBS.
    pub fn parabola(p0: &Point, p1: &Point, p2: &Point) -> NurbsCurve {
        let mut curve = NurbsCurve::new(3, false, 3, 3);
        curve.m_nurbsknot = vec![0.0, 0.0, 1.0, 1.0];
        curve.set_cv(0, p0);
        curve.set_cv(
            1,
            &Point::new(
                2.0 * p1[0] - (p0[0] + p2[0]) / 2.0,
                2.0 * p1[1] - (p0[1] + p2[1]) / 2.0,
                2.0 * p1[2] - (p0[2] + p2[2]) / 2.0,
            ),
        );
        curve.set_cv(2, p2);

        curve
    }

    /// Hyperbola x = a cosh(t), y = b sinh(t) for t in [-extent, extent] as a cubic NURBS through 9 points.
    pub fn hyperbola(center: &Point, a: f64, b: f64, extent: f64) -> NurbsCurve {
        let segments = 8;
        let mut points = Vec::new();

        for i in 0..=segments {
            let t = -extent + 2.0 * extent * i as f64 / segments as f64;
            points.push(Point::new(
                center[0] + a * t.cosh(),
                center[1] + b * t.sinh(),
                center[2],
            ));
        }

        let mut curve = NurbsCurve::default();

        if !curve.create_clamped_uniform(3, 4, &points, 1.0) {
            return NurbsCurve::default();
        }

        curve
    }

    /// Helix with linearly varying radius as a cubic NURBS, 8 points per turn.
    pub fn spiral(start_radius: f64, end_radius: f64, pitch: f64, turns: f64) -> NurbsCurve {
        let segments = ((turns * 8.0) as usize).max(4);
        let mut points = Vec::new();

        for i in 0..=segments {
            let t = i as f64 / segments as f64;
            let angle = t * turns * 2.0 * PI;
            let r = start_radius + t * (end_radius - start_radius);
            points.push(Point::new(
                r * angle.cos(),
                r * angle.sin(),
                t * turns * pitch,
            ));
        }

        let mut curve = NurbsCurve::default();

        if !curve.create_clamped_uniform(3, 4, &points, 1.0) {
            return NurbsCurve::default();
        }

        curve
    }

    /// Interpolated cubic NURBS through points.
    pub fn create_interpolated(
        points: &[Point],
        parameterization: CurveNurbsKnotStyle,
        end_condition: CurveInterpStyle,
    ) -> NurbsCurve {
        NurbsCurve::create_interpolated(points, parameterization, end_condition)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Surface primitives
    // ═══════════════════════════════════════════════════════════════════════════

    /// Rational cylinder surface of degree 2x1 around the z axis through (cx, cy, cz).
    pub fn cylinder_surface(cx: f64, cy: f64, cz: f64, radius: f64, height: f64) -> NurbsSurface {
        let mut srf = NurbsSurface::new(3, true, 3, 2, 9, 2);

        for i in 0..10 {
            srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);
        }

        set_circle_row(&mut srf, 0, cx, cy, cz, radius, 1.0);
        set_circle_row(&mut srf, 1, cx, cy, cz + height, radius, 1.0);

        srf
    }

    /// Rational cone surface of degree 2x1 with the apex at cz + height.
    pub fn cone_surface(cx: f64, cy: f64, cz: f64, radius: f64, height: f64) -> NurbsSurface {
        let mut srf = NurbsSurface::new(3, true, 3, 2, 9, 2);

        for i in 0..10 {
            srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);
        }

        set_circle_row(&mut srf, 0, cx, cy, cz, radius, 1.0);
        set_circle_row(&mut srf, 1, cx, cy, cz + height, 0.0, 1.0);

        srf
    }

    /// Rational torus surface of degree 2x2.
    pub fn torus_surface(
        cx: f64,
        cy: f64,
        cz: f64,
        major_radius: f64,
        minor_radius: f64,
    ) -> NurbsSurface {
        let mut srf = NurbsSurface::new(3, true, 3, 3, 9, 9);

        for i in 0..10 {
            srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);
            srf.set_nurbsknot(1, i, CIRCLE_NURBSKNOTS[i]);
        }

        for j in 0..9 {
            set_circle_row(
                &mut srf,
                j,
                cx,
                cy,
                cz + minor_radius * CIRCLE_Y[j],
                major_radius + minor_radius * CIRCLE_X[j],
                CIRCLE_WEIGHTS[j],
            );
        }

        srf
    }

    /// Rational sphere surface of degree 2x2 with poles on the z axis.
    pub fn sphere_surface(cx: f64, cy: f64, cz: f64, radius: f64) -> NurbsSurface {
        let lat_r = [0.0, 1.0, 1.0, 1.0, 0.0];
        let lat_z = [-1.0, -1.0, 0.0, 1.0, 1.0];
        let lat_w = [1.0, CIRCLE_W, 1.0, CIRCLE_W, 1.0];
        let v_nurbsknots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0];
        let mut srf = NurbsSurface::new(3, true, 3, 3, 9, 5);

        for i in 0..10 {
            srf.set_nurbsknot(0, i, CIRCLE_NURBSKNOTS[i]);
        }

        for i in 0..6 {
            srf.set_nurbsknot(1, i, v_nurbsknots[i]);
        }

        for j in 0..5 {
            set_circle_row(
                &mut srf,
                j,
                cx,
                cy,
                cz + radius * lat_z[j],
                radius * lat_r[j],
                lat_w[j],
            );
        }

        srf
    }

    /// Sphere as 6 rational biquadratic patches projected from the cube faces.
    pub fn quad_sphere(cx: f64, cy: f64, cz: f64, radius: f64) -> Vec<NurbsSurface> {
        let a = radius / 3.0_f64.sqrt();
        let e = radius * 3.0_f64.sqrt() / 2.0;
        let wk = (2.0_f64 / 3.0).sqrt();
        let wc = (-72.0 - 32.0 * 6.0_f64.sqrt() + 48.0 * 3.0_f64.sqrt() + 56.0 * 2.0_f64.sqrt())
            / (48.0 * (1.0 + (2.0_f64 / 3.0).sqrt() - 1.0 / 3.0_f64.sqrt() - 1.0 / 2.0_f64.sqrt()));
        let k =
            radius * (1.0 - 1.0 / 3.0_f64.sqrt() + 2.0 * (2.0_f64 / 3.0).sqrt() - 2.0_f64.sqrt());
        let h = radius + k / wc;
        let zf: [[[f64; 4]; 3]; 3] = [
            [[-a, -a, a, 1.0], [-e, 0.0, e, wk], [-a, a, a, 1.0]],
            [[0.0, -e, e, wk], [0.0, 0.0, h, wc], [0.0, e, e, wk]],
            [[a, -a, a, 1.0], [e, 0.0, e, wk], [a, a, a, 1.0]],
        ];
        let rot: [[[f64; 3]; 3]; 6] = [
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
            [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        ];
        let mut faces = Vec::new();

        for f in 0..6 {
            let mut srf = NurbsSurface::new(3, true, 3, 3, 3, 3);

            for i in 0..3 {
                for j in 0..3 {
                    let p = zf[i][j];
                    let rx = rot[f][0][0] * p[0] + rot[f][0][1] * p[1] + rot[f][0][2] * p[2] + cx;
                    let ry = rot[f][1][0] * p[0] + rot[f][1][1] * p[1] + rot[f][1][2] * p[2] + cy;
                    let rz = rot[f][2][0] * p[0] + rot[f][2][1] * p[1] + rot[f][2][2] * p[2] + cz;
                    srf.set_cv_4d(i, j, rx * p[3], ry * p[3], rz * p[3], p[3]);
                }
            }

            faces.push(srf);
        }

        faces
    }

    /// Tileable egg-crate surface z = amplitude sin(2 pi x / size) sin(2 pi y / size) as a 13x13 cubic NURBS.
    pub fn wave_surface(size: f64, amplitude: f64) -> NurbsSurface {
        let n = 13;
        let mut pts = Vec::new();

        for i in 0..n {
            let u = i as f64 / (n - 1) as f64;

            for j in 0..n {
                let v = j as f64 / (n - 1) as f64;
                pts.push(Point::new(
                    size * u,
                    size * v,
                    amplitude * (2.0 * PI * u).sin() * (2.0 * PI * v).sin(),
                ));
            }
        }

        NurbsSurface::create(false, false, 3, 3, n, n, &pts).unwrap_or_default()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Surface factories
    // ═══════════════════════════════════════════════════════════════════════════

    /// Ruled surface between two curves.
    pub fn create_ruled(curve_a: &NurbsCurve, curve_b: &NurbsCurve) -> NurbsSurface {
        if !curve_a.is_valid() || !curve_b.is_valid() {
            return NurbsSurface::default();
        }

        let mut curves = vec![curve_a.duplicate(), curve_b.duplicate()];
        curves[0].set_domain(0.0, 1.0);
        curves[1].set_domain(0.0, 1.0);
        make_curves_compatible(&mut curves);

        let cv_count_u = curves[0].cv_count();
        let is_rat = curves[0].is_rational();
        let mut surface = NurbsSurface::new(3, is_rat, curves[0].order(), 2, cv_count_u, 2);

        if !surface.is_valid() {
            return NurbsSurface::default();
        }

        for i in 0..surface.nurbsknot_count(0) {
            surface.set_nurbsknot(0, i, curves[0].nurbsknot(i).unwrap_or_default());
        }

        for i in 0..cv_count_u {
            for j in 0..2 {
                if is_rat {
                    let (x, y, z, w) = curves[j].get_cv_4d(i).unwrap_or_default();
                    surface.set_cv_4d(i, j, x, y, z, w);
                } else {
                    surface.set_cv(i, j, &curves[j].get_cv(i).unwrap_or_default());
                }
            }
        }

        surface
    }

    /// Extrusion of a curve along a direction.
    pub fn create_extrusion(curve: &NurbsCurve, direction: &Vector) -> NurbsSurface {
        if !curve.is_valid() {
            return NurbsSurface::default();
        }

        let mut translated = curve.duplicate();
        translated.transform(&Xform::translation(
            direction[0],
            direction[1],
            direction[2],
        ));

        Self::create_ruled(curve, &translated)
    }

    /// Bilinear planar patch containing a closed boundary curve.
    pub fn create_planar(boundary: &NurbsCurve) -> NurbsSurface {
        if !boundary.is_valid() {
            return NurbsSurface::default();
        }

        let mut pts = Vec::new();

        for i in 0..boundary.cv_count() {
            pts.push(boundary.get_cv(i).unwrap_or_default());
        }

        if pts.len() >= 2 && pts[0].distance(&pts[pts.len() - 1], None) < 1e-10 {
            pts.pop();
        }

        if pts.len() < 3 {
            return NurbsSurface::default();
        }

        if boundary.degree() <= 1 && pts.len() == 3 {
            return bilinear_patch(&pts[0], &pts[1], &pts[0], &pts[2]);
        }

        if boundary.degree() <= 1 && pts.len() == 4 {
            return bilinear_patch(&pts[0], &pts[1], &pts[3], &pts[2]);
        }

        if boundary.degree() <= 1 {
            let mut normal = (&pts[1] - &pts[0]).cross(&(&pts[2] - &pts[0]));

            if !normal.normalize_self() {
                return NurbsSurface::default();
            }

            let x_axis = longest_edge_dir(&pts);
            let mut y_axis = normal.cross(&x_axis);

            if !y_axis.normalize_self() {
                return NurbsSurface::default();
            }

            return bounded_patch(&pts, &pts[0], &x_axis, &y_axis);
        }

        let (samples, _params) = boundary.divide_by_count(20.max(boundary.cv_count() * 4), true);
        let plane = Plane::from_points_pca(samples.clone());

        if plane.z_axis().magnitude() < 1e-10 {
            return NurbsSurface::default();
        }

        bounded_patch(&samples, &plane.origin(), &plane.x_axis(), &plane.y_axis())
    }

    /// Loft through section curves, interpolating them in v.
    pub fn create_loft(input_curves: &[NurbsCurve], degree_v: usize) -> NurbsSurface {
        if input_curves.len() < 2 {
            return NurbsSurface::default();
        }

        for c in input_curves {
            if !c.is_valid() {
                return NurbsSurface::default();
            }
        }

        let mut curves = Vec::new();

        for c in input_curves {
            curves.push(c.duplicate());
        }

        make_curves_compatible(&mut curves);

        let n = curves.len();
        let cv_count_u = curves[0].cv_count();
        let is_rat = curves[0].is_rational();
        let order_v = degree_v.clamp(1, n - 1) + 1;
        let v_params = loft_section_params(&curves);
        let nurbsknots_v = loft_nurbsknots(&v_params, order_v);
        let mut surface = NurbsSurface::new(3, is_rat, curves[0].order(), order_v, cv_count_u, n);

        if !surface.is_valid() {
            return NurbsSurface::default();
        }

        for i in 0..surface.nurbsknot_count(0) {
            surface.set_nurbsknot(0, i, curves[0].nurbsknot(i).unwrap_or_default());
        }

        for i in 0..surface.nurbsknot_count(1) {
            surface.set_nurbsknot(1, i, nurbsknots_v[i]);
        }

        let mut basis = Vec::new();

        for k in 0..n {
            basis.push(loft_basis_row(&nurbsknots_v, order_v, n, v_params[k]));
        }

        let dim = if is_rat { 4 } else { 3 };

        for i in 0..cv_count_u {
            let mut rhs = vec![vec![0.0; dim]; n];

            for k in 0..n {
                if is_rat {
                    let (x, y, z, w) = curves[k].get_cv_4d(i).unwrap_or_default();
                    rhs[k] = vec![x, y, z, w];
                } else {
                    let p = curves[k].get_cv(i).unwrap_or_default();
                    rhs[k] = vec![p[0], p[1], p[2]];
                }
            }

            let q = solve_linear(&basis, &rhs);

            for j in 0..n {
                if is_rat {
                    surface.set_cv_4d(i, j, q[j][0], q[j][1], q[j][2], q[j][3]);
                } else {
                    surface.set_cv(i, j, &Point::new(q[j][0], q[j][1], q[j][2]));
                }
            }
        }

        surface
    }

    /// Surface of revolution of a profile around an axis.
    pub fn create_revolve(
        profile: &NurbsCurve,
        axis_origin: &Point,
        axis_direction: &Vector,
        angle: f64,
    ) -> NurbsSurface {
        if !profile.is_valid() {
            return NurbsSurface::default();
        }

        let mut axis = axis_direction.clone();

        if !axis.normalize_self() {
            return NurbsSurface::default();
        }

        let angle = angle.abs().min(2.0 * PI);

        if angle < 1e-14 {
            return NurbsSurface::default();
        }

        let mut n_arcs = 4;

        if angle <= PI / 2.0 + 1e-10 {
            n_arcs = 1;
        } else if angle <= PI + 1e-10 {
            n_arcs = 2;
        } else if angle <= 3.0 * PI / 2.0 + 1e-10 {
            n_arcs = 3;
        }

        let d_theta = angle / n_arcs as f64;
        let w_mid = (d_theta / 2.0).cos();
        let n_u = 2 * n_arcs + 1;
        let cv_count_v = profile.cv_count();
        let mut surface = NurbsSurface::new(3, true, 3, profile.order(), n_u, cv_count_v);

        if !surface.is_valid() {
            return NurbsSurface::default();
        }

        for i in 0..surface.nurbsknot_count(0) {
            surface.set_nurbsknot(
                0,
                i,
                if i / 2 == n_arcs {
                    angle
                } else {
                    (i / 2) as f64 * d_theta
                },
            );
        }

        for i in 0..surface.nurbsknot_count(1) {
            surface.set_nurbsknot(1, i, profile.nurbsknot(i).unwrap_or_default());
        }

        for j in 0..cv_count_v {
            let p = profile.get_cv(j).unwrap_or_default();
            let profile_w = if profile.is_rational() {
                profile.weight(j)
            } else {
                1.0
            };
            let center = axis_origin + &axis * (&p - axis_origin).dot(&axis);
            let mut x_local = &p - &center;
            let r = x_local.magnitude();

            if r > 1e-14 {
                x_local /= r;
            }

            let y_local = axis.cross(&x_local);

            for i in 0..n_u {
                let shoulder = i % 2 == 1;
                let theta = (i / 2) as f64 * d_theta + if shoulder { d_theta / 2.0 } else { 0.0 };
                let w = if shoulder { w_mid } else { 1.0 } * profile_w;
                let q = &center
                    + (&x_local * theta.cos() + &y_local * theta.sin())
                        * if shoulder { r / w_mid } else { r };
                surface.set_cv_4d(i, j, q[0] * w, q[1] * w, q[2] * w, w);
            }
        }

        surface
    }

    /// Sweep of a closed profile along one rail.
    pub fn create_sweep1(rail: &NurbsCurve, profile: &NurbsCurve) -> NurbsSurface {
        if !rail.is_valid() || !profile.is_valid() {
            return NurbsSurface::default();
        }

        let count = (rail.span_count() * 2 + 1).clamp(5, 200);
        let frames = rail.get_perpendicular_planes(count);

        if frames.is_empty() {
            return NurbsSurface::default();
        }

        let to_xy = profile_to_xy(profile);
        let mut sections = Vec::new();

        for frame in &frames {
            let mut section = profile.duplicate();
            section.transform(&(&Xform::to_frame(frame) * &to_xy));
            sections.push(section);
        }

        Self::create_loft(&sections, 3.min(sections.len() - 1))
    }

    /// Sweep of shape curves between two rails.
    pub fn create_sweep2(
        rail1: &NurbsCurve,
        rail2: &NurbsCurve,
        shapes: &[NurbsCurve],
    ) -> NurbsSurface {
        if !rail1.is_valid() || !rail2.is_valid() || shapes.is_empty() {
            return NurbsSurface::default();
        }

        for shape in shapes {
            if !shape.is_valid() {
                return NurbsSurface::default();
            }
        }

        let mut compat = Vec::new();

        for shape in shapes {
            compat.push(shape.duplicate());
        }

        make_curves_compatible(&mut compat);

        let n_shapes = compat.len();
        let mut planes = Vec::new();
        let mut widths = Vec::new();

        for shape in &compat {
            planes.push(shape_plane(shape));
            widths.push(shape_width(shape));
        }

        let count = (rail1.span_count().max(rail2.span_count()) * 2 + 1).clamp(5, 200);
        let (pts1, _params1) = rail1.divide_by_count(count + 1, true);
        let (pts2, _params2) = rail2.divide_by_count(count + 1, true);
        let frames = rail1.get_perpendicular_planes(count);

        if frames.is_empty() {
            return NurbsSurface::default();
        }

        let mut sections = Vec::new();

        for i in 0..frames.len().min(pts1.len()).min(pts2.len()) {
            let t = if frames.len() <= 1 {
                0.0
            } else {
                i as f64 / (frames.len() - 1) as f64
            };
            let j = if n_shapes == 1 {
                0
            } else {
                ((t * (n_shapes - 1) as f64) as usize).min(n_shapes - 2)
            };
            let j1 = if n_shapes == 1 { 0 } else { j + 1 };
            let s = if n_shapes == 1 {
                0.0
            } else {
                (t * (n_shapes - 1) as f64 - j as f64).clamp(0.0, 1.0)
            };
            let mut section = compat[j].duplicate();

            for c in 0..section.cv_count() {
                section.set_cv(
                    c,
                    &lerp_point(
                        &compat[j].get_cv(c).unwrap_or_default(),
                        &compat[j1].get_cv(c).unwrap_or_default(),
                        s,
                    ),
                );
            }

            let source = Plane::new(
                lerp_point(&planes[j].origin(), &planes[j1].origin(), s),
                lerp_vector(&planes[j].x_axis(), &planes[j1].x_axis(), s),
                lerp_vector(&planes[j].y_axis(), &planes[j1].y_axis(), s),
            );
            let width = widths[j] * (1.0 - s) + widths[j1] * s;
            let p1 = pts1[i].clone();
            let mut x_dir = &pts2[i] - &p1;
            let rail_dist = x_dir.magnitude();

            if !x_dir.normalize_self() {
                x_dir = frames[i].x_axis();
            }

            let mut y_dir = frames[i].z_axis().cross(&x_dir);

            if !y_dir.normalize_self() {
                y_dir = frames[i].y_axis();
            }

            if y_dir.dot(&source.y_axis()) < 0.0 {
                y_dir = -y_dir;
            }

            let scale = if rail_dist > 1e-14 && width > 1e-14 {
                rail_dist / width
            } else {
                1.0
            };
            let target = Plane::new(p1, x_dir, y_dir);
            let to_source = Xform::world_to_frame(
                &source.origin(),
                &source.x_axis(),
                &source.y_axis(),
                &source.z_axis(),
            );
            section.transform(
                &(&(&Xform::to_frame(&target) * &Xform::scale_xyz(scale, scale, scale))
                    * &to_source),
            );
            sections.push(section);
        }

        Self::create_loft(&sections, 3.min(sections.len() - 1))
    }

    /// Coons patch from four boundary curves in any order and direction.
    pub fn create_edge(
        c0: &NurbsCurve,
        c1: &NurbsCurve,
        c2: &NurbsCurve,
        c3: &NurbsCurve,
    ) -> NurbsSurface {
        if !c0.is_valid() || !c1.is_valid() || !c2.is_valid() || !c3.is_valid() {
            return NurbsSurface::default();
        }

        let chain = chain_curves(&[
            c0.duplicate(),
            c1.duplicate(),
            c2.duplicate(),
            c3.duplicate(),
        ]);

        if chain.is_empty() {
            return NurbsSurface::default();
        }

        let mut v_pair = vec![chain[0].duplicate(), chain[2].duplicate()];
        v_pair[1].reverse();
        make_curves_compatible(&mut v_pair);

        let mut u_pair = vec![chain[3].duplicate(), chain[1].duplicate()];
        u_pair[0].reverse();
        make_curves_compatible(&mut u_pair);

        let south = &v_pair[0];
        let north = &v_pair[1];
        let west = &u_pair[0];
        let east = &u_pair[1];
        let cv_count_u = west.cv_count();
        let cv_count_v = south.cv_count();
        let mut surface = NurbsSurface::new(
            3,
            south.is_rational() || west.is_rational(),
            west.order(),
            south.order(),
            cv_count_u,
            cv_count_v,
        );

        if !surface.is_valid() {
            return NurbsSurface::default();
        }

        for i in 0..surface.nurbsknot_count(0) {
            surface.set_nurbsknot(0, i, west.nurbsknot(i).unwrap_or_default());
        }

        for i in 0..surface.nurbsknot_count(1) {
            surface.set_nurbsknot(1, i, south.nurbsknot(i).unwrap_or_default());
        }

        let u_grev = normalized_greville(west);
        let v_grev = normalized_greville(south);
        let c00 = south.get_cv(0).unwrap_or_default();
        let c01 = south.get_cv(cv_count_v - 1).unwrap_or_default();
        let c10 = north.get_cv(0).unwrap_or_default();
        let c11 = north.get_cv(cv_count_v - 1).unwrap_or_default();

        for i in 0..cv_count_u {
            let ui = u_grev[i];
            let wi = west.get_cv(i).unwrap_or_default();
            let ei = east.get_cv(i).unwrap_or_default();

            for j in 0..cv_count_v {
                let vj = v_grev[j];
                let sj = south.get_cv(j).unwrap_or_default();
                let nj = north.get_cv(j).unwrap_or_default();
                let mut q = [0.0; 3];

                for axis in 0..3 {
                    q[axis] = (1.0 - ui) * sj[axis]
                        + ui * nj[axis]
                        + (1.0 - vj) * wi[axis]
                        + vj * ei[axis]
                        - (1.0 - ui) * (1.0 - vj) * c00[axis]
                        - (1.0 - ui) * vj * c01[axis]
                        - ui * (1.0 - vj) * c10[axis]
                        - ui * vj * c11[axis];
                }

                surface.set_cv(i, j, &Point::new(q[0], q[1], q[2]));
            }
        }

        surface
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Surface to mesh
    // ═══════════════════════════════════════════════════════════════════════════

    /// Quad mesh sampled on a u_count x v_count grid.
    pub fn quad_mesh(surface: &NurbsSurface, u_count: usize, v_count: usize) -> Mesh {
        let mut mesh = Mesh::new();
        let grid = surface_grid(surface, u_count, v_count, &mut mesh);
        let singular_south = surface.is_singular(0);
        let singular_north = surface.is_singular(2);

        if singular_south {
            for i in 0..u_count {
                mesh.add_face(vec![grid[0][0], grid[i + 1][1], grid[i][1]], None);
            }
        }

        if singular_north {
            for i in 0..u_count {
                mesh.add_face(
                    vec![
                        grid[0][v_count],
                        grid[i][v_count - 1],
                        grid[i + 1][v_count - 1],
                    ],
                    None,
                );
            }
        }

        let j0 = if singular_south { 1 } else { 0 };
        let j1 = if singular_north { v_count - 1 } else { v_count };

        for i in 0..u_count {
            for j in j0..j1 {
                mesh.add_face(
                    vec![
                        grid[i][j],
                        grid[i + 1][j],
                        grid[i + 1][j + 1],
                        grid[i][j + 1],
                    ],
                    None,
                );
            }
        }

        mesh
    }

    /// Diamond mesh sampled on a u_count x v_count grid.
    pub fn diamond_mesh(surface: &NurbsSurface, u_count: usize, v_count: usize) -> Mesh {
        let mut mesh = Mesh::new();
        let grid = surface_grid(surface, u_count, v_count, &mut mesh);
        let closed_u = surface.is_closed(0);
        let u_end = if closed_u { u_count - 1 } else { u_count };

        for i in 0..=u_end {
            for j in 0..=v_count {
                if (i + j) % 2 != 0 {
                    continue;
                }

                let center = grid[i][j];
                let il = if i > 0 {
                    Some(i - 1)
                } else if closed_u {
                    Some(u_count - 1)
                } else {
                    None
                };
                let left = if let Some(il) = il {
                    grid[il][j]
                } else {
                    center
                };
                let bottom = if j > 0 { grid[i][j - 1] } else { center };
                let right = if i < u_count { grid[i + 1][j] } else { center };
                let top = if j < v_count { grid[i][j + 1] } else { center };
                let face = dedup_face(&[left, bottom, right, top]);

                if face.len() >= 3 {
                    mesh.add_face(face, None);
                }
            }
        }

        mesh
    }

    /// Hexagonal mesh sampled on a u_count x v_count grid, t the split of each v cell.
    pub fn hex_mesh(surface: &NurbsSurface, u_count: usize, v_count: usize, t: f64) -> Mesh {
        let mut mesh = Mesh::new();
        let grid = surface_grid(surface, u_count, v_count, &mut mesh);
        let mid_a = surface_mid_grid(surface, u_count, v_count, t, &mut mesh);
        let mid_b = surface_mid_grid(surface, u_count, v_count, 1.0 - t, &mut mesh);
        let closed_u = surface.is_closed(0);
        let u_end = if closed_u { u_count - 1 } else { u_count };

        for i in 0..=u_end {
            for j in 0..=v_count {
                if (i + j) % 2 != 0 {
                    continue;
                }

                let center = grid[i][j];
                let il = if i > 0 {
                    Some(i - 1)
                } else if closed_u {
                    Some(u_count - 1)
                } else {
                    None
                };
                let ul = match il {
                    Some(il) if j < v_count => mid_a[il][j],
                    Some(il) => grid[il][j],
                    None => center,
                };
                let ll = match il {
                    Some(il) if j > 0 => mid_b[il][j - 1],
                    Some(il) => grid[il][j],
                    None => center,
                };
                let bt = if j > 0 { mid_a[i][j - 1] } else { center };
                let lr = if i < u_count && j > 0 {
                    mid_b[i + 1][j - 1]
                } else if i < u_count {
                    grid[i + 1][j]
                } else {
                    center
                };
                let ur = if i < u_count && j < v_count {
                    mid_a[i + 1][j]
                } else if i < u_count {
                    grid[i + 1][j]
                } else {
                    center
                };
                let tp = if j < v_count { mid_b[i][j] } else { center };
                let face = dedup_face(&[ul, ll, bt, lr, ur, tp]);

                if face.len() >= 3 {
                    mesh.add_face(face, None);
                }
            }
        }

        mesh
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mesh geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /// Ten-sided unit cylinder: radius 0.5, z from -0.5 to 0.5.
    fn unit_cylinder_geometry() -> (Vec<Point>, Vec<[usize; 3]>) {
        let n = 10;
        let mut vertices = Vec::new();
        add_ring(&mut vertices, n, 0.5, -0.5);
        add_ring(&mut vertices, n, 0.5, 0.5);

        let mut triangles = Vec::new();

        for i in 0..n {
            let next = (i + 1) % n;
            triangles.push([i, next, n + next]);
            triangles.push([i, n + next, n + i]);
        }

        (vertices, triangles)
    }

    /// Eight-sided unit cone: base radius 0.5 at z = -0.5, apex at z = 0.5.
    fn unit_cone_geometry() -> (Vec<Point>, Vec<[usize; 3]>) {
        let n = 8;
        let mut vertices = vec![Point::new(0.0, 0.0, 0.5)];
        add_ring(&mut vertices, n, 0.5, -0.5);

        let mut triangles = Vec::new();

        for i in 0..n {
            triangles.push([0, 1 + i, 1 + (i + 1) % n]);
        }

        (vertices, triangles)
    }

    /// Ten-sided capsule along z from 0 to length with hemispherical caps.
    fn capsule_geometry(length: f64, radius: f64) -> (Vec<Point>, Vec<[usize; 3]>) {
        let n = 10;
        let r_hemi = radius * (PI / 4.0).sin();
        let off = radius * (PI / 4.0).cos();
        let top = n;
        let hemi_a = 2 * n;
        let pole_a = 3 * n;
        let hemi_b = 3 * n + 1;
        let pole_b = 4 * n + 1;
        let mut vertices = Vec::new();
        add_ring(&mut vertices, n, radius, 0.0);
        add_ring(&mut vertices, n, radius, length);
        add_ring(&mut vertices, n, r_hemi, -off);
        vertices.push(Point::new(0.0, 0.0, -radius));
        add_ring(&mut vertices, n, r_hemi, length + off);
        vertices.push(Point::new(0.0, 0.0, length + radius));

        let mut triangles = Vec::new();

        for i in 0..n {
            let next = (i + 1) % n;
            triangles.push([i, next, top + next]);
            triangles.push([i, top + next, top + i]);
            triangles.push([hemi_a + i, next, i]);
            triangles.push([hemi_a + i, hemi_a + next, next]);
            triangles.push([top + i, top + next, hemi_b + next]);
            triangles.push([top + i, hemi_b + next, hemi_b + i]);
            triangles.push([pole_a, hemi_a + next, hemi_a + i]);
            triangles.push([pole_b, hemi_b + i, hemi_b + next]);
        }

        (vertices, triangles)
    }

    /// Frame at origin with z along the line.
    fn line_frame(line: &Line, origin: &Point) -> Xform {
        let mut z_axis = line.to_vector();

        if !z_axis.normalize_self() {
            z_axis = Vector::new(0.0, 0.0, 1.0);
        }

        let pole = if z_axis[2].abs() < 0.9 {
            Vector::new(0.0, 0.0, 1.0)
        } else {
            Vector::new(1.0, 0.0, 0.0)
        };
        let x_axis = pole.cross(&z_axis);

        Xform::frame_to_world(origin, &x_axis, &z_axis.cross(&x_axis), &z_axis)
    }

    /// Appends transformed geometry to a mesh.
    fn add_geometry(mesh: &mut Mesh, geometry: &(Vec<Point>, Vec<[usize; 3]>), xform: &Xform) {
        let (vertices, triangles) = geometry;
        let mut keys = Vec::new();

        for v in vertices {
            keys.push(mesh.add_vertex(v.transformed(xform), None));
        }

        for tri in triangles {
            mesh.add_face(vec![keys[tri[0]], keys[tri[1]], keys[tri[2]]], None);
        }
    }
}
