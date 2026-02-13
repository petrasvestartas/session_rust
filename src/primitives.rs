use crate::knot;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::plane::Plane;
use crate::point::Point;
use crate::tolerance::Tolerance;
use crate::vector::Vector;
use crate::xform::Xform;
use std::f64::consts::PI;

fn merge_knot_vectors(a: &[f64], b: &[f64]) -> Vec<f64> {
    let mut merged = Vec::new();
    let (mut i, mut j) = (0, 0);
    let tol = 1e-10;
    while i < a.len() && j < b.len() {
        if (a[i] - b[j]).abs() < tol {
            merged.push(a[i]);
            i += 1; j += 1;
        } else if a[i] < b[j] {
            merged.push(a[i]);
            i += 1;
        } else {
            merged.push(b[j]);
            j += 1;
        }
    }
    while i < a.len() { merged.push(a[i]); i += 1; }
    while j < b.len() { merged.push(b[j]); j += 1; }
    merged
}

fn knot_vectors_equal(a: &[f64], b: &[f64]) -> bool {
    if a.len() != b.len() { return false; }
    let tol = 1e-10;
    for i in 0..a.len() {
        if (a[i] - b[i]).abs() > tol { return false; }
    }
    true
}

fn make_curves_compatible(curves: &mut Vec<NurbsCurve>) {
    if curves.len() < 2 { return; }
    let max_deg = curves.iter().map(|c| c.degree()).max().unwrap_or(0);
    for c in curves.iter_mut() {
        if c.degree() < max_deg { c.increase_degree(max_deg); }
    }
    let any_rational = curves.iter().any(|c| c.is_rational());
    if any_rational {
        for c in curves.iter_mut() { c.make_rational(); }
    }
    let mut already_compatible = true;
    let cv0 = curves[0].cv_count();
    let knots0 = curves[0].get_knots();
    for i in 1..curves.len() {
        if curves[i].cv_count() != cv0 || !knot_vectors_equal(&curves[i].get_knots(), &knots0) {
            already_compatible = false;
            break;
        }
    }
    if already_compatible { return; }
    for c in curves.iter_mut() { c.set_domain(0.0, 1.0); }
    let mut unified = curves[0].get_knots();
    for i in 1..curves.len() {
        unified = merge_knot_vectors(&unified, &curves[i].get_knots());
    }
    let tol = 1e-10;
    for c in curves.iter_mut() {
        let cur_knots = c.get_knots();
        let mut ci = 0usize;
        for ui in 0..unified.len() {
            if ci < cur_knots.len() && (cur_knots[ci] - unified[ui]).abs() < tol {
                ci += 1;
            } else {
                c.insert_knot(unified[ui], 1);
            }
        }
    }
}

fn find_curve_intersection(crv1: &NurbsCurve, crv2: &NurbsCurve) -> (f64, f64, Point) {
    let (a1, b1) = crv1.domain();
    let (a2, b2) = crv2.domain();
    let n = 80;
    let (mut best_d2, mut t1, mut t2) = (1e30_f64, a1, a2);

    for i in 0..=n {
        let s1 = a1 + (b1 - a1) * i as f64 / n as f64;
        let p1 = crv1.point_at(s1);
        for j in 0..=n {
            let s2 = a2 + (b2 - a2) * j as f64 / n as f64;
            let p2 = crv2.point_at(s2);
            let (dx, dy, dz) = (p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]);
            let d2 = dx*dx + dy*dy + dz*dz;
            if d2 < best_d2 { best_d2 = d2; t1 = s1; t2 = s2; }
        }
    }

    let (mut dt1, mut dt2) = ((b1 - a1) / n as f64, (b2 - a2) / n as f64);
    for _ in 0..4 {
        let lo1 = a1.max(t1 - dt1); let hi1 = b1.min(t1 + dt1);
        let lo2 = a2.max(t2 - dt2); let hi2 = b2.min(t2 + dt2);
        for i in 0..=20 {
            let s1 = lo1 + (hi1 - lo1) * i as f64 / 20.0;
            let p1 = crv1.point_at(s1);
            for j in 0..=20 {
                let s2 = lo2 + (hi2 - lo2) * j as f64 / 20.0;
                let p2 = crv2.point_at(s2);
                let (dx, dy, dz) = (p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]);
                let d2 = dx*dx + dy*dy + dz*dz;
                if d2 < best_d2 { best_d2 = d2; t1 = s1; t2 = s2; }
            }
        }
        dt1 /= 20.0; dt2 /= 20.0;
    }

    let p1 = crv1.point_at(t1);
    let p2 = crv2.point_at(t2);
    let pt = Point::new((p1[0]+p2[0])/2.0, (p1[1]+p2[1])/2.0, (p1[2]+p2[2])/2.0);
    (t1, t2, pt)
}

fn make_surfaces_knot_compatible_1d(a: &mut NurbsSurface, b: &mut NurbsSurface, c: &mut NurbsSurface) {
    let dir = 0;
    let max_deg = a.degree(dir).max(b.degree(dir)).max(c.degree(dir));
    if a.degree(dir) < max_deg { a.increase_degree(dir, max_deg); }
    if b.degree(dir) < max_deg { b.increase_degree(dir, max_deg); }
    if c.degree(dir) < max_deg { c.increase_degree(dir, max_deg); }

    a.set_domain(dir, 0.0, 1.0);
    b.set_domain(dir, 0.0, 1.0);
    c.set_domain(dir, 0.0, 1.0);

    let ka = a.get_knots(dir);
    let kb = b.get_knots(dir);
    let kc = c.get_knots(dir);
    let unified = merge_knot_vectors(&merge_knot_vectors(&ka, &kb), &kc);

    fn insert_missing(srf: &mut NurbsSurface, dir: usize, uni: &[f64]) {
        let orig = srf.get_knots(dir);
        let mut ci = 0;
        for ui in 0..uni.len() {
            if ci < orig.len() && (orig[ci] - uni[ui]).abs() < 1e-10 {
                ci += 1;
            } else {
                srf.insert_knot(dir, uni[ui], 1);
            }
        }
    }
    insert_missing(a, dir, &unified);
    insert_missing(b, dir, &unified);
    insert_missing(c, dir, &unified);
}

fn cubic_interpolation_curve(points: &[Point], params: &[f64]) -> NurbsCurve {
    let n = points.len();
    if n < 2 { return NurbsCurve::new(3, false, 4, 0); }
    let dim = 3usize;
    let degree = 3.min(n - 1);
    let order = degree + 1;

    if degree < 3 {
        let cv_count = n;
        if degree == 1 {
            let mut result = NurbsCurve::new(dim, false, order, cv_count);
            for i in 0..n { result.set_knot(i, params[i]); }
            for i in 0..n { result.set_cv(i, &points[i]); }
            return result;
        }
        // degree 2
        let kn_count = cv_count + order - 2;
        let mut kn = vec![0.0; kn_count];
        for i in 0..order-1 { kn[i] = params[0]; }
        for i in kn_count-order+1..kn_count { kn[i] = params[n-1]; }
        for j in 1..=(n as i32 - order as i32).max(0) as usize {
            let mut s = 0.0;
            for i in j..j+degree { s += params[i]; }
            kn[order-2+j] = s / degree as f64;
        }
        let mut a_mat = vec![vec![0.0; n]; n];
        for k in 0..n {
            let span = knot::find_span(order, cv_count, &kn, params[k]);
            let basis = knot::eval_basis(order, &kn, span, params[k]);
            for jj in 0..order {
                if span + jj < n { a_mat[k][span + jj] = basis[jj]; }
            }
        }
        let mut cv = vec![0.0; n * dim];
        for i in 0..n { for d in 0..dim { cv[i*dim+d] = points[i][d]; } }
        for col in 0..n {
            let mut pivot = col;
            for row in col+1..n {
                if a_mat[row][col].abs() > a_mat[pivot][col].abs() { pivot = row; }
            }
            if pivot != col {
                a_mat.swap(col, pivot);
                for d in 0..dim { cv.swap(col*dim+d, pivot*dim+d); }
            }
            for row in col+1..n {
                let factor = a_mat[row][col] / a_mat[col][col];
                for jj in col..n { a_mat[row][jj] -= factor * a_mat[col][jj]; }
                for d in 0..dim { cv[row*dim+d] -= factor * cv[col*dim+d]; }
            }
        }
        for i in (0..n).rev() {
            for d in 0..dim {
                let mut s = cv[i*dim+d];
                for jj in i+1..n { s -= a_mat[i][jj] * cv[jj*dim+d]; }
                cv[i*dim+d] = s / a_mat[i][i];
            }
        }
        let mut result = NurbsCurve::new(dim, false, order, cv_count);
        for i in 0..kn_count { result.set_knot(i, kn[i]); }
        for i in 0..cv_count { result.set_cv(i, &Point::new(cv[i*3], cv[i*3+1], cv[i*3+2])); }
        return result;
    }

    // Degree 3: Bessel tangents + tridiagonal solver
    let cv_count = n + 2;
    let knots_vec = knot::build_interp_knots(&params.to_vec(), degree);
    let kc = knots_vec.len();

    let pdist = |a: &Point, b: &Point| -> f64 {
        let (dx, dy, dz) = (a[0]-b[0], a[1]-b[1], a[2]-b[2]);
        (dx*dx + dy*dy + dz*dz).sqrt()
    };

    let estimate_tangent = |i0: usize, i1: usize, i2: usize| -> Vector {
        let d01 = pdist(&points[i0], &points[i1]);
        let d21 = pdist(&points[i2], &points[i1]);
        if d01 + d21 < 1e-300 { return Vector::new(0.0,0.0,0.0); }
        let s = d01 / (d01 + d21);
        let t = 1.0 - s;
        let denom = 2.0 * s * t;
        if denom < 1e-16 {
            let dx = points[i1][0]-points[i0][0];
            let dy = points[i1][1]-points[i0][1];
            let dz = points[i1][2]-points[i0][2];
            let ln = (dx*dx+dy*dy+dz*dz).sqrt();
            return if ln > 0.0 { Vector::new(dx/ln, dy/ln, dz/ln) } else { Vector::new(0.0,0.0,0.0) };
        }
        let cvx = (-t*t*points[i0][0] + points[i1][0] - s*s*points[i2][0]) / denom;
        let cvy = (-t*t*points[i0][1] + points[i1][1] - s*s*points[i2][1]) / denom;
        let cvz = (-t*t*points[i0][2] + points[i1][2] - s*s*points[i2][2]) / denom;
        let dx = cvx - points[i0][0];
        let dy = cvy - points[i0][1];
        let dz = cvz - points[i0][2];
        let ln = (dx*dx+dy*dy+dz*dz).sqrt();
        if ln > 0.0 { Vector::new(dx/ln, dy/ln, dz/ln) } else { Vector::new(0.0,0.0,0.0) }
    };

    let (tan_start, tan_end) = if n >= 3 {
        let ts = estimate_tangent(0, 1, 2);
        let end_raw = estimate_tangent(n-1, n-2, n-3);
        (ts, Vector::new(-end_raw[0], -end_raw[1], -end_raw[2]))
    } else {
        let dx = points[1][0]-points[0][0];
        let dy = points[1][1]-points[0][1];
        let dz = points[1][2]-points[0][2];
        let ln = (dx*dx+dy*dy+dz*dz).sqrt();
        if ln > 0.0 {
            let v = Vector::new(dx/ln, dy/ln, dz/ln);
            (v.clone(), v)
        } else {
            (Vector::new(0.0,0.0,0.0), Vector::new(0.0,0.0,0.0))
        }
    };

    let d_start = pdist(&points[0], &points[1]);
    let d_end = pdist(&points[n-1], &points[n-2]);

    let mut cv = vec![0.0; cv_count * dim];
    for d in 0..dim { cv[d] = points[0][d]; }
    let s0 = d_start / 3.0;
    for d in 0..dim { cv[dim+d] = points[0][d] + s0 * tan_start[d]; }
    for i in 1..n-1 { for d in 0..dim { cv[(i+1)*dim+d] = points[i][d]; } }
    let s1 = -d_end / 3.0;
    for d in 0..dim { cv[n*dim+d] = points[n-1][d] + s1 * tan_end[d]; }
    for d in 0..dim { cv[(n+1)*dim+d] = points[n-1][d]; }

    let sys_n = n;
    let mut lower = vec![0.0; sys_n];
    let mut diag = vec![0.0; sys_n];
    let mut upper = vec![0.0; sys_n];
    let mut rhs = vec![0.0; sys_n * dim];

    diag[0] = 1.0;
    for d in 0..dim { rhs[d] = cv[dim+d]; }

    for i in 1..n-1 {
        let basis = knot::eval_basis(order, &knots_vec, i, params[i]);
        lower[i] = basis[0];
        diag[i] = basis[1];
        upper[i] = basis[2];
        for d in 0..dim { rhs[i*dim+d] = points[i][d]; }
    }

    diag[n-1] = 1.0;
    for d in 0..dim { rhs[(n-1)*dim+d] = cv[n*dim+d]; }

    let solution = knot::solve_tridiagonal(dim, &lower, &diag, &upper, &rhs).unwrap();

    for i in 0..sys_n { for d in 0..dim { cv[(i+1)*dim+d] = solution[i*dim+d]; } }

    let mut result = NurbsCurve::new(dim, false, order, cv_count);
    for i in 0..kc { result.set_knot(i, knots_vec[i]); }
    for i in 0..cv_count { result.set_cv(i, &Point::new(cv[i*3], cv[i*3+1], cv[i*3+2])); }
    result
}

fn create_loft_with_params(input_curves: &[NurbsCurve], cross_params: &[f64]) -> NurbsSurface {
    let mut curves: Vec<NurbsCurve> = input_curves.to_vec();
    make_curves_compatible(&mut curves);

    let n_sections = curves.len();
    let cv_u = curves[0].cv_count();
    let is_rat = curves[0].is_rational();

    let mut col_curves = Vec::new();
    for col in 0..cv_u {
        let mut col_pts = Vec::new();
        for k in 0..n_sections {
            if is_rat {
                if let Some((x, y, z, w)) = curves[k].get_cv_4d(col) {
                    col_pts.push(Point::new(x/w, y/w, z/w));
                }
            } else {
                col_pts.push(curves[k].get_cv(col).unwrap());
            }
        }
        col_curves.push(cubic_interpolation_curve(&col_pts, cross_params));
    }

    let cv_v = col_curves[0].cv_count();
    let order_v = col_curves[0].order();

    let mut srf = NurbsSurface::create_raw(3, is_rat, curves[0].order(), order_v,
        cv_u, cv_v, false, false, 1.0, 1.0).unwrap();
    for i in 0..srf.knot_count(0) { srf.set_knot(0, i, curves[0].knot(i).unwrap()); }
    for i in 0..srf.knot_count(1) { srf.set_knot(1, i, col_curves[0].knot(i).unwrap()); }
    for i in 0..cv_u {
        for j in 0..cv_v {
            srf.set_cv(i, j, &col_curves[i].get_cv(j).unwrap());
        }
    }
    srf
}

fn create_bicubic_grid_surface(grid: &[Vec<Point>], u_params: &[f64], v_params: &[f64]) -> NurbsSurface {
    let n_rows = grid.len();
    let mut row_curves = Vec::new();
    for i in 0..n_rows {
        row_curves.push(cubic_interpolation_curve(&grid[i], u_params));
    }
    make_curves_compatible(&mut row_curves);

    let cv_u = row_curves[0].cv_count();

    let mut col_curves = Vec::new();
    for col in 0..cv_u {
        let mut col_pts = Vec::new();
        for row in 0..n_rows {
            col_pts.push(row_curves[row].get_cv(col).unwrap());
        }
        col_curves.push(cubic_interpolation_curve(&col_pts, v_params));
    }

    let cv_v = col_curves[0].cv_count();
    let order_v = col_curves[0].order();

    let mut srf = NurbsSurface::create_raw(3, false, row_curves[0].order(), order_v,
        cv_u, cv_v, false, false, 1.0, 1.0).unwrap();
    for i in 0..srf.knot_count(0) { srf.set_knot(0, i, row_curves[0].knot(i).unwrap()); }
    for i in 0..srf.knot_count(1) { srf.set_knot(1, i, col_curves[0].knot(i).unwrap()); }
    for i in 0..cv_u {
        for j in 0..cv_v {
            srf.set_cv(i, j, &col_curves[i].get_cv(j).unwrap());
        }
    }
    srf
}

fn reparametrize_curve(curve: &NurbsCurve, old_params: &[f64], new_params: &[f64], samples_per_span: usize) -> NurbsCurve {
    let n_pts = old_params.len();
    let mut pts = Vec::new();
    let mut pars = Vec::new();
    for i in 0..n_pts-1 {
        let (t0, t1) = (old_params[i], old_params[i+1]);
        let (a0, a1) = (new_params[i], new_params[i+1]);
        let nsamp = if i == n_pts - 2 { samples_per_span + 1 } else { samples_per_span };
        for s in 0..nsamp {
            let frac = s as f64 / samples_per_span as f64;
            pts.push(curve.point_at(t0 + (t1 - t0) * frac));
            pars.push(a0 + (a1 - a0) * frac);
        }
    }
    cubic_interpolation_curve(&pts, &pars)
}

pub struct Primitives;

impl Primitives {
    /// Create a circle as a rational NURBS curve (9 control points)
    pub fn circle(cx: f64, cy: f64, cz: f64, radius: f64) -> NurbsCurve {
        let w = (2.0_f64).sqrt() / 2.0;
        let cx_pat: [f64; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let cy_pat: [f64; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];

        let mut curve = NurbsCurve::new(3, true, 3, 9);
        curve.m_knot = vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

        for i in 0..9 {
            let px = cx + radius * cx_pat[i];
            let py = cy + radius * cy_pat[i];
            curve.set_cv_4d(i, px * weights[i], py * weights[i], cz * weights[i], weights[i]);
        }
        curve
    }

    /// Create an ellipse as a rational NURBS curve
    pub fn ellipse(cx: f64, cy: f64, cz: f64, major_radius: f64, minor_radius: f64) -> NurbsCurve {
        let w = (2.0_f64).sqrt() / 2.0;
        let ex: [f64; 9] = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let ey: [f64; 9] = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];

        let mut curve = NurbsCurve::new(3, true, 3, 9);
        curve.m_knot = vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

        for i in 0..9 {
            let px = cx + major_radius * ex[i];
            let py = cy + minor_radius * ey[i];
            curve.set_cv_4d(i, px * weights[i], py * weights[i], cz * weights[i], weights[i]);
        }
        curve
    }

    /// Create an arc through three points as a rational NURBS curve
    pub fn arc(start: &Point, mid: &Point, end: &Point) -> NurbsCurve {
        let d1 = [mid[0] - start[0], mid[1] - start[1], mid[2] - start[2]];
        let d2 = [end[0] - mid[0], end[1] - mid[1], end[2] - mid[2]];

        let normal = [
            d1[1] * d2[2] - d1[2] * d2[1],
            d1[2] * d2[0] - d1[0] * d2[2],
            d1[0] * d2[1] - d1[1] * d2[0]
        ];
        let normal_len = (normal[0].powi(2) + normal[1].powi(2) + normal[2].powi(2)).sqrt();

        if normal_len < Tolerance::ZERO_TOLERANCE {
            return NurbsCurve::create(false, 1, &[start.clone(), end.clone()]);
        }

        // Calculate weight from arc geometry
        let chord_mid = Point::new(
            (start[0] + end[0]) / 2.0,
            (start[1] + end[1]) / 2.0,
            (start[2] + end[2]) / 2.0
        );
        let sagitta = chord_mid.distance(mid, None);
        let chord_len = start.distance(end, None);

        if sagitta < Tolerance::ZERO_TOLERANCE {
            return NurbsCurve::create(false, 1, &[start.clone(), end.clone()]);
        }

        let half_chord = chord_len / 2.0;
        let r_approx = if sagitta > 0.0 {
            (half_chord.powi(2) + sagitta.powi(2)) / (2.0 * sagitta)
        } else {
            f64::INFINITY
        };

        let w = if r_approx > 0.0 {
            let cos_half = (r_approx - sagitta) / r_approx;
            let cos_half = cos_half.max(-1.0).min(1.0);
            if cos_half > 0.0 { cos_half.abs() } else { 0.5 }
        } else {
            0.5
        };
        let w = w.max(0.1).min(1.0);

        let mut curve = NurbsCurve::new(3, true, 3, 3);
        curve.m_knot = vec![0.0, 0.0, 1.0, 1.0];

        let shoulder = Point::new(
            (start[0] + end[0]) / 2.0 + (mid[0] - (start[0] + end[0]) / 2.0) / w,
            (start[1] + end[1]) / 2.0 + (mid[1] - (start[1] + end[1]) / 2.0) / w,
            (start[2] + end[2]) / 2.0 + (mid[2] - (start[2] + end[2]) / 2.0) / w
        );

        curve.set_cv_4d(0, start[0], start[1], start[2], 1.0);
        curve.set_cv_4d(1, shoulder[0] * w, shoulder[1] * w, shoulder[2] * w, w);
        curve.set_cv_4d(2, end[0], end[1], end[2], 1.0);

        curve
    }

    /// Create a parabola through 3 points as a non-rational quadratic NURBS
    pub fn parabola(p0: &Point, p1: &Point, p2: &Point) -> NurbsCurve {
        let mut curve = NurbsCurve::new(3, false, 3, 3);
        curve.m_knot = vec![0.0, 0.0, 1.0, 1.0];

        let cv1 = Point::new(
            2.0 * p1[0] - (p0[0] + p2[0]) / 2.0,
            2.0 * p1[1] - (p0[1] + p2[1]) / 2.0,
            2.0 * p1[2] - (p0[2] + p2[2]) / 2.0
        );

        curve.set_cv(0, p0);
        curve.set_cv(1, &cv1);
        curve.set_cv(2, p2);

        curve
    }

    /// Create a hyperbola segment as a NURBS curve
    pub fn hyperbola(center: &Point, a: f64, b: f64, extent: f64) -> NurbsCurve {
        let num_segments = 8;
        let cv_count = num_segments + 1;

        let points: Vec<Point> = (0..cv_count)
            .map(|i| {
                let t = -extent + 2.0 * extent * (i as f64) / (num_segments as f64);
                Point::new(center[0] + a * t.cosh(), center[1] + b * t.sinh(), center[2])
            })
            .collect();

        NurbsCurve::create_clamped_uniform(3, 4, &points, 1.0)
    }

    /// Create a spiral (helix with varying radius)
    pub fn spiral(start_radius: f64, end_radius: f64, pitch: f64, turns: f64) -> NurbsCurve {
        let segments_per_turn = 8;
        let total_segments = ((turns * segments_per_turn as f64) as usize).max(4);
        let cv_count = total_segments + 1;
        let total_angle = turns * 2.0 * PI;

        let points: Vec<Point> = (0..cv_count)
            .map(|i| {
                let t = (i as f64) / (total_segments as f64);
                let angle = t * total_angle;
                let r = start_radius + t * (end_radius - start_radius);
                Point::new(r * angle.cos(), r * angle.sin(), t * turns * pitch)
            })
            .collect();

        NurbsCurve::create_clamped_uniform(3, 4, &points, 1.0)
    }

    fn unit_cylinder_geometry() -> (Vec<Point>, Vec<[usize; 3]>) {
        let vertices = vec![
            Point::new(0.5, 0.0, -0.5),
            Point::new(0.404508, 0.293893, -0.5),
            Point::new(0.154508, 0.475528, -0.5),
            Point::new(-0.154508, 0.475528, -0.5),
            Point::new(-0.404508, 0.293893, -0.5),
            Point::new(-0.5, 0.0, -0.5),
            Point::new(-0.404508, -0.293893, -0.5),
            Point::new(-0.154508, -0.475528, -0.5),
            Point::new(0.154508, -0.475528, -0.5),
            Point::new(0.404508, -0.293893, -0.5),
            Point::new(0.5, 0.0, 0.5),
            Point::new(0.404508, 0.293893, 0.5),
            Point::new(0.154508, 0.475528, 0.5),
            Point::new(-0.154508, 0.475528, 0.5),
            Point::new(-0.404508, 0.293893, 0.5),
            Point::new(-0.5, 0.0, 0.5),
            Point::new(-0.404508, -0.293893, 0.5),
            Point::new(-0.154508, -0.475528, 0.5),
            Point::new(0.154508, -0.475528, 0.5),
            Point::new(0.404508, -0.293893, 0.5),
        ];
        let triangles = vec![
            [0, 1, 11], [0, 11, 10], [1, 2, 12], [1, 12, 11],
            [2, 3, 13], [2, 13, 12], [3, 4, 14], [3, 14, 13],
            [4, 5, 15], [4, 15, 14], [5, 6, 16], [5, 16, 15],
            [6, 7, 17], [6, 17, 16], [7, 8, 18], [7, 18, 17],
            [8, 9, 19], [8, 19, 18], [9, 0, 10], [9, 10, 19],
        ];
        (vertices, triangles)
    }

    fn unit_cone_geometry() -> (Vec<Point>, Vec<[usize; 3]>) {
        let vertices = vec![
            Point::new(0.0, 0.0, 0.5),
            Point::new(0.5, 0.0, -0.5),
            Point::new(0.353553, -0.353553, -0.5),
            Point::new(0.0, -0.5, -0.5),
            Point::new(-0.353553, -0.353553, -0.5),
            Point::new(-0.5, 0.0, -0.5),
            Point::new(-0.353553, 0.353553, -0.5),
            Point::new(0.0, 0.5, -0.5),
            Point::new(0.353553, 0.353553, -0.5),
        ];
        let triangles = vec![
            [0, 2, 1], [0, 3, 2], [0, 4, 3], [0, 5, 4],
            [0, 6, 5], [0, 7, 6], [0, 8, 7], [0, 1, 8],
        ];
        (vertices, triangles)
    }

    fn line_to_cylinder_transform(line: &Line, radius: f64) -> Xform {
        let start = line.start();
        let end = line.end();
        let line_vec = line.to_vector();
        let length = line.length();

        let z_axis = line_vec.normalized();
        let x_axis = if z_axis[2].abs() < 0.9 {
            Vector::new(0.0, 0.0, 1.0).cross(&z_axis).normalized()
        } else {
            Vector::new(1.0, 0.0, 0.0).cross(&z_axis).normalized()
        };
        let y_axis = z_axis.cross(&x_axis).normalized();

        let scale = Xform::scale_xyz(radius * 2.0, radius * 2.0, length);
        let rotation = Xform::from_cols(x_axis, y_axis, z_axis);
        let center = Point::new(
            (start[0] + end[0]) * 0.5,
            (start[1] + end[1]) * 0.5,
            (start[2] + end[2]) * 0.5,
        );
        let translation = Xform::translation(center[0], center[1], center[2]);
        &translation * &(&rotation * &scale)
    }

    fn transform_geometry(geometry: &(Vec<Point>, Vec<[usize; 3]>), xform: &Xform) -> Mesh {
        let (vertices, triangles) = geometry;
        let mut mesh = Mesh::new();
        let vertex_keys: Vec<usize> = vertices
            .iter()
            .map(|v| {
                let transformed = xform.transformed_point(v);
                mesh.add_vertex(transformed, None)
            })
            .collect();
        for tri in triangles {
            let face_vertices = vec![vertex_keys[tri[0]], vertex_keys[tri[1]], vertex_keys[tri[2]]];
            mesh.add_face(face_vertices, None);
        }
        mesh
    }

    pub fn cylinder_mesh(line: &Line, radius: f64) -> Mesh {
        let unit_cyl = Self::unit_cylinder_geometry();
        let xform = Self::line_to_cylinder_transform(line, radius);
        Self::transform_geometry(&unit_cyl, &xform)
    }

    pub fn arrow_mesh(line: &Line, radius: f64) -> Mesh {
        let start = line.start();
        let line_vec = line.to_vector();
        let length = line.length();

        let z_axis = line_vec.normalized();
        let x_axis = if z_axis[2].abs() < 0.9 {
            Vector::new(0.0, 0.0, 1.0).cross(&z_axis).normalized()
        } else {
            Vector::new(1.0, 0.0, 0.0).cross(&z_axis).normalized()
        };
        let y_axis = z_axis.cross(&x_axis).normalized();

        let cone_length = length * 0.2;
        let body_length = length * 0.8;

        let body_center = Point::new(
            start[0] + line_vec[0] * 0.4,
            start[1] + line_vec[1] * 0.4,
            start[2] + line_vec[2] * 0.4,
        );
        let cone_base_center = Point::new(
            start[0] + line_vec[0] * 0.9,
            start[1] + line_vec[1] * 0.9,
            start[2] + line_vec[2] * 0.9,
        );

        let body_scale = Xform::scale_xyz(radius * 2.0, radius * 2.0, body_length);
        let origin = Point::new(0.0, 0.0, 0.0);
        let rotation = Xform::xy_to_plane(&origin, &x_axis, &y_axis, &z_axis);
        let body_translation = Xform::translation(body_center[0], body_center[1], body_center[2]);
        let body_xform = &body_translation * &(&rotation * &body_scale);

        let cone_scale = Xform::scale_xyz(radius * 3.0, radius * 3.0, cone_length);
        let cone_translation = Xform::translation(cone_base_center[0], cone_base_center[1], cone_base_center[2]);
        let cone_xform = &cone_translation * &(&rotation * &cone_scale);

        let body_geometry = Self::unit_cylinder_geometry();
        let cone_geometry = Self::unit_cone_geometry();

        let mut mesh = Mesh::new();

        let mut body_vertex_map = Vec::new();
        for v in &body_geometry.0 {
            let transformed = body_xform.transformed_point(v);
            let key = mesh.add_vertex(transformed, None);
            body_vertex_map.push(key);
        }
        for tri in &body_geometry.1 {
            let face_vertices = vec![body_vertex_map[tri[0]], body_vertex_map[tri[1]], body_vertex_map[tri[2]]];
            mesh.add_face(face_vertices, None);
        }

        let mut cone_vertex_map = Vec::new();
        for v in &cone_geometry.0 {
            let transformed = cone_xform.transformed_point(v);
            let key = mesh.add_vertex(transformed, None);
            cone_vertex_map.push(key);
        }
        for tri in &cone_geometry.1 {
            let face_vertices = vec![cone_vertex_map[tri[0]], cone_vertex_map[tri[1]], cone_vertex_map[tri[2]]];
            mesh.add_face(face_vertices, None);
        }

        mesh
    }

    // Surface factory methods

    pub fn cylinder_surface(cx: f64, cy: f64, cz: f64, radius: f64, height: f64) -> NurbsSurface {
        let w = (2.0_f64).sqrt() / 2.0;
        let circle_weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let circle_x = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let circle_y = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let u_knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
        let v_knots = [0.0, 1.0];

        let mut srf = NurbsSurface::create_simple(3, true, 3, 2, 9, 2).unwrap();
        for i in 0..10 { srf.set_knot(0, i, u_knots[i]); }
        for i in 0..2 { srf.set_knot(1, i, v_knots[i]); }
        for i in 0..9 {
            let wi = circle_weights[i];
            let px = cx + radius * circle_x[i];
            let py = cy + radius * circle_y[i];
            srf.set_cv_4d(i, 0, px * wi, py * wi, cz * wi, wi);
            srf.set_cv_4d(i, 1, px * wi, py * wi, (cz + height) * wi, wi);
        }
        srf
    }

    pub fn cone_surface(cx: f64, cy: f64, cz: f64, radius: f64, height: f64) -> NurbsSurface {
        let w = (2.0_f64).sqrt() / 2.0;
        let circle_weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let circle_x = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let circle_y = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let u_knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
        let v_knots = [0.0, 1.0];

        let mut srf = NurbsSurface::create_simple(3, true, 3, 2, 9, 2).unwrap();
        for i in 0..10 { srf.set_knot(0, i, u_knots[i]); }
        for i in 0..2 { srf.set_knot(1, i, v_knots[i]); }
        let apex_z = cz + height;
        for i in 0..9 {
            let wi = circle_weights[i];
            let px = cx + radius * circle_x[i];
            let py = cy + radius * circle_y[i];
            srf.set_cv_4d(i, 0, px * wi, py * wi, cz * wi, wi);
            srf.set_cv_4d(i, 1, cx * wi, cy * wi, apex_z * wi, wi);
        }
        srf
    }

    pub fn torus_surface(cx: f64, cy: f64, cz: f64, major_radius: f64, minor_radius: f64) -> NurbsSurface {
        let w = (2.0_f64).sqrt() / 2.0;
        let cw = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let cos_a = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let sin_a = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let u_knots = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

        let mut srf = NurbsSurface::create_simple(3, true, 3, 3, 9, 9).unwrap();
        for d in 0..2 {
            for i in 0..10 { srf.set_knot(d, i, u_knots[i]); }
        }
        for i in 0..9 {
            let ca = cos_a[i];
            let sa = sin_a[i];
            for j in 0..9 {
                let cb = cos_a[j];
                let sb = sin_a[j];
                let r = major_radius + minor_radius * cb;
                let px = cx + r * ca;
                let py = cy + r * sa;
                let pz = cz + minor_radius * sb;
                let wij = cw[i] * cw[j];
                srf.set_cv_4d(i, j, px * wij, py * wij, pz * wij, wij);
            }
        }
        srf
    }

    pub fn create_ruled(curve_a: &NurbsCurve, curve_b: &NurbsCurve) -> NurbsSurface {
        if !curve_a.is_valid() || !curve_b.is_valid() { return NurbsSurface::new(); }

        let mut ca = curve_a.duplicate();
        let mut cb = curve_b.duplicate();
        ca.set_domain(0.0, 1.0);
        cb.set_domain(0.0, 1.0);

        if ca.degree() < cb.degree() { ca.increase_degree(cb.degree()); }
        else if cb.degree() < ca.degree() { cb.increase_degree(ca.degree()); }

        if ca.is_rational() || cb.is_rational() {
            ca.make_rational();
            cb.make_rational();
        }

        let tol = 1e-10;
        let knots_b = cb.get_knots();
        for &k in &knots_b {
            let found = ca.get_knots().iter().any(|&ka| (ka - k).abs() < tol);
            if !found { ca.insert_knot(k, 1); }
        }
        let knots_a = ca.get_knots();
        for &k in &knots_a {
            let found = cb.get_knots().iter().any(|&kb| (kb - k).abs() < tol);
            if !found { cb.insert_knot(k, 1); }
        }

        let order_u = ca.order();
        let cv_count_u = ca.cv_count();
        let is_rat = ca.is_rational();

        let mut surface = match NurbsSurface::create_simple(3, is_rat, order_u, 2, cv_count_u, 2) {
            Some(s) => s,
            None => return NurbsSurface::new(),
        };

        for i in 0..ca.knot_count() {
            if let Some(kv) = ca.knot(i) { surface.set_knot(0, i, kv); }
        }
        surface.set_knot(1, 0, 0.0);
        surface.set_knot(1, 1, 1.0);

        if is_rat {
            for i in 0..cv_count_u {
                if let Some((ax, ay, az, aw)) = ca.get_cv_4d(i) { surface.set_cv_4d(i, 0, ax, ay, az, aw); }
                if let Some((bx, by, bz, bw)) = cb.get_cv_4d(i) { surface.set_cv_4d(i, 1, bx, by, bz, bw); }
            }
        } else {
            for i in 0..cv_count_u {
                if let Some(pt_a) = ca.get_cv(i) { surface.set_cv(i, 0, &pt_a); }
                if let Some(pt_b) = cb.get_cv(i) { surface.set_cv(i, 1, &pt_b); }
            }
        }
        surface
    }

    pub fn create_extrusion(curve: &NurbsCurve, direction: &Vector) -> NurbsSurface {
        if !curve.is_valid() { return NurbsSurface::new(); }
        let mut translated = curve.duplicate();
        let t = Xform::translation(direction[0], direction[1], direction[2]);
        translated.transform(Some(&t));
        Self::create_ruled(curve, &translated)
    }

    pub fn create_planar(boundary: &NurbsCurve) -> NurbsSurface {
        if !boundary.is_valid() { return NurbsSurface::new(); }

        let mut all_pts = Vec::new();
        for i in 0..boundary.cv_count() {
            if let Some(pt) = boundary.get_cv(i) { all_pts.push(pt); }
        }

        let mut unique_pts = all_pts.clone();
        if unique_pts.len() >= 2 {
            let f = &unique_pts[0];
            let l = &unique_pts[unique_pts.len() - 1];
            let d2 = (f[0]-l[0]).powi(2) + (f[1]-l[1]).powi(2) + (f[2]-l[2]).powi(2);
            if d2 < 1e-20 { unique_pts.pop(); }
        }
        if unique_pts.len() < 3 { return NurbsSurface::new(); }

        let make_bilinear = |orig: &Point, xax: &Vector, yax: &Vector,
                             min_u: f64, max_u: f64, min_v: f64, max_v: f64| -> NurbsSurface {
            let mut srf = NurbsSurface::create_simple(3, false, 2, 2, 2, 2).unwrap();
            srf.set_knot(0, 0, 0.0); srf.set_knot(0, 1, 1.0);
            srf.set_knot(1, 0, 0.0); srf.set_knot(1, 1, 1.0);
            let pt = |u: f64, v: f64| -> Point {
                Point::new(orig[0] + u*xax[0] + v*yax[0],
                           orig[1] + u*xax[1] + v*yax[1],
                           orig[2] + u*xax[2] + v*yax[2])
            };
            srf.set_cv(0, 0, &pt(min_u, min_v));
            srf.set_cv(1, 0, &pt(max_u, min_v));
            srf.set_cv(1, 1, &pt(max_u, max_v));
            srf.set_cv(0, 1, &pt(min_u, max_v));
            srf
        };

        let longest_edge_dir = |pts: &[Point]| -> Vector {
            let mut best_d2 = 0.0f64;
            let mut best_i = 0;
            for i in 0..pts.len() {
                let j = (i + 1) % pts.len();
                let dx = pts[j][0]-pts[i][0];
                let dy = pts[j][1]-pts[i][1];
                let dz = pts[j][2]-pts[i][2];
                let d2 = dx*dx + dy*dy + dz*dz;
                if d2 > best_d2 { best_d2 = d2; best_i = i; }
            }
            let j = (best_i + 1) % pts.len();
            let dx = pts[j][0]-pts[best_i][0];
            let dy = pts[j][1]-pts[best_i][1];
            let dz = pts[j][2]-pts[best_i][2];
            let len = (dx*dx + dy*dy + dz*dz).sqrt();
            Vector::new(dx/len, dy/len, dz/len)
        };

        if unique_pts.len() == 3 && boundary.degree() <= 1 {
            let mut srf = NurbsSurface::create_simple(3, false, 2, 2, 2, 2).unwrap();
            srf.set_knot(0, 0, 0.0); srf.set_knot(0, 1, 1.0);
            srf.set_knot(1, 0, 0.0); srf.set_knot(1, 1, 1.0);
            srf.set_cv(0, 0, &unique_pts[0]);
            srf.set_cv(1, 0, &unique_pts[1]);
            srf.set_cv(1, 1, &unique_pts[2]);
            srf.set_cv(0, 1, &unique_pts[0]);
            return srf;
        }

        if unique_pts.len() == 4 && boundary.degree() <= 1 {
            let mut srf = NurbsSurface::create_simple(3, false, 2, 2, 2, 2).unwrap();
            srf.set_knot(0, 0, 0.0); srf.set_knot(0, 1, 1.0);
            srf.set_knot(1, 0, 0.0); srf.set_knot(1, 1, 1.0);
            srf.set_cv(0, 0, &unique_pts[0]);
            srf.set_cv(1, 0, &unique_pts[1]);
            srf.set_cv(1, 1, &unique_pts[2]);
            srf.set_cv(0, 1, &unique_pts[3]);
            return srf;
        }

        if boundary.degree() <= 1 {
            let e1 = Vector::new(unique_pts[1][0]-unique_pts[0][0], unique_pts[1][1]-unique_pts[0][1], unique_pts[1][2]-unique_pts[0][2]);
            let e2 = Vector::new(unique_pts[2][0]-unique_pts[0][0], unique_pts[2][1]-unique_pts[0][1], unique_pts[2][2]-unique_pts[0][2]);
            let normal = e1.cross(&e2);
            let nlen = normal.magnitude();
            if nlen < 1e-14 { return NurbsSurface::new(); }
            let normal = &normal * (1.0 / nlen);

            let xax = longest_edge_dir(&unique_pts);
            let yax = normal.cross(&xax);
            let ylen = yax.magnitude();
            if ylen < 1e-14 { return NurbsSurface::new(); }
            let yax = &yax * (1.0 / ylen);

            let orig = unique_pts[0].clone();
            let (mut min_u, mut max_u, mut min_v, mut max_v) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
            for pt in &unique_pts {
                let dx = pt[0]-orig[0]; let dy = pt[1]-orig[1]; let dz = pt[2]-orig[2];
                let u = dx*xax[0] + dy*xax[1] + dz*xax[2];
                let v = dx*yax[0] + dy*yax[1] + dz*yax[2];
                if u < min_u { min_u = u; } if u > max_u { max_u = u; }
                if v < min_v { min_v = v; } if v > max_v { max_v = v; }
            }
            let mut pad = (max_u - min_u).max(max_v - min_v) * 0.05;
            if pad < 1e-6 { pad = 1.0; }
            min_u -= pad; max_u += pad; min_v -= pad; max_v += pad;
            return make_bilinear(&orig, &xax, &yax, min_u, max_u, min_v, max_v);
        }

        let n_samples = 20usize.max(boundary.cv_count() * 4);
        let (sample_pts, _sample_params) = boundary.divide_by_count(n_samples, true);
        let plane = Plane::from_points_pca(sample_pts.clone());
        if plane.z_axis().magnitude() < 1e-10 { return NurbsSurface::new(); }

        let xax = plane.x_axis();
        let yax = plane.y_axis();
        let orig = plane.origin();

        let (mut min_u, mut max_u, mut min_v, mut max_v) = (1e30f64, -1e30f64, 1e30f64, -1e30f64);
        for pt in &sample_pts {
            let dx = pt[0]-orig[0]; let dy = pt[1]-orig[1]; let dz = pt[2]-orig[2];
            let u = dx*xax[0] + dy*xax[1] + dz*xax[2];
            let v = dx*yax[0] + dy*yax[1] + dz*yax[2];
            if u < min_u { min_u = u; } if u > max_u { max_u = u; }
            if v < min_v { min_v = v; } if v > max_v { max_v = v; }
        }
        let mut pad = (max_u - min_u).max(max_v - min_v) * 0.05;
        if pad < 1e-6 { pad = 1.0; }
        min_u -= pad; max_u += pad; min_v -= pad; max_v += pad;

        make_bilinear(&orig, &xax, &yax, min_u, max_u, min_v, max_v)
    }

    pub fn create_loft(input_curves: &[NurbsCurve], degree_v: usize) -> NurbsSurface {
        if input_curves.len() < 2 { return NurbsSurface::new(); }
        for c in input_curves {
            if !c.is_valid() { return NurbsSurface::new(); }
        }

        let mut curves: Vec<NurbsCurve> = input_curves.iter().map(|c| c.duplicate()).collect();
        make_curves_compatible(&mut curves);
        make_curves_compatible(&mut curves);

        let n_sections = curves.len();
        let cv_count_u = curves[0].cv_count();
        let order_u = curves[0].order();
        let is_rat = curves[0].is_rational();

        let mut degree_v = degree_v;
        if degree_v >= n_sections { degree_v = n_sections - 1; }
        if degree_v < 1 { degree_v = 1; }
        let order_v = degree_v + 1;

        let mut v_params = vec![0.0; n_sections];
        for k in 1..n_sections {
            let pk_prev = curves[k - 1].point_at_middle();
            let pk_curr = curves[k].point_at_middle();
            let dx = pk_curr[0] - pk_prev[0];
            let dy = pk_curr[1] - pk_prev[1];
            let dz = pk_curr[2] - pk_prev[2];
            v_params[k] = v_params[k - 1] + (dx * dx + dy * dy + dz * dz).sqrt();
        }
        let total_len = v_params[n_sections - 1];
        if total_len > 1e-14 {
            for k in 0..n_sections { v_params[k] /= total_len; }
        } else {
            for k in 0..n_sections { v_params[k] = k as f64 / (n_sections - 1) as f64; }
        }

        let cv_count_v = n_sections;
        let knot_count_v = order_v + cv_count_v - 2;
        let mut knots_v = vec![0.0; knot_count_v];

        if degree_v >= n_sections - 1 {
            let d = degree_v;
            for i in 0..d { knots_v[i] = 0.0; }
            for i in d..knot_count_v { knots_v[i] = 1.0; }
        } else {
            for i in 0..(order_v - 1) { knots_v[i] = v_params[0]; }
            for j in 1..=(n_sections - order_v) {
                let mut sum = 0.0;
                for i in j..(j + degree_v) { sum += v_params[i]; }
                knots_v[order_v - 2 + j] = sum / degree_v as f64;
            }
            for i in (knot_count_v - order_v + 1)..knot_count_v {
                knots_v[i] = v_params[n_sections - 1];
            }
        }

        let mut surface = match NurbsSurface::create_simple(3, is_rat, order_u, order_v, cv_count_u, cv_count_v) {
            Some(s) => s,
            None => return NurbsSurface::new(),
        };

        for i in 0..surface.knot_count(0) {
            if let Some(k) = curves[0].knot(i) { surface.set_knot(0, i, k); }
        }
        for i in 0..knots_v.len() {
            if i < surface.knot_count(1) { surface.set_knot(1, i, knots_v[i]); }
        }

        let n = n_sections;
        let mut n_matrix = vec![vec![0.0; n]; n];

        for k in 0..n {
            let mut t = v_params[k];
            let t0 = knots_v[order_v - 2];
            let t1 = knots_v[knot_count_v - order_v + 1];
            if t < t0 { t = t0; }
            if t > t1 { t = t1; }

            let span = knot::find_span(order_v, cv_count_v, &knots_v, t);
            let d = order_v - 1;
            let knot_base = span + d;

            if knots_v[knot_base - 1] == knots_v[knot_base] {
                if t <= knots_v[knot_base] {
                    n_matrix[k][span] = 1.0;
                } else {
                    n_matrix[k][span + order_v - 1] = 1.0;
                }
                continue;
            }

            let mut nvals = vec![0.0; order_v * order_v];
            nvals[order_v * order_v - 1] = 1.0;
            let mut left = vec![0.0; d];
            let mut right = vec![0.0; d];
            let mut n_idx = (order_v * order_v - 1) as i64;
            let mut k_right = knot_base;
            let mut k_left = knot_base - 1;

            for j in 0..d {
                let n0_idx = n_idx;
                n_idx -= (order_v + 1) as i64;
                left[j] = t - knots_v[k_left];
                right[j] = knots_v[k_right] - t;
                if k_left > 0 { k_left -= 1; } else { k_left = 0; }
                k_right += 1;

                let mut x = 0.0;
                for r in 0..=j {
                    let a0 = left[j - r];
                    let a1 = right[r];
                    let denom = a0 + a1;
                    let y = if denom != 0.0 { nvals[n0_idx as usize + r] / denom } else { 0.0 };
                    nvals[n_idx as usize + r] = x + a1 * y;
                    x = a0 * y;
                }
                nvals[n_idx as usize + j + 1] = x;
            }

            for j in 0..order_v {
                let col = span + j;
                if col < n { n_matrix[k][col] = nvals[j]; }
            }
        }

        let dim = if is_rat { 4 } else { 3 };
        for i in 0..cv_count_u {
            let mut rhs = vec![vec![0.0; dim]; n];
            for k in 0..n {
                if is_rat {
                    if let Some((cx, cy, cz, cw)) = curves[k].get_cv_4d(i) {
                        rhs[k] = vec![cx, cy, cz, cw];
                    }
                } else {
                    if let Some(p) = curves[k].get_cv(i) {
                        rhs[k] = vec![p[0], p[1], p[2]];
                    }
                }
            }

            let mut a = n_matrix.clone();
            let mut b = rhs;

            for col in 0..n {
                let mut max_row = col;
                let mut max_val = a[col][col].abs();
                for row in (col + 1)..n {
                    if a[row][col].abs() > max_val {
                        max_val = a[row][col].abs();
                        max_row = row;
                    }
                }
                if max_val < 1e-14 { continue; }
                a.swap(col, max_row);
                b.swap(col, max_row);
                for row in (col + 1)..n {
                    let factor = a[row][col] / a[col][col];
                    for c in col..n { a[row][c] -= factor * a[col][c]; }
                    for d2 in 0..dim { b[row][d2] -= factor * b[col][d2]; }
                }
            }

            let mut q = vec![vec![0.0; dim]; n];
            for row in (0..n).rev() {
                for d2 in 0..dim {
                    q[row][d2] = b[row][d2];
                    for c in (row + 1)..n { q[row][d2] -= a[row][c] * q[c][d2]; }
                    if a[row][row].abs() > 1e-14 { q[row][d2] /= a[row][row]; }
                }
            }

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

    pub fn create_revolve(profile: &NurbsCurve, axis_origin: &Point,
                          axis_direction: &Vector, angle: f64) -> NurbsSurface {
        if !profile.is_valid() { return NurbsSurface::new(); }
        let ax_len = axis_direction.magnitude();
        if ax_len < 1e-14 { return NurbsSurface::new(); }
        let axis_dir = &(*axis_direction) / ax_len;

        let mut angle = angle.abs();
        if angle > 2.0 * PI { angle = 2.0 * PI; }
        if angle < 1e-14 { return NurbsSurface::new(); }

        let n_arcs = if angle <= PI / 2.0 + 1e-10 { 1 }
                     else if angle <= PI + 1e-10 { 2 }
                     else if angle <= 3.0 * PI / 2.0 + 1e-10 { 3 }
                     else { 4 };

        let d_theta = angle / n_arcs as f64;
        let w_mid = (d_theta / 2.0).cos();
        let n_u = 2 * n_arcs + 1;

        let knot_count_u = n_u + 1;
        let mut knots_u = vec![0.0; knot_count_u];
        knots_u[0] = 0.0;
        knots_u[1] = 0.0;
        for i in 1..=n_arcs {
            let kv = i as f64 * d_theta;
            knots_u[2 * i] = kv;
            knots_u[2 * i + 1] = kv;
        }
        knots_u[knot_count_u - 1] = angle;
        knots_u[knot_count_u - 2] = angle;

        let cv_count_v = profile.cv_count();
        let order_v = profile.order();
        let profile_rational = profile.is_rational();

        let mut surface = match NurbsSurface::create_simple(3, true, 3, order_v, n_u, cv_count_v) {
            Some(s) => s,
            None => return NurbsSurface::new(),
        };

        for i in 0..knot_count_u.min(surface.knot_count(0)) {
            surface.set_knot(0, i, knots_u[i]);
        }
        for i in 0..profile.knot_count().min(surface.knot_count(1)) {
            if let Some(kv) = profile.knot(i) { surface.set_knot(1, i, kv); }
        }

        let mut u_angles = vec![0.0; n_u];
        let mut u_weights = vec![0.0; n_u];
        for i in 0..n_u {
            if i % 2 == 0 {
                u_angles[i] = (i / 2) as f64 * d_theta;
                u_weights[i] = 1.0;
            } else {
                u_angles[i] = (i / 2) as f64 * d_theta + d_theta / 2.0;
                u_weights[i] = w_mid;
            }
        }

        for j in 0..cv_count_v {
            let p_j = profile.get_cv(j).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let profile_w = if profile_rational { profile.weight(j) } else { 1.0 };

            let dx = p_j[0] - axis_origin[0];
            let dy = p_j[1] - axis_origin[1];
            let dz = p_j[2] - axis_origin[2];
            let proj = dx * axis_dir[0] + dy * axis_dir[1] + dz * axis_dir[2];
            let o_j = Point::new(
                axis_origin[0] + proj * axis_dir[0],
                axis_origin[1] + proj * axis_dir[1],
                axis_origin[2] + proj * axis_dir[2],
            );

            let rx = p_j[0] - o_j[0];
            let ry = p_j[1] - o_j[1];
            let rz = p_j[2] - o_j[2];
            let r_j = (rx * rx + ry * ry + rz * rz).sqrt();

            if r_j < 1e-14 {
                for i in 0..n_u {
                    let combined_w = u_weights[i] * profile_w;
                    surface.set_cv(i, j, &o_j);
                    surface.set_weight(i, j, combined_w);
                }
            } else {
                let x_local = Vector::new(rx / r_j, ry / r_j, rz / r_j);
                let mut y_local = axis_dir.cross(&x_local);
                let y_len = y_local.magnitude();
                if y_len > 1e-14 { y_local = &y_local / y_len; }

                for i in 0..n_u {
                    let theta = u_angles[i];
                    let cos_t = theta.cos();
                    let sin_t = theta.sin();

                    let effective_r = if i % 2 == 1 { r_j / w_mid } else { r_j };

                    let px = o_j[0] + effective_r * (cos_t * x_local[0] + sin_t * y_local[0]);
                    let py = o_j[1] + effective_r * (cos_t * x_local[1] + sin_t * y_local[1]);
                    let pz = o_j[2] + effective_r * (cos_t * x_local[2] + sin_t * y_local[2]);

                    let combined_w = u_weights[i] * profile_w;
                    surface.set_cv_4d(i, j, px * combined_w, py * combined_w, pz * combined_w, combined_w);
                }
            }
        }
        surface
    }

    pub fn create_revolve_full(profile: &NurbsCurve, axis_origin: &Point,
                               axis_direction: &Vector) -> NurbsSurface {
        Self::create_revolve(profile, axis_origin, axis_direction, 2.0 * PI)
    }

    pub fn create_sweep1(rail: &NurbsCurve, profile: &NurbsCurve) -> NurbsSurface {
        if !rail.is_valid() || !profile.is_valid() { return NurbsSurface::new(); }

        let working_profile = profile.duplicate();

        let n = (rail.span_count() * 2 + 1).max(5).min(20);
        let frames = rail.get_perpendicular_planes(n);
        if frames.is_empty() { return NurbsSurface::new(); }

        let nc = working_profile.cv_count();
        let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
        for k in 0..nc {
            if let Some(cv) = working_profile.get_cv(k) {
                cx += cv[0]; cy += cv[1]; cz += cv[2];
            }
        }
        cx /= nc as f64; cy /= nc as f64; cz /= nc as f64;

        let (t0, t1) = working_profile.domain();
        let pa = working_profile.point_at(t0);
        let pb = working_profile.point_at(t0 + (t1 - t0) / 3.0);
        let pc = working_profile.point_at(t0 + 2.0 * (t1 - t0) / 3.0);
        let v1 = Vector::new(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
        let v2 = Vector::new(pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]);
        let mut prof_normal = v1.cross(&v2);
        let nlen = prof_normal.magnitude();
        if nlen < 1e-14 { prof_normal = Vector::new(1.0, 0.0, 0.0); }
        else { prof_normal = &prof_normal / nlen; }

        let mut prof_x = Vector::new(pa[0] - cx, pa[1] - cy, pa[2] - cz);
        let mut pxlen = prof_x.magnitude();
        if pxlen < 1e-14 { prof_x = Vector::new(0.0, 1.0, 0.0); }
        else { prof_x = &prof_x / pxlen; }
        let dot = prof_x[0] * prof_normal[0] + prof_x[1] * prof_normal[1] + prof_x[2] * prof_normal[2];
        prof_x = Vector::new(prof_x[0] - dot * prof_normal[0], prof_x[1] - dot * prof_normal[1], prof_x[2] - dot * prof_normal[2]);
        pxlen = prof_x.magnitude();
        if pxlen < 1e-14 { prof_x = Vector::new(0.0, 1.0, 0.0); }
        else { prof_x = &prof_x / pxlen; }
        let mut prof_y = prof_normal.cross(&prof_x);
        let pylen = prof_y.magnitude();
        if pylen > 1e-14 { prof_y = &prof_y / pylen; }

        let mut positioned_profiles = Vec::with_capacity(frames.len());
        for i in 0..frames.len() {
            let mut prof_copy = working_profile.duplicate();
            let fo = frames[i].origin();
            let fx = frames[i].x_axis();
            let fy = frames[i].y_axis();
            let fz = frames[i].z_axis();

            let t1x = Xform::translation(-cx, -cy, -cz);

            let mut rot = Xform::identity();
            rot.m[0]  = fx[0]*prof_x[0] + fy[0]*prof_y[0] + fz[0]*prof_normal[0];
            rot.m[1]  = fx[1]*prof_x[0] + fy[1]*prof_y[0] + fz[1]*prof_normal[0];
            rot.m[2]  = fx[2]*prof_x[0] + fy[2]*prof_y[0] + fz[2]*prof_normal[0];
            rot.m[4]  = fx[0]*prof_x[1] + fy[0]*prof_y[1] + fz[0]*prof_normal[1];
            rot.m[5]  = fx[1]*prof_x[1] + fy[1]*prof_y[1] + fz[1]*prof_normal[1];
            rot.m[6]  = fx[2]*prof_x[1] + fy[2]*prof_y[1] + fz[2]*prof_normal[1];
            rot.m[8]  = fx[0]*prof_x[2] + fy[0]*prof_y[2] + fz[0]*prof_normal[2];
            rot.m[9]  = fx[1]*prof_x[2] + fy[1]*prof_y[2] + fz[1]*prof_normal[2];
            rot.m[10] = fx[2]*prof_x[2] + fy[2]*prof_y[2] + fz[2]*prof_normal[2];
            rot.m[12] = fo[0]; rot.m[13] = fo[1]; rot.m[14] = fo[2];

            prof_copy.transform(Some(&t1x));
            prof_copy.transform(Some(&rot));
            positioned_profiles.push(prof_copy);
        }

        let loft_degree = 3usize.min(positioned_profiles.len() - 1);
        Self::create_loft(&positioned_profiles, loft_degree)
    }

    pub fn create_sweep2(rail1: &NurbsCurve, rail2: &NurbsCurve,
                         shapes: &[NurbsCurve]) -> NurbsSurface {
        if !rail1.is_valid() || !rail2.is_valid() || shapes.is_empty() { return NurbsSurface::new(); }
        for s in shapes { if !s.is_valid() { return NurbsSurface::new(); } }

        let mut compat_shapes: Vec<NurbsCurve> = shapes.iter().map(|s| s.duplicate()).collect();
        if compat_shapes.len() >= 2 { make_curves_compatible(&mut compat_shapes); }

        let n_shapes = compat_shapes.len();
        let shape_params: Vec<f64> = (0..n_shapes).map(|k| {
            if n_shapes == 1 { 0.0 } else { k as f64 / (n_shapes - 1) as f64 }
        }).collect();

        let n = (rail1.span_count().max(rail2.span_count()) * 2 + 1).max(5).min(20);

        let (pts1, _params1) = rail1.divide_by_count(n + 1, true);
        let (pts2, _params2) = rail2.divide_by_count(n + 1, true);

        let frames1 = rail1.get_perpendicular_planes(n);
        if frames1.is_empty() { return NurbsSurface::new(); }

        struct ShapeInfo {
            start: Point, _end: Point, width: f64,
            dir: Vector, side: Vector, up: Vector,
        }
        let sinfo: Vec<ShapeInfo> = (0..n_shapes).map(|k| {
            let start = compat_shapes[k].point_at_start();
            let end = compat_shapes[k].point_at_end();
            let span = Vector::new(end[0]-start[0], end[1]-start[1], end[2]-start[2]);
            let mut width = span.magnitude();
            if width < 1e-14 { width = 1.0; }
            let dir = &span / width;
            let up_try = Vector::new(0.0, 0.0, 1.0);
            let mut side = dir.cross(&up_try);
            if side.magnitude() < 1e-10 {
                let up_try2 = Vector::new(0.0, 1.0, 0.0);
                side = dir.cross(&up_try2);
            }
            side = &side / side.magnitude();
            let mut up = side.cross(&dir);
            let ulen = up.magnitude();
            if ulen > 1e-14 { up = &up / ulen; }
            ShapeInfo { start, _end: end, width, dir, side, up }
        }).collect();

        let mut positioned_profiles = Vec::with_capacity(frames1.len());

        for i in 0..frames1.len().min(pts1.len()).min(pts2.len()) {
            let t = if frames1.len() <= 1 { 0.0 } else { i as f64 / (frames1.len() - 1) as f64 };

            let mut j = 0usize;
            let mut s;
            if n_shapes == 1 {
                j = 0; s = 0.0;
            } else {
                for k in 0..(n_shapes - 1) {
                    if t <= shape_params[k + 1] + 1e-14 { j = k; break; }
                    j = k;
                }
                let denom = shape_params[j + 1] - shape_params[j];
                s = if denom > 1e-14 { (t - shape_params[j]) / denom } else { 0.0 };
                s = s.max(0.0).min(1.0);
            }

            let mut interp_shape = compat_shapes[j].duplicate();
            if n_shapes > 1 && j + 1 < n_shapes {
                let nc = compat_shapes[j].cv_count();
                for c in 0..nc {
                    let cv0 = compat_shapes[j].get_cv(c).unwrap_or(Point::new(0.0,0.0,0.0));
                    let cv1 = compat_shapes[j + 1].get_cv(c).unwrap_or(Point::new(0.0,0.0,0.0));
                    let lerped = Point::new(cv0[0]*(1.0-s) + cv1[0]*s, cv0[1]*(1.0-s) + cv1[1]*s, cv0[2]*(1.0-s) + cv1[2]*s);
                    interp_shape.set_cv(c, &lerped);
                }
            }

            let shape_width = if n_shapes == 1 { sinfo[0].width }
                else { sinfo[j].width * (1.0 - s) + if j + 1 < n_shapes { sinfo[j+1].width * s } else { 0.0 } };

            let lerp_vec = |a: &Vector, b: &Vector| -> Vector {
                Vector::new(a[0]*(1.0-s)+b[0]*s, a[1]*(1.0-s)+b[1]*s, a[2]*(1.0-s)+b[2]*s)
            };

            let (mut prof_dir, mut prof_side, mut prof_up) = if n_shapes > 1 && j + 1 < n_shapes {
                (lerp_vec(&sinfo[j].dir, &sinfo[j+1].dir),
                 lerp_vec(&sinfo[j].side, &sinfo[j+1].side),
                 lerp_vec(&sinfo[j].up, &sinfo[j+1].up))
            } else {
                (sinfo[j].dir.clone(), sinfo[j].side.clone(), sinfo[j].up.clone())
            };
            let pdlen = prof_dir.magnitude();
            if pdlen > 1e-14 { prof_dir = &prof_dir / pdlen; }
            let pslen = prof_side.magnitude();
            if pslen > 1e-14 { prof_side = &prof_side / pslen; }
            let pulen = prof_up.magnitude();
            if pulen > 1e-14 { prof_up = &prof_up / pulen; }

            let interp_start = if n_shapes == 1 { sinfo[0].start.clone() }
                else if j + 1 < n_shapes {
                    Point::new(sinfo[j].start[0]*(1.0-s) + sinfo[j+1].start[0]*s,
                               sinfo[j].start[1]*(1.0-s) + sinfo[j+1].start[1]*s,
                               sinfo[j].start[2]*(1.0-s) + sinfo[j+1].start[2]*s)
                } else { sinfo[j].start.clone() };

            let p1 = &pts1[i];
            let p2 = &pts2[i];
            let dx = p2[0] - p1[0]; let dy = p2[1] - p1[1]; let dz = p2[2] - p1[2];
            let rail_dist = (dx*dx + dy*dy + dz*dz).sqrt();
            let scale_factor = if rail_dist > 1e-14 && shape_width > 1e-14 { rail_dist / shape_width } else { 1.0 };

            let mut prof_copy = interp_shape;
            let t1 = Xform::translation(-interp_start[0], -interp_start[1], -interp_start[2]);
            prof_copy.transform(Some(&t1));

            let sc = Xform::scale_xyz(scale_factor, scale_factor, scale_factor);
            prof_copy.transform(Some(&sc));

            let tangent_orig = frames1[i].z_axis();
            let mut x_dir = Vector::new(dx, dy, dz);
            let x_len = x_dir.magnitude();
            if x_len > 1e-14 { x_dir = &x_dir / x_len; }
            else { x_dir = frames1[i].x_axis(); }
            let mut y_dir = tangent_orig.cross(&x_dir);
            let y_len = y_dir.magnitude();
            if y_len > 1e-14 { y_dir = &y_dir / y_len; }
            else { y_dir = frames1[i].y_axis(); }
            let dot_up = y_dir[0]*prof_up[0] + y_dir[1]*prof_up[1] + y_dir[2]*prof_up[2];
            if dot_up < 0.0 { y_dir = Vector::new(-y_dir[0], -y_dir[1], -y_dir[2]); }
            let mut tangent = x_dir.cross(&y_dir);
            let tz = tangent.magnitude();
            if tz > 1e-14 { tangent = &tangent / tz; }

            let mut rot = Xform::identity();
            rot.m[0]  = tangent[0]*prof_side[0] + x_dir[0]*prof_dir[0] + y_dir[0]*prof_up[0];
            rot.m[1]  = tangent[1]*prof_side[0] + x_dir[1]*prof_dir[0] + y_dir[1]*prof_up[0];
            rot.m[2]  = tangent[2]*prof_side[0] + x_dir[2]*prof_dir[0] + y_dir[2]*prof_up[0];
            rot.m[4]  = tangent[0]*prof_side[1] + x_dir[0]*prof_dir[1] + y_dir[0]*prof_up[1];
            rot.m[5]  = tangent[1]*prof_side[1] + x_dir[1]*prof_dir[1] + y_dir[1]*prof_up[1];
            rot.m[6]  = tangent[2]*prof_side[1] + x_dir[2]*prof_dir[1] + y_dir[2]*prof_up[1];
            rot.m[8]  = tangent[0]*prof_side[2] + x_dir[0]*prof_dir[2] + y_dir[0]*prof_up[2];
            rot.m[9]  = tangent[1]*prof_side[2] + x_dir[1]*prof_dir[2] + y_dir[1]*prof_up[2];
            rot.m[10] = tangent[2]*prof_side[2] + x_dir[2]*prof_dir[2] + y_dir[2]*prof_up[2];
            rot.m[12] = p1[0]; rot.m[13] = p1[1]; rot.m[14] = p1[2];

            prof_copy.transform(Some(&rot));
            positioned_profiles.push(prof_copy);
        }

        let loft_degree = 3usize.min(positioned_profiles.len() - 1);
        Self::create_loft(&positioned_profiles, loft_degree)
    }

    pub fn create_edge(c0: &NurbsCurve, c1: &NurbsCurve,
                       c2: &NurbsCurve, c3: &NurbsCurve) -> NurbsSurface {
        if !c0.is_valid() || !c1.is_valid() || !c2.is_valid() || !c3.is_valid() {
            return NurbsSurface::new();
        }

        let input = [c0.duplicate(), c1.duplicate(), c2.duplicate(), c3.duplicate()];
        let mut loop_curves: Vec<NurbsCurve> = Vec::new();
        let mut used = [false; 4];
        let tol = 1e-6;

        loop_curves.push(input[0].duplicate());
        used[0] = true;

        for _step in 0..3 {
            let tail = loop_curves.last().unwrap().point_at_end();
            let mut found = false;
            for i in 0..4 {
                if used[i] { continue; }
                let s = input[i].point_at_start();
                let e = input[i].point_at_end();
                if s.distance(&tail, None) < tol {
                    loop_curves.push(input[i].duplicate());
                    used[i] = true;
                    found = true;
                    break;
                }
                if e.distance(&tail, None) < tol {
                    let mut rev = input[i].duplicate();
                    rev.reverse();
                    loop_curves.push(rev);
                    used[i] = true;
                    found = true;
                    break;
                }
            }
            if !found { return NurbsSurface::new(); }
        }

        if loop_curves[3].point_at_end().distance(&loop_curves[0].point_at_start(), None) > tol {
            return NurbsSurface::new();
        }

        let south = loop_curves[0].duplicate();
        let east = loop_curves[1].duplicate();
        let mut north = loop_curves[2].duplicate(); north.reverse();
        let mut west = loop_curves[3].duplicate(); west.reverse();

        let mut v_pair = vec![south.duplicate(), north.duplicate()];
        make_curves_compatible(&mut v_pair);
        let south = v_pair.remove(0);
        let north = v_pair.remove(0);

        let mut u_pair = vec![west.duplicate(), east.duplicate()];
        make_curves_compatible(&mut u_pair);
        let west = u_pair.remove(0);
        let east = u_pair.remove(0);

        let order_v = south.order();
        let cv_count_v = south.cv_count();
        let order_u = west.order();
        let cv_count_u = west.cv_count();
        let is_rat = south.is_rational() || west.is_rational();

        let mut surface = match NurbsSurface::create_simple(3, is_rat, order_u, order_v, cv_count_u, cv_count_v) {
            Some(s) => s,
            None => return NurbsSurface::new(),
        };

        for i in 0..surface.knot_count(0) {
            if let Some(kv) = west.knot(i) { surface.set_knot(0, i, kv); }
        }
        for i in 0..surface.knot_count(1) {
            if let Some(kv) = south.knot(i) { surface.set_knot(1, i, kv); }
        }

        let u_grev = west.get_greville_abcissae();
        let v_grev = south.get_greville_abcissae();

        let (u0, u1) = west.domain();
        let (v0, v1) = south.domain();
        let u_grev: Vec<f64> = u_grev.iter().map(|&g| if u1 > u0 { (g - u0) / (u1 - u0) } else { 0.0 }).collect();
        let v_grev: Vec<f64> = v_grev.iter().map(|&g| if v1 > v0 { (g - v0) / (v1 - v0) } else { 0.0 }).collect();

        let c00 = south.get_cv(0).unwrap_or(Point::new(0.0,0.0,0.0));
        let c01 = south.get_cv(cv_count_v - 1).unwrap_or(Point::new(0.0,0.0,0.0));
        let c10 = north.get_cv(0).unwrap_or(Point::new(0.0,0.0,0.0));
        let c11 = north.get_cv(cv_count_v - 1).unwrap_or(Point::new(0.0,0.0,0.0));

        for i in 0..cv_count_u {
            let ui = u_grev[i];
            let wi = west.get_cv(i).unwrap_or(Point::new(0.0,0.0,0.0));
            let ei = east.get_cv(i).unwrap_or(Point::new(0.0,0.0,0.0));
            for j in 0..cv_count_v {
                let vj = v_grev[j];
                let sj = south.get_cv(j).unwrap_or(Point::new(0.0,0.0,0.0));
                let nj = north.get_cv(j).unwrap_or(Point::new(0.0,0.0,0.0));

                let x = (1.0-ui)*sj[0] + ui*nj[0] + (1.0-vj)*wi[0] + vj*ei[0]
                       - (1.0-ui)*(1.0-vj)*c00[0] - (1.0-ui)*vj*c01[0]
                       - ui*(1.0-vj)*c10[0] - ui*vj*c11[0];
                let y = (1.0-ui)*sj[1] + ui*nj[1] + (1.0-vj)*wi[1] + vj*ei[1]
                       - (1.0-ui)*(1.0-vj)*c00[1] - (1.0-ui)*vj*c01[1]
                       - ui*(1.0-vj)*c10[1] - ui*vj*c11[1];
                let z = (1.0-ui)*sj[2] + ui*nj[2] + (1.0-vj)*wi[2] + vj*ei[2]
                       - (1.0-ui)*(1.0-vj)*c00[2] - (1.0-ui)*vj*c01[2]
                       - ui*(1.0-vj)*c10[2] - ui*vj*c11[2];

                surface.set_cv(i, j, &Point::new(x, y, z));
            }
        }
        surface
    }

    pub fn create_network(u_curves: &[NurbsCurve], v_curves: &[NurbsCurve]) -> NurbsSurface {
        let n_u = u_curves.len();
        let n_v = v_curves.len();
        if n_u < 2 || n_v < 2 { return NurbsSurface::new(); }

        for c in u_curves.iter().chain(v_curves.iter()) {
            if !c.is_valid() { return NurbsSurface::new(); }
        }

        // Step 1: Find intersection parameters
        let mut u_par: Vec<Vec<f64>> = vec![vec![0.0; n_v]; n_u];
        let mut v_par: Vec<Vec<f64>> = vec![vec![0.0; n_v]; n_u];
        for i in 0..n_u {
            for j in 0..n_v {
                let (t1, t2, _) = find_curve_intersection(&u_curves[i], &v_curves[j]);
                u_par[i][j] = t1;
                v_par[i][j] = t2;
            }
        }

        // Step 2: Build point grid with averaging
        let mut grid: Vec<Vec<Point>> = vec![vec![Point::new(0.0,0.0,0.0); n_v]; n_u];
        for i in 0..n_u {
            for j in 0..n_v {
                let pu = u_curves[i].point_at(u_par[i][j]);
                let pv = v_curves[j].point_at(v_par[i][j]);
                grid[i][j] = Point::new(
                    0.5*(pu[0]+pv[0]), 0.5*(pu[1]+pv[1]), 0.5*(pu[2]+pv[2]));
            }
        }

        // Step 3: Centripetal parameterization
        let pdist = |a: &Point, b: &Point| -> f64 {
            let (dx, dy, dz) = (a[0]-b[0], a[1]-b[1], a[2]-b[2]);
            (dx*dx + dy*dy + dz*dz).sqrt()
        };

        let mut v_arc_params = vec![0.0; n_u];
        for i in 1..n_u {
            let (mut mx, mut mn) = (0.0_f64, 1e30_f64);
            for j in 0..n_v {
                let d = pdist(&grid[i-1][j], &grid[i][j]);
                mx = mx.max(d); mn = mn.min(d);
            }
            let mut inc = (mx.sqrt() + mn.sqrt()) * 0.5;
            inc = inc * inc;
            if inc < 0.0001 { inc = 0.0001; }
            v_arc_params[i] = v_arc_params[i-1] + inc;
        }
        if v_arc_params[n_u-1] > 1e-14 {
            let total = v_arc_params[n_u-1];
            for x in v_arc_params.iter_mut() { *x /= total; }
        }

        let mut u_arc_params = vec![0.0; n_v];
        for j in 1..n_v {
            let (mut mx, mut mn) = (0.0_f64, 1e30_f64);
            for i in 0..n_u {
                let d = pdist(&grid[i][j-1], &grid[i][j]);
                mx = mx.max(d); mn = mn.min(d);
            }
            let mut inc = (mx.sqrt() + mn.sqrt()) * 0.5;
            inc = inc * inc;
            if inc < 0.0001 { inc = 0.0001; }
            u_arc_params[j] = u_arc_params[j-1] + inc;
        }
        if u_arc_params[n_v-1] > 1e-14 {
            let total = u_arc_params[n_v-1];
            for x in u_arc_params.iter_mut() { *x /= total; }
        }

        // Step 4: Build grid surface T
        let mut t_srf = create_bicubic_grid_surface(&grid, &u_arc_params, &v_arc_params);

        // Step 5: Build L_u (loft through u-curves)
        let mut l_u = create_loft_with_params(u_curves, &v_arc_params);

        // Step 6: Build L_v with reparametrized v-curves, then transpose
        let mut v_repar = Vec::new();
        for j in 0..n_v {
            let old_p: Vec<f64> = (0..n_u).map(|i| v_par[i][j]).collect();
            v_repar.push(reparametrize_curve(&v_curves[j], &old_p, &v_arc_params, 10));
        }
        let mut l_v = create_loft_with_params(&v_repar, &u_arc_params);
        l_v.transpose();

        if !t_srf.is_valid() || !l_u.is_valid() || !l_v.is_valid() {
            return NurbsSurface::new();
        }

        // Step 7: Knot compatibility via transpose trick
        make_surfaces_knot_compatible_1d(&mut t_srf, &mut l_u, &mut l_v);
        t_srf.transpose(); l_u.transpose(); l_v.transpose();
        make_surfaces_knot_compatible_1d(&mut t_srf, &mut l_u, &mut l_v);
        t_srf.transpose(); l_u.transpose(); l_v.transpose();

        let any_rat = t_srf.is_rational() || l_u.is_rational() || l_v.is_rational();
        if any_rat {
            t_srf.make_rational();
            l_u.make_rational();
            l_v.make_rational();
        }

        // Step 8: Gordon formula: S_cv = L_u_cv + L_v_cv - T_cv
        let mut surface = t_srf.duplicate();
        for i in 0..surface.cv_count_dir(Some(0)) {
            for j in 0..surface.cv_count_dir(Some(1)) {
                if any_rat {
                    let (lux, luy, luz, luw) = l_u.get_cv_4d(i, j).unwrap_or((0.0,0.0,0.0,1.0));
                    let (lvx, lvy, lvz, lvw) = l_v.get_cv_4d(i, j).unwrap_or((0.0,0.0,0.0,1.0));
                    let (tx, ty, tz, tw) = t_srf.get_cv_4d(i, j).unwrap_or((0.0,0.0,0.0,1.0));
                    surface.set_cv_4d(i, j, lux+lvx-tx, luy+lvy-ty, luz+lvz-tz, luw+lvw-tw);
                } else {
                    let lu = l_u.get_cv(i, j).unwrap_or(Point::new(0.0,0.0,0.0));
                    let lv = l_v.get_cv(i, j).unwrap_or(Point::new(0.0,0.0,0.0));
                    let t = t_srf.get_cv(i, j).unwrap_or(Point::new(0.0,0.0,0.0));
                    surface.set_cv(i, j, &Point::new(lu[0]+lv[0]-t[0], lu[1]+lv[1]-t[1], lu[2]+lv[2]-t[2]));
                }
            }
        }
        surface
    }

    pub fn create_interpolated(points: &[Point], parameterization: knot::CurveKnotStyle) -> NurbsCurve {
        NurbsCurve::create_interpolated(points, parameterization)
    }
}
