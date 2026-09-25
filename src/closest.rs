use crate::aabb::AABB;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbsknot::CurveInterpStyle;
use crate::nurbsknot::CurveNurbsKnotStyle;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::pointcloud::PointCloud;
use crate::polyline::Polyline;
use crate::spatial_aabbtree::SpatialAABBTree;
use crate::spatial_kdtree::SpatialKDTree;

const STACK_SIZE: usize = 64;

// ═══════════════════════════════════════════════════════════════════════════
// Curve helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Parameter of the closest sample on a dense grid over [t0, t1].
fn curve_seed(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64) -> f64 {
    let num_samples = (curve.cv_count() * 10).max(50);
    let dt = (t1 - t0) / num_samples as f64;
    let mut best_t = t0;
    let mut best_dist = curve.point_at(t0).distance(test_point, None);

    for i in 0..=num_samples {
        let t = t0 + i as f64 * dt;
        let dist = curve.point_at(t).distance(test_point, None);

        if dist < best_dist {
            best_dist = dist;
            best_t = t;
        }
    }

    best_t
}

/// Newton on (C(t) - P) . C'(t) = 0 from t, clamped to [t0, t1].
fn curve_newton(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64, mut t: f64) -> f64 {
    let max_iterations = 32;
    let step_tolerance = (t1 - t0) * 1e-12;

    for _ in 0..max_iterations {
        let derivs = curve.evaluate(t, 2);

        if derivs.len() < 3 {
            break;
        }

        let pt = &derivs[0];
        let d1 = &derivs[1];
        let d2 = &derivs[2];
        let rx = pt[0] - test_point[0];
        let ry = pt[1] - test_point[1];
        let rz = pt[2] - test_point[2];
        let f = rx * d1[0] + ry * d1[1] + rz * d1[2];

        if f.abs() < step_tolerance {
            break;
        }

        let df =
            d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2] + rx * d2[0] + ry * d2[1] + rz * d2[2];

        if df.abs() < 1e-14 {
            break;
        }

        let mut dt_step = -f / df;

        if dt_step.abs() > (t1 - t0) * 0.5 {
            dt_step = ((t1 - t0) * 0.5).copysign(dt_step);
        }

        t += dt_step;

        if t < t0 {
            t = t0;
        }

        if t > t1 {
            t = t1;
        }

        if dt_step.abs() < step_tolerance {
            break;
        }
    }

    t
}

/// Parameters of the closest pair on dense grids over both domains.
fn curve_curve_seed(curve0: &NurbsCurve, curve1: &NurbsCurve) -> (f64, f64) {
    let u0 = curve0.domain_start();
    let u1 = curve0.domain_end();
    let v0 = curve1.domain_start();
    let v1 = curve1.domain_end();
    let n0 = (curve0.cv_count() * 8).max(40);
    let n1 = (curve1.cv_count() * 8).max(40);
    let mut p0 = Vec::with_capacity(n0 + 1);
    let mut p1 = Vec::with_capacity(n1 + 1);

    for i in 0..=n0 {
        p0.push(curve0.point_at(u0 + (u1 - u0) * i as f64 / n0 as f64));
    }

    for j in 0..=n1 {
        p1.push(curve1.point_at(v0 + (v1 - v0) * j as f64 / n1 as f64));
    }

    let mut best = f64::INFINITY;
    let mut u = u0;
    let mut v = v0;

    for (i, pi) in p0.iter().enumerate() {
        for (j, pj) in p1.iter().enumerate() {
            let d2 = (pi - pj).magnitude_squared();

            if d2 < best {
                best = d2;
                u = u0 + (u1 - u0) * i as f64 / n0 as f64;
                v = v0 + (v1 - v0) * j as f64 / n1 as f64;
            }
        }
    }

    (u, v)
}

// ═══════════════════════════════════════════════════════════════════════════
// Surface helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Surface point with the origin fallback of an invalid evaluation.
fn surface_at(surface: &NurbsSurface, u: f64, v: f64) -> Point {
    surface.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0))
}

/// Surface domain with the empty fallback of an invalid surface.
fn surface_domain(surface: &NurbsSurface, dir: usize) -> (f64, f64) {
    surface.domain(dir).unwrap_or((0.0, 0.0))
}

/// Parameters of the closest sample on a grid whose resolution follows the window size.
fn surface_seed(
    surface: &NurbsSurface,
    test_point: &Point,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
) -> (f64, f64) {
    let (domain_u0, domain_u1) = surface_domain(surface, 0);
    let (domain_v0, domain_v1) = surface_domain(surface, 1);

    let full_u = surface.order(0).max(10);
    let full_v = surface.order(1).max(10);
    let u_frac = (u1 - u0) / (domain_u1 - domain_u0).max(1e-12);
    let v_frac = (v1 - v0) / (domain_v1 - domain_v0).max(1e-12);
    let u_samples = ((full_u as f64 * u_frac.min(1.0)).ceil() as usize).max(3);
    let v_samples = ((full_v as f64 * v_frac.min(1.0)).ceil() as usize).max(3);
    let du_param = (u1 - u0) / u_samples as f64;
    let dv_param = (v1 - v0) / v_samples as f64;
    let mut best_u = u0;
    let mut best_v = v0;
    let mut best_dist = f64::INFINITY;

    for i in 0..=u_samples {
        for j in 0..=v_samples {
            let uu = u0 + i as f64 * du_param;
            let vv = v0 + j as f64 * dv_param;
            let dist = surface_at(surface, uu, vv).distance(test_point, None);

            if dist < best_dist {
                best_dist = dist;
                best_u = uu;
                best_v = vv;
            }
        }
    }

    (best_u, best_v)
}

/// Newton on the perpendicular-foot conditions from the seed, clamped to the window.
fn surface_newton(
    surface: &NurbsSurface,
    test_point: &Point,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
    seed: (f64, f64),
) -> (f64, f64) {
    let (mut u, mut v) = seed;
    let max_iterations = 20;
    let step_tolerance = (u1 - u0).min(v1 - v0) * 1e-10;
    let max_step = (u1 - u0).min(v1 - v0) * 0.5;

    for _ in 0..max_iterations {
        let derivs = surface.evaluate(u, v, 1);

        if derivs.len() < 3 {
            break;
        }

        let pt = surface_at(surface, u, v);
        let du_vec = &derivs[2];
        let dv_vec = &derivs[1];
        let delta = test_point - &pt;
        let fu = -delta.dot(du_vec);
        let fv = -delta.dot(dv_vec);

        if fu.abs() < step_tolerance && fv.abs() < step_tolerance {
            break;
        }

        let duu = du_vec.dot(du_vec);
        let dvv = dv_vec.dot(dv_vec);
        let duv = du_vec.dot(dv_vec);
        let det = duu * dvv - duv * duv;

        if det.abs() < 1e-12 {
            break;
        }

        let mut du_step = (dvv * fu - duv * fv) / det;
        let mut dv_step = (duu * fv - duv * fu) / det;

        if du_step.abs() > max_step {
            du_step = max_step.copysign(du_step);
        }

        if dv_step.abs() > max_step {
            dv_step = max_step.copysign(dv_step);
        }

        u = (u - du_step).clamp(u0, u1);
        v = (v - dv_step).clamp(v0, v1);

        if du_step.abs() < step_tolerance && dv_step.abs() < step_tolerance {
            break;
        }
    }

    (u, v)
}

// ═══════════════════════════════════════════════════════════════════════════
// Pullback helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Surface domain, trace step and tolerances shared by the surface_curve steps.
#[derive(Default)]
struct Pullback {
    u0: f64,          // Surface domain start in u.
    u1: f64,          // Surface domain end in u.
    v0: f64,          // Surface domain start in v.
    v1: f64,          // Surface domain end in v.
    range_u: f64,     // Domain length in u.
    range_v: f64,     // Domain length in v.
    closed_u: bool,   // Surface closed in u.
    closed_v: bool,   // Surface closed in v.
    du: f64,          // Quarter-span step in u.
    dv: f64,          // Quarter-span step in v.
    step: f64,        // Uv deviation bound of a fitted pcurve.
    fit_tol: f64,     // 3d deviation bound of a lifted uv midpoint.
    reject_tol: f64,  // Residual above which a sample is re-inverted globally.
    on_surf_tol: f64, // Residual above which the curve is off the surface.
}

/// Domain, steps and tolerances of the surface for one pullback.
fn pullback_setup(surface: &NurbsSurface, tolerance: f64) -> Pullback {
    let mut pb = Pullback::default();
    (pb.u0, pb.u1) = surface_domain(surface, 0);
    (pb.v0, pb.v1) = surface_domain(surface, 1);
    pb.range_u = pb.u1 - pb.u0;
    pb.range_v = pb.v1 - pb.v0;
    pb.closed_u = surface.is_closed(0);
    pb.closed_v = surface.is_closed(1);

    let nu = surface.get_span_vector(0).len().saturating_sub(1).max(1) * 4;
    let nv = surface.get_span_vector(1).len().saturating_sub(1).max(1) * 4;
    pb.du = pb.range_u / nu as f64;
    pb.dv = pb.range_v / nv as f64;

    let mu = (pb.u0 + pb.u1) * 0.5;
    let mv = (pb.v0 + pb.v1) * 0.5;
    let pmid = surface_at(surface, mu, mv);
    let wu_probe = (mu + pb.du).min(pb.u1);
    let wv_probe = (mv + pb.dv).min(pb.v1);
    let uv_to_3d_u = pmid.distance(&surface_at(surface, wu_probe, mv), None) / pb.du;
    let uv_to_3d_v = pmid.distance(&surface_at(surface, mu, wv_probe), None) / pb.dv;
    let mut uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);
    let mut uv_to_3d_min = uv_to_3d_u.min(uv_to_3d_v);

    if uv_to_3d < 1e-10 {
        uv_to_3d = 1.0;
    }

    if uv_to_3d_min < 1e-10 {
        uv_to_3d_min = 1.0;
    }

    pb.step = pb.du.min(pb.dv) * 0.25;
    pb.fit_tol = if tolerance > 0.0 {
        tolerance
    } else {
        pb.step * (uv_to_3d + uv_to_3d_min) * 0.5
    };
    pb.reject_tol = pb.fit_tol * 100.0;

    let mut corner_diag =
        surface_at(surface, pb.u0, pb.v0).distance(&surface_at(surface, pb.u1, pb.v1), None);

    if corner_diag < 1e-12 {
        corner_diag = pb.range_u.max(pb.range_v);
    }

    pb.on_surf_tol = corner_diag * 0.05;

    pb
}

/// x folded into [x0, x1] by period when closed, clamped otherwise.
fn pullback_wrap(x: f64, x0: f64, x1: f64, closed: bool) -> f64 {
    if !closed {
        return x.clamp(x0, x1);
    }

    let mut t = (x - x0) % (x1 - x0);

    if t < 0.0 {
        t += x1 - x0;
    }

    x0 + t
}

/// x shifted by whole periods to within half a period of prev.
fn pullback_unwrap(prev: f64, mut x: f64, range: f64, closed: bool) -> f64 {
    if !closed {
        return x;
    }

    while x - prev > range * 0.5 {
        x -= range;
    }

    while x - prev < -range * 0.5 {
        x += range;
    }

    x
}

/// Windowed inversion of pt around (up, vp), trying the seam-mirrored windows when closed.
fn pullback_invert(
    surface: &NurbsSurface,
    pb: &Pullback,
    pt: &Point,
    up: f64,
    vp: f64,
    wu: f64,
    wv: f64,
) -> (f64, f64, f64) {
    let mut u_centers = vec![up];

    if pb.closed_u && up - wu < pb.u0 {
        u_centers.push(up + pb.range_u);
    }

    if pb.closed_u && up + wu > pb.u1 {
        u_centers.push(up - pb.range_u);
    }

    let mut v_centers = vec![vp];

    if pb.closed_v && vp - wv < pb.v0 {
        v_centers.push(vp + pb.range_v);
    }

    if pb.closed_v && vp + wv > pb.v1 {
        v_centers.push(vp - pb.range_v);
    }

    let mut best = (up, vp, f64::INFINITY);

    for &uc in &u_centers {
        for &vc in &v_centers {
            let wu0 = (uc - wu).max(pb.u0);
            let wu1 = (uc + wu).min(pb.u1);
            let wv0 = (vc - wv).max(pb.v0);
            let wv1 = (vc + wv).min(pb.v1);

            if wu1 - wu0 < 1e-14 || wv1 - wv0 < 1e-14 {
                continue;
            }

            let res = Closest::surface_point(surface, pt, wu0, wu1, wv0, wv1);

            if res.2 < best.2 {
                best = res;
            }

            if best.2 < pb.fit_tol * 0.01 {
                break;
            }
        }
    }

    best
}

/// Warm-started samples [t, u, v, residual] along [t0, t1], empty when the curve is off the surface.
fn pullback_samples(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    pb: &Pullback,
    t0: f64,
    t1: f64,
) -> Vec<[f64; 4]> {
    let n0 = (4 * curve.span_count()).max(16);
    let mut samples: Vec<[f64; 4]> = Vec::new();
    let mut max_residual = 0.0f64;
    let mut min_residual = f64::INFINITY;

    for i in 0..=n0 {
        let t = t0 + (t1 - t0) * i as f64 / n0 as f64;
        let pt = curve.point_at(t);
        let (uu, vv, rd) = if i == 0 {
            Closest::surface_point(surface, &pt, 0.0, 0.0, 0.0, 0.0)
        } else {
            let prev = samples[samples.len() - 1];
            let prev2 = samples[samples.len().saturating_sub(2)];
            let wu = pb.du.max(pb.dv) * 2.0 + (prev[1] - prev2[1]).abs();
            let wv = pb.du.max(pb.dv) * 2.0 + (prev[2] - prev2[2]).abs();
            let up = pullback_wrap(prev[1], pb.u0, pb.u1, pb.closed_u);
            let vp = pullback_wrap(prev[2], pb.v0, pb.v1, pb.closed_v);
            let (mut ru, mut rv, mut rd) = pullback_invert(surface, pb, &pt, up, vp, wu, wv);

            if rd > pb.reject_tol {
                (ru, rv, rd) = Closest::surface_point(surface, &pt, 0.0, 0.0, 0.0, 0.0);
            }

            let uu = pullback_unwrap(prev[1], ru, pb.range_u, pb.closed_u);
            let vv = pullback_unwrap(prev[2], rv, pb.range_v, pb.closed_v);

            (uu, vv, rd)
        };

        samples.push([t, uu, vv, rd]);
        max_residual = max_residual.max(rd);
        min_residual = min_residual.min(rd);
    }

    if max_residual > pb.reject_tol || min_residual > pb.on_surf_tol {
        samples.clear();
    }

    samples
}

/// Bisect every span whose lifted uv midpoint strays from the curve, up to 8 rounds or 4096 samples.
fn pullback_refine(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    pb: &Pullback,
    samples: &mut Vec<[f64; 4]>,
) {
    for _ in 0..8 {
        let mut inserted = 0;
        let mut i = 0;

        while i + 1 < samples.len() {
            let a = samples[i];
            let b = samples[i + 1];
            let tm = (a[0] + b[0]) * 0.5;
            let um = pullback_wrap((a[1] + b[1]) * 0.5, pb.u0, pb.u1, pb.closed_u);
            let vm = pullback_wrap((a[2] + b[2]) * 0.5, pb.v0, pb.v1, pb.closed_v);
            let pm = curve.point_at(tm);

            if surface_at(surface, um, vm).distance(&pm, None) <= pb.fit_tol
                || samples.len() >= 4096
            {
                i += 1;
                continue;
            }

            let wu = (b[1] - a[1]).abs().max(pb.du);
            let wv = (b[2] - a[2]).abs().max(pb.dv);
            let (ru, rv, rd) = pullback_invert(surface, pb, &pm, um, vm, wu, wv);

            if rd > pb.on_surf_tol {
                i += 1;
                continue;
            }

            let uu = pullback_unwrap(a[1], ru, pb.range_u, pb.closed_u);
            let vv = pullback_unwrap(a[2], rv, pb.range_v, pb.closed_v);

            samples.insert(i + 1, [tm, uu, vv, rd]);
            inserted += 1;
            i += 2;
        }

        if inserted == 0 {
            break;
        }
    }
}

/// Smallest seam crossing of one axis between a and b that beats bestt; level is the seam value.
fn pullback_seam_axis(
    a: f64,
    b: f64,
    x0: f64,
    range: f64,
    closed: bool,
    bestt: &mut f64,
    level: &mut f64,
) -> bool {
    if !closed || (b - a).abs() <= 1e-15 {
        return false;
    }

    let k0 = ((a - x0) / range).floor() as i64;
    let k1 = ((b - x0) / range).floor() as i64;
    let mut found = false;

    for k in (k0.min(k1) + 1)..=k0.max(k1) {
        let seam = x0 + k as f64 * range;
        let t = (seam - a) / (b - a);

        if t > 1e-9 && t < 1.0 - 1e-9 && t < *bestt {
            *bestt = t;
            *level = seam;
            found = true;
        }
    }

    found
}

/// First seam crossing on segment a -> b, written to (cu, cv).
fn pullback_first_seam(
    pb: &Pullback,
    a: &[f64; 2],
    b: &[f64; 2],
    cu: &mut f64,
    cv: &mut f64,
) -> bool {
    let mut bestt = 2.0;
    let mut level = 0.0;
    let mut found = false;

    if pullback_seam_axis(
        a[0],
        b[0],
        pb.u0,
        pb.range_u,
        pb.closed_u,
        &mut bestt,
        &mut level,
    ) {
        found = true;
        *cu = level;
        *cv = a[1] + (b[1] - a[1]) * bestt;
    }

    if pullback_seam_axis(
        a[1],
        b[1],
        pb.v0,
        pb.range_v,
        pb.closed_v,
        &mut bestt,
        &mut level,
    ) {
        found = true;
        *cv = level;
        *cu = a[0] + (b[0] - a[0]) * bestt;
    }

    found
}

/// True when x sits on a seam level of a closed axis.
fn pullback_at_seam(x: f64, x0: f64, range: f64, closed: bool) -> bool {
    if !closed {
        return false;
    }

    let seam = x0 + ((x - x0) / range).round() * range;

    (x - seam).abs() < range * 1e-6
}

/// True when p sits on a seam of either closed axis.
fn pullback_on_seam(pb: &Pullback, p: &[f64; 2]) -> bool {
    pullback_at_seam(p[0], pb.u0, pb.range_u, pb.closed_u)
        || pullback_at_seam(p[1], pb.v0, pb.range_v, pb.closed_v)
}

/// Shift a segment by whole periods so its middle point lies inside the domain.
fn pullback_shift(pb: &Pullback, seg: &mut [[f64; 2]]) {
    let mid = seg[seg.len() / 2];
    let k_u = if pb.closed_u {
        ((mid[0] - pb.u0) / pb.range_u).floor()
    } else {
        0.0
    };
    let k_v = if pb.closed_v {
        ((mid[1] - pb.v0) / pb.range_v).floor()
    } else {
        0.0
    };

    for p in seg.iter_mut() {
        p[0] -= k_u * pb.range_u;
        p[1] -= k_v * pb.range_v;
    }
}

/// Split the unwrapped uv polyline at every seam crossing; rejoin the two arcs of a mid-arc loop start.
fn pullback_split(
    pb: &Pullback,
    pts: &[[f64; 2]],
    rejoin: bool,
    any_cross: &mut bool,
) -> Vec<Vec<[f64; 2]>> {
    let mut raw: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut cur: Vec<[f64; 2]> = vec![pts[0]];
    *any_cross = false;

    for i in 1..pts.len() {
        let mut a = pts[i - 1];
        let b = pts[i];
        let mut cu = 0.0;
        let mut cv = 0.0;

        while pullback_first_seam(pb, &a, &b, &mut cu, &mut cv) {
            cur.push([cu, cv]);
            raw.push(cur);
            cur = vec![[cu, cv]];
            *any_cross = true;
            a = [cu, cv];
        }

        cur.push(b);

        if i + 1 < pts.len() && pullback_on_seam(pb, &b) {
            raw.push(cur);
            cur = vec![b];
            *any_cross = true;
        }
    }

    raw.push(cur);

    if rejoin && raw.len() > 1 {
        let mut merged = raw[raw.len() - 1].clone();

        for p in raw[0].iter().skip(1) {
            merged.push(*p);
        }

        raw.remove(0);
        let last = raw.len() - 1;
        raw[last] = merged;
    }

    raw
}

/// In-domain uv pieces with a closed flag, slivers dropped.
fn pullback_pieces(
    pb: &Pullback,
    curve: &NurbsCurve,
    samples: &[[f64; 4]],
) -> Vec<(Vec<[f64; 2]>, bool)> {
    let mut pts: Vec<[f64; 2]> = Vec::with_capacity(samples.len());

    for s in samples {
        pts.push([s[1], s[2]]);
    }

    let p_first = curve.point_at(samples[0][0]);
    let p_last = curve.point_at(samples[samples.len() - 1][0]);
    let is_loop = p_first.distance(&p_last, None) < pb.fit_tol * 4.0 && pts.len() >= 6;

    if is_loop {
        pts.pop();
    }

    let wind_u = if pb.closed_u {
        samples[samples.len() - 1][1] - samples[0][1]
    } else {
        0.0
    };
    let wind_v = if pb.closed_v {
        samples[samples.len() - 1][2] - samples[0][2]
    } else {
        0.0
    };
    let crosses = wind_u.abs() > pb.range_u * 0.5 || wind_v.abs() > pb.range_v * 0.5;
    let rejoin = is_loop && !crosses && !pullback_on_seam(pb, &pts[0]);
    let mut any_cross = false;
    let raw = pullback_split(pb, &pts, rejoin, &mut any_cross);
    let mut pieces: Vec<(Vec<[f64; 2]>, bool)> = Vec::new();

    for mut seg in raw {
        if seg.len() < 2 {
            continue;
        }

        pullback_shift(pb, &mut seg);

        let mut umin = 1e300;
        let mut umax = -1e300;
        let mut vmin = 1e300;
        let mut vmax = -1e300;
        let mut len = 0.0;

        for i in 0..seg.len() {
            umin = f64::min(umin, seg[i][0]);
            umax = f64::max(umax, seg[i][0]);
            vmin = f64::min(vmin, seg[i][1]);
            vmax = f64::max(vmax, seg[i][1]);

            if i > 0 {
                len += f64::hypot(seg[i][0] - seg[i - 1][0], seg[i][1] - seg[i - 1][1]);
            }
        }

        if len < pb.range_u.min(pb.range_v) * 1e-4 {
            continue;
        }

        let seg_loop = is_loop
            && !any_cross
            && umax - umin < pb.range_u * 0.9
            && vmax - vmin < pb.range_v * 0.9;

        pieces.push((seg, seg_loop));
    }

    pieces
}

/// Total turning angle of a uv polyline.
fn pullback_turning(pts_uv: &[Point]) -> f64 {
    let mp = pts_uv.len();
    let mut total_turning = 0.0;

    for i in 1..mp - 1 {
        let dx1 = pts_uv[i][0] - pts_uv[i - 1][0];
        let dy1 = pts_uv[i][1] - pts_uv[i - 1][1];
        let dx2 = pts_uv[i + 1][0] - pts_uv[i][0];
        let dy2 = pts_uv[i + 1][1] - pts_uv[i][1];
        let l1 = f64::hypot(dx1, dy1);
        let l2 = f64::hypot(dx2, dy2);

        if l1 <= 1e-14 || l2 <= 1e-14 {
            continue;
        }

        let c = ((dx1 * dx2 + dy1 * dy2) / (l1 * l2)).clamp(-1.0, 1.0);
        total_turning += c.acos();
    }

    total_turning
}

/// Normalized chord-length parameters of a uv polyline.
fn pullback_chords(pts_uv: &[Point], piece_loop: bool) -> Vec<f64> {
    let mp = pts_uv.len();
    let mut chords = vec![0.0; mp];
    let mut total_len = 0.0;

    for i in 1..mp {
        total_len += pts_uv[i].distance(&pts_uv[i - 1], None);
        chords[i] = total_len;
    }

    if piece_loop {
        total_len += pts_uv[0].distance(&pts_uv[mp - 1], None);
    }

    if total_len > 1e-14 {
        for chord in chords.iter_mut().skip(1) {
            *chord /= total_len;
        }
    }

    chords
}

/// Fit one piece as a uv pcurve on [0, 1]; interpolation and a degree-1 polyline are the fallbacks.
fn pullback_fit(pb: &Pullback, piece_pts: &mut [[f64; 2]], piece_loop: bool) -> NurbsCurve {
    pullback_shift(pb, piece_pts);

    let mut pts_uv: Vec<Point> = Vec::with_capacity(piece_pts.len());

    for p in piece_pts.iter() {
        pts_uv.push(Point::new(p[0], p[1], 0.0));
    }

    let mp = pts_uv.len();
    let chords = pullback_chords(&pts_uv, piece_loop);
    let mut target_cvs = ((pullback_turning(&pts_uv) / 0.5) as usize + 6).max(8);
    let max_cvs = mp - 1;
    let mut pcurve = NurbsCurve::default();

    for _ in 0..5 {
        if target_cvs > max_cvs {
            break;
        }

        pcurve = NurbsCurve::create_fitted(&pts_uv, target_cvs, 3, piece_loop);

        if !pcurve.is_valid() {
            break;
        }

        let ft0 = pcurve.domain_start();
        let ft1 = pcurve.domain_end();
        let mut max_dev = 0.0f64;

        for i in 0..mp {
            max_dev = max_dev.max(
                pcurve
                    .point_at(ft0 + (ft1 - ft0) * chords[i])
                    .distance(&pts_uv[i], None),
            );
        }

        if max_dev < pb.step {
            break;
        }

        target_cvs = (target_cvs * 2).min(max_cvs);
    }

    if !pcurve.is_valid() {
        pcurve = if piece_loop {
            NurbsCurve::create_interpolated(
                &pts_uv,
                CurveNurbsKnotStyle::ChordPeriodic,
                CurveInterpStyle::Rhino,
            )
        } else {
            NurbsCurve::create_interpolated(
                &pts_uv,
                CurveNurbsKnotStyle::Chord,
                CurveInterpStyle::Rhino,
            )
        };
    }

    if !pcurve.is_valid() {
        pcurve = NurbsCurve::create(false, 1, &pts_uv);
    }

    if pcurve.is_valid() {
        pcurve.set_domain(0.0, 1.0);
    }

    pcurve
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_point_on_triangle(p: &Point, a: &Point, b: &Point, c: &Point) -> Point {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);

    if d1 <= 0.0 && d2 <= 0.0 {
        return a.clone();
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);

    if d3 >= 0.0 && d4 <= d3 {
        return b.clone();
    }

    let vc = d1 * d4 - d3 * d2;

    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);

        return a + &ab * v;
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);

    if d6 >= 0.0 && d5 <= d6 {
        return c.clone();
    }

    let vb = d5 * d2 - d1 * d6;

    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);

        return a + &ac * w;
    }

    let va = d3 * d6 - d5 * d4;

    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

        return b + (c - b) * w;
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;

    a + &ab * v + &ac * w
}

/// Distance from p to the box, zero inside.
fn aabb_min_distance(aabb: &AABB, p: &Point) -> f64 {
    let dx = ((p[0] - aabb.cx).abs() - aabb.hx).max(0.0);
    let dy = ((p[1] - aabb.cy).abs() - aabb.hy).max(0.0);
    let dz = ((p[2] - aabb.cz).abs() - aabb.hz).max(0.0);

    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Distance between two boxes, zero when they overlap.
fn aabb_to_aabb_min_dist(a: &AABB, b: &AABB) -> f64 {
    let dx = ((a.cx - b.cx).abs() - a.hx - b.hx).max(0.0);
    let dy = ((a.cy - b.cy).abs() - a.hy - b.hy).max(0.0);
    let dz = ((a.cz - b.cz).abs() - a.hz - b.hz).max(0.0);

    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Face keys in the face-index order used by the triangle caches.
fn mesh_face_keys(mesh: &Mesh) -> Vec<usize> {
    let mut face_keys: Vec<usize> = mesh.face.keys().copied().collect();
    face_keys.sort();

    face_keys
}

/// Closest point, face key and distance on triangle object_id of mesh; infinite distance for an invalid id.
fn mesh_triangle_point(
    mesh: &Mesh,
    face_keys: &[usize],
    object_id: usize,
    test_point: &Point,
) -> (Point, usize, f64) {
    let Some((face_idx, _sub_idx, v0, v1, v2)) = mesh.get_triangle_by_id(object_id) else {
        return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
    };

    let cp = closest_point_on_triangle(test_point, &v0, &v1, &v2);
    let dist = cp.distance(test_point, None);

    (cp, face_keys[face_idx], dist)
}

/// Push the children nearer than best_dist, the nearer one last so it pops first.
fn push_nearer_last(
    stack: &mut [usize; STACK_SIZE],
    top: &mut usize,
    left: usize,
    right: usize,
    ld: f64,
    rd: f64,
    best_dist: f64,
) {
    assert!(*top + 2 <= STACK_SIZE);

    if ld <= rd {
        if rd < best_dist {
            stack[*top] = right;
            *top += 1;
        }

        if ld < best_dist {
            stack[*top] = left;
            *top += 1;
        }
    } else {
        if ld < best_dist {
            stack[*top] = left;
            *top += 1;
        }

        if rd < best_dist {
            stack[*top] = right;
            *top += 1;
        }
    }
}

/// Closest-point queries between points, curves, surfaces, meshes and clouds.
pub struct Closest;

impl Closest {
    // ═══════════════════════════════════════════════════════════════════════════
    // Curves
    // ═══════════════════════════════════════════════════════════════════════════
    /// Parameter and distance of the closest curve point within [t0, t1] (0 means the domain end).
    pub fn curve_point(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64) -> (f64, f64) {
        if !curve.is_valid() {
            return (0.0, f64::INFINITY);
        }

        let domain_start = curve.domain_start();
        let domain_end = curve.domain_end();
        let mut t0 = if t0 <= 0.0 { domain_start } else { t0 };
        let mut t1 = if t1 <= 0.0 { domain_end } else { t1 };
        t0 = t0.max(domain_start);
        t1 = t1.min(domain_end);

        let mut t = curve_newton(
            curve,
            test_point,
            t0,
            t1,
            curve_seed(curve, test_point, t0, t1),
        );
        let mut final_dist = curve.point_at(t).distance(test_point, None);
        let dist_start = curve.point_at(t0).distance(test_point, None);
        let dist_end = curve.point_at(t1).distance(test_point, None);

        if dist_start < final_dist {
            t = t0;
            final_dist = dist_start;
        }

        if dist_end < final_dist {
            t = t1;
            final_dist = dist_end;
        }

        (t, final_dist)
    }

    /// Return the parameters and distance of the closest approach between two curves.
    pub fn curve_curve(curve0: &NurbsCurve, curve1: &NurbsCurve) -> (f64, f64, f64) {
        if !curve0.is_valid() || !curve1.is_valid() {
            return (0.0, 0.0, f64::INFINITY);
        }

        let u0 = curve0.domain_start();
        let u1 = curve0.domain_end();
        let v0 = curve1.domain_start();
        let v1 = curve1.domain_end();
        let (mut u, mut v) = curve_curve_seed(curve0, curve1);

        for _ in 0..64 {
            let e0 = curve0.evaluate(u, 2);
            let e1 = curve1.evaluate(v, 2);

            if e0.len() < 3 || e1.len() < 3 {
                break;
            }

            let c0 = &e0[0];
            let c0p = &e0[1];
            let c0pp = &e0[2];
            let c1 = &e1[0];
            let c1p = &e1[1];
            let c1pp = &e1[2];
            let rx = c0[0] - c1[0];
            let ry = c0[1] - c1[1];
            let rz = c0[2] - c1[2];
            let gu = rx * c0p[0] + ry * c0p[1] + rz * c0p[2];
            let gv = -(rx * c1p[0] + ry * c1p[1] + rz * c1p[2]);
            let huu = c0p[0] * c0p[0]
                + c0p[1] * c0p[1]
                + c0p[2] * c0p[2]
                + rx * c0pp[0]
                + ry * c0pp[1]
                + rz * c0pp[2];
            let huv = -(c0p[0] * c1p[0] + c0p[1] * c1p[1] + c0p[2] * c1p[2]);
            let hvv = c1p[0] * c1p[0] + c1p[1] * c1p[1] + c1p[2] * c1p[2]
                - (rx * c1pp[0] + ry * c1pp[1] + rz * c1pp[2]);
            let det = huu * hvv - huv * huv;

            if det.abs() < 1e-14 {
                break;
            }

            let mut du = -(hvv * gu - huv * gv) / det;
            let mut dv = -(-huv * gu + huu * gv) / det;

            if du.abs() > (u1 - u0) * 0.5 {
                du = ((u1 - u0) * 0.5).copysign(du);
            }

            if dv.abs() > (v1 - v0) * 0.5 {
                dv = ((v1 - v0) * 0.5).copysign(dv);
            }

            u = (u + du).clamp(u0, u1);
            v = (v + dv).clamp(v0, v1);

            if du.abs().max(dv.abs()) < 1e-13 {
                break;
            }
        }

        let dist = curve0.point_at(u).distance(&curve1.point_at(v), None);

        (u, v, dist)
    }

    /// Return the closest point, parameter in [0, 1] and distance on a segment.
    pub fn line_point(line: &Line, test_point: &Point) -> (Point, f64, f64) {
        let start = line.start();
        let end = line.end();
        let direction = &end - &start;
        let len_sq = direction.magnitude_squared();

        if len_sq < 1e-20 {
            let dist = start.distance(test_point, None);

            return (start, 0.0, dist);
        }

        let t = ((test_point - &start).dot(&direction) / len_sq).clamp(0.0, 1.0);
        let closest = &start + &direction * t;
        let dist = closest.distance(test_point, None);

        (closest, t, dist)
    }

    /// Return the closest point, length parameter in [0, 1] and distance on a polyline.
    pub fn polyline_point(polyline: &Polyline, test_point: &Point) -> (Point, f64, f64) {
        let points = polyline.get_points();

        if points.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), 0.0, f64::INFINITY);
        }

        if points.len() == 1 {
            return (points[0].clone(), 0.0, points[0].distance(test_point, None));
        }

        let mut best_point = points[0].clone();
        let mut best_param = 0.0;
        let mut best_dist = f64::INFINITY;
        let mut cumulative_length = 0.0;
        let total_length = polyline.length();

        for i in 0..points.len() - 1 {
            let segment = Line::from_points(&points[i], &points[i + 1]);
            let segment_length = segment.length();
            let (closest, t, dist) = Self::line_point(&segment, test_point);

            if dist < best_dist {
                best_dist = dist;
                best_point = closest;

                if total_length > 1e-20 {
                    best_param = (cumulative_length + t * segment_length) / total_length;
                } else {
                    best_param = i as f64 / (points.len() - 1) as f64;
                }
            }

            cumulative_length += segment_length;
        }

        (best_point, best_param, best_dist)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Surfaces
    // ═══════════════════════════════════════════════════════════════════════════
    /// Parameters and distance of the closest surface point within a uv window (0 means the domain end).
    pub fn surface_point(
        surface: &NurbsSurface,
        test_point: &Point,
        u0: f64,
        u1: f64,
        v0: f64,
        v1: f64,
    ) -> (f64, f64, f64) {
        if !surface.is_valid() {
            return (0.0, 0.0, f64::INFINITY);
        }

        let (domain_u0, domain_u1) = surface_domain(surface, 0);
        let (domain_v0, domain_v1) = surface_domain(surface, 1);
        let mut u0 = if u0 <= 0.0 { domain_u0 } else { u0 };
        let mut u1 = if u1 <= 0.0 { domain_u1 } else { u1 };
        let mut v0 = if v0 <= 0.0 { domain_v0 } else { v0 };
        let mut v1 = if v1 <= 0.0 { domain_v1 } else { v1 };
        u0 = u0.max(domain_u0);
        u1 = u1.min(domain_u1);
        v0 = v0.max(domain_v0);
        v1 = v1.min(domain_v1);

        let seed = surface_seed(surface, test_point, u0, u1, v0, v1);
        let (u, v) = surface_newton(surface, test_point, u0, u1, v0, v1, seed);

        (u, v, surface_at(surface, u, v).distance(test_point, None))
    }

    /// Seam-split uv pcurves of a curve lying on the surface, empty when it does not.
    pub fn surface_curve(
        surface: &NurbsSurface,
        curve: &NurbsCurve,
        t0: f64,
        t1: f64,
        tolerance: f64,
    ) -> Vec<NurbsCurve> {
        if !surface.is_valid() || !curve.is_valid() {
            return Vec::new();
        }

        let ct0 = curve.domain_start();
        let ct1 = curve.domain_end();
        let mut t0 = if t0 <= 0.0 { ct0 } else { t0 };
        let mut t1 = if t1 <= 0.0 { ct1 } else { t1 };
        t0 = t0.max(ct0);
        t1 = t1.min(ct1);

        if t1 - t0 < 1e-14 {
            return Vec::new();
        }

        let pb = pullback_setup(surface, tolerance);
        let mut samples = pullback_samples(surface, curve, &pb, t0, t1);

        if samples.is_empty() {
            return Vec::new();
        }

        pullback_refine(surface, curve, &pb, &mut samples);

        let mut result = Vec::new();

        for (mut piece_pts, piece_loop) in pullback_pieces(&pb, curve, &samples) {
            let pcurve = pullback_fit(&pb, &mut piece_pts, piece_loop);

            if pcurve.is_valid() {
                result.push(pcurve);
            }
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshes and clouds
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the closest point, face key and distance on a mesh via its triangle BVH.
    pub fn mesh_point(mesh: &mut Mesh, test_point: &Point) -> (Point, usize, f64) {
        let mut best_point = Point::new(0.0, 0.0, 0.0);
        let mut best_face_key: usize = 0;
        let mut best_dist = f64::INFINITY;

        if mesh.number_of_faces() == 0 {
            return (best_point, best_face_key, best_dist);
        }

        mesh.build_triangle_bvh(false);
        let Some(bvh) = mesh.get_cached_bvh() else {
            return (best_point, best_face_key, best_dist);
        };

        if bvh.empty() {
            return (best_point, best_face_key, best_dist);
        }

        let face_keys = mesh_face_keys(mesh);
        let mut stack = [0usize; STACK_SIZE];
        let mut top = 0;
        stack[top] = 0;
        top += 1;

        while top > 0 {
            top -= 1;
            let node = &bvh.nodes[stack[top]];

            if aabb_min_distance(&node.aabb, test_point) >= best_dist {
                continue;
            }

            if node.is_leaf() {
                let hit =
                    mesh_triangle_point(mesh, &face_keys, node.object_id as usize, test_point);

                if hit.2 < best_dist {
                    (best_point, best_face_key, best_dist) = hit;
                }

                continue;
            }

            let ld = aabb_min_distance(&bvh.nodes[node.left as usize].aabb, test_point);
            let rd = aabb_min_distance(&bvh.nodes[node.right as usize].aabb, test_point);

            push_nearer_last(
                &mut stack,
                &mut top,
                node.left as usize,
                node.right as usize,
                ld,
                rd,
                best_dist,
            );
        }

        (best_point, best_face_key, best_dist)
    }

    /// Return the closest point, face key and distance on a mesh via its triangle AABB tree.
    pub fn mesh_point_aabb(mesh: &mut Mesh, test_point: &Point) -> (Point, usize, f64) {
        let mut best_point = Point::new(0.0, 0.0, 0.0);
        let mut best_face_key: usize = 0;
        let mut best_dist = f64::INFINITY;

        if mesh.number_of_faces() == 0 {
            return (best_point, best_face_key, best_dist);
        }

        mesh.build_triangle_aabb_tree(false);
        let Some(tree) = mesh.get_cached_aabb_tree() else {
            return (best_point, best_face_key, best_dist);
        };

        if tree.empty() {
            return (best_point, best_face_key, best_dist);
        }

        let face_keys = mesh_face_keys(mesh);
        let mut stack = [0usize; STACK_SIZE];
        let mut top = 0;
        stack[top] = 0;
        top += 1;

        while top > 0 {
            top -= 1;
            let ni = stack[top];
            let node = &tree.nodes[ni];

            if aabb_min_distance(&node.aabb, test_point) >= best_dist {
                continue;
            }

            if node.object_id >= 0 {
                let hit =
                    mesh_triangle_point(mesh, &face_keys, node.object_id as usize, test_point);

                if hit.2 < best_dist {
                    (best_point, best_face_key, best_dist) = hit;
                }

                continue;
            }

            let left = ni + 1;
            let right = node.right as usize;
            let ld = aabb_min_distance(&tree.nodes[left].aabb, test_point);
            let rd = aabb_min_distance(&tree.nodes[right].aabb, test_point);

            push_nearer_last(&mut stack, &mut top, left, right, ld, rd, best_dist);
        }

        (best_point, best_face_key, best_dist)
    }

    /// Return the closest point, index and distance in a cloud by linear scan.
    pub fn pointcloud_point(cloud: &PointCloud, test_point: &Point) -> (Point, usize, f64) {
        if cloud.point_count() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let mut best_point = cloud.get_point(0);
        let mut best_index: usize = 0;
        let mut best_dist = best_point.distance(test_point, None);

        for i in 1..cloud.point_count() {
            let p = cloud.get_point(i);
            let dist = p.distance(test_point, None);

            if dist < best_dist {
                best_dist = dist;
                best_point = p;
                best_index = i;
            }
        }

        (best_point, best_index, best_dist)
    }

    /// Return the closest point, index and distance in a cloud via a kd-tree.
    pub fn pointcloud_point_kdtree(cloud: &PointCloud, test_point: &Point) -> (Point, usize, f64) {
        if cloud.point_count() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let mut pts = Vec::with_capacity(cloud.point_count());

        for i in 0..cloud.point_count() {
            pts.push(cloud.get_point(i));
        }

        let kd = SpatialKDTree::new(pts);
        let (idx, dist) = kd.nearest(test_point);

        (cloud.get_point(idx), idx, dist)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Collections
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the index pairs of lines whose endpoints come within threshold of each other.
    pub fn lines_closest(lines: &[Line], threshold: f64) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();

        if threshold < 0.0 || lines.len() < 2 {
            return pairs;
        }

        let mut aabbs = Vec::with_capacity(lines.len());

        for ln in lines {
            aabbs.push(AABB::from_line(ln, threshold));
        }

        let mut tree = SpatialAABBTree::new();
        tree.build(&aabbs);

        for i in 0..lines.len() {
            for j_raw in tree.query_aabb(&aabbs[i]) {
                let j = j_raw as usize;

                if j <= i {
                    continue;
                }

                let d_a = Self::line_point(&lines[j], &lines[i].start()).2;
                let d_b = Self::line_point(&lines[j], &lines[i].end()).2;
                let d_c = Self::line_point(&lines[i], &lines[j].start()).2;
                let d_d = Self::line_point(&lines[i], &lines[j].end()).2;

                if d_a.min(d_b).min(d_c).min(d_d) <= threshold {
                    pairs.push((i, j));
                }
            }
        }

        pairs
    }

    /// Index pairs of polylines whose vertices come within threshold of each other.
    pub fn polylines_closest(polylines: &[Polyline], threshold: f64) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();

        if threshold < 0.0 || polylines.len() < 2 {
            return pairs;
        }

        let mut aabbs = Vec::with_capacity(polylines.len());

        for pl in polylines {
            aabbs.push(AABB::from_polyline(pl, threshold));
        }

        let mut tree = SpatialAABBTree::new();
        tree.build(&aabbs);

        for i in 0..polylines.len() {
            for j_raw in tree.query_aabb(&aabbs[i]) {
                let j = j_raw as usize;

                if j <= i {
                    continue;
                }

                let mut dist = f64::INFINITY;

                for pt in &polylines[i].get_points() {
                    let d = Self::polyline_point(&polylines[j], pt).2;

                    if d < dist {
                        dist = d;
                    }
                }

                if dist <= threshold {
                    pairs.push((i, j));
                }
            }
        }

        pairs
    }

    /// Index pairs of curves whose endpoints come within threshold of each other.
    pub fn nurbscurves_closest(curves: &[NurbsCurve], threshold: f64) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();

        if threshold < 0.0 || curves.len() < 2 {
            return pairs;
        }

        let mut aabbs = Vec::with_capacity(curves.len());

        for crv in curves {
            aabbs.push(AABB::from_nurbscurve(crv, threshold, false));
        }

        let mut tree = SpatialAABBTree::new();
        tree.build(&aabbs);

        for i in 0..curves.len() {
            for j_raw in tree.query_aabb(&aabbs[i]) {
                let j = j_raw as usize;

                if j <= i {
                    continue;
                }

                let p_start = curves[i].point_at(curves[i].domain_start());
                let p_end = curves[i].point_at(curves[i].domain_end());
                let d_a = Self::curve_point(&curves[j], &p_start, 0.0, 0.0).1;
                let d_b = Self::curve_point(&curves[j], &p_end, 0.0, 0.0).1;

                if d_a.min(d_b) <= threshold {
                    pairs.push((i, j));
                }
            }
        }

        pairs
    }

    /// Return the index pairs of boxes within threshold of each other.
    pub fn boxes_closest(boxes: &[AABB], threshold: f64) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();

        if threshold < 0.0 || boxes.len() < 2 {
            return pairs;
        }

        let mut inflated = Vec::with_capacity(boxes.len());

        for b in boxes {
            let mut inf = *b;
            inf.inflate(threshold);
            inflated.push(inf);
        }

        let mut tree = SpatialAABBTree::new();
        tree.build(&inflated);

        for i in 0..boxes.len() {
            for j_raw in tree.query_aabb(&inflated[i]) {
                let j = j_raw as usize;

                if j <= i {
                    continue;
                }

                if aabb_to_aabb_min_dist(&boxes[i], &boxes[j]) <= threshold {
                    pairs.push((i, j));
                }
            }
        }

        pairs
    }
}
