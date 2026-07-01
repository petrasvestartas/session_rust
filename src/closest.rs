//! Closest point operations for geometry classes.

use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::pointcloud::PointCloud;
use crate::polyline::Polyline;
use crate::vector::Vector;

pub struct Closest;

impl Closest {
    /// Find closest point on NURBS curve to test point.
    ///
    /// # Arguments
    /// * `curve` - The NURBS curve
    /// * `test_point` - Point to find closest curve point to
    /// * `t0` - Start of search interval (0.0 means curve start)
    /// * `t1` - End of search interval (0.0 means curve end)
    ///
    /// # Returns
    /// (parameter, distance) of closest point
    pub fn curve_point(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64) -> (f64, f64) {
        if !curve.is_valid() {
            return (0.0, f64::INFINITY);
        }

        let (domain_start, domain_end) = curve.domain();
        let mut t0 = if t0 <= 0.0 { domain_start } else { t0 };
        let mut t1 = if t1 <= 0.0 { domain_end } else { t1 };

        t0 = t0.max(domain_start);
        t1 = t1.min(domain_end);

        // Dense seed grid: sample every knot span several times so the global minimum's
        // basin is captured before Newton refines (matches OCCT's robust initial sampling
        // in GeomAPI_ProjectPointOnCurve).
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

        let max_iterations = 32;
        let step_tolerance = (t1 - t0) * 1e-12;

        let mut t = best_t;

        // Newton on h(t) = (C(t) - P) . C'(t)  (= 0 at a foot of perpendicular).
        // h'(t) = |C'(t)|^2 + (C(t) - P) . C''(t).  Use the RAW derivatives C', C''.
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

            let df = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]
                + rx * d2[0] + ry * d2[1] + rz * d2[2];

            if df.abs() < 1e-14 {
                break;
            }

            let mut dt_step = -f / df;

            if dt_step.abs() > (t1 - t0) * 0.5 {
                dt_step = dt_step.signum() * (t1 - t0) * 0.5;
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

    /// Closest approach between two curves: minimize |C0(u) - C1(v)|^2.
    /// Returns (u, v, distance). Matches OCCT GeomAPI_ExtremaCurveCurve.
    pub fn curve_curve(curve0: &NurbsCurve, curve1: &NurbsCurve) -> (f64, f64, f64) {
        if !curve0.is_valid() || !curve1.is_valid() {
            return (0.0, 0.0, f64::INFINITY);
        }
        let (u0, u1) = curve0.domain();
        let (v0, v1) = curve1.domain();
        let n0 = (curve0.cv_count() * 8).max(40);
        let n1 = (curve1.cv_count() * 8).max(40);

        let p0: Vec<Point> = (0..=n0).map(|i| curve0.point_at(u0 + (u1 - u0) * i as f64 / n0 as f64)).collect();
        let p1: Vec<Point> = (0..=n1).map(|j| curve1.point_at(v0 + (v1 - v0) * j as f64 / n1 as f64)).collect();

        let mut best = f64::INFINITY;
        let mut u = u0; let mut v = v0;
        for i in 0..=n0 {
            for j in 0..=n1 {
                let dx = p0[i][0] - p1[j][0]; let dy = p0[i][1] - p1[j][1]; let dz = p0[i][2] - p1[j][2];
                let d2 = dx*dx + dy*dy + dz*dz;
                if d2 < best {
                    best = d2;
                    u = u0 + (u1 - u0) * i as f64 / n0 as f64;
                    v = v0 + (v1 - v0) * j as f64 / n1 as f64;
                }
            }
        }

        // 2D Newton on f(u,v) = |C0(u) - C1(v)|^2.
        for _ in 0..64 {
            let e0 = curve0.evaluate(u, 2);
            let e1 = curve1.evaluate(v, 2);
            if e0.len() < 3 || e1.len() < 3 { break; }
            let (c0, c0p, c0pp) = (&e0[0], &e0[1], &e0[2]);
            let (c1, c1p, c1pp) = (&e1[0], &e1[1], &e1[2]);
            let rx = c0[0] - c1[0]; let ry = c0[1] - c1[1]; let rz = c0[2] - c1[2];

            let gu = rx*c0p[0] + ry*c0p[1] + rz*c0p[2];
            let gv = -(rx*c1p[0] + ry*c1p[1] + rz*c1p[2]);

            let huu = c0p[0]*c0p[0] + c0p[1]*c0p[1] + c0p[2]*c0p[2]
                    + rx*c0pp[0] + ry*c0pp[1] + rz*c0pp[2];
            let huv = -(c0p[0]*c1p[0] + c0p[1]*c1p[1] + c0p[2]*c1p[2]);
            let hvv = c1p[0]*c1p[0] + c1p[1]*c1p[1] + c1p[2]*c1p[2]
                    - (rx*c1pp[0] + ry*c1pp[1] + rz*c1pp[2]);

            let det = huu*hvv - huv*huv;
            if det.abs() < 1e-14 { break; }
            let mut du = -(hvv*gu - huv*gv) / det;
            let mut dv = -(-huv*gu + huu*gv) / det;

            if du.abs() > (u1 - u0) * 0.5 { du = du.signum() * (u1 - u0) * 0.5; }
            if dv.abs() > (v1 - v0) * 0.5 { dv = dv.signum() * (v1 - v0) * 0.5; }

            u = (u + du).clamp(u0, u1);
            v = (v + dv).clamp(v0, v1);
            if du.abs().max(dv.abs()) < 1e-13 { break; }
        }

        let dist = curve0.point_at(u).distance(&curve1.point_at(v), None);
        (u, v, dist)
    }

    /// Find closest point on line to test point.
    ///
    /// # Arguments
    /// * `line` - The line segment
    /// * `test_point` - Point to find closest line point to
    ///
    /// # Returns
    /// (closest_point, parameter, distance)
    pub fn line_point(line: &Line, test_point: &Point) -> (Point, f64, f64) {
        let start = line.start();
        let end = line.end();

        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];

        let len_sq = dx * dx + dy * dy + dz * dz;

        if len_sq < 1e-20 {
            let dist = start.distance(test_point, None);
            return (start, 0.0, dist);
        }

        let mut t = ((test_point[0] - start[0]) * dx
            + (test_point[1] - start[1]) * dy
            + (test_point[2] - start[2]) * dz)
            / len_sq;

        t = t.max(0.0).min(1.0);

        let closest = Point::new(start[0] + t * dx, start[1] + t * dy, start[2] + t * dz);

        let dist = closest.distance(test_point, None);

        (closest, t, dist)
    }

    /// Find closest point on polyline to test point.
    ///
    /// # Arguments
    /// * `polyline` - The polyline
    /// * `test_point` - Point to find closest polyline point to
    ///
    /// # Returns
    /// (closest_point, parameter, distance)
    pub fn polyline_point(polyline: &Polyline, test_point: &Point) -> (Point, f64, f64) {
        let points = polyline.get_points();

        if points.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), 0.0, f64::INFINITY);
        }

        if points.len() == 1 {
            let dist = points[0].distance(test_point, None);
            return (points[0].clone(), 0.0, dist);
        }

        let mut best_point = points[0].clone();
        let mut best_param = 0.0;
        let mut best_dist = f64::INFINITY;

        let mut cumulative_length = 0.0;
        let total_length = polyline.length();

        for i in 0..points.len() - 1 {
            let segment = Line::from_points(&points[i], &points[i + 1]);
            let (closest, t, dist) = Self::line_point(&segment, test_point);

            if dist < best_dist {
                best_dist = dist;
                best_point = closest;
                let segment_length = segment.length();
                if total_length > 1e-20 {
                    best_param = (cumulative_length + t * segment_length) / total_length;
                } else {
                    best_param = i as f64 / (points.len() - 1) as f64;
                }
            }

            cumulative_length += segment.length();
        }

        (best_point, best_param, best_dist)
    }

    /// Find closest point on NURBS surface to test point.
    ///
    /// # Arguments
    /// * `surface` - The NURBS surface
    /// * `test_point` - Point to find closest surface point to
    /// * `u0`, `u1` - U parameter search interval (0.0 means use surface domain)
    /// * `v0`, `v1` - V parameter search interval (0.0 means use surface domain)
    ///
    /// # Returns
    /// (u_param, v_param, distance)
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

        let (domain_u0, domain_u1) = surface.domain(0).unwrap_or((0.0, 1.0));
        let (domain_v0, domain_v1) = surface.domain(1).unwrap_or((0.0, 1.0));

        let mut u0 = if u0 <= 0.0 { domain_u0 } else { u0 };
        let mut u1 = if u1 <= 0.0 { domain_u1 } else { u1 };
        let mut v0 = if v0 <= 0.0 { domain_v0 } else { v0 };
        let mut v1 = if v1 <= 0.0 { domain_v1 } else { v1 };

        u0 = u0.max(domain_u0);
        u1 = u1.min(domain_u1);
        v0 = v0.max(domain_v0);
        v1 = v1.min(domain_v1);

        let u_samples = surface.order(0).max(10) as usize;
        let v_samples = surface.order(1).max(10) as usize;

        let du_param = (u1 - u0) / u_samples as f64;
        let dv_param = (v1 - v0) / v_samples as f64;

        let mut best_u = u0;
        let mut best_v = v0;
        let mut best_dist = f64::INFINITY;

        for i in 0..=u_samples {
            for j in 0..=v_samples {
                let uu = u0 + i as f64 * du_param;
                let vv = v0 + j as f64 * dv_param;
                if let Some(pt) = surface.point_at(uu, vv) {
                    let dist = pt.distance(test_point, None);
                    if dist < best_dist {
                        best_dist = dist;
                        best_u = uu;
                        best_v = vv;
                    }
                }
            }
        }

        let max_iterations = 20;
        let step_tolerance = (u1 - u0).min(v1 - v0) * 1e-10;

        let mut u = best_u;
        let mut v = best_v;

        for _ in 0..max_iterations {
            let derivs = surface.evaluate(u, v, 1);
            if derivs.len() < 3 { break; }

            if let Some(pt) = surface.point_at(u, v) {
                let du_vec = &derivs[2];  // evaluate returns [S, Sv, Su, ...]
                let dv_vec = &derivs[1];

                let delta = Vector::new(
                    test_point[0] - pt[0],
                    test_point[1] - pt[1],
                    test_point[2] - pt[2],
                );

                let fu = -delta.dot(du_vec);
                let fv = -delta.dot(dv_vec);

                if fu.abs() < step_tolerance && fv.abs() < step_tolerance { break; }

                let duu = du_vec.dot(du_vec);
                let dvv = dv_vec.dot(dv_vec);
                let duv = du_vec.dot(dv_vec);

                let det = duu * dvv - duv * duv;
                if det.abs() < 1e-12 { break; }

                let mut du_step = (dvv * fu - duv * fv) / det;
                let mut dv_step = (duu * fv - duv * fu) / det;

                let max_step = (u1 - u0).min(v1 - v0) * 0.5;
                if du_step.abs() > max_step { du_step = du_step.signum() * max_step; }
                if dv_step.abs() > max_step { dv_step = dv_step.signum() * max_step; }

                u -= du_step;
                v -= dv_step;

                u = u.clamp(u0, u1);
                v = v.clamp(v0, v1);

                if du_step.abs() < step_tolerance && dv_step.abs() < step_tolerance { break; }
            } else {
                break;
            }
        }

        let final_dist = surface.point_at(u, v)
            .map(|pt| pt.distance(test_point, None))
            .unwrap_or(f64::INFINITY);

        (u, v, final_dist)
    }

    /// Project a 3D curve onto a surface (curve pullback).
    ///
    /// Samples the curve, inverts each sample with warm-started windowed
    /// point inversion, unwraps across seams of closed surfaces, refines
    /// adaptively, and refits seam-split UV pcurves (x=u, y=v, z=0) with
    /// domain [0, 1]. Returns an empty list if the curve does not lie on the
    /// surface within the rejection tolerance.
    ///
    /// # Arguments
    /// * `surface` - The surface to project onto
    /// * `curve` - The 3D curve to pull back
    /// * `t0`, `t1` - Curve sub-domain (0.0 means use the curve domain end)
    /// * `tolerance` - Fit deviation budget (defaults to a trace-step heuristic)
    ///
    /// # Returns
    /// Seam-split UV pcurves
    pub fn surface_curve(surface: &NurbsSurface, curve: &NurbsCurve, t0: f64, t1: f64, tolerance: Option<f64>) -> Vec<NurbsCurve> {
        use crate::nurbsknot::CurveNurbsKnotStyle;

        if !surface.is_valid() || !curve.is_valid() {
            return vec![];
        }

        let (u0, u1) = match surface.domain(0) { Some(d) => d, None => return vec![] };
        let (v0, v1) = match surface.domain(1) { Some(d) => d, None => return vec![] };
        let range_u = u1 - u0;
        let range_v = v1 - v0;
        let closed_u = surface.is_closed(0);
        let closed_v = surface.is_closed(1);

        let (ct0, ct1) = curve.domain();
        let mut t0 = if t0 <= 0.0 { ct0 } else { t0 };
        let mut t1 = if t1 <= 0.0 { ct1 } else { t1 };
        t0 = t0.max(ct0);
        t1 = t1.min(ct1);
        if t1 - t0 < 1e-14 {
            return vec![];
        }

        let spans_u = surface.get_span_vector(0);
        let spans_v = surface.get_span_vector(1);
        let nu = spans_u.len().saturating_sub(1).max(1) * 4;
        let nv = spans_v.len().saturating_sub(1).max(1) * 4;
        let du = range_u / nu as f64;
        let dv = range_v / nv as f64;

        let mu = (u0 + u1) * 0.5;
        let mv = (v0 + v1) * 0.5;
        let pmid = surface.point_at(mu, mv).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let wu_probe = (mu + du).min(u1);
        let wv_probe = (mv + dv).min(v1);
        let uv_to_3d_u = pmid.distance(&surface.point_at(wu_probe, mv).unwrap_or(Point::new(0.0, 0.0, 0.0)), None) / du;
        let uv_to_3d_v = pmid.distance(&surface.point_at(mu, wv_probe).unwrap_or(Point::new(0.0, 0.0, 0.0)), None) / dv;
        let mut uv_to_3d = uv_to_3d_u.max(uv_to_3d_v);
        let mut uv_to_3d_min = uv_to_3d_u.min(uv_to_3d_v);
        if uv_to_3d < 1e-10 {
            uv_to_3d = 1.0;
        }
        if uv_to_3d_min < 1e-10 {
            uv_to_3d_min = 1.0;
        }

        let step = du.min(dv) * 0.25;
        let fit_tol = match tolerance {
            Some(tol) if tol > 0.0 => tol,
            _ => step * (uv_to_3d + uv_to_3d_min) * 0.5,
        };
        let reject_tol = fit_tol * 100.0;
        // Absolute "lies on the surface" gate (fraction of the surface size).
        // Used to (a) reject a curve that nowhere touches the surface and
        // (b) stop bisecting stick-out portions of a curve that extends past
        // the face, both of which otherwise burn a full 4096-sample bisection.
        let c00 = surface.point_at(u0, v0).unwrap_or(crate::point::Point::new(0.0, 0.0, 0.0));
        let c11 = surface.point_at(u1, v1).unwrap_or(crate::point::Point::new(0.0, 0.0, 0.0));
        let mut corner_diag = c00.distance(&c11, None);
        if corner_diag < 1e-12 {
            corner_diag = range_u.max(range_v);
        }
        let on_surf_tol = corner_diag * 0.05;

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

        let invert_near = |pt: &Point, up: f64, vp: f64, wu: f64, wv: f64| -> (f64, f64, f64) {
            // Windowed inversion with seam-aware candidate windows
            let mut u_centers = vec![up];
            if closed_u {
                if up - wu < u0 { u_centers.push(up + range_u); }
                if up + wu > u1 { u_centers.push(up - range_u); }
            }
            let mut v_centers = vec![vp];
            if closed_v {
                if vp - wv < v0 { v_centers.push(vp + range_v); }
                if vp + wv > v1 { v_centers.push(vp - range_v); }
            }
            let mut best = (up, vp, f64::INFINITY);
            for &uc in &u_centers {
                for &vc in &v_centers {
                    let wu0 = (uc - wu).max(u0);
                    let wu1 = (uc + wu).min(u1);
                    let wv0 = (vc - wv).max(v0);
                    let wv1 = (vc + wv).min(v1);
                    if wu1 - wu0 < 1e-14 || wv1 - wv0 < 1e-14 { continue; }
                    let res = Closest::surface_point(surface, pt, wu0, wu1, wv0, wv1);
                    if res.2 < best.2 { best = res; }
                    if best.2 < fit_tol * 0.01 { break; }
                }
            }
            best
        };

        let unwrap_to = |prev_u: f64, prev_v: f64, mut u: f64, mut v: f64| -> (f64, f64) {
            if closed_u {
                while u - prev_u > range_u * 0.5 { u -= range_u; }
                while u - prev_u < -range_u * 0.5 { u += range_u; }
            }
            if closed_v {
                while v - prev_v > range_v * 0.5 { v -= range_v; }
                while v - prev_v < -range_v * 0.5 { v += range_v; }
            }
            (u, v)
        };

        // 1. Initial samples with warm-started inversion
        let n0 = 16.max(4 * curve.span_count());
        let mut samples: Vec<[f64; 4]> = Vec::new(); // [t, u_unwrapped, v_unwrapped, residual]
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
                let wu = du.max(dv) * 2.0 + (prev[1] - prev2[1]).abs();
                let wv = du.max(dv) * 2.0 + (prev[2] - prev2[2]).abs();
                let (mut ru, mut rv, mut rd) = invert_near(&pt, wrap_u(prev[1]), wrap_v(prev[2]), wu, wv);
                if rd > reject_tol {
                    let full = Closest::surface_point(surface, &pt, 0.0, 0.0, 0.0, 0.0);
                    ru = full.0;
                    rv = full.1;
                    rd = full.2;
                }
                let (uu, vv) = unwrap_to(prev[1], prev[2], ru, rv);
                (uu, vv, rd)
            };
            samples.push([t, uu, vv, rd]);
            max_residual = max_residual.max(rd);
            min_residual = min_residual.min(rd);
        }

        // Reject a curve that nowhere lies on the surface (no sample touches it).
        if max_residual > reject_tol || min_residual > on_surf_tol {
            return vec![];
        }

        // 2. Adaptive bisection where the lifted UV midpoint strays from the curve
        let mut depth = 0;
        while depth < 8 {
            let mut inserted = 0;
            let mut i = 0;
            while i < samples.len() - 1 {
                let a = samples[i];
                let b = samples[i + 1];
                let tm = (a[0] + b[0]) * 0.5;
                let um = (a[1] + b[1]) * 0.5;
                let vm = (a[2] + b[2]) * 0.5;
                let pm = curve.point_at(tm);
                let lift = surface.point_at(wrap_u(um), wrap_v(vm)).unwrap_or(Point::new(0.0, 0.0, 0.0));
                if lift.distance(&pm, None) > fit_tol && samples.len() < 4096 {
                    let wu = (b[1] - a[1]).abs().max(du) * 1.0;
                    let wv = (b[2] - a[2]).abs().max(dv) * 1.0;
                    let (ru, rv, rd) = invert_near(&pm, wrap_u(um), wrap_v(vm), wu, wv);
                    if rd > on_surf_tol {
                        // Midpoint is off the surface: stick-out portion of a
                        // curve that extends past the face, not a curvature
                        // stray. Do not refine it (avoids unbounded bisection).
                        i += 1;
                        continue;
                    }
                    let (uu, vv) = unwrap_to(a[1], a[2], ru, rv);
                    samples.insert(i + 1, [tm, uu, vv, rd]);
                    inserted += 1;
                    i += 2;
                } else {
                    i += 1;
                }
            }
            if inserted == 0 {
                break;
            }
            depth += 1;
        }

        let mut pts: Vec<[f64; 2]> = samples.iter().map(|s| [s[1], s[2]]).collect();

        // 3. Closed-loop closure and seam-crossing split (same scheme as surface_plane_uv)
        let p_first = curve.point_at(samples[0][0]);
        let p_last = curve.point_at(samples[samples.len() - 1][0]);
        let is_loop = p_first.distance(&p_last, None) < fit_tol * 4.0 && pts.len() >= 6;

        let mut closure_du = 0.0;
        let mut closure_dv = 0.0;
        if is_loop {
            pts.pop();
            let mut du_j = pts[0][0] - pts[pts.len() - 1][0];
            let mut dv_j = pts[0][1] - pts[pts.len() - 1][1];
            if closed_u {
                while du_j > range_u * 0.5 { du_j -= range_u; }
                while du_j < -range_u * 0.5 { du_j += range_u; }
            }
            if closed_v {
                while dv_j > range_v * 0.5 { dv_j -= range_v; }
                while dv_j < -range_v * 0.5 { dv_j += range_v; }
            }
            closure_du = (pts[pts.len() - 1][0] + du_j) - pts[0][0];
            closure_dv = (pts[pts.len() - 1][1] + dv_j) - pts[0][1];
            pts.push([pts[0][0] + closure_du, pts[0][1] + closure_dv]);
        }

        let mut out_pts: Vec<[f64; 2]> = vec![pts[0]];
        let mut cross_idx: Vec<usize> = Vec::new();
        for i in 1..pts.len() {
            let pa = pts[i - 1];
            let pb = pts[i];
            let mut crossings: Vec<(f64, i32, f64)> = Vec::new();
            if closed_u && (pb[0] - pa[0]).abs() > 1e-15 {
                let k0 = ((pa[0] - u0) / range_u).floor() as i64;
                let k1 = ((pb[0] - u0) / range_u).floor() as i64;
                for k in (k0.min(k1) + 1)..=(k0.max(k1)) {
                    let l = u0 + k as f64 * range_u;
                    let t = (l - pa[0]) / (pb[0] - pa[0]);
                    if 0.0 < t && t < 1.0 { crossings.push((t, 0, l)); }
                }
            }
            if closed_v && (pb[1] - pa[1]).abs() > 1e-15 {
                let k0 = ((pa[1] - v0) / range_v).floor() as i64;
                let k1 = ((pb[1] - v0) / range_v).floor() as i64;
                for k in (k0.min(k1) + 1)..=(k0.max(k1)) {
                    let l = v0 + k as f64 * range_v;
                    let t = (l - pa[1]) / (pb[1] - pa[1]);
                    if 0.0 < t && t < 1.0 { crossings.push((t, 1, l)); }
                }
            }
            crossings.sort_by(|a, b| a.partial_cmp(b).unwrap());
            for &(t, axis, l) in &crossings {
                let mut cu = pa[0] + (pb[0] - pa[0]) * t;
                let mut cv_ = pa[1] + (pb[1] - pa[1]) * t;
                if axis == 0 {
                    cu = l;
                } else {
                    cv_ = l;
                }
                out_pts.push([cu, cv_]);
                cross_idx.push(out_pts.len() - 1);
            }
            out_pts.push([pb[0], pb[1]]);
            // An interior sample sitting exactly on a seam level is a crossing
            if i < pts.len() - 1 {
                let mut on_seam = false;
                if closed_u {
                    let k = ((pb[0] - u0) / range_u).round();
                    let l = u0 + k * range_u;
                    if (pb[0] - l).abs() < range_u * 1e-9 && (pb[0] - pa[0]).abs() > range_u * 1e-9 {
                        out_pts.last_mut().unwrap()[0] = l;
                        on_seam = true;
                    }
                }
                if closed_v {
                    let k = ((pb[1] - v0) / range_v).round();
                    let l = v0 + k * range_v;
                    if (pb[1] - l).abs() < range_v * 1e-9 && (pb[1] - pa[1]).abs() > range_v * 1e-9 {
                        out_pts.last_mut().unwrap()[1] = l;
                        on_seam = true;
                    }
                }
                if on_seam {
                    cross_idx.push(out_pts.len() - 1);
                }
            }
        }

        let wrap_drift = closure_du.abs() > range_u * 0.5 || closure_dv.abs() > range_v * 0.5;
        let mut pieces: Vec<(Vec<[f64; 2]>, bool)> = Vec::new();
        if cross_idx.is_empty() {
            pieces.push((out_pts.clone(), is_loop && !wrap_drift));
        } else if is_loop {
            for w in cross_idx.windows(2) {
                let (a, b) = (w[0], w[1]);
                pieces.push((out_pts[a..=b].to_vec(), false));
            }
            let mut wrap_piece: Vec<[f64; 2]> = out_pts[cross_idx[cross_idx.len() - 1]..].to_vec();
            for p in &out_pts[1..=cross_idx[0]] {
                wrap_piece.push([p[0] + closure_du, p[1] + closure_dv]);
            }
            pieces.push((wrap_piece, false));
        } else {
            let mut bounds: Vec<usize> = vec![0];
            for &c in &cross_idx { bounds.push(c); }
            bounds.push(out_pts.len() - 1);
            for w in bounds.windows(2) {
                let (a, b) = (w[0], w[1]);
                if b > a { pieces.push((out_pts[a..=b].to_vec(), false)); }
            }
        }

        // 4. Refit each piece as a UV pcurve
        let mut result: Vec<NurbsCurve> = Vec::new();
        for (mut piece_pts, piece_loop) in pieces {
            if piece_pts.len() < 2 {
                continue;
            }
            let mid = piece_pts[piece_pts.len() / 2];
            if closed_u {
                let k_u = ((mid[0] - u0) / range_u).floor();
                if k_u != 0.0 {
                    for p in piece_pts.iter_mut() { p[0] -= k_u * range_u; }
                }
            }
            if closed_v {
                let k_v = ((mid[1] - v0) / range_v).floor();
                if k_v != 0.0 {
                    for p in piece_pts.iter_mut() { p[1] -= k_v * range_v; }
                }
            }

            let pts_uv: Vec<Point> = piece_pts.iter().map(|p| Point::new(p[0], p[1], 0.0)).collect();
            let mp = pts_uv.len();
            let fit_tol_uv = step;
            let mut total_turning = 0.0f64;
            for i in 1..(mp - 1) {
                let dx1 = pts_uv[i][0] - pts_uv[i - 1][0];
                let dy1 = pts_uv[i][1] - pts_uv[i - 1][1];
                let dx2 = pts_uv[i + 1][0] - pts_uv[i][0];
                let dy2 = pts_uv[i + 1][1] - pts_uv[i][1];
                let l1 = f64::hypot(dx1, dy1);
                let l2 = f64::hypot(dx2, dy2);
                if l1 > 1e-14 && l2 > 1e-14 {
                    let c = ((dx1 * dx2 + dy1 * dy2) / (l1 * l2)).max(-1.0).min(1.0);
                    total_turning += c.acos();
                }
            }

            let mut chords = vec![0.0f64; mp];
            let mut total_len = 0.0f64;
            for i in 1..mp {
                total_len += pts_uv[i].distance(&pts_uv[i - 1], None);
                chords[i] = total_len;
            }
            if piece_loop && mp > 1 {
                total_len += pts_uv[0].distance(&pts_uv[mp - 1], None);
            }
            if total_len > 1e-14 {
                for i in 1..mp { chords[i] /= total_len; }
            }

            let mut target_cvs = 8_i32.max((total_turning / 0.5) as i32 + 6);
            let max_cvs = (mp as i32) - 1;
            let mut pcurve = NurbsCurve::new(3, false, 4, 0);
            for _ in 0..5 {
                if target_cvs > max_cvs { break; }
                pcurve = NurbsCurve::create_fitted(&pts_uv, target_cvs as usize, 3, piece_loop);
                if !pcurve.is_valid() { break; }
                let (ft0, ft1) = pcurve.domain();
                let mut max_dev = 0.0f64;
                for i in 0..mp {
                    let t = ft0 + (ft1 - ft0) * chords[i];
                    max_dev = max_dev.max(pcurve.point_at(t).distance(&pts_uv[i], None));
                }
                if max_dev < fit_tol_uv { break; }
                target_cvs = (target_cvs * 2).min(max_cvs);
            }

            if !pcurve.is_valid() {
                pcurve = if piece_loop {
                    NurbsCurve::create_interpolated(&pts_uv, CurveNurbsKnotStyle::ChordPeriodic)
                } else {
                    NurbsCurve::create_interpolated(&pts_uv, CurveNurbsKnotStyle::Chord)
                };
            }
            if !pcurve.is_valid() && pts_uv.len() >= 2 {
                // Last resort: a degree-1 polyline through the inverted UV samples
                // (always valid; lies on the surface piecewise-linearly in UV).
                pcurve = NurbsCurve::create(false, 1, &pts_uv);
            }
            if !pcurve.is_valid() {
                continue;
            }

            pcurve.set_domain(0.0, 1.0);
            result.push(pcurve);
        }

        result
    }

    fn closest_point_on_triangle(p: &Point, a: &Point, b: &Point, c: &Point) -> Point {
        let (abx, aby, abz) = (b[0]-a[0], b[1]-a[1], b[2]-a[2]);
        let (acx, acy, acz) = (c[0]-a[0], c[1]-a[1], c[2]-a[2]);
        let (apx, apy, apz) = (p[0]-a[0], p[1]-a[1], p[2]-a[2]);

        let d1 = abx*apx + aby*apy + abz*apz;
        let d2 = acx*apx + acy*apy + acz*apz;
        if d1 <= 0.0 && d2 <= 0.0 { return a.clone(); }

        let (bpx, bpy, bpz) = (p[0]-b[0], p[1]-b[1], p[2]-b[2]);
        let d3 = abx*bpx + aby*bpy + abz*bpz;
        let d4 = acx*bpx + acy*bpy + acz*bpz;
        if d3 >= 0.0 && d4 <= d3 { return b.clone(); }

        let vc = d1*d4 - d3*d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            return Point::new(a[0] + v*abx, a[1] + v*aby, a[2] + v*abz);
        }

        let (cpx, cpy, cpz) = (p[0]-c[0], p[1]-c[1], p[2]-c[2]);
        let d5 = abx*cpx + aby*cpy + abz*cpz;
        let d6 = acx*cpx + acy*cpy + acz*cpz;
        if d6 >= 0.0 && d5 <= d6 { return c.clone(); }

        let vb = d5*d2 - d1*d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            return Point::new(a[0] + w*acx, a[1] + w*acy, a[2] + w*acz);
        }

        let va = d3*d6 - d5*d4;
        if va <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0 {
            let w = (d4-d3) / ((d4-d3) + (d5-d6));
            return Point::new(b[0] + w*(c[0]-b[0]), b[1] + w*(c[1]-b[1]), b[2] + w*(c[2]-b[2]));
        }

        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        Point::new(a[0] + abx*v + acx*w, a[1] + aby*v + acy*w, a[2] + abz*v + acz*w)
    }

    pub fn mesh_point(mesh: &Mesh, test_point: &Point) -> (Point, usize, f64) {
        if mesh.number_of_faces() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let (vertices, faces) = mesh.to_vertices_and_faces();
        let mut sorted_face_keys: Vec<usize> = mesh.face.keys().copied().collect();
        sorted_face_keys.sort();

        let mut best_point = Point::new(0.0, 0.0, 0.0);
        let mut best_face_key: usize = 0;
        let mut best_dist = f64::INFINITY;

        for (fi, fv) in faces.iter().enumerate() {
            if fv.len() < 3 { continue; }
            let v0 = &vertices[fv[0]];
            for j in 1..fv.len() - 1 {
                let v1 = &vertices[fv[j]];
                let v2 = &vertices[fv[j + 1]];
                let cp = Self::closest_point_on_triangle(test_point, v0, v1, v2);
                let dist = cp.distance(test_point, None);
                if dist < best_dist {
                    best_dist = dist;
                    best_point = cp;
                    best_face_key = sorted_face_keys[fi];
                }
            }
        }

        (best_point, best_face_key, best_dist)
    }

    pub fn mesh_point_aabb(mesh: &Mesh, test_point: &Point) -> (Point, usize, f64) {
        if mesh.number_of_faces() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let (vertices, faces) = mesh.to_vertices_and_faces();
        let mut sorted_face_keys: Vec<usize> = mesh.face.keys().copied().collect();
        sorted_face_keys.sort();

        let mut tris: Vec<(Point, Point, Point)> = Vec::new();
        let mut tri_face_idx: Vec<usize> = Vec::new();
        for (fi, fv) in faces.iter().enumerate() {
            if fv.len() < 3 { continue; }
            let v0 = &vertices[fv[0]];
            for j in 1..fv.len() - 1 {
                tris.push((v0.clone(), vertices[fv[j]].clone(), vertices[fv[j + 1]].clone()));
                tri_face_idx.push(fi);
            }
        }

        if tris.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        // Build AABB boxes for each triangle: (cx, cy, cz, hx, hy, hz)
        let boxes: Vec<[f64; 6]> = tris.iter().map(|(v0, v1, v2)| {
            let lx = v0[0].min(v1[0]).min(v2[0]);
            let ly = v0[1].min(v1[1]).min(v2[1]);
            let lz = v0[2].min(v1[2]).min(v2[2]);
            let hx = v0[0].max(v1[0]).max(v2[0]);
            let hy = v0[1].max(v1[1]).max(v2[1]);
            let hz = v0[2].max(v1[2]).max(v2[2]);
            [(lx+hx)*0.5, (ly+hy)*0.5, (lz+hz)*0.5,
             (hx-lx)*0.5, (hy-ly)*0.5, (hz-lz)*0.5]
        }).collect();

        // SpatialAABBTree nodes: (aabb, right_child, object_id)
        let mut nodes: Vec<([f64; 6], i32, i32)> = Vec::new();

        fn build_node(ids: &mut [usize], boxes: &[[f64; 6]], nodes: &mut Vec<([f64; 6], i32, i32)>) {
            let ni = nodes.len();
            nodes.push(([0.0; 6], -1, -1));
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            for &i in ids.iter() {
                let b = &boxes[i];
                for a in 0..3 {
                    lo[a] = lo[a].min(b[a] - b[a + 3]);
                    hi[a] = hi[a].max(b[a] + b[a + 3]);
                }
            }
            nodes[ni].0 = [
                (lo[0]+hi[0])*0.5, (lo[1]+hi[1])*0.5, (lo[2]+hi[2])*0.5,
                (hi[0]-lo[0])*0.5, (hi[1]-lo[1])*0.5, (hi[2]-lo[2])*0.5,
            ];
            if ids.len() == 1 {
                nodes[ni].2 = ids[0] as i32;
                return;
            }
            let dx = hi[0]-lo[0]; let dy = hi[1]-lo[1]; let dz = hi[2]-lo[2];
            let axis = if dx >= dy && dx >= dz { 0 } else if dy >= dz { 1 } else { 2 };
            let mid = ids.len() / 2;
            ids.select_nth_unstable_by(mid, |&a, &b| {
                boxes[a][axis].partial_cmp(&boxes[b][axis]).unwrap()
            });
            let (left_ids, right_ids) = ids.split_at_mut(mid);
            build_node(left_ids, boxes, nodes);
            nodes[ni].1 = nodes.len() as i32;
            build_node(right_ids, boxes, nodes);
        }

        let mut ids: Vec<usize> = (0..tris.len()).collect();
        build_node(&mut ids, &boxes, &mut nodes);

        fn aabb_min_dist(aabb: &[f64; 6], pt: &Point) -> f64 {
            let dx = (pt[0] - aabb[0]).abs() - aabb[3];
            let dy = (pt[1] - aabb[1]).abs() - aabb[4];
            let dz = (pt[2] - aabb[2]).abs() - aabb[5];
            let dx = dx.max(0.0); let dy = dy.max(0.0); let dz = dz.max(0.0);
            (dx*dx + dy*dy + dz*dz).sqrt()
        }

        let mut best_point = Point::new(0.0, 0.0, 0.0);
        let mut best_face_key: usize = 0;
        let mut best_dist = f64::INFINITY;

        fn dfs(
            ni: usize, nodes: &[([f64; 6], i32, i32)],
            tris: &[(Point, Point, Point)], tri_face_idx: &[usize],
            sorted_face_keys: &[usize], test_point: &Point,
            best_point: &mut Point, best_face_key: &mut usize, best_dist: &mut f64,
        ) {
            let (ref aabb, right, obj) = nodes[ni];
            if aabb_min_dist(aabb, test_point) >= *best_dist { return; }
            if obj >= 0 {
                let (ref v0, ref v1, ref v2) = tris[obj as usize];
                let cp = Closest::closest_point_on_triangle(test_point, v0, v1, v2);
                let d = cp.distance(test_point, None);
                if d < *best_dist {
                    *best_dist = d;
                    *best_point = cp;
                    *best_face_key = sorted_face_keys[tri_face_idx[obj as usize]];
                }
                return;
            }
            let left = ni + 1;
            let right = right as usize;
            let ld = aabb_min_dist(&nodes[left].0, test_point);
            let rd = aabb_min_dist(&nodes[right].0, test_point);
            if ld <= rd {
                if ld < *best_dist { dfs(left, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
                if rd < *best_dist { dfs(right, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
            } else {
                if rd < *best_dist { dfs(right, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
                if ld < *best_dist { dfs(left, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
            }
        }

        dfs(0, &nodes, &tris, &tri_face_idx, &sorted_face_keys, test_point,
            &mut best_point, &mut best_face_key, &mut best_dist);

        (best_point, best_face_key, best_dist)
    }

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

    pub fn pointcloud_point_kdtree(cloud: &PointCloud, test_point: &Point) -> (Point, usize, f64) {
        if cloud.point_count() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }
        use crate::spatial_kdtree::SpatialKDTree;
        let pts: Vec<Point> = (0..cloud.point_count()).map(|i| cloud.get_point(i)).collect();
        let kd = SpatialKDTree::new(pts);
        let (idx, dist) = kd.nearest(test_point);
        (cloud.get_point(idx), idx, dist)
    }

    fn build_raw_boxes(aabbs: &[[f64; 6]]) -> Vec<([f64; 6], i32, i32)> {
        let mut nodes: Vec<([f64; 6], i32, i32)> = Vec::new();
        fn build(ids: &mut [usize], boxes: &[[f64; 6]], nodes: &mut Vec<([f64; 6], i32, i32)>) {
            let ni = nodes.len();
            nodes.push(([0.0; 6], -1, -1));
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            for &i in ids.iter() {
                let b = &boxes[i];
                for a in 0..3 { lo[a] = lo[a].min(b[a] - b[a + 3]); hi[a] = hi[a].max(b[a] + b[a + 3]); }
            }
            nodes[ni].0 = [(lo[0]+hi[0])*0.5, (lo[1]+hi[1])*0.5, (lo[2]+hi[2])*0.5,
                           (hi[0]-lo[0])*0.5, (hi[1]-lo[1])*0.5, (hi[2]-lo[2])*0.5];
            if ids.len() == 1 { nodes[ni].2 = ids[0] as i32; return; }
            let dx = hi[0]-lo[0]; let dy = hi[1]-lo[1]; let dz = hi[2]-lo[2];
            let axis = if dx >= dy && dx >= dz { 0 } else if dy >= dz { 1 } else { 2 };
            let mid = ids.len() / 2;
            ids.select_nth_unstable_by(mid, |&a, &b| boxes[a][axis].partial_cmp(&boxes[b][axis]).unwrap());
            let (left_ids, right_ids) = ids.split_at_mut(mid);
            build(left_ids, boxes, nodes);
            nodes[ni].1 = nodes.len() as i32;
            build(right_ids, boxes, nodes);
        }
        let mut ids: Vec<usize> = (0..aabbs.len()).collect();
        build(&mut ids, aabbs, &mut nodes);
        nodes
    }

    fn query_raw_nodes(ni: usize, nodes: &[([f64; 6], i32, i32)], query: &[f64; 6], result: &mut Vec<usize>) {
        let (ref aabb, right, obj) = nodes[ni];
        let overlaps = (0..3).all(|a| (aabb[a] - query[a]).abs() <= aabb[a+3] + query[a+3]);
        if !overlaps { return; }
        if obj >= 0 { result.push(obj as usize); return; }
        Self::query_raw_nodes(ni + 1, nodes, query, result);
        Self::query_raw_nodes(right as usize, nodes, query, result);
    }

    fn aabb_to_aabb_min_dist(a: &[f64; 6], b: &[f64; 6]) -> f64 {
        let dx = ((a[0] - b[0]).abs() - a[3] - b[3]).max(0.0);
        let dy = ((a[1] - b[1]).abs() - a[4] - b[4]).max(0.0);
        let dz = ((a[2] - b[2]).abs() - a[5] - b[5]).max(0.0);
        (dx*dx + dy*dy + dz*dz).sqrt()
    }

    pub fn lines_closest(lines: &[Line], threshold: f64) -> Vec<(usize, usize)> {
        use crate::aabb::AABB;
        if lines.len() < 2 { return Vec::new(); }
        let raw: Vec<[f64; 6]> = lines.iter().map(|ln| {
            let b = AABB::from_line(ln, threshold);
            [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]
        }).collect();
        let nodes = Self::build_raw_boxes(&raw);
        let mut pairs = Vec::new();
        for i in 0..lines.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let (_, _, d_a) = Self::line_point(&lines[j], &lines[i].start());
                let (_, _, d_b) = Self::line_point(&lines[j], &lines[i].end());
                let (_, _, d_c) = Self::line_point(&lines[i], &lines[j].start());
                let (_, _, d_d) = Self::line_point(&lines[i], &lines[j].end());
                if d_a.min(d_b).min(d_c).min(d_d) <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }

    pub fn polylines_closest(polylines: &[Polyline], threshold: f64) -> Vec<(usize, usize)> {
        use crate::aabb::AABB;
        if polylines.len() < 2 { return Vec::new(); }
        let raw: Vec<[f64; 6]> = polylines.iter().map(|pl| {
            let b = AABB::from_polyline(pl, threshold);
            [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]
        }).collect();
        let nodes = Self::build_raw_boxes(&raw);
        let mut pairs = Vec::new();
        for i in 0..polylines.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let pts_a = polylines[i].get_points();
                let dist = pts_a.iter().map(|pt| Self::polyline_point(&polylines[j], pt).2)
                    .fold(f64::INFINITY, f64::min);
                if dist <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }

    pub fn nurbscurves_closest(curves: &[NurbsCurve], threshold: f64) -> Vec<(usize, usize)> {
        use crate::aabb::AABB;
        if curves.len() < 2 { return Vec::new(); }
        let raw: Vec<[f64; 6]> = curves.iter().map(|crv| {
            let b = AABB::from_nurbscurve(crv, threshold, false);
            [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]
        }).collect();
        let nodes = Self::build_raw_boxes(&raw);
        let mut pairs = Vec::new();
        for i in 0..curves.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let (t0, t1) = curves[i].domain();
                let p_start = curves[i].point_at(t0);
                let p_end = curves[i].point_at(t1);
                let (_, d_a) = Self::curve_point(&curves[j], &p_start, 0.0, 0.0);
                let (_, d_b) = Self::curve_point(&curves[j], &p_end, 0.0, 0.0);
                if d_a.min(d_b) <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }

    pub fn boxes_closest(boxes: &[crate::aabb::AABB], threshold: f64) -> Vec<(usize, usize)> {
        if boxes.len() < 2 { return Vec::new(); }
        let raw_orig: Vec<[f64; 6]> = boxes.iter().map(|b| [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]).collect();
        let raw_inf: Vec<[f64; 6]> = boxes.iter().map(|b| [b.cx, b.cy, b.cz, b.hx + threshold, b.hy + threshold, b.hz + threshold]).collect();
        let nodes = Self::build_raw_boxes(&raw_inf);
        let mut pairs = Vec::new();
        for i in 0..boxes.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw_inf[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let dist = Self::aabb_to_aabb_min_dist(&raw_orig[i], &raw_orig[j]);
                if dist <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }
}
