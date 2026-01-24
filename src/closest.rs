//! Closest point operations for geometry classes.

use crate::line::Line;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
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

        let num_samples = (curve.degree() * 2).max(10);
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

        let max_iterations = 20;
        let step_tolerance = (t1 - t0) * 1e-10;

        let mut t = best_t;

        for _ in 0..max_iterations {
            let pt = curve.point_at(t);
            let tangent = curve.tangent_at(t);

            let delta = Vector::new(
                test_point[0] - pt[0],
                test_point[1] - pt[1],
                test_point[2] - pt[2],
            );

            let f = -delta.dot(&tangent);

            if f.abs() < step_tolerance {
                break;
            }

            let derivs = curve.evaluate(t, 2);
            if derivs.len() < 3 {
                break;
            }

            let d2 = Vector::new(derivs[2][0], derivs[2][1], derivs[2][2]);
            let tangent_mag = tangent.magnitude();
            let df = delta.dot(&d2) - tangent_mag * tangent_mag;

            if df.abs() < 1e-12 {
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

        let u_samples = surface.order(0).max(10);
        let v_samples = surface.order(1).max(10);

        let du = (u1 - u0) / u_samples as f64;
        let dv = (v1 - v0) / v_samples as f64;

        let mut best_u = u0;
        let mut best_v = v0;
        let mut best_dist = f64::INFINITY;

        for i in 0..=u_samples {
            for j in 0..=v_samples {
                let u = u0 + i as f64 * du;
                let v = v0 + j as f64 * dv;
                if let Some(pt) = surface.point_at(u, v) {
                    let dist = pt.distance(test_point, None);
                    if dist < best_dist {
                        best_dist = dist;
                        best_u = u;
                        best_v = v;
                    }
                }
            }
        }

        // Grid-based refinement without derivatives (simplified)
        let refinement_iterations = 5;
        let mut u = best_u;
        let mut v = best_v;
        let mut search_range_u = du * 2.0;
        let mut search_range_v = dv * 2.0;

        for _ in 0..refinement_iterations {
            let mut local_best_u = u;
            let mut local_best_v = v;
            let mut local_best_dist = best_dist;

            let sub_samples = 5;
            let sub_du = search_range_u / sub_samples as f64;
            let sub_dv = search_range_v / sub_samples as f64;

            for i in 0..=sub_samples {
                for j in 0..=sub_samples {
                    let test_u = (u - search_range_u * 0.5 + i as f64 * sub_du).max(u0).min(u1);
                    let test_v = (v - search_range_v * 0.5 + j as f64 * sub_dv).max(v0).min(v1);
                    if let Some(pt) = surface.point_at(test_u, test_v) {
                        let dist = pt.distance(test_point, None);
                        if dist < local_best_dist {
                            local_best_dist = dist;
                            local_best_u = test_u;
                            local_best_v = test_v;
                        }
                    }
                }
            }

            u = local_best_u;
            v = local_best_v;
            best_dist = local_best_dist;
            search_range_u *= 0.5;
            search_range_v *= 0.5;
        }

        let final_dist = surface.point_at(u, v)
            .map(|pt| pt.distance(test_point, None))
            .unwrap_or(f64::INFINITY);

        (u, v, final_dist)
    }
}
