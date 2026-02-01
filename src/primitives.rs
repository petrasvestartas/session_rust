use crate::nurbscurve::NurbsCurve;
use crate::point::Point;
use crate::tolerance::Tolerance;
use std::f64::consts::PI;

pub struct Primitives;

impl Primitives {
    /// Create a circle as a rational NURBS curve (9 control points)
    pub fn circle(cx: f64, cy: f64, cz: f64, radius: f64) -> NurbsCurve {
        let w = (2.0_f64).sqrt() / 2.0;

        let mut curve = NurbsCurve::new(3, true, 3, 9);
        curve.m_knot = vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0];

        let angles = [
            0.0, PI / 4.0, PI / 2.0, 3.0 * PI / 4.0,
            PI, 5.0 * PI / 4.0, 3.0 * PI / 2.0,
            7.0 * PI / 4.0, 2.0 * PI
        ];
        let weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];

        for i in 0..9 {
            let x = cx + radius * angles[i].cos();
            let y = cy + radius * angles[i].sin();
            let z = cz;
            curve.set_cv_4d(i, x * weights[i], y * weights[i], z * weights[i], weights[i]);
        }

        curve
    }

    /// Create an ellipse as a rational NURBS curve
    pub fn ellipse(cx: f64, cy: f64, cz: f64, major_radius: f64, minor_radius: f64) -> NurbsCurve {
        let w = (2.0_f64).sqrt() / 2.0;

        let mut curve = NurbsCurve::new(3, true, 3, 9);
        curve.m_knot = vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0];

        let angles = [
            0.0, PI / 4.0, PI / 2.0, 3.0 * PI / 4.0,
            PI, 5.0 * PI / 4.0, 3.0 * PI / 2.0,
            7.0 * PI / 4.0, 2.0 * PI
        ];
        let weights = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];

        for i in 0..9 {
            let x = cx + major_radius * angles[i].cos();
            let y = cy + minor_radius * angles[i].sin();
            let z = cz;
            curve.set_cv_4d(i, x * weights[i], y * weights[i], z * weights[i], weights[i]);
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
}
