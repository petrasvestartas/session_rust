#![allow(
    clippy::needless_range_loop,
    clippy::manual_memcpy,
    clippy::excessive_precision
)]
use crate::closest::Closest;
use crate::color::Color;
use crate::nurbsknot;
use crate::nurbsknot::CurveInterpStyle;
use crate::nurbsknot::CurveNurbsKnotStyle;
use crate::plane::Plane;
use crate::point::Point;
use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::ser::SerializeMap;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde::Serializer;
use std::fmt;
use std::sync::OnceLock;

const SQRT_EPSILON: f64 = 1.490116119385e-8;
const GL_X: [f64; 10] = [
    -0.9739065285171717,
    -0.8650633666889845,
    -0.6794095682990244,
    -0.4333953941292472,
    -0.1488743389816312,
    0.1488743389816312,
    0.4333953941292472,
    0.6794095682990244,
    0.8650633666889845,
    0.9739065285171717,
];
const GL_W: [f64; 10] = [
    0.0666713443086881,
    0.1494513491505806,
    0.2190863625159820,
    0.2692667193099963,
    0.2955242247147529,
    0.2955242247147529,
    0.2692667193099963,
    0.2190863625159820,
    0.1494513491505806,
    0.0666713443086881,
];
const GL_NODES: [f64; 5] = [
    -0.9061798459386640,
    -0.5384693101056831,
    0.0,
    0.5384693101056831,
    0.9061798459386640,
];
const GL_WEIGHTS: [f64; 5] = [
    0.2369268850561891,
    0.4786286704993665,
    0.5688888888888889,
    0.4786286704993665,
    0.2369268850561891,
];

/// A NURBS curve: OpenNURBS layout, nurbsknot count = order + cv_count - 2, homogeneous CVs when rational.
#[derive(Clone, Debug)]
pub struct NurbsCurve {
    guid: OnceLock<String>,      // Lazily minted GUID.
    pub name: String,            // Curve name.
    pub width: f64,              // Display width.
    pub pointcolors: Vec<Color>, // Display color per control point.
    pub linecolors: Vec<Color>,  // Display color per control polygon segment.
    pub m_dim: usize,            // Coordinate dimension.
    pub m_is_rat: bool,          // True when rational.
    pub m_order: usize,          // Degree + 1.
    pub m_cv_count: usize,       // Number of control vertices.
    pub m_cv_stride: usize,      // Doubles between consecutive CVs.
    pub m_nurbsknot: Vec<f64>,   // NurbsKnot vector, order + cv_count - 2 values.
    pub m_cv: Vec<f64>,          // Flat CV array, homogeneous when rational.
}

impl Default for NurbsCurve {
    /// Construct an empty curve.
    fn default() -> Self {
        NurbsCurve {
            guid: OnceLock::new(),
            name: "my_nurbscurve".to_string(),
            width: 1.0,
            pointcolors: Vec::new(),
            linecolors: Vec::new(),
            m_dim: 0,
            m_is_rat: false,
            m_order: 0,
            m_cv_count: 0,
            m_cv_stride: 0,
            m_nurbsknot: Vec::new(),
            m_cv: Vec::new(),
        }
    }
}

impl NurbsCurve {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an unset curve with the given layout.
    pub fn new(dimension: usize, is_rational: bool, order: usize, cv_count: usize) -> Self {
        let mut curve = Self::default();
        curve.create_curve(dimension, is_rational, order, cv_count);

        curve
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a clamped or periodic uniform curve through control points, domain rescaled to [0, arc length].
    pub fn create(periodic: bool, degree: usize, points: &[Point]) -> Self {
        let mut curve = Self::default();
        let order = degree + 1;

        if periodic {
            curve.create_periodic_uniform(3, order, points, 1.0);
        } else {
            curve.create_clamped_uniform(3, order, points, 1.0);
        }

        if !curve.is_valid() {
            return curve;
        }

        let mut length = 0.0;

        if degree == 1 {
            let np = points.len();

            for i in 1..np {
                length += points[i - 1].distance(&points[i], None);
            }

            if periodic && np > 1 {
                length += points[np - 1].distance(&points[0], None);
            }
        } else {
            length = curve.length(None);
        }

        if length > 0.0 {
            curve.set_domain(0.0, length);
        }

        curve
    }

    /// Construct an interpolated cubic through points; Rhino (Bessel) or Occt (Lagrange) end tangents.
    pub fn create_interpolated(
        points: &[Point],
        parameterization: CurveNurbsKnotStyle,
        end_condition: CurveInterpStyle,
    ) -> Self {
        let n = points.len();

        if n < 2 {
            return Self::default();
        }

        let periodic = matches!(
            parameterization,
            CurveNurbsKnotStyle::UniformPeriodic
                | CurveNurbsKnotStyle::ChordPeriodic
                | CurveNurbsKnotStyle::ChordSquareRootPeriodic
        );

        if periodic && n < 3 {
            return Self::default();
        }

        if n == 2 && !periodic {
            return Self::create(false, 1, points);
        }

        if periodic {
            return Self::create_interpolated_periodic(points, parameterization);
        }

        Self::create_interpolated_clamped(points, parameterization, end_condition)
    }

    /// Construct from poles, weights, distinct knots and multiplicities (OCCT convention).
    pub fn create_from_parameters(
        points: &[Point],
        weights: &[f64],
        knots: &[f64],
        mults: &[usize],
        degree: usize,
        periodic: bool,
    ) -> Self {
        let n = points.len();
        let order = degree + 1;

        if n < order {
            return Self::default();
        }

        if weights.len() != n {
            return Self::default();
        }

        if knots.len() != mults.len() || knots.is_empty() {
            return Self::default();
        }

        if periodic {
            return Self::default();
        }

        let mut rational = false;

        for &w in weights {
            if (w - 1.0).abs() > Tolerance::ZERO_TOLERANCE {
                rational = true;
            }
        }

        let mut full: Vec<f64> = Vec::new();

        for i in 0..knots.len() {
            for _ in 0..mults[i] {
                full.push(knots[i]);
            }
        }

        let kc = order + n - 2;

        if full.len() != kc + 2 {
            return Self::default();
        }

        let mut curve = Self::default();

        if !curve.create_curve(3, rational, order, n) {
            return Self::default();
        }

        for i in 0..kc {
            curve.set_nurbsknot(i, full[i + 1]);
        }

        for i in 0..n {
            if rational {
                let w = weights[i];
                curve.set_cv_4d(i, points[i][0] * w, points[i][1] * w, points[i][2] * w, w);
            } else {
                curve.set_cv(i, &points[i]);
            }
        }

        curve
    }

    /// Construct a least-squares fit with num_cvs control points (Piegl & Tiller 9.4).
    pub fn create_fitted(
        points: &[Point],
        num_cvs: usize,
        degree: usize,
        is_periodic: bool,
    ) -> Self {
        if is_periodic {
            return Self::create_fitted_periodic(points, num_cvs, degree);
        }

        Self::create_fitted_clamped(points, num_cvs, degree)
    }

    /// Chain segments by endpoint matching, raise to a common degree and merge with C0 junctions.
    pub fn join(curves: &[NurbsCurve], tolerance: Option<f64>) -> Vec<NurbsCurve> {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        let mut segs: Vec<NurbsCurve> = Vec::new();

        for c in curves {
            if c.is_valid() {
                segs.push(c.duplicate());
            }
        }

        Self::promote_to_3d(&mut segs);

        let chains = Self::chain_segments(&segs, tolerance);
        let mut result: Vec<NurbsCurve> = Vec::new();

        for chain in chains {
            Self::join_chain(chain, &mut result);
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform in place.
    pub fn transform(&mut self, xform: &Xform) -> bool {
        for i in 0..self.m_cv_count {
            let p = self.get_cv(i).unwrap_or_default();
            let x = xform.m[0] * p[0] + xform.m[4] * p[1] + xform.m[8] * p[2] + xform.m[12];
            let y = xform.m[1] * p[0] + xform.m[5] * p[1] + xform.m[9] * p[2] + xform.m[13];
            let z = xform.m[2] * p[0] + xform.m[6] * p[1] + xform.m[10] * p[2] + xform.m[14];

            if self.m_is_rat {
                let w = self.weight(i);
                self.set_cv_4d(i, x * w, y * w, z * w, w);
            } else {
                self.set_cv(i, &Point::new(x, y, z));
            }
        }

        true
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Initialization
    // ═══════════════════════════════════════════════════════════════════════════
    /// Zero every field.
    pub fn initialize(&mut self) {
        self.m_dim = 0;
        self.m_is_rat = false;
        self.m_order = 0;
        self.m_cv_count = 0;
        self.m_cv_stride = 0;
        self.m_nurbsknot.clear();
        self.m_cv.clear();
    }

    /// Allocate layout for dimension, rationality, order and cv_count.
    pub fn create_curve(
        &mut self,
        dimension: usize,
        is_rational: bool,
        order: usize,
        cv_count: usize,
    ) -> bool {
        if dimension < 1 || order < 2 || cv_count < order {
            return false;
        }

        self.destroy();
        self.m_dim = dimension;
        self.m_is_rat = is_rational;
        self.m_order = order;
        self.m_cv_count = cv_count;
        self.m_cv_stride = if is_rational {
            dimension + 1
        } else {
            dimension
        };
        self.m_nurbsknot = vec![0.0; self.m_order + self.m_cv_count - 2];
        self.m_cv = vec![0.0; self.m_cv_count * self.m_cv_stride];

        true
    }

    /// Set clamped uniform nurbsknots over control points.
    pub fn create_clamped_uniform(
        &mut self,
        dimension: usize,
        order: usize,
        points: &[Point],
        nurbsknot_delta: f64,
    ) -> bool {
        let point_count = points.len();

        if !self.create_curve(dimension, false, order, point_count) {
            return false;
        }

        for i in 0..point_count {
            self.set_cv(i, &points[i]);
        }

        let kc = self.m_order + self.m_cv_count - 2;
        let mut k = 0.0;

        for i in (self.m_order - 2)..self.m_cv_count {
            self.m_nurbsknot[i] = k;
            k += nurbsknot_delta;
        }

        let mut i0 = self.m_order - 2;

        for i in 0..i0 {
            self.m_nurbsknot[i] = self.m_nurbsknot[i0];
        }

        i0 = self.m_cv_count - 1;

        for i in (i0 + 1)..kc {
            self.m_nurbsknot[i] = self.m_nurbsknot[i0];
        }

        true
    }

    /// Set periodic uniform nurbsknots over control points wrapped by order - 1.
    pub fn create_periodic_uniform(
        &mut self,
        dimension: usize,
        order: usize,
        points: &[Point],
        nurbsknot_delta: f64,
    ) -> bool {
        let point_count = points.len();

        if !self.create_curve(dimension, false, order, point_count + order - 1) {
            return false;
        }

        for i in 0..point_count {
            self.set_cv(i, &points[i]);
        }

        for i in 0..(order - 1) {
            self.set_cv(point_count + i, &points[i]);
        }

        let kc = self.m_order + self.m_cv_count - 2;

        for i in 0..kc {
            self.m_nurbsknot[i] = (i as f64 - self.m_order as f64 + 1.0) * nurbsknot_delta;
        }

        true
    }

    /// Reset to the empty state.
    pub fn destroy(&mut self) {
        self.m_nurbsknot.clear();
        self.m_cv.clear();
        self.initialize();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the layout, nurbsknots and CVs are consistent.
    pub fn is_valid(&self) -> bool {
        if self.m_dim == 0 {
            return false;
        }

        if self.m_order < 2 {
            return false;
        }

        if self.m_cv_count < self.m_order {
            return false;
        }

        if self.m_cv_stride < self.cv_size() {
            return false;
        }

        if self.m_cv.is_empty() || self.m_nurbsknot.is_empty() {
            return false;
        }

        if self.m_cv.len() < (self.m_cv_count - 1) * self.m_cv_stride + self.cv_size() {
            return false;
        }

        if !self.is_valid_nurbsknot_vector() {
            return false;
        }

        for i in 0..self.m_cv.len() {
            if !self.m_cv[i].is_finite() {
                return false;
            }
        }

        true
    }

    /// Return whether the CVs carry weights.
    pub fn is_rational(&self) -> bool {
        self.m_is_rat
    }

    /// Return whether the start point equals the end point.
    pub fn is_closed(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        self.point_at_start().distance(&self.point_at_end(), None) < Tolerance::ZERO_TOLERANCE
    }

    /// Return whether the last degree CVs repeat the first and nurbsknots are uniform.
    pub fn is_periodic(&self) -> bool {
        if self.m_order < 2 {
            return false;
        }

        let deg = self.degree();

        for i in 0..deg {
            let p0 = self.get_cv(i).unwrap_or_default();
            let p1 = self.get_cv(self.m_cv_count - deg + i).unwrap_or_default();

            if p0.distance(&p1, None) > Tolerance::ZERO_TOLERANCE {
                return false;
            }
        }

        let kc = self.nurbsknot_count();

        if kc < 2 {
            return false;
        }

        let delta = self.m_nurbsknot[self.m_order - 1] - self.m_nurbsknot[self.m_order - 2];

        if delta < Tolerance::ZERO_TOLERANCE {
            return false;
        }

        for i in 1..kc {
            if ((self.m_nurbsknot[i] - self.m_nurbsknot[i - 1]) - delta).abs()
                > Tolerance::ZERO_TOLERANCE
            {
                return false;
            }
        }

        true
    }

    /// Return whether every CV is within tolerance of the chord.
    pub fn is_linear(&self, tolerance: Option<f64>) -> bool {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

        if !self.is_valid() || self.m_cv_count < 2 {
            return false;
        }

        let p0 = self.get_cv(0).unwrap_or_default();
        let p1 = self.get_cv(self.m_cv_count - 1).unwrap_or_default();
        let line_vec = &p1 - &p0;
        let line_length = line_vec.magnitude();

        if line_length < tolerance {
            return true;
        }

        for i in 1..(self.m_cv_count - 1) {
            let p = self.get_cv(i).unwrap_or_default();
            let v = &p - &p0;

            if line_vec.cross(&v).magnitude() / line_length > tolerance {
                return false;
            }
        }

        true
    }

    /// Return whether every CV is within tolerance of one plane, written to plane when given.
    pub fn is_planar(&self, plane: Option<&mut Plane>, tolerance: Option<f64>) -> bool {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

        if !self.is_valid() || self.m_cv_count < 3 {
            return true;
        }

        let p0 = self.get_cv(0).unwrap_or_default();
        let p1 = self.get_cv(self.m_cv_count / 2).unwrap_or_default();
        let p2 = self.get_cv(self.m_cv_count - 1).unwrap_or_default();
        let v1 = &p1 - &p0;
        let v2 = &p2 - &p0;
        let mut normal = v1.cross(&v2);

        if normal.magnitude() < tolerance {
            return true;
        }

        for i in 0..self.m_cv_count {
            let p = self.get_cv(i).unwrap_or_default();
            let v = &p - &p0;

            if v.dot(&normal).abs() / normal.magnitude() > tolerance {
                return false;
            }
        }

        if let Some(plane) = plane {
            normal.normalize_self();

            let mut x_axis = v1.clone();
            x_axis.normalize_self();
            *plane = Plane::new(p0, x_axis.clone(), normal.cross(&x_axis));
        }

        true
    }

    /// Return whether the curve is planar and equidistant from one center, plane written when given.
    pub fn is_arc(&self, plane: Option<&mut Plane>, tolerance: Option<f64>) -> bool {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

        if !self.is_valid() {
            return false;
        }

        if self.m_dim != 2 && self.m_dim != 3 {
            return false;
        }

        if self.m_order < 3 {
            return false;
        }

        if self.is_linear(Some(tolerance)) {
            return false;
        }

        let mut test_plane = Plane::default();

        if !self.is_planar(Some(&mut test_plane), Some(tolerance)) {
            return false;
        }

        let (t0, t1) = self.domain();
        let p0 = self.point_at(t0);
        let Some(center) =
            Self::circle_center(&p0, &self.point_at((t0 + t1) * 0.5), &self.point_at(t1))
        else {
            return false;
        };

        let radius = center.distance(&p0, None);

        if radius < Tolerance::ZERO_TOLERANCE {
            return false;
        }

        let samples_per_span = 4.max(2 * self.degree() + 1);
        let num_samples = self.span_count() * samples_per_span;

        for i in 0..=num_samples {
            let t = t0 + (t1 - t0) * i as f64 / num_samples as f64;

            if (self.point_at(t).distance(&center, None) - radius).abs() > tolerance {
                return false;
            }
        }

        if let Some(plane) = plane {
            *plane = test_plane;
        }

        true
    }

    /// Return whether every CV is within tolerance of test_plane.
    pub fn is_in_plane(&self, test_plane: &Plane, tolerance: Option<f64>) -> bool {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

        if !self.is_valid() {
            return false;
        }

        for i in 0..self.m_cv_count {
            let pt = self.get_cv(i).unwrap_or_default();
            let v = &pt - &test_plane.origin();

            if v.dot(&test_plane.z_axis()).abs() > tolerance {
                return false;
            }
        }

        true
    }

    /// Return whether the second derivative is zero at end (0 = start, 1 = end, 2 = both).
    pub fn is_natural(&self, end: Option<i32>) -> bool {
        let end = end.unwrap_or(2);

        if !self.is_valid() {
            return false;
        }

        let tol_factor = 1e-8;
        let (t0, t1) = self.domain();
        let first = if end == 0 || end == 2 { 0 } else { 1 };
        let stop = if end == 1 || end == 2 { 2 } else { 1 };

        for pass in first..stop {
            let t = if pass == 0 { t0 } else { t1 };
            let derivs = self.evaluate(t, 2);

            if derivs.len() < 3 {
                return false;
            }

            let d2_len = derivs[2].magnitude();
            let cv0 = self
                .get_cv(if pass == 0 { 0 } else { self.m_cv_count - 1 })
                .unwrap_or_default();

            let cv2 = self
                .get_cv(if pass == 0 {
                    2.min(self.m_cv_count - 1)
                } else {
                    self.m_cv_count.saturating_sub(3)
                })
                .unwrap_or_default();

            if d2_len > cv0.distance(&cv2, None) * tol_factor {
                return false;
            }
        }

        true
    }

    /// Return the vertex count when every span is a line, else 0; vertices and params written when given.
    pub fn is_polyline(&self) -> (usize, Vec<Point>, Vec<f64>) {
        let mut points: Vec<Point> = Vec::new();
        let mut params: Vec<f64> = Vec::new();

        if !self.is_valid() {
            return (0, points, params);
        }

        if self.m_order == 2 {
            for i in 0..self.m_cv_count {
                points.push(self.get_cv(i).unwrap_or_default());
            }

            for i in 0..self.m_cv_count {
                params.push(self.m_nurbsknot[i]);
            }

            return (self.m_cv_count, points, params);
        }

        if self.m_order > 2 && self.m_dim >= 2 && self.m_dim <= 3 {
            let span_cnt = self.span_count();
            let mut all_linear = true;

            for i in 0..span_cnt {
                if !self.span_is_linear(i, Tolerance::ZERO_TOLERANCE, Tolerance::ZERO_TOLERANCE) {
                    all_linear = false;
                    break;
                }
            }

            if all_linear && span_cnt > 0 {
                points.push(self.get_cv(0).unwrap_or_default());

                for i in 0..span_cnt {
                    points.push(
                        self.get_cv(i * (self.m_order - 1) + (self.m_order - 1))
                            .unwrap_or_default(),
                    );
                }

                params = self.get_span_vector();

                return (span_cnt + 1, points, params);
            }
        }

        (0, points, params)
    }

    /// Return whether every span is collapsed to a point.
    pub fn is_singular(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let span_cnt = self.span_count();

        for i in 0..span_cnt {
            if !self.span_is_singular(i) {
                return false;
            }
        }

        true
    }

    /// Return whether layout, CVs and weights match to tolerance, and nurbsknots unless ignore_parameterization.
    pub fn is_duplicate(
        &self,
        other: &NurbsCurve,
        ignore_parameterization: bool,
        tolerance: Option<f64>,
    ) -> bool {
        let tolerance = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

        if !self.is_valid() || !other.is_valid() {
            return false;
        }

        if self.m_dim != other.m_dim {
            return false;
        }

        if self.m_is_rat != other.m_is_rat {
            return false;
        }

        if self.m_order != other.m_order {
            return false;
        }

        if self.m_cv_count != other.m_cv_count {
            return false;
        }

        for i in 0..self.m_cv_count {
            let p1 = self.get_cv(i).unwrap_or_default();
            let p2 = other.get_cv(i).unwrap_or_default();

            if p1.distance(&p2, None) > tolerance {
                return false;
            }

            if self.m_is_rat && (self.weight(i) - other.weight(i)).abs() > tolerance {
                return false;
            }
        }

        if !ignore_parameterization {
            for i in 0..self.nurbsknot_count() {
                if (self.m_nurbsknot[i] - other.m_nurbsknot[i]).abs() > tolerance {
                    return false;
                }
            }
        }

        true
    }

    /// Return the continuity at t from nurbsknot multiplicity (0 = C0, 1 = C1, 2 = C2, 3 = G1, 4 = G2).
    #[allow(clippy::too_many_arguments)]
    pub fn is_continuous(
        &self,
        continuity_type: i32,
        t: f64,
        point_tolerance: Option<f64>,
        d1_tolerance: Option<f64>,
        d2_tolerance: Option<f64>,
        cos_angle_tolerance: Option<f64>,
        curvature_tolerance: Option<f64>,
    ) -> bool {
        let _ = (
            point_tolerance,
            d1_tolerance,
            d2_tolerance,
            cos_angle_tolerance,
            curvature_tolerance,
        );

        if !self.is_valid() {
            return false;
        }

        let (d0, d1) = self.domain();

        if t < d0 || t > d1 {
            return false;
        }

        let mut nurbsknot_idx: i64 = -1;

        for i in 0..self.nurbsknot_count() {
            if (self.m_nurbsknot[i] - t).abs() < Tolerance::ZERO_TOLERANCE {
                nurbsknot_idx = i as i64;
                break;
            }
        }

        if nurbsknot_idx < 0 {
            return true;
        }

        let mult = self.nurbsknot_multiplicity(nurbsknot_idx as usize);

        if continuity_type == 0 {
            return mult < self.m_order;
        }

        if continuity_type == 1 {
            return mult < self.m_order - 1;
        }

        if continuity_type == 2 {
            return mult < self.m_order - 2;
        }

        mult < self.m_order - 1
    }

    /// Return whether the nurbsknots have the right count, are non-decreasing and span a non-empty domain.
    pub fn is_valid_nurbsknot_vector(&self) -> bool {
        let kc = self.nurbsknot_count();

        if self.m_nurbsknot.len() != kc {
            return false;
        }

        for i in 1..kc {
            if self.m_nurbsknot[i] < self.m_nurbsknot[i - 1] {
                return false;
            }
        }

        if self.m_nurbsknot[self.m_order - 2] >= self.m_nurbsknot[self.m_cv_count - 1] {
            return false;
        }

        true
    }

    /// Return whether end has full multiplicity (0 = start, 1 = end, 2 = both).
    pub fn is_clamped(&self, end: i32) -> bool {
        if !self.is_valid() {
            return false;
        }

        nurbsknot::is_clamped(self.m_order, self.m_cv_count, &self.m_nurbsknot, end)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    /// Return the coordinate dimension.
    pub fn dimension(&self) -> usize {
        self.m_dim
    }

    /// Return the order (degree + 1).
    pub fn order(&self) -> usize {
        self.m_order
    }

    /// Return the degree (order - 1).
    pub fn degree(&self) -> usize {
        self.m_order.saturating_sub(1)
    }

    /// Return the number of control vertices.
    pub fn cv_count(&self) -> usize {
        self.m_cv_count
    }

    /// Return the doubles per CV: dimension + 1 when rational.
    pub fn cv_size(&self) -> usize {
        if self.m_dim == 0 {
            return 0;
        }

        if self.m_is_rat {
            self.m_dim + 1
        } else {
            self.m_dim
        }
    }

    /// Return order + cv_count - 2.
    pub fn nurbsknot_count(&self) -> usize {
        (self.m_order + self.m_cv_count).saturating_sub(2)
    }

    /// Return the number of distinct nurbsknot intervals inside the domain.
    pub fn span_count(&self) -> usize {
        let mut count = 0;
        let kc = self.nurbsknot_count();

        if self.m_order < 2 || self.m_cv_count < 1 {
            return 0;
        }

        for i in (self.m_order - 2)..(self.m_cv_count - 1) {
            if i + 1 < kc && self.m_nurbsknot[i] < self.m_nurbsknot[i + 1] {
                count += 1;
            }
        }

        count
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Control vertex access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the pointer to the CV doubles, nullptr when out of range.
    pub fn cv(&self, cv_index: usize) -> Option<&[f64]> {
        if cv_index >= self.m_cv_count {
            return None;
        }

        let idx = cv_index * self.m_cv_stride;

        Some(&self.m_cv[idx..idx + self.m_cv_stride])
    }

    /// Return the mutable pointer to the CV doubles, nullptr when out of range.
    pub fn cv_mut(&mut self, cv_index: usize) -> Option<&mut [f64]> {
        if cv_index >= self.m_cv_count {
            return None;
        }

        let idx = cv_index * self.m_cv_stride;

        Some(&mut self.m_cv[idx..idx + self.m_cv_stride])
    }

    /// Return the Euclidean CV (divided by weight when rational).
    pub fn get_cv(&self, cv_index: usize) -> Option<Point> {
        let cv_ptr = self.cv(cv_index)?;

        if self.m_is_rat {
            let w = cv_ptr[self.m_dim];

            if w.abs() < 1e-14 {
                return Some(Point::new(0.0, 0.0, 0.0));
            }

            return Some(Point::new(
                cv_ptr[0] / w,
                cv_ptr[1] / w,
                if self.m_dim > 2 { cv_ptr[2] / w } else { 0.0 },
            ));
        }

        Some(Point::new(
            cv_ptr[0],
            cv_ptr[1],
            if self.m_dim > 2 { cv_ptr[2] } else { 0.0 },
        ))
    }

    /// Return the homogeneous CV (x, y, z, w).
    pub fn get_cv_4d(&self, cv_index: usize) -> Option<(f64, f64, f64, f64)> {
        let cv_ptr = self.cv(cv_index)?;
        let x = cv_ptr[0];
        let y = if self.m_dim > 1 { cv_ptr[1] } else { 0.0 };
        let z = if self.m_dim > 2 { cv_ptr[2] } else { 0.0 };
        let w = if self.m_is_rat {
            cv_ptr[self.m_dim]
        } else {
            1.0
        };

        Some((x, y, z, w))
    }

    /// Set the CV from a point, weight reset to 1.
    pub fn set_cv(&mut self, cv_index: usize, point: &Point) -> bool {
        let dim = self.m_dim;
        let is_rat = self.m_is_rat;
        let Some(cv_ptr) = self.cv_mut(cv_index) else {
            return false;
        };

        cv_ptr[0] = point[0];

        if dim > 1 {
            cv_ptr[1] = point[1];
        }

        if dim > 2 {
            cv_ptr[2] = point[2];
        }

        if is_rat {
            cv_ptr[dim] = 1.0;
        }

        true
    }

    /// Set the homogeneous CV, making the curve rational when w != 1.
    pub fn set_cv_4d(&mut self, cv_index: usize, x: f64, y: f64, z: f64, w: f64) -> bool {
        if cv_index >= self.m_cv_count {
            return false;
        }

        if !self.m_is_rat && w != 1.0 && !self.make_rational() {
            return false;
        }

        let dim = self.m_dim;
        let is_rat = self.m_is_rat;
        let Some(cv_ptr) = self.cv_mut(cv_index) else {
            return false;
        };

        cv_ptr[0] = x;

        if dim > 1 {
            cv_ptr[1] = y;
        }

        if dim > 2 {
            cv_ptr[2] = z;
        }

        if is_rat {
            cv_ptr[dim] = w;
        }

        true
    }

    /// Return the weight of a CV, 1 when non-rational.
    pub fn weight(&self, cv_index: usize) -> f64 {
        if !self.m_is_rat {
            return 1.0;
        }

        match self.cv(cv_index) {
            Some(c) => c[self.m_dim],
            None => 1.0,
        }
    }

    /// Set the weight, making the curve rational first.
    pub fn set_weight(&mut self, cv_index: usize, weight: f64) -> bool {
        if !self.m_is_rat && !self.make_rational() {
            return false;
        }

        let dim = self.m_dim;
        let Some(cv_ptr) = self.cv_mut(cv_index) else {
            return false;
        };

        cv_ptr[dim] = weight;

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // NurbsKnot access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the nurbsknot at nurbsknot_index.
    pub fn nurbsknot(&self, nurbsknot_index: usize) -> Option<f64> {
        if nurbsknot_index >= self.m_nurbsknot.len() {
            return None;
        }

        Some(self.m_nurbsknot[nurbsknot_index])
    }

    /// Set the nurbsknot at nurbsknot_index.
    pub fn set_nurbsknot(&mut self, nurbsknot_index: usize, nurbsknot_value: f64) -> bool {
        if nurbsknot_index >= self.m_nurbsknot.len() {
            return false;
        }

        self.m_nurbsknot[nurbsknot_index] = nurbsknot_value;

        true
    }

    /// Return the count of nurbsknots equal to the one at nurbsknot_index.
    pub fn nurbsknot_multiplicity(&self, nurbsknot_index: usize) -> usize {
        if nurbsknot_index >= self.nurbsknot_count() {
            return 0;
        }

        let nurbsknot_value = self.m_nurbsknot[nurbsknot_index];
        let mut mult = 1;

        for i in (nurbsknot_index + 1)..self.nurbsknot_count() {
            if (self.m_nurbsknot[i] - nurbsknot_value).abs() >= Tolerance::ZERO_TOLERANCE {
                break;
            }

            mult += 1;
        }

        for i in (0..nurbsknot_index).rev() {
            if (self.m_nurbsknot[i] - nurbsknot_value).abs() >= Tolerance::ZERO_TOLERANCE {
                break;
            }

            mult += 1;
        }

        mult
    }

    /// Return the reflected end nurbsknot (0 = start, 1 = end).
    pub fn superfluous_nurbsknot(&self, end: usize) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }

        if end == 0 {
            return 2.0 * self.m_nurbsknot[0] - self.m_nurbsknot[self.m_order - 2];
        }

        2.0 * self.m_nurbsknot[self.nurbsknot_count() - 1]
            - self.m_nurbsknot[self.m_cv_count - self.m_order]
    }

    /// Return the nurbsknot array pointer.
    pub fn nurbsknot_array(&self) -> &[f64] {
        &self.m_nurbsknot
    }

    /// Return the CV array pointer.
    pub fn cv_array(&self) -> &[f64] {
        &self.m_cv
    }

    /// Return the mutable CV array pointer.
    pub fn cv_array_mut(&mut self) -> &mut [f64] {
        &mut self.m_cv
    }

    /// Return a copy of the nurbsknot vector.
    pub fn get_nurbsknots(&self) -> Vec<f64> {
        self.m_nurbsknot.clone()
    }

    /// Insert a nurbsknot by Boehm to the given multiplicity.
    pub fn insert_nurbsknot(
        &mut self,
        nurbsknot_value: f64,
        nurbsknot_multiplicity: usize,
    ) -> bool {
        if !self.is_valid() {
            return false;
        }

        let p = self.degree();

        if nurbsknot_multiplicity < 1 || nurbsknot_multiplicity > p {
            return false;
        }

        let (d0, d1) = self.domain();

        if nurbsknot_value < d0 || nurbsknot_value > d1 {
            return false;
        }

        if nurbsknot_value == d0 {
            if nurbsknot_multiplicity == p {
                return self.clamp_end(0);
            }

            return nurbsknot_multiplicity == 1;
        }

        if nurbsknot_value == d1 {
            if nurbsknot_multiplicity == p {
                return self.clamp_end(1);
            }

            return nurbsknot_multiplicity == 1;
        }

        let tol = (d0.abs() + d1.abs() + (d1 - d0).abs()) * SQRT_EPSILON;

        for _ in 0..nurbsknot_multiplicity {
            let u = self.full_nurbsknots();
            let mut mult = 0;

            for &knot in &u {
                if (knot - nurbsknot_value).abs() <= tol {
                    mult += 1;
                }
            }

            if mult >= nurbsknot_multiplicity {
                return true;
            }

            if mult >= p {
                return false;
            }

            self.insert_nurbsknot_once(nurbsknot_value, &u);
        }

        true
    }

    /// Return the Greville abcissa of a CV.
    pub fn greville_abcissa(&self, cv_index: usize) -> f64 {
        if cv_index >= self.m_cv_count {
            return 0.0;
        }

        let nurbsknot = &self.m_nurbsknot[cv_index..];
        let order = self.m_order;

        if order <= 2 || nurbsknot[0] == nurbsknot[order - 2] {
            return nurbsknot[0];
        }

        let p = order - 1;
        let k0 = nurbsknot[0];
        let k = nurbsknot[p / 2];
        let k1 = nurbsknot[p - 1];
        let tol = (k1 - k0) * SQRT_EPSILON;
        let mut g = 0.0;

        for i in 0..p {
            g += nurbsknot[i];
        }

        g /= p as f64;

        if (2.0 * k - (k0 + k1)).abs() <= tol && (g - k).abs() <= (g.abs() * SQRT_EPSILON + tol) {
            g = k;
        }

        g
    }

    /// Return the Greville abcissae of every CV.
    pub fn get_greville_abcissae(&self) -> Vec<f64> {
        let mut abcissae: Vec<f64> = Vec::new();

        if !self.is_valid() {
            return abcissae;
        }

        for i in 0..self.m_cv_count {
            abcissae.push(self.greville_abcissa(i));
        }

        abcissae
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Domain
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the domain (t0, t1).
    pub fn domain(&self) -> (f64, f64) {
        if self.m_nurbsknot.is_empty() {
            return (0.0, 0.0);
        }

        (
            self.m_nurbsknot[self.m_order - 2],
            self.m_nurbsknot[self.m_cv_count - 1],
        )
    }

    /// Return the domain start.
    pub fn domain_start(&self) -> f64 {
        if self.m_nurbsknot.is_empty() {
            return 0.0;
        }

        self.m_nurbsknot[self.m_order - 2]
    }

    /// Return the domain end.
    pub fn domain_end(&self) -> f64 {
        if self.m_nurbsknot.is_empty() {
            return 0.0;
        }

        self.m_nurbsknot[self.m_cv_count - 1]
    }

    /// Return the domain midpoint.
    pub fn domain_middle(&self) -> f64 {
        if self.m_nurbsknot.is_empty() {
            return 0.0;
        }

        (self.m_nurbsknot[self.m_order - 2] + self.m_nurbsknot[self.m_cv_count - 1]) * 0.5
    }

    /// Rescale the nurbsknots to [t0, t1].
    pub fn set_domain(&mut self, t0: f64, t1: f64) -> bool {
        if t0 >= t1 || !self.is_valid() {
            return false;
        }

        let (d0, d1) = self.domain();

        if d0 >= d1 {
            return false;
        }

        let clamped_start = self.m_order >= 2
            && (self.m_nurbsknot[0] - self.m_nurbsknot[self.m_order - 2]).abs()
                < Tolerance::ZERO_TOLERANCE;

        let clamped_end = self.m_cv_count < self.m_nurbsknot.len()
            && (self.m_nurbsknot[self.m_nurbsknot.len() - 1]
                - self.m_nurbsknot[self.m_cv_count - 1])
                .abs()
                < Tolerance::ZERO_TOLERANCE;

        let scale = (t1 - t0) / (d1 - d0);

        for k in self.m_nurbsknot.iter_mut() {
            *k = t0 + (*k - d0) * scale;
        }

        if clamped_start {
            for i in 0..(self.m_order - 1) {
                self.m_nurbsknot[i] = t0;
            }
        }

        if clamped_end {
            for i in (self.m_cv_count - 1)..self.m_nurbsknot.len() {
                self.m_nurbsknot[i] = t1;
            }
        }

        true
    }

    /// Return the distinct nurbsknot values inside the domain.
    pub fn get_span_vector(&self) -> Vec<f64> {
        let mut spans = vec![self.m_nurbsknot[self.m_order - 2]];

        for i in (self.m_order - 1)..self.m_cv_count {
            if self.m_nurbsknot[i] > spans[spans.len() - 1] {
                spans.push(self.m_nurbsknot[i]);
            }
        }

        spans
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return (found, t) for the first interior nurbsknot in (t0, t1) whose multiplicity breaks continuity_type.
    pub fn get_next_discontinuity(&self, continuity_type: i32, t0: f64, t1: f64) -> (bool, f64) {
        if !self.is_valid() {
            return (false, 0.0);
        }

        if t0 >= t1 {
            return (false, 0.0);
        }

        let (d0, d1) = self.domain();
        let t0 = t0.max(d0);
        let t1 = t1.min(d1);

        if t0 >= t1 {
            return (false, 0.0);
        }

        for i in (self.m_order - 1)..(self.m_cv_count - 1) {
            let t = self.m_nurbsknot[i];

            if t <= t0 || t >= t1 {
                continue;
            }

            let mult = self.nurbsknot_multiplicity(i);
            let mut found = false;

            if continuity_type == 0 {
                found = mult >= self.m_order;
            } else if continuity_type == 1 || continuity_type == 3 || continuity_type == 4 {
                found = mult >= self.m_order - 1;
            } else if continuity_type == 2 {
                found = mult >= self.m_order - 2;
            }

            if !found {
                continue;
            }

            return (true, t);
        }

        (false, 0.0)
    }

    /// Return the arc length by 10-point Gauss-Legendre over 4 subdivisions per span.
    pub fn length(&self, tolerance: Option<f64>) -> f64 {
        let _ = tolerance;

        if !self.is_valid() {
            return 0.0;
        }

        const SUBDIVISIONS: usize = 4;
        let mut total = 0.0;
        let n_spans = self.span_count();

        for span in 0..n_spans {
            let span_a = self.m_nurbsknot[self.m_order - 2 + span];
            let span_b = self.m_nurbsknot[self.m_order - 1 + span];

            if span_b <= span_a {
                continue;
            }

            let span_width = (span_b - span_a) / SUBDIVISIONS as f64;

            for sub in 0..SUBDIVISIONS {
                let a = span_a + sub as f64 * span_width;
                let b = a + span_width;
                let mid = (a + b) * 0.5;
                let half = (b - a) * 0.5;
                let mut s = 0.0;

                for i in 0..10 {
                    s += GL_W[i] * self.evaluate(mid + half * GL_X[i], 1)[1].magnitude();
                }

                total += half * s;
            }
        }

        total
    }

    /// Return the chord-deviation subdivision points and parameters.
    pub fn to_polyline_adaptive(
        &self,
        angle_tolerance: f64,
        min_edge_length: f64,
        max_edge_length: f64,
    ) -> (Vec<Point>, Vec<f64>) {
        let mut points: Vec<Point> = Vec::new();
        let mut params: Vec<f64> = Vec::new();

        if !self.is_valid() {
            return (points, params);
        }

        let angle_tolerance = if angle_tolerance <= 0.0 {
            0.1
        } else {
            angle_tolerance
        };
        let curve_len = self.length(None);
        let max_edge_length = if max_edge_length <= 0.0 {
            curve_len / 10.0
        } else {
            max_edge_length
        };
        let mut min_edge_length = if min_edge_length <= 0.0 {
            curve_len / 1000.0
        } else {
            min_edge_length
        };

        if min_edge_length > max_edge_length {
            min_edge_length = max_edge_length * 0.1;
        }

        let samples = self.adaptive_samples(angle_tolerance, min_edge_length, max_edge_length);

        for (t, p) in samples {
            points.push(p);
            params.push(t);
        }

        (points, params)
    }

    /// Return count points at equal arc length and their parameters.
    pub fn divide_by_count(&self, count: usize, include_endpoints: bool) -> (Vec<Point>, Vec<f64>) {
        let mut points: Vec<Point> = Vec::new();
        let mut params: Vec<f64> = Vec::new();

        if !self.is_valid() {
            return (points, params);
        }

        if count < 2 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let h = (t1 - t0) * 1e-8;
        let n_samples = 1000.max(count * 100);
        let dt = (t1 - t0) / n_samples as f64;
        let mut t_vals = vec![0.0; n_samples + 1];
        let mut s_vals = vec![0.0; n_samples + 1];
        t_vals[0] = t0;
        s_vals[0] = 0.0;

        for i in 1..=n_samples {
            t_vals[i] = t0 + i as f64 * dt;
            s_vals[i] = s_vals[i - 1] + self.arc_length_gauss(t_vals[i - 1], t_vals[i], h);
        }

        let n_segs = if include_endpoints {
            count - 1
        } else {
            count + 1
        };
        let seg_len = s_vals[n_samples] / n_segs as f64;

        for i in 0..count {
            let s_target = if include_endpoints {
                seg_len * i as f64
            } else {
                seg_len * (i + 1) as f64
            };
            let t = self.find_t_at_s(s_target, &t_vals, &s_vals, h);
            points.push(self.point_at(t));
            params.push(t);
        }

        (points, params)
    }

    /// Return points every segment_length of arc length and their parameters.
    pub fn divide_by_length(&self, segment_length: f64) -> (Vec<Point>, Vec<f64>) {
        let mut points: Vec<Point> = Vec::new();
        let mut params: Vec<f64> = Vec::new();

        if !self.is_valid() {
            return (points, params);
        }

        if segment_length <= 0.0 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let h = (t1 - t0) * 1e-8;
        let n_samples = 1000.max((self.length(None) / segment_length) as usize * 100);
        let dt = (t1 - t0) / n_samples as f64;
        let mut t_vals = vec![0.0; n_samples + 1];
        let mut s_vals = vec![0.0; n_samples + 1];
        t_vals[0] = t0;
        s_vals[0] = 0.0;

        for i in 1..=n_samples {
            t_vals[i] = t0 + i as f64 * dt;
            s_vals[i] = s_vals[i - 1] + self.arc_length_gauss(t_vals[i - 1], t_vals[i], h);
        }

        let total_len = s_vals[n_samples];
        let mut s = 0.0;

        while s <= total_len + 1e-10 {
            let t = self.find_t_at_s(s, &t_vals, &s_vals, h);
            points.push(self.point_at(t));
            params.push(t);
            s += segment_length;
        }

        (points, params)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the point at parameter t.
    pub fn point_at(&self, t: f64) -> Point {
        if !self.is_valid() {
            return Point::new(0.0, 0.0, 0.0);
        }

        let span = self.find_span(t);
        let basis = self.basis_functions(span, t);
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut w = 0.0;

        for i in 0..self.m_order {
            let Some(cv_ptr) = self.cv(span + i) else {
                continue;
            };

            let n = basis[i];
            x += n * cv_ptr[0];
            y += n * if self.m_dim > 1 { cv_ptr[1] } else { 0.0 };
            z += n * if self.m_dim > 2 { cv_ptr[2] } else { 0.0 };

            if self.m_is_rat {
                w += n * cv_ptr[self.m_dim];
            } else {
                w = 1.0;
            }
        }

        if self.m_is_rat && w != 0.0 {
            return Point::new(x / w, y / w, z / w);
        }

        Point::new(x, y, z)
    }

    /// Return [point, first derivative, ..., derivative_count] with zeros past the degree.
    pub fn evaluate(&self, t: f64, derivative_count: usize) -> Vec<Vector> {
        let mut result: Vec<Vector> = Vec::new();

        if !self.is_valid() {
            result.push(Vector::new(0.0, 0.0, 0.0));

            return result;
        }

        let max_derivs = derivative_count.min(self.degree());
        let span = self.find_span(t);
        let ders = self.basis_functions_derivatives(span, t, max_derivs);
        let aders = self.homogeneous_derivatives(span, &ders);
        let mut cders = vec![[0.0; 3]; max_derivs + 1];

        if !self.m_is_rat {
            for k in 0..=max_derivs {
                cders[k] = [aders[k][0], aders[k][1], aders[k][2]];
            }
        } else {
            for k in 0..=max_derivs {
                let w = aders[0][3];
                let inv_w = if w != 0.0 { 1.0 / w } else { 0.0 };
                let mut ck_x = aders[k][0];
                let mut ck_y = aders[k][1];
                let mut ck_z = aders[k][2];

                for j in 1..=k {
                    let coeff = Self::binomial(k, j) as f64;
                    let wj = aders[j][3];
                    ck_x -= coeff * wj * cders[k - j][0];
                    ck_y -= coeff * wj * cders[k - j][1];
                    ck_z -= coeff * wj * cders[k - j][2];
                }

                cders[k] = [ck_x * inv_w, ck_y * inv_w, ck_z * inv_w];
            }
        }

        for k in 0..=max_derivs {
            result.push(Vector::new(cders[k][0], cders[k][1], cders[k][2]));
        }

        for _ in (max_derivs + 1)..=derivative_count {
            result.push(Vector::new(0.0, 0.0, 0.0));
        }

        result
    }

    /// Return the unit tangent by central difference.
    pub fn tangent_at(&self, t: f64) -> Vector {
        if !self.is_valid() {
            return Vector::new(0.0, 0.0, 0.0);
        }

        let (t0, t1) = self.domain();
        let h = (t1 - t0) * 1e-7;
        let p1;
        let p2;

        if t <= t0 + h {
            p1 = self.point_at(t0);
            p2 = self.point_at(t0 + h);
        } else if t >= t1 - h {
            p1 = self.point_at(t1 - h);
            p2 = self.point_at(t1);
        } else {
            p1 = self.point_at(t - h);
            p2 = self.point_at(t + h);
        }

        let mut tan = &p2 - &p1;

        if tan.magnitude() > 1e-14 {
            tan.normalize_self();
        }

        tan
    }

    /// Return |C' x C''| / |C'|^3.
    pub fn curvature_at(&self, t: f64) -> f64 {
        let d = self.evaluate(t, 2);

        if d.len() < 3 {
            return 0.0;
        }

        let s = d[1].magnitude();

        if s < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }

        d[1].cross(&d[2]).magnitude() / (s * s * s)
    }

    /// Return the parameter of the closest point to test_point.
    pub fn closest_parameter(&self, test_point: &Point) -> f64 {
        Closest::curve_point(self, test_point, 0.0, 0.0).0
    }

    /// Return the closest point to test_point.
    pub fn closest_point(&self, test_point: &Point) -> Point {
        self.point_at(self.closest_parameter(test_point))
    }

    /// Return the parameters (u, v) where this curve and other are closest.
    pub fn closest_parameters_curve(&self, other: &NurbsCurve) -> (f64, f64) {
        let (u, v, _dist) = Closest::curve_curve(self, other);

        (u, v)
    }

    /// Return the points where this curve and other are closest.
    pub fn closest_points_curve(&self, other: &NurbsCurve) -> (Point, Point) {
        let (u, v) = self.closest_parameters_curve(other);

        (self.point_at(u), other.point_at(v))
    }

    /// Return the Frenet frame (tangent, normal, binormal); normalized maps t from [0, 1].
    pub fn plane_at(&self, t: f64, normalized: bool) -> Plane {
        if !self.is_valid() {
            return Plane::invalid();
        }

        let (t0, t1) = self.domain();
        let param = if normalized {
            if !(0.0..=1.0).contains(&t) {
                return Plane::invalid();
            }

            t0 + t * (t1 - t0)
        } else {
            if t < t0 || t > t1 {
                return Plane::invalid();
            }

            t
        };
        let h = (t1 - t0) * 1e-5;
        let origin = self.point_at(param);

        if param <= t0 + h {
            let p0 = self.point_at(t0);
            let pp = self.point_at(t0 + h);
            let pp2 = self.point_at(t0 + 2.0 * h);
            let d1 = &pp - &p0;
            let d2 = Vector::new(
                (pp2[0] - 2.0 * pp[0] + p0[0]) / (h * h),
                (pp2[1] - 2.0 * pp[1] + p0[1]) / (h * h),
                (pp2[2] - 2.0 * pp[2] + p0[2]) / (h * h),
            );

            return Self::frenet_frame(origin, &d1, &d2);
        }

        if param >= t1 - h {
            let pm = self.point_at(t1 - h);
            let p0 = self.point_at(t1);
            let pm2 = self.point_at(t1 - 2.0 * h);
            let d1 = &p0 - &pm;
            let d2 = Vector::new(
                (p0[0] - 2.0 * pm[0] + pm2[0]) / (h * h),
                (p0[1] - 2.0 * pm[1] + pm2[1]) / (h * h),
                (p0[2] - 2.0 * pm[2] + pm2[2]) / (h * h),
            );

            return Self::frenet_frame(origin, &d1, &d2);
        }

        let pm = self.point_at(param - h);
        let p0 = self.point_at(param);
        let pp = self.point_at(param + h);
        let d1 = (&pp - &pm) / (2.0 * h);
        let d2 = Vector::new(
            (pp[0] - 2.0 * p0[0] + pm[0]) / (h * h),
            (pp[1] - 2.0 * p0[1] + pm[1]) / (h * h),
            (pp[2] - 2.0 * p0[2] + pm[2]) / (h * h),
        );

        Self::frenet_frame(origin, &d1, &d2)
    }

    /// Return the rotation minimizing frame by double reflection (Wang et al. 2008).
    pub fn perpendicular_plane_at(&self, t: f64, normalized: bool) -> Plane {
        if !self.is_valid() {
            return Plane::invalid();
        }

        let (t0, t1) = self.domain();
        let param = if normalized {
            if !(0.0..=1.0).contains(&t) {
                return Plane::invalid();
            }

            t0 + t * (t1 - t0)
        } else {
            if t < t0 || t > t1 {
                return Plane::invalid();
            }

            t
        };

        let Some((t_0, r0)) = self.start_frame() else {
            return Plane::invalid();
        };

        let origin = self.point_at(param);

        if (param - t0).abs() < 1e-14 {
            let mut s0 = t_0.cross(&r0);
            s0.normalize_self();

            return Plane::from_frame(origin, r0, s0, t_0);
        }

        let mut ri = self.double_reflection(param, &r0, &t_0);
        let mut tangent = self.tangent_at(param);
        tangent.normalize_self();

        let ri_dot_t = ri.dot(&tangent);
        ri -= &tangent * ri_dot_t;

        if ri.magnitude() > 1e-14 {
            ri.normalize_self();
        }

        let mut s = tangent.cross(&ri);
        s.normalize_self();

        Plane::from_frame(origin, ri, s, tangent)
    }

    /// Return count + 1 rotation minimizing frames at equal arc length.
    pub fn get_perpendicular_planes(&self, count: usize) -> Vec<Plane> {
        let mut frames: Vec<Plane> = Vec::new();
        let (_pts, params) = self.divide_by_count(count + 1, true);

        for t in params {
            frames.push(self.perpendicular_plane_at(t, false));
        }

        frames
    }

    /// Return the point at the domain start.
    pub fn point_at_start(&self) -> Point {
        self.point_at(self.domain_start())
    }

    /// Return the point at the domain midpoint.
    pub fn point_at_middle(&self) -> Point {
        self.point_at(self.domain_middle())
    }

    /// Return the point at the domain end.
    pub fn point_at_end(&self) -> Point {
        self.point_at(self.domain_end())
    }

    /// Clamp and move the first CV.
    pub fn set_start_point(&mut self, start_point: &Point) -> bool {
        if !self.is_valid() || !self.clamp_end(2) {
            return false;
        }

        let w = if self.m_is_rat { self.weight(0) } else { 1.0 };

        if self.m_is_rat && w != 1.0 {
            self.set_cv_4d(
                0,
                start_point[0] * w,
                start_point[1] * w,
                start_point[2] * w,
                w,
            );
        } else {
            self.set_cv(0, start_point);

            if self.m_is_rat {
                self.set_weight(0, w);
            }
        }

        true
    }

    /// Clamp and move the last CV.
    pub fn set_end_point(&mut self, end_point: &Point) -> bool {
        if !self.is_valid() || !self.clamp_end(2) {
            return false;
        }

        let last = self.m_cv_count - 1;
        let w = if self.m_is_rat {
            self.weight(last)
        } else {
            1.0
        };

        if self.m_is_rat && w != 1.0 {
            self.set_cv_4d(
                last,
                end_point[0] * w,
                end_point[1] * w,
                end_point[2] * w,
                w,
            );
        } else {
            self.set_cv(last, end_point);

            if self.m_is_rat {
                self.set_weight(last, w);
            }
        }

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Modifications
    // ═══════════════════════════════════════════════════════════════════════════
    /// Reverse the direction keeping the domain.
    pub fn reverse(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let (d0, d1) = self.domain();

        for k in self.m_nurbsknot.iter_mut() {
            *k = d0 + d1 - *k;
        }

        self.m_nurbsknot.reverse();

        for i in 0..(self.m_cv_count / 2) {
            let j = self.m_cv_count - 1 - i;
            let Some((xi, yi, zi, wi)) = self.get_cv_4d(i) else {
                continue;
            };

            let Some((xj, yj, zj, wj)) = self.get_cv_4d(j) else {
                continue;
            };

            self.set_cv_4d(i, xj, yj, zj, wj);
            self.set_cv_4d(j, xi, yi, zi, wi);
        }

        true
    }

    /// Swap two coordinate axes of every CV.
    pub fn swap_coordinates(&mut self, axis_i: usize, axis_j: usize) -> bool {
        if !self.is_valid() {
            return false;
        }

        if axis_i >= self.m_dim {
            return false;
        }

        if axis_j >= self.m_dim {
            return false;
        }

        if axis_i == axis_j {
            return true;
        }

        for cv_idx in 0..self.m_cv_count {
            let idx = cv_idx * self.m_cv_stride;
            self.m_cv.swap(idx + axis_i, idx + axis_j);
        }

        true
    }

    /// Keep [t0, t1] by nurbsknot insertion.
    pub fn trim(&mut self, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || t0 >= t1 {
            return false;
        }

        let (d0, d1) = self.domain();

        if t0 < d0 - Tolerance::ZERO_TOLERANCE || t1 > d1 + Tolerance::ZERO_TOLERANCE {
            return false;
        }

        let mut t0 = t0.max(d0);
        let mut t1 = t1.min(d1);

        if (t0 - d0).abs() < Tolerance::ZERO_TOLERANCE
            && (t1 - d1).abs() < Tolerance::ZERO_TOLERANCE
        {
            return true;
        }

        let p = self.degree();
        let trim_start = t0 > d0 + Tolerance::ZERO_TOLERANCE;
        let trim_end = t1 < d1 - Tolerance::ZERO_TOLERANCE;

        let stol = (d0.abs() + d1.abs() + (d1 - d0).abs()) * SQRT_EPSILON;

        for &k in &self.m_nurbsknot {
            if trim_start && (k - t0).abs() <= stol && (k - t0).abs() > 0.0 {
                t0 = k;
            }

            if trim_end && (k - t1).abs() <= stol && (k - t1).abs() > 0.0 {
                t1 = k;
            }
        }

        if t0 >= t1 {
            return false;
        }

        if trim_start && !self.insert_nurbsknot(t0, p) {
            return false;
        }

        if trim_end && !self.insert_nurbsknot(t1, p) {
            return false;
        }

        self.keep_span_range(t0, t1)
    }

    /// Return trimmed copies on both sides of t.
    pub fn split(&self, t: f64) -> (NurbsCurve, NurbsCurve) {
        let left_curve = NurbsCurve::default();
        let right_curve = NurbsCurve::default();

        if !self.is_valid() {
            return (left_curve, right_curve);
        }

        let (t0, t1) = self.domain();

        if t <= t0 || t >= t1 {
            return (left_curve, right_curve);
        }

        let mut left_curve = self.duplicate();
        let mut right_curve = self.duplicate();

        if !left_curve.trim(t0, t) {
            return (left_curve, right_curve);
        }

        right_curve.trim(t, t1);

        (left_curve, right_curve)
    }

    /// Extrapolate the domain to cover [t0, t1] by de Boor.
    pub fn extend(&mut self, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || self.is_closed() {
            return false;
        }

        let (d0, d1) = self.domain();
        let cvdim = self.cv_size();
        let order = self.m_order;
        let stride = self.m_cv_stride;
        let mut changed = false;

        if t0 < d0 {
            if !self.clamp_end(0)
                || !Self::evaluate_nurbs_de_boor(
                    cvdim,
                    order,
                    stride,
                    &mut self.m_cv,
                    0,
                    &self.m_nurbsknot,
                    0,
                    1,
                    t0,
                )
            {
                return false;
            }

            for i in 0..(order - 1) {
                self.m_nurbsknot[i] = t0;
            }

            changed = true;
        }

        if t1 > d1 {
            if !self.clamp_end(1) {
                return false;
            }

            let i0 = self.m_cv_count - order;

            if !Self::evaluate_nurbs_de_boor(
                cvdim,
                order,
                stride,
                &mut self.m_cv,
                i0 * stride,
                &self.m_nurbsknot,
                i0,
                -1,
                t1,
            ) {
                return false;
            }

            let kc = self.nurbsknot_count();

            for i in (self.m_cv_count - 1)..kc {
                self.m_nurbsknot[i] = t1;
            }

            changed = true;
        }

        changed
    }

    /// Add unit weights.
    pub fn make_rational(&mut self) -> bool {
        if self.m_is_rat {
            return true;
        }

        let new_stride = self.m_dim + 1;
        let mut new_cv = vec![0.0; self.m_cv_count * new_stride];

        for i in 0..self.m_cv_count {
            for j in 0..self.m_dim {
                new_cv[i * new_stride + j] = self.m_cv[i * self.m_cv_stride + j];
            }

            new_cv[i * new_stride + self.m_dim] = 1.0;
        }

        self.m_cv = new_cv;
        self.m_is_rat = true;
        self.m_cv_stride = new_stride;

        true
    }

    /// Drop the weights; fails when they differ unless force.
    pub fn make_non_rational(&mut self, force: bool) -> bool {
        if !self.m_is_rat {
            return true;
        }

        if force {
            for i in 0..self.m_cv_count {
                let idx = i * self.m_cv_stride + self.m_dim;
                self.m_cv[idx] = 1.0;
            }
        } else {
            let w0 = self.weight(0);

            for i in 1..self.m_cv_count {
                if (self.weight(i) - w0).abs() > Tolerance::ZERO_TOLERANCE {
                    return false;
                }
            }
        }

        let new_stride = self.m_dim;
        let mut new_cv = vec![0.0; self.m_cv_count * new_stride];

        for i in 0..self.m_cv_count {
            let p = self.get_cv(i).unwrap_or_default();
            new_cv[i * new_stride] = p[0];

            if self.m_dim > 1 {
                new_cv[i * new_stride + 1] = p[1];
            }

            if self.m_dim > 2 {
                new_cv[i * new_stride + 2] = p[2];
            }
        }

        self.m_cv = new_cv;
        self.m_is_rat = false;
        self.m_cv_stride = new_stride;

        true
    }

    /// Set full multiplicity at end (0 = start, 1 = end, 2 = both) with CVs adjusted.
    pub fn clamp_end(&mut self, end: i32) -> bool {
        if !self.is_valid() {
            return false;
        }

        if !(0..=2).contains(&end) {
            return false;
        }

        let cvdim = self.cv_size();
        let order = self.m_order;
        let stride = self.m_cv_stride;
        let mut rc = true;

        if end == 0 || end == 2 {
            let t = self.m_nurbsknot[order - 2];

            if Self::evaluate_nurbs_de_boor(
                cvdim,
                order,
                stride,
                &mut self.m_cv,
                0,
                &self.m_nurbsknot,
                0,
                1,
                t,
            ) {
                for i in 0..(order - 2) {
                    self.m_nurbsknot[i] = t;
                }
            } else {
                rc = false;
            }
        }

        if end == 1 || end == 2 {
            let i0 = self.m_cv_count - order;
            let t = self.m_nurbsknot[self.m_cv_count - 1];

            if Self::evaluate_nurbs_de_boor(
                cvdim,
                order,
                stride,
                &mut self.m_cv,
                i0 * stride,
                &self.m_nurbsknot,
                i0,
                -1,
                t,
            ) {
                let kc = self.nurbsknot_count();

                for i in self.m_cv_count..kc {
                    self.m_nurbsknot[i] = t;
                }
            } else {
                rc = false;
            }
        }

        rc
    }

    /// Raise the degree by blossoming without changing the shape.
    pub fn increase_degree(&mut self, desired_degree: usize) -> bool {
        if !self.is_valid() {
            return false;
        }

        if desired_degree < 1 || desired_degree < self.degree() {
            return false;
        }

        if desired_degree == self.degree() {
            return true;
        }

        if !self.clamp_end(2) {
            return false;
        }

        let del = desired_degree - self.degree();

        for _ in 0..del {
            if !increment_nurbs_degree(self) {
                return false;
            }
        }

        true
    }

    /// Move the seam of a closed curve to t.
    pub fn change_closed_curve_seam(&mut self, t: f64) -> bool {
        if !self.is_valid() {
            return false;
        }

        if !self.is_closed() {
            return false;
        }

        let (t0, t1) = self.domain();
        let dom_len = t1 - t0;
        let mut t = t;
        let mut s = (t - t0) / dom_len;

        if !(0.0..=1.0).contains(&s) {
            s %= 1.0;

            if s < 0.0 {
                s += 1.0;
            }

            t = t0 + s * dom_len;
        }

        if (t - t0).abs() < Tolerance::ZERO_TOLERANCE || (t - t1).abs() < Tolerance::ZERO_TOLERANCE
        {
            return true;
        }

        if t <= t0 || t >= t1 {
            return true;
        }

        let p = self.degree();

        if self.is_periodic() {
            let kc = self.nurbsknot_count();

            if self.span_count() + 2 * p > kc {
                let mut nurbsknot_index = self.first_nurbsknot_above(t);

                if nurbsknot_index >= p as i64 && nurbsknot_index <= (kc - p) as i64 {
                    nurbsknot_index = self.seam_nurbsknot_index(t, nurbsknot_index);

                    if nurbsknot_index < 0 {
                        return false;
                    }

                    if nurbsknot_index >= p as i64
                        && nurbsknot_index < (self.nurbsknot_count() - p) as i64
                    {
                        return self.rotate_periodic_seam(nurbsknot_index as usize, t, dom_len);
                    }
                }
            }
        }

        self.split_seam(t, dom_len)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(filepath, json)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(filepath)?;

        Ok(serde_json::from_str(&contents)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::NurbsCurve {
        let mut pointcolors: Vec<crate::proto::Color> = Vec::new();

        for c in &self.pointcolors {
            pointcolors.push(crate::proto::Color {
                guid: String::new(),
                name: String::new(),
                r: c.r,
                g: c.g,
                b: c.b,
                a: c.a,
            });
        }

        let mut linecolors: Vec<crate::proto::Color> = Vec::new();

        for c in &self.linecolors {
            linecolors.push(crate::proto::Color {
                guid: String::new(),
                name: String::new(),
                r: c.r,
                g: c.g,
                b: c.b,
                a: c.a,
            });
        }

        crate::proto::NurbsCurve {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            dimension: self.m_dim as i32,
            is_rational: self.m_is_rat,
            order: self.m_order as i32,
            cv_count: self.m_cv_count as i32,
            cv_stride: self.m_cv_stride as i32,
            nurbsknots: self.m_nurbsknot.clone(),
            cvs: self.m_cv.clone(),
            width: self.width,
            pointcolors,
            linecolors,
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::NurbsCurve) -> Self {
        let mut curve = Self::new(
            proto.dimension as usize,
            proto.is_rational,
            proto.order as usize,
            proto.cv_count as usize,
        );

        if !proto.guid.is_empty() {
            curve.set_guid(proto.guid.clone());
        }

        curve.name = proto.name;
        curve.width = if proto.width != 0.0 { proto.width } else { 1.0 };
        curve.m_nurbsknot = proto.nurbsknots;
        curve.m_cv = proto.cvs;

        for c in &proto.pointcolors {
            curve.pointcolors.push(Color::new(c.r, c.g, c.b, c.a));
        }

        for c in &proto.linecolors {
            curve.linecolors.push(Color::new(c.r, c.g, c.b, c.a));
        }

        curve
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::NurbsCurve::decode(data)?))
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "NurbsCurve(name=..., degree=..., cvs=...)".
    pub fn str(&self) -> String {
        format!(
            "NurbsCurve(name={}, degree={}, cvs={})",
            self.name,
            self.degree(),
            self.cv_count()
        )
    }

    /// Return the multi-line form with every control point.
    pub fn repr(&self) -> String {
        let prec = Tolerance::ROUNDING;
        let mut result = format!("NurbsCurve(\n  name={},\n  degree={},\n  cvs={},\n  rational={},\n  control_points=[\n", self.name, self.degree(), self.m_cv_count, if self.m_is_rat { "true" } else { "false" });

        for i in 0..self.m_cv_count {
            let p = self.get_cv(i).unwrap_or_default();
            result += &format!(
                "    {}, {}, {}\n",
                TOLERANCE.format_number(p[0], prec),
                TOLERANCE.format_number(p[1], prec),
                TOLERANCE.format_number(p[2], prec)
            );
        }

        result += "  ]\n)";

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Private helpers
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the span has full end multiplicity and its CVs lie on its chord.
    fn span_is_linear(&self, span_index: usize, min_length: f64, tolerance: f64) -> bool {
        if !self.is_valid() {
            return false;
        }

        if span_index >= self.m_cv_count - self.m_order {
            return false;
        }

        if self.m_dim < 2 || self.m_dim > 3 {
            return false;
        }

        let ki = span_index + self.m_order - 2;

        if self.m_nurbsknot[ki] >= self.m_nurbsknot[ki + 1] {
            return false;
        }

        let mut mult_start = 1;
        let mut i = ki;

        while i > 0 && self.m_nurbsknot[i - 1] == self.m_nurbsknot[ki] {
            mult_start += 1;
            i -= 1;
        }

        let mut mult_end = 1;
        let kc = self.nurbsknot_count();
        let mut i = ki + 2;

        while i < kc && self.m_nurbsknot[i] == self.m_nurbsknot[ki + 1] {
            mult_end += 1;
            i += 1;
        }

        if mult_start < self.m_order - 1 || mult_end < self.m_order - 1 {
            return false;
        }

        let p0 = self.get_cv(span_index).unwrap_or_default();
        let p1 = self
            .get_cv(span_index + self.m_order - 1)
            .unwrap_or_default();

        let line_vec = &p1 - &p0;
        let line_length = line_vec.magnitude();

        if line_length < min_length {
            return false;
        }

        for i in 1..(self.m_order - 1) {
            let p = self.get_cv(span_index + i).unwrap_or_default();
            let v = &p - &p0;

            if line_vec.cross(&v).magnitude() / line_length > tolerance {
                return false;
            }

            let t = v.dot(&line_vec) / (line_length * line_length);

            if !(-0.01..=1.01).contains(&t) {
                return false;
            }
        }

        true
    }

    /// Return whether the span is collapsed to a point.
    fn span_is_singular(&self, span_index: usize) -> bool {
        if !self.is_valid() {
            return false;
        }

        if span_index >= self.m_cv_count - self.m_order {
            return false;
        }

        let ki = span_index + self.m_order - 2;

        if self.m_nurbsknot[ki] >= self.m_nurbsknot[ki + 1] {
            return true;
        }

        let p0 = self.get_cv(span_index).unwrap_or_default();

        for i in 1..self.m_order {
            let p = self.get_cv(span_index + i).unwrap_or_default();

            if p0.distance(&p, None) > Tolerance::ZERO_TOLERANCE {
                return false;
            }
        }

        true
    }

    /// Return the span index of t relative to nurbsknot[order - 2] by binary search.
    fn find_span(&self, t: f64) -> usize {
        let offset = self.m_order - 2;
        let len = self.m_cv_count - self.m_order + 2;

        if t <= self.m_nurbsknot[offset] {
            return 0;
        }

        if t >= self.m_nurbsknot[offset + len - 1] {
            return len - 2;
        }

        let mut low = 0;
        let mut high = len - 1;

        while high > low + 1 {
            let mid = (low + high) / 2;

            if t < self.m_nurbsknot[offset + mid] {
                high = mid;
            } else {
                low = mid;
            }
        }

        low
    }

    /// Compute the Cox-de Boor basis at t.
    fn basis_functions(&self, span: usize, t: f64) -> Vec<f64> {
        let mut basis = vec![0.0; self.m_order];
        let mut left = vec![0.0; self.m_order];
        let mut right = vec![0.0; self.m_order];
        let offset = self.m_order - 2 + span;
        basis[0] = 1.0;

        for j in 1..self.m_order {
            left[j] = t - self.m_nurbsknot[offset + 1 - j];
            right[j] = self.m_nurbsknot[offset + j] - t;

            let mut saved = 0.0;

            for r in 0..j {
                let denom = right[r + 1] + left[j - r];
                let temp = if denom != 0.0 { basis[r] / denom } else { 0.0 };
                basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }

            basis[j] = saved;
        }

        basis
    }

    /// Compute the basis derivatives (Piegl & Tiller A2.3).
    fn basis_functions_derivatives(
        &self,
        span: usize,
        t: f64,
        deriv_order: usize,
    ) -> Vec<Vec<f64>> {
        let p = self.degree();
        let n_der = deriv_order.min(p);
        let mut ders = vec![vec![0.0; p + 1]; n_der + 1];
        let ndu = self.basis_functions_ndu(span, t);

        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }

        let mut a = vec![vec![0.0; p + 1]; 2];

        for r in 0..=p {
            let mut s1 = 0;
            let mut s2 = 1;
            a[0][0] = 1.0;

            for k in 1..=n_der {
                let mut d = 0.0;
                let rk = r as i64 - k as i64;
                let pk = p as i64 - k as i64;

                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }

                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if r as i64 - 1 <= pk { k - 1 } else { p - r };

                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j - 1])
                        / ndu[(pk + 1) as usize][(rk + j as i64) as usize];

                    d += a[s2][j] * ndu[(rk + j as i64) as usize][pk as usize];
                }

                if r as i64 <= pk {
                    a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                    d += a[s2][k] * ndu[r][pk as usize];
                }

                ders[k][r] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }

        let mut scale = p as f64;

        for k in 1..=n_der {
            for j in 0..=p {
                ders[k][j] *= scale;
            }

            scale *= (p - k) as f64;
        }

        ders
    }

    /// Compute the triangular table of basis functions and nurbsknot differences (Piegl & Tiller A2.3).
    fn basis_functions_ndu(&self, span: usize, t: f64) -> Vec<Vec<f64>> {
        let p = self.degree();
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        let offset = self.m_order - 2 + span;
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        ndu[0][0] = 1.0;

        for j in 1..=p {
            left[j] = t - self.m_nurbsknot[offset + 1 - j];
            right[j] = self.m_nurbsknot[offset + j] - t;

            let mut saved = 0.0;

            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }

            ndu[j][j] = saved;
        }

        ndu
    }

    /// Reshape one span's CVs so it starts (side > 0) or ends (side < 0) at t (OpenNURBS ON_EvaluateNurbsDeBoor).
    #[allow(clippy::too_many_arguments)]
    fn evaluate_nurbs_de_boor(
        cv_dim: usize,
        order: usize,
        cv_stride: usize,
        cv: &mut [f64],
        cv0: usize,
        nurbsknots: &[f64],
        kn0: usize,
        side: i32,
        t: f64,
    ) -> bool {
        let degree = order - 1;

        if nurbsknots[kn0 + degree - 1] == nurbsknots[kn0 + degree] {
            return false;
        }

        if side < 0 {
            return Self::de_boor_end(cv_dim, order, cv_stride, cv, cv0, nurbsknots, kn0, t);
        }

        Self::de_boor_start(cv_dim, order, cv_stride, cv, cv0, nurbsknots, kn0, t)
    }

    /// Reshape one span's CVs so it ends at t.
    #[allow(clippy::too_many_arguments)]
    fn de_boor_end(
        cv_dim: usize,
        order: usize,
        cv_stride: usize,
        cv: &mut [f64],
        cv0: usize,
        nurbsknots: &[f64],
        kn0: usize,
        t: f64,
    ) -> bool {
        let degree = order - 1;
        let t0 = nurbsknots[kn0 + degree - 1];
        let t1 = nurbsknots[kn0 + degree];

        if t == t1 && t1 == nurbsknots[kn0 + 2 * degree - 1] {
            return true;
        }

        let fully_multiple = t0 == nurbsknots[kn0];
        let kn = kn0 + degree - 1;
        let mut delta_t = vec![0.0; degree];

        if !fully_multiple {
            for idx in 0..degree {
                delta_t[idx] = t - nurbsknots[kn - idx];
            }
        }

        for k in (1..order).rev() {
            for i in (0..k).rev() {
                let di = k - 1 - i;
                let alpha1 = if fully_multiple {
                    (t - t0) / (nurbsknots[kn + k - di] - t0)
                } else {
                    delta_t[di] / (nurbsknots[kn + k - di] - nurbsknots[kn - di])
                };
                let alpha0 = 1.0 - alpha1;
                let row1 = cv0 + (order - k + i) * cv_stride;
                let row0 = row1 - cv_stride;

                for j in 0..cv_dim {
                    cv[row1 + j] = cv[row0 + j] * alpha0 + cv[row1 + j] * alpha1;
                }
            }
        }

        true
    }

    /// Reshape one span's CVs so it starts at t.
    #[allow(clippy::too_many_arguments)]
    fn de_boor_start(
        cv_dim: usize,
        order: usize,
        cv_stride: usize,
        cv: &mut [f64],
        cv0: usize,
        nurbsknots: &[f64],
        kn0: usize,
        t: f64,
    ) -> bool {
        let degree = order - 1;
        let t0 = nurbsknots[kn0 + degree - 1];
        let t1 = nurbsknots[kn0 + degree];

        if t == t0 && t0 == nurbsknots[kn0] {
            return true;
        }

        let fully_multiple = t1 == nurbsknots[kn0 + 2 * degree - 1];
        let kn = kn0 + degree;
        let mut delta_t = vec![0.0; degree];

        if !fully_multiple {
            for idx in 0..degree {
                delta_t[idx] = nurbsknots[kn + idx] - t;
            }
        }

        for k in (1..order).rev() {
            for i in 0..k {
                let alpha0 = if fully_multiple {
                    (t1 - t) / (t1 - nurbsknots[kn - k + i])
                } else {
                    delta_t[i] / (nurbsknots[kn + i] - nurbsknots[kn - k + i])
                };
                let alpha1 = 1.0 - alpha0;
                let row0 = cv0 + i * cv_stride;
                let row1 = row0 + cv_stride;

                for j in 0..cv_dim {
                    cv[row0 + j] = cv[row0 + j] * alpha0 + cv[row1 + j] * alpha1;
                }
            }
        }

        true
    }

    /// Solve matrix * x = rhs in place by Gaussian elimination with partial pivoting, dim values per row.
    fn solve_dense(matrix: &mut [Vec<f64>], rhs: &mut [f64], n: usize, dim: usize) -> bool {
        for col in 0..n {
            let mut pivot = col;

            for row in (col + 1)..n {
                if matrix[row][col].abs() > matrix[pivot][col].abs() {
                    pivot = row;
                }
            }

            if pivot != col {
                matrix.swap(col, pivot);

                for d in 0..dim {
                    rhs.swap(col * dim + d, pivot * dim + d);
                }
            }

            if matrix[col][col].abs() < 1e-300 {
                return false;
            }

            for row in (col + 1)..n {
                let factor = matrix[row][col] / matrix[col][col];

                for j in col..n {
                    matrix[row][j] -= factor * matrix[col][j];
                }

                for d in 0..dim {
                    rhs[row * dim + d] -= factor * rhs[col * dim + d];
                }
            }
        }

        for i in (0..n).rev() {
            for d in 0..dim {
                let mut sum = rhs[i * dim + d];

                for j in (i + 1)..n {
                    sum -= matrix[i][j] * rhs[j * dim + d];
                }

                rhs[i * dim + d] = sum / matrix[i][i];
            }
        }

        true
    }

    /// Return the un-normalized derivative by finite difference with step h.
    fn derivative_at(&self, t: f64, h: f64) -> Vector {
        let (t0, t1) = self.domain();
        let p1;
        let p2;
        let dt;

        if t <= t0 + h {
            p1 = self.point_at(t0);
            p2 = self.point_at(t0 + h);
            dt = h;
        } else if t >= t1 - h {
            p1 = self.point_at(t1 - h);
            p2 = self.point_at(t1);
            dt = h;
        } else {
            p1 = self.point_at(t - h);
            p2 = self.point_at(t + h);
            dt = 2.0 * h;
        }

        (&p2 - &p1) / dt
    }

    /// Return the arc length of [ta, tb] by 5-point Gauss-Legendre.
    fn arc_length_gauss(&self, ta: f64, tb: f64, h: f64) -> f64 {
        let mid = (ta + tb) * 0.5;
        let half = (tb - ta) * 0.5;
        let mut sum = 0.0;

        for i in 0..5 {
            sum += GL_WEIGHTS[i] * self.derivative_at(mid + half * GL_NODES[i], h).magnitude();
        }

        half * sum
    }

    /// Return the parameter at arc length s_target from the (t, s) table by bracketed Newton.
    fn find_t_at_s(&self, s_target: f64, t_vals: &[f64], s_vals: &[f64], h: f64) -> f64 {
        let n_samples = t_vals.len() - 1;

        if s_target <= 0.0 {
            return t_vals[0];
        }

        if s_target >= s_vals[n_samples] {
            return t_vals[n_samples];
        }

        let mut lo = 0;
        let mut hi = n_samples;

        while hi - lo > 1 {
            let mid = (lo + hi) / 2;

            if s_vals[mid] < s_target {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
        let mut t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);
        let mut t_lo = t_vals[lo];
        let mut t_hi = t_vals[hi];

        for _ in 0..20 {
            let error = s_vals[lo] + self.arc_length_gauss(t_vals[lo], t, h) - s_target;

            if error.abs() < 1e-12 {
                break;
            }

            let speed = self.derivative_at(t, h).magnitude();
            let t_new = t - error / speed;

            if speed < 1e-14 || t_new <= t_lo || t_new >= t_hi {
                if error > 0.0 {
                    t_hi = t;
                } else {
                    t_lo = t;
                }

                t = (t_lo + t_hi) * 0.5;
            } else {
                t = t_new;
            }
        }

        t
    }

    /// Return the Frenet frame from first and second derivatives, world Z then Y as normal fallback.
    fn frenet_frame(origin: Point, d1: &Vector, d2: &Vector) -> Plane {
        if d1.magnitude() < 1e-14 {
            return Plane::invalid();
        }

        let mut tangent = d1.clone();
        tangent.normalize_self();

        let d2_dot_t = d2.dot(&tangent);
        let mut normal = d2 - &tangent * d2_dot_t;
        let mut n_mag = normal.magnitude();

        if n_mag < 1e-14 {
            normal = tangent.cross(&Vector::new(0.0, 0.0, 1.0));
            n_mag = normal.magnitude();

            if n_mag < 1e-14 {
                normal = tangent.cross(&Vector::new(0.0, 1.0, 0.0));
                n_mag = normal.magnitude();
            }
        }

        if n_mag > 1e-14 {
            normal.normalize_self();
        }

        let mut binormal = tangent.cross(&normal);
        binormal.normalize_self();

        Plane::from_frame(origin, tangent, normal, binormal)
    }

    /// Return the unit Bessel tangent at points[i0] from the parabola through i0, i1, i2.
    fn bessel_tangent(points: &[Point], i0: usize, i1: usize, i2: usize) -> Vector {
        let d01 = points[i0].distance(&points[i1], None);
        let d21 = points[i2].distance(&points[i1], None);

        if d01 + d21 < 1e-300 {
            return Vector::new(0.0, 0.0, 0.0);
        }

        let s = d01 / (d01 + d21);
        let t = 1.0 - s;
        let denom = 2.0 * s * t;

        if denom < 1e-16 {
            let mut chord = &points[i1] - &points[i0];

            return if chord.normalize_self() {
                chord
            } else {
                Vector::new(0.0, 0.0, 0.0)
            };
        }

        let cvx = (-t * t * points[i0][0] + points[i1][0] - s * s * points[i2][0]) / denom;
        let cvy = (-t * t * points[i0][1] + points[i1][1] - s * s * points[i2][1]) / denom;
        let cvz = (-t * t * points[i0][2] + points[i1][2] - s * s * points[i2][2]) / denom;
        let mut tangent = &Point::new(cvx, cvy, cvz) - &points[i0];

        if tangent.normalize_self() {
            tangent
        } else {
            Vector::new(0.0, 0.0, 0.0)
        }
    }

    /// Return the derivative at t of the Lagrange polynomial through m points from i0 (OCCT BuildTangents).
    fn lagrange_tangent(points: &[Point], params: &[f64], i0: usize, m: usize, t: f64) -> Vector {
        let mut result = Vector::new(0.0, 0.0, 0.0);

        for j in 0..m {
            let uj = params[i0 + j];
            let mut dsum = 0.0;

            for i in 0..m {
                if i == j {
                    continue;
                }

                let mut term = 1.0 / (uj - params[i0 + i]);

                for k in 0..m {
                    if k == j || k == i {
                        continue;
                    }

                    term *= (t - params[i0 + k]) / (uj - params[i0 + k]);
                }

                dsum += term;
            }

            let pj = &points[i0 + j];
            result += Vector::new(pj[0], pj[1], pj[2]) * dsum;
        }

        result
    }

    /// Construct the closed interpolated cubic through points, wrapped by three CVs.
    fn create_interpolated_periodic(
        points: &[Point],
        parameterization: CurveNurbsKnotStyle,
    ) -> Self {
        let n = points.len();
        let dim = 3;
        let order = 4;
        let cv_count = n + 3;
        let kc = cv_count + order - 2;
        let params = Self::periodic_interpolation_parameters(points, parameterization);
        let mut dmin = 1e300;
        let mut dmax = 0.0;

        for i in 0..n {
            let d = params[i + 1] - params[i];

            if d < dmin {
                dmin = d;
            }

            if d > dmax {
                dmax = d;
            }
        }

        if dmax <= 0.0 || dmax * SQRT_EPSILON >= dmin {
            return Self::default();
        }

        let nurbsknots = Self::periodic_interpolation_nurbsknots(&params, cv_count);
        let mut a = vec![vec![0.0; n]; n];
        let mut cv = vec![0.0; n * dim];

        for i in 0..n {
            let basis = nurbsknot::eval_basis(order, &nurbsknots, i, params[i]);
            a[i][i % n] += basis[0];
            a[i][(i + 1) % n] += basis[1];
            a[i][(i + 2) % n] += basis[2];

            for d in 0..dim {
                cv[i * dim + d] = points[i][d];
            }
        }

        if !Self::solve_dense(&mut a, &mut cv, n, dim) {
            return Self::default();
        }

        let mut curve = NurbsCurve::new(dim, false, order, cv_count);

        for i in 0..kc {
            curve.set_nurbsknot(i, nurbsknots[i]);
        }

        for i in 0..n {
            curve.set_cv(i, &Point::new(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));
        }

        let cv0 = curve.get_cv(0).unwrap_or_default();
        let cv1 = curve.get_cv(1).unwrap_or_default();
        let cv2 = curve.get_cv(2).unwrap_or_default();
        curve.set_cv(n, &cv0);
        curve.set_cv(n + 1, &cv1);
        curve.set_cv(n + 2, &cv2);

        curve
    }

    /// Return the n + 1 parameters of the closed point loop, uniform or (square root) chord spaced.
    fn periodic_interpolation_parameters(
        points: &[Point],
        parameterization: CurveNurbsKnotStyle,
    ) -> Vec<f64> {
        let n = points.len();
        let mut base_style = CurveNurbsKnotStyle::Chord;

        if matches!(parameterization, CurveNurbsKnotStyle::UniformPeriodic) {
            base_style = CurveNurbsKnotStyle::Uniform;
        }

        if matches!(
            parameterization,
            CurveNurbsKnotStyle::ChordSquareRootPeriodic
        ) {
            base_style = CurveNurbsKnotStyle::ChordSquareRoot;
        }

        let mut params = vec![0.0; n + 1];

        if matches!(base_style, CurveNurbsKnotStyle::Uniform) {
            for i in 1..=n {
                params[i] = i as f64;
            }

            return params;
        }

        for i in 1..n {
            let mut d = points[i - 1].distance(&points[i], None);

            if matches!(base_style, CurveNurbsKnotStyle::ChordSquareRoot) {
                d = d.sqrt();
            }

            params[i] = params[i - 1] + d;
        }

        let mut d_close = points[n - 1].distance(&points[0], None);

        if matches!(base_style, CurveNurbsKnotStyle::ChordSquareRoot) {
            d_close = d_close.sqrt();
        }

        params[n] = params[n - 1] + d_close;

        params
    }

    /// Return the periodic nurbsknots over params, extended by the wrapped spans at both ends.
    fn periodic_interpolation_nurbsknots(params: &[f64], cv_count: usize) -> Vec<f64> {
        let n = params.len() - 1;
        let mut nurbsknots = vec![0.0; cv_count + 2];

        for i in 0..=n {
            nurbsknots[i + 2] = params[i];
        }

        nurbsknots[cv_count] = nurbsknots[3] - nurbsknots[2] + nurbsknots[cv_count - 1];
        nurbsknots[1] = nurbsknots[cv_count - 2] - nurbsknots[cv_count - 1] + nurbsknots[2];
        nurbsknots[cv_count + 1] = nurbsknots[4] - nurbsknots[3] + nurbsknots[cv_count];
        nurbsknots[0] = nurbsknots[cv_count - 3] - nurbsknots[cv_count - 2] + nurbsknots[1];

        nurbsknots
    }

    /// Construct the open interpolated cubic through points with end tangents from end_condition.
    fn create_interpolated_clamped(
        points: &[Point],
        parameterization: CurveNurbsKnotStyle,
        end_condition: CurveInterpStyle,
    ) -> Self {
        let n = points.len();
        let dim = 3;
        let degree = 3;
        let cv_count = n + 2;
        let pts = Self::flatten_points(points, n);
        let params = nurbsknot::compute_parameters(&pts, n, dim, parameterization);
        let nurbsknots = nurbsknot::build_interp_nurbsknots(&params, degree);
        let kc = nurbsknots.len();
        let mut cv = Self::interpolation_end_cvs(points, &params, end_condition);

        if !Self::solve_interpolation_cvs(points, &params, &nurbsknots, &mut cv) {
            return Self::default();
        }

        let mut curve = NurbsCurve::new(dim, false, degree + 1, cv_count);

        for i in 0..kc {
            curve.set_nurbsknot(i, nurbsknots[i]);
        }

        for i in 0..cv_count {
            curve.set_cv(i, &Point::new(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));
        }

        curve
    }

    /// Return the n + 2 CVs: the points with an end tangent CV after the first and before the last.
    fn interpolation_end_cvs(
        points: &[Point],
        params: &[f64],
        end_condition: CurveInterpStyle,
    ) -> Vec<f64> {
        let n = points.len();
        let dim = 3;
        let tan_start;
        let tan_end;
        let s0;
        let s1;

        if matches!(end_condition, CurveInterpStyle::Occt) {
            let deg_t = if n == 3 { 2 } else { 3 };
            tan_start = Self::lagrange_tangent(points, params, 0, deg_t + 1, params[0]);
            tan_end =
                Self::lagrange_tangent(points, params, n - 1 - deg_t, deg_t + 1, params[n - 1]);

            s0 = (params[1] - params[0]) / 3.0;
            s1 = -(params[n - 1] - params[n - 2]) / 3.0;
        } else {
            tan_start = Self::bessel_tangent(points, 0, 1, 2);

            let end_raw = Self::bessel_tangent(points, n - 1, n - 2, n - 3);
            tan_end = -end_raw;
            s0 = points[0].distance(&points[1], None) / 3.0;
            s1 = -points[n - 1].distance(&points[n - 2], None) / 3.0;
        }

        let mut cv = vec![0.0; (n + 2) * dim];

        for d in 0..dim {
            cv[d] = points[0][d];
        }

        for d in 0..dim {
            cv[dim + d] = points[0][d] + s0 * tan_start[d];
        }

        for i in 1..=(n - 2) {
            for d in 0..dim {
                cv[(i + 1) * dim + d] = points[i][d];
            }
        }

        for d in 0..dim {
            cv[n * dim + d] = points[n - 1][d] + s1 * tan_end[d];
        }

        for d in 0..dim {
            cv[(n + 1) * dim + d] = points[n - 1][d];
        }

        cv
    }

    /// Solve the tridiagonal interpolation system and write the interior CVs into cv.
    fn solve_interpolation_cvs(
        points: &[Point],
        params: &[f64],
        nurbsknots: &[f64],
        cv: &mut [f64],
    ) -> bool {
        let n = points.len();
        let dim = 3;
        let order = 4;
        let sys_n = n;
        let mut lower = vec![0.0; sys_n];
        let mut diag = vec![0.0; sys_n];
        let mut upper = vec![0.0; sys_n];
        let mut rhs = vec![0.0; sys_n * dim];
        diag[0] = 1.0;

        for d in 0..dim {
            rhs[d] = cv[dim + d];
        }

        for i in 1..=(n - 2) {
            let basis = nurbsknot::eval_basis(order, nurbsknots, i, params[i]);
            lower[i] = basis[0];
            diag[i] = basis[1];
            upper[i] = basis[2];

            for d in 0..dim {
                rhs[i * dim + d] = points[i][d];
            }
        }

        diag[n - 1] = 1.0;

        for d in 0..dim {
            rhs[(n - 1) * dim + d] = cv[n * dim + d];
        }

        let Some(solution) = nurbsknot::solve_tridiagonal(dim, sys_n, &lower, &diag, &upper, &rhs)
        else {
            return false;
        };

        for i in 0..sys_n {
            for d in 0..dim {
                cv[(i + 1) * dim + d] = solution[i * dim + d];
            }
        }

        true
    }

    /// Return the x, y, z of the first count points as one flat array.
    fn flatten_points(points: &[Point], count: usize) -> Vec<f64> {
        let mut flat = vec![0.0; count * 3];

        for i in 0..count {
            flat[i * 3] = points[i][0];
            flat[i * 3 + 1] = points[i][1];
            flat[i * 3 + 2] = points[i][2];
        }

        flat
    }

    /// Construct the closed least-squares fit with num_cvs distinct CVs.
    fn create_fitted_periodic(points: &[Point], num_cvs: usize, degree: usize) -> Self {
        let dim = 3;
        let order = degree + 1;
        let mut n = points.len();

        if n >= 2 && points[0].distance(&points[n - 1], None) < 1e-10 {
            n -= 1;
        }

        if n <= num_cvs || num_cvs < order {
            if n < 3 {
                return Self::default();
            }

            return Self::create_interpolated(
                &points[..n],
                CurveNurbsKnotStyle::ChordPeriodic,
                CurveInterpStyle::Rhino,
            );
        }

        let cv_count = num_cvs + degree;
        let kc = cv_count + order - 2;
        let mut params = vec![0.0; n + 1];

        for i in 1..n {
            params[i] = params[i - 1] + points[i - 1].distance(&points[i], None);
        }

        params[n] = params[n - 1] + points[n - 1].distance(&points[0], None);

        if params[n] < 1e-14 {
            return Self::default();
        }

        let ppts = Self::flatten_points(points, n);
        let nurbsknots = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(
            &params, &ppts, n, dim, num_cvs, degree, 3.0,
        );
        let mut ntn = vec![vec![0.0; num_cvs]; num_cvs];
        let mut cv = vec![0.0; num_cvs * dim];

        for k in 0..n {
            let span = nurbsknot::find_span(order, cv_count, &nurbsknots, params[k], 0, 0);
            let basis = nurbsknot::eval_basis(order, &nurbsknots, span, params[k]);

            for a in 0..order {
                let ci = (span + a) % num_cvs;

                for d in 0..dim {
                    cv[ci * dim + d] += basis[a] * points[k][d];
                }

                for b in 0..order {
                    ntn[ci][(span + b) % num_cvs] += basis[a] * basis[b];
                }
            }
        }

        if !Self::solve_dense(&mut ntn, &mut cv, num_cvs, dim) {
            return Self::default();
        }

        let mut curve = NurbsCurve::new(dim, false, order, cv_count);

        for i in 0..kc {
            curve.set_nurbsknot(i, nurbsknots[i]);
        }

        for i in 0..num_cvs {
            curve.set_cv(i, &Point::new(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));
        }

        for i in 0..degree {
            let p = curve.get_cv(i).unwrap_or_default();
            curve.set_cv(num_cvs + i, &p);
        }

        curve
    }

    /// Construct the open least-squares fit through the first and last point.
    fn create_fitted_clamped(points: &[Point], num_cvs: usize, degree: usize) -> Self {
        let m = points.len();
        let dim = 3;

        if m <= num_cvs || num_cvs < degree + 1 {
            return Self::create_interpolated(
                points,
                CurveNurbsKnotStyle::Chord,
                CurveInterpStyle::Rhino,
            );
        }

        let pts = Self::flatten_points(points, m);
        let params = nurbsknot::compute_parameters(&pts, m, dim, CurveNurbsKnotStyle::Chord);
        let nurbsknots = nurbsknot::build_fitted_nurbsknots_adaptive(
            &params, &pts, m, dim, num_cvs, degree, 3.0,
        );
        let sys_n = num_cvs - 2;
        let mut band = vec![0.0; sys_n * (degree + 1)];
        let mut rhs = vec![0.0; sys_n * dim];
        Self::fitted_band_system(
            points,
            &params,
            &nurbsknots,
            num_cvs,
            degree,
            &mut band,
            &mut rhs,
        );

        if !nurbsknot::solve_banded_spd(dim, sys_n, degree, &mut band, &mut rhs) {
            return Self::create_interpolated(
                points,
                CurveNurbsKnotStyle::Chord,
                CurveInterpStyle::Rhino,
            );
        }

        let kc = nurbsknots.len();
        let mut curve = NurbsCurve::new(dim, false, degree + 1, num_cvs);

        for i in 0..kc {
            curve.set_nurbsknot(i, nurbsknots[i]);
        }

        curve.set_cv(0, &points[0]);

        for i in 0..sys_n {
            curve.set_cv(
                i + 1,
                &Point::new(rhs[i * 3], rhs[i * 3 + 1], rhs[i * 3 + 2]),
            );
        }

        curve.set_cv(num_cvs - 1, &points[m - 1]);

        curve
    }

    /// Accumulate the banded normal equations of the open fit, end CVs fixed.
    fn fitted_band_system(
        points: &[Point],
        params: &[f64],
        nurbsknots: &[f64],
        num_cvs: usize,
        degree: usize,
        band: &mut [f64],
        rhs: &mut [f64],
    ) {
        let m = points.len();
        let dim = 3;
        let order = degree + 1;
        let n = num_cvs - 1;
        let bw1 = degree + 1;

        for k in 1..(m - 1) {
            let span = nurbsknot::find_span(order, num_cvs, nurbsknots, params[k], 0, 0);
            let basis = nurbsknot::eval_basis(order, nurbsknots, span, params[k]);
            let mut rk = [points[k][0], points[k][1], points[k][2]];

            for a in 0..order {
                let ci = span + a;

                if ci == 0 {
                    for d in 0..dim {
                        rk[d] -= basis[a] * points[0][d];
                    }
                }

                if ci == n {
                    for d in 0..dim {
                        rk[d] -= basis[a] * points[m - 1][d];
                    }
                }
            }

            for a in 0..order {
                let ci = span + a;

                if ci < 1 || ci > n - 1 {
                    continue;
                }

                let ri = ci - 1;

                for d in 0..dim {
                    rhs[ri * dim + d] += basis[a] * rk[d];
                }

                for b in a..order {
                    let cj = span + b;

                    if cj < 1 || cj > n - 1 {
                        continue;
                    }

                    let rj = cj - 1;
                    band[rj * bw1 + (rj - ri)] += basis[a] * basis[b];
                }
            }
        }
    }

    /// Lift 2D segments to 3D when 2D and 3D segments are mixed.
    fn promote_to_3d(segs: &mut [NurbsCurve]) {
        let mut any2 = false;
        let mut any3 = false;

        for c in segs.iter() {
            if c.m_dim == 2 {
                any2 = true;
            } else if c.m_dim == 3 {
                any3 = true;
            }
        }

        if !any2 || !any3 {
            return;
        }

        for c in segs.iter_mut() {
            if c.m_dim != 2 {
                continue;
            }

            let os = c.m_cv_stride;
            let ns = os + 1;
            let mut cv = vec![0.0; c.m_cv_count * ns];

            for i in 0..c.m_cv_count {
                cv[i * ns] = c.m_cv[i * os];
                cv[i * ns + 1] = c.m_cv[i * os + 1];

                if c.m_is_rat {
                    cv[i * ns + 3] = c.m_cv[i * os + 2];
                }
            }

            c.m_cv = cv;
            c.m_cv_stride = ns;
            c.m_dim = 3;
        }
    }

    /// Group segments into chains by endpoint matching, reversing where needed.
    fn chain_segments(segs: &[NurbsCurve], tolerance: f64) -> Vec<Vec<NurbsCurve>> {
        let mut chains: Vec<Vec<NurbsCurve>> = Vec::new();
        let mut used = vec![false; segs.len()];

        for i in 0..segs.len() {
            if used[i] {
                continue;
            }

            used[i] = true;

            let mut chain: Vec<NurbsCurve> = vec![segs[i].clone()];
            let mut grown = !segs[i].is_closed();

            while grown {
                grown = false;
                let start = chain[0].point_at_start();
                let end = chain[chain.len() - 1].point_at_end();

                for j in 0..segs.len() {
                    if used[j] || segs[j].is_closed() {
                        continue;
                    }

                    let s = segs[j].point_at_start();
                    let e = segs[j].point_at_end();

                    if s.distance(&end, None) <= tolerance {
                        chain.push(segs[j].clone());
                    } else if e.distance(&end, None) <= tolerance {
                        let mut r = segs[j].clone();
                        r.reverse();
                        chain.push(r);
                    } else if e.distance(&start, None) <= tolerance {
                        chain.insert(0, segs[j].clone());
                    } else if s.distance(&start, None) <= tolerance {
                        let mut r = segs[j].clone();
                        r.reverse();
                        chain.insert(0, r);
                    } else {
                        continue;
                    }

                    used[j] = true;
                    grown = true;
                    break;
                }
            }

            chains.push(chain);
        }

        chains
    }

    /// Append the chain merged into one curve to result, or its segments when they cannot be merged.
    fn join_chain(mut chain: Vec<NurbsCurve>, result: &mut Vec<NurbsCurve>) {
        if chain.len() == 1 {
            result.push(chain.remove(0));

            return;
        }

        let mut rational = false;
        let mut max_degree = 1;

        for c in &chain {
            if c.is_rational() {
                rational = true;
            }

            if c.degree() > max_degree {
                max_degree = c.degree();
            }
        }

        let mut aligned = true;

        for c in chain.iter_mut() {
            if rational {
                c.make_rational();
            }

            if !c.clamp_end(2) || !c.increase_degree(max_degree) {
                aligned = false;
            }
        }

        let mut joined = chain[0].clone();

        if aligned {
            for ci in 1..chain.len() {
                Self::append_segment(&mut joined, &mut chain[ci], rational);
            }
        }

        if !aligned
            || joined.m_cv.len() < (joined.m_cv_count - 1) * joined.m_cv_stride + joined.cv_size()
            || joined.m_nurbsknot.len() != joined.m_cv_count + joined.m_order - 2
        {
            for c in chain {
                result.push(c);
            }

            return;
        }

        result.push(joined);
    }

    /// Append segment to joined with a C0 junction at the averaged shared CV.
    fn append_segment(joined: &mut NurbsCurve, segment: &mut NurbsCurve, rational: bool) {
        let stride = joined.m_cv_stride;
        let cvdim = joined.cv_size();
        let a1 = joined.domain_end();
        let (s0, s1) = segment.domain();
        segment.set_domain(a1, a1 + (s1 - s0));

        if rational {
            let w_end = joined.weight(joined.m_cv_count - 1);
            let w_start = segment.weight(0);

            if w_start.abs() > Tolerance::ZERO_TOLERANCE {
                let scale = w_end / w_start;

                for k in 0..segment.m_cv.len() {
                    segment.m_cv[k] *= scale;
                }
            }
        }

        let last = (joined.m_cv_count - 1) * stride;

        if stride == 0
            || cvdim == 0
            || segment.m_order != joined.m_order
            || segment.m_cv_stride != stride
            || segment.cv_size() != cvdim
            || joined.m_cv.len() < last + cvdim
            || segment.m_cv.len() < segment.m_cv_count * stride
            || segment.m_cv.len() <= stride
            || segment.m_nurbsknot.len() != segment.m_cv_count + segment.m_order - 2
        {
            return;
        }

        for k in 0..cvdim {
            joined.m_cv[last + k] = 0.5 * (joined.m_cv[last + k] + segment.m_cv[k]);
        }

        joined
            .m_nurbsknot
            .extend_from_slice(&segment.m_nurbsknot[(joined.m_order - 1)..]);

        joined.m_cv.extend_from_slice(&segment.m_cv[stride..]);
        joined.m_cv_count = joined.m_cv_count + segment.m_cv_count - 1;
    }

    /// Return the center of the circle through three points, None when they are collinear.
    fn circle_center(p0: &Point, p1: &Point, p2: &Point) -> Option<Point> {
        let d1 = p1 - p0;
        let d2 = p2 - p1;
        let mut normal = d1.cross(&d2);

        if normal.magnitude() < Tolerance::ZERO_TOLERANCE {
            return None;
        }

        normal = normal.normalized();

        let m1 = Point::sum(p0, p1) * 0.5;
        let m2 = Point::sum(p1, p2) * 0.5;
        let perp1 = d1.cross(&normal).normalized();
        let perp2 = d2.cross(&normal).normalized();
        let mut denom = perp1[0] * perp2[1] - perp1[1] * perp2[0];

        if denom.abs() < Tolerance::ZERO_TOLERANCE {
            denom = perp1[0] * perp2[2] - perp1[2] * perp2[0];
        }

        if denom.abs() < Tolerance::ZERO_TOLERANCE {
            return None;
        }

        let dx = m2[0] - m1[0];
        let dy = m2[1] - m1[1];
        let s = (dx * perp2[1] - dy * perp2[0]) / denom;

        Some(&m1 + &perp1 * s)
    }

    /// Return the nurbsknots padded with one superfluous value at each end.
    fn full_nurbsknots(&self) -> Vec<f64> {
        let full_nurbsknot_count = self.m_cv_count + self.m_order;
        let mut u = vec![0.0; full_nurbsknot_count];
        u[0] = self.m_nurbsknot[0];

        for i in 0..self.m_nurbsknot.len() {
            u[i + 1] = self.m_nurbsknot[i];
        }

        u[full_nurbsknot_count - 1] = self.m_nurbsknot[self.m_nurbsknot.len() - 1];

        u
    }

    /// Insert one nurbsknot by Boehm, u the padded nurbsknots.
    fn insert_nurbsknot_once(&mut self, nurbsknot_value: f64, u: &[f64]) {
        let p = self.degree();
        let n = self.m_cv_count - 1;
        let full_nurbsknot_count = self.m_cv_count + self.m_order;
        let k = self.find_span(nurbsknot_value) + self.m_order - 1;
        let new_cv_count = self.m_cv_count + 1;
        let stride = self.m_cv_stride;
        let mut u_new = vec![0.0; full_nurbsknot_count + 1];
        let mut cv_new = vec![0.0; new_cv_count * stride];

        for i in 0..=k {
            u_new[i] = u[i];
        }

        u_new[k + 1] = nurbsknot_value;

        for i in (k + 1)..full_nurbsknot_count {
            u_new[i + 1] = u[i];
        }

        for i in 0..=(k - p) {
            cv_new[i * stride..(i + 1) * stride]
                .copy_from_slice(&self.m_cv[i * stride..(i + 1) * stride]);
        }

        for i in (k + 1)..=(n + 1) {
            cv_new[i * stride..(i + 1) * stride]
                .copy_from_slice(&self.m_cv[(i - 1) * stride..i * stride]);
        }

        for i in (k - p + 1)..=k {
            let mut alpha = 0.0;
            let denom = u[i + p] - u[i];

            if denom != 0.0 {
                alpha = (nurbsknot_value - u[i]) / denom;
            }

            for d in 0..stride {
                cv_new[i * stride + d] = (1.0 - alpha) * self.m_cv[(i - 1) * stride + d]
                    + alpha * self.m_cv[i * stride + d];
            }
        }

        self.m_cv_count = new_cv_count;
        self.m_cv = cv_new;

        let kc = self.m_order + self.m_cv_count - 2;
        let mut nurbsknot_new = vec![0.0; kc];

        for i in 0..kc {
            nurbsknot_new[i] = u_new[i + 1];
        }

        self.m_nurbsknot = nurbsknot_new;
    }

    /// Return the (t, point) samples of the chord-deviation bisection, sorted by t.
    fn adaptive_samples(
        &self,
        angle_tolerance: f64,
        min_edge_length: f64,
        max_edge_length: f64,
    ) -> Vec<(f64, Point)> {
        let (t0, t1) = self.domain();
        let mut samples: Vec<(f64, Point)> = vec![(t0, self.point_at(t0)), (t1, self.point_at(t1))];
        let mut work_queue: Vec<(f64, f64)> = vec![(t0, t1)];
        let max_iterations = 10000;
        let mut iterations = 0;

        while !work_queue.is_empty() && iterations < max_iterations {
            iterations += 1;

            let (ta, tb) = work_queue.pop().unwrap_or((t0, t1));
            let pa = self.point_at(ta);
            let pb = self.point_at(tb);
            let chord_length = pa.distance(&pb, None);

            if chord_length < min_edge_length {
                continue;
            }

            let tm = (ta + tb) * 0.5;
            let pm = self.point_at(tm);
            let chord = &pb - &pa;
            let to_mid = &pm - &pa;
            let chord_len_sq = chord.dot(&chord);
            let mut deviation = 0.0;

            if chord_len_sq > 1e-20 {
                let proj = to_mid.dot(&chord) / chord_len_sq;
                deviation = pm.distance(&(&pa + &chord * proj), None);
            }

            let deviation_tolerance = chord_length * angle_tolerance * 0.5;

            if deviation > deviation_tolerance || chord_length > max_edge_length {
                samples.push((tm, pm));
                work_queue.push((ta, tm));
                work_queue.push((tm, tb));
            }
        }

        samples.sort_by(sample_before);

        samples
    }

    /// Return the homogeneous derivatives (x, y, z, w) at span from the basis derivatives.
    fn homogeneous_derivatives(&self, span: usize, ders: &[Vec<f64>]) -> Vec<[f64; 4]> {
        let p = self.degree();
        let count = ders.len();
        let mut aders = vec![[0.0; 4]; count];

        for k in 0..count {
            for j in 0..=p {
                let Some(cv_ptr) = self.cv(span + j) else {
                    continue;
                };

                let nx = ders[k][j];
                aders[k][0] += nx * cv_ptr[0];
                aders[k][1] += nx * if self.m_dim > 1 { cv_ptr[1] } else { 0.0 };
                aders[k][2] += nx * if self.m_dim > 2 { cv_ptr[2] } else { 0.0 };
                aders[k][3] += nx
                    * if self.m_is_rat {
                        cv_ptr[self.m_dim]
                    } else {
                        1.0
                    };
            }
        }

        aders
    }

    /// Return the unit tangent and normal at the domain start, None when the derivative vanishes.
    fn start_frame(&self) -> Option<(Vector, Vector)> {
        let derivs0 = self.evaluate(self.domain_start(), 2);
        let d1_0 = derivs0[1].clone();
        let d2_0 = derivs0[2].clone();
        let d1_0_mag = d1_0.magnitude();

        if d1_0_mag < 1e-14 {
            return None;
        }

        let t_0 = &d1_0 / d1_0_mag;
        let d2_dot_d1 = d2_0.dot(&d1_0);
        let d1_0_mag_sq = d1_0_mag * d1_0_mag;
        let mut n0_unnorm = &d2_0 - &d1_0 * (d2_dot_d1 / d1_0_mag_sq);
        let mut n0_mag = n0_unnorm.magnitude();

        if n0_mag < 1e-14 {
            n0_unnorm = Vector::new(0.0, 0.0, 1.0).cross(&t_0);
            n0_mag = n0_unnorm.magnitude();

            if n0_mag < 1e-14 {
                n0_unnorm = Vector::new(0.0, 1.0, 0.0).cross(&t_0);
                n0_mag = n0_unnorm.magnitude();
            }
        }

        let r0 = &n0_unnorm / n0_mag;

        Some((t_0, r0))
    }

    /// Return r0 carried from the domain start to param by double reflection.
    fn double_reflection(&self, param: f64, r0: &Vector, t_0: &Vector) -> Vector {
        let (t0, t1) = self.domain();
        let num_steps = 10.max(((param - t0) / (t1 - t0) * 100.0) as i32) as usize;
        let dt = (param - t0) / num_steps as f64;
        let mut ri = r0.clone();
        let mut ti = t0;
        let mut xi = self.point_at(ti);
        let mut t_i = t_0.clone();

        for _ in 0..num_steps {
            if ti >= param - 1e-14 {
                break;
            }

            let ti_next = (ti + dt).min(param);
            let xi_next = self.point_at(ti_next);
            let mut t_i_next = self.tangent_at(ti_next);
            t_i_next.normalize_self();

            let v1 = &xi_next - &xi;
            let c1 = v1.dot(&v1);

            if c1 < 1e-28 {
                ti = ti_next;
                xi = xi_next;
                t_i = t_i_next;
                continue;
            }

            let ri_dot_v1 = ri.dot(&v1);
            let r_l = &ri - &v1 * (2.0 * ri_dot_v1 / c1);
            let t_i_dot_v1 = t_i.dot(&v1);
            let t_l = &t_i - &v1 * (2.0 * t_i_dot_v1 / c1);
            let v2 = &t_i_next - &t_l;
            let c2 = v2.dot(&v2);

            if c2 < 1e-28 {
                ri = r_l;
            } else {
                let r_l_dot_v2 = r_l.dot(&v2);
                ri = &r_l - &v2 * (2.0 * r_l_dot_v2 / c2);
            }

            if ri.magnitude() > 1e-14 {
                ri.normalize_self();
            }

            ti = ti_next;
            xi = xi_next;
            t_i = t_i_next;
        }

        ri
    }

    /// Keep the CVs and nurbsknots between the full-multiplicity nurbsknots t0 and t1.
    fn keep_span_range(&mut self, t0: f64, t1: f64) -> bool {
        let p = self.degree();
        let u = self.full_nurbsknots();
        let full_nurbsknot_count = u.len();
        let tol = Tolerance::ZERO_TOLERANCE;
        let mut start_span: i64 = -1;

        for i in (0..full_nurbsknot_count).rev() {
            if (u[i] - t0).abs() < tol {
                start_span = i as i64;
                break;
            }
        }

        let mut end_span: i64 = -1;

        for i in 0..full_nurbsknot_count {
            if (u[i] - t1).abs() < tol {
                end_span = i as i64;
                break;
            }
        }

        if start_span < 0 || end_span < 0 || start_span >= end_span {
            return false;
        }

        let start_span = start_span as usize;
        let end_span = end_span as usize;
        let first_cv = start_span.saturating_sub(p);
        let last_cv = (end_span - 1).min(self.m_cv_count - 1);
        let mut new_cv_count = last_cv - first_cv + 1;

        if new_cv_count < self.m_order {
            new_cv_count = self.m_order;

            if first_cv + new_cv_count > self.m_cv_count {
                return false;
            }
        }

        let new_nurbsknot = self.trimmed_nurbsknots(&u, start_span, new_cv_count, t0, t1);
        let stride = self.m_cv_stride;
        let mut new_cv = vec![0.0; new_cv_count * stride];

        for i in 0..new_cv_count {
            new_cv[i * stride..(i + 1) * stride]
                .copy_from_slice(&self.m_cv[(first_cv + i) * stride..(first_cv + i + 1) * stride]);
        }

        self.m_cv_count = new_cv_count;
        self.m_cv = new_cv;
        self.m_nurbsknot = new_nurbsknot;

        true
    }

    /// Return the nurbsknots of the kept range, clamped at t0 and t1.
    fn trimmed_nurbsknots(
        &self,
        u: &[f64],
        start_span: usize,
        new_cv_count: usize,
        t0: f64,
        t1: f64,
    ) -> Vec<f64> {
        let p = self.degree();
        let full_nurbsknot_count = u.len();
        let new_nurbsknot_count = new_cv_count + self.m_order - 2;
        let mut new_nurbsknot = vec![0.0; new_nurbsknot_count];

        for i in 0..(p - 1) {
            new_nurbsknot[i] = t0;
        }

        let mid_count = new_nurbsknot_count as i64 - 2 * (p as i64 - 1);

        for i in 0..mid_count.max(0) as usize {
            let src_idx = start_span + i;
            new_nurbsknot[p - 1 + i] = if src_idx < full_nurbsknot_count {
                u[src_idx]
            } else {
                t1
            };
        }

        for i in 0..(p - 1) {
            new_nurbsknot[new_nurbsknot_count - p + 1 + i] = t1;
        }

        new_nurbsknot
    }

    /// Return the index of the first nurbsknot greater than value, -1 when none.
    fn first_nurbsknot_above(&self, value: f64) -> i64 {
        let kc = self.nurbsknot_count();

        for i in 0..kc {
            if self.m_nurbsknot[i] > value {
                return i as i64;
            }
        }

        -1
    }

    /// Return the seam nurbsknot index near t, snapping to an existing nurbsknot or inserting one; -1 on failure.
    fn seam_nurbsknot_index(&mut self, t: f64, nurbsknot_index: i64) -> i64 {
        let d0 = t - self.m_nurbsknot[nurbsknot_index as usize - 1];
        let d1 = self.m_nurbsknot[nurbsknot_index as usize] - t;

        if d0 <= d1 && d0 < Tolerance::ZERO_TOLERANCE {
            return nurbsknot_index - 1;
        }

        if d0 > d1 && d1 < Tolerance::ZERO_TOLERANCE {
            return nurbsknot_index;
        }

        if !self.insert_nurbsknot(t, 1) {
            return -1;
        }

        self.first_nurbsknot_above(t + Tolerance::ZERO_TOLERANCE)
    }

    /// Rotate the nurbsknots and CVs of a periodic curve so the domain starts at t.
    fn rotate_periodic_seam(&mut self, nurbsknot_index: usize, t: f64, dom_len: f64) -> bool {
        let p = self.degree();
        let sc = self.span_count();
        let cvc = self.m_cv_count;
        let distinct_cvc = cvc - p;
        let cvdim = self.cv_size();
        let old_nurbsknots = self.m_nurbsknot.clone();
        let old_cv = self.m_cv.clone();
        let mut curr = p - 1;

        for i in nurbsknot_index..(sc + p - 1) {
            self.m_nurbsknot[curr] = old_nurbsknots[i];
            curr += 1;
        }

        for i in 0..=(nurbsknot_index + 1 - p) {
            self.m_nurbsknot[curr] = old_nurbsknots[p - 1 + i] + dom_len;
            curr += 1;
        }

        for i in 0..(p - 1) {
            self.m_nurbsknot[curr + i] = self.m_nurbsknot[curr + i - 1] + self.m_nurbsknot[p + i]
                - self.m_nurbsknot[p + i - 1];

            self.m_nurbsknot[p - 2 - i] = self.m_nurbsknot[p - i - 1]
                - self.m_nurbsknot[curr - 1 - i]
                + self.m_nurbsknot[curr - 2 - i];
        }

        let cv_id = nurbsknot_index as i64 - p as i64 + 1;

        for i in 0..cvc {
            let mut src = (cv_id + i as i64) % distinct_cvc as i64;

            if src < 0 {
                src += distinct_cvc as i64;
            }

            let src = src as usize;

            for j in 0..cvdim {
                self.m_cv[i * self.m_cv_stride + j] = old_cv[src * self.m_cv_stride + j];
            }
        }

        self.set_domain(t, t + dom_len)
    }

    /// Split at t and join the right part before the left so the domain starts at t.
    fn split_seam(&mut self, t: f64, dom_len: f64) -> bool {
        let (left_crv, right_crv) = self.split(t);

        if !left_crv.is_valid() || !right_crv.is_valid() {
            return false;
        }

        let order = self.m_order;
        let cvdim = self.cv_size();
        let stride = self.m_cv_stride;
        let new_cv_count = right_crv.m_cv_count + left_crv.m_cv_count - 1;
        let new_kc = order + new_cv_count - 2;
        let mut new_cv = vec![0.0; new_cv_count * stride];
        let mut new_nurbsknots = vec![0.0; new_kc];

        for i in 0..right_crv.m_cv_count {
            for j in 0..cvdim {
                new_cv[i * stride + j] = right_crv.m_cv[i * right_crv.m_cv_stride + j];
            }
        }

        for i in 1..left_crv.m_cv_count {
            let dst = right_crv.m_cv_count + i - 1;

            for j in 0..cvdim {
                new_cv[dst * stride + j] = left_crv.m_cv[i * left_crv.m_cv_stride + j];
            }
        }

        let rkc = right_crv.nurbsknot_count();

        for i in 0..rkc {
            new_nurbsknots[i] = right_crv.m_nurbsknot[i];
        }

        let lkc = left_crv.nurbsknot_count();

        for i in (order - 1)..lkc {
            new_nurbsknots[rkc + i - (order - 1)] = left_crv.m_nurbsknot[i] + dom_len;
        }

        self.m_cv_count = new_cv_count;
        self.m_cv = new_cv;
        self.m_nurbsknot = new_nurbsknots;

        self.set_domain(t, t + dom_len)
    }

    /// Return the binomial coefficient C(n, k).
    fn binomial(n: usize, k: usize) -> usize {
        if k > n {
            return 0;
        }

        if k == 0 || k == n {
            return 1;
        }

        let k = k.min(n - k);
        let mut c = 1usize;

        for i in 0..k {
            c = c * (n - i) / (i + 1);
        }

        c
    }
}

impl fmt::Display for NurbsCurve {
    /// Write the curve string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl PartialEq for NurbsCurve {
    /// Compare name, width, colors, layout, nurbsknots and CVs to 1e-12; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        if self.m_dim != other.m_dim || self.m_is_rat != other.m_is_rat {
            return false;
        }

        if self.m_order != other.m_order || self.m_cv_count != other.m_cv_count {
            return false;
        }

        if self.m_cv_stride != other.m_cv_stride {
            return false;
        }

        if self.name != other.name {
            return false;
        }

        if (self.width - other.width).abs() > Tolerance::ZERO_TOLERANCE {
            return false;
        }

        if self.pointcolors != other.pointcolors {
            return false;
        }

        if self.linecolors != other.linecolors {
            return false;
        }

        if self.m_nurbsknot.len() != other.m_nurbsknot.len() {
            return false;
        }

        for i in 0..self.m_nurbsknot.len() {
            if (self.m_nurbsknot[i] - other.m_nurbsknot[i]).abs() > Tolerance::ZERO_TOLERANCE {
                return false;
            }
        }

        if self.m_cv.len() != other.m_cv.len() {
            return false;
        }

        for i in 0..self.m_cv.len() {
            if (self.m_cv[i] - other.m_cv[i]).abs() > Tolerance::ZERO_TOLERANCE {
                return false;
            }
        }

        true
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════
impl Serialize for NurbsCurve {
    /// Serialize to flat JSON fields.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut control_points: Vec<Vec<f64>> = Vec::new();

        for i in 0..self.m_cv_count {
            if self.m_is_rat {
                let (x, y, z, w) = self.get_cv_4d(i).unwrap_or((0.0, 0.0, 0.0, 1.0));
                control_points.push(vec![x, y, z, w]);
            } else {
                let p = self.get_cv(i).unwrap_or_default();
                control_points.push(vec![p[0], p[1], p[2]]);
            }
        }

        let mut linecolors: Vec<f32> = Vec::new();

        for c in &self.linecolors {
            linecolors.extend_from_slice(&[c.r, c.g, c.b, c.a]);
        }

        let mut pointcolors: Vec<f32> = Vec::new();

        for c in &self.pointcolors {
            pointcolors.extend_from_slice(&[c.r, c.g, c.b, c.a]);
        }

        let mut map = serializer.serialize_map(Some(13))?;
        map.serialize_entry("control_points", &control_points)?;
        map.serialize_entry("cv_count", &self.m_cv_count)?;
        map.serialize_entry("cv_stride", &self.m_cv_stride)?;
        map.serialize_entry("dimension", &self.m_dim)?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("is_rational", &self.m_is_rat)?;
        map.serialize_entry("linecolors", &linecolors)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("nurbsknots", &self.m_nurbsknot)?;
        map.serialize_entry("order", &self.m_order)?;
        map.serialize_entry("pointcolors", &pointcolors)?;
        map.serialize_entry("type", "NurbsCurve")?;
        map.serialize_entry("width", &self.width)?;

        map.end()
    }
}

/// Flat JSON fields for deserialization.
#[derive(Deserialize)]
struct NurbsCurveData {
    #[serde(default)]
    control_points: Vec<Vec<f64>>,
    cv_count: usize,
    dimension: usize,
    #[serde(default)]
    guid: String,
    #[serde(default)]
    is_rational: bool,
    #[serde(default)]
    linecolors: Vec<f32>,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    nurbsknots: Vec<f64>,
    order: usize,
    #[serde(default)]
    pointcolors: Vec<f32>,
    #[serde(default)]
    width: Option<f64>,
}

impl<'de> Deserialize<'de> for NurbsCurve {
    /// Deserialize from flat JSON fields.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let data = NurbsCurveData::deserialize(deserializer)?;
        let mut curve =
            NurbsCurve::new(data.dimension, data.is_rational, data.order, data.cv_count);

        if !data.nurbsknots.is_empty() {
            curve.m_nurbsknot = data.nurbsknots;
        }

        let n = data.cv_count.min(data.control_points.len());

        for i in 0..n {
            let cp = &data.control_points[i];
            let x = cp[0];
            let y = cp[1];
            let z = if cp.len() > 2 { cp[2] } else { 0.0 };

            if data.is_rational && cp.len() > 3 {
                curve.set_cv_4d(i, x, y, z, cp[3]);
            } else {
                curve.set_cv(i, &Point::new(x, y, z));
            }
        }

        if !data.guid.is_empty() {
            curve.set_guid(data.guid);
        }

        curve.name = data.name.unwrap_or_else(|| "my_nurbscurve".to_string());
        curve.width = data.width.unwrap_or(1.0);

        let arr = data.pointcolors;
        let mut i = 0;

        while i + 3 < arr.len() {
            curve
                .pointcolors
                .push(Color::new(arr[i], arr[i + 1], arr[i + 2], arr[i + 3]));

            i += 4;
        }

        let arr = data.linecolors;
        let mut i = 0;

        while i + 3 < arr.len() {
            curve
                .linecolors
                .push(Color::new(arr[i], arr[i + 1], arr[i + 2], arr[i + 3]));

            i += 4;
        }

        Ok(curve)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Sampling
// ═══════════════════════════════════════════════════════════════════════════
/// Order two (t, point) samples by parameter.
fn sample_before(a: &(f64, Point), b: &(f64, Point)) -> std::cmp::Ordering {
    a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal)
}

// ═══════════════════════════════════════════════════════════════════════════
// Degree elevation
// ═══════════════════════════════════════════════════════════════════════════
/// Compute the blossom of one span at order - 1 parameters by the de Boor recurrence.
fn evaluate_nurbs_blossom(
    cvdim: usize,
    order: usize,
    cv_stride: usize,
    cv: &[f64],
    nurbsknot: &[f64],
    t: &[f64],
    p: &mut [f64],
) -> bool {
    if cv_stride < cvdim {
        return false;
    }

    let degree = order - 1;

    for i in 1..(2 * degree) {
        if nurbsknot[i] - nurbsknot[i - 1] < 0.0 {
            return false;
        }
    }

    if nurbsknot[degree] - nurbsknot[degree - 1] < Tolerance::ZERO_TOLERANCE {
        return false;
    }

    let mut space = vec![0.0; order];

    for i in 0..cvdim {
        for j in 0..order {
            space[j] = cv[j * cv_stride + i];
        }

        for j in 1..order {
            for k in j..order {
                let denom = nurbsknot[degree + k - j] - nurbsknot[k - 1];
                space[k - j] = (nurbsknot[degree + k - j] - t[j - 1]) / denom * space[k - j]
                    + (t[j - 1] - nurbsknot[k - 1]) / denom * space[k - j + 1];
            }
        }

        p[i] = space[0];
    }

    true
}

/// Compute one CV of the degree-raised span as the average of blossoms.
#[allow(clippy::too_many_arguments)]
fn get_raised_degree_cv(
    old_order: usize,
    cvdim: usize,
    old_cv_stride: usize,
    old_cv: &[f64],
    old_kn: &[f64],
    new_kn: &[f64],
    cv_id: usize,
    new_cv: &mut [f64],
) -> bool {
    if cv_id > old_order {
        return false;
    }

    let old_degree = old_order - 1;
    let new_degree = old_degree + 1;
    let mut t = vec![0.0; old_degree];
    let mut pp = vec![0.0; cvdim];

    for v in new_cv.iter_mut() {
        *v = 0.0;
    }

    let kn = &new_kn[cv_id..];

    for i in 0..new_degree {
        let mut k = 0;

        for j in 0..new_degree {
            if j != i {
                t[k] = kn[j];
                k += 1;
            }
        }

        if !evaluate_nurbs_blossom(cvdim, old_order, old_cv_stride, old_cv, old_kn, &t, &mut pp) {
            return false;
        }

        for k in 0..cvdim {
            new_cv[k] += pp[k];
        }
    }

    for i in 0..cvdim {
        new_cv[i] /= new_degree as f64;
    }

    true
}

/// Return the next span index past degenerate spans.
fn next_span_index(order: usize, cv_count: usize, nurbsknot: &[f64], span_index: usize) -> usize {
    let mut span_index = span_index;

    if span_index > cv_count - order {
        return span_index;
    }

    if span_index < cv_count - order {
        span_index += 1;

        while span_index < cv_count - order
            && nurbsknot[span_index + order - 2] == nurbsknot[span_index + order - 1]
        {
            span_index += 1;
        }
    }

    span_index
}

/// Raise the degree of n by one.
fn increment_nurbs_degree(n: &mut NurbsCurve) -> bool {
    let m = n.clone();
    let sc = m.span_count();
    let new_kcount = m.nurbsknot_count() + sc + 1;
    let new_order = m.order() + 1;
    let new_cv_count = new_kcount - new_order + 2;
    let cvdim = m.cv_size();
    n.m_order = new_order;
    n.m_cv_count = new_cv_count;
    n.m_nurbsknot = vec![0.0; new_order + new_cv_count - 2];
    n.m_cv = vec![0.0; new_cv_count * n.m_cv_stride];

    let mut ki = 0;
    let mut ko = 0;
    let mkc = m.nurbsknot_count();

    while ki < mkc {
        let kn = m.m_nurbsknot[ki];
        let mut mult = 1;

        while ki + mult < mkc && (m.m_nurbsknot[ki + mult] - kn).abs() < Tolerance::ZERO_TOLERANCE {
            mult += 1;
        }

        for _ in 0..=mult {
            n.m_nurbsknot[ko] = kn;
            ko += 1;
        }

        ki += mult;
    }

    let mut si_n = 0;
    let mut si_m = 0;

    for _ in 0..sc {
        let span_mult = n.nurbsknot_multiplicity(si_n + n.degree() - 1);
        let skip = n.order() - span_mult;

        for j in skip..n.order() {
            let mut cv_n = vec![0.0; cvdim];
            get_raised_degree_cv(
                m.order(),
                cvdim,
                m.m_cv_stride,
                &m.m_cv[si_m * m.m_cv_stride..],
                &m.m_nurbsknot[si_m..],
                &n.m_nurbsknot[si_n..],
                j,
                &mut cv_n,
            );
            let dst = (si_n + j) * n.m_cv_stride;

            for k in 0..cvdim {
                n.m_cv[dst + k] = cv_n[k];
            }
        }

        si_n = next_span_index(n.order(), n.cv_count(), &n.m_nurbsknot, si_n);
        si_m = next_span_index(m.order(), m.cv_count(), &m.m_nurbsknot, si_m);
    }

    let last_dst = (n.m_cv_count - 1) * n.m_cv_stride;
    let last_src = (m.m_cv_count - 1) * m.m_cv_stride;

    for i in 0..cvdim {
        n.m_cv[i] = m.m_cv[i];
        n.m_cv[last_dst + i] = m.m_cv[last_src + i];
    }

    true
}

// ═══════════════════════════════════════════════════════════════════════════
// SESSION_VIEWER
// ═══════════════════════════════════════════════════════════════════════════
impl NurbsCurve {
    /// Set a CV under the name the viewer's edit lane calls.
    pub fn set_cv_point(&mut self, index: usize, point: &Point) -> bool {
        self.set_cv(index, point)
    }
}
