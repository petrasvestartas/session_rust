use crate::point::Point;
use crate::vector::Vector;
use crate::plane::Plane;
use crate::tolerance::Tolerance;
use crate::xform::Xform;
use crate::color::Color;
use crate::knot;
use serde::{Serialize, Deserialize, Serializer, Deserializer};
use serde::ser::SerializeMap;
use uuid::Uuid;

/// Non-Uniform Rational B-Spline (NURBS) curve implementation
///
/// Based on OpenNURBS ground truth implementation.
/// All methods match the fixed C++ and Python versions.
#[derive(Clone, Debug)]
pub struct NurbsCurve {
    pub guid: String,
    pub name: String,
    pub width: f64,
    pub linecolor: Color,
    pub xform: Xform,
    pub m_dim: usize,
    pub m_is_rat: bool,
    pub m_order: usize,
    pub m_cv_count: usize,
    pub m_cv_stride: usize,
    pub m_knot: Vec<f64>,
    pub m_cv: Vec<f64>,
}

impl Serialize for NurbsCurve {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(12))?;
        let control_points: Vec<Vec<f64>> = (0..self.m_cv_count)
            .map(|i| {
                if let Some(p) = self.get_cv(i) {
                    vec![p[0], p[1], p[2]]
                } else {
                    vec![0.0, 0.0, 0.0]
                }
            })
            .collect();
        // Fields in alphabetical order (per CLAUDE.md)
        map.serialize_entry("control_points", &control_points)?;
        map.serialize_entry("cv_count", &self.m_cv_count)?;
        map.serialize_entry("cv_stride", &self.m_cv_stride)?;
        map.serialize_entry("dimension", &self.m_dim)?;
        map.serialize_entry("guid", &self.guid)?;
        map.serialize_entry("is_rational", &self.m_is_rat)?;
        map.serialize_entry("knots", &self.m_knot)?;
        map.serialize_entry("linecolor", &self.linecolor)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("order", &self.m_order)?;
        map.serialize_entry("width", &self.width)?;
        map.serialize_entry("xform", &self.xform)?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for NurbsCurve {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct NurbsCurveData {
            control_points: Vec<Vec<f64>>,
            cv_count: usize,
            #[serde(default)]
            cv_stride: Option<usize>,
            dimension: usize,
            guid: String,
            is_rational: bool,
            knots: Vec<f64>,
            #[serde(default)]
            linecolor: Option<Color>,
            name: String,
            order: usize,
            #[serde(default)]
            width: Option<f64>,
            #[serde(default)]
            xform: Option<Xform>,
        }
        let data = NurbsCurveData::deserialize(deserializer)?;
        let cv_stride = data.cv_stride.unwrap_or_else(|| {
            if data.is_rational { data.dimension + 1 } else { data.dimension }
        });
        let mut m_cv = Vec::with_capacity(data.cv_count * cv_stride);
        for cp in &data.control_points {
            for j in 0..cv_stride.min(cp.len()) {
                m_cv.push(cp[j]);
            }
            for _ in cp.len()..cv_stride {
                m_cv.push(0.0);
            }
        }
        Ok(NurbsCurve {
            guid: data.guid,
            name: data.name,
            width: data.width.unwrap_or(1.0),
            linecolor: data.linecolor.unwrap_or_else(Color::white),
            xform: data.xform.unwrap_or_else(Xform::identity),
            m_dim: data.dimension,
            m_is_rat: data.is_rational,
            m_order: data.order,
            m_cv_count: data.cv_count,
            m_cv_stride: cv_stride,
            m_knot: data.knots,
            m_cv,
        })
    }
}

impl NurbsCurve {
    // =========================================================================
    // Constructors
    // =========================================================================

    /// Create a NURBS curve with specified parameters (matches C++/Python constructor)
    pub fn new(dimension: usize, is_rational: bool, order: usize, cv_count: usize) -> Self {
        let cv_stride = if is_rational { dimension + 1 } else { dimension };
        let knot_count = if order > 0 && cv_count >= order { order + cv_count - 2 } else { 0 };

        NurbsCurve {
            guid: Uuid::new_v4().to_string(),
            name: "my_nurbscurve".to_string(),
            width: 1.0,
            linecolor: Color::white(),
            xform: Xform::identity(),
            m_dim: dimension,
            m_is_rat: is_rational,
            m_order: order,
            m_cv_count: cv_count,
            m_cv_stride: cv_stride,
            m_knot: vec![0.0; knot_count],
            m_cv: vec![0.0; cv_count * cv_stride],
        }
    }

    /// Create an empty NURBS curve (default constructor)
    pub fn default() -> Self {
        NurbsCurve {
            guid: Uuid::new_v4().to_string(),
            name: "my_nurbscurve".to_string(),
            width: 1.0,
            linecolor: Color::white(),
            xform: Xform::identity(),
            m_dim: 0,
            m_is_rat: false,
            m_order: 0,
            m_cv_count: 0,
            m_cv_stride: 0,
            m_knot: Vec::new(),
            m_cv: Vec::new(),
        }
    }

    // =========================================================================
    // String Representation
    // =========================================================================

    /// Simple string representation
    pub fn str(&self) -> String {
        format!("NurbsCurve(name={}, degree={}, cvs={})", self.name, self.degree(), self.cv_count())
    }

    /// Detailed representation
    pub fn repr(&self) -> String {
        let mut lines = vec![
            "NurbsCurve(".to_string(),
            format!("  name={},", self.name),
            format!("  degree={},", self.degree()),
            format!("  cvs={},", self.m_cv_count),
            format!("  rational={},", self.m_is_rat),
            "  control_points=[".to_string(),
        ];
        for i in 0..self.m_cv_count {
            if let Some(p) = self.get_cv(i) {
                lines.push(format!("    {}, {}, {}", p[0], p[1], p[2]));
            }
        }
        lines.push("  ]".to_string());
        lines.push(")".to_string());
        lines.join("\n")
    }

    /// Create a duplicate with new GUID
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = Uuid::new_v4().to_string();
        copy
    }

    // =========================================================================
    // Static Factory Methods
    // =========================================================================

    /// Create NURBS curve from points (unified API)
    ///
    /// # Arguments
    /// * `periodic` - If true, creates a periodic curve; if false, creates a clamped curve
    /// * `degree` - Degree of the curve (order = degree + 1)
    /// * `points` - Control points for the curve
    pub fn create(periodic: bool, degree: usize, points: &[Point]) -> Self {
        let order = degree + 1;

        if periodic {
            Self::create_periodic_uniform(3, order, points, 1.0)
        } else {
            Self::create_clamped_uniform(3, order, points, 1.0)
        }
    }

    /// Create clamped uniform NURBS curve from control points
    ///
    /// Implementation matches OpenNURBS ON_MakeClampedUniformKnotVector exactly.
    pub fn create_clamped_uniform(
        dimension: usize,
        order: usize,
        points: &[Point],
        knot_delta: f64,
    ) -> Self {
        let point_count = points.len();
        
        if order < 2 || point_count < order {
            return Self::default();
        }

        let mut curve = Self::default();
        if !curve.initialize_curve(dimension, false, order, point_count) {
            return Self::default();
        }

        // Set control points
        for (i, point) in points.iter().enumerate() {
            curve.set_cv(i, point);
        }

        // Create clamped uniform knot vector - matches OpenNURBS exactly
        let knot_count = order + point_count - 2;

        // Fill interior knots with uniform spacing
        // Start from index (order-2) up to (cv_count-1)
        let mut k = 0.0;
        for i in (order - 2)..point_count {
            curve.m_knot[i] = k;
            k += knot_delta;
        }

        // Clamp both ends: sets first (order-2) and last (order-2) knots
        // Left clamp: knot[0..order-3] = knot[order-2]
        let i0 = order - 2;
        for i in 0..i0 {
            curve.m_knot[i] = curve.m_knot[i0];
        }

        // Right clamp: knot[cv_count..knot_count-1] = knot[cv_count-1]
        let i0 = point_count - 1;
        for i in (i0 + 1)..knot_count {
            curve.m_knot[i] = curve.m_knot[i0];
        }

        curve
    }

    /// Create periodic uniform NURBS curve from control points
    pub fn create_periodic_uniform(
        dimension: usize,
        order: usize,
        points: &[Point],
        knot_delta: f64,
    ) -> Self {
        let point_count = points.len();

        if order < 2 || point_count < order {
            return Self::default();
        }

        let mut curve = Self::default();
        let cv_count = point_count + order - 1;

        if !curve.initialize_curve(dimension, false, order, cv_count) {
            return Self::default();
        }

        // Set control points with wrapping
        for (i, point) in points.iter().enumerate() {
            curve.set_cv(i, point);
        }

        // Wrap control points for periodicity
        for i in 0..(order - 1) {
            let idx = i % point_count;
            curve.set_cv(point_count + i, &points[idx]);
        }

        // Create uniform knot vector
        let knot_count = order + cv_count - 2;
        for i in 0..knot_count {
            curve.m_knot[i] = i as f64 * knot_delta;
        }

        curve
    }

    // =========================================================================
    // Initialization & Creation
    // =========================================================================

    /// Initialize curve with specified parameters
    fn initialize_curve(
        &mut self,
        dimension: usize,
        is_rational: bool,
        order: usize,
        cv_count: usize,
    ) -> bool {
        if dimension < 1 || order < 2 || cv_count < order {
            return false;
        }

        self.m_dim = dimension;
        self.m_is_rat = is_rational;
        self.m_order = order;
        self.m_cv_count = cv_count;
        self.m_cv_stride = if is_rational { dimension + 1 } else { dimension };

        let knot_count = order + cv_count - 2;
        self.m_knot = vec![0.0; knot_count];
        self.m_cv = vec![0.0; cv_count * self.m_cv_stride];

        // Initialize weights to 1.0 for rational curves
        if is_rational {
            for i in 0..cv_count {
                self.m_cv[i * self.m_cv_stride + dimension] = 1.0;
            }
        }

        true
    }

    // =========================================================================
    // Control Vertex Access
    // =========================================================================

    /// Set control vertex at index
    fn set_cv(&mut self, index: usize, point: &Point) {
        if index >= self.m_cv_count {
            return;
        }

        let idx = index * self.m_cv_stride;
        self.m_cv[idx] = point[0];
        if self.m_dim > 1 {
            self.m_cv[idx + 1] = point[1];
        }
        if self.m_dim > 2 {
            self.m_cv[idx + 2] = point[2];
        }
    }

    /// Get control vertex at index
    pub fn get_cv(&self, index: usize) -> Option<Point> {
        if index >= self.m_cv_count {
            return None;
        }

        let idx = index * self.m_cv_stride;
        let x = self.m_cv[idx];
        let y = if self.m_dim > 1 { self.m_cv[idx + 1] } else { 0.0 };
        let z = if self.m_dim > 2 { self.m_cv[idx + 2] } else { 0.0 };

        Some(Point::new(x, y, z))
    }

    /// Set control vertex at index (public version)
    pub fn set_cv_point(&mut self, index: usize, point: &Point) -> bool {
        if index >= self.m_cv_count {
            return false;
        }
        self.set_cv(index, point);
        true
    }

    /// Get weight at control vertex index (returns 1.0 if non-rational)
    pub fn weight(&self, cv_index: usize) -> f64 {
        if !self.m_is_rat || cv_index >= self.m_cv_count {
            return 1.0;
        }
        let idx = cv_index * self.m_cv_stride + self.m_dim;
        self.m_cv[idx]
    }

    /// Set weight at control vertex index
    pub fn set_weight(&mut self, cv_index: usize, weight: f64) -> bool {
        if cv_index >= self.m_cv_count {
            return false;
        }
        if !self.m_is_rat {
            return false;
        }
        let idx = cv_index * self.m_cv_stride + self.m_dim;
        self.m_cv[idx] = weight;
        true
    }

    // =========================================================================
    // Knot Access
    // =========================================================================

    /// Get knot value at index
    pub fn knot(&self, knot_index: usize) -> Option<f64> {
        if knot_index >= self.m_knot.len() {
            return None;
        }
        Some(self.m_knot[knot_index])
    }

    /// Set knot value at index
    pub fn set_knot(&mut self, knot_index: usize, knot_value: f64) -> bool {
        if knot_index >= self.m_knot.len() {
            return false;
        }
        self.m_knot[knot_index] = knot_value;
        true
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    /// Get dimension
    pub fn dimension(&self) -> usize {
        self.m_dim
    }

    /// Check if curve is rational
    pub fn is_rational(&self) -> bool {
        self.m_is_rat
    }

    /// Get curve degree
    pub fn degree(&self) -> usize {
        if self.m_order < 2 {
            0
        } else {
            self.m_order - 1
        }
    }

    /// Get curve order
    pub fn order(&self) -> usize {
        self.m_order
    }

    /// Get number of control vertices
    pub fn cv_count(&self) -> usize {
        self.m_cv_count
    }

    /// Get knot count
    pub fn knot_count(&self) -> usize {
        self.m_knot.len()
    }

    /// Get size of each control vertex (dimension + 1 if rational, else dimension)
    pub fn cv_size(&self) -> usize {
        self.m_cv_stride
    }

    /// Get number of spans
    pub fn span_count(&self) -> usize {
        if !self.is_valid() {
            return 0;
        }
        let spans = self.get_span_vector();
        if spans.len() > 1 {
            spans.len() - 1
        } else {
            0
        }
    }

    /// Get all knot values
    pub fn get_knots(&self) -> Vec<f64> {
        self.m_knot.clone()
    }

    /// Get knot array pointer (for compatibility)
    pub fn knot_array(&self) -> &[f64] {
        &self.m_knot
    }

    /// Get CV array pointer (for compatibility)
    pub fn cv_array(&self) -> &[f64] {
        &self.m_cv
    }

    /// Get CV array mutable pointer (for expert use)
    pub fn cv_array_mut(&mut self) -> &mut [f64] {
        &mut self.m_cv
    }

    // =========================================================================
    // Validation
    // =========================================================================

    /// Check if curve is valid
    pub fn is_valid(&self) -> bool {
        if self.m_order < 2 || self.m_cv_count < self.m_order {
            return false;
        }
        if self.m_knot.len() != self.m_order + self.m_cv_count - 2 {
            return false;
        }
        // Check for sufficient distinct knots
        if self.m_order >= 2 && self.m_cv_count >= self.m_order {
            let idx1 = self.m_order - 2;
            let idx2 = self.m_cv_count - 1;
            if idx2 < self.m_knot.len() && self.m_knot[idx1] >= self.m_knot[idx2] {
                return false;
            }
        }
        true
    }

    // =========================================================================
    // Domain & Parameterization
    // =========================================================================

    /// Get curve domain [t_start, t_end]
    pub fn domain(&self) -> (f64, f64) {
        if !self.is_valid() {
            return (0.0, 0.0);
        }
        let t0 = self.m_knot[self.m_order - 2];
        let t1 = self.m_knot[self.m_cv_count - 1];
        (t0, t1)
    }

    /// Get start of domain
    pub fn domain_start(&self) -> f64 {
        self.domain().0
    }

    /// Get end of domain
    pub fn domain_end(&self) -> f64 {
        self.domain().1
    }

    /// Get middle of domain
    pub fn domain_middle(&self) -> f64 {
        let (t0, t1) = self.domain();
        (t0 + t1) * 0.5
    }

    /// Get raw CV data at index (like C++ double* cv(int))
    pub fn cv(&self, cv_index: usize) -> Option<&[f64]> {
        if cv_index >= self.m_cv_count {
            return None;
        }
        let idx = cv_index * self.m_cv_stride;
        Some(&self.m_cv[idx..idx + self.m_cv_stride])
    }

    /// Get control point at index as homogeneous point (x, y, z, w)
    pub fn get_cv_4d(&self, cv_index: usize) -> Option<(f64, f64, f64, f64)> {
        if cv_index >= self.m_cv_count {
            return None;
        }
        let idx = cv_index * self.m_cv_stride;
        let x = self.m_cv[idx];
        let y = if self.m_dim > 1 { self.m_cv[idx + 1] } else { 0.0 };
        let z = if self.m_dim > 2 { self.m_cv[idx + 2] } else { 0.0 };
        let w = if self.m_is_rat { self.m_cv[idx + self.m_dim] } else { 1.0 };
        Some((x, y, z, w))
    }

    /// Set control point at index from homogeneous coordinates
    pub fn set_cv_4d(&mut self, cv_index: usize, x: f64, y: f64, z: f64, w: f64) -> bool {
        if cv_index >= self.m_cv_count {
            return false;
        }

        // Make rational if w != 1.0 (matches C++ implementation)
        if !self.m_is_rat && w != 1.0 {
            self.make_rational();
        }

        let idx = cv_index * self.m_cv_stride;
        self.m_cv[idx] = x;
        if self.m_dim > 1 { self.m_cv[idx + 1] = y; }
        if self.m_dim > 2 { self.m_cv[idx + 2] = z; }
        if self.m_is_rat { self.m_cv[idx + self.m_dim] = w; }
        true
    }

    /// Get knot multiplicity at index
    pub fn knot_multiplicity(&self, knot_index: usize) -> usize {
        if knot_index >= self.m_knot.len() {
            return 0;
        }
        let val = self.m_knot[knot_index];
        let mut count = 1;
        // Count forward
        let mut i = knot_index + 1;
        while i < self.m_knot.len() && (self.m_knot[i] - val).abs() < Tolerance::ZERO_TOLERANCE {
            count += 1;
            i += 1;
        }
        // Count backward
        let mut j = knot_index;
        while j > 0 {
            j -= 1;
            if (self.m_knot[j] - val).abs() < Tolerance::ZERO_TOLERANCE {
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// Get superfluous knot value at end (0=start, 1=end)
    pub fn superfluous_knot(&self, end: usize) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }
        let kc = self.knot_count();
        if end == 0 {
            // First superfluous knot: reflect first knot across knot[order-2]
            return 2.0 * self.m_knot[0] - self.m_knot[self.m_order - 2];
        } else {
            // Last superfluous knot: reflect last knot across knot[cv_count-order]
            return 2.0 * self.m_knot[kc - 1] - self.m_knot[self.m_cv_count - self.m_order];
        }
    }

    /// Create a clamped uniform knot vector for this curve
    pub fn make_clamped_uniform_knot_vector(&mut self, delta: f64) -> bool {
        if delta <= 0.0 || self.m_dim == 0 || self.m_order < 2 || self.m_cv_count < self.m_order {
            return false;
        }
        self.m_knot = knot::make_clamped_uniform(self.m_order, self.m_cv_count, delta);
        !self.m_knot.is_empty()
    }

    /// Check if knot vector is valid
    pub fn is_valid_knot_vector(&self) -> bool {
        if self.m_knot.len() != self.m_order + self.m_cv_count - 2 {
            return false;
        }
        // Check non-decreasing
        for i in 1..self.m_knot.len() {
            if self.m_knot[i] < self.m_knot[i - 1] - Tolerance::ZERO_TOLERANCE {
                return false;
            }
        }
        // Check valid domain exists
        if self.m_order >= 2 && self.m_cv_count >= self.m_order {
            let idx1 = self.m_order - 2;
            let idx2 = self.m_cv_count - 1;
            if idx2 < self.m_knot.len() && self.m_knot[idx1] >= self.m_knot[idx2] {
                return false;
            }
        }
        true
    }

    // =========================================================================
    // Internal Helpers
    // =========================================================================

    /// Find knot span index for parameter t
    ///
    /// Implementation matches OpenNURBS ON_NurbsSpanIndex with offset knot pointer.
    fn find_span(&self, t: f64) -> usize {
        // Use knot module function
        knot::find_span(self.m_order, self.m_cv_count, &self.m_knot, t)
    }

    /// Compute non-zero basis functions at parameter t
    ///
    /// Implementation matches OpenNURBS Cox-de Boor algorithm with offset knot pointer.
    fn basis_functions(&self, span: usize, t: f64) -> Vec<f64> {
        let mut basis = vec![0.0; self.m_order];
        let mut left = vec![0.0; self.m_order];
        let mut right = vec![0.0; self.m_order];

        // Offset knot pointer like OpenNURBS does
        let offset = self.m_order - 2 + span;

        basis[0] = 1.0;

        for j in 1..self.m_order {
            left[j] = t - self.m_knot[offset + 1 - j];
            right[j] = self.m_knot[offset + j] - t;
            let mut saved = 0.0;

            for r in 0..j {
                let temp = basis[r] / (right[r + 1] + left[j - r]);
                basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }

            basis[j] = saved;
        }

        basis
    }

    /// Compute basis functions and their derivatives at parameter t
    /// Returns ders[k][j] = k-th derivative of j-th basis function
    fn basis_functions_derivatives(&self, span: usize, t: f64, deriv_order: usize) -> Vec<Vec<f64>> {
        let p = self.degree();
        let n_der = deriv_order.min(p);

        let mut ders = vec![vec![0.0; p + 1]; n_der + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];

        // Use same knot offset as basis_functions
        let offset = self.m_order - 2 + span;

        ndu[0][0] = 1.0;
        for j in 1..=p {
            left[j] = t - self.m_knot[offset + 1 - j];
            right[j] = self.m_knot[offset + j] - t;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        // Load basis functions
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }

        // Compute derivatives using Eq. 2.10 from The NURBS Book
        let mut a = vec![vec![0.0; p + 1]; 2];
        for r in 0..=p {
            let mut s1 = 0usize;
            let mut s2 = 1usize;
            a[0][0] = 1.0;

            for k in 1..=n_der {
                let mut d = 0.0;
                let rk = r as i32 - k as i32;
                let pk = p as i32 - k as i32;

                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }

                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if (r as i32 - 1) <= pk { k - 1 } else { p - r };

                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[(pk + 1) as usize][(rk + j as i32) as usize];
                    d += a[s2][j] * ndu[(rk + j as i32) as usize][pk as usize];
                }

                if r <= pk as usize {
                    a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                    d += a[s2][k] * ndu[r][pk as usize];
                }

                ders[k][r] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }

        // Apply factorial scaling: p!/(p-k)!
        let mut scale = p as f64;
        for k in 1..=n_der {
            for j in 0..=p {
                ders[k][j] *= scale;
            }
            scale *= (p - k) as f64;
        }

        ders
    }

    /// Set curve domain
    pub fn set_domain(&mut self, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || t0 >= t1 {
            return false;
        }

        let (old_t0, old_t1) = self.domain();
        if (old_t0 - old_t1).abs() < 1e-14 {
            return false;
        }

        let scale = (t1 - t0) / (old_t1 - old_t0);

        for i in 0..self.m_knot.len() {
            self.m_knot[i] = t0 + (self.m_knot[i] - old_t0) * scale;
        }

        true
    }

    // =========================================================================
    // Evaluation
    // =========================================================================

    /// Evaluate point at parameter t
    ///
    /// Implementation matches OpenNURBS evaluation approach.
    pub fn point_at(&self, t: f64) -> Point {
        if !self.is_valid() {
            return Point::new(0.0, 0.0, 0.0);
        }

        // Find span (returns index relative to shifted knot array)
        let span = self.find_span(t);

        // Evaluate using Cox-de Boor algorithm
        let basis = self.basis_functions(span, t);

        // Compute point
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut w = 0.0;

        // In OpenNURBS, span index directly corresponds to CV starting index
        for i in 0..self.m_order {
            let cv_idx = span + i;
            if cv_idx >= self.m_cv_count {
                continue;
            }

            let idx = cv_idx * self.m_cv_stride;
            let n = basis[i];

            if self.m_is_rat {
                let weight = self.m_cv[idx + self.m_dim];
                w += n * weight;
                x += n * self.m_cv[idx] * weight;
                if self.m_dim > 1 {
                    y += n * self.m_cv[idx + 1] * weight;
                }
                if self.m_dim > 2 {
                    z += n * self.m_cv[idx + 2] * weight;
                }
            } else {
                x += n * self.m_cv[idx];
                if self.m_dim > 1 {
                    y += n * self.m_cv[idx + 1];
                }
                if self.m_dim > 2 {
                    z += n * self.m_cv[idx + 2];
                }
                w = 1.0;
            }
        }

        if self.m_is_rat && w.abs() > 1e-10 {
            Point::new(x / w, y / w, z / w)
        } else {
            Point::new(x, y, z)
        }
    }

    /// Get start point of curve
    pub fn point_at_start(&self) -> Point {
        let (t0, _) = self.domain();
        self.point_at(t0)
    }

    /// Get end point of curve
    pub fn point_at_end(&self) -> Point {
        let (_, t1) = self.domain();
        self.point_at(t1)
    }

    /// Get middle point of curve
    pub fn point_at_middle(&self) -> Point {
        self.point_at(self.domain_middle())
    }

    /// Set the start point of the curve (modifies first CV)
    pub fn set_start_point(&mut self, point: &Point) {
        if self.m_cv_count > 0 {
            self.set_cv(0, point);
        }
    }

    /// Set the end point of the curve (modifies last CV)
    pub fn set_end_point(&mut self, point: &Point) {
        if self.m_cv_count > 0 {
            self.set_cv(self.m_cv_count - 1, point);
        }
    }

    // =========================================================================
    // Transformation
    // =========================================================================

    /// Apply stored xform to the curve (in-place)
    pub fn transform(&mut self) {
        self.transform_by(&self.xform.clone());
    }

    /// Apply provided xform to the curve (in-place)
    pub fn transform_by(&mut self, xf: &Xform) -> bool {
        for i in 0..self.m_cv_count {
            if let Some(p) = self.get_cv(i) {
                let x = xf.m[0] * p[0] + xf.m[4] * p[1] + xf.m[8] * p[2] + xf.m[12];
                let y = xf.m[1] * p[0] + xf.m[5] * p[1] + xf.m[9] * p[2] + xf.m[13];
                let z = xf.m[2] * p[0] + xf.m[6] * p[1] + xf.m[10] * p[2] + xf.m[14];
                self.set_cv(i, &Point::new(x, y, z));
            }
        }
        true
    }

    /// Get copy with stored xform applied
    pub fn transformed(&self) -> NurbsCurve {
        let mut result = self.clone();
        result.guid = Uuid::new_v4().to_string();
        result.transform();
        result
    }

    /// Get copy with provided xform applied
    pub fn transformed_by(&self, xf: &Xform) -> NurbsCurve {
        let mut result = self.clone();
        result.guid = Uuid::new_v4().to_string();
        result.transform_by(xf);
        result
    }

    // =========================================================================
    // Modification Operations
    // =========================================================================

    /// Swap two coordinate indices for all CVs
    pub fn swap_coordinates(&mut self, i: usize, j: usize) {
        if i >= self.m_dim || j >= self.m_dim || i == j {
            return;
        }
        for cv_idx in 0..self.m_cv_count {
            let idx = cv_idx * self.m_cv_stride;
            let temp = self.m_cv[idx + i];
            self.m_cv[idx + i] = self.m_cv[idx + j];
            self.m_cv[idx + j] = temp;
        }
    }

    /// Split curve at parameter t into two curves
    pub fn split(&self, t: f64) -> (NurbsCurve, NurbsCurve) {
        if !self.is_valid() {
            return (NurbsCurve::default(), NurbsCurve::default());
        }

        let (t0, t1) = self.domain();
        if t <= t0 || t >= t1 {
            return (NurbsCurve::default(), NurbsCurve::default());
        }

        // Simple approach: use dense point sampling and rebuild curves
        let num_samples = (self.m_cv_count * 4).max(20);

        // Left curve points
        let mut left_points = Vec::new();
        for i in 0..=num_samples {
            let param = t0 + (t - t0) * (i as f64) / (num_samples as f64);
            left_points.push(self.point_at(param));
        }

        // Right curve points
        let mut right_points = Vec::new();
        for i in 0..=num_samples {
            let param = t + (t1 - t) * (i as f64) / (num_samples as f64);
            right_points.push(self.point_at(param));
        }

        let degree = self.degree();
        let left = NurbsCurve::create(false, degree, &left_points);
        let right = NurbsCurve::create(false, degree, &right_points);

        (left, right)
    }

    /// Extend curve domain using de Boor extrapolation (matches C++ implementation)
    pub fn extend(&mut self, new_t0: f64, new_t1: f64) -> bool {
        if !self.is_valid() || self.is_closed() {
            return false;
        }

        let (d0, d1) = self.domain();
        let cvdim = self.cv_size();
        let mut changed = false;

        // Extend start (new_t0 < current domain start)
        if new_t0 < d0 {
            self.clamp_end(0);
            self.evaluate_nurbs_de_boor_inplace(cvdim, self.m_order, 0, 1, new_t0);
            for i in 0..(self.m_order - 1) {
                self.m_knot[i] = new_t0;
            }
            changed = true;
        }

        // Extend end (new_t1 > current domain end)
        if new_t1 > d1 {
            self.clamp_end(1);
            let i0 = self.m_cv_count - self.m_order;
            self.evaluate_nurbs_de_boor_inplace(cvdim, self.m_order, i0, -1, new_t1);
            let kc = self.knot_count();
            for i in (self.m_cv_count - 1)..kc {
                self.m_knot[i] = new_t1;
            }
            changed = true;
        }

        changed
    }

    /// Internal de Boor evaluation for curve extension (modifies CVs in place)
    fn evaluate_nurbs_de_boor_inplace(&mut self, cvdim: usize, order: usize, cv_start: usize, direction: i32, t: f64) {
        if order < 2 {
            return;
        }

        let stride = self.m_cv_stride;
        for i in 1..order {
            let k0 = if direction > 0 { cv_start + i - 1 } else { cv_start + order - i };
            let k1 = if direction > 0 { k0 + 1 } else { k0.saturating_sub(1) };

            let a = self.m_knot[cv_start + if direction > 0 { order - 1 } else { 0 }];
            let b = self.m_knot[cv_start + if direction > 0 { i } else { order - 1 - i }];

            if (b - a).abs() < 1e-14 {
                continue;
            }

            let s = (t - a) / (b - a);

            for j in 0..cvdim {
                let cv0_val = self.m_cv[k0 * stride + j];
                let cv1_val = self.m_cv[k1 * stride + j];
                self.m_cv[k0 * stride + j] = cv0_val + s * (cv0_val - cv1_val);
            }
        }
    }

    /// Clamp curve end (0=start, 1=end, 2=both)
    pub fn clamp_end(&mut self, end: i32) {
        if !self.is_valid() || self.m_order < 2 {
            return;
        }

        // Clamp start
        if end == 0 || end == 2 {
            let knot_val = self.m_knot[self.m_order - 2];
            for i in 0..(self.m_order - 2) {
                self.m_knot[i] = knot_val;
            }
        }

        // Clamp end
        if end == 1 || end == 2 {
            let kc = self.knot_count();
            let knot_val = self.m_knot[self.m_cv_count - 1];
            for i in self.m_cv_count..kc {
                self.m_knot[i] = knot_val;
            }
        }
    }

    /// Check if knot vector is clamped at ends (0=start, 1=end, 2=both)
    pub fn is_clamped(&self, end: i32) -> bool {
        if !self.is_valid() {
            return false;
        }
        knot::is_clamped(self.m_order, self.m_cv_count, &self.m_knot, end)
    }

    /// Get Greville abcissa for a control point (aligned with opennurbs)
    pub fn greville_abcissa(&self, cv_index: usize) -> f64 {
        if cv_index >= self.m_cv_count {
            return 0.0;
        }

        let knot = &self.m_knot[cv_index..];
        let order = self.m_order;

        if order <= 2 || knot[0] == knot[order - 2] {
            return knot[0];
        }

        let p = order - 1;
        let k0 = knot[0];
        let k = knot[p / 2];
        let k1 = knot[p - 1];
        let tol = (k1 - k0) * 1.490116119385e-8_f64;
        let dp = p as f64;

        let mut g: f64 = knot[..p].iter().sum();
        g /= dp;

        // Snap to exact middle knot for uniform knot vectors
        if (2.0 * k - (k0 + k1)).abs() <= tol
            && (g - k).abs() <= (g.abs() * 1.490116119385e-8_f64 + tol)
        {
            g = k;
        }

        g
    }

    /// Get all Greville abcissae
    pub fn get_greville_abcissae(&self) -> Vec<f64> {
        (0..self.m_cv_count).map(|i| self.greville_abcissa(i)).collect()
    }

    /// Insert knot into curve (Boehm's algorithm)
    pub fn insert_knot(&mut self, knot_value: f64, knot_multiplicity: usize) -> bool {
        if !self.is_valid() {
            return false;
        }

        let p = self.degree();
        if knot_multiplicity < 1 || knot_multiplicity > p {
            return false;
        }

        let (d0, d1) = self.domain();
        if knot_value < d0 || knot_value > d1 {
            return false;
        }

        // Handle end knots
        if knot_value == d0 {
            if knot_multiplicity == p { self.clamp_end(0); return true; }
            if knot_multiplicity == 1 { return true; }
            return false;
        }
        if knot_value == d1 {
            if knot_multiplicity == p { self.clamp_end(1); return true; }
            if knot_multiplicity == 1 { return true; }
            return false;
        }

        let mut n = self.m_cv_count - 1;
        let mut full_knot_count = self.m_cv_count + self.m_order;

        for _insert_iter in 0..knot_multiplicity {
            // Build full knot vector
            let mut u = vec![0.0f64; full_knot_count];
            u[0] = self.m_knot[0];
            for i in 0..self.m_knot.len() {
                u[i + 1] = self.m_knot[i];
            }
            u[full_knot_count - 1] = *self.m_knot.last().unwrap();

            // Count current multiplicity
            let tol = (d0.abs() + d1.abs() + (d1 - d0).abs()) * f64::EPSILON.sqrt();
            let mult = u.iter().filter(|&&v| (v - knot_value).abs() <= tol).count();
            if mult >= p {
                return false;
            }

            // Find span
            let span = self.find_span(knot_value);
            let k = span + self.m_order - 1;

            let m_full = full_knot_count - 1;
            let new_full_knot_count = full_knot_count + 1;
            let new_cv_count = self.m_cv_count + 1;

            let mut u_new = vec![0.0f64; new_full_knot_count];
            let mut cv_new = vec![0.0f64; new_cv_count * self.m_cv_stride];

            // Copy unaffected knots
            for i in 0..=k {
                u_new[i] = u[i];
            }
            u_new[k + 1] = knot_value;
            for i in (k + 1)..=m_full {
                u_new[i + 1] = u[i];
            }

            // Copy unaffected CVs before
            for i in 0..=(k - p) {
                let src = i * self.m_cv_stride;
                let dst = i * self.m_cv_stride;
                cv_new[dst..dst + self.m_cv_stride].copy_from_slice(&self.m_cv[src..src + self.m_cv_stride]);
            }

            // Copy unaffected CVs after
            for i in (k + 1)..=(n + 1) {
                let src = (i - 1) * self.m_cv_stride;
                let dst = i * self.m_cv_stride;
                cv_new[dst..dst + self.m_cv_stride].copy_from_slice(&self.m_cv[src..src + self.m_cv_stride]);
            }

            // Compute new CVs in affected region
            for i in (k - p + 1)..=(k) {
                let denom = u[i + p] - u[i];
                let alpha = if denom != 0.0 { (knot_value - u[i]) / denom } else { 0.0 };

                let src_prev = (i - 1) * self.m_cv_stride;
                let src_curr = i * self.m_cv_stride;
                let dst = i * self.m_cv_stride;

                for d in 0..self.m_cv_stride {
                    cv_new[dst + d] = (1.0 - alpha) * self.m_cv[src_prev + d] + alpha * self.m_cv[src_curr + d];
                }
            }

            // Update internal state
            self.m_cv_count = new_cv_count;
            self.m_cv = cv_new;

            let new_compressed = self.m_order + self.m_cv_count - 2;
            self.m_knot = (0..new_compressed).map(|i| u_new[i + 1]).collect();

            full_knot_count = new_full_knot_count;
            n = self.m_cv_count - 1;
        }

        true
    }

    /// Get tangent vector at parameter t
    pub fn tangent_at(&self, t: f64) -> Vector {
        if !self.is_valid() {
            return Vector::new(0.0, 0.0, 0.0);
        }

        // Use numerical differentiation for simplicity
        let (t0, t1) = self.domain();
        let eps = (t1 - t0) * 1e-8;
        
        let p1 = self.point_at((t - eps).max(t0));
        let p2 = self.point_at((t + eps).min(t1));
        
        let tangent = Vector::new(
            (p2[0] - p1[0]) / (2.0 * eps),
            (p2[1] - p1[1]) / (2.0 * eps),
            (p2[2] - p1[2]) / (2.0 * eps),
        );
        
        // Normalize
        tangent.normalized()
    }

    /// Evaluate point and derivatives on curve at parameter t.
    /// Returns [point, d1, d2, ...] depending on derivative_count.
    /// Uses analytical basis function derivatives matching C++ implementation.
    pub fn evaluate(&self, t: f64, derivative_count: usize) -> Vec<Vector> {
        let mut result = Vec::new();

        if !self.is_valid() {
            result.push(Vector::new(0.0, 0.0, 0.0));
            return result;
        }

        // Clamp derivative order to degree
        let max_derivs = derivative_count.min(self.degree());

        let span = self.find_span(t);
        let ders = self.basis_functions_derivatives(span, t, max_derivs);

        // Evaluate homogeneous coordinates and derivatives
        let p = self.degree();
        let mut aders: Vec<[f64; 4]> = vec![[0.0; 4]; max_derivs + 1];

        for k in 0..=max_derivs {
            for j in 0..=p {
                let cv_idx = span + j;
                if cv_idx >= self.m_cv_count {
                    continue;
                }
                let idx = cv_idx * self.m_cv_stride;

                let nx = ders[k][j];
                let cx = self.m_cv[idx];
                let cy = if self.m_dim > 1 { self.m_cv[idx + 1] } else { 0.0 };
                let cz = if self.m_dim > 2 { self.m_cv[idx + 2] } else { 0.0 };
                let wv = if self.m_is_rat { self.m_cv[idx + self.m_dim] } else { 1.0 };

                aders[k][0] += nx * cx * wv;
                aders[k][1] += nx * cy * wv;
                aders[k][2] += nx * cz * wv;
                aders[k][3] += nx * wv;
            }
        }

        // Convert from homogeneous derivatives (Aders) to Cartesian derivatives
        let mut cders: Vec<[f64; 3]> = vec![[0.0; 3]; max_derivs + 1];

        if !self.m_is_rat {
            // Non-rational: derivatives are directly Aders (w == 1)
            for k in 0..=max_derivs {
                cders[k] = [aders[k][0], aders[k][1], aders[k][2]];
            }
        } else {
            // Rational: use standard formula (Piegl & Tiller, Eq. 2.28)
            for k in 0..=max_derivs {
                let w = aders[0][3];
                let inv_w = if w != 0.0 { 1.0 / w } else { 0.0 };

                let mut ck_x = aders[k][0];
                let mut ck_y = aders[k][1];
                let mut ck_z = aders[k][2];

                // Subtract contributions of weight derivatives
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

        // Fill result vectors (0th derivative = point)
        for k in 0..=max_derivs {
            result.push(Vector::new(cders[k][0], cders[k][1], cders[k][2]));
        }

        // If caller requested more derivatives than degree, pad with zeros
        for _ in (max_derivs + 1)..=derivative_count {
            result.push(Vector::new(0.0, 0.0, 0.0));
        }

        result
    }

    /// Binomial coefficient C(n, k)
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

    /// Get Frenet frame at parameter t (tangent, normal, binormal)
    pub fn frame_at(&self, t: f64, normalized: bool) -> Option<(Point, Vector, Vector, Vector)> {
        if !self.is_valid() {
            return None;
        }

        let (t0, t1) = self.domain();
        let param = if normalized {
            if t < 0.0 || t > 1.0 { return None; }
            t0 + t * (t1 - t0)
        } else {
            if t < t0 || t > t1 { return None; }
            t
        };

        let origin = self.point_at(param);
        let derivs = self.evaluate(param, 2);
        if derivs.len() < 3 {
            return None;
        }

        let d1 = &derivs[1];
        let d2 = &derivs[2];

        let d1_mag = d1.magnitude();
        if d1_mag < 1e-14 {
            return None;
        }

        let tangent = d1.normalized();

        let d2_dot_t = d2.dot(&tangent);
        let mut normal = Vector::new(
            d2[0] - d2_dot_t * tangent[0],
            d2[1] - d2_dot_t * tangent[1],
            d2[2] - d2_dot_t * tangent[2],
        );

        let n_mag = normal.magnitude();
        if n_mag < 1e-14 {
            let world_z = Vector::new(0.0, 0.0, 1.0);
            normal = tangent.cross(&world_z);
            if normal.magnitude() < 1e-14 {
                let world_y = Vector::new(0.0, 1.0, 0.0);
                normal = tangent.cross(&world_y);
            }
        }
        normal = normal.normalized();

        let binormal = tangent.cross(&normal).normalized();

        Some((origin, tangent, normal, binormal))
    }

    /// Internal: compute RMF with Frenet initialization (matches Rhino)
    fn perpendicular_frame_at_internal(&self, t: f64, normalized: bool) -> Option<(Point, Vector, Vector, Vector)> {
        if !self.is_valid() {
            return None;
        }

        let (t0, t1) = self.domain();
        let param = if normalized {
            if t < 0.0 || t > 1.0 { return None; }
            t0 + t * (t1 - t0)
        } else {
            if t < t0 || t > t1 { return None; }
            t
        };

        // Get initial frame at t0 using Frenet (curvature-based)
        let derivs0 = self.evaluate(t0, 2);
        let d1_0 = &derivs0[1];
        let d2_0 = &derivs0[2];

        let d1_0_mag = d1_0.magnitude();
        if d1_0_mag < 1e-14 {
            return None;
        }

        let tangent0 = d1_0.normalized();

        // Initial normal from curvature (Frenet)
        let d2_dot_d1 = d2_0.dot(d1_0);
        let d1_0_mag_sq = d1_0_mag * d1_0_mag;
        let mut n0_unnorm = Vector::new(
            d2_0[0] - (d2_dot_d1 / d1_0_mag_sq) * d1_0[0],
            d2_0[1] - (d2_dot_d1 / d1_0_mag_sq) * d1_0[1],
            d2_0[2] - (d2_dot_d1 / d1_0_mag_sq) * d1_0[2],
        );

        let mut n0_mag = n0_unnorm.magnitude();
        if n0_mag < 1e-14 {
            let world_z = Vector::new(0.0, 0.0, 1.0);
            n0_unnorm = world_z.cross(&tangent0);
            n0_mag = n0_unnorm.magnitude();
            if n0_mag < 1e-14 {
                let world_y = Vector::new(0.0, 1.0, 0.0);
                n0_unnorm = world_y.cross(&tangent0);
                n0_mag = n0_unnorm.magnitude();
            }
        }
        let r0 = Vector::new(n0_unnorm[0] / n0_mag, n0_unnorm[1] / n0_mag, n0_unnorm[2] / n0_mag);

        let origin = self.point_at(param);

        // If at start, return Frenet frame directly
        if (param - t0).abs() < 1e-14 {
            let s0 = tangent0.cross(&r0).normalized();
            return Some((origin, r0, s0, tangent0));
        }

        // Propagate frame using Double Reflection (RMF) algorithm
        let num_steps = 10.max(((param - t0) / (t1 - t0) * 100.0) as i32);
        let dt = (param - t0) / num_steps as f64;

        let mut ri = r0;
        let mut ti_param = t0;
        let mut xi = self.point_at(ti_param);
        let mut tangent_i = tangent0;

        for _ in 0..num_steps {
            if ti_param >= param - 1e-14 {
                break;
            }
            let ti_next = (ti_param + dt).min(param);
            let xi_next = self.point_at(ti_next);
            let mut tangent_next = self.tangent_at(ti_next);
            tangent_next = tangent_next.normalized();

            let v1 = Vector::new(xi_next[0] - xi[0], xi_next[1] - xi[1], xi_next[2] - xi[2]);
            let c1 = v1.dot(&v1);
            if c1 < 1e-28 {
                ti_param = ti_next;
                xi = xi_next;
                tangent_i = tangent_next;
                continue;
            }

            let ri_dot_v1 = ri.dot(&v1);
            let r_l = Vector::new(
                ri[0] - 2.0 * ri_dot_v1 / c1 * v1[0],
                ri[1] - 2.0 * ri_dot_v1 / c1 * v1[1],
                ri[2] - 2.0 * ri_dot_v1 / c1 * v1[2],
            );

            let ti_dot_v1 = tangent_i.dot(&v1);
            let t_l = Vector::new(
                tangent_i[0] - 2.0 * ti_dot_v1 / c1 * v1[0],
                tangent_i[1] - 2.0 * ti_dot_v1 / c1 * v1[1],
                tangent_i[2] - 2.0 * ti_dot_v1 / c1 * v1[2],
            );

            let v2 = Vector::new(tangent_next[0] - t_l[0], tangent_next[1] - t_l[1], tangent_next[2] - t_l[2]);
            let c2 = v2.dot(&v2);
            ri = if c2 < 1e-28 {
                r_l
            } else {
                let rl_dot_v2 = r_l.dot(&v2);
                Vector::new(
                    r_l[0] - 2.0 * rl_dot_v2 / c2 * v2[0],
                    r_l[1] - 2.0 * rl_dot_v2 / c2 * v2[1],
                    r_l[2] - 2.0 * rl_dot_v2 / c2 * v2[2],
                )
            };

            ri = ri.normalized();
            ti_param = ti_next;
            xi = xi_next;
            tangent_i = tangent_next;
        }

        let mut tangent = self.tangent_at(param);
        tangent = tangent.normalized();

        let ri_dot_t = ri.dot(&tangent);
        ri = Vector::new(
            ri[0] - ri_dot_t * tangent[0],
            ri[1] - ri_dot_t * tangent[1],
            ri[2] - ri_dot_t * tangent[2],
        ).normalized();

        let s = tangent.cross(&ri).normalized();

        Some((origin, ri, s, tangent))
    }

    /// Get rotation minimizing perpendicular frame at parameter t
    /// Uses the exact Double Reflection algorithm for accuracy
    pub fn perpendicular_frame_at(&self, t: f64, normalized: bool) -> Option<(Point, Vector, Vector, Vector)> {
        self.perpendicular_frame_at_internal(t, normalized)
    }

    /// Get multiple perpendicular frames along the curve
    pub fn get_perpendicular_frames(&self, params: &[f64]) -> Vec<(Point, Vector, Vector, Vector)> {
        params.iter()
            .filter_map(|&t| self.perpendicular_frame_at(t, true))
            .collect()
    }

    // =========================================================================
    // Geometric Queries
    // =========================================================================

    /// Check if curve is closed (start point == end point)
    pub fn is_closed(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let start = self.point_at_start();
        let end = self.point_at_end();
        
        start.distance(&end, None) < Tolerance::ZERO_TOLERANCE
    }

    /// Check if curve is periodic (wraps around seamlessly)
    pub fn is_periodic(&self) -> bool {
        // For now, return false - full implementation would check
        // if the curve is clamped and if removing end knots makes it periodic
        false
    }

    /// Check if curve is a straight line within tolerance
    pub fn is_linear(&self, tolerance: Option<f64>) -> bool {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        
        if !self.is_valid() || self.m_cv_count < 2 {
            return false;
        }

        if self.m_cv_count == 2 {
            return true;
        }

        // Check if all control points are collinear
        let p0 = self.get_cv(0).unwrap();
        let p1 = self.get_cv(self.m_cv_count - 1).unwrap();
        
        let line_vec = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let line_len = line_vec.magnitude();
        
        if line_len < tol {
            return true; // Degenerate to a point
        }

        for i in 1..(self.m_cv_count - 1) {
            let p = self.get_cv(i).unwrap();
            let v = Vector::new(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
            
            // Cross product to check collinearity
            let cross = line_vec.cross(&v);
            
            if cross.magnitude() > tol * line_len {
                return false;
            }
        }

        true
    }

    /// Check if curve is planar (all CVs lie in a single plane)
    pub fn is_planar(&self, tolerance: Option<f64>) -> bool {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        if !self.is_valid() || self.m_cv_count < 3 {
            return true;
        }
        let p0 = self.get_cv(0).unwrap();
        let p1 = self.get_cv(1).unwrap();
        let mut normal = None;
        for i in 2..self.m_cv_count {
            let p = self.get_cv(i).unwrap();
            let v1 = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
            let v2 = Vector::new(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
            let cross = v1.cross(&v2);
            if cross.magnitude() > tol {
                if normal.is_none() {
                    normal = Some(cross.normalized());
                } else {
                    let n = normal.as_ref().unwrap();
                    if (1.0 - n.dot(&cross.normalized()).abs()).abs() > tol {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Check if curve is an arc (matches C++ is_arc stub)
    pub fn is_arc(&self, _tolerance: Option<f64>) -> bool {
        false
    }

    /// Check if curve lies entirely in a given plane
    pub fn is_in_plane(&self, plane: &Plane, tolerance: Option<f64>) -> bool {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        if !self.is_valid() {
            return false;
        }
        for i in 0..self.m_cv_count {
            let p = self.get_cv(i).unwrap();
            let v = Vector::new(
                p[0] - plane.origin()[0],
                p[1] - plane.origin()[1],
                p[2] - plane.origin()[2],
            );
            if v.dot(&plane.z_axis()).abs() > tol {
                return false;
            }
        }
        true
    }

    /// Check if curve has "natural" end conditions (zero 2nd derivative at endpoints)
    pub fn is_natural(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let tol_factor = 1e-8;
        let (t0, t1) = self.domain();

        // Check both endpoints
        for pass in 0..2 {
            let t = if pass == 0 { t0 } else { t1 };

            // Evaluate 2nd derivative
            let derivs = self.evaluate(t, 2);
            if derivs.len() < 3 {
                return false;
            }

            let d2 = &derivs[2];
            let d2_len = d2.magnitude();

            // Get control polygon length for tolerance
            let (cv0, cv2) = if pass == 0 {
                (self.get_cv(0), self.get_cv(2.min(self.m_cv_count - 1)))
            } else {
                (self.get_cv(self.m_cv_count - 1), self.get_cv(0.max(self.m_cv_count as i32 - 3) as usize))
            };

            if cv0.is_none() || cv2.is_none() {
                return false;
            }

            let tol = cv0.unwrap().distance(&cv2.unwrap(), None) * tol_factor;

            if d2_len > tol {
                return false;
            }
        }

        true
    }

    /// Check if curve is a polyline (all CVs connected by straight segments)
    pub fn is_polyline(&self, _tolerance: Option<f64>) -> bool {
        if !self.is_valid() {
            return false;
        }
        self.degree() == 1
    }

    // =========================================================================
    // Conversion Methods
    // =========================================================================

    /// Convert curve to polyline using adaptive subdivision
    pub fn to_polyline_adaptive(
        &self,
        angle_tolerance: f64,
        min_edge_length: f64,
        max_edge_length: f64,
    ) -> (Vec<Point>, Vec<f64>) {
        let mut points = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() {
            return (points, params);
        }

        let angle_tol = if angle_tolerance <= 0.0 { 0.1 } else { angle_tolerance };

        let (t0, t1) = self.domain();
        let curve_len = self.length(Some(1e-6));

        let max_len = if max_edge_length <= 0.0 { curve_len / 10.0 } else { max_edge_length };
        let mut min_len = if min_edge_length <= 0.0 { curve_len / 1000.0 } else { min_edge_length };
        if min_len > max_len {
            min_len = max_len * 0.1;
        }

        // Collect (param, point) pairs, then sort by param
        let mut samples: Vec<(f64, Point)> = Vec::new();
        samples.push((t0, self.point_at(t0)));
        samples.push((t1, self.point_at(t1)));

        // Work queue: segments to potentially subdivide (ta, tb)
        let mut work_queue: Vec<(f64, f64)> = Vec::new();
        work_queue.push((t0, t1));

        const MAX_ITERATIONS: i32 = 10000;
        let mut iterations = 0;

        while !work_queue.is_empty() && iterations < MAX_ITERATIONS {
            iterations += 1;
            let (ta, tb) = work_queue.pop().unwrap();

            let pa = self.point_at(ta);
            let pb = self.point_at(tb);
            let chord_length = pa.distance(&pb, None);

            if chord_length < min_len {
                continue;
            }

            let tm = (ta + tb) * 0.5;
            let pm = self.point_at(tm);

            // Check deviation: distance from midpoint to chord
            let chord = Vector::new(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
            let to_mid = Vector::new(pm[0] - pa[0], pm[1] - pa[1], pm[2] - pa[2]);
            let chord_len_sq = chord.dot(&chord);
            let mut deviation = 0.0;

            if chord_len_sq > 1e-20 {
                let proj = to_mid.dot(&chord) / chord_len_sq;
                let projected = Point::new(
                    pa[0] + proj * chord[0],
                    pa[1] + proj * chord[1],
                    pa[2] + proj * chord[2],
                );
                deviation = pm.distance(&projected, None);
            }

            // Convert angle tolerance to approximate deviation tolerance
            // For small angles: deviation ≈ chord_length * sin(angle/2) ≈ chord_length * angle/2
            let deviation_tolerance = chord_length * angle_tol * 0.5;

            let need_subdivide = (deviation > deviation_tolerance) || (chord_length > max_len);

            if need_subdivide {
                samples.push((tm, pm));
                work_queue.push((ta, tm));
                work_queue.push((tm, tb));
            }
        }

        // Sort by parameter
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // Extract results
        for (t, p) in samples {
            points.push(p);
            params.push(t);
        }

        (points, params)
    }

    /// Divide curve by arc length using Gauss-Legendre quadrature.
    /// Matches C++ implementation exactly.
    pub fn divide_by_length(&self, segment_length: f64) -> (Vec<Point>, Vec<f64>) {
        let mut points = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() || segment_length <= 0.0 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let dom_len = t1 - t0;
        let h = dom_len * 1e-8;

        // 5-point Gauss-Legendre nodes and weights for [-1, 1]
        const GL_NODES: [f64; 5] = [-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640];
        const GL_WEIGHTS: [f64; 5] = [0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891];

        // Compute derivative (un-normalized) at parameter t
        let derivative_at = |curve: &NurbsCurve, t: f64| -> Vector {
            let (p1, p2, dt);
            if t <= t0 + h {
                p1 = curve.point_at(t0);
                p2 = curve.point_at(t0 + h);
                dt = h;
            } else if t >= t1 - h {
                p1 = curve.point_at(t1 - h);
                p2 = curve.point_at(t1);
                dt = h;
            } else {
                p1 = curve.point_at(t - h);
                p2 = curve.point_at(t + h);
                dt = 2.0 * h;
            }
            Vector::new((p2[0] - p1[0]) / dt, (p2[1] - p1[1]) / dt, (p2[2] - p1[2]) / dt)
        };

        // Arc length via Gauss-Legendre quadrature
        let arc_length_gauss = |curve: &NurbsCurve, ta: f64, tb: f64| -> f64 {
            let mid = (ta + tb) * 0.5;
            let half = (tb - ta) * 0.5;
            let mut sum = 0.0;
            for i in 0..5 {
                let t = mid + half * GL_NODES[i];
                sum += GL_WEIGHTS[i] * derivative_at(curve, t).magnitude();
            }
            half * sum
        };

        // Build arc-length table with high resolution
        let approx_len = self.length(Some(1e-6));
        let n_samples = (1000usize).max((approx_len / segment_length) as usize * 100);
        let dt = (t1 - t0) / n_samples as f64;

        let mut t_vals = vec![0.0; n_samples + 1];
        let mut s_vals = vec![0.0; n_samples + 1];

        t_vals[0] = t0;
        s_vals[0] = 0.0;

        for i in 1..=n_samples {
            t_vals[i] = t0 + i as f64 * dt;
            s_vals[i] = s_vals[i - 1] + arc_length_gauss(self, t_vals[i - 1], t_vals[i]);
        }

        let total_len = s_vals[n_samples];

        // Find parameter at target arc length with Newton-Raphson refinement
        let find_t_at_s = |curve: &NurbsCurve, s_target: f64| -> f64 {
            if s_target <= 0.0 { return t0; }
            if s_target >= total_len { return t1; }

            // Binary search for bracket
            let mut lo = 0usize;
            let mut hi = n_samples;
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if s_vals[mid] < s_target { lo = mid; }
                else { hi = mid; }
            }

            // Initial guess: linear interpolation
            let frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
            let mut t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);

            // Newton-Raphson refinement
            let mut t_lo = t_vals[lo];
            let mut t_hi = t_vals[hi];
            for _ in 0..20 {
                let s_cur = s_vals[lo] + arc_length_gauss(curve, t_vals[lo], t);
                let error = s_cur - s_target;

                if error.abs() < 1e-12 { break; }

                let speed = derivative_at(curve, t).magnitude();
                if speed < 1e-14 {
                    if error > 0.0 { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                    else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
                    continue;
                }

                let t_new = t - error / speed;
                if t_new <= t_lo || t_new >= t_hi {
                    if error > 0.0 { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                    else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
                } else {
                    t = t_new;
                }
            }
            t
        };

        // Add points at each segment_length interval
        let mut s = 0.0;
        while s <= total_len + 1e-10 {
            let t = find_t_at_s(self, s);
            points.push(self.point_at(t));
            params.push(t);
            s += segment_length;
        }

        (points, params)
    }

    /// Make curve rational (all weights = 1.0)
    pub fn make_rational(&mut self) -> bool {
        if self.m_is_rat {
            return true; // Already rational
        }
        if !self.is_valid() {
            return false;
        }

        // Create new CV array with weights
        let new_stride = self.m_dim + 1;
        let mut new_cv = vec![0.0; self.m_cv_count * new_stride];
        
        for i in 0..self.m_cv_count {
            let old_idx = i * self.m_cv_stride;
            let new_idx = i * new_stride;
            
            // Copy coordinates
            for j in 0..self.m_dim {
                new_cv[new_idx + j] = self.m_cv[old_idx + j];
            }
            // Set weight to 1.0
            new_cv[new_idx + self.m_dim] = 1.0;
        }

        self.m_cv = new_cv;
        self.m_cv_stride = new_stride;
        self.m_is_rat = true;
        true
    }

    /// Make curve non-rational. If force=false (default), fails when weights differ.
    /// If force=true, sets all weights to 1.0 (changes geometry!).
    pub fn make_non_rational(&mut self) -> bool {
        self.make_non_rational_force(false)
    }

    pub fn make_non_rational_force(&mut self, force: bool) -> bool {
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
            let old_idx = i * self.m_cv_stride;
            let new_idx = i * new_stride;
            for j in 0..self.m_dim {
                new_cv[new_idx + j] = self.m_cv[old_idx + j];
            }
        }

        self.m_cv = new_cv;
        self.m_cv_stride = new_stride;
        self.m_is_rat = false;
        true
    }

    /// Get curve length using Gauss-Legendre quadrature
    pub fn length(&self, _tolerance: Option<f64>) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }

        const GL_X: [f64; 10] = [
            -0.9739065285171717, -0.8650633666889845, -0.6794095682990244,
            -0.4333953941292472, -0.1488743389816312,
             0.1488743389816312,  0.4333953941292472,  0.6794095682990244,
             0.8650633666889845,  0.9739065285171717
        ];
        const GL_W: [f64; 10] = [
            0.0666713443086881, 0.1494513491505806, 0.2190863625159820,
            0.2692667193099963, 0.2955242247147529,
            0.2955242247147529, 0.2692667193099963, 0.2190863625159820,
            0.1494513491505806, 0.0666713443086881
        ];

        let mut total = 0.0;
        let n_spans = self.span_count();
        const SUBDIVISIONS: usize = 4;

        for span in 0..n_spans {
            let span_a = self.m_knot[self.m_order - 2 + span];
            let span_b = self.m_knot[self.m_order - 1 + span];
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
                    let t = mid + half * GL_X[i];
                    let derivs = self.evaluate(t, 1);
                    s += GL_W[i] * derivs[1].magnitude();
                }
                total += half * s;
            }
        }
        total
    }

    /// Reverse curve direction
    pub fn reverse(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let mut temp_cv = vec![0.0; self.m_cv_stride];
        for i in 0..(self.m_cv_count / 2) {
            let j = self.m_cv_count - 1 - i;

            for k in 0..self.m_cv_stride {
                temp_cv[k] = self.m_cv[i * self.m_cv_stride + k];
                self.m_cv[i * self.m_cv_stride + k] = self.m_cv[j * self.m_cv_stride + k];
                self.m_cv[j * self.m_cv_stride + k] = temp_cv[k];
            }
        }

        knot::reverse(self.m_order, self.m_cv_count, &mut self.m_knot)
    }

    /// Get span vector (parameter values at span boundaries)
    pub fn get_span_vector(&self) -> Vec<f64> {
        let mut spans = Vec::new();
        if !self.is_valid() {
            return spans;
        }

        let offset = self.m_order - 2;
        spans.push(self.m_knot[offset]);

        for i in (offset + 1)..self.m_cv_count {
            if i == offset || (self.m_knot[i] - self.m_knot[i - 1]).abs() > Tolerance::ZERO_TOLERANCE {
                spans.push(self.m_knot[i]);
            }
        }

        spans
    }

    /// Divide curve into equal arc-length segments using Gauss-Legendre quadrature.
    /// Matches C++ implementation exactly.
    ///
    /// # Arguments
    /// * `count` - Number of points to generate
    /// * `include_endpoints` - If true, includes start and end points
    ///
    /// # Returns
    /// Tuple of (points, parameters)
    pub fn divide_by_count(&self, count: usize, include_endpoints: bool) -> (Vec<Point>, Vec<f64>) {
        let mut points = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() || count < 2 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let dom_len = t1 - t0;
        let h = dom_len * 1e-8;

        // 5-point Gauss-Legendre nodes and weights for [-1, 1]
        const GL_NODES: [f64; 5] = [-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640];
        const GL_WEIGHTS: [f64; 5] = [0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891];

        // Compute derivative (un-normalized) at parameter t
        let derivative_at = |curve: &NurbsCurve, t: f64| -> Vector {
            let (p1, p2, dt);
            if t <= t0 + h {
                p1 = curve.point_at(t0);
                p2 = curve.point_at(t0 + h);
                dt = h;
            } else if t >= t1 - h {
                p1 = curve.point_at(t1 - h);
                p2 = curve.point_at(t1);
                dt = h;
            } else {
                p1 = curve.point_at(t - h);
                p2 = curve.point_at(t + h);
                dt = 2.0 * h;
            }
            Vector::new((p2[0] - p1[0]) / dt, (p2[1] - p1[1]) / dt, (p2[2] - p1[2]) / dt)
        };

        // Arc length via Gauss-Legendre quadrature
        let arc_length_gauss = |curve: &NurbsCurve, ta: f64, tb: f64| -> f64 {
            let mid = (ta + tb) * 0.5;
            let half = (tb - ta) * 0.5;
            let mut sum = 0.0;
            for i in 0..5 {
                let t = mid + half * GL_NODES[i];
                sum += GL_WEIGHTS[i] * derivative_at(curve, t).magnitude();
            }
            half * sum
        };

        // Build arc-length table with high resolution
        let n_samples = (1000usize).max(count * 100);
        let dt = (t1 - t0) / n_samples as f64;

        let mut t_vals = vec![0.0; n_samples + 1];
        let mut s_vals = vec![0.0; n_samples + 1];

        t_vals[0] = t0;
        s_vals[0] = 0.0;

        for i in 1..=n_samples {
            t_vals[i] = t0 + i as f64 * dt;
            s_vals[i] = s_vals[i - 1] + arc_length_gauss(self, t_vals[i - 1], t_vals[i]);
        }

        let total_len = s_vals[n_samples];
        let n_segs = if include_endpoints { count - 1 } else { count + 1 };
        let seg_len = total_len / n_segs as f64;

        // Find parameter at target arc length with Newton-Raphson refinement
        let find_t_at_s = |curve: &NurbsCurve, s_target: f64| -> f64 {
            if s_target <= 0.0 { return t0; }
            if s_target >= total_len { return t1; }

            // Binary search for bracket
            let mut lo = 0usize;
            let mut hi = n_samples;
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if s_vals[mid] < s_target { lo = mid; }
                else { hi = mid; }
            }

            // Initial guess: linear interpolation
            let frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
            let mut t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);

            // Newton-Raphson refinement
            let mut t_lo = t_vals[lo];
            let mut t_hi = t_vals[hi];
            for _ in 0..20 {
                let s_cur = s_vals[lo] + arc_length_gauss(curve, t_vals[lo], t);
                let error = s_cur - s_target;

                if error.abs() < 1e-12 { break; }

                let speed = derivative_at(curve, t).magnitude();
                if speed < 1e-14 {
                    if error > 0.0 { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                    else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
                    continue;
                }

                let t_new = t - error / speed;
                if t_new <= t_lo || t_new >= t_hi {
                    if error > 0.0 { t_hi = t; t = (t_lo + t_hi) * 0.5; }
                    else { t_lo = t; t = (t_lo + t_hi) * 0.5; }
                } else {
                    t = t_new;
                }
            }
            t
        };

        points.reserve(count);
        params.reserve(count);

        for i in 0..count {
            let s_target = if include_endpoints {
                seg_len * i as f64
            } else {
                seg_len * (i + 1) as f64
            };

            let t = find_t_at_s(self, s_target);
            points.push(self.point_at(t));
            params.push(t);
        }

        (points, params)
    }

    // =========================================================================
    // JSON Serialization
    // =========================================================================

    /// Serialize to JSON and write to file
    pub fn json_dump(&self, filename: &str) {
        use std::fs::File;
        use std::io::Write;
        if let Ok(json) = serde_json::to_string_pretty(self) {
            if let Ok(mut file) = File::create(filename) {
                let _ = file.write_all(json.as_bytes());
            }
        }
    }

    /// Load from JSON file
    pub fn json_load(filename: &str) -> Self {
        use std::fs::File;
        use std::io::Read;
        let mut file = match File::open(filename) {
            Ok(f) => f,
            Err(_) => return Self::default(),
        };
        let mut contents = String::new();
        if file.read_to_string(&mut contents).is_err() {
            return Self::default();
        }
        serde_json::from_str(&contents).unwrap_or_else(|_| Self::default())
    }

    /// Convert to protobuf binary format
    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::NurbsCurve {
            guid: self.guid.clone(),
            name: self.name.clone(),
            dimension: self.m_dim as i32,
            is_rational: self.m_is_rat,
            order: self.m_order as i32,
            cv_count: self.m_cv_count as i32,
            cv_stride: self.m_cv_stride as i32,
            knots: self.m_knot.clone(),
            cvs: self.m_cv.clone(),
            width: 1.0,
            linecolor: Some(crate::proto::Color {
                guid: String::new(),
                name: "white".to_string(),
                r: 255,
                g: 255,
                b: 255,
                a: 255,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    /// Create NurbsCurve from protobuf binary data
    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::NurbsCurve::decode(data)?;
        let mut curve = Self::new(
            proto.dimension as usize,
            proto.is_rational,
            proto.order as usize,
            proto.cv_count as usize,
        );
        curve.guid = proto.guid;
        curve.name = proto.name;
        curve.m_knot = proto.knots;
        curve.m_cv = proto.cvs;
        if let Some(xform) = proto.xform {
            curve.xform.guid = xform.guid;
            curve.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    curve.xform.m[i] = *val;
                }
            }
        }
        Ok(curve)
    }

    /// Serialize to protobuf and write to file
    pub fn protobuf_dump(&self, filename: &str) {
        let data = self.to_protobuf();
        std::fs::write(filename, data).expect("Failed to write protobuf file");
    }

    /// Load from protobuf file
    pub fn protobuf_load(filename: &str) -> Self {
        let data = std::fs::read(filename).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
    }
}

impl Default for NurbsCurve {
    fn default() -> Self {
        Self::default()
    }
}
