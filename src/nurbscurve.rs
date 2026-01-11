use crate::point::Point;
use crate::vector::Vector;
use crate::plane::Plane;
use crate::tolerance::Tolerance;
use crate::knot;
use crate::knot::CurveKnotStyle;
use serde::{Serialize, Deserialize};
use uuid::Uuid;

/// Non-Uniform Rational B-Spline (NURBS) curve implementation
///
/// Based on OpenNURBS ground truth implementation.
/// All methods match the fixed C++ and Python versions.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NurbsCurve {
    pub guid: String,           // Unique identifier
    pub name: String,           // Name of the curve
    pub m_dim: usize,           // Dimension (typically 3 for 3D curves)
    pub m_is_rat: bool,         // true if rational, false if non-rational
    pub m_order: usize,         // Order = degree + 1 (order >= 2)
    pub m_cv_count: usize,      // Number of control vertices (>= order)
    pub m_cv_stride: usize,     // Stride between control vertices in m_cv array
    pub m_knot: Vec<f64>,       // Knot vector (length = m_order + m_cv_count - 2)
    pub m_cv: Vec<f64>,         // Control vertex data (homogeneous if rational)
}

impl NurbsCurve {
    /// Create a NURBS curve with specified parameters (matches C++/Python constructor)
    pub fn new(dimension: usize, is_rational: bool, order: usize, cv_count: usize) -> Self {
        let cv_stride = if is_rational { dimension + 1 } else { dimension };
        let knot_count = if order > 0 && cv_count >= order { order + cv_count - 2 } else { 0 };

        NurbsCurve {
            guid: Uuid::new_v4().to_string(),
            name: String::new(),
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
            name: String::new(),
            m_dim: 0,
            m_is_rat: false,
            m_order: 0,
            m_cv_count: 0,
            m_cv_stride: 0,
            m_knot: Vec::new(),
            m_cv: Vec::new(),
        }
    }

    /// Simple string representation
    pub fn str(&self) -> String {
        format!("degree={}, cvs={}", self.degree(), self.cv_count())
    }

    /// Detailed representation
    pub fn repr(&self) -> String {
        format!(
            "NurbsCurve({}, dim={}, order={}, cvs={}, rational={})",
            self.name, self.m_dim, self.m_order, self.m_cv_count, self.m_is_rat
        )
    }

    /// Create a duplicate with new GUID
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = Uuid::new_v4().to_string();
        copy
    }

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

    /// Create interpolated curve through points (matches Rhino TL_CubicNurbThroughPoints)
    /// knot_style: 0=uniform, 1=chord-length (default), 2=centripetal
    pub fn create_interpolated(points: &[Point], degree: usize, closed: bool, knot_style: i32) -> Self {
        let n = points.len();
        if n < 2 {
            return Self::default();
        }

        let mut degree = degree;
        if degree < 1 {
            degree = 3;
        }
        if degree > n - 1 {
            degree = n - 1;
        }

        // Currently only cubic open curves
        if degree != 3 || closed {
            return Self::create(closed, degree, points);
        }

        let order = degree + 1;

        // Step 1: Compute parameters
        let mut params = vec![0.0; n];
        let mut max_dist = 0.0_f64;
        let mut min_dist = f64::MAX;

        for i in 1..n {
            let dx = points[i][0] - points[i - 1][0];
            let dy = points[i][1] - points[i - 1][1];
            let dz = points[i][2] - points[i - 1][2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();

            if dist > max_dist {
                max_dist = dist;
            }
            if dist < min_dist && dist > 0.0 {
                min_dist = dist;
            }

            let delta = match knot_style {
                0 => 1.0,
                2 => dist.sqrt(),
                _ => dist,
            };

            params[i] = params[i - 1] + delta;
        }

        if min_dist <= max_dist * 1.49e-08 {
            return Self::create(false, degree, points);
        }

        // Step 2: Build clamped knot vector
        let mut knots = vec![0.0; n + order];

        for i in 0..order {
            knots[i] = params[0];
            knots[n + order - 1 - i] = params[n - 1];
        }

        // Interior knots from averaging parameters
        for j in 1..=(n - order) {
            let mut sum = 0.0;
            for i in j..(j + degree) {
                sum += params[i];
            }
            knots[j + degree] = sum / degree as f64;
        }

        // Step 3: Build and solve interpolation system
        let mut n_matrix = vec![vec![0.0; n]; n];
        let mut rhs_x = vec![0.0; n];
        let mut rhs_y = vec![0.0; n];
        let mut rhs_z = vec![0.0; n];

        for row in 0..n {
            let t = params[row];
            let span = find_interp_span(t, n, degree, &knots);
            let basis = evaluate_cubic_basis(t, span, &knots);

            for j in 0..=degree {
                let col = span as i32 - degree as i32 + j as i32;
                if col >= 0 && (col as usize) < n {
                    n_matrix[row][col as usize] = basis[j];
                }
            }

            rhs_x[row] = points[row][0];
            rhs_y[row] = points[row][1];
            rhs_z[row] = points[row][2];
        }

        // Extract tridiagonal for Thomas algorithm
        let mut lower = vec![0.0; n];
        let mut diag = vec![0.0; n];
        let mut upper = vec![0.0; n];

        for i in 0..n {
            if i > 0 {
                lower[i] = n_matrix[i][i - 1];
            }
            diag[i] = n_matrix[i][i];
            if i < n - 1 {
                upper[i] = n_matrix[i][i + 1];
            }
        }

        // Solve for each dimension
        let cv_x = match solve_tridiagonal(n, &lower, &diag, &upper, &rhs_x) {
            Some(sol) => sol,
            None => return Self::create(false, degree, points),
        };
        let cv_y = match solve_tridiagonal(n, &lower, &diag, &upper, &rhs_y) {
            Some(sol) => sol,
            None => return Self::create(false, degree, points),
        };
        let cv_z = match solve_tridiagonal(n, &lower, &diag, &upper, &rhs_z) {
            Some(sol) => sol,
            None => return Self::create(false, degree, points),
        };

        // Step 4: Create curve
        let mut result = Self::default();
        result.initialize_curve(3, false, order, n);

        for i in 0..n {
            result.set_cv(i, &Point::new(cv_x[i], cv_y[i], cv_z[i]));
        }

        // Set knots (stored knots = cv_count + order - 2)
        for i in 0..(n + order - 2) {
            result.m_knot[i] = knots[i + 1];
        }

        result
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
            // Would need to convert to rational - not implemented yet
            return false;
        }
        let idx = cv_index * self.m_cv_stride + self.m_dim;
        self.m_cv[idx] = weight;
        true
    }

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

    /// Get curve domain [t_start, t_end]
    pub fn domain(&self) -> (f64, f64) {
        if !self.is_valid() {
            return (0.0, 0.0);
        }
        let t0 = self.m_knot[self.m_order - 2];
        let t1 = self.m_knot[self.m_cv_count - 1];
        (t0, t1)
    }

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
        
        // Reparameterize knots
        for i in 0..self.m_knot.len() {
            self.m_knot[i] = t0 + (self.m_knot[i] - old_t0) * scale;
        }
        
        true
    }

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
                x += n * self.m_cv[idx];
                if self.m_dim > 1 {
                    y += n * self.m_cv[idx + 1];
                }
                if self.m_dim > 2 {
                    z += n * self.m_cv[idx + 2];
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
    /// Returns [point, d1, d2] depending on derivative_count.
    pub fn evaluate(&self, t: f64, derivative_count: usize) -> Vec<Vector> {
        let mut result = Vec::new();
        if !self.is_valid() {
            result.push(Vector::new(0.0, 0.0, 0.0));
            return result;
        }

        let p = self.point_at(t);
        result.push(Vector::new(p[0], p[1], p[2]));

        if derivative_count == 0 {
            return result;
        }

        // Numerical derivatives (consistent with tangent_at semantics)
        let (t0, t1) = self.domain();
        let eps = (t1 - t0) * 1e-8;
        let ta = (t - eps).max(t0);
        let tb = (t + eps).min(t1);

        let pa = self.point_at(ta);
        let pb = self.point_at(tb);

        let d1 = Vector::new(
            (pb[0] - pa[0]) / (tb - ta),
            (pb[1] - pa[1]) / (tb - ta),
            (pb[2] - pa[2]) / (tb - ta),
        );
        result.push(d1.clone());

        if derivative_count > 1 {
            // Central second derivative approximation
            // d2 ~ (C(t+eps) - 2*C(t) + C(t-eps)) / eps^2
            let d2 = Vector::new(
                (pb[0] - 2.0 * p[0] + pa[0]) / (eps * eps),
                (pb[1] - 2.0 * p[1] + pa[1]) / (eps * eps),
                (pb[2] - 2.0 * p[2] + pa[2]) / (eps * eps),
            );
            result.push(d2);
        }

        result
    }

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

    /// Make curve non-rational (remove weights if all equal to 1.0)
    pub fn make_non_rational(&mut self) -> bool {
        if !self.m_is_rat {
            return true; // Already non-rational
        }
        if !self.is_valid() {
            return false;
        }

        // Check if all weights are 1.0
        for i in 0..self.m_cv_count {
            let w = self.weight(i);
            if (w - 1.0).abs() > Tolerance::ZERO_TOLERANCE {
                return false; // Cannot make non-rational
            }
        }

        // Create new CV array without weights
        let new_stride = self.m_dim;
        let mut new_cv = vec![0.0; self.m_cv_count * new_stride];
        
        for i in 0..self.m_cv_count {
            let old_idx = i * self.m_cv_stride;
            let new_idx = i * new_stride;
            
            // Copy coordinates only
            for j in 0..self.m_dim {
                new_cv[new_idx + j] = self.m_cv[old_idx + j];
            }
        }

        self.m_cv = new_cv;
        self.m_cv_stride = new_stride;
        self.m_is_rat = false;
        true
    }

    /// Get approximate length of curve using numerical integration
    pub fn length(&self, tolerance: Option<f64>) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }

        let tol = tolerance.unwrap_or(1e-6);
        let (t0, t1) = self.domain();
        
        // Use adaptive Simpson's rule for length computation
        self.length_adaptive(t0, t1, tol)
    }

    fn length_adaptive(&self, t0: f64, t1: f64, tolerance: f64) -> f64 {
        let t_mid = (t0 + t1) / 2.0;
        
        let p0 = self.point_at(t0);
        let p1 = self.point_at(t_mid);
        let p2 = self.point_at(t1);
        
        let chord_len = p0.distance(&p2, None);
        let arc_len = p0.distance(&p1, None) + p1.distance(&p2, None);
        
        if (arc_len - chord_len).abs() < tolerance || (t1 - t0) < 1e-10 {
            return arc_len;
        }
        
        // Subdivide
        self.length_adaptive(t0, t_mid, tolerance / 2.0) + 
        self.length_adaptive(t_mid, t1, tolerance / 2.0)
    }

    /// Reverse curve direction
    pub fn reverse(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        // Reverse control points
        let mut temp_cv = vec![0.0; self.m_cv_stride];
        for i in 0..(self.m_cv_count / 2) {
            let j = self.m_cv_count - 1 - i;
            
            // Swap CVs
            for k in 0..self.m_cv_stride {
                temp_cv[k] = self.m_cv[i * self.m_cv_stride + k];
                self.m_cv[i * self.m_cv_stride + k] = self.m_cv[j * self.m_cv_stride + k];
                self.m_cv[j * self.m_cv_stride + k] = temp_cv[k];
            }
        }

        // Reverse knot vector using knot module function
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

    /// Divide curve into equal parameter intervals
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

        if !self.is_valid() || count == 0 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let n = if include_endpoints { count - 1 } else { count + 1 };
        let dt = (t1 - t0) / n as f64;

        for i in 0..count {
            let offset = if include_endpoints { 0 } else { 1 };
            let t = t0 + (i + offset) as f64 * dt;
            params.push(t);
            points.push(self.point_at(t));
        }

        (points, params)
    }

    /// Find all intersections between curve and plane
    ///
    /// Implementation matches C++ version with span-based subdivision and endpoint checking.
    pub fn intersect_plane(&self, plane: &Plane, tolerance: Option<f64>) -> Vec<f64> {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        let mut results = Vec::new();

        if !self.is_valid() {
            return results;
        }

        let signed_distance = |p: &Point| -> f64 {
            let v = Vector::new(
                p[0] - plane.origin()[0],
                p[1] - plane.origin()[1],
                p[2] - plane.origin()[2],
            );
            v.dot(&plane.z_axis())
        };

        let (_t_start, t_end) = self.domain();
        let span_params = self.get_span_vector();

        // Check each span for intersections
        for i in 0..(span_params.len() - 1) {
            let t0 = span_params[i];
            let t1 = span_params[i + 1];

            // Skip zero-length spans
            if (t1 - t0).abs() < tol {
                continue;
            }

            // Check for sign change (intersection) in this span
            let d0 = signed_distance(&self.point_at(t0));
            let d1 = signed_distance(&self.point_at(t1));

            // Check if span crosses plane
            if d0 * d1 < 0.0 {
                // Sign change - there's an intersection
                // Use bisection to find it
                let mut ta = t0;
                let mut tb = t1;
                let mut tm = 0.0;
                
                for _ in 0..50 {
                    tm = (ta + tb) * 0.5;
                    let dm = signed_distance(&self.point_at(tm));
                    if dm.abs() < tol {
                        break;
                    }
                    if dm * d0 < 0.0 {
                        tb = tm;
                    } else {
                        ta = tm;
                    }
                }
                results.push(tm);
            } else if d0.abs() < tol {
                // Start point is on plane
                // Avoid duplicates
                if results.is_empty() || (results.last().unwrap() - t0).abs() >= tol {
                    results.push(t0);
                }
            }
        }

        // Check end point explicitly
        let d_end = signed_distance(&self.point_at(t_end));
        if d_end.abs() < tol {
            if results.is_empty() || (results.last().unwrap() - t_end).abs() >= tol {
                results.push(t_end);
            }
        }

        // Sort and remove any remaining duplicates
        results.sort_by(|a, b| a.partial_cmp(b).unwrap());
        results.dedup_by(|a, b| (*a - *b).abs() < tol * 2.0);

        results
    }

    /// Find all intersection points between curve and plane
    pub fn intersect_plane_points(&self, plane: &Plane, tolerance: Option<f64>) -> Vec<Point> {
        self.intersect_plane(plane, tolerance)
            .iter()
            .map(|&t| self.point_at(t))
            .collect()
    }

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

    /// Serialize to protobuf and write to file (stub - protobuf not yet implemented)
    pub fn protobuf_dump(&self, filename: &str) {
        // For now, just use JSON as a fallback
        self.json_dump(&filename.replace(".bin", ".json"));
    }

    /// Load from protobuf file (stub - protobuf not yet implemented)
    pub fn protobuf_load(filename: &str) -> Self {
        // For now, just use JSON as a fallback
        Self::json_load(&filename.replace(".bin", ".json"))
    }
}

impl Default for NurbsCurve {
    fn default() -> Self {
        Self::default()
    }
}

// Helper functions for create_interpolated
fn solve_tridiagonal(n: usize, lower: &[f64], diag: &[f64], upper: &[f64], rhs: &[f64]) -> Option<Vec<f64>> {
    if n < 1 {
        return None;
    }

    let mut c_prime = vec![0.0; n];
    let mut d_prime = vec![0.0; n];

    // Forward sweep
    if diag[0].abs() < 1e-14 {
        return None;
    }
    c_prime[0] = upper[0] / diag[0];
    d_prime[0] = rhs[0] / diag[0];

    for i in 1..n {
        let denom = diag[i] - lower[i] * c_prime[i - 1];
        if denom.abs() < 1e-14 {
            return None;
        }

        if i < n - 1 {
            c_prime[i] = upper[i] / denom;
        }

        d_prime[i] = (rhs[i] - lower[i] * d_prime[i - 1]) / denom;
    }

    // Back substitution
    let mut solution = vec![0.0; n];
    solution[n - 1] = d_prime[n - 1];

    for i in (0..n - 1).rev() {
        solution[i] = d_prime[i] - c_prime[i] * solution[i + 1];
    }

    Some(solution)
}

fn find_interp_span(t: f64, cv_count: usize, degree: usize, knots: &[f64]) -> usize {
    let n = cv_count - 1;
    if t >= knots[n + 1] {
        return n;
    }
    if t <= knots[degree] {
        return degree;
    }

    let mut low = degree;
    let mut high = n + 1;
    let mut mid = (low + high) / 2;

    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

fn evaluate_cubic_basis(t: f64, span: usize, knots: &[f64]) -> [f64; 4] {
    let mut n_basis = [0.0; 4];
    n_basis[0] = 1.0;

    let mut left = [0.0; 4];
    let mut right = [0.0; 4];

    for j in 1..4 {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;

        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            if denom.abs() > 1e-14 {
                let temp = n_basis[r] / denom;
                n_basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            } else {
                n_basis[r] = saved;
                saved = 0.0;
            }
        }
        n_basis[j] = saved;
    }

    n_basis
}
