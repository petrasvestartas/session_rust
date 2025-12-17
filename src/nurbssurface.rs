use crate::point::Point;
use crate::nurbscurve::NurbsCurve;
use crate::xform::Xform;
use crate::color::Color;
use crate::vector::Vector;

/// Non-Uniform Rational B-Spline (NURBS) surface implementation
/// 
/// Based on OpenNURBS ground truth implementation.
/// Matches the C++ and Python implementations exactly.
#[derive(Clone, Debug)]
pub struct NurbsSurface {
    // Metadata
    pub guid: String,
    pub name: String,
    pub width: f64,
    pub surfacecolor: Color,
    pub xform: Xform,
    
    // Core NURBS data
    pub m_dim: usize,                // Dimension (typically 3 for 3D surfaces)
    pub m_is_rat: bool,              // true if rational, false if non-rational
    pub m_order: [usize; 2],         // Order = degree + 1 (order >= 2) for u and v
    pub m_cv_count: [usize; 2],      // Number of control vertices in u and v directions
    pub m_cv_stride: [usize; 2],     // Stride between control vertices in m_cv array
    pub m_knot: [Vec<f64>; 2],       // Knot vectors for u and v directions
    pub m_cv: Vec<f64>,              // Control vertex data (homogeneous if rational)
    pub m_knot_capacity: [usize; 2], // Capacity of knot arrays
    pub m_cv_capacity: usize,        // Capacity of CV array
}

impl NurbsSurface {
    /// Create a new empty NURBS surface
    pub fn new() -> Self {
        NurbsSurface {
            guid: uuid::Uuid::new_v4().to_string(),
            name: "my_nurbssurface".to_string(),
            width: 1.0,
            surfacecolor: Color::white(),
            xform: Xform::identity(),
            m_dim: 0,
            m_is_rat: false,
            m_order: [0, 0],
            m_cv_count: [0, 0],
            m_cv_stride: [0, 0],
            m_knot: [Vec::new(), Vec::new()],
            m_cv: Vec::new(),
            m_knot_capacity: [0, 0],
            m_cv_capacity: 0,
        }
    }

    /// Create NURBS surface with specified parameters
    pub fn create(
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
    ) -> Option<Self> {
        if dimension < 1 || order0 < 2 || order1 < 2 
           || cv_count0 < order0 || cv_count1 < order1 {
            return None;
        }

        let mut srf = Self::new();
        srf.m_dim = dimension;
        srf.m_is_rat = is_rational;
        srf.m_order[0] = order0;
        srf.m_order[1] = order1;
        srf.m_cv_count[0] = cv_count0;
        srf.m_cv_count[1] = cv_count1;

        // Calculate CV size and strides
        let cv_size = if is_rational { dimension + 1 } else { dimension };
        srf.m_cv_stride[0] = cv_size;
        srf.m_cv_stride[1] = cv_size * cv_count0;

        // Allocate knot vectors
        let knot_count0 = order0 + cv_count0 - 2;
        let knot_count1 = order1 + cv_count1 - 2;
        srf.m_knot[0] = vec![0.0; knot_count0];
        srf.m_knot[1] = vec![0.0; knot_count1];
        srf.m_knot_capacity[0] = knot_count0;
        srf.m_knot_capacity[1] = knot_count1;

        // Allocate CV array
        let cv_capacity = cv_size * cv_count0 * cv_count1;
        srf.m_cv = vec![0.0; cv_capacity];
        srf.m_cv_capacity = cv_capacity;

        Some(srf)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // ACCESSORS
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get dimension
    pub fn dimension(&self) -> usize {
        self.m_dim
    }
    
    /// Check if surface is rational
    pub fn is_rational(&self) -> bool {
        self.m_is_rat
    }
    
    /// Get order (degree + 1) in specified direction
    pub fn order(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        self.m_order[dir]
    }
    
    /// Get degree (order - 1) in specified direction
    pub fn degree(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        if self.m_order[dir] > 0 { self.m_order[dir] - 1 } else { 0 }
    }
    
    /// Get number of control vertices in specified direction (or total if no direction)
    pub fn cv_count_dir(&self, dir: Option<usize>) -> usize {
        match dir {
            None => self.m_cv_count[0] * self.m_cv_count[1],
            Some(d) if d < 2 => self.m_cv_count[d],
            _ => 0,
        }
    }

    /// Get size of each control vertex (dimension + 1 if rational, else dimension)
    pub fn cv_size(&self) -> usize {
        if self.m_is_rat { self.m_dim + 1 } else { self.m_dim }
    }

    /// Get knot count in specified direction
    pub fn knot_count(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        self.m_knot[dir].len()
    }
    
    /// Get number of spans in specified direction
    pub fn span_count(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        if self.m_cv_count[dir] < self.m_order[dir] { return 0; }
        self.m_cv_count[dir] - self.m_order[dir] + 1
    }
    
    /// Get CV capacity
    pub fn cv_capacity(&self) -> usize {
        self.m_cv_capacity
    }
    
    /// Get knot capacity in specified direction
    pub fn knot_capacity(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        self.m_knot_capacity[dir]
    }

    /// Get knot value at index in specified direction
    pub fn knot(&self, dir: usize, index: usize) -> Option<f64> {
        if dir >= 2 || index >= self.m_knot[dir].len() {
            return None;
        }
        Some(self.m_knot[dir][index])
    }

    /// Set knot value at index in specified direction
    pub fn set_knot(&mut self, dir: usize, index: usize, value: f64) -> bool {
        if dir >= 2 || index >= self.m_knot[dir].len() {
            return false;
        }
        self.m_knot[dir][index] = value;
        true
    }

    /// Get pointer to CV data at indices (i, j)
    pub fn cv(&self, i: usize, j: usize) -> Option<&[f64]> {
        if i >= self.m_cv_count[0] || j >= self.m_cv_count[1] {
            return None;
        }
        let index = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
        let cv_size = self.cv_size();
        if index + cv_size > self.m_cv.len() {
            return None;
        }
        Some(&self.m_cv[index..index + cv_size])
    }

    /// Get mutable pointer to CV data at indices (i, j)
    pub fn cv_mut(&mut self, i: usize, j: usize) -> Option<&mut [f64]> {
        if i >= self.m_cv_count[0] || j >= self.m_cv_count[1] {
            return None;
        }
        let index = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
        let cv_size = self.cv_size();
        if index + cv_size > self.m_cv.len() {
            return None;
        }
        Some(&mut self.m_cv[index..index + cv_size])
    }

    /// Get control vertex as Point
    pub fn get_cv(&self, i: usize, j: usize) -> Option<Point> {
        let cv = self.cv(i, j)?;
        if self.m_is_rat && cv.len() > self.m_dim {
            let w = cv[self.m_dim];
            if w.abs() > 1e-14 {
                Some(Point::new(cv[0] / w, cv[1] / w, cv[2] / w))
            } else {
                Some(Point::new(0.0, 0.0, 0.0))
            }
        } else {
            Some(Point::new(
                if cv.len() > 0 { cv[0] } else { 0.0 },
                if cv.len() > 1 { cv[1] } else { 0.0 },
                if cv.len() > 2 { cv[2] } else { 0.0 },
            ))
        }
    }

    /// Set control vertex from Point
    pub fn set_cv(&mut self, i: usize, j: usize, point: &Point) -> bool {
        let is_rat = self.m_is_rat;
        let dim = self.m_dim;
        
        if let Some(cv) = self.cv_mut(i, j) {
            cv[0] = point[0];
            if cv.len() > 1 { cv[1] = point[1]; }
            if cv.len() > 2 { cv[2] = point[2]; }
            if is_rat && cv.len() > dim {
                cv[dim] = 1.0; // Set weight to 1.0
            }
            true
        } else {
            false
        }
    }
    
    /// Get weight at control vertex index
    pub fn weight(&self, i: usize, j: usize) -> f64 {
        if !self.m_is_rat {
            return 1.0;
        }
        if let Some(cv) = self.cv(i, j) {
            if cv.len() > self.m_dim {
                return cv[self.m_dim];
            }
        }
        1.0
    }
    
    /// Set weight at control vertex index
    pub fn set_weight(&mut self, i: usize, j: usize, w: f64) -> bool {
        if !self.m_is_rat {
            return false;
        }
        let dim = self.m_dim;
        if let Some(cv) = self.cv_mut(i, j) {
            if cv.len() > dim {
                cv[dim] = w;
                return true;
            }
        }
        false
    }

    /// Make knot vector a clamped uniform knot vector
    /// Matches OpenNURBS algorithm exactly
    pub fn make_clamped_uniform_knot_vector(&mut self, dir: usize, delta: f64) -> bool {
        if dir >= 2 {
            return false;
        }
        if self.m_order[dir] < 2 || self.m_cv_count[dir] < self.m_order[dir] {
            return false;
        }

        let order = self.m_order[dir];
        let cv_count = self.m_cv_count[dir];
        let knot_count = self.knot_count(dir);

        // Fill knots from order-2 to cv_count-1
        let mut k = 0.0;
        for i in (order - 2)..cv_count {
            self.m_knot[dir][i] = k;
            k += delta;
        }

        // Clamp start: knot[0..order-3] = knot[order-2]
        for i in 0..(order - 2) {
            self.m_knot[dir][i] = self.m_knot[dir][order - 2];
        }

        // Clamp end: knot[cv_count..knot_count-1] = knot[cv_count-1]
        for i in cv_count..knot_count {
            self.m_knot[dir][i] = self.m_knot[dir][cv_count - 1];
        }

        true
    }

    /// Get parameter domain in specified direction
    pub fn domain(&self, dir: usize) -> Option<(f64, f64)> {
        if dir >= 2 {
            return None;
        }
        let order = self.m_order[dir];
        let cv_count = self.m_cv_count[dir];
        if order < 2 || cv_count < order || self.m_knot[dir].len() < order + cv_count - 2 {
            return None;
        }
        Some((self.m_knot[dir][order - 2], self.m_knot[dir][cv_count - 1]))
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // GEOMETRIC QUERIES
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Check if surface is closed in specified direction
    pub fn is_closed(&self, dir: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        // Check if first and last rows/columns match
        let tol = 1e-10;
        let cv_size = self.cv_size();
        
        for i in 0..if dir == 0 { self.m_cv_count[1] } else { self.m_cv_count[0] } {
            let (cv1, cv2) = if dir == 0 {
                (self.cv(0, i), self.cv(self.m_cv_count[0] - 1, i))
            } else {
                (self.cv(i, 0), self.cv(i, self.m_cv_count[1] - 1))
            };
            
            if let (Some(c1), Some(c2)) = (cv1, cv2) {
                for k in 0..cv_size {
                    if (c1[k] - c2[k]).abs() > tol {
                        return false;
                    }
                }
            }
        }
        true
    }
    
    /// Check if surface is periodic in specified direction
    pub fn is_periodic(&self, dir: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        // Check knot vector periodicity
        let order = self.m_order[dir];
        if self.m_knot[dir].len() < order * 2 {
            return false;
        }
        
        let delta = self.m_knot[dir][order] - self.m_knot[dir][0];
        let tol = 1e-10;
        
        for i in 0..order {
            let expected = self.m_knot[dir][i] + delta;
            let actual = self.m_knot[dir][i + order];
            if (expected - actual).abs() > tol {
                return false;
            }
        }
        
        // Must also be closed
        self.is_closed(dir)
    }
    
    /// Check if surface is clamped in specified direction (at both ends by default)
    /// end: 0=start only, 1=end only, 2=both
    pub fn is_clamped(&self, dir: usize, end: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        let order = self.m_order[dir];
        
        if end == 0 || end == 2 {
            // Check start: first 'order' knots should be equal
            let start_val = self.m_knot[dir][0];
            for i in 1..order {
                if (self.m_knot[dir][i] - start_val).abs() > 1e-10 {
                    return false;
                }
            }
        }
        
        if end == 1 || end == 2 {
            // Check end: last 'order' knots should be equal
            let knot_count = self.knot_count(dir);
            if knot_count < order {
                return false;
            }
            let end_val = self.m_knot[dir][knot_count - 1];
            for i in 1..order {
                if (self.m_knot[dir][knot_count - 1 - i] - end_val).abs() > 1e-10 {
                    return false;
                }
            }
        }
        
        true
    }

    /// Evaluate point and first derivatives at (u, v)
    /// Returns [point, du, dv] if num_derivs > 0, else [point]
    pub fn evaluate(&self, u: f64, v: f64, num_derivs: usize) -> Vec<Vector> {
        let mut result = Vec::new();

        if !self.is_valid() {
            result.push(Vector::new(0.0, 0.0, 0.0));
            return result;
        }

        let pt_opt = self.point_at(u, v);
        if pt_opt.is_none() {
            result.push(Vector::new(0.0, 0.0, 0.0));
            return result;
        }
        let pt = pt_opt.unwrap();
        result.push(Vector::new(pt[0], pt[1], pt[2]));

        if num_derivs > 0 {
            // Finite difference step (match Python implementation semantics)
            let h = 1e-6;
            let (_, u1) = match self.domain(0) { Some(d) => d, None => (u - h, u + h) };
            let (_, v1) = match self.domain(1) { Some(d) => d, None => (v - h, v + h) };

            // du derivative (forward if possible, else backward)
            let du_vec = if u + h <= u1 {
                if let Some(pt_u) = self.point_at(u + h, v) {
                    Vector::new(
                        (pt_u[0] - pt[0]) / h,
                        (pt_u[1] - pt[1]) / h,
                        (pt_u[2] - pt[2]) / h,
                    )
                } else {
                    Vector::new(0.0, 0.0, 0.0)
                }
            } else {
                if let Some(pt_um) = self.point_at(u - h, v) {
                    Vector::new(
                        (pt[0] - pt_um[0]) / h,
                        (pt[1] - pt_um[1]) / h,
                        (pt[2] - pt_um[2]) / h,
                    )
                } else {
                    Vector::new(0.0, 0.0, 0.0)
                }
            };
            result.push(du_vec);

            // dv derivative (forward if possible, else backward)
            let dv_vec = if v + h <= v1 {
                if let Some(pt_v) = self.point_at(u, v + h) {
                    Vector::new(
                        (pt_v[0] - pt[0]) / h,
                        (pt_v[1] - pt[1]) / h,
                        (pt_v[2] - pt[2]) / h,
                    )
                } else {
                    Vector::new(0.0, 0.0, 0.0)
                }
            } else {
                if let Some(pt_vm) = self.point_at(u, v - h) {
                    Vector::new(
                        (pt[0] - pt_vm[0]) / h,
                        (pt[1] - pt_vm[1]) / h,
                        (pt[2] - pt_vm[2]) / h,
                    )
                } else {
                    Vector::new(0.0, 0.0, 0.0)
                }
            };
            result.push(dv_vec);
        }

        result
    }

    /// Get normal vector at parameter (u, v)
    pub fn normal_at(&self, u: f64, v: f64) -> Vector {
        let derivs = self.evaluate(u, v, 1);
        if derivs.len() < 3 {
            return Vector::new(0.0, 0.0, 1.0);
        }
        let du = &derivs[1];
        let dv = &derivs[2];
        let n = du.cross(dv);
        if n.magnitude() < 1e-14 {
            Vector::new(0.0, 0.0, 1.0)
        } else {
            n.normalized()
        }
    }

    /// Find span index for parameter value (OpenNURBS algorithm)
    /// 
    /// Matches ON_NurbsSpanIndex from opennurbs_knot.cpp exactly
    fn find_span(&self, dir: usize, t: f64) -> isize {
        if dir >= 2 {
            return -1;
        }

        let order = self.m_order[dir];
        let cv_count = self.m_cv_count[dir];
        
        if order < 2 || cv_count < order {
            return -1;
        }

        let knot_count = order + cv_count - 2;
        if self.m_knot[dir].len() < knot_count {
            return -1;
        }

        // OpenNURBS: Shift knot pointer by (order-2) for the search
        let search_start = order - 2;
        let search_len = cv_count - order + 2;
        
        if search_len < 1 {
            return -1;
        }

        // Binary search in the shifted range
        let t0 = self.m_knot[dir][search_start];
        let t1 = self.m_knot[dir][search_start + search_len - 1];

        if t < t0 {
            return 0;
        }
        if t >= t1 {
            return (cv_count - order) as isize;
        }

        // Binary search
        let mut i = search_start;
        let mut j = search_start + search_len - 1;

        while i < j {
            let k = (i + j) / 2;
            let knot_k = self.m_knot[dir][k];

            if t < knot_k {
                j = k;
            } else if t >= self.m_knot[dir][k + 1] {
                i = k + 1;
            } else {
                // t is in interval [knot[k], knot[k+1])
                return (k - search_start) as isize;
            }
        }

        (i - search_start) as isize
    }

    /// Compute basis functions (OpenNURBS ON_EvaluateNurbsBasis algorithm)
    fn basis_functions(&self, dir: usize, span_index: usize, t: f64) -> Vec<f64> {
        let order = self.m_order[dir];
        
        if order < 2 {
            return vec![0.0; order];
        }

        let degree = order - 1;  // d = order - 1
        
        // OpenNURBS shifts knot by (order-2) + span, then by d inside basis
        let knot_base = span_index + degree;
        let knot = &self.m_knot[dir];
        
        // Check for degenerate span
        if knot[knot_base - 1] == knot[knot_base] {
            return vec![0.0; order];
        }
        
        let mut big_n = vec![0.0; order * order];
        big_n[order * order - 1] = 1.0;
        
        let mut left = vec![0.0; degree];
        let mut right = vec![0.0; degree];
        
        // Cox-de Boor recursion - matches OpenNURBS lines 702-718
        let mut n_idx = order * order - 1;
        let mut k_right = knot_base;
        let mut k_left = knot_base - 1;
        
        for j in 0..degree {
            let n0_idx = n_idx;
            n_idx -= order + 1;
            left[j] = t - knot[k_left];
            right[j] = knot[k_right] - t;
            k_left = k_left.wrapping_sub(1);
            k_right += 1;
            
            let mut x = 0.0;
            for r in 0..=j {
                let a0 = left[j - r];
                let a1 = right[r];
                let y = big_n[n0_idx + r] / (a0 + a1);
                big_n[n_idx + r] = x + a1 * y;
                x = a0 * y;
            }
            big_n[n_idx + j + 1] = x;
        }
        
        // Return just the final row of basis functions
        big_n[0..order].to_vec()
    }

    /// Evaluate point on surface at parameters (u, v)
    /// Matches OpenNURBS EvPoint algorithm
    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> {
        // Find span indices
        let u_span = self.find_span(0, u);
        let v_span = self.find_span(1, v);

        if u_span < 0 || v_span < 0 {
            return None;
        }

        let u_span = u_span as usize;
        let v_span = v_span as usize;

        // Compute basis functions
        let nu = self.basis_functions(0, u_span, u);
        let nv = self.basis_functions(1, v_span, v);

        // Evaluate point using tensor product
        let cv_size = self.cv_size();
        let mut temp = vec![0.0; cv_size];

        let order_u = self.m_order[0];
        let order_v = self.m_order[1];

        for k in 0..order_u {
            for l in 0..order_v {
                let i = u_span + k;
                let j = v_span + l;
                
                if let Some(cv_ptr) = self.cv(i, j) {
                    let weight = nu[k] * nv[l];
                    for m in 0..cv_size {
                        temp[m] += weight * cv_ptr[m];
                    }
                }
            }
        }

        // Convert from homogeneous coordinates if rational
        if self.m_is_rat && temp.len() > self.m_dim {
            let w = temp[self.m_dim];
            if w.abs() > 1e-14 {
                Some(Point::new(temp[0] / w, temp[1] / w, temp[2] / w))
            } else {
                Some(Point::new(0.0, 0.0, 0.0))
            }
        } else {
            Some(Point::new(
                if temp.len() > 0 { temp[0] } else { 0.0 },
                if temp.len() > 1 { temp[1] } else { 0.0 },
                if temp.len() > 2 { temp[2] } else { 0.0 },
            ))
        }
    }

    /// Check if surface is valid
    pub fn is_valid(&self) -> bool {
        if self.m_dim < 1 || self.m_order[0] < 2 || self.m_order[1] < 2 {
            return false;
        }
        if self.m_cv_count[0] < self.m_order[0] || self.m_cv_count[1] < self.m_order[1] {
            return false;
        }
        let cv_size = self.cv_size();
        let required_cv_size = cv_size * self.m_cv_count[0] * self.m_cv_count[1];
        if self.m_cv.len() < required_cv_size {
            return false;
        }
        for dir in 0..2 {
            let knot_count = self.m_order[dir] + self.m_cv_count[dir] - 2;
            if self.m_knot[dir].len() < knot_count {
                return false;
            }
        }
        true
    }

    /// Extract isoparametric curve from surface
    /// 
    /// # Arguments
    /// * `dir` - Direction (0=iso-u curve where v varies, 1=iso-v curve where u varies)
    /// * `c` - Parameter value at which to extract the curve
    /// 
    /// # Returns
    /// Option containing the NurbsCurve, or None if invalid
    pub fn iso_curve(&self, dir: usize, c: f64) -> Option<NurbsCurve> {
        if dir >= 2 || !self.is_valid() {
            return None;
        }

        // Create output curve
        let mut nurbs_crv = NurbsCurve::default();
        nurbs_crv.m_dim = self.m_dim;
        nurbs_crv.m_is_rat = self.m_is_rat;
        nurbs_crv.m_order = self.m_order[dir];
        nurbs_crv.m_cv_count = self.m_cv_count[dir];
        
        let cv_size = if self.m_is_rat { self.m_dim + 1 } else { self.m_dim };
        nurbs_crv.m_cv_stride = cv_size;
        
        // Allocate knot vector
        let knot_count = nurbs_crv.m_order + nurbs_crv.m_cv_count - 2;
        nurbs_crv.m_knot = vec![0.0; knot_count];
        
        // Copy knot vector for varying direction
        for i in 0..knot_count {
            nurbs_crv.m_knot[i] = self.m_knot[dir][i];
        }
        
        // Allocate CV array
        nurbs_crv.m_cv = vec![0.0; cv_size * nurbs_crv.m_cv_count];
        
        // Find span in constant direction
        let mut span_index = self.find_span(1 - dir, c);
        if span_index < 0 {
            span_index = 0;
        } else if span_index as usize > self.m_cv_count[1 - dir] - self.m_order[1 - dir] {
            span_index = (self.m_cv_count[1 - dir] - self.m_order[1 - dir]) as isize;
        }
        let span_index = span_index as usize;
        
        // Compute basis functions in constant direction
        let basis = self.basis_functions(1 - dir, span_index, c);
        
        // Evaluate CVs for isocurve
        for i in 0..nurbs_crv.m_cv_count {
            let mut cv_sum = vec![0.0; cv_size];
            
            for k in 0..self.m_order[1 - dir] {
                let (row, col) = if dir == 0 {
                    // iso-u: v varies, u is constant at c
                    (span_index + k, i)
                } else {
                    // iso-v: u varies, v is constant at c
                    (i, span_index + k)
                };
                
                if let Some(cv_ptr) = self.cv(row, col) {
                    for m in 0..cv_size {
                        cv_sum[m] += basis[k] * cv_ptr[m];
                    }
                }
            }
            
            // Set CV in curve
            let cv_index = i * nurbs_crv.m_cv_stride;
            for m in 0..cv_size {
                nurbs_crv.m_cv[cv_index + m] = cv_sum[m];
            }
        }
        
        Some(nurbs_crv)
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // MODIFICATION OPERATIONS
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Make surface rational (if not already)
    pub fn make_rational(&mut self) -> bool {
        if self.m_is_rat {
            return true; // Already rational
        }
        
        if !self.is_valid() {
            return false;
        }
        
        let old_cv_size = self.m_dim;
        let new_cv_size = self.m_dim + 1;
        let cv_count_total = self.m_cv_count[0] * self.m_cv_count[1];
        
        // Create new CV array with weights
        let mut new_cv = vec![0.0; new_cv_size * cv_count_total];
        
        // Copy existing CVs and add weight=1.0
        for i in 0..cv_count_total {
            for d in 0..self.m_dim {
                new_cv[i * new_cv_size + d] = self.m_cv[i * old_cv_size + d];
            }
            new_cv[i * new_cv_size + self.m_dim] = 1.0; // Set weight
        }
        
        self.m_cv = new_cv;
        self.m_is_rat = true;
        self.m_cv_stride[0] = new_cv_size;
        self.m_cv_stride[1] = new_cv_size * self.m_cv_count[0];
        self.m_cv_capacity = new_cv_size * cv_count_total;
        
        true
    }
    
    /// Make surface non-rational if all weights are equal
    pub fn make_non_rational(&mut self) -> bool {
        if !self.m_is_rat {
            return true; // Already non-rational
        }
        
        if !self.is_valid() {
            return false;
        }
        
        // Check if all weights are equal (within tolerance)
        let tol = 1e-10;
        let first_weight = if let Some(cv) = self.cv(0, 0) {
            cv[self.m_dim]
        } else {
            return false;
        };
        
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(cv) = self.cv(i, j) {
                    if (cv[self.m_dim] - first_weight).abs() > tol {
                        return false; // Weights not equal
                    }
                }
            }
        }
        
        let old_cv_size = self.m_dim + 1;
        let new_cv_size = self.m_dim;
        let cv_count_total = self.m_cv_count[0] * self.m_cv_count[1];
        
        // Create new CV array without weights
        let mut new_cv = vec![0.0; new_cv_size * cv_count_total];
        
        // Copy existing CVs (without weight)
        for i in 0..cv_count_total {
            for d in 0..self.m_dim {
                new_cv[i * new_cv_size + d] = self.m_cv[i * old_cv_size + d];
            }
        }
        
        self.m_cv = new_cv;
        self.m_is_rat = false;
        self.m_cv_stride[0] = new_cv_size;
        self.m_cv_stride[1] = new_cv_size * self.m_cv_count[0];
        self.m_cv_capacity = new_cv_size * cv_count_total;
        
        true
    }
    
    /// Reverse surface direction
    pub fn reverse(&mut self, dir: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        let cv_size = self.cv_size();
        
        // Reverse control points in specified direction
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let (i1, j1) = if dir == 0 {
                    (self.m_cv_count[0] - 1 - i, j)
                } else {
                    (i, self.m_cv_count[1] - 1 - j)
                };
                
                if dir == 0 && i >= (self.m_cv_count[0] + 1) / 2 {
                    break;
                }
                if dir == 1 && j >= (self.m_cv_count[1] + 1) / 2 {
                    break;
                }
                
                // Swap CVs
                let idx1 = i * self.m_cv_stride[1] + j * self.m_cv_stride[0];
                let idx2 = i1 * self.m_cv_stride[1] + j1 * self.m_cv_stride[0];
                
                for k in 0..cv_size {
                    self.m_cv.swap(idx1 + k, idx2 + k);
                }
            }
        }
        
        // Reverse knot vector
        let knot_count = self.m_knot[dir].len();
        let domain_start = self.m_knot[dir][self.m_order[dir] - 2];
        let domain_end = self.m_knot[dir][self.m_cv_count[dir] - 1];
        let domain_length = domain_end - domain_start;
        
        for i in 0..knot_count {
            self.m_knot[dir][i] = domain_start + domain_length - (self.m_knot[dir][knot_count - 1 - i] - domain_start);
        }
        self.m_knot[dir].reverse();
        
        true
    }
    
    /// Transpose surface (swap u and v parameters)
    pub fn transpose(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }
        
        // Swap orders
        self.m_order.swap(0, 1);
        
        // Swap CV counts
        self.m_cv_count.swap(0, 1);
        
        // Rebuild CV array with transposed indices
        let cv_size = self.cv_size();
        let cv_count_total = self.m_cv_count[0] * self.m_cv_count[1];
        let mut new_cv = vec![0.0; cv_size * cv_count_total];
        
        for i in 0..self.m_cv_count[1] { // Note: swapped
            for j in 0..self.m_cv_count[0] {
                let old_idx = i * self.m_cv_stride[1] + j * self.m_cv_stride[0];
                let new_idx = j * cv_size * self.m_cv_count[1] + i * cv_size;
                
                for k in 0..cv_size {
                    new_cv[new_idx + k] = self.m_cv[old_idx + k];
                }
            }
        }
        
        self.m_cv = new_cv;
        
        // Update strides
        self.m_cv_stride[0] = cv_size;
        self.m_cv_stride[1] = cv_size * self.m_cv_count[0];
        
        // Swap knot vectors
        self.m_knot.swap(0, 1);
        self.m_knot_capacity.swap(0, 1);
        
        true
    }
    
    /// Clamp end in specified direction
    /// end: 0=start, 1=end, 2=both
    pub fn clamp_end(&mut self, dir: usize, end: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        let order = self.m_order[dir];
        
        if end == 0 || end == 2 {
            // Clamp start: set first 'order' knots to same value
            let start_val = self.m_knot[dir][order - 2];
            for i in 0..order {
                self.m_knot[dir][i] = start_val;
            }
        }
        
        if end == 1 || end == 2 {
            // Clamp end: set last 'order' knots to same value
            let knot_count = self.m_knot[dir].len();
            let end_val = self.m_knot[dir][self.m_cv_count[dir] - 1];
            for i in 0..order {
                self.m_knot[dir][knot_count - 1 - i] = end_val;
            }
        }
        
        true
    }
    
    /// Check if surface is planar within tolerance
    pub fn is_planar(&self, tolerance: f64) -> bool {
        if !self.is_valid() || self.m_cv_count[0] < 3 || self.m_cv_count[1] < 3 {
            return false;
        }
        
        // Get three non-collinear points to define plane
        let p0 = match self.get_cv(0, 0) {
            Some(p) => p,
            None => return false,
        };
        let p1 = match self.get_cv(self.m_cv_count[0] - 1, 0) {
            Some(p) => p,
            None => return false,
        };
        let p2 = match self.get_cv(0, self.m_cv_count[1] - 1) {
            Some(p) => p,
            None => return false,
        };
        
        // Compute plane normal
        let v1_x = p1[0] - p0[0];
        let v1_y = p1[1] - p0[1];
        let v1_z = p1[2] - p0[2];

        let v2_x = p2[0] - p0[0];
        let v2_y = p2[1] - p0[1];
        let v2_z = p2[2] - p0[2];
        
        let nx = v1_y * v2_z - v1_z * v2_y;
        let ny = v1_z * v2_x - v1_x * v2_z;
        let nz = v1_x * v2_y - v1_y * v2_x;
        
        let n_len = (nx * nx + ny * ny + nz * nz).sqrt();
        if n_len < 1e-10 {
            return false; // Degenerate
        }
        
        let nx = nx / n_len;
        let ny = ny / n_len;
        let nz = nz / n_len;
        
        // Check all CVs are on the plane
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(p) = self.get_cv(i, j) {
                    let dx = p[0] - p0[0];
                    let dy = p[1] - p0[1];
                    let dz = p[2] - p0[2];
                    let dist = (nx * dx + ny * dy + nz * dz).abs();
                    
                    if dist > tolerance {
                        return false;
                    }
                }
            }
        }
        
        true
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // TRANSFORMATION
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Apply stored xform transformation (in-place)
    pub fn transform_self(&mut self) {
        let xf = self.xform.clone();
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(mut pt) = self.get_cv(i, j) {
                    xf.transform_point(&mut pt);
                    self.set_cv(i, j, &pt);
                }
            }
        }
    }
    
    /// Apply custom transformation matrix (in-place)
    pub fn transform(&mut self, xform: &Xform) -> bool {
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(mut pt) = self.get_cv(i, j) {
                    xform.transform_point(&mut pt);
                    if !self.set_cv(i, j, &pt) {
                        return false;
                    }
                }
            }
        }
        true
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // STRING REPRESENTATION
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get string representation
    pub fn to_string(&self) -> String {
        format!(
            "NurbsSurface(name='{}', dim={}, is_rational={}, order=[{},{}], cv_count=[{},{}])",
            self.name,
            self.m_dim,
            self.m_is_rat,
            self.m_order[0],
            self.m_order[1],
            self.m_cv_count[0],
            self.m_cv_count[1]
        )
    }
}

impl Default for NurbsSurface {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Display for NurbsSurface {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}

#[cfg(test)]
#[path = "nurbssurface_test.rs"]
mod nurbssurface_test;
