use crate::point::Point;
use crate::nurbscurve::NurbsCurve;
use crate::xform::Xform;
use crate::color::Color;
use crate::vector::Vector;
use crate::boundingbox::BoundingBox;
use crate::knot;
use serde::{Serialize, Deserialize};

/// Non-Uniform Rational B-Spline (NURBS) surface implementation
///
/// Based on OpenNURBS ground truth implementation.
/// Matches the C++ and Python implementations exactly.
#[derive(Debug, Serialize, Deserialize)]
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
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub m_outer_loop: Option<NurbsCurve>,
}

impl NurbsSurface {
    /// Create a new empty NURBS surface
    pub fn new() -> Self {
        NurbsSurface {
            guid: uuid::Uuid::new_v4().to_string(),
            name: "my_nurbssurface".to_string(),
            width: 1.0,
            surfacecolor: Color::black(),
            xform: Xform::identity(),
            m_dim: 0,
            m_is_rat: false,
            m_order: [0, 0],
            m_cv_count: [0, 0],
            m_cv_stride: [0, 0],
            m_knot: [Vec::new(), Vec::new()],
            m_cv: Vec::new(),
            m_outer_loop: None,
        }
    }

    /// Create NURBS surface with specified parameters and optional knot vector initialization
    ///
    /// # Parameters
    /// - `dimension`: Dimension of the surface (typically 3)
    /// - `is_rational`: Whether the surface should be rational
    /// - `order0`: Order in u direction (degree + 1)
    /// - `order1`: Order in v direction (degree + 1)
    /// - `cv_count0`: Number of control vertices in u direction
    /// - `cv_count1`: Number of control vertices in v direction
    /// - `is_periodic_u`: If true, creates periodic uniform knot vector in u direction
    /// - `is_periodic_v`: If true, creates periodic uniform knot vector in v direction
    /// - `knot_delta_u`: Knot spacing in u direction
    /// - `knot_delta_v`: Knot spacing in v direction
    pub fn create_raw(
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
        is_periodic_u: bool,
        is_periodic_v: bool,
        knot_delta_u: f64,
        knot_delta_v: f64,
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

        // Allocate CV array
        let cv_size_total = cv_size * cv_count0 * cv_count1;
        srf.m_cv = vec![0.0; cv_size_total];

        // Initialize knot vectors
        // TODO: Add make_periodic_uniform_knot_vector implementation
        if is_periodic_u {
            eprintln!("Warning: Periodic uniform knot vectors not yet implemented in Rust. Using clamped uniform.");
        }
        srf.make_clamped_uniform_knot_vector(0, knot_delta_u);

        if is_periodic_v {
            eprintln!("Warning: Periodic uniform knot vectors not yet implemented in Rust. Using clamped uniform.");
        }
        srf.make_clamped_uniform_knot_vector(1, knot_delta_v);

        Some(srf)
    }

    /// Create NURBS surface from flat list of control points (row-major: u varies slowest)
    pub fn create(
        periodic_u: bool,
        periodic_v: bool,
        degree_u: usize,
        degree_v: usize,
        cv_count_u: usize,
        cv_count_v: usize,
        points: &[Point],
    ) -> Option<Self> {
        if cv_count_u < 2 || cv_count_v < 2 { return None; }
        if points.len() != cv_count_u * cv_count_v { return None; }
        let order_u = degree_u + 1;
        let order_v = degree_v + 1;
        let mut srf = Self::create_raw(
            3, false, order_u, order_v, cv_count_u, cv_count_v,
            periodic_u, periodic_v, 1.0, 1.0,
        )?;
        for i in 0..cv_count_u {
            for j in 0..cv_count_v {
                srf.set_cv(i, j, &points[i * cv_count_v + j]);
            }
        }
        Some(srf)
    }

    /// Create NURBS surface with default knot vectors (clamped uniform, delta=1.0)
    ///
    /// Convenience method for backward compatibility. Equivalent to:
    /// `create(dimension, is_rational, order0, order1, cv_count0, cv_count1, false, false, 1.0, 1.0)`
    pub fn create_simple(
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
    ) -> Option<Self> {
        Self::create_raw(dimension, is_rational, order0, order1, cv_count0, cv_count1,
                    false, false, 1.0, 1.0)
    }

    /// Create clamped uniform NURBS surface
    pub fn create_clamped_uniform(
        &mut self,
        dimension: usize,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
        knot_delta0: f64,
        knot_delta1: f64,
    ) -> bool {
        // Create surface with given parameters and knot vectors
        let srf = match Self::create_raw(dimension, false, order0, order1, cv_count0, cv_count1,
                                     false, false, knot_delta0, knot_delta1) {
            Some(s) => s,
            None => return false,
        };

        *self = srf;

        true
    }

    pub fn create_ruled(curve_a: &NurbsCurve, curve_b: &NurbsCurve) -> Option<Self> {
        if !curve_a.is_valid() || !curve_b.is_valid() {
            return None;
        }

        let mut ca = curve_a.duplicate();
        let mut cb = curve_b.duplicate();

        ca.set_domain(0.0, 1.0);
        cb.set_domain(0.0, 1.0);

        if ca.degree() < cb.degree() {
            ca.increase_degree(cb.degree());
        } else if cb.degree() < ca.degree() {
            cb.increase_degree(ca.degree());
        }

        if ca.is_rational() || cb.is_rational() {
            ca.make_rational();
            cb.make_rational();
        }

        let knots_b = cb.get_knots();
        let tol = 1e-10;

        for &k in &knots_b {
            let found = ca.get_knots().iter().any(|&ka| (ka - k).abs() < tol);
            if !found {
                ca.insert_knot(k, 1);
            }
        }

        let knots_a = ca.get_knots();
        for &k in &knots_a {
            let found = cb.get_knots().iter().any(|&kb| (kb - k).abs() < tol);
            if !found {
                cb.insert_knot(k, 1);
            }
        }

        let order_u = ca.order();
        let cv_count_u = ca.cv_count();
        let is_rat = ca.is_rational();

        let mut surface = Self::create_raw(3, is_rat, order_u, 2, cv_count_u, 2, false, false, 1.0, 1.0)?;

        for i in 0..ca.knot_count() {
            if let Some(kv) = ca.knot(i) {
                surface.set_knot(0, i, kv);
            }
        }

        surface.set_knot(1, 0, 0.0);
        surface.set_knot(1, 1, 1.0);

        if is_rat {
            for i in 0..cv_count_u {
                if let Some((ax, ay, az, aw)) = ca.get_cv_4d(i) {
                    surface.set_cv_4d(i, 0, ax, ay, az, aw);
                }
                if let Some((bx, by, bz, bw)) = cb.get_cv_4d(i) {
                    surface.set_cv_4d(i, 1, bx, by, bz, bw);
                }
            }
        } else {
            for i in 0..cv_count_u {
                if let Some(pt_a) = ca.get_cv(i) {
                    surface.set_cv(i, 0, &pt_a);
                }
                if let Some(pt_b) = cb.get_cv(i) {
                    surface.set_cv(i, 1, &pt_b);
                }
            }
        }

        Some(surface)
    }

    pub fn create_planar(curves: &[NurbsCurve]) -> Option<Self> {
        if curves.is_empty() { return None; }

        let mut all_pts = Vec::new();
        for crv in curves {
            for i in 0..crv.cv_count() {
                if let Some(pt) = crv.get_cv(i) {
                    all_pts.push(pt);
                }
            }
        }
        if all_pts.len() < 3 { return None; }

        use crate::plane::Plane;
        let plane = Plane::from_points_pca(all_pts.clone());
        if plane.z_axis().magnitude() < 1e-10 { return None; }

        let xax = plane.x_axis();
        let yax = plane.y_axis();
        let orig = plane.origin();

        let mut min_u = f64::MAX;
        let mut max_u = f64::MIN;
        let mut min_v = f64::MAX;
        let mut max_v = f64::MIN;

        for pt in &all_pts {
            let dx = pt[0] - orig[0];
            let dy = pt[1] - orig[1];
            let dz = pt[2] - orig[2];
            let u = dx * xax[0] + dy * xax[1] + dz * xax[2];
            let v = dx * yax[0] + dy * yax[1] + dz * yax[2];
            if u < min_u { min_u = u; }
            if u > max_u { max_u = u; }
            if v < min_v { min_v = v; }
            if v > max_v { max_v = v; }
        }

        let mut pad = (max_u - min_u).max(max_v - min_v) * 0.05;
        if pad < 1e-6 { pad = 1.0; }
        min_u -= pad; max_u += pad;
        min_v -= pad; max_v += pad;

        let range_u = max_u - min_u;
        let range_v = max_v - min_v;

        let mut surface = Self::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0)?;
        surface.set_knot(0, 0, 0.0);
        surface.set_knot(0, 1, 1.0);
        surface.set_knot(1, 0, 0.0);
        surface.set_knot(1, 1, 1.0);

        let plane_pt = |u: f64, v: f64| -> Point {
            Point::new(
                orig[0] + u * xax[0] + v * yax[0],
                orig[1] + u * xax[1] + v * yax[1],
                orig[2] + u * xax[2] + v * yax[2],
            )
        };

        surface.set_cv(0, 0, &plane_pt(min_u, min_v));
        surface.set_cv(0, 1, &plane_pt(min_u, max_v));
        surface.set_cv(1, 0, &plane_pt(max_u, min_v));
        surface.set_cv(1, 1, &plane_pt(max_u, max_v));

        let mut uv_pts = Vec::new();
        for crv in curves {
            let (pts3d, _params) = crv.divide_by_count(50, true);
            for pt in &pts3d {
                let dx = pt[0] - orig[0];
                let dy = pt[1] - orig[1];
                let dz = pt[2] - orig[2];
                let pu = dx * xax[0] + dy * xax[1] + dz * xax[2];
                let pv = dx * yax[0] + dy * yax[1] + dz * yax[2];
                let nu = (pu - min_u) / range_u;
                let nv = (pv - min_v) / range_v;
                uv_pts.push(Point::new(nu, nv, 0.0));
            }
        }

        if uv_pts.len() >= 3 {
            let loop_crv = NurbsCurve::create(false, 3, &uv_pts);
            surface.m_outer_loop = Some(loop_crv);
        }

        Some(surface)
    }

    pub fn set_outer_loop(&mut self, loop_crv: NurbsCurve) {
        self.m_outer_loop = Some(loop_crv);
    }

    pub fn get_outer_loop(&self) -> Option<&NurbsCurve> {
        self.m_outer_loop.as_ref()
    }

    pub fn is_trimmed(&self) -> bool {
        self.m_outer_loop.as_ref().map_or(false, |c| c.is_valid())
    }

    pub fn clear_outer_loop(&mut self) {
        self.m_outer_loop = None;
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

    /// Get control vertex as homogeneous coordinates (x, y, z, w)
    pub fn get_cv_4d(&self, i: usize, j: usize) -> Option<(f64, f64, f64, f64)> {
        let cv = self.cv(i, j)?;
        let x = if cv.len() > 0 { cv[0] } else { 0.0 };
        let y = if cv.len() > 1 { cv[1] } else { 0.0 };
        let z = if cv.len() > 2 { cv[2] } else { 0.0 };
        let w = if self.m_is_rat && cv.len() > self.m_dim { cv[self.m_dim] } else { 1.0 };
        Some((x, y, z, w))
    }

    /// Set control vertex from homogeneous coordinates (x, y, z, w)
    pub fn set_cv_4d(&mut self, i: usize, j: usize, x: f64, y: f64, z: f64, w: f64) -> bool {
        let is_rat = self.m_is_rat;
        let dim = self.m_dim;
        if let Some(cv) = self.cv_mut(i, j) {
            cv[0] = x;
            if cv.len() > 1 { cv[1] = y; }
            if cv.len() > 2 { cv[2] = z; }
            if is_rat && cv.len() > dim { cv[dim] = w; }
            true
        } else {
            false
        }
    }

    /// Set control vertex from Point
    pub fn set_cv(&mut self, i: usize, j: usize, point: &Point) -> bool {
        let is_rat = self.m_is_rat;
        let dim = self.m_dim;

        if let Some(cv) = self.cv_mut(i, j) {
            if is_rat && cv.len() > dim {
                // For rational surfaces, store homogeneous coordinates (x*w, y*w, z*w, w)
                let mut w = cv[dim];
                if w.abs() < 1e-14 { w = 1.0; }
                cv[0] = point[0] * w;
                if cv.len() > 1 { cv[1] = point[1] * w; }
                if cv.len() > 2 { cv[2] = point[2] * w; }
            } else {
                cv[0] = point[0];
                if cv.len() > 1 { cv[1] = point[1]; }
                if cv.len() > 2 { cv[2] = point[2]; }
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
                // Rescale homogeneous coordinates when changing weight
                let mut old_w = cv[dim];
                if old_w.abs() < 1e-14 { old_w = 1.0; }
                let mut new_w = w;
                if new_w.abs() < 1e-14 { new_w = 1.0; }
                let scale = new_w / old_w;
                cv[0] *= scale;
                if cv.len() > 1 { cv[1] *= scale; }
                if cv.len() > 2 { cv[2] *= scale; }
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

        // Use knot module function
        let result = knot::make_clamped_uniform(self.m_order[dir], self.m_cv_count[dir], delta);
        if result.is_empty() {
            return false;
        }
        self.m_knot[dir] = result;
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

    /// Set surface domain in specified direction
    pub fn set_domain(&mut self, dir: usize, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || dir >= 2 || t0 >= t1 {
            return false;
        }

        let (d0, d1) = match self.domain(dir) {
            Some(d) => d,
            None => return false,
        };

        if (d1 - d0).abs() < 1e-14 {
            return false;
        }

        let scale = (t1 - t0) / (d1 - d0);
        for i in 0..self.m_knot[dir].len() {
            self.m_knot[dir][i] = t0 + (self.m_knot[dir][i] - d0) * scale;
        }
        true
    }

    /// Get span (distinct knot intervals) values in specified direction
    pub fn get_span_vector(&self, dir: usize) -> Vec<f64> {
        if dir >= 2 || !self.is_valid() {
            return Vec::new();
        }

        let mut spans = Vec::new();
        let tol = 1e-10;

        if self.m_knot[dir].is_empty() {
            return spans;
        }

        spans.push(self.m_knot[dir][0]);

        for i in 1..self.m_knot[dir].len() {
            let diff = self.m_knot[dir][i] - *spans.last().unwrap();
            if diff.abs() > tol {
                spans.push(self.m_knot[dir][i]);
            }
        }

        spans
    }

    /// Get knot multiplicity at index in specified direction
    pub fn knot_multiplicity(&self, dir: usize, knot_index: usize) -> usize {
        if dir >= 2 {
            return 0;
        }
        
        // Use knot module function
        knot::multiplicity(self.m_order[dir], self.m_cv_count[dir], &self.m_knot[dir], knot_index)
    }

    /// Get all knot values for specified direction
    pub fn get_knots(&self, dir: usize) -> Vec<f64> {
        if dir < 2 {
            self.m_knot[dir].clone()
        } else {
            Vec::new()
        }
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
        if dir >= 2 || self.m_knot[dir].is_empty() {
            return false;
        }
        
        // Use knot module function
        knot::is_clamped(self.m_order[dir], self.m_cv_count[dir], &self.m_knot[dir], end as i32)
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
        let n = dv.cross(du);
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

        // Use knot module function
        knot::find_span(self.m_order[dir], self.m_cv_count[dir], &self.m_knot[dir], t) as isize
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

    /// Get point at corner (u_end, v_end) where end is 0 or 1
    pub fn point_at_corner(&self, u_end: usize, v_end: usize) -> Option<Point> {
        let (u0, u1) = self.domain(0)?;
        let (v0, v1) = self.domain(1)?;

        let u = if u_end == 0 { u0 } else { u1 };
        let v = if v_end == 0 { v0 } else { v1 };

        self.point_at(u, v)
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
                let idx1 = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
                let idx2 = i1 * self.m_cv_stride[0] + j1 * self.m_cv_stride[1];
                
                for k in 0..cv_size {
                    self.m_cv.swap(idx1 + k, idx2 + k);
                }
            }
        }
        
        // Reverse knot vector using knot module function
        knot::reverse(self.m_order[dir], self.m_cv_count[dir], &mut self.m_knot[dir])
    }
    
    /// Transpose surface (swap u and v parameters)
    pub fn transpose(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        // Save original values before swapping
        let old_cv_count_0 = self.m_cv_count[0];
        let old_cv_count_1 = self.m_cv_count[1];
        let old_cv_stride_0 = self.m_cv_stride[0];
        let old_cv_stride_1 = self.m_cv_stride[1];

        // Swap orders
        self.m_order.swap(0, 1);

        // Swap CV counts
        self.m_cv_count.swap(0, 1);

        // Rebuild CV array with transposed indices
        let cv_size = self.cv_size();
        let cv_count_total = self.m_cv_count[0] * self.m_cv_count[1];
        let mut new_cv = vec![0.0; cv_size * cv_count_total];

        // Use OLD counts and strides to read from old array
        for i in 0..old_cv_count_0 {
            for j in 0..old_cv_count_1 {
                let old_idx = i * old_cv_stride_0 + j * old_cv_stride_1;
                // Transpose: old (i,j) becomes new (j,i)
                let new_idx = j * cv_size * self.m_cv_count[1] + i * cv_size;

                for k in 0..cv_size {
                    new_cv[new_idx + k] = self.m_cv[old_idx + k];
                }
            }
        }

        self.m_cv = new_cv;

        // Update strides for new layout
        self.m_cv_stride[0] = cv_size * self.m_cv_count[1];
        self.m_cv_stride[1] = cv_size;

        // Swap knot vectors
        self.m_knot.swap(0, 1);

        true
    }

    /// Swap two coordinate axes
    pub fn swap_coordinates(&mut self, axis_i: usize, axis_j: usize) -> bool {
        if axis_i >= self.m_dim || axis_j >= self.m_dim || axis_i == axis_j {
            return false;
        }
        if !self.is_valid() {
            return false;
        }

        // Swap coordinates in all control vertices
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(cv_slice) = self.cv_mut(i, j) {
                    cv_slice.swap(axis_i, axis_j);
                }
            }
        }
        true
    }

    /// Zero all control vertices (set weights to 1 if rational)
    pub fn zero_cvs(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        // Store values before borrowing
        let dim = self.m_dim;
        let is_rat = self.m_is_rat;

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(cv_slice) = self.cv_mut(i, j) {
                    // Zero coordinates
                    for k in 0..dim {
                        cv_slice[k] = 0.0;
                    }
                    // Set weight to 1 if rational
                    if is_rat && cv_slice.len() > dim {
                        cv_slice[dim] = 1.0;
                    }
                }
            }
        }
        true
    }

    /// Clamp end in specified direction
    /// end: 0=start, 1=end, 2=both
    pub fn clamp_end(&mut self, dir: usize, end: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        // Use knot module function
        knot::clamp(self.m_order[dir], self.m_cv_count[dir], &mut self.m_knot[dir], end as i32)
    }
    
    /// Subdivide surface into a grid of points.
    ///
    /// Evaluates the surface at regular intervals in both parameter directions
    /// to create a grid of points.
    ///
    /// # Arguments
    ///
    /// * `nu` - Number of subdivisions in u direction
    /// * `nv` - Number of subdivisions in v direction
    ///
    /// # Returns
    ///
    /// 2D vector of points, where grid\[i\]\[j\] is the point at subdivision (i, j).
    /// Grid dimensions are (nu+1) x (nv+1).
    pub fn divide_by_count(&self, nu: usize, nv: usize) -> (Vec<Vec<Point>>, Vec<Vec<(f64, f64)>>) {
        let (u0, u1) = match self.domain(0) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new()),
        };
        let (v0, v1) = match self.domain(1) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new()),
        };

        let mut grid = vec![vec![Point::new(0.0, 0.0, 0.0); nv + 1]; nu + 1];
        let mut params = vec![vec![(0.0, 0.0); nv + 1]; nu + 1];

        for i in 0..=nu {
            let u = if nu > 0 {
                u0 + (u1 - u0) * (i as f64 / nu as f64)
            } else {
                u0
            };

            for j in 0..=nv {
                let v = if nv > 0 {
                    v0 + (v1 - v0) * (j as f64 / nv as f64)
                } else {
                    v0
                };

                grid[i][j] = self.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                params[i][j] = (u, v);
            }
        }

        (grid, params)
    }

    /// Check if surface is planar within tolerance
    pub fn is_planar(&self, tolerance: f64) -> bool {
        if !self.is_valid() || self.m_cv_count[0] < 2 || self.m_cv_count[1] < 2 {
            return false;
        }
        
        // For 2x2 or smaller, all points define a plane (4 points always coplanar)
        if self.m_cv_count[0] <= 2 && self.m_cv_count[1] <= 2 {
            return true;
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
    
    /// Check if a surface side is singular (all CVs along edge coincide)
    /// side: 0=south (v=0), 1=east (u=max), 2=north (v=max), 3=west (u=0)
    pub fn is_singular(&self, side: usize) -> bool {
        if !self.is_valid() { return false; }
        let tol = 1e-10;
        let (count, get_pt): (usize, Box<dyn Fn(usize) -> Option<Point>>) = match side {
            0 => (self.m_cv_count[0], Box::new(|i| self.get_cv(i, 0))),
            1 => (self.m_cv_count[1], Box::new(|j| self.get_cv(self.m_cv_count[0] - 1, j))),
            2 => (self.m_cv_count[0], Box::new(|i| self.get_cv(i, self.m_cv_count[1] - 1))),
            3 => (self.m_cv_count[1], Box::new(|j| self.get_cv(0, j))),
            _ => return false,
        };
        if count < 2 { return true; }
        let first = match get_pt(0) { Some(p) => p, None => return false };
        for k in 1..count {
            if let Some(p) = get_pt(k) {
                let dx = (p[0] - first[0]).abs();
                let dy = (p[1] - first[1]).abs();
                let dz = (p[2] - first[2]).abs();
                if dx > tol || dy > tol || dz > tol { return false; }
            }
        }
        true
    }

    /// Get axis-aligned bounding box from control vertices
    pub fn get_bounding_box(&self) -> BoundingBox {
        let mut min_pt = Point::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max_pt = Point::new(f64::MIN, f64::MIN, f64::MIN);
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(pt) = self.get_cv(i, j) {
                    if pt[0] < min_pt[0] { min_pt[0] = pt[0]; }
                    if pt[1] < min_pt[1] { min_pt[1] = pt[1]; }
                    if pt[2] < min_pt[2] { min_pt[2] = pt[2]; }
                    if pt[0] > max_pt[0] { max_pt[0] = pt[0]; }
                    if pt[1] > max_pt[1] { max_pt[1] = pt[1]; }
                    if pt[2] > max_pt[2] { max_pt[2] = pt[2]; }
                }
            }
        }
        let center = Point::new(
            (min_pt[0] + max_pt[0]) * 0.5,
            (min_pt[1] + max_pt[1]) * 0.5,
            (min_pt[2] + max_pt[2]) * 0.5,
        );
        let half_size = Vector::new(
            (max_pt[0] - min_pt[0]) * 0.5,
            (max_pt[1] - min_pt[1]) * 0.5,
            (max_pt[2] - min_pt[2]) * 0.5,
        );
        BoundingBox::new(
            center,
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            half_size,
        )
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
    
    /// Get string representation (matches Python to_string format)
    pub fn to_string(&self) -> String {
        format!(
            "NurbsSurface(name={}, degree=({},{}), cvs=({},{}))",
            self.name,
            self.degree(0),
            self.degree(1),
            self.m_cv_count[0],
            self.m_cv_count[1]
        )
    }

    pub fn repr(&self) -> String {
        let mut result = format!("NurbsSurface(\n  name={},\n  degree=({},{}),\n  cvs=({},{}),\n  rational={},\n  control_points=[\n",
            self.name, self.degree(0), self.degree(1), self.m_cv_count[0], self.m_cv_count[1], self.m_is_rat);
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(p) = self.get_cv(i, j) {
                    result += &format!("    {}, {}, {}\n", p[0], p[1], p[2]);
                }
            }
        }
        result += "  ]\n)";
        result
    }

    /// Create a duplicate with a new GUID (copies all data except generates new GUID)
    pub fn duplicate(&self) -> Self {
        self.clone()
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

    pub fn json_dumps(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default()
    }

    pub fn json_loads(json_string: &str) -> Self {
        serde_json::from_str(json_string).unwrap_or_else(|_| Self::default())
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

    /// Serialize to protobuf binary data.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        let proto = crate::proto::NurbsSurface {
            guid: self.guid.clone(),
            name: self.name.clone(),
            dimension: self.m_dim as i32,
            is_rational: self.m_is_rat,
            order_u: self.m_order[0] as i32,
            order_v: self.m_order[1] as i32,
            cv_count_u: self.m_cv_count[0] as i32,
            cv_count_v: self.m_cv_count[1] as i32,
            cv_stride_u: self.m_cv_stride[0] as i32,
            cv_stride_v: self.m_cv_stride[1] as i32,
            knots_u: self.m_knot[0].clone(),
            knots_v: self.m_knot[1].clone(),
            cvs: self.m_cv.clone(),
            width: self.width,
            surfacecolor: Some(crate::proto::Color {
                guid: self.surfacecolor.guid.clone(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r as i32,
                g: self.surfacecolor.g as i32,
                b: self.surfacecolor.b as i32,
                a: self.surfacecolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    /// Create NurbsSurface from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded surface data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized NurbsSurface or an error.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::NurbsSurface::decode(data)?;

        // Create surface with correct dimensions
        let mut surface = if let Some(srf) = Self::create_raw(
            proto.dimension as usize,
            proto.is_rational,
            proto.order_u as usize,
            proto.order_v as usize,
            proto.cv_count_u as usize,
            proto.cv_count_v as usize,
            false,
            false,
            1.0,
            1.0,
        ) {
            srf
        } else {
            return Err("Failed to create NurbsSurface from protobuf".into());
        };

        // Load metadata
        surface.guid = proto.guid;
        surface.name = proto.name;
        surface.width = proto.width;

        // Load knot vectors
        if proto.knots_u.len() == surface.m_knot[0].len() {
            surface.m_knot[0] = proto.knots_u;
        }
        if proto.knots_v.len() == surface.m_knot[1].len() {
            surface.m_knot[1] = proto.knots_v;
        }

        // Load control vertices
        if proto.cvs.len() == surface.m_cv.len() {
            surface.m_cv = proto.cvs;
        }

        // Load color
        if let Some(color) = proto.surfacecolor {
            surface.surfacecolor.guid = color.guid;
            surface.surfacecolor.name = color.name;
            surface.surfacecolor.r = color.r as u8;
            surface.surfacecolor.g = color.g as u8;
            surface.surfacecolor.b = color.b as u8;
            surface.surfacecolor.a = color.a as u8;
        }

        // Load transform
        if let Some(xform) = proto.xform {
            surface.xform.guid = xform.guid;
            surface.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    surface.xform.m[i] = *val;
                }
            }
        }

        Ok(surface)
    }

    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized NurbsSurface.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl Default for NurbsSurface {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for NurbsSurface {
    fn eq(&self, other: &Self) -> bool {
        // Compare metadata (excluding guid)
        if self.name != other.name { return false; }
        if self.width != other.width { return false; }
        if self.surfacecolor != other.surfacecolor { return false; }
        if self.xform != other.xform { return false; }

        // Compare NURBS structure
        if self.m_dim != other.m_dim { return false; }
        if self.m_is_rat != other.m_is_rat { return false; }
        if self.m_order != other.m_order { return false; }
        if self.m_cv_count != other.m_cv_count { return false; }
        if self.m_cv_stride != other.m_cv_stride { return false; }

        // Compare knot vectors
        if self.m_knot[0] != other.m_knot[0] { return false; }
        if self.m_knot[1] != other.m_knot[1] { return false; }

        // Compare control vertices
        if self.m_cv != other.m_cv { return false; }

        true
    }
}

impl Eq for NurbsSurface {}

impl Clone for NurbsSurface {
    fn clone(&self) -> Self {
        NurbsSurface {
            guid: uuid::Uuid::new_v4().to_string(), // Generate new GUID
            name: self.name.clone(),
            width: self.width,
            surfacecolor: self.surfacecolor.clone(),
            xform: self.xform.clone(),
            m_dim: self.m_dim,
            m_is_rat: self.m_is_rat,
            m_order: self.m_order,
            m_cv_count: self.m_cv_count,
            m_cv_stride: self.m_cv_stride,
            m_knot: self.m_knot.clone(),
            m_cv: self.m_cv.clone(),
            m_outer_loop: self.m_outer_loop.clone(),
        }
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
