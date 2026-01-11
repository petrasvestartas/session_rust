//! Knot vector utility functions for NURBS curves and surfaces.
//!
//! This module provides standalone functions for working with knot vectors,
//! following the OpenNURBS pattern (opennurbs_knot.h).
//!
//! These functions operate on slices and can be used independently
//! or called by NurbsCurve and NurbsSurface.

use serde::{Deserialize, Serialize};

/// Knot spacing style for interpolated curves (matches Rhino's CurveKnotStyle).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(u8)]
pub enum CurveKnotStyle {
    /// Parameter spacing = 1.0
    Uniform = 0,
    /// Chord-length parameterization
    #[default]
    Chord = 1,
    /// Centripetal (sqrt chord) parameterization
    ChordSquareRoot = 2,
    /// Periodic + uniform
    UniformPeriodic = 3,
    /// Periodic + chord
    ChordPeriodic = 4,
    /// Periodic + centripetal
    ChordSquareRootPeriodic = 5,
}

/// Compute the number of knots in a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1), must be >= 2.
/// * `cv_count` - Number of control vertices, must be >= order.
///
/// # Returns
/// Number of knots: order + cv_count - 2
#[inline]
pub fn knot_count(order: usize, cv_count: usize) -> usize {
    order + cv_count - 2
}

/// Compute tolerance associated with a domain interval.
///
/// # Arguments
/// * `a` - Start of domain.
/// * `b` - End of domain.
///
/// # Returns
/// Tolerance value.
#[inline]
pub fn domain_tolerance(a: f64, b: f64) -> f64 {
    if a == b {
        return 0.0;
    }
    const SQRT_EPSILON: f64 = 1.4901161193847656e-08;
    const EPSILON: f64 = 2.220446049250313e-16;
    let tol = (a.abs() + b.abs() + (a - b).abs()) * SQRT_EPSILON;
    if tol < EPSILON { EPSILON } else { tol }
}

/// Create a clamped uniform knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1), must be >= 2.
/// * `cv_count` - Number of control vertices, must be >= order.
/// * `delta` - Spacing between interior knots.
///
/// # Returns
/// Clamped uniform knot vector, or empty if invalid params.
pub fn make_clamped_uniform(order: usize, cv_count: usize, delta: f64) -> Vec<f64> {
    if order < 2 || cv_count < order || delta <= 0.0 {
        return Vec::new();
    }
    
    let kc = knot_count(order, cv_count);
    let mut knot = vec![0.0; kc];
    
    // Fill interior knots: from index (order-2) to (cv_count-1)
    let mut k = 0.0;
    for i in (order - 2)..cv_count {
        knot[i] = k;
        k += delta;
    }
    
    // Clamp both ends
    clamp(order, cv_count, &mut knot, 2);
    
    knot
}

/// Create a periodic uniform knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1), must be >= 2.
/// * `cv_count` - Number of control vertices, must be >= order.
/// * `delta` - Spacing between knots.
///
/// # Returns
/// Periodic uniform knot vector, or empty if invalid params.
pub fn make_periodic_uniform(order: usize, cv_count: usize, delta: f64) -> Vec<f64> {
    if order < 2 || cv_count < order || delta <= 0.0 {
        return Vec::new();
    }
    
    let kc = knot_count(order, cv_count);
    let mut knot = vec![0.0; kc];
    
    let mut k = 0.0;
    for i in 0..kc {
        knot[i] = k;
        k += delta;
    }
    
    knot
}

/// Clamp the ends of a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector to clamp (modified in place).
/// * `end` - Which end to clamp: 0 = left, 1 = right, 2 = both.
///
/// # Returns
/// True if successful.
pub fn clamp(order: usize, cv_count: usize, knot: &mut [f64], end: i32) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return false;
    }
    
    // Clamp left end
    if end == 0 || end == 2 {
        let clamp_value = knot[order - 2];
        for i in 0..(order - 2) {
            knot[i] = clamp_value;
        }
    }
    
    // Clamp right end
    if end == 1 || end == 2 {
        let clamp_value = knot[cv_count - 1];
        for i in cv_count..kc {
            knot[i] = clamp_value;
        }
    }
    
    true
}

/// Check if a knot vector is valid.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector to validate.
///
/// # Returns
/// True if the knot vector is valid.
pub fn is_valid(order: usize, cv_count: usize, knot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return false;
    }
    
    // Check non-decreasing
    for i in 1..kc {
        if knot[i] < knot[i - 1] {
            return false;
        }
    }
    
    // Check no degenerate spans (knot[i] < knot[i + order - 1])
    for i in 0..(kc - order + 1) {
        if knot[i] >= knot[i + order - 1] {
            return false;
        }
    }
    
    true
}

/// Check if a knot vector is clamped.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector to check.
/// * `end` - Which end to check: 0 = left, 1 = right, 2 = both.
///
/// # Returns
/// True if the knot vector is clamped at the specified end(s).
pub fn is_clamped(order: usize, cv_count: usize, knot: &[f64], end: i32) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return false;
    }
    
    let mult = order - 1;
    const TOL: f64 = 1e-10;
    
    // Check left end
    if end == 0 || end == 2 {
        if mult > kc {
            return false;
        }
        let start_value = knot[0];
        for i in 1..mult {
            if (knot[i] - start_value).abs() > TOL {
                return false;
            }
        }
    }
    
    // Check right end
    if end == 1 || end == 2 {
        if mult > kc {
            return false;
        }
        let end_value = knot[kc - 1];
        for i in 1..mult {
            if (knot[kc - 1 - i] - end_value).abs() > TOL {
                return false;
            }
        }
    }
    
    true
}

/// Check if a knot vector is periodic.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector to check.
///
/// # Returns
/// True if the knot vector is periodic.
pub fn is_periodic(order: usize, cv_count: usize, knot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc || kc < 2 {
        return false;
    }
    
    let delta = knot[1] - knot[0];
    if delta <= 0.0 {
        return false;
    }
    
    const TOL: f64 = 1e-10;
    for i in 2..kc {
        if ((knot[i] - knot[i - 1]) - delta).abs() > TOL {
            return false;
        }
    }
    
    true
}

/// Check if a knot vector is uniform (interior knots evenly spaced).
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector to check.
///
/// # Returns
/// True if the interior knots are uniformly spaced.
pub fn is_uniform(order: usize, cv_count: usize, knot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return false;
    }
    
    // Check interior knots (from order-2 to cv_count-1)
    if cv_count <= order {
        return true;  // No interior knots
    }
    
    let start_idx = order - 2;
    let end_idx = cv_count - 1;
    
    if end_idx <= start_idx {
        return true;
    }
    
    let delta = knot[start_idx + 1] - knot[start_idx];
    if delta <= 0.0 {
        return false;
    }
    
    const TOL: f64 = 1e-10;
    for i in (start_idx + 2)..=end_idx {
        if ((knot[i] - knot[i - 1]) - delta).abs() > TOL {
            return false;
        }
    }
    
    true
}

/// Get the domain of a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
///
/// # Returns
/// Tuple (t0, t1) domain interval.
pub fn get_domain(order: usize, cv_count: usize, knot: &[f64]) -> (f64, f64) {
    if order < 2 || cv_count < order || knot.len() < knot_count(order, cv_count) {
        return (0.0, 0.0);
    }
    
    (knot[order - 2], knot[cv_count - 1])
}

/// Set the domain of a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector (modified in place).
/// * `t0` - New domain start.
/// * `t1` - New domain end.
///
/// # Returns
/// True if successful.
pub fn set_domain(order: usize, cv_count: usize, knot: &mut [f64], t0: f64, t1: f64) -> bool {
    if order < 2 || cv_count < order || t0 >= t1 {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return false;
    }
    
    let (old_t0, old_t1) = get_domain(order, cv_count, knot);
    if old_t1 <= old_t0 {
        return false;
    }
    
    let scale = (t1 - t0) / (old_t1 - old_t0);
    for i in 0..kc {
        knot[i] = t0 + (knot[i] - old_t0) * scale;
    }
    
    true
}

/// Reverse a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector (modified in place).
///
/// # Returns
/// True if successful.
pub fn reverse(order: usize, cv_count: usize, knot: &mut [f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return false;
    }
    
    // Reverse the array
    knot.reverse();
    
    // Negate and shift to maintain same domain direction
    let t0 = knot[0];
    let t1 = knot[kc - 1];
    for i in 0..kc {
        knot[i] = t0 + t1 - knot[i];
    }
    
    true
}

/// Get the multiplicity of a knot.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
/// * `knot_index` - Index of the knot to check.
///
/// # Returns
/// Multiplicity of the knot at the given index.
pub fn multiplicity(order: usize, cv_count: usize, knot: &[f64], knot_index: usize) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc || knot_index >= kc {
        return 0;
    }
    
    let knot_value = knot[knot_index];
    let mut mult = 1usize;
    const TOL: f64 = 1e-14;
    
    // Count preceding equal knots
    let mut i = knot_index;
    while i > 0 {
        i -= 1;
        if (knot[i] - knot_value).abs() < TOL {
            mult += 1;
        } else {
            break;
        }
    }
    
    // Count following equal knots
    let mut i = knot_index + 1;
    while i < kc {
        if (knot[i] - knot_value).abs() < TOL {
            mult += 1;
            i += 1;
        } else {
            break;
        }
    }
    
    mult
}

/// Get the number of spans in a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
///
/// # Returns
/// Number of non-empty spans.
pub fn span_count(order: usize, cv_count: usize, knot: &[f64]) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return 0;
    }
    
    let mut count = 0;
    let d = order - 1;  // degree
    
    for i in 0..(cv_count - order + 1) {
        if knot[i + d - 1] < knot[i + d] {
            count += 1;
        }
    }
    
    count
}

/// Get the span breakpoints of a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
///
/// # Returns
/// Vector of unique knot values that define span boundaries.
pub fn get_span_vector(order: usize, cv_count: usize, knot: &[f64]) -> Vec<f64> {
    if order < 2 || cv_count < order {
        return Vec::new();
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return Vec::new();
    }
    
    let mut spans = Vec::new();
    const TOL: f64 = 1e-14;
    
    for i in 0..(kc - 1) {
        if (knot[i + 1] - knot[i]).abs() > TOL {
            spans.push(knot[i]);
        }
    }
    
    if kc > 0 {
        spans.push(knot[kc - 1]);
    }
    
    spans
}

/// Find the knot span containing parameter t.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
/// * `t` - Parameter value to locate.
///
/// # Returns
/// Span index in range [0, cv_count - order].
pub fn find_span(order: usize, cv_count: usize, knot: &[f64], t: f64) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return 0;
    }
    
    // Shift by (order - 2) as in OpenNURBS
    let knot_offset = order - 2;
    let span_len = cv_count - order + 2;
    
    // Handle boundary cases
    if t <= knot[knot_offset] {
        return 0;
    }
    if t >= knot[knot_offset + span_len - 1] {
        return span_len - 2;
    }
    
    // Binary search
    let mut low = 0;
    let mut high = span_len - 1;
    
    while high > low + 1 {
        let mid = (low + high) / 2;
        if t < knot[knot_offset + mid] {
            high = mid;
        } else {
            low = mid;
        }
    }
    
    low
}

/// Get the superfluous knot value at the specified end.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
/// * `end` - 0 = first superfluous knot, 1 = last superfluous knot.
///
/// # Returns
/// Superfluous knot value.
pub fn superfluous_knot(order: usize, cv_count: usize, knot: &[f64], end: i32) -> f64 {
    if order < 2 || cv_count < order {
        return 0.0;
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return 0.0;
    }
    
    if end == 0 {
        // First superfluous knot
        2.0 * knot[0] - knot[order - 2]
    } else {
        // Last superfluous knot
        2.0 * knot[kc - 1] - knot[cv_count - order]
    }
}

/// Compute a single Greville abscissa.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `knot` - Slice of (order - 1) knot values.
///
/// # Returns
/// Greville abscissa (average of the knots).
pub fn greville_abcissa(order: usize, knot: &[f64]) -> f64 {
    if order < 2 || knot.len() < order - 1 {
        return 0.0;
    }
    
    let d = order - 1;  // degree
    let sum: f64 = knot[..d].iter().sum();
    sum / d as f64
}

/// Get all Greville abscissae for a knot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `knot` - Knot vector.
/// * `periodic` - True for periodic curves.
///
/// # Returns
/// Vector of Greville abscissae.
pub fn get_greville_abcissae(order: usize, cv_count: usize, knot: &[f64], periodic: bool) -> Vec<f64> {
    if order < 2 || cv_count < order {
        return Vec::new();
    }
    
    let kc = knot_count(order, cv_count);
    if knot.len() != kc {
        return Vec::new();
    }
    
    let d = order - 1;  // degree
    let count = if periodic { cv_count - order + 1 } else { cv_count };
    
    let mut g = vec![0.0; count];

    for i in 0..count {
        let sum: f64 = knot[i..(i + d)].iter().sum();
        g[i] = sum / d as f64;
    }

    g
}

/// Solve tridiagonal linear system using Thomas algorithm.
///
/// # Arguments
/// * `dim` - Dimension of each variable (e.g., 3 for 3D points).
/// * `lower` - Lower diagonal coefficients (first element unused).
/// * `diag` - Main diagonal coefficients.
/// * `upper` - Upper diagonal coefficients (last element unused).
/// * `rhs` - Right-hand side values (length n * dim).
///
/// # Returns
/// Solution vector (length n * dim), or None if singular.
pub fn solve_tridiagonal(dim: usize, lower: &[f64], diag: &[f64], upper: &[f64], rhs: &[f64]) -> Option<Vec<f64>> {
    let n = diag.len();
    if n < 1 || dim < 1 || lower.len() < n || upper.len() < n || rhs.len() < n * dim {
        return None;
    }

    const EPS: f64 = 1e-14;
    let mut c_star = vec![0.0; n];
    let mut d_star = vec![0.0; n * dim];
    let mut solution = vec![0.0; n * dim];

    if diag[0].abs() < EPS {
        return None;
    }

    c_star[0] = upper[0] / diag[0];
    for d in 0..dim {
        d_star[d] = rhs[d] / diag[0];
    }

    for i in 1..n {
        let denom = diag[i] - lower[i] * c_star[i - 1];
        if denom.abs() < EPS {
            return None;
        }

        c_star[i] = if i < n - 1 { upper[i] / denom } else { 0.0 };
        for d in 0..dim {
            d_star[i * dim + d] = (rhs[i * dim + d] - lower[i] * d_star[(i - 1) * dim + d]) / denom;
        }
    }

    for d in 0..dim {
        solution[(n - 1) * dim + d] = d_star[(n - 1) * dim + d];
    }

    for i in (0..n - 1).rev() {
        for d in 0..dim {
            solution[i * dim + d] = d_star[i * dim + d] - c_star[i] * solution[(i + 1) * dim + d];
        }
    }

    Some(solution)
}

/// Compute parameters for interpolation based on knot style.
///
/// # Arguments
/// * `points` - Input points (flat array: x0,y0,z0,x1,y1,z1,...).
/// * `dim` - Dimension (2 or 3).
/// * `style` - Knot style (Uniform, Chord, or ChordSquareRoot).
///
/// # Returns
/// Parameter values for each point.
pub fn compute_parameters(points: &[f64], dim: usize, style: CurveKnotStyle) -> Vec<f64> {
    let point_count = points.len() / dim;
    let mut params = vec![0.0; point_count];
    if point_count < 2 {
        return params;
    }

    let base_style = (style as u8) % 3;  // 0=Uniform, 1=Chord, 2=ChordSquareRoot

    for i in 1..point_count {
        let mut dist_sq = 0.0;
        for d in 0..dim {
            let diff = points[i * dim + d] - points[(i - 1) * dim + d];
            dist_sq += diff * diff;
        }
        let dist = dist_sq.sqrt();

        let delta = match base_style {
            0 => 1.0,           // Uniform
            1 => dist,          // Chord
            2 => dist.sqrt(),   // ChordSquareRoot (centripetal)
            _ => dist,
        };

        params[i] = params[i - 1] + delta;
    }

    params
}

/// Build clamped knot vector from parameters for interpolation.
///
/// # Arguments
/// * `params` - Parameter values for each input point.
/// * `degree` - Curve degree.
///
/// # Returns
/// Knot vector for interpolated curve.
pub fn build_interp_knots(params: &[f64], degree: usize) -> Vec<f64> {
    let n = params.len();
    if n < 2 || degree < 1 {
        return Vec::new();
    }

    let order = degree + 1;
    let cv_count = n + 2;  // Natural end conditions add 2 CVs
    let kc = knot_count(order, cv_count);

    let mut knots = vec![0.0; kc];
    let t_max = params[n - 1];

    // Clamped start: first (order-1) knots = 0
    for i in 0..(order - 1) {
        knots[i] = 0.0;
    }

    // Interior knots from parameters (skip first and last)
    for i in 1..(n - 1) {
        knots[order - 2 + i] = params[i];
    }

    // Clamped end: last (order-1) knots = t_max
    for i in 0..(order - 1) {
        knots[kc - 1 - i] = t_max;
    }

    knots
}
