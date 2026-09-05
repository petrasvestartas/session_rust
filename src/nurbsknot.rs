//! NurbsKnot vector utility functions for NURBS curves and surfaces.
//!
//! This module provides standalone functions for working with nurbsknot vectors,
//! following the OpenNURBS pattern (opennurbs_nurbsknot.h).
//!
//! These functions operate on slices and can be used independently
//! or called by NurbsCurve and NurbsSurface.

use serde::{Deserialize, Serialize};

/// NurbsKnot spacing style for interpolated curves (matches Rhino's CurveNurbsKnotStyle).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(u8)]
pub enum CurveNurbsKnotStyle {
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

/// End-tangent (boundary) condition for cubic interpolation.
///
/// Both styles share chord-length parameters and clamped knots; they differ only
/// in how the start/end tangents (and hence the 2nd/penultimate control points)
/// are estimated:
///  - `Rhino`: normalized Bessel tangents (matches Rhino / OpenNURBS).
///  - `Occt`:  un-normalized derivative of the cubic Lagrange polynomial through the
///    first/last 4 points (matches OCCT `GeomAPI_Interpolate::BuildTangents`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(u8)]
pub enum CurveInterpStyle {
    /// Bessel end tangents (matches Rhino / OpenNURBS).
    #[default]
    Rhino = 0,
    /// Cubic Lagrange end tangents (matches OCCT GeomAPI_Interpolate).
    Occt = 1,
}

/// Compute the number of nurbsknots in a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1), must be >= 2.
/// * `cv_count` - Number of control vertices, must be >= order.
///
/// # Returns
/// Number of nurbsknots: order + cv_count - 2
#[inline]
pub fn nurbsknot_count(order: usize, cv_count: usize) -> usize {
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
    if tol < EPSILON {
        EPSILON
    } else {
        tol
    }
}

/// Create a clamped uniform nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1), must be >= 2.
/// * `cv_count` - Number of control vertices, must be >= order.
/// * `delta` - Spacing between interior nurbsknots.
///
/// # Returns
/// Clamped uniform nurbsknot vector, or empty if invalid params.
pub fn make_clamped_uniform(order: usize, cv_count: usize, delta: f64) -> Vec<f64> {
    if order < 2 || cv_count < order || delta <= 0.0 {
        return Vec::new();
    }

    let kc = nurbsknot_count(order, cv_count);
    let mut nurbsknot = vec![0.0; kc];

    // Fill interior nurbsknots: from index (order-2) to (cv_count-1)
    let mut k = 0.0;
    for i in (order - 2)..cv_count {
        nurbsknot[i] = k;
        k += delta;
    }

    // Clamp both ends
    clamp(order, cv_count, &mut nurbsknot, 2);

    nurbsknot
}

/// Create a periodic uniform nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1), must be >= 2.
/// * `cv_count` - Number of control vertices, must be >= order.
/// * `delta` - Spacing between nurbsknots.
///
/// # Returns
/// Periodic uniform nurbsknot vector, or empty if invalid params.
pub fn make_periodic_uniform(order: usize, cv_count: usize, delta: f64) -> Vec<f64> {
    if order < 2 || cv_count < order || delta <= 0.0 {
        return Vec::new();
    }

    let kc = nurbsknot_count(order, cv_count);
    let mut nurbsknot = vec![0.0; kc];

    let mut k = 0.0;
    for i in 0..kc {
        nurbsknot[i] = k;
        k += delta;
    }

    nurbsknot
}

/// Clamp the ends of a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector to clamp (modified in place).
/// * `end` - Which end to clamp: 0 = left, 1 = right, 2 = both.
///
/// # Returns
/// True if successful.
pub fn clamp(order: usize, cv_count: usize, nurbsknot: &mut [f64], end: i32) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return false;
    }

    // Clamp left end
    if end == 0 || end == 2 {
        let clamp_value = nurbsknot[order - 2];
        for i in 0..(order - 2) {
            nurbsknot[i] = clamp_value;
        }
    }

    // Clamp right end
    if end == 1 || end == 2 {
        let clamp_value = nurbsknot[cv_count - 1];
        for i in cv_count..kc {
            nurbsknot[i] = clamp_value;
        }
    }

    true
}

/// Check if a nurbsknot vector is valid.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector to validate.
///
/// # Returns
/// True if the nurbsknot vector is valid.
pub fn is_valid(order: usize, cv_count: usize, nurbsknot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return false;
    }

    // Check non-decreasing
    for i in 1..kc {
        if nurbsknot[i] < nurbsknot[i - 1] {
            return false;
        }
    }

    // Check no degenerate spans (nurbsknot[i] < nurbsknot[i + order - 1])
    for i in 0..(kc - order + 1) {
        if nurbsknot[i] >= nurbsknot[i + order - 1] {
            return false;
        }
    }

    true
}

/// Check if a nurbsknot vector is clamped.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector to check.
/// * `end` - Which end to check: 0 = left, 1 = right, 2 = both.
///
/// # Returns
/// True if the nurbsknot vector is clamped at the specified end(s).
pub fn is_clamped(order: usize, cv_count: usize, nurbsknot: &[f64], end: i32) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return false;
    }

    let mult = order - 1;
    const TOL: f64 = 1e-10;

    // Check left end
    if end == 0 || end == 2 {
        if mult > kc {
            return false;
        }
        let start_value = nurbsknot[0];
        for i in 1..mult {
            if (nurbsknot[i] - start_value).abs() > TOL {
                return false;
            }
        }
    }

    // Check right end
    if end == 1 || end == 2 {
        if mult > kc {
            return false;
        }
        let end_value = nurbsknot[kc - 1];
        for i in 1..mult {
            if (nurbsknot[kc - 1 - i] - end_value).abs() > TOL {
                return false;
            }
        }
    }

    true
}

/// Check if a nurbsknot vector is periodic.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector to check.
///
/// # Returns
/// True if the nurbsknot vector is periodic.
pub fn is_periodic(order: usize, cv_count: usize, nurbsknot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc || kc < 2 {
        return false;
    }

    let delta = nurbsknot[1] - nurbsknot[0];
    if delta <= 0.0 {
        return false;
    }

    const TOL: f64 = 1e-10;
    for i in 2..kc {
        if ((nurbsknot[i] - nurbsknot[i - 1]) - delta).abs() > TOL {
            return false;
        }
    }

    true
}

/// Check if a nurbsknot vector is uniform (interior nurbsknots evenly spaced).
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector to check.
///
/// # Returns
/// True if the interior nurbsknots are uniformly spaced.
pub fn is_uniform(order: usize, cv_count: usize, nurbsknot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return false;
    }

    // Check interior nurbsknots (from order-2 to cv_count-1)
    if cv_count <= order {
        return true; // No interior nurbsknots
    }

    let start_idx = order - 2;
    let end_idx = cv_count - 1;

    if end_idx <= start_idx {
        return true;
    }

    let delta = nurbsknot[start_idx + 1] - nurbsknot[start_idx];
    if delta <= 0.0 {
        return false;
    }

    const TOL: f64 = 1e-10;
    for i in (start_idx + 2)..=end_idx {
        if ((nurbsknot[i] - nurbsknot[i - 1]) - delta).abs() > TOL {
            return false;
        }
    }

    true
}

/// Get the domain of a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
///
/// # Returns
/// Tuple (t0, t1) domain interval.
pub fn get_domain(order: usize, cv_count: usize, nurbsknot: &[f64]) -> (f64, f64) {
    if order < 2 || cv_count < order || nurbsknot.len() < nurbsknot_count(order, cv_count) {
        return (0.0, 0.0);
    }

    (nurbsknot[order - 2], nurbsknot[cv_count - 1])
}

/// Set the domain of a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector (modified in place).
/// * `t0` - New domain start.
/// * `t1` - New domain end.
///
/// # Returns
/// True if successful.
pub fn set_domain(order: usize, cv_count: usize, nurbsknot: &mut [f64], t0: f64, t1: f64) -> bool {
    if order < 2 || cv_count < order || t0 >= t1 {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return false;
    }

    let (old_t0, old_t1) = get_domain(order, cv_count, nurbsknot);
    if old_t1 <= old_t0 {
        return false;
    }

    let scale = (t1 - t0) / (old_t1 - old_t0);
    for i in 0..kc {
        nurbsknot[i] = t0 + (nurbsknot[i] - old_t0) * scale;
    }

    true
}

/// Reverse a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector (modified in place).
///
/// # Returns
/// True if successful.
pub fn reverse(order: usize, cv_count: usize, nurbsknot: &mut [f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return false;
    }

    // Reverse the array
    nurbsknot.reverse();

    // Negate and shift to maintain same domain direction
    let t0 = nurbsknot[0];
    let t1 = nurbsknot[kc - 1];
    for i in 0..kc {
        nurbsknot[i] = t0 + t1 - nurbsknot[i];
    }

    true
}

/// Get the multiplicity of a nurbsknot.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
/// * `nurbsknot_index` - Index of the nurbsknot to check.
///
/// # Returns
/// Multiplicity of the nurbsknot at the given index.
pub fn multiplicity(
    order: usize,
    cv_count: usize,
    nurbsknot: &[f64],
    nurbsknot_index: usize,
) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc || nurbsknot_index >= kc {
        return 0;
    }

    let nurbsknot_value = nurbsknot[nurbsknot_index];
    let mut mult = 1usize;
    const TOL: f64 = 1e-14;

    // Count preceding equal nurbsknots
    let mut i = nurbsknot_index;
    while i > 0 {
        i -= 1;
        if (nurbsknot[i] - nurbsknot_value).abs() < TOL {
            mult += 1;
        } else {
            break;
        }
    }

    // Count following equal nurbsknots
    let mut i = nurbsknot_index + 1;
    while i < kc {
        if (nurbsknot[i] - nurbsknot_value).abs() < TOL {
            mult += 1;
            i += 1;
        } else {
            break;
        }
    }

    mult
}

/// Get the number of spans in a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
///
/// # Returns
/// Number of non-empty spans.
pub fn span_count(order: usize, cv_count: usize, nurbsknot: &[f64]) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return 0;
    }

    let mut count = 0;
    let d = order - 1; // degree

    for i in 0..(cv_count - order + 1) {
        if nurbsknot[i + d - 1] < nurbsknot[i + d] {
            count += 1;
        }
    }

    count
}

/// Get the span breakpoints of a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
///
/// # Returns
/// Vector of unique nurbsknot values that define span boundaries.
pub fn get_span_vector(order: usize, cv_count: usize, nurbsknot: &[f64]) -> Vec<f64> {
    if order < 2 || cv_count < order {
        return Vec::new();
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return Vec::new();
    }

    let mut spans = Vec::new();
    const TOL: f64 = 1e-14;

    for i in 0..(kc - 1) {
        if (nurbsknot[i + 1] - nurbsknot[i]).abs() > TOL {
            spans.push(nurbsknot[i]);
        }
    }

    if kc > 0 {
        spans.push(nurbsknot[kc - 1]);
    }

    spans
}

/// Find the nurbsknot span containing parameter t.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
/// * `t` - Parameter value to locate.
///
/// # Returns
/// Span index in range [0, cv_count - order].
pub fn find_span(order: usize, cv_count: usize, nurbsknot: &[f64], t: f64) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return 0;
    }

    // Shift by (order - 2) as in OpenNURBS
    let nurbsknot_offset = order - 2;
    let span_len = cv_count - order + 2;

    // Handle boundary cases
    if t <= nurbsknot[nurbsknot_offset] {
        return 0;
    }
    if t >= nurbsknot[nurbsknot_offset + span_len - 1] {
        return span_len - 2;
    }

    // Binary search
    let mut low = 0;
    let mut high = span_len - 1;

    while high > low + 1 {
        let mid = (low + high) / 2;
        if t < nurbsknot[nurbsknot_offset + mid] {
            high = mid;
        } else {
            low = mid;
        }
    }

    low
}

/// Get the superfluous nurbsknot value at the specified end.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
/// * `end` - 0 = first superfluous nurbsknot, 1 = last superfluous nurbsknot.
///
/// # Returns
/// Superfluous nurbsknot value.
pub fn superfluous_nurbsknot(order: usize, cv_count: usize, nurbsknot: &[f64], end: i32) -> f64 {
    if order < 2 || cv_count < order {
        return 0.0;
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return 0.0;
    }

    if end == 0 {
        // First superfluous nurbsknot
        2.0 * nurbsknot[0] - nurbsknot[order - 2]
    } else {
        // Last superfluous nurbsknot
        2.0 * nurbsknot[kc - 1] - nurbsknot[cv_count - order]
    }
}

/// Compute a single Greville abscissa.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `nurbsknot` - Slice of (order - 1) nurbsknot values.
///
/// # Returns
/// Greville abscissa (average of the nurbsknots).
pub fn greville_abcissa(order: usize, nurbsknot: &[f64]) -> f64 {
    if order < 2 || nurbsknot.len() < order - 1 {
        return 0.0;
    }

    let d = order - 1; // degree
    let sum: f64 = nurbsknot[..d].iter().sum();
    sum / d as f64
}

/// Get all Greville abscissae for a nurbsknot vector.
///
/// # Arguments
/// * `order` - Order of the NURBS (degree + 1).
/// * `cv_count` - Number of control vertices.
/// * `nurbsknot` - NurbsKnot vector.
/// * `periodic` - True for periodic curves.
///
/// # Returns
/// Vector of Greville abscissae.
pub fn get_greville_abcissae(
    order: usize,
    cv_count: usize,
    nurbsknot: &[f64],
    periodic: bool,
) -> Vec<f64> {
    if order < 2 || cv_count < order {
        return Vec::new();
    }

    let kc = nurbsknot_count(order, cv_count);
    if nurbsknot.len() != kc {
        return Vec::new();
    }

    let d = order - 1; // degree
    let count = if periodic {
        cv_count - order + 1
    } else {
        cv_count
    };

    let mut g = vec![0.0; count];

    for i in 0..count {
        let sum: f64 = nurbsknot[i..(i + d)].iter().sum();
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
pub fn solve_tridiagonal(
    dim: usize,
    lower: &[f64],
    diag: &[f64],
    upper: &[f64],
    rhs: &[f64],
) -> Option<Vec<f64>> {
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

/// Compute parameters for interpolation based on nurbsknot style.
///
/// # Arguments
/// * `points` - Input points (flat array: x0,y0,z0,x1,y1,z1,...).
/// * `dim` - Dimension (2 or 3).
/// * `style` - NurbsKnot style (Uniform, Chord, or ChordSquareRoot).
///
/// # Returns
/// Parameter values for each point.
pub fn compute_parameters(points: &[f64], dim: usize, style: CurveNurbsKnotStyle) -> Vec<f64> {
    let point_count = points.len() / dim;
    let mut params = vec![0.0; point_count];
    if point_count < 2 {
        return params;
    }

    let base_style = (style as u8) % 3; // 0=Uniform, 1=Chord, 2=ChordSquareRoot

    for i in 1..point_count {
        let mut dist_sq = 0.0;
        for d in 0..dim {
            let diff = points[i * dim + d] - points[(i - 1) * dim + d];
            dist_sq += diff * diff;
        }
        let dist = dist_sq.sqrt();

        let delta = match base_style {
            0 => 1.0,         // Uniform
            1 => dist,        // Chord
            2 => dist.sqrt(), // ChordSquareRoot (centripetal)
            _ => dist,
        };

        params[i] = params[i - 1] + delta;
    }

    params
}

/// Build clamped nurbsknot vector from parameters for interpolation.
///
/// # Arguments
/// * `params` - Parameter values for each input point.
/// * `degree` - Curve degree.
///
/// # Returns
/// NurbsKnot vector for interpolated curve.
pub fn build_interp_nurbsknots(params: &[f64], degree: usize) -> Vec<f64> {
    let n = params.len();
    if n < 2 || degree < 1 {
        return Vec::new();
    }

    let order = degree + 1;
    let cv_count = n + 2; // Natural end conditions add 2 CVs
    let kc = nurbsknot_count(order, cv_count);

    let mut nurbsknots = vec![0.0; kc];
    let t_max = params[n - 1];

    // Clamped start: first (order-1) nurbsknots = 0
    for i in 0..(order - 1) {
        nurbsknots[i] = 0.0;
    }

    // Interior nurbsknots from parameters (skip first and last)
    for i in 1..(n - 1) {
        nurbsknots[order - 2 + i] = params[i];
    }

    // Clamped end: last (order-1) nurbsknots = t_max
    for i in 0..(order - 1) {
        nurbsknots[kc - 1 - i] = t_max;
    }

    nurbsknots
}

/// Evaluate B-spline basis functions at parameter t (Cox-de Boor).
///
/// # Arguments
/// * `order` - Order of the B-spline (degree + 1).
/// * `nurbsknot` - Full nurbsknot vector.
/// * `span` - Span index (from find_span).
/// * `t` - Parameter value.
///
/// # Returns
/// Vector of `order` basis function values.
pub fn eval_basis(order: usize, nurbsknot: &[f64], span: usize, t: f64) -> Vec<f64> {
    let mut basis = vec![0.0; order];
    let mut left = vec![0.0; order];
    let mut right = vec![0.0; order];

    let k_offset = order - 2 + span;
    basis[0] = 1.0;

    for j in 1..order {
        left[j] = t - nurbsknot[k_offset + 1 - j];
        right[j] = nurbsknot[k_offset + j] - t;
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

pub fn build_fitted_nurbsknots(params: &[f64], num_cvs: usize, degree: usize) -> Vec<f64> {
    let m = params.len();
    let n_interior = num_cvs - degree - 1;
    let order = degree + 1;
    let kc = nurbsknot_count(order, num_cvs);
    let mut nurbsknots = vec![0.0; kc];

    for i in 0..degree {
        nurbsknots[i] = params[0];
    }

    let d = m as f64 / (num_cvs - degree) as f64;
    for j in 1..=n_interior {
        let i = (j as f64 * d) as usize;
        let alpha = j as f64 * d - i as f64;
        nurbsknots[degree - 1 + j] = (1.0 - alpha) * params[i - 1] + alpha * params[i];
    }

    for i in (num_cvs - 1)..kc {
        nurbsknots[i] = params[m - 1];
    }

    nurbsknots
}

pub fn build_fitted_nurbsknots_adaptive(
    params: &[f64],
    points: &[f64],
    dim: usize,
    num_cvs: usize,
    degree: usize,
    scale: f64,
) -> Vec<f64> {
    let m = params.len();
    if m < 3 || points.is_empty() {
        return build_fitted_nurbsknots(params, num_cvs, degree);
    }

    let mut turn = vec![0.0; m];
    for i in 1..m - 1 {
        let (mut dot, mut len1sq, mut len2sq) = (0.0, 0.0, 0.0);
        for d in 0..dim {
            let a = points[i * dim + d] - points[(i - 1) * dim + d];
            let b = points[(i + 1) * dim + d] - points[i * dim + d];
            dot += a * b;
            len1sq += a * a;
            len2sq += b * b;
        }
        let (len1, len2) = (len1sq.sqrt(), len2sq.sqrt());
        if len1 > 1e-14 && len2 > 1e-14 {
            let c = (dot / (len1 * len2)).clamp(-1.0, 1.0);
            turn[i] = c.acos();
        }
    }

    let mut cum = vec![0.0; m];
    for i in 0..m - 1 {
        let mut chord = params[i + 1] - params[i];
        if chord < 1e-14 {
            chord = 1e-14;
        }
        cum[i + 1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[i + 1]) * 0.5);
    }
    let total = cum[m - 1];

    let n_interior = num_cvs - degree - 1;
    let order = degree + 1;
    let kc = nurbsknot_count(order, num_cvs);
    let mut nurbsknots = vec![0.0; kc];
    for i in 0..degree {
        nurbsknots[i] = params[0];
    }

    for j in 1..=n_interior {
        let target = total * j as f64 / (n_interior + 1) as f64;
        let (mut lo, mut hi) = (0usize, m - 2);
        while lo < hi {
            let mid = (lo + hi) / 2;
            if cum[mid + 1] < target {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        let frac = if cum[lo + 1] > cum[lo] {
            (target - cum[lo]) / (cum[lo + 1] - cum[lo])
        } else {
            0.0
        };
        nurbsknots[degree - 1 + j] = params[lo] + frac * (params[lo + 1] - params[lo]);
    }

    for i in (num_cvs - 1)..kc {
        nurbsknots[i] = params[m - 1];
    }
    nurbsknots
}

pub fn build_fitted_nurbsknots_periodic_adaptive(
    params: &[f64],
    points: &[f64],
    n: usize,
    dim: usize,
    num_cvs: usize,
    degree: usize,
    scale: f64,
) -> Vec<f64> {
    let cv_count = num_cvs + degree;
    let order = degree + 1;
    let kc = cv_count + order - 2;
    let big_t = params[n];

    if n < 3 || points.is_empty() {
        let delta = big_t / num_cvs as f64;
        return (0..kc)
            .map(|i| (i as f64 - degree as f64 + 1.0) * delta)
            .collect();
    }

    let mut turn = vec![0.0; n];
    for i in 0..n {
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        let (mut dot, mut len1sq, mut len2sq) = (0.0, 0.0, 0.0);
        for d in 0..dim {
            let a = points[i * dim + d] - points[prev * dim + d];
            let b = points[next * dim + d] - points[i * dim + d];
            dot += a * b;
            len1sq += a * a;
            len2sq += b * b;
        }
        let (len1, len2) = (len1sq.sqrt(), len2sq.sqrt());
        if len1 > 1e-14 && len2 > 1e-14 {
            let c = (dot / (len1 * len2)).clamp(-1.0, 1.0);
            turn[i] = c.acos();
        }
    }

    let mut cum = vec![0.0; n + 1];
    for i in 0..n {
        let mut chord = params[i + 1] - params[i];
        if chord < 1e-14 {
            chord = 1e-14;
        }
        let next = (i + 1) % n;
        cum[i + 1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[next]) * 0.5);
    }
    let total = cum[n];

    let mut base = vec![0.0; num_cvs];
    for j in 0..num_cvs {
        let target = total * j as f64 / num_cvs as f64;
        let (mut lo, mut hi) = (0usize, n - 1);
        while lo < hi {
            let mid = (lo + hi) / 2;
            if cum[mid + 1] < target {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        let frac = if cum[lo + 1] > cum[lo] {
            (target - cum[lo]) / (cum[lo + 1] - cum[lo])
        } else {
            0.0
        };
        base[j] = params[lo] + frac * (params[lo + 1] - params[lo]);
    }

    let mut intervals = vec![0.0; num_cvs];
    for j in 0..num_cvs - 1 {
        intervals[j] = base[j + 1] - base[j];
    }
    intervals[num_cvs - 1] = big_t - base[num_cvs - 1];

    let mut nurbsknots = vec![0.0; kc];
    nurbsknots[degree - 1] = 0.0;
    for i in 1..degree {
        nurbsknots[degree - 1 - i] = nurbsknots[degree - i] - intervals[num_cvs - i];
    }
    for i in 0..kc - degree {
        nurbsknots[degree + i] = nurbsknots[degree - 1 + i] + intervals[i % num_cvs];
    }

    nurbsknots
}

pub fn solve_banded_spd(
    dim: usize,
    n: usize,
    half_bw: usize,
    band: &mut [f64],
    rhs: &mut [f64],
) -> bool {
    let bw1 = half_bw + 1;

    for i in 0..n {
        let jmin = if i >= half_bw { i - half_bw } else { 0 };
        for j in jmin..=i {
            let mut sum = 0.0;
            let kmin = if i >= half_bw { i - half_bw } else { 0 };
            for k in kmin..j {
                sum += band[i * bw1 + (i - k)] * band[j * bw1 + (j - k)];
            }
            if i == j {
                let val = band[i * bw1] - sum;
                if val <= 1e-30 {
                    return false;
                }
                band[i * bw1] = val.sqrt();
            } else {
                band[i * bw1 + (i - j)] = (band[i * bw1 + (i - j)] - sum) / band[j * bw1];
            }
        }
    }

    for i in 0..n {
        for d in 0..dim {
            let mut sum = 0.0;
            let kmin = if i >= half_bw { i - half_bw } else { 0 };
            for k in kmin..i {
                sum += band[i * bw1 + (i - k)] * rhs[k * dim + d];
            }
            rhs[i * dim + d] = (rhs[i * dim + d] - sum) / band[i * bw1];
        }
    }

    for i in (0..n).rev() {
        for d in 0..dim {
            let mut sum = 0.0;
            let kmax = std::cmp::min(n, i + half_bw + 1);
            for k in (i + 1)..kmax {
                sum += band[k * bw1 + (k - i)] * rhs[k * dim + d];
            }
            rhs[i * dim + d] = (rhs[i * dim + d] - sum) / band[i * bw1];
        }
    }

    true
}
