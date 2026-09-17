use crate::tolerance::Tolerance;
use serde::{Deserialize, Serialize};

const KNOT_TOLERANCE: f64 = Tolerance::ABSOLUTE / 10.0;
const PIVOT_TOLERANCE: f64 = Tolerance::ZERO_TOLERANCE / 100.0;
const POSITIVE_DEFINITE_TOLERANCE: f64 =
    Tolerance::ABSOLUTE * Tolerance::ABSOLUTE * Tolerance::ZERO_TOLERANCE;

fn are_finite(values: &[f64], count: usize) -> bool {
    if values.len() < count {
        return false;
    }

    for value in values.iter().take(count) {
        if !value.is_finite() {
            return false;
        }
    }

    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Knot styles
// ═══════════════════════════════════════════════════════════════════════════

/// Parameter spacing for interpolated curves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(u8)]
pub enum CurveNurbsKnotStyle {
    Uniform = 0, // Equal parameter spacing.
    #[default]
    Chord = 1, // Spacing proportional to chord length.
    ChordSquareRoot = 2, // Spacing proportional to the square root of chord length.
    UniformPeriodic = 3, // Equal spacing for a periodic curve.
    ChordPeriodic = 4, // Chord-length spacing for a periodic curve.
    ChordSquareRootPeriodic = 5, // Square-root chord spacing for a periodic curve.
}

/// End-tangent estimate for cubic curve interpolation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(u8)]
pub enum CurveInterpStyle {
    #[default]
    Rhino = 0, // Bessel end tangents matching Rhino.
    Occt = 1, // Cubic Lagrange end tangents matching OCCT.
}

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════

/// Returns the number of nurbsknots, or zero for invalid or overflowing counts.
#[inline]
pub fn nurbsknot_count(order: usize, cv_count: usize) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }

    order.checked_add(cv_count - 2).unwrap_or(0)
}

/// Returns the floating-point tolerance associated with the domain interval `[a, b]`.
#[inline]
pub fn domain_tolerance(a: f64, b: f64) -> f64 {
    if a == b {
        return 0.0;
    }

    let epsilon = f64::EPSILON;
    let tol = (a.abs() + b.abs() + (a - b).abs()) * epsilon.sqrt();

    if tol < epsilon {
        epsilon
    } else {
        tol
    }
}

/// Returns a clamped uniform nurbsknot vector, or an empty vector for invalid arguments.
pub fn make_clamped_uniform(order: usize, cv_count: usize, delta: f64) -> Vec<f64> {
    if order < 2 || cv_count < order || !delta.is_finite() || delta <= 0.0 {
        return Vec::new();
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 {
        return Vec::new();
    }

    let mut nurbsknot = vec![0.0; kc];

    let mut k = 0.0;

    for value in nurbsknot.iter_mut().take(cv_count).skip(order - 2) {
        *value = k;
        k += delta;
    }

    clamp(order, cv_count, &mut nurbsknot, 2);
    nurbsknot
}

/// Returns a periodic uniform nurbsknot vector, or an empty vector for invalid arguments.
pub fn make_periodic_uniform(order: usize, cv_count: usize, delta: f64) -> Vec<f64> {
    if order < 2 || cv_count < order || !delta.is_finite() || delta <= 0.0 {
        return Vec::new();
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 {
        return Vec::new();
    }

    let mut nurbsknot = vec![0.0; kc];

    let mut k = 0.0;

    for value in &mut nurbsknot {
        *value = k;
        k += delta;
    }

    nurbsknot
}

/// Clamps selected ends in place, where `end` is 0 for left, 1 for right, or 2 for both.
pub fn clamp(order: usize, cv_count: usize, nurbsknot: &mut [f64], end: i32) -> bool {
    if order < 2 || cv_count < order || !(0..=2).contains(&end) {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return false;
    }

    if end == 0 || end == 2 {
        let clamp_value = nurbsknot[order - 2];
        nurbsknot[..(order - 2)].fill(clamp_value);
    }

    if end == 1 || end == 2 {
        let clamp_value = nurbsknot[cv_count - 1];
        nurbsknot[cv_count..kc].fill(clamp_value);
    }

    true
}

// ═══════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════

/// Returns whether the vector has the required length, finite values, and valid spans.
pub fn is_valid(order: usize, cv_count: usize, nurbsknot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return false;
    }

    for values in nurbsknot.windows(2) {
        if values[1] < values[0] {
            return false;
        }
    }

    for i in 0..(kc - order + 1) {
        if nurbsknot[i] >= nurbsknot[i + order - 1] {
            return false;
        }
    }

    true
}

/// Returns whether selected ends contain `order - 1` equal nurbsknots.
pub fn is_clamped(order: usize, cv_count: usize, nurbsknot: &[f64], end: i32) -> bool {
    if order < 2 || cv_count < order || !(0..=2).contains(&end) {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return false;
    }

    let mult = order - 1;
    let tol = KNOT_TOLERANCE;

    if end == 0 || end == 2 {
        if mult > kc {
            return false;
        }

        let start_value = nurbsknot[0];

        for value in nurbsknot.iter().take(mult).skip(1) {
            if (*value - start_value).abs() > tol {
                return false;
            }
        }
    }

    if end == 1 || end == 2 {
        if mult > kc {
            return false;
        }

        let end_value = nurbsknot[kc - 1];

        for value in nurbsknot.iter().rev().take(mult).skip(1) {
            if (*value - end_value).abs() > tol {
                return false;
            }
        }
    }

    true
}

/// Returns whether the nurbsknot vector has finite, positive, uniform spacing.
pub fn is_periodic(order: usize, cv_count: usize, nurbsknot: &[f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc < 2 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return false;
    }

    let delta = nurbsknot[1] - nurbsknot[0];

    if delta <= 0.0 {
        return false;
    }

    let tol = KNOT_TOLERANCE;

    for values in nurbsknot.windows(2).skip(1) {
        if ((values[1] - values[0]) - delta).abs() > tol {
            return false;
        }
    }

    true
}

/// Returns the finite domain endpoints, or `(0.0, 0.0)` when they cannot be read.
pub fn get_domain(order: usize, cv_count: usize, nurbsknot: &[f64]) -> (f64, f64) {
    if order < 2 || cv_count < order {
        return (0.0, 0.0);
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() < kc {
        return (0.0, 0.0);
    }

    let start = nurbsknot[order - 2];
    let end = nurbsknot[cv_count - 1];

    if !start.is_finite() || !end.is_finite() {
        return (0.0, 0.0);
    }

    (start, end)
}

/// Rescales the nurbsknot vector in place to the finite domain `[t0, t1]`.
pub fn set_domain(order: usize, cv_count: usize, nurbsknot: &mut [f64], t0: f64, t1: f64) -> bool {
    if order < 2 || cv_count < order || !t0.is_finite() || !t1.is_finite() || t0 >= t1 {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return false;
    }

    let (old_t0, old_t1) = get_domain(order, cv_count, nurbsknot);

    if old_t1 <= old_t0 {
        return false;
    }

    let scale = (t1 - t0) / (old_t1 - old_t0);

    for value in nurbsknot {
        *value = t0 + (*value - old_t0) * scale;
    }

    true
}

/// Reverses a finite nurbsknot vector in place while preserving its domain.
pub fn reverse(order: usize, cv_count: usize, nurbsknot: &mut [f64]) -> bool {
    if order < 2 || cv_count < order {
        return false;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return false;
    }

    nurbsknot.reverse();

    let t0 = nurbsknot[0];
    let t1 = nurbsknot[kc - 1];

    for value in nurbsknot {
        *value = t0 + t1 - *value;
    }

    true
}

/// Returns the multiplicity at `nurbsknot_index`, or zero for invalid arguments.
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

    if kc == 0 || nurbsknot.len() != kc || nurbsknot_index >= kc || !are_finite(nurbsknot, kc) {
        return 0;
    }

    let nurbsknot_value = nurbsknot[nurbsknot_index];
    let tol = PIVOT_TOLERANCE;
    let mut mult = 1;

    let mut i = nurbsknot_index;

    while i > 0 && (nurbsknot[i - 1] - nurbsknot_value).abs() < tol {
        mult += 1;
        i -= 1;
    }

    i = nurbsknot_index + 1;

    while i < kc && (nurbsknot[i] - nurbsknot_value).abs() < tol {
        mult += 1;
        i += 1;
    }

    mult
}

/// Returns the number of non-empty spans, or zero for invalid arguments.
pub fn span_count(order: usize, cv_count: usize, nurbsknot: &[f64]) -> usize {
    if order < 2 || cv_count < order {
        return 0;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return 0;
    }

    let d = order - 1;
    let mut count = 0;

    for i in 0..(cv_count - order + 1) {
        if nurbsknot[i + d - 1] < nurbsknot[i + d] {
            count += 1;
        }
    }

    count
}

/// Returns the index of the span containing finite parameter `t` in a valid nondecreasing nurbsknot vector.
pub fn find_span(
    order: usize,
    cv_count: usize,
    nurbsknot: &[f64],
    t: f64,
    side: i32,
    hint: i32,
) -> usize {
    let _ = (side, hint);

    if order < 2 || cv_count < order || !t.is_finite() {
        return 0;
    }

    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 || nurbsknot.len() != kc {
        return 0;
    }

    let nurbsknot_offset = order - 2;
    let span_len = cv_count - order + 2;
    let start = nurbsknot[nurbsknot_offset];
    let end = nurbsknot[nurbsknot_offset + span_len - 1];

    if !start.is_finite() || !end.is_finite() {
        return 0;
    }

    if t <= start {
        return 0;
    }

    if t >= end {
        return span_len - 2;
    }

    let mut low = 0;
    let mut high = span_len - 1;

    while high > low + 1 {
        let mid = low + (high - low) / 2;
        let mid_value = nurbsknot[nurbsknot_offset + mid];

        if !mid_value.is_finite() {
            return 0;
        }

        if t < mid_value {
            high = mid;
        } else {
            low = mid;
        }
    }

    low
}

/// Returns the Greville abscissae, or an empty vector for invalid arguments.
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

    if kc == 0 || nurbsknot.len() != kc || !are_finite(nurbsknot, kc) {
        return Vec::new();
    }

    let d = order - 1;
    let count = if periodic {
        cv_count - order + 1
    } else {
        cv_count
    };
    let mut g = vec![0.0; count];

    for i in 0..count {
        let mut sum = 0.0;

        for j in 0..d {
            sum += nurbsknot[i + j];
        }

        g[i] = sum / d as f64;
    }

    g
}

// ═══════════════════════════════════════════════════════════════════════════
// Interpolation
// ═══════════════════════════════════════════════════════════════════════════

/// Solves a finite tridiagonal system, returning `None` if invalid or singular.
pub fn solve_tridiagonal(
    dim: usize,
    n: usize,
    lower: &[f64],
    diag: &[f64],
    upper: &[f64],
    rhs: &[f64],
) -> Option<Vec<f64>> {
    if n < 1 || dim < 1 {
        return None;
    }

    let rhs_count = n.checked_mul(dim)?;

    if lower.len() < n || diag.len() < n || upper.len() < n || rhs.len() < rhs_count {
        return None;
    }

    if !are_finite(lower, n)
        || !are_finite(diag, n)
        || !are_finite(upper, n)
        || !are_finite(rhs, rhs_count)
    {
        return None;
    }

    let eps = PIVOT_TOLERANCE;
    let mut c_star = vec![0.0; n];
    let mut d_star = vec![0.0; rhs_count];
    let mut solution = vec![0.0; rhs_count];

    if diag[0].abs() < eps {
        return None;
    }

    c_star[0] = upper[0] / diag[0];

    for d in 0..dim {
        d_star[d] = rhs[d] / diag[0];
    }

    for i in 1..n {
        let denom = diag[i] - lower[i] * c_star[i - 1];

        if denom.abs() < eps {
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

/// Returns one parameter per point from a flat `point_count` by `dim` coordinate array.
pub fn compute_parameters(
    points: &[f64],
    point_count: usize,
    dim: usize,
    style: CurveNurbsKnotStyle,
) -> Vec<f64> {
    let Some(value_count) = point_count.checked_mul(dim) else {
        return Vec::new();
    };

    if point_count < 1 || dim < 1 || !are_finite(points, value_count) {
        return Vec::new();
    }

    let mut params = vec![0.0; point_count];

    if point_count < 2 {
        return params;
    }

    let base_style = (style as u8) % 3;

    for i in 1..point_count {
        let mut dist = 0.0;

        for d in 0..dim {
            let diff = points[i * dim + d] - points[(i - 1) * dim + d];
            dist += diff * diff;
        }

        dist = dist.sqrt();

        let mut delta = dist;

        if base_style == 0 {
            delta = 1.0;
        } else if base_style == 2 {
            delta = dist.sqrt();
        }

        params[i] = params[i - 1] + delta;
    }

    params
}

/// Returns a clamped interpolation nurbsknot vector with natural end conditions.
pub fn build_interp_nurbsknots(params: &[f64], degree: usize) -> Vec<f64> {
    let n = params.len();

    if n < 2 || degree < 1 || !are_finite(params, n) {
        return Vec::new();
    }

    let Some(order) = degree.checked_add(1) else {
        return Vec::new();
    };
    let Some(cv_count) = n.checked_add(2) else {
        return Vec::new();
    };
    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 {
        return Vec::new();
    }

    let t_max = params[n - 1];
    let mut nurbsknots = vec![0.0; kc];

    for i in 1..(n - 1) {
        nurbsknots[order - 2 + i] = params[i];
    }

    for i in 0..(order - 1) {
        nurbsknots[kc - 1 - i] = t_max;
    }

    nurbsknots
}

/// Returns the `order` nonzero B-spline basis values at `t` by Cox-de Boor evaluation over a finite span window.
pub fn eval_basis(order: usize, nurbsknot: &[f64], span: usize, t: f64) -> Vec<f64> {
    if order < 1 || !t.is_finite() {
        return Vec::new();
    }

    if order == 1 {
        return vec![1.0];
    }

    let Some(double_order) = order.checked_mul(2) else {
        return Vec::new();
    };
    let width = double_order - 2;
    let Some(end) = span.checked_add(width) else {
        return Vec::new();
    };

    if nurbsknot.len() < end {
        return Vec::new();
    }

    for value in &nurbsknot[span..end] {
        if !value.is_finite() {
            return Vec::new();
        }
    }

    let mut basis = vec![0.0; order];
    let mut left = vec![0.0; order];
    let mut right = vec![0.0; order];

    let Some(k_offset) = span.checked_add(order - 2) else {
        return Vec::new();
    };
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

// ═══════════════════════════════════════════════════════════════════════════
// Fitting
// ═══════════════════════════════════════════════════════════════════════════

fn build_fitted_nurbsknots(params: &[f64], num_cvs: usize, degree: usize) -> Vec<f64> {
    let m = params.len();
    let n_interior = num_cvs - degree - 1;
    let order = degree + 1;
    let kc = nurbsknot_count(order, num_cvs);

    if kc == 0 {
        return Vec::new();
    }

    let mut nurbsknots = vec![0.0; kc];

    nurbsknots[..degree].fill(params[0]);

    let d = m as f64 / (num_cvs - degree) as f64;

    for j in 1..=n_interior {
        let i = (j as f64 * d) as usize;
        let alpha = j as f64 * d - i as f64;
        nurbsknots[degree - 1 + j] = (1.0 - alpha) * params[i - 1] + alpha * params[i];
    }

    nurbsknots[(num_cvs - 1)..kc].fill(params[m - 1]);

    nurbsknots
}

fn turn_angle(points: &[f64], dim: usize, prev: usize, i: usize, next: usize) -> f64 {
    let mut dot = 0.0;
    let mut len1sq = 0.0;
    let mut len2sq = 0.0;

    for d in 0..dim {
        let a = points[i * dim + d] - points[prev * dim + d];
        let b = points[next * dim + d] - points[i * dim + d];
        dot += a * b;
        len1sq += a * a;
        len2sq += b * b;
    }

    let len1 = len1sq.sqrt();
    let len2 = len2sq.sqrt();

    if len1 <= PIVOT_TOLERANCE || len2 <= PIVOT_TOLERANCE {
        return 0.0;
    }

    (dot / (len1 * len2)).clamp(-1.0, 1.0).acos()
}

fn locate_target(params: &[f64], cum: &[f64], last: usize, target: f64) -> f64 {
    let mut lo = 0;
    let mut hi = last;

    while lo < hi {
        let mid = lo + (hi - lo) / 2;

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
    params[lo] + frac * (params[lo + 1] - params[lo])
}

/// Returns a clamped fitting vector with denser nurbsknots where the points turn.
pub fn build_fitted_nurbsknots_adaptive(
    params: &[f64],
    points: &[f64],
    point_count: usize,
    dim: usize,
    num_cvs: usize,
    degree: usize,
    scale: f64,
) -> Vec<f64> {
    let m = point_count;

    if m < 2
        || dim < 1
        || num_cvs <= degree
        || degree < 1
        || !scale.is_finite()
        || !are_finite(params, m)
    {
        return Vec::new();
    }

    if m < 3 || points.is_empty() {
        if m < num_cvs - degree {
            return Vec::new();
        }

        return build_fitted_nurbsknots(params, num_cvs, degree);
    }

    let Some(value_count) = m.checked_mul(dim) else {
        return Vec::new();
    };

    if !are_finite(points, value_count) {
        return Vec::new();
    }

    let mut turn = vec![0.0; m];

    for (i, value) in turn.iter_mut().enumerate().take(m - 1).skip(1) {
        *value = turn_angle(points, dim, i - 1, i, i + 1);
    }

    let mut cum = vec![0.0; m];

    for i in 0..(m - 1) {
        let chord = (params[i + 1] - params[i]).max(PIVOT_TOLERANCE);
        cum[i + 1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[i + 1]) * 0.5);
    }

    let total = cum[m - 1];

    let n_interior = num_cvs - degree - 1;
    let order = degree + 1;
    let kc = nurbsknot_count(order, num_cvs);

    if kc == 0 {
        return Vec::new();
    }

    let mut nurbsknots = vec![0.0; kc];
    nurbsknots[..degree].fill(params[0]);

    for j in 1..=n_interior {
        nurbsknots[degree - 1 + j] = locate_target(
            params,
            &cum,
            m - 2,
            total * j as f64 / (n_interior + 1) as f64,
        );
    }

    nurbsknots[(num_cvs - 1)..kc].fill(params[m - 1]);

    nurbsknots
}

/// Returns a periodic fitting vector with denser nurbsknots where the closed points turn.
pub fn build_fitted_nurbsknots_periodic_adaptive(
    params: &[f64],
    points: &[f64],
    n: usize,
    dim: usize,
    num_cvs: usize,
    degree: usize,
    scale: f64,
) -> Vec<f64> {
    let Some(param_count) = n.checked_add(1) else {
        return Vec::new();
    };

    if degree < 1
        || degree - 1 > num_cvs
        || !scale.is_finite()
        || params.len() < param_count
        || !are_finite(params, param_count)
    {
        return Vec::new();
    }

    let Some(cv_count) = num_cvs.checked_add(degree) else {
        return Vec::new();
    };
    let Some(order) = degree.checked_add(1) else {
        return Vec::new();
    };
    let kc = nurbsknot_count(order, cv_count);

    if kc == 0 {
        return Vec::new();
    }

    let period = params[n];
    let mut nurbsknots = vec![0.0; kc];

    if !period.is_finite() || period <= 0.0 {
        return Vec::new();
    }

    if n < 3 || points.is_empty() {
        let delta = period / num_cvs as f64;

        for (i, value) in nurbsknots.iter_mut().enumerate() {
            *value = (i as f64 - degree as f64 + 1.0) * delta;
        }

        return nurbsknots;
    }

    let Some(value_count) = n.checked_mul(dim) else {
        return Vec::new();
    };

    if dim < 1 || !are_finite(points, value_count) {
        return Vec::new();
    }

    let mut turn = vec![0.0; n];

    for (i, value) in turn.iter_mut().enumerate() {
        *value = turn_angle(
            points,
            dim,
            if i == 0 { n - 1 } else { i - 1 },
            i,
            (i + 1) % n,
        );
    }

    let mut cum = vec![0.0; n + 1];

    for i in 0..n {
        let chord = (params[i + 1] - params[i]).max(PIVOT_TOLERANCE);
        cum[i + 1] = cum[i] + chord * (1.0 + scale * (turn[i] + turn[(i + 1) % n]) * 0.5);
    }

    let total = cum[n];

    let mut base = vec![0.0; num_cvs];

    for (j, value) in base.iter_mut().enumerate() {
        *value = locate_target(params, &cum, n - 1, total * j as f64 / num_cvs as f64);
    }

    let mut intervals = vec![0.0; num_cvs];

    for j in 0..(num_cvs - 1) {
        intervals[j] = base[j + 1] - base[j];
    }

    intervals[num_cvs - 1] = period - base[num_cvs - 1];

    for i in 1..degree {
        nurbsknots[degree - 1 - i] = nurbsknots[degree - i] - intervals[num_cvs - i];
    }

    for i in 0..(kc - degree) {
        nurbsknots[degree + i] = nurbsknots[degree - 1 + i] + intervals[i % num_cvs];
    }

    nurbsknots
}

/// Solves a finite banded SPD system in place, returning false for invalid or indefinite input.
pub fn solve_banded_spd(
    dim: usize,
    n: usize,
    half_bw: usize,
    band: &mut [f64],
    rhs: &mut [f64],
) -> bool {
    let Some(bw1) = half_bw.checked_add(1) else {
        return false;
    };
    let Some(band_count) = n.checked_mul(bw1) else {
        return false;
    };
    let Some(rhs_count) = n.checked_mul(dim) else {
        return false;
    };

    if dim < 1 || n < 1 || !are_finite(band, band_count) || !are_finite(rhs, rhs_count) {
        return false;
    }

    for i in 0..n {
        for j in i.saturating_sub(half_bw)..=i {
            let mut sum = 0.0;

            for k in i.saturating_sub(half_bw)..j {
                sum += band[i * bw1 + (i - k)] * band[j * bw1 + (j - k)];
            }

            if i == j {
                let val = band[i * bw1] - sum;

                if val <= POSITIVE_DEFINITE_TOLERANCE {
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

            for k in i.saturating_sub(half_bw)..i {
                sum += band[i * bw1 + (i - k)] * rhs[k * dim + d];
            }

            rhs[i * dim + d] = (rhs[i * dim + d] - sum) / band[i * bw1];
        }
    }

    for i in (0..n).rev() {
        for d in 0..dim {
            let mut sum = 0.0;
            let upper = i.saturating_add(bw1).min(n);

            for k in (i + 1)..upper {
                sum += band[k * bw1 + (k - i)] * rhs[k * dim + d];
            }

            rhs[i * dim + d] = (rhs[i * dim + d] - sum) / band[i * bw1];
        }
    }

    true
}
