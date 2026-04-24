//! Tests for nurbsknot module.

use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_make_clamped_uniform() -> TestResult {
    MINI_TEST!("Make Clamped Uniform", {
        use crate::nurbsknot;

        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count, 1.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));
    })
}

pub fn run_make_periodic_uniform() -> TestResult {
    MINI_TEST!("Make Periodic Uniform", {
        use crate::nurbsknot;

        // 0 1 2 3 4 5 6
        let order = 4;
        let cv_count = 5;
        let nurbsknots = nurbsknot::make_periodic_uniform(order, cv_count, 1.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
    })
}

pub fn run_is_clamped() -> TestResult {
    MINI_TEST!("Is Clamped", {
        use crate::nurbsknot;

        // 0 0 0 1 2 2 2
        // 0 1 2 3 4 5 6
        let order = 4;
        let cv_count = 5;
        let nurbsknots_periodic = nurbsknot::make_periodic_uniform(order, cv_count, 1.0);
        let nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count, 1.0);
        let is_not_clamped = nurbsknot::is_clamped(order, cv_count, &nurbsknots_periodic, 2);
        let is_clamped = nurbsknot::is_clamped(order, cv_count, &nurbsknots_clamped, 2);
        MINI_CHECK!(!is_not_clamped && is_clamped);
    })
}

pub fn run_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::nurbsknot;

        // Symmetric nurbsknot vector -> reverse gives back the same (palindrome)
        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let mut nurbsknots_sym = nurbsknot::make_clamped_uniform(order, cv_count, 1.0);
        nurbsknot::reverse(order, cv_count, &mut nurbsknots_sym);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots_sym, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));

        // Asymmetric nurbsknot vector -> extra nurbsknot at 0.5 shifts to 1.5 after reverse
        // 0 0 0 0.5 1 2 2 2 -> 0 0 0 1 1.5 2 2 2
        let mut nurbsknots_asym = vec![0.0, 0.0, 0.0, 0.5, 1.0, 2.0, 2.0, 2.0];
        nurbsknot::reverse(4, 6, &mut nurbsknots_asym);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots_asym, &[0.0, 0.0, 0.0, 1.0, 1.5, 2.0, 2.0, 2.0]));
    })
}

pub fn run_find_span() -> TestResult {
    MINI_TEST!("Find Span", {
        use crate::nurbsknot;

        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let nurbsknots_clamped = nurbsknot::make_clamped_uniform(order, cv_count, 1.0);
        //   - 0.5 falls in span [0, 1] -> index 0
        //   - 1.5 falls in span [1, 2] -> index 1
        let spancount0 = nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, 0.5);
        let spancount1 = nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, 1.5);
        MINI_CHECK!(spancount0 == 0 && spancount1 == 1);
    })
}

pub fn run_solve_tridiagonal() -> TestResult {
    MINI_TEST!("Solve Tridiagonal", {
        use crate::nurbsknot;

        // Thomas algorithm -- an O(n) solver for tridiagonal linear systems
        //   | 2 1 | |x0|   |3|
        //   | 1 2 | |x1| = |3|
        //   -> solution: x0 = 1, x1 = 1
        let lo = [0.0, 1.0];
        let di = [2.0, 2.0];
        let up = [1.0, 0.0];
        let rh = [3.0, 3.0];
        let sol = nurbsknot::solve_tridiagonal(1, &lo, &di, &up, &rh).unwrap();
        MINI_CHECK!(TOLERANCE.is_allclose(&sol, &[1.0, 1.0]));
    })
}

pub fn run_compute_parameters() -> TestResult {
    MINI_TEST!("Compute Parameters", {
        use crate::nurbsknot;

        let pts = [0.0,0.0,0.0, 1.0,0.0,0.0, 2.0,0.0,0.0, 3.0,0.0,0.0];
        // Chord-length parameterization: since all gaps are 1.0, params = {0, 1, 2, 3}
        let t = nurbsknot::compute_parameters(&pts, 3, nurbsknot::CurveNurbsKnotStyle::Chord);
        MINI_CHECK!(TOLERANCE.is_allclose(&t, &[0.0, 1.0, 2.0, 3.0]));
    })
}

pub fn run_build_interp_nurbsknots() -> TestResult {
    MINI_TEST!("Build Interpolation NurbsKnots", {
        use crate::nurbsknot;

        let params = [0.0, 1.0, 2.0, 3.0];
        let degree = 3;
        // cv_count = n + 2 = 6 (natural end conditions add 2 CVs)
        // kc = order + cv_count - 2 = 4 + 6 - 2 = 8
        //   [0, 0, 0,  |  1, 2,  |  3, 3, 3]
        //   <-clamp->    interior    <-clamp->
        let nurbsknots = nurbsknot::build_interp_nurbsknots(&params, degree);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0]));
    })
}

pub fn run_eval_basis() -> TestResult {
    MINI_TEST!("Evaluation Basis", {
        use crate::nurbsknot;

        // Cox-de Boor recursive evaluation of B-spline basis functions
        // At parameter t, exactly 'order' basis functions are non-zero
        // Partition of unity: they always sum to 1.0
        // Used to evaluate NURBS curves/surfaces: C(t) = sum(N_i(t) * P_i)
        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let nurbsknots = nurbsknot::make_clamped_uniform(order, cv_count, 1.0);
        let span = nurbsknot::find_span(order, cv_count, &nurbsknots, 0.5);
        let basis = nurbsknot::eval_basis(order, &nurbsknots, span, 0.5);
        MINI_CHECK!(TOLERANCE.is_allclose(&basis, &[0.125, 0.59375, 0.25, 0.03125]));
    })
}

pub fn run_build_fitted_nurbsknots_adaptive() -> TestResult {
    MINI_TEST!("Build Fitted NurbsKnots Adaptive", {
        use crate::nurbsknot;

        // Builds nurbsknot vectors for least-squares fitting
        // Concentrates nurbsknots where curvature is high (sharp turns)
        // For collinear points (zero curvature), interior nurbsknots are evenly distributed
        let pts = [0.0,0.0,0.0, 1.0,0.0,0.0, 2.0,0.0,0.0, 3.0,0.0,0.0, 4.0,0.0,0.0];
        let params = [0.0, 1.0, 2.0, 3.0, 4.0];
        let nurbsknots = nurbsknot::build_fitted_nurbsknots_adaptive(&params, &pts, 3, 5, 3, 3.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 2.0, 4.0, 4.0, 4.0]));
    })
}

pub fn run_build_fitted_nurbsknots_periodic_adaptive() -> TestResult {
    MINI_TEST!("Build Fitted NurbsKnots Periodic Adaptive", {
        use crate::nurbsknot;

        // Periodic version for closed curves -- nurbsknots wrap around
        // For a regular square (equal turns, equal chords), nurbsknots are uniformly spaced
        let pts = [0.0,0.0,0.0, 1.0,0.0,0.0, 1.0,1.0,0.0, 0.0,1.0,0.0];
        let params = [0.0, 1.0, 2.0, 3.0, 4.0];
        let nurbsknots = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(&params, &pts, 4, 3, 4, 3, 3.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
    })
}

pub fn run_solve_banded_spd() -> TestResult {
    MINI_TEST!("Solve Banded SPD", {
        use crate::nurbsknot;

        // Cholesky solver for banded symmetric positive-definite systems
        //   | 4 2 0 |       |8 |       |1|
        //   | 2 5 1 | * x = |13| -> x = |2|
        //   | 0 1 3 |       |5 |       |1|
        let mut band = vec![4.0, 0.0, 5.0, 2.0, 3.0, 1.0];
        let mut rhs = vec![8.0, 13.0, 5.0];
        nurbsknot::solve_banded_spd(1, 3, 1, &mut band, &mut rhs);
        MINI_CHECK!(TOLERANCE.is_allclose(&rhs, &[1.0, 2.0, 1.0]));
    })
}

// Register all tests
REGISTER_MINI_TEST!("NurbsKnot", "Make Clamped Uniform", crate::nurbsknot_test::run_make_clamped_uniform);
REGISTER_MINI_TEST!("NurbsKnot", "Make Periodic Uniform", crate::nurbsknot_test::run_make_periodic_uniform);
REGISTER_MINI_TEST!("NurbsKnot", "Is Clamped", crate::nurbsknot_test::run_is_clamped);
REGISTER_MINI_TEST!("NurbsKnot", "Reverse", crate::nurbsknot_test::run_reverse);
REGISTER_MINI_TEST!("NurbsKnot", "Find Span", crate::nurbsknot_test::run_find_span);
REGISTER_MINI_TEST!("NurbsKnot", "Solve Tridiagonal", crate::nurbsknot_test::run_solve_tridiagonal);
REGISTER_MINI_TEST!("NurbsKnot", "Compute Parameters", crate::nurbsknot_test::run_compute_parameters);
REGISTER_MINI_TEST!("NurbsKnot", "Build Interpolation NurbsKnots", crate::nurbsknot_test::run_build_interp_nurbsknots);
REGISTER_MINI_TEST!("NurbsKnot", "Evaluation Basis", crate::nurbsknot_test::run_eval_basis);
REGISTER_MINI_TEST!("NurbsKnot", "Build Fitted NurbsKnots Adaptive", crate::nurbsknot_test::run_build_fitted_nurbsknots_adaptive);
REGISTER_MINI_TEST!("NurbsKnot", "Build Fitted NurbsKnots Periodic Adaptive", crate::nurbsknot_test::run_build_fitted_nurbsknots_periodic_adaptive);
REGISTER_MINI_TEST!("NurbsKnot", "Solve Banded SPD", crate::nurbsknot_test::run_solve_banded_spd);
