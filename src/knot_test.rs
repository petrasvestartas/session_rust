//! Tests for knot module.

use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_make_clamped_uniform() -> TestResult {
    MINI_TEST!("Make Clamped Uniform", {
        use crate::knot;

        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let knots = knot::make_clamped_uniform(order, cv_count, 1.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));
    })
}

pub fn run_make_periodic_uniform() -> TestResult {
    MINI_TEST!("Make Periodic Uniform", {
        use crate::knot;

        // 0 1 2 3 4 5 6
        let order = 4;
        let cv_count = 5;
        let knots = knot::make_periodic_uniform(order, cv_count, 1.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots, &[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
    })
}

pub fn run_is_clamped() -> TestResult {
    MINI_TEST!("Is Clamped", {
        use crate::knot;

        // 0 0 0 1 2 2 2
        // 0 1 2 3 4 5 6
        let order = 4;
        let cv_count = 5;
        let knots_periodic = knot::make_periodic_uniform(order, cv_count, 1.0);
        let knots_clamped = knot::make_clamped_uniform(order, cv_count, 1.0);
        let is_not_clamped = knot::is_clamped(order, cv_count, &knots_periodic, 2);
        let is_clamped = knot::is_clamped(order, cv_count, &knots_clamped, 2);
        MINI_CHECK!(!is_not_clamped && is_clamped);
    })
}

pub fn run_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::knot;

        // Symmetric knot vector -> reverse gives back the same (palindrome)
        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let mut knots_sym = knot::make_clamped_uniform(order, cv_count, 1.0);
        knot::reverse(order, cv_count, &mut knots_sym);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots_sym, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));

        // Asymmetric knot vector -> extra knot at 0.5 shifts to 1.5 after reverse
        // 0 0 0 0.5 1 2 2 2 -> 0 0 0 1 1.5 2 2 2
        let mut knots_asym = vec![0.0, 0.0, 0.0, 0.5, 1.0, 2.0, 2.0, 2.0];
        knot::reverse(4, 6, &mut knots_asym);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots_asym, &[0.0, 0.0, 0.0, 1.0, 1.5, 2.0, 2.0, 2.0]));
    })
}

pub fn run_find_span() -> TestResult {
    MINI_TEST!("Find Span", {
        use crate::knot;

        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let knots_clamped = knot::make_clamped_uniform(order, cv_count, 1.0);
        //   - 0.5 falls in span [0, 1] -> index 0
        //   - 1.5 falls in span [1, 2] -> index 1
        let spancount0 = knot::find_span(order, cv_count, &knots_clamped, 0.5);
        let spancount1 = knot::find_span(order, cv_count, &knots_clamped, 1.5);
        MINI_CHECK!(spancount0 == 0 && spancount1 == 1);
    })
}

pub fn run_solve_tridiagonal() -> TestResult {
    MINI_TEST!("Solve Tridiagonal", {
        use crate::knot;

        // Thomas algorithm -- an O(n) solver for tridiagonal linear systems
        //   | 2 1 | |x0|   |3|
        //   | 1 2 | |x1| = |3|
        //   -> solution: x0 = 1, x1 = 1
        let lo = [0.0, 1.0];
        let di = [2.0, 2.0];
        let up = [1.0, 0.0];
        let rh = [3.0, 3.0];
        let sol = knot::solve_tridiagonal(1, &lo, &di, &up, &rh).unwrap();
        MINI_CHECK!(TOLERANCE.is_allclose(&sol, &[1.0, 1.0]));
    })
}

pub fn run_compute_parameters() -> TestResult {
    MINI_TEST!("Compute Parameters", {
        use crate::knot;

        let pts = [0.0,0.0,0.0, 1.0,0.0,0.0, 2.0,0.0,0.0, 3.0,0.0,0.0];
        // Chord-length parameterization: since all gaps are 1.0, params = {0, 1, 2, 3}
        let t = knot::compute_parameters(&pts, 3, knot::CurveKnotStyle::Chord);
        MINI_CHECK!(TOLERANCE.is_allclose(&t, &[0.0, 1.0, 2.0, 3.0]));
    })
}

pub fn run_build_interp_knots() -> TestResult {
    MINI_TEST!("Build Interpolation Knots", {
        use crate::knot;

        let params = [0.0, 1.0, 2.0, 3.0];
        let degree = 3;
        // cv_count = n + 2 = 6 (natural end conditions add 2 CVs)
        // kc = order + cv_count - 2 = 4 + 6 - 2 = 8
        //   [0, 0, 0,  |  1, 2,  |  3, 3, 3]
        //   <-clamp->    interior    <-clamp->
        let knots = knot::build_interp_knots(&params, degree);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots, &[0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0]));
    })
}

pub fn run_eval_basis() -> TestResult {
    MINI_TEST!("Evaluation Basis", {
        use crate::knot;

        // Cox-de Boor recursive evaluation of B-spline basis functions
        // At parameter t, exactly 'order' basis functions are non-zero
        // Partition of unity: they always sum to 1.0
        // Used to evaluate NURBS curves/surfaces: C(t) = sum(N_i(t) * P_i)
        // 0 0 0 1 2 2 2
        let order = 4;
        let cv_count = 5;
        let knots = knot::make_clamped_uniform(order, cv_count, 1.0);
        let span = knot::find_span(order, cv_count, &knots, 0.5);
        let basis = knot::eval_basis(order, &knots, span, 0.5);
        MINI_CHECK!(TOLERANCE.is_allclose(&basis, &[0.125, 0.59375, 0.25, 0.03125]));
    })
}

pub fn run_build_fitted_knots_adaptive() -> TestResult {
    MINI_TEST!("Build Fitted Knots Adaptive", {
        use crate::knot;

        // Builds knot vectors for least-squares fitting
        // Concentrates knots where curvature is high (sharp turns)
        // For collinear points (zero curvature), interior knots are evenly distributed
        let pts = [0.0,0.0,0.0, 1.0,0.0,0.0, 2.0,0.0,0.0, 3.0,0.0,0.0, 4.0,0.0,0.0];
        let params = [0.0, 1.0, 2.0, 3.0, 4.0];
        let knots = knot::build_fitted_knots_adaptive(&params, &pts, 3, 5, 3, 3.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots, &[0.0, 0.0, 0.0, 2.0, 4.0, 4.0, 4.0]));
    })
}

pub fn run_build_fitted_knots_periodic_adaptive() -> TestResult {
    MINI_TEST!("Build Fitted Knots Periodic Adaptive", {
        use crate::knot;

        // Periodic version for closed curves -- knots wrap around
        // For a regular square (equal turns, equal chords), knots are uniformly spaced
        let pts = [0.0,0.0,0.0, 1.0,0.0,0.0, 1.0,1.0,0.0, 0.0,1.0,0.0];
        let params = [0.0, 1.0, 2.0, 3.0, 4.0];
        let knots = knot::build_fitted_knots_periodic_adaptive(&params, &pts, 4, 3, 4, 3, 3.0);
        MINI_CHECK!(TOLERANCE.is_allclose(&knots, &[-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
    })
}

pub fn run_solve_banded_spd() -> TestResult {
    MINI_TEST!("Solve Banded SPD", {
        use crate::knot;

        // Cholesky solver for banded symmetric positive-definite systems
        //   | 4 2 0 |       |8 |       |1|
        //   | 2 5 1 | * x = |13| -> x = |2|
        //   | 0 1 3 |       |5 |       |1|
        let mut band = vec![4.0, 0.0, 5.0, 2.0, 3.0, 1.0];
        let mut rhs = vec![8.0, 13.0, 5.0];
        knot::solve_banded_spd(1, 3, 1, &mut band, &mut rhs);
        MINI_CHECK!(TOLERANCE.is_allclose(&rhs, &[1.0, 2.0, 1.0]));
    })
}

// Register all tests
REGISTER_MINI_TEST!("Knot", "Make Clamped Uniform", crate::knot_test::run_make_clamped_uniform);
REGISTER_MINI_TEST!("Knot", "Make Periodic Uniform", crate::knot_test::run_make_periodic_uniform);
REGISTER_MINI_TEST!("Knot", "Is Clamped", crate::knot_test::run_is_clamped);
REGISTER_MINI_TEST!("Knot", "Reverse", crate::knot_test::run_reverse);
REGISTER_MINI_TEST!("Knot", "Find Span", crate::knot_test::run_find_span);
REGISTER_MINI_TEST!("Knot", "Solve Tridiagonal", crate::knot_test::run_solve_tridiagonal);
REGISTER_MINI_TEST!("Knot", "Compute Parameters", crate::knot_test::run_compute_parameters);
REGISTER_MINI_TEST!("Knot", "Build Interpolation Knots", crate::knot_test::run_build_interp_knots);
REGISTER_MINI_TEST!("Knot", "Evaluation Basis", crate::knot_test::run_eval_basis);
REGISTER_MINI_TEST!("Knot", "Build Fitted Knots Adaptive", crate::knot_test::run_build_fitted_knots_adaptive);
REGISTER_MINI_TEST!("Knot", "Build Fitted Knots Periodic Adaptive", crate::knot_test::run_build_fitted_knots_periodic_adaptive);
REGISTER_MINI_TEST!("Knot", "Solve Banded SPD", crate::knot_test::run_solve_banded_spd);
