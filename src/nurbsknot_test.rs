use crate::mini_test::TestResult;
use crate::tolerance::Tolerance;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_nurbsknot_count() -> TestResult {
    MINI_TEST!("Nurbsknot Count", {
        use crate::nurbsknot;

        MINI_CHECK!(nurbsknot::nurbsknot_count(4, 5) == 7);
        MINI_CHECK!(nurbsknot::nurbsknot_count(0, 0) == 0);
        MINI_CHECK!(nurbsknot::nurbsknot_count(4, 3) == 0);
        MINI_CHECK!(nurbsknot::nurbsknot_count(2, usize::MAX) == usize::MAX);
        MINI_CHECK!(nurbsknot::nurbsknot_count(usize::MAX, usize::MAX) == 0);
    })
}

pub fn run_domain_tolerance() -> TestResult {
    MINI_TEST!("Domain Tolerance", {
        use crate::nurbsknot;

        MINI_CHECK!(nurbsknot::domain_tolerance(1.0, 1.0) == 0.0);
        MINI_CHECK!(
            TOLERANCE.is_close(nurbsknot::domain_tolerance(0.0, 1.0), 2.980232238769531e-08)
        );
        MINI_CHECK!(nurbsknot::domain_tolerance(0.0, f64::from_bits(1)) == f64::EPSILON);
    })
}

pub fn run_make_clamped_uniform() -> TestResult {
    MINI_TEST!("Make Clamped Uniform", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);

        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));
        MINI_CHECK!(nurbsknot::compute_clamped_uniform(1, cv_count, 1.0).is_empty());
        MINI_CHECK!(nurbsknot::compute_clamped_uniform(order, cv_count, f64::NAN).is_empty());
        MINI_CHECK!(nurbsknot::compute_clamped_uniform(usize::MAX, usize::MAX, 1.0).is_empty());
    })
}

pub fn run_make_periodic_uniform() -> TestResult {
    MINI_TEST!("Make Periodic Uniform", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let nurbsknots = nurbsknot::compute_periodic_uniform(order, cv_count, 1.0);

        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]));
        MINI_CHECK!(nurbsknot::compute_periodic_uniform(order, cv_count, 0.0).is_empty());
        MINI_CHECK!(nurbsknot::compute_periodic_uniform(order, cv_count, f64::INFINITY).is_empty());
    })
}

pub fn run_clamp() -> TestResult {
    MINI_TEST!("Clamp", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots = vec![9.0, 9.0, 0.0, 1.0, 2.0, 9.0, 9.0];
        let ok = nurbsknot::clamp(order, cv_count, &mut nurbsknots, 2);

        MINI_CHECK!(ok);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));

        let mut left = vec![9.0, 9.0, 0.0, 1.0, 2.0, 8.0, 9.0];
        let mut right = vec![9.0, 8.0, 0.0, 1.0, 2.0, 9.0, 9.0];

        MINI_CHECK!(nurbsknot::clamp(order, cv_count, &mut left, 0));
        MINI_CHECK!(nurbsknot::clamp(order, cv_count, &mut right, 1));
        MINI_CHECK!(TOLERANCE.is_allclose(&left, &[0.0, 0.0, 0.0, 1.0, 2.0, 8.0, 9.0]));
        MINI_CHECK!(TOLERANCE.is_allclose(&right, &[9.0, 8.0, 0.0, 1.0, 2.0, 2.0, 2.0]));
        MINI_CHECK!(!nurbsknot::clamp(order, cv_count, &mut right, 3));
    })
}

pub fn run_is_valid() -> TestResult {
    MINI_TEST!("Is Valid", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let nurbsknots_clamped = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let nurbsknots_flat = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut nurbsknots_nan = nurbsknots_clamped.clone();
        nurbsknots_nan[3] = f64::NAN;

        MINI_CHECK!(nurbsknot::is_valid(order, cv_count, &nurbsknots_clamped));
        MINI_CHECK!(!nurbsknot::is_valid(order, cv_count, &nurbsknots_flat));
        MINI_CHECK!(!nurbsknot::is_valid(order, cv_count, &nurbsknots_nan));
        MINI_CHECK!(!nurbsknot::is_valid(order, cv_count, &[0.0, 1.0]));
    })
}

pub fn run_is_clamped() -> TestResult {
    MINI_TEST!("Is Clamped", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let nurbsknots_periodic = nurbsknot::compute_periodic_uniform(order, cv_count, 1.0);
        let nurbsknots_clamped = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let is_not_clamped = nurbsknot::is_clamped(order, cv_count, &nurbsknots_periodic, 2);
        let is_clamped = nurbsknot::is_clamped(order, cv_count, &nurbsknots_clamped, 2);

        MINI_CHECK!(!is_not_clamped && is_clamped);
        MINI_CHECK!(nurbsknot::is_clamped(
            order,
            cv_count,
            &nurbsknots_clamped,
            0
        ));
        MINI_CHECK!(nurbsknot::is_clamped(
            order,
            cv_count,
            &nurbsknots_clamped,
            1
        ));
        MINI_CHECK!(!nurbsknot::is_clamped(
            order,
            cv_count,
            &nurbsknots_clamped,
            3
        ));
    })
}

pub fn run_is_periodic() -> TestResult {
    MINI_TEST!("Is Periodic", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots_periodic = nurbsknot::compute_periodic_uniform(order, cv_count, 1.0);
        let nurbsknots_clamped = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);

        MINI_CHECK!(nurbsknot::is_periodic(
            order,
            cv_count,
            &nurbsknots_periodic
        ));
        MINI_CHECK!(!nurbsknot::is_periodic(
            order,
            cv_count,
            &nurbsknots_clamped
        ));

        nurbsknots_periodic[3] = f64::NAN;

        MINI_CHECK!(!nurbsknot::is_periodic(
            order,
            cv_count,
            &nurbsknots_periodic
        ));
    })
}

pub fn run_get_domain() -> TestResult {
    MINI_TEST!("Get Domain", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let domain = nurbsknot::get_domain(order, cv_count, &nurbsknots);

        MINI_CHECK!(TOLERANCE.is_close(domain.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(domain.1, 2.0));

        nurbsknots[3] = f64::NAN;

        MINI_CHECK!(nurbsknot::get_domain(order, cv_count, &nurbsknots) == domain);

        nurbsknots[2] = f64::NAN;

        MINI_CHECK!(nurbsknot::get_domain(order, cv_count, &nurbsknots) == (0.0, 0.0));
    })
}

pub fn run_set_domain() -> TestResult {
    MINI_TEST!("Set Domain", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let ok = nurbsknot::set_domain(order, cv_count, &mut nurbsknots, 0.0, 1.0);

        MINI_CHECK!(ok);
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]));
        MINI_CHECK!(!nurbsknot::set_domain(
            order,
            cv_count,
            &mut nurbsknots,
            1.0,
            1.0
        ));
        MINI_CHECK!(!nurbsknot::set_domain(
            order,
            cv_count,
            &mut nurbsknots,
            0.0,
            f64::NAN
        ));
    })
}

pub fn run_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots_sym = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);

        MINI_CHECK!(nurbsknot::reverse(order, cv_count, &mut nurbsknots_sym));
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots_sym, &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]));

        let mut nurbsknots_asym = vec![0.0, 0.0, 0.0, 0.5, 1.0, 2.0, 2.0, 2.0];

        MINI_CHECK!(nurbsknot::reverse(4, 6, &mut nurbsknots_asym));
        MINI_CHECK!(
            TOLERANCE.is_allclose(&nurbsknots_asym, &[0.0, 0.0, 0.0, 1.0, 1.5, 2.0, 2.0, 2.0])
        );

        nurbsknots_asym[3] = f64::INFINITY;

        MINI_CHECK!(!nurbsknot::reverse(4, 6, &mut nurbsknots_asym));
    })
}

pub fn run_multiplicity() -> TestResult {
    MINI_TEST!("Multiplicity", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);

        MINI_CHECK!(nurbsknot::multiplicity(order, cv_count, &nurbsknots, 0) == 3);
        MINI_CHECK!(nurbsknot::multiplicity(order, cv_count, &nurbsknots, 3) == 1);
        MINI_CHECK!(nurbsknot::multiplicity(order, cv_count, &nurbsknots, 7) == 0);

        nurbsknots[3] = f64::NAN;

        MINI_CHECK!(nurbsknot::multiplicity(order, cv_count, &nurbsknots, 3) == 0);
    })
}

pub fn run_span_count() -> TestResult {
    MINI_TEST!("Span Count", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);

        MINI_CHECK!(nurbsknot::span_count(order, cv_count, &nurbsknots) == 2);

        nurbsknots[3] = f64::NAN;

        MINI_CHECK!(nurbsknot::span_count(order, cv_count, &nurbsknots) == 0);
    })
}

pub fn run_find_span() -> TestResult {
    MINI_TEST!("Find Span", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let nurbsknots_clamped = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let spancount0 = nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, 0.5, 0, 0);
        let spancount1 = nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, 1.5, 0, 0);

        MINI_CHECK!(spancount0 == 0 && spancount1 == 1);
        MINI_CHECK!(nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, -1.0, 0, 0) == 0);
        MINI_CHECK!(nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, 3.0, 0, 0) == 1);
        MINI_CHECK!(nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, 0.5, -1, 42) == 0);
        MINI_CHECK!(
            nurbsknot::find_span(order, cv_count, &nurbsknots_clamped, f64::NAN, 0, 0) == 0
        );
    })
}

pub fn run_get_greville_abcissae() -> TestResult {
    MINI_TEST!("Get Greville Abcissae", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let mut nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let greville = nurbsknot::get_greville_abcissae(order, cv_count, &nurbsknots, false);
        let periodic = nurbsknot::get_greville_abcissae(order, cv_count, &nurbsknots, true);

        MINI_CHECK!(TOLERANCE.is_allclose(&greville, &[0.0, 1.0 / 3.0, 1.0, 5.0 / 3.0, 2.0]));
        MINI_CHECK!(TOLERANCE.is_allclose(&periodic, &[0.0, 1.0 / 3.0]));

        nurbsknots[2] = f64::INFINITY;

        MINI_CHECK!(
            nurbsknot::get_greville_abcissae(order, cv_count, &nurbsknots, false).is_empty()
        );
    })
}

pub fn run_solve_tridiagonal() -> TestResult {
    MINI_TEST!("Solve Tridiagonal", {
        use crate::nurbsknot;

        let lo = [0.0, 1.0];
        let di = [2.0, 2.0];
        let up = [1.0, 0.0];
        let rh = [3.0, 3.0];
        let sol = nurbsknot::solve_tridiagonal(1, 2, &lo, &di, &up, &rh);

        MINI_CHECK!(sol.is_some());
        MINI_CHECK!(TOLERANCE.is_allclose(&sol.unwrap_or_default(), &[1.0, 1.0]));

        let rh2 = [3.0, 0.0, 3.0, 3.0];
        let sol = nurbsknot::solve_tridiagonal(2, 2, &lo, &di, &up, &rh2);

        MINI_CHECK!(sol.is_some());
        MINI_CHECK!(TOLERANCE.is_allclose(&sol.unwrap_or_default(), &[1.0, -1.0, 1.0, 2.0]));

        let singular = [0.0, 2.0];

        MINI_CHECK!(nurbsknot::solve_tridiagonal(1, 2, &lo, &singular, &up, &rh).is_none());
        MINI_CHECK!(nurbsknot::solve_tridiagonal(usize::MAX, 2, &lo, &di, &up, &rh).is_none());
    })
}

pub fn run_compute_parameters() -> TestResult {
    MINI_TEST!("Compute Parameters", {
        use crate::nurbsknot;
        use crate::nurbsknot::CurveInterpStyle;
        use crate::nurbsknot::CurveNurbsKnotStyle;

        let pts = [0.0, 0.0, 4.0, 0.0, 4.0, 9.0];
        let uniform = nurbsknot::compute_parameters(&pts, 3, 2, CurveNurbsKnotStyle::Uniform);
        let chord = nurbsknot::compute_parameters(&pts, 3, 2, CurveNurbsKnotStyle::Chord);
        let root = nurbsknot::compute_parameters(&pts, 3, 2, CurveNurbsKnotStyle::ChordSquareRoot);
        let periodic =
            nurbsknot::compute_parameters(&pts, 3, 2, CurveNurbsKnotStyle::ChordPeriodic);

        MINI_CHECK!(TOLERANCE.is_allclose(&uniform, &[0.0, 1.0, 2.0]));
        MINI_CHECK!(TOLERANCE.is_allclose(&chord, &[0.0, 4.0, 13.0]));
        MINI_CHECK!(TOLERANCE.is_allclose(&root, &[0.0, 2.0, 5.0]));
        MINI_CHECK!(TOLERANCE.is_allclose(&periodic, &chord));
        MINI_CHECK!(CurveInterpStyle::Rhino as u8 == 0 && CurveInterpStyle::Occt as u8 == 1);
        MINI_CHECK!(
            nurbsknot::compute_parameters(&[], 3, 2, CurveNurbsKnotStyle::Chord).is_empty()
        );
    })
}

pub fn run_build_interp_nurbsknots() -> TestResult {
    MINI_TEST!("Build Interp Nurbsknots", {
        use crate::nurbsknot;

        let mut params = [0.0, 1.0, 2.0, 3.0];
        let degree = 3;
        let nurbsknots = nurbsknot::build_interp_nurbsknots(&params, degree);

        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0]));

        params[2] = f64::NAN;

        MINI_CHECK!(nurbsknot::build_interp_nurbsknots(&params, degree).is_empty());
        MINI_CHECK!(nurbsknot::build_interp_nurbsknots(&[0.0, 1.0, 2.0], 5).is_empty());
    })
}

pub fn run_eval_basis() -> TestResult {
    MINI_TEST!("Eval Basis", {
        use crate::nurbsknot;

        let order = 4;
        let cv_count = 5;
        let nurbsknots = nurbsknot::compute_clamped_uniform(order, cv_count, 1.0);
        let span = nurbsknot::find_span(order, cv_count, &nurbsknots, 0.5, 0, 0);
        let basis = nurbsknot::eval_basis(order, &nurbsknots, span, 0.5);
        let nan = f64::NAN;

        MINI_CHECK!(TOLERANCE.is_allclose(&basis, &[0.125, 0.59375, 0.25, 0.03125]));
        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknot::eval_basis(1, &[], 0, 0.5), &[1.0]));
        MINI_CHECK!(nurbsknot::eval_basis(0, &[], 0, 0.5).is_empty());
        MINI_CHECK!(nurbsknot::eval_basis(order, &[0.0], span, 0.5).is_empty());
        MINI_CHECK!(TOLERANCE.is_allclose(
            &nurbsknot::eval_basis(3, &[nan, -1.0, 0.0, 1.0, 2.0, 3.0], 2, 1.5),
            &[0.125, 0.75, 0.125]
        ));
        MINI_CHECK!(nurbsknot::eval_basis(3, &[-2.0, -1.0, nan, 1.0, 2.0, 3.0], 2, 1.5).is_empty());
    })
}

pub fn run_build_fitted_nurbsknots_adaptive() -> TestResult {
    MINI_TEST!("Build Fitted Nurbsknots Adaptive", {
        use crate::nurbsknot;
        use crate::nurbsknot::CurveNurbsKnotStyle;

        let pts = [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 3.0, 0.0, 0.0, 4.0, 0.0, 0.0,
        ];
        let params = nurbsknot::compute_parameters(&pts, 5, 3, CurveNurbsKnotStyle::Chord);
        let nurbsknots =
            nurbsknot::build_fitted_nurbsknots_adaptive(&params, &pts, 5, 3, 5, 3, 3.0);
        let fallback = nurbsknot::build_fitted_nurbsknots_adaptive(&params, &[], 5, 3, 5, 3, 3.0);
        let dense =
            nurbsknot::build_fitted_nurbsknots_adaptive(&[0.0, 1.0, 2.0], &pts, 3, 3, 5, 1, 1.0);

        MINI_CHECK!(TOLERANCE.is_allclose(&nurbsknots, &[0.0, 0.0, 0.0, 2.0, 4.0, 4.0, 4.0]));
        MINI_CHECK!(TOLERANCE.is_allclose(&fallback, &[0.0, 0.0, 0.0, 1.5, 4.0, 4.0, 4.0]));
        MINI_CHECK!(
            nurbsknot::build_fitted_nurbsknots_adaptive(&params, &pts, 5, 3, 3, 3, 3.0).is_empty()
        );
        MINI_CHECK!(
            nurbsknot::build_fitted_nurbsknots_adaptive(&[0.0, 1.0], &[], 2, 3, 4, 1, 1.0)
                .is_empty()
        );
        MINI_CHECK!(TOLERANCE.is_allclose(&dense, &[0.0, 0.5, 1.0, 1.5, 2.0]));
    })
}

pub fn run_build_fitted_nurbsknots_periodic_adaptive() -> TestResult {
    MINI_TEST!("Build Fitted Nurbsknots Periodic Adaptive", {
        use crate::nurbsknot;

        let pts = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0];
        let params = [0.0, 1.0, 2.0, 3.0, 4.0];
        let nurbsknots =
            nurbsknot::build_fitted_nurbsknots_periodic_adaptive(&params, &pts, 4, 3, 4, 3, 3.0);
        let fallback = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(
            &[0.0, 1.0, 2.0],
            &[],
            2,
            3,
            4,
            3,
            3.0,
        );
        let boundary = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(
            &[0.0, 1.0, 2.0, 3.0],
            &pts,
            3,
            3,
            1,
            2,
            1.0,
        );

        MINI_CHECK!(TOLERANCE.is_allclose(
            &nurbsknots,
            &[-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        ));
        MINI_CHECK!(
            TOLERANCE.is_allclose(&fallback, &[-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
        );
        MINI_CHECK!(nurbsknot::build_fitted_nurbsknots_periodic_adaptive(
            &[0.0],
            &[],
            0,
            3,
            4,
            3,
            3.0
        )
        .is_empty());
        MINI_CHECK!(nurbsknot::build_fitted_nurbsknots_periodic_adaptive(
            &[0.0, 1.0, 2.0, 3.0],
            &pts,
            3,
            3,
            1,
            3,
            1.0
        )
        .is_empty());
        MINI_CHECK!(TOLERANCE.is_allclose(&boundary, &[-3.0, 0.0, 3.0, 6.0]));
    })
}

pub fn run_solve_banded_spd() -> TestResult {
    MINI_TEST!("Solve Banded SPD", {
        use crate::nurbsknot;

        let mut band = vec![4.0, 0.0, 5.0, 2.0, 3.0, 1.0];
        let mut rhs = vec![8.0, 13.0, 5.0];

        MINI_CHECK!(nurbsknot::solve_banded_spd(1, 3, 1, &mut band, &mut rhs));
        MINI_CHECK!(TOLERANCE.is_allclose(&rhs, &[1.0, 2.0, 1.0]));

        let mut singular = vec![0.0, 0.0];
        let mut value = vec![1.0];

        MINI_CHECK!(!nurbsknot::solve_banded_spd(
            1,
            1,
            1,
            &mut singular,
            &mut value
        ));
        MINI_CHECK!(!nurbsknot::solve_banded_spd(
            1,
            2,
            1,
            &mut singular,
            &mut value
        ));

        let cutoff_value = Tolerance::ABSOLUTE * Tolerance::ABSOLUTE * Tolerance::ZERO_TOLERANCE;
        let mut cutoff = vec![cutoff_value];
        value = vec![1.0];

        MINI_CHECK!(!nurbsknot::solve_banded_spd(
            1,
            1,
            0,
            &mut cutoff,
            &mut value
        ));

        cutoff = vec![f64::from_bits(cutoff_value.to_bits() + 1)];
        value = vec![1.0];

        MINI_CHECK!(nurbsknot::solve_banded_spd(
            1,
            1,
            0,
            &mut cutoff,
            &mut value
        ));
        MINI_CHECK!(!nurbsknot::solve_banded_spd(
            usize::MAX,
            2,
            1,
            &mut singular,
            &mut value
        ));
    })
}

REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Nurbsknot Count",
    crate::nurbsknot_test::run_nurbsknot_count
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Domain Tolerance",
    crate::nurbsknot_test::run_domain_tolerance
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Make Clamped Uniform",
    crate::nurbsknot_test::run_make_clamped_uniform
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Make Periodic Uniform",
    crate::nurbsknot_test::run_make_periodic_uniform
);
REGISTER_MINI_TEST!("NurbsKnot", "Clamp", crate::nurbsknot_test::run_clamp);
REGISTER_MINI_TEST!("NurbsKnot", "Is Valid", crate::nurbsknot_test::run_is_valid);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Is Clamped",
    crate::nurbsknot_test::run_is_clamped
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Is Periodic",
    crate::nurbsknot_test::run_is_periodic
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Get Domain",
    crate::nurbsknot_test::run_get_domain
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Set Domain",
    crate::nurbsknot_test::run_set_domain
);
REGISTER_MINI_TEST!("NurbsKnot", "Reverse", crate::nurbsknot_test::run_reverse);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Multiplicity",
    crate::nurbsknot_test::run_multiplicity
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Span Count",
    crate::nurbsknot_test::run_span_count
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Find Span",
    crate::nurbsknot_test::run_find_span
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Get Greville Abcissae",
    crate::nurbsknot_test::run_get_greville_abcissae
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Solve Tridiagonal",
    crate::nurbsknot_test::run_solve_tridiagonal
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Compute Parameters",
    crate::nurbsknot_test::run_compute_parameters
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Build Interp Nurbsknots",
    crate::nurbsknot_test::run_build_interp_nurbsknots
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Eval Basis",
    crate::nurbsknot_test::run_eval_basis
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Build Fitted Nurbsknots Adaptive",
    crate::nurbsknot_test::run_build_fitted_nurbsknots_adaptive
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Build Fitted Nurbsknots Periodic Adaptive",
    crate::nurbsknot_test::run_build_fitted_nurbsknots_periodic_adaptive
);
REGISTER_MINI_TEST!(
    "NurbsKnot",
    "Solve Banded SPD",
    crate::nurbsknot_test::run_solve_banded_spd
);
