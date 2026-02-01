//! Tests for knot module.

use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_knot_count() -> TestResult {
    MINI_TEST!("knot_count", {
        use crate::knot;

        // Calculate knot counts for various order/cv_count combinations
        let count1 = knot::knot_count(2, 2);
        let count2 = knot::knot_count(3, 3);
        let count3 = knot::knot_count(4, 4);
        let count4 = knot::knot_count(4, 5);
        let count5 = knot::knot_count(3, 4);

        MINI_CHECK!(count1 == 2);
        MINI_CHECK!(count2 == 4);
        MINI_CHECK!(count3 == 6);
        MINI_CHECK!(count4 == 7);
        MINI_CHECK!(count5 == 5);
    })
}

pub fn run_make_clamped_uniform() -> TestResult {
    MINI_TEST!("make_clamped_uniform", {
        use crate::knot;

        // Basic clamped uniform knot vector
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let k_len = k.len();
        let k0 = k[0];
        let k1 = k[1];
        let k2 = k[2];
        let k3 = k[3];
        let k4 = k[4];
        let k5 = k[5];

        // With custom delta
        let k2_vec = knot::make_clamped_uniform(3, 4, 2.5);
        let k2_len = k2_vec.len();
        let (t0, t1) = knot::get_domain(3, 4, &k2_vec);

        // Invalid params
        let k_invalid1 = knot::make_clamped_uniform(1, 2, 1.0);
        let k_invalid1_empty = k_invalid1.is_empty();
        let k_invalid2 = knot::make_clamped_uniform(4, 3, 1.0);
        let k_invalid2_empty = k_invalid2.is_empty();

        MINI_CHECK!(k_len == 6);
        MINI_CHECK!(k0 == 0.0 && k1 == 0.0 && k2 == 0.0);
        MINI_CHECK!(k3 == 1.0 && k4 == 1.0 && k5 == 1.0);
        MINI_CHECK!(k2_len == 5);
        MINI_CHECK!(t0 == 0.0 && t1 == 5.0);
        MINI_CHECK!(k_invalid1_empty);
        MINI_CHECK!(k_invalid2_empty);
    })
}

pub fn run_make_periodic_uniform() -> TestResult {
    MINI_TEST!("make_periodic_uniform", {
        use crate::knot;

        // Create periodic uniform knot vector
        let k = knot::make_periodic_uniform(3, 4, 1.0);
        let k_len = k.len();
        let k0 = k[0];
        let k1 = k[1];
        let k2 = k[2];
        let k3 = k[3];
        let k4 = k[4];

        MINI_CHECK!(k_len == 5);
        MINI_CHECK!(k0 == 0.0 && k1 == 1.0 && k2 == 2.0 && k3 == 3.0 && k4 == 4.0);
    })
}

pub fn run_clamp() -> TestResult {
    MINI_TEST!("clamp", {
        use crate::knot;

        // Clamp a periodic knot vector
        let mut k = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let clamp_result = knot::clamp(4, 4, &mut k, 2);
        let first_clamped = k[0] == k[1] && k[1] == k[2];
        let last_clamped = k[3] == k[4] && k[4] == k[5];

        MINI_CHECK!(clamp_result);
        MINI_CHECK!(first_clamped);
        MINI_CHECK!(last_clamped);
    })
}

pub fn run_is_valid() -> TestResult {
    MINI_TEST!("is_valid", {
        use crate::knot;

        // Valid clamped knot vector
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let valid = knot::is_valid(4, 4, &k);

        // Invalid - decreasing values
        let k_invalid = vec![0.0, 0.0, 1.0, 0.5, 1.0, 1.0];
        let invalid = knot::is_valid(4, 4, &k_invalid);

        MINI_CHECK!(valid);
        MINI_CHECK!(!invalid);
    })
}

pub fn run_is_clamped() -> TestResult {
    MINI_TEST!("is_clamped", {
        use crate::knot;

        // Clamped knot vector
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let clamped_both = knot::is_clamped(4, 4, &k, 2);
        let clamped_start = knot::is_clamped(4, 4, &k, 0);
        let clamped_end = knot::is_clamped(4, 4, &k, 1);

        // Periodic knot vector (not clamped)
        let k2 = knot::make_periodic_uniform(4, 5, 1.0);
        let not_clamped = knot::is_clamped(4, 5, &k2, 2);

        MINI_CHECK!(clamped_both);
        MINI_CHECK!(clamped_start);
        MINI_CHECK!(clamped_end);
        MINI_CHECK!(!not_clamped);
    })
}

pub fn run_is_periodic() -> TestResult {
    MINI_TEST!("is_periodic", {
        use crate::knot;

        // Periodic knot vector
        let k = knot::make_periodic_uniform(3, 4, 1.0);
        let periodic = knot::is_periodic(3, 4, &k);

        // Clamped knot vector (not periodic)
        let k2 = knot::make_clamped_uniform(4, 4, 1.0);
        let not_periodic = knot::is_periodic(4, 4, &k2);

        MINI_CHECK!(periodic);
        MINI_CHECK!(!not_periodic);
    })
}

pub fn run_get_domain() -> TestResult {
    MINI_TEST!("get_domain", {
        use crate::knot;

        // Get domain of clamped knot vector
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let (t0, t1) = knot::get_domain(4, 4, &k);

        MINI_CHECK!(t0 == 0.0);
        MINI_CHECK!(t1 == 1.0);
    })
}

pub fn run_set_domain() -> TestResult {
    MINI_TEST!("set_domain", {
        use crate::knot;

        // Create knot vector and set domain
        let mut k = knot::make_clamped_uniform(4, 4, 1.0);
        let set_result = knot::set_domain(4, 4, &mut k, 5.0, 10.0);
        let (t0, t1) = knot::get_domain(4, 4, &k);
        let t0_close = (t0 - 5.0).abs() < 1e-10;
        let t1_close = (t1 - 10.0).abs() < 1e-10;

        MINI_CHECK!(set_result);
        MINI_CHECK!(t0_close);
        MINI_CHECK!(t1_close);
    })
}

pub fn run_reverse() -> TestResult {
    MINI_TEST!("reverse", {
        use crate::knot;

        // Reverse knot vector
        let mut k = knot::make_clamped_uniform(4, 4, 1.0);
        let (t0_orig, t1_orig) = knot::get_domain(4, 4, &k);
        let reverse_result = knot::reverse(4, 4, &mut k);
        let (t0, t1) = knot::get_domain(4, 4, &k);
        let t0_preserved = (t0 - t0_orig).abs() < 1e-10;
        let t1_preserved = (t1 - t1_orig).abs() < 1e-10;

        MINI_CHECK!(reverse_result);
        MINI_CHECK!(t0_preserved);
        MINI_CHECK!(t1_preserved);
    })
}

pub fn run_multiplicity() -> TestResult {
    MINI_TEST!("multiplicity", {
        use crate::knot;

        // Check multiplicity at clamped ends
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let mult_first = knot::multiplicity(4, 4, &k, 0);
        let mult_last = knot::multiplicity(4, 4, &k, 5);

        MINI_CHECK!(mult_first == 3);
        MINI_CHECK!(mult_last == 3);
    })
}

pub fn run_span_count() -> TestResult {
    MINI_TEST!("span_count", {
        use crate::knot;

        // Single Bezier span
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let span1 = knot::span_count(4, 4, &k);

        // Multiple spans
        let k2 = knot::make_clamped_uniform(3, 5, 1.0);
        let span2 = knot::span_count(3, 5, &k2);

        MINI_CHECK!(span1 == 1);
        MINI_CHECK!(span2 == 3);
    })
}

pub fn run_find_span() -> TestResult {
    MINI_TEST!("find_span", {
        use crate::knot;

        // Find span in single-span knot vector
        let k = knot::make_clamped_uniform(4, 4, 1.0);
        let span_0 = knot::find_span(4, 4, &k, 0.0);
        let span_mid = knot::find_span(4, 4, &k, 0.5);
        let span_1 = knot::find_span(4, 4, &k, 1.0);

        // Find span in multi-span knot vector
        let k2 = knot::make_clamped_uniform(3, 5, 1.0);
        let span2_0 = knot::find_span(3, 5, &k2, 0.0);
        let span2_mid = knot::find_span(3, 5, &k2, 1.5);
        let span2_end = knot::find_span(3, 5, &k2, 2.5);

        MINI_CHECK!(span_0 == 0 && span_mid == 0 && span_1 == 0);
        MINI_CHECK!(span2_0 == 0 && span2_mid == 1 && span2_end == 2);
    })
}

pub fn run_greville_abcissae() -> TestResult {
    MINI_TEST!("greville_abcissae", {
        use crate::knot;

        // Get Greville abcissae (control point parameter values)
        let k = knot::make_clamped_uniform(3, 4, 1.0);
        let g = knot::get_greville_abcissae(3, 4, &k, false);
        let g_len = g.len();

        MINI_CHECK!(g_len == 4);
    })
}

pub fn run_domain_tolerance() -> TestResult {
    MINI_TEST!("domain_tolerance", {
        use crate::knot;

        // Calculate domain tolerance
        let tol_same = knot::domain_tolerance(1.0, 1.0);
        let tol_diff = knot::domain_tolerance(0.0, 1.0);

        MINI_CHECK!(tol_same == 0.0);
        MINI_CHECK!(tol_diff > 0.0);
    })
}

// Register all tests
REGISTER_MINI_TEST!("Knot", "knot_count", crate::knot_test::run_knot_count);
REGISTER_MINI_TEST!("Knot", "make_clamped_uniform", crate::knot_test::run_make_clamped_uniform);
REGISTER_MINI_TEST!("Knot", "make_periodic_uniform", crate::knot_test::run_make_periodic_uniform);
REGISTER_MINI_TEST!("Knot", "clamp", crate::knot_test::run_clamp);
REGISTER_MINI_TEST!("Knot", "is_valid", crate::knot_test::run_is_valid);
REGISTER_MINI_TEST!("Knot", "is_clamped", crate::knot_test::run_is_clamped);
REGISTER_MINI_TEST!("Knot", "is_periodic", crate::knot_test::run_is_periodic);
REGISTER_MINI_TEST!("Knot", "get_domain", crate::knot_test::run_get_domain);
REGISTER_MINI_TEST!("Knot", "set_domain", crate::knot_test::run_set_domain);
REGISTER_MINI_TEST!("Knot", "reverse", crate::knot_test::run_reverse);
REGISTER_MINI_TEST!("Knot", "multiplicity", crate::knot_test::run_multiplicity);
REGISTER_MINI_TEST!("Knot", "span_count", crate::knot_test::run_span_count);
REGISTER_MINI_TEST!("Knot", "find_span", crate::knot_test::run_find_span);
REGISTER_MINI_TEST!("Knot", "greville_abcissae", crate::knot_test::run_greville_abcissae);
REGISTER_MINI_TEST!("Knot", "domain_tolerance", crate::knot_test::run_domain_tolerance);
