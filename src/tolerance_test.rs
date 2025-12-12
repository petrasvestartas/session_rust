use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_tolerance_is_zero() -> TestResult {
    MINI_TEST!("is_zero", {
        let result = TOLERANCE.is_zero(1e-10);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_close() -> TestResult {
    MINI_TEST!("is_close", {
        let result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_positive() -> TestResult {
    MINI_TEST!("is_positive", {
        let result = TOLERANCE.is_positive(1.0);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_negative() -> TestResult {
    MINI_TEST!("is_negative", {
        let result = TOLERANCE.is_negative(-1.0);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_between() -> TestResult {
    MINI_TEST!("is_between", {
        let result = TOLERANCE.is_between(0.5, 0.0, 1.0);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_format_number() -> TestResult {
    MINI_TEST!("format_number", {
        let result = TOLERANCE.format_number(3.14159, 2);
        MINI_CHECK!(result == "3.14");
    })
}

pub fn run_tolerance_key() -> TestResult {
    MINI_TEST!("key", {
        let result = TOLERANCE.key([1.0, 2.0, 3.0], -999);
        MINI_CHECK!(result == "1.000,2.000,3.000");
    })
}

REGISTER_MINI_TEST!("Tolerance", "is_zero", crate::tolerance_test::run_tolerance_is_zero);
REGISTER_MINI_TEST!("Tolerance", "is_close", crate::tolerance_test::run_tolerance_is_close);
REGISTER_MINI_TEST!("Tolerance", "is_positive", crate::tolerance_test::run_tolerance_is_positive);
REGISTER_MINI_TEST!("Tolerance", "is_negative", crate::tolerance_test::run_tolerance_is_negative);
REGISTER_MINI_TEST!("Tolerance", "is_between", crate::tolerance_test::run_tolerance_is_between);
REGISTER_MINI_TEST!("Tolerance", "format_number", crate::tolerance_test::run_tolerance_format_number);
REGISTER_MINI_TEST!("Tolerance", "key", crate::tolerance_test::run_tolerance_key);
