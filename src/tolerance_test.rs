use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_tolerance_is_zero() -> TestResult {
    MINI_TEST!("Is_zero", {
        let result = TOLERANCE.is_zero(1e-10);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_close() -> TestResult {
    MINI_TEST!("Is_close", {
        let result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_positive() -> TestResult {
    MINI_TEST!("Is_positive", {
        let result = TOLERANCE.is_positive(1.0);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_negative() -> TestResult {
    MINI_TEST!("Is_negative", {
        let result = TOLERANCE.is_negative(-1.0);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_is_between() -> TestResult {
    MINI_TEST!("Is_between", {
        let result = TOLERANCE.is_between(0.5, 0.0, 1.0);
        MINI_CHECK!(result == true);
    })
}

pub fn run_tolerance_format_number() -> TestResult {
    MINI_TEST!("Format_number", {
        let result = TOLERANCE.format_number(3.14159, 2);
        MINI_CHECK!(result == "3.14");
    })
}

pub fn run_tolerance_key() -> TestResult {
    MINI_TEST!("Key", {
        let result = TOLERANCE.key([1.0, 2.0, 3.0], -999);
        MINI_CHECK!(result == "1.000,2.000,3.000");
    })
}

pub fn run_tolerance_runtime_modification() -> TestResult {
    MINI_TEST!("Runtime_modification", {
        // Get current default values
        let original_absolute = TOLERANCE.absolute();
        let original_relative = TOLERANCE.relative();
        MINI_CHECK!(original_absolute == 1e-9);
        MINI_CHECK!(original_relative == 1e-6);

        // Modify tolerance values at runtime
        TOLERANCE.set_absolute(1e-12);
        TOLERANCE.set_relative(1e-12);
        MINI_CHECK!(TOLERANCE.absolute() == 1e-12);
        MINI_CHECK!(TOLERANCE.relative() == 1e-12);

        // Test with new tolerance - 1e-11 difference now fails is_close
        let close_with_tight = TOLERANCE.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK!(close_with_tight == false);

        // Reset to defaults
        TOLERANCE.reset();
        MINI_CHECK!(TOLERANCE.absolute() == 1e-9);
        MINI_CHECK!(TOLERANCE.relative() == 1e-6);

        // Same test now passes with default tolerance
        let close_with_default = TOLERANCE.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK!(close_with_default == true);
    })
}

REGISTER_MINI_TEST!("Tolerance", "Is_zero", crate::tolerance_test::run_tolerance_is_zero);
REGISTER_MINI_TEST!("Tolerance", "Is_close", crate::tolerance_test::run_tolerance_is_close);
REGISTER_MINI_TEST!("Tolerance", "Is_positive", crate::tolerance_test::run_tolerance_is_positive);
REGISTER_MINI_TEST!("Tolerance", "Is_negative", crate::tolerance_test::run_tolerance_is_negative);
REGISTER_MINI_TEST!("Tolerance", "Is_between", crate::tolerance_test::run_tolerance_is_between);
REGISTER_MINI_TEST!("Tolerance", "Format_number", crate::tolerance_test::run_tolerance_format_number);
REGISTER_MINI_TEST!("Tolerance", "Key", crate::tolerance_test::run_tolerance_key);
REGISTER_MINI_TEST!("Tolerance", "Runtime_modification", crate::tolerance_test::run_tolerance_runtime_modification);
