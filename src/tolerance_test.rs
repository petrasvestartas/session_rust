use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_tolerance_is_zero() -> TestResult {
    MINI_TEST!("is_zero", {
        // Values within absolute tolerance should be considered zero
        MINI_CHECK!(TOLERANCE.is_zero(0.0));
        MINI_CHECK!(TOLERANCE.is_zero(1e-10));
        MINI_CHECK!(TOLERANCE.is_zero(-1e-10));

        // Values outside absolute tolerance should not be zero
        MINI_CHECK!(!TOLERANCE.is_zero(1e-5));
        MINI_CHECK!(!TOLERANCE.is_zero(-1e-5));
    })
}

pub fn run_tolerance_is_close() -> TestResult {
    MINI_TEST!("is_close", {
        // Equal values
        MINI_CHECK!(TOLERANCE.is_close(1.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(0.0, 0.0));

        // Values within tolerance
        MINI_CHECK!(TOLERANCE.is_close(1.0, 1.0 + 1e-7));
        MINI_CHECK!(TOLERANCE.is_close(1.0, 1.0 - 1e-7));

        // Values outside tolerance
        MINI_CHECK!(!TOLERANCE.is_close(1.0, 1.0 + 1e-4));
        MINI_CHECK!(!TOLERANCE.is_close(1.0, 2.0));
    })
}

pub fn run_tolerance_is_positive() -> TestResult {
    MINI_TEST!("is_positive", {
        // Positive values above tolerance
        MINI_CHECK!(TOLERANCE.is_positive(1.0));
        MINI_CHECK!(TOLERANCE.is_positive(1e-7));

        // Values at or below tolerance are not positive
        MINI_CHECK!(!TOLERANCE.is_positive(1e-10));
        MINI_CHECK!(!TOLERANCE.is_positive(0.0));
        MINI_CHECK!(!TOLERANCE.is_positive(-1.0));
    })
}

pub fn run_tolerance_is_negative() -> TestResult {
    MINI_TEST!("is_negative", {
        // Negative values below -tolerance
        MINI_CHECK!(TOLERANCE.is_negative(-1.0));
        MINI_CHECK!(TOLERANCE.is_negative(-1e-7));

        // Values at or above -tolerance are not negative
        MINI_CHECK!(!TOLERANCE.is_negative(-1e-10));
        MINI_CHECK!(!TOLERANCE.is_negative(0.0));
        MINI_CHECK!(!TOLERANCE.is_negative(1.0));
    })
}

pub fn run_tolerance_is_between() -> TestResult {
    MINI_TEST!("is_between", {
        // Values within range
        MINI_CHECK!(TOLERANCE.is_between(0.5, 0.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_between(0.0, 0.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_between(1.0, 0.0, 1.0));

        // Values at boundaries (within tolerance)
        MINI_CHECK!(TOLERANCE.is_between(-1e-10, 0.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_between(1.0 + 1e-10, 0.0, 1.0));

        // Values outside range
        MINI_CHECK!(!TOLERANCE.is_between(-0.5, 0.0, 1.0));
        MINI_CHECK!(!TOLERANCE.is_between(1.5, 0.0, 1.0));
    })
}

pub fn run_tolerance_format_number() -> TestResult {
    MINI_TEST!("format_number", {
        // Default precision (3)
        MINI_CHECK!(TOLERANCE.format_number(0.0, None) == "0.000");
        MINI_CHECK!(TOLERANCE.format_number(1.5, None) == "1.500");
        MINI_CHECK!(TOLERANCE.format_number(3.14159, None) == "3.142");

        // Custom precision
        MINI_CHECK!(TOLERANCE.format_number(3.14159, Some(2)) == "3.14");
        MINI_CHECK!(TOLERANCE.format_number(3.14159, Some(5)) == "3.14159");
    })
}

pub fn run_tolerance_geometric_key() -> TestResult {
    MINI_TEST!("geometric_key", {
        // Default precision
        let key = TOLERANCE.geometric_key([1.0, 2.0, 3.0], None);
        MINI_CHECK!(key == "1.000,2.000,3.000");

        // Custom precision
        let key2 = TOLERANCE.geometric_key([1.5, 2.5, 3.5], Some(1));
        MINI_CHECK!(key2 == "1.5,2.5,3.5");

        // Integer precision
        let key3 = TOLERANCE.geometric_key([1.9, 2.1, 3.5], Some(-1));
        MINI_CHECK!(key3 == "1,2,3");
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Tolerance", "is_zero", crate::tolerance_test::run_tolerance_is_zero);
REGISTER_MINI_TEST!("Tolerance", "is_close", crate::tolerance_test::run_tolerance_is_close);
REGISTER_MINI_TEST!("Tolerance", "is_positive", crate::tolerance_test::run_tolerance_is_positive);
REGISTER_MINI_TEST!("Tolerance", "is_negative", crate::tolerance_test::run_tolerance_is_negative);
REGISTER_MINI_TEST!("Tolerance", "is_between", crate::tolerance_test::run_tolerance_is_between);
REGISTER_MINI_TEST!("Tolerance", "format_number", crate::tolerance_test::run_tolerance_format_number);
REGISTER_MINI_TEST!("Tolerance", "geometric_key", crate::tolerance_test::run_tolerance_geometric_key);
