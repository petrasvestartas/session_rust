use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;


pub fn run_tolerance_is_zero() -> TestResult {
    MINI_TEST!("Is Zero", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.is_zero(1e-10);

        MINI_CHECK!(result);
    })
}

pub fn run_tolerance_is_close() -> TestResult {
    MINI_TEST!("Is Close", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.is_close(1.0, 1.0 + 1e-7);

        MINI_CHECK!(result);
    })
}

pub fn run_tolerance_is_positive() -> TestResult {
    MINI_TEST!("Is Positive", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.is_positive(1.0);

        MINI_CHECK!(result);
    })
}

pub fn run_tolerance_is_negative() -> TestResult {
    MINI_TEST!("Is Negative", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.is_negative(-1.0);

        MINI_CHECK!(result);
    })
}

pub fn run_tolerance_is_between() -> TestResult {
    MINI_TEST!("Is Between", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.is_between(0.5, 0.0, 1.0);

        MINI_CHECK!(result);
    })
}

pub fn run_tolerance_format_number() -> TestResult {
    MINI_TEST!("Format Number", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.format_number(3.14159, 2);

        MINI_CHECK!(result == "3.14");
    })
}

pub fn run_tolerance_key() -> TestResult {
    MINI_TEST!("Key", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.key([1.0, 2.0, 3.0], -999);

        MINI_CHECK!(result == "1.000,2.000,3.000");
    })
}

pub fn run_tolerance_runtime_modification() -> TestResult {
    MINI_TEST!("Runtime Modification", {
        use crate::tolerance::TOLERANCE;
        // Defend against test pollution: other tests may have set_absolute'd
        // without restoring (e.g. primitives_test when an assertion fires early).
        TOLERANCE.reset();
        let original_absolute = TOLERANCE.absolute();
        let original_relative = TOLERANCE.relative();

        MINI_CHECK!(original_absolute == 1e-5);
        MINI_CHECK!(original_relative == 1e-5);

        // Modify tolerance values at runtime
        TOLERANCE.set_absolute(1e-7);
        TOLERANCE.set_relative(1e-7);
        MINI_CHECK!(TOLERANCE.absolute() == 1e-7);
        MINI_CHECK!(TOLERANCE.relative() == 1e-7);

        // Test with new tolerance - 1e-6 difference now fails is_close
        let close_with_tight = TOLERANCE.is_close(1.0, 1.0 + 1e-6);
        MINI_CHECK!(!close_with_tight);

        // Reset to defaults
        TOLERANCE.reset();
        MINI_CHECK!(TOLERANCE.absolute() == 1e-5);
        MINI_CHECK!(TOLERANCE.relative() == 1e-5);

        // Same test now passes with default tolerance
        let close_with_default = TOLERANCE.is_close(1.0, 1.0 + 1e-6);
        MINI_CHECK!(close_with_default);
    })
}

pub fn run_tolerance_unique_from_two_int() -> TestResult {
    MINI_TEST!("Unique From Two Int", {
        use crate::tolerance::unique_from_two_int;
        let r0 = unique_from_two_int(3, 7);
        let r1 = unique_from_two_int(7, 3);

        MINI_CHECK!(r0 == r1);
        MINI_CHECK!(r0 == ((7u64 << 32) | 3u64));
    })
}

pub fn run_tolerance_wrap_index() -> TestResult {
    MINI_TEST!("Wrap Index", {
        use crate::tolerance::wrap_index;
        let r0 = wrap_index(0, 4);
        let r1 = wrap_index(3, 4);
        let r2 = wrap_index(4, 4);
        let r3 = wrap_index(-1, 4);
        let r4 = wrap_index(0, 0);

        MINI_CHECK!(r0 == 0);
        MINI_CHECK!(r1 == 3);
        MINI_CHECK!(r2 == 0);
        MINI_CHECK!(r3 == 3);
        MINI_CHECK!(r4 == 0);
    })
}

pub fn run_tolerance_triangle_edge_by_angle() -> TestResult {
    MINI_TEST!("Triangle Edge By Angle", {
        use crate::tolerance::triangle_edge_by_angle;
        let r = triangle_edge_by_angle(1.0, 45.0);

        MINI_CHECK!((r - 1.0).abs() < 1e-9);
        let r2 = triangle_edge_by_angle(5.0, 0.0);
        MINI_CHECK!(r2.abs() < 1e-9);
    })
}

pub fn run_tolerance_rad_deg() -> TestResult {
    MINI_TEST!("Rad Deg Conversion", {
        use crate::tolerance::rad_to_deg;
        use crate::tolerance::deg_to_rad;
        use crate::tolerance::Tolerance;
        let r0 = rad_to_deg(Tolerance::PI);
        let r1 = deg_to_rad(180.0);
        let r2 = deg_to_rad(rad_to_deg(1.234));

        MINI_CHECK!((r0 - 180.0).abs() < 1e-9);
        MINI_CHECK!((r1 - Tolerance::PI).abs() < 1e-9);
        MINI_CHECK!((r2 - 1.234).abs() < 1e-9);
    })
}

pub fn run_tolerance_to_radians() -> TestResult {
    MINI_TEST!("To Radians", {
        use crate::tolerance::Tolerance;
        let r0 = Tolerance::to_radians(180.0);
        let r1 = Tolerance::to_radians(90.0);
        let r2 = Tolerance::to_radians(0.0);

        MINI_CHECK!((r0 - Tolerance::PI).abs() < 1e-9);
        MINI_CHECK!((r1 - Tolerance::PI / 2.0).abs() < 1e-9);
        MINI_CHECK!(r2.abs() < 1e-9);
    })
}

pub fn run_tolerance_to_degrees() -> TestResult {
    MINI_TEST!("To Degrees", {
        use crate::tolerance::Tolerance;
        let d0 = Tolerance::to_degrees(Tolerance::PI);
        let d1 = Tolerance::to_degrees(Tolerance::PI / 2.0);
        let d2 = Tolerance::to_degrees(0.0);

        MINI_CHECK!((d0 - 180.0).abs() < 1e-9);
        MINI_CHECK!((d1 - 90.0).abs() < 1e-9);
        MINI_CHECK!(d2.abs() < 1e-9);
    })
}

pub fn run_tolerance_count_digits() -> TestResult {
    MINI_TEST!("Count Digits", {
        use crate::tolerance::count_digits;
        let r0 = count_digits(0.0);
        let r1 = count_digits(1.0);
        let r2 = count_digits(9.9);
        let r3 = count_digits(10.0);
        let r4 = count_digits(100.5);
        let r5 = count_digits(-42.0);

        MINI_CHECK!(r0 == 0);
        MINI_CHECK!(r1 == 1);
        MINI_CHECK!(r2 == 1);
        MINI_CHECK!(r3 == 2);
        MINI_CHECK!(r4 == 3);
        MINI_CHECK!(r5 == 2);
    })
}

pub fn run_tolerance_is_angle_zero() -> TestResult {
    MINI_TEST!("Is Angle Zero", {
        use crate::tolerance::TOLERANCE;
        // Angular tolerance default is 1e-6
        let r0 = TOLERANCE.is_angle_zero(1e-8);
        let r1 = TOLERANCE.is_angle_zero(0.1);

        MINI_CHECK!(r0);
        MINI_CHECK!(!r1);
    })
}

pub fn run_tolerance_is_angles_close() -> TestResult {
    MINI_TEST!("Is Angles Close", {
        use crate::tolerance::TOLERANCE;
        let r0 = TOLERANCE.is_angles_close(1.0, 1.0 + 1e-8);
        let r1 = TOLERANCE.is_angles_close(1.0, 2.0);

        MINI_CHECK!(r0);
        MINI_CHECK!(!r1);
    })
}

pub fn run_tolerance_is_point_close() -> TestResult {
    MINI_TEST!("Is Point Close", {
        use crate::tolerance::TOLERANCE;
        use crate::Point;

        let a = Point::new(1.0, 2.0, 3.0);
        let b = Point::new(1.0, 2.0, 3.0 + 1e-12);
        let c = Point::new(1.0, 2.0, 4.0);

        MINI_CHECK!(TOLERANCE.is_point_close(&a, &b));
        MINI_CHECK!(!TOLERANCE.is_point_close(&a, &c));
    })
}

pub fn run_tolerance_is_allclose() -> TestResult {
    MINI_TEST!("Is Allclose", {
        use crate::tolerance::TOLERANCE;
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![1.0, 2.0, 3.0 + 1e-12];
        let c = vec![1.0, 2.0, 4.0];

        MINI_CHECK!(TOLERANCE.is_allclose(&a, &b));
        MINI_CHECK!(!TOLERANCE.is_allclose(&a, &c));
    })
}

pub fn run_tolerance_key_xy() -> TestResult {
    MINI_TEST!("Key Xy", {
        use crate::tolerance::TOLERANCE;
        let result = TOLERANCE.key_xy([1.0, 2.0], -999);

        MINI_CHECK!(result == "1.000,2.000");
    })
}

pub fn run_tolerance_round_to() -> TestResult {
    MINI_TEST!("Round To", {
        use crate::tolerance::Tolerance;
        let r0 = Tolerance::round_to(3.14159, 2);
        let r1 = Tolerance::round_to(2.5, 0);

        MINI_CHECK!((r0 - 3.14).abs() < 1e-9);
        MINI_CHECK!((r1 - 3.0).abs() < 1e-9);
    })
}

pub fn run_tolerance_precision_from_tolerance() -> TestResult {
    MINI_TEST!("Precision From Tolerance", {
        use crate::tolerance::TOLERANCE;
        // Defend against test pollution.
        TOLERANCE.reset();
        // Default absolute tolerance is 1e-5 -> precision should be 5
        let prec = TOLERANCE.precision_from_tolerance(None);

        MINI_CHECK!(prec == 5);
    })
}

pub fn run_tolerance_tolerance() -> TestResult {
    MINI_TEST!("Tolerance", {
        use crate::tolerance::TOLERANCE;
        // rtol * abs(truevalue) + atol
        let result = TOLERANCE.tolerance(1.0, 1e-6, 1e-9);

        MINI_CHECK!((result - (1e-6 + 1e-9)).abs() < 1e-18);
    })
}

pub fn run_tolerance_compare() -> TestResult {
    MINI_TEST!("Compare", {
        use crate::tolerance::TOLERANCE;
        let r0 = TOLERANCE.compare(1.0, 1.0 + 1e-7, 1e-6, 1e-9);
        let r1 = TOLERANCE.compare(1.0, 2.0, 1e-6, 1e-9);

        MINI_CHECK!(r0);
        MINI_CHECK!(!r1);
    })
}

pub fn run_tolerance_is_finite() -> TestResult {
    MINI_TEST!("Is Finite", {
        let r0 = 1.0_f64.is_finite();
        let r1 = f64::INFINITY.is_finite();

        MINI_CHECK!(r0);
        MINI_CHECK!(!r1);
    })
}

pub fn run_tolerance_is_vector_close() -> TestResult {
    MINI_TEST!("Is Vector Close", {
        use crate::tolerance::TOLERANCE;
        use crate::Vector;

        let a = Vector::new(1.0, 2.0, 3.0);
        let b = Vector::new(1.0, 2.0, 3.0 + 1e-12);
        let c = Vector::new(1.0, 2.0, 4.0);

        MINI_CHECK!(TOLERANCE.is_vector_close(&a, &b));
        MINI_CHECK!(!TOLERANCE.is_vector_close(&a, &c));
    })
}

pub fn run_tolerance_temporary() -> TestResult {
    MINI_TEST!("Temporary", {
        use crate::tolerance::TOLERANCE;
        let original = TOLERANCE.absolute();
        let inside = TOLERANCE.temporary(|t| {
            t.set_absolute(1e-12);
            t.absolute()
        });

        MINI_CHECK!(inside == 1e-12);
        MINI_CHECK!(TOLERANCE.absolute() == original);
    })
}

REGISTER_MINI_TEST!("Tolerance", "Is Zero", crate::tolerance_test::run_tolerance_is_zero);
REGISTER_MINI_TEST!("Tolerance", "Is Close", crate::tolerance_test::run_tolerance_is_close);
REGISTER_MINI_TEST!("Tolerance", "Is Positive", crate::tolerance_test::run_tolerance_is_positive);
REGISTER_MINI_TEST!("Tolerance", "Is Negative", crate::tolerance_test::run_tolerance_is_negative);
REGISTER_MINI_TEST!("Tolerance", "Is Between", crate::tolerance_test::run_tolerance_is_between);
REGISTER_MINI_TEST!("Tolerance", "Format Number", crate::tolerance_test::run_tolerance_format_number);
REGISTER_MINI_TEST!("Tolerance", "Key", crate::tolerance_test::run_tolerance_key);
REGISTER_MINI_TEST!("Tolerance", "To Radians", crate::tolerance_test::run_tolerance_to_radians);
REGISTER_MINI_TEST!("Tolerance", "To Degrees", crate::tolerance_test::run_tolerance_to_degrees);
REGISTER_MINI_TEST!("Tolerance", "Runtime Modification", crate::tolerance_test::run_tolerance_runtime_modification);
REGISTER_MINI_TEST!("Tolerance", "Unique From Two Int", crate::tolerance_test::run_tolerance_unique_from_two_int);
REGISTER_MINI_TEST!("Tolerance", "Wrap Index", crate::tolerance_test::run_tolerance_wrap_index);
REGISTER_MINI_TEST!("Tolerance", "Triangle Edge By Angle", crate::tolerance_test::run_tolerance_triangle_edge_by_angle);
REGISTER_MINI_TEST!("Tolerance", "Rad Deg Conversion", crate::tolerance_test::run_tolerance_rad_deg);
REGISTER_MINI_TEST!("Tolerance", "Count Digits", crate::tolerance_test::run_tolerance_count_digits);
REGISTER_MINI_TEST!("Tolerance", "Is Angle Zero", crate::tolerance_test::run_tolerance_is_angle_zero);
REGISTER_MINI_TEST!("Tolerance", "Is Angles Close", crate::tolerance_test::run_tolerance_is_angles_close);
REGISTER_MINI_TEST!("Tolerance", "Is Point Close", crate::tolerance_test::run_tolerance_is_point_close);
REGISTER_MINI_TEST!("Tolerance", "Is Allclose", crate::tolerance_test::run_tolerance_is_allclose);
REGISTER_MINI_TEST!("Tolerance", "Key Xy", crate::tolerance_test::run_tolerance_key_xy);
REGISTER_MINI_TEST!("Tolerance", "Round To", crate::tolerance_test::run_tolerance_round_to);
REGISTER_MINI_TEST!("Tolerance", "Precision From Tolerance", crate::tolerance_test::run_tolerance_precision_from_tolerance);
REGISTER_MINI_TEST!("Tolerance", "Tolerance", crate::tolerance_test::run_tolerance_tolerance);
REGISTER_MINI_TEST!("Tolerance", "Compare", crate::tolerance_test::run_tolerance_compare);
REGISTER_MINI_TEST!("Tolerance", "Is Finite", crate::tolerance_test::run_tolerance_is_finite);
REGISTER_MINI_TEST!("Tolerance", "Is Vector Close", crate::tolerance_test::run_tolerance_is_vector_close);
REGISTER_MINI_TEST!("Tolerance", "Temporary", crate::tolerance_test::run_tolerance_temporary);
