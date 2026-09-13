use crate::mini_test::TestResult;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

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

#[allow(clippy::approx_constant)]
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
        let result = TOLERANCE.key(1.0, 2.0, 3.0, -999);

        MINI_CHECK!(result == "1.000,2.000,3.000");
    })
}

pub fn run_tolerance_to_radians() -> TestResult {
    MINI_TEST!("To Radians", {
        use crate::Tolerance;
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
        use crate::Tolerance;
        let d0 = Tolerance::to_degrees(Tolerance::PI);
        let d1 = Tolerance::to_degrees(Tolerance::PI / 2.0);
        let d2 = Tolerance::to_degrees(0.0);

        MINI_CHECK!((d0 - 180.0).abs() < 1e-9);
        MINI_CHECK!((d1 - 90.0).abs() < 1e-9);
        MINI_CHECK!(d2.abs() < 1e-9);
    })
}

pub fn run_tolerance_runtime_modification() -> TestResult {
    MINI_TEST!("Runtime Modification", {
        use crate::Tolerance;

        let mut tolerance = Tolerance::default();
        let original_absolute = tolerance.absolute();
        let original_relative = tolerance.relative();

        MINI_CHECK!(original_absolute == 1e-9);
        MINI_CHECK!(original_relative == 1e-6);

        tolerance.set_absolute(1e-12);
        tolerance.set_relative(1e-12);
        MINI_CHECK!(tolerance.absolute() == 1e-12);
        MINI_CHECK!(tolerance.relative() == 1e-12);

        let close_with_tight = tolerance.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK!(!close_with_tight);

        tolerance.reset();
        MINI_CHECK!(tolerance.absolute() == 1e-9);
        MINI_CHECK!(tolerance.relative() == 1e-6);

        let close_with_default = tolerance.is_close(1.0, 1.0 + 1e-11);
        MINI_CHECK!(close_with_default);
    })
}

pub fn run_tolerance_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Tolerance;

        let mut tolerance = Tolerance::new("MM");
        tolerance.set_absolute(1e-8);
        tolerance.set_angular(2e-6);
        tolerance.set_angulardeflection(0.2);
        tolerance.set_approximation(0.002);
        tolerance.set_lineardeflection(0.003);
        tolerance.set_precision(4);
        tolerance.set_relative(3e-6);

        let filename = "serialization/test_tolerance.json";
        tolerance.file_json_dump(filename).unwrap();
        let loaded = Tolerance::file_json_load(filename).unwrap();
        let parsed = Tolerance::file_json_loads(&tolerance.file_json_dumps().unwrap()).unwrap();

        MINI_CHECK!(loaded.unit() == "MM");
        MINI_CHECK!(loaded.absolute() == 1e-8);
        MINI_CHECK!(loaded.angular() == 2e-6);
        MINI_CHECK!(loaded.angulardeflection() == 0.2);
        MINI_CHECK!(loaded.approximation() == 0.002);
        MINI_CHECK!(loaded.lineardeflection() == 0.003);
        MINI_CHECK!(loaded.precision() == 4);
        MINI_CHECK!(loaded.relative() == 3e-6);
        MINI_CHECK!(parsed.relative() == 3e-6);
    })
}

pub fn run_tolerance_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Tolerance;

        let mut tolerance = Tolerance::new("MM");
        tolerance.set_absolute(1e-8);
        tolerance.set_angular(2e-6);
        tolerance.set_angulardeflection(0.2);
        tolerance.set_approximation(0.002);
        tolerance.set_lineardeflection(0.003);
        tolerance.set_precision(4);
        tolerance.set_relative(3e-6);

        let filename = "serialization/test_tolerance.bin";
        tolerance.pb_dump(filename).unwrap();
        let loaded = Tolerance::pb_load(filename).unwrap();
        let parsed = Tolerance::pb_loads(&tolerance.pb_dumps()).unwrap();
        let converted = Tolerance::from_proto(tolerance.to_proto());

        MINI_CHECK!(loaded.unit() == "MM");
        MINI_CHECK!(loaded.absolute() == 1e-8);
        MINI_CHECK!(loaded.angular() == 2e-6);
        MINI_CHECK!(loaded.angulardeflection() == 0.2);
        MINI_CHECK!(loaded.approximation() == 0.002);
        MINI_CHECK!(loaded.lineardeflection() == 0.003);
        MINI_CHECK!(loaded.precision() == 4);
        MINI_CHECK!(loaded.relative() == 3e-6);
        MINI_CHECK!(parsed.relative() == 3e-6);
        MINI_CHECK!(converted.relative() == 3e-6);
    })
}

pub fn run_tolerance_serialization_errors() -> TestResult {
    MINI_TEST!("Serialization Errors", {
        use crate::Tolerance;

        let tolerance = Tolerance::default();
        let malformed = Tolerance::pb_loads(&[0xff]).is_err();
        let json_write_failed = tolerance.file_json_dump("").is_err();
        let pb_write_failed = tolerance.pb_dump("").is_err();

        MINI_CHECK!(malformed);
        MINI_CHECK!(json_write_failed);
        MINI_CHECK!(pb_write_failed);
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
        use crate::tolerance::deg_to_rad;
        use crate::tolerance::rad_to_deg;
        use crate::Tolerance;
        let r0 = rad_to_deg(Tolerance::PI);
        let r1 = deg_to_rad(180.0);
        let r2 = deg_to_rad(rad_to_deg(1.234));

        MINI_CHECK!((r0 - 180.0).abs() < 1e-9);
        MINI_CHECK!((r1 - Tolerance::PI).abs() < 1e-9);
        MINI_CHECK!((r2 - 1.234).abs() < 1e-9);
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
        let result = TOLERANCE.key_xy(1.0, 2.0, -999);

        MINI_CHECK!(result == "1.000,2.000");
    })
}

#[allow(clippy::approx_constant)]
pub fn run_tolerance_round_to() -> TestResult {
    MINI_TEST!("Round To", {
        use crate::Tolerance;
        let r0 = Tolerance::round_to(3.14159, 2);
        let r1 = Tolerance::round_to(2.5, 0);

        MINI_CHECK!((r0 - 3.14).abs() < 1e-9);
        MINI_CHECK!((r1 - 3.0).abs() < 1e-9);
    })
}

pub fn run_tolerance_precision_from_tolerance() -> TestResult {
    MINI_TEST!("Precision From Tolerance", {
        use crate::tolerance::TOLERANCE;
        let prec = TOLERANCE.precision_from_tolerance(-1.0);

        MINI_CHECK!(prec == 9);
    })
}

pub fn run_tolerance_tolerance() -> TestResult {
    MINI_TEST!("Tolerance", {
        use crate::tolerance::TOLERANCE;
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
        use crate::tolerance::is_finite;
        let r0 = is_finite(1.0);
        let r1 = is_finite(f64::INFINITY);

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
        use crate::Tolerance;

        let mut tolerance = Tolerance::default();
        let original = tolerance.absolute();
        let inside = tolerance.temporary(|guard| {
            guard.set_absolute(1e-12);
            guard.absolute() == 1e-12
        });
        let restored = tolerance.absolute() == original;

        MINI_CHECK!(inside);
        MINI_CHECK!(restored);

        // Release builds use panic = "abort", so only unwind-capable builds can
        // exercise restoration while a panic crosses the temporary callback.
        #[cfg(panic = "unwind")]
        {
            let threw = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                tolerance.temporary(|guard| {
                    guard.set_absolute(1e-12);
                    panic!("test");
                });
            }))
            .is_err();
            let restored_after_error = tolerance.absolute() == original;

            MINI_CHECK!(threw);
            MINI_CHECK!(restored_after_error);
        }
    })
}

REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Zero",
    crate::tolerance_test::run_tolerance_is_zero
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Close",
    crate::tolerance_test::run_tolerance_is_close
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Positive",
    crate::tolerance_test::run_tolerance_is_positive
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Negative",
    crate::tolerance_test::run_tolerance_is_negative
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Between",
    crate::tolerance_test::run_tolerance_is_between
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Format Number",
    crate::tolerance_test::run_tolerance_format_number
);
REGISTER_MINI_TEST!("Tolerance", "Key", crate::tolerance_test::run_tolerance_key);
REGISTER_MINI_TEST!(
    "Tolerance",
    "To Radians",
    crate::tolerance_test::run_tolerance_to_radians
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "To Degrees",
    crate::tolerance_test::run_tolerance_to_degrees
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Runtime Modification",
    crate::tolerance_test::run_tolerance_runtime_modification
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Json Roundtrip",
    crate::tolerance_test::run_tolerance_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Protobuf Roundtrip",
    crate::tolerance_test::run_tolerance_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Serialization Errors",
    crate::tolerance_test::run_tolerance_serialization_errors
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Unique From Two Int",
    crate::tolerance_test::run_tolerance_unique_from_two_int
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Wrap Index",
    crate::tolerance_test::run_tolerance_wrap_index
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Triangle Edge By Angle",
    crate::tolerance_test::run_tolerance_triangle_edge_by_angle
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Rad Deg Conversion",
    crate::tolerance_test::run_tolerance_rad_deg
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Count Digits",
    crate::tolerance_test::run_tolerance_count_digits
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Angle Zero",
    crate::tolerance_test::run_tolerance_is_angle_zero
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Angles Close",
    crate::tolerance_test::run_tolerance_is_angles_close
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Point Close",
    crate::tolerance_test::run_tolerance_is_point_close
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Allclose",
    crate::tolerance_test::run_tolerance_is_allclose
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Key Xy",
    crate::tolerance_test::run_tolerance_key_xy
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Round To",
    crate::tolerance_test::run_tolerance_round_to
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Precision From Tolerance",
    crate::tolerance_test::run_tolerance_precision_from_tolerance
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Tolerance",
    crate::tolerance_test::run_tolerance_tolerance
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Compare",
    crate::tolerance_test::run_tolerance_compare
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Finite",
    crate::tolerance_test::run_tolerance_is_finite
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Is Vector Close",
    crate::tolerance_test::run_tolerance_is_vector_close
);
REGISTER_MINI_TEST!(
    "Tolerance",
    "Temporary",
    crate::tolerance_test::run_tolerance_temporary
);
