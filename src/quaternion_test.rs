use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_quaternion_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::encoders::{json_dump, json_load};
        let axis = Vector::new(0.0, 0.0, 1.0);
        let original = Quaternion::from_axis_angle(axis, 1.5708);
        json_dump(&original, "serialization/test_quaternion.json", false).unwrap();
        let loaded = json_load::<Quaternion>("serialization/test_quaternion.json").unwrap();

        MINI_CHECK!(TOLERANCE.is_close(loaded.s, original.s));
    })
}

pub fn run_quaternion_from_arc() -> TestResult {
    MINI_TEST!("From Arc", {
        use crate::Quaternion;
        use crate::Vector;
        let src = Vector::new(1.0, 0.0, 0.0);
        let dst = Vector::new(0.0, 1.0, 0.0);
        let q = Quaternion::from_arc(src.clone(), dst.clone());
        let rotated = q.rotate_vector(src);

        MINI_CHECK!(TOLERANCE.is_close(rotated[0], dst[0]));
        MINI_CHECK!(TOLERANCE.is_close(rotated[1], dst[1]));
        MINI_CHECK!(TOLERANCE.is_close(rotated[2], dst[2]));
        let src2 = Vector::new(1.0, 0.0, 0.0);
        let dst2 = Vector::new(-1.0, 0.0, 0.0);
        let q2 = Quaternion::from_arc(src2.clone(), dst2);
        let rot = q2.rotate_vector(src2);
        MINI_CHECK!(TOLERANCE.is_close(rot[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(rot[1], 0.0));
    })
}

pub fn run_quaternion_from_euler() -> TestResult {
    MINI_TEST!("From Euler", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q_euler = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        let q_axis = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);

        MINI_CHECK!(TOLERANCE.is_close(q_euler.s, q_axis.s));
        MINI_CHECK!(TOLERANCE.is_close(q_euler.v[2], q_axis.v[2]));
    })
}

pub fn run_quaternion_slerp() -> TestResult {
    MINI_TEST!("Slerp", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q1 = Quaternion::identity();
        let q2 = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let mid = q1.slerp(&q2, 0.5);
        let expected = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);

        MINI_CHECK!(TOLERANCE.is_close(mid.s, expected.s));
        MINI_CHECK!(TOLERANCE.is_close(mid.v[2], expected.v[2]));
        let q3 = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), 0.001);
        let mid2 = q1.slerp(&q3, 0.5);
        let half = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), 0.0005);
        MINI_CHECK!(TOLERANCE.is_close(mid2.s, half.s));
    })
}

pub fn run_quaternion_nlerp() -> TestResult {
    MINI_TEST!("Nlerp", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q1 = Quaternion::identity();
        let q2 = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let r0 = q1.nlerp(&q2, 0.0);
        let r1 = q1.nlerp(&q2, 1.0);

        MINI_CHECK!(TOLERANCE.is_close(r0.s, q1.s));
        MINI_CHECK!(TOLERANCE.is_close(r1.s, q2.s));
    })
}

pub fn run_quaternion_invert() -> TestResult {
    MINI_TEST!("Invert", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 3.0);
        let result = q.clone() * q.invert();

        MINI_CHECK!(TOLERANCE.is_close(result.s, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(result.v[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.v[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.v[2], 0.0));
    })
}

pub fn run_quaternion_dot() -> TestResult {
    MINI_TEST!("Dot", {
        use crate::Quaternion;
        let q = Quaternion::identity();

        MINI_CHECK!(TOLERANCE.is_close(q.dot(&q), 1.0));
    })
}

pub fn run_quaternion_add() -> TestResult {
    MINI_TEST!("Add", {
        use crate::Quaternion;
        let a = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let b = Quaternion::new(0.0, 0.0, 0.0, 1.0);
        let r = a + b;

        MINI_CHECK!(TOLERANCE.is_close(r.s, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(r.v[2], 1.0));
    })
}

pub fn run_quaternion_sub() -> TestResult {
    MINI_TEST!("Sub", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);
        let r = q.clone() - q;

        MINI_CHECK!(TOLERANCE.is_close(r.s, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(r.v[2], 0.0));
    })
}

pub fn run_quaternion_neg() -> TestResult {
    MINI_TEST!("Neg", {
        use crate::Quaternion;
        let q = Quaternion::identity();
        let r = -q;

        MINI_CHECK!(TOLERANCE.is_close(r.s, -1.0));
    })
}

pub fn run_quaternion_mul_scalar() -> TestResult {
    MINI_TEST!("Mul Scalar", {
        use crate::Quaternion;
        let q = Quaternion::identity();
        let r = q * 2.0;

        MINI_CHECK!(TOLERANCE.is_close(r.s, 2.0));
    })
}

pub fn run_quaternion_magnitude2() -> TestResult {
    MINI_TEST!("Magnitude2", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);

        MINI_CHECK!(TOLERANCE.is_close(q.magnitude2(), q.magnitude() * q.magnitude()));
    })
}

pub fn run_quaternion_conjugate() -> TestResult {
    MINI_TEST!("Conjugate", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);
        let r = q.conjugate();

        MINI_CHECK!(TOLERANCE.is_close(r.s, q.s));
        MINI_CHECK!(TOLERANCE.is_close(r.v[0], -q.v[0]));
        MINI_CHECK!(TOLERANCE.is_close(r.v[2], -q.v[2]));
    })
}

pub fn run_quaternion_mul() -> TestResult {
    MINI_TEST!("Mul", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::tolerance::PI;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let r = q.clone() * q;
        let v = r.rotate_vector(Vector::new(1.0, 0.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(v[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(v[1], 0.0));
    })
}

REGISTER_MINI_TEST!("Quaternion", "Json Roundtrip", crate::quaternion_test::run_quaternion_json_roundtrip);
REGISTER_MINI_TEST!("Quaternion", "From Arc", crate::quaternion_test::run_quaternion_from_arc);
REGISTER_MINI_TEST!("Quaternion", "From Euler", crate::quaternion_test::run_quaternion_from_euler);
REGISTER_MINI_TEST!("Quaternion", "Slerp", crate::quaternion_test::run_quaternion_slerp);
REGISTER_MINI_TEST!("Quaternion", "Nlerp", crate::quaternion_test::run_quaternion_nlerp);
REGISTER_MINI_TEST!("Quaternion", "Invert", crate::quaternion_test::run_quaternion_invert);
REGISTER_MINI_TEST!("Quaternion", "Dot", crate::quaternion_test::run_quaternion_dot);
REGISTER_MINI_TEST!("Quaternion", "Add", crate::quaternion_test::run_quaternion_add);
REGISTER_MINI_TEST!("Quaternion", "Sub", crate::quaternion_test::run_quaternion_sub);
REGISTER_MINI_TEST!("Quaternion", "Neg", crate::quaternion_test::run_quaternion_neg);
REGISTER_MINI_TEST!("Quaternion", "Mul Scalar", crate::quaternion_test::run_quaternion_mul_scalar);
REGISTER_MINI_TEST!("Quaternion", "Magnitude2", crate::quaternion_test::run_quaternion_magnitude2);
REGISTER_MINI_TEST!("Quaternion", "Conjugate", crate::quaternion_test::run_quaternion_conjugate);
REGISTER_MINI_TEST!("Quaternion", "Mul", crate::quaternion_test::run_quaternion_mul);
