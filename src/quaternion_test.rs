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

REGISTER_MINI_TEST!("Quaternion", "Json Roundtrip", crate::quaternion_test::run_quaternion_json_roundtrip);
