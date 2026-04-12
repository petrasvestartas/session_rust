use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_objects_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Objects;
        let obj = Objects::new();

        MINI_CHECK!(obj.name == "my_objects");
        MINI_CHECK!(!obj.guid().is_empty());
        MINI_CHECK!(!obj.to_string().is_empty());
    })
}

pub fn run_objects_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::{Objects, Point};
        use crate::encoders::{json_dump, json_load};
        let mut original = Objects::new();
        original.points.push(Point::new(1.0, 2.0, 3.0));
        original.points.push(Point::new(4.0, 5.0, 6.0));
        json_dump(&original, "serialization/test_objects.json", false).unwrap();
        let loaded = json_load::<Objects>("serialization/test_objects.json").unwrap();
        MINI_CHECK!(loaded.points.len() == original.points.len());
    })
}

pub fn run_objects_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::{Objects, Point};
        let mut original = Objects::new();
        original.points.push(Point::new(1.0, 2.0, 3.0));
        original.points.push(Point::new(4.0, 5.0, 6.0));
        original.pb_dump("serialization/test_objects.bin");
        let loaded = Objects::pb_load("serialization/test_objects.bin");
        MINI_CHECK!(loaded.points.len() == original.points.len());
    })
}

REGISTER_MINI_TEST!("Objects", "Constructor", crate::objects_test::run_objects_constructor);
REGISTER_MINI_TEST!("Objects", "Json Roundtrip", crate::objects_test::run_objects_json_roundtrip);
REGISTER_MINI_TEST!("Objects", "Protobuf Roundtrip", crate::objects_test::run_objects_protobuf_roundtrip);
