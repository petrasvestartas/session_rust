use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_boundingbox_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::BoundingBox;
        use crate::Point;
        use crate::encoders::{json_dump, json_load};
        let mut original = BoundingBox::from_point(Point::new(1.0, 2.0, 3.0), 5.0);
        original.name = "test_bbox".to_string();
        json_dump(&original, "serialization/test_boundingbox.json", false).unwrap();
        let loaded = json_load::<BoundingBox>("serialization/test_boundingbox.json").unwrap();
        MINI_CHECK!(loaded.name == original.name);
    })
}

REGISTER_MINI_TEST!("BoundingBox", "Json Roundtrip", crate::boundingbox_test::run_boundingbox_json_roundtrip);
