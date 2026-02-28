use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_vertex_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::vertex::Vertex;
        use crate::encoders::{json_dump, json_load};
        let original = Vertex::new(Some("v0".to_string()), Some("./serialization/test_attribute".to_string()));
        json_dump(&original, "serialization/test_vertex.json", false).unwrap();
        let loaded = json_load::<Vertex>("serialization/test_vertex.json").unwrap();
        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.attribute == original.attribute);
    })
}

REGISTER_MINI_TEST!("Vertex", "Json Roundtrip", crate::vertex_test::run_vertex_json_roundtrip);
