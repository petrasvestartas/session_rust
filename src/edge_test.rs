use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_edge_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::edge::Edge;
        use crate::encoders::{json_dump, json_load};
        let original = Edge::new(
            Some("./serialization/test_edge".to_string()),
            Some("v0".to_string()),
            Some("v1".to_string()),
            None,
        );
        json_dump(&original, "serialization/test_edge.json", false).unwrap();
        let loaded = json_load::<Edge>("serialization/test_edge.json").unwrap();
        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.v0 == original.v0);
        MINI_CHECK!(loaded.v1 == original.v1);
    })
}

REGISTER_MINI_TEST!("Edge", "Json Roundtrip", crate::edge_test::run_edge_json_roundtrip);
