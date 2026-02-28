use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_graph_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::graph::Graph;
        use crate::encoders::{json_dump, json_load};
        let mut original = Graph::new("./serialization/test_graph");
        original.add_node("node1", "Node 1");
        original.add_node("node2", "Node 2");
        original.add_edge("node1", "node2", "edge1");
        json_dump(&original, "serialization/test_graph.json", false).unwrap();
        let loaded = json_load::<Graph>("serialization/test_graph.json").unwrap();
        MINI_CHECK!(loaded.number_of_vertices() == 2);
        MINI_CHECK!(loaded.number_of_edges() == 1);
        MINI_CHECK!(loaded.has_edge(("node1", "node2")));
    })
}

REGISTER_MINI_TEST!("Graph", "Json Roundtrip", crate::graph_test::run_graph_json_roundtrip);
