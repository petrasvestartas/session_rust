use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_vertex_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::graph::Vertex;
        use crate::encoders::{json_dump, json_load};
        let original = Vertex::new(Some("v0".to_string()), Some("./serialization/test_attribute".to_string()));
        json_dump(&original, "serialization/test_vertex.json", false).unwrap();
        let loaded = json_load::<Vertex>("serialization/test_vertex.json").unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.attribute == original.attribute);
    })
}

pub fn run_edge_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::graph::Edge;
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

pub fn run_graph_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::graph::Graph;
        let mut original = Graph::new("./serialization/test_graph");
        original.add_node("node1", "Node 1");
        original.add_node("node2", "Node 2");
        original.add_edge("node1", "node2", "edge1");
        original.json_dump("serialization/test_graph.json").unwrap();
        let loaded = Graph::json_load("serialization/test_graph.json").unwrap();

        MINI_CHECK!(loaded.number_of_vertices() == 2);
        MINI_CHECK!(loaded.number_of_edges() == 1);
        MINI_CHECK!(loaded.has_edge(("node1", "node2")));
    })
}

pub fn run_graph_bfs() -> TestResult {
    MINI_TEST!("Bfs", {
        use crate::graph::Graph;
        let mut g = Graph::new("test");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");
        let result = g.bfs("a");

        MINI_CHECK!(result == vec!["a", "b", "c", "d"]);
    })
}

pub fn run_graph_dfs() -> TestResult {
    MINI_TEST!("Dfs", {
        use crate::graph::Graph;
        let mut g = Graph::new("test");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");
        let result = g.dfs("a");

        MINI_CHECK!(result == vec!["a", "b", "c", "d"]);
    })
}

pub fn run_graph_connected_components() -> TestResult {
    MINI_TEST!("Connected Components", {
        use crate::graph::Graph;
        let mut g = Graph::new("test");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");
        let comps = g.connected_components();

        MINI_CHECK!(comps.len() == 2);
        MINI_CHECK!(comps[0] == vec!["a", "b", "c", "d"]);
        MINI_CHECK!(comps[1] == vec!["e", "f"]);
        MINI_CHECK!(g.is_connected() == false);
        MINI_CHECK!(g.number_connected_components() == 2);
    })
}

pub fn run_graph_shortest_path() -> TestResult {
    MINI_TEST!("Shortest Path", {
        use crate::graph::Graph;
        let mut g = Graph::new("test");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");

        MINI_CHECK!(g.shortest_path("a", "d") == vec!["a", "b", "d"]);
        MINI_CHECK!(g.shortest_path_length("a", "d") == 2);
        MINI_CHECK!(g.shortest_path("a", "e") == Vec::<String>::new());
        MINI_CHECK!(g.shortest_path_length("a", "e") == -1);
    })
}

pub fn run_graph_has_cycle() -> TestResult {
    MINI_TEST!("Has Cycle", {
        use crate::graph::Graph;
        let mut g = Graph::new("test");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");

        MINI_CHECK!(g.has_cycle() == true);
        let mut g2 = Graph::new("test2");
        g2.add_edge("x", "y", "");
        g2.add_edge("y", "z", "");

        MINI_CHECK!(g2.has_cycle() == false);
    })
}

pub fn run_graph_cycle_basis() -> TestResult {
    MINI_TEST!("Cycle Basis", {
        use crate::graph::Graph;
        let mut g = Graph::new("test");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");
        let cycles = g.cycle_basis();

        MINI_CHECK!(cycles.len() == 1);
        let cycle_set: std::collections::BTreeSet<_> = cycles[0].iter().cloned().collect();
        let expected: std::collections::BTreeSet<_> = vec!["a".to_string(), "b".to_string(), "c".to_string()].into_iter().collect();

        MINI_CHECK!(cycle_set == expected);
    })
}

REGISTER_MINI_TEST!("Vertex", "Json Roundtrip", crate::graph_test::run_vertex_json_roundtrip);
REGISTER_MINI_TEST!("Edge", "Json Roundtrip", crate::graph_test::run_edge_json_roundtrip);
REGISTER_MINI_TEST!("Graph", "Json Roundtrip", crate::graph_test::run_graph_json_roundtrip);
REGISTER_MINI_TEST!("Graph", "Bfs", crate::graph_test::run_graph_bfs);
REGISTER_MINI_TEST!("Graph", "Dfs", crate::graph_test::run_graph_dfs);
REGISTER_MINI_TEST!("Graph", "Connected Components", crate::graph_test::run_graph_connected_components);
REGISTER_MINI_TEST!("Graph", "Shortest Path", crate::graph_test::run_graph_shortest_path);
REGISTER_MINI_TEST!("Graph", "Has Cycle", crate::graph_test::run_graph_has_cycle);
REGISTER_MINI_TEST!("Graph", "Cycle Basis", crate::graph_test::run_graph_cycle_basis);
