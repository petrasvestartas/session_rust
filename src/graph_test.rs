use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

///////////////////////////////////////////////////////////////////////////////////////////
// Vertex
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_vertex_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Vertex;

        // Default constructor
        let v0 = Vertex::default();

        // Constructor with name + attribute
        let v = Vertex::new(Some("v_named".to_string()), Some("attr".to_string()));

        MINI_CHECK!(v0.name == "my_vertex");
        MINI_CHECK!(v0.attribute == "");
        MINI_CHECK!(!v0.guid().is_empty());
        MINI_CHECK!(v.name == "v_named");
        MINI_CHECK!(v.attribute == "attr");
    })
}

pub fn run_vertex_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Vertex;
        use crate::file_encoders::{file_json_dump, file_json_load};
        let original = Vertex::new(Some("v0".to_string()), Some("test_attribute".to_string()));
        file_json_dump(&original, "serialization/test_vertex.json", false).unwrap();
        let loaded = file_json_load::<Vertex>("serialization/test_vertex.json").unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.attribute == original.attribute);
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Edge
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_edge_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Edge;

        // Constructor with v0/v1/attribute
        let e = Edge::new(
            Some("my_edge".to_string()),
            Some("a".to_string()),
            Some("b".to_string()),
            Some("attr".to_string()),
        );

        MINI_CHECK!(e.v0 == "a");
        MINI_CHECK!(e.v1 == "b");
        MINI_CHECK!(e.attribute == "attr");
        MINI_CHECK!(!e.guid().is_empty());
    })
}

pub fn run_edge_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Edge;
        use crate::file_encoders::{file_json_dump, file_json_load};
        let original = Edge::new(
            Some("my_edge".to_string()),
            Some("v0".to_string()),
            Some("v1".to_string()),
            Some("test_edge_attr".to_string()),
        );
        file_json_dump(&original, "serialization/test_edge.json", false).unwrap();
        let loaded = file_json_load::<Edge>("serialization/test_edge.json").unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.v0 == original.v0);
        MINI_CHECK!(loaded.v1 == original.v1);
    })
}

pub fn run_edge_vertices() -> TestResult {
    MINI_TEST!("Vertices", {
        use crate::Edge;
        let e = Edge::new(
            Some("my_edge".to_string()),
            Some("a".to_string()),
            Some("b".to_string()),
            None,
        );
        let (u, v) = e.vertices();

        MINI_CHECK!(u == "a" && v == "b");
    })
}

pub fn run_edge_connects() -> TestResult {
    MINI_TEST!("Connects", {
        use crate::Edge;
        let e = Edge::new(
            Some("my_edge".to_string()),
            Some("a".to_string()),
            Some("b".to_string()),
            None,
        );

        MINI_CHECK!(e.connects("a"));
        MINI_CHECK!(e.connects("b"));
        MINI_CHECK!(!e.connects("c"));
    })
}

pub fn run_edge_other_vertex() -> TestResult {
    MINI_TEST!("Other Vertex", {
        use crate::Edge;
        let e = Edge::new(
            Some("my_edge".to_string()),
            Some("a".to_string()),
            Some("b".to_string()),
            None,
        );

        MINI_CHECK!(e.other_vertex("a") == "b");
        MINI_CHECK!(e.other_vertex("b") == "a");
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Graph
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_graph_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Graph;

        // Default constructor
        let g0 = Graph::default();

        // Constructor with name
        let g = Graph::new("my_named_graph");

        MINI_CHECK!(g0.name == "my_graph");
        MINI_CHECK!(!g0.guid().is_empty());
        MINI_CHECK!(g.name == "my_named_graph");
    })
}

pub fn run_graph_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Graph;
        let mut original = Graph::new("test_graph");
        original.add_node("node1", "Node 1");
        original.add_node("node2", "Node 2");
        original.add_edge("node1", "node2", "edge1");
        original.file_json_dump("serialization/test_graph.json").unwrap();
        let loaded = Graph::file_json_load("serialization/test_graph.json").unwrap();

        MINI_CHECK!(loaded.number_of_vertices() == 2);
        MINI_CHECK!(loaded.number_of_edges() == 1);
        MINI_CHECK!(loaded.has_edge(("node1", "node2")));
    })
}

pub fn run_graph_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Graph;
        let mut original = Graph::new("test_graph");
        original.add_node("node1", "Node 1");
        original.add_node("node2", "Node 2");
        original.add_edge("node1", "node2", "edge1");
        original.pb_dump("serialization/test_graph.bin");
        let loaded = Graph::pb_load("serialization/test_graph.bin");

        MINI_CHECK!(loaded.number_of_vertices() == 2);
        MINI_CHECK!(loaded.number_of_edges() == 1);
        MINI_CHECK!(loaded.has_edge(("node1", "node2")));
    })
}

pub fn run_graph_has_node() -> TestResult {
    MINI_TEST!("Has Node", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_node("a", "");

        MINI_CHECK!(g.has_node("a"));
        MINI_CHECK!(!g.has_node("missing"));
    })
}

pub fn run_graph_has_edge() -> TestResult {
    MINI_TEST!("Has Edge", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");

        MINI_CHECK!(g.has_edge(("a", "b")));
        MINI_CHECK!(!g.has_edge(("a", "c")));
    })
}

pub fn run_graph_has_guid() -> TestResult {
    MINI_TEST!("Has Guid", {
        use crate::graph::{Edge, Vertex};
        // A guid is lazily minted, so ASKING for one creates it. The writers used to ask for
        // every vertex and edge, which minted 34,592 UUIDs for one drawing sheet and wrote
        // 1.3 MB of them into a file whose reader discards them. has_guid() answers without
        // minting, so a thing nobody names never pays for one.
        let v = Vertex::new(Some("a".to_string()), None);
        let e = Edge::new(None, Some("a".to_string()), Some("b".to_string()), None);

        MINI_CHECK!(!v.has_guid());              // nobody has asked
        MINI_CHECK!(!e.has_guid());

        let minted = v.guid().to_string();
        MINI_CHECK!(!minted.is_empty());
        MINI_CHECK!(v.has_guid());               // asking created it
        MINI_CHECK!(v.guid() == minted);         // and it is stable
    })
}

pub fn run_graph_add_node() -> TestResult {
    MINI_TEST!("Add Node", {
        use crate::Graph;
        let mut g = Graph::new("g");
        let key = g.add_node("a", "");

        MINI_CHECK!(key == "a");
        MINI_CHECK!(g.has_node("a"));
        MINI_CHECK!(g.number_of_vertices() == 1);
    })
}

pub fn run_graph_add_edge() -> TestResult {
    MINI_TEST!("Add Edge", {
        use crate::Graph;
        let mut g = Graph::new("g");
        let edge = g.add_edge("a", "b", "");
        let (u, v) = edge;

        MINI_CHECK!(u == "a" && v == "b");
        MINI_CHECK!(g.number_of_edges() == 1);
    })
}

pub fn run_graph_remove_node() -> TestResult {
    MINI_TEST!("Remove Node", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.remove_node("a");

        MINI_CHECK!(!g.has_node("a"));
        MINI_CHECK!(g.number_of_edges() == 0);
    })
}

pub fn run_graph_remove_edge() -> TestResult {
    MINI_TEST!("Remove Edge", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.remove_edge(("a", "b"));

        MINI_CHECK!(g.number_of_edges() == 0);
        MINI_CHECK!(g.has_node("a"));
        MINI_CHECK!(g.has_node("b"));
    })
}

pub fn run_graph_get_vertices() -> TestResult {
    MINI_TEST!("Get Vertices", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.add_node("b", "");

        let verts = g.get_vertices();

        MINI_CHECK!(verts.len() == 2);
    })
}

pub fn run_graph_get_edges() -> TestResult {
    MINI_TEST!("Get Edges", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");

        let edges = g.get_edges();

        MINI_CHECK!(edges.len() == 2);
    })
}

pub fn run_graph_neighbors() -> TestResult {
    MINI_TEST!("Neighbors", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("a", "c", "");

        let neigh = g.neighbors("a");

        MINI_CHECK!(neigh.len() == 2);
    })
}

pub fn run_graph_get_neighbors() -> TestResult {
    MINI_TEST!("Get Neighbors", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("a", "c", "");

        let neigh = g.get_neighbors("a");

        MINI_CHECK!(neigh.len() == 2);
    })
}

pub fn run_graph_number_of_vertices() -> TestResult {
    MINI_TEST!("Number Of Vertices", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.add_node("b", "");
        g.add_node("c", "");

        MINI_CHECK!(g.number_of_vertices() == 3);
    })
}

pub fn run_graph_number_of_edges() -> TestResult {
    MINI_TEST!("Number Of Edges", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");

        MINI_CHECK!(g.number_of_edges() == 2);
    })
}

pub fn run_graph_clear() -> TestResult {
    MINI_TEST!("Clear", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.clear();

        MINI_CHECK!(g.number_of_vertices() == 0);
        MINI_CHECK!(g.number_of_edges() == 0);
    })
}

pub fn run_graph_node_attribute() -> TestResult {
    MINI_TEST!("Node Attribute", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_node("a", "initial");
        g.node_attribute("a", Some("updated"));

        MINI_CHECK!(g.node_attribute("a", None) == Some("updated".to_string()));
    })
}

pub fn run_graph_edge_attribute() -> TestResult {
    MINI_TEST!("Edge Attribute", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "initial");
        g.edge_attribute("a", "b", Some("updated"));

        MINI_CHECK!(g.edge_attribute("a", "b", None) == Some("updated".to_string()));
    })
}

pub fn run_graph_bfs() -> TestResult {
    MINI_TEST!("Bfs", {
        use crate::Graph;
        let mut g = Graph::new("g");
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
        use crate::Graph;
        let mut g = Graph::new("g");
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
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        g.add_edge("b", "d", "");
        g.add_edge("e", "f", "");
        let comps = g.connected_components();

        MINI_CHECK!(comps.len() == 2);
        MINI_CHECK!(g.is_connected() == false);
        MINI_CHECK!(g.number_connected_components() == 2);
    })
}

pub fn run_graph_shortest_path() -> TestResult {
    MINI_TEST!("Shortest Path", {
        use crate::Graph;
        let mut g = Graph::new("g");
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
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");

        MINI_CHECK!(g.has_cycle() == true);
        let mut g2 = Graph::new("g2");
        g2.add_edge("x", "y", "");
        g2.add_edge("y", "z", "");
        MINI_CHECK!(g2.has_cycle() == false);
    })
}

pub fn run_graph_cycle_basis() -> TestResult {
    MINI_TEST!("Cycle Basis", {
        use crate::Graph;
        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "a", "");
        let cycles = g.cycle_basis();

        MINI_CHECK!(cycles.len() == 1);
    })
}

REGISTER_MINI_TEST!("Vertex", "Constructor", crate::graph_test::run_vertex_constructor);
REGISTER_MINI_TEST!("Vertex", "Json Roundtrip", crate::graph_test::run_vertex_json_roundtrip);
REGISTER_MINI_TEST!("Edge", "Constructor", crate::graph_test::run_edge_constructor);
REGISTER_MINI_TEST!("Edge", "Json Roundtrip", crate::graph_test::run_edge_json_roundtrip);
REGISTER_MINI_TEST!("Edge", "Vertices", crate::graph_test::run_edge_vertices);
REGISTER_MINI_TEST!("Edge", "Connects", crate::graph_test::run_edge_connects);
REGISTER_MINI_TEST!("Edge", "Other Vertex", crate::graph_test::run_edge_other_vertex);
REGISTER_MINI_TEST!("Graph", "Constructor", crate::graph_test::run_graph_constructor);
REGISTER_MINI_TEST!("Graph", "Json Roundtrip", crate::graph_test::run_graph_json_roundtrip);
REGISTER_MINI_TEST!("Graph", "Protobuf Roundtrip", crate::graph_test::run_graph_protobuf_roundtrip);
REGISTER_MINI_TEST!("Graph", "Has Node", crate::graph_test::run_graph_has_node);
REGISTER_MINI_TEST!("Graph", "Has Edge", crate::graph_test::run_graph_has_edge);
REGISTER_MINI_TEST!("Graph", "Has Guid", crate::graph_test::run_graph_has_guid);
REGISTER_MINI_TEST!("Graph", "Add Node", crate::graph_test::run_graph_add_node);
REGISTER_MINI_TEST!("Graph", "Add Edge", crate::graph_test::run_graph_add_edge);
REGISTER_MINI_TEST!("Graph", "Remove Node", crate::graph_test::run_graph_remove_node);
REGISTER_MINI_TEST!("Graph", "Remove Edge", crate::graph_test::run_graph_remove_edge);
REGISTER_MINI_TEST!("Graph", "Get Vertices", crate::graph_test::run_graph_get_vertices);
REGISTER_MINI_TEST!("Graph", "Get Edges", crate::graph_test::run_graph_get_edges);
REGISTER_MINI_TEST!("Graph", "Neighbors", crate::graph_test::run_graph_neighbors);
REGISTER_MINI_TEST!("Graph", "Get Neighbors", crate::graph_test::run_graph_get_neighbors);
REGISTER_MINI_TEST!("Graph", "Number Of Vertices", crate::graph_test::run_graph_number_of_vertices);
REGISTER_MINI_TEST!("Graph", "Number Of Edges", crate::graph_test::run_graph_number_of_edges);
REGISTER_MINI_TEST!("Graph", "Clear", crate::graph_test::run_graph_clear);
REGISTER_MINI_TEST!("Graph", "Node Attribute", crate::graph_test::run_graph_node_attribute);
REGISTER_MINI_TEST!("Graph", "Edge Attribute", crate::graph_test::run_graph_edge_attribute);
REGISTER_MINI_TEST!("Graph", "Bfs", crate::graph_test::run_graph_bfs);
REGISTER_MINI_TEST!("Graph", "Dfs", crate::graph_test::run_graph_dfs);
REGISTER_MINI_TEST!("Graph", "Connected Components", crate::graph_test::run_graph_connected_components);
REGISTER_MINI_TEST!("Graph", "Shortest Path", crate::graph_test::run_graph_shortest_path);
REGISTER_MINI_TEST!("Graph", "Has Cycle", crate::graph_test::run_graph_has_cycle);
REGISTER_MINI_TEST!("Graph", "Cycle Basis", crate::graph_test::run_graph_cycle_basis);
