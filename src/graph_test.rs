use crate::mini_test::TestResult;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

// ═══════════════════════════════════════════════════════════════════════════
// Vertex
// ═══════════════════════════════════════════════════════════════════════════
pub fn run_vertex_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Vertex;

        let v0 = Vertex::default();
        let v = Vertex::new("v_named", "attr");

        MINI_CHECK!(v0.name == "my_vertex");
        MINI_CHECK!(v0.attribute.is_empty());
        MINI_CHECK!(!v0.guid().is_empty());
        MINI_CHECK!(v.name == "v_named");
        MINI_CHECK!(v.attribute == "attr");
    })
}

pub fn run_vertex_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load;
        use crate::Vertex;

        let mut original = Vertex::new("v0", "test_attribute");
        original.attributes.insert("load".to_string(), 1.5);

        let fname = "serialization/test_vertex.json";
        file_json_dump(&original, fname, false).unwrap();
        let loaded = file_json_load::<Vertex>(fname).unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.attribute == original.attribute);
        MINI_CHECK!(loaded.attributes == original.attributes);
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge
// ═══════════════════════════════════════════════════════════════════════════
pub fn run_edge_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Edge;

        let e = Edge::new("a", "b", "attr");

        MINI_CHECK!(e.v0 == "a");
        MINI_CHECK!(e.v1 == "b");
        MINI_CHECK!(e.attribute == "attr");
        MINI_CHECK!(!e.guid().is_empty());
    })
}

pub fn run_edge_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load;
        use crate::Edge;

        let mut original = Edge::new("v0", "v1", "test_edge_attr");
        original.attributes.insert("weight".to_string(), 2.5);

        let fname = "serialization/test_edge.json";
        file_json_dump(&original, fname, false).unwrap();
        let loaded = file_json_load::<Edge>(fname).unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.v0 == original.v0);
        MINI_CHECK!(loaded.v1 == original.v1);
        MINI_CHECK!(loaded.attributes == original.attributes);
    })
}

pub fn run_edge_vertices() -> TestResult {
    MINI_TEST!("Vertices", {
        use crate::Edge;

        let e = Edge::new("a", "b", "");
        let (u, v) = e.vertices();

        MINI_CHECK!(u == "a" && v == "b");
    })
}

pub fn run_edge_connects() -> TestResult {
    MINI_TEST!("Connects", {
        use crate::Edge;

        let e = Edge::new("a", "b", "");

        MINI_CHECK!(e.connects("a"));
        MINI_CHECK!(e.connects("b"));
        MINI_CHECK!(!e.connects("c"));
    })
}

pub fn run_edge_other_vertex() -> TestResult {
    MINI_TEST!("Other Vertex", {
        use crate::Edge;

        let e = Edge::new("a", "b", "");

        MINI_CHECK!(e.other_vertex("a") == "b");
        MINI_CHECK!(e.other_vertex("b") == "a");
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Graph
// ═══════════════════════════════════════════════════════════════════════════
pub fn run_graph_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Graph;

        let g0 = Graph::default();
        let g = Graph::new("my_named_graph");
        let gstr = g0.str();
        let grepr = g0.repr();

        MINI_CHECK!(g0.name == "my_graph");
        MINI_CHECK!(!g0.guid().is_empty());
        MINI_CHECK!(g0.vertex_count == 0);
        MINI_CHECK!(g0.edge_count == 0);
        MINI_CHECK!(g.name == "my_named_graph");
        MINI_CHECK!(gstr == "<Graph with 0 vertices, 0 edges: my_graph>");
        MINI_CHECK!(grepr == format!("Graph({}, my_graph, 0, 0)", g0.guid()));
    })
}

pub fn run_graph_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Graph;

        let mut original = Graph::new("test_graph");
        original.add_node("node1", "Node 1");
        original.add_node("node2", "Node 2");
        original.add_edge("node1", "node2", "edge1");

        let edge_key = ("node1", "node2");
        original.update_default_vertex_attributes(&[("load", 1.0)]);
        original.update_default_edge_attributes(&[("weight", 2.0)]);
        original.set_vertex_attribute("node1", "load", 3.0);
        original.set_edge_attribute(edge_key, "weight", 4.0);

        let fname = "serialization/test_graph.json";
        original.file_json_dump(fname).unwrap();
        let loaded = Graph::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded.number_of_vertices() == 2);
        MINI_CHECK!(loaded.number_of_edges() == 1);
        MINI_CHECK!(loaded.has_edge(edge_key));
        MINI_CHECK!(loaded.default_vertex_attributes == original.default_vertex_attributes);
        MINI_CHECK!(loaded.default_edge_attributes == original.default_edge_attributes);
        MINI_CHECK!(loaded.vertex_attribute("node1", "load") == Some(3.0));
        MINI_CHECK!(loaded.vertex_attribute("node2", "load") == Some(1.0));
        MINI_CHECK!(loaded.edge_attribute(edge_key, "weight") == Some(4.0));
    })
}

pub fn run_graph_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Graph;

        let mut original = Graph::new("test_graph");
        original.add_node("node1", "Node 1");
        original.add_node("node2", "Node 2");
        original.add_edge("node1", "node2", "edge1");

        let edge_key = ("node1", "node2");
        original.update_default_vertex_attributes(&[("load", 1.0)]);
        original.update_default_edge_attributes(&[("weight", 2.0)]);
        original.set_vertex_attribute("node1", "load", 3.0);
        original.set_edge_attribute(edge_key, "weight", 4.0);

        let guid = original.guid().to_string();
        let filename = "serialization/test_graph.bin";
        original.pb_dump(filename).unwrap();

        let loaded = Graph::pb_load(filename).unwrap();
        let converted = Graph::from_proto(original.to_proto());

        MINI_CHECK!(loaded.number_of_vertices() == 2);
        MINI_CHECK!(loaded.number_of_edges() == 1);
        MINI_CHECK!(loaded.has_edge(edge_key));
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(loaded.default_vertex_attributes == original.default_vertex_attributes);
        MINI_CHECK!(loaded.default_edge_attributes == original.default_edge_attributes);
        MINI_CHECK!(loaded.vertex_attribute("node1", "load") == Some(3.0));
        MINI_CHECK!(loaded.vertex_attribute("node2", "load") == Some(1.0));
        MINI_CHECK!(loaded.edge_attribute(edge_key, "weight") == Some(4.0));
        MINI_CHECK!(converted.number_of_edges() == 1);
        MINI_CHECK!(converted.guid() == guid);
        MINI_CHECK!(converted.edge_attribute(edge_key, "weight") == Some(4.0));
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

        let ab = ("a", "b");
        let ac = ("a", "c");

        MINI_CHECK!(g.has_edge(ab));
        MINI_CHECK!(!g.has_edge(ac));
    })
}

pub fn run_graph_has_guid() -> TestResult {
    MINI_TEST!("Has Guid", {
        use crate::Edge;
        use crate::Vertex;

        let v = Vertex::new("a", "");
        let e = Edge::new("a", "b", "");

        MINI_CHECK!(!v.has_guid());
        MINI_CHECK!(!e.has_guid());

        let minted = v.guid().to_string();

        MINI_CHECK!(!minted.is_empty());
        MINI_CHECK!(v.has_guid());
        MINI_CHECK!(v.guid() == minted);
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
        g.add_edge("b", "a", "updated");

        MINI_CHECK!(u == "a" && v == "b");
        MINI_CHECK!(g.number_of_edges() == 1);
        MINI_CHECK!(g.edge_count == 1);
        MINI_CHECK!(g.edge_label("a", "b", None).as_deref() == Some("updated"));
        MINI_CHECK!(g.edges["a"]["b"].guid() == g.edges["b"]["a"].guid());
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
        MINI_CHECK!(g.edge_count == 0);
    })
}

pub fn run_graph_remove_edge() -> TestResult {
    MINI_TEST!("Remove Edge", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        let edge_key = ("a", "b");
        g.remove_edge(edge_key);

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

pub fn run_graph_node_label() -> TestResult {
    MINI_TEST!("Node Label", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_node("a", "initial");
        g.node_label("a", Some("updated"));

        MINI_CHECK!(g.node_label("a", None) == Some("updated".to_string()));
    })
}

pub fn run_graph_edge_label() -> TestResult {
    MINI_TEST!("Edge Label", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "initial");
        g.edge_label("a", "b", Some("updated"));

        MINI_CHECK!(g.edge_label("a", "b", None) == Some("updated".to_string()));
    })
}

pub fn run_graph_update_default_vertex_attributes() -> TestResult {
    MINI_TEST!("Update Default Vertex Attributes", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.update_default_vertex_attributes(&[("is_support", 0.0), ("load", 0.0)]);
        g.update_default_vertex_attributes(&[("load", -1.0)]);

        MINI_CHECK!(g.default_vertex_attributes.len() == 2);
        MINI_CHECK!(g.default_vertex_attributes["is_support"] == 0.0);
        MINI_CHECK!(g.default_vertex_attributes["load"] == -1.0);
    })
}

pub fn run_graph_update_default_edge_attributes() -> TestResult {
    MINI_TEST!("Update Default Edge Attributes", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.update_default_edge_attributes(&[("weight", 1.0), ("stiffness", 0.0)]);
        g.update_default_edge_attributes(&[("weight", 2.0)]);

        MINI_CHECK!(g.default_edge_attributes.len() == 2);
        MINI_CHECK!(g.default_edge_attributes["weight"] == 2.0);
        MINI_CHECK!(g.default_edge_attributes["stiffness"] == 0.0);
    })
}

pub fn run_graph_vertex_attribute() -> TestResult {
    MINI_TEST!("Vertex Attribute", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.add_node("b", "");
        g.update_default_vertex_attributes(&[("is_support", 0.0)]);
        g.set_vertex_attribute("a", "is_support", 1.0);

        MINI_CHECK!(g.vertex_attribute("a", "is_support") == Some(1.0));
        MINI_CHECK!(g.vertex_attribute("b", "is_support") == Some(0.0));
        MINI_CHECK!(g.vertex_attribute("a", "missing").is_none());
        MINI_CHECK!(g.vertex_attribute("missing", "is_support").is_none());
    })
}

pub fn run_graph_set_vertex_attribute() -> TestResult {
    MINI_TEST!("Set Vertex Attribute", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.set_vertex_attribute("a", "load", -2.5);
        g.set_vertex_attribute("missing", "load", 1.0);

        let vertices = g.get_vertices();

        MINI_CHECK!(vertices[0].attributes["load"] == -2.5);
        MINI_CHECK!(!g.has_node("missing"));
    })
}

pub fn run_graph_edge_attribute() -> TestResult {
    MINI_TEST!("Edge Attribute", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.update_default_edge_attributes(&[("weight", 1.0)]);

        let ab = ("a", "b");
        let ba = ("b", "a");
        let bc = ("b", "c");
        let ac = ("a", "c");
        g.set_edge_attribute(ab, "weight", 5.0);

        MINI_CHECK!(g.edge_attribute(ab, "weight") == Some(5.0));
        MINI_CHECK!(g.edge_attribute(ba, "weight") == Some(5.0));
        MINI_CHECK!(g.edge_attribute(bc, "weight") == Some(1.0));
        MINI_CHECK!(g.edge_attribute(ab, "missing").is_none());
        MINI_CHECK!(g.edge_attribute(ac, "weight").is_none());
    })
}

pub fn run_graph_set_edge_attribute() -> TestResult {
    MINI_TEST!("Set Edge Attribute", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");

        let ba = ("b", "a");
        let ac = ("a", "c");
        g.set_edge_attribute(ba, "weight", 3.0);
        g.set_edge_attribute(ac, "weight", 1.0);
        g.add_edge("a", "b", "");

        MINI_CHECK!(g.edges["a"]["b"].attributes["weight"] == 3.0);
        MINI_CHECK!(g.edges["b"]["a"].attributes["weight"] == 3.0);
        MINI_CHECK!(!g.has_edge(ac));
    })
}

pub fn run_graph_vertices_where() -> TestResult {
    MINI_TEST!("Vertices Where", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.add_node("b", "");
        g.add_node("c", "");
        g.update_default_vertex_attributes(&[("is_support", 0.0), ("level", 1.0)]);
        g.set_vertex_attribute("a", "is_support", 1.0);
        g.set_vertex_attribute("c", "is_support", 1.0);
        g.set_vertex_attribute("c", "level", 2.0);

        MINI_CHECK!(g.vertices_where(&[("is_support", 1.0)]) == vec!["a", "c"]);
        MINI_CHECK!(g.vertices_where(&[("is_support", 1.0), ("level", 1.0)]) == vec!["a"]);
        MINI_CHECK!(g.vertices_where(&[("is_support", 0.0)]) == vec!["b"]);
        MINI_CHECK!(g.vertices_where(&[("missing", 0.0)]).is_empty());
    })
}

pub fn run_graph_edges_where() -> TestResult {
    MINI_TEST!("Edges Where", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.add_edge("c", "d", "");
        g.update_default_edge_attributes(&[("weight", 0.0)]);

        let bc = ("b", "c");
        let cd = ("c", "d");
        let dc = ("d", "c");
        g.set_edge_attribute(bc, "weight", 3.0);
        g.set_edge_attribute(dc, "weight", 3.0);

        let heavy = g.edges_where(&[("weight", 3.0)]);
        let light = g.edges_where(&[("weight", 0.0)]);

        MINI_CHECK!(heavy.len() == 2);
        MINI_CHECK!((heavy[0].0.as_str(), heavy[0].1.as_str()) == bc);
        MINI_CHECK!((heavy[1].0.as_str(), heavy[1].1.as_str()) == cd);
        MINI_CHECK!(light.len() == 1);
    })
}

pub fn run_graph_vertices_where_predicate() -> TestResult {
    MINI_TEST!("Vertices Where Predicate", {
        use crate::Graph;
        use std::collections::BTreeMap;

        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.add_node("b", "");
        g.add_node("c", "");
        g.update_default_vertex_attributes(&[("load", 1.0)]);
        g.set_vertex_attribute("b", "load", 5.0);
        g.set_vertex_attribute("c", "load", 10.0);

        let heavy = g.vertices_where_predicate(&|_: &str, attributes: &BTreeMap<String, f64>| {
            attributes["load"] > 4.0
        });

        MINI_CHECK!(heavy == vec!["b", "c"]);
    })
}

pub fn run_graph_edges_where_predicate() -> TestResult {
    MINI_TEST!("Edges Where Predicate", {
        use crate::Graph;
        use std::collections::BTreeMap;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("b", "c", "");
        g.update_default_edge_attributes(&[("weight", 1.0)]);

        let bc = ("b", "c");
        g.set_edge_attribute(bc, "weight", 5.0);

        let heavy =
            g.edges_where_predicate(&|_: (&str, &str), attributes: &BTreeMap<String, f64>| {
                attributes["weight"] > 4.0
            });

        MINI_CHECK!(heavy.len() == 1);
        MINI_CHECK!((heavy[0].0.as_str(), heavy[0].1.as_str()) == bc);
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
        MINI_CHECK!(!g.is_connected());
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

        let mut g2 = Graph::new("g2");
        g2.add_edge("x", "y", "");
        g2.add_edge("y", "z", "");

        MINI_CHECK!(g.has_cycle());
        MINI_CHECK!(!g2.has_cycle());
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

pub fn run_graph_take_node() -> TestResult {
    MINI_TEST!("Take Node", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_node("a", "");
        g.add_node("b", "bee");
        g.add_edge("a", "b", "ab");
        g.add_edge("b", "c", "bc");
        g.set_vertex_attribute("b", "load", 2.0);
        g.set_edge_attribute(("a", "b"), "weight", 3.0);
        let before = g.jsondump().unwrap();
        let (vertex, edges) = g.take_node("b").unwrap();

        MINI_CHECK!(
            before.contains(vertex.guid()) && vertex.index == 1 && vertex.attribute == "bee"
        );
        MINI_CHECK!(vertex.attributes.get("load") == Some(&2.0));
        MINI_CHECK!(edges.len() == 2 && edges.iter().all(|e| e.v0 == "b" || e.v1 == "b"));
        MINI_CHECK!(edges
            .iter()
            .any(|e| e.attributes.get("weight") == Some(&3.0)));
        MINI_CHECK!(!g.has_node("b") && !g.has_edge(("a", "b")) && !g.has_edge(("c", "b")));
        MINI_CHECK!(g.vertex_count == 3 && g.edge_count == 2 && g.edges.is_empty());
        MINI_CHECK!(g.get_vertices()[0].index == 0 && g.get_vertices()[1].index == 2);
        MINI_CHECK!(g.str() == "<Graph with 2 vertices, 0 edges: g>");
        MINI_CHECK!(g.take_node("b").is_none());
    })
}

pub fn run_graph_put_node() -> TestResult {
    MINI_TEST!("Put Node", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "ab");
        g.add_edge("b", "c", "bc");
        g.set_vertex_attribute("b", "load", 2.0);
        g.set_edge_attribute(("b", "c"), "weight", 3.0);
        let before = g.jsondump().unwrap();
        let (vertex, edges) = g.take_node("b").unwrap();
        g.put_node(vertex, edges);
        let after = g.jsondump().unwrap();
        let ab = g.edges["a"]["b"].guid().to_string();
        let (vertex, edges) = g.take_node("b").unwrap();
        g.remove_node("c");
        g.add_edge("b", "d", "bd");
        let guid = vertex.guid().to_string();
        g.put_node(vertex, edges);

        MINI_CHECK!(before == after);
        MINI_CHECK!(g.edges["a"]["b"].guid() == ab && g.edges["b"]["a"].guid() == ab);
        MINI_CHECK!(!g.has_edge(("b", "c")) && g.has_edge(("b", "d")));
        MINI_CHECK!(
            g.get_vertices()[1].guid() == guid && g.vertex_attribute("b", "load") == Some(2.0)
        );
        MINI_CHECK!(g.number_of_edges() == 2);
    })
}

pub fn run_graph_renumber() -> TestResult {
    MINI_TEST!("Renumber", {
        use crate::Graph;

        let mut g = Graph::new("g");
        g.add_edge("a", "b", "");
        g.add_edge("a", "c", "");
        g.add_edge("b", "d", "");
        g.add_edge("c", "e", "");
        g.add_edge("d", "e", "");
        g.add_edge("a", "e", "");
        g.take_node("b");
        g.take_node("d");
        g.renumber();
        let indices: Vec<i32> = g.get_vertices().iter().map(|v| v.index).collect();

        MINI_CHECK!(indices == [0, 1, 2]);
        MINI_CHECK!(g.edges["a"]["c"].index == 0 && g.edges["e"]["c"].index == 1);
        MINI_CHECK!(g.edges["a"]["e"].index == 2 && g.edges["e"]["a"].index == 2);
        MINI_CHECK!(g.vertex_count as usize == g.number_of_vertices() && g.vertex_count == 3);
        MINI_CHECK!(g.edge_count as usize == g.number_of_edges() && g.edge_count == 3);
    })
}

REGISTER_MINI_TEST!(
    "Vertex",
    "Constructor",
    crate::graph_test::run_vertex_constructor
);
REGISTER_MINI_TEST!(
    "Vertex",
    "Json Roundtrip",
    crate::graph_test::run_vertex_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Edge",
    "Constructor",
    crate::graph_test::run_edge_constructor
);
REGISTER_MINI_TEST!(
    "Edge",
    "Json Roundtrip",
    crate::graph_test::run_edge_json_roundtrip
);
REGISTER_MINI_TEST!("Edge", "Vertices", crate::graph_test::run_edge_vertices);
REGISTER_MINI_TEST!("Edge", "Connects", crate::graph_test::run_edge_connects);
REGISTER_MINI_TEST!(
    "Edge",
    "Other Vertex",
    crate::graph_test::run_edge_other_vertex
);
REGISTER_MINI_TEST!(
    "Graph",
    "Constructor",
    crate::graph_test::run_graph_constructor
);
REGISTER_MINI_TEST!(
    "Graph",
    "Json Roundtrip",
    crate::graph_test::run_graph_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Graph",
    "Protobuf Roundtrip",
    crate::graph_test::run_graph_protobuf_roundtrip
);
REGISTER_MINI_TEST!("Graph", "Has Node", crate::graph_test::run_graph_has_node);
REGISTER_MINI_TEST!("Graph", "Has Edge", crate::graph_test::run_graph_has_edge);
REGISTER_MINI_TEST!("Graph", "Has Guid", crate::graph_test::run_graph_has_guid);
REGISTER_MINI_TEST!("Graph", "Add Node", crate::graph_test::run_graph_add_node);
REGISTER_MINI_TEST!("Graph", "Add Edge", crate::graph_test::run_graph_add_edge);
REGISTER_MINI_TEST!(
    "Graph",
    "Remove Node",
    crate::graph_test::run_graph_remove_node
);
REGISTER_MINI_TEST!(
    "Graph",
    "Remove Edge",
    crate::graph_test::run_graph_remove_edge
);
REGISTER_MINI_TEST!(
    "Graph",
    "Get Vertices",
    crate::graph_test::run_graph_get_vertices
);
REGISTER_MINI_TEST!("Graph", "Get Edges", crate::graph_test::run_graph_get_edges);
REGISTER_MINI_TEST!("Graph", "Neighbors", crate::graph_test::run_graph_neighbors);
REGISTER_MINI_TEST!(
    "Graph",
    "Number Of Vertices",
    crate::graph_test::run_graph_number_of_vertices
);
REGISTER_MINI_TEST!(
    "Graph",
    "Number Of Edges",
    crate::graph_test::run_graph_number_of_edges
);
REGISTER_MINI_TEST!("Graph", "Clear", crate::graph_test::run_graph_clear);
REGISTER_MINI_TEST!(
    "Graph",
    "Node Label",
    crate::graph_test::run_graph_node_label
);
REGISTER_MINI_TEST!(
    "Graph",
    "Edge Label",
    crate::graph_test::run_graph_edge_label
);
REGISTER_MINI_TEST!(
    "Graph",
    "Update Default Vertex Attributes",
    crate::graph_test::run_graph_update_default_vertex_attributes
);
REGISTER_MINI_TEST!(
    "Graph",
    "Update Default Edge Attributes",
    crate::graph_test::run_graph_update_default_edge_attributes
);
REGISTER_MINI_TEST!(
    "Graph",
    "Vertex Attribute",
    crate::graph_test::run_graph_vertex_attribute
);
REGISTER_MINI_TEST!(
    "Graph",
    "Set Vertex Attribute",
    crate::graph_test::run_graph_set_vertex_attribute
);
REGISTER_MINI_TEST!(
    "Graph",
    "Edge Attribute",
    crate::graph_test::run_graph_edge_attribute
);
REGISTER_MINI_TEST!(
    "Graph",
    "Set Edge Attribute",
    crate::graph_test::run_graph_set_edge_attribute
);
REGISTER_MINI_TEST!(
    "Graph",
    "Vertices Where",
    crate::graph_test::run_graph_vertices_where
);
REGISTER_MINI_TEST!(
    "Graph",
    "Edges Where",
    crate::graph_test::run_graph_edges_where
);
REGISTER_MINI_TEST!(
    "Graph",
    "Vertices Where Predicate",
    crate::graph_test::run_graph_vertices_where_predicate
);
REGISTER_MINI_TEST!(
    "Graph",
    "Edges Where Predicate",
    crate::graph_test::run_graph_edges_where_predicate
);
REGISTER_MINI_TEST!("Graph", "Bfs", crate::graph_test::run_graph_bfs);
REGISTER_MINI_TEST!("Graph", "Dfs", crate::graph_test::run_graph_dfs);
REGISTER_MINI_TEST!(
    "Graph",
    "Connected Components",
    crate::graph_test::run_graph_connected_components
);
REGISTER_MINI_TEST!(
    "Graph",
    "Shortest Path",
    crate::graph_test::run_graph_shortest_path
);
REGISTER_MINI_TEST!("Graph", "Has Cycle", crate::graph_test::run_graph_has_cycle);
REGISTER_MINI_TEST!(
    "Graph",
    "Cycle Basis",
    crate::graph_test::run_graph_cycle_basis
);
REGISTER_MINI_TEST!("Graph", "Take Node", crate::graph_test::run_graph_take_node);
REGISTER_MINI_TEST!("Graph", "Put Node", crate::graph_test::run_graph_put_node);
REGISTER_MINI_TEST!("Graph", "Renumber", crate::graph_test::run_graph_renumber);
