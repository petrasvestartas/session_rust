#[cfg(test)]
mod tests {
    use crate::edge::Edge;
    use crate::encoders::{json_dump, json_load};

    #[test]
    fn test_edge_json_roundtrip() {
        let edge = Edge::new(
            Some("test_edge".to_string()),
            Some("v0".to_string()),
            Some("v1".to_string()),
            Some("attribute".to_string()),
        );

        //   json_dumps()    │ String       │ to JSON string
        //   json_loads(s)   │ String       │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        let json_str = serde_json::to_string_pretty(&edge).unwrap();
        let loaded: Edge = serde_json::from_str(&json_str).unwrap();

        assert_eq!(loaded.name, "test_edge");
        assert_eq!(loaded.v0, "v0");
        assert_eq!(loaded.v1, "v1");
        assert_eq!(loaded.attribute, "attribute");

        // File I/O test
        json_dump(&edge, "serialization/test_edge.json", true).unwrap();
        let from_file: Edge = json_load("serialization/test_edge.json").unwrap();
        assert_eq!(from_file.name, edge.name);
    }
}
