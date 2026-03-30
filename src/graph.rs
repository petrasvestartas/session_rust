use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;

/// A graph vertex with a unique identifier and attribute string.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Vertex")]
pub struct Vertex {
    /// The unique identifier of the vertex.
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    /// The name of the vertex.
    pub name: String,
    /// Vertex attribute data as string.
    pub attribute: String,
    /// Integer index for the vertex. Set internally by Graph.
    pub index: i32,
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            name: "my_vertex".to_string(),
            guid: std::sync::OnceLock::new(),
            attribute: String::new(),
            index: -1,
        }
    }
}

impl fmt::Display for Graph {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Graph({}, {}, vertices={}, edges={})",
            self.name, self.guid(), self.vertex_count, self.edge_count
        )
    }
}

impl fmt::Display for Vertex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Vertex({}, {}, attr={}, index={})",
            self.name, self.guid(), self.attribute, self.index
        )
    }
}

impl fmt::Display for Edge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Edge({}, {}, {} -> {}, attr={}, index={})",
            self.name, self.guid(), self.v0, self.v1, self.attribute, self.index
        )
    }
}

impl Vertex {
    /// Initialize a new Vertex.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new(name: Option<String>, attribute: Option<String>) -> Self {
        Self {
            name: name.unwrap_or_default(),
            attribute: attribute.unwrap_or_default(),
            ..Default::default()
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert the Vertex to a JSON-serializable string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Create Vertex from JSON string data.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }
}

/// A graph edge with a unique identifier and attribute string.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Edge")]
pub struct Edge {
    /// The unique identifier of the edge.
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    /// The name of the edge.
    pub name: String,
    /// The first vertex of the edge.
    pub v0: String,
    /// The second vertex of the edge.
    pub v1: String,
    /// Edge attribute data as string.
    pub attribute: String,
    /// Integer index for the edge.
    pub index: i32,
}

impl Default for Edge {
    fn default() -> Self {
        Self {
            name: "my_edge".to_string(),
            guid: std::sync::OnceLock::new(),
            v0: String::new(),
            v1: String::new(),
            attribute: String::new(),
            index: -1,
        }
    }
}

impl Edge {
    /// Initialize a new Edge.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new(
        name: Option<String>,
        v0: Option<String>,
        v1: Option<String>,
        attribute: Option<String>,
    ) -> Self {
        Self {
            name: name.unwrap_or_default(),
            v0: v0.unwrap_or_default(),
            v1: v1.unwrap_or_default(),
            attribute: attribute.unwrap_or_default(),
            ..Default::default()
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert the Edge to a JSON-serializable string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Create Edge from JSON string data.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Get the edge vertices as a tuple.
    pub fn vertices(&self) -> (String, String) {
        (self.v0.clone(), self.v1.clone())
    }

    /// Check if this edge connects to a given vertex.
    pub fn connects(&self, vertex_id: &str) -> bool {
        self.v0 == vertex_id || self.v1 == vertex_id
    }

    /// Get the other vertex ID connected by this edge.
    pub fn other_vertex(&self, vertex_id: &str) -> String {
        if self.v0 == vertex_id {
            self.v1.clone()
        } else {
            self.v0.clone()
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Graph")]
pub struct Graph {
    // Public fields, similar to C++
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub vertex_count: i32,
    pub edge_count: i32,

    // Private fields (by Rust's default visibility)
    // std::map translates to HashMap in Rust.
    vertices: HashMap<String, Vertex>,
    pub edges: HashMap<String, HashMap<String, Edge>>,
}

impl Default for Graph {
    fn default() -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: "my_graph".to_string(),
            vertex_count: 0,
            edge_count: 0,
            vertices: HashMap::new(),
            edges: HashMap::new(),
        }
    }
}

impl Graph {
    /// Creates a new, empty `Graph` with a specific name.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            ..Default::default()
        }
    }

    /// Checks if a node exists in the graph.
    pub fn has_node(&self, key: &str) -> bool {
        self.vertices.contains_key(key)
    }

    /// Adds a node to the graph.
    pub fn add_node(&mut self, key: &str, attribute: &str) -> String {
        if self.has_node(key) {
            return self.vertices.get(key).unwrap().name.clone();
        }

        let mut vertex = Vertex::new(Some(key.to_string()), Some(attribute.to_string()));
        vertex.index = self.vertices.len() as i32;
        self.vertices.insert(key.to_string(), vertex.clone());
        self.vertex_count = self.vertices.len() as i32;
        vertex.name
    }

    /// Adds an edge between u and v.
    pub fn add_edge(&mut self, u: &str, v: &str, attribute: &str) -> (String, String) {
        // Add vertices if they don't exist
        if !self.has_node(u) {
            self.add_node(u, "");
        }
        if !self.has_node(v) {
            self.add_node(v, "");
        }

        if self.has_edge((u, v)) {
            return (u.to_string(), v.to_string());
        }

        let mut edge = Edge::new(
            Some("my_edge".to_string()),
            Some(u.to_string()),
            Some(v.to_string()),
            Some(attribute.to_string()),
        );
        edge.index = self.edge_count;

        self.edges
            .entry(u.to_string())
            .or_default()
            .insert(v.to_string(), edge.clone());
        self.edges
            .entry(v.to_string())
            .or_default()
            .insert(u.to_string(), edge);

        self.edge_count += 1;

        (u.to_string(), v.to_string())
    }

    /// Checks if an edge exists in the graph.
    pub fn has_edge(&self, edge: (&str, &str)) -> bool {
        let (u, v) = edge;
        self.edges
            .get(u)
            .is_some_and(|neighbors| neighbors.contains_key(v))
    }

    /// Gets the number of vertices in the graph.
    pub fn number_of_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Gets the number of edges in the graph.
    pub fn number_of_edges(&self) -> usize {
        let mut count = 0;
        let mut seen = std::collections::HashSet::new();
        for (u, neighbors) in &self.edges {
            for v in neighbors.keys() {
                let edge = if u < v {
                    (u.clone(), v.clone())
                } else {
                    (v.clone(), u.clone())
                };
                if seen.insert(edge) {
                    count += 1;
                }
            }
        }
        count
    }

    /// Gets all vertices in the graph.
    pub fn get_vertices(&self) -> Vec<Vertex> {
        self.vertices.values().cloned().collect()
    }

    /// Gets all edges in the graph as tuples of vertex names.
    pub fn get_edges(&self) -> Vec<(String, String)> {
        let mut result = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for (u, neighbors) in &self.edges {
            for v in neighbors.keys() {
                let edge = if u < v {
                    (u.clone(), v.clone())
                } else {
                    (v.clone(), u.clone())
                };
                if seen.insert(edge.clone()) {
                    result.push(edge);
                }
            }
        }
        result
    }

    /// Gets all neighbors of a node.
    pub fn neighbors(&self, node: &str) -> Vec<String> {
        self.edges
            .get(node)
            .map_or(Vec::new(), |neighbors| neighbors.keys().cloned().collect())
    }

    /// Gets all neighbors of a node (API compatibility method).
    pub fn get_neighbors(&self, node: &str) -> Vec<String> {
        self.neighbors(node)
    }

    /// Removes a node and all its edges from the graph.
    pub fn remove_node(&mut self, key: &str) {
        if !self.has_node(key) {
            return;
        }

        if let Some(neighbors) = self.edges.remove(key) {
            for neighbor_key in neighbors.keys() {
                if let Some(neighbor_edges) = self.edges.get_mut(neighbor_key) {
                    neighbor_edges.remove(key);
                }
            }
        }

        self.vertices.remove(key);
        self.vertex_count = self.vertices.len() as i32;
        self.edge_count = self.number_of_edges() as i32;
    }

    /// Removes an edge from the graph.
    pub fn remove_edge(&mut self, edge: (&str, &str)) {
        let (u, v) = edge;
        let mut edge_removed = false;

        if let Some(neighbors) = self.edges.get_mut(u) {
            if neighbors.remove(v).is_some() {
                edge_removed = true;
            }
        }
        if let Some(neighbors) = self.edges.get_mut(v) {
            neighbors.remove(u);
        }

        if edge_removed {
            self.edge_count = self.number_of_edges() as i32;
        }
    }

    /// Removes all vertices and edges from the graph.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.edges.clear();
        self.vertex_count = 0;
        self.edge_count = 0;
    }

    /// Get or set node attribute.
    pub fn node_attribute(&mut self, node: &str, value: Option<&str>) -> Option<String> {
        if !self.has_node(node) {
            return None;
        }
        let vertex = self.vertices.get_mut(node).unwrap();
        if let Some(val) = value {
            vertex.attribute = val.to_string();
            Some(vertex.attribute.clone())
        } else {
            Some(vertex.attribute.clone())
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Graph to a JSON string.
    pub fn jsondump(&self) -> Result<String, serde_json::Error> {
        // Convert vertices to array, sorted by index to ensure consistent order
        let mut vertices: Vec<&Vertex> = self.vertices.values().collect();
        vertices.sort_by_key(|v| v.index);

        // Convert edges to array (store each edge only once), sorted by index
        let mut edges = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for (u, neighbors) in &self.edges {
            for (v, edge) in neighbors {
                let edge_tuple = if u < v { (u, v) } else { (v, u) };
                if seen.insert(edge_tuple) {
                    edges.push(edge);
                }
            }
        }
        edges.sort_by_key(|e| e.index);

        let json_obj = serde_json::json!({
            "type": "Graph",
            "name": self.name,
            "guid": self.guid(),
            "vertices": vertices,
            "edges": edges,
            "vertex_count": self.vertex_count,
            "edge_count": self.edge_count
        });

        serde_json::to_string_pretty(&json_obj)
    }

    /// Deserializes a Graph from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, serde_json::Error> {
        let json_obj: serde_json::Value = serde_json::from_str(json_data)?;

        let mut graph = Graph::new(json_obj["name"].as_str().unwrap_or("my_graph"));
        graph.set_guid(json_obj["guid"].as_str().unwrap_or("").to_string());
        graph.vertex_count = json_obj["vertex_count"].as_i64().unwrap_or(0) as i32;
        graph.edge_count = json_obj["edge_count"].as_i64().unwrap_or(0) as i32;

        // Restore vertices
        if let Some(vertices_array) = json_obj["vertices"].as_array() {
            for vertex_data in vertices_array {
                let vertex: Vertex = serde_json::from_value(vertex_data.clone())?;
                graph.vertices.insert(vertex.name.clone(), vertex);
            }
        }

        // Restore edges
        if let Some(edges_array) = json_obj["edges"].as_array() {
            for edge_data in edges_array {
                let edge: Edge = serde_json::from_value(edge_data.clone())?;
                let u = &edge.v0;
                let v = &edge.v1;

                graph
                    .edges
                    .entry(u.clone())
                    .or_default()
                    .insert(v.clone(), edge.clone());
                graph
                    .edges
                    .entry(v.clone())
                    .or_default()
                    .insert(u.clone(), edge);
            }
        }

        Ok(graph)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    pub fn json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json_data = self.jsondump()?;
        std::fs::write(filepath, json_data)?;
        Ok(())
    }

    pub fn json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_data = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_data).map_err(|e| e.into())
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        use std::collections::HashMap as ProtoMap;

        let mut proto_vertices: ProtoMap<String, crate::proto::Vertex> = ProtoMap::new();
        for (name, vertex) in &self.vertices {
            proto_vertices.insert(name.clone(), crate::proto::Vertex {
                name: vertex.name.clone(),
                guid: vertex.guid().to_string(),
                attribute: vertex.attribute.clone(),
                index: vertex.index,
            });
        }

        let mut proto_edges = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for (u, neighbors) in &self.edges {
            for (v, edge) in neighbors {
                let key = if u < v { (u.clone(), v.clone()) } else { (v.clone(), u.clone()) };
                if seen.insert(key) {
                    proto_edges.push(crate::proto::Edge {
                        guid: edge.guid().to_string(),
                        name: edge.name.clone(),
                        v0: edge.v0.clone(),
                        v1: edge.v1.clone(),
                        attribute: edge.attribute.clone(),
                        index: edge.index,
                    });
                }
            }
        }

        let proto = crate::proto::Graph {
            name: self.name.clone(),
            guid: self.guid().to_string(),
            vertices: proto_vertices,
            edges: proto_edges,
            vertex_count: self.vertex_count,
            edge_count: self.edge_count,
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Graph::decode(data)?;
        let mut graph = Graph::new(&proto.name);
        graph.set_guid(proto.guid.clone());
        graph.vertex_count = proto.vertex_count;
        graph.edge_count = proto.edge_count;

        for (name, v) in &proto.vertices {
            let vertex = Vertex {
                name: v.name.clone(),
                attribute: v.attribute.clone(),
                index: v.index,
                ..Default::default()
            };
            vertex.set_guid(v.guid.clone());
            graph.vertices.insert(name.clone(), vertex);
        }

        for e in &proto.edges {
            let edge = Edge {
                name: e.name.clone(),
                v0: e.v0.clone(),
                v1: e.v1.clone(),
                attribute: e.attribute.clone(),
                index: e.index,
                ..Default::default()
            };
            edge.set_guid(e.guid.clone());
            graph.edges.entry(e.v0.clone()).or_default().insert(e.v1.clone(), edge.clone());
            graph.edges.entry(e.v1.clone()).or_default().insert(e.v0.clone(), edge);
        }

        Ok(graph)
    }

    pub fn pb_dump(&self, path: &str) {
        std::fs::write(path, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(path: &str) -> Self {
        let data = std::fs::read(path).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Algorithms
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn bfs(&self, start: &str) -> Vec<String> {
        if !self.has_node(start) {
            return Vec::new();
        }
        let mut visited = std::collections::HashSet::new();
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start.to_string());
        visited.insert(start.to_string());
        let mut result = Vec::new();
        while let Some(node) = queue.pop_front() {
            result.push(node.clone());
            let mut nbrs = self.neighbors(&node);
            nbrs.sort();
            for neighbor in nbrs {
                if !visited.contains(&neighbor) {
                    visited.insert(neighbor.clone());
                    queue.push_back(neighbor);
                }
            }
        }
        result
    }

    pub fn dfs(&self, start: &str) -> Vec<String> {
        if !self.has_node(start) {
            return Vec::new();
        }
        let mut visited = std::collections::HashSet::new();
        let mut result = Vec::new();
        let mut stack = vec![start.to_string()];
        while let Some(node) = stack.pop() {
            if visited.contains(&node) {
                continue;
            }
            visited.insert(node.clone());
            result.push(node.clone());
            let mut nbrs = self.neighbors(&node);
            nbrs.sort();
            nbrs.reverse();
            for neighbor in nbrs {
                if !visited.contains(&neighbor) {
                    stack.push(neighbor);
                }
            }
        }
        result
    }

    pub fn connected_components(&self) -> Vec<Vec<String>> {
        let mut visited = std::collections::HashSet::new();
        let mut comps = Vec::new();
        let mut keys: Vec<String> = self.vertices.keys().cloned().collect();
        keys.sort();
        for start in keys {
            if visited.contains(&start) {
                continue;
            }
            let comp = self.bfs(&start);
            for n in &comp {
                visited.insert(n.clone());
            }
            let mut sorted_comp = comp;
            sorted_comp.sort();
            comps.push(sorted_comp);
        }
        comps
    }

    pub fn is_connected(&self) -> bool {
        self.connected_components().len() <= 1
    }

    pub fn number_connected_components(&self) -> usize {
        self.connected_components().len()
    }

    pub fn shortest_path(&self, u: &str, v: &str) -> Vec<String> {
        if !self.has_node(u) || !self.has_node(v) {
            return Vec::new();
        }
        if u == v {
            return vec![u.to_string()];
        }
        let mut parent: std::collections::HashMap<String, Option<String>> = std::collections::HashMap::new();
        parent.insert(u.to_string(), None);
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(u.to_string());
        while let Some(node) = queue.pop_front() {
            let mut nbrs = self.neighbors(&node);
            nbrs.sort();
            for neighbor in nbrs {
                if !parent.contains_key(&neighbor) {
                    parent.insert(neighbor.clone(), Some(node.clone()));
                    if neighbor == v {
                        let mut path = Vec::new();
                        let mut cur = v.to_string();
                        loop {
                            path.push(cur.clone());
                            match parent.get(&cur).unwrap() {
                                Some(p) => cur = p.clone(),
                                None => break,
                            }
                        }
                        path.reverse();
                        return path;
                    }
                    queue.push_back(neighbor);
                }
            }
        }
        Vec::new()
    }

    pub fn shortest_path_length(&self, u: &str, v: &str) -> i32 {
        let path = self.shortest_path(u, v);
        if path.is_empty() {
            return -1;
        }
        (path.len() as i32) - 1
    }

    pub fn has_cycle(&self) -> bool {
        let mut visited = std::collections::HashSet::new();
        let mut keys: Vec<String> = self.vertices.keys().cloned().collect();
        keys.sort();
        for start in keys {
            if visited.contains(&start) {
                continue;
            }
            let mut parent: std::collections::HashMap<String, String> = std::collections::HashMap::new();
            parent.insert(start.clone(), String::new());
            let mut queue = std::collections::VecDeque::new();
            queue.push_back(start.clone());
            visited.insert(start.clone());
            while let Some(node) = queue.pop_front() {
                let nbrs = self.neighbors(&node);
                for neighbor in nbrs {
                    if !visited.contains(&neighbor) {
                        visited.insert(neighbor.clone());
                        parent.insert(neighbor.clone(), node.clone());
                        queue.push_back(neighbor);
                    } else if parent.get(&node).map_or(true, |p| p != &neighbor) {
                        return true;
                    }
                }
            }
        }
        false
    }

    pub fn cycle_basis(&self) -> Vec<Vec<String>> {
        let mut result = Vec::new();
        let mut disc: std::collections::HashMap<String, i32> = std::collections::HashMap::new();
        let mut par: std::collections::HashMap<String, String> = std::collections::HashMap::new();
        let mut timer = 0i32;
        let mut keys: Vec<String> = self.vertices.keys().cloned().collect();
        keys.sort();
        for start in keys {
            if disc.contains_key(&start) {
                continue;
            }
            par.insert(start.clone(), String::new());
            disc.insert(start.clone(), timer);
            timer += 1;
            let mut nbrs = self.neighbors(&start);
            nbrs.sort();
            let mut stk: Vec<(String, String, Vec<String>, usize)> = Vec::new();
            stk.push((start.clone(), String::new(), nbrs, 0));
            while !stk.is_empty() {
                let (u, p, nbrs_u, idx) = stk.last().unwrap().clone();
                if idx < nbrs_u.len() {
                    let v = nbrs_u[idx].clone();
                    stk.last_mut().unwrap().3 += 1;
                    if !disc.contains_key(&v) {
                        par.insert(v.clone(), u.clone());
                        disc.insert(v.clone(), timer);
                        timer += 1;
                        let mut v_nbrs = self.neighbors(&v);
                        v_nbrs.sort();
                        stk.push((v, u.clone(), v_nbrs, 0));
                    } else if v != p && disc[&v] < disc[&u] {
                        let mut cycle = Vec::new();
                        let mut node = u.clone();
                        while node != v {
                            cycle.push(node.clone());
                            node = par[&node].clone();
                        }
                        cycle.push(v);
                        result.push(cycle);
                    }
                } else {
                    stk.pop();
                }
            }
        }
        result
    }

    /// Get or set edge attribute.
    pub fn edge_attribute(&mut self, u: &str, v: &str, value: Option<&str>) -> Option<String> {
        if !self.has_edge((u, v)) {
            return None;
        }
        if let Some(val) = value {
            let new_attr = val.to_string();
            if let Some(neighbors) = self.edges.get_mut(u) {
                if let Some(edge) = neighbors.get_mut(v) {
                    edge.attribute = new_attr.clone();
                }
            }
            if let Some(neighbors) = self.edges.get_mut(v) {
                if let Some(edge) = neighbors.get_mut(u) {
                    edge.attribute = new_attr.clone();
                }
            }
            Some(new_attr)
        } else {
            self.edges
                .get(u)
                .and_then(|neighbors| neighbors.get(v))
                .map(|edge| edge.attribute.clone())
        }
    }
}

#[cfg(test)]
#[path = "graph_test.rs"]
mod graph_test;
