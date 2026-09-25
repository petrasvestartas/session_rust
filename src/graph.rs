use serde::Deserialize;
use serde::Serialize;
use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::collections::VecDeque;
use std::fmt;
use std::sync::OnceLock;

/// Predicate over an edge key and its merged attributes.
pub type EdgePredicate<'a> = &'a dyn Fn((&str, &str), &BTreeMap<String, f64>) -> bool;

// ═══════════════════════════════════════════════════════════════════════════
// Vertex
// ═══════════════════════════════════════════════════════════════════════════

/// A graph vertex with a name, attribute string and integer index.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Vertex")]
pub struct Vertex {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,      // Vertex name, also the key in Graph::vertices.
    pub attribute: String, // Vertex attribute data as string.
    #[serde(default)]
    pub attributes: BTreeMap<String, f64>, // Name -> value, overriding the graph defaults.
    pub index: i32,        // Integer index of the vertex, assigned by Graph.
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_vertex".to_string(),
            attribute: String::new(),
            attributes: BTreeMap::new(),
            index: -1,
        }
    }
}

impl Vertex {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from name and attribute.
    pub fn new(name: &str, attribute: &str) -> Self {
        Self {
            name: name.to_string(),
            attribute: attribute.to_string(),
            ..Default::default()
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the GUID, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Vertex(guid, name, attribute, index)".
    pub fn str(&self) -> String {
        format!(
            "Vertex({}, {}, {}, {})",
            self.guid(),
            self.name,
            self.attribute,
            self.index
        )
    }
}

impl fmt::Display for Vertex {
    /// Write the string representation to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Edge
// ═══════════════════════════════════════════════════════════════════════════

/// A graph edge connecting two vertices by name.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Edge")]
pub struct Edge {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,      // Edge name.
    pub v0: String,        // First vertex name.
    pub v1: String,        // Second vertex name.
    pub attribute: String, // Edge attribute data as string.
    #[serde(default)]
    pub attributes: BTreeMap<String, f64>, // Name -> value, overriding the graph defaults.
    pub index: i32,        // Integer index of the edge, assigned by Graph.
}

impl Default for Edge {
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_edge".to_string(),
            v0: String::new(),
            v1: String::new(),
            attribute: String::new(),
            attributes: BTreeMap::new(),
            index: -1,
        }
    }
}

impl Edge {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from endpoints and attribute.
    pub fn new(v0: &str, v1: &str, attribute: &str) -> Self {
        Self {
            v0: v0.to_string(),
            v1: v1.to_string(),
            attribute: attribute.to_string(),
            ..Default::default()
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the GUID, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the GUID if it has not already been created.
    pub fn set_guid(&self, guid: String) {
        let _ = self.guid.set(guid);
    }

    /// Return the (v0, v1) tuple.
    pub fn vertices(&self) -> (String, String) {
        (self.v0.clone(), self.v1.clone())
    }

    /// Return whether this edge touches the given vertex.
    pub fn connects(&self, vertex_id: &str) -> bool {
        self.v0 == vertex_id || self.v1 == vertex_id
    }

    /// Return the other endpoint given one endpoint, empty if not connected.
    pub fn other_vertex(&self, vertex_id: &str) -> String {
        if self.v0 == vertex_id {
            return self.v1.clone();
        }

        if self.v1 == vertex_id {
            return self.v0.clone();
        }

        String::new()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Edge(guid, name, v0, v1, attribute)".
    pub fn str(&self) -> String {
        format!(
            "Edge({}, {}, {}, {}, {})",
            self.guid(),
            self.name,
            self.v0,
            self.v1,
            self.attribute
        )
    }
}

impl fmt::Display for Edge {
    /// Write the string representation to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Graph
// ═══════════════════════════════════════════════════════════════════════════

/// An undirected graph with string vertices, string labels and double attributes.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Graph")]
pub struct Graph {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,                                    // Graph name.
    pub vertex_count: i32,                               // Next available vertex index.
    pub edge_count: i32,                                 // Next available edge index.
    vertices: BTreeMap<String, Vertex>,                  // name -> Vertex.
    pub edges: BTreeMap<String, BTreeMap<String, Edge>>, // node_name -> {neighbor_name -> Edge}, every edge stored in both directions.
    #[serde(default)]
    pub default_vertex_attributes: BTreeMap<String, f64>, // Vertex attribute defaults.
    #[serde(default)]
    pub default_edge_attributes: BTreeMap<String, f64>, // Edge attribute defaults.
}

impl Default for Graph {
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_graph".to_string(),
            vertex_count: 0,
            edge_count: 0,
            vertices: BTreeMap::new(),
            edges: BTreeMap::new(),
            default_vertex_attributes: BTreeMap::new(),
            default_edge_attributes: BTreeMap::new(),
        }
    }
}

impl Graph {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct from name.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            ..Default::default()
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy GUID has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the GUID, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether a node with the given key exists.
    pub fn has_node(&self, key: &str) -> bool {
        self.vertices.contains_key(key)
    }

    /// Return whether an edge between the given endpoints exists.
    pub fn has_edge(&self, key: (&str, &str)) -> bool {
        match self.edges.get(key.0) {
            Some(neighbors) => neighbors.contains_key(key.1),
            None => false,
        }
    }

    /// Add a node and return its key.
    pub fn add_node(&mut self, key: &str, attribute: &str) -> String {
        if self.has_node(key) {
            return self.vertices[key].name.clone();
        }

        let mut vertex = Vertex::new(key, attribute);
        vertex.index = self.vertex_count;

        let name = vertex.name.clone();
        self.vertices.insert(key.to_string(), vertex);
        self.vertex_count += 1;

        name
    }

    /// Add an edge between u and v, creating missing nodes, and return (u, v).
    pub fn add_edge(&mut self, u: &str, v: &str, attribute: &str) -> (String, String) {
        if !self.has_node(u) {
            self.add_node(u, "");
        }

        if !self.has_node(v) {
            self.add_node(v, "");
        }

        if self.has_edge((u, v)) {
            self.edges.get_mut(u).unwrap().get_mut(v).unwrap().attribute = attribute.to_string();
            self.edges.get_mut(v).unwrap().get_mut(u).unwrap().attribute = attribute.to_string();

            return (u.to_string(), v.to_string());
        }

        let mut edge = Edge::new(u, v, attribute);
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

    /// Remove a node and all its edges; an unknown node is ignored.
    pub fn remove_node(&mut self, key: &str) {
        if !self.has_node(key) {
            return;
        }

        if let Some(neighbors) = self.edges.remove(key) {
            for neighbor in neighbors.keys() {
                if let Some(other) = self.edges.get_mut(neighbor) {
                    other.remove(key);
                }
            }
        }

        self.vertices.remove(key);
        self.reassign_indices();
        self.reassign_edge_indices();
    }

    /// Remove an edge, keeping its nodes.
    pub fn remove_edge(&mut self, edge: (&str, &str)) {
        if !self.has_edge(edge) {
            return;
        }

        let u = edge.0.to_string();
        let v = edge.1.to_string();

        self.edges.get_mut(&u).unwrap().remove(&v);
        self.edges.get_mut(&v).unwrap().remove(&u);
        self.reassign_edge_indices();
    }

    /// Renumber vertex indices 0, 1, 2, ... keeping their relative order.
    fn reassign_indices(&mut self) {
        let mut list: Vec<(i32, String)> = Vec::new();

        for (vertex_name, vertex) in &self.vertices {
            list.push((vertex.index, vertex_name.clone()));
        }

        list.sort();

        for (i, (_, name)) in list.iter().enumerate() {
            self.vertices.get_mut(name).unwrap().index = i as i32;
        }

        self.vertex_count = list.len() as i32;
    }

    /// Renumber edge indices 0, 1, 2, ... keeping their relative order.
    fn reassign_edge_indices(&mut self) {
        let mut list: Vec<(i32, String, String)> = Vec::new();

        for (u, neighbors) in &self.edges {
            for (v, edge) in neighbors {
                if u < v {
                    list.push((edge.index, u.clone(), v.clone()));
                }
            }
        }

        list.sort();

        for (i, (_, u, v)) in list.iter().enumerate() {
            self.edges.get_mut(u).unwrap().get_mut(v).unwrap().index = i as i32;
            self.edges.get_mut(v).unwrap().get_mut(u).unwrap().index = i as i32;
        }

        self.edge_count = list.len() as i32;
    }

    /// Return all vertices in the graph.
    pub fn get_vertices(&self) -> Vec<Vertex> {
        let mut result = Vec::new();

        for vertex in self.vertices.values() {
            result.push(vertex.clone());
        }

        result
    }

    /// Return all edges in the graph as (u, v) tuples, each once.
    pub fn get_edges(&self) -> Vec<(String, String)> {
        let mut result = Vec::new();

        for (u, neighbors) in &self.edges {
            for v in neighbors.keys() {
                if u < v {
                    result.push((u.clone(), v.clone()));
                }
            }
        }

        result
    }

    /// Return all neighbors of a node; panics for a node not in the graph.
    pub fn neighbors(&self, node: &str) -> Vec<String> {
        if !self.has_node(node) {
            panic!("Node {node} not in graph");
        }

        let mut result = Vec::new();

        if let Some(neighbors) = self.edges.get(node) {
            for neighbor in neighbors.keys() {
                result.push(neighbor.clone());
            }
        }

        result
    }

    /// Return incident edges as (other, attribute, forward); forward when node is the edge's v0.
    pub fn edges_of(&self, node: &str) -> Vec<(String, String, bool)> {
        let mut result = Vec::new();

        if let Some(neighbors) = self.edges.get(node) {
            for (other, edge) in neighbors {
                result.push((other.clone(), edge.attribute.clone(), edge.v0 == node));
            }
        }

        result
    }

    /// Return the number of vertices in the graph.
    pub fn number_of_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Return the number of edges in the graph.
    pub fn number_of_edges(&self) -> usize {
        let mut count = 0;

        for (u, neighbors) in &self.edges {
            for v in neighbors.keys() {
                if u < v {
                    count += 1;
                }
            }
        }

        count
    }

    /// Remove all vertices and edges.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.edges.clear();
        self.vertex_count = 0;
        self.edge_count = 0;
    }

    /// Get or set a node label (sets if value is given); None for an unknown node.
    pub fn node_label(&mut self, node: &str, value: Option<&str>) -> Option<String> {
        if !self.has_node(node) {
            return None;
        }

        if let Some(val) = value {
            self.vertices.get_mut(node).unwrap().attribute = val.to_string();
        }

        Some(self.vertices[node].attribute.clone())
    }

    /// Get or set an edge label (sets if value is given); None for an unknown edge.
    pub fn edge_label(&mut self, u: &str, v: &str, value: Option<&str>) -> Option<String> {
        if !self.has_edge((u, v)) {
            return None;
        }

        if let Some(val) = value {
            self.edges.get_mut(u).unwrap().get_mut(v).unwrap().attribute = val.to_string();
            self.edges.get_mut(v).unwrap().get_mut(u).unwrap().attribute = val.to_string();
        }

        Some(self.edges[u][v].attribute.clone())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Attribute API
    // ═══════════════════════════════════════════════════════════════════════════
    /// Merge attrs into the default vertex attributes.
    pub fn update_default_vertex_attributes(&mut self, attrs: &[(&str, f64)]) {
        for (name, value) in attrs {
            self.default_vertex_attributes
                .insert(name.to_string(), *value);
        }
    }

    /// Merge attrs into the default edge attributes.
    pub fn update_default_edge_attributes(&mut self, attrs: &[(&str, f64)]) {
        for (name, value) in attrs {
            self.default_edge_attributes
                .insert(name.to_string(), *value);
        }
    }

    /// Return the attribute of a vertex, falling back to the default; None when neither exists.
    pub fn vertex_attribute(&self, key: &str, name: &str) -> Option<f64> {
        let vertex = self.vertices.get(key)?;

        if let Some(value) = vertex.attributes.get(name) {
            return Some(*value);
        }

        self.default_vertex_attributes.get(name).copied()
    }

    /// Store an attribute on a vertex.
    pub fn set_vertex_attribute(&mut self, key: &str, name: &str, value: f64) {
        if let Some(vertex) = self.vertices.get_mut(key) {
            vertex.attributes.insert(name.to_string(), value);
        }
    }

    /// Return the attribute of an edge, falling back to the default; None when neither exists.
    pub fn edge_attribute(&self, edge: (&str, &str), name: &str) -> Option<f64> {
        let stored = self.edges.get(edge.0)?.get(edge.1)?;

        if let Some(value) = stored.attributes.get(name) {
            return Some(*value);
        }

        self.default_edge_attributes.get(name).copied()
    }

    /// Store an attribute on an edge, in both stored directions.
    pub fn set_edge_attribute(&mut self, edge: (&str, &str), name: &str, value: f64) {
        if !self.has_edge(edge) {
            return;
        }

        let (u, v) = edge;

        self.edges
            .get_mut(u)
            .unwrap()
            .get_mut(v)
            .unwrap()
            .attributes
            .insert(name.to_string(), value);
        self.edges
            .get_mut(v)
            .unwrap()
            .get_mut(u)
            .unwrap()
            .attributes
            .insert(name.to_string(), value);
    }

    /// Return the vertices whose attributes match every (name, value) condition.
    pub fn vertices_where(&self, conditions: &[(&str, f64)]) -> Vec<String> {
        let mut result = Vec::new();

        for key in self.vertices.keys() {
            let mut matched = true;

            for (name, value) in conditions {
                if self.vertex_attribute(key, name) != Some(*value) {
                    matched = false;
                }
            }

            if matched {
                result.push(key.clone());
            }
        }

        result
    }

    /// Return the edges whose attributes match every (name, value) condition.
    pub fn edges_where(&self, conditions: &[(&str, f64)]) -> Vec<(String, String)> {
        let mut result = Vec::new();

        for edge in self.get_edges() {
            let mut matched = true;

            for (name, value) in conditions {
                if self.edge_attribute((&edge.0, &edge.1), name) != Some(*value) {
                    matched = false;
                }
            }

            if matched {
                result.push(edge);
            }
        }

        result
    }

    /// Return the vertices for which pred(key, attributes) is true.
    pub fn vertices_where_predicate(
        &self,
        pred: &dyn Fn(&str, &BTreeMap<String, f64>) -> bool,
    ) -> Vec<String> {
        let mut result = Vec::new();

        for (key, vertex) in &self.vertices {
            let mut attributes = self.default_vertex_attributes.clone();

            for (name, value) in &vertex.attributes {
                attributes.insert(name.clone(), *value);
            }

            if pred(key, &attributes) {
                result.push(key.clone());
            }
        }

        result
    }

    /// Return the edges for which pred(edge, attributes) is true.
    pub fn edges_where_predicate(&self, pred: EdgePredicate) -> Vec<(String, String)> {
        let mut result = Vec::new();

        for edge in self.get_edges() {
            let stored = &self.edges[&edge.0][&edge.1];
            let mut attributes = self.default_edge_attributes.clone();

            for (name, value) in &stored.attributes {
                attributes.insert(name.clone(), *value);
            }

            if pred((&edge.0, &edge.1), &attributes) {
                result.push(edge);
            }
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Algorithms
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the breadth-first order from start.
    pub fn bfs(&self, start: &str) -> Vec<String> {
        let mut result = Vec::new();

        if !self.has_node(start) {
            return result;
        }

        let mut visited = BTreeSet::new();
        let mut queue = VecDeque::new();
        queue.push_back(start.to_string());
        visited.insert(start.to_string());

        while let Some(node) = queue.pop_front() {
            result.push(node.clone());

            for neighbor in self.neighbors(&node) {
                if visited.contains(&neighbor) {
                    continue;
                }

                visited.insert(neighbor.clone());
                queue.push_back(neighbor);
            }
        }

        result
    }

    /// Return the depth-first order from start.
    pub fn dfs(&self, start: &str) -> Vec<String> {
        let mut result = Vec::new();

        if !self.has_node(start) {
            return result;
        }

        let mut visited = BTreeSet::new();
        let mut stack = vec![start.to_string()];

        while let Some(node) = stack.pop() {
            if visited.contains(&node) {
                continue;
            }

            visited.insert(node.clone());
            result.push(node.clone());

            let nbrs = self.neighbors(&node);

            for i in (0..nbrs.len()).rev() {
                if !visited.contains(&nbrs[i]) {
                    stack.push(nbrs[i].clone());
                }
            }
        }

        result
    }

    /// Return the connected components as sorted node name lists.
    pub fn connected_components(&self) -> Vec<Vec<String>> {
        let mut visited = BTreeSet::new();
        let mut components = Vec::new();

        for vertex_name in self.vertices.keys() {
            if visited.contains(vertex_name) {
                continue;
            }

            let mut component = self.bfs(vertex_name);

            for node in &component {
                visited.insert(node.clone());
            }

            component.sort();
            components.push(component);
        }

        components
    }

    /// Return whether the graph has at most one connected component.
    pub fn is_connected(&self) -> bool {
        self.connected_components().len() <= 1
    }

    /// Return the number of connected components.
    pub fn number_connected_components(&self) -> usize {
        self.connected_components().len()
    }

    /// Return the shortest path between u and v, empty if disconnected.
    pub fn shortest_path(&self, u: &str, v: &str) -> Vec<String> {
        let mut path = Vec::new();

        if !self.has_node(u) || !self.has_node(v) {
            return path;
        }

        if u == v {
            return vec![u.to_string()];
        }

        let mut parent: BTreeMap<String, String> = BTreeMap::new();
        parent.insert(u.to_string(), String::new());

        let mut queue = VecDeque::new();
        queue.push_back(u.to_string());

        while let Some(node) = queue.pop_front() {
            for neighbor in self.neighbors(&node) {
                if parent.contains_key(&neighbor) {
                    continue;
                }

                parent.insert(neighbor.clone(), node.clone());

                if neighbor == v {
                    let mut current = v.to_string();

                    while current != u {
                        path.push(current.clone());
                        current = parent[&current].clone();
                    }

                    path.push(u.to_string());
                    path.reverse();

                    return path;
                }

                queue.push_back(neighbor);
            }
        }

        path
    }

    /// Return the length of the shortest path between u and v, -1 if disconnected.
    pub fn shortest_path_length(&self, u: &str, v: &str) -> i32 {
        let path = self.shortest_path(u, v);

        if path.is_empty() {
            return -1;
        }

        path.len() as i32 - 1
    }

    /// Return whether the graph contains a cycle.
    pub fn has_cycle(&self) -> bool {
        let mut visited = BTreeSet::new();

        for vertex_name in self.vertices.keys() {
            if visited.contains(vertex_name) {
                continue;
            }

            let mut parent: BTreeMap<String, String> = BTreeMap::new();
            parent.insert(vertex_name.clone(), String::new());

            let mut queue = VecDeque::new();
            queue.push_back(vertex_name.clone());
            visited.insert(vertex_name.clone());

            while let Some(node) = queue.pop_front() {
                for neighbor in self.neighbors(&node) {
                    if !visited.contains(&neighbor) {
                        visited.insert(neighbor.clone());
                        parent.insert(neighbor.clone(), node.clone());
                        queue.push_back(neighbor);
                    } else if parent[&node] != neighbor {
                        return true;
                    }
                }
            }
        }

        false
    }

    /// Return a basis of fundamental cycles.
    pub fn cycle_basis(&self) -> Vec<Vec<String>> {
        let mut result = Vec::new();
        let mut order: BTreeMap<String, i32> = BTreeMap::new();
        let mut parent: BTreeMap<String, String> = BTreeMap::new();
        let mut timer = 0;

        for vertex_name in self.vertices.keys() {
            if order.contains_key(vertex_name) {
                continue;
            }

            parent.insert(vertex_name.clone(), String::new());
            order.insert(vertex_name.clone(), timer);
            timer += 1;

            let mut stack: Vec<(String, String, Vec<String>, usize)> = Vec::new();
            stack.push((
                vertex_name.clone(),
                String::new(),
                self.neighbors(vertex_name),
                0,
            ));

            while let Some(frame) = stack.last_mut() {
                let u = frame.0.clone();
                let p = frame.1.clone();

                if frame.3 >= frame.2.len() {
                    stack.pop();
                    continue;
                }

                let v = frame.2[frame.3].clone();
                frame.3 += 1;

                if !order.contains_key(&v) {
                    parent.insert(v.clone(), u.clone());
                    order.insert(v.clone(), timer);
                    timer += 1;
                    stack.push((v.clone(), u, self.neighbors(&v), 0));
                } else if v != p && order[&v] < order[&u] {
                    let mut cycle = Vec::new();
                    let mut node = u;

                    while node != v {
                        cycle.push(node.clone());
                        node = parent[&node].clone();
                    }

                    cycle.push(v);
                    result.push(cycle);
                }
            }
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let mut vertices_json = Vec::new();

        for vertex in self.vertices.values() {
            vertices_json.push(serde_json::to_value(vertex)?);
        }

        let mut edges_json = Vec::new();

        for (u, neighbors) in &self.edges {
            for (v, edge) in neighbors {
                if u < v {
                    edges_json.push(serde_json::to_value(edge)?);
                }
            }
        }

        let data = serde_json::json!({
            "default_edge_attributes": self.default_edge_attributes,
            "default_vertex_attributes": self.default_vertex_attributes,
            "edge_count": self.edge_count,
            "edges": edges_json,
            "guid": self.guid(),
            "name": self.name,
            "type": "Graph",
            "vertex_count": self.vertex_count,
            "vertices": vertices_json,
        });

        crate::file_encoders::sorted_json_string(&data)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data: serde_json::Value = serde_json::from_str(json_data)?;
        let mut graph = Graph::new(data["name"].as_str().unwrap_or("my_graph"));
        graph.set_guid(data["guid"].as_str().unwrap_or("").to_string());
        graph.vertex_count = data["vertex_count"].as_i64().unwrap_or(0) as i32;
        graph.edge_count = data["edge_count"].as_i64().unwrap_or(0) as i32;

        if let Some(defaults) = data.get("default_edge_attributes") {
            graph.default_edge_attributes = serde_json::from_value(defaults.clone())?;
        }

        if let Some(defaults) = data.get("default_vertex_attributes") {
            graph.default_vertex_attributes = serde_json::from_value(defaults.clone())?;
        }

        if let Some(vertices) = data["vertices"].as_array() {
            for vertex_data in vertices {
                let vertex: Vertex = serde_json::from_value(vertex_data.clone())?;
                graph.vertices.insert(vertex.name.clone(), vertex);
            }
        }

        if let Some(edges) = data["edges"].as_array() {
            for edge_data in edges {
                let edge: Edge = serde_json::from_value(edge_data.clone())?;
                graph
                    .edges
                    .entry(edge.v0.clone())
                    .or_default()
                    .insert(edge.v1.clone(), edge.clone());
                graph
                    .edges
                    .entry(edge.v1.clone())
                    .or_default()
                    .insert(edge.v0.clone(), edge);
            }
        }

        Ok(graph)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.jsondump()?)?;

        Ok(())
    }

    /// Read JSON from a file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message, each edge once.
    pub fn to_proto(&self) -> crate::proto::Graph {
        let mut proto = crate::proto::Graph {
            name: self.name.clone(),
            guid: String::new(),
            vertices: BTreeMap::new(),
            edges: Vec::new(),
            vertex_count: self.vertex_count,
            edge_count: self.edge_count,
            default_vertex_attributes: self.default_vertex_attributes.clone(),
            default_edge_attributes: self.default_edge_attributes.clone(),
        };

        if self.has_guid() {
            proto.guid = self.guid().to_string();
        }

        for (vertex_name, vertex) in &self.vertices {
            let mut v = crate::proto::Vertex {
                name: vertex.name.clone(),
                guid: String::new(),
                attribute: vertex.attribute.clone(),
                index: vertex.index,
                attributes: vertex.attributes.clone(),
            };

            if vertex.has_guid() {
                v.guid = vertex.guid().to_string();
            }

            proto.vertices.insert(vertex_name.clone(), v);
        }

        for (u, neighbors) in &self.edges {
            for (v, edge) in neighbors {
                if u > v {
                    continue;
                }

                let mut e = crate::proto::Edge {
                    guid: String::new(),
                    name: edge.name.clone(),
                    v0: edge.v0.clone(),
                    v1: edge.v1.clone(),
                    attribute: edge.attribute.clone(),
                    index: edge.index,
                    attributes: edge.attributes.clone(),
                };

                if edge.has_guid() {
                    e.guid = edge.guid().to_string();
                }

                proto.edges.push(e);
            }
        }

        proto
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Graph) -> Self {
        let mut graph = Graph::new(&proto.name);

        if !proto.guid.is_empty() {
            graph.set_guid(proto.guid.clone());
        }

        graph.vertex_count = proto.vertex_count;
        graph.edge_count = proto.edge_count;
        graph.default_vertex_attributes = proto.default_vertex_attributes.clone();
        graph.default_edge_attributes = proto.default_edge_attributes.clone();

        for (vertex_name, v) in &proto.vertices {
            let mut vertex = Vertex::new(&v.name, &v.attribute);
            vertex.set_guid(v.guid.clone());
            vertex.index = v.index;
            vertex.attributes = v.attributes.clone();
            graph.vertices.insert(vertex_name.clone(), vertex);
        }

        for e in &proto.edges {
            let mut edge = Edge::new(&e.v0, &e.v1, &e.attribute);
            edge.name = e.name.clone();
            edge.set_guid(e.guid.clone());
            edge.index = e.index;
            edge.attributes = e.attributes.clone();
            graph
                .edges
                .entry(e.v0.clone())
                .or_default()
                .insert(e.v1.clone(), edge.clone());
            graph
                .edges
                .entry(e.v1.clone())
                .or_default()
                .insert(e.v0.clone(), edge);
        }

        graph
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Ok(Self::from_proto(crate::proto::Graph::decode(data)?))
    }

    /// Write protobuf bytes to a file.
    pub fn pb_dump(&self, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filename, self.pb_dumps())?;

        Ok(())
    }

    /// Read protobuf bytes from a file.
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filename)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// "<Graph with V vertices, E edges: name>"
    pub fn str(&self) -> String {
        format!(
            "<Graph with {} vertices, {} edges: {}>",
            self.vertex_count, self.edge_count, self.name
        )
    }

    /// "Graph(guid, name, vertex_count, edge_count)"
    pub fn repr(&self) -> String {
        format!(
            "Graph({}, {}, {}, {})",
            self.guid(),
            self.name,
            self.vertex_count,
            self.edge_count
        )
    }
}

impl fmt::Display for Graph {
    /// Write the string representation to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}
