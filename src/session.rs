use crate::{
    BRep, Element, Obb, Graph, Line, Mesh, Objects, Plane, Point, PointCloud, Polyline,
    Tolerance, Tree, TreeNode, BVH,
};
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt;
use std::fs;
use std::rc::Rc;
use uuid::Uuid;

/// Enum representing all possible geometry types in a Session.
/// This is equivalent to C++'s std::variant<...> for heterogeneous geometry storage.
#[derive(Debug, Clone)]
pub enum Geometry {
    Obb(Obb),
    BRep(BRep),
    Element(Element),
    Line(Line),
    Mesh(Mesh),
    Plane(Plane),
    Point(Point),
    PointCloud(PointCloud),
    Polyline(Polyline),
}

impl Geometry {
    /// Get the GUID of the geometry object
    pub fn guid(&self) -> &str {
        match self {
            Geometry::Obb(g) => &g.guid,
            Geometry::BRep(g) => &g.guid,
            Geometry::Element(g) => &g.guid,
            Geometry::Line(g) => &g.guid,
            Geometry::Mesh(g) => &g.guid,
            Geometry::Plane(g) => &g.guid,
            Geometry::Point(g) => &g.guid,
            Geometry::PointCloud(g) => &g.guid,
            Geometry::Polyline(g) => &g.guid,
        }
    }
}

/// A Session containing geometry objects with hierarchical and graph structures.
///
/// The Session serves as a container for managing geometry objects (currently Points)
/// along with their relationships through tree and graph data structures. It provides
/// JSON serialization capabilities for cross-language interoperability.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Session")]
pub struct Session {
    /// Unique identifier for the session
    pub guid: String,
    /// Human-readable name for the session
    pub name: String,
    /// Collection of geometry objects (Points)
    #[serde(rename = "objects")]
    pub objects: Objects,
    /// Lookup table mapping object GUIDs to geometry objects (fast heterogeneous lookup)
    #[serde(skip)]
    pub lookup: HashMap<String, Geometry>,
    /// Hierarchical tree structure for organizing objects
    #[serde(rename = "tree")]
    pub tree: Tree,
    /// Graph structure for representing object relationships
    #[serde(rename = "graph")]
    pub graph: Graph,
    /// Boundary Volume Hierarchy for spatial collision detection
    #[serde(skip)]
    pub bvh: BVH,
    /// Cached BVH for ray casting (indices map to cached_guids)
    #[serde(skip)]
    pub cached_ray_bvh: Option<BVH>,
    /// Cached GUIDs corresponding to cached_boxes order
    #[serde(skip)]
    pub cached_guids: Vec<String>,
    /// Cached AABBs for ray-casting BVH
    #[serde(skip)]
    pub cached_boxes: Vec<Obb>,
    /// Dirty flag for cached ray BVH
    #[serde(skip)]
    pub bvh_cache_dirty: bool,
}

#[derive(Debug, Clone)]
pub struct RayHit {
    pub guid: String,
    pub point: Point,
    pub distance: f64,
}

impl Default for Session {
    /// Creates a default Session with the name "my_session".
    fn default() -> Self {
        Self::new("my_session")
    }
}

impl Session {
    /// Creates a new Session with the specified name.
    ///
    /// # Arguments
    /// * `name` - The name for the session
    ///
    /// # Returns
    /// A new Session instance with a unique GUID, empty objects collection,
    /// and initialized tree and graph structures.
    pub fn new(name: &str) -> Self {
        let guid = Uuid::new_v4().to_string();
        let objects = Objects::new();
        let lookup = HashMap::new();
        let mut tree = Tree::new(&format!("{name}_tree"));
        let graph = Graph::new(&format!("{name}_graph"));

        // Create empty root node with session name
        let root_node = TreeNode::new(name);
        tree.add(&root_node, None);

        // Create boundary-volume-hierarchy, each time we add object we store inside bvh
        let bvh = BVH::new();

        Self {
            guid,
            name: name.to_string(),
            objects,
            lookup,
            tree,
            graph,
            bvh,
            cached_ray_bvh: None,
            cached_guids: Vec::new(),
            cached_boxes: Vec::new(),
            bvh_cache_dirty: true,
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Session to a JSON string.
    ///
    /// # Returns
    /// A Result containing the JSON string representation of the Session,
    /// or an error if serialization fails.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        // Use custom serialization to ensure consistent structure with C++/Python
        // Convert graph to use array structure instead of nested objects
        let graph_json: serde_json::Value = serde_json::from_str(&self.graph.jsondump()?)?;

        let json_obj = serde_json::json!({
            "type": "Session",
            "guid": self.guid,
            "name": self.name,
            "objects": self.objects,
            "tree": self.tree,
            "graph": graph_json
        });

        Ok(serde_json::to_string_pretty(&json_obj)?)
    }

    /// Deserializes Session from a JSON string.
    ///
    /// # Arguments
    /// * `json_data` - The JSON string to deserialize
    ///
    /// # Returns
    /// A Result containing the deserialized Session, or an error if parsing fails.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_obj: serde_json::Value = serde_json::from_str(json_data)?;

        // Deserialize components using their custom methods
        let objects: Objects = serde_json::from_value(json_obj["objects"].clone())?;
        let tree: Tree = serde_json::from_value(json_obj["tree"].clone())?;
        // Convert graph JSON value to properly formatted string
        let graph_json_str = serde_json::to_string(&json_obj["graph"])?;
        let graph: Graph = Graph::jsonload(&graph_json_str)?;

        // Rebuild lookup table from all objects
        let mut lookup = HashMap::new();
        for bbox in &objects.bboxes {
            lookup.insert(bbox.guid.clone(), Geometry::Obb(bbox.clone()));
        }
        for line in &objects.lines {
            lookup.insert(line.guid.clone(), Geometry::Line(line.clone()));
        }
        for mesh in &objects.meshes {
            lookup.insert(mesh.guid.clone(), Geometry::Mesh(mesh.clone()));
        }
        for plane in &objects.planes {
            lookup.insert(plane.guid.clone(), Geometry::Plane(plane.clone()));
        }
        for point in &objects.points {
            lookup.insert(point.guid.clone(), Geometry::Point(point.clone()));
        }
        for pointcloud in &objects.pointclouds {
            lookup.insert(
                pointcloud.guid.clone(),
                Geometry::PointCloud(pointcloud.clone()),
            );
        }
        for polyline in &objects.polylines {
            lookup.insert(polyline.guid.clone(), Geometry::Polyline(polyline.clone()));
        }
        for brep in &objects.breps {
            lookup.insert(brep.guid.clone(), Geometry::BRep(brep.clone()));
        }
        for elem in &objects.elements {
            lookup.insert(elem.guid.clone(), Geometry::Element(elem.clone()));
        }

        let session = Session {
            guid: json_obj["guid"].as_str().unwrap_or("").to_string(),
            name: json_obj["name"]
                .as_str()
                .unwrap_or("my_session")
                .to_string(),
            objects,
            lookup,
            tree,
            graph,
            bvh: BVH::new(),
            cached_ray_bvh: None,
            cached_guids: Vec::new(),
            cached_boxes: Vec::new(),
            bvh_cache_dirty: true,
        };

        Ok(session)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    pub fn json_dump(&self, filepath: &str) {
        let json = self.jsondump().unwrap_or_default();
        fs::write(filepath, json).expect("Failed to write JSON file");
    }

    pub fn json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");
        Self::jsonload(&json).unwrap_or_else(|_| Self::default())
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        // Build Objects proto
        let mut objects_proto = crate::proto::Objects {
            name: self.objects.name.clone(),
            guid: self.objects.guid.clone(),
            ..Default::default()
        };
        for p in &self.objects.points {
            objects_proto.points.push(crate::proto::Point::decode(p.pb_dumps().as_slice()).unwrap());
        }
        for l in &self.objects.lines {
            objects_proto.lines.push(crate::proto::Line::decode(l.pb_dumps().as_slice()).unwrap());
        }
        for pl in &self.objects.planes {
            objects_proto.planes.push(crate::proto::Plane::decode(pl.pb_dumps().as_slice()).unwrap());
        }
        for b in &self.objects.bboxes {
            objects_proto.bboxes.push(crate::proto::BoundingBox::decode(b.pb_dumps().as_slice()).unwrap());
        }
        for pl in &self.objects.polylines {
            objects_proto.polylines.push(crate::proto::Polyline::decode(pl.pb_dumps().as_slice()).unwrap());
        }
        for pc in &self.objects.pointclouds {
            objects_proto.pointclouds.push(crate::proto::PointCloud::decode(pc.pb_dumps().as_slice()).unwrap());
        }
        for m in &self.objects.meshes {
            objects_proto.meshes.push(crate::proto::Mesh::decode(m.pb_dumps().as_slice()).unwrap());
        }
        for b in &self.objects.breps {
            objects_proto.breps.push(crate::proto::BRep::decode(b.pb_dumps().as_slice()).unwrap());
        }
        for e in &self.objects.elements {
            objects_proto.elements.push(crate::proto::Element::decode(e.pb_dumps().as_slice()).unwrap());
        }

        // Build Tree proto
        fn treenode_to_proto(node: &Rc<RefCell<TreeNode>>) -> crate::proto::TreeNode {
            let b = node.borrow();
            let children: Vec<crate::proto::TreeNode> = b.children().iter().map(|c| treenode_to_proto(c)).collect();
            crate::proto::TreeNode {
                guid: b.guid.clone(),
                name: b.name.clone(),
                parent_guid: b.parent().map(|p| p.borrow().guid.clone()).unwrap_or_default(),
                children,
                color: None,
            }
        }
        let tree_proto = crate::proto::Tree {
            guid: self.tree.guid.clone(),
            name: self.tree.name.clone(),
            root: self.tree.root().map(|r| treenode_to_proto(&r)),
        };

        // Build Graph proto
        let mut vertices_map: std::collections::HashMap<String, crate::proto::Vertex> = std::collections::HashMap::new();
        for v in self.graph.get_vertices() {
            vertices_map.insert(v.name.clone(), crate::proto::Vertex {
                name: v.name.clone(),
                guid: v.guid.clone(),
                attribute: v.attribute.clone(),
                index: v.index,
            });
        }
        let mut edges_proto: Vec<crate::proto::Edge> = Vec::new();
        for (_u, neighbors) in &self.graph.edges {
            for (_v, edge) in neighbors {
                edges_proto.push(crate::proto::Edge {
                    guid: edge.guid.clone(),
                    name: edge.name.clone(),
                    v0: edge.v0.clone(),
                    v1: edge.v1.clone(),
                    attribute: edge.attribute.clone(),
                    index: edge.index,
                });
            }
        }
        let graph_proto = crate::proto::Graph {
            name: self.graph.name.clone(),
            guid: self.graph.guid.clone(),
            vertices: vertices_map,
            edges: edges_proto,
            vertex_count: self.graph.vertex_count,
            edge_count: self.graph.edge_count,
        };

        let proto = crate::proto::Session {
            name: self.name.clone(),
            guid: self.guid.clone(),
            objects: Some(objects_proto),
            tree: Some(tree_proto),
            graph: Some(graph_proto),
            bvh_boxes: Vec::new(),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Session::decode(data)?;

        let mut session = Session::new(&proto.name);
        session.guid = proto.guid;

        // Rebuild objects
        if let Some(objects_proto) = &proto.objects {
            session.objects.guid = objects_proto.guid.clone();
            session.objects.name = objects_proto.name.clone();
            for p in &objects_proto.points {
                let pt = Point::pb_loads(&p.encode_to_vec())?;
                session.objects.points.push(pt);
            }
            for l in &objects_proto.lines {
                let ln = Line::pb_loads(&l.encode_to_vec())?;
                session.objects.lines.push(ln);
            }
            for pl in &objects_proto.planes {
                let pln = Plane::pb_loads(&pl.encode_to_vec())?;
                session.objects.planes.push(pln);
            }
            for b in &objects_proto.bboxes {
                let bb = Obb::pb_loads(&b.encode_to_vec())?;
                session.objects.bboxes.push(bb);
            }
            for pl in &objects_proto.polylines {
                let pll = Polyline::pb_loads(&pl.encode_to_vec())?;
                session.objects.polylines.push(pll);
            }
            for pc in &objects_proto.pointclouds {
                let pcl = PointCloud::pb_loads(&pc.encode_to_vec());
                session.objects.pointclouds.push(pcl);
            }
            for m in &objects_proto.meshes {
                let msh = Mesh::pb_loads(&m.encode_to_vec())?;
                session.objects.meshes.push(msh);
            }
            for b in &objects_proto.breps {
                let brp = BRep::pb_loads(&b.encode_to_vec())?;
                session.objects.breps.push(brp);
            }
            for e in &objects_proto.elements {
                let elem = Element::pb_loads(&e.encode_to_vec())?;
                session.objects.elements.push(elem);
            }
        }

        // Rebuild tree
        if let Some(tree_proto) = &proto.tree {
            session.tree = Tree::new(&tree_proto.name);
            session.tree.guid = tree_proto.guid.clone();
            if let Some(root_proto) = &tree_proto.root {
                fn proto_to_treenode(proto: &crate::proto::TreeNode) -> Rc<RefCell<TreeNode>> {
                    let node = TreeNode::new(&proto.name);
                    for child_proto in &proto.children {
                        let child = proto_to_treenode(child_proto);
                        node.borrow_mut().add(&child);
                    }
                    node
                }
                let root = proto_to_treenode(root_proto);
                session.tree.add(&root, None);
            }
        }

        // Rebuild graph
        if let Some(graph_proto) = &proto.graph {
            session.graph = Graph::new(&graph_proto.name);
            session.graph.guid = graph_proto.guid.clone();
            for (name, v) in &graph_proto.vertices {
                session.graph.add_node(name, &v.attribute);
            }
            for e in &graph_proto.edges {
                session.graph.add_edge(&e.v0, &e.v1, &e.attribute);
            }
        }

        // Rebuild lookup
        for bbox in &session.objects.bboxes {
            session.lookup.insert(bbox.guid.clone(), Geometry::Obb(bbox.clone()));
        }
        for line in &session.objects.lines {
            session.lookup.insert(line.guid.clone(), Geometry::Line(line.clone()));
        }
        for mesh in &session.objects.meshes {
            session.lookup.insert(mesh.guid.clone(), Geometry::Mesh(mesh.clone()));
        }
        for plane in &session.objects.planes {
            session.lookup.insert(plane.guid.clone(), Geometry::Plane(plane.clone()));
        }
        for point in &session.objects.points {
            session.lookup.insert(point.guid.clone(), Geometry::Point(point.clone()));
        }
        for pointcloud in &session.objects.pointclouds {
            session.lookup.insert(pointcloud.guid.clone(), Geometry::PointCloud(pointcloud.clone()));
        }
        for polyline in &session.objects.polylines {
            session.lookup.insert(polyline.guid.clone(), Geometry::Polyline(polyline.clone()));
        }
        for brep in &session.objects.breps {
            session.lookup.insert(brep.guid.clone(), Geometry::BRep(brep.clone()));
        }

        Ok(session)
    }

    pub fn pb_dump(&self, path: &str) {
        std::fs::write(path, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(path: &str) -> Self {
        let data = std::fs::read(path).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // BVH Collision Detection
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Compute bounding box for a geometry object, inflated by tolerance
    fn compute_bounding_box(geometry: &Geometry) -> Obb {
        let inflate = Tolerance::APPROXIMATION;
        match geometry {
            Geometry::Point(p) => Obb::from_point(p.clone(), inflate),
            Geometry::Line(l) => {
                let points = vec![l.start(), l.end()];
                Obb::from_points(&points, inflate)
            }
            Geometry::Polyline(pl) => Obb::from_points(&pl.get_points(), inflate),
            Geometry::PointCloud(pc) => Obb::from_points(&pc.get_points(), inflate),
            Geometry::Mesh(m) => {
                // Extract vertices from mesh vertex data
                let points: Vec<Point> = m
                    .vertex
                    .values()
                    .map(|v| Point::new(v.x, v.y, v.z))
                    .collect();
                if points.is_empty() {
                    Obb::from_point(Point::new(0.0, 0.0, 0.0), inflate)
                } else {
                    Obb::from_points(&points, inflate)
                }
            }
            Geometry::Obb(bb) => {
                // Inflate existing bounding box
                let mut inflated = bb.clone();
                inflated.half_size = crate::Vector::new(
                    inflated.half_size[0] + inflate,
                    inflated.half_size[1] + inflate,
                    inflated.half_size[2] + inflate,
                );
                inflated
            }
            Geometry::Plane(p) => {
                Obb::from_point(p.origin(), inflate * 10.0)
            }
            Geometry::BRep(b) => {
                let points: Vec<Point> = b.m_vertices.clone();
                if points.is_empty() {
                    Obb::from_point(Point::new(0.0, 0.0, 0.0), inflate)
                } else {
                    Obb::from_points(&points, inflate)
                }
            }
            Geometry::Element(e) => {
                let mut e2 = e.clone();
                e2.aabb()
            }
        }
    }

    /// Get all collision pairs using BVH and add them as graph edges.
    ///
    /// Automatically:
    /// - Computes bounding boxes for all objects with tolerance inflation
    /// - Builds/rebuilds the BVH with auto-computed world size
    /// - Detects all collision pairs
    /// - Adds collision edges to the graph
    ///
    /// # Returns
    /// A vector of tuples (guid1, guid2) representing colliding geometry pairs
    pub fn get_collisions(&mut self) -> Vec<(String, String)> {
        // Collect all objects with their bounding boxes and GUIDs
        let mut boxes_with_guids: Vec<(Obb, String)> = Vec::new();

        for (guid, geometry) in &self.lookup {
            let bbox = Self::compute_bounding_box(geometry);
            boxes_with_guids.push((bbox, guid.clone()));
        }

        if boxes_with_guids.is_empty() {
            return Vec::new();
        }

        // Build BVH with GUIDs (auto-computes world size)
        self.bvh.build_with_guids(&boxes_with_guids);

        // Extract just the boxes for collision checking
        let boxes: Vec<Obb> = boxes_with_guids
            .iter()
            .map(|(bbox, _)| bbox.clone())
            .collect();

        // Get collision pairs as GUIDs directly
        let collision_pairs = self.bvh.check_all_collisions_guids(&boxes);

        // Add collision edges to graph
        for (guid1, guid2) in &collision_pairs {
            self.graph.add_edge(guid1, guid2, "bvh_collision");
        }

        collision_pairs
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Ray BVH Cache
    ///////////////////////////////////////////////////////////////////////////////////////////

    fn cache_geometry_aabb(&mut self, guid: &str, geometry: &Geometry) {
        let bbox = Self::compute_bounding_box(geometry);
        self.cached_boxes.push(bbox);
        self.cached_guids.push(guid.to_string());
        self.bvh_cache_dirty = true;
    }

    fn rebuild_ray_bvh_cache(&mut self) {
        if self.cached_boxes.len() != self.lookup.len() {
            self.cached_boxes.clear();
            self.cached_guids.clear();
            self.cached_boxes.reserve(self.lookup.len());
            self.cached_guids.reserve(self.lookup.len());
            for (guid, geometry) in &self.lookup {
                let bbox = Self::compute_bounding_box(geometry);
                self.cached_boxes.push(bbox);
                self.cached_guids.push(guid.clone());
            }
        }
        if !self.cached_boxes.is_empty() {
            let world_size = BVH::compute_world_size(&self.cached_boxes);
            self.cached_ray_bvh = Some(BVH::from_boxes(&self.cached_boxes, world_size));
        } else {
            self.cached_ray_bvh = None;
        }
    }

    fn invalidate_bvh_cache(&mut self) {
        self.bvh_cache_dirty = true;
    }

    pub fn ray_cast(
        &mut self,
        origin: &Point,
        direction: &crate::Vector,
        tolerance: f64,
    ) -> Vec<RayHit> {
        let dir_len = direction.magnitude();
        if dir_len <= 0.0 {
            return Vec::new();
        }
        let dir_unit = crate::Vector::new(
            direction[0] / dir_len,
            direction[1] / dir_len,
            direction[2] / dir_len,
        );

        let far = 1e6f64;
        let ray_end = Point::new(
            origin[0] + dir_unit[0] * far,
            origin[1] + dir_unit[1] * far,
            origin[2] + dir_unit[2] * far,
        );
        let ray_line = Line::from_points(origin, &ray_end);

        // Use cached BVH for ray casting
        if self.bvh_cache_dirty || self.cached_ray_bvh.is_none() {
            self.rebuild_ray_bvh_cache();
            self.bvh_cache_dirty = false;
        }
        let bvh = match &self.cached_ray_bvh {
            Some(b) => b,
            None => return Vec::new(),
        };

        let mut candidates: Vec<usize> = Vec::new();
        bvh.ray_cast(origin, &dir_unit, &mut candidates, true);

        let mut hits_all: Vec<RayHit> = Vec::new();

        for idx in candidates {
            if idx >= self.cached_guids.len() {
                continue;
            }
            let guid = self.cached_guids[idx].clone();
            let geom = match self.lookup.get_mut(&guid) {
                Some(g) => g,
                None => continue,
            };

            let mut hit_point: Option<Point> = None;

            match geom {
                Geometry::Obb(bb) => {
                    if let Some(pts) = crate::intersection::ray_box(&ray_line, bb, 0.0, far) {
                        if !pts.is_empty() {
                            hit_point = Some(pts[0].clone());
                        }
                    }
                }
                Geometry::Plane(pl) => {
                    if let Some(p) = crate::intersection::line_plane(&ray_line, pl, true) {
                        hit_point = Some(p);
                    }
                }
                Geometry::Line(l) => {
                    if let Some(p) =
                        crate::intersection::line_line(&ray_line, l, Tolerance::APPROXIMATION)
                    {
                        hit_point = Some(p);
                    }
                }
                Geometry::Polyline(pl) => {
                    let mut best_t = f64::INFINITY;
                    let mut best_p: Option<Point> = None;
                    let pl_points = pl.get_points();
                    if pl_points.len() >= 2 {
                        for i in 0..(pl_points.len() - 1) {
                            let seg = Line::from_points(&pl_points[i], &pl_points[i + 1]);
                            if let Some(p) = crate::intersection::line_line(
                                &ray_line,
                                &seg,
                                Tolerance::APPROXIMATION,
                            ) {
                                let dx = p[0] - origin[0];
                                let dy = p[1] - origin[1];
                                let dz = p[2] - origin[2];
                                let t = dx * dir_unit[0] + dy * dir_unit[1] + dz * dir_unit[2];
                                if t >= 0.0 && t < best_t {
                                    best_t = t;
                                    best_p = Some(p);
                                }
                            }
                        }
                    }
                    if let Some(p) = best_p {
                        hit_point = Some(p);
                    }
                }
                Geometry::Mesh(m) => {
                    if let Some(p) = m.ray_cast_bvh(&ray_line, 1e-6) {
                        hit_point = Some(p);
                    }
                }
                Geometry::Point(p) => {
                    let vx = p[0] - origin[0];
                    let vy = p[1] - origin[1];
                    let vz = p[2] - origin[2];
                    let cross_x = vy * dir_unit[2] - vz * dir_unit[1];
                    let cross_y = vz * dir_unit[0] - vx * dir_unit[2];
                    let cross_z = vx * dir_unit[1] - vy * dir_unit[0];
                    let dist = (cross_x * cross_x + cross_y * cross_y + cross_z * cross_z).sqrt();
                    if dist <= tolerance {
                        let t = vx * dir_unit[0] + vy * dir_unit[1] + vz * dir_unit[2];
                        if t >= 0.0 {
                            let hp = Point::new(
                                origin[0] + dir_unit[0] * t,
                                origin[1] + dir_unit[1] * t,
                                origin[2] + dir_unit[2] * t,
                            );
                            hit_point = Some(hp);
                        }
                    }
                }
                Geometry::PointCloud(_) => {}
                Geometry::BRep(_) => {}
                Geometry::Element(_) => {}
            }

            if let Some(hp) = hit_point {
                let dx = hp[0] - origin[0];
                let dy = hp[1] - origin[1];
                let dz = hp[2] - origin[2];
                let forward = dx * dir_unit[0] + dy * dir_unit[1] + dz * dir_unit[2];
                if forward >= 0.0 {
                    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                    hits_all.push(RayHit {
                        guid: guid.clone(),
                        point: hp,
                        distance: dist,
                    });
                }
            }
        }

        if hits_all.is_empty() {
            return Vec::new();
        }

        let mut min_d = f64::INFINITY;
        for h in &hits_all {
            if h.distance < min_d {
                min_d = h.distance;
            }
        }
        let eps = tolerance;
        let mut hits: Vec<RayHit> = hits_all
            .into_iter()
            .filter(|h| (h.distance - min_d).abs() <= eps)
            .collect();
        hits.sort_by(|a, b| {
            a.distance
                .partial_cmp(&b.distance)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        hits
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Adds a point to the Session.
    ///
    /// The point is added to the objects collection, lookup table, graph as a node,
    /// and tree as a child of the root node.
    ///
    /// # Arguments
    /// * `point` - The Point object to add to the session
    ///
    /// # Returns
    /// The TreeNode created for this point
    pub fn add_point(&mut self, point: Point) -> Rc<RefCell<TreeNode>> {
        let point_guid = point.guid.clone();
        let point_name = point.name.clone();
        let geometry = Geometry::Point(point.clone());

        self.objects.points.push(point);
        self.lookup.insert(point_guid.clone(), geometry);
        if let Some(Geometry::Point(p)) = self.lookup.get(&point_guid) {
            self.cache_geometry_aabb(&point_guid, &Geometry::Point(p.clone()));
        }
        self.graph
            .add_node(&point_guid, &format!("point_{point_name}"));

        TreeNode::new(&point_guid)
    }

    pub fn add_line(&mut self, line: Line) -> Rc<RefCell<TreeNode>> {
        let guid = line.guid.clone();
        let name = line.name.clone();
        let geometry = Geometry::Line(line.clone());

        self.objects.lines.push(line);
        self.lookup.insert(guid.clone(), geometry);
        if let Some(Geometry::Line(l)) = self.lookup.get(&guid) {
            self.cache_geometry_aabb(&guid, &Geometry::Line(l.clone()));
        }
        self.graph.add_node(&guid, &format!("line_{name}"));

        TreeNode::new(&guid)
    }

    pub fn add_plane(&mut self, plane: Plane) -> Rc<RefCell<TreeNode>> {
        let guid = plane.guid.clone();
        let name = plane.name.clone();
        let geometry = Geometry::Plane(plane.clone());

        self.objects.planes.push(plane);
        self.lookup.insert(guid.clone(), geometry);
        if let Some(Geometry::Plane(p)) = self.lookup.get(&guid) {
            self.cache_geometry_aabb(&guid, &Geometry::Plane(p.clone()));
        }
        self.graph.add_node(&guid, &format!("plane_{name}"));

        TreeNode::new(&guid)
    }

    pub fn add_bbox(&mut self, bbox: Obb) -> Rc<RefCell<TreeNode>> {
        let guid = bbox.guid.clone();
        let name = bbox.name.clone();
        let geometry = Geometry::Obb(bbox.clone());

        self.objects.bboxes.push(bbox);
        self.lookup.insert(guid.clone(), geometry);
        if let Some(Geometry::Obb(b)) = self.lookup.get(&guid) {
            self.cache_geometry_aabb(&guid, &Geometry::Obb(b.clone()));
        }
        self.graph.add_node(&guid, &format!("bbox_{name}"));

        TreeNode::new(&guid)
    }

    pub fn add_polyline(&mut self, polyline: Polyline) -> Rc<RefCell<TreeNode>> {
        let guid = polyline.guid.clone();
        let name = polyline.name.clone();
        let geometry = Geometry::Polyline(polyline.clone());

        self.objects.polylines.push(polyline);
        self.lookup.insert(guid.clone(), geometry);
        if let Some(Geometry::Polyline(p)) = self.lookup.get(&guid) {
            self.cache_geometry_aabb(&guid, &Geometry::Polyline(p.clone()));
        }
        self.graph.add_node(&guid, &format!("polyline_{name}"));

        TreeNode::new(&guid)
    }

    pub fn add_pointcloud(&mut self, pointcloud: PointCloud) -> Rc<RefCell<TreeNode>> {
        let guid = pointcloud.guid.clone();
        let name = pointcloud.name.clone();
        let geometry = Geometry::PointCloud(pointcloud.clone());

        self.objects.pointclouds.push(pointcloud);
        self.lookup.insert(guid.clone(), geometry);
        if let Some(Geometry::PointCloud(p)) = self.lookup.get(&guid) {
            self.cache_geometry_aabb(&guid, &Geometry::PointCloud(p.clone()));
        }
        self.graph.add_node(&guid, &format!("pointcloud_{name}"));

        TreeNode::new(&guid)
    }

    pub fn add_mesh(&mut self, mesh: Mesh) -> Rc<RefCell<TreeNode>> {
        let guid = mesh.guid.clone();
        let name = mesh.name.clone();
        let geometry = Geometry::Mesh(mesh.clone());

        self.objects.meshes.push(mesh);
        self.lookup.insert(guid.clone(), geometry);
        if let Some(Geometry::Mesh(m)) = self.lookup.get(&guid) {
            self.cache_geometry_aabb(&guid, &Geometry::Mesh(m.clone()));
        }
        self.graph.add_node(&guid, &format!("mesh_{name}"));

        TreeNode::new(&guid)
    }

    pub fn add_brep(&mut self, brep: BRep) -> Rc<RefCell<TreeNode>> {
        let guid = brep.guid.clone();
        let name = brep.name.clone();

        self.objects.breps.push(brep.clone());
        self.lookup.insert(guid.clone(), Geometry::BRep(brep));
        self.graph.add_node(&guid, &format!("brep_{name}"));

        TreeNode::new(&guid)
    }

    /// Adds a TreeNode to the tree hierarchy.
    ///
    /// # Arguments
    /// * `node` - The TreeNode to add
    /// * `parent` - Optional parent TreeNode (defaults to root if None)
    pub fn add<'a>(&mut self, node: &Rc<RefCell<TreeNode>>, parent: impl Into<Option<&'a Rc<RefCell<TreeNode>>>>)
    where
        Rc<RefCell<TreeNode>>: 'a,
    {
        let parent_opt = parent.into();
        if parent_opt.is_none() {
            if let Some(root) = self.tree.root() {
                self.tree.add(node, Some(&root));
            }
        } else {
            self.tree.add(node, parent_opt);
        }
    }

    /// Create a named layer (TreeNode) and add it to the root of the tree.
    pub fn add_group(&mut self, name: &str) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(name);
        self.add(&node, None);
        node
    }

    /// Adds an edge between two geometry objects in the graph.
    ///
    /// # Arguments
    /// * `from_guid` - The GUID of the source object
    /// * `to_guid` - The GUID of the target object
    /// * `attribute` - The attribute or label for the edge
    pub fn add_edge(&mut self, from_guid: &str, to_guid: &str, attribute: &str) {
        self.graph.add_edge(from_guid, to_guid, attribute);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details - Lookup
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Gets a geometry object by its GUID.
    ///
    /// # Arguments
    /// * `guid` - The GUID of the object to retrieve
    ///
    /// # Returns
    /// An Option containing a reference to the Geometry enum if found, or None if not found.
    pub fn get_object(&self, guid: &str) -> Option<&Geometry> {
        self.lookup.get(guid)
    }

    /// Remove a geometry object by its GUID.
    ///
    /// # Arguments
    /// * `guid` - The UUID of the geometry object to remove.
    ///
    /// # Returns
    /// `true` if the object was removed, `false` if not found.
    pub fn remove_object(&mut self, guid: &str) -> bool {
        // Check if object exists in lookup table
        if !self.lookup.contains_key(guid) {
            return false;
        }

        // Remove from all object collections
        self.objects.points.retain(|p| p.guid != guid);
        self.objects.lines.retain(|l| l.guid != guid);
        self.objects.polylines.retain(|p| p.guid != guid);
        self.objects.planes.retain(|p| p.guid != guid);
        self.objects.bboxes.retain(|b| b.guid != guid);
        self.objects.meshes.retain(|m| m.guid != guid);
        self.objects.pointclouds.retain(|p| p.guid != guid);
        self.objects.breps.retain(|b| b.guid != guid);

        // Remove from lookup table
        self.lookup.remove(guid);
        self.invalidate_bvh_cache();

        // Remove from tree - find node by GUID and remove it
        if let Some(node) = self.tree.find_node_by_guid(&guid.to_string()) {
            self.tree.remove(&node);
        }

        // Remove from graph using string GUID
        if self.graph.has_node(guid) {
            self.graph.remove_node(guid);
        }

        true
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details - Tree
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Add a parent-child relationship in the tree structure.
    ///
    /// # Arguments
    /// * `parent_guid` - The GUID of the parent geometry object.
    /// * `child_guid` - The GUID of the child geometry object.
    ///
    /// # Returns
    /// `true` if the relationship was added successfully.
    pub fn add_hierarchy(&mut self, parent_guid: &str, child_guid: &str) -> bool {
        self.tree
            .add_child_by_guid(&parent_guid.to_string(), &child_guid.to_string())
    }

    /// Get all children GUIDs of a geometry object in the tree.
    ///
    /// # Arguments
    /// * `guid` - The GUID of the geometry object.
    ///
    /// # Returns
    /// A vector containing the GUIDs of all children of the specified geometry object.
    pub fn get_children(&self, guid: &str) -> Vec<String> {
        self.tree.get_children(guid)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details - Graph
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Add a relationship edge in the graph structure.
    ///
    /// # Arguments
    /// * `from_guid` - The GUID of the source geometry object.
    /// * `to_guid` - The GUID of the target geometry object.
    /// * `relationship_type` - The type of relationship.
    pub fn add_relationship(&mut self, from_guid: &str, to_guid: &str, relationship_type: &str) {
        self.graph.add_edge(from_guid, to_guid, relationship_type);
    }

    /// Get all GUIDs connected to the given GUID in the graph.
    ///
    /// # Arguments
    /// * `guid` - The GUID of the geometry object.
    ///
    /// # Returns
    /// A vector containing the GUIDs of all connected geometry objects.
    pub fn get_neighbours(&self, guid: &str) -> Vec<String> {
        self.graph.get_neighbors(guid)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details - Transformed Geometry
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get all geometry with transformations applied from tree hierarchy.
    ///
    /// Recursively traverses the tree and applies parent transformations to children.
    /// Each child's transformation is the composition of all ancestor transformations
    /// multiplied by its own transformation.
    ///
    /// # Returns
    /// Objects collection with transformed geometry
    pub fn get_geometry(&self) -> Objects {
        use crate::Xform;

        // Deep copy all objects
        let mut transformed_objects = self.objects.clone();

        // Rebuild lookup from copied objects
        let mut transformed_lookup: HashMap<String, Geometry> = HashMap::new();

        for point in &transformed_objects.points {
            transformed_lookup.insert(point.guid.clone(), Geometry::Point(point.clone()));
        }
        for line in &transformed_objects.lines {
            transformed_lookup.insert(line.guid.clone(), Geometry::Line(line.clone()));
        }
        for plane in &transformed_objects.planes {
            transformed_lookup.insert(plane.guid.clone(), Geometry::Plane(plane.clone()));
        }
        for bbox in &transformed_objects.bboxes {
            transformed_lookup.insert(bbox.guid.clone(), Geometry::Obb(bbox.clone()));
        }
        for polyline in &transformed_objects.polylines {
            transformed_lookup.insert(polyline.guid.clone(), Geometry::Polyline(polyline.clone()));
        }
        for pointcloud in &transformed_objects.pointclouds {
            transformed_lookup.insert(
                pointcloud.guid.clone(),
                Geometry::PointCloud(pointcloud.clone()),
            );
        }
        for mesh in &transformed_objects.meshes {
            transformed_lookup.insert(mesh.guid.clone(), Geometry::Mesh(mesh.clone()));
        }
        for brep in &transformed_objects.breps {
            transformed_lookup.insert(brep.guid.clone(), Geometry::BRep(brep.clone()));
        }

        fn transform_node(
            node: &Rc<RefCell<TreeNode>>,
            parent_xform: &Xform,
            transformed_lookup: &HashMap<String, Geometry>,
            transformed_objects: &mut Objects,
        ) {
            let node_name = node.borrow().name.clone();
            let geometry = transformed_lookup.get(&node_name);

            let current_xform = if let Some(geom) = geometry {
                // Get mutable reference and transform in-place
                let combined_xform = parent_xform
                    * match geom {
                        Geometry::Point(g) => &g.xform,
                        Geometry::Line(g) => &g.xform,
                        Geometry::Plane(g) => &g.xform,
                        Geometry::Obb(g) => &g.xform,
                        Geometry::Polyline(g) => &g.xform,
                        Geometry::PointCloud(g) => &g.xform,
                        Geometry::Mesh(g) => &g.xform,
                        Geometry::BRep(g) => &g.xform,
                        Geometry::Element(g) => &g.session_transformation,
                    };

                // Find and update the geometry in the collections
                match geom {
                    Geometry::Point(_) => {
                        if let Some(g) = transformed_objects
                            .points
                            .iter_mut()
                            .find(|p| p.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::Line(_) => {
                        if let Some(g) = transformed_objects
                            .lines
                            .iter_mut()
                            .find(|l| l.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::Plane(_) => {
                        if let Some(g) = transformed_objects
                            .planes
                            .iter_mut()
                            .find(|p| p.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::Obb(_) => {
                        if let Some(g) = transformed_objects
                            .bboxes
                            .iter_mut()
                            .find(|b| b.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::Polyline(_) => {
                        if let Some(g) = transformed_objects
                            .polylines
                            .iter_mut()
                            .find(|p| p.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::PointCloud(_) => {
                        if let Some(g) = transformed_objects
                            .pointclouds
                            .iter_mut()
                            .find(|p| p.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::Mesh(_) => {
                        if let Some(g) = transformed_objects
                            .meshes
                            .iter_mut()
                            .find(|m| m.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::BRep(_) => {
                        if let Some(g) = transformed_objects
                            .breps
                            .iter_mut()
                            .find(|b| b.guid == node_name)
                        {
                            g.xform = combined_xform.clone();
                        }
                    }
                    Geometry::Element(_) => {
                        if let Some(g) = transformed_objects
                            .elements
                            .iter_mut()
                            .find(|e| e.guid == node_name)
                        {
                            g.session_transformation = combined_xform.clone();
                        }
                    }
                }

                combined_xform
            } else {
                parent_xform.clone()
            };

            for child in node.borrow().children() {
                transform_node(
                    &child,
                    &current_xform,
                    transformed_lookup,
                    transformed_objects,
                );
            }
        }

        if let Some(root) = self.tree.root() {
            transform_node(
                &root,
                &Xform::identity(),
                &transformed_lookup,
                &mut transformed_objects,
            );
        }

        // Apply accumulated transformations to actual geometry coordinates
        for point in &mut transformed_objects.points {
            point.transform();
        }
        for line in &mut transformed_objects.lines {
            line.transform();
        }
        for plane in &mut transformed_objects.planes {
            plane.transform();
        }
        for bbox in &mut transformed_objects.bboxes {
            bbox.transform();
        }
        for polyline in &mut transformed_objects.polylines {
            polyline.transform();
        }
        for pointcloud in &mut transformed_objects.pointclouds {
            pointcloud.transform();
        }
        for mesh in &mut transformed_objects.meshes {
            mesh.transform(None);
        }
        for brep in &mut transformed_objects.breps {
            brep.transform();
        }

        transformed_objects
    }
}

impl fmt::Display for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Session({}, {}, points={}, vertices={}, edges={})",
            self.name,
            self.guid,
            self.objects.points.len(),
            self.graph.vertex_count,
            self.graph.edge_count
        )
    }
}

