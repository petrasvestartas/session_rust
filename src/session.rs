use crate::{
    BRep, Element, OBB, Graph, Line, Mesh, NurbsCurve, NurbsSurface, Objects, Plane, Point,
    PointCloud, Polyline, Tolerance, Tree, TreeNode, SpatialBVH, Xform,
};
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt;
use std::rc::Rc;
use std::fs;

/// Enum representing all possible geometry types in a Session.
/// This is equivalent to C++'s std::variant<std::shared_ptr<...>> for heterogeneous geometry
/// storage. Every variant holds an Rc SHARED with the objects vectors — ONE allocation per
/// object (the shared_ptr model), enum stays pointer-sized (16 B). Reads are unaffected
/// (deref coercion); writes go copy-on-write via Rc::make_mut, and objects_synced() re-shares
/// any COW splits at save time.
#[derive(Debug, Clone)]
pub enum Geometry {
    OBB(Rc<OBB>),
    BRep(Rc<BRep>),
    Element(Rc<Element>),
    Line(Rc<Line>),
    Mesh(Rc<Mesh>),
    NurbsCurve(Rc<NurbsCurve>),
    NurbsSurface(Rc<NurbsSurface>),
    Plane(Rc<Plane>),
    Point(Rc<Point>),
    PointCloud(Rc<PointCloud>),
    Polyline(Rc<Polyline>),
}

impl Geometry {
    /// Get the GUID of the geometry object
    pub fn guid(&self) -> &str {
        match self {
            Geometry::OBB(g) => g.guid(),
            Geometry::BRep(g) => g.guid(),
            Geometry::Element(g) => g.guid(),
            Geometry::Line(g) => g.guid(),
            Geometry::Mesh(g) => g.guid(),
            Geometry::NurbsCurve(g) => g.guid(),
            Geometry::NurbsSurface(g) => g.guid(),
            Geometry::Plane(g) => g.guid(),
            Geometry::Point(g) => g.guid(),
            Geometry::PointCloud(g) => g.guid(),
            Geometry::Polyline(g) => g.guid(),
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
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    /// Human-readable name for the session
    pub name: String,
    /// Typed collections in INSERTION ORDER — the serialization layout. Between loads and
    /// saves `lookup` is the mutable truth; every save path syncs from it (objects_synced).
    #[serde(rename = "objects")]
    pub objects: Objects,
    /// Guid → geometry: THE authoritative store between load and save. Mutate objects HERE;
    /// jsondump/pb_dumps read this truth back in objects' insertion order.
    #[serde(skip)]
    pub lookup: HashMap<String, Geometry>,
    /// Hierarchical tree structure for organizing objects
    #[serde(rename = "tree")]
    pub tree: Tree,
    /// Graph structure for representing object relationships
    #[serde(rename = "graph")]
    pub graph: Graph,
    /// Guid → LOCAL transform, relative to the tree parent. THE only place a transform is
    /// stored: geometry types carry no transformation member. Cumulative placement comes from
    /// `world_xform`, which multiplies down the tree. Serialized explicitly by
    /// jsondump/pb_dumps in `order()` sequence (a HashMap has no deterministic order).
    #[serde(skip)]
    pub xforms: HashMap<String, Xform>,
    /// Boundary Volume Hierarchy for spatial collision detection
    #[serde(skip)]
    pub bvh: SpatialBVH,
    /// Cached SpatialBVH for ray casting (indices map to cached_guids)
    #[serde(skip)]
    pub cached_ray_bvh: Option<SpatialBVH>,
    /// Cached GUIDs corresponding to cached_boxes order
    #[serde(skip)]
    pub cached_guids: Vec<String>,
    /// Cached AABBs for ray-casting SpatialBVH
    #[serde(skip)]
    pub cached_boxes: Vec<OBB>,
    /// Dirty flag for cached ray SpatialBVH
    #[serde(skip)]
    pub bvh_cache_dirty: bool,
}

#[derive(Debug, Clone)]
pub struct RayHit {
    guid: std::sync::OnceLock<String>,
    pub point: Point,
    pub distance: f64,
}

impl RayHit {
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }
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
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new(name: &str) -> Self {
        let objects = Objects::new();
        let lookup = HashMap::new();
        let mut tree = Tree::new(&format!("{name}_tree"));
        let graph = Graph::new(&format!("{name}_graph"));

        // Create empty root node with session name
        let root_node = TreeNode::new(name);
        tree.add(&root_node, None);

        // Create boundary-volume-hierarchy, each time we add object we store inside bvh
        let bvh = SpatialBVH::new();

        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            objects,
            lookup,
            tree,
            graph,
            xforms: HashMap::new(),
            bvh,
            cached_ray_bvh: None,
            cached_guids: Vec::new(),
            cached_boxes: Vec::new(),
            bvh_cache_dirty: true,
        }
    }

    /// The objects collections refreshed from `lookup` (the authoritative store), keeping the
    /// vectors' insertion order — so a mutation made through `lookup` (the sanctioned path) is
    /// what jsondump/pb_dumps write, and serialization order stays deterministic.
    /// DIRECTION CONTRACT: lookup wins. A COW split created by `Rc::make_mut` on an
    /// `objects.*` slot is NOT persisted — mutate through `lookup`, or re-point the lookup
    /// entry afterwards (see compute_face_to_face).
    fn objects_synced(&self) -> Objects {
        let mut objects = self.objects.clone();
        macro_rules! sync {
            ($vec:expr, $variant:ident) => {
                for item in $vec.iter_mut() {
                    if let Some(Geometry::$variant(g)) = self.lookup.get(item.guid()) {
                        if !Rc::ptr_eq(item, g) {
                            *item = Rc::clone(g); // re-share: heals any COW split, lookup wins
                        }
                    }
                }
            };
        }
        sync!(objects.points, Point);
        sync!(objects.lines, Line);
        sync!(objects.planes, Plane);
        sync!(objects.bboxes, OBB);
        sync!(objects.polylines, Polyline);
        sync!(objects.pointclouds, PointCloud);
        sync!(objects.meshes, Mesh);
        sync!(objects.nurbscurves, NurbsCurve);
        sync!(objects.nurbssurfaces, NurbsSurface);
        sync!(objects.breps, BRep);
        sync!(objects.elements, Element);
        objects
    }

    /// Canonical object order: the objects vectors walked in one fixed type sequence —
    /// deterministic across runs AND languages (lookup/map iteration is neither).
    /// Viewers and reconcile key their rows off this.
    pub fn order(&self) -> Vec<String> {
        let mut order = Vec::with_capacity(self.lookup.len());
        for p in &self.objects.points { order.push(p.guid().to_string()); }
        for l in &self.objects.lines { order.push(l.guid().to_string()); }
        for p in &self.objects.planes { order.push(p.guid().to_string()); }
        for b in &self.objects.bboxes { order.push(b.guid().to_string()); }
        for p in &self.objects.polylines { order.push(p.guid().to_string()); }
        for p in &self.objects.pointclouds { order.push(p.guid().to_string()); }
        for m in &self.objects.meshes { order.push(m.guid().to_string()); }
        for n in &self.objects.nurbscurves { order.push(n.guid().to_string()); }
        for n in &self.objects.nurbssurfaces { order.push(n.guid().to_string()); }
        for b in &self.objects.breps { order.push(b.guid().to_string()); }
        for e in &self.objects.elements { order.push(e.guid().to_string()); }
        order
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // XFORMS — the one place a transformation is stored
    ///////////////////////////////////////////////////////////////////////////////////////////


    /// Sets the LOCAL transform of an object, relative to its tree parent.
    pub fn set_xform(&mut self, guid: &str, xform: Xform) {
        self.xforms.insert(guid.to_string(), xform);
        self.bvh_cache_dirty = true;
    }

    /// The LOCAL transform of an object, identity when none was set.
    pub fn xform(&self, guid: &str) -> Xform {
        self.xforms.get(guid).cloned().unwrap_or_else(Xform::identity)
    }

    /// Removes an object's local transform, returning whether one was present.
    pub fn remove_xform(&mut self, guid: &str) -> bool {
        let removed = self.xforms.remove(guid).is_some();
        if removed {
            self.bvh_cache_dirty = true;
        }
        removed
    }

    /// The CUMULATIVE placement of an object: every ancestor's transform multiplied down the
    /// tree onto its own. An object with no tree node is its own root and returns its local
    /// transform — objects added without a parent are never attached, so treating a missing
    /// node as identity would silently move them to the origin.
    pub fn world_xform(&self, guid: &str) -> Xform {
        let mut acc = self.xform(guid);
        if let Some(node) = self.tree.get_node_by_name(guid) {
            // ancestors() runs immediate parent → root, so left-multiplying each in turn
            // yields root * … * parent * local — the same order the tree walk composes.
            for ancestor in node.borrow().ancestors() {
                let name = ancestor.borrow().name.clone();
                if let Some(xf) = self.xforms.get(&name) {
                    acc = xf * &acc;
                }
            }
        }
        acc
    }

    /// Every object's cumulative placement, computed in ONE downward pass. Use this instead of
    /// calling `world_xform` per object: that does a whole-tree scan to find each node, which
    /// is quadratic over a session.
    pub fn world_xforms(&self) -> HashMap<String, Xform> {
        // Nothing to compose: with no local transforms every composed frame IS the identity,
        // and every caller already falls back to identity for a guid the map lacks. Walking
        // the tree anyway costs one String clone + hash insert per NODE - measured at 38 ms
        // and 58,572 wasted entries on one flat PDF sheet, paid again on every rebuild.
        if self.xforms.is_empty() {
            return HashMap::new();
        }
        fn walk(
            node: &Rc<RefCell<TreeNode>>,
            parent_xform: &Xform,
            xforms: &HashMap<String, Xform>,
            out: &mut HashMap<String, Xform>,
        ) {
            let name = node.borrow().name.clone();
            let current = match xforms.get(&name) {
                Some(local) => parent_xform * local,
                None => parent_xform.clone(),
            };
            out.insert(name, current.clone());
            for child in node.borrow().children() {
                walk(&child, &current, xforms, out);
            }
        }

        let mut out = HashMap::new();
        if let Some(root) = self.tree.root() {
            walk(&root, &Xform::identity(), &self.xforms, &mut out);
        }
        // Objects that were added without a parent have no tree node; they are their own roots.
        for (guid, xform) in &self.xforms {
            out.entry(guid.clone()).or_insert_with(|| xform.clone());
        }
        out
    }

    /// The xforms in canonical `order()` sequence, identity entries omitted — the exact
    /// sequence jsondump and pb_dumps write, so both formats share one order.
    /// Group nodes carry transforms too but hold no geometry, so they are absent from
    /// `order()`; they follow, sorted by guid, or a group's placement would be lost on save.
    fn xforms_ordered(&self) -> Vec<(String, Xform)> {
        let mut ordered = Vec::new();
        for guid in self.order() {
            if let Some(xform) = self.xforms.get(&guid) {
                if !xform.is_identity() {
                    ordered.push((guid, xform.clone()));
                }
            }
        }
        let listed: std::collections::HashSet<&String> = ordered.iter().map(|(g, _)| g).collect();
        let mut rest: Vec<(String, Xform)> = self
            .xforms
            .iter()
            .filter(|(guid, xform)| !listed.contains(guid) && !xform.is_identity())
            .map(|(guid, xform)| (guid.clone(), xform.clone()))
            .collect();
        rest.sort_by(|a, b| a.0.cmp(&b.0));
        ordered.extend(rest);
        ordered
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
        let graph_json: serde_json::Value = serde_json::from_str(&self.graph.jsondump()?)?;

        let xforms_json: Vec<serde_json::Value> = self
            .xforms_ordered()
            .into_iter()
            .map(|(guid, xform)| serde_json::json!({ "guid": guid, "xform": xform }))
            .collect();

        let json_obj = serde_json::json!({
            "type": "Session",
            "guid": self.guid(),
            "name": self.name,
            "objects": self.objects_synced(),
            "tree": self.tree,
            "graph": graph_json,
            "xforms": xforms_json
        });

        let sorted = crate::file_encoders::sort_json_keys(json_obj);
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        serde::Serialize::serialize(&sorted, &mut ser)?;
        Ok(String::from_utf8(buf)?)
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
            lookup.insert(bbox.guid().to_string(), Geometry::OBB(Rc::clone(bbox)));
        }
        for line in &objects.lines {
            lookup.insert(line.guid().to_string(), Geometry::Line(Rc::clone(line)));
        }
        for mesh in &objects.meshes {
            lookup.insert(mesh.guid().to_string(), Geometry::Mesh(Rc::clone(mesh)));
        }
        for plane in &objects.planes {
            lookup.insert(plane.guid().to_string(), Geometry::Plane(Rc::clone(plane)));
        }
        for point in &objects.points {
            lookup.insert(point.guid().to_string(), Geometry::Point(Rc::clone(point)));
        }
        for pointcloud in &objects.pointclouds {
            lookup.insert(
                pointcloud.guid().to_string(),
                Geometry::PointCloud(Rc::clone(pointcloud)),
            );
        }
        for polyline in &objects.polylines {
            lookup.insert(polyline.guid().to_string(), Geometry::Polyline(Rc::clone(polyline)));
        }
        for nurbscurve in &objects.nurbscurves {
            lookup.insert(nurbscurve.guid().to_string(), Geometry::NurbsCurve(Rc::clone(nurbscurve)));
        }
        for nurbssurface in &objects.nurbssurfaces {
            lookup.insert(nurbssurface.guid().to_string(), Geometry::NurbsSurface(Rc::clone(nurbssurface)));
        }
        for brep in &objects.breps {
            lookup.insert(brep.guid().to_string(), Geometry::BRep(Rc::clone(brep)));
        }
        for elem in &objects.elements {
            lookup.insert(elem.guid().to_string(), Geometry::Element(Rc::clone(elem)));
        }

        let mut xforms = HashMap::new();
        if let Some(entries) = json_obj["xforms"].as_array() {
            for entry in entries {
                if let Some(guid) = entry["guid"].as_str() {
                    if let Ok(xform) = serde_json::from_value::<Xform>(entry["xform"].clone()) {
                        xforms.insert(guid.to_string(), xform);
                    }
                }
            }
        }

        let session = Session {
            guid: { let lock = std::sync::OnceLock::new(); let _ = lock.set(json_obj["guid"].as_str().unwrap_or("").to_string()); lock },
            name: json_obj["name"]
                .as_str()
                .unwrap_or("my_session")
                .to_string(),
            objects,
            lookup,
            tree,
            graph,
            xforms,
            bvh: SpatialBVH::new(),
            cached_ray_bvh: None,
            cached_guids: Vec::new(),
            cached_boxes: Vec::new(),
            bvh_cache_dirty: true,
        };

        Ok(session)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    pub fn file_json_dump(&self, filepath: &str) {
        let json = self.jsondump().unwrap_or_default();
        fs::write(filepath, json).expect("Failed to write JSON file");
    }

    pub fn file_json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");
        Self::jsonload(&json).unwrap_or_else(|_| Self::default())
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        // Build Objects proto — from the lookup-synced view (lookup is the mutable truth)
        let objects = self.objects_synced();
        let mut objects_proto = crate::proto::Objects {
            name: objects.name.clone(),
            guid: objects.guid().to_string(),
            ..Default::default()
        };
        // to_proto directly — the old decode(pb_dumps()) round-trip re-encoded and re-decoded
        // every object just to obtain the proto struct it was built from.
        for p in &objects.points { objects_proto.points.push(p.to_proto()); }
        for l in &objects.lines { objects_proto.lines.push(l.to_proto()); }
        for pl in &objects.planes { objects_proto.planes.push(pl.to_proto()); }
        for b in &objects.bboxes { objects_proto.bboxes.push(b.to_proto()); }
        for pl in &objects.polylines { objects_proto.polylines.push(pl.to_proto()); }
        for pc in &objects.pointclouds { objects_proto.pointclouds.push(pc.to_proto()); }
        for m in &objects.meshes { objects_proto.meshes.push(m.to_proto()); }
        for nc in &objects.nurbscurves { objects_proto.nurbscurves.push(nc.to_proto()); }
        for ns in &objects.nurbssurfaces { objects_proto.nurbssurfaces.push(ns.to_proto()); }
        for b in &objects.breps { objects_proto.breps.push(b.to_proto()); }
        for e in &objects.elements { objects_proto.elements.push(e.to_proto()); }

        // Build Tree proto
        fn treenode_to_proto(node: &Rc<RefCell<TreeNode>>) -> crate::proto::TreeNode {
            let b = node.borrow();
            let children: Vec<crate::proto::TreeNode> = b.children().iter().map(|c| treenode_to_proto(c)).collect();
            crate::proto::TreeNode {
                guid: b.guid().to_string(),
                name: b.name.clone(),
                parent_guid: b.parent().map(|p| p.borrow().guid().to_string()).unwrap_or_default(),
                children,
                color: None,
            }
        }
        let tree_proto = crate::proto::Tree {
            guid: self.tree.guid().to_string(),
            name: self.tree.name.clone(),
            root: self.tree.root().map(|r| treenode_to_proto(&r)),
        };

        // Build Graph proto
        let mut vertices_map: std::collections::BTreeMap<String, crate::proto::Vertex> = std::collections::BTreeMap::new();
        for v in self.graph.get_vertices() {
            vertices_map.insert(v.name.clone(), crate::proto::Vertex {
                name: v.name.clone(),
                guid: v.guid().to_string(),
                attribute: v.attribute.clone(),
                index: v.index,
            });
        }
        let mut edges_proto: Vec<crate::proto::Edge> = Vec::new();
        for (_u, neighbors) in &self.graph.edges {
            for (_v, edge) in neighbors {
                edges_proto.push(crate::proto::Edge {
                    guid: edge.guid().to_string(),
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
            guid: self.graph.guid().to_string(),
            vertices: vertices_map,
            edges: edges_proto,
            vertex_count: self.graph.vertex_count,
            edge_count: self.graph.edge_count,
        };

        // Xforms in canonical order() sequence — a map would not be deterministic
        let xforms_proto: Vec<crate::proto::XformEntry> = self
            .xforms_ordered()
            .into_iter()
            .map(|(guid, xform)| crate::proto::XformEntry {
                guid,
                xform: Some(crate::proto::Xform {
                    guid: xform.guid().to_string(),
                    name: xform.name.clone(),
                    matrix: xform.m.to_vec(),
                }),
            })
            .collect();

        let proto = crate::proto::Session {
            name: self.name.clone(),
            guid: self.guid().to_string(),
            objects: Some(objects_proto),
            tree: Some(tree_proto),
            graph: Some(graph_proto),
            bvh_boxes: Vec::new(),
            xforms: xforms_proto,
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Session::decode(data)?;

        let mut session = Session::new(&proto.name);
        session.set_guid(proto.guid.clone());

        // Rebuild objects — from_proto by value (moved out of the decoded Session proto); the
        // old path re-ENCODED each decoded proto object and decoded it a second time.
        if let Some(objects_proto) = proto.objects {
            session.objects.set_guid(objects_proto.guid);
            session.objects.name = objects_proto.name;
            for p in objects_proto.points {
                session.objects.points.push(Rc::new(Point::from_proto(p)));
            }
            for l in objects_proto.lines {
                session.objects.lines.push(Rc::new(Line::from_proto(l)));
            }
            for pl in objects_proto.planes {
                session.objects.planes.push(Rc::new(Plane::from_proto(pl)));
            }
            for b in objects_proto.bboxes {
                session.objects.bboxes.push(Rc::new(OBB::from_proto(b)?));
            }
            for pl in objects_proto.polylines {
                session.objects.polylines.push(Rc::new(Polyline::from_proto(pl)));
            }
            for pc in objects_proto.pointclouds {
                session.objects.pointclouds.push(Rc::new(PointCloud::from_proto(pc)));
            }
            for m in objects_proto.meshes {
                session.objects.meshes.push(Rc::new(Mesh::from_proto(m)));
            }
            for nc in objects_proto.nurbscurves {
                session.objects.nurbscurves.push(Rc::new(NurbsCurve::from_proto(nc)));
            }
            for ns in objects_proto.nurbssurfaces {
                session.objects.nurbssurfaces.push(Rc::new(NurbsSurface::from_proto(ns)?));
            }
            for b in objects_proto.breps {
                session.objects.breps.push(Rc::new(BRep::from_proto(b)?));
            }
            for e in objects_proto.elements {
                session.objects.elements.push(Rc::new(Element::from_proto(e)?));
            }
        }

        // Rebuild tree
        if let Some(tree_proto) = &proto.tree {
            session.tree = Tree::new(&tree_proto.name);
            session.tree.set_guid(tree_proto.guid.clone());
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
            session.graph.set_guid(graph_proto.guid.clone());
            for (name, v) in &graph_proto.vertices {
                session.graph.add_node(name, &v.attribute);
            }
            for e in &graph_proto.edges {
                session.graph.add_edge(&e.v0, &e.v1, &e.attribute);
            }
        }

        // Rebuild lookup
        for bbox in &session.objects.bboxes {
            session.lookup.insert(bbox.guid().to_string(), Geometry::OBB(Rc::clone(bbox)));
        }
        for line in &session.objects.lines {
            session.lookup.insert(line.guid().to_string(), Geometry::Line(Rc::clone(line)));
        }
        for mesh in &session.objects.meshes {
            session.lookup.insert(mesh.guid().to_string(), Geometry::Mesh(Rc::clone(mesh)));
        }
        for plane in &session.objects.planes {
            session.lookup.insert(plane.guid().to_string(), Geometry::Plane(Rc::clone(plane)));
        }
        for point in &session.objects.points {
            session.lookup.insert(point.guid().to_string(), Geometry::Point(Rc::clone(point)));
        }
        for pointcloud in &session.objects.pointclouds {
            session.lookup.insert(pointcloud.guid().to_string(), Geometry::PointCloud(Rc::clone(pointcloud)));
        }
        for polyline in &session.objects.polylines {
            session.lookup.insert(polyline.guid().to_string(), Geometry::Polyline(Rc::clone(polyline)));
        }
        for nurbscurve in &session.objects.nurbscurves {
            session.lookup.insert(nurbscurve.guid().to_string(), Geometry::NurbsCurve(Rc::clone(nurbscurve)));
        }
        for nurbssurface in &session.objects.nurbssurfaces {
            session.lookup.insert(nurbssurface.guid().to_string(), Geometry::NurbsSurface(Rc::clone(nurbssurface)));
        }
        for brep in &session.objects.breps {
            session.lookup.insert(brep.guid().to_string(), Geometry::BRep(Rc::clone(brep)));
        }
        for elem in &session.objects.elements {
            session.lookup.insert(elem.guid().to_string(), Geometry::Element(Rc::clone(elem)));
        }

        // Rebuild xforms
        for entry in &proto.xforms {
            if let Some(xf) = &entry.xform {
                let mut xform = Xform::identity();
                xform.set_guid(xf.guid.clone());
                xform.name = xf.name.clone();
                for (i, val) in xf.matrix.iter().enumerate().take(16) {
                    xform.m[i] = *val;
                }
                session.xforms.insert(entry.guid.clone(), xform);
            }
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
    // SpatialBVH Collision Detection
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Compute bounding box for a geometry object, inflated by tolerance
    /// Bounding box of an object in WORLD placement. `xform` is its cumulative transform from
    /// `world_xform` — the geometry itself stores no placement, so it must be supplied here.
    fn compute_bounding_box(geometry: &Geometry, xform: &Xform) -> OBB {
        let inflate = Tolerance::APPROXIMATION;
        let tp = |p: &Point| -> Point { xform.transform_point(p) };
        match geometry {
            Geometry::Point(p) => OBB::from_point(tp(p), inflate),
            Geometry::Line(l) => {
                let points = vec![tp(&l.start()), tp(&l.end())];
                OBB::from_points(&points, inflate)
            }
            Geometry::Polyline(pl) => {
                let points: Vec<Point> = pl.get_points().iter().map(|p| tp(p)).collect();
                OBB::from_points(&points, inflate)
            }
            Geometry::PointCloud(pc) => {
                let points: Vec<Point> = pc.get_points().iter().map(|p| tp(p)).collect();
                OBB::from_points(&points, inflate)
            }
            Geometry::Mesh(m) => {
                let points: Vec<Point> = m
                    .vertex
                    .values()
                    .map(|v| tp(&Point::new(v.x, v.y, v.z)))
                    .collect();
                if points.is_empty() {
                    OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate)
                } else {
                    OBB::from_points(&points, inflate)
                }
            }
            Geometry::OBB(bb) => {
                // Inflate existing bounding box
                let mut inflated = (**bb).clone();
                inflated.half_size = crate::Vector::new(
                    inflated.half_size[0] + inflate,
                    inflated.half_size[1] + inflate,
                    inflated.half_size[2] + inflate,
                );
                inflated.transform(xform);
                inflated
            }
            Geometry::Plane(p) => {
                OBB::from_point(tp(&p.origin()), inflate * 10.0)
            }
            Geometry::BRep(b) => {
                let mut points: Vec<Point> = b.m_vertices.iter().map(|p| tp(p)).collect();
                // Sample surface points to cover curved surfaces (e.g. sphere with only pole vertices)
                for srf in &b.m_surfaces {
                    if let (Some((u0, u1)), Some((v0, v1))) = (srf.domain(0), srf.domain(1)) {
                        for ui in 0..=2usize {
                            for vi in 0..=2usize {
                                let u = u0 + (u1 - u0) * ui as f64 / 2.0;
                                let v = v0 + (v1 - v0) * vi as f64 / 2.0;
                                if let Some(p) = srf.point_at(u, v) {
                                    points.push(tp(&p));
                                }
                            }
                        }
                    }
                }
                if points.is_empty() {
                    OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate)
                } else {
                    OBB::from_points(&points, inflate)
                }
            }
            Geometry::NurbsCurve(c) => {
                let mut points: Vec<Point> = Vec::new();
                for i in 0..c.cv_count() {
                    if let Some(p) = c.get_cv(i) {
                        points.push(tp(&p));
                    }
                }
                if points.is_empty() {
                    OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate)
                } else {
                    OBB::from_points(&points, inflate)
                }
            }
            Geometry::NurbsSurface(s) => {
                let mut points: Vec<Point> = Vec::new();
                for i in 0..s.cv_count_dir(Some(0)) {
                    for j in 0..s.cv_count_dir(Some(1)) {
                        if let Some(p) = s.get_cv(i, j) {
                            points.push(tp(&p));
                        }
                    }
                }
                if points.is_empty() {
                    OBB::from_point(Point::new(0.0, 0.0, 0.0), inflate)
                } else {
                    OBB::from_points(&points, inflate)
                }
            }
            Geometry::Element(e) => {
                let mut e2 = (**e).clone();
                let mut box_ = e2.aabb();
                box_.transform(xform);
                box_
            }
        }
    }

    /// Get all collision pairs using SpatialBVH and add them as graph edges.
    ///
    /// Automatically:
    /// - Computes bounding boxes for all objects with tolerance inflation
    /// - Builds/rebuilds the SpatialBVH with auto-computed world size
    /// - Detects all collision pairs
    /// - Adds collision edges to the graph
    ///
    /// # Returns
    /// A vector of tuples (guid1, guid2) representing colliding geometry pairs
    pub fn get_collisions(&mut self) -> Vec<(String, String)> {
        // Collect all objects with their bounding boxes and GUIDs
        let mut boxes_with_guids: Vec<(OBB, String)> = Vec::new();

        let world = self.world_xforms();
        for guid in self.order() {
            let Some(geometry) = self.lookup.get(&guid) else { continue };
            let bbox = Self::compute_bounding_box(geometry, world.get(&guid).unwrap_or(&Xform::identity()));
            boxes_with_guids.push((bbox, guid));
        }

        if boxes_with_guids.is_empty() {
            return Vec::new();
        }

        // Build SpatialBVH with GUIDs (auto-computes world size)
        self.bvh.build_with_guids(&boxes_with_guids);

        // Extract just the boxes for collision checking
        let boxes: Vec<OBB> = boxes_with_guids
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
    // Ray SpatialBVH Cache
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// LAZY: boxes are recomputed here from the document in canonical order() — always fresh
    /// (an object mutated through `lookup` gets a fresh box, unlike the old eager add-time
    /// cache), deterministic across runs and languages, and the boxes are LOCAL: the BVH
    /// copies them into its nodes, so nothing box-shaped is retained on Session
    /// (`cached_boxes` stays empty; kept only for API compatibility).
    fn rebuild_ray_bvh_cache(&mut self) {
        self.cached_boxes.clear();
        self.cached_guids.clear();
        let mut boxes: Vec<OBB> = Vec::with_capacity(self.lookup.len());
        let world = self.world_xforms();
        for guid in self.order() {
            if let Some(geometry) = self.lookup.get(&guid) {
                boxes.push(Self::compute_bounding_box(geometry, world.get(&guid).unwrap_or(&Xform::identity())));
                self.cached_guids.push(guid);
            }
        }
        if !boxes.is_empty() {
            let world_size = SpatialBVH::compute_world_size(&boxes);
            self.cached_ray_bvh = Some(SpatialBVH::from_boxes(&boxes, world_size));
        } else {
            self.cached_ray_bvh = None;
        }
    }

    pub fn invalidate_bvh_cache(&mut self) {
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

        // Use cached SpatialBVH for ray casting
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

        // Thin geometry (Line/Polyline/Point/PointCloud) has near-degenerate BVH
        // boxes (inflated by only 0.001mm) so the ray rarely hits them. Always add
        // them as candidates so the line_line / point distance tests run.
        for (idx, guid) in self.cached_guids.iter().enumerate() {
            if let Some(geom) = self.lookup.get(guid) {
                match geom {
                    Geometry::Line(_) | Geometry::Polyline(_)
                    | Geometry::Point(_) | Geometry::PointCloud(_) => {
                        if !candidates.contains(&idx) {
                            candidates.push(idx);
                        }
                    }
                    _ => {}
                }
            }
        }

        let mut hits_all: Vec<RayHit> = Vec::new();
        // Placements come from the session, not the geometry; resolve them all up front
        // because the loop below borrows `lookup` mutably.
        let world = self.world_xforms();

        for idx in candidates {
            if idx >= self.cached_guids.len() {
                continue;
            }
            let guid = self.cached_guids[idx].clone();
            let placement = world.get(&guid).cloned().unwrap_or_else(Xform::identity);
            let geom = match self.lookup.get_mut(&guid) {
                Some(g) => g,
                None => continue,
            };

            let mut hit_point: Option<Point> = None;

            match geom {
                Geometry::OBB(bb) => {
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
                        crate::intersection::line_line(&ray_line, l, tolerance)
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
                                tolerance,
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
                    // The session holds the placement: cast in the mesh's LOCAL frame, return a WORLD hit.
                    // COW only when the triangle BVH is missing — a shared mesh with a built BVH
                    // is cast through &self (no clone after every save/re-share).
                    if !m.has_triangle_bvh() {
                        Rc::make_mut(m).build_triangle_bvh();
                    }
                    if let Some(inv) = placement.inverse() {
                        let local_ray = Line::from_points(
                            &inv.transform_point(&ray_line.start()),
                            &inv.transform_point(&ray_line.end()),
                        );
                        if let Some(p) = m.ray_cast_bvh_ready(&local_ray, 1e-6) {
                            hit_point = Some(placement.transform_point(&p));
                        }
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
                Geometry::PointCloud(pc) => {
                    let pts = pc.get_points();
                    let mut best_t = f64::INFINITY;
                    let mut best_p: Option<Point> = None;
                    for p in &pts {
                        let vx = p[0] - origin[0];
                        let vy = p[1] - origin[1];
                        let vz = p[2] - origin[2];
                        let cross_x = vy * dir_unit[2] - vz * dir_unit[1];
                        let cross_y = vz * dir_unit[0] - vx * dir_unit[2];
                        let cross_z = vx * dir_unit[1] - vy * dir_unit[0];
                        let dist = (cross_x * cross_x + cross_y * cross_y + cross_z * cross_z).sqrt();
                        if dist <= tolerance {
                            let t = vx * dir_unit[0] + vy * dir_unit[1] + vz * dir_unit[2];
                            if t >= 0.0 && t < best_t {
                                best_t = t;
                                best_p = Some(Point::new(
                                    origin[0] + dir_unit[0] * t,
                                    origin[1] + dir_unit[1] * t,
                                    origin[2] + dir_unit[2] * t,
                                ));
                            }
                        }
                    }
                    if let Some(p) = best_p {
                        hit_point = Some(p);
                    }
                }
                Geometry::BRep(_) => {
                    // BRep tessellation is expensive (re-tessellates every call).
                    // Viewers must use pre-cached tessellations with pre-built BVH.
                    // hit_point stays None — callers handle BReps separately.
                }
                Geometry::NurbsCurve(_) => {
                    // Exact ray-curve intersection is out of scope; viewers pick via sampled polylines.
                }
                Geometry::NurbsSurface(_) => {
                    // Exact ray-surface intersection is out of scope; viewers pick via cached tessellations.
                }
                Geometry::Element(_) => {}
            }

            if let Some(hp) = hit_point {
                let dx = hp[0] - origin[0];
                let dy = hp[1] - origin[1];
                let dz = hp[2] - origin[2];
                let forward = dx * dir_unit[0] + dy * dir_unit[1] + dz * dir_unit[2];
                if forward >= 0.0 {
                    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                    let guid_lock = std::sync::OnceLock::new();
                    let _ = guid_lock.set(guid.clone());
                    hits_all.push(RayHit {
                        guid: guid_lock,
                        point: hp,
                        distance: dist,
                    });
                }
            }
        }

        hits_all.sort_by(|a, b| {
            a.distance.partial_cmp(&b.distance).unwrap_or(std::cmp::Ordering::Equal)
        });
        hits_all
    }

    /// Like `ray_cast` but filters to only hits within `tolerance` of the nearest.
    /// Useful when you want "the object at the cursor" without iterating through
    /// everything behind it.
    pub fn ray_cast_nearest(
        &mut self,
        origin: &Point,
        direction: &crate::Vector,
        tolerance: f64,
    ) -> Vec<RayHit> {
        let hits_all = self.ray_cast(origin, direction, tolerance);
        if hits_all.is_empty() { return Vec::new(); }
        let min_d = hits_all[0].distance;
        hits_all.into_iter().filter(|h| (h.distance - min_d).abs() <= tolerance).collect()
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
    pub fn add_point(&mut self, point: Point, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = point.guid().to_string();
        let name = point.name.clone();
        let point = Rc::new(point);
        self.objects.points.push(Rc::clone(&point));
        self.lookup.insert(guid.clone(), Geometry::Point(point));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("point_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_line(&mut self, line: Line, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = line.guid().to_string();
        let name = line.name.clone();
        let line = Rc::new(line);
        self.objects.lines.push(Rc::clone(&line));
        self.lookup.insert(guid.clone(), Geometry::Line(line));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("line_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_plane(&mut self, plane: Plane, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = plane.guid().to_string();
        let name = plane.name.clone();
        let plane = Rc::new(plane);
        self.objects.planes.push(Rc::clone(&plane));
        self.lookup.insert(guid.clone(), Geometry::Plane(plane));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("plane_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_obb(&mut self, bbox: OBB) -> Rc<RefCell<TreeNode>> {
        let guid = bbox.guid().to_string();
        let name = bbox.name.clone();
        let bbox = Rc::new(bbox);
        self.objects.bboxes.push(Rc::clone(&bbox));
        self.lookup.insert(guid.clone(), Geometry::OBB(bbox));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("bbox_{name}"));
        TreeNode::new(&guid)
    }

    pub fn add_polyline(&mut self, polyline: Polyline, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = polyline.guid().to_string();
        // Boxes are computed lazily in rebuild_ray_bvh_cache (from the canonical order) —
        // adds only mark the cache dirty.
        let label = format!("polyline_{}", polyline.name);
        let polyline = Rc::new(polyline);
        self.objects.polylines.push(Rc::clone(&polyline));
        self.lookup.insert(guid.clone(), Geometry::Polyline(polyline));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &label);
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_pointcloud(&mut self, pointcloud: PointCloud, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = pointcloud.guid().to_string();
        let name = pointcloud.name.clone();
        let pointcloud = Rc::new(pointcloud);
        self.objects.pointclouds.push(Rc::clone(&pointcloud));
        self.lookup.insert(guid.clone(), Geometry::PointCloud(pointcloud));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("pointcloud_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_mesh(&mut self, mesh: Mesh, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = mesh.guid().to_string();
        let name = mesh.name.clone();
        let mesh = Rc::new(mesh);
        self.objects.meshes.push(Rc::clone(&mesh));
        self.lookup.insert(guid.clone(), Geometry::Mesh(mesh));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("mesh_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_nurbscurve(&mut self, nurbscurve: NurbsCurve, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = nurbscurve.guid().to_string();
        let name = nurbscurve.name.clone();
        let nurbscurve = Rc::new(nurbscurve);
        self.objects.nurbscurves.push(Rc::clone(&nurbscurve));
        self.lookup.insert(guid.clone(), Geometry::NurbsCurve(nurbscurve));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("nurbscurve_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_nurbssurface(&mut self, nurbssurface: NurbsSurface, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = nurbssurface.guid().to_string();
        let name = nurbssurface.name.clone();
        let nurbssurface = Rc::new(nurbssurface);
        self.objects.nurbssurfaces.push(Rc::clone(&nurbssurface));
        self.lookup.insert(guid.clone(), Geometry::NurbsSurface(nurbssurface));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("nurbssurface_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_brep(&mut self, brep: BRep, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = brep.guid().to_string();
        let name = brep.name.clone();
        let brep = Rc::new(brep);
        self.objects.breps.push(Rc::clone(&brep));
        self.lookup.insert(guid.clone(), Geometry::BRep(brep));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("brep_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_component(&mut self, component: crate::objects::Component, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = component.guid().to_string();
        let name = component.name.clone();
        self.objects.components.push(component);
        self.graph.add_node(&guid, &format!("component_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
    }

    pub fn add_element(&mut self, element: Element, parent: Option<&Rc<RefCell<TreeNode>>>) -> Rc<RefCell<TreeNode>> {
        let guid = element.guid().to_string();
        let name = element.name.clone();
        let element = Rc::new(element);
        self.objects.elements.push(Rc::clone(&element));
        self.lookup.insert(guid.clone(), Geometry::Element(element));
        self.bvh_cache_dirty = true;
        self.graph.add_node(&guid, &format!("element_{name}"));
        let node = TreeNode::new(&guid);
        if let Some(p) = parent { self.tree.add(&node, Some(p)); }
        node
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

    /// Create a named group (TreeNode) and add it to the root of the tree.
    pub fn add_group(&mut self, name: &str) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(name);
        self.add(&node, None);
        node
    }

    /// Find an existing group by name. Panics if the group does not exist.
    pub fn find_group(&self, name: &str) -> Rc<RefCell<TreeNode>> {
        if let Some(root) = self.tree.root() {
            for child in root.borrow().children() {
                if child.borrow().name == name {
                    return child.clone();
                }
            }
        }
        panic!("Group '{}' not found", name);
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
        self.objects.points.retain(|p| p.guid() != guid);
        self.objects.lines.retain(|l| l.guid() != guid);
        self.objects.polylines.retain(|p| p.guid() != guid);
        self.objects.planes.retain(|p| p.guid() != guid);
        self.objects.bboxes.retain(|b| b.guid() != guid);
        self.objects.meshes.retain(|m| m.guid() != guid);
        self.objects.pointclouds.retain(|p| p.guid() != guid);
        self.objects.nurbscurves.retain(|c| c.guid() != guid);
        self.objects.nurbssurfaces.retain(|s| s.guid() != guid);
        self.objects.breps.retain(|b| b.guid() != guid);
        self.objects.elements.retain(|e| e.guid() != guid);

        // Remove from lookup table
        self.lookup.remove(guid);
        self.xforms.remove(guid);
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

    /// All geometry with its hierarchical placement BAKED into the coordinates.
    ///
    /// Each object is transformed by its cumulative `world_xform` — its own transform with
    /// every ancestor's multiplied down the tree. The result is a FLATTENED snapshot: every
    /// guid's world transform is identity by construction, so never pair it back with
    /// `self.xforms` or the placement would be applied twice.
    ///
    /// # Returns
    /// Objects collection with transformed geometry
    pub fn get_geometry(&self) -> Objects {
        let mut objects = self.objects_synced(); // lookup is the truth
        let world = self.world_xforms();

        // No type is skipped: Rust used to leave nurbs and elements un-baked and C++ left
        // breps un-baked, so a placement in the tree silently did nothing for them.
        macro_rules! bake {
            ($vec:expr) => {
                for item in $vec.iter_mut() {
                    let Some(xform) = world.get(item.guid()) else { continue };
                    if !xform.is_identity() {
                        Rc::make_mut(item).transform(xform);
                    }
                }
            };
        }
        bake!(objects.points);
        bake!(objects.lines);
        bake!(objects.planes);
        bake!(objects.bboxes);
        bake!(objects.polylines);
        bake!(objects.pointclouds);
        bake!(objects.meshes);
        bake!(objects.nurbscurves);
        bake!(objects.nurbssurfaces);
        bake!(objects.breps);

        // An Element holds its own geometry, so its placement is baked through session_geometry.
        for item in objects.elements.iter_mut() {
            let Some(xform) = world.get(item.guid()) else { continue };
            if !xform.is_identity() {
                Rc::make_mut(item).place(xform);
            }
        }

        objects
    }
}

impl fmt::Display for Session {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Session({}, {}, points={}, vertices={}, edges={})",
            self.name,
            self.guid(),
            self.objects.points.len(),
            self.graph.vertex_count,
            self.graph.edge_count
        )
    }
}

