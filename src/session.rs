use crate::history::{clone, History, Op, ReplaceOp, Tombstone, XformOp};
use crate::intersection::{line_line, line_plane, ray_box, ray_mesh_bvh};
use crate::objects::Component;
use crate::{
    BRep, Element, Graph, Line, Mesh, NurbsCurve, NurbsSurface, Objects, Plane, Point, PointCloud,
    Polyline, SpatialBVH, Tolerance, Tree, TreeNode, Vector, Xform, OBB,
};
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::fs;
use std::rc::Rc;

/// All geometry types as a variant; a new type joins here and in the Objects vectors.
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
    /// Returns the guid of the wrapped object.
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

    /// Returns the name of the wrapped object.
    pub fn name(&self) -> &str {
        match self {
            Geometry::OBB(g) => &g.name,
            Geometry::BRep(g) => &g.name,
            Geometry::Element(g) => &g.name,
            Geometry::Line(g) => &g.name,
            Geometry::Mesh(g) => &g.name,
            Geometry::NurbsCurve(g) => &g.name,
            Geometry::NurbsSurface(g) => &g.name,
            Geometry::Plane(g) => &g.name,
            Geometry::Point(g) => &g.name,
            Geometry::PointCloud(g) => &g.name,
            Geometry::Polyline(g) => &g.name,
        }
    }

    /// Overwrites the guid: a minted one is cleared first, since `set_guid` never replaces one.
    pub fn set_guid(&mut self, guid: &str) {
        macro_rules! reset {
            ($rc:expr) => {{
                let g = Rc::make_mut($rc);
                g.refresh_guid();
                g.set_guid(guid.to_string());
            }};
        }

        match self {
            Geometry::OBB(g) => reset!(g),
            Geometry::BRep(g) => reset!(g),
            Geometry::Element(g) => reset!(g),
            Geometry::Line(g) => reset!(g),
            Geometry::Mesh(g) => reset!(g),
            Geometry::NurbsCurve(g) => reset!(g),
            Geometry::NurbsSurface(g) => reset!(g),
            Geometry::Plane(g) => reset!(g),
            Geometry::Point(g) => reset!(g),
            Geometry::PointCloud(g) => reset!(g),
            Geometry::Polyline(g) => reset!(g),
        }
    }
}

/// Extracts a concrete geometry type out of a `Geometry` variant, what C++ gets from `std::get_if`.
pub trait FromGeometry: Sized {
    /// The object inside `geometry`, or None when the variant holds another type.
    fn from_geometry(geometry: &Geometry) -> Option<&Self>;
}

macro_rules! impl_from_geometry {
    ($($variant:ident => $type:ty),* $(,)?) => {
        $(impl FromGeometry for $type {
            fn from_geometry(geometry: &Geometry) -> Option<&Self> {

                match geometry {
                    Geometry::$variant(g) => Some(g.as_ref()),
                    _ => None,
                }
            }
        })*
    };
}

impl_from_geometry!(
    OBB => OBB,
    BRep => BRep,
    Element => Element,
    Line => Line,
    Mesh => Mesh,
    NurbsCurve => NurbsCurve,
    NurbsSurface => NurbsSurface,
    Plane => Plane,
    Point => Point,
    PointCloud => PointCloud,
    Polyline => Polyline,
);

/// The Objects vectors in `order()` sequence, each with the prefix of its graph node attribute.
pub const COLLECTIONS: [(&str, &str); 12] = [
    ("points", "point"),
    ("lines", "line"),
    ("planes", "plane"),
    ("bboxes", "bbox"),
    ("polylines", "polyline"),
    ("pointclouds", "pointcloud"),
    ("meshes", "mesh"),
    ("nurbscurves", "nurbscurve"),
    ("nurbssurfaces", "nurbssurface"),
    ("breps", "brep"),
    ("elements", "element"),
    ("components", "component"),
];

/// Runs `$op!(vec, Variant)` on the typed vector of that name, the Rust spelling of getattr(objects, collection).
macro_rules! typed {
    ($collection:expr, $objects:expr, $op:ident) => {
        match $collection {
            "points" => $op!($objects.points, Point),
            "lines" => $op!($objects.lines, Line),
            "planes" => $op!($objects.planes, Plane),
            "bboxes" => $op!($objects.bboxes, OBB),
            "polylines" => $op!($objects.polylines, Polyline),
            "pointclouds" => $op!($objects.pointclouds, PointCloud),
            "meshes" => $op!($objects.meshes, Mesh),
            "nurbscurves" => $op!($objects.nurbscurves, NurbsCurve),
            "nurbssurfaces" => $op!($objects.nurbssurfaces, NurbsSurface),
            "breps" => $op!($objects.breps, BRep),
            "elements" => $op!($objects.elements, Element),
            _ => {}
        }
    };
}

/// A deep copy of every vector and every object in it, guids included.
fn clone_objects(objects: &Objects) -> Objects {
    let mut out = objects.clone();

    for item in out.points.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.lines.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.planes.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.bboxes.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.polylines.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.pointclouds.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.meshes.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.nurbscurves.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.nurbssurfaces.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.breps.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    for item in out.elements.iter_mut() {
        *item = Rc::new((**item).clone());
    }

    out
}

/// Inflated box around the placed points, around the origin when there are none.
fn placed_box(points: &[Point], xform: &Xform, inflate: f64) -> OBB {
    if points.is_empty() {
        return OBB::from_point(&Point::new(0.0, 0.0, 0.0), inflate);
    }

    let mut placed = Vec::with_capacity(points.len());

    for point in points {
        placed.push(xform.transform_point(point));
    }

    OBB::from_points(&placed, inflate, None)
}

/// The point on the ray closest to point when it lies ahead and within tolerance.
fn ray_point(ray: &Line, point: &Point, tolerance: f64) -> Option<Point> {
    let ray_dir = ray.end() - ray.start();
    let to_point = point - &ray.start();
    let t = to_point.dot(&ray_dir) / ray_dir.dot(&ray_dir);

    if t < 0.0 {
        return None;
    }

    let closest = ray.start() + ray_dir * t;

    if point.distance(&closest, None) > tolerance {
        return None;
    }

    Some(closest)
}

/// One object a ray touched: which one, where, and how far from the ray origin.
#[derive(Debug, Clone)]
pub struct RayHit {
    pub guid: String,     // GUID of the hit object.
    pub hit_point: Point, // Intersection point in world coordinates.
    pub distance: f64,    // Distance from the ray origin.
}

/// A session containing geometry objects.
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Session")]
pub struct Session {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: std::sync::OnceLock<String>, // Lazily minted guid.
    pub name: String,     // The name of the session.
    pub objects: Objects, // Collection of geometry objects.
    #[serde(skip)]
    pub lookup: HashMap<String, Geometry>, // Fast lookup table for geometry by GUID.
    pub tree: Tree,       // Tree structure for hierarchy.
    pub graph: Graph,     // Graph structure for relationships.
    #[serde(skip)]
    pub component_lookup: HashMap<String, Component>, // Fast lookup table for components by GUID.
    #[serde(skip)]
    pub xforms: HashMap<String, Xform>, // LOCAL transform per guid, relative to the parent.
    #[serde(skip)]
    pub history: History, // Undo/redo buffer, in memory only; every save purges it.
    #[serde(skip)]
    pub bvh: SpatialBVH, // Bounding volume hierarchy for collision detection.
    #[serde(skip)]
    pub cached_ray_bvh: Option<SpatialBVH>, // Cached SpatialBVH for ray casting.
    #[serde(skip)]
    pub cached_guids: Vec<String>, // GUID per leaf of cached_ray_bvh.
    #[serde(skip)]
    pub cached_boxes: Vec<OBB>, // Box per leaf of cached_ray_bvh.
    #[serde(skip)]
    pub bvh_cache_dirty: bool, // Flag to rebuild cached_ray_bvh.
}

impl Default for Session {
    /// Constructs a session named "my_session".
    fn default() -> Self {
        Self::new("my_session")
    }
}

impl Clone for Session {
    /// Copies every table and object, guids included; caches are rebuilt on demand and history starts empty.
    fn clone(&self) -> Self {
        let mut session = Session::new(&self.name);

        if self.has_guid() {
            session.set_guid(self.guid().to_string());
        }

        session.objects = clone_objects(&self.objects);
        session.tree = self.tree.clone();
        session.graph = self.graph.clone();
        session.xforms = self.xforms.clone();
        session._index_objects();

        session
    }
}

impl Session {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Constructs an empty session whose tree root carries the session name.
    pub fn new(name: &str) -> Self {
        let mut tree = Tree::new(&format!("{name}_tree"));
        tree.add(&TreeNode::new(name), None);

        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            objects: Objects::new(),
            lookup: HashMap::new(),
            tree,
            graph: Graph::new(&format!("{name}_graph")),
            component_lookup: HashMap::new(),
            xforms: HashMap::new(),
            history: History::new(),
            bvh: SpatialBVH::new(),
            cached_ray_bvh: None,
            cached_guids: Vec::new(),
            cached_boxes: Vec::new(),
            bvh_cache_dirty: true,
        }
    }

    /// Returns whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Returns the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Sets the guid if it has not already been created.
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Gets a geometry object by GUID, None when there is none.
    pub fn get_object(&self, guid: &str) -> Option<&Geometry> {
        self.lookup.get(guid)
    }

    /// Selects objects of one type, grouped by the top-level nodes of the tree.
    pub fn select_by_type<T: FromGeometry + Clone>(&self) -> Vec<Vec<T>> {
        let mut groups: Vec<Vec<T>> = Vec::new();
        let Some(root) = self.tree.root() else {
            return groups;
        };
        let children = root.borrow().children();

        for group in children {
            let mut items: Vec<T> = Vec::new();
            let descendants = group.borrow().descendants();

            for node in descendants {
                let name = node.borrow().name.clone();

                if let Some(object) = self.get_object(&name).and_then(T::from_geometry) {
                    items.push(object.clone());
                }
            }

            if !items.is_empty() {
                groups.push(items);
            }
        }

        groups
    }

    /// Finds an existing group by name; panics when there is none.
    pub fn find_group(&self, group_name: &str) -> Rc<RefCell<TreeNode>> {
        if let Some(root) = self.tree.root() {
            for child in root.borrow().children() {
                if child.borrow().name == group_name {
                    return child.clone();
                }
            }
        }

        panic!("Group '{}' not found", group_name);
    }

    /// Canonical object order: the objects vectors walked in one fixed type sequence.
    pub fn order(&self) -> Vec<String> {
        let mut order = Vec::with_capacity(self.lookup.len());

        for point in &self.objects.points {
            order.push(point.guid().to_string());
        }

        for line in &self.objects.lines {
            order.push(line.guid().to_string());
        }

        for plane in &self.objects.planes {
            order.push(plane.guid().to_string());
        }

        for bbox in &self.objects.bboxes {
            order.push(bbox.guid().to_string());
        }

        for polyline in &self.objects.polylines {
            order.push(polyline.guid().to_string());
        }

        for pointcloud in &self.objects.pointclouds {
            order.push(pointcloud.guid().to_string());
        }

        for mesh in &self.objects.meshes {
            order.push(mesh.guid().to_string());
        }

        for nurbscurve in &self.objects.nurbscurves {
            order.push(nurbscurve.guid().to_string());
        }

        for nurbssurface in &self.objects.nurbssurfaces {
            order.push(nurbssurface.guid().to_string());
        }

        for brep in &self.objects.breps {
            order.push(brep.guid().to_string());
        }

        for element in &self.objects.elements {
            order.push(element.guid().to_string());
        }

        order
    }

    /// The LOCAL transform of an object, identity when none was set.
    pub fn xform(&self, guid: &str) -> Xform {
        self.xforms
            .get(guid)
            .cloned()
            .unwrap_or_else(Xform::identity)
    }

    /// The CUMULATIVE placement of an object: every ancestor's transform multiplied down the tree onto its own.
    pub fn world_xform(&self, guid: &str) -> Xform {
        let mut acc = self.xform(guid);
        let Some(node) = self.tree.get_node_by_name(guid) else {
            return acc;
        };

        for ancestor in node.borrow().ancestors() {
            let name = ancestor.borrow().name.clone();

            if let Some(xform) = self.xforms.get(&name) {
                acc = xform * &acc;
            }
        }

        acc
    }

    /// Every object's cumulative placement, computed in one downward pass.
    pub fn world_xforms(&self) -> HashMap<String, Xform> {
        let mut out: HashMap<String, Xform> = HashMap::new();

        if self.xforms.is_empty() {
            return out;
        }

        let mut stack: Vec<(Rc<RefCell<TreeNode>>, Xform)> = Vec::new();

        if let Some(root) = self.tree.root() {
            stack.push((root, Xform::identity()));
        }

        while let Some((node, parent_xform)) = stack.pop() {
            let name = node.borrow().name.clone();
            let current = match self.xforms.get(&name) {
                Some(local) => &parent_xform * local,
                None => parent_xform.clone(),
            };
            out.insert(name, current.clone());

            for child in node.borrow().children() {
                stack.push((child, current.clone()));
            }
        }

        for (obj_guid, obj_xform) in &self.xforms {
            out.entry(obj_guid.clone())
                .or_insert_with(|| obj_xform.clone());
        }

        out
    }

    /// Gets the children of a parent GUID.
    pub fn get_children(&self, obj_guid: &str) -> Vec<String> {
        self.tree.get_children_guids(obj_guid)
    }

    /// Gets the neighbours of a GUID.
    pub fn get_neighbours(&self, obj_guid: &str) -> Vec<String> {
        self.graph.neighbors(obj_guid)
    }

    /// All geometry with its hierarchical placement BAKED into the coordinates.
    pub fn get_geometry(&self) -> Objects {
        let mut out = clone_objects(&self.objects_synced());
        let world = self.world_xforms();
        macro_rules! bake {
            ($vec:expr) => {
                for item in $vec.iter_mut() {
                    let Some(xform) = world.get(item.guid()) else {
                        continue;
                    };

                    if !xform.is_identity() {
                        Rc::make_mut(item).transform(xform);
                    }
                }
            };
        }

        bake!(out.points);
        bake!(out.lines);
        bake!(out.planes);
        bake!(out.bboxes);
        bake!(out.polylines);
        bake!(out.pointclouds);
        bake!(out.meshes);
        bake!(out.nurbscurves);
        bake!(out.nurbssurfaces);
        bake!(out.breps);

        for element in out.elements.iter_mut() {
            let Some(xform) = world.get(element.guid()) else {
                continue;
            };

            if !xform.is_identity() {
                Rc::make_mut(element).place(xform);
            }
        }

        out
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry management
    // ═══════════════════════════════════════════════════════════════════════════

    /// Adds a point.
    pub fn add_point(
        &mut self,
        point: Point,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("points", Geometry::Point(Rc::new(point)), "point", parent)
    }

    /// Adds a line.
    pub fn add_line(
        &mut self,
        line: Line,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("lines", Geometry::Line(Rc::new(line)), "line", parent)
    }

    /// Adds a plane.
    pub fn add_plane(
        &mut self,
        plane: Plane,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("planes", Geometry::Plane(Rc::new(plane)), "plane", parent)
    }

    /// Adds a bounding box.
    pub fn add_obb(&mut self, bbox: OBB) -> Rc<RefCell<TreeNode>> {
        self._add_object("bboxes", Geometry::OBB(Rc::new(bbox)), "bbox", None)
    }

    /// Adds a polyline; fewer than two points adds nothing and returns None.
    pub fn add_polyline(
        &mut self,
        polyline: Polyline,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if polyline.point_count() < 2 {
            return None;
        }

        Some(self._add_object(
            "polylines",
            Geometry::Polyline(Rc::new(polyline)),
            "polyline",
            parent,
        ))
    }

    /// Adds a point cloud; no points adds nothing and returns None.
    pub fn add_pointcloud(
        &mut self,
        pointcloud: PointCloud,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if pointcloud.is_empty() {
            return None;
        }

        Some(self._add_object(
            "pointclouds",
            Geometry::PointCloud(Rc::new(pointcloud)),
            "pointcloud",
            parent,
        ))
    }

    /// Adds a mesh; no faces adds nothing and returns None.
    pub fn add_mesh(
        &mut self,
        mesh: Mesh,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if mesh.is_empty() || mesh.number_of_faces() == 0 {
            return None;
        }

        Some(self._add_object("meshes", Geometry::Mesh(Rc::new(mesh)), "mesh", parent))
    }

    /// Adds a curve; fewer than two control vertices adds nothing and returns None.
    pub fn add_nurbscurve(
        &mut self,
        nurbscurve: NurbsCurve,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if nurbscurve.cv_count() < 2 {
            return None;
        }

        Some(self._add_object(
            "nurbscurves",
            Geometry::NurbsCurve(Rc::new(nurbscurve)),
            "nurbscurve",
            parent,
        ))
    }

    /// Adds a surface; no control vertices adds nothing and returns None.
    pub fn add_nurbssurface(
        &mut self,
        nurbssurface: NurbsSurface,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if nurbssurface.cv_count_total() == 0 {
            return None;
        }

        Some(self._add_object(
            "nurbssurfaces",
            Geometry::NurbsSurface(Rc::new(nurbssurface)),
            "nurbssurface",
            parent,
        ))
    }

    /// Adds a brep; no faces and no vertices adds nothing and returns None.
    pub fn add_brep(
        &mut self,
        brep: BRep,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if brep.face_count() == 0 && brep.vertex_count() == 0 {
            return None;
        }

        Some(self._add_object("breps", Geometry::BRep(Rc::new(brep)), "brep", parent))
    }

    /// Adds an element; an Element is a data record kept even without geometry.
    pub fn add_element(
        &mut self,
        element: Element,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object(
            "elements",
            Geometry::Element(Rc::new(element)),
            "element",
            parent,
        )
    }

    /// Adds a custom component (any object with type_name/guid/name/extra); it has no `Geometry` variant, so it is outside the history.
    pub fn add_component(
        &mut self,
        component: Component,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        let guid = component.guid().to_string();
        let attribute = format!("component_{}", component.name);
        self.objects.components.push(component.clone());
        self.component_lookup.insert(guid.clone(), component);
        self.graph.add_node(&guid, &attribute);
        let node = TreeNode::new(&guid);

        if let Some(p) = parent {
            self.add(&node, Some(p));
        }

        node
    }

    /// Adds a TreeNode to the tree hierarchy, under the root when no parent is given.
    pub fn add<'a>(
        &mut self,
        node: &Rc<RefCell<TreeNode>>,
        parent: impl Into<Option<&'a Rc<RefCell<TreeNode>>>>,
    ) where
        Rc<RefCell<TreeNode>>: 'a,
    {
        let parent = parent.into();

        if parent.is_none() {
            if let Some(root) = self.tree.root() {
                self.tree.add(node, Some(&root));
            }
        } else {
            self.tree.add(node, parent);
        }
    }

    /// Creates a named group (TreeNode) and adds it to the root of the tree.
    pub fn add_group(&mut self, group_name: &str) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(group_name);
        self.add(&node, None);

        node
    }

    /// Adds an edge between two geometry objects in the graph.
    pub fn add_edge(&mut self, guid1: &str, guid2: &str, attribute: &str) {
        self.graph.add_edge(guid1, guid2, attribute);
    }

    /// Adds a parent-child relationship in the tree.
    pub fn add_hierarchy(&mut self, parent_guid: &str, child_guid: &str) -> bool {
        self.tree.add_child_by_guid(parent_guid, child_guid)
    }

    /// Adds a relationship edge in the graph.
    pub fn add_relationship(&mut self, from_guid: &str, to_guid: &str, relationship_type: &str) {
        self.graph.add_edge(from_guid, to_guid, relationship_type);
    }

    /// Removes an object by its GUID from every live table at once; the removal record is the tombstone undo restores from.
    pub fn remove_object(&mut self, obj_guid: &str) -> bool {
        let Some(op) = self._detach(obj_guid) else {
            return false;
        };
        self.history.record(Op::Remove(op));

        true
    }

    /// Swaps the object stored under guid for obj, which takes over that guid; the recorded edit undo and redo restore as absolute snapshots.
    pub fn replace(&mut self, guid: &str, obj: Geometry) -> bool {
        let Some(before) = self.lookup.get(guid) else {
            return false;
        };
        let mut obj = obj;
        obj.set_guid(guid);

        if self.history.current.is_some() {
            self.history.record(Op::Replace(ReplaceOp::new(
                guid.to_string(),
                clone(before),
                clone(&obj),
            )));
        }

        self._swap(guid, obj);

        true
    }

    /// Sets the LOCAL transform of an object, relative to its tree parent.
    pub fn set_xform(&mut self, guid: &str, xform: Xform) {
        if self.history.current.is_some() {
            let before = self.xforms.get(guid).cloned();
            self.history.record(Op::Xform(XformOp::new(
                guid.to_string(),
                before,
                Some(xform.clone()),
            )));
        }

        self.xforms.insert(guid.to_string(), xform);
        self.bvh_cache_dirty = true;
    }

    /// Removes an object's local transform, returning whether one was present.
    pub fn remove_xform(&mut self, guid: &str) -> bool {
        let Some(before) = self.xforms.get(guid).cloned() else {
            return false;
        };

        if self.history.current.is_some() {
            self.history.record(Op::Xform(XformOp::new(
                guid.to_string(),
                Some(before),
                None,
            )));
        }

        self.xforms.remove(guid);
        self.bvh_cache_dirty = true;

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // History
    // ═══════════════════════════════════════════════════════════════════════════

    /// Opens a history transaction: every add, remove, replace and xform change until commit becomes one undo step.
    pub fn begin(&mut self, label: &str) {
        self.history.begin(label);
    }

    /// Closes the open transaction as one undo step.
    pub fn commit(&mut self) {
        self.history.commit();
    }

    /// Reverts the latest committed transaction, returning whether there was one; the buffer is taken out for the call since it walks this session.
    pub fn undo(&mut self) -> bool {
        let mut history = std::mem::take(&mut self.history);
        let undone = history.undo(self);
        self.history = history;

        undone
    }

    /// Reapplies the latest undone transaction, returning whether there was one.
    pub fn redo(&mut self) -> bool {
        let mut history = std::mem::take(&mut self.history);
        let redone = history.redo(self);
        self.history = history;

        redone
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Collision detection and ray casting
    // ═══════════════════════════════════════════════════════════════════════════

    /// Bounding box of an object in WORLD placement, inflated by tolerance.
    pub fn compute_bounding_box(geometry: &Geometry, xform: &Xform) -> OBB {
        let inflate = Tolerance::APPROXIMATION;
        let mut points: Vec<Point> = Vec::new();

        match geometry {
            Geometry::Point(point) => {
                return OBB::from_point(&xform.transform_point(point), inflate);
            }

            Geometry::Plane(plane) => {
                return OBB::from_point(&xform.transform_point(&plane.origin()), inflate * 10.0);
            }

            Geometry::OBB(bbox) => {
                let mut inflated = (**bbox).clone();
                inflated.half_size = &inflated.half_size + &Vector::new(inflate, inflate, inflate);
                inflated.transform(xform);

                return inflated;
            }

            Geometry::Element(element) => {
                let mut copy = (**element).clone();
                let mut bbox = copy.aabb();
                bbox.transform(xform);

                return bbox;
            }

            Geometry::Line(line) => {
                points.push(line.start());
                points.push(line.end());
            }

            Geometry::Polyline(polyline) => points = polyline.get_points(),
            Geometry::PointCloud(pointcloud) => points = pointcloud.get_points(),
            Geometry::Mesh(mesh) => {
                for vertex in mesh.vertex.values() {
                    points.push(vertex.position());
                }
            }

            Geometry::BRep(brep) => {
                for vertex in &brep.m_vertices {
                    points.push(vertex.point.clone());
                }

                for surface in &brep.m_surfaces {
                    let (Some((u0, u1)), Some((v0, v1))) = (surface.domain(0), surface.domain(1))
                    else {
                        continue;
                    };

                    for i in 0..=2usize {
                        for j in 0..=2usize {
                            let u = u0 + (u1 - u0) * i as f64 / 2.0;
                            let v = v0 + (v1 - v0) * j as f64 / 2.0;

                            if let Some(point) = surface.point_at(u, v) {
                                points.push(point);
                            }
                        }
                    }
                }
            }

            Geometry::NurbsCurve(nurbscurve) => {
                for i in 0..nurbscurve.cv_count() {
                    if let Some(point) = nurbscurve.get_cv(i) {
                        points.push(point);
                    }
                }
            }

            Geometry::NurbsSurface(nurbssurface) => {
                for i in 0..nurbssurface.cv_count(0) {
                    for j in 0..nurbssurface.cv_count(1) {
                        if let Some(point) = nurbssurface.get_cv(i, j) {
                            points.push(point);
                        }
                    }
                }
            }
        }

        placed_box(&points, xform, inflate)
    }

    /// Gets all collision pairs using SpatialBVH and adds them as graph edges.
    pub fn get_collisions(&mut self) -> Vec<(String, String)> {
        let mut guids: Vec<String> = Vec::new();
        let boxes = self._compute_boxes(&mut guids);

        if boxes.is_empty() {
            return Vec::new();
        }

        self.bvh = SpatialBVH::from_boxes(&boxes, SpatialBVH::compute_world_size(&boxes));
        let (pairs, _colliding, _checks) = self.bvh.check_all_collisions(&boxes);
        let mut guid_pairs: Vec<(String, String)> = Vec::with_capacity(pairs.len());

        for (i, j) in pairs {
            if i >= guids.len() || j >= guids.len() {
                continue;
            }

            guid_pairs.push((guids[i].clone(), guids[j].clone()));
            self.graph.add_edge(&guids[i], &guids[j], "bvh_collision");
        }

        guid_pairs
    }

    /// Casts a ray through the scene, returning the hits within tolerance of the closest one.
    pub fn ray_cast(&mut self, origin: &Point, direction: &Vector, tolerance: f64) -> Vec<RayHit> {
        if self.bvh_cache_dirty {
            self._rebuild_ray_bvh_cache();
            self.bvh_cache_dirty = false;
        }

        if self.cached_guids.is_empty() {
            return Vec::new();
        }

        let mut candidates: Vec<usize> = Vec::new();

        if let Some(bvh) = &self.cached_ray_bvh {
            bvh.ray_cast(origin, direction, &mut candidates, true);
        }

        let ray = Line::from_points(origin, &(origin + direction * 10000.0));
        let world = self.world_xforms();
        let mut hits: Vec<RayHit> = Vec::new();
        let mut closest = f64::INFINITY;

        for index in candidates {
            let guid = self.cached_guids[index].clone();
            let Some(geometry) = self.lookup.get(&guid) else {
                continue;
            };
            let placement = world.get(&guid).cloned().unwrap_or_else(Xform::identity);
            let Some(hit) = self._ray_intersect_geometry(&ray, geometry, tolerance, &placement)
            else {
                continue;
            };
            let distance = origin.distance(&hit, None);

            if distance >= closest {
                continue;
            }

            if distance < closest - tolerance {
                hits.clear();
            }

            hits.push(RayHit {
                guid,
                hit_point: hit,
                distance,
            });
            closest = distance;
        }

        hits
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serializes to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let graph_json: serde_json::Value = serde_json::from_str(&self.graph.jsondump()?)?;
        let mut xforms_json: Vec<serde_json::Value> = Vec::new();

        for (obj_guid, obj_xform) in self._xforms_ordered() {
            xforms_json.push(serde_json::json!({ "guid": obj_guid, "xform": obj_xform }));
        }

        let json_obj = serde_json::json!({
            "type": "Session",
            "name": self.name,
            "guid": self.guid(),
            "objects": self.objects_synced(),
            "tree": self.tree,
            "graph": graph_json,
            "xforms": xforms_json
        });
        let sorted = crate::file_encoders::sort_json_keys(json_obj);
        let mut buf: Vec<u8> = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        serde::Serialize::serialize(&sorted, &mut ser)?;

        Ok(String::from_utf8(buf)?)
    }

    /// Deserializes from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json_obj: serde_json::Value = serde_json::from_str(json_data)?;
        let mut session = Session::new(json_obj["name"].as_str().unwrap_or("my_session"));

        if let Some(guid) = json_obj["guid"].as_str() {
            session.set_guid(guid.to_string());
        }

        if json_obj["objects"].is_object() {
            session.objects = serde_json::from_value(json_obj["objects"].clone())?;
        }

        if json_obj["tree"].is_object() {
            session.tree = serde_json::from_value(json_obj["tree"].clone())?;
        }

        if json_obj["graph"].is_object() {
            session.graph = Graph::jsonload(&serde_json::to_string(&json_obj["graph"])?)?;
        }

        session._index_objects();

        if let Some(entries) = json_obj["xforms"].as_array() {
            for entry in entries {
                let Some(guid) = entry["guid"].as_str() else {
                    continue;
                };
                let xform: Xform = serde_json::from_value(entry["xform"].clone())?;
                session.xforms.insert(guid.to_string(), xform);
            }
        }

        Ok(session)
    }

    /// Serializes to a JSON string, purging the history.
    pub fn file_json_dumps(&mut self) -> String {
        self.history.clear();

        self.jsondump().unwrap_or_default()
    }

    /// Deserializes from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Writes to a JSON file.
    pub fn file_json_dump(&mut self, filename: &str) {
        self.history.clear();
        fs::write(filename, self.jsondump().unwrap_or_default())
            .expect("Failed to write JSON file");
    }

    /// Reads from a JSON file.
    pub fn file_json_load(filename: &str) -> Self {
        let json = fs::read_to_string(filename).expect("Failed to read JSON file");

        Self::jsonload(&json).unwrap_or_else(|_| Self::default())
    }

    /// Serializes to protobuf bytes.
    pub fn pb_dumps(&mut self) -> Vec<u8> {
        use prost::Message;
        self.history.clear();
        let objects = self.objects_synced();
        let mut objects_proto = crate::proto::Objects {
            name: objects.name.clone(),
            guid: objects.guid().to_string(),
            ..Default::default()
        };

        for point in &objects.points {
            objects_proto.points.push(point.to_proto());
        }

        for line in &objects.lines {
            objects_proto.lines.push(line.to_proto());
        }

        for plane in &objects.planes {
            objects_proto.planes.push(plane.to_proto());
        }

        for bbox in &objects.bboxes {
            objects_proto.bboxes.push(bbox.to_proto());
        }

        for polyline in &objects.polylines {
            objects_proto.polylines.push(polyline.to_proto());
        }

        for pointcloud in &objects.pointclouds {
            objects_proto.pointclouds.push(pointcloud.to_proto());
        }

        for mesh in &objects.meshes {
            objects_proto.meshes.push(mesh.to_proto());
        }

        for nurbscurve in &objects.nurbscurves {
            objects_proto.nurbscurves.push(nurbscurve.to_proto());
        }

        for nurbssurface in &objects.nurbssurfaces {
            objects_proto.nurbssurfaces.push(nurbssurface.to_proto());
        }

        for brep in &objects.breps {
            objects_proto.breps.push(brep.to_proto());
        }

        for element in &objects.elements {
            objects_proto.elements.push(element.to_proto());
        }

        for component in &objects.components {
            objects_proto.components.push(
                crate::proto::Component::decode(component.pb_dumps().as_slice())
                    .unwrap_or_default(),
            );
        }

        let mut xforms_proto: Vec<crate::proto::XformEntry> = Vec::new();

        for (obj_guid, obj_xform) in self._xforms_ordered() {
            xforms_proto.push(crate::proto::XformEntry {
                guid: obj_guid,
                xform: Some(crate::proto::Xform {
                    guid: obj_xform.guid().to_string(),
                    name: obj_xform.name.clone(),
                    matrix: obj_xform.m.to_vec(),
                }),
            });
        }

        let proto = crate::proto::Session {
            name: self.name.clone(),
            guid: self.guid.get().cloned().unwrap_or_default(),
            objects: Some(objects_proto),
            tree: crate::proto::Tree::decode(self.tree.pb_dumps().as_slice()).ok(),
            graph: Some(self.graph.to_proto()),
            bvh_boxes: Vec::new(),
            xforms: xforms_proto,
        };

        proto.encode_to_vec()
    }

    /// Deserializes from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Session::decode(data)?;
        let mut session = Session::new(&proto.name);

        if !proto.guid.is_empty() {
            session.set_guid(proto.guid.clone());
        }

        if let Some(objects_proto) = proto.objects {
            session.objects.set_guid(objects_proto.guid);
            session.objects.name = objects_proto.name;

            for point in objects_proto.points {
                session
                    .objects
                    .points
                    .push(Rc::new(Point::from_proto(point)));
            }

            for line in objects_proto.lines {
                session.objects.lines.push(Rc::new(Line::from_proto(line)));
            }

            for plane in objects_proto.planes {
                session
                    .objects
                    .planes
                    .push(Rc::new(Plane::from_proto(plane)));
            }

            for bbox in objects_proto.bboxes {
                session.objects.bboxes.push(Rc::new(OBB::from_proto(bbox)?));
            }

            for polyline in objects_proto.polylines {
                session
                    .objects
                    .polylines
                    .push(Rc::new(Polyline::from_proto(polyline)));
            }

            for pointcloud in objects_proto.pointclouds {
                session
                    .objects
                    .pointclouds
                    .push(Rc::new(PointCloud::from_proto(pointcloud)));
            }

            for mesh in objects_proto.meshes {
                session.objects.meshes.push(Rc::new(Mesh::from_proto(mesh)));
            }

            for nurbscurve in objects_proto.nurbscurves {
                session
                    .objects
                    .nurbscurves
                    .push(Rc::new(NurbsCurve::from_proto(nurbscurve)));
            }

            for nurbssurface in objects_proto.nurbssurfaces {
                session
                    .objects
                    .nurbssurfaces
                    .push(Rc::new(NurbsSurface::from_proto(nurbssurface)?));
            }

            for brep in objects_proto.breps {
                session.objects.breps.push(Rc::new(BRep::from_proto(brep)?));
            }

            for element in objects_proto.elements {
                session
                    .objects
                    .elements
                    .push(Rc::new(Element::from_proto(element)?));
            }

            for component in objects_proto.components {
                session
                    .objects
                    .components
                    .push(Component::pb_loads(&component.encode_to_vec())?);
            }
        }

        if let Some(tree_proto) = &proto.tree {
            session.tree = Tree::pb_loads(&tree_proto.encode_to_vec())?;
        }

        if let Some(graph_proto) = &proto.graph {
            session.graph = Graph::pb_loads(&graph_proto.encode_to_vec())?;
        }

        session._index_objects();

        for entry in &proto.xforms {
            let Some(xform_proto) = &entry.xform else {
                continue;
            };
            session.xforms.insert(
                entry.guid.clone(),
                Xform::pb_loads(&xform_proto.encode_to_vec())?,
            );
        }

        Ok(session)
    }

    /// Writes to a protobuf file.
    pub fn pb_dump(&mut self, filename: &str) {
        self.history.clear();
        fs::write(filename, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Reads from a protobuf file.
    pub fn pb_load(filename: &str) -> Self {
        let data = fs::read(filename).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════

    /// The objects vectors re-pointed at `lookup`: `Rc::make_mut` on a lookup entry splits it from the vector, and the lookup is the mutable truth.
    fn objects_synced(&self) -> Objects {
        let mut objects = self.objects.clone();
        macro_rules! sync {
            ($vec:expr, $variant:ident) => {
                for item in $vec.iter_mut() {
                    if let Some(Geometry::$variant(g)) = self.lookup.get(item.guid()) {
                        if !Rc::ptr_eq(item, g) {
                            *item = Rc::clone(g);
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

    /// Stores an object in its typed vector, lookup, graph and tree, recording an AddOp when a transaction is open.
    fn _add_object(
        &mut self,
        collection: &str,
        obj: Geometry,
        type_prefix: &str,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        let guid = obj.guid().to_string();
        let attribute = format!("{type_prefix}_{}", obj.name());
        let mut obj_index: i64 = 0;
        macro_rules! push {
            ($vec:expr, $variant:ident) => {
                if let Geometry::$variant(g) = &obj {
                    $vec.push(Rc::clone(g));
                    obj_index = $vec.len() as i64 - 1;
                }
            };
        }

        typed!(collection, self.objects, push);
        let snapshot = self.history.current.is_some().then(|| clone(&obj));
        self.lookup.insert(guid.clone(), obj);
        self.graph.add_node(&guid, &attribute);
        self.bvh_cache_dirty = true;
        let node = TreeNode::new(&guid);
        let mut parent_guid: Option<String> = None;
        let mut index: usize = 0;

        if let Some(p) = parent {
            self.add(&node, Some(p));
            parent_guid = Some(p.borrow().name.clone());
            index = p.borrow().children().len() - 1;
        }

        if let Some(obj) = snapshot {
            self.history.record(Op::Add(Tombstone::new(
                guid,
                obj,
                collection.to_string(),
                obj_index,
                None,
                parent_guid,
                index,
                None,
                attribute,
                Vec::new(),
            )));
        }

        node
    }

    /// Which Objects vector holds a guid, and where; ("", -1) when none does.
    fn _locate(&self, guid: &str) -> (String, i64) {
        macro_rules! find {
            ($vec:expr, $name:expr) => {
                for i in 0..$vec.len() {
                    if $vec[i].guid() == guid {
                        return ($name.to_string(), i as i64);
                    }
                }
            };
        }

        find!(self.objects.points, "points");
        find!(self.objects.lines, "lines");
        find!(self.objects.planes, "planes");
        find!(self.objects.bboxes, "bboxes");
        find!(self.objects.polylines, "polylines");
        find!(self.objects.pointclouds, "pointclouds");
        find!(self.objects.meshes, "meshes");
        find!(self.objects.nurbscurves, "nurbscurves");
        find!(self.objects.nurbssurfaces, "nurbssurfaces");
        find!(self.objects.breps, "breps");
        find!(self.objects.elements, "elements");
        find!(self.objects.components, "components");

        (String::new(), -1)
    }

    /// Takes an object out of every live table, unrecorded, returning its tombstone.
    pub(crate) fn _detach(&mut self, guid: &str) -> Option<Tombstone> {
        let obj = clone(self.lookup.get(guid)?);
        let (collection, obj_index) = self._locate(guid);

        if obj_index >= 0 {
            macro_rules! pop {
                ($vec:expr, $variant:ident) => {{
                    $vec.remove(obj_index as usize);
                }};
            }

            typed!(collection.as_str(), self.objects, pop);
        }

        self.lookup.remove(guid);
        self.component_lookup.remove(guid);
        let xform = self.xforms.remove(guid);
        self.bvh_cache_dirty = true;
        let mut parent_guid: Option<String> = None;
        let mut index: usize = 0;
        let mut node = self.tree.get_node_by_name(guid);

        if let Some(found) = node.take() {
            if let Some(parent) = found.borrow().parent() {
                parent_guid = Some(parent.borrow().name.clone());
                let children = parent.borrow().children();
                index = children
                    .iter()
                    .position(|child| Rc::ptr_eq(child, &found))
                    .unwrap_or(0);
            }

            node = self.tree.remove(&found);
        }

        let mut attribute = String::new();
        let mut edges: Vec<(String, String, bool)> = Vec::new();

        if self.graph.has_node(guid) {
            attribute = self.graph.node_attribute(guid, None).unwrap_or_default();
            edges = self.graph.edges_of(guid);
            self.graph.remove_node(guid);
        }

        Some(Tombstone::new(
            guid.to_string(),
            obj,
            collection,
            obj_index,
            xform,
            parent_guid,
            index,
            node,
            attribute,
            edges,
        ))
    }

    /// Puts an object back from its tombstone, unrecorded: typed vector, lookup, xform, tree node with its subtree, graph node and edges.
    pub(crate) fn _attach(&mut self, op: &Tombstone) {
        let obj = clone(&op.obj);
        let obj_index = op.obj_index.max(0) as usize;
        macro_rules! insert {
            ($vec:expr, $variant:ident) => {
                if let Geometry::$variant(g) = &obj {
                    $vec.insert(obj_index.min($vec.len()), Rc::clone(g));
                }
            };
        }

        typed!(op.collection.as_str(), self.objects, insert);
        self.lookup.insert(op.guid.clone(), obj);

        if let Some(xform) = &op.xform {
            self.xforms.insert(op.guid.clone(), xform.clone());
        }

        self.bvh_cache_dirty = true;
        let node = match &op.node {
            Some(node) => Rc::clone(node),
            None => TreeNode::new(&op.guid),
        };

        if let Some(parent_guid) = &op.parent_guid {
            if let Some(parent) = self.tree.get_node_by_name(parent_guid) {
                self.tree.add(&node, Some(&parent));
                let children = parent.borrow().children();

                for child in &children[op.index.min(children.len() - 1)..children.len() - 1] {
                    parent.borrow_mut().remove(child);
                    parent.borrow_mut().add(child);
                }
            }
        }

        self.graph.add_node(&op.guid, &op.attribute);

        for (other, attribute, forward) in &op.edges {
            if !self.graph.has_node(other) {
                continue;
            }

            if *forward {
                self.graph.add_edge(&op.guid, other, attribute);
            } else {
                self.graph.add_edge(other, &op.guid, attribute);
            }
        }
    }

    /// Stores obj under guid in its typed vector and lookup, unrecorded.
    pub(crate) fn _swap(&mut self, guid: &str, obj: Geometry) {
        let (collection, obj_index) = self._locate(guid);

        if obj_index < 0 {
            return;
        }

        let obj_index = obj_index as usize;
        macro_rules! store {
            ($vec:expr, $variant:ident) => {
                if let Geometry::$variant(g) = &obj {
                    $vec[obj_index] = Rc::clone(g);
                }
            };
        }

        typed!(collection.as_str(), self.objects, store);
        let mut attribute = String::new();

        for (name, prefix) in COLLECTIONS {
            if name == collection {
                attribute = format!("{prefix}_{}", obj.name());
            }
        }

        self.lookup.insert(guid.to_string(), obj);
        self.bvh_cache_dirty = true;

        if self.graph.has_node(guid) {
            self.graph.node_attribute(guid, Some(&attribute));
        }
    }

    /// Points lookup and component_lookup at the objects this session currently holds.
    fn _index_objects(&mut self) {
        self.lookup.clear();
        self.component_lookup.clear();

        for point in &self.objects.points {
            self.lookup
                .insert(point.guid().to_string(), Geometry::Point(Rc::clone(point)));
        }

        for line in &self.objects.lines {
            self.lookup
                .insert(line.guid().to_string(), Geometry::Line(Rc::clone(line)));
        }

        for plane in &self.objects.planes {
            self.lookup
                .insert(plane.guid().to_string(), Geometry::Plane(Rc::clone(plane)));
        }

        for bbox in &self.objects.bboxes {
            self.lookup
                .insert(bbox.guid().to_string(), Geometry::OBB(Rc::clone(bbox)));
        }

        for polyline in &self.objects.polylines {
            self.lookup.insert(
                polyline.guid().to_string(),
                Geometry::Polyline(Rc::clone(polyline)),
            );
        }

        for pointcloud in &self.objects.pointclouds {
            self.lookup.insert(
                pointcloud.guid().to_string(),
                Geometry::PointCloud(Rc::clone(pointcloud)),
            );
        }

        for mesh in &self.objects.meshes {
            self.lookup
                .insert(mesh.guid().to_string(), Geometry::Mesh(Rc::clone(mesh)));
        }

        for nurbscurve in &self.objects.nurbscurves {
            self.lookup.insert(
                nurbscurve.guid().to_string(),
                Geometry::NurbsCurve(Rc::clone(nurbscurve)),
            );
        }

        for nurbssurface in &self.objects.nurbssurfaces {
            self.lookup.insert(
                nurbssurface.guid().to_string(),
                Geometry::NurbsSurface(Rc::clone(nurbssurface)),
            );
        }

        for brep in &self.objects.breps {
            self.lookup
                .insert(brep.guid().to_string(), Geometry::BRep(Rc::clone(brep)));
        }

        for element in &self.objects.elements {
            self.lookup.insert(
                element.guid().to_string(),
                Geometry::Element(Rc::clone(element)),
            );
        }

        for component in &self.objects.components {
            self.component_lookup
                .insert(component.guid().to_string(), component.clone());
        }
    }

    /// Sets or drops (None) the local transform under guid, unrecorded.
    pub(crate) fn _place(&mut self, guid: &str, xform: Option<&Xform>) {
        match xform {
            Some(xform) => {
                self.xforms.insert(guid.to_string(), xform.clone());
            }

            None => {
                self.xforms.remove(guid);
            }
        }

        self.bvh_cache_dirty = true;
    }

    /// The xforms in canonical order() sequence, identity entries omitted, the exact sequence jsondump and pb_dumps write.
    fn _xforms_ordered(&self) -> Vec<(String, Xform)> {
        let mut ordered: Vec<(String, Xform)> = Vec::new();
        let mut rest: BTreeMap<String, Xform> = BTreeMap::new();

        for (obj_guid, obj_xform) in &self.xforms {
            if !obj_xform.is_identity() {
                rest.insert(obj_guid.clone(), obj_xform.clone());
            }
        }

        for obj_guid in self.order() {
            let Some(obj_xform) = rest.remove(&obj_guid) else {
                continue;
            };
            ordered.push((obj_guid, obj_xform));
        }

        for (obj_guid, obj_xform) in rest {
            ordered.push((obj_guid, obj_xform));
        }

        ordered
    }

    /// World bounding box of every object in order() sequence, with the guid of each.
    fn _compute_boxes(&self, guids: &mut Vec<String>) -> Vec<OBB> {
        guids.clear();
        let mut boxes: Vec<OBB> = Vec::with_capacity(self.lookup.len());
        let world = self.world_xforms();

        for guid in self.order() {
            let Some(geometry) = self.lookup.get(&guid) else {
                continue;
            };
            let placement = world.get(&guid).cloned().unwrap_or_else(Xform::identity);
            boxes.push(Self::compute_bounding_box(geometry, &placement));
            guids.push(guid);
        }

        boxes
    }

    /// Rebuilds the cached SpatialBVH for ray casting.
    fn _rebuild_ray_bvh_cache(&mut self) {
        let mut guids = std::mem::take(&mut self.cached_guids);
        self.cached_boxes = self._compute_boxes(&mut guids);
        self.cached_guids = guids;

        if self.cached_boxes.is_empty() {
            self.cached_ray_bvh = None;
        } else {
            let world_size = SpatialBVH::compute_world_size(&self.cached_boxes);
            self.cached_ray_bvh = Some(SpatialBVH::from_boxes(&self.cached_boxes, world_size));
        }
    }

    /// Tests ray intersection with a specific geometry object, returning the world hit.
    fn _ray_intersect_geometry(
        &self,
        ray: &Line,
        geometry: &Geometry,
        tolerance: f64,
        placement: &Xform,
    ) -> Option<Point> {
        match geometry {
            Geometry::Point(point) => ray_point(ray, point, tolerance),
            Geometry::Line(line) => line_line(ray, line, tolerance),
            Geometry::Plane(plane) => line_plane(ray, plane, true),
            Geometry::Polyline(polyline) => {
                let mut closest: Option<Point> = None;
                let mut min_dist = f64::INFINITY;

                for i in 0..polyline.segment_count() {
                    let segment =
                        Line::from_points(&polyline.get_point(i)?, &polyline.get_point(i + 1)?);

                    let Some(hit) = line_line(ray, &segment, tolerance) else {
                        continue;
                    };
                    let dist = ray.start().distance(&hit, None);

                    if dist < min_dist {
                        min_dist = dist;
                        closest = Some(hit);
                    }
                }

                closest
            }

            Geometry::PointCloud(pointcloud) => {
                let mut closest: Option<Point> = None;
                let mut min_dist = f64::INFINITY;

                for point in pointcloud.get_points() {
                    let Some(hit) = ray_point(ray, &point, tolerance) else {
                        continue;
                    };
                    let dist = point.distance(&hit, None);

                    if dist < min_dist {
                        min_dist = dist;
                        closest = Some(hit);
                    }
                }

                closest
            }

            Geometry::Mesh(mesh) => {
                let inverse = placement.inverse()?;
                let local_ray = Line::from_points(
                    &inverse.transform_point(&ray.start()),
                    &inverse.transform_point(&ray.end()),
                );
                let hits = ray_mesh_bvh(&local_ray, mesh, tolerance, true)?;

                Some(placement.transform_point(hits.first()?))
            }

            Geometry::OBB(bbox) => {
                let hits = ray_box(ray, bbox, 0.0, 1.0)?;

                hits.first().cloned()
            }

            _ => None,
        }
    }
}

impl fmt::Display for Session {
    /// Writes the session string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Session(name={}, objects={}, tree={}, graph={})",
            self.name, self.objects, self.tree, self.graph
        )
    }
}
