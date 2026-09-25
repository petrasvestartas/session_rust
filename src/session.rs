use crate::collection::Collection;
use crate::history::clone;
use crate::history::clone_item;
use crate::history::DefinitionOp;
use crate::history::History;
use crate::history::Op;
use crate::history::ReplaceOp;
use crate::history::Tombstone;
use crate::history::XformOp;
use crate::interaction::Interaction;
use crate::intersection::line_line;
use crate::intersection::line_plane;
use crate::intersection::ray_box;
use crate::intersection::ray_mesh_bvh;
use crate::objects::Component;
use crate::BRep;
use crate::Element;
use crate::Graph;
use crate::InstanceRef;
use crate::Line;
use crate::Mesh;
use crate::NurbsCurve;
use crate::NurbsSurface;
use crate::Objects;
use crate::Plane;
use crate::Point;
use crate::PointCloud;
use crate::Polyline;
use crate::SpatialBVH;
use crate::Tolerance;
use crate::Tree;
use crate::TreeNode;
use crate::Vector;
use crate::Xform;
use crate::OBB;
use serde::Deserialize;
use serde::Serialize;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::collections::HashMap;
use std::fmt;
use std::fs;
use std::rc::Rc;
use std::rc::Weak;

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
    /// Return the guid of the wrapped object.
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

    /// Return the name of the wrapped object.
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

    /// Overwrite the guid.
    pub fn set_guid(&mut self, guid: &str) {
        macro_rules! reset {
            ($rc:expr) => {{
                Rc::make_mut($rc).set_guid(guid.to_string());
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

    /// Overwrite the name.
    pub(crate) fn set_name(&mut self, name: &str) {
        macro_rules! rename {
            ($rc:expr) => {{
                Rc::make_mut($rc).name = name.to_string();
            }};
        }

        match self {
            Geometry::OBB(g) => rename!(g),
            Geometry::BRep(g) => rename!(g),
            Geometry::Element(g) => rename!(g),
            Geometry::Line(g) => rename!(g),
            Geometry::Mesh(g) => rename!(g),
            Geometry::NurbsCurve(g) => rename!(g),
            Geometry::NurbsSurface(g) => rename!(g),
            Geometry::Plane(g) => rename!(g),
            Geometry::Point(g) => rename!(g),
            Geometry::PointCloud(g) => rename!(g),
            Geometry::Polyline(g) => rename!(g),
        }
    }
}

/// Anything an Objects collection holds: geometry, a Component or an InstanceRef.
#[derive(Debug, Clone)]
pub enum Item {
    Geometry(Geometry),
    Component(Component),
    InstanceRef(Rc<InstanceRef>),
}

impl Item {
    /// Return the guid of the wrapped object.
    pub fn guid(&self) -> &str {
        match self {
            Item::Geometry(g) => g.guid(),
            Item::Component(c) => c.guid(),
            Item::InstanceRef(i) => i.guid(),
        }
    }

    /// Return the name of the wrapped object.
    pub fn name(&self) -> &str {
        match self {
            Item::Geometry(g) => g.name(),
            Item::Component(c) => &c.name,
            Item::InstanceRef(i) => &i.name,
        }
    }
}

impl From<Geometry> for Item {
    /// Wrap geometry.
    fn from(geometry: Geometry) -> Self {
        Item::Geometry(geometry)
    }
}

/// Extract a concrete geometry type out of a `Geometry` variant, what C++ gets from `std::get_if`.
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

/// The Objects vectors, those `order()` walks first and in its sequence, each with the prefix of its graph node attribute.
pub const COLLECTIONS: [(&str, &str); 13] = [
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
    ("instances", "instance"),
];

/// Run `$op!(vec, Variant)` on the typed vector of that name, the Rust spelling of getattr(objects, collection).
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
    macro_rules! deep {
        ($field:ident) => {
            out.$field = objects
                .$field
                .iter()
                .map(|item| Rc::new((**item).clone()))
                .collect();
        };
    }

    deep!(points);
    deep!(lines);
    deep!(planes);
    deep!(bboxes);
    deep!(polylines);
    deep!(pointclouds);
    deep!(meshes);
    deep!(nurbscurves);
    deep!(nurbssurfaces);
    deep!(breps);
    deep!(elements);
    deep!(instances);

    out
}

/// The vectors of objects re-pointed at lookup: `Rc::make_mut` on a lookup entry splits it from its vector, and the lookup is the mutable truth.
fn synced(objects: &Objects, lookup: &HashMap<String, Geometry>) -> Objects {
    let mut objects = objects.clone();
    repoint(&mut objects, lookup);

    objects
}

/// Point every live slot whose guid lookup holds with another value at the lookup value.
fn repoint(objects: &mut Objects, lookup: &HashMap<String, Geometry>) {
    for (guid, geometry) in lookup {
        let (collection, _) = collection_of(geometry);
        macro_rules! sync {
            ($vec:expr, $variant:ident) => {
                if let (Some(slot), Geometry::$variant(g)) = ($vec.get_slot(guid), geometry) {
                    if !Rc::ptr_eq($vec.get_item(slot), g) {
                        $vec.set_item(slot, Rc::clone(g));
                    }
                }
            };
        }

        typed!(collection, objects, sync);
    }
}

/// Index every live slot lookup lacks, then push every geometry only lookup holds, in guid order.
fn adopt(objects: &mut Objects, lookup: &mut HashMap<String, Geometry>) {
    macro_rules! index {
        ($vec:expr, $variant:ident) => {
            for item in &$vec {
                if !lookup.contains_key(item.guid()) {
                    lookup.insert(item.guid().to_string(), Geometry::$variant(Rc::clone(item)));
                }
            }
        };
    }

    index!(objects.points, Point);
    index!(objects.lines, Line);
    index!(objects.planes, Plane);
    index!(objects.bboxes, OBB);
    index!(objects.polylines, Polyline);
    index!(objects.pointclouds, PointCloud);
    index!(objects.meshes, Mesh);
    index!(objects.nurbscurves, NurbsCurve);
    index!(objects.nurbssurfaces, NurbsSurface);
    index!(objects.breps, BRep);
    index!(objects.elements, Element);
    let mut orphans: Vec<&Geometry> = Vec::new();

    for geometry in lookup.values() {
        let (collection, _) = collection_of(geometry);
        let mut held = false;
        macro_rules! find {
            ($vec:expr, $variant:ident) => {
                held = $vec.get_slot(geometry.guid()).is_some()
            };
        }

        typed!(collection, objects, find);

        if !held {
            orphans.push(geometry);
        }
    }

    orphans.sort_by(|a, b| a.guid().cmp(b.guid()));

    for geometry in orphans {
        let (collection, _) = collection_of(geometry);
        insert_at(
            objects,
            collection,
            usize::MAX,
            &Item::Geometry(geometry.clone()),
        );
    }
}

/// Which vector of objects holds a guid, and where; ("", -1) when none does.
fn locate(objects: &Objects, guid: &str) -> (String, i64) {
    macro_rules! find {
        ($vec:expr, $name:expr) => {
            if let Some(slot) = $vec.get_slot(guid) {
                return ($name.to_string(), slot as i64);
            }
        };
    }

    find!(objects.points, "points");
    find!(objects.lines, "lines");
    find!(objects.planes, "planes");
    find!(objects.bboxes, "bboxes");
    find!(objects.polylines, "polylines");
    find!(objects.pointclouds, "pointclouds");
    find!(objects.meshes, "meshes");
    find!(objects.nurbscurves, "nurbscurves");
    find!(objects.nurbssurfaces, "nurbssurfaces");
    find!(objects.breps, "breps");
    find!(objects.elements, "elements");
    find!(objects.components, "components");
    find!(objects.instances, "instances");

    (String::new(), -1)
}

/// The COLLECTIONS entry whose vector holds the type of geometry.
fn collection_of(geometry: &Geometry) -> (&'static str, &'static str) {
    match geometry {
        Geometry::Point(_) => ("points", "point"),
        Geometry::Line(_) => ("lines", "line"),
        Geometry::Plane(_) => ("planes", "plane"),
        Geometry::OBB(_) => ("bboxes", "bbox"),
        Geometry::Polyline(_) => ("polylines", "polyline"),
        Geometry::PointCloud(_) => ("pointclouds", "pointcloud"),
        Geometry::Mesh(_) => ("meshes", "mesh"),
        Geometry::NurbsCurve(_) => ("nurbscurves", "nurbscurve"),
        Geometry::NurbsSurface(_) => ("nurbssurfaces", "nurbssurface"),
        Geometry::BRep(_) => ("breps", "brep"),
        Geometry::Element(_) => ("elements", "element"),
    }
}

/// Put an object into the vector of that name at index, clamped to its end, and return where it went.
fn insert_at(objects: &mut Objects, collection: &str, index: usize, obj: &Item) -> usize {
    let mut at = 0;
    macro_rules! insert {
        ($vec:expr, $variant:ident) => {
            if let Item::Geometry(Geometry::$variant(g)) = obj {
                at = put(&mut $vec, index, Rc::clone(g));
            }
        };
    }

    typed!(collection, objects, insert);

    if let Item::Component(component) = obj {
        at = put(&mut objects.components, index, component.clone());
    }

    if let Item::InstanceRef(instance) = obj {
        at = put(&mut objects.instances, index, Rc::clone(instance));
    }

    at
}

/// Push item, or rebuild the list with it at index while index is inside; returns where it went.
fn put<E: crate::collection::Keyed + Clone>(
    list: &mut Collection<E>,
    index: usize,
    item: E,
) -> usize {
    if index >= list.len() {
        list.push(item);

        return list.len() - 1;
    }

    let mut items = list.to_vec();
    items.insert(index, item);
    *list = Collection::from(items);

    index
}

/// Take the object at index out of the vector of that name.
fn remove_at(objects: &mut Objects, collection: &str, index: usize) {
    macro_rules! remove {
        ($vec:expr, $variant:ident) => {{
            take(&mut $vec, index)
        }};
    }

    typed!(collection, objects, remove);

    if collection == "components" {
        take(&mut objects.components, index);
    }

    if collection == "instances" {
        take(&mut objects.instances, index);
    }
}

/// Rebuild the list without the entry at index.
fn take<E: crate::collection::Keyed + Clone>(list: &mut Collection<E>, index: usize) {
    let mut items = list.to_vec();
    items.remove(index);
    *list = Collection::from(items);
}

/// Put an object in place of the one at index of the vector of that name.
fn store_at(objects: &mut Objects, collection: &str, index: usize, obj: &Item) {
    macro_rules! store {
        ($vec:expr, $variant:ident) => {
            if let Item::Geometry(Geometry::$variant(g)) = obj {
                $vec.set_item(index, Rc::clone(g));
            }
        };
    }

    typed!(collection, objects, store);

    if let Item::Component(component) = obj {
        objects.components.set_item(index, component.clone());
    }

    if let Item::InstanceRef(instance) = obj {
        objects.instances.set_item(index, Rc::clone(instance));
    }
}

/// Move geometry in place: an element is placed, anything else transformed; identity leaves it untouched.
fn place(geometry: &mut Geometry, xform: &Xform) {
    if xform.is_identity() {
        return;
    }

    macro_rules! transform {
        ($rc:expr) => {{
            Rc::make_mut($rc).transform(xform);
        }};
    }

    match geometry {
        Geometry::Element(g) => Rc::make_mut(g).place(xform),
        Geometry::OBB(g) => transform!(g),
        Geometry::BRep(g) => transform!(g),
        Geometry::Line(g) => transform!(g),
        Geometry::Mesh(g) => transform!(g),
        Geometry::NurbsCurve(g) => transform!(g),
        Geometry::NurbsSurface(g) => transform!(g),
        Geometry::Plane(g) => transform!(g),
        Geometry::Point(g) => transform!(g),
        Geometry::PointCloud(g) => transform!(g),
        Geometry::Polyline(g) => transform!(g),
    }
}

/// The definition copied as the instance, its guid and name and on an element its features, then moved by xform.
fn resolve(instance: &InstanceRef, definition: &Geometry, xform: &Xform) -> Geometry {
    let mut copy = clone(definition);
    copy.set_guid(instance.guid());
    copy.set_name(&instance.name);

    if let Geometry::Element(element) = &mut copy {
        for feature in &instance.features {
            Rc::make_mut(element).add_feature(feature.clone());
        }
    }

    place(&mut copy, xform);

    copy
}

/// The guid of the edge between a and b, from whichever stored copy has one; "" when neither was minted.
fn edge_guid(graph: &Graph, a: &str, b: &str) -> String {
    let forward = &graph.edges[a][b];
    let backward = &graph.edges[b][a];

    if forward.has_guid() {
        return forward.guid().to_string();
    }

    if backward.has_guid() {
        backward.guid().to_string()
    } else {
        String::new()
    }
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

/// The segment hit closest to the ray start.
fn ray_polyline(ray: &Line, polyline: &Polyline, tolerance: f64) -> Option<Point> {
    let mut closest: Option<Point> = None;
    let mut min_dist = f64::INFINITY;

    for i in 0..polyline.segment_count() {
        let segment = Line::from_points(&polyline.get_point(i)?, &polyline.get_point(i + 1)?);
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

/// The ray point closest to a cloud point within tolerance.
fn ray_pointcloud(ray: &Line, pointcloud: &PointCloud, tolerance: f64) -> Option<Point> {
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

/// The first hit of the ray on the placed mesh, tested in the mesh frame.
fn ray_mesh(ray: &Line, mesh: &Mesh, tolerance: f64, placement: &Xform) -> Option<Point> {
    let inverse = placement.inverse()?;
    let local_ray = Line::from_points(
        &inverse.transform_point(&ray.start()),
        &inverse.transform_point(&ray.end()),
    );
    let hits = ray_mesh_bvh(&local_ray, mesh, tolerance, true)?;

    Some(placement.transform_point(hits.first()?))
}

/// The points whose box bounds a geometry: vertices, control points or surface samples.
fn box_points(geometry: &Geometry) -> Vec<Point> {
    let mut points: Vec<Point> = Vec::new();

    match geometry {
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

        _ => {}
    }

    points
}

/// Whether guid is a graph node held by an object, instance or component.
fn registered(session: &Session, guid: &str) -> bool {
    let held = session.lookup.contains_key(guid)
        || session.instance_lookup.contains_key(guid)
        || session.component_lookup.contains_key(guid);

    session.graph.has_node(guid) && held
}

/// One object a ray touched: which one, where, and how far from the ray origin.
#[derive(Debug, Clone)]
pub struct RayHit {
    pub guid: String,     // GUID of the hit object.
    pub hit_point: Point, // Intersection point in world coordinates.
    pub distance: f64,    // Distance from the ray origin.
}

/// A session containing geometry objects.
#[derive(Serialize, Deserialize)]
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
    #[serde(default)]
    pub definitions: Objects, // Shared geometry instances place, each in its own frame; never in order(), the tree, the graph or xforms.
    #[serde(skip)]
    pub definition_lookup: HashMap<String, Geometry>, // Definitions by guid.
    #[serde(skip)]
    pub instance_lookup: HashMap<String, Rc<InstanceRef>>, // Instances by guid.
    #[serde(skip)]
    pub interactions: BTreeMap<String, Vec<Box<dyn Interaction>>>, // Interactions per graph edge, by the edge's guid; a boxed implementor keeps its type.
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
    #[serde(skip)]
    pub node_lookup: HashMap<String, Rc<RefCell<TreeNode>>>, // Tree node per live object guid.
    #[serde(skip)]
    indexed: Option<Weak<RefCell<TreeNode>>>, // Tree root at the last reindex, stale after a wholesale tree swap.
    #[serde(skip)]
    pub revision: u64, // Bumped by every Session mutation.
}

impl Default for Session {
    /// Construct a session named "my_session".
    fn default() -> Self {
        Self::new("my_session")
    }
}

impl Clone for Session {
    /// Copy every table and object, guids included; caches are rebuilt on demand and history starts empty.
    fn clone(&self) -> Self {
        let mut session = Session::new(&self.name);

        if self.has_guid() {
            session.set_guid(self.guid().to_string());
        }

        session.objects = clone_objects(&self.objects);
        session.definitions = clone_objects(&self.definitions);
        session.tree = self.tree.clone();
        session.graph = self.graph.clone();
        session.xforms = self.xforms.clone();
        session.interactions = self.interactions.clone();
        session.reindex();

        session
    }
}

impl Session {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty session whose tree root carries the session name.
    pub fn new(name: &str) -> Self {
        let mut tree = Tree::new(&format!("{name}_tree"));
        tree.add(&TreeNode::new(name), None);
        let indexed = tree.root().map(|root| Rc::downgrade(&root));

        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            objects: Objects::new(),
            lookup: HashMap::new(),
            tree,
            graph: Graph::new(&format!("{name}_graph")),
            component_lookup: HashMap::new(),
            xforms: HashMap::new(),
            definitions: Objects::new(),
            definition_lookup: HashMap::new(),
            instance_lookup: HashMap::new(),
            interactions: BTreeMap::new(),
            history: History::new(),
            bvh: SpatialBVH::new(),
            cached_ray_bvh: None,
            cached_guids: Vec::new(),
            cached_boxes: Vec::new(),
            bvh_cache_dirty: true,
            node_lookup: HashMap::new(),
            indexed,
            revision: 0,
        }
    }

    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = std::sync::OnceLock::from(guid);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Get a geometry object by GUID with type safety.
    pub fn get_object(&self, guid: &str) -> Option<&Geometry> {
        self.lookup.get(guid)
    }

    /// The tree node of a live object in O(1) through node_lookup; a tree search when the index is stale, None for a guid that is no live object.
    pub fn get_node(&self, guid: &str) -> Option<Rc<RefCell<TreeNode>>> {
        let live = self.lookup.contains_key(guid)
            || self.component_lookup.contains_key(guid)
            || self.instance_lookup.contains_key(guid);

        if !live {
            return None;
        }

        let root = self.tree.root();
        let fresh = match (&self.indexed, &root) {
            (Some(indexed), Some(root)) => std::ptr::eq(indexed.as_ptr(), Rc::as_ptr(root)),
            _ => false,
        };

        if let (true, Some(node)) = (fresh, self.node_lookup.get(guid)) {
            let owned = node.borrow().name == guid && node.borrow().parent().is_some();

            if owned {
                return Some(Rc::clone(node));
            }
        }

        self.tree.get_node_by_name(guid)
    }

    /// Select objects of one type, grouped by the top-level nodes of the tree.
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

    /// Find an existing group by name; panics when there is none.
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

    /// Canonical object order: the objects vectors walked in one fixed type sequence; instances are not in it.
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
        let Some(node) = self.get_node(guid) else {
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

    /// Get the children of a parent GUID.
    pub fn get_children(&self, obj_guid: &str) -> Vec<String> {
        self.tree.get_children_guids(obj_guid)
    }

    /// Get the neighbours of a GUID.
    pub fn get_neighbours(&self, obj_guid: &str) -> Vec<String> {
        self.graph.neighbors(obj_guid)
    }

    /// All geometry with its hierarchical placement BAKED into the coordinates; each instance becomes its definition placed, in the definition's vector.
    pub fn get_geometry(&self) -> Objects {
        let objects = self.objects_synced();
        let mut out = objects.clone();
        let world = self.world_xforms();
        macro_rules! bake {
            ($field:ident, $method:ident) => {
                out.$field = objects
                    .$field
                    .iter()
                    .map(|item| {
                        let mut copy = (**item).clone();

                        if let Some(xform) = world.get(item.guid()) {
                            if !xform.is_identity() {
                                copy.$method(xform);
                            }
                        }

                        Rc::new(copy)
                    })
                    .collect();
            };
        }

        bake!(points, transform);
        bake!(lines, transform);
        bake!(planes, transform);
        bake!(bboxes, transform);
        bake!(polylines, transform);
        bake!(pointclouds, transform);
        bake!(meshes, transform);
        bake!(nurbscurves, transform);
        bake!(nurbssurfaces, transform);
        bake!(breps, transform);
        bake!(elements, place);

        for instance in &objects.instances {
            let Some(definition) = self.definition_lookup.get(&instance.definition_guid) else {
                continue;
            };
            let placement = world
                .get(instance.guid())
                .cloned()
                .unwrap_or_else(Xform::identity);
            let resolved = resolve(instance, definition, &placement);
            let (collection, _) = collection_of(&resolved);
            insert_at(&mut out, collection, usize::MAX, &Item::Geometry(resolved));
        }

        out.instances.clear();

        out
    }

    /// The definition an instance places; None when guid is no instance or its definition is missing.
    pub fn definition_of(&self, instance_guid: &str) -> Option<Geometry> {
        let instance = self.instance_lookup.get(instance_guid)?;

        self.definition_lookup
            .get(&instance.definition_guid)
            .cloned()
    }

    /// Guids of every instance of a definition, in objects.instances order.
    pub fn instances_of(&self, definition_guid: &str) -> Vec<String> {
        let mut guids = Vec::new();

        for instance in &self.objects.instances {
            if instance.definition_guid == definition_guid {
                guids.push(instance.guid().to_string());
            }
        }

        guids
    }

    /// One object in world placement, as a copy: an instance becomes its definition moved by the world transform, carrying the instance's guid, name and features.
    pub fn world_geometry(&self, guid: &str) -> Option<Geometry> {
        let world = self.world_xform(guid);

        if let Some(geometry) = self.lookup.get(guid) {
            let mut copy = clone(geometry);
            place(&mut copy, &world);

            return Some(copy);
        }

        let definition = self.definition_of(guid)?;

        Some(resolve(&self.instance_lookup[guid], &definition, &world))
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry management
    // ═══════════════════════════════════════════════════════════════════════════
    /// Add a point.
    pub fn add_point(
        &mut self,
        point: Point,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("points", Geometry::Point(Rc::new(point)), "point", parent)
    }

    /// Add a line.
    pub fn add_line(
        &mut self,
        line: Line,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("lines", Geometry::Line(Rc::new(line)), "line", parent)
    }

    /// Add a plane.
    pub fn add_plane(
        &mut self,
        plane: Plane,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("planes", Geometry::Plane(Rc::new(plane)), "plane", parent)
    }

    /// Add a bounding box.
    pub fn add_obb(&mut self, bbox: OBB) -> Rc<RefCell<TreeNode>> {
        self._add_object("bboxes", Geometry::OBB(Rc::new(bbox)), "bbox", None)
    }

    /// Add a polyline; fewer than two points adds nothing and returns None.
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

    /// Add a point cloud; no points adds nothing and returns None.
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

    /// Add a mesh; no faces adds nothing and returns None.
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

    /// Add a curve; fewer than two control vertices adds nothing and returns None.
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

    /// Add a surface; no control vertices adds nothing and returns None.
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

    /// Add a brep; no faces and no vertices adds nothing and returns None.
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

    /// Add an element; an Element is a data record kept even without geometry.
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

    /// Add a custom component (any object with type_name/guid/name/extra).
    pub fn add_component(
        &mut self,
        component: Component,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object(
            "components",
            Item::Component(component),
            "component",
            parent,
        )
    }

    /// Add a definition, geometry in its own frame that instances share; returns its guid, also when that guid is already defined, and "" for a guid an object, instance or component holds.
    pub fn add_definition(&mut self, definition: Geometry) -> String {
        let guid = definition.guid().to_string();

        if self.definition_lookup.contains_key(&guid) {
            return guid;
        }

        if self.lookup.contains_key(&guid)
            || self.instance_lookup.contains_key(&guid)
            || self.component_lookup.contains_key(&guid)
        {
            return String::new();
        }

        if self.history.current.is_some() {
            self.history.record(Op::Definition(DefinitionOp::new(
                guid.clone(),
                None,
                Some(clone(&definition)),
            )));
        }

        self._define(&guid, Some(definition));

        guid
    }

    /// Add an instance under parent, placed by xform relative to the parent with its own xform folded in; None when its definition_guid names no definition.
    pub fn add_instance(
        &mut self,
        mut instance: InstanceRef,
        xform: Xform,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if !self
            .definition_lookup
            .contains_key(&instance.definition_guid)
        {
            return None;
        }

        let placement = &xform * &instance.xform;
        instance.xform = Xform::identity();
        let guid = instance.guid().to_string();
        let node = self._add_object(
            "instances",
            Item::InstanceRef(Rc::new(instance)),
            "instance",
            parent,
        );

        if !placement.is_identity() {
            self.set_xform(&guid, placement);
        }

        Some(node)
    }

    /// Add a TreeNode to the tree hierarchy, under the root when no parent is given.
    pub fn add<'a>(
        &mut self,
        node: &Rc<RefCell<TreeNode>>,
        parent: impl Into<Option<&'a Rc<RefCell<TreeNode>>>>,
    ) where
        Rc<RefCell<TreeNode>>: 'a,
    {
        let parent = parent.into();
        let guid = node.borrow().name.clone();
        self.revision += 1;

        if self.get_object(&guid).is_some()
            || self.component_lookup.contains_key(&guid)
            || self.instance_lookup.contains_key(&guid)
        {
            self.node_lookup.insert(guid, Rc::clone(node));
        }

        if parent.is_none() {
            if let Some(root) = self.tree.root() {
                self.tree.add(node, Some(&root));
            }
        } else {
            self.tree.add(node, parent);
        }
    }

    /// Create a named group (TreeNode) and add it to the root of the tree.
    pub fn add_group(&mut self, group_name: &str) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(group_name);
        self.add(&node, None);

        node
    }

    /// Add an edge between two geometry objects in the graph.
    pub fn add_edge(&mut self, guid1: &str, guid2: &str, attribute: &str) {
        self.revision += 1;
        self.graph.add_edge(guid1, guid2, attribute);
    }

    /// Add a parent-child relationship in the tree.
    pub fn add_hierarchy(&mut self, parent_guid: &str, child_guid: &str) -> bool {
        self.revision += 1;
        self.tree.add_child_by_guid(parent_guid, child_guid)
    }

    /// Add a relationship edge in the graph.
    pub fn add_relationship(&mut self, from_guid: &str, to_guid: &str, relationship_type: &str) {
        self.revision += 1;
        self.graph.add_edge(from_guid, to_guid, relationship_type);
    }

    /// Remove an object by its GUID from every live table at once; the removal record is the tombstone undo restores from.
    pub fn remove_object(&mut self, obj_guid: &str) -> bool {
        let Some(op) = self._detach(obj_guid) else {
            return false;
        };
        self.history.record(Op::Remove(op));

        true
    }

    /// Swap the object stored under guid for obj, which takes over that guid; the recorded edit undo and redo restore as absolute snapshots.
    pub fn replace(&mut self, guid: &str, obj: Geometry) -> bool {
        let Some(before) = self.lookup.get(guid) else {
            return false;
        };
        let mut obj = obj;
        obj.set_guid(guid);

        if self.history.current.is_some() {
            self.history.record(Op::Replace(ReplaceOp::new(
                guid.to_string(),
                Item::Geometry(clone(before)),
                Item::Geometry(clone(&obj)),
            )));
        }

        self._swap(guid, Item::Geometry(obj));

        true
    }

    /// Swap the geometry of a definition, which keeps its guid, so every instance of it changes at once; false when guid is no definition.
    pub fn replace_definition(&mut self, guid: &str, definition: Geometry) -> bool {
        let Some(before) = self.definition_lookup.get(guid) else {
            return false;
        };
        let mut definition = definition;
        definition.set_guid(guid);

        if self.history.current.is_some() {
            self.history.record(Op::Definition(DefinitionOp::new(
                guid.to_string(),
                Some(clone(before)),
                Some(clone(&definition)),
            )));
        }

        self._define(guid, Some(definition));

        true
    }

    /// Remove a definition; false when guid is no definition or an instance still names it.
    pub fn remove_definition(&mut self, guid: &str) -> bool {
        let Some(before) = self.definition_lookup.get(guid) else {
            return false;
        };

        if !self.instances_of(guid).is_empty() {
            return false;
        }

        if self.history.current.is_some() {
            self.history.record(Op::Definition(DefinitionOp::new(
                guid.to_string(),
                Some(clone(before)),
                None,
            )));
        }

        self._define(guid, None);

        true
    }

    /// Turn an object into an instance of a definition, keeping its guid, name, tree node and edges; frame maps the definition onto the object and is folded into its local transform.
    pub fn to_instance(&mut self, guid: &str, definition_guid: &str, frame: Xform) -> bool {
        let Some(object) = self.lookup.get(guid) else {
            return false;
        };

        if !self.definition_lookup.contains_key(definition_guid) {
            return false;
        }

        let mut instance = InstanceRef::new(definition_guid, Xform::identity());
        instance.set_guid(guid.to_string());
        instance.name = object.name().to_string();
        let attribute = format!("instance_{}", instance.name);
        let placement = &self.xform(guid) * &frame;
        let Some(removed) = self._detach(guid) else {
            return false;
        };
        let mut added = Tombstone::new(
            guid.to_string(),
            Item::InstanceRef(Rc::new(instance)),
            "instances".to_string(),
            self.objects.instances.len() as i64,
            (!placement.is_identity()).then_some(placement),
            removed.parent_guid.clone(),
            removed.index,
            removed.node.clone(),
            attribute,
            removed.edges.clone(),
        );
        added.interactions = removed.interactions.clone();
        self.history.record(Op::Remove(removed));
        self._attach(&added);
        self.history.record(Op::Add(added));

        true
    }

    /// Turn an instance into a standalone copy of its definition in the definition frame, keeping its guid, name, transform, tree node and edges, and on an element its features.
    pub fn explode(&mut self, instance_guid: &str) -> bool {
        let Some(definition) = self.definition_of(instance_guid) else {
            return false;
        };
        let instance = Rc::clone(&self.instance_lookup[instance_guid]);
        let copy = resolve(&instance, &definition, &Xform::identity());
        let (collection, prefix) = collection_of(&copy);
        let mut size: i64 = 0;
        macro_rules! count {
            ($vec:expr, $variant:ident) => {
                size = $vec.len() as i64
            };
        }

        typed!(collection, self.objects, count);
        let Some(removed) = self._detach(instance_guid) else {
            return false;
        };
        let mut added = Tombstone::new(
            instance_guid.to_string(),
            Item::Geometry(copy),
            collection.to_string(),
            size,
            removed.xform.clone(),
            removed.parent_guid.clone(),
            removed.index,
            removed.node.clone(),
            format!("{prefix}_{}", instance.name),
            removed.edges.clone(),
        );
        added.interactions = removed.interactions.clone();
        self.history.record(Op::Remove(removed));
        self._attach(&added);
        self.history.record(Op::Add(added));

        true
    }

    /// Set the LOCAL transform of an object, relative to its tree parent; a guid that names only a definition is ignored.
    pub fn set_xform(&mut self, guid: &str, xform: Xform) {
        if self.definition_lookup.contains_key(guid)
            && !self.lookup.contains_key(guid)
            && !self.instance_lookup.contains_key(guid)
        {
            return;
        }

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
        self.revision += 1;
    }

    /// Remove an object's local transform, returning whether one was present.
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
        self.revision += 1;

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Session - Interactions
    // ═══════════════════════════════════════════════════════════════════════════
    /// Make or reuse the pair's undirected edge, an existing edge keeping its attributes, and append interaction to its list; returns the stored interaction. Errs unless both elements are in the session and distinct.
    pub fn add_interaction(
        &mut self,
        a: &Element,
        b: &Element,
        interaction: Box<dyn Interaction>,
    ) -> Result<&dyn Interaction, Box<dyn std::error::Error>> {
        let first = a.guid();
        let second = b.guid();

        if first == second || !registered(self, first) || !registered(self, second) {
            return Err(
                "Session::add_interaction: add two distinct elements to the session first".into(),
            );
        }

        if !self.graph.has_edge((first, second)) {
            self.graph.add_edge(first, second, "");
        }

        self.revision += 1;

        let id = self.graph.edges[first][second].guid().to_string();
        let list = self.interactions.entry(id).or_default();
        list.push(interaction);

        Ok(list[list.len() - 1].as_ref())
    }

    /// The pair's interactions in either order, empty when there are none.
    pub fn get_interaction(&self, a: &Element, b: &Element) -> &[Box<dyn Interaction>] {
        if !self.graph.has_edge((a.guid(), b.guid())) {
            return &[];
        }

        let id = self.graph.edges[a.guid()][b.guid()].guid();

        match self.interactions.get(id) {
            Some(list) => list,
            None => &[],
        }
    }

    /// True when the pair has an edge in either order.
    pub fn has_interaction(&self, a: &Element, b: &Element) -> bool {
        self.graph.has_edge((a.guid(), b.guid()))
    }

    /// Remove the pair's edge and all of its interactions in either order; a missing pair is a no-op.
    pub fn remove_interaction(&mut self, a: &Element, b: &Element) {
        if !self.graph.has_edge((a.guid(), b.guid())) {
            return;
        }

        let id = self.graph.edges[a.guid()][b.guid()].guid().to_string();
        self.revision += 1;
        self.interactions.remove(&id);
        self.graph.remove_edge((a.guid(), b.guid()));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // History
    // ═══════════════════════════════════════════════════════════════════════════
    /// Open a history transaction: every add, remove, replace and xform change until commit becomes one undo step.
    pub fn begin(&mut self, label: &str) {
        self.history.begin(label);
    }

    /// Close the open transaction as one undo step.
    pub fn commit(&mut self) {
        self.history.commit();
    }

    /// Revert the latest committed transaction, returning whether there was one.
    pub fn undo(&mut self) -> bool {
        self.revision += 1;
        let mut history = std::mem::take(&mut self.history);
        let undone = history.undo(self);
        self.history = history;

        undone
    }

    /// Reapply the latest undone transaction, returning whether there was one.
    pub fn redo(&mut self) -> bool {
        self.revision += 1;
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

        match geometry {
            Geometry::Point(point) => OBB::from_point(&xform.transform_point(point), inflate),
            Geometry::Plane(plane) => {
                OBB::from_point(&xform.transform_point(&plane.origin()), inflate * 10.0)
            }

            Geometry::OBB(bbox) => {
                let mut inflated = (**bbox).clone();
                inflated.half_size = &inflated.half_size + &Vector::new(inflate, inflate, inflate);
                inflated.transform(xform);

                inflated
            }

            Geometry::Element(element) => {
                let mut copy = (**element).clone();
                let mut bbox = copy.aabb();
                bbox.transform(xform);

                bbox
            }

            _ => placed_box(&box_points(geometry), xform, inflate),
        }
    }

    /// Get all collision pairs using SpatialBVH and add them as graph edges.
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

    /// Cast a ray through the scene, returning the hits within tolerance of the closest one.
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
            let Some(geometry) = self
                .lookup
                .get(&guid)
                .cloned()
                .or_else(|| self.definition_of(&guid))
            else {
                continue;
            };
            let placement = world.get(&guid).cloned().unwrap_or_else(Xform::identity);
            let Some(hit) = self._ray_intersect_geometry(&ray, &geometry, tolerance, &placement)
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
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let graph_json: serde_json::Value = serde_json::from_str(&self.graph.jsondump()?)?;
        let mut xforms_json: Vec<serde_json::Value> = Vec::new();

        for (obj_guid, obj_xform) in self._xforms_ordered() {
            xforms_json.push(serde_json::json!({ "guid": obj_guid, "xform": obj_xform }));
        }

        let mut interactions_json: Vec<serde_json::Value> = Vec::new();

        for (edge, interactions) in &self.interactions {
            let mut items: Vec<serde_json::Value> = Vec::new();

            for interaction in interactions {
                items.push(interaction.jsondump());
            }

            interactions_json.push(serde_json::json!({ "guid": edge, "interactions": items }));
        }

        let mut json_obj = serde_json::json!({
            "type": "Session",
            "name": self.name,
            "guid": self.guid(),
            "objects": self.objects_synced(),
            "tree": self.tree,
            "graph": graph_json,
            "interactions": interactions_json,
            "xforms": xforms_json
        });

        if !self.definition_lookup.is_empty() {
            json_obj["definitions"] =
                serde_json::to_value(synced(&self.definitions, &self.definition_lookup))?;
        }

        let sorted = crate::file_encoders::sort_json_keys(json_obj);
        let mut buf: Vec<u8> = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        serde::Serialize::serialize(&sorted, &mut ser)?;

        Ok(String::from_utf8(buf)?)
    }

    /// Deserialize from a JSON string.
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

        if json_obj["definitions"].is_object() {
            session.definitions = serde_json::from_value(json_obj["definitions"].clone())?;
        }

        if let Some(entries) = json_obj["xforms"].as_array() {
            for entry in entries {
                let Some(guid) = entry["guid"].as_str() else {
                    continue;
                };
                let xform: Xform = serde_json::from_value(entry["xform"].clone())?;
                session.xforms.insert(guid.to_string(), xform);
            }
        }

        if let Some(entries) = json_obj["interactions"].as_array() {
            for entry in entries {
                let guid = entry["guid"].as_str().unwrap_or("").to_string();
                let Some(items) = entry["interactions"].as_array() else {
                    continue;
                };

                for item in items {
                    session
                        .interactions
                        .entry(guid.clone())
                        .or_default()
                        .push(<dyn Interaction>::jsonload(item));
                }
            }
        }

        session.reindex();

        Ok(session)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&mut self) -> String {
        self.history.clear();

        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&mut self, filename: &str) {
        self.history.clear();
        fs::write(filename, self.jsondump().unwrap_or_default())
            .expect("Failed to write JSON file");
    }

    /// Read from a JSON file.
    pub fn file_json_load(filename: &str) -> Self {
        let json = fs::read_to_string(filename).expect("Failed to read JSON file");

        Self::jsonload(&json).unwrap_or_else(|_| Self::default())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Session {
        use prost::Message;

        let mut xforms: Vec<crate::proto::XformEntry> = Vec::new();

        for (obj_guid, obj_xform) in self._xforms_ordered() {
            xforms.push(crate::proto::XformEntry {
                guid: obj_guid,
                xform: Some(obj_xform.to_proto()),
            });
        }

        let mut interactions: Vec<crate::proto::InteractionEntry> = Vec::new();

        for (edge, list) in &self.interactions {
            let mut items: Vec<crate::proto::Interaction> = Vec::new();

            for interaction in list {
                items.push(interaction.to_proto());
            }

            interactions.push(crate::proto::InteractionEntry {
                guid: edge.clone(),
                interactions: items,
            });
        }

        let definitions = if self.definition_lookup.is_empty() {
            None
        } else {
            Some(synced(&self.definitions, &self.definition_lookup).to_proto())
        };

        crate::proto::Session {
            name: self.name.clone(),
            guid: self.guid.get().cloned().unwrap_or_default(),
            objects: Some(self.objects_synced().to_proto()),
            tree: crate::proto::Tree::decode(self.tree.pb_dumps().as_slice()).ok(),
            graph: Some(self.graph.to_proto()),
            bvh_boxes: Vec::new(),
            xforms,
            definitions,
            interactions,
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Session) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let mut session = Session::new(&proto.name);

        if !proto.guid.is_empty() {
            session.set_guid(proto.guid);
        }

        if let Some(objects) = proto.objects {
            session.objects = Objects::from_proto(objects)?;
        }

        if let Some(tree) = proto.tree {
            session.tree = Tree::pb_loads(&tree.encode_to_vec())?;
        }

        if let Some(graph) = proto.graph {
            session.graph = Graph::from_proto(graph);
        }

        if let Some(definitions) = proto.definitions {
            session.definitions = Objects::from_proto(definitions)?;
        }

        for entry in proto.xforms {
            let Some(xform) = entry.xform else {
                continue;
            };
            session.xforms.insert(entry.guid, Xform::from_proto(xform));
        }

        for entry in proto.interactions {
            for item in entry.interactions {
                session
                    .interactions
                    .entry(entry.guid.clone())
                    .or_default()
                    .push(<dyn Interaction>::from_proto(item));
            }
        }

        session.reindex();

        Ok(session)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&mut self) -> Vec<u8> {
        use prost::Message;

        self.history.clear();

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Self::from_proto(crate::proto::Session::decode(data)?)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&mut self, filename: &str) {
        fs::write(filename, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Read from a protobuf file.
    pub fn pb_load(filename: &str) -> Self {
        let data = fs::read(filename).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the spatial hierarchy and the element interactions as a banner block.
    pub fn str(&self) -> String {
        let bar = "=".repeat(80);

        format!(
            "{0}\nSpatial Hierarchy\n{0}\n{1}{0}\nElement Interactions\n{0}\n{2}\n{0}\n",
            bar,
            self.tree.str(),
            self.graph.str()
        )
    }

    /// Return "Session(name=..., objects=..., tree=..., graph=...)".
    pub fn repr(&self) -> String {
        format!(
            "Session(name={}, objects={}, tree={}, graph={})",
            self.name,
            self.objects,
            self.tree.repr(),
            self.graph.repr()
        )
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════
    /// The objects vectors re-pointed at `lookup` and `instance_lookup`: `Rc::make_mut` on a lookup entry splits it from the vector, and the lookup is the mutable truth.
    fn objects_synced(&self) -> Objects {
        let mut objects = synced(&self.objects, &self.lookup);

        for (guid, instance) in &self.instance_lookup {
            if let Some(slot) = objects.instances.get_slot(guid) {
                objects.instances.set_item(slot, Rc::clone(instance));
            }
        }

        objects
    }

    /// Store an object in its typed vector, lookup, graph and tree, recording an AddOp when a transaction is open.
    fn _add_object(
        &mut self,
        collection: &str,
        obj: impl Into<Item>,
        type_prefix: &str,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        let obj: Item = obj.into();
        let guid = obj.guid().to_string();
        let attribute = format!("{type_prefix}_{}", obj.name());
        let obj_index = insert_at(&mut self.objects, collection, usize::MAX, &obj) as i64;
        let snapshot = self.history.current.is_some().then(|| clone_item(&obj));

        match obj {
            Item::Geometry(geometry) => {
                self.lookup.insert(guid.clone(), geometry);
            }

            Item::Component(component) => {
                self.component_lookup.insert(guid.clone(), component);
            }

            Item::InstanceRef(instance) => {
                self.instance_lookup.insert(guid.clone(), instance);
            }
        }

        self.graph.add_node(&guid, &attribute);
        self.bvh_cache_dirty = true;
        let node = TreeNode::new(&guid);
        self.node_lookup.insert(guid.clone(), Rc::clone(&node));
        self.revision += 1;
        let host = parent.cloned().or_else(|| self.tree.root());
        let mut parent_guid: Option<String> = None;
        let mut index: usize = 0;

        if let Some(p) = &host {
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
        locate(&self.objects, guid)
    }

    /// Take an object out of every live table, unrecorded, returning its tombstone.
    pub(crate) fn _detach(&mut self, guid: &str) -> Option<Tombstone> {
        let obj = if let Some(geometry) = self.lookup.get(guid) {
            Item::Geometry(clone(geometry))
        } else if let Some(component) = self.component_lookup.get(guid) {
            Item::Component(component.clone())
        } else {
            Item::InstanceRef(Rc::new((**self.instance_lookup.get(guid)?).clone()))
        };
        let (collection, obj_index) = self._locate(guid);

        if obj_index >= 0 {
            remove_at(&mut self.objects, &collection, obj_index as usize);
        }

        let mut node = self.get_node(guid);
        self.lookup.remove(guid);
        self.component_lookup.remove(guid);
        self.instance_lookup.remove(guid);
        self.node_lookup.remove(guid);
        let xform = self.xforms.remove(guid);
        self.bvh_cache_dirty = true;
        self.revision += 1;
        let mut parent_guid: Option<String> = None;
        let mut index: usize = 0;

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
        let mut edges: Vec<(String, String, bool, String)> = Vec::new();

        if self.graph.has_node(guid) {
            attribute = self.graph.node_label(guid, None).unwrap_or_default();

            for (other, label, forward) in self.graph.edges_of(guid) {
                let id = edge_guid(&self.graph, guid, &other);
                edges.push((other, label, forward, id));
            }

            self.graph.remove_node(guid);
        }

        let mut interactions: BTreeMap<String, Vec<Box<dyn Interaction>>> = BTreeMap::new();

        for (_, _, _, id) in &edges {
            if let Some(list) = self.interactions.remove(id) {
                interactions.insert(id.clone(), list);
            }
        }

        let mut op = Tombstone::new(
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
        );
        op.interactions = interactions;

        Some(op)
    }

    /// Put an object back from its tombstone, unrecorded: typed vector, lookup, xform, tree node with its subtree, graph node and edges.
    pub(crate) fn _attach(&mut self, op: &Tombstone) {
        let obj = clone_item(&op.obj);
        insert_at(
            &mut self.objects,
            &op.collection,
            op.obj_index.max(0) as usize,
            &obj,
        );

        match obj {
            Item::Geometry(geometry) => {
                self.lookup.insert(op.guid.clone(), geometry);
            }

            Item::Component(component) => {
                self.component_lookup.insert(op.guid.clone(), component);
            }

            Item::InstanceRef(instance) => {
                self.instance_lookup.insert(op.guid.clone(), instance);
            }
        }

        if let Some(xform) = &op.xform {
            self.xforms.insert(op.guid.clone(), xform.clone());
        }

        self.bvh_cache_dirty = true;
        let node = match &op.node {
            Some(node) => Rc::clone(node),
            None => TreeNode::new(&op.guid),
        };
        self.node_lookup.insert(op.guid.clone(), Rc::clone(&node));
        self.revision += 1;

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

        for (other, attribute, forward, id) in &op.edges {
            if !self.graph.has_node(other) {
                continue;
            }

            if *forward {
                self.graph.add_edge(&op.guid, other, attribute);
            } else {
                self.graph.add_edge(other, &op.guid, attribute);
            }

            if id.is_empty() {
                continue;
            }

            self.graph
                .edges
                .get_mut(&op.guid)
                .unwrap()
                .get_mut(other)
                .unwrap()
                .set_guid(id.clone());
            self.graph
                .edges
                .get_mut(other)
                .unwrap()
                .get_mut(&op.guid)
                .unwrap()
                .set_guid(id.clone());

            let Some(list) = op.interactions.get(id) else {
                continue;
            };

            for interaction in list {
                self.interactions
                    .entry(id.clone())
                    .or_default()
                    .push(interaction.clone_box());
            }
        }
    }

    /// Store obj under guid in its typed vector and lookup, unrecorded.
    pub(crate) fn _swap(&mut self, guid: &str, obj: Item) {
        let (collection, obj_index) = self._locate(guid);

        if obj_index < 0 {
            return;
        }

        store_at(&mut self.objects, &collection, obj_index as usize, &obj);
        self.revision += 1;
        let mut attribute = String::new();

        for (name, prefix) in COLLECTIONS {
            if name == collection {
                attribute = format!("{prefix}_{}", obj.name());
            }
        }

        match obj {
            Item::Geometry(geometry) => {
                self.lookup.insert(guid.to_string(), geometry);
            }

            Item::Component(component) => {
                self.component_lookup.insert(guid.to_string(), component);
            }

            Item::InstanceRef(instance) => {
                self.instance_lookup.insert(guid.to_string(), instance);
            }
        }

        self.bvh_cache_dirty = true;

        if self.graph.has_node(guid) {
            self.graph.node_label(guid, Some(&attribute));
        }
    }

    /// Rebuild every index from the tables in O(n + N): the maps win over the slots, map-only and slot-only entries are adopted, a non-identity instance xform folds into xforms, node_lookup is refilled from the live tree.
    pub fn reindex(&mut self) {
        repoint(&mut self.objects, &self.lookup);
        adopt(&mut self.objects, &mut self.lookup);
        repoint(&mut self.definitions, &self.definition_lookup);
        adopt(&mut self.definitions, &mut self.definition_lookup);

        for slot in 0..self.objects.components.number_of_slots() {
            if self.objects.components.is_dead(slot) {
                continue;
            }

            let guid = self.objects.components.get_item(slot).guid.clone();

            match self.component_lookup.get(&guid) {
                Some(component) => {
                    let component = component.clone();
                    self.objects.components.set_item(slot, component);
                }

                None => {
                    let component = self.objects.components.get_item(slot).clone();
                    self.component_lookup.insert(guid, component);
                }
            }
        }

        let mut components: Vec<&Component> = Vec::new();

        for (guid, component) in &self.component_lookup {
            if self.objects.components.get_slot(guid).is_none() {
                components.push(component);
            }
        }

        components.sort_by(|a, b| a.guid.cmp(&b.guid));

        for component in components {
            self.objects.components.push(component.clone());
        }

        let mut instances: Vec<&Rc<InstanceRef>> = Vec::new();

        for (guid, instance) in &self.instance_lookup {
            if self.objects.instances.get_slot(guid).is_none() {
                instances.push(instance);
            }
        }

        instances.sort_by(|a, b| a.guid().cmp(b.guid()));

        for instance in instances {
            self.objects.instances.push(Rc::clone(instance));
        }

        for slot in 0..self.objects.instances.number_of_slots() {
            if self.objects.instances.is_dead(slot) {
                continue;
            }

            let mut instance = Rc::clone(self.objects.instances.get_item(slot));
            let guid = instance.guid().to_string();

            if let Some(truth) = self.instance_lookup.get(&guid) {
                instance = Rc::clone(truth);
            }

            if !instance.xform.is_identity() {
                let folded = &self.xform(&guid) * &instance.xform;
                self.xforms.insert(guid.clone(), folded);
                Rc::make_mut(&mut instance).xform = Xform::identity();
            }

            self.objects.instances.set_item(slot, Rc::clone(&instance));
            self.instance_lookup.insert(guid, instance);
        }

        self.node_lookup.clear();

        for node in self.tree.nodes() {
            let guid = node.borrow().name.clone();
            let live = self.lookup.contains_key(&guid)
                || self.component_lookup.contains_key(&guid)
                || self.instance_lookup.contains_key(&guid);

            if live && !self.node_lookup.contains_key(&guid) {
                self.node_lookup.insert(guid, node);
            }
        }

        self.indexed = self.tree.root().map(|root| Rc::downgrade(&root));
    }

    /// Set or drops (None) a definition under guid, unrecorded.
    pub(crate) fn _define(&mut self, guid: &str, definition: Option<Geometry>) {
        let (collection, position) = locate(&self.definitions, guid);

        if position >= 0 {
            remove_at(&mut self.definitions, &collection, position as usize);
        }

        self.definition_lookup.remove(guid);
        self.bvh_cache_dirty = true;
        self.revision += 1;
        let Some(definition) = definition else {
            return;
        };
        let (collection, _) = collection_of(&definition);
        let at = usize::try_from(position).unwrap_or(usize::MAX);
        insert_at(
            &mut self.definitions,
            collection,
            at,
            &Item::Geometry(definition.clone()),
        );
        self.definition_lookup.insert(guid.to_string(), definition);
    }

    /// Set or drops (None) the local transform under guid, unrecorded.
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
        self.revision += 1;
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

    /// World bounding box of every object in order() sequence, then of every instance, with the guid of each.
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

        let mut local: HashMap<String, OBB> = HashMap::new();

        for instance in &self.objects.instances {
            let Some(definition) = self.definition_lookup.get(&instance.definition_guid) else {
                continue;
            };
            let mut placed = local
                .entry(instance.definition_guid.clone())
                .or_insert_with(|| Self::compute_bounding_box(definition, &Xform::identity()))
                .clone();

            if let Some(xform) = world.get(instance.guid()) {
                placed.transform(xform);
            }

            boxes.push(placed);
            guids.push(instance.guid().to_string());
        }

        boxes
    }

    /// Rebuild the cached SpatialBVH for ray casting.
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

    /// Test ray intersection with a specific geometry object, returning the world hit.
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
            Geometry::Polyline(polyline) => ray_polyline(ray, polyline, tolerance),
            Geometry::PointCloud(pointcloud) => ray_pointcloud(ray, pointcloud, tolerance),
            Geometry::Mesh(mesh) => ray_mesh(ray, mesh, tolerance, placement),
            Geometry::OBB(bbox) => {
                let hits = ray_box(ray, bbox, 0.0, 1.0)?;

                hits.first().cloned()
            }

            _ => None,
        }
    }
}

impl fmt::Display for Session {
    /// Write the session block to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl fmt::Debug for Session {
    /// Write the one-line session string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}
