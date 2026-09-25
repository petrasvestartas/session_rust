use crate::color::Color;
use crate::history::clone;
use crate::history::weight;
use crate::history::History;
use crate::history::Op;
use crate::history::ReplaceOp;
use crate::history::Tomb;
use crate::history::Tombstone;
use crate::history::TreeOp;
use crate::history::XformOp;
use crate::history::RECORD;
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
        push(objects, collection, &Item::Geometry(geometry.clone()));
    }
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

/// Run `$op!(list)` on the Collection of that name, components and instances included.
macro_rules! listed {
    ($collection:expr, $objects:expr, $op:ident) => {
        match $collection {
            "points" => $op!($objects.points),
            "lines" => $op!($objects.lines),
            "planes" => $op!($objects.planes),
            "bboxes" => $op!($objects.bboxes),
            "polylines" => $op!($objects.polylines),
            "pointclouds" => $op!($objects.pointclouds),
            "meshes" => $op!($objects.meshes),
            "nurbscurves" => $op!($objects.nurbscurves),
            "nurbssurfaces" => $op!($objects.nurbssurfaces),
            "breps" => $op!($objects.breps),
            "elements" => $op!($objects.elements),
            "components" => $op!($objects.components),
            "instances" => $op!($objects.instances),
            _ => {}
        }
    };
}

/// The COLLECTIONS entry whose list holds an item.
fn collection_for(item: &Item) -> (&'static str, &'static str) {
    match item {
        Item::Geometry(geometry) => collection_of(geometry),
        Item::Component(_) => ("components", "component"),
        Item::InstanceRef(_) => ("instances", "instance"),
    }
}

/// The graph attribute prefix of the list of that name.
fn prefix_of(collection: &str) -> &'static str {
    for (name, prefix) in COLLECTIONS {
        if name == collection {
            return prefix;
        }
    }

    ""
}

/// The live slot of a guid in the list of that name.
fn slot_of(objects: &Objects, collection: &str, guid: &str) -> Option<usize> {
    let mut slot = None;
    macro_rules! find {
        ($list:expr) => {
            slot = $list.get_slot(guid)
        };
    }

    listed!(collection, objects, find);

    slot
}

/// The item in a slot of the list of that name, dead or alive, as the stored pointer.
fn item_at(objects: &Objects, collection: &str, slot: usize) -> Option<Item> {
    let mut item = None;
    macro_rules! get {
        ($list:expr, $variant:ident) => {
            item = Some(Item::Geometry(Geometry::$variant(Rc::clone(
                $list.get_item(slot),
            ))))
        };
    }

    typed!(collection, objects, get);

    if collection == "components" {
        item = Some(Item::Component(objects.components.get_item(slot).clone()));
    }

    if collection == "instances" {
        item = Some(Item::InstanceRef(Rc::clone(
            objects.instances.get_item(slot),
        )));
    }

    item
}

/// Append an item to the list of that name and return its slot.
fn push(objects: &mut Objects, collection: &str, obj: &Item) -> usize {
    let mut slot = 0;
    macro_rules! append {
        ($list:expr, $variant:ident) => {
            if let Item::Geometry(Geometry::$variant(g)) = obj {
                $list.push(Rc::clone(g));
                slot = $list.number_of_slots() - 1;
            }
        };
    }

    typed!(collection, objects, append);

    if let Item::Component(component) = obj {
        objects.components.push(component.clone());
        slot = objects.components.number_of_slots() - 1;
    }

    if let Item::InstanceRef(instance) = obj {
        objects.instances.push(Rc::clone(instance));
        slot = objects.instances.number_of_slots() - 1;
    }

    slot
}

/// Put an item in a slot of the list of that name.
fn store(objects: &mut Objects, collection: &str, slot: usize, obj: &Item) {
    macro_rules! set {
        ($list:expr, $variant:ident) => {
            if let Item::Geometry(Geometry::$variant(g)) = obj {
                $list.set_item(slot, Rc::clone(g));
            }
        };
    }

    typed!(collection, objects, set);

    if let Item::Component(component) = obj {
        objects.components.set_item(slot, component.clone());
    }

    if let Item::InstanceRef(instance) = obj {
        objects.instances.set_item(slot, Rc::clone(instance));
    }
}

/// Kill or revive a slot of the list of that name.
fn flag(objects: &mut Objects, collection: &str, slot: usize, dead: bool) {
    macro_rules! set {
        ($list:expr) => {
            $list.set_dead(slot, dead)
        };
    }

    listed!(collection, objects, set);
}

/// The tomb pinning a slot of the list of that name, while a record still holds it.
fn tomb_at(objects: &Objects, collection: &str, slot: usize) -> Option<Rc<Tomb>> {
    let mut tomb = None;
    macro_rules! get {
        ($list:expr) => {
            tomb = $list.get_tomb(slot)
        };
    }

    listed!(collection, objects, get);

    tomb
}

/// Pin a slot of the list of that name to a tomb.
fn pin(objects: &mut Objects, collection: &str, slot: usize, tomb: &Rc<Tomb>) {
    macro_rules! set {
        ($list:expr) => {
            $list.set_tomb(slot, tomb)
        };
    }

    listed!(collection, objects, set);
}

/// Whether two items are the same stored pointer; a component is never, so the map value is stored back.
fn same(a: &Item, b: &Item) -> bool {
    match (a, b) {
        (Item::Geometry(Geometry::OBB(x)), Item::Geometry(Geometry::OBB(y))) => Rc::ptr_eq(x, y),
        (Item::Geometry(Geometry::BRep(x)), Item::Geometry(Geometry::BRep(y))) => Rc::ptr_eq(x, y),
        (Item::Geometry(Geometry::Element(x)), Item::Geometry(Geometry::Element(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::Geometry(Geometry::Line(x)), Item::Geometry(Geometry::Line(y))) => Rc::ptr_eq(x, y),
        (Item::Geometry(Geometry::Mesh(x)), Item::Geometry(Geometry::Mesh(y))) => Rc::ptr_eq(x, y),
        (Item::Geometry(Geometry::NurbsCurve(x)), Item::Geometry(Geometry::NurbsCurve(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::Geometry(Geometry::NurbsSurface(x)), Item::Geometry(Geometry::NurbsSurface(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::Geometry(Geometry::Plane(x)), Item::Geometry(Geometry::Plane(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::Geometry(Geometry::Point(x)), Item::Geometry(Geometry::Point(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::Geometry(Geometry::PointCloud(x)), Item::Geometry(Geometry::PointCloud(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::Geometry(Geometry::Polyline(x)), Item::Geometry(Geometry::Polyline(y))) => {
            Rc::ptr_eq(x, y)
        }
        (Item::InstanceRef(x), Item::InstanceRef(y)) => Rc::ptr_eq(x, y),
        _ => false,
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
    #[serde(skip)]
    pub sweep: Vec<Weak<RefCell<TreeNode>>>, // Parents whose children died, for the purge to compact.
    #[serde(skip)]
    pub pinned: Vec<Weak<RefCell<TreeNode>>>, // Parents the purge left while a record pinned a child.
    #[serde(skip)]
    pub purging: Option<usize>, // The list the purge cursor is in, None between cycles.
}

impl Default for Session {
    /// Construct a session named "my_session".
    fn default() -> Self {
        Self::new("my_session")
    }
}

impl Clone for Session {
    /// Copy every live table and object into compacted lists, guids included; caches are rebuilt on demand and history starts empty.
    fn clone(&self) -> Self {
        let mut session = Session::new(&self.name);

        if self.has_guid() {
            session.set_guid(self.guid().to_string());
        }

        session.objects = clone_objects(&self.objects_synced());
        session.definitions = clone_objects(&synced(&self.definitions, &self.definition_lookup));
        session.tree = self.tree.clone();
        session.graph = self.graph.clone();
        session.graph.renumber();
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
            sweep: Vec::new(),
            pinned: Vec::new(),
            purging: None,
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
        if !self._is_live(guid) {
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
            push(&mut out, collection, &Item::Geometry(resolved));
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

        if self._is_live(&guid) {
            return String::new();
        }

        let (collection, _) = collection_of(&definition);
        let slot = push(
            &mut self.definitions,
            collection,
            &Item::Geometry(definition.clone()),
        );
        self.definition_lookup.insert(guid.clone(), definition);
        self.bvh_cache_dirty = true;
        self.revision += 1;

        if self.history.current.is_some() {
            let tomb = Tomb::new(collection, true, slot, None);
            pin(&mut self.definitions, collection, slot, &tomb);
            self.history.record(
                Op::Add(Tombstone::new(
                    guid.clone(),
                    "definitions".to_string(),
                    None,
                    0,
                    None,
                    tomb,
                )),
                RECORD,
            );
        }

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

    /// Put a TreeNode under a parent, the root when none is given: a placed node moves and leaves a ghost, one already there is left alone.
    pub fn add<'a>(
        &mut self,
        node: &Rc<RefCell<TreeNode>>,
        parent: impl Into<Option<&'a Rc<RefCell<TreeNode>>>>,
    ) where
        Rc<RefCell<TreeNode>>: 'a,
    {
        let Some(parent) = parent.into().cloned().or_else(|| self.tree.root()) else {
            return;
        };
        let name = node.borrow().name.clone();
        let held = node
            .borrow()
            .parent()
            .is_some_and(|held| Rc::ptr_eq(&held, &parent));

        if held || Rc::ptr_eq(node, &parent) {
            return;
        }

        let was_dead = node.borrow().is_dead();
        let old = node.borrow().parent();
        let ghost = parent.borrow_mut().add(node);

        if !parent.borrow().has_child(node) {
            return;
        }

        node.borrow_mut().set_dead(false);
        self.revision += 1;

        if let (Some(old), true) = (old, ghost.is_some()) {
            self._queue(&old);
        }

        // a group comes back with its transform; an object's stays with its own tomb
        let tomb = node
            .borrow()
            .get_tomb()
            .filter(|tomb| tomb.collection.is_empty());

        if let (true, Some(tomb)) = (was_dead, tomb) {
            if let Some(xform) = tomb.xform.borrow_mut().take() {
                self.xforms.insert(name.clone(), xform);
            }
        }

        if self._is_live(&name) {
            self.node_lookup.insert(name.clone(), Rc::clone(node));
        }

        if self.history.current.is_none() {
            self.history.dropped += usize::from(ghost.is_some());

            return;
        }

        let tomb = self._node_tomb(ghost.as_ref().unwrap_or(node));
        let color = node.borrow().color.clone();
        let dead_before = was_dead || ghost.is_none();
        self.history.record(
            Op::Tree(TreeOp::new(
                name.clone(),
                Rc::clone(node),
                tomb,
                ghost,
                name.clone(),
                name,
                color.clone(),
                color,
                dead_before,
                false,
            )),
            RECORD,
        );
    }

    /// Create a named group (TreeNode) and add it to the root of the tree.
    pub fn add_group(&mut self, group_name: &str) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(group_name);
        self.add(&node, None);

        node
    }

    /// Rename a group node; false for an object node, a dead node or the same name.
    pub fn rename_node(&mut self, node: &Rc<RefCell<TreeNode>>, name: &str) -> bool {
        let before = node.borrow().name.clone();

        if self._is_live(&before) || node.borrow().is_dead() || before == name {
            return false;
        }

        node.borrow_mut().name = name.to_string();
        self.revision += 1;

        if self.history.current.is_some() {
            let tomb = self._node_tomb(node);
            let color = node.borrow().color.clone();
            self.history.record(
                Op::Tree(TreeOp::new(
                    before.clone(),
                    Rc::clone(node),
                    tomb,
                    None,
                    before,
                    name.to_string(),
                    color.clone(),
                    color,
                    false,
                    false,
                )),
                RECORD,
            );
        }

        true
    }

    /// Set or clear (None) the display colour of a node; false for a dead node.
    pub fn set_node_color(&mut self, node: &Rc<RefCell<TreeNode>>, color: Option<Color>) -> bool {
        if node.borrow().is_dead() {
            return false;
        }

        let before = node.borrow().color.clone();
        node.borrow_mut().color = color.clone();
        self.revision += 1;

        if self.history.current.is_some() {
            let name = node.borrow().name.clone();
            let tomb = self._node_tomb(node);
            self.history.record(
                Op::Tree(TreeOp::new(
                    name.clone(),
                    Rc::clone(node),
                    tomb,
                    None,
                    name.clone(),
                    name,
                    before,
                    color,
                    false,
                    false,
                )),
                RECORD,
            );
        }

        true
    }

    /// Kill a group node with everything below it, parking its transform; false for an object node, the root or a dead node.
    pub fn remove_group(&mut self, node: &Rc<RefCell<TreeNode>>) -> bool {
        let name = node.borrow().name.clone();
        let Some(parent) = node.borrow().parent() else {
            return false;
        };

        if self._is_live(&name) {
            return false;
        }

        let tomb = self._node_tomb(node);
        node.borrow_mut().set_dead(true);
        *tomb.xform.borrow_mut() = self.xforms.remove(&name);
        self._queue(&parent);
        self.revision += 1;

        if self.history.current.is_none() {
            self.history.dropped += 1;

            return true;
        }

        let color = node.borrow().color.clone();
        self.history.record(
            Op::Tree(TreeOp::new(
                name.clone(),
                Rc::clone(node),
                tomb,
                None,
                name.clone(),
                name,
                color.clone(),
                color,
                false,
                true,
            )),
            RECORD,
        );

        true
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

    /// Kill an object in place: its slot, node, transform, vertex, edges and interactions flip dead until undo revives them; O(1 + d log V).
    pub fn remove_object(&mut self, obj_guid: &str) -> bool {
        let Some(tomb) = self._tomb(obj_guid) else {
            return false;
        };
        let obj = self._item(obj_guid);
        let degree = self.graph.edges.get(obj_guid).map_or(0, BTreeMap::len);
        let node = tomb
            .node
            .clone()
            .filter(|node| node.borrow().parent().is_some());
        let index = node.as_ref().map_or(0, |node| node.borrow().at());
        let parent_guid = node
            .as_ref()
            .and_then(|node| node.borrow().parent())
            .map(|parent| parent.borrow().name.clone());
        self._kill(&tomb);

        if self.history.current.is_none() {
            self.history.dropped += 1;

            return true;
        }

        let bytes = RECORD + obj.as_ref().map_or(0, weight) + 128 * degree;
        let collection = tomb.collection.clone();
        self.history.record(
            Op::Remove(Tombstone::new(
                obj_guid.to_string(),
                collection,
                parent_guid,
                index,
                node,
                tomb,
            )),
            bytes,
        );

        true
    }

    /// Swap the object stored under guid for obj, which takes over that guid; a different type moves the guid to that type's list, the node and edges staying.
    pub fn replace(&mut self, guid: &str, obj: Geometry) -> bool {
        let Some(before) = self.lookup.get(guid).cloned() else {
            return false;
        };
        let mut obj = obj;

        if obj.guid() != guid {
            obj.set_guid(guid);
        }

        let (old, _) = collection_of(&before);
        let (new, prefix) = collection_of(&obj);
        let bytes = RECORD + weight(&Item::Geometry(before.clone()));

        if old == new {
            if self.history.current.is_some() {
                self.history.record(
                    Op::Replace(ReplaceOp::new(
                        guid.to_string(),
                        Item::Geometry(before),
                        Item::Geometry(obj.clone()),
                    )),
                    bytes,
                );
            }

            self._swap(guid, Item::Geometry(obj));

            return true;
        }

        let node = self.get_node(guid);
        let Some(removed) = self._half(false, old, guid) else {
            return false;
        };
        self._kill(&removed);
        let slot = push(&mut self.objects, new, &Item::Geometry(obj.clone()));
        let added = Tomb::new(new, false, slot, None);
        pin(&mut self.objects, new, slot, &added);
        self.lookup.insert(guid.to_string(), obj.clone());
        self._label(guid, &format!("{prefix}_{}", obj.name()));
        self._pair(guid, old, new, node, removed, added, bytes);

        true
    }

    /// Swap the geometry of a definition, which keeps its guid, so every instance of it changes at once; false when guid is no definition.
    pub fn replace_definition(&mut self, guid: &str, definition: Geometry) -> bool {
        let Some(before) = self.definition_lookup.get(guid).cloned() else {
            return false;
        };
        let mut definition = definition;

        if definition.guid() != guid {
            definition.set_guid(guid);
        }

        let (old, _) = collection_of(&before);
        let (new, _) = collection_of(&definition);
        let bytes = RECORD + weight(&Item::Geometry(before.clone()));

        if old == new {
            if self.history.current.is_some() {
                self.history.record(
                    Op::Replace(ReplaceOp::new(
                        guid.to_string(),
                        Item::Geometry(before),
                        Item::Geometry(definition.clone()),
                    )),
                    bytes,
                );
            }

            self._swap(guid, Item::Geometry(definition));

            return true;
        }

        let Some(removed) = self._half(true, old, guid) else {
            return false;
        };
        self._kill(&removed);
        let slot = push(
            &mut self.definitions,
            new,
            &Item::Geometry(definition.clone()),
        );
        let added = Tomb::new(new, true, slot, None);
        pin(&mut self.definitions, new, slot, &added);
        self.definition_lookup.insert(guid.to_string(), definition);
        self._pair(
            guid,
            "definitions",
            "definitions",
            None,
            removed,
            added,
            bytes,
        );

        true
    }

    /// Remove a definition; false when guid is no definition or an instance still names it.
    pub fn remove_definition(&mut self, guid: &str) -> bool {
        let Some(before) = self.definition_lookup.get(guid).cloned() else {
            return false;
        };

        if !self.instances_of(guid).is_empty() {
            return false;
        }

        let (collection, _) = collection_of(&before);
        let Some(tomb) = self._half(true, collection, guid) else {
            return false;
        };
        self._kill(&tomb);

        if self.history.current.is_none() {
            self.history.dropped += 1;

            return true;
        }

        self.history.record(
            Op::Remove(Tombstone::new(
                guid.to_string(),
                "definitions".to_string(),
                None,
                0,
                None,
                tomb,
            )),
            RECORD + weight(&Item::Geometry(before)),
        );

        true
    }

    /// Turn an object into an instance of a definition, keeping its guid, name, tree node and edges; frame maps the definition onto the object and is folded into its local transform.
    pub fn to_instance(&mut self, guid: &str, definition_guid: &str, frame: Xform) -> bool {
        let Some(object) = self.lookup.get(guid).cloned() else {
            return false;
        };

        if !self.definition_lookup.contains_key(definition_guid) {
            return false;
        }

        let mut instance = InstanceRef::new(definition_guid, Xform::identity());
        instance.set_guid(guid.to_string());
        instance.name = object.name().to_string();
        let label = format!("instance_{}", instance.name);
        let placement = &self.xform(guid) * &frame;
        let (old, _) = collection_of(&object);
        let node = self.get_node(guid);
        let Some(removed) = self._half(false, old, guid) else {
            return false;
        };
        self._kill(&removed);
        let instance = Rc::new(instance);
        let slot = push(
            &mut self.objects,
            "instances",
            &Item::InstanceRef(Rc::clone(&instance)),
        );
        let added = Tomb::new("instances", false, slot, None);
        pin(&mut self.objects, "instances", slot, &added);
        self.instance_lookup.insert(guid.to_string(), instance);
        self._label(guid, &label);
        let bytes = RECORD + weight(&Item::Geometry(object));
        self._pair(guid, old, "instances", node, removed, added, bytes);

        if placement.is_identity() {
            self.remove_xform(guid);
        } else {
            self.set_xform(guid, placement);
        }

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
        let label = format!("{prefix}_{}", instance.name);
        let node = self.get_node(instance_guid);
        let Some(removed) = self._half(false, "instances", instance_guid) else {
            return false;
        };
        self._kill(&removed);
        let slot = push(&mut self.objects, collection, &Item::Geometry(copy.clone()));
        let added = Tomb::new(collection, false, slot, None);
        pin(&mut self.objects, collection, slot, &added);
        self.lookup.insert(instance_guid.to_string(), copy);
        self._label(instance_guid, &label);
        let bytes = RECORD + weight(&Item::InstanceRef(instance));
        self._pair(
            instance_guid,
            "instances",
            collection,
            node,
            removed,
            added,
            bytes,
        );

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
            self.history.record(
                Op::Xform(XformOp::new(guid.to_string(), before, Some(xform.clone()))),
                RECORD,
            );
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
            self.history.record(
                Op::Xform(XformOp::new(guid.to_string(), Some(before), None)),
                RECORD,
            );
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

    /// Revert and drop the open transaction, leaving the stacks as they are; false when none is open.
    pub fn abort(&mut self) -> bool {
        self.revision += 1;
        let mut history = std::mem::take(&mut self.history);
        let aborted = history.abort(self);
        self.history = history;

        aborted
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

    /// Store an object in its list, lookup, graph and tree, recording an add when a transaction is open.
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
        let slot = push(&mut self.objects, collection, &obj);

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

        if let Some(host) = &host {
            self.tree.add(&node, Some(host));
            parent_guid = Some(host.borrow().name.clone());
        }

        if self.history.current.is_some() {
            let tomb = Tomb::new(collection, false, slot, Some(Rc::clone(&node)));
            pin(&mut self.objects, collection, slot, &tomb);
            node.borrow_mut().set_tomb(&tomb);
            let index = node.borrow().at();
            self.history.record(
                Op::Add(Tombstone::new(
                    guid,
                    collection.to_string(),
                    parent_guid,
                    index,
                    Some(Rc::clone(&node)),
                    tomb,
                )),
                RECORD,
            );
        }

        node
    }

    /// Whether guid names a live object, component or instance.
    fn _is_live(&self, guid: &str) -> bool {
        self.lookup.contains_key(guid)
            || self.component_lookup.contains_key(guid)
            || self.instance_lookup.contains_key(guid)
    }

    /// The stored object, component or instance under guid, the pointer itself.
    fn _item(&self, guid: &str) -> Option<Item> {
        if let Some(geometry) = self.lookup.get(guid) {
            return Some(Item::Geometry(geometry.clone()));
        }

        if let Some(component) = self.component_lookup.get(guid) {
            return Some(Item::Component(component.clone()));
        }

        Some(Item::InstanceRef(Rc::clone(
            self.instance_lookup.get(guid)?,
        )))
    }

    /// The object tomb of a live guid, reused while a record still holds it, else made and pinned on its slot and node; O(1).
    fn _tomb(&mut self, guid: &str) -> Option<Rc<Tomb>> {
        let item = self._item(guid)?;
        let (collection, _) = collection_for(&item);
        let slot = match slot_of(&self.objects, collection, guid) {
            Some(slot) => slot,
            None => push(&mut self.objects, collection, &item),
        };

        if let Some(tomb) = tomb_at(&self.objects, collection, slot) {
            if tomb.node.is_some() {
                return Some(tomb);
            }
        }

        let node = match self.get_node(guid) {
            Some(node) => {
                self.node_lookup.insert(guid.to_string(), Rc::clone(&node));
                node
            }

            // an object outside the tree parks its transform and vertex on a detached node
            None => TreeNode::new(guid),
        };
        let tomb = Tomb::new(collection, false, slot, Some(Rc::clone(&node)));
        pin(&mut self.objects, collection, slot, &tomb);
        node.borrow_mut().set_tomb(&tomb);

        Some(tomb)
    }

    /// The node-only tomb pinned on a node, reused while a record still holds it.
    fn _node_tomb(&mut self, node: &Rc<RefCell<TreeNode>>) -> Rc<Tomb> {
        if let Some(tomb) = node.borrow().get_tomb() {
            if tomb.collection.is_empty() {
                return tomb;
            }
        }

        let tomb = Tomb::new("", false, 0, Some(Rc::clone(node)));
        node.borrow_mut().set_tomb(&tomb);

        tomb
    }

    /// A slot-only tomb on the live slot of guid in the list of that name, reused while a record still holds it; a map-only entry is pushed first.
    fn _half(&mut self, definition: bool, collection: &str, guid: &str) -> Option<Rc<Tomb>> {
        let item = if definition {
            Item::Geometry(self.definition_lookup.get(guid)?.clone())
        } else {
            self._item(guid)?
        };
        let objects = if definition {
            &mut self.definitions
        } else {
            &mut self.objects
        };
        let slot =
            slot_of(objects, collection, guid).unwrap_or_else(|| push(objects, collection, &item));

        if let Some(tomb) = tomb_at(objects, collection, slot) {
            if tomb.node.is_none() && tomb.definition == definition {
                return Some(tomb);
            }
        }

        let tomb = Tomb::new(collection, definition, slot, None);
        pin(objects, collection, slot, &tomb);

        Some(tomb)
    }

    /// Record the halves of a type change under one guid, or drop them when no transaction is open.
    #[allow(clippy::too_many_arguments)]
    fn _pair(
        &mut self,
        guid: &str,
        old: &str,
        new: &str,
        node: Option<Rc<RefCell<TreeNode>>>,
        removed: Rc<Tomb>,
        added: Rc<Tomb>,
        bytes: usize,
    ) {
        self.revision += 1;
        self.bvh_cache_dirty = true;

        if self.history.current.is_none() {
            self.history.dropped += 1;

            return;
        }

        let index = node.as_ref().map_or(0, |node| node.borrow().at());
        let parent_guid = node
            .as_ref()
            .and_then(|node| node.borrow().parent())
            .map(|parent| parent.borrow().name.clone());
        self.history.record(
            Op::Remove(Tombstone::new(
                guid.to_string(),
                old.to_string(),
                parent_guid.clone(),
                index,
                node.clone(),
                removed,
            )),
            bytes,
        );
        self.history.record(
            Op::Add(Tombstone::new(
                guid.to_string(),
                new.to_string(),
                parent_guid,
                index,
                node,
                added,
            )),
            RECORD,
        );
    }

    /// Relabel the graph vertex of guid, when it has one.
    fn _label(&mut self, guid: &str, label: &str) {
        if self.graph.has_node(guid) {
            self.graph.node_label(guid, Some(label));
        }
    }

    /// Remember a parent whose child died, once, for the sweep.
    fn _queue(&mut self, parent: &Rc<RefCell<TreeNode>>) {
        if parent.borrow().is_queued() {
            return;
        }

        parent.borrow_mut().set_queued(true);
        self.sweep.push(Rc::downgrade(parent));
    }

    /// Flip a tomb dead: its slot and map entry, and for an object tomb its node, transform, vertex, edges and interactions; O(1 + d log V).
    pub(crate) fn _kill(&mut self, tomb: &Rc<Tomb>) {
        if tomb.collection.is_empty() {
            return;
        }

        let slot = tomb.slot.get();
        let collection = tomb.collection.as_str();
        self.revision += 1;
        self.bvh_cache_dirty = true;

        if tomb.definition {
            let Some(stored) = item_at(&self.definitions, collection, slot) else {
                return;
            };
            let guid = stored.guid().to_string();

            if let Some(held) = self.definition_lookup.get(&guid) {
                let held = Item::Geometry(held.clone());

                if !same(&held, &stored) {
                    store(&mut self.definitions, collection, slot, &held);
                }
            }

            let owner = slot_of(&self.definitions, collection, &guid) == Some(slot);
            flag(&mut self.definitions, collection, slot, true);

            if owner {
                self.definition_lookup.remove(&guid);
            }

            return;
        }

        let Some(stored) = item_at(&self.objects, collection, slot) else {
            return;
        };
        let guid = stored.guid().to_string();

        if let Some(held) = self._item(&guid) {
            if !same(&held, &stored) {
                store(&mut self.objects, collection, slot, &held);
            }
        }

        let owner = slot_of(&self.objects, collection, &guid) == Some(slot);
        flag(&mut self.objects, collection, slot, true);

        if owner {
            self.lookup.remove(&guid);
            self.component_lookup.remove(&guid);
            self.instance_lookup.remove(&guid);
        }

        let Some(node) = &tomb.node else {
            return;
        };
        let parent = node.borrow().parent();
        node.borrow_mut().set_dead(true);
        node.borrow_mut().set_tomb(tomb);

        if self
            .node_lookup
            .get(&guid)
            .is_some_and(|held| Rc::ptr_eq(held, node))
        {
            self.node_lookup.remove(&guid);
        }

        if let Some(parent) = parent {
            self._queue(&parent);
        }

        *tomb.xform.borrow_mut() = self.xforms.remove(&guid);

        if let Some((vertex, edges)) = self.graph.take_node(&guid) {
            for edge in &edges {
                if !edge.has_guid() {
                    continue;
                }

                if let Some(list) = self.interactions.remove(edge.guid()) {
                    tomb.interactions
                        .borrow_mut()
                        .insert(edge.guid().to_string(), list);
                }
            }

            *tomb.vertex.borrow_mut() = Some(vertex);
            *tomb.edges.borrow_mut() = edges;
        }
    }

    /// Flip a tomb live again: the same slot and pointer, and for an object tomb the same node, transform, vertex, edges and interactions; O(1 + d log V).
    pub(crate) fn _revive(&mut self, tomb: &Rc<Tomb>) {
        if tomb.collection.is_empty() {
            return;
        }

        let slot = tomb.slot.get();
        let collection = tomb.collection.as_str();
        self.revision += 1;
        self.bvh_cache_dirty = true;

        if tomb.definition {
            flag(&mut self.definitions, collection, slot, false);

            if let Some(Item::Geometry(geometry)) = item_at(&self.definitions, collection, slot) {
                self.definition_lookup
                    .insert(geometry.guid().to_string(), geometry);
            }

            return;
        }

        flag(&mut self.objects, collection, slot, false);
        let Some(item) = item_at(&self.objects, collection, slot) else {
            return;
        };
        let guid = item.guid().to_string();

        match &item {
            Item::Geometry(geometry) => {
                self.lookup.insert(guid.clone(), geometry.clone());
            }

            Item::Component(component) => {
                self.component_lookup
                    .insert(guid.clone(), component.clone());
            }

            Item::InstanceRef(instance) => {
                self.instance_lookup
                    .insert(guid.clone(), Rc::clone(instance));
            }
        }

        let Some(node) = &tomb.node else {
            self._label(&guid, &format!("{}_{}", prefix_of(collection), item.name()));

            return;
        };
        node.borrow_mut().set_dead(false);

        if node.borrow().parent().is_some() {
            self.node_lookup.insert(guid.clone(), Rc::clone(node));
        }

        if let Some(xform) = tomb.xform.borrow_mut().take() {
            self.xforms.insert(guid.clone(), xform);
        }

        let Some(vertex) = tomb.vertex.borrow_mut().take() else {
            return;
        };
        let edges = std::mem::take(&mut *tomb.edges.borrow_mut());
        let mut ids: Vec<(String, String)> = Vec::with_capacity(edges.len());

        for edge in &edges {
            if edge.has_guid() {
                ids.push((edge.other_vertex(&guid), edge.guid().to_string()));
            }
        }

        self.graph.put_node(vertex, edges);

        for (other, id) in ids {
            let back = self
                .graph
                .edges
                .get(&guid)
                .and_then(|neighbors| neighbors.get(&other))
                .is_some_and(|edge| edge.guid() == id);

            if !back {
                continue;
            }

            if let Some(list) = tomb.interactions.borrow_mut().remove(&id) {
                self.interactions.insert(id, list);
            }
        }
    }

    /// Store obj under guid in its slot and map, relabelling its vertex; a guid that is only a definition swaps in Session.definitions; O(1).
    pub(crate) fn _swap(&mut self, guid: &str, obj: Item) {
        let (collection, prefix) = collection_for(&obj);
        let label = format!("{prefix}_{}", obj.name());
        self.revision += 1;
        self.bvh_cache_dirty = true;

        if let (Item::Geometry(geometry), false) = (&obj, self._is_live(guid)) {
            if self.definition_lookup.contains_key(guid) {
                if let Some(slot) = slot_of(&self.definitions, collection, guid) {
                    store(&mut self.definitions, collection, slot, &obj);
                }

                self.definition_lookup
                    .insert(guid.to_string(), geometry.clone());
            }

            return;
        }

        let Some(slot) = slot_of(&self.objects, collection, guid) else {
            return;
        };
        store(&mut self.objects, collection, slot, &obj);

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

        self._label(guid, &label);
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

            if self._is_live(&guid) && !self.node_lookup.contains_key(&guid) {
                self.node_lookup.insert(guid, node);
            }
        }

        self.indexed = self.tree.root().map(|root| Rc::downgrade(&root));
    }

    /// Apply the before (back) or after state of a tree record: name, colour, liveness, and for a move the swap of node and ghost.
    pub(crate) fn _tree(&mut self, op: &TreeOp, back: bool) {
        let (name, color, dead) = if back {
            (&op.name_before, &op.color_before, op.dead_before)
        } else {
            (&op.name_after, &op.color_after, op.dead_after)
        };
        let was = op.node.borrow().is_dead();
        let live = self._is_live(name);

        // the ghost takes the node's place, and that parent is swept
        if let Some(ghost) = &op.ghost {
            let from = op.node.borrow().parent();
            TreeNode::swap(&op.node, ghost);
            ghost.borrow_mut().set_tomb(&op.tomb);

            if let Some(from) = from {
                self._queue(&from);
            }
        }

        let parent = op.node.borrow().parent();
        op.node.borrow_mut().name = name.clone();
        op.node.borrow_mut().color = color.clone();
        op.node.borrow_mut().set_dead(dead);
        self.revision += 1;

        if dead && !was {
            op.node.borrow_mut().set_tomb(&op.tomb);

            if let Some(parent) = parent {
                self._queue(&parent);
            }
        }

        // a live object keeps its transform, only a group parks it
        if dead && !was && !live {
            *op.tomb.xform.borrow_mut() = self.xforms.remove(name);
        }

        if was && !dead && !live {
            if let Some(xform) = op.tomb.xform.borrow_mut().take() {
                self.xforms.insert(name.clone(), xform);
            }
        }

        if !live {
            return;
        }

        if !dead {
            self.node_lookup.insert(name.clone(), Rc::clone(&op.node));
        } else if self
            .node_lookup
            .get(name)
            .is_some_and(|held| Rc::ptr_eq(held, &op.node))
        {
            self.node_lookup.remove(name);
        }
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
