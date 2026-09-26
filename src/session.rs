use crate::collection::Keyed;
use crate::color::Color;
use crate::history::clone;
use crate::history::weight;
use crate::history::Entry;
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
use crate::tree::node_head;
use crate::tree::node_tail;
use crate::BRep;
use crate::Collection;
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
use std::collections::BTreeSet;
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
        match self {
            Geometry::OBB(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::BRep(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::Element(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::Line(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::Mesh(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::NurbsCurve(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::NurbsSurface(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::Plane(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::Point(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::PointCloud(g) => Rc::make_mut(g).set_guid(guid.to_string()),
            Geometry::Polyline(g) => Rc::make_mut(g).set_guid(guid.to_string()),
        }
    }

    /// Overwrite the name.
    pub(crate) fn set_name(&mut self, name: &str) {
        match self {
            Geometry::OBB(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::BRep(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::Element(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::Line(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::Mesh(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::NurbsCurve(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::NurbsSurface(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::Plane(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::Point(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::PointCloud(g) => Rc::make_mut(g).name = name.to_string(),
            Geometry::Polyline(g) => Rc::make_mut(g).name = name.to_string(),
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

/// The slot bookkeeping every Collection shares, whatever it holds.
trait Slots {
    /// Return the slot of a live guid.
    fn get_slot(&self, guid: &str) -> Option<usize>;

    /// Return the guid in a slot, dead or alive.
    fn key_at(&self, slot: usize) -> &str;

    /// Return whether a slot is dead.
    fn is_dead(&self, slot: usize) -> bool;

    /// Kill or revive a slot.
    fn set_dead(&mut self, slot: usize, dead: bool);

    /// Return the tomb pinning a slot while a record still holds it.
    fn get_tomb(&self, slot: usize) -> Option<Rc<Tomb>>;

    /// Pin a slot to a tomb.
    fn set_tomb(&mut self, slot: usize, tomb: &Rc<Tomb>);

    /// Return the number of dead slots not yet purged.
    fn number_of_dead(&self) -> usize;

    /// Return the number of raw slots, dead ones included.
    fn number_of_slots(&self) -> usize;

    /// Return whether a compaction is part way.
    fn is_compacting(&self) -> bool;

    /// Purge unpinned dead slots for at most `work` slots; returns the slots examined.
    fn compact_step(&mut self, work: usize) -> usize;
}

impl<E: Keyed> Slots for Collection<E> {
    fn get_slot(&self, guid: &str) -> Option<usize> {
        Collection::get_slot(self, guid)
    }

    fn key_at(&self, slot: usize) -> &str {
        self.get_item(slot).key()
    }

    fn is_dead(&self, slot: usize) -> bool {
        Collection::is_dead(self, slot)
    }

    fn set_dead(&mut self, slot: usize, dead: bool) {
        Collection::set_dead(self, slot, dead)
    }

    fn get_tomb(&self, slot: usize) -> Option<Rc<Tomb>> {
        Collection::get_tomb(self, slot)
    }

    fn set_tomb(&mut self, slot: usize, tomb: &Rc<Tomb>) {
        Collection::set_tomb(self, slot, tomb)
    }

    fn number_of_dead(&self) -> usize {
        Collection::number_of_dead(self)
    }

    fn number_of_slots(&self) -> usize {
        Collection::number_of_slots(self)
    }

    fn is_compacting(&self) -> bool {
        Collection::is_compacting(self)
    }

    fn compact_step(&mut self, work: usize) -> usize {
        Collection::compact_step(self, work)
    }
}

/// The Collection of that name, the Rust spelling of getattr(objects, collection).
fn list<'a>(objects: &'a Objects, collection: &str) -> Option<&'a dyn Slots> {
    match collection {
        "points" => Some(&objects.points),
        "lines" => Some(&objects.lines),
        "planes" => Some(&objects.planes),
        "bboxes" => Some(&objects.bboxes),
        "polylines" => Some(&objects.polylines),
        "pointclouds" => Some(&objects.pointclouds),
        "meshes" => Some(&objects.meshes),
        "nurbscurves" => Some(&objects.nurbscurves),
        "nurbssurfaces" => Some(&objects.nurbssurfaces),
        "breps" => Some(&objects.breps),
        "elements" => Some(&objects.elements),
        "components" => Some(&objects.components),
        "instances" => Some(&objects.instances),
        _ => None,
    }
}

/// The Collection of that name, mutable.
fn list_mut<'a>(objects: &'a mut Objects, collection: &str) -> Option<&'a mut dyn Slots> {
    match collection {
        "points" => Some(&mut objects.points),
        "lines" => Some(&mut objects.lines),
        "planes" => Some(&mut objects.planes),
        "bboxes" => Some(&mut objects.bboxes),
        "polylines" => Some(&mut objects.polylines),
        "pointclouds" => Some(&mut objects.pointclouds),
        "meshes" => Some(&mut objects.meshes),
        "nurbscurves" => Some(&mut objects.nurbscurves),
        "nurbssurfaces" => Some(&mut objects.nurbssurfaces),
        "breps" => Some(&mut objects.breps),
        "elements" => Some(&mut objects.elements),
        "components" => Some(&mut objects.components),
        "instances" => Some(&mut objects.instances),
        _ => None,
    }
}

/// A deep copy of a list, every object cloned, guids included.
fn deep<T: Clone>(list: &Collection<Rc<T>>) -> Collection<Rc<T>>
where
    Rc<T>: Keyed,
{
    list.iter().map(|item| Rc::new((**item).clone())).collect()
}

/// A copy of a list with each object's world placement baked into its coordinates.
fn baked<T: Clone>(
    list: &Collection<Rc<T>>,
    world: &HashMap<String, Xform>,
    bake: impl Fn(&mut T, &Xform),
) -> Collection<Rc<T>>
where
    Rc<T>: Keyed,
{
    list.iter()
        .map(|item| {
            let mut copy = (**item).clone();

            if let Some(xform) = world.get(item.key()) {
                if !xform.is_identity() {
                    bake(&mut copy, xform);
                }
            }

            Rc::new(copy)
        })
        .collect()
}

/// A deep copy of every vector and every object in it, guids included.
fn clone_objects(objects: &Objects) -> Objects {
    let mut out = objects.clone();
    out.points = deep(&objects.points);
    out.lines = deep(&objects.lines);
    out.planes = deep(&objects.planes);
    out.bboxes = deep(&objects.bboxes);
    out.polylines = deep(&objects.polylines);
    out.pointclouds = deep(&objects.pointclouds);
    out.meshes = deep(&objects.meshes);
    out.nurbscurves = deep(&objects.nurbscurves);
    out.nurbssurfaces = deep(&objects.nurbssurfaces);
    out.breps = deep(&objects.breps);
    out.elements = deep(&objects.elements);
    out.instances = deep(&objects.instances);

    out
}

/// The vectors of objects re-pointed at lookup: `Rc::make_mut` on a lookup entry splits it from its vector, and the lookup is the mutable truth.
fn synced(objects: &Objects, lookup: &HashMap<String, Geometry>) -> Objects {
    let mut objects = objects.clone();
    repoint(&mut objects, lookup);

    objects
}

/// Point the live slot of guid at held when it stores another pointer.
fn resync<T>(list: &mut Collection<Rc<T>>, guid: &str, held: &Rc<T>)
where
    Rc<T>: Keyed,
{
    let Some(slot) = list.get_slot(guid) else {
        return;
    };

    if !Rc::ptr_eq(list.get_item(slot), held) {
        list.set_item(slot, Rc::clone(held));
    }
}

/// Point every live slot whose guid lookup holds with another value at the lookup value.
fn repoint(objects: &mut Objects, lookup: &HashMap<String, Geometry>) {
    for (guid, geometry) in lookup {
        match geometry {
            Geometry::Point(g) => resync(&mut objects.points, guid, g),
            Geometry::Line(g) => resync(&mut objects.lines, guid, g),
            Geometry::Plane(g) => resync(&mut objects.planes, guid, g),
            Geometry::OBB(g) => resync(&mut objects.bboxes, guid, g),
            Geometry::Polyline(g) => resync(&mut objects.polylines, guid, g),
            Geometry::PointCloud(g) => resync(&mut objects.pointclouds, guid, g),
            Geometry::Mesh(g) => resync(&mut objects.meshes, guid, g),
            Geometry::NurbsCurve(g) => resync(&mut objects.nurbscurves, guid, g),
            Geometry::NurbsSurface(g) => resync(&mut objects.nurbssurfaces, guid, g),
            Geometry::BRep(g) => resync(&mut objects.breps, guid, g),
            Geometry::Element(g) => resync(&mut objects.elements, guid, g),
        }
    }
}

/// Index every live entry of a list that lookup lacks, wrapped as its Geometry variant.
fn index<T>(
    list: &Collection<Rc<T>>,
    lookup: &mut HashMap<String, Geometry>,
    wrap: fn(Rc<T>) -> Geometry,
) where
    Rc<T>: Keyed,
{
    for item in list {
        if !lookup.contains_key(item.key()) {
            lookup.insert(item.key().to_string(), wrap(Rc::clone(item)));
        }
    }
}

/// Index every live slot lookup lacks, then push every geometry only lookup holds, in guid order.
fn adopt(objects: &mut Objects, lookup: &mut HashMap<String, Geometry>) {
    index(&objects.points, lookup, Geometry::Point);
    index(&objects.lines, lookup, Geometry::Line);
    index(&objects.planes, lookup, Geometry::Plane);
    index(&objects.bboxes, lookup, Geometry::OBB);
    index(&objects.polylines, lookup, Geometry::Polyline);
    index(&objects.pointclouds, lookup, Geometry::PointCloud);
    index(&objects.meshes, lookup, Geometry::Mesh);
    index(&objects.nurbscurves, lookup, Geometry::NurbsCurve);
    index(&objects.nurbssurfaces, lookup, Geometry::NurbsSurface);
    index(&objects.breps, lookup, Geometry::BRep);
    index(&objects.elements, lookup, Geometry::Element);
    let mut orphans: Vec<&Geometry> = Vec::new();

    for geometry in lookup.values() {
        let (collection, _) = collection_of(geometry);

        if slot_of(objects, collection, geometry.guid()).is_none() {
            orphans.push(geometry);
        }
    }

    orphans.sort_by(|a, b| a.guid().cmp(b.guid()));

    for geometry in orphans {
        push(objects, &Item::Geometry(geometry.clone()));
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
    list(objects, collection)?.get_slot(guid)
}

/// The stored pointer in a slot of a list, dead or alive.
fn held<T>(list: &Collection<Rc<T>>, slot: usize) -> Rc<T> {
    Rc::clone(list.get_item(slot))
}

/// The item in a slot of the list of that name, dead or alive, as the stored pointer.
fn item_at(objects: &Objects, collection: &str, slot: usize) -> Option<Item> {
    match collection {
        "points" => Some(Geometry::Point(held(&objects.points, slot)).into()),
        "lines" => Some(Geometry::Line(held(&objects.lines, slot)).into()),
        "planes" => Some(Geometry::Plane(held(&objects.planes, slot)).into()),
        "bboxes" => Some(Geometry::OBB(held(&objects.bboxes, slot)).into()),
        "polylines" => Some(Geometry::Polyline(held(&objects.polylines, slot)).into()),
        "pointclouds" => Some(Geometry::PointCloud(held(&objects.pointclouds, slot)).into()),
        "meshes" => Some(Geometry::Mesh(held(&objects.meshes, slot)).into()),
        "nurbscurves" => Some(Geometry::NurbsCurve(held(&objects.nurbscurves, slot)).into()),
        "nurbssurfaces" => Some(Geometry::NurbsSurface(held(&objects.nurbssurfaces, slot)).into()),
        "breps" => Some(Geometry::BRep(held(&objects.breps, slot)).into()),
        "elements" => Some(Geometry::Element(held(&objects.elements, slot)).into()),
        "components" => Some(Item::Component(objects.components.get_item(slot).clone())),
        "instances" => Some(Item::InstanceRef(held(&objects.instances, slot))),
        _ => None,
    }
}

/// Append an entry to a list and return its slot.
fn pushed<E: Keyed>(list: &mut Collection<E>, item: E) -> usize {
    list.push(item);

    list.number_of_slots() - 1
}

/// Append an item to the list of its type and return its slot.
fn push(objects: &mut Objects, obj: &Item) -> usize {
    match obj {
        Item::Geometry(Geometry::Point(g)) => pushed(&mut objects.points, Rc::clone(g)),
        Item::Geometry(Geometry::Line(g)) => pushed(&mut objects.lines, Rc::clone(g)),
        Item::Geometry(Geometry::Plane(g)) => pushed(&mut objects.planes, Rc::clone(g)),
        Item::Geometry(Geometry::OBB(g)) => pushed(&mut objects.bboxes, Rc::clone(g)),
        Item::Geometry(Geometry::Polyline(g)) => pushed(&mut objects.polylines, Rc::clone(g)),
        Item::Geometry(Geometry::PointCloud(g)) => pushed(&mut objects.pointclouds, Rc::clone(g)),
        Item::Geometry(Geometry::Mesh(g)) => pushed(&mut objects.meshes, Rc::clone(g)),
        Item::Geometry(Geometry::NurbsCurve(g)) => pushed(&mut objects.nurbscurves, Rc::clone(g)),
        Item::Geometry(Geometry::NurbsSurface(g)) => {
            pushed(&mut objects.nurbssurfaces, Rc::clone(g))
        }
        Item::Geometry(Geometry::BRep(g)) => pushed(&mut objects.breps, Rc::clone(g)),
        Item::Geometry(Geometry::Element(g)) => pushed(&mut objects.elements, Rc::clone(g)),
        Item::Component(component) => pushed(&mut objects.components, component.clone()),
        Item::InstanceRef(instance) => pushed(&mut objects.instances, Rc::clone(instance)),
    }
}

/// Put an item in a slot of the list of that name; an item of another type is left out.
fn store(objects: &mut Objects, collection: &str, slot: usize, obj: &Item) {
    if collection_for(obj).0 != collection {
        return;
    }

    match obj {
        Item::Geometry(Geometry::Point(g)) => objects.points.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::Line(g)) => objects.lines.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::Plane(g)) => objects.planes.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::OBB(g)) => objects.bboxes.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::Polyline(g)) => objects.polylines.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::PointCloud(g)) => objects.pointclouds.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::Mesh(g)) => objects.meshes.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::NurbsCurve(g)) => objects.nurbscurves.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::NurbsSurface(g)) => {
            objects.nurbssurfaces.set_item(slot, Rc::clone(g))
        }
        Item::Geometry(Geometry::BRep(g)) => objects.breps.set_item(slot, Rc::clone(g)),
        Item::Geometry(Geometry::Element(g)) => objects.elements.set_item(slot, Rc::clone(g)),
        Item::Component(component) => objects.components.set_item(slot, component.clone()),
        Item::InstanceRef(instance) => objects.instances.set_item(slot, Rc::clone(instance)),
    }
}

/// Kill or revive a slot of the list of that name.
fn flag(objects: &mut Objects, collection: &str, slot: usize, dead: bool) {
    if let Some(list) = list_mut(objects, collection) {
        list.set_dead(slot, dead);
    }
}

/// The tomb pinning a slot of the list of that name, while a record still holds it.
fn tomb_at(objects: &Objects, collection: &str, slot: usize) -> Option<Rc<Tomb>> {
    list(objects, collection)?.get_tomb(slot)
}

/// Pin a slot of the list of that name to a tomb.
fn pin(objects: &mut Objects, collection: &str, slot: usize, tomb: &Rc<Tomb>) {
    if let Some(list) = list_mut(objects, collection) {
        list.set_tomb(slot, tomb);
    }
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

    match geometry {
        Geometry::Element(g) => Rc::make_mut(g).place(xform),
        Geometry::OBB(g) => Rc::make_mut(g).transform(xform),
        Geometry::BRep(g) => Rc::make_mut(g).transform(xform),
        Geometry::Line(g) => Rc::make_mut(g).transform(xform),
        Geometry::Mesh(g) => {
            Rc::make_mut(g).transform(xform);
        }

        Geometry::NurbsCurve(g) => {
            Rc::make_mut(g).transform(xform);
        }

        Geometry::NurbsSurface(g) => {
            Rc::make_mut(g).transform(xform);
        }

        Geometry::Plane(g) => Rc::make_mut(g).transform(xform),
        Geometry::Point(g) => Rc::make_mut(g).transform(xform),
        Geometry::PointCloud(g) => Rc::make_mut(g).transform(xform),
        Geometry::Polyline(g) => Rc::make_mut(g).transform(xform),
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

/// The vertices of a BRep and a 3x3 sample of each of its surfaces.
fn brep_points(brep: &BRep) -> Vec<Point> {
    let mut points: Vec<Point> = Vec::new();

    for vertex in &brep.m_vertices {
        points.push(vertex.point.clone());
    }

    for surface in &brep.m_surfaces {
        let (Some((u0, u1)), Some((v0, v1))) = (surface.domain(0), surface.domain(1)) else {
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

    points
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

        Geometry::BRep(brep) => points = brep_points(brep),

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

/// Work units of one idle purge or checkpoint step, about 2 ms: one raw slot, child, vertex or entry each.
pub const PURGE_WORK: usize = 16_384;

const HEAD: usize = 0; // Checkpoint phase: the session name and guid.
const OBJECTS: usize = 1; // Checkpoint phases 1..=13: the objects lists.
const TREE: usize = 14; // Checkpoint phase: the tree, depth first.
const VERTICES: usize = 15; // Checkpoint phase: the graph vertices.
const EDGES: usize = 16; // Checkpoint phase: the graph edges.
const ORDERED: usize = 17; // Checkpoint phases 17..=27: xforms in order() sequence.
const REST: usize = 28; // Checkpoint phase: xforms outside order(), by guid.
const DEFINITIONS: usize = 29; // Checkpoint phases 29..=41: the definitions lists.
const INTERACTIONS: usize = 42; // Checkpoint phase: the interactions, by edge guid.
const ASSEMBLY: usize = 43; // Checkpoint phase: the sections joined into one message.
const CHUNK: usize = 64 << 10; // Bytes past which a finished tree node's chunks move instead of being copied.

/// Field numbers the checkpoint writer frames by hand, mirrored from session.proto; the tests check them against the prost messages.
pub(crate) struct Tags {
    pub sections: [u32; 7], // Session field per section, 0 for one written framed: head, objects, tree, graph, xforms, definitions, interactions.
    pub root: u32,          // Tree.root
    pub children: u32,      // TreeNode.children
    pub lists: [u32; 13],   // Objects field per COLLECTIONS entry.
}

pub(crate) const TAGS: Tags = Tags {
    sections: [0, 3, 4, 5, 0, 8, 0],
    root: 3,
    children: 4,
    lists: [3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 16, 18],
};

/// A tree node being written.
struct Frame {
    node: Rc<RefCell<TreeNode>>, // The node.
    next: usize,                 // Its next raw child.
    chunks: Vec<Vec<u8>>,        // Its bytes so far.
}

impl Frame {
    /// Open a node with its head bytes.
    fn new(node: Rc<RefCell<TreeNode>>) -> Self {
        let head = node_head(&node.borrow());

        Self {
            node,
            next: 0,
            chunks: vec![head],
        }
    }
}

/// A resumable protobuf writer over a session: live entries only, the layout to_proto encodes.
struct Checkpoint {
    revision: u64,          // The revision it writes.
    phase: usize,           // The section being written.
    cursor: usize,          // Slot or entry count in the phase.
    key: String,            // The last key or guid written.
    stack: Vec<Frame>,      // Nodes being written, root first.
    tree: Vec<Vec<u8>>,     // The framed root, in chunks after the Tree head.
    sections: Vec<Vec<u8>>, // The seven Session fields.
    out: Vec<u8>,           // The joined message.
    hits: usize,            // Xforms entries order() reached, every one once the rest scan is done.
    rest: BTreeSet<String>, // Xforms guids outside order(), in guid order.
}

impl Checkpoint {
    /// Construct a writer at the first phase.
    fn new(revision: u64) -> Self {
        Self {
            revision,
            phase: HEAD,
            cursor: 0,
            key: String::new(),
            stack: Vec::new(),
            tree: Vec::new(),
            sections: vec![Vec::new(); 7],
            out: Vec::new(),
            hits: 0,
            rest: BTreeSet::new(),
        }
    }
}

/// The key and length of a length-delimited protobuf field.
fn prefix(tag: u32, length: usize) -> Vec<u8> {
    let mut bytes = Vec::new();
    prost::encoding::encode_key(tag, prost::encoding::WireType::LengthDelimited, &mut bytes);
    prost::encoding::encode_varint(length as u64, &mut bytes);

    bytes
}

/// Append bytes to a chunked buffer: a small piece is copied into the last chunk, a large one moves whole.
fn append(chunks: &mut Vec<Vec<u8>>, bytes: Vec<u8>) {
    match chunks.last_mut() {
        Some(last) if bytes.len() <= CHUNK && last.len() < CHUNK => last.extend(bytes),
        _ => chunks.push(bytes),
    }
}

/// Encode the live entries of a list from slot `start` for at most `work` slots, each framed under tag; returns the end slot and the slot count.
fn emit<E>(
    list: &Collection<E>,
    tag: u32,
    start: usize,
    work: usize,
    buffer: &mut Vec<u8>,
    entry: impl Fn(&E) -> Vec<u8>,
) -> (usize, usize) {
    let total = list.number_of_slots();
    let end = total.min(start.saturating_add(work));

    for slot in start..end {
        if list.is_dead(slot) {
            continue;
        }

        let bytes = entry(list.get_item(slot));
        buffer.extend(prefix(tag, bytes.len()));
        buffer.extend(bytes);
    }

    (end, total)
}

/// The lookup's pointer for an entry when it holds one of that type, else the entry itself.
fn truth<'a, T: FromGeometry>(lookup: &'a HashMap<String, Geometry>, item: &'a Rc<T>) -> &'a T
where
    Rc<T>: Keyed,
{
    lookup
        .get(item.key())
        .and_then(T::from_geometry)
        .unwrap_or(item.as_ref())
}

/// Append the live entries of the geometry list of that name from slot `start` for at most `work` slots, each as its lookup entry; returns the end slot and the slot count.
fn emit_geometry(
    objects: &Objects,
    lookup: &HashMap<String, Geometry>,
    name: &str,
    tag: u32,
    start: usize,
    work: usize,
    buffer: &mut Vec<u8>,
) -> (usize, usize) {
    use prost::Message;

    match name {
        "points" => emit(&objects.points, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "lines" => emit(&objects.lines, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "planes" => emit(&objects.planes, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "bboxes" => emit(&objects.bboxes, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "polylines" => emit(&objects.polylines, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "pointclouds" => emit(&objects.pointclouds, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "meshes" => emit(&objects.meshes, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "nurbscurves" => emit(&objects.nurbscurves, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "nurbssurfaces" => emit(&objects.nurbssurfaces, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        "breps" => emit(&objects.breps, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
        _ => emit(&objects.elements, tag, start, work, buffer, |item| {
            truth(lookup, item).to_proto().encode_to_vec()
        }),
    }
}

/// The name and guid fields of an Objects message.
fn objects_head(objects: &Objects) -> Vec<u8> {
    use prost::Message;

    let guid = if objects.has_guid() {
        objects.guid().to_string()
    } else {
        String::new()
    };

    crate::proto::Objects {
        name: objects.name.clone(),
        guid,
        ..Default::default()
    }
    .encode_to_vec()
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
    sweep: Vec<Weak<RefCell<TreeNode>>>, // Parents whose children died, for the purge to compact.
    #[serde(skip)]
    pinned: Vec<Weak<RefCell<TreeNode>>>, // Parents the purge left while a record pinned a child.
    #[serde(skip)]
    purging: Option<usize>, // The purge phase: 0..=12 objects lists, 13..=25 definitions lists, 26 the tree; None between cycles.
    #[serde(skip)]
    writer: Option<Checkpoint>, // The checkpoint being written, stale once revision moves.
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
            writer: None,
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
        out.points = baked(&objects.points, &world, Point::transform);
        out.lines = baked(&objects.lines, &world, Line::transform);
        out.planes = baked(&objects.planes, &world, Plane::transform);
        out.bboxes = baked(&objects.bboxes, &world, OBB::transform);
        out.polylines = baked(&objects.polylines, &world, Polyline::transform);
        out.pointclouds = baked(&objects.pointclouds, &world, PointCloud::transform);
        out.meshes = baked(&objects.meshes, &world, |mesh, xform| {
            mesh.transform(xform);
        });
        out.nurbscurves = baked(&objects.nurbscurves, &world, |curve, xform| {
            curve.transform(xform);
        });
        out.nurbssurfaces = baked(&objects.nurbssurfaces, &world, |surface, xform| {
            surface.transform(xform);
        });
        out.breps = baked(&objects.breps, &world, BRep::transform);
        out.elements = baked(&objects.elements, &world, Element::place);

        for instance in &objects.instances {
            let Some(definition) = self.definition_lookup.get(&instance.definition_guid) else {
                continue;
            };
            let placement = world
                .get(instance.guid())
                .cloned()
                .unwrap_or_else(Xform::identity);
            let resolved = resolve(instance, definition, &placement);
            push(&mut out, &Item::Geometry(resolved));
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
    /// Add a point; a guid already live adds nothing and returns the node that guid has.
    pub fn add_point(
        &mut self,
        point: Point,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("points", Geometry::Point(Rc::new(point)), "point", parent)
            .unwrap_or_else(|guid| self._node_of(&guid))
    }

    /// Add a line; a guid already live adds nothing and returns the node that guid has.
    pub fn add_line(
        &mut self,
        line: Line,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("lines", Geometry::Line(Rc::new(line)), "line", parent)
            .unwrap_or_else(|guid| self._node_of(&guid))
    }

    /// Add a plane; a guid already live adds nothing and returns the node that guid has.
    pub fn add_plane(
        &mut self,
        plane: Plane,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Rc<RefCell<TreeNode>> {
        self._add_object("planes", Geometry::Plane(Rc::new(plane)), "plane", parent)
            .unwrap_or_else(|guid| self._node_of(&guid))
    }

    /// Add a bounding box; a guid already live adds nothing and returns the node that guid has.
    pub fn add_obb(&mut self, bbox: OBB) -> Rc<RefCell<TreeNode>> {
        self._add_object("bboxes", Geometry::OBB(Rc::new(bbox)), "bbox", None)
            .unwrap_or_else(|guid| self._node_of(&guid))
    }

    /// Add a polyline; fewer than two points, or a guid already live, adds nothing and returns None.
    pub fn add_polyline(
        &mut self,
        polyline: Polyline,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if polyline.point_count() < 2 {
            return None;
        }

        self._add_object(
            "polylines",
            Geometry::Polyline(Rc::new(polyline)),
            "polyline",
            parent,
        )
        .ok()
    }

    /// Add a point cloud; no points, or a guid already live, adds nothing and returns None.
    pub fn add_pointcloud(
        &mut self,
        pointcloud: PointCloud,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if pointcloud.is_empty() {
            return None;
        }

        self._add_object(
            "pointclouds",
            Geometry::PointCloud(Rc::new(pointcloud)),
            "pointcloud",
            parent,
        )
        .ok()
    }

    /// Add a mesh; no faces, or a guid already live, adds nothing and returns None.
    pub fn add_mesh(
        &mut self,
        mesh: Mesh,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if mesh.is_empty() || mesh.number_of_faces() == 0 {
            return None;
        }

        self._add_object("meshes", Geometry::Mesh(Rc::new(mesh)), "mesh", parent)
            .ok()
    }

    /// Add a curve; fewer than two control vertices, or a guid already live, adds nothing and returns None.
    pub fn add_nurbscurve(
        &mut self,
        nurbscurve: NurbsCurve,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if nurbscurve.cv_count() < 2 {
            return None;
        }

        self._add_object(
            "nurbscurves",
            Geometry::NurbsCurve(Rc::new(nurbscurve)),
            "nurbscurve",
            parent,
        )
        .ok()
    }

    /// Add a surface; no control vertices, or a guid already live, adds nothing and returns None.
    pub fn add_nurbssurface(
        &mut self,
        nurbssurface: NurbsSurface,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if nurbssurface.cv_count_total() == 0 {
            return None;
        }

        self._add_object(
            "nurbssurfaces",
            Geometry::NurbsSurface(Rc::new(nurbssurface)),
            "nurbssurface",
            parent,
        )
        .ok()
    }

    /// Add a brep; no faces and no vertices, or a guid already live, adds nothing and returns None.
    pub fn add_brep(
        &mut self,
        brep: BRep,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        if brep.face_count() == 0 && brep.vertex_count() == 0 {
            return None;
        }

        self._add_object("breps", Geometry::BRep(Rc::new(brep)), "brep", parent)
            .ok()
    }

    /// Add an element, a data record kept even without geometry; a guid already live adds nothing and returns the node that guid has.
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
        .unwrap_or_else(|guid| self._node_of(&guid))
    }

    /// Add a custom component (any object with type_name/guid/name/extra); a guid already live adds nothing and returns the node that guid has.
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
        .unwrap_or_else(|guid| self._node_of(&guid))
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
        let slot = push(&mut self.definitions, &Item::Geometry(definition.clone()));
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

    /// Add an instance under parent, placed by xform relative to the parent with its own xform folded in; None when its definition_guid names no definition or its guid is already live.
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
        let node = self
            ._add_object(
                "instances",
                Item::InstanceRef(Rc::new(instance)),
                "instance",
                parent,
            )
            .ok()?;

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
            self.node_lookup.insert(name, Rc::clone(node));
        }

        self._record_add(node, ghost, was_dead);
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
        let Some(obj) = self._item(obj_guid) else {
            return false;
        };
        let tomb = self._tomb(obj_guid, &obj);
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

        let bytes = RECORD + weight(&obj) + 128 * degree;
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
            let entry = Entry::Object(self.get_node(guid));

            if self.history.current.is_some() {
                self.history.record(
                    Op::Replace(ReplaceOp::new(
                        guid.to_string(),
                        Item::Geometry(before),
                        Item::Geometry(obj.clone()),
                        entry.clone(),
                    )),
                    bytes,
                );
            }

            self._swap(guid, Item::Geometry(obj), &entry);

            return true;
        }

        let node = self.get_node(guid);
        let Some(removed) = self._half(false, old, guid) else {
            return false;
        };
        self._kill(&removed);
        let slot = push(&mut self.objects, &Item::Geometry(obj.clone()));
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
            let Some(tomb) = self._half(true, old, guid) else {
                return false;
            };
            let entry = Entry::Definition(tomb);

            if self.history.current.is_some() {
                self.history.record(
                    Op::Replace(ReplaceOp::new(
                        guid.to_string(),
                        Item::Geometry(before),
                        Item::Geometry(definition.clone()),
                        entry.clone(),
                    )),
                    bytes,
                );
            }

            self._swap(guid, Item::Geometry(definition), &entry);

            return true;
        }

        let Some(removed) = self._half(true, old, guid) else {
            return false;
        };
        self._kill(&removed);
        let slot = push(&mut self.definitions, &Item::Geometry(definition.clone()));
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
        let slot = push(&mut self.objects, &Item::InstanceRef(Rc::clone(&instance)));
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
        let slot = push(&mut self.objects, &Item::Geometry(copy.clone()));
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
            let node = self.get_node(guid);
            self.history.record(
                Op::Xform(XformOp::new(
                    guid.to_string(),
                    before,
                    Some(xform.clone()),
                    node,
                )),
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
            let node = self.get_node(guid);
            self.history.record(
                Op::Xform(XformOp::new(guid.to_string(), Some(before), None, node)),
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
    // Purge
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the dead slots not yet purged, over the objects and the definitions lists.
    pub fn number_of_dead(&self) -> usize {
        let mut count = 0;

        for (collection, _) in COLLECTIONS {
            count += list(&self.objects, collection).map_or(0, Slots::number_of_dead);
            count += list(&self.definitions, collection).map_or(0, Slots::number_of_dead);
        }

        count
    }

    /// Return whether dropped records left dead entries or swept parents a purge cycle can free.
    pub fn purge_due(&self) -> bool {
        self.history.dropped > 0 && (self.number_of_dead() > 0 || !self.sweep.is_empty())
    }

    /// Return whether a purge cycle is part way.
    pub fn is_purging(&self) -> bool {
        self.purging.is_some()
    }

    /// Purge what no record reaches for at most `work` slots or children, resuming the running cycle; true while it is unfinished.
    pub fn purge_step(&mut self, work: usize) -> bool {
        let fresh = self
            .writer
            .as_ref()
            .is_some_and(|writer| writer.revision == self.revision);

        if fresh || (self.purging.is_none() && !self.purge_due()) {
            return false;
        }

        self._purge(work);

        self.purging.is_some()
    }

    /// Drop the history, then purge everything in one call: a whole cycle, every live tree node, dense graph indices no record could revive into; O(n + N + V log V + E log E).
    pub fn purge(&mut self) {
        self.history.clear();
        self.writer = None;

        if self.purging.is_some() {
            self._purge(usize::MAX);
        }

        self._purge(usize::MAX);

        for node in self.tree.nodes() {
            node.borrow_mut().compact();
        }

        self.graph.renumber();
        self.revision += 1;
    }

    /// Write the live session as protobuf bytes for at most `work` units, purging first when due; Some once done, history kept, restarted by any edit.
    pub fn checkpoint(&mut self, mut work: usize) -> Option<Vec<u8>> {
        if self
            .writer
            .as_ref()
            .is_some_and(|writer| writer.revision != self.revision)
        {
            self.writer = None;
        }

        if self.writer.is_none() && (self.purging.is_some() || self.purge_due()) {
            work = self._purge(work);
        }

        if self.purging.is_some() || work == 0 {
            return None;
        }

        let mut writer = self
            .writer
            .take()
            .unwrap_or_else(|| Checkpoint::new(self.revision));

        if self._write(&mut writer, work) {
            return Some(writer.out);
        }

        self.writer = Some(writer);

        None
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
    /// Serialize to a JSON object.
    fn to_json_value(&self) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
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
            "graph": self.graph.to_json_value()?,
            "interactions": interactions_json,
            "xforms": xforms_json
        });

        if !self.definition_lookup.is_empty() {
            json_obj["definitions"] =
                serde_json::to_value(synced(&self.definitions, &self.definition_lookup))?;
        }

        Ok(json_obj)
    }

    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::file_json_dumps(&self.to_json_value()?, false)
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

    /// Serialize to a JSON string; saving purges, which clears the history, so no undo crosses a save.
    pub fn file_json_dumps(&mut self) -> String {
        self.purge();

        self.jsondump().expect("Failed to serialize Session JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Session JSON")
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&mut self, filename: &str) {
        self.purge();

        let json_obj = self
            .to_json_value()
            .expect("Failed to serialize Session JSON");

        crate::file_encoders::file_json_dump(&json_obj, filename, true)
            .expect("Failed to write JSON file");
    }

    /// Read from a JSON file.
    pub fn file_json_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&fs::read_to_string(filename)?)
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

    /// Serialize to protobuf bytes; saving purges, which clears the history, so no undo crosses a save.
    pub fn pb_dumps(&mut self) -> Vec<u8> {
        use prost::Message;

        self.purge();

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
    pub fn pb_load(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&fs::read(filename)?)
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

    /// Store an object in its list, lookup, graph and tree, recording an add when a transaction is open; Err carries a guid that is already live, as an object or a definition, which adds nothing.
    fn _add_object(
        &mut self,
        collection: &str,
        obj: impl Into<Item>,
        type_prefix: &str,
        parent: Option<&Rc<RefCell<TreeNode>>>,
    ) -> Result<Rc<RefCell<TreeNode>>, String> {
        let obj: Item = obj.into();
        let guid = obj.guid().to_string();

        if self._is_live(&guid) || self.definition_lookup.contains_key(&guid) {
            return Err(guid);
        }

        let slot = push(&mut self.objects, &obj);
        let attribute = format!("{type_prefix}_{}", obj.name());
        self._hold(&guid, obj);
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

        Ok(node)
    }

    /// The node of a live guid, a detached one named guid when the object is outside the tree.
    fn _node_of(&self, guid: &str) -> Rc<RefCell<TreeNode>> {
        self.get_node(guid).unwrap_or_else(|| TreeNode::new(guid))
    }

    /// Whether the live entry under guid, if any, is another entry than the one in a slot of the list of that name: one on the other side of the object/definition divide, of another type, or of the same type at another slot.
    fn _twin(&self, definition: bool, collection: &str, slot: usize, guid: &str) -> bool {
        let (objects, held, other) = if definition {
            let held = self
                .definition_lookup
                .get(guid)
                .cloned()
                .map(Item::Geometry);

            (&self.definitions, held, self._is_live(guid))
        } else {
            let other = self.definition_lookup.contains_key(guid);

            (&self.objects, self._item(guid), other)
        };

        if other {
            return true;
        }

        let Some(held) = held else {
            return false;
        };

        collection_for(&held).0 != collection || slot_of(objects, collection, guid) != Some(slot)
    }

    /// Whether the entry under guid is the one a record was taken on: any entry when no node was recorded, else the live entry whose node it is.
    fn _owns(&self, guid: &str, node: Option<&Rc<RefCell<TreeNode>>>) -> bool {
        node.is_none_or(|node| {
            self.get_node(guid)
                .is_some_and(|live| Rc::ptr_eq(&live, node))
        })
    }

    /// Whether guid names a live object, component or instance.
    fn _is_live(&self, guid: &str) -> bool {
        self.lookup.contains_key(guid)
            || self.component_lookup.contains_key(guid)
            || self.instance_lookup.contains_key(guid)
    }

    /// Take the object, component or instance under guid out of its map, the pointer itself.
    fn _take(&mut self, guid: &str) -> Option<Item> {
        if let Some(geometry) = self.lookup.remove(guid) {
            return Some(Item::Geometry(geometry));
        }

        if let Some(component) = self.component_lookup.remove(guid) {
            return Some(Item::Component(component));
        }

        Some(Item::InstanceRef(self.instance_lookup.remove(guid)?))
    }

    /// Put an object, component or instance in its map under guid.
    fn _hold(&mut self, guid: &str, item: Item) {
        match item {
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

    /// The object tomb of a live guid holding item, reused while a record still holds it, else made and pinned on its slot and node; O(1).
    fn _tomb(&mut self, guid: &str, item: &Item) -> Rc<Tomb> {
        let (collection, _) = collection_for(item);
        let slot = match slot_of(&self.objects, collection, guid) {
            Some(slot) => slot,
            None => push(&mut self.objects, item),
        };

        if let Some(tomb) = tomb_at(&self.objects, collection, slot) {
            if tomb.node.is_some() {
                return tomb;
            }
        }

        let node = match self.get_node(guid) {
            Some(node) => {
                self.node_lookup.insert(guid.to_string(), Rc::clone(&node));
                node
            }

            None => TreeNode::new(guid),
        };
        let tomb = Tomb::new(collection, false, slot, Some(Rc::clone(&node)));
        pin(&mut self.objects, collection, slot, &tomb);
        node.borrow_mut().set_tomb(&tomb);

        tomb
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

    /// Record a tree add of node, which left ghost at its old parent or revived when was_dead; outside a transaction it only counts a dropped move.
    fn _record_add(
        &mut self,
        node: &Rc<RefCell<TreeNode>>,
        ghost: Option<Rc<RefCell<TreeNode>>>,
        was_dead: bool,
    ) {
        if self.history.current.is_none() {
            self.history.dropped += usize::from(ghost.is_some());

            return;
        }

        let name = node.borrow().name.clone();
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
        let slot = slot_of(objects, collection, guid).unwrap_or_else(|| push(objects, &item));

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

    /// Run the purge cycle for at most `work` units, starting one when idle; returns the work left.
    fn _purge(&mut self, mut work: usize) -> usize {
        if self.purging.is_none() {
            self.purging = Some(0);
            self.history.dropped = 0;
        }

        while work > 0 {
            let Some(phase) = self.purging else {
                break;
            };

            if phase < 26 {
                work = self._purge_list(phase, work);
                continue;
            }

            let Some(weak) = self.sweep.last() else {
                self.sweep = std::mem::take(&mut self.pinned);
                self.purging = None;
                break;
            };
            let Some(parent) = weak.upgrade() else {
                self.sweep.pop();
                work -= 1;
                continue;
            };
            let spent = parent.borrow_mut().compact_step(work);
            work -= spent.clamp(1, work);

            if parent.borrow().is_compacting() {
                continue;
            }

            self.sweep.pop();

            if parent.borrow().has_dead() {
                self.pinned.push(Rc::downgrade(&parent));
            } else {
                parent.borrow_mut().set_queued(false);
            }
        }

        work
    }

    /// Compact the list of a purge phase (objects below 13, definitions from 13) for at most `work` slots, stepping to the next phase once it is done; returns the work left.
    fn _purge_list(&mut self, phase: usize, work: usize) -> usize {
        let objects = if phase < 13 {
            &mut self.objects
        } else {
            &mut self.definitions
        };
        let mut spent = 0;
        let mut done = true;

        if let Some(list) = list_mut(objects, COLLECTIONS[phase % 13].0) {
            if list.number_of_dead() > 0 || list.is_compacting() {
                spent = list.compact_step(work);
                done = !list.is_compacting();
            }
        }

        if done {
            self.purging = Some(phase + 1);
        }

        work - spent.min(work)
    }

    /// Advance a checkpoint writer for at most `work` units; true once its message is complete.
    fn _write(&self, writer: &mut Checkpoint, mut work: usize) -> bool {
        use prost::Message;

        while work > 0 {
            let spent = match writer.phase {
                HEAD => {
                    writer.sections[0] = crate::proto::Session {
                        name: self.name.clone(),
                        guid: self.guid.get().cloned().unwrap_or_default(),
                        ..Default::default()
                    }
                    .encode_to_vec();
                    writer.sections[1] = objects_head(&self.objects);
                    writer.phase = OBJECTS;

                    1
                }

                OBJECTS..TREE => self._write_list(writer, false, work),
                TREE => self._write_tree(writer, work),
                VERTICES => self._write_vertices(writer, work),
                EDGES => self._write_edges(writer, work),
                ORDERED..REST => self._write_ordered(writer, work),
                REST => self._write_rest(writer, work),
                DEFINITIONS..INTERACTIONS => self._write_list(writer, true, work),
                INTERACTIONS => self._write_interactions(writer, work),
                _ => return self._assemble(writer, work),
            };
            work -= spent.clamp(1, work);
        }

        false
    }

    /// Write the live entries of one objects or definitions list from the cursor slot; returns the slots examined.
    fn _write_list(&self, writer: &mut Checkpoint, definition: bool, work: usize) -> usize {
        use prost::Message;

        let (objects, lookup, first, section) = if definition {
            (&self.definitions, &self.definition_lookup, DEFINITIONS, 5)
        } else {
            (&self.objects, &self.lookup, OBJECTS, 1)
        };
        let phase = writer.phase - first;
        let tag = TAGS.lists[phase];
        let start = writer.cursor;
        let buffer = &mut writer.sections[section];
        let (end, total) = match COLLECTIONS[phase].0 {
            "components" => emit(&objects.components, tag, start, work, buffer, |item| {
                item.to_proto().encode_to_vec()
            }),
            "instances" => emit(&objects.instances, tag, start, work, buffer, |item| {
                let held = self
                    .instance_lookup
                    .get(item.guid())
                    .filter(|_| !definition);

                held.unwrap_or(item).to_proto().encode_to_vec()
            }),
            name => emit_geometry(objects, lookup, name, tag, start, work, buffer),
        };
        writer.cursor = end;

        if end >= total {
            writer.cursor = 0;
            writer.phase += 1;
        }

        end - start
    }

    /// Write the live tree depth first from an explicit stack, a finished node appended to its parent; returns the children examined.
    fn _write_tree(&self, writer: &mut Checkpoint, work: usize) -> usize {
        if writer.cursor == 0 {
            writer.cursor = 1;
            writer.sections[2] = self._tree_head();

            if let Some(root) = self.tree.root() {
                writer.stack.push(Frame::new(root));
            }
        }

        let mut spent = 0;

        while spent < work {
            let Some(frame) = writer.stack.last_mut() else {
                break;
            };
            let child = frame.node.borrow().get_child(frame.next);
            frame.next += 1;
            spent += 1;

            if let Some(child) = child {
                if !child.borrow().is_dead() {
                    writer.stack.push(Frame::new(child));
                }

                continue;
            }

            let Some(frame) = writer.stack.pop() else {
                break;
            };
            let mut chunks = frame.chunks;
            append(&mut chunks, node_tail(&frame.node.borrow()));
            let length = chunks.iter().map(Vec::len).sum();
            let (tag, parent) = match writer.stack.last_mut() {
                Some(frame) => (TAGS.children, &mut frame.chunks),
                None => (TAGS.root, &mut writer.tree),
            };
            append(parent, prefix(tag, length));

            for chunk in chunks {
                append(parent, chunk);
            }
        }

        if writer.stack.is_empty() {
            writer.cursor = 0;
            writer.phase = VERTICES;
        }

        spent
    }

    /// The guid and name fields of the Tree message.
    fn _tree_head(&self) -> Vec<u8> {
        use prost::Message;

        let guid = if self.tree.has_guid() {
            self.tree.guid().to_string()
        } else {
            String::new()
        };

        crate::proto::Tree {
            guid,
            name: self.tree.name.clone(),
            root: None,
        }
        .encode_to_vec()
    }

    /// Write the graph head, then the vertices in name order after the last key written, at most work per call; returns the vertices written.
    fn _write_vertices(&self, writer: &mut Checkpoint, work: usize) -> usize {
        use crate::graph::vertex_to_proto;
        use prost::Message;

        let after = (writer.cursor > 0).then_some(writer.key.as_str());
        let buffer = &mut writer.sections[3];
        let mut spent = 0;
        let mut last = None;

        if writer.cursor == 0 {
            let guid = if self.graph.has_guid() {
                self.graph.guid().to_string()
            } else {
                String::new()
            };
            *buffer = crate::proto::Graph {
                name: self.graph.name.clone(),
                guid,
                ..Default::default()
            }
            .encode_to_vec();
        }

        for (name, vertex) in self.graph.vertices_after(after).take(work) {
            crate::proto::Graph {
                vertices: BTreeMap::from([(name.clone(), vertex_to_proto(vertex))]),
                ..Default::default()
            }
            .encode_raw(buffer);
            last = Some(name.clone());
            spent += 1;
        }

        if let Some(last) = last {
            writer.key = last;
            writer.cursor += 1;

            if spent >= work {
                return spent;
            }
        }

        writer.key.clear();
        writer.cursor = 0;
        writer.phase = EDGES;

        spent
    }

    /// Write the graph edges in vertex order after the last key written, at most work entries per call, then the counts and defaults; returns the entries examined.
    fn _write_edges(&self, writer: &mut Checkpoint, work: usize) -> usize {
        use crate::graph::edge_to_proto;
        use prost::Message;

        let start = if writer.cursor > 0 {
            std::ops::Bound::Excluded(writer.key.as_str())
        } else {
            std::ops::Bound::Unbounded
        };
        let buffer = &mut writer.sections[3];
        let mut spent = 0;
        let mut last = None;

        for (u, neighbors) in self
            .graph
            .edges
            .range::<str, _>((start, std::ops::Bound::Unbounded))
        {
            if spent >= work {
                break;
            }

            for (v, edge) in neighbors {
                if u <= v {
                    crate::proto::Graph {
                        edges: vec![edge_to_proto(edge)],
                        ..Default::default()
                    }
                    .encode_raw(buffer);
                }
            }

            last = Some(u.clone());
            spent += neighbors.len().max(1);
        }

        if let Some(last) = last {
            writer.key = last;
            writer.cursor += 1;

            if spent >= work {
                return spent;
            }
        }

        crate::proto::Graph {
            vertex_count: self.graph.vertex_count,
            edge_count: self.graph.edge_count,
            default_vertex_attributes: self.graph.default_vertex_attributes.clone(),
            default_edge_attributes: self.graph.default_edge_attributes.clone(),
            ..Default::default()
        }
        .encode_raw(buffer);
        writer.key.clear();
        writer.cursor = 0;
        writer.phase = ORDERED;

        spent
    }

    /// Write the non-identity xforms of the live objects of one order() list; returns the slots examined.
    fn _write_ordered(&self, writer: &mut Checkpoint, work: usize) -> usize {
        let start = writer.cursor;
        let mut end = start;
        let mut total = 0;

        if let Some(list) = list(&self.objects, COLLECTIONS[writer.phase - ORDERED].0) {
            total = list.number_of_slots();
            end = total.min(start.saturating_add(work));

            for slot in start..end {
                if list.is_dead(slot) {
                    continue;
                }

                let guid = list.key_at(slot);
                let Some(xform) = self.xforms.get(guid) else {
                    continue;
                };
                writer.hits += 1;

                if !xform.is_identity() {
                    self._write_xform(writer, guid.to_string(), xform);
                }
            }
        }

        writer.cursor = end;

        if end >= total {
            writer.cursor = 0;
            writer.phase += 1;
        }

        end - start
    }

    /// Write the non-identity xforms of guids outside order() in guid order, after a scan of xforms in slices of `work` that runs only when order() missed some; returns the entries examined or written.
    fn _write_rest(&self, writer: &mut Checkpoint, work: usize) -> usize {
        if writer.hits < self.xforms.len() {
            return self._scan_rest(writer, work);
        }

        let start = if writer.cursor > 0 {
            std::ops::Bound::Excluded(writer.key.as_str())
        } else {
            std::ops::Bound::Unbounded
        };
        let rest = std::mem::take(&mut writer.rest);
        let mut spent = 0;
        let mut last = None;

        for guid in rest
            .range::<str, _>((start, std::ops::Bound::Unbounded))
            .take(work)
        {
            self._write_xform(writer, guid.clone(), &self.xforms[guid]);
            last = Some(guid.clone());
            spent += 1;
        }

        writer.rest = rest;

        if let (Some(last), true) = (last, spent >= work) {
            writer.key = last;
            writer.cursor += 1;

            return spent;
        }

        writer.cursor = 0;
        writer.phase = if self.definition_lookup.is_empty() {
            INTERACTIONS
        } else {
            writer.sections[5] = objects_head(&self.definitions);
            DEFINITIONS
        };

        spent
    }

    /// Scan a slice of at most `work` xforms for non-identity ones whose guid order() misses, into the rest set; returns the entries scanned.
    fn _scan_rest(&self, writer: &mut Checkpoint, work: usize) -> usize {
        let start = writer.cursor;
        let end = self.xforms.len().min(start.saturating_add(work));

        for (guid, xform) in self.xforms.iter().skip(start).take(end - start) {
            let ordered = self.lookup.get(guid).is_some_and(|geometry| {
                slot_of(&self.objects, collection_of(geometry).0, guid).is_some()
            });

            if !ordered && !xform.is_identity() {
                writer.rest.insert(guid.clone());
            }
        }

        writer.cursor = end;

        if end >= self.xforms.len() {
            writer.hits = self.xforms.len();
            writer.cursor = 0;
        }

        end - start
    }

    /// Append one XformEntry to the xforms section.
    fn _write_xform(&self, writer: &mut Checkpoint, guid: String, xform: &Xform) {
        use prost::Message;

        crate::proto::Session {
            xforms: vec![crate::proto::XformEntry {
                guid,
                xform: Some(xform.to_proto()),
            }],
            ..Default::default()
        }
        .encode_raw(&mut writer.sections[4]);
    }

    /// Write the interactions per edge guid, resuming after the last guid written; returns the entries written.
    fn _write_interactions(&self, writer: &mut Checkpoint, work: usize) -> usize {
        use prost::Message;

        let start = if writer.cursor > 0 {
            std::ops::Bound::Excluded(writer.key.as_str())
        } else {
            std::ops::Bound::Unbounded
        };
        let mut spent = 0;
        let mut last = None;

        for (guid, list) in self
            .interactions
            .range::<str, _>((start, std::ops::Bound::Unbounded))
            .take(work)
        {
            crate::proto::Session {
                interactions: vec![crate::proto::InteractionEntry {
                    guid: guid.clone(),
                    interactions: list.iter().map(|item| item.to_proto()).collect(),
                }],
                ..Default::default()
            }
            .encode_raw(&mut writer.sections[6]);
            last = Some(guid.clone());
            spent += 1;
        }

        if let (Some(last), true) = (last, spent >= work) {
            writer.key = last;
            writer.cursor += 1;

            return spent;
        }

        writer.phase = ASSEMBLY;
        writer.cursor = 0;

        spent
    }

    /// Join the sections into one Session message, copying at most `work` KiB; true once complete.
    fn _assemble(&self, writer: &mut Checkpoint, work: usize) -> bool {
        if writer.phase == ASSEMBLY {
            let mut pieces = Vec::new();

            for (i, body) in std::mem::take(&mut writer.sections).into_iter().enumerate() {
                let tag = TAGS.sections[i];

                if i == 5 && self.definition_lookup.is_empty() {
                    continue;
                }

                let tree = if i == 2 {
                    std::mem::take(&mut writer.tree)
                } else {
                    Vec::new()
                };

                if tag > 0 {
                    let length = body.len() + tree.iter().map(Vec::len).sum::<usize>();
                    pieces.push(prefix(tag, length));
                }

                pieces.push(body);
                pieces.extend(tree);
            }
            writer.out.reserve_exact(pieces.iter().map(Vec::len).sum());
            writer.sections = pieces;
            writer.phase += 1;
        }

        let mut budget = work.saturating_mul(1024);
        let mut skip = writer.out.len();

        for piece in &writer.sections {
            if skip >= piece.len() {
                skip -= piece.len();
                continue;
            }

            let end = piece.len().min(skip.saturating_add(budget));
            writer.out.extend_from_slice(&piece[skip..end]);
            budget -= end - skip;
            skip = 0;

            if budget == 0 {
                break;
            }
        }

        writer.out.len() == writer.sections.iter().map(Vec::len).sum::<usize>()
    }

    /// Flip a tomb dead: its slot and map entry, and for an object tomb its node, transform, vertex, edges and interactions; a guid a live twin owns keeps those with the twin; O(1 + d log V).
    pub(crate) fn _kill(&mut self, tomb: &Rc<Tomb>) {
        if tomb.collection.is_empty() {
            return;
        }

        let slot = tomb.slot.get();
        let collection = tomb.collection.as_str();
        self.revision += 1;
        self.bvh_cache_dirty = true;

        if tomb.definition {
            self._kill_definition(tomb);

            return;
        }

        let Some(stored) = item_at(&self.objects, collection, slot) else {
            return;
        };
        let guid = stored.guid();
        let owner = !self._twin(false, collection, slot, guid)
            && (self._is_live(guid) || slot_of(&self.objects, collection, guid) == Some(slot));
        let held = self._take(guid);

        if let Some(held) = held {
            if owner && !same(&held, &stored) {
                store(&mut self.objects, collection, slot, &held);
            }

            if !owner {
                self._hold(guid, held);
            }
        }

        flag(&mut self.objects, collection, slot, true);

        let Some(node) = &tomb.node else {
            return;
        };
        let parent = node.borrow().parent();
        node.borrow_mut().set_dead(true);
        node.borrow_mut().set_tomb(tomb);

        if let Some((key, held)) = self.node_lookup.remove_entry(guid) {
            if !Rc::ptr_eq(&held, node) {
                self.node_lookup.insert(key, held);
            }
        }

        if let Some(parent) = parent {
            self._queue(&parent);
        }

        if owner {
            self._park(tomb, guid);
        }
    }

    /// Flip a tomb live again: the same slot and pointer, and for an object tomb the same node, transform, vertex, edges and interactions; a guid a live twin owns stays dead; O(1 + d log V).
    pub(crate) fn _revive(&mut self, tomb: &Rc<Tomb>) {
        if tomb.collection.is_empty() {
            return;
        }

        let slot = tomb.slot.get();
        let collection = tomb.collection.as_str();
        self.revision += 1;
        self.bvh_cache_dirty = true;

        if tomb.definition {
            self._revive_definition(tomb);

            return;
        }

        let Some(item) = item_at(&self.objects, collection, slot) else {
            return;
        };
        let guid = item.guid().to_string();

        if self._twin(false, collection, slot, &guid) {
            return;
        }

        flag(&mut self.objects, collection, slot, false);
        self._hold(&guid, item.clone());

        let Some(node) = &tomb.node else {
            self._label(&guid, &format!("{}_{}", prefix_of(collection), item.name()));

            return;
        };
        node.borrow_mut().set_dead(false);

        if node.borrow().parent().is_some() {
            self.node_lookup.insert(guid.clone(), Rc::clone(node));
        }

        self._unpark(tomb, &guid);
    }

    /// Flip a definition tomb dead: its slot, and its map entry when it owns the guid; O(1).
    fn _kill_definition(&mut self, tomb: &Rc<Tomb>) {
        let slot = tomb.slot.get();
        let collection = tomb.collection.as_str();
        let Some(stored) = item_at(&self.definitions, collection, slot) else {
            return;
        };
        let guid = stored.guid().to_string();
        let owner = !self._twin(true, collection, slot, &guid);

        if let Some(held) = self.definition_lookup.get(&guid) {
            let held = Item::Geometry(held.clone());

            if owner && !same(&held, &stored) {
                store(&mut self.definitions, collection, slot, &held);
            }
        }

        flag(&mut self.definitions, collection, slot, true);

        if owner {
            self.definition_lookup.remove(&guid);
        }
    }

    /// Flip a definition tomb live again, unless a live twin owns its guid; O(1).
    fn _revive_definition(&mut self, tomb: &Rc<Tomb>) {
        let slot = tomb.slot.get();
        let collection = tomb.collection.as_str();
        let Some(Item::Geometry(geometry)) = item_at(&self.definitions, collection, slot) else {
            return;
        };
        let guid = geometry.guid().to_string();

        if self._twin(true, collection, slot, &guid) {
            return;
        }

        flag(&mut self.definitions, collection, slot, false);
        self.definition_lookup.insert(guid, geometry);
    }

    /// Move the transform, graph vertex, incident edges and their interactions of guid into its tomb; O(d log V).
    fn _park(&mut self, tomb: &Rc<Tomb>, guid: &str) {
        *tomb.xform.borrow_mut() = self.xforms.remove(guid);

        let Some((vertex, edges)) = self.graph.take_node(guid) else {
            return;
        };

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

    /// Move a tomb's transform, vertex and edges back, with the interactions of every edge that returns; O(d log V).
    fn _unpark(&mut self, tomb: &Rc<Tomb>, guid: &str) {
        if let Some(xform) = tomb.xform.borrow_mut().take() {
            self.xforms.insert(guid.to_string(), xform);
        }

        let Some(vertex) = tomb.vertex.borrow_mut().take() else {
            return;
        };
        let edges = std::mem::take(&mut *tomb.edges.borrow_mut());
        let mut ids: Vec<(String, String)> = Vec::with_capacity(edges.len());

        for edge in &edges {
            if edge.has_guid() {
                ids.push((edge.other_vertex(guid), edge.guid().to_string()));
            }
        }

        self.graph.put_node(vertex, edges);

        for (other, id) in ids {
            let Some(neighbours) = self.graph.edges.get(guid) else {
                continue;
            };
            let Some(back) = neighbours.get(&other) else {
                continue;
            };

            if !back.has_guid() || back.guid() != id {
                continue;
            }

            if let Some(list) = tomb.interactions.borrow_mut().remove(&id) {
                self.interactions.insert(id, list);
            }
        }
    }

    /// Store obj under guid in the slot and map of the recorded entry, relabelling an object's vertex; a guid now live on the other side, or on the same side as another entry, is left alone; O(1).
    pub(crate) fn _swap(&mut self, guid: &str, obj: Item, entry: &Entry) {
        let (collection, prefix) = collection_for(&obj);

        match entry {
            Entry::Definition(tomb) => {
                let Item::Geometry(geometry) = &obj else {
                    return;
                };
                let slot = tomb.slot.get();

                if self._is_live(guid) || slot_of(&self.definitions, collection, guid) != Some(slot)
                {
                    return;
                }

                store(&mut self.definitions, collection, slot, &obj);
                self.definition_lookup
                    .insert(guid.to_string(), geometry.clone());
            }

            Entry::Object(node) => {
                if self.definition_lookup.contains_key(guid) || !self._owns(guid, node.as_ref()) {
                    return;
                }

                let Some(slot) = slot_of(&self.objects, collection, guid) else {
                    return;
                };
                store(&mut self.objects, collection, slot, &obj);
                self._label(guid, &format!("{prefix}_{}", obj.name()));
                self._hold(guid, obj);
            }
        }

        self.revision += 1;
        self.bvh_cache_dirty = true;
    }

    /// Rebuild every index from the tables in O(n + N): the maps win over the slots, map-only and slot-only entries are adopted, a non-identity instance xform folds into xforms, node_lookup is refilled from the live tree.
    pub fn reindex(&mut self) {
        repoint(&mut self.objects, &self.lookup);
        adopt(&mut self.objects, &mut self.lookup);
        repoint(&mut self.definitions, &self.definition_lookup);
        adopt(&mut self.definitions, &mut self.definition_lookup);
        self._reindex_components();
        self._reindex_instances();
        self.node_lookup.clear();

        for node in self.tree.nodes() {
            let guid = node.borrow().name.clone();

            if self._is_live(&guid) && !self.node_lookup.contains_key(&guid) {
                self.node_lookup.insert(guid, node);
            }
        }

        self.indexed = self.tree.root().map(|root| Rc::downgrade(&root));
    }

    /// Adopt slot-only components into component_lookup and map-only ones into the slots, sorted by guid; the map wins a shared guid.
    fn _reindex_components(&mut self) {
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
    }

    /// Adopt map-only instances into the slots, sorted by guid, the map winning a shared guid; a non-identity instance xform folds into xforms.
    fn _reindex_instances(&mut self) {
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

    /// Set or drops (None) the local transform under guid, unrecorded; a guid whose entry is not the recorded node is left alone.
    pub(crate) fn _place(
        &mut self,
        guid: &str,
        xform: Option<&Xform>,
        node: Option<&Rc<RefCell<TreeNode>>>,
    ) {
        if !self._owns(guid, node) {
            return;
        }

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
    fn _xforms_ordered(&self) -> Vec<(String, &Xform)> {
        let mut ordered: Vec<(String, &Xform)> = Vec::new();
        let mut rest: BTreeMap<String, &Xform> = BTreeMap::new();

        for (obj_guid, obj_xform) in &self.xforms {
            if !obj_xform.is_identity() {
                rest.insert(obj_guid.clone(), obj_xform);
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
