use crate::brep::BRep;
use crate::element::Element;
use crate::instance_ref::InstanceRef;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
use crate::obb::OBB;
use crate::plane::Plane;
use crate::point::Point;
use crate::pointcloud::PointCloud;
use crate::polyline::Polyline;
use prost::Message;
use serde::Deserialize;
use serde::Serialize;
use std::collections::HashMap;
use std::fmt;
use std::rc::Rc;
use std::sync::OnceLock;

/// Convert every object of a list into a repeated proto field.
fn dump_pb_list<T, P>(list: &[Rc<T>], to_proto: fn(&T) -> P) -> Vec<P> {
    let mut out = Vec::with_capacity(list.len());

    for item in list {
        out.push(to_proto(item));
    }

    out
}

/// Load every message of a repeated proto field, keeping guids.
fn load_pb_list<T, P>(repeated: Vec<P>, from_proto: fn(P) -> T) -> Vec<Rc<T>> {
    let mut out = Vec::with_capacity(repeated.len());

    for item in repeated {
        out.push(Rc::new(from_proto(item)));
    }

    out
}

/// Load every message of a repeated proto field whose conversion can fail, keeping guids.
fn try_load_pb_list<T, P>(
    repeated: Vec<P>,
    from_proto: fn(P) -> Result<T, Box<dyn std::error::Error>>,
) -> Result<Vec<Rc<T>>, Box<dyn std::error::Error>> {
    let mut out = Vec::with_capacity(repeated.len());

    for item in repeated {
        out.push(Rc::new(from_proto(item)?));
    }

    Ok(out)
}

/// A custom domain object stored generically in a Session; every field except type/guid/name lives in extra.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Component {
    pub guid: String, // GUID.
    #[serde(rename = "type")]
    pub type_name: String, // Class name, e.g. "FloorBuilder".
    pub name: String, // Human-readable name.
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>, // All custom fields.
}

impl Default for Component {
    /// Construct an empty component.
    fn default() -> Self {
        Self {
            guid: uuid::Uuid::new_v4().to_string(),
            type_name: String::new(),
            name: "my_component".to_string(),
            extra: HashMap::new(),
        }
    }
}

impl Component {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty component.
    pub fn new() -> Self {
        Self::default()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the guid has been created.
    pub fn has_guid(&self) -> bool {
        !self.guid.is_empty()
    }

    /// Return the guid.
    pub fn guid(&self) -> &str {
        &self.guid
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON object.
    pub fn jsondump(&self) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
        Ok(serde_json::to_value(self)?)
    }

    /// Deserialize from a JSON object.
    pub fn jsonload(data: &serde_json::Value) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_value(data.clone())?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Component {
        crate::proto::Component {
            type_name: self.type_name.clone(),
            guid: self.guid.clone(),
            name: self.name.clone(),
            json_data: serde_json::to_string(&self.extra).unwrap_or_default(),
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(proto: crate::proto::Component) -> Self {
        let extra = serde_json::from_str(&proto.json_data).unwrap_or_default();

        Self {
            guid: proto.guid,
            type_name: proto.type_name,
            name: proto.name,
            extra,
        }
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self::from_proto(crate::proto::Component::decode(data)?))
    }
}

/// A collection of geometry objects.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Objects")]
pub struct Objects {
    #[serde(
        serialize_with = "crate::guid_serde::serialize",
        deserialize_with = "crate::guid_serde::deserialize"
    )]
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,                         // The name of the collection.
    pub points: Vec<Rc<Point>>,               // Points.
    pub lines: Vec<Rc<Line>>,                 // Lines.
    pub planes: Vec<Rc<Plane>>,               // Planes.
    pub bboxes: Vec<Rc<OBB>>,                 // Bounding boxes.
    pub polylines: Vec<Rc<Polyline>>,         // Polylines.
    pub pointclouds: Vec<Rc<PointCloud>>,     // Point clouds.
    pub meshes: Vec<Rc<Mesh>>,                // Meshes.
    pub nurbscurves: Vec<Rc<NurbsCurve>>,     // NURBS curves.
    pub nurbssurfaces: Vec<Rc<NurbsSurface>>, // NURBS surfaces.
    // SESSION_VIEWER
    #[serde(default)]
    pub nurbssurfacetrimmeds: Vec<Rc<NurbsSurfaceTrimmed>>,
    pub breps: Vec<Rc<BRep>>,       // BReps.
    pub elements: Vec<Rc<Element>>, // Elements.
    pub components: Vec<Component>, // Components.
    #[serde(default)]
    pub instances: Vec<Rc<InstanceRef>>, // Instances, each placing a definition of Session::definitions by guid.
}

impl Default for Objects {
    /// Construct an empty collection with every list allocated.
    fn default() -> Self {
        Self {
            guid: OnceLock::new(),
            name: "my_objects".to_string(),
            points: Vec::new(),
            lines: Vec::new(),
            planes: Vec::new(),
            bboxes: Vec::new(),
            polylines: Vec::new(),
            pointclouds: Vec::new(),
            meshes: Vec::new(),
            nurbscurves: Vec::new(),
            nurbssurfaces: Vec::new(),
            nurbssurfacetrimmeds: Vec::new(),
            breps: Vec::new(),
            elements: Vec::new(),
            components: Vec::new(),
            instances: Vec::new(),
        }
    }
}

impl Objects {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty collection with every list allocated.
    pub fn new() -> Self {
        Self::default()
    }

    /// Construct an empty named collection with every list allocated.
    pub fn with_name(name: &str) -> Self {
        Self {
            name: name.to_string(),
            ..Default::default()
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid if it has not already been created.
    pub fn set_guid(&self, guid: String) {
        let _ = self.guid.set(guid);
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

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().expect("Failed to serialize Objects JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Objects JSON")
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Objects {
        let mut components = Vec::with_capacity(self.components.len());

        for component in &self.components {
            components.push(component.to_proto());
        }

        crate::proto::Objects {
            name: self.name.clone(),
            guid: self.guid.get().cloned().unwrap_or_default(),
            points: dump_pb_list(&self.points, Point::to_proto),
            lines: dump_pb_list(&self.lines, Line::to_proto),
            planes: dump_pb_list(&self.planes, Plane::to_proto),
            bboxes: dump_pb_list(&self.bboxes, OBB::to_proto),
            polylines: dump_pb_list(&self.polylines, Polyline::to_proto),
            pointclouds: dump_pb_list(&self.pointclouds, PointCloud::to_proto),
            meshes: dump_pb_list(&self.meshes, Mesh::to_proto),
            nurbscurves: dump_pb_list(&self.nurbscurves, NurbsCurve::to_proto),
            nurbssurfaces: dump_pb_list(&self.nurbssurfaces, NurbsSurface::to_proto),
            breps: dump_pb_list(&self.breps, BRep::to_proto),
            elements: dump_pb_list(&self.elements, Element::to_proto),
            components,
            instances: dump_pb_list(&self.instances, InstanceRef::to_proto),
            ..Default::default()
        }
    }

    /// Construct from the protobuf message; elements load through the polymorphic registry.
    pub fn from_proto(proto: crate::proto::Objects) -> Result<Self, Box<dyn std::error::Error>> {
        let mut objects = Self::with_name(&proto.name);

        if !proto.guid.is_empty() {
            objects.set_guid(proto.guid);
        }

        objects.points = load_pb_list(proto.points, Point::from_proto);
        objects.lines = load_pb_list(proto.lines, Line::from_proto);
        objects.planes = load_pb_list(proto.planes, Plane::from_proto);
        objects.bboxes = try_load_pb_list(proto.bboxes, OBB::from_proto)?;
        objects.polylines = load_pb_list(proto.polylines, Polyline::from_proto);
        objects.pointclouds = load_pb_list(proto.pointclouds, PointCloud::from_proto);
        objects.meshes = load_pb_list(proto.meshes, Mesh::from_proto);
        objects.nurbscurves = load_pb_list(proto.nurbscurves, NurbsCurve::from_proto);
        objects.nurbssurfaces = try_load_pb_list(proto.nurbssurfaces, NurbsSurface::from_proto)?;
        objects.breps = try_load_pb_list(proto.breps, BRep::from_proto)?;

        for element in proto.elements {
            objects.elements.push(Rc::new(Element::pb_loads_polymorphic(
                &element.encode_to_vec(),
            )?));
        }

        for component in proto.components {
            objects.components.push(Component::from_proto(component));
        }

        objects.instances = load_pb_list(proto.instances, InstanceRef::from_proto);

        Ok(objects)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        Self::from_proto(crate::proto::Objects::decode(data)?)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        std::fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Objects(name=..., guid=..., points=...)".
    pub fn str(&self) -> String {
        format!(
            "Objects(name={}, guid={}, points={})",
            self.name,
            self.guid(),
            self.points.len()
        )
    }

    /// Return "Objects(name=..., guid=..., points=...)".
    pub fn repr(&self) -> String {
        self.str()
    }
}

impl fmt::Display for Objects {
    /// Write the collection string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}
