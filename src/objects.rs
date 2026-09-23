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
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use std::fs;
use std::rc::Rc;

/// A custom domain object stored generically in a Session; every field except type/guid/name lives in extra.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Component {
    #[serde(rename = "type")]
    pub type_name: String, // Class name, e.g. "FloorBuilder".
    pub guid: String, // Guid.
    pub name: String, // Human-readable name.
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>, // All custom fields.
}

impl Component {
    /// Returns the guid.
    pub fn guid(&self) -> &str {
        &self.guid
    }

    /// Serializes to a JSON string.
    pub fn jsondump(&self) -> Result<serde_json::Value, Box<dyn std::error::Error>> {
        Ok(serde_json::to_value(self)?)
    }

    /// Deserializes from a JSON string.
    pub fn jsonload(data: &serde_json::Value) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_value(data.clone())?)
    }

    /// Serializes to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        let proto = crate::proto::Component {
            type_name: self.type_name.clone(),
            guid: self.guid.clone(),
            name: self.name.clone(),
            json_data: serde_json::to_string(&self.extra).unwrap_or_default(),
        };

        proto.encode_to_vec()
    }

    /// Deserializes from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        let proto = crate::proto::Component::decode(data)?;
        let extra: HashMap<String, serde_json::Value> =
            serde_json::from_str(&proto.json_data).unwrap_or_default();

        Ok(Component {
            type_name: proto.type_name,
            guid: proto.guid,
            name: proto.name,
            extra,
        })
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
    guid: std::sync::OnceLock<String>, // Guid.
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
    /// Constructs an empty collection with every list allocated.
    fn default() -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
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
    /// Constructs an empty collection.
    pub fn new() -> Self {
        Self::default()
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
    pub fn set_guid(&self, guid: String) {
        let _ = self.guid.set(guid);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serializes to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserializes from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serializes to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserializes from a JSON string.
    pub fn file_json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    /// Writes to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) {
        fs::write(filepath, self.file_json_dumps()).expect("Failed to write JSON file");
    }

    /// Reads from a JSON file.
    pub fn file_json_load(filepath: &str) -> Self {
        let json = fs::read_to_string(filepath).expect("Failed to read JSON file");

        Self::file_json_loads(&json)
    }

    /// Serializes to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        let mut proto = crate::proto::Objects {
            name: self.name.clone(),
            guid: self.guid.get().cloned().unwrap_or_default(),
            ..Default::default()
        };

        for p in &self.points {
            proto
                .points
                .push(crate::proto::Point::decode(p.pb_dumps().as_slice()).unwrap());
        }

        for l in &self.lines {
            proto
                .lines
                .push(crate::proto::Line::decode(l.pb_dumps().as_slice()).unwrap());
        }

        for pl in &self.planes {
            proto
                .planes
                .push(crate::proto::Plane::decode(pl.pb_dumps().as_slice()).unwrap());
        }

        for b in &self.bboxes {
            proto
                .bboxes
                .push(crate::proto::BoundingBox::decode(b.pb_dumps().as_slice()).unwrap());
        }

        for pl in &self.polylines {
            proto
                .polylines
                .push(crate::proto::Polyline::decode(pl.pb_dumps().as_slice()).unwrap());
        }

        for pc in &self.pointclouds {
            proto
                .pointclouds
                .push(crate::proto::PointCloud::decode(pc.pb_dumps().as_slice()).unwrap());
        }

        for m in &self.meshes {
            proto
                .meshes
                .push(crate::proto::Mesh::decode(m.pb_dumps().as_slice()).unwrap());
        }

        for nc in &self.nurbscurves {
            proto
                .nurbscurves
                .push(crate::proto::NurbsCurve::decode(nc.pb_dumps().as_slice()).unwrap());
        }

        for ns in &self.nurbssurfaces {
            proto
                .nurbssurfaces
                .push(crate::proto::NurbsSurface::decode(ns.pb_dumps().as_slice()).unwrap());
        }

        for b in &self.breps {
            proto
                .breps
                .push(crate::proto::BRep::decode(b.pb_dumps().as_slice()).unwrap());
        }

        for e in &self.elements {
            proto
                .elements
                .push(crate::proto::Element::decode(e.pb_dumps().as_slice()).unwrap());
        }

        for c in &self.components {
            proto
                .components
                .push(crate::proto::Component::decode(c.pb_dumps().as_slice()).unwrap());
        }

        for i in &self.instances {
            proto.instances.push(i.to_proto());
        }

        proto.encode_to_vec()
    }

    /// Deserializes from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        let proto = crate::proto::Objects::decode(data)?;
        let mut objects = Objects::new();

        if !proto.guid.is_empty() {
            objects.set_guid(proto.guid.clone());
        }

        objects.name = proto.name;

        for p in &proto.points {
            objects
                .points
                .push(Rc::new(Point::pb_loads(&p.encode_to_vec())?));
        }

        for l in &proto.lines {
            objects
                .lines
                .push(Rc::new(Line::pb_loads(&l.encode_to_vec())?));
        }

        for pl in &proto.planes {
            objects
                .planes
                .push(Rc::new(Plane::pb_loads(&pl.encode_to_vec())?));
        }

        for b in &proto.bboxes {
            objects
                .bboxes
                .push(Rc::new(OBB::pb_loads(&b.encode_to_vec())?));
        }

        for pl in &proto.polylines {
            objects
                .polylines
                .push(Rc::new(Polyline::pb_loads(&pl.encode_to_vec())?));
        }

        for pc in &proto.pointclouds {
            objects
                .pointclouds
                .push(Rc::new(PointCloud::pb_loads(&pc.encode_to_vec())?));
        }

        for m in &proto.meshes {
            objects
                .meshes
                .push(Rc::new(Mesh::pb_loads(&m.encode_to_vec())?));
        }

        for nc in &proto.nurbscurves {
            objects
                .nurbscurves
                .push(Rc::new(NurbsCurve::pb_loads(&nc.encode_to_vec())?));
        }

        for ns in &proto.nurbssurfaces {
            objects
                .nurbssurfaces
                .push(Rc::new(NurbsSurface::pb_loads(&ns.encode_to_vec())?));
        }

        for b in &proto.breps {
            objects
                .breps
                .push(Rc::new(BRep::pb_loads(&b.encode_to_vec())?));
        }

        for e in &proto.elements {
            objects
                .elements
                .push(Rc::new(Element::pb_loads(&e.encode_to_vec())?));
        }

        for c in &proto.components {
            objects
                .components
                .push(Component::pb_loads(&c.encode_to_vec())?);
        }

        for i in proto.instances {
            objects.instances.push(Rc::new(InstanceRef::from_proto(i)));
        }

        Ok(objects)
    }

    /// Writes to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) {
        fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Reads from a protobuf file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl fmt::Display for Objects {
    /// Writes the collection string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Objects(name={}, guid={}, points={})",
            self.name,
            self.guid(),
            self.points.len()
        )
    }
}
