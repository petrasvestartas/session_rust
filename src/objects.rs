use crate::element::Element;
use crate::obb::OBB;
use crate::brep::BRep;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::plane::Plane;
use crate::point::Point;
use crate::pointcloud::PointCloud;
use crate::polyline::Polyline;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;

/// A collection of all geometry objects.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Objects")]
pub struct Objects {
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub points: Vec<Point>,
    pub lines: Vec<Line>,
    pub planes: Vec<Plane>,
    pub bboxes: Vec<OBB>,
    pub polylines: Vec<Polyline>,
    pub pointclouds: Vec<PointCloud>,
    pub meshes: Vec<Mesh>,
    pub nurbscurves: Vec<NurbsCurve>,
    pub nurbssurfaces: Vec<NurbsSurface>,
    pub breps: Vec<BRep>,
    pub elements: Vec<Element>,
}

impl Default for Objects {
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
            breps: Vec::new(),
            elements: Vec::new(),
        }
    }
}

impl Objects {
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new() -> Self {
        Self {
            name: "my_objects".to_string(),
            ..Default::default()
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Objects to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::encoders::sorted_json_string(self)
    }

    /// Deserializes Objects from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
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
        let proto = crate::proto::Objects {
            name: self.name.clone(),
            guid: self.guid().to_string(),
            points: self.points.iter().map(|p| {
                crate::proto::Point::decode(p.pb_dumps().as_slice()).unwrap()
            }).collect(),
            lines: self.lines.iter().map(|l| {
                crate::proto::Line::decode(l.pb_dumps().as_slice()).unwrap()
            }).collect(),
            planes: self.planes.iter().map(|p| {
                crate::proto::Plane::decode(p.pb_dumps().as_slice()).unwrap()
            }).collect(),
            bboxes: self.bboxes.iter().map(|b| {
                crate::proto::BoundingBox::decode(b.pb_dumps().as_slice()).unwrap()
            }).collect(),
            polylines: self.polylines.iter().map(|p| {
                crate::proto::Polyline::decode(p.pb_dumps().as_slice()).unwrap()
            }).collect(),
            pointclouds: self.pointclouds.iter().map(|p| {
                crate::proto::PointCloud::decode(p.pb_dumps().as_slice()).unwrap()
            }).collect(),
            meshes: self.meshes.iter().map(|m| {
                crate::proto::Mesh::decode(m.pb_dumps().as_slice()).unwrap()
            }).collect(),
            nurbscurves: self.nurbscurves.iter().map(|nc| {
                crate::proto::NurbsCurve::decode(nc.pb_dumps().as_slice()).unwrap()
            }).collect(),
            nurbssurfaces: self.nurbssurfaces.iter().map(|ns| {
                crate::proto::NurbsSurface::decode(ns.pb_dumps().as_slice()).unwrap()
            }).collect(),
            breps: self.breps.iter().map(|b| {
                crate::proto::BRep::decode(b.pb_dumps().as_slice()).unwrap()
            }).collect(),
            elements: self.elements.iter().map(|e| {
                crate::proto::Element::decode(e.pb_dumps().as_slice()).unwrap()
            }).collect(),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Objects::decode(data)?;
        let mut objects = Objects::new();
        objects.set_guid(proto.guid.clone());
        objects.name = proto.name;
        for p in &proto.points {
            objects.points.push(crate::point::Point::pb_loads(&p.encode_to_vec())?);
        }
        for l in &proto.lines {
            objects.lines.push(crate::line::Line::pb_loads(&l.encode_to_vec())?);
        }
        for p in &proto.planes {
            objects.planes.push(crate::plane::Plane::pb_loads(&p.encode_to_vec())?);
        }
        for b in &proto.bboxes {
            objects.bboxes.push(crate::obb::OBB::pb_loads(&b.encode_to_vec())?);
        }
        for p in &proto.polylines {
            objects.polylines.push(crate::polyline::Polyline::pb_loads(&p.encode_to_vec())?);
        }
        for p in &proto.pointclouds {
            objects.pointclouds.push(crate::pointcloud::PointCloud::pb_loads(&p.encode_to_vec()));
        }
        for m in &proto.meshes {
            objects.meshes.push(crate::mesh::Mesh::pb_loads(&m.encode_to_vec())?);
        }
        for nc in &proto.nurbscurves {
            objects.nurbscurves.push(crate::nurbscurve::NurbsCurve::pb_loads(&nc.encode_to_vec())?);
        }
        for ns in &proto.nurbssurfaces {
            objects.nurbssurfaces.push(crate::nurbssurface::NurbsSurface::pb_loads(&ns.encode_to_vec())?);
        }
        for b in &proto.breps {
            objects.breps.push(crate::brep::BRep::pb_loads(&b.encode_to_vec())?);
        }
        for e in &proto.elements {
            objects.elements.push(crate::element::Element::pb_loads(&e.encode_to_vec())?);
        }
        Ok(objects)
    }

    pub fn pb_dump(&self, filepath: &str) {
        fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl fmt::Display for Objects {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Objects({}, {}, points={})",
            self.name,
            self.guid(),
            self.points.len()
        )
    }
}

#[cfg(test)]
#[path = "objects_test.rs"]
mod objects_test;
