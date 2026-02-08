use crate::arrow::Arrow;
use crate::boundingbox::BoundingBox;
use crate::cylinder::Cylinder;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::plane::Plane;
use crate::point::Point;
use crate::pointcloud::PointCloud;
use crate::polyline::Polyline;
use serde::{ser::Serialize as SerTrait, Deserialize, Serialize};
use std::fmt;
use std::fs;
use uuid::Uuid;

/// A collection of all geometry objects.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Objects")]
pub struct Objects {
    pub guid: String,
    pub name: String,
    pub points: Vec<Point>,
    pub lines: Vec<Line>,
    pub planes: Vec<Plane>,
    pub bboxes: Vec<BoundingBox>,
    pub polylines: Vec<Polyline>,
    pub pointclouds: Vec<PointCloud>,
    pub meshes: Vec<Mesh>,
    pub cylinders: Vec<Cylinder>,
    pub arrows: Vec<Arrow>,
}

impl Default for Objects {
    fn default() -> Self {
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_objects".to_string(),
            points: Vec::new(),
            lines: Vec::new(),
            planes: Vec::new(),
            bboxes: Vec::new(),
            polylines: Vec::new(),
            pointclouds: Vec::new(),
            meshes: Vec::new(),
            cylinders: Vec::new(),
            arrows: Vec::new(),
        }
    }
}

impl Objects {
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
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        SerTrait::serialize(self, &mut ser)?;
        Ok(String::from_utf8(buf)?)
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
            guid: self.guid.clone(),
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
            cylinders: self.cylinders.iter().map(|c| {
                crate::proto::Cylinder::decode(c.pb_dumps().as_slice()).unwrap()
            }).collect(),
            arrows: self.arrows.iter().map(|a| {
                crate::proto::Arrow::decode(a.pb_dumps().as_slice()).unwrap()
            }).collect(),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Objects::decode(data)?;
        let mut objects = Objects::new();
        objects.guid = proto.guid;
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
            objects.bboxes.push(crate::boundingbox::BoundingBox::pb_loads(&b.encode_to_vec())?);
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
        for c in &proto.cylinders {
            objects.cylinders.push(crate::cylinder::Cylinder::pb_loads(&c.encode_to_vec())?);
        }
        for a in &proto.arrows {
            objects.arrows.push(crate::arrow::Arrow::pb_loads(&a.encode_to_vec())?);
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
            self.guid,
            self.points.len()
        )
    }
}

#[cfg(test)]
#[path = "objects_test.rs"]
mod objects_test;
