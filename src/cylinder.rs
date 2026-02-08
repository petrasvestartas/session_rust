use crate::{Line, Mesh, Point, Vector, Xform};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// A cylinder geometry defined by a line and radius.
///
/// The cylinder is generated as a 10-sided cylinder mesh that is oriented
/// along the line direction and scaled to match the line length and specified radius.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Cylinder")]
pub struct Cylinder {
    pub guid: String,
    pub name: String,
    pub radius: f64,
    pub line: Line,
    pub mesh: Mesh,
    #[serde(default = "Xform::identity")]
    pub xform: Xform,
}

impl Cylinder {
    /// Creates a new `Cylinder` from a line and radius.
    ///
    /// # Arguments
    ///
    /// * `line` - The centerline of the cylinder
    /// * `radius` - The radius of the cylinder
    ///
    /// # Returns
    ///
    /// A new `Cylinder` with a generated 10-sided cylinder mesh
    pub fn new(line: Line, radius: f64) -> Self {
        let mesh = Self::create_cylinder_mesh(&line, radius);
        Self {
            guid: Uuid::new_v4().to_string(),
            name: "my_cylinder".to_string(),
            radius,
            line,
            mesh,
            xform: Xform::identity(),
        }
    }

    fn create_cylinder_mesh(line: &Line, radius: f64) -> Mesh {
        let unit_cylinder = Self::unit_cylinder_geometry();
        let xform = Self::line_to_cylinder_transform(line, radius);
        Self::transform_geometry(&unit_cylinder, &xform)
    }

    fn unit_cylinder_geometry() -> (Vec<Point>, Vec<[usize; 3]>) {
        let vertices = vec![
            Point::new(0.5, 0.0, -0.5),
            Point::new(0.404508, 0.293893, -0.5),
            Point::new(0.154508, 0.475528, -0.5),
            Point::new(-0.154508, 0.475528, -0.5),
            Point::new(-0.404508, 0.293893, -0.5),
            Point::new(-0.5, 0.0, -0.5),
            Point::new(-0.404508, -0.293893, -0.5),
            Point::new(-0.154508, -0.475528, -0.5),
            Point::new(0.154508, -0.475528, -0.5),
            Point::new(0.404508, -0.293893, -0.5),
            Point::new(0.5, 0.0, 0.5),
            Point::new(0.404508, 0.293893, 0.5),
            Point::new(0.154508, 0.475528, 0.5),
            Point::new(-0.154508, 0.475528, 0.5),
            Point::new(-0.404508, 0.293893, 0.5),
            Point::new(-0.5, 0.0, 0.5),
            Point::new(-0.404508, -0.293893, 0.5),
            Point::new(-0.154508, -0.475528, 0.5),
            Point::new(0.154508, -0.475528, 0.5),
            Point::new(0.404508, -0.293893, 0.5),
        ];

        let triangles = vec![
            [0, 1, 11],
            [0, 11, 10],
            [1, 2, 12],
            [1, 12, 11],
            [2, 3, 13],
            [2, 13, 12],
            [3, 4, 14],
            [3, 14, 13],
            [4, 5, 15],
            [4, 15, 14],
            [5, 6, 16],
            [5, 16, 15],
            [6, 7, 17],
            [6, 17, 16],
            [7, 8, 18],
            [7, 18, 17],
            [8, 9, 19],
            [8, 19, 18],
            [9, 0, 10],
            [9, 10, 19],
        ];

        (vertices, triangles)
    }

    fn line_to_cylinder_transform(line: &Line, radius: f64) -> Xform {
        let start = line.start();
        let end = line.end();
        let line_vec = line.to_vector();
        let length = line.length();

        let z_axis = line_vec.normalized();
        let x_axis = if z_axis[2].abs() < 0.9 {
            Vector::new(0.0, 0.0, 1.0).cross(&z_axis).normalized()
        } else {
            Vector::new(1.0, 0.0, 0.0).cross(&z_axis).normalized()
        };
        let y_axis = z_axis.cross(&x_axis).normalized();

        let scale = Xform::scale_xyz(radius * 2.0, radius * 2.0, length);
        let rotation = Xform::from_cols(x_axis, y_axis, z_axis);
        let center = Point::new(
            (start[0] + end[0]) * 0.5,
            (start[1] + end[1]) * 0.5,
            (start[2] + end[2]) * 0.5,
        );
        let translation = Xform::translation(center[0], center[1], center[2]);

        &translation * &(&rotation * &scale)
    }

    fn transform_geometry(geometry: &(Vec<Point>, Vec<[usize; 3]>), xform: &Xform) -> Mesh {
        let (vertices, triangles) = geometry;
        let mut mesh = Mesh::new();

        let vertex_keys: Vec<usize> = vertices
            .iter()
            .map(|v| {
                let transformed = xform.transformed_point(v);
                mesh.add_vertex(transformed, None)
            })
            .collect();

        for tri in triangles {
            let face_vertices = vec![
                vertex_keys[tri[0]],
                vertex_keys[tri[1]],
                vertex_keys[tri[2]],
            ];
            mesh.add_face(face_vertices, None);
        }

        mesh
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn transform(&mut self) {
        self.line.transform();
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Cylinder to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let data = serde_json::json!({
            "type": "Cylinder",
            "guid": self.guid,
            "name": self.name,
            "radius": self.radius,
            "line": self.line,
            "mesh": self.mesh.jsondump()
        });
        Ok(serde_json::to_string_pretty(&data)?)
    }

    /// Deserializes a Cylinder from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serializes the Cylinder to a JSON file.
    pub fn to_json(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserializes a Cylinder from a JSON file.
    pub fn from_json(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::new(Line::default(), 1.0))
    }

    pub fn json_dump(&self, filepath: &str) {
        let _ = self.to_json(filepath);
    }

    pub fn json_load(filepath: &str) -> Self {
        Self::from_json(filepath).unwrap_or_else(|_| Self::new(Line::default(), 1.0))
    }

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Cylinder {
            guid: self.guid.clone(),
            name: self.name.clone(),
            radius: self.radius,
            line: Some(crate::proto::Line::decode(self.line.pb_dumps().as_slice()).unwrap()),
            mesh: Some(crate::proto::Mesh::decode(self.mesh.pb_dumps().as_slice()).unwrap()),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Cylinder::decode(data)?;
        let line = if let Some(l) = &proto.line {
            crate::line::Line::pb_loads(&l.encode_to_vec())?
        } else {
            Line::default()
        };
        let mut cyl = Cylinder::new(line, proto.radius);
        cyl.guid = proto.guid;
        cyl.name = proto.name;
        if let Some(m) = &proto.mesh {
            cyl.mesh = crate::mesh::Mesh::pb_loads(&m.encode_to_vec())?;
        }
        if let Some(xform) = proto.xform {
            cyl.xform.guid = xform.guid;
            cyl.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 { cyl.xform.m[i] = *val; }
            }
        }
        Ok(cyl)
    }

    pub fn pb_dump(&self, filepath: &str) {
        std::fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

#[cfg(test)]
#[path = "cylinder_test.rs"]
mod cylinder_test;
