use crate::{Color, Vector, Xform};
use serde::{ser::Serialize as SerTrait, Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign};
use uuid::Uuid;

/// A 3D point with visual properties and JSON serialization support.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Point")]
pub struct Point {
    pub guid: String, // Unique identifier
    pub name: String, // Name of the point
    #[serde(rename = "x")]
    _x: f64, // X coordinate (private)
    #[serde(rename = "y")]
    _y: f64, // Y coordinate (private)
    #[serde(rename = "z")]
    _z: f64, // Z coordinate (private)
    pub width: f64,   // Width of the point
    pub pointcolor: Color, // Color of the point
    #[serde(default = "Xform::identity")]
    pub xform: Xform, // Transformation matrix
}

impl Default for Point {
    fn default() -> Self {
        Self {
            _x: 0.0,
            _y: 0.0,
            _z: 0.0,
            guid: Uuid::new_v4().to_string(),
            name: "my_point".to_string(),
            pointcolor: Color::blue(),
            width: 1.0,
            xform: Xform::identity(),
        }
    }
}

impl Point {
    /// Creates a new Point with specified coordinates.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            _x: x,
            _y: y,
            _z: z,
            ..Default::default()
        }
    }
    /// Deep copy this point but regenerate the guid, mirroring the behavior of
    /// the C++ copy constructor / Python duplicate which duplicate all data
    /// except the unique identifier.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = Uuid::new_v4().to_string();
        copy
    }
    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Point to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        SerTrait::serialize(self, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

    /// Deserializes a Point from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serializes the Point to a JSON file.
    pub fn to_json(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn transform(&mut self) {
        let xform = self.xform.clone();
        xform.transform_point(self);
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    /// Deserializes a Point from a JSON file.
    pub fn from_json(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    #[cfg(feature = "protobuf")]
    /// Convert to protobuf binary format.
    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;
        
        let proto = crate::proto::Point {
            guid: self.guid.clone(),
            name: self.name.clone(),
            x: self._x,
            y: self._y,
            z: self._z,
            width: self.width,
            pointcolor: Some(crate::proto::Color {
                name: self.pointcolor.name.clone(),
                r: self.pointcolor.r as i32,
                g: self.pointcolor.g as i32,
                b: self.pointcolor.b as i32,
                a: self.pointcolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    #[cfg(feature = "protobuf")]
    /// Create Point from protobuf binary data.
    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        
        let proto = crate::proto::Point::decode(data)?;
        
        let mut pt = Self::new(proto.x, proto.y, proto.z);
        pt.guid = proto.guid;
        pt.name = proto.name;
        pt.width = proto.width;
        
        if let Some(color) = proto.pointcolor {
            pt.pointcolor.name = color.name;
            pt.pointcolor.r = color.r as u8;
            pt.pointcolor.g = color.g as u8;
            pt.pointcolor.b = color.b as u8;
            pt.pointcolor.a = color.a as u8;
        }
        
        if let Some(xform) = proto.xform {
            pt.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    pt.xform.m[i] = *val;
                }
            }
        }
        
        Ok(pt)
    }

    #[cfg(feature = "protobuf")]
    /// Write protobuf to file.
    pub fn protobuf_dump(&self, filepath: &str) {
        let data = self.to_protobuf();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    #[cfg(feature = "protobuf")]
    /// Read protobuf from file.
    pub fn protobuf_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Check if the points are in counter-clockwise order.
    pub fn ccw(a: &Point, b: &Point, c: &Point) -> bool {
        (c._y - a._y) * (b._x - a._x) > (b._y - a._y) * (c._x - a._x)
    }

    pub fn is_ccw(a: &Point, b: &Point, c: &Point) -> bool {
        Self::ccw(a, b, c)
    }

    /// Calculate the mid point between this point and another point.
    pub fn mid_point(a: &Point, b: &Point) -> Point {
        Point::new(
            (a._x + b._x) / 2.0,
            (a._y + b._y) / 2.0,
            (a._z + b._z) / 2.0,
        )
    }

    /// Calculate the distance between this point and another point.
    pub fn distance(&self, p: &Point) -> f64 {
        self.distance_with_min(p, 1e-12)
    }

    /// Calculate the distance between this point and another point with custom minimum.
    pub fn distance_with_min(&self, p: &Point, double_min: f64) -> f64 {
        let mut dx = (self[0] - p[0]).abs();
        let mut dy = (self[1] - p[1]).abs();
        let mut dz = (self[2] - p[2]).abs();

        // Reorder coordinates to put largest in dx
        if dy >= dx && dy >= dz {
            std::mem::swap(&mut dx, &mut dy);
        } else if dz >= dx && dz >= dy {
            std::mem::swap(&mut dx, &mut dz);
        }

        if dx > double_min {
            dy /= dx;
            dz /= dx;
            dx * (1.0 + dy * dy + dz * dz).sqrt()
        } else if dx > 0.0 && dx.is_finite() {
            dx
        } else {
            0.0
        }
    }

    pub fn squared_distance(&self, p: &Point) -> f64 {
        self.squared_distance_with_min(p, 1e-12)
    }

    pub fn squared_distance_with_min(&self, p: &Point, double_min: f64) -> f64 {
        let mut dx = (self[0] - p[0]).abs();
        let mut dy = (self[1] - p[1]).abs();
        let mut dz = (self[2] - p[2]).abs();

        if dy >= dx && dy >= dz {
            std::mem::swap(&mut dx, &mut dy);
        } else if dz >= dx && dz >= dy {
            std::mem::swap(&mut dx, &mut dz);
        }

        if dx > double_min {
            dy /= dx;
            dz /= dx;
            dx * dx * (1.0 + dy * dy + dz * dz)
        } else if dx > 0.0 && dx.is_finite() {
            dx * dx
        } else {
            0.0
        }
    }

    /// Calculate the area of a polygon.
    pub fn area(points: &[Point]) -> f64 {
        let n = points.len();
        let mut area = 0.0;

        for i in 0..n {
            let j = (i + 1) % n;
            area += points[i][0] * points[j][1];
            area -= points[j][0] * points[i][1];
        }

        area.abs() / 2.0
    }

    /// Calculate the centroid of a quadrilateral.
    pub fn centroid_quad(vertices: &[Point]) -> Result<Point, &'static str> {
        if vertices.len() != 4 {
            return Err("Polygon must have exactly 4 vertices.");
        }

        let mut total_area = 0.0;
        let mut centroid_sum = Vector::new(0.0, 0.0, 0.0);

        for i in 0..4 {
            let p0 = &vertices[i];
            let p1 = &vertices[(i + 1) % 4];
            let p2 = &vertices[(i + 2) % 4];

            let tri_area =
                ((p0[0] * (p1[1] - p2[1]) + p1[0] * (p2[1] - p0[1]) + p2[0] * (p0[1] - p1[1]))
                    .abs())
                    / 2.0;
            total_area += tri_area;

            let tri_centroid = Vector::new(
                (p0[0] + p1[0] + p2[0]) / 3.0,
                (p0[1] + p1[1] + p2[1]) / 3.0,
                (p0[2] + p1[2] + p2[2]) / 3.0,
            );
            centroid_sum += tri_centroid * tri_area;
        }

        let result = centroid_sum / total_area;
        Ok(Point::new(result.x(), result.y(), result.z()))
    }

    /// Simple string form (like Python __str__): just coordinates
    pub fn str(&self) -> String {
        use crate::tolerance::TOL;
        let prec = Some(crate::tolerance::Tolerance::ROUNDING);
        format!(
            "{}, {}, {}",
            TOL.format_number(self._x, prec),
            TOL.format_number(self._y, prec),
            TOL.format_number(self._z, prec),
        )
    }

    /// Detailed representation (like Python __repr__)
    pub fn repr(&self) -> String {
        use crate::tolerance::TOL;
        let prec = Some(crate::tolerance::Tolerance::ROUNDING);
        format!(
            "Point({}, {}, {}, {}, Color({}, {}, {}, {}), {})",
            self.name,
            TOL.format_number(self._x, prec),
            TOL.format_number(self._y, prec),
            TOL.format_number(self._z, prec),
            self.pointcolor.r,
            self.pointcolor.g,
            self.pointcolor.b,
            self.pointcolor.a,
            TOL.format_number(self.width, prec),
        )
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use crate::tolerance::TOL;
        write!(
            f,
            "Point(x={}, y={}, z={})",
            TOL.format_number(self._x, None),
            TOL.format_number(self._y, None),
            TOL.format_number(self._z, None)
        )
    }
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && (self._x * 1000000.0).round() == (other._x * 1000000.0).round()
            && (self._y * 1000000.0).round() == (other._y * 1000000.0).round()
            && (self._z * 1000000.0).round() == (other._z * 1000000.0).round()
            && (self.width * 1000000.0).round() == (other.width * 1000000.0).round()
            && self.pointcolor == other.pointcolor
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Indexing operators
///////////////////////////////////////////////////////////////////////////////////////////

impl Index<usize> for Point {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self._x,
            1 => &self._y,
            2 => &self._z,
            _ => panic!("Index out of range"),
        }
    }
}

impl IndexMut<usize> for Point {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self._x,
            1 => &mut self._y,
            2 => &mut self._z,
            _ => panic!("Index out of range"),
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// No-copy operators
///////////////////////////////////////////////////////////////////////////////////////////

impl MulAssign<f64> for Point {
    fn mul_assign(&mut self, rhs: f64) {
        self._x *= rhs;
        self._y *= rhs;
        self._z *= rhs;
    }
}

impl DivAssign<f64> for Point {
    fn div_assign(&mut self, rhs: f64) {
        self._x /= rhs;
        self._y /= rhs;
        self._z /= rhs;
    }
}

impl AddAssign<Vector> for Point {
    fn add_assign(&mut self, rhs: Vector) {
        self._x += rhs.x();
        self._y += rhs.y();
        self._z += rhs.z();
    }
}

impl SubAssign<Vector> for Point {
    fn sub_assign(&mut self, rhs: Vector) {
        self._x -= rhs.x();
        self._y -= rhs.y();
        self._z -= rhs.z();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Copy operators
///////////////////////////////////////////////////////////////////////////////////////////

impl Mul<f64> for Point {
    type Output = Point;

    fn mul(self, rhs: f64) -> Self::Output {
        Point::new(self._x * rhs, self._y * rhs, self._z * rhs)
    }
}

impl Div<f64> for Point {
    type Output = Point;

    fn div(self, rhs: f64) -> Self::Output {
        Point::new(self._x / rhs, self._y / rhs, self._z / rhs)
    }
}

impl Sub<Point> for Point {
    type Output = Vector;

    fn sub(self, rhs: Point) -> Self::Output {
        Vector::new(self._x - rhs._x, self._y - rhs._y, self._z - rhs._z)
    }
}

impl Add<Vector> for Point {
    type Output = Point;

    fn add(self, rhs: Vector) -> Self::Output {
        Point::new(self._x + rhs.x(), self._y + rhs.y(), self._z + rhs.z())
    }
}

impl Sub<Vector> for Point {
    type Output = Point;

    fn sub(self, rhs: Vector) -> Self::Output {
        Point::new(self._x - rhs.x(), self._y - rhs.y(), self._z - rhs.z())
    }
}

#[cfg(test)]
#[path = "point_test.rs"]
mod point_test;
