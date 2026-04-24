use crate::{Color, Vector, Xform};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign};
/// A 3D point with visual properties and JSON serialization support.
///
/// The Point struct represents a location in 3D space with x, y, z coordinates.
/// It includes visual properties like color and width for display purposes,
/// as well as transformation matrix support.
///
/// # Attributes
///
/// * `guid` - Unique identifier for the point.
/// * `name` - Name of the point.
/// * `width` - Width of the point for display.
/// * `pointcolor` - Color of the point.
/// * `xform` - Transformation matrix.
///
/// # Examples
///
/// ```
/// # use session_rust::Point;
/// let p = Point::new(1.0, 2.0, 3.0);
/// let dist = p.distance(&Point::new(4.0, 5.0, 6.0), None);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Point")]
pub struct Point {
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>, // Unique identifier
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
            guid: std::sync::OnceLock::new(),
            name: "my_point".to_string(),
            pointcolor: Color::blue(),
            width: 1.0,
            xform: Xform::identity(),
        }
    }
}

impl Point {
    /// Creates a new Point with specified coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - X coordinate of the point.
    /// * `y` - Y coordinate of the point.
    /// * `z` - Z coordinate of the point.
    ///
    /// # Returns
    ///
    /// A new Point with the specified coordinates and a unique GUID.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            _x: x,
            _y: y,
            _z: z,
            ..Default::default()
        }
    }

    /// Creates a new Point with specified coordinates and name.
    ///
    /// # Arguments
    ///
    /// * `x` - X coordinate of the point.
    /// * `y` - Y coordinate of the point.
    /// * `z` - Z coordinate of the point.
    /// * `name` - Name for the point.
    ///
    /// # Returns
    ///
    /// A new Point with the specified coordinates, name, and a unique GUID.
    pub fn with_name(x: f64, y: f64, z: f64, name: &str) -> Self {
        Self {
            _x: x,
            _y: y,
            _z: z,
            name: name.to_string(),
            ..Default::default()
        }
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Deep copy this point with a new GUID.
    ///
    /// Creates a clone of this point with all properties copied
    /// but assigns a new unique GUID.
    ///
    /// # Returns
    ///
    /// A new Point with identical values but a different GUID.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    /// Apply the stored xform transformation to the point coordinates.
    ///
    /// Transforms the point in-place and resets xform to identity.
    pub fn transform(&mut self) {
        let (x, y, z) = (self._x, self._y, self._z);
        let m = &self.xform.m;
        let w = m[3]*x + m[7]*y + m[11]*z + m[15];
        let w_inv = if w.abs() > 1e-10 { 1.0 / w } else { 1.0 };
        self._x = (m[0]*x + m[4]*y + m[8]*z + m[12]) * w_inv;
        self._y = (m[1]*x + m[5]*y + m[9]*z + m[13]) * w_inv;
        self._z = (m[2]*x + m[6]*y + m[10]*z + m[14]) * w_inv;
        self.xform = Xform::identity();
    }

    /// Return a transformed copy of the point.
    ///
    /// Returns a new point with the transformation applied.
    /// The original point and its xform remain unchanged.
    ///
    /// # Returns
    ///
    /// A new transformed Point.
    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Point to a JSON string.
    ///
    /// # Returns
    ///
    /// A Result containing the pretty-printed JSON string or an error.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserializes a Point from a JSON string.
    ///
    /// # Arguments
    ///
    /// * `json_data` - JSON string containing point data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Point or an error.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Serializes the Point to a JSON file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    ///
    /// # Returns
    ///
    /// A Result indicating success or an error.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    /// Deserializes a Point from a JSON file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the JSON file.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Point or an error.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Convert to protobuf binary format.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        
        let proto = crate::proto::Point {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            x: self._x,
            y: self._y,
            z: self._z,
            width: self.width,
            pointcolor: Some(crate::proto::Color {
                guid: self.pointcolor.guid().to_string(),
                name: self.pointcolor.name.clone(),
                r: self.pointcolor.r as i32,
                g: self.pointcolor.g as i32,
                b: self.pointcolor.b as i32,
                a: self.pointcolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    /// Create Point from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded point data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized Point or an error.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        
        let proto = crate::proto::Point::decode(data)?;
        
        let mut pt = Self::new(proto.x, proto.y, proto.z);
        pt.set_guid(proto.guid);
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
            pt.xform.set_guid(xform.guid);
            pt.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    pt.xform.m[i] = *val;
                }
            }
        }
        
        Ok(pt)
    }

    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized Point.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    /// Simple string form (like Python __str__): just coordinates.
    ///
    /// # Returns
    ///
    /// A string in the format "x, y, z".
    pub fn str(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "{}, {}, {}",
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec),
        )
    }

    /// Detailed representation (like Python __repr__).
    ///
    /// # Returns
    ///
    /// A string with full point details including name, coordinates, color, and width.
    pub fn repr(&self) -> String {
        use crate::tolerance::TOLERANCE;
        let prec = crate::tolerance::Tolerance::ROUNDING;
        format!(
            "Point({}, {}, {}, {}, Color({}, {}, {}, {}), {})",
            self.name,
            TOLERANCE.format_number(self._x, prec),
            TOLERANCE.format_number(self._y, prec),
            TOLERANCE.format_number(self._z, prec),
            self.pointcolor.r,
            self.pointcolor.g,
            self.pointcolor.b,
            self.pointcolor.a,
            TOLERANCE.format_number(self.width, prec),
        )
    }

    /// Returns a new point that is the sum of two points.
    ///
    /// # Arguments
    ///
    /// * `p0` - First point.
    /// * `p1` - Second point.
    ///
    /// # Returns
    ///
    /// A new Point with coordinates (p0.x + p1.x, p0.y + p1.y, p0.z + p1.z).
    pub fn sum(p0: &Point, p1: &Point) -> Self {
        Point::new(p0._x + p1._x, p0._y + p1._y, p0._z + p1._z)
    }

    /// Returns a new point that is the difference of two points.
    ///
    /// # Arguments
    ///
    /// * `p0` - First point.
    /// * `p1` - Second point.
    ///
    /// # Returns
    ///
    /// A new Point with coordinates (p0.x - p1.x, p0.y - p1.y, p0.z - p1.z).
    pub fn sub(p0: &Point, p1: &Point) -> Self {
        Point::new(p0._x - p1._x, p0._y - p1._y, p0._z - p1._z)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Details
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Check if the points are in counter-clockwise order on the XY plane.
    ///
    /// # Arguments
    ///
    /// * `a` - First point.
    /// * `b` - Second point.
    /// * `c` - Third point.
    ///
    /// # Returns
    ///
    /// `true` if the points are in counter-clockwise order, `false` otherwise.
    pub fn ccw(a: &Point, b: &Point, c: &Point) -> bool {
        (c._y - a._y) * (b._x - a._x) > (b._y - a._y) * (c._x - a._x)
    }

    /// Alias for `ccw` - check if points are in counter-clockwise order.
    ///
    /// # Arguments
    ///
    /// * `a` - First point.
    /// * `b` - Second point.
    /// * `c` - Third point.
    ///
    /// # Returns
    ///
    /// `true` if the points are in counter-clockwise order, `false` otherwise.
    pub fn is_ccw(a: &Point, b: &Point, c: &Point) -> bool {
        Self::ccw(a, b, c)
    }

    /// Calculate the mid point between two points.
    ///
    /// # Arguments
    ///
    /// * `a` - First point.
    /// * `b` - Second point.
    ///
    /// # Returns
    ///
    /// A new Point at the midpoint between `a` and `b`.
    pub fn mid_point(a: &Point, b: &Point) -> Point {
        Point::new(
            (a._x + b._x) / 2.0,
            (a._y + b._y) / 2.0,
            (a._z + b._z) / 2.0,
        )
    }

    /// Calculate the distance between this point and another point.
    /// 
    /// # Arguments
    /// * `p` - The other point
    /// * `double_min` - Optional minimum value for distance calculation. Defaults to 1e-12.
    pub fn distance(&self, p: &Point, double_min: Option<f64>) -> f64 {
        let double_min = double_min.unwrap_or(1e-12);
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

    /// Calculate the squared distance between this point and another point.
    /// 
    /// # Arguments
    /// * `p` - The other point
    /// * `double_min` - Optional minimum value for distance calculation. Defaults to 1e-12.
    pub fn squared_distance(&self, p: &Point, double_min: Option<f64>) -> f64 {
        let double_min = double_min.unwrap_or(1e-12);
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

    /// Calculate the area of a 2D polygon using the shoelace formula.
    ///
    /// # Arguments
    ///
    /// * `points` - Slice of points defining the polygon vertices.
    ///
    /// # Returns
    ///
    /// The area of the polygon.
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
    ///
    /// # Arguments
    ///
    /// * `vertices` - Slice of exactly 4 points defining the quadrilateral.
    ///
    /// # Returns
    ///
    /// A Result containing the centroid Point, or an error if not 4 vertices.
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
        Ok(Point::new(result[0], result[1], result[2]))
    }

    /// Average of an arbitrary list of points.
    pub fn centroid(points: &[Point]) -> Point {
        if points.is_empty() {
            return Point::new(0.0, 0.0, 0.0);
        }
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;
        for p in points {
            cx += p[0];
            cy += p[1];
            cz += p[2];
        }
        let n = points.len() as f64;
        Point::new(cx / n, cy / n, cz / n)
    }

    /// Approximate dihedral angle in degrees between half-planes (p,q,r) and (p,q,s).
    pub fn dihedral_angle_deg(p: &Point, q: &Point, r: &Point, s: &Point) -> f64 {
        use crate::tolerance::Tolerance;
        let pq = Vector::new(q[0] - p[0], q[1] - p[1], q[2] - p[2]);
        let pr = Vector::new(r[0] - p[0], r[1] - p[1], r[2] - p[2]);
        let ps = Vector::new(s[0] - p[0], s[1] - p[1], s[2] - p[2]);
        let n1 = pq.cross(&pr);
        let n2 = pq.cross(&ps);
        let m1 = n1.magnitude();
        let m2 = n2.magnitude();
        if m1 < Tolerance::ZERO_TOLERANCE || m2 < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }
        let mut cos_t = n1.dot(&n2) / (m1 * m2);
        if cos_t > 1.0 {
            cos_t = 1.0;
        }
        if cos_t < -1.0 {
            cos_t = -1.0;
        }
        cos_t.acos() * (180.0 / 3.141592653589793)
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use crate::tolerance::TOLERANCE;
        write!(
            f,
            "Point(x={}, y={}, z={})",
            TOLERANCE.format_number(self._x, -999),
            TOLERANCE.format_number(self._y, -999),
            TOLERANCE.format_number(self._z, -999)
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
        self._x += rhs[0];
        self._y += rhs[1];
        self._z += rhs[2];
    }
}

impl SubAssign<Vector> for Point {
    fn sub_assign(&mut self, rhs: Vector) {
        self._x -= rhs[0];
        self._y -= rhs[1];
        self._z -= rhs[2];
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
        Point::new(self._x + rhs[0], self._y + rhs[1], self._z + rhs[2])
    }
}

impl Sub<Vector> for Point {
    type Output = Point;

    fn sub(self, rhs: Vector) -> Self::Output {
        Point::new(self._x - rhs[0], self._y - rhs[1], self._z - rhs[2])
    }
}

// Reference operators (avoid cloning)
impl Mul<f64> for &Point {
    type Output = Point;

    fn mul(self, rhs: f64) -> Self::Output {
        Point::new(self._x * rhs, self._y * rhs, self._z * rhs)
    }
}

impl Div<f64> for &Point {
    type Output = Point;

    fn div(self, rhs: f64) -> Self::Output {
        Point::new(self._x / rhs, self._y / rhs, self._z / rhs)
    }
}

impl Add<Vector> for &Point {
    type Output = Point;

    fn add(self, rhs: Vector) -> Self::Output {
        Point::new(self._x + rhs[0], self._y + rhs[1], self._z + rhs[2])
    }
}

impl Add<&Vector> for &Point {
    type Output = Point;

    fn add(self, rhs: &Vector) -> Self::Output {
        Point::new(self._x + rhs[0], self._y + rhs[1], self._z + rhs[2])
    }
}

impl Sub<Vector> for &Point {
    type Output = Point;

    fn sub(self, rhs: Vector) -> Self::Output {
        Point::new(self._x - rhs[0], self._y - rhs[1], self._z - rhs[2])
    }
}

impl Sub<&Vector> for &Point {
    type Output = Point;

    fn sub(self, rhs: &Vector) -> Self::Output {
        Point::new(self._x - rhs[0], self._y - rhs[1], self._z - rhs[2])
    }
}

impl Sub<&Point> for &Point {
    type Output = Vector;

    fn sub(self, rhs: &Point) -> Self::Output {
        Vector::new(self._x - rhs._x, self._y - rhs._y, self._z - rhs._z)
    }
}

#[cfg(test)]
#[path = "point_test.rs"]
mod point_test;
